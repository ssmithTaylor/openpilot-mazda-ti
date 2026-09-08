# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Audit recorded controller input identities without fitting controller output.

Exact model identity comes from controlsState.lateralPlanMonoTime. CarControl is
paired by publication order and verified against the logged output/curvature.
CarState identity is inferred from independent recorded curvature/actual LA:
vEgo = sqrt(actualLateralAccel / curvature). LiveParameters identity is inferred
from the real VehicleModel's curvature and the selected recorded CarState.
Near-tied speed candidates are additionally checked against that curvature;
indistinguishable parameter messages are labeled explicitly, not called unique.
The inferred input mapping must be frozen across controller variants. These
identities do not establish socket receipt time, contact, or alternative motion.
"""

import bisect
from collections import Counter
import math

from .runtime import bootstrap

bootstrap()
from cereal import log
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel


def audit(paths, start, end):
  from types import SimpleNamespace

  args = SimpleNamespace(start=start, end=end)
  services = {}
  required_services = {
    'carParams',
    'controlsState',
    'carControl',
    'carState',
    'modelV2',
    'liveParameters',
    'carOutput',
    'frogpilotCarState',
    'liveDelay',
    'liveTorqueParameters',
  }
  for path in paths:
    for event in log.Event.read_multiple_bytes(path.read_bytes()):
      if event.which() in required_services:
        services.setdefault(event.which(), []).append(event)
  for events in services.values():
    events.sort(key=lambda e: e.logMonoTime)
  times = {name: [e.logMonoTime for e in events] for name, events in services.items()}
  model_by_time = {e.logMonoTime: i for i, e in enumerate(services['modelV2'])}
  vm = VehicleModel(services['carParams'][0].carParams)
  rows = []
  tally = Counter()
  cs_ranks, lp_ranks = Counter(), Counter()

  for control_index, event in enumerate(services['controlsState']):
    if not args.start <= event.logMonoTime / 1e9 < args.end:
      continue
    state = event.controlsState
    if state.lateralControlState.which() != 'torqueState':
      continue
    torque = state.lateralControlState.torqueState
    row = {
      'controlsState_index': control_index,
      'controlsState_logMonoTime': event.logMonoTime,
      'startMonoTime': state.startMonoTime,
      'model_logMonoTime': state.lateralPlanMonoTime,
      'model_index': model_by_time.get(state.lateralPlanMonoTime),
    }
    tally['frames'] += 1
    tally['model_exact_found'] += row['model_index'] is not None

    cc_i = bisect.bisect_left(times['carControl'], event.logMonoTime)
    if cc_i < len(services['carControl']):
      cc = services['carControl'][cc_i]
      row.update(
        carControl_index=cc_i,
        carControl_logMonoTime=cc.logMonoTime,
        carControl_steer_error=abs(cc.carControl.actuators.steer - torque.output),
        carControl_curvature_error=abs(cc.carControl.actuators.curvature - state.desiredCurvature),
      )
      tally['carControl_exact_output'] += row['carControl_steer_error'] == 0.0
      tally['carControl_exact_curvature'] += row['carControl_curvature_error'] == 0.0

    cs_latest = bisect.bisect_right(times['carState'], event.logMonoTime) - 1
    if not torque.active:
      row['identity_unresolved'] = 'Inactive controller: no usable actual LA identity; use generic timing'
      rows.append(row)
      continue
    ratio = torque.actualLateralAccel / state.curvature if state.curvature else -1
    if ratio <= 0 or cs_latest < 0:
      row['identity_unresolved'] = 'No usable speed identity from actual LA / curvature'
      rows.append(row)
      continue
    observed_speed = math.sqrt(ratio)
    candidates = [(abs(services['carState'][i].carState.vEgo - observed_speed), i) for i in range(max(0, cs_latest - 3), cs_latest + 1)]
    speed_error, cs_i = min(candidates)
    lp_latest = bisect.bisect_right(times['liveParameters'], event.logMonoTime) - 1

    def parameter_candidates(car_state, lp_latest=lp_latest, state=state):
      result = []
      for i in range(max(0, lp_latest - 2), lp_latest + 1):
        params = services['liveParameters'][i].liveParameters
        vm.update_params(max(params.stiffnessFactor, 0.1), max(params.steerRatio, 0.1))
        curvature = -vm.calc_curvature(math.radians(car_state.steeringAngleDeg - params.angleOffsetDeg), car_state.vEgo, params.roll)
        result.append((abs(curvature - state.curvature), i))
      return result

    joint_candidates = []
    for error, i in candidates:
      if error <= 2e-6 and any(value <= 2e-9 for value, _ in parameter_candidates(services['carState'][i].carState)):
        joint_candidates.append((error, i))
    if joint_candidates:
      speed_error, cs_i = min(joint_candidates)
    row['carState_candidates_consistent_with_speed_and_curvature'] = len(joint_candidates)
    tally['carState_unique_with_speed_and_curvature'] += len(joint_candidates) == 1
    cs_event = services['carState'][cs_i]
    cs = cs_event.carState
    row.update(
      carState_index=cs_i,
      carState_logMonoTime=cs_event.logMonoTime,
      carState_rank_before_publication=cs_latest - cs_i,
      carState_speed_identity_error=speed_error,
      carState_candidates_within_2e6=sum(error <= 2e-6 for error, _ in candidates),
    )
    cs_ranks[cs_latest - cs_i] += 1
    tally['carState_unique_within_speed_tolerance'] += row['carState_candidates_within_2e6'] == 1
    tally['carState_after_startMonoTime'] += cs_event.logMonoTime > state.startMonoTime

    lp_candidates = parameter_candidates(cs)
    if not lp_candidates:
      row['identity_unresolved'] = 'No liveParameters candidates in segment'
      rows.append(row)
      continue
    curvature_error, lp_i = min(lp_candidates)
    lp_event = services['liveParameters'][lp_i]
    compatible_lp = [services['liveParameters'][i].liveParameters for error, i in lp_candidates if error <= 2e-9]
    fields = ['stiffnessFactor', 'steerRatio', 'angleOffsetDeg', 'roll']
    row['liveParameters_compatible_control_fields_identical'] = bool(compatible_lp) and all(
      all(getattr(params, field) == getattr(compatible_lp[0], field) for field in fields) for params in compatible_lp
    )
    row['liveParameters_candidates_control_equivalent'] = row['liveParameters_compatible_control_fields_identical']
    row['liveParameters_receipt_identity_unique'] = len(compatible_lp) == 1
    if len(compatible_lp) > 1:
      tally[
        'liveParameters_ambiguous_but_control_fields_identical'
        if row['liveParameters_compatible_control_fields_identical']
        else 'liveParameters_ambiguous_with_different_control_fields'
      ] += 1
    row.update(
      liveParameters_index=lp_i,
      liveParameters_logMonoTime=lp_event.logMonoTime,
      liveParameters_rank_before_publication=lp_latest - lp_i,
      liveParameters_curvature_error=curvature_error,
      liveParameters_candidates_within_2e9=sum(error <= 2e-9 for error, _ in lp_candidates),
    )
    lp_ranks[lp_latest - lp_i] += 1
    tally['liveParameters_unique_within_curvature_tolerance'] += row['liveParameters_candidates_within_2e9'] == 1
    tally['liveParameters_after_selected_carState'] += lp_event.logMonoTime > cs_event.logMonoTime
    row['other_service_sampling_ambiguity'] = {}
    for service in ['carOutput', 'frogpilotCarState', 'liveDelay', 'liveTorqueParameters', 'modelV2']:
      before_cs = bisect.bisect_right(times[service], cs_event.logMonoTime) - 1
      before_publication = bisect.bisect_right(times[service], event.logMonoTime) - 1
      row['other_service_sampling_ambiguity'][service] = {
        'latest_index_before_carState': before_cs,
        'latest_index_before_controlsState_publication': before_publication,
        'messages_between': before_publication - before_cs,
      }
    rows.append(row)

  summary = dict(tally)
  summary.update(carState_selected_ranks=dict(cs_ranks), liveParameters_selected_ranks=dict(lp_ranks))
  for field in ['carState_speed_identity_error', 'liveParameters_curvature_error']:
    summary['max_' + field] = max((row[field] for row in rows if field in row), default=None)
  result = {
    'rlogs': [path.parent.name + '/' + path.name for path in paths],
    'window_seconds': [args.start, args.end],
    'method': __doc__,
    'speed_tolerance_mps': 2e-6,
    'curvature_tolerance_per_m': 2e-9,
    'summary': summary,
    'frames': rows,
  }
  return result
