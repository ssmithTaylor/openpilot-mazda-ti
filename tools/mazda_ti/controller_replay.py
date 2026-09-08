# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Diagnostic recorded-input replay. No new vehicle trajectory or device writes.

Baseline residuals are the primary output. Variants are not trusted merely because
this script executes. Input sampling/validity and initial state remain auditable.
"""

from bisect import bisect_right
from collections import deque
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from .runtime import DT, TestInterface, controller_source, load_controller_source, bootstrap

bootstrap()
from cereal import log
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel


TOPICS = {
  'controlsState',
  'carState',
  'carControl',
  'carOutput',
  'frogpilotCarState',
  'liveParameters',
  'liveTorqueParameters',
  'liveDelay',
  'modelV2',
  'frogpilotPlan',
  'carParams',
}
METRICS = ['actualLateralAccel', 'desiredLateralAccel', 'error', 'p', 'i', 'd', 'f', 'frictionTorque', 'output', 'plantState']

# Optional local observer hook. Default preserves all prior replay behavior.
# Context uses the exact model identity consumed by controlsd, not a nearest
# model frame. Production transport/freshness integration is not simulated.
MODEL_CONTEXT_BUILDER = None


def instrument(source):
  needle = '    wheel_rate = abs(float(CS.steeringRateDeg))'
  assert source.count(needle) == 1
  source = source.replace(needle, '    self._replay_pre_break_command = command\n' + needle)
  needle = '    output_torque = command / u_max'
  assert source.count(needle) == 1
  return source.replace(
    needle,
    '''    self._replay_trace = dict(
      lane_valid=getattr(getattr(self,"_release_context",None),"valid",False),
      lane_reason=getattr(getattr(self,"_release_context",None),"reason","missing"),
      lane_heading10=getattr(getattr(self,"_release_context",None),"heading10",0.0),
      lane_heading20=getattr(getattr(self,"_release_context",None),"heading20",0.0),
      lane_curvature10=getattr(getattr(self,"_release_context",None),"curvature10",0.0),
      lane_curvature20=getattr(getattr(self,"_release_context",None),"curvature20",0.0),
      lane_offset=getattr(getattr(self,"_release_context",None),"offset",0.0),
      lane_age=getattr(getattr(self,"_release_context",None),"age",0.0),
      lane_position_permitted=getattr(getattr(self,"lane_release",None),"position_permitted",False),
      lane_permitted=getattr(getattr(self,"lane_release",None),"permitted",False),
      lane_removed=list(getattr(getattr(self,"lane_release",None),"removed",[0.0,0.0])),
      friction_withdrawal=getattr(getattr(self,"friction_release",None),"removed",0.0),
      friction_episode_completed=getattr(getattr(self,"friction_release",None),"completed",False),
      tracked_setpoint=tracked_setpoint, setpoint=setpoint, ff_lat_accel=ff_lat_accel,
      future_lateral_accel=future_lateral_accel, fric_comp=fric_comp,
      pre_break_command=self._replay_pre_break_command, break_target=target,
      break_boost=self.break_boost, break_frames=self.break_frames,
      freeze_integrator=freeze_integrator, commit_blend=self.commit_blend,
      commit_ff_state=self.commit_ff_filter.x, commit_sp_state=self.commit_sp_filter.x,
      commit_release_coefficient=self.commit_ff_filter.a_release,
      offset=lat_accel_offset, roll_compensation=roll_compensation,
      pid_unclipped=self.pid.p+self.pid.i+self.pid.d+self.pid.f,
      pid_clipped=output_lataccel, la_max=la_max)
'''
    + needle,
  )


class Streams:
  def __init__(self, paths):
    self.rows = {k: [] for k in TOPICS}
    self.buffers = [p.read_bytes() for p in paths]
    for raw in self.buffers:
      for event in log.Event.read_multiple_bytes(raw):
        kind = event.which()
        if kind in TOPICS:
          self.rows[kind].append((int(event.logMonoTime), event))
    for rows in self.rows.values():
      rows.sort(key=lambda row: row[0])
    self.times = {k: [t for t, _ in rows] for k, rows in self.rows.items()}

  def latest(self, topic, t):
    i = bisect_right(self.times[topic], t) - 1
    if i < 0:
      return None, None, None
    mono, event = self.rows[topic][i]
    return getattr(event, topic), mono, bool(event.valid)


def distribution(values):
  values = np.asarray(values)
  return dict(zip(['median', 'p95', 'p99', 'max'], map(float, np.quantile(np.abs(values), [0.5, 0.95, 0.99, 1])), strict=False))


def switch_controller(ctrl, controller_class, cp, previous_active):
  """Retain controller state and initialize fields introduced by a candidate."""
  fresh = controller_class(cp, TestInterface(), DT)
  for name, value in vars(fresh).items():
    if not hasattr(ctrl, name):
      if name == '_plant_measurement_initialized':
        # Legacy observers skipped inactive samples. Only an immediately preceding
        # active sample is valid history; existing new observers remain untouched.
        value = previous_active
        if not previous_active:
          ctrl.measurement_rate_filter.x = 0.0
      setattr(ctrl, name, value)
  ctrl.__class__ = controller_class


def publish_paired_request(feedback, streams, mono, curvature, recorded_output, candidate_output, active):
  """Publish the original carControl identity paired with a controller update."""
  cc_index = bisect_right(streams.times['carControl'], mono)
  if cc_index == len(streams.times['carControl']):
    raise RuntimeError('Missing paired carControl publication')
  cc_mono, cc_event = streams.rows['carControl'][cc_index]
  if not feedback.fixture.start_ns < cc_mono <= feedback.fixture.end_ns:
    return
  if cc_event.carControl.actuators.steer != recorded_output or cc_event.carControl.actuators.curvature != curvature:
    raise RuntimeError('CarControl pairing does not match recorded controller output')
  # A post-activation publication can use pre-activation consumed input, or even
  # a pre-activation controller update. Publication and computation are separate.
  steer = candidate_output if mono >= feedback.fixture.start_ns else recorded_output
  feedback.publish_request(cc_mono, steer, active)


def run(args):
  paths = [Path(p) for p in args.rlogs]
  streams = Streams(paths)
  feedback = None
  feedback_baseline = None
  software_from = getattr(args, 'software_feedback_from', None)
  if software_from is not None:
    from .feedback import Fixture

    parameters = args.settings
    keys = {
      'TI_STEER_MAX': 'TiSteerMax',
      'TI_STEER_DELTA_UP': 'TiSteerDeltaUp',
      'TI_STEER_DELTA_DOWN': 'TiSteerDeltaDown',
      'TI_STEER_DRIVER_ALLOWANCE': 'TiSteerDriverAllowance',
      'TI_STEER_DRIVER_MULTIPLIER': 'TiSteerDriverMultiplier',
      'TI_STEER_DELTA_UP_KNEE': 'TiSteerDeltaUpKnee',
      'TI_STEER_DELTA_UP_HIGH': 'TiSteerDeltaUpHigh',
    }
    limits = SimpleNamespace(**{k: float(parameters[v]) for k, v in keys.items()}, TI_STEER_DRIVER_FACTOR=1)
    if limits.TI_STEER_MAX != 600.0:
      raise ValueError('This controller fixture supports only the recorded 600-count configuration')
    request_identity = None
    if getattr(args, 'request_identity_map', None):
      request_identity = json.loads(Path(args.request_identity_map).read_text())['mappings'][args.request_identity_mode]
    fixture = Fixture(paths, round(software_from * 1e9), round(args.end * 1e9), limits, args.limiter_ref, request_identity=request_identity)
    feedback_baseline = fixture.verify_baseline()
    feedback = fixture.new()
  identity = {}
  if args.input_audit:
    audit = json.loads(Path(args.input_audit).read_text(encoding='utf-8'))
    identity = {r['controlsState_logMonoTime']: r for r in audit['frames']}
  by_time = {k: dict(streams.rows[k]) for k in ['carState', 'liveParameters', 'modelV2']}
  cp = streams.rows['carParams'][0][1].carParams.as_builder()
  source = controller_source(args.controller_ref)
  module = load_controller_source(instrument(source), 'recorded_replay_' + args.variant)
  baseline_module = (
    module if args.variant_start is None else load_controller_source(instrument(controller_source(args.warmup_controller_ref)), 'recorded_replay_warmup')
  )
  ctrl = baseline_module.LatControlTorque(cp, TestInterface(), DT)
  vm = VehicleModel(cp)
  if args.kp is not None:
    ctrl.pid._k_p = [[0], [args.kp]]
  toggles = SimpleNamespace(
    lat_damping=True, lat_commit_setpoint=not args.no_commit, lat_friction_comp=True, lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=600.0
  )
  history = deque([0.0] * 3, maxlen=3)
  limited = False
  previous_output = None
  previous_active = False
  rows, skipped = [], 0
  i_anchor = None
  for mono, event in streams.rows['controlsState']:
    if mono / 1e9 > args.end:
      break
    c = event.controlsState
    if c.lateralControlState.which() != 'torqueState':
      continue
    observed = c.lateralControlState.torqueState
    cs, cs_time, cs_valid = streams.latest('carState', mono)
    if cs is None:
      skipped += 1
      continue
    if mono in identity and 'carState_logMonoTime' in identity[mono]:
      cs_time = identity[mono]['carState_logMonoTime']
      cs_event = by_time['carState'][cs_time]
      cs, _cs_valid = cs_event.carState, bool(cs_event.valid)
    # Sampling is AFTER blocking carState receive and BEFORE control computation.
    # The default upper-bound policy is latest publication before controlsState;
    # alternate offsets expose its uncertainty without assuming startMonoTime is
    # the sample time (start precedes the blocking receive).
    cutoff = mono if args.sample_offset_ms is None else cs_time + int(args.sample_offset_ms * 1e6)
    latest = {k: streams.latest(k, cutoff) for k in ['liveParameters', 'liveTorqueParameters', 'liveDelay', 'frogpilotCarState', 'carOutput']}
    if getattr(args, 'fpcs_sample_at', 'carState') == 'controlsState':
      # Source-supported latest-publication upper bound, not receipt proof.
      # Keep other service identities/feedback cutoff unchanged.
      latest['frogpilotCarState'] = streams.latest('frogpilotCarState', mono)
    if mono in identity and observed.active:
      match = identity[mono]
      if match.get('liveParameters_candidates_within_2e9') != 1 and not match.get('liveParameters_candidates_control_equivalent', False):
        raise RuntimeError('Ambiguous liveParameters identity in frozen audit')
      lp_time = match['liveParameters_logMonoTime']
      lp_event = by_time['liveParameters'][lp_time]
      latest['liveParameters'] = (lp_event.liveParameters, lp_time, bool(lp_event.valid))
    if any(item[0] is None for item in latest.values()) or cs is None:
      skipped += 1
      continue
    lp = latest['liveParameters'][0]
    torque = latest['liveTorqueParameters'][0]
    delay = latest['liveDelay'][0]
    fp = latest['frogpilotCarState'][0]
    co = latest['carOutput'][0]
    software_active = feedback is not None and cutoff >= feedback.fixture.start_ns
    if software_active:
      simulated_output = feedback.output_at(cutoff)
      co = SimpleNamespace(actuatorsOutput=SimpleNamespace(steer=simulated_output.steer))
    active = bool(observed.active)
    variant_active = args.variant_start is None or mono / 1e9 >= args.variant_start
    if variant_active and ctrl.__class__ is not module.LatControlTorque:
      # Switch methods on the SAME controller instance; every filter, PID,
      # plant and breaker state is preserved at the intervention boundary.
      switch_controller(ctrl, module.LatControlTorque, cp, previous_active)
    toggles.lat_commit_setpoint = not args.no_commit
    toggles.lat_damping = not args.no_damping
    toggles.lat_friction_comp = not args.no_friction_comp
    vm.update_params(max(lp.stiffnessFactor, 0.1), max(lp.steerRatio, 0.1))
    if latest['liveTorqueParameters'][2] and (torque.useParams or args.force_offset):
      ctrl.update_live_torque_params(torque.latAccelFactorFiltered, torque.latAccelOffsetFiltered, torque.frictionCoefficientFiltered)
    if not active:
      ctrl.reset()
    if previous_output is None and args.seed_i:
      # One initial observed state, not per-frame forcing. Helps isolate
      # finite-segment initialization and is reported explicitly.
      ctrl.pid.i = float(observed.i)
    ctrl._replay_trace = {}
    if MODEL_CONTEXT_BUILDER is not None:
      ctrl._release_context = MODEL_CONTEXT_BUILDER(by_time['modelV2'].get(int(c.lateralPlanMonoTime)), mono)
    if getattr(args, 'integrated_model_context', False) and variant_active:
      model_event = by_time['modelV2'].get(int(c.lateralPlanMonoTime))
      ctrl.update_model_context(
        model_event.modelV2 if model_event is not None else None,
        int(c.lateralPlanMonoTime),
        bool(model_event.valid) if model_event is not None else False,
        mono,
      )
    steer, _, actual = ctrl.update(active, cs, vm, lp, limited, c.desiredCurvature, False, delay.lateralDelay + 0.1, None, None, toggles, fp)
    row = {
      'mono': mono / 1e9,
      'start_mono': c.startMonoTime / 1e9,
      'cs_mono': cs_time / 1e9,
      'sample_cutoff': cutoff / 1e9,
      'active': active,
      'limited': limited,
      'pressed': bool(cs.steeringPressed),
      'variant_active': variant_active,
      'software_feedback_active': software_active,
      'feedback_steer': float(co.actuatorsOutput.steer),
      'v_ego': cs.vEgo,
      'angle': cs.steeringAngleDeg,
      'rate': cs.steeringRateDeg,
      'column': fp.columnTorque,
      'curvature': c.desiredCurvature,
      'model_mono': c.lateralPlanMonoTime / 1e9,
      'delay': delay.lateralDelay + 0.1,
      'kp': float(ctrl.pid.k_p),
    }
    for k in METRICS:
      row['logged_' + k] = float(getattr(observed, k))
      row['replay_' + k] = float(getattr(actual, k))
    row.update(ctrl._replay_trace)
    model_event = by_time['modelV2'].get(int(c.lateralPlanMonoTime))
    if model_event is not None:
      row['model_action_curvature'] = float(model_event.modelV2.action.desiredCurvature)
      row['model_action_la'] = row['model_action_curvature'] * cs.vEgo**2
    error_scale = 1.0 + (np.interp(cs.vEgo, module.LOW_SPEED_X, module.LOW_SPEED_Y) / max(cs.vEgo, module.MIN_SPEED)) ** 2 / ctrl.torque_params.kp
    row['logged_tracked_setpoint'] = float(observed.actualLateralAccel + observed.error / error_scale)
    for k, (_, t, valid) in latest.items():
      row[k + '_age_ms'] = (mono - t) / 1e6
      row[k + '_valid'] = valid
    rows.append(row)
    if args.anchor_i_at is not None and i_anchor is None and active and mono / 1e9 >= args.anchor_i_at:
      # The rlog segment starts mid-drive; I has no natural decay and is not
      # reset on disengagement. Anchor its *post-update* logged state once,
      # after reference filters warm, before the scored event. There is no
      # per-frame forcing, and the anchored frame is explicitly recorded.
      ctrl.pid.i = float(observed.i)
      i_anchor = {'mono': mono / 1e9, 'logged_i': float(observed.i)}
    # Historical mode preserves recorded requests AND feedback together.
    # Software mode instead generates feedback from the candidate's own sends
    # and compares its own recent requests. Never mix the two conventions.
    limited = all(abs(req - co.actuatorsOutput.steer) > 1e-2 for req in history)
    candidate_request = feedback is not None and mono >= feedback.fixture.start_ns
    history.append(float(np.float32(steer)) if candidate_request else float(observed.output))
    previous_active = active
    if feedback is not None:
      publish_paired_request(feedback, streams, mono, c.desiredCurvature, observed.output, steer, active)
    previous_output = steer
  selected = [r for r in rows if args.start <= r['mono'] <= args.end and r['active']]
  if not selected:
    raise RuntimeError('No active scored rows')
  summary = {
    'scope': (
      'Recorded-input replay with candidate-generated TI command/feedback loop; not vehicle simulation.'
      if feedback is not None
      else 'Recorded-input replay; historical limiter feedback; not vehicle simulation.'
    ),
    'arguments': args.report_arguments,
    'rows_run': len(rows),
    'rows_scored': len(selected),
    'rows_skipped_missing_inputs': skipped,
    'integrator_anchor': i_anchor,
    'residuals': {k: distribution([r['replay_' + k] - r['logged_' + k] for r in selected]) for k in METRICS},
    'output_residual_counts': distribution([600 * (r['replay_output'] - r['logged_output']) for r in selected]),
    'plant_state_mismatch_rows': sum(r['replay_plantState'] != r['logged_plantState'] for r in selected),
    'limits': [
      'Full service validity/frequency health is not reconstructed.',
      'Initial setting snapshots are checked against the spec; continuous runtime toggle identity is not reconstructed.',
      'Partial-segment initial state; integrator anchoring is optional and reported.',
      'Generic cutoff inputs can differ from consumed inputs; audited liveParameters mapping overrides the cutoff only in mapped frames.',
      'Input identity and CarControl pairing require the separate audit; do not extrapolate its event-specific result.',
      'curvature_limited=False affects saturation reporting; output path uses recorded clipped curvature.',
    ],
  }
  output = Path(args.output_prefix)
  if feedback is not None:
    feedback.advance_until(feedback.fixture.end_ns)
    scored_sends = [(t, u) for t, u in feedback.sends if t / 1e9 >= args.start]
    deltas = [u - feedback.fixture.actual_sends[t] for t, u in scored_sends]
    summary['software_feedback'] = {
      'baseline_adapter_verification': feedback_baseline,
      'initial_send': feedback.fixture.initial_send,
      'initial_output': vars(feedback.fixture.initial_output),
      'scored_sends': len(scored_sends),
      'changed_send_count': sum(d != 0 for d in deltas),
      'send_difference_counts': distribution(deltas),
      'limits': 'Only software commands/feedback are simulated; wheel, sensors, model, live parameters and EPS state remain recorded.',
    }
    output.with_name(output.name + '-sends').with_suffix('.json').write_text(
      json.dumps(
        {
          'sends': [{'mono': t, 'counts': u, 'recorded_counts': feedback.fixture.actual_sends[t]} for t, u in feedback.sends],
          'outputs': [vars(r) for r in feedback.outputs],
        },
        indent=2,
      ),
      encoding='utf-8',
    )
  output.with_suffix('.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
  # Raw capnp buffers already occupy substantial memory. Stream the trace
  # instead of allocating a second, combined copy of every serialized row.
  trace_tmp = output.with_suffix('.jsonl.tmp')
  with trace_tmp.open('w', encoding='utf-8') as trace_file:
    for row in rows:
      trace_file.write(json.dumps(row) + '\n')
  trace_tmp.replace(output.with_suffix('.jsonl'))
  print(json.dumps({k: summary[k] for k in ['rows_run', 'rows_scored', 'residuals', 'output_residual_counts', 'plant_state_mismatch_rows']}, indent=2))
  return summary, rows
