"""Exact-input full-controller adapter promoted from replay_keepout_history.py.

Only software requests/feedback evolve. Sensors, planner, availability and apply schedule stay fixed.
"""

from collections import deque
from dataclasses import asdict
import hashlib
import json
from types import SimpleNamespace as NS

from . import run
from .audit_diagnostics import INPUTS, SERVICES, audit, read_events
from .provenance import (check_artifacts, check_sources, environment, finish_sources, read_json, resolve_ref,
                         sha256, source_snapshot, under, write_json)
from .evaluation_contract import canonical_request, completed_result

REPLAY_FILES = (*run.REPLAY_FILES, 'trace-stock-sends.json')
COMPONENT_UNITS = {
  **{name: 'm/s^2' for name in ('rawRequest', 'effectiveFeedforward', 'effectiveSetpoint', 'measurement',
                              'error', 'integralBefore', 'integralAfter', 'feedforward', 'pidOutput', 'frictionGate')},
  'measurementRate': 'm/s^3',
  **{name: 'right-positive TI counts' for name in ('inverseCommand', 'frictionCompensation', 'frictionRelay',
                                                'breakerBoost', 'commandBeforeSmoothing', 'command')},
  'freezeReasons': 'bitmask: limiter=1, steeringPressed=2, low-speed=4',
}
COMPONENT_METRICS = {
  'request_mps2': 'rawRequest', 'effective_setpoint_mps2': 'effectiveSetpoint',
  'effective_feedforward_mps2': 'effectiveFeedforward', 'integral_before_mps2': 'integralBefore',
  'integral_after_mps2': 'integralAfter', 'inverse_command_counts': 'inverseCommand',
  'controller_command_counts': 'command', 'compensation_counts': 'frictionCompensation',
}
LIMITATIONS = [
  'Exact nested diagnostic inputs and recorded model/camera clocks; no nearest-publication reconstruction.',
  'TI availability and physical motion stay recorded; candidates cannot predict avoidance of the dropout or its counterfactual recovery.',
  'Both software limiter histories evolve; stock requests normalize by 600, while measured EPS response is separately limited to about +/-308.',
  'One post-update integral/friction-gate anchor follows supplied warmup; limiter previous-command seeds are recorded at activation.',
  'Runtime controller gains/settings are recorded inputs; active-loop defaults during inactivity are not measurements.',
]


def component_snapshot(diagnostics, active):
  """Retain native controller components; inactive/default fields are unavailable."""
  if not active or diagnostics.version != 1:
    return None
  values = diagnostics.to_dict()
  return {name: values[name] for name in COMPONENT_UNITS}


def validate_baseline(path, prep_hash, history, runtime):
  baseline = run.validate_baseline(path, prep_hash, history, runtime, replay_files=REPLAY_FILES)
  if (baseline.get('method') != 'instrumented' or baseline.get('both_command_histories_qualified') is not True
      or baseline.get('serialized_controller_outputs_equal') is not True):
    raise ValueError('Instrumented baseline must qualify controller outputs and both TI and stock histories')
  return baseline


def controller_inputs(event, streams):
  """Resolve every declared nested identity, including inactive updates and non-applied controllers."""
  observed = event.controlsState.lateralControlState.torqueState
  d, mono = observed.mazdaDiagnostics, int(event.logMonoTime)
  if d.version != 1 or sorted(str(s.service) for s in d.inputs) != sorted(INPUTS):
    raise ValueError('Absent, unsupported or incomplete controller diagnostic identities')
  snapshots = {str(s.service): s for s in d.inputs}
  used = {}
  for name, snapshot in snapshots.items():
    identity = int(snapshot.logMonoTime)
    if not snapshot.seen or not 0 < identity <= mono or identity not in streams[name]:
      raise ValueError(f'Missing or inconsistent consumed identity: {name}')
    used[name] = streams[name][identity]
    if bool(used[name].valid) != bool(snapshot.valid):
      raise ValueError(f'Consumed validity differs: {name}')
  return snapshots, used


def replay(spec, events, output, variant, preparation, prep_hash, baseline_path=None):
  from .runtime import bootstrap, TestInterface, controller_source, expose_applied_feedback, load_controller_source

  before, runtime_before = source_snapshot(), environment()
  check_sources(preparation['repository_sources'])
  if runtime_before != preparation['environment']:
    raise ValueError('Runtime changed since preparation')
  if variant == 'candidate':
    validate_baseline(baseline_path, prep_hash, 'exact', runtime_before)
  bootstrap()
  from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
  from .controller_replay import switch_controller
  from .feedback import pure_limiter, pure_stock_limiter
  from .recorded_feedback import RecordedTiFeedback, serialized_steer

  streams = {name: {} for name in SERVICES | {'carParams'}}
  for e in events:
    if e.which() in streams:
      streams[e.which()][int(e.logMonoTime)] = e
  window = spec['window']
  anchor_ns, activation, start, end = (round(window[k] * 1e9) for k in ('anchor_i_at', 'activation', 'start', 'end'))
  original_ref = spec['baseline_controller']
  ref = spec['candidate_controller'] if variant == 'candidate' else original_ref
  original = load_controller_source(controller_source(original_ref), 'instrumented_original')
  candidate = load_controller_source(controller_source(ref), 'instrumented_' + variant)
  cp = next(iter(streams['carParams'].values())).carParams.as_builder()
  ctrl, vm = original.LatControlTorque(cp, TestInterface(), .01), VehicleModel(cp)
  limiter, limiter_hash = pure_limiter(spec['limiter'])
  stock, stock_limits, stock_hashes = pure_stock_limiter(spec['limiter'])
  feedback = RecordedTiFeedback(sorted(streams['carOutput'].items()), streams['carControl'], activation, end, limiter,
                                stock_limiter=stock, stock_limits=stock_limits)
  history, limited, previous_active, anchor, switched = deque([0.0] * 3, maxlen=3), False, False, None, None
  rows, skipped = [], []
  toggle_names = ['lat_commit_setpoint', 'lat_damping', 'lat_friction_comp', 'lat_output_filter', 'lat_no_friction_relay']
  for mono, event in sorted(streams['controlsState'].items()):
    if mono >= end:
      break
    if event.controlsState.lateralControlState.which() != 'torqueState':
      continue
    observed = event.controlsState.lateralControlState.torqueState
    d = observed.mazdaDiagnostics
    try:
      snapshots, used = controller_inputs(event, streams)
    except ValueError:
      if mono >= anchor_ns:
        raise
      skipped.append(mono)
      continue
    cs, lp, ltp = used['carState'].carState, used['liveParameters'].liveParameters, used['liveTorqueParameters'].liveTorqueParameters
    vm.update_params(max(lp.stiffnessFactor, .1), max(lp.steerRatio, .1))
    if snapshots['liveTorqueParameters'].checksPassed:
      ctrl.update_live_torque_params(ltp.latAccelFactorFiltered, ltp.latAccelOffsetFiltered, ltp.frictionCoefficientFiltered)
    if d.settings & ~31:
      raise ValueError('Unsupported controller setting bits')
    toggles = NS(**{name: bool(d.settings & (1 << bit)) for bit, name in enumerate(toggle_names)}, ti_steer_max=600.0)
    if observed.active:
      ctrl.pid._k_p = [[0.0], [d.kp]]
    if variant == 'candidate' and switched is None and mono >= activation:
      switch_controller(ctrl, candidate.LatControlTorque, cp, previous_active)
      switched = {'mono': mono, 'previous_active': previous_active, 'migration': 'shared switch_controller'}
    if not observed.active:
      ctrl.reset()
    model = used['modelV2']
    ctrl.update_model_context(model.modelV2, int(model.logMonoTime), snapshots['modelV2'].checksPassed, d.modelContextNow, d.cameraContextNow)
    curvature = d.rawRequest / (cs.vEgo ** 2) if cs.vEgo else event.controlsState.desiredCurvature
    fp = used['frogpilotCarState'].frogpilotCarState if snapshots['frogpilotCarState'].checksPassed else None
    applied_feedback = feedback.previous_applied_for_update(
      mono, int(used['carOutput'].logMonoTime), limiter_frozen=limited,
    )
    expose_applied_feedback(ctrl, mono, applied_feedback)
    feedback_steer = applied_feedback.observed_steer
    steer, _, actual = ctrl.update(bool(observed.active), cs, vm, lp, limited, curvature, bool(d.curvatureLimited),
                                  used['liveDelay'].liveDelay.lateralDelay + .1, None, model.modelV2, toggles, fp)
    feedback.publish(mono, steer)
    if mono >= start:
      rows.append({'mono': mono, 'active': bool(observed.active), 'output': steer, 'recorded_output': float(observed.output),
                   'serialized_output_match': serialized_steer(steer) == observed.output,
                   'command_residual': abs(actual.mazdaDiagnostics.command - d.command) if observed.active else None,
                   'plant_match': actual.plantState == observed.plantState,
                   'limited_match': limited == bool(d.freezeReasons & 1) if observed.active else True,
                   'consumed': {name: int(s.logMonoTime) for name, s in snapshots.items()},
                   'model_context_now': int(d.modelContextNow), 'camera_context_now': int(d.cameraContextNow),
                   'previous_applied_feedback': (
                     asdict(applied_feedback.candidate) if applied_feedback.candidate_available else None
                   )})
      components = component_snapshot(actual.mazdaDiagnostics, bool(observed.active))
      rows[-1].update(recorded_components=component_snapshot(d, bool(observed.active)), replay_components=components)
      term_ablation = getattr(ctrl, '_term_ablation_trace', None)
      if term_ablation is not None:
        rows[-1]['term_ablation_trace'] = term_ablation
      if components is not None:
        rows[-1].update({name: components[field] for name, field in COMPONENT_METRICS.items()})
        rows[-1].update(limiter_feedback_limited=limited, consumed_feedback_steer=feedback_steer)
    if anchor is None and observed.active and mono >= anchor_ns:
      ctrl.pid.i, ctrl.fric_gate_filter.x = float(d.integralAfter), float(d.frictionGate)
      anchor = {'mono': mono, 'integral_after': float(d.integralAfter), 'friction_gate_after': float(d.frictionGate)}
    # Preserve the production caller's previous-history comparison before appending this request.
    limited = all(abs(request - feedback_steer) > 1e-2 for request in history)
    history.append(feedback.history_request(mono, steer, observed.output))
    previous_active = bool(observed.active)
  feedback.finish()
  if anchor is None or anchor['mono'] >= activation or not rows or not any(row['active'] for row in rows):
    raise ValueError('Missing active coverage or settled pre-activation state anchor')
  ti = [{'mono': a.applied_at, 'counts': int(feedback.counts[a.sequence]), 'recorded_counts': a.recorded} for a in feedback.applies]
  stock_sends = [{'mono': a.applied_at, 'counts': int(feedback.stock_counts[a.sequence]), 'recorded_counts': a.stock_recorded} for a in feedback.applies]
  residual = max(row['command_residual'] for row in rows if row['active'])
  output_exact = all(row['serialized_output_match'] for row in rows)
  exact = output_exact and residual < 1 and all(row['plant_match'] and row['limited_match'] for row in rows)
  exact = exact and all(row['counts'] == row['recorded_counts'] for row in ti + stock_sends)
  bounds = all(abs(row['counts']) <= 600 for row in ti + stock_sends)
  selected = [a for a in feedback.applies if a.applied_at >= start]
  availability = {'ti_loss': sum(a.ti_selected and not b.ti_selected for a, b in zip(selected, selected[1:], strict=False)),
                  'ti_reentry': sum(not a.ti_selected and b.ti_selected for a, b in zip(selected, selected[1:], strict=False)),
                  'stock_fallback_applies': sum(a.active and not a.ti_selected for a in selected),
                  'inactive_applies': sum(not a.active for a in selected)}
  output.mkdir()
  write_json(output / 'trace.json', {'anchor': anchor, 'skipped_before_anchor': skipped, 'candidate_activation': switched,
                                   'limiter_initialization': {'ti': feedback.applies[0].previous, 'stock': feedback.applies[0].stock_previous},
                                   'component_trace': {'format_version': 1, 'units': COMPONENT_UNITS,
                                                       'feedback_units': 'published steer normalized by 600; opposite controller-count sign',
                                                       'scope': 'Recorded and replayed active controller components; integral is not a TI-count contribution.'},
                                   'capabilities': {'sample_identity_fields': ['consumed', 'model_context_now', 'camera_context_now'],
                                                    'metrics': {name: COMPONENT_UNITS[field] for name, field in COMPONENT_METRICS.items()}},
                                   'availability': availability, 'scope': LIMITATIONS})
  write_json(output / 'trace-sends.json', {'sends': ti})
  write_json(output / 'trace-stock-sends.json', {'sends': stock_sends})
  with (output / 'trace.jsonl').open('x', encoding='utf-8', newline='\n') as stream:
    for row in rows:
      stream.write(json.dumps(row, allow_nan=False, sort_keys=True) + '\n')
  if environment() != runtime_before:
    raise ValueError('Runtime changed during replay')
  result = {'format_version': 2, 'stage': 'replayed', 'method': 'instrumented', 'variant': variant, 'history': 'exact',
            'qualification': ('exact_recorded_commands' if exact else 'failed_baseline') if variant == 'baseline' else 'candidate_commands_only',
            'both_command_histories_qualified': exact if variant == 'baseline' else True, 'command_bounds_pass': bounds,
            'serialized_controller_outputs_equal': output_exact,
            'preparation_sha256': prep_hash, 'baseline_result_sha256': sha256(baseline_path) if baseline_path else None,
            'repository_sources': finish_sources(before), 'environment': runtime_before, 'input_sha256': preparation['input_sha256'],
            'controller_sources': {r: hashlib.sha256(controller_source(r).encode()).hexdigest() for r in (original_ref, ref)},
            'limiter_source_sha256': limiter_hash, 'stock_limiter_sources': stock_hashes,
            'output_sha256': {name: sha256(output / name) for name in REPLAY_FILES}, 'frames': len(rows), 'sends': len(ti),
            'controller_residual_counts': {'max': residual}, 'limitations': LIMITATIONS}
  write_json(output / 'result.json', result)
  if not bounds or variant == 'baseline' and not exact:
    raise ValueError('Instrumented baseline/controller/actuator qualification failed; evidence retained')
  return result


def compare(output, spec, same_source):
  from .evaluate import compare_commands

  result = compare_commands(output / 'baseline/trace-sends.json', output / 'candidate/trace-sends.json', spec['window'],
                            same_source, historical_bounds=False)
  result['stock'] = compare_commands(output / 'baseline/trace-stock-sends.json', output / 'candidate/trace-stock-sends.json', spec['window'],
                                     same_source, historical_bounds=False)
  result['stock']['domain'] = 'Stock wire request counts; candidate minus baseline; normalization 600, not measured EPS 308'
  result['availability'] = read_json(output / 'candidate/trace.json')['availability']
  return result


def evaluate(request, data_root, output, result):
  before, runtime_before = source_snapshot(), environment()
  case = request['case']
  spec = dict(case['experiment'], candidate_controller=request['candidate_revision'])
  for key in ('baseline_controller', 'warmup_controller', 'candidate_controller', 'limiter'):
    spec[key] = resolve_ref(spec[key])
    if spec[key] == 'worktree':
      raise ValueError('Instrumented replay requires pinned source revisions')
  if spec['baseline_controller'] != spec['warmup_controller'] or not spec['force_offset']:
    raise ValueError('Instrumented baseline requires original warmup source and explicit recorded offset policy')
  window = spec['window']
  if not 0 < window['anchor_i_at'] < window['activation'] <= window['start'] < window['end']:
    raise ValueError('Require 0 < anchor < activation <= start < end')
  resolved = canonical_request(dict(request, candidate_revision=spec['candidate_controller'], case=dict(case, experiment=spec)), resolved=True)
  write_json(output / 'request.json', request)
  write_json(output / 'resolved-request.json', resolved)
  write_json(output / 'experiment.json', spec)
  paths = [under(data_root, name) for name in spec['rlogs']]
  events = list(read_events(paths))
  prepared = output / 'prepared'
  prepared.mkdir()
  identity = audit(events, round(window['anchor_i_at'] * 1e9), round(window['end'] * 1e9))
  write_json(prepared / 'audit.json', identity)
  if not identity['active_identity_coverage_complete']:
    raise ValueError('Instrumented diagnostic identity audit failed; load adjacent segments or repair the declared inputs')
  commands = {int(e.logMonoTime): e.carControl for e in events if e.which() == 'carControl'}
  controllers = {int(e.logMonoTime): e.controlsState for e in events if e.which() == 'controlsState'}
  for row in identity['rows']:
    command = commands[row['carControl']]
    observed = controllers[row['controlsState']].lateralControlState.torqueState
    if command.actuators.steer != observed.output or bool(command.latActive) != bool(observed.active):
      raise ValueError('Applied command differs from its exact controller publication')
  run.settings_check(paths, spec['settings'])
  for path in paths:
    initial = next((e.initData for e in read_events([path]) if e.which() == 'initData'), None)
    if initial is None or str(initial.gitCommit) != spec['baseline_controller']:
      raise ValueError('Recorded source does not match the declared original baseline')
  write_json(prepared / 'spec.json', spec)
  write_json(prepared / 'mapping.json', {'frames': identity['rows']})
  write_json(prepared / 'sampling.json', {'method': 'exact_nested_identities', 'window_ns': identity['window_ns']})
  preparation = {'format_version': 2, 'stage': 'prepared', 'method': 'instrumented', 'input_sha256': case['input_sha256'],
                 'environment': runtime_before, 'repository_sources': finish_sources(before),
                 'prepared_sha256': {name: sha256(prepared / name) for name in run.PREPARED_FILES}}
  write_json(prepared / 'preparation.json', preparation)
  prep_hash, baseline_path = sha256(prepared / 'preparation.json'), output / 'baseline/result.json'
  baseline = replay(spec, events, output / 'baseline', 'baseline', preparation, prep_hash)
  result['qualification'] = baseline['qualification']
  candidate = replay(spec, events, output / 'candidate', 'candidate', preparation, prep_hash, baseline_path)
  if environment() != runtime_before or any(sha256(path) != case['input_sha256'][name] for name, path in zip(spec['rlogs'], paths, strict=True)):
    raise ValueError('Runtime or raw data changed during evaluation')
  check_artifacts(prepared, preparation['prepared_sha256'], run.PREPARED_FILES)
  same = baseline['controller_sources'][spec['baseline_controller']] == candidate['controller_sources'][spec['candidate_controller']]
  result.update(completed_result(resolved, baseline, candidate, compare(output, spec, same)))
