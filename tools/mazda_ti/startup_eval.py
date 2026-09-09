"""Isolated Mazda startup evidence through real openpilot process interfaces."""

import argparse
import ast
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import importlib
import json
import os
import platform
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import traceback
import uuid

from .provenance import ROOT, sha256, write_json
from .ingest import WHITELISTED_SETTINGS


class StartupUnsupported(RuntimeError):
  """The declared process runtime cannot be exercised on this host."""


def capability():
  missing = []
  if platform.system() != 'Linux':
    missing.append('linux_openpilot_runtime')
  try:
    importlib.import_module('msgq.ipc_pyx')
  except ImportError as error:
    missing.append(f'msgq.ipc_pyx: {error}')
  if not missing and (importlib.util.find_spec('openpilot.selfdrive.controls.controlsd') is None or
                      not (ROOT / 'selfdrive/test/process_replay/process_replay.py').is_file()):
    missing.append('process_replay_controlsd')
  return {'schema_boundary_supported': not missing, 'full_process_supported': not missing, 'missing': missing}


@contextmanager
def isolated_environment():
  """Own Params/msgq state for one run and restore every ambient variable."""
  keys = ('PARAMS_ROOT', 'OPENPILOT_PREFIX', 'REPLAY', 'SIMULATION')
  original = {key: os.environ.get(key) for key in keys}
  with tempfile.TemporaryDirectory(prefix='mazda-startup-params-') as params_root:
    prefix = 'mazda-startup-' + uuid.uuid4().hex
    os.environ.update(PARAMS_ROOT=params_root, OPENPILOT_PREFIX=prefix, REPLAY='1', SIMULATION='1')
    try:
      yield {'params_root': params_root, 'messaging_prefix': prefix}
    finally:
      for key, value in original.items():
        if value is None:
          os.environ.pop(key, None)
        else:
          os.environ[key] = value


def construct_controlsd_subscriptions(source=None, messaging_module=None):
  """Execute controlsd's real constructor subscription block and validate it."""
  source = source or (ROOT / 'selfdrive/controls/controlsd.py').read_text(encoding='utf-8')
  tree = ast.parse(source)
  controls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Controls')
  init = next(node for node in controls.body if isinstance(node, ast.FunctionDef) and node.name == '__init__')
  start = next(index for index, node in enumerate(init.body) if isinstance(node, ast.Assign) and
               any(isinstance(target, ast.Attribute) and target.attr == 'sensor_packets' for target in node.targets))
  stop = next(index for index, node in enumerate(init.body) if isinstance(node, ast.Assign) and
              any(isinstance(target, ast.Attribute) and target.attr == 'sm' for target in node.targets))
  messaging = messaging_module
  if messaging is None:
    from cereal import messaging
  class RecordingMessaging:
    services = None
    sub_sock = staticmethod(messaging.sub_sock)

    @classmethod
    def SubMaster(cls, services, **kwargs):
      cls.services = tuple(services)
      return messaging.SubMaster(services, **kwargs)

  host = type('ConstructorSubscriptions', (), {})()
  block = ast.Module(body=init.body[start:stop + 1], type_ignores=[])
  exec(compile(block, 'controlsd-constructor-subscriptions', 'exec'),
       {'self': host, 'messaging': RecordingMessaging, 'SIMULATION': True, 'REPLAY': True, 'DT_CTRL': 0.01})
  services = set(RecordingMessaging.services or ())
  if 'carState' in services:
    raise ValueError('carState must remain a dedicated subscriber, not a SubMaster service')
  return host


def controls_constructor_boundary():
  return construct_controlsd_subscriptions()


def qualify_nested_timestamps(before, after):
  before_by_id = {row['id']: row for row in before}
  after_by_id = {row['id']: row for row in after}
  if not before_by_id or before_by_id.keys() != after_by_id.keys():
    raise ValueError('timestamp identities differ')
  offsets = {after_by_id[key]['outer_ns'] - row['outer_ns'] for key, row in before_by_id.items()}
  if len(offsets) != 1:
    raise ValueError('outer timestamp transform is not uniform')
  if any(after_by_id[key]['nested_ns'] != row['nested_ns'] for key, row in before_by_id.items()):
    raise ValueError('nested diagnostic timestamp changed during transform')
  return {'status': 'qualified', 'outer_offset_ns': offsets.pop(), 'records': len(before_by_id)}


def actual_boundary():
  """Pass a real inactive carState event through controlsd's dedicated socket."""
  from cereal import messaging
  from openpilot.selfdrive.controls import controlsd

  messaging.toggle_fake_events(True)
  try:
    host = construct_controlsd_subscriptions(messaging_module=messaging)
    publisher = messaging.PubMaster(['carState'])
    with ThreadPoolExecutor(max_workers=1) as executor:
      received = executor.submit(messaging.recv_one_retry, host.car_state_sock)
      if not publisher.wait_for_readers_to_update('carState', 2):
        raise StartupUnsupported('controlsd dedicated carState subscriber did not connect')
      message = messaging.new_message('carState')
      message.carState.vEgo = 0.0
      publisher.send('carState', message)
      event = received.result(timeout=2)
    if event.carState.vEgo != 0.0:
      raise StartupUnsupported('carState schema message did not cross controlsd dedicated subscriber boundary')
    return {'schema': 'carState', 'cold_start': 'passed', 'inactive': 'passed'}
  finally:
    messaging.toggle_fake_events(False)


def _bounded_startup_events(events, max_carstate_messages):
  carstate_times = [int(event.logMonoTime) for event in events if event.which() == 'carState']
  if len(carstate_times) < max_carstate_messages:
    raise StartupUnsupported(f'rlog has only {len(carstate_times)} carState events; need {max_carstate_messages}')
  cutoff = carstate_times[max_carstate_messages - 1]
  bounded = [event for event in events if int(event.logMonoTime) <= cutoff]
  missing = {'carState', 'can'} - {event.which() for event in bounded}
  if missing:
    raise StartupUnsupported(f'bounded startup input lacks {sorted(missing)}')
  return bounded, cutoff


def _read_rlog_events(log, data):
  events = []
  parse_exception = None
  try:
    for event in log.Event.read_multiple_bytes(data):
      events.append(event)
  except Exception as error:
    parse_exception = f'{type(error).__name__}: {error}'
  if not events:
    raise StartupUnsupported(f'rlog yielded no complete events: {parse_exception}')
  return events, parse_exception


def _diagnostic_timestamp_result(source_events, output_events):
  source_identities = {}
  for event in source_events:
    source_identities.setdefault(event.which(), set()).add(int(event.logMonoTime))
  all_source_times = {int(event.logMonoTime) for event in source_events}
  before, after = [], []
  for index, event in enumerate(output_events):
    if event.which() != 'controlsState':
      continue
    lateral = event.controlsState.lateralControlState
    if lateral.which() != 'torqueState':
      continue
    diagnostics = lateral.torqueState.mazdaDiagnostics
    if diagnostics.version == 0:
      continue
    carstate = next((row for row in diagnostics.inputs if row.service == 'carState' and row.seen), None)
    if carstate is None:
      continue
    for row in diagnostics.inputs:
      if row.seen and int(row.logMonoTime) not in source_identities.get(row.service, set()):
        raise ValueError(f'generated diagnostic references unknown {row.service} identity {row.logMonoTime}')
    trigger_time = int(event.logMonoTime) - 4_000_000
    if trigger_time not in all_source_times:
      raise ValueError(f'generated outer timestamp has no process-replay input trigger {trigger_time}')
    identity = f'controlsState:{index}:{carstate.logMonoTime}'
    before.append({'id': identity, 'outer_ns': trigger_time, 'nested_ns': int(carstate.logMonoTime)})
    after.append({'id': identity, 'outer_ns': int(event.logMonoTime), 'nested_ns': int(carstate.logMonoTime)})
  if not before:
    raise ValueError('process replay generated no Mazda diagnostic input identities')
  result = qualify_nested_timestamps(before, after)
  result['identity_source'] = 'generated controlsState torqueState.mazdaDiagnostics.inputs'
  return result


def transition_stderr_diagnostics(stderr, expected_invalid_services=()):
  """Accept only declared fake-service exclusions and requested invalid inputs."""
  if not stderr:
    return []
  try:
    records = [json.loads(line) for line in stderr.splitlines()]
  except json.JSONDecodeError:
    return None
  expected_events = {'controlsd.initialized', 'commIssue'}
  expected_unavailable = {'testJoystick'}
  expected_invalid_services = set(expected_invalid_services)
  if len(records) != 2 or {record.get('event') for record in records if isinstance(record, dict)} != expected_events:
    return None
  initialized = next(record for record in records if record.get('event') == 'controlsd.initialized')
  comm_issue = next(record for record in records if record.get('event') == 'commIssue')
  if (not isinstance(initialized, dict) or initialized.get('error') is not True or initialized.get('invalid') != [] or
      initialized.get('not_freq_ok') != [] or set(initialized.get('not_alive', ())) != expected_unavailable):
    return None
  if (not isinstance(comm_issue, dict) or comm_issue.get('error') is not True or
      set(comm_issue.get('invalid', ())) != expected_invalid_services or comm_issue.get('not_freq_ok') != [] or
      set(comm_issue.get('not_alive', ())) != expected_unavailable):
    return None
  return records


def inject_transition_harness(events, log):
  """Add only missing controlsd input schemas with declared per-message identities."""
  # These are all actual controlsd subscriptions absent from the recorded
  # cutout.  `testJoystick` remains deliberately absent: it is the one fake
  # service declared by the process-replay configuration and is retained as a
  # harness exclusion in the evidence.
  services = ('managerState', 'pandaStates', 'frogpilotCarState', 'frogpilotPlan', 'liveDelay')
  car_states = [event for event in events if event.which() == 'carState']
  generated = []
  retained = [event for event in events if event.which() != 'pandaStates']
  events[:] = retained
  for index, car_state in enumerate(car_states):
    status = 'valid'
    # The retained cutout spans activation in 15, TI bypass/disengage in 16,
    # and re-entry in 17. Exercise health failures after the final re-entry.
    if index == 16_200:
      status = 'missing'
    elif 14_900 <= index < 16_150:
      status = 'stale_gap'
    elif index == 16_360:
      status = 'invalid'
    ti_active = not (9_966 <= index < 10_151)
    # Replay cold-starts controlsd, so recorded panda state cannot claim the
    # pre-existing engagement.  Supply the actual subscriber schema with a
    # declared controlsAllowed fixture.  This is an input to the process, not
    # a CAN sender or a vehicle state change.
    for offset, service in enumerate(services, start=1):
      if status in ('missing', 'stale_gap') and service == 'frogpilotCarState':
        generated.append({'service': service, 'status': 'stale' if status == 'stale_gap' else status, 'source_mono_time': None, 'frame': index,
                          'source_car_state_mono_time': int(car_state.logMonoTime), 'derived_from': 'recorded_carState'})
        continue
      mono = int(car_state.logMonoTime) - offset
      event = log.Event.new_message(logMonoTime=mono, valid=not (status == 'invalid' and service == 'frogpilotCarState'))
      event.init(service, 1) if service == 'pandaStates' else event.init(service)
      if service == 'pandaStates':
        event.pandaStates[0].controlsAllowed = True
      if service == 'frogpilotCarState':
        event.frogpilotCarState.tiActive = ti_active
        event.frogpilotCarState.alwaysOnLateralEnabled = 10_160 <= index < 15_000
      if service == 'frogpilotPlan':
        event.frogpilotPlan.lateralCheck = True
      events.append(event.as_reader())
      generated.append({'service': service, 'status': status if service == 'frogpilotCarState' else 'valid',
                        'source_mono_time': mono, 'frame': index, 'source_car_state_mono_time': int(car_state.logMonoTime),
                        'derived_from': 'recorded_carState',
                        'ti_active': ti_active if service == 'frogpilotCarState' else None,
                        'always_on_lateral': bool(10_160 <= index < 15_000) if service == 'frogpilotCarState' else None})
    # These are serialized carState events at the real dedicated subscriber.
    # Enable pulses counter unrelated retained user-disable events; the four
    # bounded inactive windows make the fault observations explicit rather
    # than relying on a later inactive frame by inference.
    inactive_windows = ((14_890, 16_160), (16_190, 16_220), (16_350, 16_400))
    action = None
    for start, end in inactive_windows:
      if index == start:
        action = 'buttonCancel'
        break
      if start < index < end:
        action = None
        break
      if index == end:
        action = 'buttonEnable'
        break
    else:
      if index % 10 == 0:
        action = 'buttonEnable'
    if action is not None:
      replaced = car_state.as_builder()
      replaced.carState.init('events', 1)
      replaced.carState.events[0].name = action
      events.remove(car_state)
      events.append(replaced.as_reader())
      generated.append({'service': 'carState', 'status': 'valid', 'source_mono_time': int(car_state.logMonoTime),
                        'frame': index, 'event': action, 'derived_from': 'recorded_carState'})
  events.sort(key=lambda event: int(event.logMonoTime))
  return generated


def actual_full_process(rlog, max_carstate_messages=100, require_inactive=True, observer=None, all_segments=False,
                        transition_harness=False):
  """Cold-start real controlsd under process_replay and verify inactive output."""
  from cereal import custom, log
  # These module-level flags must be present on controlsd's first import. The
  # normal process-replay launcher also sets them before ManagerProcess.prepare.
  from openpilot.selfdrive.controls import controlsd  # establish manager import order
  from openpilot.selfdrive.test.process_replay.process_replay import controlsd_config_callback, get_process_config, ProcessContainer, replay_process

  del controlsd
  rlogs = [Path(path) for path in (rlog if isinstance(rlog, (list, tuple)) else [rlog])]
  per_file = []
  events = []
  recorded_settings = None
  for path in rlogs:
    file_events, parse_exception = _read_rlog_events(log, path.read_bytes())
    initial = next((event for event in file_events if event.which() == 'initData'), None)
    if initial is None:
      raise StartupUnsupported(f'supplied rlog has no initData settings snapshot: {path.parent.name}')
    values = {str(entry.key): bytes(entry.value) for entry in initial.initData.params.entries
              if str(entry.key) in WHITELISTED_SETTINGS}
    missing_settings = sorted(WHITELISTED_SETTINGS - set(values))
    if missing_settings:
      raise StartupUnsupported(f'supplied rlog lacks whitelisted settings: {missing_settings}')
    if recorded_settings is None:
      recorded_settings = values
    elif values != recorded_settings:
      raise StartupUnsupported(f'whitelisted settings changed between supplied segments: {path.parent.name}')
    events.extend(file_events)
    per_file.append({'label': f'{path.parent.name}/{path.name}', 'sha256': sha256(path), 'bytes': path.stat().st_size,
                     'complete_events': len(file_events), 'trailing_parse_exception': parse_exception})
  events.sort(key=lambda event: int(event.logMonoTime))
  generated_harness = []
  if transition_harness:
    generated_harness = inject_transition_harness(events, log)
  startup_events = events if all_segments else _read_rlog_events(log, rlogs[-1].read_bytes())[0]
  bounded, cutoff = _bounded_startup_events(startup_events, max_carstate_messages)
  # Later route segments do not repeat carParams. Carry the exact earlier event
  # into the isolated startup input rather than synthesizing one.
  if not any(event.which() == 'carParams' for event in bounded):
    prior_car_params = [event for event in events if event.which() == 'carParams']
    if not prior_car_params:
      raise StartupUnsupported('supplied rlogs contain no prior carParams event')
    bounded.append(prior_car_params[-1])
    bounded.sort(key=lambda event: int(event.logMonoTime))
  car_params = next(event.carParams for event in events if event.which() == 'carParams')
  if car_params.carName != 'mazda':
    raise StartupUnsupported(f'expected Mazda rlog, got carName={car_params.carName!r}')

  cfg = get_process_config('controlsd')
  if cfg.config_callback is not controlsd_config_callback:
    raise ValueError('controlsd process replay configuration callback changed')
  replay_controls_state = next((event.controlsState for event in reversed(startup_events) if event.which() == 'controlsState'), None)
  if replay_controls_state is None:
    raise StartupUnsupported('supplied rlogs contain no controlsState for ReplayControlsState initialization')
  # The crash corpus never reached a post-initialization controlsState, so the
  # stock config callback rejects it. controlsd only reads vCruise from this
  # replay seed; preserve the real recorded message and keep the run inactive.
  def configure_isolated_startup(params, _process_cfg, _events):
    params.put('ReplayControlsState', replay_controls_state.as_builder().to_bytes())
    from openpilot.frogpilot.common.frogpilot_variables import (FrogPilotVariables, frogpilot_default_params,
                                                                get_frogpilot_toggles, misc_tuning_levels,
                                                                params_default, params_memory)
    # Build the complete toggle namespace from repository defaults without the
    # started=True blocking Params reads. Vehicle identity still comes from the
    # explicit Mazda process-replay fingerprint and recorded CarParams.
    # Avoid FrogPilotVariables.__init__: it derives branch metadata and may emit
    # host Git diagnostics that have no bearing on this repository-default,
    # inactive fixture. Reproduce its Params default initialization explicitly.
    variables = FrogPilotVariables.__new__(FrogPilotVariables)
    variables.frogpilot_toggles = get_frogpilot_toggles()
    variables.tuning_levels = {key: level for key, _, level, _ in frogpilot_default_params + misc_tuning_levels}
    variables.development_branch = False
    variables.release_branch = False
    variables.staging_branch = False
    variables.testing_branch = False
    variables.vetting_branch = False
    variables.frogpilot_toggles.block_user = False
    variables.frogpilot_toggles.frogs_go_moo = False
    variables.frogpilot_toggles.tuning_level = 3
    variables.frogpilot_toggles.use_higher_bitrate = False
    variables.frogpilot_toggles.use_konik_server = False
    for key, value, _, _ in frogpilot_default_params:
      params_default.put(key, value)
    params_memory.put('FrogPilotTuningLevels', json.dumps(variables.tuning_levels))
    variables.update('', False)
    toggles = get_frogpilot_toggles()
    toggles.startup_alert_top = str(toggles.startup_alert_top or '')
    toggles.startup_alert_bottom = str(toggles.startup_alert_bottom or '')
    params_memory.put('FrogPilotToggles', json.dumps(toggles.__dict__))
    for key, value in recorded_settings.items():
      params.put(key, value)
  cfg.config_callback = configure_isolated_startup
  if transition_harness:
    cfg.pubs = [*cfg.pubs, 'frogpilotCarState', 'frogpilotPlan', 'liveDelay']
  captured = {}
  original_run_step = ProcessContainer.run_step
  def diagnostic_run_step(container, *args, **kwargs):
    try:
      return original_run_step(container, *args, **kwargs)
    except Exception as error:
      process = container.process.proc
      raise RuntimeError(f'{error}; child_alive={process.is_alive()}; child_exitcode={process.exitcode}') from error
  ProcessContainer.run_step = diagnostic_run_step
  try:
    custom_params = {'CarParams': car_params.as_builder().to_bytes(),
                     'FrogPilotCarParams': custom.FrogPilotCarParams.new_message().to_bytes(),
                     'TorqueInterceptorEnabled': True}
    outputs = replay_process(cfg, bounded, fingerprint=car_params.carFingerprint,
                             custom_params=custom_params, captured_output_store=captured, disable_progress=True)
  except Exception as error:
    raise RuntimeError(f'{error}; process_output={captured.get("controlsd", {})}') from error
  finally:
    ProcessContainer.run_step = original_run_step
  counts = Counter(event.which() for event in outputs)
  forbidden = sorted(set(counts) & {'sendcan', 'can'})
  if forbidden:
    raise ValueError(f'controlsd-only replay unexpectedly emitted vehicle-output services: {forbidden}')

  car_controls = [event.carControl for event in outputs if event.which() == 'carControl']
  controls_states = [event.controlsState for event in outputs if event.which() == 'controlsState']
  inactive = [cc for cc in car_controls if not cc.latActive and not cc.longActive and cc.actuators.steer == 0.0]
  if not car_controls or not controls_states:
    raise ValueError('controlsd produced no carControl or controlsState messages')
  if require_inactive and len(inactive) != len(car_controls):
    raise ValueError(f'inactive startup produced {len(car_controls) - len(inactive)} active or steering-output carControl messages')
  child_error = captured.get('controlsd', {}).get('err', '').strip()
  expected_invalid = {row['service'] for row in generated_harness if row['status'] == 'invalid'}
  stderr_diagnostics = transition_stderr_diagnostics(child_error, expected_invalid)
  if child_error and (require_inactive or stderr_diagnostics is None):
    raise ValueError(f'controlsd wrote stderr: {child_error}')

  try:
    git_head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True, stderr=subprocess.STDOUT).strip()
  except (OSError, subprocess.CalledProcessError) as error:
    raise StartupUnsupported(f'runtime checkout Git identity unavailable: {error}') from error

  transform = _diagnostic_timestamp_result(bounded, outputs)
  result = {
    'process': 'controlsd', 'interface': 'selfdrive.test.process_replay.replay_process',
    'cold_start': 'passed', 'inactive': {'status': 'passed', 'frames': len(inactive)},
    'input': {'rlogs': per_file, 'events': len(bounded),
              'carState_events': max_carstate_messages, 'cutoff_mono_time': cutoff, 'carName': car_params.carName,
              'startup_source': 'all_supplied_segments' if all_segments else len(rlogs) - 1,
              'selected_segments': [path.parent.name for path in rlogs],
              'recorded_settings': {key: value.decode('utf-8') for key, value in sorted(recorded_settings.items())},
              'generated_harness_inputs': generated_harness if transition_harness else [],
              'fingerprint': car_params.carFingerprint,
              'fingerprint_mode': 'explicit process_replay fixture; no live fingerprinting',
              'frogpilot_car_params': 'isolated schema-default fixture; no safety process is started',
              'torque_interceptor_enabled': True},
    'outputs': dict(sorted(counts.items())),
    'no_vehicle_output': {'status': 'passed', 'forbidden_services': ['can', 'sendcan'], 'observed': forbidden},
    'process_output': captured.get('controlsd', {}),
    'process_output_classification': ('empty_stderr' if not child_error else 'expected_harness_exclusions_and_exercised_faults'),
    'transition_stderr_diagnostics': stderr_diagnostics, 'timestamp_transform': transform,
    'runtime_source': {'root': str(ROOT), 'git_head': git_head, 'identity': source_identity()},
  }
  if observer is not None:
    result['observed_transition'] = observer(events, outputs)
  return result


def source_identity():
  files = ('selfdrive/controls/controlsd.py', 'selfdrive/test/process_replay/process_replay.py',
           'selfdrive/car/mazda/lateral_diagnostics.py', 'cereal/car.capnp', 'cereal/log.capnp',
           'tools/mazda_ti/startup_eval.py')
  return {name: sha256(ROOT / name) for name in files}


def run(output, profile='full_process', rlog=None, max_carstate_messages=100, capability=capability, boundary=None):
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  if profile not in ('full_process', 'schema_boundary'):
    raise ValueError('Unsupported startup evaluation profile')
  started = time.perf_counter()
  with isolated_environment() as owned_isolation:
    capabilities = capability()
    result = {
      'format_version': 2, 'profile': profile,
      'runtime': {'platform': platform.platform(), 'python': sys.version, 'executable': sys.executable},
      'isolation': {'params': 'tool-owned temporary directory',
                    'messaging': 'tool-owned unique fake prefix', 'writable_state': 'temporary runtime only',
                    'vehicle_connection': 'none', 'can_publisher': 'not constructed'},
      'timing_scope': 'local process replay elapsed time; not full-system or device scheduling proof',
      'source_schema_sha256': source_identity(), 'exception': None,
    }
    supported = capabilities[profile + '_supported']
    if not supported:
      result.update(status='unsupported', missing_capabilities=capabilities['missing'], boundary=None, timestamp_transform=None,
                  scope='Required full-process integration is not satisfied; no vehicle, CAN sender, Params, or process replay was run.')
    elif profile == 'full_process' and rlog is None:
      result.update(status='unsupported', missing_capabilities=['mazda_rlog'], boundary=None, timestamp_transform=None,
                  scope='A local Mazda rlog is required for the full-process integration profile.')
    else:
      try:
        checked = boundary() if boundary is not None else (actual_full_process(rlog, max_carstate_messages) if profile == 'full_process' else actual_boundary())
        result.update(status='completed_checks', missing_capabilities=[], boundary=checked,
                    timestamp_transform=checked.get('timestamp_transform', {'status': 'not_applicable', 'reason': 'schema-only boundary has no generated diagnostics'}),
                    scope=('Actual isolated controlsd process replay with inactive Mazda inputs; command objects are observed, but no CAN publisher, vehicle connection, or physical behavior is exercised.'
                           if profile == 'full_process' else 'Actual isolated carState schema subscriber boundary only; no full process or scheduling claim.'))
      except Exception as error:
        result.update(status='failed_execution', missing_capabilities=[], boundary=None, timestamp_transform=None,
                    exception=f'{type(error).__name__}: {error}', traceback=traceback.format_exc(),
                    scope='Isolated process-boundary attempt failed; full-process integration is not satisfied.')
    result['isolation']['owned_params_root_removed_after_run'] = True
    result['isolation']['owned_messaging_prefix'] = owned_isolation['messaging_prefix']
  result['elapsed_seconds'] = time.perf_counter() - started
  write_json(output / 'result.json', result)
  return result


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--profile', choices=('full_process', 'schema_boundary'), default='full_process')
  parser.add_argument('--rlog', type=Path, action='append', help='Local Mazda rlog; repeat for prior segments, with the startup segment last')
  parser.add_argument('--max-carstate-messages', type=int, default=100)
  args = parser.parse_args(argv)
  result = run(args.output, args.profile, args.rlog, args.max_carstate_messages)
  print(json.dumps({'status': result['status'], 'missing_capabilities': result['missing_capabilities'], 'exception': result['exception']}))
  return 0 if result['status'] == 'completed_checks' else 1


if __name__ == '__main__':
  raise SystemExit(main())
