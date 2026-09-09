"""Evaluate declared Mazda control transitions from serialized process observations.

This is an evidence reader, not a controller, CAN sender, or vehicle interface.
"""

import argparse
import hashlib
import json
from pathlib import Path
import time

from .provenance import ROOT, sha256, write_json
from .startup_eval import StartupUnsupported, actual_full_process, capability, isolated_environment, source_identity


HEALTH = ('missing', 'stale', 'invalid')
REQUIRED = ('active_operation', 'disengage_reengage', 'ti_bypass_reentry')


def opaque_state_id(value):
  """Return a stable, non-semantic identity for a serialized controller state."""
  return 'sha256:' + hashlib.sha256(repr(float(value)).encode('ascii')).hexdigest()


def normalize_observations(events, source_events=()):
  """Derive observations from actual serialized controlsState/carControl/carOutput links."""
  by_identity = {(event.which(), int(event.logMonoTime)): event for event in [*source_events, *events]}
  commands = {int(event.logMonoTime): event.carControl for event in events if event.which() == 'carControl'}
  output_by_control = {}
  for event in events:
    if event.which() == 'carOutput' and event.carOutput.mazdaDiagnostics.version:
      output_by_control[int(event.carOutput.appliedCarControlMonoTime)] = event.carOutput.mazdaDiagnostics
  rows, findings = [], []
  for event in events:
    if event.which() != 'controlsState':
      continue
    torque = event.controlsState.lateralControlState.torqueState
    diagnostic = torque.mazdaDiagnostics
    if not diagnostic.version:
      continue
    mono = int(event.logMonoTime)
    command = next((item for item in commands.values() if int(item.controlsStateMonoTime) == mono), None)
    actuator = output_by_control.get(next((key for key, value in commands.items() if value is command), -1))
    health, refs = 'valid', []
    ti_available, ti_source = False, 'unavailable'
    for input_row in diagnostic.inputs:
      service, source_mono = str(input_row.service), int(input_row.logMonoTime)
      if input_row.seen and (service, source_mono) not in by_identity:
        findings.append(f'unknown diagnostic reference: {service}@{source_mono}')
      if not input_row.seen:
        health = 'missing'
      elif not input_row.valid:
        health = 'invalid'
      # carState is controlsd's dedicated blocking socket.  Its diagnostic
      # producer intentionally has no SubMaster frequency estimate, so false
      # frequencyOk is not staleness when its own checks have passed.
      elif service != 'carState' and (not input_row.alive or not input_row.frequencyOk):
        health = 'stale'
      elif not input_row.checksPassed:
        health = 'invalid'
      if service == 'frogpilotCarState' and input_row.seen:
        source = by_identity.get((service, source_mono))
        if source is not None and hasattr(source, 'frogpilotCarState'):
          ti_available = bool(source.frogpilotCarState.tiActive)
          ti_source = 'frogpilotCarState_input'
      refs.append({'service': service, 'log_mono_time': source_mono})
    if actuator is not None:
      ti_available, ti_source = bool(actuator.tiAllowed), 'carOutput_observation'
    row = {'mono_time_ns': mono, 'active': bool(torque.active),
                 'ti_allowed': ti_available, 'ti_availability_source': ti_source,
                 'health': health, 'diagnostic_references': refs,
                 'state_before': str(diagnostic.integralBefore), 'state_after': str(diagnostic.integralAfter),
                 'state_before_id': opaque_state_id(diagnostic.integralBefore),
                 'state_id': opaque_state_id(diagnostic.integralAfter)}
    rows.append(row)
  rows.sort(key=lambda row: row['mono_time_ns'])
  return rows, findings


def assess(rows, initial_findings=()):
  """Check transitions and explicit fault handling without inventing timing behavior."""
  findings, availability = list(initial_findings), {name: 0 for name in REQUIRED}
  availability.update({f'{name}_rejected_inactive': 0 for name in HEALTH})
  previous = None
  integrity_error = False
  awaiting_ti_reentry = False
  last_active_ti = None
  pending_disengage = None
  state_retention = {}
  for row in rows:
    mono = row['mono_time_ns']
    if previous is not None and mono <= previous['mono_time_ns']:
      findings.append(f'non-monotonic observation identity at {mono}')
      integrity_error = True
      continue
    if not isinstance(row.get('state_before_id'), str) or not isinstance(row.get('state_id'), str):
      findings.append(f'missing opaque state_id at {mono}')
      integrity_error = True
      continue
    if row['health'] in HEALTH:
      if row['active']:
        findings.append(f"{row['health']} message remained active at {mono}")
      else:
        availability[f"{row['health']}_rejected_inactive"] += 1
    if previous is not None:
      if not previous['active'] and row['active']:
        availability['active_operation'] += 1
        if pending_disengage is not None:
          if pending_disengage['state_after'] != row['state_before']:
            findings.append(f'state changed across disengage/reengage at {mono}')
          elif pending_disengage['state_id'] != row['state_before_id']:
            findings.append(f'opaque state_id changed across disengage/reengage at {mono}')
          else:
            availability['disengage_reengage'] += 1
            state_retention['reengage_state_before'] = row['state_before']
            state_retention['reengage_state_before_id'] = row['state_before_id']
          pending_disengage = None
      if previous['active'] and not row['active']:
        state_retention['disengage_state'] = previous['state_after']
        state_retention['disengage_state_id'] = previous['state_id']
        pending_disengage = previous
      if previous['active'] and previous['ti_allowed'] and row['active'] and not row['ti_allowed']:
        awaiting_ti_reentry = True
        if previous['state_after'] != row['state_before']:
          findings.append(f'state changed while active TI bypass at {mono}')
        elif previous['state_id'] != row['state_before_id']:
          findings.append(f'opaque state_id changed while active TI bypass at {mono}')
        else:
          state_retention['active_to_ti_bypass'] = row['state_before']
          state_retention['active_to_ti_bypass_id'] = row['state_before_id']
      elif not row['ti_allowed'] and last_active_ti is not None and not awaiting_ti_reentry:
        # The real plant controller makes the safe choice to deactivate while
        # TI is unavailable. Keep that as a visible transition rather than
        # requiring an unsafe active bypass solely to satisfy the fixture.
        awaiting_ti_reentry = True
        state_retention['ti_bypass_disengage_state'] = last_active_ti['state_after']
      if awaiting_ti_reentry and row['active'] and row['ti_allowed']:
        if last_active_ti['state_after'] != row['state_before']:
          findings.append(f'state changed across TI-loss/reentry at {mono}')
        elif last_active_ti['state_id'] != row['state_before_id']:
          findings.append(f'opaque state_id changed across TI-loss/reentry at {mono}')
        else:
          availability['ti_bypass_reentry'] += 1
          state_retention['ti_reentry_state'] = row['state_before']
          state_retention['ti_reentry_state_before_id'] = row['state_before_id']
        awaiting_ti_reentry = False
      if row['active'] and row['ti_allowed']:
        last_active_ti = row
    previous = row
  if not integrity_error:
    for name in REQUIRED:
      if not availability[name]:
        findings.append(f'missing required transition: {name}')
    for name in HEALTH:
      if not availability[f'{name}_rejected_inactive']:
        findings.append(f'missing required fault handling: {name}')
  return {'status': 'failed_check' if findings else 'completed_checks', 'findings': findings,
          'availability': availability, 'state_retention': state_retention, 'observations': rows}


def actual_process_transition(rlog, max_carstate_messages=100, all_segments=False, transition_harness=False):
  """Observe real replay output without changing controlsd or adding a sender."""
  if transition_harness and not all_segments:
    raise ValueError('--schema-input-harness requires --all-segments')
  observed = {}

  def observer(inputs, outputs):
    rows, findings = normalize_observations(outputs, source_events=inputs)
    observed.update(assess(rows, findings))
    return observed

  boundary = actual_full_process(rlog, max_carstate_messages, require_inactive=False, observer=observer,
                                 all_segments=all_segments, transition_harness=transition_harness)
  if not observed:
    raise RuntimeError('controlsd replay did not expose transition observations')
  return {'startup_boundary': boundary, **observed}


def run(output, observations=None, events=None, rlog=None, max_carstate_messages=100, process_boundary=actual_process_transition,
        capability=capability, all_segments=False, transition_harness=False):
  """Write a common-status transition result in fresh, isolated local state."""
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  started = time.perf_counter()
  with isolated_environment() as owned:
    profile = 'full_process_transition' if rlog is not None else 'serialized_transition_fixture'
    capabilities = {'full_process_supported': None, 'missing': []}
    runtime_source, input_sha256 = None, {}
    try:
      if transition_harness and (rlog is None or not all_segments):
        raise ValueError('--schema-input-harness requires --rlog and --all-segments')
      if rlog is not None:
        capabilities = capability()
        if not capabilities['full_process_supported']:
          raise StartupUnsupported(', '.join(capabilities['missing']))
        process = process_boundary(rlog, max_carstate_messages, all_segments, transition_harness)
        transition = {name: process[name] for name in ('status', 'findings', 'availability', 'state_retention', 'observations')}
        transition['process_boundary'] = process['startup_boundary']
        runtime_source = process['startup_boundary']['runtime_source']
        input_sha256 = {row['label']: row['sha256'] for row in process['startup_boundary']['input']['rlogs']}
      else:
        if observations is None:
          observations, normalization_findings = normalize_observations(events or [])
        else:
          normalization_findings = []
        transition = assess(observations, normalization_findings)
      exception = None
    except StartupUnsupported as error:
      transition, exception = None, f'{type(error).__name__}: {error}'
      status = 'unsupported'
    except Exception as error:
      transition, exception = None, f'{type(error).__name__}: {error}'
      status = 'failed_execution'
    else:
      status = transition['status']
    result = {
      'format_version': 2, 'profile': profile, 'status': status,
      'findings': transition['findings'] if transition else [], 'transition': transition,
      'missing_capabilities': capabilities['missing'],
      'source_schema_sha256': source_identity() | {'tools/mazda_ti/transition_eval.py': sha256(ROOT / 'tools/mazda_ti/transition_eval.py')},
      'runtime_source': runtime_source, 'input_sha256': input_sha256,
      'isolation': {'params': 'tool-owned temporary directory', 'messaging': 'tool-owned unique fake prefix',
                    'writable_state': 'temporary runtime only', 'vehicle_connection': 'none',
                    'can_publisher': 'not constructed', 'owned_messaging_prefix': owned['messaging_prefix'],
                    'owned_params_root_removed_after_run': True},
      'timing_scope': 'transition result excludes elapsed workload timing; device timing requires a separate device profile',
      'scope': ('Actual isolated controlsd process replay is required for this profile; its retained outputs qualify declared software transitions only.'
                if rlog is not None else 'Serialized process observations qualify declared software transitions only; no vehicle, CAN sender, or device timing is exercised.'),
      'exception': exception,
    }
  result['elapsed_seconds'] = time.perf_counter() - started
  write_json(output / 'result.json', result)
  return result


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--observations', type=Path, help='JSON list produced from retained serialized evidence')
  parser.add_argument('--rlog', type=Path, action='append', help='Local Mazda rlog; repeat for earlier segments')
  parser.add_argument('--max-carstate-messages', type=int, default=100)
  parser.add_argument('--all-segments', action='store_true', help='Replay the supplied adjacent segments in order')
  parser.add_argument('--schema-input-harness', action='store_true', help='Inject declared missing controlsd input schemas')
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args(argv)
  if (args.observations is None) == (not args.rlog):
    parser.error('supply exactly one of --observations or --rlog')
  if args.schema_input_harness and (not args.rlog or not args.all_segments):
    parser.error('--schema-input-harness requires --rlog and --all-segments')
  observations = json.loads(args.observations.read_text(encoding='utf-8')) if args.observations else None
  result = run(args.output, observations=observations, rlog=args.rlog, max_carstate_messages=args.max_carstate_messages,
               all_segments=args.all_segments, transition_harness=args.schema_input_harness)
  print(json.dumps({'status': result['status'], 'findings': result['findings']}))
  return 0 if result['status'] == 'completed_checks' else 1


if __name__ == '__main__':
  raise SystemExit(main())
