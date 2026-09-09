"""Evaluate declared Mazda control transitions from serialized process observations.

This is an evidence reader, not a controller, CAN sender, or vehicle interface.
"""

import argparse
import json
from pathlib import Path
import time

from .provenance import ROOT, sha256, write_json
from .startup_eval import StartupUnsupported, actual_full_process, capability, isolated_environment, source_identity


HEALTH = ('missing', 'stale', 'invalid')
REQUIRED = ('active_operation', 'disengage_reengage', 'ti_bypass_reentry')


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
    for input_row in diagnostic.inputs:
      service, source_mono = str(input_row.service), int(input_row.logMonoTime)
      if input_row.seen and (service, source_mono) not in by_identity:
        findings.append(f'unknown diagnostic reference: {service}@{source_mono}')
      if not input_row.seen:
        health = 'missing'
      elif not input_row.valid or not input_row.checksPassed:
        health = 'invalid'
      # carState is controlsd's dedicated blocking socket.  Its diagnostic
      # producer intentionally has no SubMaster frequency estimate, so false
      # frequencyOk is not staleness when its own checks have passed.
      elif service != 'carState' and (not input_row.alive or not input_row.frequencyOk):
        health = 'stale'
      refs.append({'service': service, 'log_mono_time': source_mono})
    rows.append({'mono_time_ns': mono, 'active': bool(torque.active),
                 'ti_allowed': bool(actuator.tiAllowed) if actuator is not None else False,
                 'health': health, 'diagnostic_references': refs,
                 'state_before': str(diagnostic.integralBefore), 'state_after': str(diagnostic.integralAfter)})
  rows.sort(key=lambda row: row['mono_time_ns'])
  return rows, findings


def assess(rows, initial_findings=()):
  """Check transitions and explicit fault handling without inventing timing behavior."""
  findings, availability = list(initial_findings), {name: 0 for name in REQUIRED}
  availability.update({f'{name}_rejected_inactive': 0 for name in HEALTH})
  previous = None
  integrity_error = False
  awaiting_ti_reentry = False
  state_retention = {}
  for row in rows:
    mono = row['mono_time_ns']
    if previous is not None and mono <= previous['mono_time_ns']:
      findings.append(f'non-monotonic observation identity at {mono}')
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
      if previous['active'] and not row['active']:
        state_retention['disengage_state'] = previous['state_after']
      if previous['active'] and previous['ti_allowed'] and row['active'] and not row['ti_allowed']:
        awaiting_ti_reentry = True
        if previous['state_after'] != row['state_before']:
          findings.append(f'state changed while active TI bypass at {mono}')
        else:
          state_retention['active_to_ti_bypass'] = row['state_before']
      if awaiting_ti_reentry and row['active'] and row['ti_allowed']:
        availability['ti_bypass_reentry'] += 1
        awaiting_ti_reentry = False
      if not previous['active'] and row['active'] and 'disengage_state' in state_retention:
        availability['disengage_reengage'] += 1
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
    try:
      if rlog is not None:
        capabilities = capability()
        if not capabilities['full_process_supported']:
          raise StartupUnsupported(', '.join(capabilities['missing']))
        process = process_boundary(rlog, max_carstate_messages, all_segments, transition_harness)
        transition = {name: process[name] for name in ('status', 'findings', 'availability', 'state_retention', 'observations')}
        transition['process_boundary'] = process['startup_boundary']
      else:
        capabilities = {'full_process_supported': None, 'missing': []}
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
  observations = json.loads(args.observations.read_text(encoding='utf-8')) if args.observations else None
  result = run(args.output, observations=observations, rlog=args.rlog, max_carstate_messages=args.max_carstate_messages,
               all_segments=args.all_segments, transition_harness=args.schema_input_harness)
  print(json.dumps({'status': result['status'], 'findings': result['findings']}))
  return 0 if result['status'] == 'completed_checks' else 1


if __name__ == '__main__':
  raise SystemExit(main())
