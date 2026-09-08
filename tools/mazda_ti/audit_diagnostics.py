# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Audit explicit controller/apply identities in original, uncompressed rlogs.

This is an evidence-coverage check, not controller replay or a handling score.
Supply one route and continuous card lifetime, including adjacent segments.
"""

import argparse
from collections import Counter
from pathlib import Path

from .provenance import environment, finish_sources, sha256, source_snapshot, under, write_json

INPUTS = ('carState', 'frogpilotCarState', 'modelV2', 'liveParameters', 'liveTorqueParameters', 'liveDelay', 'carOutput', 'liveLocationKalman')
SERVICES = {*INPUTS, 'controlsState', 'carControl'}


def audit(events, start_ns, end_ns):
  """Index exact event identities; audit outputs in the half-open scoring window."""
  if not 0 < start_ns < end_ns:
    raise ValueError('Require 0 < start_ns < end_ns')
  identities, commands, controllers, outputs = {}, {}, {}, {}
  for event in events:
    service = event.which()
    if service not in SERVICES:
      continue
    mono = int(event.logMonoTime)
    key = (service, mono)
    if mono <= 0 or key in identities:
      raise ValueError(f'Zero or duplicate event identity: {service}/{mono}')
    identities[key] = bool(event.valid)
    if service == 'carControl':
      commands[mono] = int(event.carControl.controlsStateMonoTime)
    elif service == 'controlsState':
      state = event.controlsState.lateralControlState
      if state.which() == 'torqueState':
        torque = state.torqueState
        d = torque.mazdaDiagnostics
        controllers[mono] = {'active': bool(torque.active), 'version': int(d.version), 'inputs': [s.to_dict() for s in d.inputs]}
      else:
        controllers[mono] = {'active': False, 'version': 0, 'inputs': []}
    elif service == 'carOutput':
      outputs[mono] = event.carOutput.to_dict()
      outputs[mono].setdefault('mazdaDiagnostics', {})['version'] = int(event.carOutput.mazdaDiagnostics.version)

  rows, issues, health = [], [], Counter()
  previous = None
  seen_applies = set()

  def issue(mono, reason, **details):
    issues.append({'carOutput': mono, 'reason': reason, **details})

  for mono, output in sorted(outputs.items()):
    if mono >= end_ns:
      break
    if mono < start_ns:
      previous = (output['applySequence'], output) if output['mazdaDiagnostics']['version'] == 1 and output['applySequence'] > 0 else None
      continue
    count_before = len(issues)
    sequence = output['applySequence']
    command_mono = output['appliedCarControlMonoTime']
    applied_at = output['appliedAtMonoTime']
    row = {'carOutput': mono, 'applySequence': sequence, 'carControl': command_mono,
           'appliedAt': applied_at, 'controlsState': None, 'active': None, 'inputs': {}}
    rows.append(row)
    health['output_publications'] += 1
    health['invalid_output_publications'] += int(not identities['carOutput', mono])
    if output['mazdaDiagnostics']['version'] != 1:
      issue(mono, 'absent_or_unsupported_actuator_diagnostics', version=output['mazdaDiagnostics']['version'])
      row['complete'] = False
      previous = None
      continue  # Default zero fields in old logs are not failed applies or repeated commands.
    if not 0 < command_mono <= applied_at <= mono or sequence <= 0:
      issue(mono, 'invalid_apply_identity_or_order')
      row['complete'] = False
      previous = None
      continue
    health['applies_with_failed_checks_publications'] += int(not output['appliedCarControlChecksPassed'])
    if previous:
      old_sequence, old_output = previous
      if sequence < old_sequence:
        issue(mono, 'apply_sequence_reset', previous=old_sequence)
      elif sequence == old_sequence:
        health['repeated_apply_publications'] += 1
        if output != old_output:
          issue(mono, 'repeated_apply_changed_payload')
      else:
        if applied_at <= old_output['appliedAtMonoTime']:
          issue(mono, 'nonincreasing_apply_time')
        if sequence > old_sequence + 1:
          issue(mono, 'missing_apply_publications', missing=sequence - old_sequence - 1)
    previous = sequence, output
    seen_applies.add(sequence)
    controller_mono = commands.get(command_mono)
    if controller_mono is None:
      issue(mono, 'missing_carControl', identity=command_mono)
    else:
      row['controlsState'] = controller_mono
      if not 0 < controller_mono <= command_mono:
        issue(mono, 'invalid_controller_identity_or_order')
      controller = controllers.get(controller_mono)
      if controller is None:
        issue(mono, 'missing_controlsState', identity=controller_mono)
      elif controller['version'] != 1:
        issue(mono, 'absent_or_unsupported_controller_diagnostics', version=controller['version'])
      else:
        row['active'] = controller['active']
        health['active_controller_publications'] += int(controller['active'])
        snapshots = controller['inputs']
        if sorted(s['service'] for s in snapshots) != sorted(INPUTS):
          issue(mono, 'invalid_input_service_set')
        for snapshot in snapshots:
          service, identity = snapshot['service'], snapshot['logMonoTime']
          row['inputs'][service] = identity
          health['input_snapshots'] += 1
          health['input_snapshots_failed_checks'] += int(not snapshot['checksPassed'])
          if not snapshot['seen'] or identity <= 0:
            issue(mono, 'unseen_input', service=service)
          elif identity > controller_mono:
            issue(mono, 'future_input_identity', service=service, identity=identity)
          elif (service, identity) not in identities:
            issue(mono, 'missing_input_event', service=service, identity=identity)
          elif identities[service, identity] != snapshot['valid']:
            issue(mono, 'input_validity_mismatch', service=service, identity=identity)
    row['complete'] = len(issues) == count_before

  complete = bool(rows) and not issues
  return {
    'format_version': 1, 'window_ns': [start_ns, end_ns],
    'identity_coverage_complete': complete,
    'active_identity_coverage_complete': complete and health['active_controller_publications'] > 0,
    'summary': {**dict(sorted(health.items())), 'unique_applies': len(seen_applies),
                'complete_publications': sum(r['complete'] for r in rows),
                'issue_counts': dict(sorted(Counter(i['reason'] for i in issues).items()))},
    'rows': rows, 'issues': issues,
    'limitations': [
      'Exact identity coverage only; no command reconstruction, controller replay, or physical handling qualification.',
      'Health failures are reported separately from missing or inconsistent identities.',
      'Scope is supplied carOutput publications; unrecorded leading/trailing time and missing controller publications not applied by card are not certified.',
      'Load adjacent segments; missing references are not replaced with nearest timestamps.',
      'One original route and continuous card lifetime only; process-replay timestamp rewriting is unqualified.',
    ],
  }


def read_events(paths):
  from cereal import log

  for path in paths:
    # Read one segment at a time; retain only compact identities and snapshots.
    yield from log.Event.read_multiple_bytes(path.read_bytes())


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--rlogs', nargs='+', required=True, help='Uncompressed rlog paths relative to data root')
  parser.add_argument('--start-ns', type=int, required=True)
  parser.add_argument('--end-ns', type=int, required=True)
  parser.add_argument('--output', type=Path, required=True, help='New JSON file; existing evidence is preserved')
  args = parser.parse_args()
  if args.output.exists():
    parser.error('Output already exists; select a new evidence path')
  if not args.rlogs or len(set(args.rlogs)) != len(args.rlogs):
    parser.error('Provide distinct rlogs from one route')
  paths = [under(args.data_root, name) for name in args.rlogs]
  source_before, runtime_before = source_snapshot(), environment()
  hashes = {name: sha256(path) for name, path in zip(args.rlogs, paths, strict=True)}
  result = audit(read_events(paths), args.start_ns, args.end_ns)
  if any(sha256(path) != hashes[name] for name, path in zip(args.rlogs, paths, strict=True)) or environment() != runtime_before:
    raise ValueError('Inputs or runtime changed during audit')
  result.update(input_sha256=hashes, repository_sources=finish_sources(source_before), environment=runtime_before)
  write_json(args.output, result)
  print(f'Identity coverage complete: {result["identity_coverage_complete"]}; active coverage: {result["active_identity_coverage_complete"]}')
  return 0 if result['active_identity_coverage_complete'] else 1


if __name__ == '__main__':
  raise SystemExit(main())
