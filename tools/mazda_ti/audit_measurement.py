# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Compare recorded angle-model acceleration to exactly consumed localizer yaw.

Compact, descriptive evidence only: no fitted lag, controller replay or physical
ground truth. Positive acceleration is right. Inactive measurements are excluded.
"""

import argparse
from collections import Counter
import math
from pathlib import Path
from statistics import median

from .audit_diagnostics import read_events
from .provenance import environment, finish_sources, sha256, source_snapshot, under, write_json


def distribution(values):
  values = list(values)
  return {'min': min(values), 'median': median(values), 'max': max(values)} if values else None


def compact_input(event):
  if event.which() == 'carState':
    return {'speed': float(event.carState.vEgo), 'can_valid': bool(event.carState.canValid)}
  loc = event.liveLocationKalman
  yaw = loc.angularVelocityCalibrated
  if len(yaw.value) != 3 or len(yaw.std) != 3:
    return {'malformed': True}
  return {'yaw': float(yaw.value[2]), 'yaw_std': float(yaw.std[2]),
          'yaw_valid': bool(yaw.valid), 'sensors_ok': bool(loc.sensorsOK),
          'inputs_ok': bool(loc.inputsOK), 'posenet_ok': bool(loc.posenetOK)}


def audit(events, start_ns, end_ns, sample_count=12):
  if not 0 < start_ns < end_ns or not 0 <= sample_count <= 100:
    raise ValueError('Require 0 < start < end and sample_count in [0, 100]')
  inputs, controllers = {}, {}
  for event in events:
    service, mono = event.which(), int(event.logMonoTime)
    if service in ('carState', 'liveLocationKalman'):
      key = (service, mono)
      if mono <= 0 or key in inputs:
        raise ValueError(f'Zero or duplicate input identity: {key}')
      inputs[key] = (bool(event.valid), compact_input(event))
    elif service == 'controlsState' and start_ns <= mono < end_ns:
      if mono in controllers:
        raise ValueError(f'Duplicate controller identity: {mono}')
      state = event.controlsState.lateralControlState
      controllers[mono] = (bool(event.valid), bool(state.torqueState.active),
                           state.torqueState.mazdaDiagnostics.to_dict()) if state.which() == 'torqueState' else (bool(event.valid), False, {})

  rows, issues, health = [], Counter(), Counter()
  issue_examples = []

  def issue(mono, reason):
    issues[reason] += 1
    if len(issue_examples) < 12:
      issue_examples.append({'mono': mono, 'reason': reason})

  for mono, (valid, active, d) in sorted(controllers.items()):
    if not active:
      health['inactive_or_other_controller'] += 1
      continue
    health['active_publications'] += 1
    if d.get('version') != 1:
      issue(mono, 'absent_or_unsupported_diagnostics')
      continue
    joined, snapshots = {}, {}
    for service in ('carState', 'liveLocationKalman'):
      matches = [s for s in d.get('inputs', []) if s['service'] == service]
      if len(matches) != 1:
        issue(mono, f'{service}:snapshot_count')
        continue
      s = matches[0]
      identity = s['logMonoTime']
      if not s['seen'] or not 0 < identity <= mono:
        issue(mono, f'{service}:unseen_or_invalid_identity')
        continue
      value = inputs.get((service, identity))
      if value is None:
        issue(mono, f'{service}:missing_exact_input')
        continue
      if value[0] != s['valid']:
        issue(mono, f'{service}:validity_mismatch')
        continue
      joined[service], snapshots[service] = value[1], s
    if len(joined) != 2:
      continue
    cs, loc = joined['carState'], joined['liveLocationKalman']
    if loc.get('malformed'):
      issue(mono, 'malformed_yaw_vector')
      continue
    numeric = [cs['speed'], loc['yaw'], loc['yaw_std'], d['measurement'], d['rawRequest']]
    if not all(math.isfinite(v) for v in numeric) or loc['yaw_std'] < 0 or cs['speed'] < 0:
      issue(mono, 'invalid_numeric_measurement')
      continue
    flags = {'controller_valid': valid, 'can_valid': cs['can_valid'], **{k: loc[k] for k in
             ('yaw_valid', 'sensors_ok', 'inputs_ok', 'posenet_ok')},
             **{f'{k}_checks': s['checksPassed'] for k, s in snapshots.items()},
             **{f'{k}_valid': s['valid'] for k, s in snapshots.items()}}
    for key, flag in flags.items():
      if not flag:
        health[f'false:{key}'] += 1
    healthy = all(flags.values())
    yaw_accel = cs['speed'] * loc['yaw']
    row = {'mono': mono, 'healthy': healthy, 'car_state_mono': snapshots['carState']['logMonoTime'],
           'llk_mono': snapshots['liveLocationKalman']['logMonoTime'],
           'speed_mps': cs['speed'], 'angle_accel_mps2': d['measurement'], 'yaw_accel_mps2': yaw_accel,
           'angle_minus_yaw_mps2': d['measurement'] - yaw_accel,
           'yaw_only_std_mps2': cs['speed'] * loc['yaw_std'],
           'raw_request_mps2': d['rawRequest'], 'yaw_minus_request_mps2': yaw_accel - d['rawRequest'],
           'angle_minus_request_mps2': d['measurement'] - d['rawRequest'],
           'cs_publication_age_ms': (mono - snapshots['carState']['logMonoTime']) / 1e6,
           'llk_publication_age_ms': (mono - snapshots['liveLocationKalman']['logMonoTime']) / 1e6,
           'cs_minus_llk_publication_ms': (snapshots['carState']['logMonoTime'] - snapshots['liveLocationKalman']['logMonoTime']) / 1e6}
    if not all(math.isfinite(v) for k, v in row.items() if k.endswith(('_mps2', '_mps', '_ms'))):
      issue(mono, 'nonfinite_derived_measurement')
      continue
    rows.append(row)
  selected = [r for r in rows if r['healthy']]
  metric_keys = [k for k in rows[0] if k.endswith(('_mps2', '_mps', '_ms'))] if rows else []
  count = min(sample_count, len(rows))
  sample_indices = sorted({round(i * (len(rows) - 1) / max(count - 1, 1)) for i in range(count)})
  times = sorted(controllers)
  return {'format_version': 1, 'window_ns': [start_ns, end_ns],
          'identity_numeric_coverage_complete': bool(rows) and not issues,
          'all_active_measurements_healthy': bool(selected) and len(selected) == health['active_publications'] and not issues,
          'controller_publications': len(controllers), 'joined_active_rows': len(rows), 'healthy_rows': len(selected),
          'unique_consumed_llk': len({r['llk_mono'] for r in selected}),
          'first_last_controller_ns': [times[0], times[-1]] if times else None,
          'max_controller_publication_gap_ms': max(((b-a)/1e6 for a, b in zip(times, times[1:])), default=None),
          'health_counts': dict(sorted(health.items())), 'issue_counts': dict(sorted(issues.items())),
          'issue_examples': issue_examples,
          'healthy_metrics': {k: distribution(r[k] for r in selected) for k in metric_keys},
          'samples': [rows[i] for i in sample_indices],
          'largest_healthy_discrepancy': max(selected, key=lambda r: abs(r['angle_minus_yaw_mps2']), default=None),
          'limitations': [
            'Exact consumed event identities, not nearest timestamps. Event age is not sensor exposure age.',
            'Yaw times carState speed is a rotation-based approximation, not full lateral acceleration during sideslip transients.',
            'Yaw standard deviation times speed omits speed, timing, mounting and model uncertainty; not an error bound.',
            'Samples share localizer state and are temporally correlated; no sqrt(N) reduction of uncertainty.',
            'No optimized lag, roll subtraction, candidate motion, grip ceiling or handling qualification.',
            'Statistics weight healthy controller publications, including repeated consumed localizer messages.',
            'Coverage refers to supplied publications only; gaps and window edges remain explicit.',
            'One original route only; process-replay timestamp rewriting is unqualified.',
          ]}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--rlogs', nargs='+', required=True)
  parser.add_argument('--start-ns', type=int, required=True)
  parser.add_argument('--end-ns', type=int, required=True)
  parser.add_argument('--sample-count', type=int, default=12)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if args.output.exists() or len(set(args.rlogs)) != len(args.rlogs):
    parser.error('Use a new output and distinct raw paths from one route')
  paths = [under(args.data_root, name) for name in args.rlogs]
  sources, runtime = source_snapshot(), environment()
  hashes = {name: sha256(path) for name, path in zip(args.rlogs, paths, strict=True)}
  result = audit(read_events(paths), args.start_ns, args.end_ns, args.sample_count)
  if environment() != runtime or any(sha256(path) != hashes[name] for name, path in zip(args.rlogs, paths, strict=True)):
    raise ValueError('Raw inputs or runtime changed during audit')
  result.update(input_sha256=hashes, repository_sources=finish_sources(sources), environment=runtime)
  write_json(args.output, result)
  print(f'Healthy active coverage: {result["all_active_measurements_healthy"]}; rows: {result["healthy_rows"]}')
  return 0 if result['all_active_measurements_healthy'] else 1


if __name__ == '__main__':
  raise SystemExit(main())
