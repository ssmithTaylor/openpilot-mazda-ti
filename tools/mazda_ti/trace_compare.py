"""Versioned optional trace capabilities and a portable phase comparison report."""

import argparse
import json
import math
from pathlib import Path
import re

from .provenance import read_json, sha256, write_json


FORMAT_VERSION = 1
METRICS = ('request_counts', 'retained_reference_counts', 'integral_counts', 'compensation_counts',
           'clipped', 'constant_command', 'ti_command_counts', 'stock_command_counts', 'carryover_counts')


def _bundle(path, evidence_root, data_root):
  path = Path(path)
  full_root = path.parent if (path.parent / 'resolved-request.json').is_file() else None
  if (path / 'resolved-request.json').is_file():
    raise ValueError('Full evaluation bundles require an explicit retained arm directory')
  if full_root is not None and path.name not in ('baseline', 'candidate'):
    raise ValueError('Verified evaluation bundles require baseline or candidate retained arms')
  result_path = (full_root or path) / 'result.json'
  result, metadata = read_json(result_path), read_json(path / 'trace.json')
  if result.get('format_version') != 1 or (full_root is None and metadata.get('format_version') != FORMAT_VERSION):
    raise ValueError('Unsupported retained trace bundle version')
  rows = [json.loads(line) for line in (path / 'trace.jsonl').read_text(encoding='utf-8').splitlines() if line]
  if (not rows or any(type(row.get('mono')) not in (int, float) or not math.isfinite(row['mono']) for row in rows) or
      any(type(value) is float and not math.isfinite(value) for row in rows for value in row.values())):
    raise ValueError('Trace requires finite monotonic rows')
  if any(later['mono'] <= earlier['mono'] for earlier, later in zip(rows, rows[1:], strict=False)):
    raise ValueError('Trace timestamps must increase')
  provenance = metadata.get('provenance')
  valid_provenance = isinstance(provenance, dict) and all(isinstance(provenance.get(name), str) and re.fullmatch('[a-f0-9]{64}', provenance[name])
                                                          for name in ('source_sha256', 'input_sha256'))
  try:
    reference = path.resolve().relative_to(evidence_root.resolve()).as_posix()
  except ValueError:
    raise ValueError('Evidence bundle escapes the declared evidence root')
  artifacts = {'result.json': sha256(result_path), 'trace.json': sha256(path / 'trace.json'), 'trace.jsonl': sha256(path / 'trace.jsonl')}
  level = 'content_bound_unqualified'
  if data_root is not None and full_root is not None:
    from .evaluate import verify_bundle
    verify_bundle(full_root, data_root)
    provenance = {'source_identities': result.get('source_identities'), 'input_sha256': result.get('input_sha256')}
    if not isinstance(provenance['source_identities'], dict) or not isinstance(provenance['input_sha256'], dict):
      raise ValueError('Verified bundle lacks source or input identities')
    level = 'verified_full_bundle'
  return {'path': reference, 'result': result, 'metadata': metadata, 'rows': rows,
          'provenance': provenance if valid_provenance else None, 'artifacts': artifacts, 'identity_validation': level}


def _phases(metadata, rows):
  declared = metadata.get('phase_windows', [])
  names = ('entry', 'sustained', 'unwind', 'recovery')
  if declared:
    if (not isinstance(declared, list) or [row.get('name') for row in declared] != list(names) or
        any(set(row) != {'name', 'start', 'end', 'source', 'uncertainty'} or type(row['start']) not in (int, float) or
            type(row['end']) not in (int, float) or row['start'] >= row['end'] or not isinstance(row['source'], str) or
            not isinstance(row['uncertainty'], str) for row in declared) or
        any(later['start'] < earlier['end'] for earlier, later in zip(declared, declared[1:], strict=False))):
      raise ValueError('Phase windows must declare entry, sustained, unwind, and recovery')
    return [dict(row) for row in declared]
  start, end = rows[0]['mono'], rows[-1]['mono']
  interval = (end - start) / 4
  return [{'name': name, 'start': start + index * interval, 'end': end if index == 3 else start + (index + 1) * interval,
           'source': 'documented_chronological_rule', 'uncertainty': 'inferred from trace duration; not a rider annotation'}
          for index, name in enumerate(names)]


def _values(bundle, metric, phase=None):
  active = [row for row in bundle['rows'] if row.get('active') is True and
            (phase is None or phase['start'] <= row['mono'] <= phase['end'])]
  if not active:
    return 'inactive', []
  values = [row[metric] for row in active if metric in row]
  if not values:
    return 'unavailable', []
  if len(values) != len(active):
    return 'unavailable', []
  if any(type(value) not in (int, float, bool) for value in values):
    return 'unavailable', []
  return 'supported', values


def _effect(left, right, metric, phase=None):
  left_state, left_values = _values(left, metric, phase)
  right_state, right_values = _values(right, metric, phase)
  if 'inactive' in (left_state, right_state):
    return {'availability': 'inactive', 'reason': 'At least one arm has no active trace fields'}
  if left_state != 'supported' or right_state != 'supported' or len(left_values) != len(right_values):
    return {'availability': 'unavailable', 'reason': 'Metric is absent, partial, or not comparable across retained arms'}
  left_rows = [row for row in left['rows'] if row.get('active') is True and (phase is None or phase['start'] <= row['mono'] <= phase['end'])]
  right_rows = [row for row in right['rows'] if row.get('active') is True and (phase is None or phase['start'] <= row['mono'] <= phase['end'])]
  fields = ['mono'] + left['metadata'].get('capabilities', {}).get('sample_identity_fields', [])
  if (left['metadata'].get('capabilities', {}).get('sample_identity_fields', []) != right['metadata'].get('capabilities', {}).get('sample_identity_fields', []) or
      any(any(field not in row for field in fields) for row in left_rows + right_rows) or
      [tuple(row[field] for field in fields) for row in left_rows] != [tuple(row[field] for field in fields) for row in right_rows]):
    return {'availability': 'unavailable', 'reason': 'Retained sample identities do not align'}
  deltas = [float(a) - float(b) for a, b in zip(left_values, right_values, strict=True)]
  return {'availability': 'supported', 'units': 'boolean' if isinstance(left_values[0], bool) else 'TI counts',
          'minimum_delta': min(deltas), 'maximum_delta': max(deltas), 'worst_absolute_delta': max(map(abs, deltas)),
          'constant_exposure_frames': sum(bool(value) for value in left_values) if metric == 'constant_command' else None}


def compare_trace_bundles(reference_path, candidate_path, current_control_path, output, *, data_root=None, evidence_root=None):
  """Compare retained, versioned traces; no metric is synthesized from absent fields."""
  evidence_root = Path(evidence_root) if evidence_root is not None else Path(reference_path).resolve().parent
  reference, candidate, control = (_bundle(path, evidence_root, data_root) for path in (reference_path, candidate_path, current_control_path))
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  try:
    phases = _phases(candidate['metadata'], candidate['rows'])
  except ValueError as error:
    result = {'format_version': FORMAT_VERSION, 'status': 'failed_check', 'provenance': {}, 'phases': [],
              'findings': {'invariants': [{'arm': 'candidate', 'finding': 'Invalid phase declaration: ' + str(error)}],
                           'command_effects': [], 'physical_observations': []}}
    write_json(output / 'result.json', result)
    (output / 'report.md').write_text('# Mazda trace comparison\n\nStatus: **failed_check**. Invalid phase declaration.\n', encoding='utf-8')
    return result
  metrics = {name: _effect(candidate, reference, name) for name in METRICS}
  current = {name: _effect(candidate, control, name) for name in METRICS}
  phase_effects = {phase['name']: {
    'candidate_minus_reference': {name: _effect(candidate, reference, name, phase) for name in METRICS},
    'candidate_minus_current_control': {name: _effect(candidate, control, name, phase) for name in METRICS},
  } for phase in phases}
  invariants = [{'arm': name, 'status': bundle['result'].get('status'), 'finding': 'Retained arm did not complete declared checks'}
                for name, bundle in (('reference', reference), ('candidate', candidate), ('current_control', control))
                if bundle['result'].get('status') != 'completed_checks']
  arms = {'reference': reference, 'candidate': candidate, 'current_control': control}
  for name, bundle in arms.items():
    if bundle['provenance'] is None and bundle['identity_validation'] != 'verified_full_bundle':
      invariants.append({'arm': name, 'finding': 'Retained arm lacks valid source/input provenance'})
  if all(bundle['provenance'] is not None for bundle in arms.values()):
    input_hashes = {json.dumps(bundle['provenance']['input_sha256'], sort_keys=True) for bundle in arms.values()}
    if len(input_hashes) != 1:
      invariants.append({'arm': 'current_control', 'finding': 'Retained arms have mismatched input provenance'})
  status = 'failed_check' if invariants else 'completed_checks'
  result = {'format_version': FORMAT_VERSION, 'status': status,
            'identity_validation': ('verified_full_bundle' if all(bundle['identity_validation'] == 'verified_full_bundle' for bundle in arms.values())
                                    else 'content_bound_unqualified'),
            'provenance': {name: {'bundle': bundle['path'], 'artifact_sha256': bundle['artifacts'],
                                  'identity_validation': bundle['identity_validation'], **(bundle['provenance'] or {})}
                           for name, bundle in arms.items()},
            'phases': phases, 'metrics': metrics,
            'phase_effects': phase_effects,
            'command_effects': {'candidate_minus_reference': {name: metrics[name] for name in ('ti_command_counts', 'stock_command_counts')},
                                'candidate_minus_current_control': {name: current[name] for name in ('ti_command_counts', 'stock_command_counts')}},
            'findings': {'invariants': invariants,
                         'command_effects': ['Deltas describe recorded-motion command effects, not handling improvement.'],
                         'physical_observations': candidate['metadata'].get('physical_observations', [])},
            'scope': 'Software invariants, recorded-motion command effects, and physical observations are separate. Earlier release alone is not improvement.'}
  write_json(output / 'result.json', result)
  lines = ['# Mazda trace comparison', '', f"Status: **{status}**.", '', result['scope'], '', '## Phases', '']
  lines += [f"- {item['name']}: {item['start']} to {item['end']} ({item['source']}; {item['uncertainty']})" for item in phases]
  lines += ['', '## Command effects', '']
  for name, effect in result['command_effects']['candidate_minus_reference'].items():
    lines.append(f"- {name}: {effect['availability']}; worst candidate-minus-reference delta {effect.get('worst_absolute_delta', 'unavailable')}.")
  lines += ['', '## Unfavorable, missing, and inactive evidence', '']
  lines += [f"- {name}: {effect['availability']} ({effect.get('reason', 'retained comparison')})" for name, effect in metrics.items() if effect['availability'] != 'supported']
  lines += ['', '## Physical observations', ''] + [f"- {item.get('text', 'unavailable')} ({item.get('provenance', 'unavailable')})" for item in result['findings']['physical_observations']]
  (output / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8', newline='\n')
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--reference', type=Path, required=True)
  parser.add_argument('--candidate', type=Path, required=True)
  parser.add_argument('--current-control', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--data-root', type=Path)
  parser.add_argument('--evidence-root', type=Path)
  args = parser.parse_args()
  result = compare_trace_bundles(args.reference, args.candidate, args.current_control, args.output,
                                 data_root=args.data_root, evidence_root=args.evidence_root)
  print(f"{result['status']}: {args.output / 'report.md'}")
  return 0 if result['status'] == 'completed_checks' else 1


if __name__ == '__main__':
  raise SystemExit(main())
