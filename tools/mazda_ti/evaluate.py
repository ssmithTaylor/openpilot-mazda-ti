# Standalone CLI imports precede the openpilot namespace facade.
# ruff: noqa: TID251
"""Evaluate one pinned historical case locally and retain its evidence bundle."""

import argparse
import json
from pathlib import Path
import platform
import subprocess
import time
from types import SimpleNamespace

from .provenance import check_artifacts, check_sources, environment, read_json, sha256, under, write_json
from . import run
from .verify_replay import compare_sends
from .evaluation_contract import SCOPE, Unsupported, canonical_request, completed_result


def compare_commands(baseline_path, candidate_path, window, same_source):
  """Compare integer wire commands at identical publications within scoring."""
  def selected(path):
    rows = read_json(path)['sends']
    stamps = [row['mono'] for row in rows]
    if any(type(t) is not int for t in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:], strict=False)):
      raise ValueError('Send identities must be strictly increasing integer nanoseconds')
    if any(type(row['counts']) is not int for row in rows):
      raise ValueError('Wire commands must be integer counts')
    return {r['mono']: r['counts'] for r in rows if round(window['start'] * 1e9) <= r['mono'] <= round(window['end'] * 1e9)}
  baseline, candidate = selected(baseline_path), selected(candidate_path)
  if not baseline or baseline.keys() != candidate.keys():
    raise ValueError('Baseline and candidate send coverage differs or is empty')
  deltas = [candidate[t] - baseline[t] for t in baseline]
  changed = sum(d != 0 for d in deltas)
  if same_source and changed:
    raise ValueError('Same-source candidate did not reproduce baseline integer commands')
  if same_source:
    compare_sends(baseline_path, candidate_path, window['start'], window['end'])
  return {'sends': len(deltas), 'changed_sends': changed, 'max_absolute_change_counts': max(map(abs, deltas)),
          'minimum_change_counts': min(deltas), 'maximum_change_counts': max(deltas),
          'same_source_commands_equal': changed == 0 if same_source else None,
          'domain': 'TI wire command counts; candidate minus baseline', 'scoring_window': window}


def verify_bundle(output, data_root):
  """Check a completed bundle against present source and raw bytes; does not replay the recorded runtime."""
  result = read_json(output / 'result.json')
  if result.get('format_version') != 1 or result.get('status') != 'completed_checks':
    raise ValueError('Only a completed version 1 evaluation can be verified')
  required = {'request.json', 'resolved-request.json', 'experiment.json', 'prepared/preparation.json',
              'baseline/result.json', 'candidate/result.json'}
  required.update('prepared/' + name for name in run.PREPARED_FILES)
  required.update(variant + '/' + name for variant in ('baseline', 'candidate') for name in run.REPLAY_FILES)
  check_artifacts(output, result['artifacts'], required)
  request = canonical_request(read_json(output / 'resolved-request.json'), resolved=True)
  case, prepared = request['case'], output / 'prepared'
  prep = read_json(prepared / 'preparation.json')
  if prep.get('format_version') != 2 or prep.get('stage') != 'prepared':
    raise ValueError('Unsupported preparation record')
  check_sources(prep['repository_sources'])
  check_artifacts(prepared, prep['prepared_sha256'], run.PREPARED_FILES)
  if read_json(prepared / 'spec.json') != case['experiment']:
    raise ValueError('Prepared experiment differs from resolved request')
  if prep['input_sha256'] != case['input_sha256']:
    raise ValueError('Preparation does not match requested recording identities')
  for name, digest in case['input_sha256'].items():
    if sha256(under(data_root, name)) != digest:
      raise ValueError(f'Recording identity changed: {name}')
  baseline_path = output / 'baseline/result.json'
  baseline = run.validate_baseline(baseline_path, sha256(prepared / 'preparation.json'), case['history'], prep['environment'])
  if baseline.get('input_sha256') != prep['input_sha256']:
    raise ValueError('Baseline recording identities differ from preparation')
  candidate = read_json(output / 'candidate/result.json')
  if (candidate.get('format_version') != 2 or candidate.get('stage') != 'replayed' or candidate.get('variant') != 'candidate'
      or candidate.get('qualification') != 'candidate_commands_only' or not candidate.get('command_bounds_pass')
      or candidate.get('frames', 0) <= 0 or candidate.get('sends', 0) <= 0
      or candidate.get('baseline_result_sha256') != sha256(baseline_path)
      or candidate.get('preparation_sha256') != baseline['preparation_sha256']
      or candidate.get('history') != case['history'] or candidate.get('environment') != prep['environment']
      or candidate.get('input_sha256') != prep['input_sha256']):
    raise ValueError('Candidate evidence is incomplete or does not match its baseline')
  check_sources(candidate['repository_sources'])
  check_artifacts(output / 'candidate', candidate['output_sha256'], run.REPLAY_FILES)
  spec = case['experiment']
  same = baseline['controller_sources'][spec['baseline_controller']] == candidate['controller_sources'][spec['candidate_controller']]
  comparison = compare_commands(output / 'baseline/trace-sends.json', output / 'candidate/trace-sends.json', spec['window'], same)
  if result != completed_result(request, baseline, candidate, comparison) | {'artifacts': result['artifacts']}:
    raise ValueError('Evaluation summary does not match its evidence')
  return result


def historical(request, data_root, output, result):
  case = request['case']
  experiment = dict(case['experiment'], candidate_controller=request['candidate_revision'])
  write_json(output / 'request.json', request)
  write_json(output / 'experiment.json', experiment)
  spec = run.spec_read(output / 'experiment.json')
  # Freeze aliases before prepare resolves the spec again; a moving branch must not select another controller.
  (output / 'experiment.json').write_text(json.dumps(spec, sort_keys=True, indent=2, allow_nan=False) + '\n', encoding='utf-8', newline='\n')
  resolved = canonical_request(dict(request, candidate_revision=spec['candidate_controller'], case=dict(case, experiment=spec)), resolved=True)
  write_json(output / 'resolved-request.json', resolved)
  result['history'] = case['history']
  result['window'] = spec['window']
  prepared = output / 'prepared'
  run.prepare(SimpleNamespace(spec=output / 'experiment.json', data_root=data_root, output=prepared))
  baseline_path = output / 'baseline/result.json'
  common = dict(prepared=prepared, data_root=data_root, history=case['history'])
  run.replay(SimpleNamespace(**common, variant='baseline', baseline_result=None, output=baseline_path.parent))
  baseline = run.validate_baseline(baseline_path, sha256(prepared / 'preparation.json'), case['history'], environment())
  result['qualification'] = baseline['qualification']
  run.replay(SimpleNamespace(**common, variant='candidate', baseline_result=baseline_path, output=output / 'candidate'))
  candidate = read_json(output / 'candidate/result.json')
  # Compare content identity as well as revision identity (different commits can contain identical controller bytes).
  same = baseline['controller_sources'][spec['baseline_controller']] == candidate['controller_sources'][spec['candidate_controller']]
  comparison = compare_commands(output / 'baseline/trace-sends.json', output / 'candidate/trace-sends.json', spec['window'], same)
  result.update(completed_result(resolved, baseline, candidate, comparison))


def evaluate(request_path, data_root, output):
  """Create a new bundle. Existing destinations are never overwritten."""
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  started = time.perf_counter()
  result = {'format_version': 1, 'status': 'failed_execution', 'case_id': None,
            'qualification': 'unqualified', 'comparison': None, 'findings': [], 'scope': SCOPE}
  try:
    request = canonical_request(read_json(request_path))
    case = request['case']
    result['case_id'] = case['id']
    for name in case['experiment']['rlogs']:
      path = under(Path(data_root), name)
      if not path.is_file():
        raise FileNotFoundError(f'Missing required recording: {name}')
      if sha256(path) != case['input_sha256'][name]:
        raise ValueError(f'Recording identity changed: {name}')
    historical(request, Path(data_root), output, result)
  except Unsupported as error:
    result['status'] = 'unsupported'
    result['findings'].append(str(error))
  except OSError as error:
    result['findings'].append(str(error))
  except (ValueError, KeyError, TypeError, AssertionError) as error:
    result['status'] = 'failed_check'
    result['findings'].append(str(error))
  except (RuntimeError, ImportError, AttributeError, IndexError, subprocess.SubprocessError) as error:
    result['findings'].append(str(error))
  result['artifacts'] = {p.relative_to(output).as_posix(): sha256(p) for p in sorted(output.rglob('*')) if p.is_file()}
  write_json(output / 'execution.json', {'elapsed_seconds': time.perf_counter() - started,
                                       'output_destination': str(output.resolve()),
                                       'data_root': str(Path(data_root).resolve()), 'platform': platform.platform()})
  write_json(output / 'result.json', result)
  report = f"# Evaluation: {result['case_id']}\n\nStatus: **{result['status']}**. Baseline: {result['qualification']}.\n\n{result['scope']}\n"
  report += ''.join(f'\n- {finding}\n' for finding in result['findings'])
  if result['comparison'] is not None:
    comparison = result['comparison']
    report += (f"\nHistory: {result['history']}. Scoring: {result['window']['start']}–{result['window']['end']} monotonic seconds.\n"
               f"\n{comparison['changed_sends']} of {comparison['sends']} scored sends changed; maximum absolute change "
               f"{comparison['max_absolute_change_counts']} wire counts.\n"
               f"\nBaseline controller: `{result['source_identities']['baseline_controller']}`.\n"
               f"Candidate controller: `{result['source_identities']['candidate_controller']}`.\n")
    report += ''.join(f'\n- {limit}\n' for limit in result['limitations'])
  report += '\nSee [structured result](result.json) and [execution metadata](execution.json). Artifact paths and hashes are in the result.\n'
  (output / 'report.md').write_text(report, encoding='utf-8', newline='\n')
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  selection = parser.add_mutually_exclusive_group(required=True)
  selection.add_argument('--request', type=Path)
  selection.add_argument('--verify-bundle', type=Path)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--output', type=Path, help='New evidence directory (required with --request)')
  args = parser.parse_args()
  if args.verify_bundle:
    try:
      verify_bundle(args.verify_bundle, args.data_root)
    except (OSError, ValueError, KeyError, TypeError) as error:
      parser.exit(1, f'Evidence verification failed: {error}\n')
    print('Bundle identities and recorded command comparison verified; recorded runtime was not executed.')
    return 0
  if args.output is None:
    parser.error('--output is required with --request')
  try:
    result = evaluate(args.request, args.data_root, args.output)
  except OSError as error:
    parser.exit(1, f'Cannot create evidence bundle: {error}\n')
  print(f"{result['status']}: {args.output / 'report.md'}")
  return 0 if result['status'] == 'completed_checks' else 1


if __name__ == '__main__':
  raise SystemExit(main())
