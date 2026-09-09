"""Compare a policy's incremental command effects under two qualified receipt histories."""

import argparse
from pathlib import Path

from .evaluate import compare_commands, verify_bundle
from .provenance import read_json, sha256, write_json


ARM_HISTORIES = {'earliest_control': 'earliest', 'earliest_candidate': 'earliest',
                 'latest_control': 'latest', 'latest_candidate': 'latest'}
SCOPE = ('Two output-compatible sampling histories, not bounds over all receipt histories. '
         'Differences describe software commands on fixed recorded motion, not vehicle handling.')


def _context(arms):
  normalized = []
  for name, arm in arms.items():
    request = arm['request']
    case = dict(request['case'])
    if case['method'] != 'historical' or case.pop('history') != ARM_HISTORIES[name]:
      raise ValueError('Expected qualified historical earliest/latest arms')
    case['experiment'] = dict(case['experiment'])
    case['experiment'].pop('candidate_controller')
    normalized.append(case)
  if any(case != normalized[0] for case in normalized[1:]):
    raise ValueError('Cases differ beyond candidate revision and sampling history')
  for role in ('control', 'candidate'):
    if arms['earliest_' + role]['request']['candidate_revision'] != arms['latest_' + role]['request']['candidate_revision']:
      raise ValueError('Controller revision changed between sampling histories')
  for field in ('repository_sources', 'environment'):
    values = [arm['preparation'][field] for arm in arms.values()]
    if any(value != values[0] for value in values[1:]):
      raise ValueError('Dependency or recorded runtime identities differ between arms')
  return normalized[0]['experiment']['window']


def _difference_of_effects(commands):
  first = commands['earliest_control']
  if not first or any(rows.keys() != first.keys() for rows in commands.values()):
    raise ValueError('Nonempty exact send identities must align across all four arms')
  differences = {
    t: (commands['latest_candidate'][t] - commands['latest_control'][t]) -
       (commands['earliest_candidate'][t] - commands['earliest_control'][t]) for t in first
  }
  peak = max(differences, key=lambda t: abs(differences[t]))
  return {'units': 'published TI wire counts', 'sends': len(first),
          'changed_effect_sends': sum(value != 0 for value in differences.values()),
          'minimum': min(differences.values()), 'maximum': max(differences.values()),
          'maximum_absolute': abs(differences[peak]), 'first_maximum_absolute_send_ns': peak,
          'definition': '(latest candidate - latest current control) - (earliest candidate - earliest current control)'}


def compare_histories(paths, data_root, evidence_root, output):
  """Verify all four full bundles before reporting source-matched history sensitivity."""
  if set(paths) != set(ARM_HISTORIES):
    raise ValueError('Exactly four named history/control arms are required')
  evidence_root, output = Path(evidence_root).resolve(), Path(output)
  arms, provenance = {}, {}
  for name, value in paths.items():
    path = Path(value).resolve()
    try:
      relative = path.relative_to(evidence_root).as_posix()
    except ValueError:
      raise ValueError('Evaluation arm escapes the declared evidence root') from None
    verify_bundle(path, Path(data_root))
    arms[name] = {'path': path, 'request': read_json(path / 'resolved-request.json'),
                  'preparation': read_json(path / 'prepared/preparation.json')}
    provenance[name] = {'bundle': relative, 'result_sha256': sha256(path / 'result.json')}
  window = _context(arms)
  start, end = round(window['start'] * 1e9), round(window['end'] * 1e9)
  commands, effects, ambiguity = {}, {}, {}
  for history in ('earliest', 'latest'):
    control, candidate = (arms[history + '_' + role]['path'] for role in ('control', 'candidate'))
    effects[history] = compare_commands(control / 'candidate/trace-sends.json', candidate / 'candidate/trace-sends.json', window, False)
    for role, path in (('control', control), ('candidate', candidate)):
      commands[history + '_' + role] = {r['mono']: r['counts'] for r in read_json(path / 'candidate/trace-sends.json')['sends']
                                        if start <= r['mono'] <= end}
    frames = [r for r in read_json(control / 'prepared/sampling.json')['frames'] if start <= r['send'] <= end]
    if not frames:
      raise ValueError('Sampling ambiguity evidence is empty in the scoring window')
    ambiguity[history] = {
      'sends': len(frames), 'ambiguous_sends': sum(r['compatible_count'] > 1 for r in frames),
      'maximum_compatible_requests': max(r['compatible_count'] for r in frames),
      'maximum_selected_request_age_ns': max(r['send'] - r[history + '_request'] for r in frames),
      'scope': 'Age at recorded send publication, not measured receipt latency or firmware sampling time',
    }
  result = {'format_version': 1, 'status': 'completed_checks', 'identity_validation': 'verified_full_bundles',
            'scope': SCOPE, 'window': window, 'provenance': provenance, 'candidate_minus_current_control': effects,
            'history_effect_difference': _difference_of_effects(commands), 'sampling_ambiguity': ambiguity,
            'analyzer_sha256': sha256(Path(__file__))}
  output.mkdir(parents=True, exist_ok=False)
  write_json(output / 'result.json', result)
  lines = ['# Historical sampling sensitivity', '', SCOPE, '', 'History | Changed sends | Maximum incremental TI counts',
           '--- | ---: | ---:']
  lines += [f"{name} | {effect['changed_sends']} | {effect['max_absolute_change_counts']}" for name, effect in effects.items()]
  lines += ['', f"Maximum difference between incremental effects: {result['history_effect_difference']['maximum_absolute']} TI counts.",
            '', 'Request ages and all four verified result identities are retained in result.json.']
  (output / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8', newline='\n')
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  for name in ARM_HISTORIES:
    parser.add_argument('--' + name.replace('_', '-'), type=Path, required=True)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--evidence-root', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  compare_histories({name: getattr(args, name) for name in ARM_HISTORIES}, args.data_root, args.evidence_root, args.output)
  print(f'completed_checks: {args.output / "report.md"}')


if __name__ == '__main__':
  main()
