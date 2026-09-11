"""History effects require four comparable, verified evaluations, not two raw traces."""

import copy

import pytest

from .evaluate import evaluate
from .example_fixture import create_example
from .provenance import read_json, write_json
from .sampling_sensitivity import ARM_HISTORIES, _context, _difference_of_effects, compare_histories


def test_difference_of_incremental_effects_preserves_sign_and_send_identity():
  commands = {'earliest_control': {1: 10, 2: 20}, 'earliest_candidate': {1: 8, 2: 18},
              'latest_control': {1: 15, 2: 25}, 'latest_candidate': {1: 14, 2: 21}}
  result = _difference_of_effects(commands)
  assert result['minimum'] == -2 and result['maximum'] == 1
  assert result['maximum_absolute'] == 2 and result['first_maximum_absolute_send_ns'] == 2
  commands['latest_candidate'] = {1: 14, 3: 21}
  with pytest.raises(ValueError, match='identities'):
    _difference_of_effects(commands)


@pytest.mark.parametrize('mutation', ['window', 'candidate', 'control', 'runtime', 'dependency', 'history'])
def test_context_rejects_confounded_comparisons(mutation):
  arms = {}
  for name, history in ARM_HISTORIES.items():
    arms[name] = {'request': {'candidate_revision': 'control' if name.endswith('_control') else 'candidate',
                              'case': {'method': 'historical', 'id': 'one', 'history': history,
                                       'experiment': {'window': {'start': 1, 'end': 2}, 'candidate_controller': name}}},
                  'preparation': {'repository_sources': {'pid.py': 'same'}, 'environment': {'python': 'same'}}}
  assert _context(arms) == {'start': 1, 'end': 2}
  target = arms['latest_candidate']
  if mutation == 'window':
    target['request']['case']['experiment']['window']['start'] = 1.5
  elif mutation in ('candidate', 'control'):
    arms['latest_' + mutation]['request']['candidate_revision'] = 'different'
  elif mutation == 'runtime':
    target['preparation']['environment']['python'] = 'different'
  elif mutation == 'dependency':
    target['preparation']['repository_sources']['pid.py'] = 'different'
  else:
    target['request']['case']['history'] = 'earliest'
  with pytest.raises(ValueError):
    _context(arms)


def test_four_real_adapter_bundles_are_repeatable_and_tamper_is_rejected(tmp_path):
  raw = tmp_path / 'raw'
  template = read_json(create_example(raw))
  paths = {}
  for name, history in ARM_HISTORIES.items():
    request = copy.deepcopy(template)
    request['case']['history'] = history
    path = tmp_path / (name + '.json')
    write_json(path, request)
    paths[name] = tmp_path / name
    assert evaluate(path, raw, paths[name])['status'] == 'completed_checks'
  result = compare_histories(paths, raw, tmp_path, tmp_path / 'report-one')
  assert result['history_effect_difference']['maximum_absolute'] == 0
  assert result['candidate_minus_current_control']['earliest']['changed_sends'] == 0
  assert result['candidate_minus_current_control']['latest']['changed_sends'] == 0
  compare_histories(paths, raw, tmp_path, tmp_path / 'report-two')
  assert (tmp_path / 'report-one/result.json').read_bytes() == (tmp_path / 'report-two/result.json').read_bytes()
  with pytest.raises(ValueError, match='escapes'):
    compare_histories(paths, raw, raw, tmp_path / 'bad-root')
  (paths['latest_candidate'] / 'candidate/trace-sends.json').write_text('{}')
  with pytest.raises(ValueError, match='Artifact changed'):
    compare_histories(paths, raw, tmp_path, tmp_path / 'bad-evidence')
  assert not (tmp_path / 'bad-evidence').exists()
