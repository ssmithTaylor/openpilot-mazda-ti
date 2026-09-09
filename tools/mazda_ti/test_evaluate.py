# ruff: noqa: TID251
"""The evaluation boundary retains an explicit outcome even without coverage."""

import json
import os
from pathlib import Path

import pytest

from .evaluate import compare_commands, evaluate, verify_bundle
from .provenance import read_json, sha256, write_json
from .run import PREPARED_FILES, REPLAY_FILES
from .test_workflow import evidence  # noqa: F401 - shared structural evidence fixture
from .example_fixture import create_example
from .evaluation_contract import SCOPE, LIMITATIONS


def sample_request():
  return read_json(Path(__file__).with_name('examples') / 'vw-evaluation.json')


@pytest.mark.parametrize('failure,status', [('missing_recording', 'failed_execution'), ('changed_recording', 'failed_check'),
                                          ('settings', 'failed_check'), ('method', 'unsupported'), ('version', 'unsupported')])
def test_synthetic_early_failures_never_claim_recorded_physical_evidence(tmp_path, failure, status):
  request = sample_request()
  case = request['case']
  case['origin'] = 'synthetic'
  case['experiment']['rlogs'] = ['rlog']
  case['input_sha256'] = {'rlog': '0' * 64}
  if failure == 'changed_recording':
    (tmp_path / 'rlog').write_bytes(b'changed')
  elif failure == 'settings':
    case['experiment']['settings']['SteerKP'] = 'invalid'
  elif failure == 'method':
    case['method'] = 'instrumented'
  elif failure == 'version':
    request['format_version'] = 99
  path, output = tmp_path / 'request.json', tmp_path / 'bundle'
  write_json(path, request)
  result = evaluate(path, tmp_path, output)
  assert result['status'] == status
  assert result['comparison'] is None
  assert 'no recorded physical handling evidence' in result['scope']
  report = (output / 'report.md').read_text()
  assert result['scope'] in report
  assert 'under recorded physical motion' not in report


@pytest.mark.parametrize('location', ['request', 'case', 'experiment', 'window', 'settings', 'setting_value'])
def test_unrestricted_request_data_is_rejected_before_any_bundle_copy(tmp_path, location):
  request = read_json(Path(__file__).with_name('examples') / 'vw-evaluation.json')
  case, secret = request['case'], 'PRIVATE-CONTENT-MUST-NOT-APPEAR'
  targets = {'request': request, 'case': case, 'experiment': case['experiment'],
             'window': case['experiment']['window'], 'settings': case['experiment']['settings']}
  if location == 'setting_value':
    case['experiment']['settings']['SteerKP'] = secret
  else:
    targets[location]['unrestrictedParams'] = secret
  path = tmp_path / 'request.json'
  write_json(path, request)
  output = tmp_path / 'bundle'
  result = evaluate(path, tmp_path, output)
  assert result['status'] == 'failed_check'
  assert all(secret not in p.read_text() for p in output.rglob('*') if p.is_file())


def test_unsupported_method_produces_a_readable_unscored_bundle(tmp_path):
  request = tmp_path / 'request.json'
  write_json(request, {'format_version': 1, 'case': {'id': 'instrumented-example', 'method': 'instrumented'}})
  output = tmp_path / 'evidence'
  result = evaluate(request, tmp_path / 'data', output)
  assert result['status'] == 'unsupported'
  assert result['comparison'] is None
  assert read_json(output / 'result.json') == result
  assert 'recorded physical motion' in (output / 'report.md').read_text()


def test_missing_recording_retains_failed_execution_without_candidate_scores(tmp_path):
  request = tmp_path / 'request.json'
  value = sample_request()
  value['case']['experiment']['rlogs'] = ['route--3/rlog']
  value['case']['input_sha256'] = {'route--3/rlog': '0' * 64}
  write_json(request, value)
  result = evaluate(request, tmp_path / 'data', tmp_path / 'evidence')
  assert result['status'] == 'failed_execution'
  assert result['comparison'] is None
  assert result['findings'] == ['Missing required recording: route--3/rlog']


def test_changed_recording_is_a_failed_check(tmp_path):
  (tmp_path / 'rlog').write_bytes(b'changed')
  request = tmp_path / 'request.json'
  value = sample_request()
  value['case']['experiment']['rlogs'] = ['rlog']
  value['case']['input_sha256'] = {'rlog': '0' * 64}
  write_json(request, value)
  result = evaluate(request, tmp_path, tmp_path / 'evidence')
  assert result['status'] == 'failed_check'
  assert result['comparison'] is None
  assert result['findings'] == ['Recording identity changed: rlog']


@pytest.mark.skipif(not os.environ.get('MAZDA_EVAL_DATA_ROOT'), reason='Private recorded fixture not supplied; see EVALUATION.md')
def test_recorded_same_source_evaluation_reproduces_baseline(tmp_path):
  request = Path(__file__).with_name('examples') / 'vw-evaluation.json'
  result = evaluate(request, Path(os.environ['MAZDA_EVAL_DATA_ROOT']), tmp_path / 'evidence')
  assert result['status'] == 'completed_checks', result['findings']
  assert result['qualification'] == 'exact_recorded_commands'
  assert result['comparison']['changed_sends'] == 0
  assert result['comparison']['same_source_commands_equal'] is True
  assert result['comparison']['sends'] > 0
  assert verify_bundle(tmp_path / 'evidence', Path(os.environ['MAZDA_EVAL_DATA_ROOT'])) == result


def test_public_synthetic_reference_completes_real_pipeline_without_private_data(tmp_path):
  raw = tmp_path / 'data'
  request = create_example(raw)
  result = evaluate(request, raw, tmp_path / 'bundle')
  assert result['status'] == 'completed_checks', result['findings']
  assert result['comparison']['sends'] == 19
  assert result['comparison']['changed_sends'] == 0
  assert result['comparison']['same_source_commands_equal'] is True
  assert 'no recorded physical handling evidence' in result['scope']
  assert verify_bundle(tmp_path / 'bundle', raw) == result
  repeat = tmp_path / 'same-data'
  assert read_json(create_example(repeat)) == read_json(request)


def test_command_comparison_preserves_opposing_effects_and_excludes_warmup(tmp_path):
  baseline, candidate = tmp_path / 'baseline.json', tmp_path / 'candidate.json'
  write_json(baseline, {'sends': [{'mono': 1, 'counts': 500}, {'mono': 2, 'counts': 100}, {'mono': 3, 'counts': 100}]})
  write_json(candidate, {'sends': [{'mono': 1, 'counts': 0}, {'mono': 2, 'counts': 90}, {'mono': 3, 'counts': 115}]})
  result = compare_commands(baseline, candidate, {'start': 2e-9, 'end': 3e-9}, False)
  assert result['changed_sends'] == 2
  assert result['minimum_change_counts'] == -10
  assert result['maximum_change_counts'] == 15
  assert result['max_absolute_change_counts'] == 15
  with pytest.raises(ValueError, match='Same-source'):
    compare_commands(baseline, candidate, {'start': 2e-9, 'end': 3e-9}, True)


@pytest.mark.parametrize('sends', [[], [{'mono': 2, 'counts': 0}], [{'mono': 1, 'counts': 0}, {'mono': 1, 'counts': 0}]])
def test_command_comparison_rejects_missing_or_duplicated_publications(tmp_path, sends):
  baseline, candidate = tmp_path / 'baseline.json', tmp_path / 'candidate.json'
  write_json(baseline, {'sends': [{'mono': 1, 'counts': 0}, {'mono': 2, 'counts': 0}]})
  write_json(candidate, {'sends': sends})
  with pytest.raises(ValueError):
    compare_commands(baseline, candidate, {'start': 0, 'end': 1}, False)


@pytest.fixture
def reference_bundle(evidence):
  """Small structural verifier fixture; not a substitute for the recorded controller evaluation."""
  root, baseline_path, baseline, runtime = evidence
  prepared = root / 'prepared'
  prepared.mkdir()
  (root / 'rlog').write_bytes(b'structural raw fixture')
  request = sample_request()
  request['candidate_revision'] = 'pinned'
  spec = request['case']['experiment']
  spec.update(baseline_controller='pinned', candidate_controller='pinned', warmup_controller='pinned', limiter='pinned',
              rlogs=['rlog'], window={'anchor_i_at': -.2, 'activation': 0, 'start': 0, 'end': 1})
  request['case'].update(origin='recorded', input_sha256={'rlog': sha256(root / 'rlog')})
  for name in ('request.json', 'resolved-request.json', 'experiment.json'):
    write_json(root / name, request)
  for name in PREPARED_FILES:
    write_json(prepared / name, spec if name == 'spec.json' else {})
  prep = {'format_version': 2, 'stage': 'prepared', 'environment': runtime, 'repository_sources': baseline['repository_sources'],
          'input_sha256': request['case']['input_sha256'], 'prepared_sha256': {name: sha256(prepared / name) for name in PREPARED_FILES}}
  write_json(prepared / 'preparation.json', prep)
  (baseline_path.parent / 'trace-sends.json').write_text('{"sends": [{"mono": 1, "counts": 10}, {"mono": 2, "counts": 20}]}')
  baseline['preparation_sha256'] = sha256(prepared / 'preparation.json')
  baseline['controller_sources'] = {'pinned': 'same-content'}
  baseline['output_sha256'] = {name: sha256(baseline_path.parent / name) for name in REPLAY_FILES}
  baseline['input_sha256'] = prep['input_sha256']
  baseline_path.write_text(json.dumps(baseline))
  candidate_dir = root / 'candidate'
  candidate_dir.mkdir()
  for name in REPLAY_FILES:
    (candidate_dir / name).write_bytes((baseline_path.parent / name).read_bytes())
  write_json(candidate_dir / 'result.json', dict(baseline, variant='candidate', qualification='candidate_commands_only', limitations=[],
                                              baseline_result_sha256=sha256(baseline_path), input_sha256=prep['input_sha256']))
  artifact_paths = ['request.json', 'resolved-request.json', 'experiment.json', 'prepared/preparation.json',
                    'baseline/result.json', 'candidate/result.json']
  artifact_paths += ['prepared/' + name for name in PREPARED_FILES]
  artifact_paths += [variant + '/' + name for variant in ('baseline', 'candidate') for name in REPLAY_FILES]
  result = {'format_version': 1, 'status': 'completed_checks', 'qualification': 'exact_recorded_commands',
            'case_id': request['case']['id'], 'history': 'earliest', 'window': spec['window'], 'findings': [], 'scope': SCOPE,
            'input_sha256': prep['input_sha256'], 'runtime': runtime, 'limitations': LIMITATIONS,
            'source_identities': {'baseline_controller': 'pinned', 'candidate_controller': 'pinned', 'warmup_controller': 'pinned',
                                  'limiter': 'pinned', 'repository_sources': baseline['repository_sources'],
                                  'controller_sources': baseline['controller_sources']},
            'artifacts': {name: sha256(root / name) for name in artifact_paths},
            'comparison': compare_commands(baseline_path.parent / 'trace-sends.json', candidate_dir / 'trace-sends.json', spec['window'], True)}
  write_json(root / 'result.json', result)
  return root, result


def test_reference_bundle_verifies_without_executing_the_recorded_runtime(reference_bundle):
  root, expected = reference_bundle
  assert verify_bundle(root, root) == expected


def test_reference_bundle_rejects_changed_source(reference_bundle):
  root, _ = reference_bundle
  (root / 'selfdrive/controls/lib/pid.py').write_bytes(b'changed after qualification')
  with pytest.raises(ValueError, match='Source identity changed'):
    verify_bundle(root, root)


def test_reference_bundle_rejects_unqualified_baseline_even_with_updated_hash(reference_bundle):
  root, result = reference_bundle
  baseline_path = root / 'baseline/result.json'
  baseline = read_json(baseline_path)
  baseline['qualification'] = 'failed_baseline'
  baseline_path.write_text(json.dumps(baseline))
  result['artifacts']['baseline/result.json'] = sha256(baseline_path)
  (root / 'result.json').write_text(json.dumps(result))
  with pytest.raises(ValueError, match='Baseline is unqualified'):
    verify_bundle(root, root)


def test_reference_bundle_rejects_changed_comparison(reference_bundle):
  root, result = reference_bundle
  result['comparison']['changed_sends'] = 1
  (root / 'result.json').write_text(json.dumps(result))
  with pytest.raises(ValueError, match='summary does not match'):
    verify_bundle(root, root)


@pytest.mark.parametrize('field', ['case_id', 'history', 'window', 'input_sha256', 'runtime', 'source_identities',
                                  'limitations', 'scope', 'findings', 'qualification'])
def test_reference_bundle_rejects_every_changed_provenance_field(reference_bundle, field):
  root, result = reference_bundle
  result[field] = 'forged'
  (root / 'result.json').write_text(json.dumps(result))
  with pytest.raises(ValueError, match='summary does not match'):
    verify_bundle(root, root)


def test_reference_bundle_rejects_missing_raw_data(reference_bundle):
  root, _ = reference_bundle
  (root / 'rlog').unlink()
  with pytest.raises(FileNotFoundError):
    verify_bundle(root, root)


def test_unknown_request_version_is_unsupported(tmp_path):
  request = tmp_path / 'request.json'
  write_json(request, {'format_version': 99})
  result = evaluate(request, tmp_path, tmp_path / 'evidence')
  assert result['status'] == 'unsupported'
  assert result['comparison'] is None


def test_existing_bundle_is_preserved(tmp_path):
  output = tmp_path / 'evidence'
  output.mkdir()
  (output / 'important').write_bytes(b'keep')
  with pytest.raises(FileExistsError):
    evaluate(tmp_path / 'missing-request.json', tmp_path, output)
  assert (output / 'important').read_bytes() == b'keep'
