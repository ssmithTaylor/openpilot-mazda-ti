"""Bounded corpus execution preserves common-adapter evidence."""

import copy
import json
from pathlib import Path

import pytest

from . import batch
from .batch import evaluate_batch
from .example_fixture import create_example
from .provenance import read_json, write_json


def manifest(root, cases, workers=2):
  path = root / 'corpus.json'
  write_json(path, {'format_version': 1, 'resources': {'max_workers': workers, 'host_memory_mb': 4096,
                                                        'worker_memory_mb': 256}, 'cases': cases})
  return path


def requests(root, count=2):
  create_example(root / 'raw')
  original = read_json(root / 'raw/request.json')
  result = []
  for index in range(count):
    value = copy.deepcopy(original)
    value['case']['id'] = f'case-{index}'
    path = root / f'case-{index}.json'
    write_json(path, value)
    result.append({'id': f'case-{index}', 'request': path.name, 'isolation': 'demonstrated'})
  return result


def test_serial_and_parallel_runs_have_identical_canonical_case_results(tmp_path):
  cases = requests(tmp_path)
  corpus = manifest(tmp_path, cases)
  serial = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'serial', cache_root=tmp_path / 'cache', workers=1)
  parallel = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'parallel', cache_root=tmp_path / 'cache-2', workers=2)

  assert serial['status'] == parallel['status'] == 'completed_checks'
  assert serial['canonical'] == parallel['canonical']
  assert parallel['execution']['workers'] == 2
  assert all(row['status'] == 'completed_checks' for row in serial['canonical']['cases'])


def test_relocated_bytes_reuse_valid_cache_without_changing_canonical_result(tmp_path):
  cases = requests(tmp_path)
  corpus = manifest(tmp_path, cases)
  first = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'first', cache_root=tmp_path / 'cache')
  relocated = tmp_path / 'relocated'
  create_example(relocated)
  second = evaluate_batch(corpus, relocated, tmp_path / 'second', cache_root=tmp_path / 'cache')

  assert first['canonical'] == second['canonical']
  assert all(row['reused'] for row in second['execution']['cases'])
  assert second['execution']['host']['system']


def test_interrupted_batch_resumes_completed_cases_and_keeps_failed_attempt(tmp_path):
  cases = requests(tmp_path)
  missing = read_json(tmp_path / 'case-1.json')
  missing['case']['experiment']['rlogs'] = ['missing/rlog']
  missing['case']['input_sha256'] = {'missing/rlog': '0' * 64}
  (tmp_path / 'case-1.json').unlink()
  write_json(tmp_path / 'case-1.json', missing)
  corpus = manifest(tmp_path, cases)
  interrupted = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'resume', cache_root=tmp_path / 'cache', stop_after_cases=1)
  assert interrupted['status'] == 'interrupted'
  assert (tmp_path / 'resume/cases/case-0/attempt-001/result.json').exists()

  failed = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'resume', cache_root=tmp_path / 'cache')
  assert failed['status'] == 'failed_check'
  first_failed = tmp_path / 'resume/cases/case-1/attempt-001/result.json'
  assert first_failed.exists()
  repaired = read_json(tmp_path / 'raw/request.json')
  repaired['case']['id'] = 'case-1'
  (tmp_path / 'case-1.json').unlink()
  write_json(tmp_path / 'case-1.json', repaired)

  resumed = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'resume', cache_root=tmp_path / 'cache')
  assert resumed['status'] == 'completed_checks'
  assert first_failed.exists()
  assert (tmp_path / 'resume/cases/case-1/attempt-002/result.json').exists()


def test_invalid_cache_is_retained_and_recomputed_selectively(tmp_path):
  cases = requests(tmp_path)
  corpus = manifest(tmp_path, cases)
  evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'first', cache_root=tmp_path / 'cache')
  cached = next((tmp_path / 'cache').glob('*/attempt-001/candidate/trace-sends.json'))
  cached.write_bytes(b'corrupt')

  result = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'second', cache_root=tmp_path / 'cache')

  assert result['status'] == 'completed_checks'
  assert result['execution']['cache_invalidations']
  assert sorted(row['reused'] for row in result['execution']['cases']) == [False, True]
  assert result['execution']['throughput_cases_per_second'] > 0
  assert cached.exists()
  assert len(list((tmp_path / 'cache').glob('*/attempt-*/result.json'))) > len(cases)


def test_branch_reference_is_frozen_and_unisolated_process_group_runs_serially(tmp_path):
  cases = requests(tmp_path)
  for case in cases:
    case['process_group'] = 'openpilot-runtime'
    case['isolation'] = 'unproven'
    value = read_json(tmp_path / case['request'])
    value['candidate_revision'] = 'HEAD'
    Path(tmp_path / case['request']).unlink()
    write_json(tmp_path / case['request'], value)
  result = evaluate_batch(manifest(tmp_path, cases), tmp_path / 'raw', tmp_path / 'serial-group', cache_root=tmp_path / 'cache', workers=2)

  assert result['status'] == 'completed_checks'
  assert result['execution']['workers'] == 1
  assert all(len(row['frozen_request']['candidate_revision']) == 40 for row in result['canonical']['cases'])


@pytest.mark.parametrize('change', ['settings', 'candidate', 'input'])
def test_changed_request_dependencies_do_not_reuse_same_case_attempt(tmp_path, change):
  cases = requests(tmp_path, count=1)
  corpus = manifest(tmp_path, cases, workers=1)
  first = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'output', cache_root=tmp_path / 'cache', workers=1)
  assert first['status'] == 'completed_checks'

  request_path = tmp_path / cases[0]['request']
  request = read_json(request_path)
  if change == 'settings':
    request['case']['experiment']['settings']['SteerKP'] = 1.234
  elif change == 'candidate':
    request['candidate_revision'] = 'HEAD'
  else:
    request['case']['input_sha256']['synthetic--0/rlog'] = '0' * 64
  request_path.unlink()
  write_json(request_path, request)

  second = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'output', cache_root=tmp_path / 'cache', workers=1)
  row = second['execution']['cases'][0]
  assert row['reused'] is False
  assert (tmp_path / 'output/cases/case-0/attempt-002/result.json').exists()


def test_worktree_candidate_is_frozen_to_head(tmp_path, monkeypatch):
  cases = requests(tmp_path, count=1)
  monkeypatch.setattr(batch, '_clean_worktree', lambda: None)
  request_path = tmp_path / cases[0]['request']
  request = read_json(request_path)
  request['candidate_revision'] = 'worktree'
  request_path.unlink()
  write_json(request_path, request)

  result = evaluate_batch(manifest(tmp_path, cases, workers=1), tmp_path / 'raw', tmp_path / 'output', workers=1)

  assert result['status'] == 'completed_checks'
  assert len(result['canonical']['cases'][0]['frozen_request']['candidate_revision']) == 40


@pytest.mark.parametrize('dirty_name, content', [
  ('tools/mazda_ti/README.md', b'\n'),
  ('tools/mazda_ti/_review_untracked_source.py', b'# temporary source\n'),
])
def test_worktree_rejects_dirty_relevant_source(tmp_path, dirty_name, content):
  cases = requests(tmp_path, count=1)
  target = Path(dirty_name)
  original = target.read_bytes() if target.exists() else None
  try:
    target.write_bytes((original or b'') + content)
    request_path = tmp_path / cases[0]['request']
    request = read_json(request_path)
    request['candidate_revision'] = 'worktree'
    request_path.unlink()
    write_json(request_path, request)
    with pytest.raises(ValueError, match='Worktree candidate is mutable'):
      evaluate_batch(manifest(tmp_path, cases, workers=1), tmp_path / 'raw', tmp_path / 'output', workers=1)
  finally:
    if original is None:
      target.unlink(missing_ok=True)
    else:
      target.write_bytes(original)


def test_source_drift_after_freeze_rejects_batch(tmp_path, monkeypatch):
  cases = requests(tmp_path, count=1)
  snapshots = iter([{'source': 'before'}, {'source': 'after'}])
  monkeypatch.setattr(batch, 'source_snapshot', lambda: next(snapshots))

  with pytest.raises(RuntimeError, match='Source drift after batch freeze'):
    evaluate_batch(manifest(tmp_path, cases, workers=1), tmp_path / 'raw', tmp_path / 'output', workers=1)


def test_runtime_change_invalidates_completed_cache(tmp_path, monkeypatch):
  cases = requests(tmp_path, count=1)
  corpus = manifest(tmp_path, cases, workers=1)
  evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'first', cache_root=tmp_path / 'cache', workers=1)
  monkeypatch.setattr(batch, 'environment', lambda: {'changed': True})

  result = evaluate_batch(corpus, tmp_path / 'raw', tmp_path / 'second', cache_root=tmp_path / 'cache', workers=1)

  assert result['status'] == 'completed_checks'
  assert result['execution']['cases'] == [{'id': 'case-0', 'reused': False}]


@pytest.mark.parametrize('identity', ['.', '..', 'C:', '../escaped', 'case/child'])
def test_case_identity_cannot_escape_its_output_directory(tmp_path, identity):
  cases = requests(tmp_path, count=1)
  cases[0]['id'] = identity

  with pytest.raises(ValueError, match='Invalid batch case identity'):
    evaluate_batch(manifest(tmp_path, cases), tmp_path / 'raw', tmp_path / 'output')
