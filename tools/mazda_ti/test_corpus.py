"""Corpus/profile checks exercise real historical and instrumented adapters."""
import json
import os
from pathlib import Path

import pytest

from .example_fixture import create_example
from .test_instrumented_evaluation import instrumented_fixture
from .provenance import read_json
from .corpus import evaluate_corpus


def fixture(root):
  cases = []
  for method, factory in [('historical', create_example), ('instrumented', instrumented_fixture)]:
    request = read_json(factory(root / 'data' / method))
    case = request['case']
    case['id'] = method
    case['experiment']['rlogs'] = [method + '/' + p for p in case['experiment']['rlogs']]
    case['input_sha256'] = {method + '/' + p: h for p, h in case['input_sha256'].items()}
    cases.append({'case': case, 'control_revision': case['experiment']['baseline_controller'], 'categories': ['straight_gentle'],
                  'group': 'development', 'exposures': ['Synthetic fixture; not recorded validation'], 'annotations': [],
                  'expected': {'minimum_sends': 19, 'stock_commands': method == 'instrumented'}, 'exclusion': None})
  manifest = {'format_version': 1, 'id': 'test-corpus', 'version': '1', 'cases': cases, 'sources': [],
              'unavailable': {'successful_corner': 'No rider-confirmed successful passage'},
              'profiles': {'mixed': {'required_cases': ['historical', 'instrumented'], 'required_categories': ['straight_gentle'],
                                     'require_unexposed_withheld': False}}}
  path = root / 'manifest.json'
  path.write_text(json.dumps(manifest))
  return path


def test_mixed_corpus_runs_both_adapters_and_missing_required_evidence_blocks_profile(tmp_path):
  path = fixture(tmp_path)
  candidate = read_json(path)['cases'][1]['control_revision']
  result = evaluate_corpus(path, candidate, 'mixed', tmp_path / 'data', tmp_path, tmp_path / 'ledger.json', tmp_path / 'complete')
  assert result['status'] == 'completed_checks', result
  assert result['profile_pass'] is True
  assert [c['status'] for c in result['cases']] == ['completed_checks', 'completed_checks']
  assert result['unavailable']['successful_corner']
  (tmp_path / 'data/instrumented/synthetic--0/rlog').unlink()
  missing = evaluate_corpus(path, candidate, 'mixed', tmp_path / 'data', tmp_path, tmp_path / 'ledger.json', tmp_path / 'missing')
  assert missing['status'] == 'failed_check'
  assert missing['profile_pass'] is False
  assert missing['cases'][1]['status'] == 'failed_execution'
  assert missing['cases'][0]['status'] == 'completed_checks'
  assert missing['cases'][1]['comparison'] is None

def test_exposure_ledger_prevents_repeated_withheld_validation(tmp_path):
  path = fixture(tmp_path)
  manifest = read_json(path)
  manifest['cases'] = [manifest['cases'][0]]
  manifest['cases'][0].update(group='withheld', exposures=[])
  manifest['profiles']['mixed'].update(required_cases=['historical'], require_unexposed_withheld=True)
  path.write_text(json.dumps(manifest))
  candidate = manifest['cases'][0]['control_revision']
  ledger = tmp_path / 'ledger.json'
  first = evaluate_corpus(path, candidate, 'mixed', tmp_path / 'data', tmp_path, ledger, tmp_path / 'first')
  second = evaluate_corpus(path, candidate, 'mixed', tmp_path / 'data', tmp_path, ledger, tmp_path / 'second')
  assert first['profile_pass'] is True
  assert first['cases'][0]['exposed_by_run'] is True
  assert second['profile_pass'] is False
  assert second['cases'][0]['previously_exposed'] is True


def test_retained_baseline_exclusion_never_executes_a_candidate(tmp_path):
  from .provenance import sha256
  path = fixture(tmp_path)
  manifest = read_json(path)
  source = tmp_path / 'failed-baseline.json'
  source.write_text('{"qualification":"failed_baseline","integer_mismatches":1}')
  manifest['sources'] = [{'id': 'failure', 'path': source.name, 'sha256': sha256(source), 'kind': 'baseline_failure'}]
  manifest['cases'][1]['exclusion'] = {'reason': 'One-count baseline mismatch; no tolerance relaxation', 'source': 'failure'}
  path.write_text(json.dumps(manifest))
  result = evaluate_corpus(path, manifest['cases'][1]['control_revision'], 'mixed', tmp_path / 'data', tmp_path,
                           tmp_path / 'ledger.json', tmp_path / 'bundle')
  assert result['profile_pass'] is False
  assert result['cases'][1]['status'] == 'unsupported'
  assert result['cases'][1]['exclusion']['source'] == 'failure'
  assert not (tmp_path / 'bundle/cases/instrumented/candidate').exists()

def test_missing_annotation_evidence_blocks_required_case(tmp_path):
  path = fixture(tmp_path)
  manifest = read_json(path)
  manifest['sources'] = [{'id': 'rider', 'path': 'missing-rider.json', 'sha256': 'a' * 64, 'kind': 'rider'}]
  manifest['cases'][0]['annotations'] = [{'kind': 'contact', 'value': 'Contact end unknown', 'confidence': 'uncertain',
                                         'source': 'rider', 'window': [1.1, 1.2]}]
  path.write_text(json.dumps(manifest))
  result = evaluate_corpus(path, manifest['cases'][1]['control_revision'], 'mixed', tmp_path / 'data', tmp_path,
                           tmp_path / 'ledger.json', tmp_path / 'bundle')
  assert not result['profile_pass']
  assert result['cases'][0]['status'] == 'failed_check'
  assert not (tmp_path / 'bundle/cases/historical/candidate').exists()


def test_ti_analysis_cannot_become_confirmed_rider_contact(tmp_path):
  path = fixture(tmp_path)
  manifest = read_json(path)
  manifest['sources'] = [{'id': 'telemetry', 'path': 'telemetry.json', 'sha256': 'a' * 64, 'kind': 'analysis'}]
  manifest['cases'][0]['annotations'] = [{'kind': 'catch', 'value': 'steeringPressed', 'confidence': 'confirmed',
                                         'source': 'telemetry', 'window': [1.1, 1.2]}]
  path.write_text(json.dumps(manifest))
  result = evaluate_corpus(path, manifest['cases'][1]['control_revision'], 'mixed', tmp_path / 'data', tmp_path,
                           tmp_path / 'ledger.json', tmp_path / 'bundle')
  assert result['status'] == 'failed_check'
  assert not result['cases']

def test_unresolvable_candidate_has_execution_failure_status(tmp_path):
  path = fixture(tmp_path)
  result = evaluate_corpus(path, 'missing-corpus-candidate', 'mixed', tmp_path / 'data', tmp_path,
                           tmp_path / 'ledger.json', tmp_path / 'bundle')
  assert result['status'] == 'failed_execution'
  assert not result['profile_pass']


@pytest.mark.skipif(not os.environ.get('MAZDA_EVAL_DATA_ROOT'), reason='Private annotated corpus not supplied')
def test_recorded_mixed_corpus_preserves_qualified_and_failed_coverage(tmp_path):
  manifest = Path(__file__).with_name('examples') / 'corpus-v1.json'
  data = Path(os.environ['MAZDA_EVAL_DATA_ROOT'])
  result = evaluate_corpus(manifest, 'ad14961a29f59635d5f657a43000ca2a1ade62fb', 'command-regression', data, data.parent,
                           tmp_path / 'ledger.json', tmp_path / 'bundle')
  assert result['profile_pass'], result
  assert [c['status'] for c in result['cases']] == ['completed_checks'] * 4 + ['unsupported', 'failed_check']
  assert not (tmp_path / 'bundle/cases/outward-crossing-28f/candidate').exists()
  assert {'successful_corner', 'withheld', 'matched_conditions'} <= result['unavailable'].keys()

def test_malformed_profile_shape_returns_a_failed_bundle(tmp_path):
  path = fixture(tmp_path)
  manifest = read_json(path)
  manifest['profiles'] = []
  path.write_text(json.dumps(manifest))
  result = evaluate_corpus(path, manifest['cases'][0]['control_revision'], 'mixed', tmp_path / 'data', tmp_path,
                           tmp_path / 'ledger.json', tmp_path / 'bundle')
  assert result['status'] == 'failed_check'
  assert not result['cases']
  assert read_json(tmp_path / 'bundle/result.json') == result
