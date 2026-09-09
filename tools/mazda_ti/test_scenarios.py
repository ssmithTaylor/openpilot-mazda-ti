"""Synthetic Mazda controller scenarios remain auditable without a vehicle."""

import subprocess
import sys

from .provenance import read_json, write_json
from .scenarios import run


def revision():
  return subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()


def request(case_id, scenario):
  source = revision()
  return {
    'format_version': 1,
    'candidate_revision': source,
    'case': {
      'id': case_id,
      'method': 'synthetic',
      'origin': 'synthetic',
      'scenario': scenario,
      'baseline_controller': source,
      'limiter': source,
    },
  }


def test_known_570_fixture_fails_without_hiding_recovery_or_carryover(tmp_path):
  request_path = tmp_path / 'known-defect.json'
  write_json(request_path, request('known-570-compensation-defect', 'known_570_defect'))

  result = run(request_path, tmp_path / 'known-defect')

  assert result['status'] == 'failed_check'
  assert result['qualification'] == 'synthetic_controller_limiter'
  assert result['comparison']['command_differences']
  assert any('570-count' in finding for finding in result['findings'])
  trace = read_json(tmp_path / 'known-defect' / 'trace.json')
  assert any(row['phase'] == 'recovery' for row in trace)
  assert any(row['phase'] == 'carryover' for row in trace)
  assert result['scope'].startswith('Synthetic fixed-input')


def test_reference_fixture_passes_real_controller_limiter_transitions(tmp_path):
  request_path = tmp_path / 'reference.json'
  write_json(request_path, request('reference-controller-transitions', 'reference_transitions'))

  result = run(request_path, tmp_path / 'reference')

  assert result['status'] == 'completed_checks', result['findings']
  assert result['comparison']['hard_invariant_failures'] == []
  trace = read_json(tmp_path / 'reference' / 'trace.json')
  assert {row['phase'] for row in trace} >= {'tightening', 'unwind', 'reversal', 'clipping', 'inactive', 'recovery', 'carryover'}
  assert max(abs(row['limited_counts']) for row in trace) <= 600
  tightening = [row['limited_counts'] for row in trace if row['phase'] == 'tightening']
  reversal = [row['limited_counts'] for row in trace if row['phase'] == 'reversal']
  clipping = [row['limited_counts'] for row in trace if row['phase'] == 'clipping']
  inactive = [row for row in trace if row['phase'] == 'inactive']
  carryover = [row for row in trace if row['phase'] == 'carryover']
  assert max(tightening) > 0 and min(reversal) < 0
  assert clipping.count(-600) >= 100
  assert all(row['limited_counts'] == 0 and row['integral'] == 0 for row in inactive)
  assert carryover[0]['limited_counts'] != 0 and carryover[0]['integral'] != 0
  assert all(len(result['source_identities'][key]) == 40 for key in ('baseline_controller', 'candidate_controller', 'limiter'))
  assert 'execution.json' not in result['artifacts']


def test_request_rejects_abbreviated_source_revision_without_copying_data(tmp_path):
  value = request('invalid-revision', 'reference_transitions')
  value['candidate_revision'] = value['candidate_revision'][:12]
  request_path = tmp_path / 'invalid.json'
  write_json(request_path, value)

  result = run(request_path, tmp_path / 'invalid')

  assert result['status'] == 'failed_check'
  assert result['findings'] == ['candidate_revision must be a full 40-character Git revision']
  assert not (tmp_path / 'invalid' / 'trace.json').exists()


def test_request_can_compare_an_explicitly_pinned_controller(tmp_path):
  value = request('pinned-controller', 'reference_transitions')
  value['candidate_revision'] = '9c6c8abf2d85330c701284c7d6122bda3c0edf5f'
  request_path = tmp_path / 'pinned.json'
  write_json(request_path, value)

  result = run(request_path, tmp_path / 'pinned')

  assert result['status'] == 'completed_checks', result['findings']
  assert result['source_identities']['candidate_controller'] == value['candidate_revision']
  assert result['source_identities']['controller_sources'][value['candidate_revision']]


def test_cli_returns_failure_for_known_defect_after_writing_recovery_evidence(tmp_path):
  request_path = tmp_path / 'known-defect.json'
  output = tmp_path / 'known-defect'
  write_json(request_path, request('known-570-cli', 'known_570_defect'))

  completed = subprocess.run([sys.executable, '-m', 'tools.mazda_ti.scenarios', '--request', str(request_path), '--output', str(output)],
                             text=True, capture_output=True, check=False)

  assert completed.returncode == 1
  assert completed.stdout.startswith('failed_check:')
  assert any(row['phase'] == 'carryover' for row in read_json(output / 'trace.json'))
