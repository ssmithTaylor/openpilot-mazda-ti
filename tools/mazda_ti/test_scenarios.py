"""Synthetic Mazda controller scenarios remain auditable without a vehicle."""

import subprocess
import sys

import pytest

from .provenance import read_json, write_json
from .scenarios import _feedback_frames, _run_controller, run


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


def test_feedback_scenario_uses_previous_apply_and_changes_integral_history(tmp_path):
  request_path = tmp_path / 'feedback.json'
  value = request('feedback-controller-transitions', 'feedback_transitions')
  write_json(request_path, value)
  result = run(request_path, tmp_path / 'feedback')
  assert result['status'] == 'completed_checks', result['findings']
  assert result['feedback_schedule'] == 'previous_apply_three_request_history'
  rows = read_json(tmp_path / 'feedback/trace.json')
  assert rows == read_json(tmp_path / 'feedback/reference-trace.json')
  requests, eligibility, previous_apply = [0.0] * 3, False, 0
  for row in rows:
    assert row['consumed_feedback_steer'] == -previous_apply / 600.0
    assert row['limiter_feedback_limited'] == eligibility
    eligibility = all(abs(request - row['consumed_feedback_steer']) > 1e-2 for request in requests)
    requests = (requests + [row['returned_steer']])[-3:]
    previous_apply = row['limited_counts']
    if row['active'] and row['limiter_feedback_limited']:
      assert row['integral_before_mps2'] == row['integral_after_mps2']
  assert any(row['active'] and row['limiter_feedback_limited'] for row in rows)
  assert any(row['active'] and not row['limiter_feedback_limited'] for row in rows)
  assert any(row['active'] and 485 < abs(row['inverse_command_counts']) < 570 for row in rows)
  uncoupled, _ = _run_controller(value['candidate_revision'], value['case']['limiter'], False, frames=_feedback_frames())
  assert any(a['integral_after_mps2'] != b['integral_after_mps2'] for a, b in zip(rows, uncoupled, strict=True))


@pytest.mark.parametrize('matched_request,expected_limited', [(0.1, False), (0.2, False), (0.3, False), (0.9, True)])
def test_prior_three_requests_control_next_cycle_eligibility(monkeypatch, matched_request, expected_limited):
  from . import scenarios

  factory = scenarios._controller

  def controller_factory(revision):
    controller = factory(revision)
    update = controller.update
    returned = iter([0.1, 0.2, 0.3, 0.4, 0.5])

    def scripted_update(*args):
      _, angle, diagnostic = update(*args)
      return next(returned), angle, diagnostic

    controller.update = scripted_update
    return controller

  applies = iter([0, 0, -600 * matched_request, 0, 0])
  monkeypatch.setattr(scenarios, '_controller', controller_factory)
  monkeypatch.setattr(scenarios, '_limiter', lambda revision: (lambda *args: next(applies), 'scripted-unit-fixture'))
  source = revision()
  rows, _ = scenarios._run_controller(source, source, False, feedback=True, frames=[('unit', True, 0.0, 0.0)] * 5)
  assert rows[3]['consumed_feedback_steer'] == matched_request
  assert rows[4]['limiter_feedback_limited'] == expected_limited


def test_reference_release_isolates_commitment_from_friction_and_integral(tmp_path):
  request_path = tmp_path / 'reference-release.json'
  write_json(request_path, request('reference-release', 'reference_release'))
  result = run(request_path, tmp_path / 'reference-release')
  assert result['status'] == 'completed_checks', result['findings']
  assert result['feedback_schedule'] == 'forced_integral_freeze'
  assert 'forced_integral_freeze' in result['scope']
  rows = read_json(tmp_path / 'reference-release/trace.json')
  disabled = read_json(tmp_path / 'reference-release/commitment-disabled-trace.json')
  assert all(row['integral_after_mps2'] == 0 and row['compensation_counts'] == 0 for row in rows + disabled if row['active'])
  assert all(row['active'] == other['active'] and row['desired_lateral_accel'] == other['desired_lateral_accel']
             and row['measured_lateral_accel'] == other['measured_lateral_accel'] for row, other in zip(rows, disabled, strict=True))
  assert all(row['filtered_request_mps2'] == other['filtered_request_mps2'] for row, other in zip(rows, disabled, strict=True))
  assert any(row['effective_feedforward_mps2'] > other['effective_feedforward_mps2'] + .01
             for row, other in zip(rows, disabled, strict=True) if row['phase'] == 'unwind')
  settled = [(row, other) for row, other in zip(rows, disabled, strict=True) if row['phase'] == 'tightening'][-30:]
  assert all(abs(row['effective_feedforward_mps2'] - other['effective_feedforward_mps2']) < .001 for row, other in settled)
  metrics = result['reference_probe']['metrics']
  for phase in ('unwind', 'recovery'):
    committed, off = metrics['candidate'][phase], metrics['candidate_commitment_disabled'][phase]
    assert committed['integrated_old_sign_inverse_count_seconds'] > off['integrated_old_sign_inverse_count_seconds']
    assert committed['peak_old_sign_inverse_counts'] >= off['peak_old_sign_inverse_counts']
    assert off['effective_feedforward_mps2']['first_at_or_below_threshold_seconds'] <= committed['effective_feedforward_mps2']['first_at_or_below_threshold_seconds']
    for key in ('effective_feedforward_mps2', 'inverse_command_counts'):
      zero_cross = committed[key]['first_zero_crossing_seconds']
      if zero_cross is not None:
        assert off[key]['first_zero_crossing_seconds'] is not None and off[key]['first_zero_crossing_seconds'] <= zero_cross
