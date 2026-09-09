"""Contract tests for the isolated Mazda process-startup evidence boundary."""

import pytest

from .startup_eval import controls_constructor_boundary, qualify_nested_timestamps, run


def test_nested_diagnostic_references_must_survive_one_outer_timestamp_transform():
  result = qualify_nested_timestamps(
    [{'id': 'a', 'outer_ns': 10, 'nested_ns': 100}, {'id': 'b', 'outer_ns': 20, 'nested_ns': 200}],
    [{'id': 'a', 'outer_ns': 60, 'nested_ns': 100}, {'id': 'b', 'outer_ns': 70, 'nested_ns': 200}],
  )
  assert result == {'status': 'qualified', 'outer_offset_ns': 50, 'records': 2}
  with pytest.raises(ValueError, match='nested'):
    qualify_nested_timestamps([{'id': 'a', 'outer_ns': 10, 'nested_ns': 100}], [{'id': 'a', 'outer_ns': 60, 'nested_ns': 101}])


def test_actual_controlsd_constructor_keeps_carstate_out_of_submaster():
  assert isinstance(controls_constructor_boundary(), __import__('ast').Assign)


def test_required_full_process_profile_is_explicitly_unsupported_when_runtime_is_missing(tmp_path):
  capability = lambda: {'full_process_supported': False, 'schema_boundary_supported': False, 'missing': ['linux_openpilot_runtime']}
  result = run(tmp_path / 'evidence', capability=capability)
  assert result['status'] == 'unsupported'
  assert result['profile'] == 'full_process'
  assert result['missing_capabilities'] == ['linux_openpilot_runtime']
  assert 'full-process integration is not satisfied' in result['scope']


def test_supported_boundary_records_isolation_schema_source_timing_and_cold_inactive_checks(tmp_path):
  result = run(
    tmp_path / 'evidence', profile='schema_boundary',
    capability=lambda: {'full_process_supported': False, 'schema_boundary_supported': True, 'missing': []},
    boundary=lambda: {'schema': 'carState', 'cold_start': 'passed', 'inactive': 'passed'},
  )
  assert result['status'] == 'completed_checks'
  assert result['isolation']['params'] == 'PARAMS_ROOT and OpenpilotPrefix temporary directories'
  assert result['isolation']['vehicle_connection'] == 'none'
  assert result['isolation']['can_publisher'] == 'not constructed'
  assert result['timing_scope'] == 'local process replay elapsed time; not full-system or device scheduling proof'
  assert result['exception'] is None


def test_supported_full_process_records_actual_generated_timestamp_qualification(tmp_path):
  checked = {
    'cold_start': 'passed',
    'inactive': {'status': 'passed', 'frames': 20},
    'timestamp_transform': {
      'status': 'qualified', 'outer_offset_ns': 4_000_000, 'records': 20,
      'identity_source': 'generated controlsState torqueState.mazdaDiagnostics.inputs',
    },
  }
  result = run(
    tmp_path / 'evidence', rlog=tmp_path / 'unused',
    capability=lambda: {'full_process_supported': True, 'schema_boundary_supported': True, 'missing': []},
    boundary=lambda: checked,
  )
  assert result['status'] == 'completed_checks'
  assert result['profile'] == 'full_process'
  assert result['timestamp_transform'] == checked['timestamp_transform']
  assert 'controlsd process replay' in result['scope']


def test_actual_messaging_schema_boundary_runs_when_the_declared_runtime_is_available(tmp_path):
  result = run(tmp_path / 'evidence', profile='schema_boundary')
  if result['status'] == 'unsupported':
    pytest.skip(', '.join(result['missing_capabilities']))
  assert result['boundary']['schema'] == 'carState'
