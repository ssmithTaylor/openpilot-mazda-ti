"""Contract tests for the isolated Mazda process-startup evidence boundary."""

import json
import os
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from .provenance import ROOT
from .startup_eval import construct_controlsd_subscriptions, qualify_nested_timestamps, run, transition_stderr_diagnostics


def test_nested_diagnostic_references_must_survive_one_outer_timestamp_transform():
  result = qualify_nested_timestamps(
    [{'id': 'a', 'outer_ns': 10, 'nested_ns': 100}, {'id': 'b', 'outer_ns': 20, 'nested_ns': 200}],
    [{'id': 'a', 'outer_ns': 60, 'nested_ns': 100}, {'id': 'b', 'outer_ns': 70, 'nested_ns': 200}],
  )
  assert result == {'status': 'qualified', 'outer_offset_ns': 50, 'records': 2}
  with pytest.raises(ValueError, match='nested'):
    qualify_nested_timestamps([{'id': 'a', 'outer_ns': 10, 'nested_ns': 100}], [{'id': 'a', 'outer_ns': 60, 'nested_ns': 101}])


def test_transition_profile_accepts_only_the_declared_fake_service_harness_exclusions():
  first = {'event': 'controlsd.initialized', 'error': True, 'invalid': [], 'not_freq_ok': [],
           'not_alive': ['liveDelay', 'testJoystick', 'frogpilotCarState', 'frogpilotPlan']}
  second = {**first, 'event': 'commIssue'}
  payload = '\n'.join(json.dumps(row) for row in (first, second))
  assert transition_stderr_diagnostics(payload) == [first, second]
  extra = {**second, 'not_alive': [*second['not_alive'], 'unknownService']}
  assert transition_stderr_diagnostics('\n'.join(json.dumps(row) for row in (first, extra))) is None
  assert transition_stderr_diagnostics(payload + '\nnot-json') is None


class FakeMessaging:
  @staticmethod
  def sub_sock(service, **kwargs):
    return NS(service=service, kwargs=kwargs)

  @staticmethod
  def SubMaster(services, **kwargs):
    return NS(services=services, kwargs=kwargs)


def test_actual_controlsd_constructor_subscription_path_accepts_dedicated_carstate():
  host = construct_controlsd_subscriptions(messaging_module=FakeMessaging)
  assert host.car_state_sock.service == 'carState'
  assert 'carState' not in host.sm.services


def test_actual_constructor_path_rejects_known_carstate_in_submaster_failure():
  source = (ROOT / 'selfdrive/controls/controlsd.py').read_text(encoding='utf-8')
  broken = source.replace("self.sm = messaging.SubMaster(['deviceState'", "self.sm = messaging.SubMaster(['carState', 'deviceState'", 1)
  with pytest.raises(ValueError, match='dedicated subscriber'):
    construct_controlsd_subscriptions(broken, FakeMessaging)


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
  assert result['isolation']['params'] == 'tool-owned temporary directory'
  assert result['isolation']['vehicle_connection'] == 'none'
  assert result['isolation']['can_publisher'] == 'not constructed'
  assert result['timing_scope'] == 'local process replay elapsed time; not full-system or device scheduling proof'
  assert result['exception'] is None


def test_run_owns_temporary_state_and_restores_ambient_environment(tmp_path, monkeypatch):
  ambient_params = tmp_path / 'ambient-params'
  ambient_params.mkdir()
  marker = ambient_params / 'untouched'
  marker.write_text('ambient', encoding='utf-8')
  monkeypatch.setenv('PARAMS_ROOT', str(ambient_params))
  monkeypatch.setenv('OPENPILOT_PREFIX', 'ambient-prefix')
  observed = {}

  def boundary():
    observed['params_root'] = os.environ['PARAMS_ROOT']
    observed['prefix'] = os.environ['OPENPILOT_PREFIX']
    Path(observed['params_root'], 'owned-write').write_text('isolated', encoding='utf-8')
    return {'schema': 'carState', 'cold_start': 'passed', 'inactive': 'passed'}

  result = run(
    tmp_path / 'evidence', profile='schema_boundary', boundary=boundary,
    capability=lambda: {'full_process_supported': False, 'schema_boundary_supported': True, 'missing': []},
  )
  assert result['status'] == 'completed_checks'
  assert observed['params_root'] != str(ambient_params)
  assert observed['prefix'] != 'ambient-prefix'
  assert not Path(observed['params_root']).exists()
  assert os.environ['PARAMS_ROOT'] == str(ambient_params)
  assert os.environ['OPENPILOT_PREFIX'] == 'ambient-prefix'
  assert marker.read_text(encoding='utf-8') == 'ambient'


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
