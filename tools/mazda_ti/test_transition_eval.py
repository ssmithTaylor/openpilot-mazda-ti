"""Tests for declared, serialized control-transition evidence."""

from types import SimpleNamespace as NS

from .startup_eval import StartupUnsupported
from .transition_eval import actual_process_transition, assess, main, normalize_observations, opaque_state_id, run


def observation(mono, active, ti, health='valid', state='state-a', before=None, state_id=None, before_id=None):
  before = state if before is None else before
  state_id = f'opaque:{state}' if state_id is None else state_id
  before_id = f'opaque:{before}' if before_id is None else before_id
  return {
    'mono_time_ns': mono, 'active': active, 'ti_allowed': ti,
    'health': health, 'diagnostic_references': [{'service': 'carState', 'log_mono_time': mono - 1}],
    'state_before': before, 'state_after': state, 'state_before_id': before_id, 'state_id': state_id,
  }


def complete_rows():
  return [
    observation(10, False, True, state='cold'),
    observation(20, True, True, state='active-a'),
    observation(30, True, False, state='active-a'),
    observation(40, False, False, health='missing', state='inactive-reset'),
    observation(50, False, False, health='stale', state='inactive-reset'),
    observation(60, False, False, health='invalid', state='inactive-reset'),
    observation(70, True, True, state='active-b', before='active-a'),
  ]


def test_declared_transition_fixture_requires_active_disengage_reentry_and_ti_reentry():
  result = assess(complete_rows())
  assert result['status'] == 'completed_checks'
  assert result['availability'] == {
    'active_operation': 2, 'disengage_reengage': 1, 'ti_bypass_reentry': 1,
    'missing_rejected_inactive': 1, 'stale_rejected_inactive': 1, 'invalid_rejected_inactive': 1,
  }
  assert result['state_retention']['active_to_ti_bypass'] == 'active-a'
  assert result['state_retention']['active_to_ti_bypass_id'] == 'opaque:active-a'


def test_unhealthy_input_that_remains_active_is_a_failed_check_not_a_hidden_completion():
  rows = complete_rows()
  rows[3]['active'] = True
  result = assess(rows)
  assert result['status'] == 'failed_check'
  assert 'missing message remained active at 40' in result['findings']
  assert 'missing required fault handling: missing' in result['findings']


def test_ti_unavailability_can_safely_deactivate_and_still_require_a_real_reentry():
  rows = [
    observation(10, False, True), observation(20, True, True, state='engaged'),
    observation(30, False, False, state='bypass'), observation(40, True, True, state='reentered', before='engaged'),
    observation(50, False, True, health='missing'), observation(60, False, True, health='stale'),
    observation(70, False, True, health='invalid'),
  ]
  result = assess(rows)
  assert result['status'] == 'completed_checks'
  assert result['availability']['ti_bypass_reentry'] == 1
  assert result['state_retention']['ti_bypass_disengage_state'] == 'engaged'
  assert result['state_retention']['ti_reentry_state'] == 'engaged'


def test_missing_required_transition_is_reported_in_the_common_failure_status():
  result = assess(complete_rows()[:2])
  assert result['status'] == 'failed_check'
  assert 'missing required transition: ti_bypass_reentry' in result['findings']
  assert 'missing required fault handling: missing' in result['findings']


def test_observation_identities_are_strictly_monotonic_and_do_not_mask_state_reset():
  rows = complete_rows()
  rows[2]['mono_time_ns'] = 20
  result = assess(rows)
  assert result['status'] == 'failed_check'
  assert result['findings'] == ['non-monotonic observation identity at 20']
  rows = complete_rows()
  rows[2]['state_before'] = 'reset'
  result = assess(rows)
  assert result['status'] == 'failed_check'
  assert result['findings'] == ['state changed while active TI bypass at 30']


def test_disengage_and_ti_loss_must_retain_both_state_and_opaque_identity_at_reentry():
  rows = complete_rows()
  rows[-1]['state_before'] = 'RESET'
  rows[-1]['state_before_id'] = 'opaque:RESET'
  result = assess(rows)
  assert result['status'] == 'failed_check'
  assert 'state changed across disengage/reengage at 70' in result['findings']
  assert 'state changed across TI-loss/reentry at 70' in result['findings']

  rows = complete_rows()
  rows[-1]['state_before_id'] = 'opaque:mutated'
  result = assess(rows)
  assert result['status'] == 'failed_check'
  assert 'opaque state_id changed across disengage/reengage at 70' in result['findings']
  assert 'opaque state_id changed across TI-loss/reentry at 70' in result['findings']


def test_assessment_rejects_rows_without_documented_opaque_state_identity():
  rows = complete_rows()
  del rows[1]['state_id']
  result = assess(rows)
  assert result['status'] == 'failed_check'
  assert result['findings'] == ['missing opaque state_id at 20']


def test_normalization_derives_health_and_exact_diagnostic_references_from_serialized_events():
  event = lambda mono, service, value=None: NS(logMonoTime=mono, which=lambda: service, **({service: value} if value else {}))
  snapshot = NS(service='carState', logMonoTime=9, seen=True, valid=True, alive=True, frequencyOk=True, checksPassed=True)
  diagnostics = NS(version=1, inputs=[snapshot], integralBefore=.5, integralAfter=1.0)
  torque = NS(active=True, mazdaDiagnostics=diagnostics)
  controls = event(10, 'controlsState', NS(lateralControlState=NS(torqueState=torque)))
  command = event(11, 'carControl', NS(controlsStateMonoTime=10))
  output_diag = NS(version=1, tiAllowed=True)
  output = event(12, 'carOutput', NS(appliedCarControlMonoTime=11, mazdaDiagnostics=output_diag))
  rows, findings = normalize_observations([event(9, 'carState'), controls, command, output])
  assert findings == []
  assert rows == [{**observation(10, True, True, state='1.0', before='0.5',
                                 state_id=opaque_state_id(1.0), before_id=opaque_state_id(.5)),
                   'ti_availability_source': 'carOutput_observation'}]


def test_dedicated_carstate_uses_checks_passed_while_submaster_services_require_frequency_health():
  event = lambda mono, service, value=None: NS(logMonoTime=mono, which=lambda: service, **({service: value} if value else {}))
  car_state = NS(service='carState', logMonoTime=9, seen=True, valid=True, alive=True, frequencyOk=False, checksPassed=True)
  frog_state = NS(service='frogpilotCarState', logMonoTime=8, seen=True, valid=True, alive=True, frequencyOk=True, checksPassed=True)
  diagnostics = NS(version=1, inputs=[car_state, frog_state], integralBefore=.5, integralAfter=1.0)
  controls = event(10, 'controlsState', NS(lateralControlState=NS(torqueState=NS(active=True, mazdaDiagnostics=diagnostics))))
  command = event(11, 'carControl', NS(controlsStateMonoTime=10))
  output = event(12, 'carOutput', NS(appliedCarControlMonoTime=11, mazdaDiagnostics=NS(version=1, tiAllowed=True)))
  rows, findings = normalize_observations([controls, command, output], source_events=[event(9, 'carState'), event(8, 'frogpilotCarState', NS(tiActive=True))])
  assert findings == []
  assert rows[0]['health'] == 'valid'
  frog_state.frequencyOk = False
  rows, _ = normalize_observations([controls, command, output], source_events=[event(9, 'carState'), event(8, 'frogpilotCarState', NS(tiActive=True))])
  assert rows[0]['health'] == 'stale'
  frog_state.frequencyOk, frog_state.alive, frog_state.checksPassed = True, False, False
  rows, _ = normalize_observations([controls, command, output], source_events=[event(9, 'carState'), event(8, 'frogpilotCarState', NS(tiActive=True))])
  assert rows[0]['health'] == 'stale'
  frog_state.alive = True
  rows, _ = normalize_observations([controls, command, output], source_events=[event(9, 'carState'), event(8, 'frogpilotCarState', NS(tiActive=True))])
  assert rows[0]['health'] == 'invalid'


def test_normalization_keeps_recorded_inputs_as_identity_sources_not_duplicate_transition_rows():
  event = lambda mono, service, value=None: NS(logMonoTime=mono, which=lambda: service, **({service: value} if value else {}))
  snapshot = NS(service='carState', logMonoTime=9, seen=True, valid=True, alive=True, frequencyOk=True, checksPassed=True)
  diagnostics = NS(version=1, inputs=[snapshot], integralBefore=1.0, integralAfter=2.0)
  original = event(10, 'controlsState', NS(lateralControlState=NS(torqueState=NS(active=False, mazdaDiagnostics=diagnostics))))
  replayed = event(14, 'controlsState', NS(lateralControlState=NS(torqueState=NS(active=True, mazdaDiagnostics=diagnostics))))
  command = event(15, 'carControl', NS(controlsStateMonoTime=14))
  output = event(16, 'carOutput', NS(appliedCarControlMonoTime=15, mazdaDiagnostics=NS(version=1, tiAllowed=True)))
  rows, findings = normalize_observations([output, replayed, command], source_events=[event(9, 'carState'), original])
  assert findings == []
  assert [row['mono_time_ns'] for row in rows] == [14]


def test_process_rows_bind_ti_availability_to_the_referenced_actual_subscriber_input_without_card():
  event = lambda mono, service, value=None: NS(logMonoTime=mono, which=lambda: service, **({service: value} if value else {}))
  frog = NS(service='frogpilotCarState', logMonoTime=9, seen=True, valid=True, alive=True, frequencyOk=True, checksPassed=True)
  diagnostics = NS(version=1, inputs=[frog], integralBefore=0.0, integralAfter=0.0)
  controls = event(10, 'controlsState', NS(lateralControlState=NS(torqueState=NS(active=True, mazdaDiagnostics=diagnostics))))
  command = event(11, 'carControl', NS(controlsStateMonoTime=10))
  rows, findings = normalize_observations([controls, command], source_events=[event(9, 'frogpilotCarState', NS(tiActive=True))])
  assert findings == []
  assert rows[0]['ti_allowed'] is True
  assert rows[0]['ti_availability_source'] == 'frogpilotCarState_input'


def test_run_preserves_isolation_and_keeps_elapsed_timing_outside_canonical_transition_result(tmp_path):
  result = run(tmp_path / 'evidence', observations=complete_rows())
  assert result['status'] == 'completed_checks'
  assert result['isolation']['vehicle_connection'] == 'none'
  assert result['timing_scope'] == 'transition result excludes elapsed workload timing; device timing requires a separate device profile'
  assert 'elapsed_seconds' not in result['transition']
  assert result['elapsed_seconds'] >= 0


def test_run_rejects_schema_harness_without_adjacent_segment_mode(tmp_path):
  result = run(tmp_path / 'evidence', rlog=['retained/rlog'], transition_harness=True,
               capability=lambda: {'full_process_supported': True, 'missing': []})
  assert result['status'] == 'failed_execution'
  assert result['exception'] == 'ValueError: --schema-input-harness requires --rlog and --all-segments'


def test_transition_boundary_rejects_harness_without_all_segments_before_opening_rlog():
  try:
    actual_process_transition('missing/rlog', transition_harness=True)
  except ValueError as error:
    assert str(error) == '--schema-input-harness requires --all-segments'
  else:
    raise AssertionError('process boundary accepted harness without --all-segments')


def test_cli_rejects_schema_harness_without_adjacent_segment_mode(tmp_path, capsys):
  try:
    main(['--rlog', 'retained/rlog', '--schema-input-harness', '--output', str(tmp_path / 'evidence')])
  except SystemExit as error:
    assert error.code == 2
  else:
    raise AssertionError('CLI accepted --schema-input-harness without --all-segments')
  assert '--schema-input-harness requires --rlog and --all-segments' in capsys.readouterr().err


def test_full_process_profile_only_uses_the_actual_boundary_observations(tmp_path):
  def boundary(rlog, maximum, all_segments, transition_harness):
    assert rlog == ['retained/rlog']
    assert maximum == 7
    assert all_segments is True
    assert transition_harness is False
    transition = assess(complete_rows())
    return {'startup_boundary': {'interface': 'process_replay', 'no_vehicle_output': {'status': 'passed'}}, **transition}

  result = run(tmp_path / 'evidence', rlog=['retained/rlog'], max_carstate_messages=7, all_segments=True, process_boundary=boundary,
               capability=lambda: {'full_process_supported': True, 'missing': []})
  assert result['status'] == 'completed_checks'
  assert result['profile'] == 'full_process_transition'
  assert result['transition']['process_boundary']['interface'] == 'process_replay'


def test_missing_real_process_capability_is_explicitly_unsupported(tmp_path):
  def unsupported(*_args):
    raise StartupUnsupported('msgq.ipc_pyx unavailable')

  result = run(tmp_path / 'evidence', rlog=['retained/rlog'], process_boundary=unsupported,
               capability=lambda: {'full_process_supported': True, 'missing': []})
  assert result['status'] == 'unsupported'
  assert result['transition'] is None
  assert 'msgq.ipc_pyx unavailable' in result['exception']


def test_declared_runtime_capability_prevents_a_full_profile_claim(tmp_path):
  result = run(tmp_path / 'evidence', rlog=['retained/rlog'],
               capability=lambda: {'full_process_supported': False, 'missing': ['linux_openpilot_runtime']})
  assert result['status'] == 'unsupported'
  assert result['missing_capabilities'] == ['linux_openpilot_runtime']
