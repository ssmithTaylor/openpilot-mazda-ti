# ruff: noqa: TID251
"""Serialized evidence tests: identity substitution and unhealthy data must not pass."""

import math
import pytest
from cereal import log
from .audit_measurement import audit


def fixture():
  cs = log.Event.new_message(logMonoTime=1_090_000_000, valid=True)
  cs.init('carState').vEgo = 20
  cs.carState.canValid = True
  llk = log.Event.new_message(logMonoTime=1_080_000_000, valid=True)
  loc = llk.init('liveLocationKalman')
  loc.angularVelocityCalibrated.value = [0, 0, .1]
  loc.angularVelocityCalibrated.std = [0, 0, .01]
  loc.angularVelocityCalibrated.valid = True
  loc.inputsOK = loc.sensorsOK = loc.posenetOK = True
  controller = log.Event.new_message(logMonoTime=1_100_000_000, valid=True)
  state = controller.init('controlsState').lateralControlState.init('torqueState')
  state.active = True
  d = state.mazdaDiagnostics
  d.version, d.measurement, d.rawRequest = 1, 2.25, 1.5
  d.inputs = [{'service': e.which(), 'logMonoTime': e.logMonoTime, 'seen': True,
               'valid': True, 'checksPassed': True} for e in (cs, llk)]
  return cs, llk, controller


def run(events, sample_count=12):
  data = []
  for event in events:
    event.clear_write_flag()
    data.append(event.to_bytes())
  return audit(log.Event.read_multiple_bytes(b''.join(data)), 1_095_000_000, 1_200_000_000, sample_count)


def test_exact_consumed_identity_not_nearest_and_order_independent():
  cs, llk, control = fixture()
  newer = llk.as_reader().as_builder()
  newer.logMonoTime = 1_099_000_000
  newer.liveLocationKalman.angularVelocityCalibrated.value = [0, 0, 1]
  result = run([control, newer, cs, llk])
  assert result['all_active_measurements_healthy']
  sample = result['samples'][0]
  assert sample['yaw_accel_mps2'] == pytest.approx(2)
  assert sample['angle_minus_yaw_mps2'] == pytest.approx(.25)
  assert sample['yaw_only_std_mps2'] == pytest.approx(.2)
  assert sample['cs_publication_age_ms'] == 10
  assert sample['llk_publication_age_ms'] == 20
  assert sample['cs_minus_llk_publication_ms'] == 10
  assert result == run([cs, llk, newer, control])


@pytest.mark.parametrize('fault', ['missing', 'unseen', 'future', 'duplicate_snapshot', 'validity', 'version', 'nan', 'negative_std', 'vector'])
def test_corrupt_evidence_fails(fault):
  cs, llk, control = fixture()
  d = control.controlsState.lateralControlState.torqueState.mazdaDiagnostics
  if fault == 'missing':
    d.inputs[1].logMonoTime -= 1
  elif fault == 'unseen':
    d.inputs[1].seen = False
  elif fault == 'future':
    d.inputs[1].logMonoTime = 1_110_000_000
  elif fault == 'duplicate_snapshot':
    d.inputs = [s.to_dict() for s in d.inputs] + [d.inputs[1].to_dict()]
  elif fault == 'validity':
    llk.valid = False
  elif fault == 'version':
    d.version = 0
  elif fault == 'nan':
    d.measurement = math.nan
  elif fault == 'negative_std':
    llk.liveLocationKalman.angularVelocityCalibrated.std = [0, 0, -.01]
  elif fault == 'vector':
    llk.liveLocationKalman.angularVelocityCalibrated.value = [0, 0]
  result = run([cs, llk, control])
  assert not result['identity_numeric_coverage_complete']
  assert not result['all_active_measurements_healthy']
  assert result['issue_counts']


@pytest.mark.parametrize('fault', ['yaw', 'sensors', 'checks', 'can', 'controller'])
def test_unhealthy_is_not_missing_but_excluded_from_metrics(fault):
  cs, llk, control = fixture()
  if fault == 'yaw':
    llk.liveLocationKalman.angularVelocityCalibrated.valid = False
  elif fault == 'sensors':
    llk.liveLocationKalman.sensorsOK = False
  elif fault == 'checks':
    control.controlsState.lateralControlState.torqueState.mazdaDiagnostics.inputs[1].checksPassed = False
  elif fault == 'can':
    cs.carState.canValid = False
  elif fault == 'controller':
    control.valid = False
  result = run([cs, llk, control])
  assert result['identity_numeric_coverage_complete']
  assert not result['all_active_measurements_healthy']
  assert result['healthy_rows'] == 0
  assert result['healthy_metrics']['angle_minus_yaw_mps2'] is None


def test_inactive_default_zero_is_not_measurement_and_empty_fails():
  cs, llk, control = fixture()
  control.controlsState.lateralControlState.torqueState.active = False
  result = run([cs, llk, control])
  assert result['joined_active_rows'] == 0
  assert not result['all_active_measurements_healthy']
  assert not run([])['identity_numeric_coverage_complete']


def test_repeated_localizer_counts_and_bounded_samples():
  cs, llk, control = fixture()
  second = control.as_reader().as_builder()
  second.logMonoTime += 10_000_000
  result = run([cs, llk, control, second])
  assert result['unique_consumed_llk'] == 1
  assert result['healthy_rows'] == 2
  assert len(result['samples']) == 2
  assert len(run([cs, llk, control, second], 1)['samples']) == 1
  assert run([cs, llk, control, second], 0)['samples'] == []
  assert result['max_controller_publication_gap_ms'] == 10


def test_duplicate_input_rejected():
  cs, llk, control = fixture()
  with pytest.raises(ValueError, match='duplicate input'):
    run([cs, cs, llk, control])
