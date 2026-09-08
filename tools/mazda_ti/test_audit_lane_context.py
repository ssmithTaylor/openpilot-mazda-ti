# ruff: noqa: TID251
"""Serialized identity/clock failures at the lane-observer audit boundary."""

import pytest

from cereal import log
from .audit_lane_context import audit


def model_event(mono=1_020_000_000, eof=1_000_000_000):
  event = log.Event.new_message(logMonoTime=mono, valid=True)
  model = event.init('modelV2')
  model.timestampEof = eof
  model.meta.laneChangeState = 'off'
  model.laneLineProbs = [0.0, 0.9, 0.9, 0.0]
  lanes = model.init('laneLines', 4)
  for lane, y in zip(lanes, [-6, -2, 2, 6], strict=True):
    lane.x, lane.y = [0, 5, 10, 15, 20], [y] * 5
  return event


def controller_event(mono=1_100_000_000, now=1_080_000_000, camera_now=1_080_000_000, age=0.08):
  event = log.Event.new_message(logMonoTime=mono, valid=True)
  torque = event.init('controlsState').lateralControlState.init('torqueState')
  torque.active = True
  d = torque.mazdaDiagnostics
  d.version, d.modelContextNow, d.cameraContextNow = 1, now, camera_now
  d.laneValid, d.laneReason, d.cameraAge = True, 'valid', age
  d.inputs = [{'service': 'modelV2', 'logMonoTime': 1_020_000_000, 'valid': True,
               'alive': True, 'frequencyOk': True, 'checksPassed': True, 'updated': True, 'seen': True}]
  return event


def serialized(events):
  values = []
  for event in events:
    event.clear_write_flag()
    values.append(event.to_bytes())
  return log.Event.read_multiple_bytes(b''.join(values))


def run(events):
  return audit(serialized(events), 1_090_000_000, 1_300_000_000)


def test_exact_identity_repeated_model_rechecks_age_in_each_clock_domain():
  model = model_event(eof=101_000_000_000)
  first = controller_event(camera_now=101_080_000_000)
  second = controller_event(mono=1_120_000_000, now=1_100_000_000, camera_now=101_100_000_000, age=0.10)
  newer_unconsumed = model_event(mono=1_070_000_000, eof=101_050_000_000)
  newer_unconsumed.modelV2.laneLineProbs = [0, 0, 0, 0]
  result = run([second, newer_unconsumed, model, first])
  assert result['active_context_reproduction_complete']
  assert [r['camera_age'] for r in result['rows']] == [0.08, 0.10]
  assert [r['modelV2'] for r in result['rows']] == [1_020_000_000] * 2
  assert result['maximum_absolute_errors'] == dict.fromkeys(result['maximum_absolute_errors'], 0.0)


def test_low_confidence_is_a_reproduced_decision_not_missing_evidence():
  model, controller = model_event(), controller_event()
  model.modelV2.laneLineProbs = [0, 0.9, 0.6, 0]
  d = controller.controlsState.lateralControlState.torqueState.mazdaDiagnostics
  d.laneValid, d.laneReason = False, 'confidence'
  result = run([model, controller])
  assert result['context_reproduction_complete']
  assert result['summary']['reproduced_reasons'] == {'confidence': 1}
  assert result['rows'][0]['lane_offset'] is None


def test_health_failure_and_camera_expiry_are_distinct():
  model, first = model_event(), controller_event()
  d = first.controlsState.lateralControlState.torqueState.mazdaDiagnostics
  d.inputs[0].checksPassed = False
  d.laneValid, d.laneReason, d.cameraAge = False, 'invalid_event', 0.0
  expired = controller_event(mono=1_250_000_000, now=1_230_000_000, camera_now=1_230_000_000, age=0.23)
  d = expired.controlsState.lateralControlState.torqueState.mazdaDiagnostics
  d.laneValid, d.laneReason = False, 'camera_age'
  result = run([model, first, expired])
  assert result['context_reproduction_complete']
  assert result['summary']['reproduced_reasons'] == {'camera_age': 1, 'invalid_event': 1}


@pytest.mark.parametrize(('fault', 'reason'), [
  ('missing', 'missing_consumed_model'), ('unseen', 'unseen_or_invalid_model_identity'),
  ('validity', 'model_validity_mismatch'), ('clocks', 'invalid_context_clocks'),
  ('version', 'absent_or_unsupported_diagnostics'), ('reason', 'lane_decision_mismatch'),
  ('offset', 'lane_value_mismatch'),
])
def test_rejects_incomplete_or_changed_evidence(fault, reason):
  model, controller = model_event(), controller_event()
  d = controller.controlsState.lateralControlState.torqueState.mazdaDiagnostics
  if fault == 'missing':
    d.inputs[0].logMonoTime -= 1
  elif fault == 'unseen':
    d.inputs[0].seen = False
  elif fault == 'validity':
    model.valid = False
  elif fault == 'clocks':
    d.modelContextNow = 0
  elif fault == 'version':
    d.version = 0
  elif fault == 'reason':
    d.laneReason = 'confidence'
  elif fault == 'offset':
    d.laneOffset = 0.1
  result = run([model, controller])
  assert not result['context_reproduction_complete']
  assert result['summary']['issue_counts'] == {reason: 1}


def test_empty_inactive_duplicate_and_invalid_window():
  assert not run([])['context_reproduction_complete']
  model, controller = model_event(), controller_event()
  controller.controlsState.lateralControlState.torqueState.active = False
  result = run([model, controller])
  assert result['context_reproduction_complete'] and not result['active_context_reproduction_complete']
  with pytest.raises(ValueError, match='duplicate model'):
    run([model, model, controller])
  with pytest.raises(ValueError, match='Duplicate controller'):
    run([model, controller, controller])
  with pytest.raises(ValueError, match='Require'):
    audit([], 0, 1)
