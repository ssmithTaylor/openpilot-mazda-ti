# ruff: noqa: TID251
"""Serialized production-controller state and malformed evidence at the audit boundary."""
from types import SimpleNamespace as NS

import pytest

from .test_diagnostics import synthetic_controller, car, log, LaneContext
from .audit_friction_release import audit


def fixture(direction=1):
  ctrl = synthetic_controller()
  cs = car.CarState.new_message(vEgo=25.0, canValid=True)
  params = log.LiveParametersData.new_message()
  vm = NS(calc_curvature=lambda *_: -direction*3.0/625)
  fp = NS(lkasBlocked=False, lkasEffective=-direction*308.0, tiActive=True, columnTorque=160.0)
  toggles = NS(lat_commit_setpoint=True, lat_damping=True, lat_friction_comp=True,
               lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=600.0)
  ctrl._release_context = LaneContext(True, direction*.5, 0, 0, .1, 'valid', direction*.0032, direction*.0032)
  events = []
  for frame in range(520):
    mono = 1_000_000_000+frame*10_000_000
    active = not 480 <= frame < 500
    request = 3.0 if frame < 300 else (2.4 if frame < 410 else 3.4)
    cs.steeringRateDeg = direction*(3 if 390 <= frame < 410 else 0)
    ce = log.Event.new_message(logMonoTime=mono-1_000_000, valid=True)
    ce.carState = cs
    _, _, message = ctrl.update(active, cs, vm, params, False, direction*request/625, False, .48, None, None, toggles, fp)
    message.mazdaDiagnostics.inputs = [dict(service='carState', logMonoTime=ce.logMonoTime, valid=True,
                                          seen=True, updated=True, alive=True, checksPassed=True)]
    control = log.Event.new_message(logMonoTime=mono, valid=True)
    control.init('controlsState').lateralControlState.torqueState = message
    events += [ce, control]
  return events


def run(events):
  raw = []
  for e in events:
    e.clear_write_flag()
    raw.append(e.to_bytes())
  return audit(log.Event.read_multiple_bytes(b''.join(raw)), 1_000_000_000, 7_000_000_000)


@pytest.mark.parametrize('direction', [-1, 1])
def test_real_controller_state_reproduces_both_directions_and_reset(direction):
  events = fixture(direction)
  result = run(reversed(events))
  assert result['state_consistent']
  assert result['summary']['controller_publications'] == 520
  assert result['summary']['transitions_checked'] == 519
  assert result['summary']['withdrawal_positive'] > 0
  assert result['summary']['completed'] > 0
  assert all(r['withdrawal'] == 0 and not r['completed'] for r in result['rows'] if not r['active'])


@pytest.mark.parametrize('fault,reason', [
  ('version', 'absent_or_unsupported_state_extension'), ('compensation', 'compensation_decomposition_mismatch'),
  ('latch', 'release_transition_mismatch'), ('missing', 'missing_consumed_car_state'),
  ('invalid', 'car_state_validity_or_rate_mismatch'), ('nan', 'nonfinite_state_or_input'),
  ('reset', 'reset_state_mismatch'), ('future', 'invalid_car_state_snapshot'),
])
def test_rejects_missing_corrupted_or_misidentified_state(fault, reason):
  events = fixture()
  frame = 490 if fault == 'reset' else 350
  ce, control = events[frame*2:frame*2+2]
  d = control.controlsState.lateralControlState.torqueState.mazdaDiagnostics
  if fault == 'version':
    d.frictionReleaseVersion = 0
  elif fault == 'compensation':
    d.frictionCompensation += 1
  elif fault == 'latch':
    d.frictionReleaseCompleted = not d.frictionReleaseCompleted
  elif fault == 'missing':
    events.remove(ce)
  elif fault == 'invalid':
    ce.valid = False
  elif fault == 'nan':
    d.frictionGate = float('nan')
  elif fault == 'reset':
    d.frictionReleaseCompleted = True
  elif fault == 'future':
    d.inputs[0].logMonoTime = control.logMonoTime+1
  result = run(events)
  assert not result['state_consistent']
  assert reason in result['summary']['issue_counts']


def test_gap_is_not_silently_treated_as_one_update_and_duplicate_is_rejected():
  events = fixture()
  result = run(events[:600]+events[608:])
  assert not result['state_consistent']
  assert 'controller_publication_gap' in result['summary']['issue_counts']
  with pytest.raises(ValueError, match='Duplicate controller'):
    run(events+[events[1]])


def test_one_anchor_and_empty_input_cannot_qualify_transitions():
  assert not run([])['state_consistent']
  assert not run(fixture()[:2])['state_consistent']
