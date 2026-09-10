# ruff: noqa: TID251
"""Actual limiter and serialized production-schema activation/identity regression cases."""

import pytest

from cereal import log
from .feedback import pure_limiter
from .recorded_feedback import RecordedTiFeedback, serialized_steer


def fixture_events():
  events = []
  for sequence, controller, command, applied, publication, previous, limited in [
    (9, 1060, 1070, 1090, 1110, 0, 0),
    (10, 1080, 1130, 1140, 1150, 0, 10),
    (11, 1160, 1170, 1180, 1190, 10, 20),
    (12, 1200, 1210, 1220, 1230, 20, 30),
  ]:
    cc = log.Event.new_message(logMonoTime=command, valid=True)
    cc.init('carControl')
    cc.carControl.controlsStateMonoTime = controller
    cc.carControl.latActive = sequence != 9
    cc.carControl.actuators.steer = 0.5 if sequence != 9 else 0
    co = log.Event.new_message(logMonoTime=publication, valid=True)
    out = co.init('carOutput')
    out.applySequence, out.appliedAtMonoTime, out.appliedCarControlMonoTime = sequence, applied, command
    out.actuatorsOutput.steer = limited / 600
    d = out.mazdaDiagnostics
    d.version, d.latActive, d.tiAllowed = 1, sequence != 9, True
    d.tiMax, d.tiDeltaUp, d.tiDeltaDown = 600, 10, 15
    d.tiDriverAllowance, d.tiDriverMultiplier = 15, 40
    d.tiDeltaUpKnee, d.tiDeltaUpHigh = 630, 9
    d.tiRequested, d.tiPrevious, d.tiLimited = 300 if sequence != 9 else 0, previous, limited
    events.extend([cc, co])
  return events


def make(events=None):
  events = fixture_events() if events is None else events
  data = []
  for e in events:
    e.clear_write_flag()
    data.append(e.to_bytes())
  decoded = list(log.Event.read_multiple_bytes(b''.join(data)))
  commands = {int(e.logMonoTime): e for e in decoded if e.which() == 'carControl'}
  outputs = [(int(e.logMonoTime), e) for e in decoded if e.which() == 'carOutput']
  limiter, _ = pure_limiter('HEAD')
  return RecordedTiFeedback(outputs, commands, 1100, 1300, limiter)


def test_previous_apply_boundary_and_consumed_identity_preserve_original_history():
  replay = make()
  replay.publish(1080, -1.0)  # Pre-activation replay output must not replace the original request.
  replay.publish(1160, 0.5)
  replay.publish(1200, 0.5)
  assert replay.output_for(1110) == 0  # Published AFTER activation, applied BEFORE it.
  assert replay.output_for(1150) == serialized_steer(10/600)
  assert replay.output_for(1230) == serialized_steer(30/600)
  assert replay.output_for(1190) == serialized_steer(20/600)  # Exact older consumed identity, not latest.
  assert replay.history_request(1090, -1, 0.5) == 0.5
  assert replay.history_request(1100, -1, 0.5) == -1
  assert all(row['recorded'] == row['replay'] for row in replay.finish())


def test_changed_request_changes_own_subsequent_feedback_and_repeated_apply_runs_once():
  events = fixture_events()
  repeat = log.Event.new_message(logMonoTime=1260, valid=False)
  repeat.init('carOutput')
  repeat.carOutput = events[-1].carOutput
  replay = make(events + [repeat])
  replay.publish(1160, -0.5)
  replay.publish(1200, 0.5)
  assert replay.output_for(1190) == serialized_steer(-5/600)
  assert replay.output_for(1230) == serialized_steer(10/600)
  assert replay.output_for(1260) == serialized_steer(10/600)
  assert [r['replay'] for r in replay.finish()] == [10, -5, 10]


@pytest.mark.parametrize(('fault', 'message'), [
  ('command', 'Missing or inconsistent applied'), ('sequence', 'Missing apply sequence'),
  ('previous', 'previous TI command'), ('request', 'Recorded TI request'),
  ('limited', 'does not reproduce'), ('feedback', 'does not represent'),
  ('unavailable', 'stock fallback'), ('nan', 'Non-finite driver'), ('limit', 'TI600'),
  ('time', 'apply diagnostics/identity'),
])
def test_corrupt_recording_fails_before_candidate_execution(fault, message):
  events = fixture_events()
  co = events[5].carOutput
  d = co.mazdaDiagnostics
  if fault == 'command':
    co.appliedCarControlMonoTime = 1169
  elif fault == 'sequence':
    events = events[:4] + events[6:]
  elif fault == 'previous':
    d.tiPrevious = 9
  elif fault == 'request':
    d.tiRequested = 301
  elif fault == 'limited':
    d.tiLimited = 21
    co.actuatorsOutput.steer = 21/600
  elif fault == 'feedback':
    co.actuatorsOutput.steer = 0
  elif fault == 'unavailable':
    d.tiAllowed = False
  elif fault == 'nan':
    d.driverTorque = float('nan')
  elif fault == 'limit':
    d.tiMax = 650
  elif fault == 'time':
    co.appliedAtMonoTime = 1191
  with pytest.raises(ValueError, match=message):
    make(events)


def test_missing_duplicate_and_nonfinite_candidate_requests_fail():
  replay = make()
  with pytest.raises(ValueError, match='Missing replay controller'):
    replay.output_for(1190)
  with pytest.raises(ValueError, match='Missing consumed'):
    replay.output_for(1191)
  for value in [float('nan'), float('inf'), 1.01]:
    with pytest.raises(ValueError, match='finite and within'):
      replay.publish(1160, value)
  replay.publish(1160, 0.5)
  with pytest.raises(ValueError, match='Duplicate'):
    replay.publish(1160, 0.5)


def test_repeated_payload_and_duplicate_publication_are_rejected():
  events = fixture_events()
  with pytest.raises(ValueError, match='Duplicate or inconsistent output'):
    make(events + [events[-1]])
  repeat = log.Event.new_message(logMonoTime=1260, valid=True)
  repeat.init('carOutput')
  repeat.carOutput = events[-1].carOutput
  repeat.carOutput.mazdaDiagnostics.driverTorque = 1
  with pytest.raises(ValueError, match='Repeated apply changed'):
    make(events + [repeat])


def test_previous_applied_feedback_is_owned_prior_and_never_future():
  replay = make()
  replay.publish(1160, .5)
  warmup = replay.previous_applied_for_update(1160, 1150)
  assert warmup.observed_steer == serialized_steer(10 / 600)
  assert not warmup.candidate_available

  previous = replay.previous_applied_for_update(1200, 1190, limiter_frozen=True)
  assert previous.candidate_available
  assert previous.candidate.source_controller_mono == 1160
  assert previous.candidate.source_request_mono == 1170
  assert previous.candidate.applied_mono == 1180
  assert previous.candidate.publication_mono == 1190
  assert previous.candidate.selected_counts == 20
  assert previous.candidate.limiter_frozen
  with pytest.raises(ValueError, match='future'):
    replay.previous_applied_for_update(1190, 1190)


def test_equivalent_candidate_feedback_matches_recorded_baseline_exactly():
  replay = make()
  replay.publish(1160, .5)
  previous = replay.previous_applied_for_update(1200, 1190)
  assert previous.candidate.ti_counts == 20
  assert previous.candidate.normalized_steer == serialized_steer(20 / 600)
