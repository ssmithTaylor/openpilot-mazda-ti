# ruff: noqa: TID251
"""RecordedTiFeedback on drives recorded WITH the stuck-request guard (selfdrive/car/mazda/ti_stuck_guard.py).

The recordings are produced frame by frame the way carcontroller.py produces them: pre = limiter(request, previous sent),
sent = guard.update(pre, latActive and tiAllowed, now=appliedAtMonoTime * 1e-9), tiLimited = sent, tiPrevious = the previous
sent, actuatorsOutput.steer = pre / 600. Corruptions replace the sent value on one frame (the chain stays self-consistent).
"""

import inspect

import pytest

from cereal import log
from openpilot.selfdrive.car.mazda.ti_stuck_guard import StuckGuard, STUCK_HOLD_FRAMES
from .feedback import pure_limiter
from .recorded_feedback import RecordedTiFeedback, serialized_steer

BASE, STEP = 10_000_000_000, 10_000_000
LIMITER, _ = pure_limiter('HEAD')


class _Limits:
  TI_STEER_MAX, TI_STEER_DELTA_UP, TI_STEER_DELTA_DOWN = 600., 10., 15.
  TI_STEER_DRIVER_ALLOWANCE, TI_STEER_DRIVER_MULTIPLIER, TI_STEER_DRIVER_FACTOR = 15., 40., 1
  TI_STEER_DELTA_UP_KNEE, TI_STEER_DELTA_UP_HIGH = 630., 9.


def record(frames, guard=True, sent_override=None, steer_override=None, spacing=None):
  """frames: list of (latActive, steer request). Returns (events, rows) with rows[i] = dict(pre, sent, t)."""
  sent_override, steer_override = sent_override or {}, steer_override or {}
  g, previous, events, rows, t = StuckGuard(), 0, [], [], BASE
  for i, (active, steer) in enumerate(frames):
    t += (spacing(i) if spacing else STEP) if i else 0
    request = serialized_steer(steer)
    requested = int(round(request * 600)) if active else 0
    pre = int(LIMITER(requested, previous, 0.0, _Limits)) if active else 0
    sent = g.update(pre, active, now=(t + 2_000_000) * 1e-9) if guard else pre
    sent = sent_override.get(i, sent)
    cc = log.Event.new_message(logMonoTime=t + 1_000_000, valid=True)
    cc.init('carControl')
    cc.carControl.controlsStateMonoTime = t
    cc.carControl.latActive = active
    cc.carControl.actuators.steer = request if active else 0
    co = log.Event.new_message(logMonoTime=t + 3_000_000, valid=True)
    out = co.init('carOutput')
    out.applySequence, out.appliedAtMonoTime, out.appliedCarControlMonoTime = i + 1, t + 2_000_000, t + 1_000_000
    out.actuatorsOutput.steer = steer_override.get(i, pre) / 600
    d = out.mazdaDiagnostics
    d.version, d.latActive, d.tiAllowed = 1, active, True
    d.tiMax, d.tiDeltaUp, d.tiDeltaDown = 600, 10, 15
    d.tiDriverAllowance, d.tiDriverMultiplier = 15, 40
    d.tiDeltaUpKnee, d.tiDeltaUpHigh = 630, 9
    d.tiRequested, d.tiPrevious, d.tiLimited = requested, previous, sent
    events.extend([cc, co])
    rows.append({'pre': pre, 'sent': sent, 't': t})
    previous = sent
  return events, rows


def make(events, rows, stuck_guard=True, first=1):
  data = []
  for e in events:
    e.clear_write_flag()
    data.append(e.to_bytes())
  decoded = list(log.Event.read_multiple_bytes(b''.join(data)))
  commands = {int(e.logMonoTime): e for e in decoded if e.which() == 'carControl'}
  outputs = [(int(e.logMonoTime), e) for e in decoded if e.which() == 'carOutput']
  activation, end = rows[first]['t'], rows[-1]['t'] + STEP
  return RecordedTiFeedback(outputs, commands, activation, end, LIMITER, stuck_guard=stuck_guard)


HOLD = [(False, 0.0)] + [(True, -0.95)] * 259          # -570 after a 57-frame ramp, held for two nudge bursts
SATURATED = [(False, 0.0)] + [(True, -1.0)] * 160      # -600: exempt, never nudged


def hold_start(rows, value):
  return next(i for i, r in enumerate(rows) if r['pre'] == value)


def test_generated_recording_has_the_guard_pattern():
  _, rows = record(HOLD)
  k0 = hold_start(rows, -570)
  nudges = [i for i, r in enumerate(rows) if r['sent'] != r['pre']]
  second = k0 + 2 * STUCK_HOLD_FRAMES + 2
  assert nudges == [k0 + STUCK_HOLD_FRAMES, k0 + STUCK_HOLD_FRAMES + 1, second, second + 1]
  assert all(rows[i]['sent'] == -568 and rows[i]['pre'] == -570 for i in nudges)
  assert rows[k0 + STUCK_HOLD_FRAMES + 2]['sent'] == -570   # back through the limiter


@pytest.mark.parametrize('mode', [True, 'auto'])
def test_nudged_recording_qualifies_and_replays_identically(mode):
  events, rows = record(HOLD)
  replay = make(events, rows, stuck_guard=mode)
  assert replay.stuck_guard_present
  for r in rows[1:]:
    replay.publish(r['t'], serialized_steer(-0.95))
  for i, r in enumerate(rows):
    assert replay.output_for(r['t'] + 3_000_000) == serialized_steer(r['pre'] / 600)   # steer is the PRE-nudge value
  assert all(row['recorded'] == row['replay'] for row in replay.finish())


def test_nudge_frame_feedback_snapshot_reports_pre_nudge_command():
  events, rows = record(HOLD)
  replay = make(events, rows)
  for r in rows[1:]:
    replay.publish(r['t'], serialized_steer(-0.95))
  k = hold_start(rows, -570) + STUCK_HOLD_FRAMES
  previous = replay.previous_applied_for_update(rows[k + 1]['t'], rows[k]['t'] + 3_000_000)
  assert previous.candidate.ti_counts == -570 and previous.observed_steer == serialized_steer(-570 / 600)
  assert replay.counts[k + 1] == -568                       # the sent value is the limiter's history


def test_replayed_controller_goes_through_the_guard():
  events, rows = record(HOLD)
  replay = make(events, rows)
  for r in rows[1:]:
    replay.publish(r['t'], serialized_steer(-0.9))           # a different held value: -540
  counts = [row['replay'] for row in replay.finish()]
  k0 = counts.index(-540)
  assert counts[k0 + STUCK_HOLD_FRAMES:k0 + STUCK_HOLD_FRAMES + 3] == [-538, -538, -540]
  assert counts[k0:k0 + STUCK_HOLD_FRAMES] == [-540] * STUCK_HOLD_FRAMES


def test_unnudged_recording_qualifies_in_every_mode():
  events, rows = record(SATURATED)
  for mode in (False, True, 'auto'):
    assert make(events, rows, stuck_guard=mode).applies


@pytest.mark.parametrize('fault', ['size3', 'away', 'early', 'late', 'third', 'at600', 'inactive'])
def test_illegal_nudge_is_rejected(fault):
  _, rows = record(HOLD)
  k = hold_start(rows, -570) + STUCK_HOLD_FRAMES
  frames, override = HOLD, {}
  if fault == 'size3':
    override = {k: -567}
  elif fault == 'away':
    override = {k: -572}
  elif fault == 'early':
    override = {k - 1: -568}
  elif fault == 'late':
    override = {k: -570}
  elif fault == 'third':
    override = {k + 2: -568}
  elif fault == 'at600':
    frames = SATURATED
    override = {hold_start(record(SATURATED)[1], -600) + STUCK_HOLD_FRAMES: -598}
  elif fault == 'inactive':
    frames = HOLD[:k] + [(False, 0.0)] + HOLD[k + 1:]
    override = {k: -568}
  events, rows = record(frames, sent_override=override)
  for mode in (True, 'auto'):
    with pytest.raises(ValueError, match='stuck guard does not reproduce'):
      make(events, rows, stuck_guard=mode)


def test_nudged_recording_still_rejected_without_the_guard_model():
  assert inspect.signature(RecordedTiFeedback).parameters['stuck_guard'].default is False
  events, rows = record(HOLD)
  with pytest.raises(ValueError, match='Recorded feedback does not represent the TI limited command'):
    make(events, rows, stuck_guard=False)                    # steer (pre) != tiLimited (sent): the pre-guard check
  _, rows = record(HOLD)
  events, rows = record(HOLD, steer_override={i: r['sent'] for i, r in enumerate(rows) if r['sent'] != r['pre']})
  with pytest.raises(ValueError, match='Production limiter does not reproduce recorded TI command'):
    make(events, rows, stuck_guard=False)


def test_steer_on_nudge_frame_must_be_the_pre_nudge_value():
  _, rows = record(HOLD)
  k = hold_start(rows, -570) + STUCK_HOLD_FRAMES
  events, rows = record(HOLD, steer_override={k: -568})
  with pytest.raises(ValueError, match='does not represent'):
    make(events, rows)


def test_pre_guard_recording_with_long_hold_qualifies_only_without_guard():
  events, rows = record(HOLD, guard=False)
  assert not make(events, rows, stuck_guard=False).stuck_guard_present
  auto = make(events, rows, stuck_guard='auto')
  assert not auto.stuck_guard_present                       # auto chose the exact-limiter model
  with pytest.raises(ValueError, match='stuck guard does not reproduce'):
    make(events, rows, stuck_guard=True)                     # the guard would have had to nudge


def test_time_based_hold_on_late_frames():
  slow = lambda i: 2 * STEP                                  # 20 ms frames: 0.8 s is reached at 40 frames, not 80
  events, rows = record(HOLD, spacing=slow)
  k0 = hold_start(rows, -570)
  first = next(i for i, r in enumerate(rows) if r['sent'] != r['pre'])
  assert first - k0 in (40, 41)
  assert make(events, rows).applies
  events, rows = record(HOLD, spacing=slow, sent_override={first - 1: -568})
  with pytest.raises(ValueError, match='stuck guard does not reproduce'):
    make(events, rows)
  events, rows = record(HOLD, spacing=slow, sent_override={first: -570})
  with pytest.raises(ValueError, match='stuck guard does not reproduce'):
    make(events, rows)


@pytest.mark.parametrize('offset', [1, 30, STUCK_HOLD_FRAMES, STUCK_HOLD_FRAMES + 1, STUCK_HOLD_FRAMES + 2])
def test_window_starting_mid_hold_or_mid_burst_qualifies(offset):
  events, rows = record(HOLD)
  first = hold_start(rows, -570) + offset                    # guard run unknown at the window start
  replay = make(events, rows, first=first)
  for r in rows[first:]:
    replay.publish(r['t'], serialized_steer(-0.95))
  assert all(row['recorded'] == row['replay'] for row in replay.finish())


def test_window_starting_mid_hold_without_any_nudge_for_too_long_is_rejected():
  events, rows = record(HOLD, guard=False)
  with pytest.raises(ValueError, match='stuck guard does not reproduce'):
    make(events, rows, first=hold_start(rows, -570) + 1)


@pytest.mark.parametrize('mode', [True, 'auto'])
@pytest.mark.parametrize(('fault', 'message'), [
  ('command', 'Missing or inconsistent applied'), ('sequence', 'Missing apply sequence'),
  ('previous', 'previous TI command'), ('request', 'Recorded TI request'),
  ('limited', 'does not reproduce'), ('feedback', 'does not represent'),
  ('unavailable', 'stock fallback'), ('nan', 'Non-finite driver'), ('limit', 'TI600'),
  ('time', 'apply diagnostics/identity'),
])
def test_original_corruptions_still_rejected_under_guard_models(mode, fault, message):
  from . import test_recorded_feedback as original
  events = original.fixture_events()
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
  data = []
  for e in events:
    e.clear_write_flag()
    data.append(e.to_bytes())
  decoded = list(log.Event.read_multiple_bytes(b''.join(data)))
  commands = {int(e.logMonoTime): e for e in decoded if e.which() == 'carControl'}
  outputs = [(int(e.logMonoTime), e) for e in decoded if e.which() == 'carOutput']
  with pytest.raises(ValueError, match=message):
    RecordedTiFeedback(outputs, commands, 1100, 1300, LIMITER, stuck_guard=mode)


@pytest.mark.parametrize('mode', [True, 'auto'])
def test_original_fixture_qualifies_and_replays_under_guard_models(mode):
  from . import test_recorded_feedback as original
  data = []
  for e in original.fixture_events():
    e.clear_write_flag()
    data.append(e.to_bytes())
  decoded = list(log.Event.read_multiple_bytes(b''.join(data)))
  commands = {int(e.logMonoTime): e for e in decoded if e.which() == 'carControl'}
  outputs = [(int(e.logMonoTime), e) for e in decoded if e.which() == 'carOutput']
  replay = RecordedTiFeedback(outputs, commands, 1100, 1300, LIMITER, stuck_guard=mode)
  assert replay.stuck_guard_present is (mode is True)
  replay.publish(1160, 0.5)
  replay.publish(1200, 0.5)
  assert replay.output_for(1230) == serialized_steer(30/600)
  assert all(row['recorded'] == row['replay'] for row in replay.finish())


def test_invalid_mode_is_rejected():
  events, rows = record(SATURATED)
  with pytest.raises(ValueError, match='stuck_guard'):
    make(events, rows, stuck_guard='yes')
