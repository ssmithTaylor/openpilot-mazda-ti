# ruff: noqa: TID251
"""Independent integer expectations for TI/stock selection and separate histories."""
import pytest

from cereal import log
from .feedback import pure_limiter, pure_stock_limiter
from .recorded_feedback import RecordedTiFeedback, serialized_steer


def events():
  # request, active, TI selected, previous TI, TI, previous stock, stock
  table = [
    (0, False, True, 0, 0, 0, 0),
    (.5, True, True, 0, 10, 0, 10),
    (-.5, True, True, 10, -5, 10, -10),  # distinct zero-crossing rates
    (-.5, True, False, -5, 0, -10, -20),
    (-.5, True, False, 0, 0, -20, -30),
    (-.5, True, True, 0, -10, -30, -40),  # stock kept running during TI operation
    (0, False, False, -10, 0, -40, 0),
    (.5, True, False, 0, 0, 0, 10),
  ]
  rows = []
  for i, (request, active, selected, ti_prev, ti, stock_prev, stock) in enumerate(table):
    mono = 1000 + i * 100
    command = log.Event.new_message(logMonoTime=mono, valid=True)
    cc = command.init('carControl')
    cc.controlsStateMonoTime, cc.latActive, cc.actuators.steer = mono - 10, active, request
    output = log.Event.new_message(logMonoTime=mono + 20, valid=True)
    co = output.init('carOutput')
    co.applySequence, co.appliedAtMonoTime, co.appliedCarControlMonoTime = i + 1, mono + 10, mono
    co.actuatorsOutput.steer = (ti if selected else stock) / 600
    d = co.mazdaDiagnostics
    d.version, d.latActive, d.tiAllowed = 1, active, selected
    d.tiMax, d.tiDeltaUp, d.tiDeltaDown = 600, 10, 15
    d.tiDriverAllowance, d.tiDriverMultiplier = 15, 40
    d.tiDeltaUpKnee, d.tiDeltaUpHigh = 630, 9
    d.tiPrevious, d.tiLimited = ti_prev, ti
    d.tiRequested = int(request * 600) if active and selected else 0
    d.stockPrevious, d.stockLimited = stock_prev, stock
    d.stockRequested = int(request * 600) if active else 0
    rows.extend([command, output])
  return rows


def make(rows):
  decoded = list(log.Event.read_multiple_bytes(b''.join(e.to_bytes() for e in rows)))
  commands = {int(e.logMonoTime): e for e in decoded if e.which() == 'carControl'}
  outputs = [(int(e.logMonoTime), e) for e in decoded if e.which() == 'carOutput']
  ti, _ = pure_limiter('ad14961')
  stock, limits, _ = pure_stock_limiter('ad14961')
  return RecordedTiFeedback(outputs, commands, 1080, 1800, ti, stock_limiter=stock, stock_limits=limits)


def test_original_commands_reproduce_both_paths_and_feedback_selection():
  replay = make(events())
  for i in range(1, 8):
    replay.publish(990 + i * 100, 0 if i == 6 else .5 if i in [1, 7] else -.5)
  assert [replay.output_for(1020 + i * 100) for i in range(1, 8)] == [
    serialized_steer(x / 600) for x in [10, -5, -20, -30, -10, 0, 10]]
  assert list(replay.stock_counts.values()) == [10, -10, -20, -30, -40, 0, 10]
  assert all(r['recorded'] == r['replay'] for r in replay.finish())


def test_candidate_changes_stock_history_before_fallback_and_ti_restarts_from_zero():
  replay = make(events())
  for i in range(1, 8):
    replay.publish(990 + i * 100, .5)
  assert replay.output_for(1420) == serialized_steer(40 / 600)
  assert replay.output_for(1320) == serialized_steer(30 / 600)  # older consumed identity
  assert replay.output_for(1520) == serialized_steer(10 / 600)
  assert replay.output_for(1620) == 0
  assert replay.output_for(1720) == serialized_steer(10 / 600)
  assert list(replay.stock_counts.values()) == [10, 20, 30, 40, 50, 0, 10]


@pytest.mark.parametrize(('field', 'value', 'message'), [
  ('stockPrevious', -9, 'previous stock'),
  ('stockLimited', -21, 'does not reproduce recorded stock'),
  ('stockRequested', -299, 'stock request'),
  ('stockLimited', -601, 'outside qualified envelope'),
])
def test_stock_corruption_rejected(field, value, message):
  rows = events()
  # While TI is selected, corrupting stock must still fail independently of published feedback.
  d = rows[11].carOutput.mazdaDiagnostics
  setattr(d, field, value)
  with pytest.raises(ValueError, match=message):
    make(rows)


def test_eps_delivery_scale_cannot_replace_stock_command_normalization():
  rows = events()
  rows[7].carOutput.actuatorsOutput.steer = -20 / 308
  with pytest.raises(ValueError, match='does not represent'):
    make(rows)
