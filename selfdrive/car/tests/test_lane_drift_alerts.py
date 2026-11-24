"""Steering-limit alerts need lane-drift evidence (command at its limit alone is normal on held bends)."""
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib.lane_drift import (CAR_HALF_WIDTH, HoldTimer, lane_drift_evidence, side_clearances)

DT = 0.01
TAKE_CONTROL_HOLD_S = 0.7


def model(left_y, right_y, p=0.9):
  return NS(laneLines=[NS(y=[0.0]), NS(y=[left_y]), NS(y=[right_y]), NS(y=[0.0])], laneLineProbs=[0.0, p, p, 0.0])


def run(models, steps_per_model, hold_s=TAKE_CONTROL_HOLD_S):
  """Saturated, undershooting, turning all true throughout; returns time (s) of first alert or None."""
  timer, t = HoldTimer(hold_s, DT), 0.0
  for m, n in zip(models, steps_per_model):
    for _ in range(n):
      t += DT
      if timer.step(lane_drift_evidence(m)):
        return t
  return None


def test_clearance_is_from_the_cars_side_not_the_centre():
  left, right = side_clearances(model(-1.8, 1.8))
  assert abs(left - (1.8 - CAR_HALF_WIDTH)) < 1e-9 and abs(right - (1.8 - CAR_HALF_WIDTH)) < 1e-9


def test_at_limit_for_seconds_with_car_centred_gives_no_alert():
  assert run([model(-1.8, 1.8)], [1000]) is None          # 10 s centred
  assert run([model(-1.5, 2.1)], [1000]) is None          # 0.3 m off centre, side still 0.58 m clear


def test_drifting_toward_a_line_alerts_after_the_hold():
  # side clearance 0.2 m (centre 1.12 m from the line)
  t = run([model(-1.12, 2.48)], [300])
  assert t is not None and abs(t - TAKE_CONTROL_HOLD_S) < 0.02


def test_crossing_a_line_alerts_and_either_side_counts():
  assert run([model(-0.5, 3.1)], [300]) is not None       # side over the left line
  assert run([model(-3.1, 0.5)], [300]) is not None       # side over the right line


def test_approach_from_centred_alerts_only_once_close_enough():
  # centred 2 s, then the left side closes to 0.25 m clearance: alert 0.7 s after the approach, not before
  t = run([model(-1.8, 1.8), model(-1.17, 2.43)], [200, 300])
  assert t is not None and abs(t - (2.0 + TAKE_CONTROL_HOLD_S)) < 0.02


def test_a_brief_close_pass_resets_the_hold():
  assert run([model(-1.1, 2.5), model(-1.8, 1.8), model(-1.1, 2.5)], [50, 10, 50]) is None


def test_no_visible_line_fails_open_and_low_confidence_line_is_ignored():
  assert lane_drift_evidence(model(-0.5, 3.1, p=0.2)) is True      # nothing visible: old behaviour
  assert lane_drift_evidence(NS(laneLines=[], laneLineProbs=[])) is True
