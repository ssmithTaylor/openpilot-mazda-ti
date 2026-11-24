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



# ---- review I2 (2026-11-24): in a turn, the OUTSIDE line or road edge; fail open when neither is seen -------------------------------
def model2(left_y=None, right_y=None, p_left=0.9, p_right=0.9, edge_left=None, edge_right=None, std=0.2):
  ly = left_y if left_y is not None else -1.8
  ry = right_y if right_y is not None else 1.8
  return NS(laneLines=[NS(y=[0.0]), NS(y=[ly]), NS(y=[ry]), NS(y=[0.0])],
            laneLineProbs=[0.0, p_left if left_y is not None else 0.0, p_right if right_y is not None else 0.0, 0.0],
            roadEdges=[NS(y=[edge_left if edge_left is not None else -9.0]), NS(y=[edge_right if edge_right is not None else 9.0])],
            roadEdgeStds=[std if edge_left is not None else 9.0, std if edge_right is not None else 9.0])


LEFT_TURN, RIGHT_TURN = -0.005, 0.005     # openpilot's desired curvature, RIGHT positive in this fork


def test_reviewers_scenario_left_bend_no_right_edge_line_running_wide_to_the_right():
  # centre line seen on the left and moving away, no painted right line, no edge seen: fail open (alert as before the gating)
  assert lane_drift_evidence(model2(left_y=-2.6), LEFT_TURN) is True
  # mirrored: a right bend, centre line on the right moving away, no left line
  assert lane_drift_evidence(model2(right_y=2.6), RIGHT_TURN) is True


def test_outside_line_confident_and_far_no_alert():
  assert lane_drift_evidence(model2(left_y=-0.95, right_y=2.0), LEFT_TURN) is False    # inside line close: not running wide
  assert lane_drift_evidence(model2(left_y=-2.0, right_y=0.95), RIGHT_TURN) is False


def test_outside_line_close_alerts():
  assert lane_drift_evidence(model2(left_y=-2.6, right_y=1.1), LEFT_TURN) is True
  assert lane_drift_evidence(model2(left_y=-1.1, right_y=2.6), RIGHT_TURN) is True


def test_outside_line_absent_edge_close_alerts_and_edge_far_does_not():
  assert lane_drift_evidence(model2(left_y=-2.6, edge_right=1.1), LEFT_TURN) is True
  assert lane_drift_evidence(model2(right_y=2.6, edge_left=-1.1), RIGHT_TURN) is True
  assert lane_drift_evidence(model2(left_y=-2.6, edge_right=2.5), LEFT_TURN) is False
  # an edge the model is not sure of is not "seen": fail open
  assert lane_drift_evidence(model2(left_y=-2.6, edge_right=2.5, std=1.5), LEFT_TURN) is True


def test_both_absent_fails_open_both_directions():
  for k in (LEFT_TURN, RIGHT_TURN):
    assert lane_drift_evidence(model2(), k) is True


def test_straight_road_unchanged():
  for k in (None, 0.0, 1e-4, -1e-4):
    assert lane_drift_evidence(model(-1.8, 1.8), k) is False
    assert lane_drift_evidence(model(-1.12, 2.48), k) is True
    assert lane_drift_evidence(model(-0.5, 3.1, p=0.2), k) is True
