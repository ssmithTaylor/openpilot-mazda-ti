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


def test_crossed_outside_line_reindexed_as_inside_opens_the_gate():
  # left turn (outside RIGHT): centre 0.2 m short of the line (side 0.0 + ... line at 1.12), exactly on it, 0.5 m past it
  assert lane_drift_evidence(model2(left_y=-1.8, right_y=1.12), LEFT_TURN) is True
  assert lane_drift_evidence(model2(left_y=-1.8, right_y=0.0), LEFT_TURN) is True
  assert lane_drift_evidence(model2(left_y=-0.5, right_y=3.2), LEFT_TURN) is True      # line now indexed on the inside
  # right turn (outside LEFT), mirrored
  assert lane_drift_evidence(model2(left_y=-1.12, right_y=1.8), RIGHT_TURN) is True
  assert lane_drift_evidence(model2(left_y=0.0, right_y=1.8), RIGHT_TURN) is True
  assert lane_drift_evidence(model2(left_y=-3.2, right_y=0.5), RIGHT_TURN) is True


def test_mid_lane_in_a_turn_still_suppressed_and_weak_inside_line_ignored():
  for k in (LEFT_TURN, RIGHT_TURN):
    assert lane_drift_evidence(model2(left_y=-1.8, right_y=1.8), k) is False
  # a low-probability line near the centre is not evidence
  assert lane_drift_evidence(model2(left_y=-0.5, right_y=3.2, p_left=0.2), LEFT_TURN) is False


# ---- 2026-11-24 night drive (route 00000346): stateful gate. Fixtures are the logged model ticks (t, L1y, L2y, pL1, pL2) at 0.25 s ---
from openpilot.selfdrive.controls.lib.lane_drift import LaneDriftGate, lane_change_active

CROSSING_1 = [   # t=745.75..748.5: right turn (k>0), the car runs wide LEFT over the centre line; re-index at 747.5-747.75; Take Control at 746.9
  (745.75, -1.41, 1.81, .86, .93), (746.00, -1.27, 1.93, .79, .94), (746.25, -1.02, 2.05, .85, .96), (746.50, -0.80, 2.33, .88, .96),
  (746.75, -0.65, 2.50, .88, .90), (747.00, -0.58, 2.57, .92, .95), (747.25, -0.48, 2.46, .89, .92), (747.50, -1.09, 1.67, .84, .90),
  (747.75, -1.91, 0.48, .70, .86), (748.00, -2.60, 0.41, .77, .90), (748.25, -2.43, 0.62, .78, .82), (748.50, -2.18, 0.85, .81, .80)]
CROSSING_2 = [   # t=1054.25..1057: left turn (k<0), the car runs wide RIGHT over the line, outside line faint (p 0.19-0.58); Take Control at 1055.9
  (1054.25, -1.05, 2.23, .26, .41), (1054.50, -1.21, 2.29, .40, .19), (1054.75, -1.30, 2.14, .62, .35), (1055.00, -1.74, 1.75, .48, .21),
  (1055.25, -2.12, 1.27, .27, .19), (1055.50, -1.93, 1.09, .56, .32), (1055.75, -2.26, 0.86, .69, .58), (1056.00, -2.35, 0.78, .65, .33),
  (1056.25, -2.35, 0.76, .60, .55), (1056.50, -2.40, 0.68, .65, .58), (1056.75, -2.46, 0.73, .69, .58), (1057.00, -2.48, 0.74, .78, .64)]


def tick_model(row, mirror=False):
  _, l1, l2, p1, p2 = row
  if mirror:
    l1, l2, p1, p2 = -l2, -l1, p2, p1
  return NS(laneLines=[NS(y=[0.0]), NS(y=[l1]), NS(y=[l2]), NS(y=[0.0])], laneLineProbs=[0.0, p1, p2, 0.0],
            roadEdges=[NS(y=[-9.0]), NS(y=[9.0])], roadEdgeStds=[9.0, 9.0], meta=NS(laneChangeState=0))


def replay(rows, k, mirror=False, at_limit=True, dt=0.01):
  """Gate output every 10 ms (model rows held 0.25 s, as controlsd sees them); returns [(t, open)] and the time Take Control fires."""
  gate, timer, out, fired = LaneDriftGate(dt), HoldTimer(TAKE_CONTROL_HOLD_S, dt), [], None
  for row in rows:
    m = tick_model(row, mirror)
    for i in range(25):
      t = row[0] + i * dt
      g = gate.update(m, k, at_limit)
      out.append((t, g))
      if timer.step(g) and fired is None:
        fired = t
  return out, fired


def test_crossing_1_gate_stays_open_through_the_reindex_and_take_control_fires_both_directions():
  for mirror, k in ((False, RIGHT_TURN), (True, LEFT_TURN)):
    out, fired = replay(CROSSING_1, k, mirror)
    first = next(t for t, g in out if g)
    assert first <= 746.3                       # opens as the line is within 0.30 m (logged: 746.25)
    assert all(g for t, g in out if t >= first)  # continuously open through the re-index at 747.5-747.75 and after
    assert fired is not None and fired <= 747.0  # the logged alert came at 746.9


def test_crossing_2_faint_outside_line_still_alerts_and_stays_open_both_directions():
  for mirror, k in ((False, LEFT_TURN), (True, RIGHT_TURN)):
    out, fired = replay(CROSSING_2, k, mirror)
    assert fired is not None and fired <= 1056.5      # the car's side crossed the line at ~1055.75; logged alert 1055.9
    # open from the moment the car's side is over the line (1055.75). Before that the inside-only rule is 0.05 m short at 1055.5,
    # which is why Take Control comes 0.5 s after the logged 1055.9 (see lane_drift.DEFAULT_LANE_WIDTH)
    assert all(g for t, g in out if t >= 1055.75)


def test_false_running_wide_outside_line_faint_inside_hugged_stays_closed_both_directions():
  # night drive: outside prob 0.2, inside line seen and steady 0.7 m from the car's side (centre 1.62 m): hugging the inside
  rows = [(t * 0.05, -1.62, 1.8, 0.9, 0.2) for t in range(100)]
  for mirror, k in ((False, LEFT_TURN), (True, RIGHT_TURN)):
    out, fired = replay(rows, k, mirror)
    assert not any(g for _, g in out) and fired is None


def test_no_alert_from_the_gate_during_a_lane_change():
  gate = LaneDriftGate(0.01)
  m = tick_model(CROSSING_1[4])
  assert gate.update(m, RIGHT_TURN, True) is True
  assert gate.update(m, RIGHT_TURN, True, lane_change=True) is False
  assert gate.update(tick_model(CROSSING_1[4]), RIGHT_TURN, True) is True      # re-arms on its own evidence afterwards
  assert lane_change_active(NS(meta=NS(laneChangeState=2))) and lane_change_active(NS(meta=NS(laneChangeState=0)), right_blinker=True)
  assert not lane_change_active(NS(meta=NS(laneChangeState=0)))


def test_latch_releases_when_steering_is_clear_of_the_limit_and_expires_after_latch_time():
  gate = LaneDriftGate(0.01)
  assert gate.update(tick_model(CROSSING_1[4]), RIGHT_TURN, True)
  far = tick_model((0, -1.8, 1.8, .9, .9))
  assert gate.update(far, RIGHT_TURN, True)                      # latched, evidence gone
  for _ in range(40):
    gate.update(far, RIGHT_TURN, False)                           # clear of the limit for 0.4 s: released
  assert gate.update(far, RIGHT_TURN, True) is False
  gate = LaneDriftGate(0.01)
  gate.update(tick_model(CROSSING_1[4]), RIGHT_TURN, True)
  assert all(gate.update(far, RIGHT_TURN, True) for _ in range(140))     # still open ~1.4 s later
  assert not all(gate.update(far, RIGHT_TURN, True) for _ in range(20))   # expired by 1.6 s


def test_unseen_outside_line_is_not_evidence_and_failing_open_does_not_arm_the_latch():
  gate = LaneDriftGate(0.01)
  nothing = tick_model((0, -1.8, 1.8, .1, .1))
  assert gate.update(nothing, RIGHT_TURN, True) is True        # nothing gives a position: fail open as before
  hug = tick_model((0, -1.8, 0.7, .1, .9))                     # right turn, outside (left) unseen, inside hugged at 0.7 m
  assert gate.update(hug, RIGHT_TURN, True) is False           # no latch from the fail-open tick
  # the inside line far (car wide) with the outside unseen: open (lane width default)
  assert lane_drift_evidence(tick_model((0, -3.0, 2.3, .1, .9)), RIGHT_TURN) is True
