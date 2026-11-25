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
  hug = tick_model((0, -1.8, 1.6, .1, .9))                     # right turn, outside (left) unseen, inside hugged: 0.68 m from the car's side (centre 1.6 m; review I2: a centre 0.7 m is OVER the line)
  assert gate.update(hug, RIGHT_TURN, True) is False           # no latch from the fail-open tick
  # the inside line far (car wide) with the outside unseen: open (lane width default)
  assert lane_drift_evidence(tick_model((0, -3.0, 2.3, .1, .9)), RIGHT_TURN) is True


# ---- round 3: predictive opening. Logged model frames (L1y, L2y, pL1, pL2) every 0.05 s, right turn (outside = LEFT) on both passes ----
PASS_1_T0 = 744.04
PASS_1 = [
  (-1.76, 1.28, 0.78, 0.86), (-1.75, 1.23, 0.79, 0.85), (-1.79, 1.21, 0.81, 0.87), (-1.88, 1.21, 0.8, 0.87),
  (-1.87, 1.17, 0.81, 0.88), (-1.87, 1.18, 0.8, 0.88), (-1.88, 1.19, 0.8, 0.88), (-1.88, 1.18, 0.82, 0.89),
  (-1.92, 1.22, 0.81, 0.89), (-1.87, 1.2, 0.8, 0.88), (-1.83, 1.21, 0.85, 0.92), (-1.82, 1.2, 0.84, 0.92),
  (-1.81, 1.25, 0.83, 0.9), (-1.8, 1.34, 0.83, 0.89), (-1.78, 1.36, 0.82, 0.89), (-1.78, 1.39, 0.84, 0.91),
  (-1.75, 1.37, 0.83, 0.89), (-1.74, 1.38, 0.84, 0.9), (-1.72, 1.4, 0.85, 0.92), (-1.68, 1.42, 0.83, 0.9),
  (-1.69, 1.43, 0.85, 0.91), (-1.66, 1.43, 0.85, 0.92), (-1.68, 1.47, 0.86, 0.92), (-1.68, 1.51, 0.86, 0.92),
  (-1.64, 1.53, 0.83, 0.91), (-1.65, 1.56, 0.82, 0.91), (-1.64, 1.58, 0.84, 0.92), (-1.63, 1.61, 0.85, 0.92),
  (-1.64, 1.63, 0.87, 0.92), (-1.6, 1.61, 0.87, 0.93), (-1.6, 1.66, 0.88, 0.93), (-1.56, 1.66, 0.89, 0.94),
  (-1.51, 1.67, 0.91, 0.95), (-1.53, 1.67, 0.89, 0.94), (-1.48, 1.76, 0.9, 0.95), (-1.41, 1.81, 0.86, 0.93),
  (-1.42, 1.81, 0.86, 0.93), (-1.37, 1.83, 0.81, 0.91), (-1.34, 1.88, 0.76, 0.92), (-1.33, 1.94, 0.76, 0.93),
  (-1.27, 1.93, 0.79, 0.94), (-1.29, 1.92, 0.76, 0.94), (-1.26, 1.93, 0.78, 0.94), (-1.21, 1.94, 0.79, 0.95),
  (-1.1, 1.97, 0.85, 0.96), (-1.02, 2.05, 0.85, 0.96), (-0.92, 2.2, 0.88, 0.95), (-0.9, 2.21, 0.89, 0.97),
  (-0.87, 2.29, 0.89, 0.97), (-0.85, 2.29, 0.9, 0.97), (-0.8, 2.33, 0.88, 0.96), (-0.81, 2.3, 0.9, 0.97),
  (-0.78, 2.29, 0.91, 0.96), (-0.69, 2.43, 0.92, 0.96), (-0.63, 2.51, 0.91, 0.95), (-0.65, 2.5, 0.88, 0.9),
  (-0.57, 2.62, 0.9, 0.93), (-0.57, 2.56, 0.9, 0.94), (-0.5, 2.66, 0.92, 0.95), (-0.52, 2.7, 0.91, 0.95),
]
PASS_2_T0 = 1156.85
PASS_2 = [
  (-2.03, 1.29, 0.79, 0.07), (-2.03, 1.31, 0.77, 0.04), (-2.06, 1.26, 0.78, 0.05), (-1.99, 1.34, 0.8, 0.03),
  (-1.94, 1.33, 0.8, 0.02), (-1.94, 1.3, 0.8, 0.05), (-1.91, 1.3, 0.8, 0.05), (-1.83, 1.36, 0.8, 0.03),
  (-1.82, 1.36, 0.82, 0.05), (-1.77, 1.37, 0.84, 0.04), (-1.79, 1.37, 0.86, 0.07), (-1.81, 1.38, 0.89, 0.1),
  (-1.83, 1.44, 0.9, 0.17), (-1.8, 1.46, 0.91, 0.18), (-1.77, 1.55, 0.87, 0.09), (-1.73, 1.61, 0.89, 0.16),
  (-1.7, 1.63, 0.9, 0.16), (-1.67, 1.67, 0.91, 0.12), (-1.63, 1.7, 0.9, 0.09), (-1.59, 1.72, 0.88, 0.12),
  (-1.54, 1.79, 0.85, 0.08), (-1.49, 1.81, 0.89, 0.14), (-1.42, 1.96, 0.88, 0.05), (-1.41, 1.88, 0.88, 0.07),
  (-1.38, 1.92, 0.87, 0.12), (-1.38, 1.91, 0.88, 0.15), (-1.36, 1.93, 0.89, 0.41), (-1.35, 1.93, 0.91, 0.45),
  (-1.27, 2.02, 0.9, 0.47), (-1.21, 2.02, 0.85, 0.52), (-1.13, 2.05, 0.85, 0.56), (-1.1, 2.04, 0.82, 0.51),
  (-1.1, 2.19, 0.8, 0.47), (-1.28, 1.86, 0.77, 0.6), (-0.95, 2.28, 0.82, 0.61), (-1.05, 2.11, 0.8, 0.66),
  (-0.88, 2.33, 0.84, 0.83), (-0.9, 2.29, 0.84, 0.86), (-0.93, 2.23, 0.8, 0.83), (-0.82, 2.3, 0.83, 0.87),
  (-0.75, 2.43, 0.88, 0.88), (-0.72, 2.48, 0.86, 0.87), (-0.77, 2.37, 0.83, 0.84), (-0.99, 2.1, 0.8, 0.83),
  (-1.4, 1.78, 0.64, 0.7), (-1.92, 0.95, 0.57, 0.67),
]


def frame_model(l1, l2, p1=0.9, p2=0.9):
  return tick_model((0, l1, l2, p1, p2))


def run_frames(frames, t0, k=RIGHT_TURN, mirror=False):
  """Feed 20 Hz model frames (held for 5 control ticks of 10 ms) into a gate at the limit; returns (first open time, Take Control time)."""
  gate, timer, first, fired = LaneDriftGate(0.01), HoldTimer(TAKE_CONTROL_HOLD_S, 0.01), None, None
  for n, (l1, l2, p1, p2) in enumerate(frames):
    m = tick_model((0, l1, l2, p1, p2), mirror)
    for i in range(5):
      t = t0 + n * 0.05 + i * 0.01
      g = gate.update(m, k, True)
      first = t if (g and first is None) else first
      if timer.step(g) and fired is None:
        fired = t
  return first, fired


def test_predictive_gate_gives_take_control_before_the_car_is_over_the_line_on_both_passes():
  # car's side over the outside (left) line: clearance -L1y - 0.92 <= 0. Logged: 746.3 and 1158.55; the previous gate fired 0.4-0.5 s after
  for frames, t0, cross in ((PASS_1, PASS_1_T0, 746.3), (PASS_2, PASS_2_T0, 1158.55)):
    for mirror, k in ((False, RIGHT_TURN), (True, LEFT_TURN)):
      first, fired = run_frames(frames, t0, k, mirror)
      assert fired is not None and fired <= cross - 0.5, (fired, cross)


def test_steady_half_metre_clearance_at_the_limit_never_opens():
  for mirror, k in ((False, RIGHT_TURN), (True, LEFT_TURN)):
    frames = [(-1.42 + 0.03 * ((i * 7) % 3 - 1), 1.9, 0.9, 0.9) for i in range(200)]       # 0.5 m clearance +-0.03 m frame noise, 10 s
    assert run_frames(frames, 0.0, k, mirror)[0] is None


def test_slow_drift_does_not_open_before_the_positional_rule():
  # 0.1 m/s closing from 1.0 m clearance: opens only when the clearance is under 0.30 m (t = 7 s)
  for mirror, k in ((False, RIGHT_TURN), (True, LEFT_TURN)):
    frames = [(-(1.92 - 0.1 * 0.05 * i), 1.9, 0.9, 0.9) for i in range(200)]
    first, _ = run_frames(frames, 0.0, k, mirror)
    assert first is not None and first >= 7.0 - 0.1


def test_frame_noise_alone_does_not_predict():
  import random
  rnd = random.Random(5)
  frames = [(-1.72 + rnd.gauss(0, 0.05), 1.9, 0.9, 0.9) for _ in range(600)]      # 0.8 m clearance, sd 0.05 m noise, 30 s
  assert run_frames(frames, 0.0)[0] is None


# ---- review I2 (2026-11-25): crossed line faint, then confident on the inside with the far line unseen; probability hysteresis ----
def test_inside_line_straddled_opens_when_outside_unseen_both_directions():
  # reviewer's probe B: after the re-index the crossed line (now inside) is confident, the far line unseen; the car is still over it
  approach = [(-1.4 + 0.02 * i, 1.8 + 0.02 * i, .9, .9) for i in range(20)]
  crossing = [(-0.9 + 0.02 * i, 2.3 + 0.02 * i, .3, .9) for i in range(20)]
  after = [(-3.6, 0.3, .3, .9)] * 60
  for mirror, k in ((False, RIGHT_TURN), (True, LEFT_TURN)):
    gate = LaneDriftGate(0.01)
    opens = []
    for l1, l2, p1, p2 in approach + crossing + after:
      m = tick_model((0, l1, l2, p1, p2), mirror)
      opens += [gate.update(m, k, True) for _ in range(5)]
    assert all(opens[len(approach + crossing) * 5:]), "gate closed while the car is still over the crossed line"
  # not-at-limit, no latch: the rule itself
  assert lane_drift_evidence(tick_model((0, -3.6, 0.3, .1, .9)), RIGHT_TURN) is True
  assert lane_drift_evidence(tick_model((0, -0.3, 3.6, .9, .1)), LEFT_TURN) is True


def test_line_probability_hysteresis_keeps_a_fading_line_but_not_one_never_confident():
  for mirror, k in ((False, RIGHT_TURN), (True, LEFT_TURN)):
    gate = LaneDriftGate(0.05)
    out = []
    # outside (left, right turn) line 0.2 m from the car's side and closing, fading in probability; inside line 1.28 m clear (closed alone)
    for n, p in enumerate((0.8, 0.6, 0.4, 0.35, 0.2)):
      m = tick_model((0, -1.12 - 0.002 * n, 3.0 if n < 2 else 2.2, p, .9), mirror)    # width 4.12 m learned while both lines are confident
      out.append(gate.update(m, k, False))
    assert out == [True, True, True, True, False]
    gate = LaneDriftGate(0.05)
    for n in range(4):                       # never above 0.5: not evidence at 0.4 (outside unseen, inside near: closed)
      assert gate.update(tick_model((0, -1.12 - 0.002 * n, 1.9, 0.4, .9), mirror), k, False) is False
