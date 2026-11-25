"""Curve slowing planner (ti_curve_plan.py) on SYNTHETIC roads only (no recorded position is ever used in a test)."""
import json
import math

import numpy as np
import pytest

from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.selfdrive.car.mazda import ti_curve_plan as P
from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
from openpilot.selfdrive.car.mazda import ti_curve_demand as D

MPH = P.MPH
DT = 0.2


def road(*segments, **kw):
  return M.synthetic_trace(list(segments), **kw)


def drive(mem, trace, v_mph, seconds, start=0, fix_fn=None, yaw_fn=None, set_mph=None, planner=None, model_fn=None, closed=False,
          rate=1.0):
  """Drive along `trace` (a synthetic_trace dict); returns [(t, along m, Plan, mph)]. Open loop: constant speed. closed=True: the set
  speed walks toward the plan's ceiling one tap per TAP_S (a corner ceiling lowers it, a hold ceiling caps the way back up to the
  driver's v_mph), the speed answers it as the ONE measured cruise model says (ti_cruise_model), with `rate` times its slowing."""
  pl = planner or P.Planner(mem)
  v = float(v_mph)
  ref = set_mph if set_mph is not None else v_mph
  xs, ys, hs, ks = trace["x"], trace["y"], trace["hdg"], trace["kap"]
  s_road = np.arange(len(xs)) * M.SPACING_M
  out = []
  s = float(start)
  setv = ref
  n = int(seconds / DT)
  sub = 4
  car = CM.Cruise(v, ref, DT / sub)
  tap_t = 0.0
  for k in range(n):
    t = k * DT
    if s > s_road[-1]:
      break
    x, y = np.interp(s, s_road, xs), np.interp(s, s_road, ys)
    h = hs[min(int(s // M.SPACING_M), len(hs) - 1)]
    kap = ks[min(int(s // M.SPACING_M), len(ks) - 1)]
    fix = P.Fix(True, float(x), float(y), float(h))
    if fix_fn is not None:
      fix = fix_fn(t, s, fix)
    yk = kap if yaw_fn is None else yaw_fn(t, s, kap)
    mk = model_fn(t, s) if model_fn is not None else None
    p = pl.step(t, fix, v * MPH, set_mph=setv, yaw_k=yk, model_k=mk, ref_mph=ref)    # the car side's reference, as the daemon passes it
    out.append((t, s, p, v))
    if closed:
      goal = ref if p.ceil is None else min(ref, p.ceil)
      for j in range(sub):
        tj = t + j * DT / sub
        if setv != goal and tj >= tap_t - 1e-9:
          setv += 1 if goal > setv else -1
          tap_t = tj + P.TAP_S
        b = car.v
        car.step(setv)
        if car.v < b:
          car.v = b - rate * (b - car.v)
      v = car.v
    s += v * MPH * DT
  return [(t, s_, p) for t, s_, p, _ in out] if not closed else out


# ---- the speed law -------------------------------------------------------------------------------------------------------------
def test_need_mph_monotonic_and_fits():
  # rewritten for plan T4 (was test_corner_mph_monotonic_and_fits): need_mph per direction, demand within capacity over the window
  ks = np.linspace(0.0, 0.05, 400)
  for sign in (1.0, -1.0):
    sp = D.need_mph(sign * ks)
    assert np.all(np.diff(sp) <= 0)                    # sharper never allows more speed
    for k, s_ in zip(ks, sp):
      if s_ > P.CAR_MIN_MPH:
        side = D.LEFT if sign > 0 else D.RIGHT
        for v in (s_ - 0.75, s_, s_ + 0.75):
          assert D.demand(v, sign * k) <= D.capacity(v, side) + 1e-9
  for k in (0.004, 0.008, 0.012):
    lo = int(min(D.need_mph(k)[0], D.need_mph(-k)[0]))
    assert P.fits(k, lo) and not P.fits(k, lo + 1)       # P.fits: direction unknown, the lower side


def test_need_mph_examples():
  # rewritten for plan T4 (was test_corner_mph_examples: 40-46 mph for a 200 m bend at 0.9 x the old envelope). CAP_RAW, no margin:
  # left 49 mph (2.56 m/s^2 at 45-54), right 46 (2.19)
  assert D.need_mph(1 / 200.0)[0] == 49 and D.need_mph(-1 / 200.0)[0] == 46
  assert D.need_mph(0.0)[0] == P.MAX_MPH
  assert D.need_mph(1.0)[0] == P.CAR_MIN_MPH


def test_capacity_right_below_left_from_35():
  # plan T4 / section 3: measured per direction; the car holds less to the right from 35 mph. R7 (A12): the measured figures hold at
  # the band CENTRES (45-54: 49.5 mph); between centres the capacity is linear. The raw table's 32-34 band holds MORE to the right
  # (2.30 vs 2.23), so with R7 the raw right side is below the left only from about 36.6 mph: the raw check starts at 39.5 (the
  # 35-44 band's centre); the banked one still from 35.
  assert D.capacity(49.5, D.RIGHT, banked=True) == pytest.approx(1.91) and D.capacity(49.5, D.LEFT, banked=True) == pytest.approx(2.47)
  for banked, speeds in ((True, (35, 39.5, 40, 45, 49.5, 50, 55, 60, 65)), (False, (39.5, 40, 45, 49.5, 50, 55, 60, 65))):
    for mph in speeds:
      assert D.capacity(mph, D.RIGHT, banked) < D.capacity(mph, D.LEFT, banked), (mph, banked)
      assert D.capacity(mph, D.UNKNOWN, banked) == D.capacity(mph, D.RIGHT, banked)
  assert D.capacity(25, D.LEFT) == D.capacity(25, D.RIGHT) == 1.41
  # a right bend is slowed for more than the same left bend
  assert D.need_mph(-1 / 200.0)[0] < D.need_mph(1 / 200.0)[0]


def test_a6_graded_10_percent_over_capacity_costs_about_2_mph():
  # plan A6 (owner, 2026-11-24): the target is where demand equals capacity; nothing forces the old 10 mph minimum drop. A bend whose
  # demand at 45 mph is 10 % over the capacity there (CAP_RAW, continuous per R7: left 2.53, right 2.29 m/s^2 at 45 mph). R7 (A12):
  # 2-3 mph on BOTH sides (was 4 left / 1 right with the band steps): the whole-mph target over the +-0.75 mph window is 42 left
  # and 42 right, 3 mph lower. Never 35.
  k = 1.1 * D.capacity(45, D.LEFT) / (45 * MPH) ** 2
  assert D.demand(45, k) == pytest.approx(1.1 * D.capacity(45, D.LEFT))
  assert D.need_mph(k)[0] == 42
  kr = -1.1 * D.capacity(45, D.RIGHT) / (45 * MPH) ** 2
  assert D.need_mph(kr)[0] == 42
  for kk in (k, kr):
    assert 2 <= 45 - D.need_mph(kk)[0] <= 3
  # plan T6: a bend needing at most LATE_SHED_MPH off is stage 2's (late, forecast-confirmed), not the memory ramp's: with memory
  # alone (no preview) nothing slows for it; with the preview showing it, stage 2 sets 42 (rewritten for T6: was the memory ramp)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  res = drive(M.synthetic([tr]), tr, 45, 120)
  assert not any(p.ceil is not None and p.kind == "corner" for _, _, p in res)
  res = drive(M.synthetic([tr]), tr, 45, 120, model_fn=road_preview(tr, 45))
  ceils = [p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner"]
  assert ceils and min(ceils) == 42 and all(p.stage == 2 and p.src == "fc" for _, _, p in res if p.kind == "corner")


def road_preview(tr, v_mph):
  """model_fn for drive(): the model preview of the synthetic road ahead at v_mph, as the model publishes it (right positive,
  under-read by FC_RATIO)."""
  ahead = np.arange(0, 8.01, 0.25)

  def f(t, s):
    kk = np.array([tr["kap"][min(int(q // M.SPACING_M), len(tr["kap"]) - 1)] for q in s + ahead * v_mph * MPH])
    return ahead, P.MODEL_K_SIGN * kk * np.interp(ahead, D.FC_LEAD_S, D.FC_RATIO)
  return f


def test_capacity_continuous_from_32():
  # R7 (A12): no step between 32 and 90 mph (only the real one at 32); steps of 0.1 mph change it by at most 0.01 m/s^2
  for banked in (False, True):
    for side in (D.LEFT, D.RIGHT, D.UNKNOWN):
      c = [D.capacity(32 + 0.1 * i, side, banked) for i in range(580)]
      assert max(abs(b - a) for a, b in zip(c, c[1:])) <= 0.01, (banked, side)
      assert D.capacity(31.9, side, banked) == 1.41 and D.capacity(32.0, side, banked) > 2.0


def test_need_from_median_not_sharpest():
  # plan T4: the planner plans from memory kap_med, never kap (the sharpest pass)
  # (values changed for plan T6, which leaves bends needing at most LATE_SHED_MPH off to stage 2: was kap 0.006 / kap_med 0.004,
  # now a 60 -> 55 mph bend under R7; now kap 0.007 (41 mph, floor 45) / kap_med 0.0055 (47 mph, 13 off))
  tr = road((1500, 0.0), (150, 0.007), (800, 0.0))
  med = 0.0055 * (tr["kap"] > 0)
  mem = M.synthetic([tr], kap_med=[med])
  pl = P.Planner(mem)
  res = drive(mem, tr, 60, 120, planner=pl)
  ceils = [p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner"]
  assert ceils and min(ceils) == max(int(D.need_mph(0.0055)[0]), 45) == 47
  assert int(D.need_mph(0.0055)[0]) > max(int(D.need_mph(0.007)[0]), 45)
  assert np.all(pl.need[tr["kap"] > 0] == D.need_mph(0.0055)[0])


def test_no_kap_med_no_memory_stage():
  # plan T4: a memory file without kap_med has no memory stage (it used to fall back silently to the sharpest-pass kap)
  tr = road((1500, 0.0), (150, 1 / 60.0), (800, 0.0))
  mem = M.synthetic([tr], kap_med=False)
  assert not mem.has_kap_med
  res = drive(mem, tr, 55, 80)
  assert any(p.trusted for _, _, p in res)              # localisation still works
  assert all(p.ceil is None and p.alert_mph is None and p.stage == 0 for _, _, p in res)
  assert M.synthetic([tr]).has_kap_med and M.synthetic([tr]).bank_v == 1


def _mem_n(tr, npass):
  mem = M.synthetic([tr])
  mem.npass = np.full(mem.n, npass, np.int32)
  return mem


def test_r14_need_hi_never_into_the_low_zone_because_of_the_step():
  # R14 (A15): k 0.0100 is 2 % over the left capacity at 33 mph (over the +-0.75 mph window): stepped need 25 (the 1.41 step below 32
  # catches every window that touches it), held need 32; to the right (higher capacity) 33 either way
  assert int(D.need_mph(0.0100)[0]) == 25 and int(D.need_hi(0.0100)[0]) == 32
  assert int(D.need_mph(-0.0100)[0]) == int(D.need_hi(-0.0100)[0]) == 33
  # a bend too sharp even for the capacity at the step keeps its stepped (low-speed) need: it must be taken below the step anyway
  for k in (0.012, 0.015, -0.014, 0.02):
    assert int(D.need_hi(k)[0]) == int(D.need_mph(k)[0]) < D.STEP_MPH
  # nothing changes at or above the step
  ks = np.linspace(-0.009, 0.009, 400)
  hi, lo = D.need_hi(ks), D.need_mph(ks)
  assert np.all(hi[lo >= D.STEP_MPH] == lo[lo >= D.STEP_MPH]) and np.all(hi >= lo)


def test_r14_33_mph_bend_2_percent_over_costs_at_most_1_mph():
  # the diagnosis case (A14): a 20-pass bend, remembered median curvature 0.0100, 2 % over capacity at 33 mph: the target was 25 to
  # the left; now at most 1 mph (32: under MIN_DROP_MPH, so no slowdown at all), and nothing to the right
  for sign in (1.0, -1.0):
    tr = road((1500, 0.0), (150, sign * 0.0100), (800, 0.0))
    res = drive(_mem_n(tr, 20), tr, 33, 80)
    corner = [p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner"]
    assert not corner or min(corner) >= 32, (sign, corner)
    assert all(p.ceil is None or p.ceil >= 32 for _, _, p in res)
  # from 40 mph (a stage-1 drop): one slowdown to 32 (not 25), closed loop; a dip below 32 on the way does not re-plan it to the stepped 25
  tr = road((1500, 0.0), (150, 0.0100), (800, 0.0))
  res = drive(_mem_n(tr, 20), tr, 40, 120, closed=True)
  corner = [p.corner_mph for t, s, p, v in res if p.kind == "corner"]
  assert corner and min(corner) == 32
  assert min(v for t, s, p, v in res if s <= 1650) >= 31.4


def test_r14_car_already_below_32_uses_the_low_speed_capacity():
  # a car already below the step (e.g. held by a lead at 30) plans on the stepped capacity: the same bend asks for 25 there
  tr = road((1500, 0.0), (150, 0.0100), (800, 0.0))
  res = drive(_mem_n(tr, 20), tr, 30, 120, set_mph=40)
  corner = [p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner"]
  assert corner and min(corner) == 25


def test_approach_times_at_45():
  # plan T4 + A8-R4 + A9 / A11: the planner's own manoeuvre at 45 mph (push to T - DRIVE_GAP_MPH, model-timed raise), timing variant,
  # to within the planner's ARRIVED_MPH (0.5): 5 / 10 / 15 mph off take about 4.99 / 7.95 / 10.35 s, pinned here (15 off was 10.44
  # before the 14+ mph band entered the timing table, plan A10: the push to 24 opens a gap beyond 14 mph; 10 off was 7.86, 15 off
  # 10.25 with the
  # raise judged every 0.1 s: RAISE_CHECK_S is now 0.05, which equals judging it every step, test_a8_r6)
  for drop, want in ((5, 4.99), (10, 7.95), (15, 10.35)):
    T = 45 - drop
    t, _ = CM.manoeuvre(45.0, T, max(P.CAR_MIN_MPH, T - P.DRIVE_GAP_MPH), arrived=P.ARRIVED_MPH, tap_s=P.TAP_S)
    assert t == pytest.approx(want, abs=0.05), (drop, t)


def test_slow_distance_grows_with_drop():
  d = [P.slow_distance_m(55, x) for x in (54, 50, 45, 40)]
  assert all(b > a for a, b in zip(d, d[1:]))


def test_response_model_is_the_measured_one():
  # rewritten for plan amendment A1 (was test_response_model_saturates_at_6_mph, pinning the unmeasured 0.76 / 1.6 mph/s law)
  for g in (0, 1, 2, 3, 6, 12, 20):
    assert P.decel_mph_s(g) == CM.decel_mph_s(g)
  assert P.decel_mph_s(20) * MPH == pytest.approx(1.21)  # the 14+ band (plan A10; was capped at the 10-14 band, 0.90)
  assert CM.decel_ms2(20, CM.CONSERVATIVE) == pytest.approx(0.65)   # the safety-check variant keeps its cap
  assert P.DRIVE_GAP_MPH >= 6


def test_v_allow_inverts_approach():
  for T in (30, 40, 50):
    for D in (100.0, 250.0, 500.0):
      va = P.v_allow(D, T, T - 6, P.BUFFER_S)
      u = int(math.floor(va))
      if u > T:
        assert P.approach_m(u, T, T - 6) + P.BUFFER_S * u * MPH <= D + 1e-6


def test_no_ceiling_for_corner_holdable_at_current_speed():
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  mem = M.synthetic([tr])
  need = D.need_mph(CORNER_K)[0]
  res = drive(mem, tr, need, 80, set_mph=need)
  assert all(p.ceil is None for _, _, p in res)


def test_capped_when_corner_needs_more_than_15_below_limit_or_speed():
  tr = road((2500, 0.0), (80, 1 / 60.0), (500, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  res = drive(mem, tr, 60, 120, planner=pl)
  assert any(p.capped for _, _, p in res if p.ceil is not None)


# ---- localisation and planning on synthetic roads ---------------------------------------------------------------------------------
CORNER_K = 1 / 150.0      # needs about 39 mph


def single_corner():
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  return tr, M.synthetic([tr])


def test_straight_road_no_ceiling():
  tr = road((3000, 0.0))
  mem = M.synthetic([tr])
  res = drive(mem, tr, 55, 50)
  assert all(p.ceil is None for _, _, p in res)
  assert any(p.trusted for _, _, p in res)  # straight
  assert all(p.why in ("no known corner ahead", "matching the remembered road", "road not known here") for _, _, p in res)


def test_trust_needs_stable_match():
  tr, mem = single_corner()
  res = drive(mem, tr, 55, 10)
  first = next(t for t, _, p in res if p.trusted)
  assert first >= P.STABLE_S - 1e-6


def test_single_corner_arrives_on_target_just_in_time_and_releases():
  tr, mem = single_corner()
  res = drive(mem, tr, 55, 160, closed=True)
  need = max(D.need_mph(CORNER_K)[0], 55 - 15)       # never more than 15 mph below the speed (no known limit)
  at_entry = [v for t, s, p, v in res if s >= 1500 - P.ENTRY_M][0]
  assert at_entry <= need + 1.0, at_entry
  t_entry = [t for t, s, p, v in res if s >= 1500][0]
  reached = [t for t, s, p, v in res if v <= need + 1.0][0]
  assert t_entry - reached < 6.0, "slowed much earlier than needed"
  first = [t for t, s, p, v in res if p.ceil is not None and p.kind == "corner"][0]
  assert t_entry - first < 25.0
  in_corner = [p for t, s, p, v in res if 1500 <= s <= 1650]
  assert all(p.ceil is not None and p.ceil <= need and p.kind == "corner" for p in in_corner)
  after = [p for t, s, p, v in res if s > 1650 + P.HOLD_AFTER_M + 15]
  assert after and all(p.ceil is None or p.kind == "hold" for p in after)
  assert [v for t, s, p, v in res][-1] > need + 5, "speed never came back"


def test_push_opens_the_gap_then_raises_to_target():
  tr, mem = single_corner()
  res = drive(mem, tr, 55, 160, closed=True)
  res = drive(mem, tr, 65, 160, closed=True, set_mph=65)
  need = D.need_mph(CORNER_K)[0]                     # 36: floor 50 binds at 65 mph; use a faster start to see the push
  ceils = [p.ceil for t, s, p, v in res if p.kind == "corner"]
  T = max(need, 50)
  # owner rule 2026-11-24 (RULES.md: the set speed may pass the 15 mph line, the car's speed may not): the PUSH may go below the
  # floor 50 by the push depth (was: clamped at 50, min(ceils) == 50; now 50 - DRIVE_GAP_MPH = 44); the TARGET stays at the floor
  # and the car's speed never goes below it
  assert min(ceils) == max(T - P.DRIVE_GAP_MPH, CM.push_floor(50, P.DRIVE_GAP_MPH)) == 50 - P.DRIVE_GAP_MPH and ceils[-1] == T
  assert min(v for t, s, p, v in res) >= 50 - 0.05
  res = drive(mem, tr, 50, 160, closed=True)
  ceils = [p.ceil for t, s, p, v in res if p.kind == "corner"]
  assert min(ceils) == max(need - P.DRIVE_GAP_MPH, CM.push_floor(35, P.DRIVE_GAP_MPH)) and ceils[-1] == need   # pushed, then raised


def test_downhill_slow_car_is_flagged_not_pushed_lower():
  tr, mem = single_corner()
  res = drive(mem, tr, 50, 160, closed=True, rate=0.5)  # 50 mph: floor 35, below the corner's own speed (the floor does not bind)
  need = D.need_mph(CORNER_K)[0]
  assert need > 35
  corner = [p for t, s, p, v in res if p.kind == "corner"]
  assert any(p.behind for p in corner) and not any(p.capped for p in corner)   # behind plan is NOT "needs more"
  assert min(p.ceil for p in corner) >= max(35, need - P.DRIVE_GAP_MPH)       # never below the floor, never more push than 6 mph


def test_corner_right_after_long_straight_is_seen_far_ahead():
  # a sharp corner (needs ~25 mph) after 2 km of straight at 70 mph: the ceiling appears >= 500 m before it
  tr = road((2500, 0.0), (80, 1 / 60.0), (500, 0.0))
  mem = M.synthetic([tr])
  res = drive(mem, tr, 70, 120)
  on = [s for t, s, p in res if p.ceil is not None and p.kind == "corner"]
  # owner rule 2026-11-24: the push may go below the floor 55 (to CM.push_floor(55, DRIVE_GAP_MPH) = 49), so the approach it plans
  # is the shorter push-to-49 one (was approach_m(70, 55, 55) = 373 m, seen 378 m ahead; now approach_m(70, 55, 49) = 295 m, 303 m)
  assert on and 2500 - on[0] >= P.approach_m(70, 55, CM.push_floor(55, P.DRIVE_GAP_MPH)) - 10
  assert min(p.ceil for t, s, p in res if p.ceil is not None and p.kind == "corner") == CM.push_floor(55, P.DRIVE_GAP_MPH)


def test_s_bends_both_on_target():
  tr = road((1500, 0.0), (100, 1 / 200.0), (60, 0.0), (100, -1 / 120.0), (800, 0.0))
  mem = M.synthetic([tr])
  res = drive(mem, tr, 60, 200, closed=True)
  n1, n2 = D.need_mph(1 / 200.0)[0], D.need_mph(1 / 120.0)[0]
  v1 = [v for t, s, p, v in res if s >= 1500 - P.ENTRY_M][0]
  v2 = [v for t, s, p, v in res if s >= 1660 - P.ENTRY_M][0]
  assert v1 <= max(n1, 45) + 1.0 and v2 <= max(n2, 45) + 1.0
  # no restore between the two bends (the second is too close)
  between = [p for t, s, p, v in res if 1600 <= s <= 1650]
  assert all(p.ceil is not None for p in between)


def test_unknown_road_no_ceiling():
  tr, mem = single_corner()
  far = M.synthetic_trace([(3000, 0.0)], x0=50000.0, y0=50000.0)
  res = drive(mem, far, 55, 40)
  assert all(p.ceil is None and not p.trusted and p.why == "road not known here" for _, _, p in res)


def test_wrong_direction_no_match():
  tr, mem = single_corner()
  rev = {k: v[::-1].copy() for k, v in tr.items()}
  rev["hdg"] = np.array([M.wrap(h + math.pi) for h in rev["hdg"]])
  rev["kap"] = -rev["kap"]
  res = drive(mem, rev, 55, 40)
  assert all(p.ceil is None and not p.trusted for _, _, p in res)


def test_changed_road_drops_trust():
  # remembered: a corner; driven: the road is now straight there (live yaw says straight) -> trust dropped, no ceiling after
  tr = road((800, 0.0), (300, 1 / 150.0), (300, 0.0), (400, 1 / 150.0), (600, 0.0))
  mem = M.synthetic([tr])
  res = drive(mem, tr, 45, 120, yaw_fn=lambda t, s, k: 0.0)
  lost = [s for t, s, p in res if p.why == "road differs from memory"]
  assert lost, "the change was not detected"
  first = lost[0]
  assert first < 1400
  after = [p for t, s, p in res if first <= s <= first + P.CHANGE_HOLDOFF_M - 20]
  assert all(p.ceil is None and not p.trusted for p in after)


def test_gps_dropout_short_keeps_trust_long_drops():
  tr, mem = single_corner()
  short = drive(mem, tr, 50, 60, fix_fn=lambda t, s, f: P.Fix(False) if 20 <= t < 20.8 else f)
  assert all(p.trusted for t, s, p in short if 21 <= t <= 30)
  long = drive(mem, tr, 50, 60, fix_fn=lambda t, s, f: P.Fix(False) if 20 <= t < 30 else f)
  assert any(not p.trusted for t, s, p in long if 22 <= t < 30)
  assert all(p.ceil is None for t, s, p in long if 20 + P.FIX_LOST_S + 0.5 <= t < 30)


def test_gps_jump_loses_lock_not_wild_ceiling():
  tr, mem = single_corner()
  def jump(t, s, f):
    return P.Fix(True, f.x + 3000.0, f.y, f.hdg) if 10 <= t < 14 else f
  res = drive(mem, tr, 50, 40, fix_fn=jump)
  assert any(not p.trusted for t, s, p in res if 12 <= t < 16)
  # a single bad fix does not
  res1 = drive(mem, tr, 50, 40, fix_fn=lambda t, s, f: P.Fix(True, f.x + 3000.0, f.y, f.hdg) if abs(t - 10) < 1e-6 else f)
  assert all(p.trusted for t, s, p in res1 if 10 <= t <= 12)


def test_low_confidence_gives_no_ceiling():
  tr, mem = single_corner()
  res = drive(mem, tr, 50, 60, fix_fn=lambda t, s, f: P.Fix(False) if t >= 25 else f)
  for t, s, p in res:
    if p.conf < P.MIN_TRUST_CONF:
      assert p.ceil is None and not p.trusted


def test_follows_trace_link():
  a = road((1000, 0.0))
  end_h = a["hdg"][-1]
  b = M.synthetic_trace([(600, 0.0), (150, 1 / 150.0), (500, 0.0)], x0=a["x"][-1] + 5.0, y0=a["y"][-1], h0=end_h)
  mem = M.synthetic([a, b], nexts=[1, -1])
  whole = {k: np.r_[a[k], b[k]] for k in a}
  res = drive(mem, whole, 55, 100)
  on = [s for t, s, p in res if p.ceil is not None]
  assert on and on[0] < 1000 + 600, "the corner on the next trace was not seen from the first"
  memb = M.synthetic([a, b], nexts=[-1, -1])
  res2 = drive(memb, whole, 55, 100)
  assert not any(p.ceil is not None for t, s, p in res2 if s < 990)


def test_no_memory():
  pl = P.Planner(None)
  p = pl.step(0.0, P.Fix(True, 0, 0, 0), 20.0)
  assert p.ceil is None and not p.trusted and "no road memory" in p.why


def test_plan_json_has_no_position():
  tr, mem = single_corner()
  res = drive(mem, tr, 55, 30)
  d = P.plan_json(res[-1][2], 12.0, 7, "shadow")
  assert set(d) == {"m", "seq", "mode", "ceil", "conf", "trusted", "known", "why", "dist", "past", "model_la", "capped", "limit",
                    "model_k", "kind", "corner", "end", "behind", "alert", "need", "stage", "src", "ahead", "cap", "rel",
                    "ask"}   # plan T4; "ask" (the forecast demand profile, stage 3: T7)


def test_memory_roundtrip(tmp_path):
  tr = road((500, 0.0), (100, 0.01))
  arrays = {"x": tr["x"], "y": tr["y"], "hdg": tr["hdg"], "kap": tr["kap"], "v": np.full(len(tr["x"]), 20.0),
            "flags": np.zeros(len(tr["x"]), np.uint8), "v_flag": np.full(len(tr["x"]), np.nan), "npass": np.ones(len(tr["x"])),
            "trace_start": [0, len(tr["x"])], "trace_next": [-1]}
  p = tmp_path / "m.npz"
  M.save(p, arrays, {"lat0": 0.0, "lon0": 0.0, "kx": 1.0, "ky": 1.0})
  m = M.load(p)
  assert m.n == len(tr["x"]) and m.n_traces == 1
  bad = dict(arrays, trace_start=[0, 5])
  M.save(p, bad, {})
  with pytest.raises(ValueError):
    M.load(p)


def test_each_corner_is_started_once_even_when_the_car_does_not_slow():
  # the driver holds the speed (gas, or the cruise cannot slow): the corner is started once, not again as its points are passed
  tr = road((1500, 0.0), (100, 1 / 200.0), (60, 0.0), (100, -1 / 120.0), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  drive(mem, tr, 60, 200, planner=pl)
  assert sum(1 for e in pl.events if e["kind"] == "start") <= 2


# ---- the on-road warning --------------------------------------------------------------------------------------------------------
def test_alert_when_the_floor_binds_early_once_and_clears():
  tr = road((2500, 0.0), (120, 1 / 70.0), (800, 0.0))         # needs ~27-30 mph; at 60 mph the floor (45) binds
  mem = M.synthetic([tr])
  res = drive(mem, tr, 60, 200, closed=True)
  on = [(t, s, p) for t, s, p, v in res if p.alert_mph is not None]
  assert on, "no warning for a corner that needs more than the floor"
  t0, s0, p0 = on[0]
  t_entry = [t for t, s, p, v in res if s >= 2500][0]
  assert t_entry - t0 >= P.ALERT_LEAD_S                   # early: at least the lead before the corner
  assert all(p.alert_mph == p0.alert_mph for _, _, p in on)
  # once per corner: one contiguous stretch, gone after the corner
  ts = [t for t, _, _ in on]
  assert all(b - a <= DT + 1e-6 for a, b in zip(ts, ts[1:]))
  assert not any(p.alert_mph for t, s, p, v in res if s > 2620 + P.HOLD_AFTER_M + 20)


def test_alert_when_behind_plan_and_not_when_reachable():
  tr, mem = single_corner()
  res = drive(mem, tr, 50, 160, closed=True, rate=0.4)      # the cruise slows far less than planned (floor 35 does not bind)
  assert any(p.alert_mph for t, s, p, v in res)
  res = drive(mem, tr, 50, 160, closed=True)                # as planned: no warning
  assert not any(p.alert_mph for t, s, p, v in res)


def test_single_pass_corner_is_slowed_for_like_any_other():
  # owner, 2026-11-23 (RULES.md): "One pass is enough for full slowing; phantom protection comes from the pass-quality filters only".
  # A corner from one pass gets its full need (within the floor), model or not. (Restored: R15 had replaced this test; R15 is undone.)
  assert P.S1_MIN_PASSES == 1
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  want = max(D.need_mph(CORNER_K)[0], 50 - 15)
  for mf in (None, road_preview(tr, 50)):
    mem = _mem_n(tr, 1)
    res = drive(mem, tr, 50, 120, planner=P.Planner(mem), model_fn=mf)
    corner = [p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner" and p.stage in (1, 2)]
    assert corner and min(corner) == want, mf
    # exactly as a 20-pass corner
    mem20 = _mem_n(tr, 20)
    res20 = drive(mem20, tr, 50, 120, planner=P.Planner(mem20), model_fn=mf)
    assert [p.ceil for _, _, p in res] == [p.ceil for _, _, p in res20], mf


@pytest.mark.parametrize("min_passes", [3])
def test_r15_mechanism_min_passes_at_the_constraining_points(monkeypatch, min_passes):
  # the R15 mechanism (OFF by default: S1_MIN_PASSES = 1, the owner's rule) still works when switched on: stage 1 acts only where the
  # remembered points have S1_MIN_PASSES passes; with fewer the bend is left to stage 2 (no preview here: nothing) and the warning,
  # which still sounds
  monkeypatch.setattr(P, "S1_MIN_PASSES", min_passes)
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  want = max(D.need_mph(CORNER_K)[0], 50 - 15)
  for n, slowed in ((1, False), (2, False), (3, True), (20, True)):
    mem = _mem_n(tr, n)
    res = drive(mem, tr, 50, 120, planner=P.Planner(mem))
    corner = [p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner" and p.stage in (1, 2)]
    if slowed:
      assert corner and min(corner) == want, n
    else:
      assert not corner, n
      assert any(p.alert_mph == D.need_mph(CORNER_K)[0] for _, _, p in res), n        # the warning still sounds
  # per point: a bend whose sharpest points have 2 passes but whose milder lead-in has 5 is constrained by the lead-in only
  tr2 = road((1500, 0.0), (60, CORNER_K * 0.8), (90, CORNER_K), (800, 0.0))
  mem = _mem_n(tr2, 5)
  mem.npass[np.abs(mem.kap_med - CORNER_K) < 1e-9] = 2
  res = drive(mem, tr2, 55, 120, planner=P.Planner(mem))
  corner = [p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner"]
  assert corner and min(corner) == max(int(D.need_mph(CORNER_K * 0.8)[0]), 40)


def test_hold_between_corners_only_when_the_next_slowdown_is_near():
  far = road((1500, 0.0), (100, 1 / 120.0), (1200, 0.0), (100, 1 / 120.0), (800, 0.0))
  res = drive(M.synthetic([far]), far, 60, 300, closed=True)
  gap = [p for t, s, p, v in res if 1700 <= s <= 2000]
  assert gap and not any(p.kind == "hold" for p in gap)          # far apart: restore fully, slow again later
  assert max(v for t, s, p, v in res if 1700 <= s <= 2500) > 55
  near = road((1500, 0.0), (100, 1 / 120.0), (60, 0.0), (100, 1 / 120.0), (800, 0.0))
  res = drive(M.synthetic([near]), near, 60, 300, closed=True)
  between = [p for t, s, p, v in res if 1630 <= s <= 1650]
  assert between and all(p.ceil is not None for p in between)    # the next corner's slowdown starts at once: held


def test_same_road_with_offset_and_sharper_memory_is_not_a_change():
  # the remembered curvature is 1.6x sharper (a conservative figure) and 25 m out of step with this pass: still the same road
  tr = road((600, 0.0), (120, 1 / 150.0), (200, 0.0), (120, -1 / 180.0), (200, 0.0), (150, 1 / 200.0), (600, 0.0))
  mem_tr = dict(tr, kap=np.roll(tr["kap"], 5) * 1.6)
  mem = M.synthetic([mem_tr])
  res = drive(mem, tr, 45, 200)
  assert not any(p.why == "road differs from memory" for _, _, p in res)


# ---- drive of 2026-11-23 (log analysis): regressions ----------------------------------------------------------------------------
def test_h1_floor_follows_the_drivers_current_set_speed_not_an_old_episode():
  # two corners needing ~33 mph; first driven with the driver's set speed 55 (floor 40), then with 45 (floor 30): the second corner's
  # target must be its own speed (33), not the 40 left over from the first episode (installed 23bdf89 kept the planner's floor)
  k = 1 / 95.0
  tr = road((1500, 0.0), (120, k), (1500, 0.0), (120, k), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  first = drive(mem, tr, 55, 200, planner=pl, closed=True)
  need = D.need_mph(k)[0]
  assert need < 40
  later = [p for t, s, p, v in first if 3000 <= s <= 3150 and p.kind == "corner"]
  pl2 = P.Planner(mem)
  res = drive(mem, tr, 55, 70, planner=pl2, closed=True)                  # the first episode at 55 ...
  t_end = res[-1][0]
  s_end = res[-1][1]
  res2 = drive(mem, tr, 45, 200, planner=pl2, closed=True, start=s_end)   # ... then the driver's own 45 (the same planner object)
  c2 = [p for t, s, p, v in res2 if 3000 <= s <= 3150 and p.kind == "corner"]
  assert c2 and min(p.corner_mph for p in c2) == max(need, 45 - 15)


def test_h3_an_ordinary_start_is_never_behind_and_never_warns():
  # plan T6: at 45 mph this corner needs at most LATE_SHED_MPH off: stage 2's, which acts on the preview (now given to every run;
  # without one nothing slows for it and the warning rightly sounds)
  for v in (45, 55, 65):
    tr, mem = single_corner()
    res = drive(mem, tr, v, 200, closed=True, model_fn=road_preview(tr, v))
    assert not any(p.behind for t, s, p, vv in res), v
    assert not any(p.alert_mph for t, s, p, vv in res if not p.capped), v


def test_h6_change_check_uses_the_median_not_the_conservative_figure():
  tr = road((600, 0.0), (120, 1 / 150.0), (300, 0.0), (120, -1 / 180.0), (600, 0.0))
  mem = M.synthetic([dict(tr, kap=tr["kap"] * 3.0)])        # the conservative figure, 3x sharper than this road
  mem.kap_med = tr["kap"].copy()                             # the median pass: this road
  res = drive(mem, tr, 45, 150, yaw_fn=lambda t, s, k: k * 1.0 + 0.0003)
  assert not any(p.why == "road differs from memory" for _, _, p in res)
  assert P.CHANGE_HOLDOFF_M <= 150.0


def test_r3_long_even_bend_keeps_trust():
  for radius in (300.0, 450.0, 600.0):
    tr = road((300, 0.0), (900, 1 / radius), (300, 0.0))
    mem = M.synthetic([tr])
    res = drive(mem, tr, 50, 120)
    assert not any(p.why == "road differs from memory" for _, _, p in res), radius
  # a real change on an even bend (remembered bend, now straight) is still found
  tr = road((300, 0.0), (900, 1 / 300.0), (300, 0.0))
  res = drive(M.synthetic([tr]), tr, 50, 120, yaw_fn=lambda t, s, k: 0.0)
  assert any(p.why == "road differs from memory" for _, _, p in res)


def test_alert_re_arms_when_the_car_speeds_up_again_before_the_corner():
  tr = road((2500, 0.0), (120, 1 / 70.0), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  first = drive(mem, tr, 60, 30, planner=pl, closed=True)
  pl.alerted.update(pl.alerting)
  pl.alerting.clear()
  res = drive(mem, tr, 60, 200, planner=pl, closed=True, start=first[-1][1])
  assert any(p.alert_mph for t, s, p, v in res)


# ---- a fork: a ramp leaving a highway (owner, 2026-11-23: "an on-ramp shouldn't look the same as the highway itself") ------------
def _fork():
  ramp = road((400, 0.0), (70, -0.001), (400, 0.0), (150, CORNER_K), (200, 0.0))    # leaves at about 4 degrees, then a tight bend
  hwy = road((3000, 0.0))
  return ramp, hwy, M.synthetic([ramp, hwy])                                        # the ramp first: an equal match locks on it


def test_staying_on_the_highway_is_never_slowed_for_the_ramps_bend():
  ramp, hwy, mem = _fork()
  res = drive(mem, hwy, 60, 90)
  assert any(p.trusted for _, _, p in res)
  assert all(p.ceil is None for _, _, p in res)


def test_taking_the_ramp_is_still_slowed_for_its_bend():
  ramp, hwy, mem = _fork()
  res = drive(mem, ramp, 55, 60)
  assert any(p.ceil is not None for _, _, p in res)


# ---- arrival just in time (owner, 2026-11-24: "curve slowing must arrive just in time"; measured on 00000342 / 00000343: the car
# reached the corner speed a median 8.5 s before the bend began) ---------------------------------------------------------------------
def _first_corner_s(mem, tr, v_mph, planner=None):
  """Along-road metres where the first corner ceiling appears (open loop, constant speed)."""
  res = drive(mem, tr, v_mph, 200, planner=planner or P.Planner(mem))
  return next(s for t, s, p in res if p.ceil is not None and p.kind == "corner")


def test_arrival_constants_are_the_approved_ones():
  assert P.ENTRY_LA == 1.3               # plan T4 (was AIM_MIN_LA == FLAG_MIN_LA == 1.3; the flag cap is gone)
  assert P.BUFFER_S == 0.3 and P.ENTRY_M == 0.0
  assert P.ARRIVED_MPH == 0.5            # plan A8-R1 (was 1.5; 1.0 ruled, 0.5 the largest <= 1.0 that passes the lead-held e2e cases)
  # plan amendment A1: the reaction is the measured model's dead time (0.5 s, then a 0.5 s lag), the tap walk 0.30 s
  assert P.REACTION_S == CM.DELAY_S == 0.5 and P.TAP_S == CM.TAP_S == 0.30 and P.RESP_FACTOR == 1.0


def test_entry_is_where_the_bend_loads_the_car_not_where_the_lead_in_begins():
  # rewritten for plan T4 (was test_aim_is_where_the_bend_loads_the_car_not_where_the_flags_begin: the flags are gone). A right-hand
  # gentle lead-in tight at 50 mph (needs 47) but under ENTRY_LA at the corner's target (35: 1.18 m/s^2), then the bend (needs 25,
  # floor 35): the slowdown starts where it would for the bend alone, not at the lead-in
  segs = [(1500, 0.0), (80, -0.0048), (150, -1 / 95.0), (800, 0.0)]
  tr = road(*segs)
  tr_b = road((1500, 0.0), (80, 0.0), (150, -1 / 95.0), (800, 0.0))
  assert D.need_mph(-0.0048)[0] <= 48 and D.demand(35, -0.0048) < P.ENTRY_LA
  s_lead, s_bend = _first_corner_s(M.synthetic([tr]), tr, 50), _first_corner_s(M.synthetic([tr_b]), tr_b, 50)
  assert abs(s_lead - s_bend) <= 2 * M.SPACING_M + 1.0, (s_lead, s_bend)


def test_entry_falls_back_to_the_runs_sharpest_point():
  # rewritten for plan T4 (was test_aim_falls_back_to_the_runs_sharpest_point, through flags): no point reaches ENTRY_LA at T
  kabs = np.array([0.001, 0.002, 0.003, 0.0025])
  run = np.arange(4)
  assert P.entry_point(run, kabs, 30) == 2                 # 0.003 * 13.4^2 = 0.54 m/s^2 < 1.3: the sharpest
  assert P.entry_point(run, kabs, 60) == 1                 # 0.002 * 26.8^2 = 1.44: the first reaching ENTRY_LA


def test_a_lower_physics_speed_before_the_aim_point_is_still_honoured(monkeypatch):
  # A (60 m, 1/150: capacity allows 42 mph) then B (1/120: 37). With the entry raised so that it lands on B (ENTRY_LA 1.9: A asks
  # 1.82 at 37 mph, B 2.28), the car must still be down to A's own speed when it reaches A: no later start than for A alone.
  # (Plan T4: was AIM_MIN_LA 1.7 with both flagged at 45, a 35 mph cap on B; the flags are gone.)
  monkeypatch.setattr(P, "ENTRY_LA", 1.9)
  segs = [(1500, 0.0), (60, 1 / 150.0), (150, 1 / 120.0), (800, 0.0)]
  tr = road(*segs)
  mem = M.synthetic([tr])
  tr_a = road((1500, 0.0), (60, 1 / 150.0), (950, 0.0))
  s_aim, s_a = _first_corner_s(mem, tr, 50), _first_corner_s(M.synthetic([tr_a]), tr_a, 50)
  assert s_aim <= s_a + 1.0, (s_aim, s_a)


@pytest.mark.parametrize("v0", [45, 50, 55, 60])
def test_closed_loop_never_faster_than_the_envelope_before_the_aim_point(monkeypatch, v0):
  # the entry's SAFETY GUARANTEE, closed loop (the start-distance test above only compares where slowing begins): with the entry
  # pushed past a 120 m A onto B, the car that answers as the planner expects is, at every point of A, at or below A's own speed
  # (never below the floor; within the arrival tolerance the entry itself has), and at every point of B at or below B's target.
  # Plan T4: ENTRY_LA 1.9 (was AIM_MIN_LA 1.7 with flags at 45 capping B at 35); B's target is its own capacity speed (37).
  monkeypatch.setattr(P, "ENTRY_LA", 1.9)
  segs = [(1500, 0.0), (120, 1 / 150.0), (150, 1 / 120.0), (800, 0.0)]
  tr = road(*segs)
  mem = M.synthetic([tr])
  env_a = D.need_mph(np.array([1 / 150.0]))[0]
  floor = v0 - 15
  res = drive(mem, tr, v0, 200, closed=True)
  in_a = [v for t, s, p, v in res if 1500 <= s < 1620]
  in_b = [v for t, s, p, v in res if 1620 <= s < 1770]
  assert in_a and in_b
  assert max(in_a) <= max(env_a, floor) + P.ARRIVED_MPH + 0.05, (max(in_a), env_a)
  t_b = max(D.need_mph(np.array([1 / 120.0]))[0], floor)
  assert max(in_b) <= t_b + P.ARRIVED_MPH + 0.05, (max(in_b), t_b)


def test_approach_ends_at_target_plus_arrived_with_the_measured_model():
  # rewritten for plan amendment A1: the planner's approach is ti_cruise_model's manoeuvre, stepped by hand here; A8-R1: it ends at
  # T + ARRIVED_MPH (1.0; was ..._plus_1_5_... with the end hard-coded at T + 1.5)
  def ref(u, T, S1, dt=0.05):
    car = CM.Cruise(float(u), float(u), dt)
    setn, t, d, tap_t, raised, chk = u, 0.0, 0.0, 0.0, False, 0.0
    while car.v > T + P.ARRIVED_MPH and t < 90.0:
      # A9-R2a (was: within the fixed P.RAISE_AT_MPH of T): raised when the model says, judged when a tap could go out
      if not raised and t >= tap_t - 1e-9 and t >= chk - 1e-9 and car.v <= T + CM.RAISE_CHECK_MPH:
        raised, chk = CM.raise_due(car, setn, T), t + CM.RAISE_CHECK_S
      goal = T if raised else S1
      if t >= tap_t - 1e-9 and setn != goal:
        setn += 1 if goal > setn else -1
        tap_t = t + 0.30
      car.step(setn)
      d += car.v * MPH * dt
      t += dt
    return d
  for u, T in ((55, 40), (50, 38), (65, 50), (45, 41)):
    assert abs(P.approach_m(u, T, max(T - 6, 19)) - ref(u, T, max(T - 6, 19))) < 1e-6, (u, T)


def test_dead_time_is_in_the_start_distance(monkeypatch):
  # rewritten for plan amendment A1 (was test_reaction_1_3_s_is_in_the_start_distance): the model's 0.5 s dead time is driven at
  # the start speed before anything happens, so it lengthens the approach (not by a plain 0.5 s at 55 mph: the gap the cruise acts
  # on is itself delayed, so the walk has opened a larger gap by the time it acts)
  d05 = P.approach_m(55, 40, 34)
  monkeypatch.setattr(CM, "DELAY_S", 0.0)
  monkeypatch.setattr(P, "_APPROACH", {})
  d0 = P.approach_m(55, 40, 34)
  assert d05 > d0 + 2.0
  assert P.slow_distance_m(55, 40) == pytest.approx(P.approach_m(55, 40, 34) + 0.3 * 55 * MPH)


def test_single_corner_arrives_just_in_time():
  # a car that behaves as measured (the harness: REACTION_S late, the full measured deceleration) is at T + 1.5 mph by the bend, and
  # reaches it no more than ~BUFFER_S plus a planner tick or two before
  tr, mem = single_corner()
  res = drive(mem, tr, 55, 160, closed=True)
  need = max(D.need_mph(CORNER_K)[0], 55 - 15)
  at_entry = [v for t, s, p, v in res if s >= 1500][0]
  assert at_entry <= need + P.ARRIVED_MPH + 0.05, at_entry
  reached = [t for t, s, p, v in res if v <= need + P.ARRIVED_MPH + 0.05][0]
  t_entry = [t for t, s, p, v in res if s >= 1500][0]
  assert t_entry - reached < 2.0, t_entry - reached


@pytest.mark.parametrize("v0, need", [(55, 45), (65, 52), (45, 37), (60, 47)])
def test_a9_r2a_push_then_settle_at_target_no_dip_no_reacceleration(v0, need):
  # plan A9-R2a: after a push below the target the dash is raised at the moment the model says the car will bottom out at T, so the
  # speed never dips more than 0.5 mph below T and the car never speeds up before the corner's end
  ks = np.linspace(0.0005, 0.03, 6000)
  k = float(ks[np.argmin(np.abs(D.need_mph(ks) - need))])
  T = int(max(D.need_mph(k)[0], v0 - 15))
  assert T - P.DRIVE_GAP_MPH < T and max(v0 - 15, T - P.DRIVE_GAP_MPH) < T      # a push below T happens
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  res = drive(M.synthetic([tr]), tr, v0, 160, closed=True, set_mph=v0)
  ceils = [p.ceil for t, s, p, v in res if p.kind == "corner"]
  assert ceils and min(ceils) < T, "no push"
  start = next(i for i, (t, s, p, v) in enumerate(res) if p.kind == "corner")
  vs = [v for t, s, p, v in res[start:] if s <= 1650]
  assert min(vs) >= T - 0.5, (min(vs), T)
  assert all(b <= a + 0.02 for a, b in zip(vs, vs[1:])), "sped up before the corner's end"


# ---- stage 2: late, forecast-confirmed (plan T5) -----------------------------------------------------------------------------------
_FC_T = np.arange(0.0, 8.01, 0.25)


def preview(k_left, lead=3.0, width=0.5):
  """The model preview as the model PUBLISHES it for a bend of true LEFT-positive curvature k_left centred `lead` s ahead: openpilot's
  right-positive curvature (P.MODEL_K_SIGN), under-read by the measured FC_RATIO at each lead."""
  k = np.where(np.abs(_FC_T - lead) <= width, k_left, 0.0)
  return _FC_T, P.MODEL_K_SIGN * k * np.interp(_FC_T, D.FC_LEAD_S, D.FC_RATIO)


def k_needing(mph, sign=1.0):
  ks = np.linspace(0.0005, 0.05, 20000)
  return float(sign * ks[np.argmin(np.abs(D.need_mph(sign * ks) - mph) + (D.need_mph(sign * ks) != mph) * 1e3)])


def run_s2(pl, mph, fc_fn, seconds, set_mph=None, floor_mph=None, blink_fn=None):
  """Planner (no memory) at a steady speed, 5 Hz; fc_fn(t) -> preview or None. -> [(t, Plan)]"""
  out = []
  for i in range(int(seconds / DT)):
    t = i * DT
    out.append((t, pl.step(t, P.Fix(False), mph * MPH, set_mph=set_mph or mph, model_k=fc_fn(t), floor_mph=floor_mph,
                           blinker=bool(blink_fn and blink_fn(t)))))
  return out


def test_forecast_sign_left_positive():
  # plan T5 / section 4: the model's orientationRate.z is in openpilot's frame (z down): a LEFT bend has NEGATIVE z, so
  # ti_curve_plannerd.model_curvature (z / vx) is RIGHT positive, like openpilot's desired curvature (latcontrol_angle / _pid steer
  # from -desired_curvature). The planner turns it LEFT positive (MODEL_K_SIGN), the sign of its yaw_k (CAN 0x075, left +).
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.car.mazda import ti_curve_plannerd as PD
  v, k_left = 20.0, 1 / 150.0
  yaw_left = k_left * v                                     # rad/s, left positive (as ti_curve_plannerd.yaw_from_can returns)
  m = NS(orientationRate=NS(z=[-yaw_left] * len(_FC_T), t=list(_FC_T)), velocity=NS(x=[v] * len(_FC_T)))
  ts, km = PD.model_curvature(m)
  assert km[5] < 0                                          # published right positive
  pl = P.Planner(None)
  p = pl.step(0.0, P.Fix(False), v, set_mph=v / MPH, yaw_k=yaw_left / v, model_k=(ts, km))
  assert p.ahead_la is not None and p.ahead_la > 0 and np.sign(p.ahead_la) == np.sign(yaw_left / v)
  assert p.ahead_la >= k_left * v * v - 1e-6                  # calibrated: never below the raw preview
  need, lead, la, side = D.forecast_need(ts, P.MODEL_K_SIGN * np.asarray(km), v, P.FC_LO_S, P.CONFIRM_LEAD_S)
  assert side == D.LEFT and la > 0
  _, _, la_r, side_r = D.forecast_need(ts, -P.MODEL_K_SIGN * np.asarray(km), v, P.FC_LO_S, P.CONFIRM_LEAD_S)
  assert side_r == D.RIGHT and la_r < 0


def test_calibration_6s():
  # plan section 3: FC_RATIO 0.783 at 6 s: a raw 1.57 m/s^2 there is 2.0 actual
  assert D.calibrate([6.0], [1.57])[0] == pytest.approx(2.0, abs=0.01)
  assert D.calibrate([0.0, 10.0, 12.0], [1.0, 1.0, 1.0]) == pytest.approx([1.0, 1 / 0.538, 1 / 0.538])
  # the window: a bend only 1 s or 7 s ahead is not stage 2's
  for lead in (1.0, 7.0):
    need, _, _, _ = D.forecast_need(_FC_T, np.where(np.abs(_FC_T - lead) <= 0.25, 0.02, 0.0), 22.0, P.FC_LO_S, P.CONFIRM_LEAD_S)
    assert need == P.MAX_MPH
  assert (P.FC_LO_S, P.CONFIRM_LEAD_S, P.CONFIRM_TICKS, P.LATE_SHED_MPH) == (1.5, 6.0, 2, 5)


def test_stage2_caps_at_5():
  # plan T5: confirmed on the 2nd tick in a row; sheds at most LATE_SHED_MPH; a bend needing more than that (memory did not know it:
  # here there is no memory) gets the 5 mph plus the warning; floors apply; never below 20 mph; released after the bend
  k = k_needing(38)
  res = run_s2(P.Planner(None), 50, lambda t: preview(k) if t < 3.0 else None, 12.0)
  assert res[0][1].ceil is None                              # one tick is not a confirmation
  p = res[1][1]
  assert p.ceil == 45 and p.stage == 2 and p.src == "fc" and p.kind == "corner" and p.trusted and p.alert_mph == 38
  assert p.need_mph == 38 and P.plan_json(p, 0.2, 1, "act")["stage"] == 2
  # R9 (A12): Plan.cap for stage 2 is the raw capacity of the forecast's side (left here) at the target speed
  assert p.cap == pytest.approx(round(D.capacity(45, D.LEFT), 2)) and P.plan_json(p, 0.2, 1, "act")["cap"] == p.cap
  pr = run_s2(P.Planner(None), 50, lambda t: preview(k_needing(38, -1.0)), 1.0)[-1][1]
  assert pr.stage == 2 and pr.cap == pytest.approx(round(D.capacity(pr.ceil, D.RIGHT), 2))
  assert all(q.ceil == 45 for t, q in res if 0.2 <= t < 3.0)
  held = [t for t, q in res if q.ceil is not None]
  assert 3.0 + 2.0 < held[-1] < 3.0 + 6.0                       # held lead x v + HOLD_AFTER_M past the last sighting, then released
  # 3 below: the bend's own speed, no warning
  res = run_s2(P.Planner(None), 50, lambda t: preview(k_needing(47)), 2.0)
  assert res[-1][1].ceil == 47 and res[-1][1].alert_mph is None
  # the floor applies (published by the car side)
  res = run_s2(P.Planner(None), 50, lambda t: preview(k), 2.0, floor_mph=48)
  assert res[-1][1].ceil == 48
  # never below 20 mph
  res = run_s2(P.Planner(None), 19, lambda t: preview(k_needing(15)), 2.0)
  assert all(q.ceil is None for _, q in res)


def test_fork_forecast_needs_3s_when_memory_straight():
  # plan T5 / section 4 risk 4: on trusted remembered road that is STRAIGHT where the forecast shows a bend (a fork: the model may be
  # showing the other road) stage 2 needs FORK_CONFIRM_S (3 s) in a row, not 2 ticks
  tr = road((3000, 0.0))
  mem = M.synthetic([tr])
  k = k_needing(45)
  on = 6.0
  res = drive(mem, tr, 55, 14, model_fn=lambda t, s: preview(k) if t >= on else None)
  assert all(p.trusted for t, s, p in res if t >= on - 1.0)
  first = next(t for t, s, p in res if p.stage == 2)
  assert P.FORK_CONFIRM_S - 1e-6 <= first - on <= P.FORK_CONFIRM_S + 2 * DT, first - on
  # with no memory opinion (not trusted / no memory) 2 ticks are enough
  res = run_s2(P.Planner(None), 55, lambda t: preview(k) if t >= on else None, 8.0)
  assert next(t for t, p in res if p.stage == 2) - on == pytest.approx(DT)
  # a flicker shorter than 3 s at the fork never acts
  res = drive(mem, tr, 55, 14, model_fn=lambda t, s: preview(k) if on <= t < on + 2.4 else None)
  assert not any(p.stage == 2 for t, s, p in res)


def test_blinker_suppresses_stage2():
  # plan T5 / section 4 risk 4: with a blinker on (a turn, a lane change) the forecast's bend is not stage 2's; BLINKER_HOLD_S after
  k = k_needing(40)
  res = run_s2(P.Planner(None), 50, lambda t: preview(k), 6.0, blink_fn=lambda t: t < 2.0)
  assert all(p.ceil is None for t, p in res if t < 2.0 + P.BLINKER_HOLD_S)
  first = next(t for t, p in res if p.stage == 2)
  assert first == pytest.approx(2.0 - DT + P.BLINKER_HOLD_S + DT, abs=DT + 1e-6)
  # a blinker during stage 2 drops it
  res = run_s2(P.Planner(None), 50, lambda t: preview(k), 4.0, blink_fn=lambda t: t >= 2.0)
  assert res[5][1].stage == 2 and all(p.ceil is None for t, p in res if t >= 2.0)


def test_r8_behind_plan_predicts_with_the_push_and_raise_manoeuvre():
  # R8 (A12): behind-plan detection mid-approach predicts with the same push-and-raise manoeuvre as the warning (_arrive with T)
  tr, mem = single_corner()
  pl = P.Planner(mem)
  calls = []
  orig = pl._arrive

  def spy(v, d, set0, s1, lowering, T=None):
    calls.append((s1, T))
    return orig(v, d, set0, s1, lowering, T=T)
  pl._arrive = spy
  res = drive(mem, tr, 55, 120, closed=True, planner=pl)
  assert any(p.kind == "corner" for t, s, p, v in res)
  assert calls and all(T is not None and T >= s1 for s1, T in calls)


# ---- stage 1: early geometry, with release (plan T6, A3) ---------------------------------------------------------------------------
def _reversals(xs):
  """Direction changes of a sequence (ties ignored): [down, up, down] has 2."""
  dirs = []
  for a, b in zip(xs, xs[1:]):
    d = (b > a) - (b < a)
    if d and (not dirs or dirs[-1] != d):
      dirs.append(d)
  return max(0, len(dirs) - 1)


def test_45_to_30_arrives_at_entry():
  # plan T6 / section 4 risk 5: one ramp, the stage-1 part (no lower than T + 5) first, timed to arrive at the entry (the first
  # point whose demand reaches ENTRY_LA at T): within T + 1.5 mph there, and no early dwell (<= 1 s) by the closed loop and by
  # early_dwell.Profile
  from openpilot.tools.curve_slowing import early_dwell as E
  k = k_needing(24)                                            # R14: was k_needing(25), the step-edge bend now planned at 32
  T = 30                                                       # the floor (45 - 15) binds
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  assert D.demand(T, k) >= P.ENTRY_LA                          # the entry is the bend's first point
  res = drive(M.synthetic([tr]), tr, 45, 160, closed=True)
  at_entry = next(v for t, s, p, v in res if s >= 1500)
  assert at_entry <= T + 1.5, at_entry
  i0 = next(i for i, (t, s, p, v) in enumerate(res) if p.kind == "corner")
  assert res[i0][2].stage == 1 and res[i0][2].ceil == T + P.LATE_SHED_MPH
  ss = np.array([s for t, s, p, v in res])
  kap = np.array([tr["kap"][min(int(s // M.SPACING_M), len(tr["kap"]) - 1)] for s in ss])
  # early_dwell's profile from this start, "arrived" at the planner's ARRIVED_MPH (plan A11), to where the bend first loads the car
  prof = E.Profile(45, T, arrived=P.ARRIVED_MPH)
  j = E.load_index(ss, kap, T, i0, 1650.0, DT)
  dwell = prof.at(float(ss[j] - ss[i0]))[0] - prof.t_arr
  assert dwell <= 1.0, dwell
  # the stage-1 ceiling never goes below T + 5; after the hand-over (stage 2, same source) the ramp completes
  stages = [(p.stage, p.ceil) for t, s, p, v in res if p.kind == "corner"]
  assert all(c >= T + P.LATE_SHED_MPH for st, c in stages if st == 1)
  assert any(st == 2 and c == T for st, c in stages) and all(p.src == "mem" for t, s, p, v in res if p.kind == "corner")


def _preview_reach(tr, v_mph, reach_s):
  f = road_preview(tr, v_mph)

  def g(t, s):
    ts, k = f(t, s)
    return ts[ts <= reach_s + 1e-9], k[ts <= reach_s + 1e-9]
  return g


def test_r16_switch_none_means_no_cap(monkeypatch):
  # S1_UNCONFIRMED_MAX_MPH is ONE switch: None = no cap (stage 1's full graded ramp from the start, as before R16). Same road as the
  # R16 test below: with the cap off a preview showing a STRAIGHT road (never confirmed, released at CONFIRM_LEAD_S) lets the dash go
  # below start - 5 before the release; with the cap on it never does
  monkeypatch.setattr(P, "S1_UNCONFIRMED_MAX_MPH", None)
  assert P.s1_unconfirmed_ceiling(45.0) is None
  k, T = k_needing(24), 30
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 45))
  assert "s1_release" in [e["kind"] for e in pl.events]
  assert all(a_.get("cap_c") is None for a_ in pl.active.values())
  assert min(p.ceil for t, s, p, v in res if p.ceil is not None and p.kind == "corner") < 45 - 5   # uncapped: deeper before the release
  # confirmed bend, cap off: the full ramp, on time, exactly one start and one confirmation
  pl = P.Planner(mem)
  res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=road_preview(tr, 45))
  assert next(v for t, s, p, v in res if s >= 1500) <= T + 1.5
  # with the cap on (an explicit 5; the default is off since 2026-11-24) the same never-confirmed case keeps the dash at or above
  # start - 5
  monkeypatch.setattr(P, "S1_UNCONFIRMED_MAX_MPH", 5)
  pl = P.Planner(mem)
  res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 45))
  assert min(p.ceil for t, s, p, v in res if p.ceil is not None and p.kind == "corner") >= 45 - 5
  assert min(v for t, s, p, v in res if s <= 1650) >= 45 - P.S1_UNCONFIRMED_MAX_MPH - 0.5


def test_r16_default_is_no_cap_every_bend_gets_the_full_ramp():
  # 2026-11-24: the owner rejected the arbitrary cap (S1_UNCONFIRMED_MAX_MPH = None by default); the wrong-road cases it was for are
  # caught by road matching (test_rm_*). Same road and previews as the cap test below: every confirmed bend, whenever its preview
  # arrives (or with none able to confirm), gets the full ramp on time; a preview that shows a straight road releases it at
  # CONFIRM_LEAD_S and the speed comes back by the bend; nothing is "unconfirmed"
  assert P.S1_UNCONFIRMED_MAX_MPH is None and P.s1_unconfirmed_ceiling(45.0) is None
  k, T = k_needing(24), 30
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = M.synthetic([tr])
  f8 = road_preview(tr, 45)
  for mf in (f8, lambda t, s: f8(t, s) if s < 1500 - 175 else None, _preview_reach(tr, 45, 6.0), None, _preview_reach(tr, 45, 4.0)):
    pl = P.Planner(mem)
    res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=mf)
    kinds = [e["kind"] for e in pl.events]
    assert kinds.count("start") == 1 and "s1_unconfirmed" not in kinds and "s1_release" not in kinds
    assert next(v for t, s, p, v in res if s >= 1500) <= T + 1.5
    t_arr = next(t for t, s, p, v in res if v <= T + P.ARRIVED_MPH)
    assert next(t for t, s, p, v in res if s >= 1500) - t_arr <= 1.0                  # no early dwell
  # never confirmed (the preview reaches the entry and shows a straight road): released at CONFIRM_LEAD_S; with no cap the dash went
  # deeper before the release (measured here: 45 -> 38.8 mph), and the speed is back by the bend
  pl = P.Planner(mem)
  res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 45))
  assert "s1_release" in [e["kind"] for e in pl.events]
  assert 45 - 7.0 <= min(v for t, s, p, v in res if s <= 1650) < 45 - 5
  assert max(v for t, s, p, v in res if 1500 <= s <= 1650) > 43


def test_r16_unconfirmed_sheds_at_most_5_confirmed_gets_the_full_ramp(monkeypatch):
  # R16 (A15), the kept cap path with an explicit 5 (the default is off since 2026-11-24): before the forecast confirms, stage 1 sheds
  # at most S1_UNCONFIRMED_MAX_MPH; the rest after. The one decision:
  # confirmed at the first tick a preview reaching the entry shows the bend (here the model's 8 s preview, ~8 s out), released at the
  # CONFIRM_LEAD_S point if it does not show it, unconfirmed (cap kept) if no preview reaches the entry by then
  monkeypatch.setattr(P, "S1_UNCONFIRMED_MAX_MPH", 5)
  assert P.S1_UNCONFIRMED_MAX_MPH == 5 and P.s1_unconfirmed_ceiling(45.0) == 40 and P.s1_unconfirmed_ceiling(45.3) == 41
  k, T = k_needing(24), 30                                     # 45 -> 30 (the floor binds)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=road_preview(tr, 45))
  ev = [e for e in pl.events if e["kind"] in ("start", "s1_confirm", "s1_release", "s1_unconfirmed")]
  assert [e["kind"] for e in ev] == ["start", "s1_confirm"] and ev[1]["preview"]
  t_conf = next(t for t, s, p, v in res if p.ceil is not None and p.ceil < 40)
  assert all(p.ceil >= 40 for t, s, p, v in res if p.ceil is not None and t < t_conf)
  at_entry = next(v for t, s, p, v in res if s >= 1500)
  assert at_entry <= T + 1.5, at_entry                          # one continuous ramp, still on time
  t_arr = next(t for t, s, p, v in res if v <= T + P.ARRIVED_MPH)
  t_bend = next(t for t, s, p, v in res if s >= 1500)
  assert t_bend - t_arr <= 1.0                                 # no early dwell
  # never confirmed (the preview reaches the entry and shows a straight road): at most 5 mph lost, then restored by the bend
  pl = P.Planner(mem)
  res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 45))
  assert "s1_release" in [e["kind"] for e in pl.events]
  assert min(v for t, s, p, v in res if s <= 1650) >= 45 - P.S1_UNCONFIRMED_MAX_MPH - 0.5
  assert max(v for t, s, p, v in res if 1500 <= s <= 1650) > 43
  # the preview there at the start, then gone before it reaches the entry: unconfirmed at CONFIRM_LEAD_S, stage 1 keeps the cap
  pl = P.Planner(mem)
  f8 = road_preview(tr, 45)
  # (cutoff 1500 - 220 before the hold-branch merge; the push below the floor, CM.push_floor 30 -> 25, shortens the approach, so the
  # start moved to ~1316 m: the cutoff must fall between the start and where an 8 s preview first reaches the entry, ~1339 m)
  res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=lambda t, s: f8(t, s) if s < 1500 - 175 else None)
  assert "s1_unconfirmed" in [e["kind"] for e in pl.events] and "s1_confirm" not in [e["kind"] for e in pl.events]
  assert all(p.ceil >= 40 for t, s, p, v in res if p.ceil is not None and p.src == "mem")
  # PHYSICAL LIMIT (reported, not met): a preview that confirms only at CONFIRM_LEAD_S (6 s) cannot deliver the 10 mph left after the
  # cap in time with the measured cruise (from 40 settled: 7.95 s / 128 m to 30.5 vs 6 s / 107 m): it arrives about 1.3 mph late
  pl = P.Planner(mem)
  res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=_preview_reach(tr, 45, 6.0))
  assert "s1_confirm" in [e["kind"] for e in pl.events]
  assert T + 1.5 < next(v for t, s, p, v in res if s >= 1500) <= T + 3.5
  # with no preview able to confirm (none: the model not running; or one shorter than CONFIRM_LEAD_S) there is nothing to wait for:
  # no cap, as before R16
  for mf in (None, _preview_reach(tr, 45, 4.0)):
    pl = P.Planner(mem)
    res = drive(mem, tr, 45, 160, closed=True, planner=pl, model_fn=mf)
    assert next(v for t, s, p, v in res if s >= 1500) <= T + 1.5


def test_stage1_hands_over_to_stage2_without_a_step_or_reacceleration():
  # plan T6: stage 1 (T + 5) hands over to stage 2 on the same ramp: one hand-over, the ceiling never rises before the push's
  # model-timed raise, the speed never rises from the start to the corner's end, and the corner is made
  k = k_needing(42)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  res = drive(M.synthetic([tr]), tr, 55, 160, closed=True, model_fn=road_preview(tr, 55))
  T = int(D.need_mph(k)[0])
  corner = [(t, s, p, v) for t, s, p, v in res if p.kind == "corner"]
  st = [p.stage for t, s, p, v in corner]
  assert st[0] == 1 and 2 in st and _reversals(st) == 0           # 1 ... 1 2 ... 2: one hand-over, never back
  h = st.index(2)
  ceils = [p.ceil for t, s, p, v in corner]
  assert ceils[h] <= ceils[h - 1]                                 # no step up at the hand-over
  lo = ceils.index(min(ceils))
  assert all(b <= a for a, b in zip(ceils[:lo], ceils[1:lo + 1]))  # down only until the push's low point
  vs = [v for t, s, p, v in res if corner[0][1] <= s <= 1650]
  assert all(b <= a + 0.02 for a, b in zip(vs, vs[1:])), "sped up before the corner's end"
  assert next(v for t, s, p, v in res if s >= 1500) <= T + 1.0
  assert not any(p.released for t, s, p, v in res)


def test_release_at_6s_restores_and_no_reslow():
  # plan T6 / section 4 risk 2: memory knows a bend the preview (reaching the entry) does not show: at the hand-over, CONFIRM_LEAD_S
  # before the entry, it is released (Plan.released), the set speed is restored and the bend is not slowed for again; with a
  # flickering forecast at most 1 down-up-down reversal per bend
  k = k_needing(40)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  straight = road((3000, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 160, closed=True, planner=pl, model_fn=road_preview(straight, 55))
  ev = [e["kind"] for e in pl.events if e["kind"] in ("start", "s1_release", "s1_confirm")]   # (s1_handover may come first)
  assert ev[:2] == ["start", "s1_release"] and "s1_confirm" not in ev, ev
  rel = next((t, s, v) for t, s, p, v in res if p.released)
  assert 0.0 <= 1500 - rel[1] <= P.CONFIRM_LEAD_S * rel[2] * MPH + 15.0, rel
  assert all(p.ceil is None or p.kind == "hold" for t, s, p, v in res if rel[1] < s < 1650)
  sets = [p.ceil if p.ceil is not None and p.kind == "corner" else 55 for t, s, p, v in res]
  assert _reversals(sets) <= 1                                   # down, up: never down again
  assert max(v for t, s, p, v in res if 1500 <= s <= 1650) > 50     # restored by the (absent) bend
  # flickering: the preview shows the remembered bend 0.6 s on, 0.6 s off
  flick = road_preview(tr, 55)
  flat = road_preview(straight, 55)
  res = drive(mem, tr, 55, 160, closed=True, model_fn=lambda t, s: flick(t, s) if int(t / 0.6) % 2 else flat(t, s))
  win = [(t, p) for t, s, p, v in res if s <= 1650 + P.HOLD_AFTER_M]
  sets = [p.ceil if p.ceil is not None and p.kind == "corner" else 55 for t, p in win]
  # owner rule 2026-11-24 (push below the floor): the deeper push shortens the approach, so stage 1 now starts later and its
  # release (CONFIRM_LEAD_S before the entry) can come one or two planner cycles BEFORE stage 2's forecast takes the bend over: the
  # ceiling shows one down-up-down and then the restore up (was <= 2 reversals, seen 1: [55, 45, 40, 41, 55]; now <= 3, seen 3:
  # [55, 45, 34, 55, 46, 55]). Never more than that, and the "up" of that hand-over lasts at most 2 planner cycles (0.4 s).
  assert _reversals(sets) <= 3                                   # at most one down-up-down, then the restore
  firsts = [i for i, (t, p) in enumerate(win) if p.ceil is not None and p.kind == "corner"]
  gaps = [i for i in range(firsts[0], firsts[-1] + 1) if not (win[i][1].ceil is not None and win[i][1].kind == "corner")]
  assert len(gaps) <= 2, [round(win[i][0], 1) for i in gaps]


def _map_profile(s, bend_at, bend_len, k):
  d = np.arange(-30.0, 800.0, 10.0)
  return d, np.where((s + d >= bend_at) & (s + d < bend_at + bend_len), abs(k), 0.0)


def _drive_map(pl, mph, seconds, map_fn):
  out = []
  for i in range(int(seconds / DT)):
    t = i * DT
    s = mph * MPH * t
    out.append((t, s, pl.step(t, P.Fix(False), mph * MPH, set_mph=mph, ref_mph=mph, map_k=map_fn(t, s))))
  return out


def test_map_absent_or_stale_no_map_stage():
  # plan T6 / section 4 risk 3: the map acts (stage 1, src "map") where no memory is known; absent, or stale (plannerd's map_profile
  # gives None), there is no map stage; with trusted memory the map is not used
  from openpilot.selfdrive.car.mazda import ti_curve_plannerd as PD
  k = 1 / 120.0
  res = _drive_map(P.Planner(None), 45, 90, lambda t, s: _map_profile(s, 1500.0, 150.0, k))
  m = [p for t, s, p in res if p.src == "map"]
  assert m and all(p.trusted and p.stage in (1, 2) for p in m) and any(p.kind == "corner" for p in m)
  assert min(p.ceil for p in m if p.kind == "corner") >= 30                  # the floor
  res = _drive_map(P.Planner(None), 45, 90, lambda t, s: None)
  assert all(p.ceil is None for t, s, p in res)
  good = json.dumps([{"latitude": 0.0, "longitude": 0.0, "curvature": 0.0}, {"latitude": 0.001, "longitude": 0.0, "curvature": k}])
  assert PD.map_profile(good, 0.5, 0.0, 0.0, 0.0) is not None and PD.map_profile(good, PD.MAP_STALE_S + 0.1, 0.0, 0.0, 0.0) is None
  # the map going away mid-approach: review C1 (2026-11-24) changed this (was: its ceiling dropped at once). A bend whose slowdown has
  # started is a corner in progress: its published ceiling is held (the guard, by the odometer) to its end + HOLD_AFTER_M, then released
  t_on = next(t for t, s, p in _drive_map(P.Planner(None), 45, 90, lambda t, s: _map_profile(s, 1500.0, 150.0, k))
              if p.src == "map" and p.kind == "corner")
  res = _drive_map(P.Planner(None), 45, 90, lambda t, s: _map_profile(s, 1500.0, 150.0, k) if t < t_on + 1.0 else None)
  lo = min(p.ceil for t, s, p in res if t < t_on + 1.0 and p.ceil is not None)
  assert any(p.src == "map" for t, s, p in res)
  assert all(p.ceil is not None and p.ceil <= max(lo, p.corner_mph or 0) for t, s, p in res if t >= t_on + 1.0 and s <= 1650)
  assert all(p.ceil is None for t, s, p in res if s > 1650 + P.HOLD_AFTER_M + 5)
  # trusted memory (a straight remembered road): the map's bend is ignored
  tr = road((3000, 0.0))
  pl = P.Planner(M.synthetic([tr]))
  out = []
  for i in range(int(45 / DT)):
    t = i * DT
    s = 45 * MPH * t
    j = min(int(s // M.SPACING_M), len(tr["x"]) - 1)
    out.append(pl.step(t, P.Fix(True, float(tr["x"][j]), float(tr["y"][j]), float(tr["hdg"][j])), 45 * MPH, set_mph=45, ref_mph=45,
                       yaw_k=0.0, map_k=_map_profile(s, 1500.0, 150.0, k)))
  assert any(p.trusted and p.known for p in out) and not any(p.src == "map" for p in out)


def test_map_uses_lower_side_capacity():
  # plan T6: map curvature is unsigned: the bend is judged on the LOWER side's capacity (side 0), whatever its real direction
  k = 1 / 160.0
  assert D.need_mph(-k)[0] < D.need_mph(k)[0]                  # right holds less here
  T = max(int(D.need_mph(-k)[0]), 40)
  res = _drive_map(P.Planner(None), 55, 75, lambda t, s: _map_profile(s, 1500.0, 150.0, k))
  c = [p for t, s, p in res if p.src == "map" and p.kind == "corner"]
  assert c and all(p.corner_mph == T for p in c)
  assert c[0].cap == pytest.approx(D.capacity(T, D.UNKNOWN))


def test_map_bend_must_persist_a3():
  # plan A3: a map bend acts only once it has been shown 0 s (45 mph and below) rising linearly to 2.5 s (50 mph and above);
  # a phantom shown less than that never acts
  assert P.map_persist_s(45) == 0.0 and P.map_persist_s(40) == 0.0
  assert P.map_persist_s(47.5) == pytest.approx(1.25) and P.map_persist_s(50) == 2.5 and P.map_persist_s(70) == 2.5
  k = 1 / 100.0

  def shown(t0, t1):
    return lambda t, s: _map_profile(s, 1500.0, 150.0, k if t0 <= t < t1 else 0.0)
  # at 60 mph the bend is first shown at 46 s, when its start is already due (about 270 m before it): acts 2.5 s later, not before
  res = _drive_map(P.Planner(None), 60, 60, shown(46.0, 99.0))
  first = next(t for t, s, p in res if p.src == "map")
  assert first == pytest.approx(46.0 + 2.5, abs=DT + 1e-6), first
  # a 2.1 s phantom at 60 mph never acts
  res = _drive_map(P.Planner(None), 60, 60, shown(46.0, 48.1))
  assert not any(p.src == "map" for t, s, p in res)
  # at 45 mph the same 2.1 s showing acts at once (0 s persistence) when its start is due
  res = _drive_map(P.Planner(None), 45, 80, shown(64.0, 66.1))
  assert next(t for t, s, p in res if p.src == "map") == pytest.approx(64.0, abs=1e-6)


def test_position_gap_keeps_an_active_corner_by_dead_reckoning():
  # Job 2 (2026-11-24): a remembered corner being slowed for survives a lost position fix (well past FIX_LOST_S) by the odometer
  # until passed; with the 8 s preview, stage 2's higher ceiling never replaces the memory's lower one inside the bend
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  mem = M.synthetic([tr])
  want = max(int(D.need_mph(CORNER_K)[0]), 55 - 15)
  pl = P.Planner(mem)
  gap = (1420.0, 1640.0)
  res = drive(mem, tr, 55, 160, closed=True, planner=pl, model_fn=road_preview(tr, 55),
              fix_fn=lambda t, s, f: P.Fix(False) if gap[0] <= s <= gap[1] else f)
  inside = [(t, s, p) for t, s, p, v in res if gap[0] <= s <= 1650 + P.HOLD_AFTER_M]
  assert inside and (inside[-1][0] - next(t for t, s, p in inside if s >= gap[0])) > P.FIX_LOST_S   # the gap outlasts FIX_LOST_S
  assert all(p.ceil is not None and p.ceil <= want for t, s, p in inside if s <= 1650), \
    [(round(s), p.ceil, p.stage, p.src) for t, s, p in inside if p.ceil is None or p.ceil > want][:5]
  assert any(e["kind"] == "lock_lost" and e["why"] == "no position fix" for e in pl.events)
  assert next(v for t, s, p, v in res if s >= 1500) <= want + 1.0
  # passed: released by the odometer, not kept for ever
  assert any(p.ceil is None for t, s, p, v in res if s > 1650 + P.HOLD_AFTER_M + 20)


def test_other_lock_losses_keep_the_corner_too():
  # review C1 (2026-11-24) changed this (was test_other_lock_losses_still_drop_the_corner: only a lost FIX kept the corner and a
  # matcher finding the car elsewhere dropped it, ceiling None inside the bend). Now EVERY lock loss keeps a corner in progress by
  # dead reckoning to its end + HOLD_AFTER_M: never a ceiling above the corner's inside the bend; released once passed
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 160, closed=True, planner=pl,
              fix_fn=lambda t, s, f: P.Fix(True, f.x + 400.0, f.y + 400.0, f.hdg) if 1450 <= s <= 1700 else f)
  assert any(e["kind"] == "lock_lost" and e["why"] != "no position fix" for e in pl.events)
  t_lost = next(i for i, e in enumerate(pl.events) if e["kind"] == "lock_lost")
  want = max(int(D.need_mph(CORNER_K)[0]), 55 - 15)
  assert t_lost is not None and all(p.ceil is not None and p.ceil <= want for t, s, p, v in res if 1560 <= s <= 1650)
  assert any(e["kind"] == "dead_reckon" for e in pl.events)
  assert any(p.ceil is None or p.kind == "hold" for t, s, p, v in res if s > 1650 + P.HOLD_AFTER_M + 20)


def test_warning_predicts_the_r16_cap_of_an_unconfirmed_bend(monkeypatch):
  # Job 2: the warning predicts what the ramp will DO (A11): a stage-1 bend whose preview went before reaching its entry stays
  # "unconfirmed" and capped (dash >= start - 5) to its end, so a bend needing 38 from 50 is reached at about 45: warned for.
  # The cap path with an explicit 5 (default off since 2026-11-24); with the default the same bend is reached on target, unwarned
  k = k_needing(38)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  f8 = road_preview(tr, 50)
  res = drive(mem, tr, 50, 160, closed=True, planner=pl, model_fn=lambda t, s: f8(t, s) if s < 1500 - 183 else None)
  assert "s1_unconfirmed" not in [e["kind"] for e in pl.events]
  assert next(v for t, s, p, v in res if s >= 1500) <= 38 + 1.0
  assert not any(p.alert_mph for t, s, p, v in res)
  monkeypatch.setattr(P, "S1_UNCONFIRMED_MAX_MPH", 5)
  pl = P.Planner(mem)
  f8 = road_preview(tr, 50)
  res = drive(mem, tr, 50, 160, closed=True, planner=pl, model_fn=lambda t, s: f8(t, s) if s < 1500 - 183 else None)   # start ~1314 m, reach ~1321 m
  assert "s1_unconfirmed" in [e["kind"] for e in pl.events]
  assert next(v for t, s, p, v in res if s >= 1500) > 38 + P.ALERT_MARGIN_MPH + 1      # the cap kept it fast
  assert any(p.alert_mph == int(D.need_mph(k)[0]) for t, s, p, v in res if s <= 1500)  # and the driver is told, before the bend


# ---- road matching: is the car ON the locked trace or BESIDE it (owner, 2026-11-24: wrong-road cases are solved by matching on
# position and heading, not by caps) ----------------------------------------------------------------------------------------------
def _beside(off_fn):
  """fix_fn: the car `off_fn(t, s)` = (offset m, left +; heading difference rad) from the trace, along the trace's left normal."""
  def f(t, s, fx):
    o, dh = off_fn(t, s)
    return P.Fix(True, fx.x - o * math.sin(fx.hdg), fx.y + o * math.cos(fx.hdg), M.wrap(fx.hdg + dh))
  return f


def _rm_events(pl, kind="rm_off"):
  return [e for e in pl.events if e["kind"] == kind]


def _beside_ticks(res):
  return [(t, s) for t, s, p, *_ in res if p.why == "beside the remembered road"]


def test_rm_constants_are_the_measured_ones():
  # from the noise floor on raw fixes (2026-11-24); a change here must be measured again
  assert (P.RM_TAU_S, P.RM_DIV_M, P.RM_PRED_M, P.RM_LEAD_S, P.RM_STEADY_S, P.RM_STEADY_M, P.RM_STEADY_DEG, P.RM_ABS_M) == \
    (1.0, 4.5, 5.5, 1.0, 4.0, 1.0, 1.5, 7.0)
  assert P.RM_ON


def test_rm_lane_change_does_not_trip():
  # one lane (3.7 m) over 4 s at 65 mph, then steady in the new lane: the car is still on the road, nothing changes
  tr = road((4000, 0.0))
  mem = M.synthetic([tr])
  v = 65 * MPH
  t0, T, W = 20.0, 4.0, 3.7

  def lc(t, s):
    if t < t0:
      return 0.0, 0.0
    if t >= t0 + T:
      return W, 0.0
    x = (t - t0) / T
    return W * (1 - math.cos(math.pi * x)) / 2, math.atan2(W * math.pi / (2 * T) * math.sin(math.pi * x), v)
  for sign in (1.0, -1.0):
    pl = P.Planner(mem)
    res = drive(mem, tr, 65, 50, planner=pl, fix_fn=_beside(lambda t, s: tuple(sign * q for q in lc(t, s))))
    assert not _rm_events(pl) and not _beside_ticks(res)
    assert all(p.trusted for t, s, p in res if t >= P.STABLE_S + 0.5)


def test_rm_ramp_departure_trips_within_4_s():
  # a ramp leaving the remembered highway: the offset grows 1 m per 25 m at 75 mph (heading ~2.3 deg off): beside within 4 s
  tr = road((4000, 0.0))
  mem = M.synthetic([tr])
  v = 75 * MPH
  t0 = 20.0
  for sign in (1.0, -1.0):
    pl = P.Planner(mem)
    res = drive(mem, tr, 75, 30, planner=pl,
                fix_fn=_beside(lambda t, s: (0.0, 0.0) if t < t0 else (sign * v * (t - t0) / 25.0, sign * math.atan(1 / 25.0))))
    ev = _rm_events(pl)
    assert ev and ev[0]["why"] in ("dev", "lead")
    t_off = next(t for t, s in _beside_ticks(res))
    assert t0 < t_off <= t0 + 4.0, t_off - t0
    assert pl.locked                                             # the lock keeps tracking (it is lost only when the fixes leave it)


def test_rm_steady_offset_beyond_7_m_trips_and_starts_nothing():
  # a steady 7.5 m offset (more than two lanes: another road beside the remembered one): beside from the lock, no memory slowdown for
  # the remembered corner ahead; the tick log says why
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 50, planner=pl, fix_fn=_beside(lambda t, s: (7.5, 0.0)))
  assert pl.locked and _rm_events(pl) and _rm_events(pl)[0]["why"] == "far"
  assert all(p.ceil is None and not p.trusted for t, s, p in res)
  assert not any(e["kind"] == "start" for e in pl.events)
  beside = [p for t, s, p in res if p.why == "beside the remembered road"]
  assert beside and all(p.diag.get("rm") == "far" for p in beside)
  # an ordinary lane offset (3 m, steady) is on the road: slowed for as ever, no "rm" in the log
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 50, planner=pl, fix_fn=_beside(lambda t, s: (3.0, 0.0)))
  assert not _rm_events(pl) and any(p.ceil is not None for t, s, p in res)
  assert not any("rm" in (p.diag or {}) for t, s, p in res)
  # ... and one below the limit (6.5 m) too
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 50, planner=pl, fix_fn=_beside(lambda t, s: (6.5, 0.0)))
  assert not _rm_events(pl) and any(p.ceil is not None for t, s, p in res)


def test_rm_jump_of_11_m_at_a_trace_link_trips_at_once():
  # the remembered road continues on a linked trace that lies 11 m to the side; the car drives straight on (it did not follow the
  # link): beside on the first tick the match is on the linked trace
  a = road((1000, 0.0))
  b = M.synthetic_trace([(1500, 0.0)], x0=a["x"][-1] + M.SPACING_M, y0=a["y"][-1] + 11.0, h0=a["hdg"][-1])
  mem = M.synthetic([a, b], nexts=[1, -1])
  straight = road((2500, 0.0))
  pl = P.Planner(mem)
  res = drive(mem, straight, 55, 70, planner=pl)
  first_b = next(t for t, s, p in res if p.trace == 1)
  ev = _rm_events(pl)
  assert ev and ev[0]["why"] == "link"
  t_off = next(t for t, s in _beside_ticks(res))
  assert first_b <= t_off <= first_b + DT + 1e-6, (t_off, first_b)
  assert all(p.trusted for t, s, p in res if P.STABLE_S + 0.5 <= t < first_b)
  # the same link with the linked trace in line: no trip (the reference carries across the link)
  b2 = M.synthetic_trace([(1500, 0.0)], x0=a["x"][-1] + M.SPACING_M, y0=a["y"][-1], h0=a["hdg"][-1])
  mem2 = M.synthetic([a, b2], nexts=[1, -1])
  pl = P.Planner(mem2)
  res = drive(mem2, straight, 55, 70, planner=pl)
  assert not _rm_events(pl) and pl.trace == 1 and all(p.trusted for t, s, p in res if t >= P.STABLE_S + 0.5)
  # in line but the car 3 m to the side all along: across the link the reference carries (no re-seed needed, no trip)
  pl = P.Planner(mem2)
  res = drive(mem2, straight, 55, 70, planner=pl, fix_fn=_beside(lambda t, s: (3.0, 0.0)))
  assert not _rm_events(pl) and pl.trace == 1


def test_rm_corner_in_progress_is_kept_when_it_trips():
  # the slowing for a remembered corner has started when the car's offset starts growing (a ramp the memory does not know): the
  # corner is KEPT by dead reckoning to its end (never a ceiling above its own inside the bend), and nothing new starts
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  s0 = 1380.0
  res = drive(mem, tr, 55, 160, closed=True, planner=pl,
              fix_fn=_beside(lambda t, s: (0.0, 0.0) if s < s0 else (min(12.0, (s - s0) / 25.0), math.atan(1 / 25.0) if s < s0 + 300 else 0.0)))
  kinds = [e["kind"] for e in pl.events]
  assert "rm_off" in kinds and "start" in kinds and kinds.index("start") < kinds.index("rm_off")
  i_off = kinds.index("rm_off")
  assert pl.events[i_off + 1]["kind"] == "dead_reckon" and pl.events[i_off + 1]["why"] == "beside the remembered road"
  assert "start" not in kinds[i_off:]
  want = max(int(D.need_mph(CORNER_K)[0]), 55 - 15)
  assert all(p.ceil is not None and p.ceil <= want for t, s, p, v in res if 1560 <= s <= 1650)
  assert next(v for t, s, p, v in res if s >= 1500) <= want + 1.0
  assert any(p.ceil is None or p.kind == "hold" for t, s, p, v in res if s > 1650 + P.HOLD_AFTER_M + 20)


def test_rm_returning_to_the_trace_restores_trust():
  # out to 6 m (trips), then back on the trace and steady: trusted again once steady RM_STEADY_S, not before. Steady is judged against
  # the 3 s average (RM_SLOW_TAU_S), which itself needs a few seconds to come back: about 7.6 s from settling here
  tr = road((4000, 0.0))
  mem = M.synthetic([tr])
  v = 60 * MPH
  t0, t1, t2 = 20.0, 24.0, 27.0           # out over 4 s, back over 3 s, then on the trace

  def f(t, s):
    if t < t0 or t >= t2:
      return 0.0, 0.0
    if t < t1:
      return 6.0 * (t - t0) / (t1 - t0), math.atan2(6.0 / (t1 - t0), v)
    return 6.0 * (t2 - t) / (t2 - t1), -math.atan2(6.0 / (t2 - t1), v)
  pl = P.Planner(mem)
  res = drive(mem, tr, 60, 50, planner=pl, fix_fn=_beside(f))
  assert _rm_events(pl) and _rm_events(pl, "rm_on")
  b = _beside_ticks(res)
  t_back = next(t for t, s, p in res if t > b[0][0] and p.why != "beside the remembered road")
  assert t2 + P.RM_STEADY_S - DT <= t_back <= t2 + P.RM_STEADY_S + 4.0, t_back - t2
  assert all(p.trusted for t, s, p in res if t >= t_back)
  assert pl.locked
