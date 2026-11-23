"""Curve slowing planner (ti_curve_plan.py) on SYNTHETIC roads only (no recorded position is ever used in a test)."""
import math

import numpy as np
import pytest

from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.selfdrive.car.mazda import ti_curve_plan as P

MPH = P.MPH
DT = 0.2


def road(*segments, **kw):
  return M.synthetic_trace(list(segments), **kw)


def drive(mem, trace, v_mph, seconds, start=0, fix_fn=None, yaw_fn=None, set_mph=None, planner=None, model_fn=None, closed=False,
          rate=1.0):
  """Drive along `trace` (a synthetic_trace dict); returns [(t, along m, Plan, mph)]. Open loop: constant speed. closed=True: the set
  speed follows the plan's ceiling at once (a corner ceiling lowers it, a hold ceiling caps the way back up to the driver's v_mph), the
  speed answers it REACTION_S later with `rate` times the measured deceleration and speeds back up at 0.7 mph/s."""
  pl = planner or P.Planner(mem)
  v = float(v_mph)
  ref = set_mph if set_mph is not None else v_mph
  xs, ys, hs, ks = trace["x"], trace["y"], trace["hdg"], trace["kap"]
  s_road = np.arange(len(xs)) * M.SPACING_M
  out = []
  hist = []
  s = float(start)
  setv = ref
  n = int(seconds / DT)
  lag = int(round(P.REACTION_S / DT))
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
      setv = ref if p.ceil is None else min(ref, p.ceil)
      hist.append(setv)
      eff = hist[-1 - lag] if len(hist) > lag else ref
      if v > eff:
        v = max(eff - 0.0, v - rate * P.decel_mph_s(v - eff) * DT)
      else:
        v = min(eff, v + 0.7 * DT)
    s += v * MPH * DT
  return [(t, s_, p) for t, s_, p, _ in out] if not closed else out


# ---- the speed law -------------------------------------------------------------------------------------------------------------
def test_corner_mph_monotonic_and_fits():
  ks = np.linspace(0.0, 0.05, 400)
  sp = P.corner_mph(ks)
  assert np.all(np.diff(sp) <= 0)                      # sharper never allows more speed
  for k, s in zip(ks, sp):
    if s > P.CAR_MIN_MPH:
      assert P.fits(k, s)
      assert not any(P.fits(k, r) for r in range(int(s) + 1, P.MAX_MPH + 1))


def test_corner_mph_examples():
  # a 200 m radius bend: 2.42*0.85 = 2.06 m/s^2 -> about 45 mph
  s = P.corner_mph(1 / 200.0)[0]
  assert 40 <= s <= 46
  assert P.corner_mph(0.0)[0] == P.MAX_MPH
  assert P.corner_mph(1.0)[0] == P.CAR_MIN_MPH


def test_envelope_conservative_end():
  # the lower of the two measurements per band
  # the measured limits (lateral-sensors-20261122); the 28-31 mph handover keeps the 19-27 figure; 90 % of them
  assert P.envelope(25) == 1.41 and P.envelope(30) == 1.41 and P.envelope(35) == 2.20
  assert P.envelope(50) == 2.42 and P.envelope(65) == 2.13
  assert P.MARGIN <= 0.90


def test_flagged_point_at_least_10_below():
  kap = np.array([1 / 300.0, 1 / 200.0, 0.0])               # real bends at the failing speeds (> 1 m/s^2), and a straight
  flags = np.array([M.FLAG_LIMIT, M.FLAG_TAKEOVER, M.FLAG_LANE], np.uint8)
  vf = np.array([50 * MPH, 40 * MPH, 50 * MPH])
  need = P.needed_mph(kap, flags, vf)
  assert need[0] <= 40 and need[1] <= 30
  assert need[2] == P.MAX_MPH                                # a lane-line touch on a straight is not a corner


def test_slow_distance_grows_with_drop():
  d = [P.slow_distance_m(55, x) for x in (54, 50, 45, 40)]
  assert all(b > a for a, b in zip(d, d[1:]))


def test_response_model_saturates_at_6_mph():
  assert P.decel_mph_s(0) == 0 and P.decel_mph_s(2) == P.decel_mph_s(3) == 0.76
  assert P.decel_mph_s(6) == P.decel_mph_s(12) == P.SAT_MPH_S
  assert P.SAT_MPH_S * MPH <= 0.75                     # ~0.7 m/s^2 mean: never assumes more
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
  need = P.corner_mph(CORNER_K)[0]
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
  need = max(P.corner_mph(CORNER_K)[0], 55 - 15)       # never more than 15 mph below the speed (no known limit)
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
  need = P.corner_mph(CORNER_K)[0]                     # 36: floor 50 binds at 65 mph; use a faster start to see the push
  ceils = [p.ceil for t, s, p, v in res if p.kind == "corner"]
  T = max(need, 50)
  assert min(ceils) == max(T - P.DRIVE_GAP_MPH, 50) and ceils[-1] == T
  res = drive(mem, tr, 50, 160, closed=True)
  ceils = [p.ceil for t, s, p, v in res if p.kind == "corner"]
  assert min(ceils) == max(need - P.DRIVE_GAP_MPH, 35) and ceils[-1] == need   # pushed 1 below, then raised to the target


def test_downhill_slow_car_is_flagged_not_pushed_lower():
  tr, mem = single_corner()
  res = drive(mem, tr, 50, 160, closed=True, rate=0.5)  # 50 mph: floor 35, below the corner's own speed (the floor does not bind)
  need = P.corner_mph(CORNER_K)[0]
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
  assert on and 2500 - on[0] >= P.approach_m(70, 55, 55) - 10


def test_s_bends_both_on_target():
  tr = road((1500, 0.0), (100, 1 / 200.0), (60, 0.0), (100, -1 / 120.0), (800, 0.0))
  mem = M.synthetic([tr])
  res = drive(mem, tr, 60, 200, closed=True)
  n1, n2 = P.corner_mph(1 / 200.0)[0], P.corner_mph(1 / 120.0)[0]
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


def test_model_preview_sharper_wins():
  tr, mem = single_corner()
  # memory corner needs ~39 mph; the model shows a 1/80 bend 4 s ahead while in the corner region -> lower ceiling
  def model(t, s):
    if 900 <= s <= 1150:
      return (np.array([0.0, 2.0, 4.0, 6.0]), np.array([0.0, 0.002, 1 / 80.0, 0.002]))
    return (np.array([0.0, 2.0, 4.0]), np.zeros(3))
  res = drive(mem, tr, 50, 80, model_fn=model)
  ceils = [p.corner_mph for t, s, p in res if p.ceil is not None and p.kind == "corner"]
  assert min(ceils) < P.corner_mph(CORNER_K)[0]
  assert any(p.model_la is not None for t, s, p in res)


def test_no_memory():
  pl = P.Planner(None)
  p = pl.step(0.0, P.Fix(True, 0, 0, 0), 20.0)
  assert p.ceil is None and not p.trusted and "no road memory" in p.why


def test_plan_json_has_no_position():
  tr, mem = single_corner()
  res = drive(mem, tr, 55, 30)
  d = P.plan_json(res[-1][2], 12.0, 7, "shadow")
  assert set(d) == {"m", "seq", "mode", "ceil", "conf", "trusted", "known", "why", "dist", "past", "model_la", "capped", "limit", "model_k", "kind", "corner", "end", "behind", "alert", "need"}


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


def test_proven_speed_means_no_slowing_and_trouble_wins():
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  n = len(tr["x"])
  need = P.corner_mph(CORNER_K)[0]
  mem = M.synthetic([tr])
  mem.v_proven = np.full(n, 55 * MPH)                   # earlier passes took it at 55 without trouble
  res = drive(mem, tr, 52, 120, planner=P.Planner(mem))
  assert all(p.ceil is None for _, _, p in res)        # at or below the proven speed: no slowing
  res = drive(mem, tr, 62, 120, planner=P.Planner(mem))
  assert min(p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner") >= 55   # never below proven
  # the same corner with a trouble flag: proven ignored, 10 below the failing speed
  mem2 = M.synthetic([tr], flags=[np.full(n, M.FLAG_TAKEOVER, np.uint8)], v_flag=[np.full(n, 50 * MPH)])
  mem2.v_proven = np.full(n, 55 * MPH)
  res = drive(mem2, tr, 52, 120, planner=P.Planner(mem2))
  assert any(p.ceil is not None and p.corner_mph <= 40 for _, _, p in res)


def test_single_pass_corner_is_slowed_for_like_any_other():
  # owner, 2026-11-23: no single-pass trim; a corner from one pass gets its full need (within the floor), model or not
  tr = road((1500, 0.0), (150, CORNER_K), (800, 0.0))
  mem = M.synthetic([tr])
  mem.npass = np.ones(mem.n, np.int32)
  res = drive(mem, tr, 50, 120, planner=P.Planner(mem))
  assert min(p.corner_mph for _, _, p in res if p.ceil is not None and p.kind == "corner") == max(P.corner_mph(CORNER_K)[0], 50 - 15)


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
  need = P.corner_mph(k)[0]
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
  for v in (45, 55, 65):
    tr, mem = single_corner()
    res = drive(mem, tr, v, 200, closed=True)
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
