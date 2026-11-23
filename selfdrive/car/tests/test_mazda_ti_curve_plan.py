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


def drive(mem, trace, v_mph, seconds, start=0, fix_fn=None, yaw_fn=None, set_mph=None, planner=None, model_fn=None):
  """Drive along `trace` (a synthetic_trace dict) at constant speed; returns [(t, along m, Plan)]."""
  pl = planner or P.Planner(mem)
  v = v_mph * MPH
  xs, ys, hs, ks = trace["x"], trace["y"], trace["hdg"], trace["kap"]
  s_road = np.arange(len(xs)) * M.SPACING_M
  out = []
  n = int(seconds / DT)
  for k in range(n):
    t = k * DT
    s = start + v * t
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
    out.append((t, s, pl.step(t, fix, v, set_mph=set_mph if set_mph is not None else v_mph, yaw_k=yk, model_k=mk)))
  return out


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
  assert P.envelope(25) == 1.14 and P.envelope(30) == 1.14 and P.envelope(35) == 2.12
  assert P.envelope(50) == 2.42 and P.envelope(65) == 2.13
  assert P.MARGIN <= 0.88


def test_flagged_point_at_least_10_below():
  kap = np.array([0.0, 0.0])
  flags = np.array([M.FLAG_LIMIT, M.FLAG_TAKEOVER], np.uint8)
  vf = np.array([50 * MPH, 40 * MPH])
  need = P.needed_mph(kap, flags, vf)
  assert need[0] <= 40 and need[1] <= 30


def test_slow_distance_grows_with_drop():
  d = [P.slow_distance_m(55, x) for x in (54, 50, 45, 40)]
  assert all(b > a for a, b in zip(d, d[1:]))
  # 15 mph from 55: 13.5 s measured allowance at ~47 mph average plus 1.5 s at 55: about 320 m, and not much more (as late as possible)
  assert 280 < d[-1] < 360


def test_shed_time_matches_measurements():
  assert abs(P.shed_time_s(5) - 5.2) < 1e-9 and abs(P.shed_time_s(10) - 8.5) < 1e-9 and abs(P.shed_time_s(15) - 13.5) < 1e-9
  ts = [P.shed_time_s(x) for x in np.linspace(0, 25, 60)]
  assert all(b >= a for a, b in zip(ts, ts[1:]))
  # never assumes more than ~2 mph/s after the reaction
  for d in (1, 3, 5, 8, 12, 15, 20):
    assert d / (P.shed_time_s(d) - P.REACTION_S) <= 2.0 + 1e-9


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
  assert any(p.trusted for _, _, p in res)
  assert all(p.why in ("no known corner ahead", "matching the remembered road", "road not known here") for _, _, p in res)


def test_trust_needs_stable_match():
  tr, mem = single_corner()
  res = drive(mem, tr, 55, 10)
  first = next(t for t, _, p in res if p.trusted)
  assert first >= P.STABLE_S - 1e-6


def test_single_corner_slows_early_and_releases():
  tr, mem = single_corner()
  res = drive(mem, tr, 55, 120)
  need = P.corner_mph(CORNER_K)[0]
  on = [(t, s, p) for t, s, p in res if p.ceil is not None]
  assert on, "no ceiling before a corner that needs slowing"
  assert all(p.ceil == need for _, _, p in on)
  start_s = on[0][1]
  assert 1500 - start_s >= P.slow_distance_m(55, need) - 2 * 55 * MPH * DT - 1
  # held through the corner, released after it
  in_corner = [p for t, s, p in res if 1500 <= s <= 1650]
  assert all(p.ceil == need for p in in_corner)
  after = [p for t, s, p in res if s > 1650 + P.HOLD_AFTER_M + 15]
  assert after and all(p.ceil is None and p.past for p in after)


def test_corner_right_after_long_straight_is_seen_far_ahead():
  # a sharp corner (needs ~25 mph) after 2 km of straight at 70 mph: the ceiling appears >= 500 m before it
  tr = road((2500, 0.0), (80, 1 / 60.0), (500, 0.0))
  mem = M.synthetic([tr])
  res = drive(mem, tr, 70, 60)
  on = [s for t, s, p in res if p.ceil is not None]
  assert on and 2500 - on[0] >= 500


def test_s_bends_ceiling_is_the_lowest_due():
  tr = road((1500, 0.0), (100, 1 / 200.0), (60, 0.0), (100, -1 / 120.0), (800, 0.0))
  mem = M.synthetic([tr])
  res = drive(mem, tr, 60, 60)
  need2 = P.corner_mph(1 / 120.0)[0]
  in_bends = [p.ceil for t, s, p in res if 1500 <= s <= 1660]
  assert in_bends and all(c is not None and c <= P.corner_mph(1 / 200.0)[0] for c in in_bends)
  assert min(in_bends) == need2


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
  res = drive(mem, whole, 55, 50)
  on = [s for t, s, p in res if p.ceil is not None]
  assert on and on[0] < 1000 + 600, "the corner on the next trace was not seen from the first"
  memb = M.synthetic([a, b], nexts=[-1, -1])
  res2 = drive(memb, whole, 55, 50)
  assert not any(p.ceil is not None for t, s, p in res2 if s < 990)


def test_model_preview_sharper_wins():
  tr, mem = single_corner()
  # memory corner needs ~39 mph; the model shows a 1/80 bend 4 s ahead while in the corner region -> lower ceiling
  def model(t, s):
    if 1300 <= s <= 1600:
      return (np.array([0.0, 2.0, 4.0, 6.0]), np.array([0.0, 0.002, 1 / 80.0, 0.002]))
    return (np.array([0.0, 2.0, 4.0]), np.zeros(3))
  res = drive(mem, tr, 55, 60, model_fn=model)
  ceils = [p.ceil for t, s, p in res if p.ceil is not None]
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
  assert set(d) == {"m", "seq", "mode", "ceil", "conf", "trusted", "known", "why", "dist", "past", "model_la", "capped", "limit"}


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
