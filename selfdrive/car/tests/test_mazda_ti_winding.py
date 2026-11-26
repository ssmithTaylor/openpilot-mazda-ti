"""Winding-road closed-loop sweep (mazda_ti_winding_gen): runs every family's deterministic scenarios and writes one JSON per chunk to
$WINDING_OUT (default E:/openpilot-mazda-ti-data/winding-tests/out). The sweep itself is not an assertion suite: the findings are read
by tools/mazda_sim/winding_report.py. The full sweep (test_winding_chunk) and the probe are marked slow: run on demand with
pc_test.py --slow -n 2 -- <abs path to this file> -k chunk. The normal `pc_test.py --curve` run executes only REPRO: the fixed repro
cases of the failure classes found by the sweep (a few seconds each), with each class's assertion."""
import importlib.util
import json
import math
import os
import pathlib
import time

import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location("mazda_ti_winding_gen", pathlib.Path(__file__).parent / "mazda_ti_winding_gen.py")
G = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(G)

OUT = pathlib.Path(os.environ.get("WINDING_OUT", "E:/openpilot-mazda-ti-data/winding-tests/out"))
N = int(os.environ.get("WINDING_N", "24"))          # scenarios per family
CHUNK = int(os.environ.get("WINDING_CHUNK", "4"))
FAMILIES = ("F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8")
CASES = [(fam, c) for fam in FAMILIES for c in range(0, N, CHUNK)]


@pytest.mark.slow
@pytest.mark.parametrize("fam,c0", CASES)
def test_winding_chunk(fam, c0):
  OUT.mkdir(parents=True, exist_ok=True)
  res = []
  for seed in range(c0, min(N, c0 + CHUNK)):
    t0 = time.time()
    sc = G.gen(fam, seed)
    r = G.run(sc)
    r["wall_s"] = round(time.time() - t0, 1)
    r["scenario"] = sc
    res.append(r)
  (OUT / f"{fam}_{c0:03d}.json").write_text(json.dumps(res, default=str))


@pytest.mark.slow
def test_one_probe():
  """a single quick scenario, timed (smoke)"""
  t0 = time.time()
  r = G.run(G.gen(os.environ.get("WINDING_PROBE_FAM", "F2"), int(os.environ.get("WINDING_PROBE_SEED", "0"))))
  print("PROBE", round(time.time() - t0, 1), json.dumps({k: v for k, v in r.items() if k != "bends"}, default=str)[:1500])
  for b in r.get("bends", []):
    print(b)
  assert r.get("exc") is None, r["exc"]


def test_dump():
  """WINDING_DUMP=F2:0 -> out/dump_F2_0.json with the 10 Hz rows, set-speed changes, bends and events"""
  spec = os.environ.get("WINDING_DUMP")
  if not spec:
    pytest.skip("WINDING_DUMP not set")
  OUT.mkdir(parents=True, exist_ok=True)
  for one in spec.split(","):
    fam, seed = one.split(":")
    r = G.run(G.gen(fam, int(seed)), keep_rows=True)
    (OUT / f"dump_{fam}_{seed}.json").write_text(json.dumps(r, default=str))


# ---- the fixed repro subset (run by pc_test.py --curve) ---------------------------------------------------------------------------
REPRO = (("F1", 61), ("F1", 18), ("F2", 0), ("F1", 3), ("F5", 16))
_RES = {}


def _run(fam, seed):
  if (fam, seed) not in _RES:
    _RES[(fam, seed)] = G.run(G.gen(fam, seed), keep_rows=True)
  return _RES[(fam, seed)]


@pytest.mark.parametrize("fam,seed", REPRO)
def test_repro_runs(fam, seed):
  """every repro case runs to the road end without a planner / car-side exception"""
  r = _run(fam, seed)
  assert r.get("exc") is None, r["exc"]
  assert r["finished"]


def _bend_min_mph(r, b):
  a, e = r["bl"][b["i"]][0], r["bl"][b["i"]][1]
  return min(row[3] for row in r["rows"] if a <= row[1] <= e)


def _from_cruise(r, b, before_m=250.0):
  """the car was at the driver's set speed somewhere in the before_m before the bend (it arrives from cruise, not from a bend)"""
  a = r["bl"][b["i"]][0]
  return any(row[3] >= r["set"] - 0.5 for row in r["rows"] if a - before_m <= row[1] < a)


@pytest.mark.parametrize("fam,seed", (("F1", 61), ("F1", 18)))
def test_class1_record_bound_kept_for_small_drops(fam, seed):
  """CLASS 1: a remembered bend whose RECORD allows a speed only a few mph (<= LATE_SHED_MPH) under the set speed keeps that bound
  whichever stage would handle it (F1/61: set 35, record bends allowed 32 were all driven at 35; F1/18: set 45, the set speed
  walked up to 43 inside a record bend allowed 40). Every such bend: no SET+ inside it above its allowed speed; one reached from
  cruise: arrival at or under allowed + 1 mph and graded (the car is not taken more than 1.5 mph under the allowed speed)."""
  r = _run(fam, seed)
  small = [b for b in r["bends"] if not b["drv"] and b["needed"] and r["set"] - b["allowed"] <= 5 and b["allowed"] >= b["floor"]]
  assert small, "the case must contain small-drop record bends"
  for b in small:
    assert b["bad_up"] == 0, b
    if _from_cruise(r, b):
      assert b["peak"] <= b["allowed"] + 1.0, b
      nxt = [(a2, al2) for (a2, *_x), al2 in zip(r["bl"], r["allowed"]) if 0 < a2 - r["bl"][b["i"]][1] <= 150.0]
      if not any(al2 < b["allowed"] for _a, al2 in nxt):       # graded, unless a lower bend follows at once
        assert _bend_min_mph(r, b) >= b["allowed"] - 1.5, (b, _bend_min_mph(r, b))


@pytest.mark.parametrize("fam,seed", (("F2", 0), ("F1", 3)))
def test_class23_no_yoyo_between_bends_and_arrival(fam, seed):
  """CLASSES 2+3: between bends the restore climbs only to what the next bend allows by the cruise model, against the bend's position
  (F2/0: 46 -> 54 with 8 SET+, then 54 -> 34 within 4 s; F1/3: the next record bend allowed 37 reached at 39.1, no warning). No SET+
  followed by a SET- within 5 s anywhere, and in F1/3 every bend the feature may slow for is reached at or under allowed + 1 mph."""
  r = _run(fam, seed)
  assert r["yoyo"] == 0, (r["yoyo"], r["restore_undone"])
  if fam == "F1":
    for b in r["bends"]:
      if not b["drv"] and b["allowed"] >= b["floor"]:
        assert b["peak"] <= b["allowed"] + 1.0, b


@pytest.mark.parametrize("fam,seed", (("F5", 16),))
def test_class5_record_bends_kept_through_a_position_gap(fam, seed):
  """CLASS 5: a 10-60 s loss of position (fixes absent / invalid) turned record bends into table-only bends, unwarned (F5/16: set 55,
  record, a 60 s gap: a bend allowed 32 driven at 42 with no warning). The planner keeps its place by the odometer while the yaw
  agrees: no lock loss, and every bend is reached at or under max(allowed, floor) + 1 mph."""
  r = _run(fam, seed)
  assert not any(e["kind"] == "lock_lost" for e in r["pl_events"]), [e for e in r["pl_events"] if e["kind"] == "lock_lost"]
  for b in r["bends"]:
    if not b["drv"]:
      assert b["peak"] <= max(b["allowed"], b["floor"]) + 1.0, b


def _dr_drive(fix_at, yaw_at, segs, mph=50.0, seconds=None):
  """open-loop planner drive at a constant speed over the synthetic road `segs`: fix_at(s, (x, y, h)) -> Fix; yaw_at(s, k) -> live k"""
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P
  import numpy as np
  tr = M.synthetic_trace(segs)
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  s_road = np.arange(len(tr["x"])) * M.SPACING_M
  v = mph * P.MPH
  end = s_road[-1] - 50.0
  out = []
  t = 0.0
  while v * t < end:
    s = v * t
    i = min(int(s // M.SPACING_M), len(s_road) - 1)
    xyh = (float(np.interp(s, s_road, tr["x"])), float(np.interp(s, s_road, tr["y"])), float(tr["hdg"][i]))
    p = pl.step(t, fix_at(s, xyh), v, set_mph=mph, yaw_k=yaw_at(s, float(tr["kap"][i])), ref_mph=mph)
    out.append((s, p, any(e["kind"] == "dr_off" for e in pl.events[-3:])))
    t += 0.2
  return pl, out


def test_class5_car_leaving_the_road_in_a_gap_stops_dead_reckoning():
  """DR_*: the fixes go at 1000 m and the car takes a straight road off the remembered one, which bends at 1200 m: the dead
  reckoning ends (the yaw disagrees with the remembered bend) within DR_WIN_M + DR_BAD_M of the bend's start."""
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  segs = [(1200.0, 0.0), (200.0, 0.01), (1500.0, 0.0)]
  pl, out = _dr_drive(lambda s, f: P.Fix(True, *f) if s < 1000.0 else P.Fix(False),
                      lambda s, k: k if s < 1000.0 else 0.0, segs)
  off = [e for e in pl.events if e["kind"] == "dr_off"]
  assert off, [e for e in pl.events if e["kind"] in ("lock_lost", "dr_off")]
  s_off = next(s for s, p, o in out if o)
  assert 1200.0 <= s_off <= 1200.0 + P.DR_WIN_M + P.DR_BAD_M, s_off
  # on the agreeing road (the yaw as remembered) the same gap keeps the lock to the end of the bend and beyond
  pl2, out2 = _dr_drive(lambda s, f: P.Fix(True, *f) if s < 1000.0 else P.Fix(False), lambda s, k: k, segs)
  assert not any(e["kind"] == "dr_off" for e in pl2.events)
  assert [e["why"] for e in pl2.events if e["kind"] == "lock_lost"] == ["no position fix: dead reckoning limit"]
  assert all(p.trusted for s, p, o in out2 if 1000.0 <= s <= 1000.0 + P.DR_MAX_M - 10.0)


def test_class5_fixes_returning_12_m_sideways_start_no_slowdown():
  """DR_*: fixes gone 1000-1600 m, then back 12 m sideways (the gorge offset case) on a straight with a remembered bend 400 m on: the
  return starts no slowdown and publishes no corner ceiling before the bend's own approach could (no jump, no re-match start)."""
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  import math
  segs = [(2000.0, 0.0), (150.0, 0.012), (1500.0, 0.0)]

  def fix_at(s, f):
    if s < 1000.0:
      return P.Fix(True, *f)
    if s < 1600.0:
      return P.Fix(False)
    x, y, h = f
    return P.Fix(True, x - math.sin(h) * 12.0, y + math.cos(h) * 12.0, h)
  pl, out = _dr_drive(fix_at, lambda s, k: k, segs)
  starts_m = [s for s, p, o in out if 1600.0 <= s <= 1700.0 and p.ceil is not None and p.kind == "corner"]
  assert not starts_m, starts_m
  assert all(p.trusted or p.why == "beside the remembered road" for s, p, o in out if 1000.0 <= s <= 1700.0), \
    [(round(s), p.why) for s, p, o in out if 1000.0 <= s <= 1700.0 and not p.trusted][:5]


def test_i2_floor_bound_push_in_a_chain_f7_52():
  """I2 (gorge-fix): F7/52: after a restore to the hold ceiling 41 inside a chain (the push net had acted: the car side keeps the dash
  at the floor 35), a bend allowed 34 was entered at 36.7 with the warning 1.3 s before entry: the planner planned a push to 29 the
  dash could not take. Every bend: at or under max(allowed, floor) + 1 mph; any warning there comes >= 4 s before entry."""
  r = _run("F7", 52)
  for b in r["bends"]:
    if not b["drv"]:
      assert b["peak"] <= max(b["allowed"], b["floor"]) + 1.0, b
      assert b["warn_lead_s"] is None or b["warn_lead_s"] >= 4.0, b


def test_i2_push_floor_is_the_floor_inside_an_intervention():
  """I2 unit: the push may go DRIVE_GAP_MPH under the floor (plan A10) only when no intervention is in progress; with the dash below
  the driver's set speed after a corner of the intervention has passed (a chain) it is planned at the floor, and the start for the same bend comes earlier (no shorter approach)."""
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P, ti_cruise_model as CM
  pl = P.Planner(None)
  pl.step(0.0, P.Fix(False), 41 * P.MPH, set_mph=41, ref_mph=50, floor_mph=35)
  assert not pl.chain                                  # lowered, but no corner of the intervention passed yet: A10's push
  pl.int_passed = True                                 # (as _ramp sets it when a corner of the intervention is passed)
  pl.step(0.1, P.Fix(False), 41 * P.MPH, set_mph=41, ref_mph=50, floor_mph=35)
  assert pl.chain and pl._push_floor(35) == 35
  pl.step(0.2, P.Fix(False), 50 * P.MPH, set_mph=50, ref_mph=50, floor_mph=35)
  assert not pl.chain and pl._push_floor(35) == CM.push_floor(35, P.DRIVE_GAP_MPH) < 35
  assert P.approach_m(41, 35, 35) > P.approach_m(41, 35, 29)


# ---- I1 (gorge-fix): yaw trust with offset fixes ------------------------------------------------------------------------------------
WIND_SEGS = [(600.0, 0.0)] + [seg for i in range(14) for seg in ((110.0, 0.009 * (1 if i % 2 else -1)), (90.0, 0.0))] + [(800.0, 0.0)]


def _yt_drive(segs, fix_at, yaw_at=None, mph=50.0, switch=True, monkeypatch=None):
  """open loop at a constant speed: -> (planner, [(s, plan, pos)])"""
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P
  import numpy as np
  monkeypatch.setattr(P, "RM_YAW_TRUST", switch)
  tr = M.synthetic_trace(segs)
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  s_road = np.arange(len(tr["x"])) * M.SPACING_M
  v = mph * P.MPH
  out, t = [], 0.0
  while v * t < s_road[-1] - 50.0:
    s = v * t
    i = min(int(s // M.SPACING_M), len(s_road) - 1)
    f = (float(np.interp(s, s_road, tr["x"])), float(np.interp(s, s_road, tr["y"])), float(tr["hdg"][i]))
    k = float(tr["kap"][i])
    p = pl.step(t, fix_at(s, f), v, set_mph=mph, yaw_k=k if yaw_at is None else yaw_at(s, k), ref_mph=mph)
    out.append((s, p, pl.pos))
    t += 0.2
  return pl, out


def _offset(a, b, off):
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  import math

  def f(s, xyh):
    x, y, h = xyh
    if a <= s < b:
      return P.Fix(True, x - math.sin(h) * off, y + math.cos(h) * off, h)
    return P.Fix(True, x, y, h)
  return f


@pytest.mark.parametrize("off", (12.0, 25.0, -25.0))
def test_i1_offset_fixes_on_a_winding_trace_keep_trust(off, monkeypatch):
  """I1: fixes 12 / 25 m sideways for 60 s (1000 .. 2340 m at 50 mph) on a winding trace: the memory stays trusted (the yaw agrees),
  the lock is kept, and the corner ceilings are the clean-fix run's (within 1 mph); with RM_YAW_TRUST off the memory is untrusted."""
  b = 1000.0 + 60 * 50 * 0.44704
  pl, out = _yt_drive(WIND_SEGS, _offset(1000.0, b, off), monkeypatch=monkeypatch)
  assert not any(e["kind"] in ("lock_lost", "yt_off") for e in pl.events), [e for e in pl.events if e["kind"] in ("lock_lost", "yt_off")]
  assert all(p.trusted for s, p, q in out if 1000.0 <= s <= b + 100.0), [(round(s), p.why) for s, p, q in out if 1000 <= s <= b and not p.trusted][:4]
  _, clean = _yt_drive(WIND_SEGS, _offset(0.0, 0.0, 0.0), monkeypatch=monkeypatch)
  for (s, p, q), (s2, p2, q2) in zip(out, clean):
    if 1000.0 <= s <= b + 300.0:
      assert (p.ceil is None) == (p2.ceil is None) and (p.ceil is None or abs(p.ceil - p2.ceil) <= 1), (round(s), p.ceil, p2.ceil)
      assert abs(q - q2) * 5.0 <= 0.5 + 0.02 * (s - 1000.0), (round(s), q, q2)      # odometer only beside: within the drift
  pl0, out0 = _yt_drive(WIND_SEGS, _offset(1000.0, b, off), switch=False, monkeypatch=monkeypatch)
  assert any(not p.trusted for s, p, q in out0 if 1000.0 <= s <= b)


@pytest.mark.parametrize("off", (12.0, 25.0))
def test_i1_offset_fixes_on_a_straight_unchanged(off, monkeypatch):
  """I1: the same offsets on a straight trace: exactly as without yaw trust (the positional rule: untrusted / lock lost)."""
  segs = [(3000.0, 0.0)]
  b = 1000.0 + 60 * 50 * 0.44704
  pl1, on = _yt_drive(segs, _offset(1000.0, b, off), monkeypatch=monkeypatch)
  pl0, off_ = _yt_drive(segs, _offset(1000.0, b, off), switch=False, monkeypatch=monkeypatch)
  # (review fixes: a fix beyond the match radius now marks the car beside the trace at once while the window decides, so the switch
  # on is never LESS strict than off: trusted only where off is, and no ceiling off does not publish)
  for (s, p, q), (s0, p0, q0) in zip(on, off_):
    assert (not p.trusted or p0.trusted) and (p.ceil is None or p.ceil == p0.ceil), (round(s), p.why, p0.why, p.ceil, p0.ceil)
  assert all(not p.trusted for s, p, q in on if 1000.0 + 60.0 <= s <= b)
  assert any(not p.trusted for s, p, q in on if 1000.0 <= s <= b)


def test_i1_a_different_winding_road_untrusted_within_the_window(monkeypatch):
  """I1: fixes 12 m sideways from 1000 m; from 1600 m the car is on ANOTHER winding road (its bends the other way round): the memory
  is untrusted within YT_WIN_M of where the roads part."""
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  pl, out = _yt_drive(WIND_SEGS, _offset(1000.0, 9e9, 12.0), yaw_at=lambda s, k: k if s < 1600.0 else -k, monkeypatch=monkeypatch)
  assert all(p.trusted for s, p, q in out if 1000.0 <= s < 1600.0)
  off = [e for e in pl.events if e["kind"] == "yt_off"]
  assert off and off[0]["why"] == "yaw disagrees" and 1600.0 <= off[0]["odo"] <= 1600.0 + P.YT_WIN_M, off
  # from then on nothing new starts on the memory (corners already started run to their end, as for any trust loss)
  i_off = next(i for i, e in enumerate(pl.events) if e["kind"] == "yt_off")
  assert not any(e["kind"] == "start" for e in pl.events[i_off:]), [e for e in pl.events[i_off:] if e["kind"] == "start"]
  assert all(not p.trusted or p.why in ("corner ahead", "in the corner") for s, p, q in out if s >= off[0]["odo"])


def test_i1_offset_fixes_returning_to_truth_no_jump(monkeypatch):
  """I1: fixes 25 m sideways 1000 .. 1800 m, then true again: no jump in the along-road position (each tick moves by the distance
  driven + at most the FIX_GAIN pull of the drift: < 2 m) and no ceiling the clean-fix run does not have."""
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  pl, out = _yt_drive(WIND_SEGS, _offset(1000.0, 1800.0, 25.0), monkeypatch=monkeypatch)
  _, clean = _yt_drive(WIND_SEGS, _offset(0.0, 0.0, 0.0), monkeypatch=monkeypatch)
  step = 50 * P.MPH * 0.2 / 5.0
  for (s, p, q), (s1, p1, q1) in zip(out, out[1:]):
    if 900.0 <= s <= 2200.0:
      assert abs((q1 - q) - step) * 5.0 < 2.0, (round(s), q1 - q)
  for (s, p, q), (s2, p2, q2) in zip(out, clean):
    if 1000.0 <= s <= 2400.0 and p.ceil is not None:
      assert p2.ceil is not None and p.ceil >= p2.ceil - 1, (round(s), p.ceil, p2.ceil)


# ---- yaw-trust review (2026-11-25): the highway / ramp phantom (the reviewer's probe, as a test) -----------------------------------
def _ph_run(pre, gap, div_k, par_m, bend_k, mph, switch, monkeypatch, both=False):
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P
  monkeypatch.setattr(P, "RM_YAW_TRUST", switch)
  shared = [(800.0, 0.0)] + pre + [(gap, 0.0)]
  ramp = shared + [(70.0, div_k), (70.0, -div_k), (par_m, 0.0), (80.0, bend_k), (600.0, 0.0)]
  hwy = shared + [(2000.0, 0.0)]
  tr_r = M.synthetic_trace(ramp)
  tr_h = M.synthetic_trace(hwy)
  mem = M.synthetic([tr_r, tr_h] if both else [tr_r])
  pl = P.Planner(mem)
  split = sum(l for l, _ in shared)
  bend_at = split + 140.0 + par_m
  s_road = np.arange(len(tr_h["x"])) * M.SPACING_M
  v = mph * P.MPH
  t, starts, ceils, untr = 0.0, [], [], []
  while v * t < bend_at + 300.0:
    s = v * t
    i = min(int(s // M.SPACING_M), len(s_road) - 1)
    f = P.Fix(True, float(np.interp(s, s_road, tr_h["x"])), float(np.interp(s, s_road, tr_h["y"])), float(tr_h["hdg"][i]))
    n0 = len(pl.events)
    p = pl.step(t, f, v, set_mph=mph, yaw_k=float(tr_h["kap"][i]), ref_mph=mph)
    for e in pl.events[n0:]:
      if e["kind"] in ("start", "yt_off", "rm_off", "lock_lost", "alert", "lock"):
        starts.append((round(s - split), e["kind"], e.get("why", "")))
    if p.ceil is not None and s > split:
      ceils.append((round(s - split), p.ceil))
    if s > split and not p.trusted:
      untr.append(round(s - split))
    t += 0.2
  return starts, ceils, (untr[0] if untr else None)


PH_CASES = [
  ("S500_gap100_off10_par200_k6", [(200.0, 0.002), (200.0, -0.002)], 100.0, 0.002, 200.0, 0.006),
  ("S500_gap100_off10_par300_k8", [(200.0, 0.002), (200.0, -0.002)], 100.0, 0.002, 300.0, 0.008),
  ("S500_gap100_off7_par120_k5", [(200.0, 0.002), (200.0, -0.002)], 100.0, 0.0015, 120.0, 0.005),
  ("none_gap100_off7_par200_k6", [], 100.0, 0.0015, 200.0, 0.006),
  # name, pre-split feature, straight gap before split, divergence k (offset = k*70^2), parallel m, ramp bend k
  ("S500_gap100_off7", [(200.0, 0.002), (200.0, -0.002)], 100.0, 0.0015, 60.0, 0.012),
  ("S500_gap100_off10", [(200.0, 0.002), (200.0, -0.002)], 100.0, 0.002, 60.0, 0.012),
  ("S500_gap250_off10", [(200.0, 0.002), (200.0, -0.002)], 250.0, 0.002, 90.0, 0.012),
  ("bend400_gap150_off7", [(150.0, 0.0025)], 150.0, 0.0015, 60.0, 0.012),
  ("none_gap100_off7", [], 100.0, 0.0015, 60.0, 0.012),
]



@pytest.mark.parametrize("name,pre,gap,dk,par,bk", PH_CASES)
@pytest.mark.parametrize("mph", (70.0, 55.0))
@pytest.mark.parametrize("both", (False, True))
def test_yt_ramp_phantom_same_as_switch_off(name, pre, gap, dk, par, bk, mph, both, monkeypatch):
  """memory holds the ramp (and maybe the highway); the car drives straight on the highway past the split: with yaw trust on, no
  ceiling the switch-off planner does not publish (no phantom), and untrusted no later than with it off + one evaluation step"""
  on = _ph_run(pre, gap, dk, par, bk, mph, True, monkeypatch, both)
  off = _ph_run(pre, gap, dk, par, bk, mph, False, monkeypatch, both)
  assert on[1] == off[1], (on[1][:4], off[1][:4], on[0][:8])
  assert (on[2] is None) == (off[2] is None) and (on[2] is None or on[2] <= off[2] + 15), (on[2], off[2])


def test_yt_parallel_road_30_m_away_with_other_bends_untrusted(monkeypatch):
  """yaw-trust review: from 1000 m the car is on a parallel same-direction road 30 m to the side whose bends differ (the remembered
  bends negated): nothing new starts on the memory, and once the corners already started are passed it is untrusted."""
  pl, out = _yt_drive(WIND_SEGS, _offset(1000.0, 9e9, 30.0), yaw_at=lambda s, k: k if s < 1000.0 else -k, monkeypatch=monkeypatch)
  n_start = [i for i, e in enumerate(pl.events) if e["kind"] == "rm_off"]
  assert n_start and not any(e["kind"] == "start" for e in pl.events[n_start[0]:]), [e for e in pl.events if e["kind"] == "start"][-3:]
  assert all(p.why not in ("corner ahead", "in the corner", "no known corner ahead") or not p.trusted for s, p, q in out if s >= 1400.0)
  assert not any(p.trusted and p.why == "no known corner ahead" for s, p, q in out if s >= 1150.0)


def test_yt_3_km_gorge_offset_throughout_trust_held(monkeypatch):
  """yaw-trust review: fixes 25 m sideways from 700 m to 3600 m (2.9 km, past DR_MAX_M): trust comes within half a window plus two
  evaluations of the onset, the along-road position is re-anchored on the straights (D), and it is never lost for good: beside but
  untrusted on under 10 % of the stretch, none in its last 300 m, and the lock is never lost."""
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  pl, out = _yt_drive(WIND_SEGS, _offset(700.0, 3600.0, 25.0), monkeypatch=monkeypatch)
  assert not any(e["kind"] == "lock_lost" for e in pl.events)
  a = 700.0 + 0.5 * P.YT_WIN_M + 2 * P.YT_EVAL_M + 20.0
  sel = [p for s, p, q in out if a <= s <= 3600.0]
  bad = [p for p in sel if p.why == "beside the remembered road"]
  assert len(bad) <= 0.1 * len(sel), (len(bad), len(sel))
  assert not any(p.why == "beside the remembered road" for s, p, q in out if 3300.0 <= s <= 3600.0)
  _, clean = _yt_drive(WIND_SEGS, _offset(0.0, 0.0, 0.0), monkeypatch=monkeypatch)
  for (s, p, q), (s2, p2, q2) in zip(out, clean):
    if a <= s <= 3600.0 and p.why != "beside the remembered road":
      assert abs(q - q2) * 5.0 <= 8.0, (round(s), round((q - q2) * 5.0, 1))          # along-road error held by the re-anchoring


def test_yt_decision_in_the_tick_record_bounded_and_position_free(monkeypatch):
  """Replayability (owner, 2026-11-25): the yaw-trust decision (trusted or not, why, its distances) is in every tick record while the
  car is beside the trace; bounded; no position; absent with RM_YAW_TRUST off (the switch-off record is integ2's)."""
  import json
  from openpilot.selfdrive.car.mazda import ti_curve_plannerd as D
  b = 1000.0 + 60 * 50 * 0.44704
  pl, out = _yt_drive(WIND_SEGS, _offset(1000.0, b, 25.0), monkeypatch=monkeypatch)
  beside = [(s, p) for s, p, q in out if (p.diag or {}).get("rm")]
  assert beside, "the offset fixes never put the car beside the trace"
  whys = set()
  for s, p in beside:
    yt = p.diag.get("yt")
    assert isinstance(yt, list) and len(yt) == 5 and yt[0] in (0, 1) and yt[1] in ("ok", "pre", "lapsed", "none"), (round(s), yt)
    assert (yt[0] == 1) == (yt[1] == "ok") and yt[4] >= 0
    rec = D.tick_log(p, {"seq": 1, "v": 22.0})
    assert rec["yt"] == yt and len(json.dumps(rec["yt"])) <= 40
    whys.add(yt[1])
  assert "ok" in whys and whys - {"ok"}, whys          # both the trusted decision and the wait for the window are logged
  _, off = _yt_drive(WIND_SEGS, _offset(1000.0, b, 25.0), switch=False, monkeypatch=monkeypatch)
  assert not any("yt" in (p.diag or {}) for s, p, q in off)
  assert not any("yt" in D.tick_log(p, {"seq": 1}) for s, p, q in off)
