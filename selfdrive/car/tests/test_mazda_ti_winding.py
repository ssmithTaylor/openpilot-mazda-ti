"""Winding-road closed-loop sweep (mazda_ti_winding_gen): runs every family's deterministic scenarios and writes one JSON per chunk to
$WINDING_OUT (default E:/openpilot-mazda-ti-data/winding-tests/out). The sweep itself is not an assertion suite: the findings are read
by tools/mazda_sim/winding_report.py. The full sweep (test_winding_chunk) and the probe are marked slow: run on demand with
pc_test.py --slow -n 2 -- <abs path to this file> -k chunk. The normal `pc_test.py --curve` run executes only REPRO: the fixed repro
cases of the failure classes found by the sweep (a few seconds each), with each class's assertion."""
import importlib.util
import json
import os
import pathlib
import time

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
