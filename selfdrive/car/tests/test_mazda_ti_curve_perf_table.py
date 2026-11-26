"""Planner cycle time (perf 2026-11-25: with a bend in play the step was 81-121 ms at p99 on the device, 95 % of it cold cruise-model
simulations on approach-table misses for shallow pushes, and the raise thresholds' bisection). The table now also holds the shallow
pushes (T - S1 below DRIVE_GAP_MPH, ported from ramp2 f27ce822) and the raise thresholds for the set speeds this car's cruise reports;
both are caches only: IDENTICAL to the cold computation for the same key. Cold simulations are counted and logged per tick."""
import math

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.selfdrive.car.mazda import ti_curve_plan as P
from openpilot.selfdrive.car.mazda import ti_curve_plannerd as D


def test_shallow_pushes_are_table_keys():
  keys = set(P.approach_table_keys())
  pf = P.CM.push_floor(P.CAR_MIN_MPH, P.DRIVE_GAP_MPH)
  for T in (25, 33, 41, 47, 60, 72):
    for d in range(0, P.DRIVE_GAP_MPH + P.APPROACH_TABLE_SHED_MPH + 1):
      S1 = max(pf, T - d)
      for u in (T + 1, T + 7, min(P.MAX_MPH, T + 20)):
        assert (u, T, S1, u) in keys, (u, T, S1)


def test_new_keys_are_identical_to_the_cold_simulation():
  z = np.load(P.APPROACH_TABLE, allow_pickle=False)
  keys, dist = z["keys"].tolist(), z["dist"]
  shallow = [i for i, k in enumerate(keys) if k[1] - k[2] < P.DRIVE_GAP_MPH]
  assert len(shallow) > 1000
  saved = dict(P._APPROACH)
  try:
    for i in shallow[::97]:
      P._APPROACH.clear()
      u, T, S1, s0 = keys[i]
      assert P.approach_m(u, T, S1, set0=s0) == float(dist[i]), keys[i]
  finally:
    P._APPROACH.clear()
    P._APPROACH.update(saved)


def test_raise_thresholds_table_is_identical_to_the_bisection():
  z = np.load(P.APPROACH_TABLE, allow_pickle=False)
  assert str(z["rsrc"]) == P.raise_src_hash(), "stale: run tools/dev/build_approach_table.py"
  sets, Ts, thr = z["rthr_set"], z["rthr_T"], z["rthr"]
  assert len(thr) > 500
  saved = dict(P._RAISE_THR)
  try:
    for i in range(0, len(thr), 61):
      P._RAISE_THR.clear()
      got = P._raise_threshold(float(sets[i]), int(Ts[i]))
      want = None if not math.isfinite(float(thr[i])) else float(thr[i])
      assert got == want, (float(sets[i]), int(Ts[i]), got, want)
  finally:
    P._RAISE_THR.clear()
    P._RAISE_THR.update(saved)


def test_raise_table_keys_are_the_cruise_set_speeds_in_mph():
  # the planner is given carState.cruiseState.speed / MPH (a float32 read back as a float): the table is keyed by exactly that float
  for v in P.RAISE_SET_MS[:5]:
    assert float(np.float32(v)) == v
  z = np.load(P.APPROACH_TABLE, allow_pickle=False)
  assert {float(s) for s in z["rthr_set"]} <= {v / P.MPH for v in P.RAISE_SET_MS}


def test_planner_loads_both():
  P.Planner._table_tried = False
  P.Planner(None)
  assert P.Planner.table_keys == len(P.approach_table_keys())
  assert P.Planner.raise_keys > 500


def _shallow_road():
  # a corner whose push is shallow: the floor (the driver's 45 - 15 = 30) sits just under a 33 mph target
  return M.synthetic_trace([(1500, 0.0), (150, 1 / 75.0), (800, 0.0)])


def test_a_bend_with_a_shallow_push_runs_without_cold_simulations():
  tr = _shallow_road()
  mem = M.synthetic([tr])
  P.Planner._table_tried = False
  pl = P.Planner(mem)
  n0 = P.COLD[0]
  cold_ticks = 0
  s, v = 0.0, 45 * P.MPH
  for k in range(int(70 / 0.2)):
    j = min(int(s // M.SPACING_M), len(tr["x"]) - 1)
    p = pl.step(k * 0.2, P.Fix(True, float(tr["x"][j]), float(tr["y"][j]), float(tr["hdg"][j])), v, set_mph=45.0,
                yaw_k=float(tr["kap"][j]), ref_mph=45.0, floor_mph=30)
    assert isinstance(p.diag.get("nc"), int)
    cold_ticks += p.diag["nc"] > 0
    s += v * 0.2
  assert P.COLD[0] - n0 <= 5, P.COLD[0] - n0
  assert cold_ticks <= 3


def test_tick_record_carries_the_cold_count_and_wall_time():
  plan = P.Plan(None, 1.0, True, True, "", None, None, False, None, False, None, None, "", None, False, None, None, -1, -1, 0, "",
                None, None, False, {"nc": 3}, None)
  d = D.tick_log(plan, {"seq": 1, "step_us": 1234, "wall_us": 2345})
  assert d["nc"] == 3 and d["wall_us"] == 2345
