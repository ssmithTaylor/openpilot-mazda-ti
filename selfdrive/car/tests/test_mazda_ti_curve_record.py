"""The bend's own record (2026-11-24): a remembered point's allowed speed from what the car delivered there at its limit. SYNTHETIC
roads and passes only (no recorded position is ever used in a test)."""
import math
import os
import sys

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.selfdrive.car.mazda import ti_curve_plan as P
from openpilot.selfdrive.car.mazda import ti_curve_demand as D

MPH = D.MPH


def bend_k(mph, la, side=1):
  return side * la / (mph * MPH) ** 2


def mem_with(k, n, res, length=400.0):
  tr = M.synthetic_trace([(200.0, 0.0), (length, k), (200.0, 0.0)])
  npt = len(tr["x"])
  cn = np.where(np.abs(tr["kap"]) > 0, n, 0).astype(np.int32)
  cr = np.where(cn > 0, res, np.nan)
  return M.synthetic([tr], record=([cn], [cr])), tr, npt


def peak_need(pl):
  return float(np.min(pl.need_hi))


def test_zero_offset_is_the_table_exactly():
  k = np.linspace(-0.012, 0.012, 241)
  assert np.array_equal(P.record_need(k, np.zeros_like(k)), D.need_mph(k).astype(float))
  assert np.array_equal(P.record_need(k, np.full_like(k, np.nan)), D.need_mph(k).astype(float))
  assert np.array_equal(P.record_need_hi(k, np.zeros_like(k)), D.need_hi(k).astype(float))


def test_no_record_plans_as_before():
  tr = M.synthetic_trace([(200.0, 0.0), (400.0, bend_k(45, 2.6)), (200.0, 0.0)])
  old = P.Planner(M.synthetic([tr]))
  assert not M.synthetic([tr]).has_record
  new = P.Planner(M.synthetic([tr], record=([np.zeros(len(tr["x"]), np.int32)], [np.full(len(tr["x"]), np.nan)])))
  assert np.array_equal(old.need_hi, new.need_hi) and np.array_equal(old.need, new.need)


def test_record_off_switch_restores_table():
  mem, tr, _ = mem_with(bend_k(45, 2.4, -1), 10, -0.5)
  P.RECORD = False
  try:
    off = P.Planner(mem).need_hi.copy()
  finally:
    P.RECORD = True
  assert np.array_equal(off, P.Planner(M.synthetic([tr])).need_hi)


def test_low_delivering_place_slows_more_and_high_one_less():
  k = bend_k(45, 2.45, -1)          # a right bend at about the table's capacity at 45 mph
  table = peak_need(P.Planner(M.synthetic([M.synthetic_trace([(200.0, 0.0), (400.0, k), (200.0, 0.0)])])))
  low = peak_need(P.Planner(mem_with(k, 15, -0.45)[0]))
  high = peak_need(P.Planner(mem_with(k, 15, +0.35)[0]))
  assert low < table - 3 and high > table


def test_graded_a_bend_just_at_its_own_limit_costs_1_to_2_mph():
  # a place that delivers exactly the table, seen many times; a bend asking exactly that at 45 mph: a small, graded drop
  for side in (1, -1):
    cap = D.capacity(45, side)
    k = bend_k(45, cap, side)
    need = peak_need(P.Planner(mem_with(k, 30, 0.0)[0]))
    table = float(D.need_hi(np.array([k]))[0])          # 44: the table's own speed window already costs 1 mph here
    assert 0 <= table - need <= 2 and need >= 42, (side, need, table)
  # and no cliff: the need falls smoothly as the bend sharpens
  ns = [peak_need(P.Planner(mem_with(bend_k(45, la, -1), 10, 0.0)[0])) for la in np.arange(2.0, 3.2, 0.05)]
  assert all(0 <= a - b <= 2 for a, b in zip(ns[:-1], ns[1:]))


def test_one_pass_already_outweighs_the_table():
  k = bend_k(45, 2.4, -1)
  n1 = peak_need(P.Planner(mem_with(k, 1, -0.5)[0]))
  n10 = peak_need(P.Planner(mem_with(k, 10, -0.5)[0]))
  table = peak_need(P.Planner(M.synthetic([M.synthetic_trace([(200.0, 0.0), (400.0, k), (200.0, 0.0)])])))
  assert n10 <= n1 < table
  # the weight of the place's own mean after one pass: n / (n + sw^2 / tau^2) > 0.5
  assert 1 / (1 + P.REC_SW ** 2 / P.REC_TAU ** 2) > 0.5


def test_more_passes_narrow_the_quantile():
  # the same mean residual with more passes: less uncertainty, so a higher allowed speed (never lower)
  k = bend_k(45, 2.3, 1)
  ns = [peak_need(P.Planner(mem_with(k, n, 0.0)[0])) for n in (1, 2, 4, 9, 30)]
  assert all(b >= a for a, b in zip(ns[:-1], ns[1:]))


def test_file_round_trip_and_old_loader_compat(tmp_path):
  mem, tr, npt = mem_with(bend_k(45, 2.4, -1), 7, -0.2)
  arrays = {k: getattr(mem, k) for k in ("x", "y", "hdg", "kap", "v", "flags", "v_flag", "npass", "bank", "trace_start", "trace_next",
                                         "kap_med", "cap_n", "cap_res")}
  p = os.path.join(str(tmp_path), "m.npz")
  M.save(p, arrays, {"version": M.VERSION})
  m2 = M.load(p)
  assert m2.has_record and int(m2.cap_n.max()) == 7 and np.nanmin(m2.cap_res) < -0.19
  # a file without the record (v6 / v8) loads with none
  del arrays["cap_n"], arrays["cap_res"]
  M.save(p, arrays, {"version": M.VERSION})
  m3 = M.load(p)
  assert not m3.has_record and int(m3.cap_n.max()) == 0
  # an OLD loader reads only its own keys: the new keys are extra arrays in the npz and are ignored
  with np.load(p) as z:
    assert "cap_n" not in z.files


def test_builder_limit_residuals_and_cleared_hook():
  here = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools", "curve_slowing"))
  sys.path.insert(0, here)
  import build_memory as B
  n = 200
  v = np.full(n, 45 * MPH)
  la = np.full(n, 2.0)
  p = dict(s=np.arange(n) * 1.0, v=v, kap=la / v ** 2, ti=np.where((np.arange(n) >= 50) & (np.arange(n) < 100), 600.0, 300.0),
           lat_act=np.ones(n), pressed=np.zeros(n), drv=np.zeros(n))
  r = B.limit_residuals(p)
  assert np.all(np.isnan(r[:50])) and np.all(np.isnan(r[100:]))
  assert math.isclose(r[60], 2.0 - D.capacity(45, D.LEFT), abs_tol=1e-6)
  # the driver pressing, or a stretch shorter than 0.5 s, gives no record
  p2 = dict(p, pressed=np.where(np.arange(n) >= 50, 1.0, 0.0))
  assert np.all(np.isnan(B.limit_residuals(p2)))
  p3 = dict(p, ti=np.where((np.arange(n) >= 50) & (np.arange(n) < 58), 600.0, 300.0))
  assert np.all(np.isnan(B.limit_residuals(p3)))
  # HOOK: a stretch the driver cleared does not count
  assert np.all(np.isnan(B.limit_residuals(dict(p, cleared_i=np.array([70])))))
  # an older cached pass without `ti`: no record
  assert np.all(np.isnan(B.limit_residuals({k: x for k, x in p.items() if k != "ti"})))
