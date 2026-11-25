"""The bend's own record (2026-11-24): a remembered point's allowed speed from how earlier passes through it turned out. SYNTHETIC
roads and passes only (no recorded position is ever used in a test)."""
import math
import os
import sys

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.selfdrive.car.mazda import ti_curve_plan as P
from openpilot.selfdrive.car.mazda import ti_curve_demand as D

MPH = D.MPH
HERE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools", "curve_slowing"))


def builder():
  if HERE not in sys.path:
    sys.path.insert(0, HERE)
  import build_memory as B
  return B


def bend_k(mph, la, side=1):
  return side * la / (mph * MPH) ** 2


def bend(k):
  return M.synthetic_trace([(200.0, 0.0), (400.0, k), (200.0, 0.0)])


def mem_with(k, v_trb_mph=None, v_ok_mph=None, cap_n=0, cap_res=np.nan, n_trb=1):
  tr = bend(k)
  on = np.abs(tr["kap"]) > 0
  nan = np.full(len(on), np.nan)
  rec = {"v_trb": [np.where(on, (v_trb_mph or np.nan) * MPH, np.nan)],
         "v_ok": [np.where(on, (v_ok_mph if v_ok_mph is not None else np.nan) * MPH, nan)],
         "n_trb": [np.where(on & (v_trb_mph is not None), n_trb, 0)],
         "cap_n": [np.where(on, cap_n, 0)], "cap_res": [np.where(on, cap_res, np.nan)]}
  return M.synthetic([tr], record=rec), tr


def peak(pl):
  return float(np.min(pl.need_hi))


def table_peak(k):
  return peak(P.Planner(M.synthetic([bend(k)])))


def table_floor(t, side):
  """the lowest the record may go under a table speed t (review fix: the single-pass scatter)"""
  return math.floor(t * float(P.scatter_speed_ratio(np.array([float(t)]), np.array([side]))[0]) + 0.05)


# ---- the planner ---------------------------------------------------------------------------------------------------------------------
def test_no_record_or_no_trouble_plans_as_before():
  k = bend_k(45, 2.6)
  assert not M.synthetic([bend(k)]).has_record
  m, _ = mem_with(k)                                  # a record without trouble
  assert m.has_record and peak(P.Planner(m)) == table_peak(k)
  assert np.array_equal(P.Planner(m).need, P.Planner(M.synthetic([bend(k)])).need)


def test_trouble_lowers_to_the_highest_untroubled_speed_below_it():
  k = bend_k(45, 2.3, -1)                             # the table lets this bend go at about 45
  assert table_peak(k) >= 44
  m, _ = mem_with(k, v_trb_mph=36.2, v_ok_mph=35.1)   # troubled from 36.2 mph, untroubled at 35.1
  t = table_peak(k)
  assert peak(P.Planner(m)) == max(35, table_floor(t, -1)) < t


def test_record_never_raises_the_table():
  k = bend_k(45, 2.8, 1)
  t = table_peak(k)
  m, _ = mem_with(k, v_trb_mph=60.0, v_ok_mph=55.0)
  assert peak(P.Planner(m)) == t


def test_untroubled_at_the_troubled_speed_still_one_below():
  k = bend_k(45, 2.0, 1)
  m, _ = mem_with(k, v_trb_mph=44.0, v_ok_mph=43.99)
  t = table_peak(k)
  assert peak(P.Planner(m)) == min(t, max(43, table_floor(t, 1)))


def test_no_untroubled_pass_below_ignores_the_place_capacity_fallback():
  # review fix: the unbounded capacity fallback is gone; a low-delivering place gives the same bound as none
  k = bend_k(45, 2.4, -1)
  m_low, _ = mem_with(k, v_trb_mph=44.5, cap_n=5, cap_res=-0.6)
  m_none, _ = mem_with(k, v_trb_mph=44.5)
  assert peak(P.Planner(m_low)) == peak(P.Planner(m_none)) >= table_floor(table_peak(k), -1)


def test_record_off_switch_restores_table():
  k = bend_k(45, 2.4, -1)
  m, _ = mem_with(k, v_trb_mph=40.0, v_ok_mph=38.0)
  P.RECORD = False
  try:
    off = peak(P.Planner(m))
  finally:
    P.RECORD = True
  assert off == table_peak(k)


def test_record_need_zero_offset_is_the_table_exactly():
  k = np.linspace(-0.012, 0.012, 241)
  assert np.array_equal(P.record_need(k, np.zeros_like(k)), D.need_mph(k).astype(float))
  assert np.array_equal(P.record_need(k, np.full_like(k, np.nan)), D.need_mph(k).astype(float))
  assert np.array_equal(P.record_need_hi(k, np.zeros_like(k)), D.need_hi(k).astype(float))


def test_capacity_shrinkage_one_pass_outweighs_the_table():
  assert 1 / (1 + P.REC_SW ** 2 / P.REC_TAU ** 2) > 0.5
  off1 = P.record_offset([1], [-0.5], 0.0)[0]
  off9 = P.record_offset([9], [-0.5], 0.0)[0]
  assert -0.5 < off9 < off1 < -0.25


# ---- the file -----------------------------------------------------------------------------------------------------------------------
def test_file_round_trip_and_old_files(tmp_path):
  m, _ = mem_with(bend_k(45, 2.4, -1), v_trb_mph=40.0, v_ok_mph=38.0, cap_n=7, cap_res=-0.2)
  keys = ("x", "y", "hdg", "kap", "v", "flags", "v_flag", "npass", "bank", "trace_start", "trace_next", "kap_med") + M.REC_KEYS
  arrays = {k: getattr(m, k) for k in keys}
  p = os.path.join(str(tmp_path), "m.npz")
  M.save(p, arrays, {"version": M.VERSION})
  m2 = M.load(p)
  assert m2.has_record and int(m2.cap_n.max()) == 7 and abs(np.nanmax(m2.v_trb) / MPH - 40.0) < 0.05
  assert peak(P.Planner(m2)) == max(38, table_floor(table_peak(bend_k(45, 2.4, -1)), -1))
  for k in M.REC_KEYS:                                # a v6 / v8 file: no record, the table as before
    del arrays[k]
  M.save(p, arrays, {"version": M.VERSION})
  m3 = M.load(p)
  assert not m3.has_record and np.all(np.isnan(m3.v_trb))
  assert peak(P.Planner(m3)) == table_peak(bend_k(45, 2.4, -1))


# ---- the builder --------------------------------------------------------------------------------------------------------------------
def test_outcome_bound_majority_rule():
  B = builder()
  # troubled from 36.2 mph, untroubled at 35.1: (36.2, 35.1)
  vt, vo = B.outcome_bound(np.array([35.1, 36.2, 36.5, 44.8, 44.9]) * MPH, [False, True, True, True, False])
  assert math.isclose(vt / MPH, 36.2, abs_tol=1e-6) and math.isclose(vo / MPH, 35.1, abs_tol=1e-6)
  # conflicting evidence: troubled at 44 five times, clean at 45 once: trouble wins, nothing untroubled below
  vt, vo = B.outcome_bound(np.array([44.0] * 5 + [45.0]) * MPH, [True] * 5 + [False])
  assert math.isclose(vt / MPH, 44.0, abs_tol=1e-6) and np.isnan(vo)
  # one fluke at 44 among ten clean passes at 44-47: no trouble bound
  vt, vo = B.outcome_bound(np.array([44.0] + [44.5 + 0.25 * i for i in range(10)]) * MPH, [True] + [False] * 10)
  assert np.isnan(vt) and np.isnan(vo)
  # one troubled pass is enough
  vt, _ = B.outcome_bound([20.0], [True])
  assert vt == 20.0
  assert all(np.isnan(x) for x in B.outcome_bound([20.0, 21.0], [False, False]))


def synthetic_pass(n=600, lim=(200, 260), tk=None, lane=None, lat=True):
  v = np.full(n, 45 * MPH)
  la = np.full(n, 2.0)
  i = np.arange(n)
  return dict(s=i * float(v[0]) * 0.05, v=v, kap=la / v ** 2, ti=np.where((i >= lim[0]) & (i < lim[1]), 600.0, 300.0),
              lat_act=np.full(n, 1.0 if lat else 0.0), pressed=np.zeros(n), drv=np.zeros(n),
              tk_i=np.array([] if tk is None else [tk], int), lane_i=np.array([] if lane is None else [lane], int))


def test_trouble_per_sample_and_cleared_hook():
  B = builder()
  t = B.trouble_per_sample(synthetic_pass(tk=250))
  assert t[240] == 1.0 and t[0] == 0.0                       # 3 s at the limit + a takeover; far away: car-steered, untroubled
  assert np.all(B.trouble_per_sample(synthetic_pass(lim=(200, 220), tk=210)) == 0.0)     # 1 s at the limit: not trouble
  assert np.all(B.trouble_per_sample(synthetic_pass()) == 0.0)                           # at the limit, no takeover / lane: not trouble
  assert B.trouble_per_sample(synthetic_pass(lane=255))[240] == 1.0
  assert np.all(np.isnan(B.trouble_per_sample(synthetic_pass(tk=250, lat=False))))       # the car was not steering: no evidence
  c = B.trouble_per_sample(dict(synthetic_pass(tk=250), cleared_i=np.array([250])))
  assert np.isnan(c[240]) and c[0] == 0.0                    # HOOK: cleared by the driver: no evidence there
  p = synthetic_pass(tk=250)
  del p["ti"]
  assert np.all(np.isnan(B.trouble_per_sample(p)))           # an older cache without the command: no evidence


def test_builder_limit_residuals_and_cleared_hook():
  B = builder()
  n = 200
  v = np.full(n, 45 * MPH)
  la = np.full(n, 2.0)
  p = dict(s=np.arange(n) * 1.0, v=v, kap=la / v ** 2, ti=np.where((np.arange(n) >= 50) & (np.arange(n) < 100), 600.0, 300.0),
           lat_act=np.ones(n), pressed=np.zeros(n), drv=np.zeros(n))
  r = B.limit_residuals(p)
  assert np.all(np.isnan(r[:50])) and np.all(np.isnan(r[100:]))
  assert math.isclose(r[60], 2.0 - D.capacity(45, D.LEFT), abs_tol=1e-6)
  assert np.all(np.isnan(B.limit_residuals(dict(p, pressed=np.where(np.arange(n) >= 50, 1.0, 0.0)))))
  assert np.all(np.isnan(B.limit_residuals(dict(p, ti=np.where((np.arange(n) >= 50) & (np.arange(n) < 58), 600.0, 300.0)))))
  assert np.all(np.isnan(B.limit_residuals(dict(p, cleared_i=np.array([70])))))
  assert np.all(np.isnan(B.limit_residuals({k: x for k, x in p.items() if k != "ti"})))


# ---- review fixes (2026-11-24) ------------------------------------------------------------------------------------------------------
def test_fluke_on_a_45_mph_bend_cannot_cap_it_hard():
  # one takeover at 25 mph, nothing untroubled below, on a bend the table allows at about 45: at most the scatter under the table
  k = bend_k(45, 2.3, -1)
  t = table_peak(k)
  m, _ = mem_with(k, v_trb_mph=25.0)
  b = peak(P.Planner(m))
  assert b >= math.floor(t * float(P.scatter_speed_ratio(np.array([t]), np.array([-1]))[0]) + 0.05) and b >= t - 4, (t, b)


def test_no_untroubled_below_bound_is_the_scatter_under_the_troubled_speed():
  k = bend_k(45, 2.6, 1)                                   # table about 42
  t = table_peak(k)
  m, _ = mem_with(k, v_trb_mph=41.0)
  r = float(P.scatter_speed_ratio(np.array([41.0]), np.array([1]))[0])
  assert 0.9 < r < 0.97
  assert peak(P.Planner(m)) == max(math.floor(41.0 * r + 0.05), math.floor(t * float(P.scatter_speed_ratio(np.array([t]), np.array([1]))[0]) + 0.05))


def test_bound_never_under_19_nor_into_the_r14_zone_and_never_raises():
  k = bend_k(40, 2.0, 1)
  t = table_peak(k)
  assert t >= 32
  m, _ = mem_with(k, v_trb_mph=24.0, v_ok_mph=20.0)
  pl = P.Planner(m)
  assert peak(pl) >= 32 and float(np.min(pl.need)) >= 19
  m2, _ = mem_with(k, v_trb_mph=80.0, v_ok_mph=70.0)
  assert peak(P.Planner(m2)) == t


def test_outcome_bound_ties_are_order_independent_and_count_as_trouble():
  B = builder()
  a = B.outcome_bound(np.array([40.0, 40.0]) * MPH, [True, False])
  b = B.outcome_bound(np.array([40.0, 40.0]) * MPH, [False, True])
  assert a[0] == b[0] and np.isnan(a[1]) and np.isnan(b[1])
  a = B.outcome_bound(np.array([38.0, 40.0, 40.0]) * MPH, [False, True, False])
  b = B.outcome_bound(np.array([40.0, 40.0, 38.0]) * MPH, [False, True, False])
  assert a == b and math.isclose(a[1] / MPH, 38.0)


def test_trouble_needs_the_limit_at_the_point():
  B = builder()
  p = synthetic_pass(n=1200, lim=(200, 300), tk=255)          # limit at samples 200-299 (about 1 m each)
  t = B.trouble_per_sample(p)
  assert t[230] == 1.0
  assert t[310] == 0.0 and t[195] == 0.0                     # ~10 / 5 m past the limit: within the old smear, takeover near


def test_trouble_far_below_the_bend_demand_is_not_the_bends():
  B = builder()
  # a 45 mph pass on a nearly straight point: demand 0.5 m/s^2, far under 0.66 x the table
  assert not B.bend_demand_ok([45 * MPH], [0.5 / (45 * MPH) ** 2])[0]
  assert B.bend_demand_ok([45 * MPH], [2.0 / (45 * MPH) ** 2])[0]


# ---- second drive of 2026-11-24 ------------------------------------------------------------------------------------------------------
def test_well_attested_trouble_goes_to_the_proven_clean_speed():
  # the owner's regular trouble bend: troubled at about 40 mph on 3+ passes, clean at 35.1: the bound is 35, not the table's scatter
  k = bend_k(44, 2.4, -1)
  t = table_peak(k)
  assert t - table_floor(t, -1) < t - 35
  m, _ = mem_with(k, v_trb_mph=39.8, v_ok_mph=35.1, n_trb=P.REC_ATTESTED)
  assert peak(P.Planner(m)) == 35


def test_fluke_protection_holds_below_the_attested_count():
  k = bend_k(44, 2.4, -1)
  t = table_peak(k)
  for n in range(1, P.REC_ATTESTED):
    m, _ = mem_with(k, v_trb_mph=39.8, v_ok_mph=25.0, n_trb=n)
    assert peak(P.Planner(m)) == max(25, table_floor(t, -1)) >= t - 4


def test_attested_bound_still_clamped_by_r14_and_never_raises():
  k = bend_k(40, 2.0, 1)
  m, _ = mem_with(k, v_trb_mph=30.0, v_ok_mph=22.0, n_trb=5)
  assert peak(P.Planner(m)) >= 32
  m2, _ = mem_with(k, v_trb_mph=80.0, v_ok_mph=70.0, n_trb=5)
  assert peak(P.Planner(m2)) == table_peak(k)


def test_clean_speed_statistic():
  B = builder()
  # the REC_ATTESTED-th highest clean speed; fewer clean passes: none
  v = np.array([40.0, 50.0, 49.0, 48.0, 30.0]) * MPH
  assert math.isclose(B.clean_speed(v) / MPH, 48.0, abs_tol=1e-6)
  assert np.isnan(B.clean_speed(v[:2]))


def test_clean_per_sample_needs_margin_under_the_limit():
  B = builder()
  p = synthetic_pass(n=600, lim=(0, 0))
  p["ti"] = np.full(600, 500.0)
  assert np.all(B.clean_per_sample(p) == 1.0)
  p["ti"] = np.full(600, 569.0)                               # always at 95 %: not clean
  assert np.all(B.clean_per_sample(p) == 0.0)
  p2 = synthetic_pass(n=600, lim=(0, 0), tk=300)
  p2["ti"] = np.full(600, 400.0)
  c = B.clean_per_sample(p2)
  assert c[300] == 0.0 and c[0] == 1.0                        # a takeover near: not clean
  assert np.all(np.isnan(B.clean_per_sample(dict(p, lat_act=np.zeros(600)))))     # driver-steered: no evidence


def test_stage2_history_veto():
  dist = np.arange(0, 300, 5.0)
  vc = np.full(len(dist), 52.0)
  vt = np.full(len(dist), np.nan)
  assert P.s2_history_clean(vc, vt, dist, 150.0, 50.0)          # driven cleanly at 52 repeatedly: 50 is fine
  assert not P.s2_history_clean(vc, vt, dist, 150.0, 53.0)      # faster than the evidence
  vt2 = vt.copy(); vt2[30] = 45 * MPH
  assert not P.s2_history_clean(vc, vt2, dist, 150.0, 50.0)     # trouble on record: act as now
  vc2 = vc.copy(); vc2[31] = np.nan
  assert not P.s2_history_clean(vc2, vt, dist, 150.0, 50.0)     # a point of the bend with no clean evidence
  assert not P.s2_history_clean(vc, vt, dist, 900.0, 40.0)      # the forecast's bend beyond the memory: no evidence


# ---- comparable passes only (owner, 2026-11-24: experimental controllers / tunings must not shape the record) -----------------------
def test_record_evidence_only_from_comparable_segments():
  B = builder()
  p = synthetic_pass(n=600, tk=250)
  p["tags"] = ["0000aaaa-0", "0000aaaa-1"]
  p["off"] = np.array([0, 300, 600])
  saved = B.COMPARABLE
  try:
    B.COMPARABLE = None                                       # no map: every segment counts (older builds)
    c, t, k = B.record_evidence(p)
    assert t[240] == 1.0
    B.COMPARABLE = {"0000aaaa-0": False, "0000aaaa-1": True}
    c, t, k = B.record_evidence(p)
    assert np.all(np.isnan(t[:300])) and np.all(np.isnan(k[:300])) and np.all(np.isnan(c[:300]))
    assert np.all(np.isfinite(k[300:]))
    B.COMPARABLE = {"0000aaaa-1": True}                       # a segment missing from the map is NOT comparable
    assert np.all(np.isnan(B.record_evidence(p)[1][:300]))
  finally:
    B.COMPARABLE = saved


def test_load_comparable_map(tmp_path):
  B = builder()
  f = os.path.join(str(tmp_path), "c.json")
  import json
  json.dump({"x-0": [True, "comparable"], "x-1": [False, "test toggle"]}, open(f, "w"))
  assert B.load_comparable(f) == {"x-0": True, "x-1": False}


def _stage2_with_clean_history(model):
  """Stage 2 ticks at 50 mph on a forecast bend ~4 s ahead needing a few mph off, where the memory holds the bend with clean history at
  52 mph (n_clean >= 3 at or above the car's speed). -> the planner's stage-2 state after 3 s."""
  pl = P.Planner(None, model=model)
  mph = 50.0
  v = mph * MPH
  ts = np.arange(0.0, 8.01, 0.25)
  k_r = (D.capacity(45, D.RIGHT) * 1.0) / (46 * MPH) ** 2          # needs about 45-46 mph: a 4-5 mph late drop
  ratio, _ = D.fc_ratio_for(model)
  k = -np.where(np.abs(ts - 4.0) <= 0.5, k_r, 0.0) * np.interp(ts, D.FC_LEAD_S, ratio)
  pl.fc = (ts, k)
  dist = np.arange(0, 400, 5.0)
  need = np.where(np.abs(dist - 4.0 * v) <= 30.0, 45.0, 90.0)
  vc = np.full(len(dist), 52.0)
  vt = np.full(len(dist), np.nan)
  for i in range(60):
    pl._stage2(i * 0.05, v, mph, mph, 35, (need, dist, mph, vc, vt))
  return pl.s2


def test_stage2_clean_history_skip_only_under_the_evidence_model():
  # I3 (owner: evidence only from passes comparable to the current setup; the driving model is part of it): the clean passes were
  # all driven with north-dakota steering; under another model stage 2 acts as it would without that history
  assert P.EVIDENCE_MODEL == "north-dakota"
  assert _stage2_with_clean_history("north-dakota") is None          # the evidence applies: quiet
  assert _stage2_with_clean_history(None) is None                    # replays / tests: north-dakota
  s2 = _stage2_with_clean_history("dark-souls")
  assert s2 is not None and s2["T"] < 50                              # not the evidence's model: acts
  assert _stage2_with_clean_history("some-other-model") is not None
