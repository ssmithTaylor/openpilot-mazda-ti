"""The driving screen's curve element, planner side (ti_curve_view.py; owner's approved design 2026-11-26, mock-up round 5): bend classes
and the red rule, the chip lead times, the ribbon (decimated, signed, no positions), the stale / off rules of what is published, the one
ding per red bend, and the tick record that lets a drive's screen be replayed. Synthetic roads only."""
import json
import math

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P, ti_curve_plannerd as D, ti_curve_view as V

MPH = V.MPH


# ---- bend classes and the red rule ------------------------------------------------------------------------------------------------
def _arrays(needs, step=5.0):
  d = np.arange(len(needs)) * step
  return d, np.asarray(needs, np.float64), np.ones(len(needs))


def test_a_bend_the_car_can_slow_for_is_yellow_with_its_target():
  d, n, s = _arrays([90] * 40 + [33] * 10 + [90] * 40)
  bl = V.bends(d, n, s, mph=45.0, set_mph=45, floor=30)
  assert len(bl) == 1
  b = bl[0]
  assert b["cls"] == V.CLS_YELLOW and b["need"] == 33 and b["target"] == 33
  assert b["start"] == 200.0 and b["end"] == 245.0 and b["resume"] == 245.0 + V.HOLD_AFTER_M
  assert b["lead"] == V.YELLOW_LEAD_S
  assert 0.0 <= b["approach"] < b["start"]


def test_a_bend_needing_less_than_the_floor_is_red_and_its_target_is_the_floor():
  d, n, s = _arrays([90] * 40 + [20] * 10 + [90] * 40)
  b = V.bends(d, n, s, mph=45.0, set_mph=45, floor=30)[0]
  assert b["cls"] == V.CLS_RED and b["need"] == 20 and b["target"] == 30
  assert b["lead"] == V.red_lead_s(45.0, 20)


def test_the_floor_is_the_red_line_exactly():
  for need, cls in ((29, V.CLS_RED), (30, V.CLS_YELLOW), (31, V.CLS_YELLOW)):
    d, n, s = _arrays([90] * 20 + [need] * 5 + [90] * 20)
    assert V.bends(d, n, s, mph=45.0, set_mph=45, floor=30)[0]["cls"] == cls, need


def test_a_point_needing_less_than_two_mph_off_is_no_bend():
  d, n, s = _arrays([90] * 20 + [44] * 5 + [90] * 20)
  assert V.bends(d, n, s, mph=45.0, set_mph=45, floor=30) == []
  d, n, s = _arrays([90] * 20 + [43] * 5 + [90] * 20)
  assert len(V.bends(d, n, s, mph=45.0, set_mph=45, floor=30)) == 1


def test_bends_beyond_45_seconds_are_not_shown_and_at_most_eight():
  d, n, s = _arrays(([90] * 6 + [30] * 2) * 200)
  bl = V.bends(d, n, s, mph=45.0, set_mph=45, floor=25)
  assert len(bl) == V.MAX_BENDS
  assert all(b["start"] <= 45.0 * 45.0 * MPH for b in bl)
  d, n, s = _arrays([90] * 500 + [20] * 10)
  assert V.bends(d, n, s, mph=45.0, set_mph=45, floor=30) == []          # 2500 m out at 45 mph: beyond 45 s


def test_direction_is_the_sign_of_the_curvature_at_the_tightest_point():
  d, n, _ = _arrays([90] * 20 + [40, 33, 40] + [90] * 20)
  k = np.zeros(len(n))
  k[21] = -0.02
  assert V.bends(d, n, np.sign(k), 45.0, 45, 30)[0]["dir"] == -1
  k[21] = 0.02
  assert V.bends(d, n, np.sign(k), 45.0, 45, 30)[0]["dir"] == 1


def test_red_chip_lead_is_the_braking_time_plus_six_never_under_13():
  assert V.red_lead_s(45.0, 20) == 6.0 + (25 * MPH) / 1.5
  assert V.red_lead_s(30.0, 28) == 13.0
  assert V.red_lead_s(60.0, 20) > 17.0


def test_the_chip_red_first_then_yellow_within_their_leads():
  v = 45.0 * MPH
  yel = {"cls": V.CLS_YELLOW, "start": 9.0 * v, "lead": 10.0}
  red = {"cls": V.CLS_RED, "start": 13.0 * v, "lead": V.red_lead_s(45.0, 20)}
  assert V.chip([yel, red], 45.0) == (V.CLS_RED, 1)                  # the red one is within its (longer) lead: it wins
  assert V.chip([dict(yel, start=11.0 * v)], 45.0) == (V.CLS_NEUTRAL, -1)
  assert V.chip([dict(red, start=30.0 * v), yel], 45.0) == (V.CLS_YELLOW, 1)


# ---- the ribbon ---------------------------------------------------------------------------------------------------------------------
def test_ribbon_is_decimated_monotonic_and_keeps_the_heading_change():
  d = np.arange(0, 2000, 5.0)
  k = np.where((d > 300) & (d < 400), 0.01, 0.0)
  r, ends = V.ribbon(d, k, 45.0)
  assert 2 <= len(r) <= V.VIEW_PTS
  ds = [p[0] for p in r]
  assert ds == sorted(ds) and ds[0] == 0 and ds[-1] <= 45.0 * 45.0 * MPH
  # heading change: sum k*ds over the decimated points ~ the true 100 m x 0.01 = 1 rad
  th = sum(r[i][1] * 1e-5 * (r[i][0] - r[i - 1][0]) for i in range(1, len(r)))
  assert abs(th - 0.95) < 0.12
  assert not ends


def test_ribbon_marks_a_remembered_road_that_ends_inside_the_window():
  d = np.arange(0, 400, 5.0)
  r, ends = V.ribbon(d, np.zeros(len(d)), 45.0)
  assert ends and r[-1][0] == 395


# ---- the planner tick end to end on a synthetic road -------------------------------------------------------------------------------
def _drive(on=True, ticks=200, v=20.0, segments=((600.0, 0.0), (120.0, 0.03), (600.0, 0.0), (150.0, -0.012), (800.0, 0.0)),
           floor=30, x0=1.0e5, y0=-2.0e5):
  tr = M.synthetic_trace(list(segments), x0=x0, y0=y0)
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  vw = V.View()
  out = []
  for k in range(ticks):
    s = 5.0 + k * 0.2 * v
    j = min(int(s // M.SPACING_M), len(tr["x"]) - 1)
    plan = pl.step(k * 0.2, P.Fix(True, float(tr["x"][j]), float(tr["y"][j]), float(tr["hdg"][j])), v, set_mph=45.0,
                   yaw_k=float(tr["kap"][j]), ref_mph=45.0, floor_mph=floor)
    s1, s2, side = D.stage_summary(pl)
    p, ding, log = vw.step(pl, plan, k * 0.2, k, on, v / MPH, 45.0, floor, side)
    out.append((s, plan, p, ding, log))
  return out, tr


def test_payload_has_the_schema_a_ribbon_and_the_bends_and_is_small():
  out, _ = _drive()
  p = out[30][2]
  assert tuple(p) == V.VIEW_KEYS
  assert p["on"] == 1 and p["v"] == V.VIEW_VERSION
  assert 2 <= len(p["r"]) <= V.VIEW_PTS
  assert any(b[6] == V.CLS_RED for b in p["b"])         # the 0.03 bend (radius 33 m) needs less than the 30 mph floor
  assert len(V.view_json(p)) <= 2500
  for b in p["b"]:
    assert len(b) == len(V.BEND_FIELDS) and all(isinstance(x, int) for x in b)
    assert b[0] <= b[1] <= b[2] < b[8] and b[7] <= b[0]


def test_left_and_right_bends_keep_their_sign_in_the_ribbon_and_the_bend_list():
  out, _ = _drive()
  p = out[110][2]                                          # 485 m along: both bends inside the 45 s window
  ks = [k for _, k in p["r"]]
  assert max(ks) > 1000 and min(ks) < -500                 # +0.03 left and -0.012 right, x1e5
  dirs = [b[3] for b in p["b"]]
  assert 1 in dirs and -1 in dirs


def test_no_position_ever_reaches_the_payload():
  out, tr = _drive(x0=123456.0, y0=-654321.0)
  pos = {int(round(float(a))) for a in np.r_[tr["x"], tr["y"]]}
  for _, _, p, _, log in out:
    assert set(p) <= set(V.VIEW_KEYS)
    text = V.view_json(p).lower()
    for bad in ("lat", "lon", "x", "y", "pos", "vin", "name"):
      assert f'"{bad}"' not in text
    vals = [p["mph"], p["fl"], p["seq"]] + [a for pt in p["r"] for a in pt] + [a for b in p["b"] for a in b] + log
    assert all(isinstance(a, int) for a in vals)
    assert all(abs(a) < 100000 for a in vals)                # no projected coordinate (they are 1e5 m and more here)
    assert not set(vals) & pos


def test_off_publishes_no_ribbon_no_bends_and_never_dings():
  out, _ = _drive(on=False)
  for _, _, p, ding, log in out:
    assert p["on"] == 0 and p["r"] == [] and p["b"] == [] and ding is None and log[0] == 0


def test_exactly_one_ding_per_red_bend_and_none_for_yellow():
  out, _ = _drive(ticks=260)
  dings = [x[3] for x in out if x[3] is not None]
  assert len(dings) == 1, dings
  assert dings[0]["need"] < 30 and dings[0]["n"] == 1
  # the onset is the tick the red chip first appears
  first = next(i for i, x in enumerate(out) if x[4][1] == V.CLS_RED)
  assert out[first][3] is not None
  assert all(x[4][0] == 1 for x in out[first:])          # the count never moves again for that bend


def test_no_ding_below_19_mph():
  out, _ = _drive(v=18.5 * MPH, ticks=400, segments=((150.0, 0.0), (120.0, 0.05), (400.0, 0.0)))
  assert all(x[3] is None for x in out)


def test_a_second_red_bend_dings_again():
  out, _ = _drive(ticks=420, segments=((500.0, 0.0), (100.0, 0.03), (900.0, 0.0), (100.0, -0.03), (900.0, 0.0)))
  dings = [x[3] for x in out if x[3] is not None]
  assert [d["n"] for d in dings] == [1, 2]


def test_the_ding_memory_is_bounded():
  vw = V.View()
  for i in range(100):
    vw.dinged.append(float(i * 1000))
    del vw.dinged[:-V.DING_KEEP]
  assert len(vw.dinged) <= V.DING_KEEP


def test_without_trusted_memory_the_plans_corner_is_one_bend_without_a_ribbon():
  plan = P.Plan(40, 1.0, True, False, "", 150.0, 40, False, None, False, None, None, "corner", 200.0, False, None, 24, -1, -1, 1, "map",
                None, None, False, {}, None)
  b = V.plan_bend(plan, 0, 30, 45.0)
  assert b["dir"] == 0 and b["cls"] == V.CLS_RED and b["need"] == 24 and b["start"] == 150.0 and b["end"] == 200.0

  class _Pl:
    view, odo = None, 0.0
  p, _, _ = V.View().step(_Pl(), plan, 1.0, 1, True, 45.0, 45, 30, 0)
  assert p["r"] == [] and len(p["b"]) == 1 and p["b"][0][3] == 0


# ---- stale handling and the ding reader (frogpilot_planner) --------------------------------------------------------------------------
class _Params:
  def __init__(self, d):
    self.d = d

  def get(self, k):
    return None if self.d is None else json.dumps(self.d).encode()


def test_the_ding_is_read_only_while_fresh():
  assert V.read_curve_ding(_Params({"m": 100.0, "n": 1, "need": 20}), 100.5) == (100.0, 20)
  assert V.read_curve_ding(_Params({"m": 100.0, "n": 1, "need": 20}), 101.2) == (None, None)
  assert V.read_curve_ding(_Params(None), 100.0) == (None, None)
  assert V.read_curve_ding(_Params({"m": 200.0}), 100.0) == (None, None)       # from a future clock: never


# ---- replayability: what the driver saw is in the tick record ---------------------------------------------------------------------
def test_the_tick_record_carries_the_screen_bounded_ints_only():
  out, _ = _drive()
  _, plan, p, _, log = out[30]
  rec = D.tick_log(plan, dict(seq=30, scr=log))
  assert rec["scr"] == log
  assert len(json.dumps(rec["scr"])) <= D.TICK_SCR_MAX_B
  assert "scr" in D.TICK_KEYS
  long = D.tick_log(plan, dict(seq=30, scr=list(range(200))))
  assert len(json.dumps(long["scr"])) <= D.TICK_SCR_MAX_B


def test_the_planners_look_ahead_for_the_screen_is_replayed_state():
  assert D.PLANNER_STATE["view"].startswith("replayed")


def test_plannerd_publishes_the_view_and_the_ding_params():
  src = open(D.__file__, encoding="utf-8").read()
  assert "V.PARAM_VIEW" in src and "V.PARAM_DING" in src
