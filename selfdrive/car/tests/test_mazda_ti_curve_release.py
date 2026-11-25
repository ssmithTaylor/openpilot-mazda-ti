"""The forecast release of a remembered bend (2026-11-24, second night drive, route 00000346, two passes of the owner's regular
trouble bend): `start` T 37 and `s1_release` in the SAME planner tick, ~115 m before the bend, while the car was leaving the
opposite bend before it (an S-bend); that one 6 s preview, scaled to 37 mph, read ~1.0-1.25 m/s^2 against the 1.42 threshold although
1.5 s later it read ~5. Released as "not there", the bend got only stage 2 (5 mph) and the car crossed into the oncoming lane.

The rule tested here: never released on ONE reading; "absent" only on REL_TICKS consecutive ticks whose preview reaches past the
bend's tightest point and while the car is not in, or just leaving, another bend; a bend with a trouble record or REL_STRONG_PASSES
passes is never released by the forecast; a start inside the decision window (a late start) is not released. A few-pass bend with a
genuinely straight forecast is still released. Synthetic roads only."""
import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_demand as D, ti_curve_memory as M, ti_curve_plan as P
from openpilot.selfdrive.car.tests.test_mazda_ti_curve_plan import drive, k_needing, road, road_preview

MPH = P.MPH
SET = 48
LEAD_M, LEFT_M, GAP_M, RIGHT_M = 1500.0, 150.0, 120.0, 150.0
RIGHT_AT = LEAD_M + LEFT_M + GAP_M          # the remembered right bend's entry (m along the road)
LEFT_END = LEAD_M + LEFT_M


def s_bend(right_need=40):
  """An opposite (left) bend the car can hold at SET (lateral acceleration ~1.6 m/s^2: the car is "in a bend"), ending GAP_M before a
  right bend needing `right_need` mph by the table."""
  k_left = 0.0035
  return road((LEAD_M, 0.0), (LEFT_M, k_left), (GAP_M, 0.0), (RIGHT_M, k_needing(right_need, -1.0)), (1200.0, 0.0))


def memory(tr, npass=3, v_trb_mph=None):
  rec = None
  if v_trb_mph is not None:
    on = tr["kap"] < 0
    nan = np.full(len(on), np.nan)
    rec = {"v_trb": [np.where(on, v_trb_mph * MPH, np.nan)], "v_ok": [nan], "n_trb": [np.where(on, 2, 0)],
           "cap_n": [np.zeros(len(on), np.uint8)], "cap_res": [nan]}
  mem = M.synthetic([tr], record=rec) if rec is not None else M.synthetic([tr])
  mem.npass[:] = npass
  return mem


def under_reading(tr, v_mph, until_s_after_left=1.5):
  """The drive's forecast: true, except from 40 m before the left bend's end until `until_s_after_left` after the car leaves it,
  where the right bend is read at a fifth of its curvature (its peak scaled to 37 mph ~1.0 m/s^2, under the release threshold)."""
  f = road_preview(tr, v_mph)
  st = {"left_t": None}

  def g(t, s):
    ts, k = f(t, s)
    if s >= LEFT_END and st["left_t"] is None:
      st["left_t"] = t
    low = s >= LEFT_END - 40.0 and (st["left_t"] is None or t - st["left_t"] < until_s_after_left)
    if low:
      d = s + ts * v_mph * MPH
      k = np.where(d >= RIGHT_AT - 10.0, 0.2 * k, k)
    return ts, k
  return g


def kinds(pl):
  return [e["kind"] for e in pl.events if e["kind"] in ("start", "s1_release", "s1_confirm")]


def at_bend(res):
  return max(v for t, s, p, v in res if RIGHT_AT + 20 <= s <= RIGHT_AT + RIGHT_M)


def test_s_bend_trouble_bend_is_not_released_and_slowed_to_its_record():
  tr = s_bend(40)
  mem = memory(tr, npass=24, v_trb_mph=38.0)                 # 24 passes, troubled at 38: the record gives 37
  pl = P.Planner(mem)
  T = int(np.min(pl.need_hi))
  assert T <= 37, T
  res = drive(mem, tr, SET, 200, closed=True, planner=pl, model_fn=under_reading(tr, SET))
  assert "s1_release" not in kinds(pl), kinds(pl)
  assert at_bend(res) <= T + 1.5, at_bend(res)


def test_s_bend_few_pass_bend_is_not_released_on_the_bend_exit_reading():
  tr = s_bend(40)
  mem = memory(tr, npass=3)                                   # few passes, no record: the release applies, but not on this reading
  pl = P.Planner(mem)
  T = int(np.min(pl.need_hi))
  res = drive(mem, tr, SET, 200, closed=True, planner=pl, model_fn=under_reading(tr, SET))
  assert "s1_release" not in kinds(pl), kinds(pl)
  assert at_bend(res) <= T + 1.5, at_bend(res)


def test_few_pass_bend_with_a_straight_forecast_is_still_released():
  k = k_needing(40)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = memory(tr, npass=3)
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 55))
  ev = kinds(pl)
  assert ev[:2] == ["start", "s1_release"], ev
  rel = [e for e in pl.events if e["kind"] == "s1_release"][0]
  st = [e for e in pl.events if e["kind"] == "start"][0]
  assert rel is not st
  assert max(v for t, s, p, v in res if 1500 <= s <= 1650) > 53      # released: the speed is back by the bend


def test_many_pass_bend_is_not_released_by_a_straight_forecast():
  k = k_needing(40)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = memory(tr, npass=P.REL_STRONG_PASSES)
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 55))
  assert "s1_release" not in kinds(pl)
  assert next(v for t, s, p, v in res if s >= 1500) <= int(np.min(pl.need_hi)) + 1.5


def test_a_late_start_is_not_released():
  # the planner locks onto the road only ~120 m before the bend (STABLE_S of matching first): its start falls inside the decision
  # window, so the decision point has passed: the bend is known, it is slowed for, never released on the forecast (straight here)
  k = k_needing(40)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = memory(tr, npass=3)
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 60, start=1500 - 210, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 55))
  st = [e for e in pl.events if e["kind"] == "start"]
  assert st and st[0]["dist"] <= P.CONFIRM_LEAD_S * 55 * MPH, st     # the case: a late start
  assert "s1_release" not in kinds(pl), kinds(pl)
  assert min(v for t, s, p, v in res if s <= 1650) < 55 - 3
