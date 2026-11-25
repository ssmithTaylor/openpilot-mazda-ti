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


def test_many_pass_bend_without_a_record(monkeypatch):
  # owner's decision (2026-11-24 night): only a trouble record makes a bend unreleasable by default (REL_STRONG "record"); a
  # many-pass bend the forecast genuinely does not show is released. The other setting (record or 10+ passes) still holds it.
  k = k_needing(40)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = memory(tr, npass=P.REL_STRONG_PASSES)
  assert P.REL_STRONG == "record"
  pl = P.Planner(mem)
  drive(mem, tr, 55, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 55))
  assert "s1_release" in kinds(pl)
  monkeypatch.setattr(P, "REL_STRONG", "record_or_passes")
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 55))
  assert "s1_release" not in kinds(pl)
  assert next(v for t, s, p, v in res if s >= 1500) <= int(np.min(pl.need_hi)) + 1.5


def test_a_late_start_is_not_released_on_its_own_reading():
  # the planner locks onto the road only ~210 m before the bend: its start falls inside the decision window. The start's own tick
  # and the next are not read (the drive's fault: start and release on one reading); after that the two-reading rule applies, so a
  # genuinely straight forecast still releases it, never sooner than REL_LATE_SKIP_S + REL_TICKS ticks after the start
  k = k_needing(40)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = memory(tr, npass=3)
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 60, start=1500 - 210, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 55))
  st = [e for e in pl.events if e["kind"] == "start"]
  assert st and st[0]["dist"] <= P.CONFIRM_LEAD_S * 55 * MPH, st     # the case: a late start
  i_start = next(i for i, e in enumerate(pl.events) if e["kind"] == "start")
  i_rel = next((i for i, e in enumerate(pl.events) if e["kind"] == "s1_release"), None)
  assert i_rel is not None
  t_start = next(t for t, s, p, v in res if p.kind == "corner")
  t_rel = next(t for t, s, p, v in res if p.released)
  assert t_rel - t_start >= P.REL_LATE_SKIP_S + (P.REL_TICKS - 1) * 0.2 - 1e-6, (t_start, t_rel)


def _record_memory(tr, npass, n_trb, v_trb_mph=40.0, v_ok_mph=35.0):
  """A remembered right bend whose points carry a trouble record: `n_trb` troubled passes at v_trb_mph of `npass`, the highest
  untroubled pass below them at v_ok_mph (nan: none)."""
  on = tr["kap"] < 0
  nan = np.full(len(on), np.nan)
  rec = {"v_trb": [np.where(on, v_trb_mph * MPH, np.nan)], "v_ok": [np.where(on, v_ok_mph * MPH, np.nan)],
         "n_trb": [np.where(on, n_trb, 0)], "cap_n": [np.zeros(len(on), np.uint8)], "cap_res": [nan]}
  mem = M.synthetic([tr], record=rec)
  mem.npass[:] = npass
  return mem


def test_attested_trouble_bend_keeps_its_protection_against_a_straight_forecast():
  # the owner's regular trouble bend: 26 passes, 3 troubled at the limit (~40 mph), clean below at 35: an attested record; the
  # memory is the stronger witness and a forecast that does not show the bend never releases it
  k = k_needing(42, -1.0)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  mem = _record_memory(tr, npass=26, n_trb=P.REC_ATTESTED)
  pl = P.Planner(mem)
  res = drive(mem, tr, 55, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 55))
  assert "start" in kinds(pl) and "s1_release" not in kinds(pl), kinds(pl)
  T = [e for e in pl.events if e["kind"] == "start"][0]["T"]   # the record's 35, held at the 15 mph shed floor (40) from 55
  assert T <= 40
  assert next(v for t, s, p, v in res if s >= 1500) <= T + 1.5


def test_one_troubled_pass_does_not_make_a_bend_unreleasable():
  # the fluke rule (I1): one troubled pass of 28 is not an attested record; the ordinary release rules apply, and a forecast
  # that validly reads the bend absent releases it
  k = k_needing(42, -1.0)
  tr = road((1500, 0.0), (150, k), (800, 0.0))
  for n_trb, v_ok in ((1, 35.0), (2, 35.0), (P.REC_ATTESTED, float("nan"))):
    mem = _record_memory(tr, npass=28, n_trb=n_trb, v_ok_mph=v_ok)
    pl = P.Planner(mem)
    res = drive(mem, tr, 55, 160, closed=True, planner=pl, model_fn=road_preview(road((3000, 0.0)), 55))
    assert kinds(pl)[:2] == ["start", "s1_release"], (n_trb, kinds(pl))
    assert max(v for t, s, p, v in res if 1500 <= s <= 1650) > 53, n_trb
