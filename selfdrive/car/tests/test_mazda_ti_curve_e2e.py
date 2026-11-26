"""Curve slowing END TO END: the real Planner (5 Hz) and the real car-side Curve (100 Hz) together, the car side's status relayed to the
planner as ti_curve_plannerd does (its reference, its floor while it intervenes, whether it acts), and a phase-aware simulated cruise
(the Car of test_mazda_ti_curve.py). Synthetic roads only. Cases (a)-(g) of the third review of the warning, plus M1-M3."""
import importlib.util
import json
import pathlib

import numpy as np
import pytest

from openpilot.selfdrive.car.mazda import ti_curve as C, ti_curve_memory as M, ti_curve_plan as P
from openpilot.selfdrive.car.mazda import ti_curve_plannerd as D

_spec = importlib.util.spec_from_file_location("tc_e2e_car", pathlib.Path(__file__).parent / "test_mazda_ti_curve.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
DT = 0.01
PREVIEW_S = 8.0     # the realistic preview (the real model previews 8-10 s; under-read by FC_RATIO, published right positive); was 4.0
_KS = np.linspace(0.0005, 0.06, 12000)
_SP = P.D.need_mph(_KS)   # plan T4: ti_curve_demand (left bends, CAP_RAW)


def k_for(need):
  return float(_KS[np.argmin(np.abs(_SP - need))])


class WeakCar(T.Car):
  """A cruise that slows at only `dec` mph/s (behind plan: downhill, or weaker than measured)."""
  dec = 0.5

  def tick(self):
    before = self.mph
    super().tick()
    if self.mph < before:
      self.mph = max(self.mph, before - self.dec * DT)


class _Store:
  """The car side's status as the daemon reads it."""

  def __init__(self):
    self.raw = None

  def get(self, k):
    return self.raw


def e2e(segs, set_mph, mph0=None, seconds=300, events=None, single=False, car_cls=None, preview_s=PREVIEW_S, car_la=False):
  """segs: [(length m, need mph or None)]. events(f, car, s) -> Inputs overrides; it may also change car.mph / car.set (a lead,
  the driver's pedals). car_la: the car side is also given the car's own lateral acceleration (the road's curvature here x v^2,
  measured and desired alike: review C1 (c)). Returns rows every 0.1 s."""
  tr = M.synthetic_trace([(L, 0.0 if n is None else k_for(n)) for L, n in segs])
  mem = M.synthetic([tr])
  if single:
    mem.npass[:] = 1
  pl, cv = P.Planner(mem), C.Curve()
  car = (car_cls or T.Car)(float(set_mph if mph0 is None else mph0), set_mph)
  s_road = np.arange(len(tr["x"])) * M.SPACING_M
  s, plan_d, plan_t, seq, log = 0.0, None, None, 0, []
  store = _Store()
  for f in range(int(seconds / DT)):
    car.tick()
    t = f * DT
    if s > s_road[-1] - 10:
      break
    ov = (events(f, car, s) or {}) if events is not None else {}
    if f % 20 == 0:
      store.raw = json.dumps(cv.status(t)).encode()
      ref, floor, acting = D.car_side(store, t, C)
      sj = s + ov.get("_jump", 0.0)                                     # a matcher miss: fixes somewhere else
      x, y = np.interp(sj, s_road, tr["x"]), np.interp(sj, s_road, tr["y"])
      i = min(int(sj // M.SPACING_M), len(tr["hdg"]) - 1)
      ahead = np.arange(0, preview_s + 0.01, 0.25)
      kk = np.array([tr["kap"][min(int(q // M.SPACING_M), len(tr["kap"]) - 1)] for q in s + ahead * car.mph * P.MPH])
      # as the real model publishes it (plan T5): openpilot's curvature is RIGHT positive (P.MODEL_K_SIGN), and the preview under-reads
      # the bend by the measured FC_RATIO at each lead (the planner calibrates it back)
      kk = P.MODEL_K_SIGN * kk * np.interp(ahead, P.D.FC_LEAD_S, P.D.FC_RATIO)
      p = pl.step(t, P.Fix(ov.get("_fix_valid", True), float(x), float(y), float(tr["hdg"][i])), car.mph * P.MPH,
                  set_mph=car.set if ov.get("cruise_on", True) else None, yaw_k=float(tr["kap"][i]), model_k=(ahead, kk),
                  ref_mph=ref, floor_mph=floor, acting=acting)
      plan_d, plan_t = P.plan_json(p, t, seq, "act"), t
      seq += 1
    kw = dict(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, lead=False,
              driver_button=False, counter=car.counter, plan=plan_d, plan_age=None if plan_t is None else t - plan_t, mono=t)
    if car_la:
      la = float(tr["kap"][min(int(s // M.SPACING_M), len(tr["kap"]) - 1)]) * (car.mph * P.MPH) ** 2
      kw.update(la_now=la, desired_la=la)
    kw.update({k: v for k, v in ov.items() if not k.startswith("_")})
    b = cv.step(f, C.Inputs(**kw))
    if b:
      car.press(b)
    s += car.mph * P.MPH * DT
    if f % 10 == 0:
      st = cv.status(t)
      log.append(dict(t=t, s=s, state=cv.state, set=car.set, mph=car.mph, alert=st["alert"], line1=st["line1"], line2=st["line2"]))
  return log


def gaps(log, a, b, need):
  """rows in [a, b] m where the car is above need + 2 (+1 slack) and nothing warns"""
  return [(round(r["s"]), round(r["mph"], 1)) for r in log if a <= r["s"] <= b and r["mph"] > need + P.ALERT_MARGIN_MPH + 1
          and not r["alert"]]


def stretches(log):
  """the unbroken warning stretches: [(first row, last row)]"""
  out, on, prev = [], None, None
  for r in log:
    if r["alert"] and on is None:
      on = r
    if not r["alert"] and on is not None:
      out.append((on, prev))
      on = None
    prev = r
  if on is not None:
    out.append((on, prev))
  return out


def first_alert_lead_s(log, corner_s):
  first = next((r for r in log if r["alert"]), None)
  t_corner = next(r["t"] for r in log if r["s"] >= corner_s)
  return None if first is None else t_corner - first["t"]


# ---- (c) M1: the warning holds through the approach, and the page agrees -----------------------------------------------------------
def test_c_m1_i_need33_set55():
  log = e2e([(2500, None), (150, 33), (1200, None)], 55)
  assert first_alert_lead_s(log, 2500) >= P.ALERT_LEAD_S
  assert not gaps(log, next(r["s"] for r in log if r["alert"]), 2650, 33)
  assert all(r["line1"].startswith("SLOW DOWN") for r in log if r["alert"])
  assert not any(r["alert"] for r in log if r["s"] > 2650 + 100)          # S1: gone after the corner


def test_c_m1_iii_need24_set45():
  log = e2e([(2500, None), (150, 24), (1200, None)], 45)                   # floor 30: a corner needing 24 warns
  assert first_alert_lead_s(log, 2500) >= P.ALERT_LEAD_S
  assert not gaps(log, next(r["s"] for r in log if r["alert"]), 2650, 24)


def test_c_graded_corner_entry45_apex33():
  log = e2e([(2500, None), (100, 45), (100, 33), (1200, None)], 55)
  assert any(r["alert"] == 33 for r in log)
  assert not gaps(log, next(r["s"] for r in log if r["alert"]), 2700, 33)


# ---- (a) a brake before any lowering, RES at 30 mph about 150 m before a corner needing 32 ----------------------------------------
def test_a_brake_then_res_at_30_before_a_corner_needing_32():
  st = {}

  def ev(f, car, s):
    if 2150 <= s < 2350:
      car.mph = max(30.0, car.mph - 4.0 * DT)                              # the driver brakes to 30, cruise off
      return {"brake": s < 2340, "cruise_on": False}
    if 2350 <= s < 2352:
      st.setdefault("res", f)
    if "res" in st and st["res"] <= f < st["res"] + 10:
      return {"driver_button": True}                                       # RES (not a SET press)
    return {}
  log = e2e([(2500, None), (150, 32), (1200, None)], 55, events=ev)
  above = [r for r in log if 2400 <= r["s"] <= 2650 and r["mph"] > 32 + P.ALERT_MARGIN_MPH + 1]
  assert above, "the case needs the car to be above the corner's speed"
  assert not gaps(log, 2400, 2650, 32)


# ---- (b) a lead holds the car at 40, set 55, a corner needing 24 ------------------------------------------------------------------
def _lead40(f, car, s):
  car.mph = min(car.mph, 40.0)
  return {"lead": True}


@pytest.mark.parametrize("cap", [None, pytest.param(5, marks=pytest.mark.xfail(strict=True, reason=(
  "R16 physical limit with the realistic 8 s preview (Job 2, 2026-11-24): lead-held at 40, the unconfirmed cap holds the dash at 35 "
  "until the preview reaches the entry (~143 m), then 35 -> 25 cannot finish: 30.3 mph 50 m before the bend, 26.2 at its entry "
  "(target 25, need 24), within the warning's need + 2.5 so no warning. Passed with the old 4 s preview, which never capped. "
  "R16 is OFF since the owner's decision of 2026-11-24 (ti_curve_plan.S1_UNCONFIRMED_MAX_MPH = None); this case pins the kept cap "
  "path's limit only; strict: this flips to a failure when it passes")))])
def test_b_lead_holds_the_car_at_40_corner_needs_24(monkeypatch, cap):
  # fourth review, item 4: the lead-held floor (min(ref, speed) - 15 = 25) is the planner's too, so the corner IS slowed for
  monkeypatch.setattr(P, "S1_UNCONFIRMED_MAX_MPH", cap)
  log = e2e([(2500, None), (150, 24), (1200, None)], 55, mph0=40, events=_lead40)
  assert min(r["set"] for r in log if 2400 <= r["s"] <= 2650) <= 26
  assert not gaps(log, 2450, 2650, 24)


def test_item4_lead_held_corners_24_to_30_are_slowed_for():
  # R14 (A15) changed the expected figure for 27 and 30: the stepped need has no value between 25 and 33, so k_for(27) and k_for(30)
  # are the gentlest stepped-25 bend (k ~ 0.0099, at the step's edge), which fits at 32 on the capacity above the step: from 40 mph
  # it is now planned at 32, not 25 (old: set <= 28 / 31 and no gap above 30 / 33; new: set <= 33, no gap above 35). 24 is unchanged.
  for need in (24, 27, 30):
    planned = int(P.D.need_hi(k_for(need))[0])
    log = e2e([(2500, None), (150, need), (1200, None)], 55, mph0=40, events=_lead40)
    assert min(r["set"] for r in log if 2400 <= r["s"] <= 2650) <= max(planned, 25) + 1, need
    assert not gaps(log, 2500, 2650, planned), need                         # in the corner: slowed, or warned
  assert int(P.D.need_hi(k_for(24))[0]) == 24 and int(P.D.need_hi(k_for(27))[0]) == 32


def test_b2_lead_held_corner_below_the_floor_warns():
  log = e2e([(2500, None), (150, 20), (1200, None)], 55, mph0=40, events=_lead40)
  assert any(r["alert"] == 20 for r in log)
  assert not any("no known corner ahead" in r["line1"] for r in log if r["alert"])
  assert not gaps(log, next(r["s"] for r in log if r["alert"]), 2650, 20)


# ---- (d) a single-pass corner is like any other (owner, 2026-11-23): it warns when the car will be too fast -------------------------
def test_d_single_pass_corner_warns_like_any_other():
  log = e2e([(2500, None), (150, 33), (1200, None)], 55, single=True)
  assert any(r["alert"] == 33 for r in log)


# ---- (e) ordinary starts on plan: no warning -----------------------------------------------------------------------------------------
def test_e_ordinary_starts_never_warn():
  for need, set_ in ((45, 55), (40, 50), (50, 60), (35, 45)):
    log = e2e([(2500, None), (150, need), (1200, None)], set_)
    assert not any(r["alert"] for r in log), (need, set_)


# ---- (f) behind plan: the warning comes within the calm-braking horizon, not 3.5 s before the corner --------------------------------
def test_f_behind_plan_warns_early():
  log = e2e([(2500, None), (150, 42), (1200, None)], 55, car_cls=WeakCar)
  lead_s = first_alert_lead_s(log, 2500)
  assert lead_s is not None and lead_s >= P.ALERT_LEAD_S + 1.0, lead_s


# ---- (g) freshness: the warning cannot flicker between writes ----------------------------------------------------------------------
def test_g_freshness_window_covers_several_writes():
  assert C.ALERT_FRESH_S >= 4 * (1.0 / D.RATE_HZ)
  log = e2e([(2500, None), (150, 33), (1200, None)], 55)
  ts = [r["t"] for r in log if r["alert"]]
  assert ts and all(b - a <= 0.1 + 1e-6 for a, b in zip(ts, ts[1:]))       # one unbroken stretch: no flicker, one chime


# ---- M3: a button pause never withholds the warning; M2: a driver SET after a brake is never exceeded ------------------------------
def test_m3_a_button_pause_never_withholds_the_warning():
  base = e2e([(2500, None), (150, 33), (1200, None)], 55)
  s_press = next(r["s"] for r in base if r["alert"]) + 10.0             # a DISTANCE press while the warning is up

  def ev(f, car, s):
    return {"driver_button": True} if s_press <= s <= s_press + 4 else {}
  log = e2e([(2500, None), (150, 33), (1200, None)], 55, events=ev)
  paused = [r for r in log if r["state"] == "paused"]
  assert paused and all(r["alert"] for r in paused if r["mph"] > 35)


def test_m2_a_driver_set_after_a_brake_is_their_speed_and_never_exceeded():
  st = {}

  def ev(f, car, s):
    if 1900 <= s < 1903 and "b" not in st:
      st["b"] = f
    if "b" in st and f < st["b"] + 50:
      return {"brake": True, "cruise_on": False}
    if "b" in st and st["b"] + 50 <= f < st["b"] + 60:
      if f == st["b"] + 55:
        car.set = 52
      return {"driver_button": True, "driver_set": True, "driver_set_plus": True}
    return {}
  log = e2e([(2500, None), (150, 45), (1200, None)], 60, events=ev)
  after = [r for r in log if "b" in st and r["t"] > (st["b"] + 70) * DT]
  assert after and max(r["set"] for r in after) <= 52


# ---- fourth review, MUST 1: losing trust never drops or delays a warning (latched corners, by the odometer) ------------------------
def _trust_case(mode, a, b):
  def ev(f, car, s):
    if a <= s <= b:
      return {"_fix_valid": False} if mode == "nofix" else {"_jump": 400.0}
    return {}
  log = e2e([(2500, None), (150, 33), (1200, None)], 55, events=ev)
  st = stretches(log)
  t_c = next(r["t"] for r in log if r["s"] >= 2500)
  assert len(st) == 1, [(round(x["t"] - t_c, 1), round(y["t"] - t_c, 1)) for x, y in st]      # one stretch: one chime
  assert t_c - st[0][0]["t"] >= P.ALERT_LEAD_S, t_c - st[0][0]["t"]
  assert not gaps(log, st[0][0]["s"], 2650, 33)
  assert not any(r["alert"] for r in log if r["s"] > 2650 + 100)
  return log


def test_must1_no_fix_250_to_80_m_before():
  _trust_case("nofix", 2250, 2420)


def test_must1_fix_lost_120_to_30_m_before():
  _trust_case("nofix", 2380, 2470)


def test_must1_matcher_lost_200_to_50_m_before():
  _trust_case("jump", 2300, 2450)


def test_must1_matcher_lost_80_m_before_to_20_m_in():
  _trust_case("jump", 2420, 2520)


def test_must1_no_warning_for_a_corner_already_slowed_for_through_a_gps_gap():
  for a, b in ((2380, 2470), (2430, 2560)):
    def ev(f, car, s, a=a, b=b):
      return {"_fix_valid": False} if a <= s <= b else {}
    log = e2e([(2500, None), (150, 45), (1200, None)], 55, events=ev)
    assert not any(r["alert"] for r in log), (a, b)


# ---- fourth review, item 3: the prediction uses the measured deceleration FOR THE GAP: a weaker cruise is warned for in time ------
class Car70(T.Car):
  """the cruise at 70 % of the deceleration the planner was measured with"""

  def tick(self):
    before = self.mph
    super().tick()
    if self.mph < before:
      self.mph = before - 0.7 * (before - self.mph)


def test_item3_a_70pct_cruise_is_warned_for_early():
  for need in (30, 33, 36, 40):
    log = e2e([(2500, None), (150, need), (1200, None)], 55, car_cls=Car70)
    entry = next(r["mph"] for r in log if r["s"] >= 2500)
    if entry > need + P.ALERT_MARGIN_MPH + 1:
      lead = first_alert_lead_s(log, 2500)
      assert lead is not None and lead >= P.ALERT_LEAD_S, (need, lead)


# ---- Job 2 (2026-11-24): a position gap INSIDE a remembered bend being slowed for, with the realistic 8 s preview ----------------
@pytest.mark.parametrize("need,a,b", [(33, 2480, 2620), (40, 2470, 2600), (33, 2400, 2560)])
def test_gps_gap_inside_a_slowed_bend_never_raises_the_set_speed(need, a, b):
  # found by the previous builder: with an 8 s preview, stage 2's ceiling (55 - 5 = 50, started from the approach speed) replaced the
  # memory's lower one once the fix was lost, and the set speed was walked back up to 50 inside the bend. The lower ceiling wins while
  # any stage is active for a bend; the memory corner runs on by the odometer through the gap; no raise before end + HOLD_AFTER_M
  def ev(f, car, s):
    return {"_fix_valid": False} if a <= s <= b else {}
  log = e2e([(2500, None), (150, need), (1200, None)], 55, events=ev, preview_s=8.0)
  # (after the hold-branch merge the push may take the dash below the floor 40 to 34 and the model-timed raise brings it back to the
  # target 40 before the bend: that planned raise is allowed; nothing above the target, and no rise at all inside the bend)
  T = max(need, 55 - 15)
  i0 = next(i for i, r in enumerate(log) if r["set"] <= T)
  win = [r for r in log[i0:] if r["s"] < 2650 + P.HOLD_AFTER_M - M.SPACING_M]   # the release (end + HOLD_AFTER_M, on the memory grid)
  assert max(r["set"] for r in win) <= T, [(round(r["s"]), r["set"]) for r in win if r["set"] > T][:5]
  bend = [r for r in win if r["s"] >= 2500]
  rises = [(round(x["s"]), x["set"], y["set"]) for x, y in zip(bend, bend[1:]) if y["set"] > x["set"]]
  assert not rises, rises
  assert min(r["set"] for r in log if 2500 <= r["s"] <= 2650) == T


# ---- review C1 (2026-11-24): ANY lock loss inside a bend being slowed for never raises the set speed there -------------------------
LOSS_REASONS = ("road differs from memory", "remembered road ends", "fixes left the remembered road", "a nearer remembered road",
                "no position fix")


def _lock_loss_run(monkeypatch, why, need, at, keep=True, guard=True, car_la=False):
  """The real planner and car side; at `at` metres the lock is lost for `why` (as the planner itself would: a changed road also
  distrusts the road for CHANGE_HOLDOFF_M). keep=False / guard=False switch off the planner's two layers (dead reckoning on every
  loss; the published-ceiling guard), to show each layer (and the car side's bend rule) holds by itself."""
  st = {"s": 0.0, "done": False}
  orig_step, orig_lose, orig_guard = P.Planner.step, P.Planner._lose, P.Planner._guard_apply

  def step(self, *a, **k):
    if not st["done"] and st["s"] >= at and self.locked:
      st["done"] = True
      if why == "road differs from memory":
        self.distrust_until = self.odo + P.CHANGE_HOLDOFF_M
        self.distrust_why = why
      self._lose(why)
    return orig_step(self, *a, **k)

  monkeypatch.setattr(P.Planner, "step", step)
  if not keep:
    monkeypatch.setattr(P.Planner, "_lose", lambda self, w, keep_active=True: orig_lose(self, w, keep_active=False))
  if not guard:
    monkeypatch.setattr(P.Planner, "_guard_apply", lambda self, c, tr_, k, cm, d, e, g, u=None: (c, tr_, k, cm, d, e, False))

  def ev(f, car, s):
    st["s"] = s
    return {}

  log = e2e([(2500, None), (150, need), (1200, None)], 55, events=ev, preview_s=8.0, car_la=car_la)
  assert st["done"]
  return log


def _bend_rises(log, need, end_m=2650 + P.HOLD_AFTER_M - M.SPACING_M):
  """Rises of the dash set speed from bend entry to its end + HOLD_AFTER_M (on the memory grid; or `end_m`) that go above the target
  (the push-raise back to the target is the one allowed rise)."""
  T = max(need, 55 - 15)
  win = [r for r in log if 2500 <= r["s"] < end_m]
  return T, [(round(x["s"]), x["set"], y["set"]) for x, y in zip(win, win[1:]) if y["set"] > x["set"] and y["set"] > T]


@pytest.mark.parametrize("why", ["road differs from memory", "road ended"])
@pytest.mark.parametrize("need,at", [(33, 2520), (40, 2520), (33, 2560)])
def test_c1_review_probe_lock_loss_inside_bend_never_raises(monkeypatch, why, need, at):
  # the reviewer's six cases (probe 2): before the fix the set speed walked 40 -> 46+ inside the bend in all six
  log = _lock_loss_run(monkeypatch, why, need, at)
  bend = [r for r in log if 2500 <= r["s"] <= 2650]
  rises = [(round(x["s"]), x["set"], y["set"]) for x, y in zip(bend, bend[1:]) if y["set"] > x["set"]]
  assert not rises, rises


@pytest.mark.parametrize("why", LOSS_REASONS)
def test_c1_each_lock_loss_reason_keeps_the_corner(monkeypatch, why):
  log = _lock_loss_run(monkeypatch, why, 33, 2530)
  T, rises = _bend_rises(log, 33)
  assert not rises, rises
  assert min(r["set"] for r in log if 2500 <= r["s"] <= 2650) <= T


@pytest.mark.parametrize("why", LOSS_REASONS)
def test_c1_planner_dead_reckons_every_loss_and_records_why(why):
  pl = P.Planner(None)
  pl.active = {7: {"T": 40, "at": 100.0}}
  pl.locked = True
  pl._lose(why)
  assert 7 in pl.active and pl.dr_why == why
  assert any(e["kind"] == "dead_reckon" and e["why"] == why for e in pl.events)


@pytest.mark.parametrize("layer", ["guard_only", "car_side_only"])
@pytest.mark.parametrize("why", ["road differs from memory", "remembered road ends"])
def test_c1_each_layer_holds_by_itself(monkeypatch, layer, why):
  # guard_only: the corner is DROPPED on the loss (the old behaviour): the published-ceiling guard alone keeps the ceiling;
  # car_side_only: neither planner layer: the car side's own bend rule alone holds the dash (the loss is inside the bend)
  if layer == "guard_only":
    log = _lock_loss_run(monkeypatch, why, 33, 2530, keep=False)
  else:
    log = _lock_loss_run(monkeypatch, why, 33, 2530, keep=False, guard=False, car_la=True)
  # the car side knows the bend only from its own signals: it holds to the bend's END (HOLD_AFTER_M past it is the planner's layer)
  T, rises = _bend_rises(log, 33, end_m=2650 if layer == "car_side_only" else 2650 + P.HOLD_AFTER_M - M.SPACING_M)
  assert not rises, rises


def test_c1_property_random_loss_points_and_reasons(monkeypatch):
  rng = np.random.default_rng(20261124)
  for _ in range(10):
    why = str(rng.choice(LOSS_REASONS))
    need = int(rng.choice([30, 33, 36, 40, 44]))
    at = float(rng.uniform(2380, 2640))
    with monkeypatch.context() as mp:
      log = _lock_loss_run(mp, why, need, at, car_la=bool(rng.integers(0, 2)))
    T, rises = _bend_rises(log, need)
    assert not rises, (why, need, round(at), rises)
