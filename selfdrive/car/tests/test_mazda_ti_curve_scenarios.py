"""Curve slowing STANDING REGRESSION CHECK: the fourth review's probes (h4 harness, 112-case sweep, scenario list, position-gap
cases, button safety) promoted to repo tests. Real Planner (5 Hz) + real car-side Curve (100 Hz), status relayed through the real
plannerd.car_side, the phase-aware Car of test_mazda_ti_curve.py. Synthetic roads only (no positions).

A reviewer's finding becomes a case here; a fix is verified by rerunning it:
  python tools/dev/pc_test.py -- selfdrive/car/tests/test_mazda_ti_curve_scenarios.py

Every run asserts the button-safety invariants on every frame (see `_check_press`): no press below 20 mph, at standstill, with the
cruise off or a driver button down; the set speed never above the owner's speed; no ratchet (the set speed never climbs above the
car side's reference)."""
import importlib.util
import json
import pathlib

import numpy as np
import pytest

from openpilot.selfdrive.car.mazda import ti_curve as C, ti_curve_memory as M, ti_curve_plan as P
from openpilot.selfdrive.car.mazda import ti_curve_plannerd as D

_spec = importlib.util.spec_from_file_location("tc_scen_car", pathlib.Path(__file__).parent / "test_mazda_ti_curve.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
DT = 0.01
PREVIEW_S = 8.0     # the realistic preview (the real model previews 8-10 s; under-read by FC_RATIO, published right positive); was 4.0
_KS = np.linspace(0.0005, 0.06, 12000)
_SP = P.D.need_mph(_KS)   # plan T4: ti_curve_demand (left bends, CAP_RAW)
R = [(2500, None), (150, 33), (1200, None)]      # the reference road: 2.5 km straight, a corner needing 33 mph, straight


def k_for(need):
  return float(_KS[np.argmin(np.abs(_SP - need))])


class _Store:
  raw = None

  def get(self, k):
    return self.raw


class WeakCar(T.Car):
  dec = 0.5

  def tick(self):
    b = self.mph
    super().tick()
    if self.mph < b:
      self.mph = max(self.mph, b - self.dec * DT)


class RealCar(T.Car):
  """The planner's own measured cruise model: reaction 0.5 s, decel by gap (decel_mph_s), times `scale`."""
  scale = 1.0

  def __init__(self, *a, **k):
    super().__init__(*a, **k)
    self.hist = []

  def tick(self):
    b = self.mph
    super().tick()
    self.hist.append(self.set)
    lag = self.hist[-51] if len(self.hist) > 50 else self.hist[0]
    if b > lag:
      self.mph = max(b - self.scale * P.decel_mph_s(b - lag) * DT, min(b, lag))


def _check_press(b, kw):
  assert b in ("SET_MINUS", "SET_PLUS"), ("unexpected button", b)
  assert kw["mph"] >= 20, ("press below 20 mph", kw["mph"])
  assert kw["cruise_on"], "press with the cruise off"
  assert not kw.get("driver_button"), "press while a driver button is down"
  assert not kw.get("standstill"), "press at standstill"


def e2e(segs, set_mph, mph0=None, seconds=400, events=None, single=False, car_cls=None, limit=None, status_lag_s=0.0,
        preview_s=PREVIEW_S):
  """segs: [(length m, need mph or None)]. events(f, car, s, cv) -> Inputs overrides, plus controls starting with '_':
  _fix_valid, _skip_plan (planner stalls), _model_off, _jump (m added to the fix: matcher loss), _limit. `driver_set` events
  record the owner's new speed (their choice is the ceiling from then on)."""
  tr = M.synthetic_trace([(L, 0.0 if n is None else k_for(n)) for L, n in segs])
  mem = M.synthetic([tr])
  if single:
    mem.npass[:] = 1
  pl, cv = P.Planner(mem), C.Curve()
  car = (car_cls or T.Car)(float(set_mph if mph0 is None else mph0), set_mph)
  owner = [float(set_mph)]
  s_road = np.arange(len(tr["x"])) * M.SPACING_M
  s, plan_d, plan_t, seq, log = 0.0, None, None, 0, []
  store, stq = _Store(), []
  prev_set = car.set
  for f in range(int(seconds / DT)):
    car.tick()
    if car.set > prev_set:          # the car registered a SET_PLUS (driver changes happen in events, after this)
      assert car.set <= owner[0] + 0.01, ("set above the owner's speed", f, car.set, owner[0])
      assert cv.ref is None or car.set <= cv.ref + 0.01, ("ratchet: set above the car side's reference", f, car.set, cv.ref)
    t = f * DT
    if s > s_road[-1] - 10:
      break
    set_before = car.set
    ov = (events(f, car, s, cv) or {}) if events is not None else {}
    if ov.get("driver_set") or car.set != set_before:
      owner[0] = float(car.set)
    if f % 10 == 0:
      stq.append((t, json.dumps(cv.status(t)).encode()))
    if f % 20 == 0 and not ov.get("_skip_plan"):
      while len(stq) > 1 and stq[1][0] <= t - status_lag_s:
        stq.pop(0)
      store.raw = stq[0][1]
      ref, floor, acting = D.car_side(store, t, C)
      sj = s + ov.get("_jump", 0.0)
      x, y = np.interp(sj, s_road, tr["x"]), np.interp(sj, s_road, tr["y"])
      i = min(int(sj // M.SPACING_M), len(tr["hdg"]) - 1)
      ahead = np.arange(0, preview_s + 0.01, 0.25)
      kk = np.array([tr["kap"][min(int(q // M.SPACING_M), len(tr["kap"]) - 1)] for q in s + ahead * car.mph * P.MPH])
      # as the real model publishes it (plan T5): openpilot's curvature is RIGHT positive (P.MODEL_K_SIGN), and the preview under-reads
      # the bend by the measured FC_RATIO at each lead (the planner calibrates it back)
      kk = P.MODEL_K_SIGN * kk * np.interp(ahead, P.D.FC_LEAD_S, P.D.FC_RATIO)
      mk = None if ov.get("_model_off") else (ahead, kk)
      p = pl.step(t, P.Fix(ov.get("_fix_valid", True), float(x), float(y), float(tr["hdg"][i])), car.mph * P.MPH,
                  set_mph=car.set if ov.get("cruise_on", True) else None, yaw_k=float(tr["kap"][i]), model_k=mk,
                  ref_mph=ref, floor_mph=floor, acting=acting, limit_mph=ov.get("_limit", limit))
      plan_d, plan_t = P.plan_json(p, t, seq, "act"), t
      seq += 1
    kw = dict(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, lead=False,
              driver_button=False, counter=car.counter, plan=plan_d, plan_age=None if plan_t is None else t - plan_t, mono=t)
    kw.update({k: v for k, v in ov.items() if not k.startswith("_")})
    b = cv.step(f, C.Inputs(**kw))
    if b:
      _check_press(b, kw)
      car.press(b)
    s += car.mph * P.MPH * DT
    prev_set = car.set
    if f % 10 == 0:
      st = cv.status(t)
      log.append(dict(t=round(t, 2), s=round(s, 1), state=cv.state, why=cv.why, set=car.set, mph=round(car.mph, 2), alert=st["alert"],
                      need=(plan_d or {}).get("need"), ref=cv.ref))
  return log, car, cv


def corner(log, c0, c1, need):
  """Seconds of warning before entry, entry / peak speed, rows in the corner above need+2.5 with no warning."""
  t_c = next(r["t"] for r in log if r["s"] >= c0)
  entry = next(r["mph"] for r in log if r["s"] >= c0)
  peak = max(r["mph"] for r in log if c0 <= r["s"] <= c1)
  first = next((r for r in log if r["alert"] and c0 - 2500 <= r["s"] <= c1), None)
  lead = None if first is None else t_c - first["t"]
  unc_in = sum(1 for r in log if c0 <= r["s"] <= c1 and r["mph"] > need + 2.5 and not r["alert"])
  return dict(entry=entry, peak=peak, lead=lead, unc_in=unc_in)


def assert_covered(log, c0, c1, need, min_lead=None):
  """Either the car reaches the corner within 2.5 mph of its need, or it is warned in time and for the whole corner."""
  pn = next((x["need"] for x in log if x["need"] is not None and c0 - 2500 <= x["s"] <= c1), None)
  need = pn if pn is not None else need    # the planner's own need for the corner (as the sweep), else the road's label
  r = corner(log, c0, c1, need)
  if r["peak"] > need + 2.5:
    assert r["lead"] is not None, ("no warning", r)
    assert r["lead"] >= (P.ALERT_LEAD_S if min_lead is None else min_lead) - 0.05, ("late warning", r)
    assert r["unc_in"] == 0, ("unwarned in the corner", r)
  return r


def _no_late_warning(log, c1):
  assert sum(1 for r in log if r["s"] > c1 + 60 and r["alert"]) == 0, "warning more than 60 m after the corner"


# ---------------------------------------------------------------- 112-case sweep: no false warnings, no misses, minimum lead
SWEEP = [(st, need) for st in (30, 35, 40, 45, 50, 55, 60, 65) for need in range(21, st + 1, 2)]
assert len(SWEEP) == 112


@pytest.mark.parametrize("st,need", SWEEP)
def test_sweep_no_false_warning_no_miss(st, need):
  log, car, cv = e2e([(2500, None), (150, need), (800, None)], st)
  nd = next((r["need"] for r in log if r["need"] is not None), None) or need
  r = corner(log, 2500, 2650, nd)
  first = next((x for x in log if x["alert"] and x["s"] > 1000), None)
  if r["peak"] <= nd + 2.0:
    assert first is None, ("false warning", st, need, nd, r)
  if r["peak"] > nd + 2.5:
    assert first is not None and r["lead"] >= P.ALERT_LEAD_S and r["unc_in"] == 0, ("miss / late", st, need, nd, r)


# ---------------------------------------------------------------- scenario list
@pytest.mark.parametrize("brake_at,res_gap,to", [(2000, 250, 45), (2150, 150, 42), (2250, 100, 45), (2300, 60, 45), (2380, 40, 40)])
def test_brake_then_res_near_a_corner(brake_at, res_gap, to):
  st = {}

  def ev(f, car, s, cv):
    if brake_at <= s < 2500 - res_gap and "res" not in st:
      car.mph = max(float(to), car.mph - 4.0 * DT)
      return {"brake": True, "cruise_on": False}
    if s >= 2500 - res_gap and "res" not in st:
      st["res"] = f
    if "res" in st and st["res"] <= f < st["res"] + 10:
      return {"driver_button": True}
    return {}
  log, car, cv = e2e(R, 55, events=ev)
  assert_covered(log, 2500, 2650, 33, min_lead=5.0)
  _no_late_warning(log, 2650)


@pytest.mark.parametrize("need,hold", [(24, 40), (30, 40), (28, 35)])
def test_lead_holds_the_car(need, hold):
  def ev(f, car, s, cv):
    car.mph = min(car.mph, float(hold))
    return {"lead": True}
  log, car, cv = e2e([(2500, None), (150, need), (1200, None)], 55, mph0=hold, events=ev)
  assert_covered(log, 2500, 2650, need, min_lead=5.0)
  assert max(x["set"] for x in log) <= 55


@pytest.mark.parametrize("kind,gap", [("distance", 350), ("distance", 200), ("setminus", 400), ("setminus", 250), ("setplus", 300)])
def test_driver_button_pause(kind, gap):
  st = {}

  def ev(f, car, s, cv):
    if s >= 2500 - gap and "p" not in st:
      st["p"] = f
      if kind == "setminus":
        car.set -= 1
      if kind == "setplus":
        car.set += 1
    if "p" in st and st["p"] <= f < st["p"] + 10:
      return {"driver_button": True, "driver_set": kind != "distance", "driver_set_plus": kind == "setplus"}
    return {}
  log, car, cv = e2e(R, 55, events=ev)
  assert_covered(log, 2500, 2650, 33, min_lead=5.0)
  _no_late_warning(log, 2650)


@pytest.mark.parametrize("a,b,top", [(2200, 2300, 55), (2300, 2420, 50), (2400, 2480, 47)])
def test_gas_before_a_corner(a, b, top):
  def ev(f, car, s, cv):
    if a <= s <= b:
      car.mph = min(float(top), car.mph + 3.0 * DT)
      return {"gas": True}
    return {}
  log, car, cv = e2e(R, 55, events=ev)
  assert_covered(log, 2500, 2650, 33, min_lead=5.0)
  assert car.set <= 55


@pytest.mark.parametrize("g,n1,n2", [(300, 42, 33), (150, 40, 30), (400, 45, 35), (250, 38, 33)])
def test_two_corners(g, n1, n2):
  log, car, cv = e2e([(2500, None), (120, n1), (g, None), (120, n2), (1200, None)], 55)
  c2 = 2500 + 120 + g
  r = corner(log, c2, c2 + 120, n2)
  if r["peak"] > n2 + 2.5:
    assert r["lead"] is not None and r["unc_in"] == 0, r
  assert corner(log, 2500, 2620, n1)["unc_in"] == 0
  _no_late_warning(log, c2 + 120)


@pytest.mark.parametrize("m0,need,d", [(40, 33, 1800), (45, 33, 2300), (35, 30, 2400), (45, 40, 2300)])
def test_set_above_car_speed(m0, need, d):
  log, car, cv = e2e([(d, None), (150, need), (1200, None)], 55, mph0=m0)
  assert_covered(log, d, d + 150, need, min_lead=5.0)
  _no_late_warning(log, d + 150)


@pytest.mark.parametrize("st,need", [(22, 19), (25, 21), (28, 22), (30, 23), (26, 19), (24, 20)])
def test_just_above_20_mph(st, need):
  log, car, cv = e2e([(1500, None), (150, need), (800, None)], st)
  r = corner(log, 1500, 1650, need)
  assert r["unc_in"] == 0, r
  assert min(x["set"] for x in log) >= 20 or min(x["set"] for x in log) == st


@pytest.mark.parametrize("lag", [0.5, 1.0, 2.5])
def test_late_status(lag):
  log, car, cv = e2e(R, 55, status_lag_s=lag)
  assert_covered(log, 2500, 2650, 33)
  _no_late_warning(log, 2650)


@pytest.mark.parametrize("a,b", [(2300, 2330), (2380, 2420), (2450, 2470)])
def test_planner_stalls_never_leave_the_corner_unwarned(a, b):
  log, car, cv = e2e(R, 55, events=lambda f, car, s, cv: {"_skip_plan": True} if a <= s <= b else {})
  assert corner(log, 2500, 2650, 33)["unc_in"] == 0


# ---------------------------------------------------------------- the six position-gap cases (fourth review, MUST 1)
def _intervals(log, c0):
  t_c = next(r["t"] for r in log if r["s"] >= c0)
  out, on = [], None
  for r in log:
    if r["alert"] and on is None:
      on = r
    if not r["alert"] and on is not None:
      out.append((on["t"] - t_c, r["t"] - t_c))
      on = None
  return out


@pytest.mark.parametrize("mode,a,b", [("nofix", 2250, 2420), ("nofix", 2380, 2470), ("jump", 2200, 2320), ("jump", 2300, 2450),
                                      ("jump", 2420, 2520), ("jump", 2460, 2600)])
def test_position_gap_one_unbroken_warning(mode, a, b):
  def ev(f, car, s, cv):
    if a <= s <= b:
      return {"_fix_valid": False} if mode == "nofix" else {"_jump": 400.0}
    return {}
  log, car, cv = e2e(R, 55, events=ev)
  iv = _intervals(log, 2500)
  assert len(iv) == 1, ("warning broken or missing", iv)
  assert iv[0][0] <= -6.5, ("warning starts late", iv)
  assert sum(1 for r in log if 2500 <= r["s"] <= 2650 and r["mph"] > 35 and not r["alert"]) == 0


# ---------------------------------------------------------------- slow: cruise-model variants of the sweep
@pytest.mark.slow
@pytest.mark.parametrize("cls", [WeakCar, RealCar])
def test_sweep_cruise_variants_never_unwarned_in_corner(cls):
  for st in (35, 45, 55, 65):
    for need in range(21, st + 1, 4):
      log, car, cv = e2e([(2500, None), (150, need), (800, None)], st, car_cls=cls)
      nd = next((r["need"] for r in log if r["need"] is not None), None) or need
      assert corner(log, 2500, 2650, nd)["unc_in"] == 0, (cls.__name__, st, need)
