"""Curve slowing END TO END: the real Planner (5 Hz) and the real car-side Curve (100 Hz) together, the car side's status relayed to the
planner as ti_curve_plannerd does (its reference always, its floor while it intervenes), and a phase-aware simulated cruise (the Car of
test_mazda_ti_curve.py). Synthetic roads only."""
import importlib.util
import pathlib

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve as C, ti_curve_memory as M, ti_curve_plan as P

_spec = importlib.util.spec_from_file_location("tc_e2e_car", pathlib.Path(__file__).parent / "test_mazda_ti_curve.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
DT = 0.01


def corner_k(need_mph):
  ks = np.linspace(0.001, 0.05, 5000)
  sp = P.corner_mph(ks)
  return float(ks[np.argmin(np.abs(sp - need_mph))])


def e2e(need_mph, set_mph, seconds=260, straight=2500.0, corner_m=150.0, events=None):
  """Run; events(f, car, s) -> dict of Inputs overrides (driver_button, driver_set, brake, cruise_on ...). Returns the log:
  [(t, s, state, set, mph, status dict, plan dict)] every 0.1 s."""
  k = corner_k(need_mph)
  tr = M.synthetic_trace([(straight, 0.0), (corner_m, k), (1200, 0.0)])
  mem = M.synthetic([tr])
  pl, cv, car = P.Planner(mem), C.Curve(), T.Car(set_mph, set_mph)
  s_road = np.arange(len(tr["x"])) * M.SPACING_M
  s, plan_d, plan_t, seq, log = 0.0, None, None, 0, []
  for f in range(int(seconds / DT)):
    car.tick()
    t = f * DT
    if s > s_road[-1] - 10:
      break
    if f % 20 == 0:
      x, y = np.interp(s, s_road, tr["x"]), np.interp(s, s_road, tr["y"])
      i = min(int(s // M.SPACING_M), len(tr["hdg"]) - 1)
      st = cv.status(t)
      floor = st.get("floor") if st.get("state") in ("slowing", "holding", "restoring") else None
      ahead = np.arange(0, 4.01, 0.25)
      kk = np.array([tr["kap"][min(int(q // M.SPACING_M), len(tr["kap"]) - 1)] for q in s + ahead * car.mph * P.MPH])
      p = pl.step(t, P.Fix(True, float(x), float(y), float(tr["hdg"][i])), car.mph * P.MPH, set_mph=car.set,
                  yaw_k=float(tr["kap"][i]), model_k=(ahead, kk), ref_mph=st.get("ref"), floor_mph=floor)
      plan_d, plan_t = P.plan_json(p, t, seq, "act"), t
      seq += 1
    kw = dict(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, lead=False,
              driver_button=False, counter=car.counter, plan=plan_d, plan_age=None if plan_t is None else t - plan_t, mono=t)
    if events is not None:
      kw.update(events(f, car, s) or {})
    b = cv.step(f, C.Inputs(**kw))
    if b:
      car.press(b)
    s += car.mph * P.MPH * DT
    if f % 10 == 0:
      log.append((t, s, cv.state, car.set, car.mph, cv.status(t), dict(plan_d or {}), cv.line2))
  return log


def test_m1_warning_holds_through_the_approach_and_needs_more_shows():
  # owner's set speed 55 (floor 40), a corner needing ~33: the warning must stay up while the car is above 35, to the corner, and
  # NEEDS MORE must show; once the corner is passed the warning is gone (S1)
  log = e2e(33, 55)
  approach = [r for r in log if 1500 <= r[1] <= 2500]
  first = next(i for i, r in enumerate(approach) if r[5]["alert"])
  after_first = approach[first:]
  assert all(r[5]["alert"] for r in after_first if r[4] > 33 + P.ALERT_MARGIN_MPH + 1), \
    [(round(r[1]), round(r[4], 1), r[5]["alert"]) for r in after_first if not r[5]["alert"]][:5]
  assert any("NEEDS MORE" in r[7] for r in approach)
  assert not any(r[5]["alert"] for r in log if r[1] > 2650 + 100)
  # it is up early: at least ALERT_LEAD_S before the corner
  t_first, t_corner = approach[first][0], next(r[0] for r in log if r[1] >= 2500)
  assert t_corner - t_first >= P.ALERT_LEAD_S


def test_m3_a_button_pause_never_withholds_the_warning():
  def ev(f, car, s):
    if 2330 <= s <= 2334:                                       # a DISTANCE press on the approach, inside the warning
      return {"driver_button": True}
    return {}
  base = e2e(33, 55)
  s_first = next(r[1] for r in base if r[5]["alert"])          # where the warning comes up without the press
  log = e2e(33, 55, events=ev)
  paused = [r for r in log if r[2] == "paused" and s_first <= r[1] < 2500]
  assert paused, "the press must fall inside the warning's stretch for this test"
  assert all(r[5]["alert"] for r in paused if r[4] > 35)


def test_m2_a_driver_set_after_a_brake_is_their_speed_and_never_exceeded():
  state = {"brake_until": None}

  def ev(f, car, s):
    if 1900 <= s < 1903 and state["brake_until"] is None:
      state["brake_until"] = f + 50                              # a brake on the approach, set speed lowered
    if state["brake_until"] is not None and f < state["brake_until"]:
      return {"brake": True, "cruise_on": False}
    if state["brake_until"] is not None and state["brake_until"] <= f < state["brake_until"] + 10:
      if f == state["brake_until"] + 5:
        car.set = 52                                            # the driver sets the cruise afresh with SET+ at 52
      return {"driver_button": True, "driver_set": True, "driver_set_plus": True}
    return {}
  log = e2e(45, 60, events=ev)
  after = [r for r in log if state["brake_until"] is not None and r[0] > (state["brake_until"] + 20) * DT]
  assert after and max(r[3] for r in after) <= 52
