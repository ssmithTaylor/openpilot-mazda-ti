"""Alert loop (owner, 2026-11-24): curve slowing lowered the set speed, the driver braked for a light and pressed RES, the restore's SET+
tap was not read back ("set speed did not respond"): the feature must NOT try that restore again by itself, and the "CRUISE SET TO x,
WAS y" alert shows once, steadily, until the driver presses SET+ / SET- (adopted as before) or the set speed is back at the reference.
A RES by the driver neither re-arms the restore nor silences / re-triggers the alert. Scenario from the investigation scratch file."""
import importlib.util
import pathlib

import pytest

from openpilot.selfdrive.car.mazda import ti_curve as C

_spec = importlib.util.spec_from_file_location("tc_car_loop", pathlib.Path(__file__).parent / "test_mazda_ti_curve.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
DT = C.DT


def scenario(resume, corner_trusted_at_stop=True, brake=True, ref=50, corner=40, stop_untrusted_s=0.0, fail_restore=False, answer=None):
  """0-25 s slowing for a corner (ceil `corner`); 25 s: the driver brakes for a light (cruise off), standstill 12 s, pulls away
  on the gas to 25 mph; then `resume` ("RES" back to the stored set speed, "SET" at 25 mph); then 90 s with no corner ahead."""
  car = T.Car(float(ref), ref)
  cv, ex = C.Curve(), C.SafeExit()
  log, alerts, prev_left, prev_state = [], [], None, None
  cruise_on, standstill = True, False
  f_brake, f_stop, f_go = int(25 / DT), int(27 / DT), int(39 / DT)
  f_btn = None
  for f in range(int(140 / DT)):
    car.tick()
    t = f * DT
    btn, dset, brk, gas = False, False, False, False
    p = T.plan(corner, corner=corner, dist=300.0) if f < f_brake else T.plan(None)
    if brake and f_brake <= f < f_stop:
      brk, cruise_on = True, False
      car.mph = max(0.0, car.mph - 12.0 * DT)
    elif f_stop <= f < f_go:
      car.mph, standstill = 0.0, True
      brk = brake
      if stop_untrusted_s:
        p = T.plan(None, trusted=False)
    elif f >= f_go and f_btn is None:
      standstill = car.mph < 0.1
      gas = True
      car.mph = min(25.0, car.mph + 3.0 * DT)
      if car.mph >= 25.0:
        f_btn = f
    if f_btn is not None and f_btn <= f < f_btn + int(0.3 / DT):
      btn = True
      dset = resume == "SET"
      if f == f_btn:
        cruise_on = True
        if resume == "SET":
          car.set = 25
    if f_btn is not None and f >= f_btn:
      standstill = False
    if fail_restore and f_btn is not None and f > f_btn + int(3 / DT):
      car.mode = "ignore"
    if answer and alerts and alerts[-1][1] is not None:
      since = f - int(round(alerts[-1][0] / DT))
      if int(1.5 / DT) <= since < int(1.8 / DT):
        btn = True
        dset = answer == "SETPLUS"
        if answer == "SETPLUS" and since == int(1.5 / DT):
          car.set += 1
    if not cruise_on and f >= f_brake:
      car.cruise.v = car.mph
    inp = C.Inputs(on=True, engaged=cruise_on, cruise_on=cruise_on, cruise_available=True, mph=car.mph, set_mph=car.set,
                   brake=brk, gas=gas, lead=False, driver_button=btn, counter=car.counter, plan=p, plan_age=0.1, mono=t,
                   standstill=standstill, driver_set=dset)
    b = C.run_frame(cv, ex, f, inp)
    for ev in cv.events + ex.events:
      if ev["kind"] in ("stop", "driver_button", "restored", "exit_armed", "exit_end", "driver_chose"):
        log.append((round(t, 2), ev))
    cv.events.clear()
    ex.events.clear()
    if b is not None and cruise_on:
      car.press(b)
    left = C.set_left(cv, ex, car.set)
    if left != prev_left:
      alerts.append((round(t, 2), left, cv.state, cv.why, car.set, cv.ref, cv.after_brake, cv.left_at, cv.left_doubt))
      prev_left = left
    if cv.state != prev_state:
      log.append((round(t, 2), ("state", cv.state, cv.why, car.set, cv.ref)))
      prev_state = cv.state
  return car, cv, ex, log, alerts


def _starts(alerts):
  return sum(1 for a in alerts if a[1] is not None)


def test_failed_restore_after_brake_and_res_alerts_once_and_never_retries():
  car, cv, ex, log, alerts = scenario("RES", fail_restore=True)
  assert _starts(alerts) == 1, alerts
  assert alerts[-1][1] is not None and len(alerts) == 1               # on, and never dropped: one steady alert
  assert sum(1 for _, ev in log if isinstance(ev, dict) and ev["kind"] == "stop" and "did not respond" in ev["why"]) == 1


def test_driver_res_answering_the_alert_does_not_rearm_or_silence_it():
  car, cv, ex, log, alerts = scenario("RES", fail_restore=True, answer="RES")
  assert _starts(alerts) == 1 and len(alerts) == 1, alerts
  assert cv.ref == 50 and car.set < 50


def test_driver_set_plus_answers_the_alert_and_is_adopted():
  car, cv, ex, log, alerts = scenario("RES", fail_restore=True, answer="SETPLUS")
  assert _starts(alerts) == 1 and alerts[-1][1] is None
  assert cv.ref == car.set == 45 and cv.restore_failed is None


def test_a_restore_that_works_is_unchanged():
  car, cv, ex, log, alerts = scenario("RES")
  assert car.set == 50 and cv.restore_failed is None and not alerts
