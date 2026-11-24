"""Review C2 (2026-11-24): the SAFE EXIT. Any stop of curve slowing while the dash is pushed below the 15-mph line (other than the
driver's own cruise button or the cruise going off) raises the dash back to the line, independent of the intervention's state
machine: after a raise tap not read back, the switch going off, the cruise check, an unexpected set-speed change, an internal error
(the Curve object gone) and a controls restart (state lost, the persisted marker found). Closed loop with the ONE cruise model, both
braking tables: the car's actual speed never goes more than 0.5 mph below the line after the stop; no tap after a driver button;
nothing ever above the driver's set speed."""
import importlib.util
import pathlib

import pytest

from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
from openpilot.selfdrive.car.mazda import ti_curve as C
from openpilot.selfdrive.car.mazda import ti_curve_plan as P

_spec = importlib.util.spec_from_file_location("tc_exit_car", pathlib.Path(__file__).parent / "test_mazda_ti_curve.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
DT = C.DT
REASONS = ("no_response", "switch_off", "check", "changed", "internal_error", "restart")


def _drive(event, depth, variant=CM.TIMING, ref=55, seconds=40.0, keep_ignoring=False, driver_at=None, monkeypatch=None):
  """The probe's run (planner stand-in: the deepest push for a corner at the floor), with the Curve and the safe exit composed as the
  car controller does (ti_curve.run_frame). The event happens the frame the dash first reads floor - depth."""
  car = T.Car(float(ref), ref)
  car.cruise = CM.Cruise(float(ref), ref, DT, variant=variant)
  cv, ex = C.Curve(), C.SafeExit()
  fl = max(C.ABS_FLOOR_MPH, ref - C.MAX_SHED_MPH)
  st = {"hit": None, "lo_all": 99.0}
  lo_after, set_hi, sent_after_driver = 99.0, 0, 0
  for f in range(int(seconds / DT)):
    car.tick()
    p = T.plan(C.ABS_FLOOR_MPH, corner=fl, dist=300.0)
    on, check, driver = True, False, False
    if st["hit"] is None and car.set <= fl - depth:
      st["hit"] = f
      if event == "changed":
        car.set -= 1                                    # a set-speed change nobody asked for
      elif event == "internal_error":
        cv = None                                       # the Curve object failed: the car controller arms the exit from its own state
        ex.arm(f, C.Inputs(set_mph=car.set), "internal error")
      elif event == "restart":
        mk = ex.marker(cv, car.set)
        assert mk is not None                           # the persisted marker exists while pushed below the line
        cv, ex = C.Curve(), C.SafeExit()                # a controls restart: all state lost, the switch cleared
        ex.arm(f, None, "controls restarted", ref=mk["ref"], floor=mk["floor"], restart=True)
    hit = st["hit"] is not None
    if hit:
      if event == "no_response":
        if car.mode == "accept" and (cv is None or cv.state != "stopped"):
          car.mode = "ignore"                           # the raise tap is not read back (a lost frame)
        elif cv is not None and cv.state == "stopped" and not keep_ignoring:
          car.mode = "accept"                           # the button path is back for the exit's taps
        p = T.plan(fl, corner=fl, dist=300.0)
      elif event in ("switch_off", "restart"):
        on = False
      elif event == "check":
        check = (f - st["hit"]) * DT < 0.5
    if driver_at is not None and hit and (f - st["hit"]) * DT >= driver_at and (f - st["hit"]) * DT < driver_at + 0.1:
      driver = True
    inp = C.Inputs(on=on, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, brake=False, gas=False,
                   lead=False, driver_button=driver, counter=car.counter, plan=p, plan_age=0.1, mono=f * DT, check_active=check)
    b = C.run_frame(cv, ex, f, inp)
    if cv is not None:
      cv.events.clear()
    if b is not None:
      car.press(b)
      if driver_at is not None and hit and (f - st["hit"]) * DT >= driver_at:
        sent_after_driver += 1
    if hit:
      lo_after = min(lo_after, car.mph)
    st["lo_all"] = min(st["lo_all"], car.mph)
    set_hi = max(set_hi, car.set)
  return car, cv, ex, st, lo_after, set_hi, fl, sent_after_driver


@pytest.mark.parametrize("variant", [CM.TIMING, CM.CONSERVATIVE])
@pytest.mark.parametrize("event", ["no_response", "switch_off", "check"])
def test_review_probe_stop_mid_push_actual_speed(event, variant):
  # the reviewer's probe 1: set 55, floor 40, dash pushed to 34, then the raise tap not read back / switch off / the cruise check:
  # before the fix the car settled at 34
  car, cv, ex, st, lo, hi, fl, _ = _drive(event, 6, variant)
  assert st["hit"] is not None
  assert lo >= fl - 0.5, (event, variant, round(lo, 2), ex.result, car.set)
  assert car.set >= fl and hi <= 55
  assert ex.result == "done", ex.result


@pytest.mark.parametrize("variant", [CM.TIMING, CM.CONSERVATIVE])
@pytest.mark.parametrize("depth", [1, 6, 10])
@pytest.mark.parametrize("event", REASONS)
def test_every_stop_reason_raises_back_to_the_line(monkeypatch, event, depth, variant):
  if depth > P.DRIVE_GAP_MPH:
    monkeypatch.setattr(P, "DRIVE_GAP_MPH", 10)        # the held-button push depth (DRIVE_GAP_MPH 10 with CM.HOLD)
  car, cv, ex, st, lo, hi, fl, _ = _drive(event, depth, variant)
  if st["hit"] is None:
    # with the timing table the push net raises the dash before it ever reads floor - 10: the stop cannot come there; the whole run
    # still never takes the car more than 0.5 mph under the line
    assert depth == 10 and variant == CM.TIMING, (event, depth, variant)
    assert st["lo_all"] >= fl - 0.5, round(st["lo_all"], 2)
    return
  assert car.set >= fl, (event, depth, variant, car.set, ex.result, ex.why)
  assert hi <= 55
  assert lo >= fl - 0.5, (event, depth, variant, round(lo, 2))


def test_exit_tap_not_read_back_fails_and_says_so():
  car, cv, ex, st, lo, hi, fl, _ = _drive("no_response", 6, keep_ignoring=True)
  assert ex.result == "failed" and not ex.active
  assert car.set < fl                                  # nothing more the code can do: the on-road alert (I4) carries it


def test_no_tap_after_a_driver_button_during_the_exit():
  car, cv, ex, st, lo, hi, fl, sent = _drive("switch_off", 6, driver_at=0.35)
  assert sent == 0 and ex.result == "driver"


@pytest.mark.parametrize("why", ["cruise off", "cruise unavailable"])
def test_cruise_off_is_exempt(why):
  ex = C.SafeExit()
  ex.ref, ex.floor = 55, 40
  cv = C.Curve()
  cv.events.append({"kind": "stop", "why": why})
  ex.observe(0, cv, C.Inputs(set_mph=34))
  assert not ex.active


@pytest.mark.parametrize("why", ["set speed did not respond", "set speed changed to 33 mph unexpectedly", "road not matched",
                                 "cruise check started", "brake pressed", "gas held", "openpilot disengaged", "set speed unreadable"])
def test_other_stop_reasons_arm_the_exit_and_it_waits_for_pedals(why):
  ex = C.SafeExit()
  ex.ref, ex.floor = 55, 40
  cv = C.Curve()
  cv.events.append({"kind": "stop", "why": why})
  ex.observe(0, cv, C.Inputs(set_mph=34))
  assert ex.active and ex.line == 40
  # it never fights: brake / gas / disengaged / standstill / slow: no press
  for over in ({"brake": True}, {"gas": True}, {"engaged": False}, {"standstill": True}, {"mph": 15.0}, {"cancel_req": True},
               {"driver_button": None}, {"check_active": True}):
    kw = dict(on=False, engaged=True, cruise_on=True, cruise_available=True, mph=45.0, set_mph=34, driver_button=False, counter=1,
              mono=1.0)
    kw.update(over)
    assert all(ex.step(f, C.Inputs(**kw)) is None for f in range(100, 200)), over
  assert ex.active


def test_nothing_above_the_reference_and_the_line_is_capped_by_it():
  assert C.exit_line(55, 40) == 40 and C.exit_line(30, None) == 25 and C.exit_line(26, 25) == 25 and C.exit_line(55, 45) == 45
  assert C.exit_line(None, 40) is None


def test_restart_without_a_reference_raises_to_the_car_speed_and_says_so():
  ex = C.SafeExit()
  assert ex.arm(0, None, "controls restarted", ref=None, floor=None, restart=True)
  car = T.Car(43.4, 30)
  for f in range(1, 1500):
    car.tick()
    b = ex.step(f, C.Inputs(on=False, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set,
                            driver_button=False, counter=car.counter, mono=f * DT))
    if f == 50:
      assert "unknown" in ex.line2
    if b:
      car.press(b)
  assert ex.result == "done" and car.set == 44           # 43.4 rounded up; never above that


def test_switch_off_behaviour_is_one_named_constant(monkeypatch):
  monkeypatch.setattr(C, "EXIT_RAISE_ON_SWITCH_OFF", False)
  car, cv, ex, st, lo, hi, fl, _ = _drive("switch_off", 6)
  assert not ex.events or all(e["kind"] != "exit_armed" for e in ex.events)
  assert car.set < fl


def test_marker_is_published_only_while_pushed_below_the_line():
  car, cv, ex = T.Car(55.0, 55), C.Curve(), C.SafeExit()
  seen = []
  for f in range(2500):
    car.tick()
    inp = C.Inputs(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, lead=False,
                   driver_button=False, counter=car.counter, plan=T.plan(C.ABS_FLOOR_MPH, corner=40, dist=300.0), plan_age=0.1,
                   mono=f * DT)
    b = C.run_frame(cv, ex, f, inp)
    cv.events.clear()
    if b:
      car.press(b)
    seen.append((car.set, ex.marker(cv, car.set)))
  assert any(m is not None for s, m in seen) and all((m is not None) == (s < 40) for s, m in seen if cv.intervening())


# ---- the car controller's hook: an internal error and a controls restart ----------------------------------------------------------
def _cc(store, monkeypatch):
  import json  # noqa: F401
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  from openpilot.selfdrive.car.mazda.values import MazdaFlags
  from openpilot.common.realtime import ControlsTimer
  spec = importlib.util.spec_from_file_location("ti_cost_check_for_exit", pathlib.Path(__file__).parents[1] / "mazda" / "ti_cost_check.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  mem = mod.MemParams(store)
  monkeypatch.setattr(ccm, "Params", lambda path=None: mem if path and "shm" in str(path) else mod.MemParams())
  ControlsTimer.objects.clear()
  cc = ccm.CarController("mazda_2017", NS(flags=int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR), carFingerprint="MAZDA_CX5",
                                          carName="mazda"), None)

  class Packer:
    def make_can_msg(self, *a, **k):
      return (0, 0, b"\0" * 8, 0)
  cc.packer = Packer()
  clock = {"k": 0}
  monkeypatch.setattr(ccm, "time", NS(monotonic=lambda: clock["k"] * 0.01, time=lambda: 1000.0 + clock["k"] * 0.01,
                                      perf_counter=lambda: 0.0))
  CC, CS = mod.inputs(45.0, 0.0)
  return cc, mem, CC, CS, clock


def _frames(cc, CC, CS, clock, n, k0=0, set_mph=None):
  cs = CS.out.cruiseState
  sent = []
  for k in range(k0, k0 + n):
    clock["k"] = k
    cc.frame = k
    if k % 10 == 3:
      CS.crz_btns_counter = (CS.crz_btns_counter + 1) % 16
    b = cc.curve_step(CC, CS)
    if b is not None:
      sent.append(k)
      set_mph[0] += 1 if (len(sent) == 1 or k - sent[-2] > 6) else 0   # the car reads one step per tap (its first pressed frame)
      cs.speed = set_mph[0] * 0.44704
  return sent


def test_controls_restart_with_the_marker_runs_the_exit(monkeypatch):
  import json
  store = {C.PARAM_STATUS: json.dumps({"state": "slowing", "set": 34, "ref": 55, "exit": {"ref": 55, "floor": 40, "line": 40}}).encode()}
  cc, mem, CC, CS, clock = _cc(store, monkeypatch)
  assert cc.curve_exit.active and cc.curve_exit.line == 40 and C.PARAM_STATUS not in mem.d   # read, then cleared
  setv = [34]
  CS.out.cruiseState.speed = 34 * 0.44704
  _frames(cc, CC, CS, clock, 1500, set_mph=setv)
  assert setv[0] == 40 and cc.curve_exit.result == "done"


def test_internal_error_mid_push_runs_the_exit_without_the_curve(monkeypatch):
  cc, mem, CC, CS, clock = _cc({}, monkeypatch)
  cv = cc.curve
  cv.state, cv.ref, cv.floor = "slowing", 55, 40      # mid-push (the dash at 34, the floor 40)
  cc.curve_exit.ref, cc.curve_exit.floor = 55, 40
  cc.curve_on = True
  mem.put(C.PARAM_ON, b"1")
  CS.out.cruiseState.speed = 34 * 0.44704
  monkeypatch.setattr(cv, "step", lambda frame, inp: (_ for _ in ()).throw(RuntimeError("boom")))
  setv = [34]
  _frames(cc, CC, CS, clock, 1, set_mph=setv)
  assert cc.curve is None and cc.curve_exit.active
  _frames(cc, CC, CS, clock, 1500, k0=1, set_mph=setv)
  assert setv[0] == 40 and cc.curve_exit.result == "done"


# ---- review I4 (2026-11-24): the on-road alert when a stopped feature left the set speed below the driver's -------------------------
class _Mem:
  def __init__(self):
    self.d = {}

  def get(self, k):
    return self.d.get(k)

  def put(self, k, v):
    self.d[k] = v.encode() if isinstance(v, str) else v


def _alert_text(mem, now):
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.controls.lib import events as ev
  val, _ = C.read_curve_code(mem, now)
  if val is None:
    return None
  return ev.steer_authority_advisory_alert(NS(), NS(), {"frogpilotPlan": NS(steerAdvisorySpeed=val)}, False, 100, NS()).alert_text_1


def _stopped(why, set_mph=40, ref=55, after_brake=False):
  cv = C.Curve()
  cv.state, cv.ref, cv.after_brake = "stopped", ref, after_brake
  cv.why = why
  return cv


@pytest.mark.parametrize("why", ["set speed did not respond", "set speed changed to 33 mph unexpectedly", "road not matched",
                                 "cruise check started"])
def test_i4_each_stopped_state_raises_the_alert(why):
  import json
  import time
  ex = C.SafeExit()
  assert C.set_left(_stopped(why), ex, 40) == (40, 55)
  mem, now = _Mem(), time.monotonic()
  mem.put(C.PARAM_LEFT, json.dumps({"m": now, "who": "curve slowing", "now": 40, "was": 55}))
  assert _alert_text(mem, now) == "CRUISE SET TO 40, WAS 55"
  assert _alert_text(mem, now + C.LEFT_FRESH_S + 0.1) is None          # stale: gone


def test_i4_internal_error_switch_off_and_failed_exit_raise_it_but_not_while_the_exit_runs():
  ex = C.SafeExit()
  ex.ref, ex.floor = 55, 40
  assert C.set_left(None, ex, 34) == (34, 55)                          # the Curve gone (internal error)
  cv = C.Curve()
  cv.state, cv.ref, cv.off_note = "off", 55, "Set speed 34 mph (was 55): set it yourself"
  assert C.set_left(cv, ex, 34) == (34, 55)                            # switched off, left lowered
  ex.active = True
  assert C.set_left(cv, ex, 34) is None                                # the exit still raising it: not yet
  ex.active, ex.result = False, "failed"
  assert C.set_left(cv, ex, 34) == (34, 55)                            # the exit's tap not read back: the driver is told


def test_i4_no_alert_once_back_or_after_a_driver_button_or_a_brake_restore():
  ex = C.SafeExit()
  assert C.set_left(_stopped("set speed did not respond"), ex, 55) is None
  assert C.set_left(_stopped("brake pressed", after_brake=True), ex, 40) is None    # restored by itself after RES
  cv = C.Curve()
  cv.state, cv.ref = "paused", 55                                      # the driver pressed a cruise button: their value stands
  assert C.set_left(cv, ex, 40) is None


def test_i4_closed_loop_exit_failure_leaves_the_alert_until_the_driver_acts():
  car, cv, ex, st, lo, hi, fl, _ = _drive("no_response", 6, keep_ignoring=True)
  assert ex.result == "failed" and C.set_left(cv, ex, car.set) == (car.set, 55)
