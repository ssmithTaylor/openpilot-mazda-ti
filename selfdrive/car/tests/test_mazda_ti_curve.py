"""Curve slowing, car side (ti_curve.py): every rule against a simulated stock cruise. Synthetic only."""
import random

import pytest

from openpilot.selfdrive.car.mazda import ti_curve as C
from openpilot.selfdrive.car.mazda.ti_cruise_taps import DT, MAX_PRESSED_FRAMES, MAX_TAP_SPAN_S, MIN_TAP_GAP_S


class Car:
  """The stock cruise as the car controller sees it, PHASE-AWARE: the button module sends its own idle (not pressed) 0x09D frame at
  10 Hz (k % 10 == 3, counter + 1). A press registers on a pressed frame that follows an idle one, so two pressed frames with an idle
  frame between them register as TWO presses. A press is read 4 frames after its first frame; an unbroken press lasting 0.52 s reads
  as a hold (jump to a multiple of 5). Modes: accept, ignore, hold_misread (every press reads as a hold), double (a SET_PLUS press
  registers +2). The speed follows the set speed at ~1.2 mph/s down and 0.7 up."""

  def __init__(self, mph=55.0, set_mph=55, mode="accept"):
    self.k = 0
    self.counter = 0
    self.mph = mph
    self.set = set_mph
    self.mode = mode
    self.pending = []
    self.press_start = None
    self.pressed_now = False      # the last frame on the bus was a pressed one
    self.sent = []                # (k, button)
    self.registered = 0           # presses the car registered
    self.history = []

  def tick(self):
    self.k += 1
    if self.k % 10 == 3:
      self.counter = (self.counter + 1) % 16
      self.pressed_now = False    # the module's own idle frame
    for p in [p for p in self.pending if p[0] <= self.k]:
      self.pending.remove(p)
      self.set = p[1]
    target = self.set
    if self.mph > target:
      self.mph = max(target, self.mph - 1.2 * DT)
    elif self.mph < target:
      self.mph = min(target, self.mph + 0.7 * DT)

  def press(self, button):
    self.sent.append((self.k, button))
    new = not self.pressed_now
    self.pressed_now = True
    if new:
      self.press_start = self.k
    if self.mode == "ignore":
      return
    step = -1 if button == "SET_MINUS" else 1
    if new:
      self.registered += 1
      if self.mode == "hold_misread":
        self.pending.append((self.k + 4, (self.set // 5 + (0 if step < 0 else 1)) * 5))
      elif self.mode == "double" and step > 0:
        self.pending.append((self.k + 4, self.set + 2))
      else:
        self.pending.append((self.k + 4, self.set + step))
    elif self.k - self.press_start >= 52:
      self.pending.append((self.k + 1, (self.set // 5 + (0 if step < 0 else 1)) * 5))


def plan(ceil=None, trusted=True, known=True, model_k=0.0005, past=None, limit=None, dist=100.0, kind="corner", corner=None, capped=False):
  return {"ceil": ceil, "trusted": trusted, "known": known, "why": "", "dist": dist, "past": ceil is None if past is None else past,
          "model_k": model_k, "capped": capped, "limit": limit, "conf": 1.0 if trusted else 0.3, "kind": kind,
          "corner": ceil if corner is None else corner}


def run(curve, car, frames, plan_fn, start=0, **over):
  """Run frames; plan_fn(frame, car) -> plan dict (or None); over: Inputs fields as values or callables(frame, car)."""
  outs = []
  for f in range(start, start + frames):
    car.tick()
    kw = dict(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, brake=False, gas=False,
              lead=False, driver_button=False, counter=car.counter, plan=plan_fn(f, car), plan_age=0.1, mono=f * DT)
    for k, v in over.items():
      kw[k] = v(f, car) if callable(v) else v
    if kw["plan"] is None and "plan_age" not in over:
      kw["plan_age"] = None
    b = curve.step(f, C.Inputs(**kw))
    if b is not None:
      car.press(b)
    outs.append((f, b, car.set, curve.state))
    car.history.append((f, car.set, curve.state, curve.ref))
  return outs


def taps(car):
  """Group the sent frames into taps."""
  out = []
  for k, b in car.sent:
    if out and k - out[-1][-1][0] <= 10 and out[-1][-1][1] == b:
      out[-1].append((k, b))
    else:
      out.append([(k, b)])
  return out


def test_off_sends_nothing_and_says_off():
  c, car = C.Curve(), Car()
  out = run(c, car, 3000, lambda f, car: plan(40), on=False)
  assert not car.sent
  assert c.state == "off" and "OFF" in c.line1


def test_slows_to_ceiling_then_holds_then_restores():
  c, car = C.Curve(), Car(55, 55)
  phase = lambda f, car: plan(45) if f < 2500 else plan(None)
  run(c, car, 6000, phase)
  sets = [h[1] for h in car.history]
  assert min(sets) == 45
  assert sets[-1] == 55
  assert all(b == "SET_MINUS" for k, b in car.sent if k < 2500)
  assert c.state == "ready"


def test_taps_follow_the_reviewed_caps():
  c, car = C.Curve(), Car(60, 60)
  run(c, car, 3000, lambda f, car: plan(46))
  tt = taps(car)
  assert len(tt) == 14
  for t in tt:
    assert len(t) <= C.PRESSED_FRAMES <= MAX_PRESSED_FRAMES
    assert (t[-1][0] - t[0][0]) * DT <= MAX_TAP_SPAN_S + 1e-9
  for a, b in zip(tt, tt[1:]):
    assert (b[0][0] - a[-1][0]) * DT >= MIN_TAP_GAP_S - 1e-9
  # prompt: 14 mph applied in well under 6 s
  last = max(k for k, _ in car.sent)
  assert last * DT < 6.0


def test_floor_from_speed_when_limit_unknown():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 4000, lambda f, car: plan(30))
  assert min(h[1] for h in car.history) == 40          # 55 - 15
  assert c.needs_more and "NEEDS MORE" in c.line2


def test_floor_from_known_limit():
  c, car = C.Curve(), Car(60, 60)
  run(c, car, 4000, lambda f, car: plan(30, limit=50))
  assert min(h[1] for h in car.history) == 45          # max(60 - 15, 50 - 15): a known limit never adds slowing
  c2, car2 = C.Curve(), Car(40, 40)
  run(c2, car2, 4000, lambda f, car: plan(20, limit=35))
  assert min(h[1] for h in car2.history) == C.ABS_FLOOR_MPH


def test_no_restore_while_model_sees_a_bend_or_lead_unreadable():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 6000, lambda f, car: plan(45) if f < 2000 else plan(None, model_k=0.05))
  assert car.history[-1][1] == 45 and c.state == "restoring" and "bend" in c.why
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 6000, lambda f, car: plan(45) if f < 2000 else plan(None), lead=lambda f, car: None if f >= 2000 else False)
  assert car.history[-1][1] == 45                       # lead flag unreadable: no restore
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 6000, lambda f, car: plan(45) if f < 2000 else plan(None), lead=lambda f, car: f >= 2000)
  assert car.history[-1][1] == 55                       # following a car: restored (never above the driver's 55)


def test_lead_still_lowers_toward_ceiling():
  c, car = C.Curve(), Car(45, 55)                       # following: 45 mph with the set speed 55
  run(c, car, 3000, lambda f, car: plan(48), lead=True)
  assert min(h[1] for h in car.history) == 48


def test_driver_button_ends_and_pauses_and_adopts():
  c, car = C.Curve(), Car(55, 55)
  def btn(f, car):
    if f == 600:
      car.set = 50                                      # the driver's own press lands
    return 590 <= f < 610
  run(c, car, 1000, lambda f, car: plan(45), driver_button=btn, driver_set=True)
  assert c.state == "paused"
  assert not [k for k, b in car.sent if k > 600]
  run(c, car, 3400, lambda f, car: plan(None), start=1000, driver_button=False)
  assert c.ref == 50                                    # their speed, never the old 55
  assert max(h[1] for h in car.history if h[0] > 600) <= 50


def test_unexpected_change_stops_without_restore():
  c, car = C.Curve(), Car(55, 55)
  def bump(f, car):
    if f == 300:
      car.set = 40                                      # a jump nobody asked for
    return False
  run(c, car, 4000, lambda f, car: plan(45) if f < 1500 else plan(None), driver_button=bump)
  assert c.state == "stopped" and "unexpectedly" in c.why
  assert "set it yourself" in c.line2
  assert not [k for k, b in car.sent if k > 300]


def test_not_read_back_stops():
  c, car = C.Curve(), Car(55, 55, mode="ignore")
  run(c, car, 1000, lambda f, car: plan(45))
  assert c.state == "stopped" and "did not respond" in c.why
  assert len(taps(car)) == 1


@pytest.mark.parametrize("field,value,why", [("brake", True, "brake"), ("engaged", False, "disengaged"),
                                             ("cruise_on", False, "cruise off"), ("cruise_available", False, "unavailable")])
def test_stops(field, value, why):
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 1500, lambda f, car: plan(45), **{field: lambda f, car: value if f >= 400 else (not value)})
  assert c.state == "stopped" and why in c.why
  assert not [k for k, b in car.sent if k > 400]


def test_short_gas_tolerated_long_gas_stops():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 1500, lambda f, car: plan(45), gas=lambda f, car: 300 <= f < 350)
  assert c.state in ("slowing", "holding")
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 1500, lambda f, car: plan(45), gas=lambda f, car: 300 <= f < 500)
  assert c.state == "stopped" and "gas" in c.why


def test_planner_dead_or_late_while_lowered_stops():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 1500, lambda f, car: plan(45), plan_age=lambda f, car: 0.1 if f < 800 else 5.0)
  assert c.state == "stopped" and "planner" in c.why
  assert not [k for k, b in car.sent if k > 800]
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 1500, lambda f, car: plan(45), plan_age=lambda f, car: 0.1 if f < 800 else 1.5)
  assert c.state == "stopped" and "late" in c.why


def test_low_confidence_restores_gently_only_with_model():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 6000, lambda f, car: plan(45) if f < 2000 else plan(None, trusted=False, known=True))
  assert car.history[-1][1] == 55
  assert c.state == "ready"


def test_unknown_road_or_untrusted_sends_nothing():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 3000, lambda f, car: plan(None, trusted=False, known=False))
  assert not car.sent and "road not known" in c.line1
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 3000, lambda f, car: plan(40, trusted=False))
  assert not car.sent


def test_cancel_resume_and_driver_frames_block_presses():
  for over in ({"cancel_req": True}, {"resume_req": True}, {"driver_button": None}, {"counter": None}, {"check_active": True}):
    c, car = C.Curve(), Car(55, 55)
    run(c, car, 1000, lambda f, car: plan(45), **over)
    assert not car.sent, over


def test_never_cancel_or_resume():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 6000, lambda f, car: plan(45) if f < 2500 else plan(None))
  assert {b for _, b in car.sent} <= {"SET_MINUS", "SET_PLUS"}


def test_property_never_above_reference_never_below_floor():
  """Random scenarios: ceilings, leads, gas blips, driver presses, plan gaps, unexpected jumps; the set speed the feature produces is
  never above the driver's last own set speed and never below the floor; a press never comes within the reviewed tap gap."""
  rng = random.Random(7)
  for trial in range(150):
    c = C.Curve()
    start = rng.choice([35, 45, 55, 62, 70])
    car = Car(start, start, mode=rng.choice(["accept"] * 4 + ["ignore"]))
    driver_ref = [start]
    events = {}
    for _ in range(rng.randint(0, 3)):
      events[rng.randrange(200, 5000)] = rng.choice(["button", "jump", "gas", "lead", "gap", "brake"])
    ceil_schedule = sorted((rng.randrange(0, 5000), rng.choice([None, None, start - rng.randint(1, 25)])) for _ in range(6))
    ceil_schedule = [(0, None)] + ceil_schedule
    limit = rng.choice([None, None, start, start + 5])
    state = {"lead": False, "gas_until": -1, "gap_until": -1, "btn_until": -1, "brake_until": -1}

    def pf(f, car):
      cur = None
      for t0, cc in ceil_schedule:
        if f >= t0:
          cur = cc
      kind = rng.choice(["corner", "corner", "hold"])
      return plan(cur, model_k=rng.choice([0.0005, 0.0005, 0.03]), limit=limit, kind=kind)

    def ev(f, car):
      e = events.get(f)
      if e == "button":
        state["btn_until"] = f + 20
        car.set = max(25, car.set + rng.choice([-3, 2, 5]))
        driver_ref[0] = car.set
      elif e == "jump":
        car.set = max(25, car.set - 5)
      elif e == "gas":
        state["gas_until"] = f + rng.choice([50, 200])
      elif e == "lead":
        state["lead"] = not state["lead"]
      elif e == "gap":
        state["gap_until"] = f + rng.choice([50, 150, 400])
      elif e == "brake":
        state["brake_until"] = f + 100
      return None

    for f in range(0, 6000):
      ev(f, car)
      car.tick()
      inp = C.Inputs(on=True, engaged=True, cruise_on=f >= state["brake_until"], cruise_available=True, mph=car.mph, set_mph=car.set,
                     brake=f < state["brake_until"], gas=f < state["gas_until"], lead=state["lead"],
                     driver_button=f < state["btn_until"], driver_set=f < state["btn_until"], counter=car.counter, plan=pf(f, car),
                     plan_age=5.0 if f < state["gap_until"] else 0.1, mono=f * DT)
      before = car.set
      b = c.step(f, inp)
      if b is not None:
        assert b in ("SET_MINUS", "SET_PLUS")
        if b == "SET_PLUS":
          assert before + 1 <= driver_ref[0], (trial, f, before, driver_ref[0])
          assert before + 1 <= c.ref
        else:
          assert c.floor is not None and before - 1 >= max(C.ABS_FLOOR_MPH, c.floor), (trial, f)
          base = limit if limit else None
          if base:
            assert before - 1 >= base - C.MAX_SHED_MPH
        assert not inp.driver_button and not inp.brake
        car.press(b)
    for t in taps(car):
      assert len(t) <= MAX_PRESSED_FRAMES and (t[-1][0] - t[0][0]) * DT <= MAX_TAP_SPAN_S + 1e-9
    tt = taps(car)
    for a, b2 in zip(tt, tt[1:]):
      assert (b2[0][0] - a[-1][0]) * DT >= MIN_TAP_GAP_S - 1e-9


def test_status_reflects_state():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 150, lambda f, car: plan(45))
  st = c.status(1000.0)
  assert st["state"] == "slowing" and "SLOWING FOR A CORNER: 55 -> 45 mph" == st["line1"]
  run(c, car, 3000, lambda f, car: plan(45), start=150)
  assert c.status(0)["state"] == "holding"


def test_push_then_raise_to_corner_speed():
  c, car = C.Curve(), Car(55, 55)
  # the planner asks for 40 (push, 6 below the corner's 46), then 46 once the car is near it
  run(c, car, 4000, lambda f, car: plan(40, corner=46) if car.mph > 47.5 else plan(46, corner=46))
  sets = [h[1] for h in car.history]
  assert min(sets) == 40 and sets[-1] == 46
  assert "55 -> 46 mph" in c.line1
  assert all(h[1] <= 55 for h in car.history)


def test_hold_ceiling_never_slows_and_caps_restore():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 2000, lambda f, car: plan(45, kind="hold"))
  assert not car.sent                                   # a hold ceiling alone never lowers the set speed
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 6000, lambda f, car: plan(40) if f < 2000 else plan(48, kind="hold"))
  assert car.history[-1][1] == 48 and c.state == "holding"
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 6000, lambda f, car: plan(40) if f < 2000 else plan(41, kind="hold"))
  assert car.history[-1][1] == 40                       # under 2 mph: not worth a restore


# ---- the car controller's hook (carcontroller.curve_step), with the cost check's in-memory params ---------------------------------
def _controller():
  import importlib.util
  import pathlib
  spec = importlib.util.spec_from_file_location("ti_cost_check_for_curve_hook",
                                                pathlib.Path(__file__).parents[1] / "mazda" / "ti_cost_check.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def test_controller_clears_the_switch_at_start_and_starts_off():
  mod = _controller()
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  store = mod.MemParams({C.PARAM_ON: b"1", C.PARAM_STATUS: b"{}"})
  orig = ccm.Params
  ccm.Params = lambda path=None: store if path and "shm" in str(path) else mod.MemParams()
  try:
    from types import SimpleNamespace as NS
    from openpilot.selfdrive.car.mazda.values import MazdaFlags
    cc = ccm.CarController("mazda_2017", NS(flags=int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR), carFingerprint="MAZDA_CX5",
                                            carName="mazda"), None)
  finally:
    ccm.Params = orig
  assert C.PARAM_ON not in store.d and C.PARAM_STATUS not in store.d
  assert cc.curve is not None and cc.curve.state == "off" and not cc.curve_on


def test_controller_sends_taps_only_when_on_and_excludes_the_cruise_check():
  mod = _controller()
  on = mod.run_curve("curve: on, working", "on, working", 2000)
  off = mod.run_curve("curve: off", "off", 2000)
  assert on["taps"] >= 5 and off["taps"] == 0
  cc, mem = mod.make_controller()
  cc.curve_on = True
  assert cc.cruise_others_on() == "curve slowing is on"


# ---- review fixes (2026-11-23): tap alignment, low speed, trust loss, pacing, messages ------------------------------------------
@pytest.mark.parametrize("direction", ["down", "up"])
def test_every_tap_is_aligned_and_registers_once(direction):
  c, car = C.Curve(), Car(55, 55)
  if direction == "down":
    run(c, car, 3000, lambda f, car: plan(41, corner=41))
  else:
    run(c, car, 6000, lambda f, car: plan(41, corner=41) if f < 2500 else plan(None))
  tt = taps(car)
  assert tt and car.registered == len(tt)               # one registered press per tap: never press-release-press
  for t in tt:
    assert (t[0][0] - 3) // 10 == (t[-1][0] - 3) // 10  # no module idle frame between a tap's pressed frames
    assert t[0][0] % 10 == 3                            # sent right after the module's new frame (the cruise check's alignment)
  assert c.state != "stopped", c.why
  assert max(h[1] for h in car.history) <= 55
  if direction == "up":
    assert any(b == "SET_PLUS" for _, b in car.sent) and car.history[-1][1] == 55


def test_set_above_reference_is_corrected_down():
  c, car = C.Curve(), Car(55, 55, mode="double")
  run(c, car, 8000, lambda f, car: plan(45, corner=45) if f < 2500 else plan(None))
  assert car.history[-1][1] <= 55                       # +2 on a restore tap: put back by single aligned SET_MINUS taps
  assert "Nothing was changed" not in c.line2


@pytest.mark.parametrize("over", [{"mph": 15.0}, {"standstill": True}])
def test_no_press_at_low_speed_or_standstill(over):
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 2500, lambda f, car: plan(45, corner=45))
  n = len(car.sent)
  run(c, car, 3000, lambda f, car: plan(None), start=2500, **over)          # stop-and-go with the set speed lowered
  assert len(car.sent) == n


def test_trust_lost_during_approach_holds_until_past_the_corner():
  c, car = C.Curve(), Car(55, 55)

  def pf(f, car):
    if f < 300:
      return plan(45, corner=45, dist=400.0 - f * 0.01 * 24.6)
    return plan(None, trusted=False)
  run(c, car, 1500, pf)
  assert not [k for k, b in car.sent if b == "SET_PLUS"]
  assert c.state == "holding" and "past the corner" in c.why
  run(c, car, 9000, pf, start=1500)
  assert car.history[-1][1] == 55                       # restored once well past where the corner ended


def test_restore_is_paced_to_the_car():
  c, car = C.Curve(), Car(55, 55)
  seen = []

  def pf(f, car):
    seen.append((f, car.mph, car.set))
    return plan(40, corner=40) if f < 2500 else plan(None)
  run(c, car, 9000, pf)
  ups = {k for k, b in car.sent if b == "SET_PLUS"}
  by_k = {f + 1: (mph, setv) for f, mph, setv in seen}
  for k in ups:
    if k in by_k:
      mph, setv = by_k[k]
      assert setv + 1 <= mph + C.RESTORE_LEAD_MPH + 0.5
  assert car.history[-1][1] == 55


def test_switch_off_mid_intervention_says_what_is_left():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 2500, lambda f, car: plan(45, corner=45))
  run(c, car, 200, lambda f, car: plan(45, corner=45), start=2500, on=False)
  assert c.state == "off" and "set it yourself" in c.line2 and "(was 55)" in c.line2


def test_gas_override_message():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 1500, lambda f, car: plan(45, corner=45), gas=lambda f, car: f >= 1200)
  assert c.state == "stopped" and "left at" in c.line2 and "set it yourself" in c.line2


def test_state_flips_are_not_logged_every_cycle():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 3000, lambda f, car: plan(50, corner=50), mph=lambda f, car: 51.0 + (0.6 if (f // 3) % 2 else -0.6))
  n = sum(1 for e in c.events if e["kind"] == "state")
  assert n < 20, n


def test_property_with_missed_presses_misreads_standstill_switch_off_and_trust_loss():
  rng = random.Random(11)
  for trial in range(120):
    c = C.Curve()
    start = rng.choice([40, 50, 60, 70])
    car = Car(start, start, mode=rng.choice(["accept", "accept", "accept", "hold_misread", "double", "ignore"]))
    ref = [start]
    ev = {rng.randrange(100, 5000): rng.choice(["missed_press", "button", "standstill", "off", "trust", "jump"]) for _ in range(3)}
    st = {"standstill_until": -1, "off_until": -1, "trust_until": -1, "btn_until": -1}
    for f in range(6000):
      e = ev.get(f)
      if e == "missed_press":
        car.set = max(25, car.set + rng.choice([-2, 3]))           # the driver's press the button flag missed
        ref[0] = car.set
      elif e == "button":
        st["btn_until"] = f + 20
        car.set = max(25, car.set + rng.choice([-3, 2]))
        ref[0] = car.set
      elif e == "standstill":
        st["standstill_until"] = f + 400
      elif e == "off":
        st["off_until"] = f + 300
      elif e == "trust":
        st["trust_until"] = f + 600
      elif e == "jump":
        car.set = max(25, car.set - 5)
      car.tick()
      slow = f < st["standstill_until"]
      if f < st["trust_until"]:
        p = plan(None, trusted=False)
      else:
        p = plan(start - 12 if (f // 1500) % 2 == 0 else None, corner=start - 8, dist=200.0)
      inp = C.Inputs(on=f >= st["off_until"], engaged=True, cruise_on=True, cruise_available=True, mph=0.0 if slow else car.mph,
                     set_mph=car.set, lead=False, driver_button=f < st["btn_until"], driver_set=f < st["btn_until"],
                     counter=car.counter, plan=p, plan_age=0.1,
                     mono=f * DT, standstill=slow)
      before = car.set
      b = c.step(f, inp)
      if b is not None:
        assert not slow and inp.mph >= C.MIN_MPH and inp.on
        if b == "SET_PLUS":
          assert before + 1 <= ref[0], (trial, f, before, ref[0])
        else:
          assert before - 1 >= C.ABS_FLOOR_MPH
        car.press(b)
    tt = taps(car)
    for t in tt:
      assert len(t) <= MAX_PRESSED_FRAMES and (t[-1][0] - t[0][0]) * DT <= MAX_TAP_SPAN_S + 1e-9
    for a_, b_ in zip(tt, tt[1:]):
      assert (b_[0][0] - a_[-1][0]) * DT >= MIN_TAP_GAP_S - 1e-9
    if car.mode == "accept":
      assert car.registered == len(tt), trial                       # aligned: one registered press per tap


def test_floor_never_more_than_15_below_the_speed_whatever_the_limit():
  for lim, mph, want in ((50, 60, 45), (35, 55, 40), (None, 55, 40), (70, 55, 55)):
    c, car = C.Curve(), Car(mph, mph)
    run(c, car, 4000, lambda f, car: plan(25, corner=25, limit=lim))
    assert min(h[1] for h in car.history) == max(C.ABS_FLOOR_MPH, want), (lim, mph)


def test_controller_restart_mid_intervention_says_check_the_set_speed():
  mod = _controller()
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  import json as _json
  store = mod.MemParams({C.PARAM_STATUS: _json.dumps({"state": "slowing", "set": 48, "ref": 62}).encode()})
  orig = ccm.Params
  ccm.Params = lambda path=None: store if path and "shm" in str(path) else mod.MemParams()
  try:
    from types import SimpleNamespace as NS
    from openpilot.selfdrive.car.mazda.values import MazdaFlags
    cc = ccm.CarController("mazda_2017", NS(flags=int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR), carFingerprint="MAZDA_CX5",
                                            carName="mazda"), None)
  finally:
    ccm.Params = orig
  st = cc.curve.status(0.0)
  assert st["state"] == "off" and "may be lowered" in st["line2"]


# ---- second review (2026-11-23): no ratchet across stops and resumes ----------------------------------------------------------
def _cycle(c, car, start, corner_ceil, f0, n=3000):
  return run(c, car, n, lambda f, car: plan(corner_ceil, corner=corner_ceil), start=f0)


def test_brake_and_resume_cannot_walk_the_set_speed_down():
  c, car = C.Curve(), Car(60, 60)
  f = 0
  for k in range(4):
    run(c, car, 2500, lambda f_, car: plan(30, corner=30), start=f)                  # a corner wanting 30: floor 45
    f += 2500
    run(c, car, 50, lambda f_, car: plan(None), start=f, brake=True, cruise_on=False)  # brake: the cruise cancels
    f += 50

    def res(f_, car, f1=f):
      return f_ < f1 + 10                                  # the driver's RES: back to the stored (lowered) set speed
    run(c, car, 200, lambda f_, car: plan(None), start=f, driver_button=res)          # RES: not a SET button
    f += 200
    assert min(h[1] for h in car.history) >= 45, (k, min(h[1] for h in car.history))
    assert c.ref == 60
  # and with no corner, the set speed comes back to the driver's 60
  run(c, car, 9000, lambda f_, car: plan(None), start=f)
  assert car.history[-1][1] == 60


def test_own_press_after_a_stop_is_a_new_choice():
  c, car = C.Curve(), Car(60, 60)
  run(c, car, 2500, lambda f, car: plan(30, corner=30))
  run(c, car, 50, lambda f, car: plan(None), start=2500, brake=True, cruise_on=False)

  def setm(f, car):
    if f == 2560:
      car.set = 44                                          # the driver's SET-: a new choice
    return 2555 <= f < 2565
  run(c, car, 4000, lambda f, car: plan(None), start=2550, driver_button=setm, driver_set=True)
  assert c.ref == 44 and car.history[-1][1] == 44          # adopted, never restored above it


def test_lead_held_car_floor_uses_the_car_speed():
  c, car = C.Curve(), Car(45, 60)
  run(c, car, 3000, lambda f, car: plan(25, corner=25), lead=True)
  assert min(h[1] for h in car.history) == 30              # min(60, 45) - 15


def test_high_stale_limit_is_not_used():
  c, car = C.Curve(), Car(45, 45)
  run(c, car, 3000, lambda f, car: plan(30, corner=30, limit=70))
  assert min(h[1] for h in car.history) == 30              # a 70 limit at 45 mph is implausible: the corner target is honoured


def test_property_never_more_than_15_below_the_drivers_last_choice():
  rng = random.Random(23)
  for trial in range(150):
    c = C.Curve()
    start = rng.choice([45, 55, 65, 75])
    car = Car(start, start)
    choice = [start]
    seen_max = [start]
    f = 0
    for _ in range(rng.randint(3, 8)):
      e = rng.choice(["corner", "corner", "brake_res", "press", "other_button", "nothing"])
      if e == "corner":
        ce = rng.randint(25, start)
        run(c, car, rng.randint(500, 3000), lambda f_, car, ce=ce: plan(ce, corner=ce), start=f)
        f = car.k
      elif e == "brake_res":
        run(c, car, 40, lambda f_, car: plan(None), start=f, brake=True, cruise_on=False)
        f = car.k
        f1 = f
        run(c, car, 100, lambda f_, car: plan(None), start=f, driver_button=lambda f_, car, f1=f1: f_ < f1 + 10)
        f = car.k
      elif e == "press":
        f1 = f
        new = max(25, car.set + rng.choice([-5, -2, 3]))

        def press(f_, car, f1=f1, new=new):
          if f_ == f1 + 5:
            car.set = new
          return f1 <= f_ < f1 + 10
        run(c, car, 3500, lambda f_, car: plan(None), start=f, driver_button=press, driver_set=True)
        choice[0] = new
        seen_max[0] = max(seen_max[0], new)
        f = car.k
      elif e == "other_button":
        # DISTANCE / MODE / CANCEL+RES / RES, during a corner or after: never the driver's choice of speed
        f1 = f
        ce = rng.choice([None, rng.randint(25, start)])
        run(c, car, 2500, lambda f_, car, ce=ce: plan(ce, corner=ce), start=f, driver_button=lambda f_, car, f1=f1: f1 + 300 <= f_ < f1 + 310)
        f = car.k
      else:
        run(c, car, rng.randint(200, 2000), lambda f_, car: plan(None), start=f)
        f = car.k
      lo = max(C.ABS_FLOOR_MPH, choice[0] - C.MAX_SHED_MPH)
      assert all(h[1] >= min(lo, choice[0]) for h in car.history[-50:]), (trial, choice[0], car.set)
      assert all(h[1] <= choice[0] for h in car.history[-50:]), (trial, choice[0], car.set)   # never above their last choice
    assert min(h[1] for h in car.history) >= max(C.ABS_FLOOR_MPH, min(start, choice[0]) - C.MAX_SHED_MPH) - 0, trial
    assert max(h[1] for h in car.history) <= seen_max[0]



# ---- the OFF page shows what the planner sees, and never implies a press --------------------------------------------------------
@pytest.mark.parametrize("p,age,want", [
  (None, None, "OFF. Planner not running"),
  (dict(plan(None, trusted=False, known=False), why="road not known here"), 0.1, "OFF. Road not known here"),
  (dict(plan(None, trusted=False, known=True), why="position uncertain"), 0.1, "OFF. No position yet"),
  (plan(None), 0.1, "OFF. Road known here: no slowdown needed ahead"),
  (plan(41, corner=41, dist=640.0), 0.1, "OFF. Corner in 0.4 mi: would slow 48 -> 41 mph"),
  (plan(44, kind="hold", dist=800.0), 0.1, "OFF. Corner in 0.5 mi: would slow 48 -> 44 mph"),
  (plan(52, corner=52), 0.1, "OFF. Road known here: no slowdown needed ahead"),
])
def test_off_page_shows_what_the_planner_sees(p, age, want):
  c = C.Curve()
  c.off_view(p, age, 48, 50.0)
  st = c.status(0.0)
  assert st["state"] == "off" and st["line1"] == "CURVE SLOWING OFF" and st["line2"] == want


def test_off_page_never_implies_a_press():
  rng = random.Random(5)
  c, car = C.Curve(), Car(55, 55)
  for f0 in range(0, 6000, 300):
    ce = rng.choice([None, 30, 45, 50])
    run(c, car, 300, lambda f, car, ce=ce: plan(ce, corner=ce, kind=rng.choice(["corner", "hold"]),
                                               trusted=rng.random() > 0.2, known=rng.random() > 0.1), start=f0, on=False)
    st = c.status(0.0)
    text = (st["line1"] + " " + st["line2"]).upper()
    assert st["state"] == "off" and not car.sent
    for word in ("SLOWING FOR", "RESTORING", "HOLDING", "LOWERING", "PAUSED", "STOPPED"):
      assert word not in text, text
    assert st["line2"].startswith("OFF.") and len(st["line2"]) <= 52


def test_cruise_set_afresh_at_a_new_speed_is_a_new_choice():
  c, car = C.Curve(), Car(60, 60)
  run(c, car, 2500, lambda f, car: plan(30, corner=30))
  run(c, car, 50, lambda f, car: plan(None), start=2500, brake=True, cruise_on=False)
  car.set = 52                                              # SET at the current speed: told by the BUTTON, not by the value
  run(c, car, 3000, lambda f, car: plan(None), start=2550, driver_button=lambda f, car: f < 2560, driver_set=True)
  assert c.ref == 52 and c.left_at is None and car.history[-1][1] == 52
  # the same new value without a SET press (cruise on by RES, value unexplained): not adopted, nothing pressed
  c2, car2 = C.Curve(), Car(60, 60)
  run(c2, car2, 2500, lambda f, car: plan(30, corner=30))
  run(c2, car2, 50, lambda f, car: plan(None), start=2500, brake=True, cruise_on=False)
  car2.set = 52
  n = len(car2.sent)
  run(c2, car2, 3000, lambda f, car: plan(None), start=2550)
  assert c2.ref == 60 and len(car2.sent) == n and car2.history[-1][1] == 52



@pytest.mark.parametrize("when", ["during", "after_stop"])
def test_distance_mode_cancel_res_never_set_the_reference(when):
  c, car = C.Curve(), Car(60, 60)
  if when == "during":
    run(c, car, 1500, lambda f, car: plan(30, corner=30))                                 # lowering toward 45
    run(c, car, 3000, lambda f, car: plan(None), start=1500, driver_button=lambda f, car: f < 1510)   # a DISTANCE press
  else:
    run(c, car, 2500, lambda f, car: plan(30, corner=30))
    run(c, car, 50, lambda f, car: plan(None), start=2500, brake=True, cruise_on=False)
    run(c, car, 3000, lambda f, car: plan(None), start=2550, driver_button=lambda f, car: f < 2560)   # RES / MODE / CANCEL
  assert c.ref == 60
  # a later corner can still only go to 45 (60 - 15), never 15 below the lowered value
  run(c, car, 3000, lambda f, car: plan(25, corner=25), start=car.k)
  assert min(h[1] for h in car.history) >= 45


def test_driver_set_at_exactly_the_lowered_value_is_a_new_choice():
  c, car = C.Curve(), Car(60, 60)
  run(c, car, 2500, lambda f, car: plan(30, corner=30))
  low = car.set
  run(c, car, 50, lambda f, car: plan(None), start=2500, brake=True, cruise_on=False)
  # SET- pressed with the cruise coming on at exactly the lowered value: a new choice, never restored upward
  run(c, car, 4000, lambda f, car: plan(None), start=2550, driver_button=lambda f, car: f < 2560, driver_set=True)
  assert c.ref == low and car.history[-1][1] == low


def test_no_restore_without_trust_unless_past_the_corner():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 1500, lambda f, car: plan(45, corner=45, dist=60.0))
  # trust lost with the corner's end unknown to the car side: no blind restore
  c.corner_end_odo = None
  run(c, car, 3000, lambda f, car: plan(None, trusted=False), start=1500)
  assert not [k for k, b in car.sent if b == "SET_PLUS"]


def test_needs_more_only_when_the_floor_binds():
  # behind plan, corner reachable within the floor: a mild line, never "NEEDS MORE"
  c, car = C.Curve(), Car(50, 50)
  run(c, car, 200, lambda f, car: dict(plan(39, corner=45), behind=True))
  assert "NEEDS MORE" not in c.line2 and "as fast as the cruise allows" in c.line2 and not c.needs_more
  # the corner's speed below the floor (50 - 15 = 35): "NEEDS MORE"
  c, car = C.Curve(), Car(50, 50)
  run(c, car, 200, lambda f, car: plan(30, corner=30))
  assert c.needs_more and "NEEDS MORE" in c.line2


def test_brake_then_res_puts_the_drivers_speed_back_promptly():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 1200, lambda f, car: plan(42, corner=42))
  run(c, car, 1300, lambda f, car: plan(None), start=1200)          # partly restored, then...
  low = car.set
  run(c, car, 50, lambda f, car: plan(30, corner=30), start=2500, brake=True, cruise_on=False)   # a brake for a light
  f0 = car.k

  def res(f, car):
    return f < f0 + 10
  # RES at speed on an UNKNOWN road (no corner): fast taps back to 55, no wait for the car to catch up
  run(c, car, 1500, lambda f, car: plan(None, trusted=False, known=False), start=f0, driver_button=res)
  sets = [h[1] for h in car.history if h[0] >= f0]
  assert sets[-1] == 55 and c.ref == 55
  t_back = next(h[0] for h in car.history if h[0] >= f0 and h[1] == 55) - f0
  assert t_back * DT < (55 - min(sets)) * 0.5 + 3.0, t_back     # about tap pace, not the car's pace


def test_brake_then_res_with_a_real_corner_ahead_keeps_it_low():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 2500, lambda f, car: plan(42, corner=42))
  run(c, car, 50, lambda f, car: plan(42, corner=42), start=2500, brake=True, cruise_on=False)
  f0 = car.k
  run(c, car, 1000, lambda f, car: plan(42, corner=42), start=f0, driver_button=lambda f, car: f < f0 + 10)
  assert max(h[1] for h in car.history if h[0] >= f0 + 50) <= 42


def test_pause_after_a_driver_button_is_10_s():
  assert C.PAUSE_S == 10.0


def test_m2_any_set_press_defines_the_drivers_speed():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 2500, lambda f, car: plan(30, corner=30))                     # lowered to 40
  run(c, car, 50, lambda f, car: plan(None), start=2500, brake=True, cruise_on=False)
  f0 = car.k

  def res_then_setplus(f, car):
    if f == f0 + 300:
      car.set = car.set + 5                                                   # SET+ from 40 gives 45, still below their 55
    return f < f0 + 10 or f0 + 295 <= f < f0 + 305
  run(c, car, 3000, lambda f, car: plan(None), start=f0, driver_button=res_then_setplus,
      driver_set=lambda f, car: f0 + 295 <= f < f0 + 305, driver_set_plus=lambda f, car: f0 + 295 <= f < f0 + 305)
  # M2 (reviewed rule restored): any SET press defines their speed; the page says it went down; never raised above it
  assert c.ref == 49 and max(h[1] for h in car.history if h[0] > f0 + 400) <= 49
  # SET- below is a real new choice
  c2, car2 = C.Curve(), Car(55, 55)

  def setm(f, car):
    if f == 205:
      car.set = 48
    return 200 <= f < 210
  run(c2, car2, 2000, lambda f, car: plan(None), driver_button=setm, driver_set=True)
  assert c2.ref == 48


def test_r1_warning_uses_the_car_sides_floor_after_brake_and_res():
  # owner's set speed 55; a brake, RES at 40 with the car at 38; a corner needing 32 (below the car side's floor 40): the car side
  # warns, from its own floor (the planner's own floor would have been 38 - 15 = 23 and stayed silent)
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 2500, lambda f, car: plan(30, corner=30))
  run(c, car, 50, lambda f, car: plan(None), start=2500, brake=True, cruise_on=False)
  car.mph = 38.0
  f0 = car.k
  run(c, car, 300, lambda f, car: plan(32, corner=32, dist=150.0), start=f0, driver_button=lambda f, car: f < f0 + 10,
      mph=lambda f, car: 38.0)
  st = c.status(0.0)
  assert c.ref == 55 and c.floor == 40 and c.needs_more and st["alert"] == 32
  # nothing to warn about once the car is at the corner's speed
  run(c, car, 50, lambda f, car: plan(32, corner=32, dist=50.0), start=car.k, mph=lambda f, car: 33.0)
  assert c.status(0.0)["alert"] is None


def test_alert_text_shows_the_true_number():
  import numpy as np_
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.controls.lib import events as E
  sm = {"frogpilotPlan": NS(steerAdvisorySpeed=-float(np_.float32(30 * 0.44704)))}   # negative: curve slowing's warning
  a = E.steer_authority_advisory_alert(None, None, sm, False, 0, None)
  assert a.alert_text_1 == "SLOW DOWN: corner ahead needs 30 mph"
  sm = {"frogpilotPlan": NS(steerAdvisorySpeed=float(np_.float32(33 * 0.44704)))}    # the model advisory: 5-mph steps as before
  a = E.steer_authority_advisory_alert(None, None, sm, False, 0, None)
  assert a.alert_text_1 == "Slow to 30 mph for the corner ahead"
