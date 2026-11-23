"""Curve slowing, car side (ti_curve.py): every rule against a simulated stock cruise. Synthetic only."""
import random

import pytest

from openpilot.selfdrive.car.mazda import ti_curve as C
from openpilot.selfdrive.car.mazda.ti_cruise_taps import DT, MAX_PRESSED_FRAMES, MAX_TAP_SPAN_S, MIN_TAP_GAP_S


class Car:
  """The stock cruise as the car controller sees it: counter at 10 Hz, set speed read 4 frames after a press's first frame, a press
  lasting 0.52 s read as a hold (jump to a multiple of 5); the speed follows the set speed at ~1.2 mph/s after 0.5 s."""

  def __init__(self, mph=55.0, set_mph=55, mode="accept"):
    self.k = 0
    self.counter = 0
    self.mph = mph
    self.set = set_mph
    self.mode = mode
    self.pending = []
    self.press_start = None
    self.last_press = None
    self.sent = []              # (k, button)
    self.history = []

  def tick(self):
    self.k += 1
    if self.k % 10 == 3:
      self.counter = (self.counter + 1) % 16
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
    new = self.last_press is None or self.k - self.last_press > 15
    self.last_press = self.k
    if new:
      self.press_start = self.k
    if self.mode == "ignore":
      return
    step = -1 if button == "SET_MINUS" else 1
    if new:
      self.pending.append((self.k + 4, self.set + step))
    elif self.k - self.press_start >= 52:
      self.pending.append((self.k + 1, (self.set // 5 + (0 if step < 0 else 1)) * 5))


def plan(ceil=None, trusted=True, known=True, model_la=0.2, past=None, limit=None, dist=100.0):
  return {"ceil": ceil, "trusted": trusted, "known": known, "why": "", "dist": dist, "past": ceil is None if past is None else past,
          "model_la": model_la, "capped": False, "limit": limit, "conf": 1.0 if trusted else 0.3}


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
  assert min(h[1] for h in car.history) == 35          # 50 - 15
  c2, car2 = C.Curve(), Car(40, 40)
  run(c2, car2, 4000, lambda f, car: plan(20, limit=35))
  assert min(h[1] for h in car2.history) == C.ABS_FLOOR_MPH


def test_no_restore_while_model_sees_a_bend_or_lead_unreadable():
  c, car = C.Curve(), Car(55, 55)
  run(c, car, 6000, lambda f, car: plan(45) if f < 2000 else plan(None, model_la=1.5))
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
  run(c, car, 1000, lambda f, car: plan(45), driver_button=btn)
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
  ups = [k for k, b in car.sent if b == "SET_PLUS"]
  assert all((b - a) * DT >= C.RESTORE_GAP_S - 1e-9 for a, b in zip(ups, ups[1:]))


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
      return plan(cur, model_la=rng.choice([0.1, 0.1, 1.4]), limit=limit)

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
                     driver_button=f < state["btn_until"], counter=car.counter, plan=pf(f, car),
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
