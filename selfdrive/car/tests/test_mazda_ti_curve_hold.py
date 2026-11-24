"""Curve slowing with the HELD cruise button (plan A7 / A10): the hold's path and caps, driver priority, fallbacks, the cruise model's
hold profile and deeper push, the push floors, and that our own frames never reach the driver-press classification. Synthetic, with
interleaved frame patterns taken from recorded drives (0x09D: the car's module at 10 Hz, counter + 1 per frame; ours at its last
counter + 1, followed about 0.1 s later by the module's own unpressed frame with the same counter)."""
import pytest

from openpilot.selfdrive.car.mazda import carstate as CSm
from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
from openpilot.selfdrive.car.mazda import ti_cruise_taps as TT
from openpilot.selfdrive.car.mazda import ti_curve as C
from openpilot.selfdrive.car.mazda import ti_curve_plan as P
from openpilot.selfdrive.car.tests.test_mazda_ti_curve import plan

DT = TT.DT


# ---- the model: hold profile ---------------------------------------------------------------------------------------------------
def test_hold_landing_rules_match_the_recorded_holds():
  # measured: 45 -> 44 -> 40 -> 35; 46 -> 45 -> 40; 47 -> 46 -> 45; up 45 -> 46 -> 50; 49 -> 50 -> 55
  assert CM.hold_first_landing(45, -1) == 40 and CM.hold_first_landing(46, -1) == 40 and CM.hold_first_landing(47, -1) == 45
  assert CM.hold_first_landing(45, 1) == 50 and CM.hold_first_landing(49, 1) == 55 and CM.hold_first_landing(51, 1) == 55
  assert CM.hold_landing(45, 40) == (40, pytest.approx(0.55))
  assert CM.hold_landing(45, 41) is None                       # under 5 mph to go: taps
  assert CM.hold_landing(44, 40) is None
  assert CM.hold_landing(45, 38) == (40, pytest.approx(0.55))   # 35 would pass the goal: released at 40, the rest tapped
  assert CM.hold_landing(47, 40) == (40, pytest.approx(1.15))
  assert CM.hold_landing(45, 30) == (30, pytest.approx(1.75))
  assert CM.hold_landing(45, 25)[0] == 25 and CM.hold_landing(55, 25)[0] == 35   # the press fits HOLD_MAX_S (2.5 s): 4 landings
  assert CM.hold_landing(30, 35) == (35, pytest.approx(0.55)) and CM.hold_landing(33, 40) == (40, pytest.approx(1.15))
  for s0 in range(25, 80):
    for g in range(25, 80):
      h = CM.hold_landing(s0, g)
      if h is not None:
        stop, t = h
        assert abs(g - s0) >= 5 and stop % 5 == 0 and (g <= stop < s0 if g < s0 else s0 < stop <= g)   # never past the goal
        assert t + CM.HOLD_READ_S <= TT.HOLD_MAX_S + 1e-9


def test_dash_walk_with_the_hold():
  assert [CM.dash_after(45, 35, t, hold=True) for t in (0.0, 0.05, 0.5, 0.55, 1.1, 1.15, 5.0)] == [45, 44, 44, 40, 40, 35, 35]
  # 45 -> 37: hold to 40 (0.55 s), then taps one per TAP_S from there
  assert [CM.dash_after(45, 37, t, hold=True) for t in (0.55, 0.84, 0.85, 1.15, 1.45, 3.0)] == [40, 40, 39, 38, 37, 37]
  assert CM.dash_after(45, 42, 0.0, hold=True) == CM.dash_after(45, 42, 0.0) == 44     # under 5 to go: the tap walk
  assert CM.dash_after(30, 40, 1.15, hold=True) == 40


def _dash_run(v0, T, gap, hold, car_variant=CM.TIMING, dt=0.01, horizon=40.0):
  """The push-and-raise the planner times with the TIMING table, executed on a car braking like `car_variant`.
  -> list of (t, x, v_timing_car, v_car)."""
  S1 = max(CM.PUSH_ABS_FLOOR_MPH, T - gap)
  plan_car, car = CM.Cruise(v0, v0, dt), CM.Cruise(v0, v0, dt, car_variant)
  walk = CM.Walk(v0, S1, 0.0, CM.TAP_S, hold)
  raised, t, x, chk, out = False, 0.0, 0.0, 0.0, []
  while t < horizon:
    dash = walk.at(t)
    if not raised and t >= walk.next_t(t) - 1e-9 and t >= chk - 1e-9 and plan_car.v <= T + CM.RAISE_CHECK_MPH:
      raised, chk = CM.raise_due(plan_car, dash, T, CM.TAP_S, hold), t + CM.RAISE_CHECK_S
      if raised:
        walk = CM.Walk(dash, T, t, CM.TAP_S, hold)
        dash = walk.at(t)
    plan_car.step(dash)
    car.step(dash)
    x += car.v * CM.MPH * dt
    t += dt
    out.append((t, x, plan_car.v, car.v))
  return out


CASES = ((45, 43), (45, 40), (45, 35), (45, 30), (55, 40))


@pytest.mark.parametrize("v0,T", CASES)
@pytest.mark.parametrize("hold,gap", ((False, 6), (True, 10)))
def test_model_timed_raise_settles_at_target(v0, T, hold, gap):
  # plan A9 / A10: with the timing table the car never dips more than 0.5 mph below T (taps, push 6; hold, push 10)
  run = _dash_run(v0, T, gap, hold)
  assert min(r[3] for r in run) >= T - 0.5, min(r[3] for r in run)


# Whole manoeuvre to within the planner's ARRIVED_MPH (0.5), timing table, dt 0.01 (plan A10 item e): taps / push 6 vs hold / push 10
TIMES = {(45, 40): (4.99, 4.07), (45, 35): (7.95, 5.83), (45, 30): (10.38, 8.79), (55, 40): (10.35, 7.69)}


@pytest.mark.parametrize("v0,T", list(TIMES))
def test_whole_manoeuvre_taps_vs_hold(v0, T):
  taps, held = TIMES[(v0, T)]
  t1, _ = CM.manoeuvre(v0, T, max(25, T - 6), arrived=P.ARRIVED_MPH, dt=0.01)
  t2, _ = CM.manoeuvre(v0, T, max(25, T - 10), arrived=P.ARRIVED_MPH, dt=0.01, hold=True)
  assert t1 == pytest.approx(taps, abs=0.05) and t2 == pytest.approx(held, abs=0.05), (t1, t2)
  assert t2 < t1


# Worst case: the start is timed with the TIMING table; a car braking like the CONSERVATIVE one is over T when the timing car arrives
LATE_MPH = {False: 3.8, True: 6.4}     # measured 3.70 / 6.30 mph over T (see the report); NOT within a warning margin


@pytest.mark.parametrize("hold,gap", ((False, 6), (True, 10)))
def test_conservative_car_lateness_is_bounded(hold, gap):
  worst = 0.0
  for v0, T in CASES:
    run = _dash_run(v0, T, gap, hold, CM.CONSERVATIVE)
    i = next(k for k, r in enumerate(run) if r[2] <= T + P.ARRIVED_MPH)
    worst = max(worst, run[i][3] - T)
    assert min(r[3] for r in run) >= T - 0.5
  assert worst <= LATE_MPH[hold], worst


def test_planner_plans_with_the_walk_it_will_get():
  assert CM.HOLD == TT.HOLD_ENABLED
  assert P.DRIVE_GAP_MPH == (10 if TT.HOLD_ENABLED else 6)
  assert CM.PUSH_ABS_FLOOR_MPH == C.ABS_FLOOR_MPH and CM.PUSH_CAR_MIN_MPH == P.CAR_MIN_MPH


def test_push_floor():
  assert CM.push_floor(40, 10) == (30 if CM.PUSH_BELOW_MAX_DROP else 40)
  orig = CM.PUSH_BELOW_MAX_DROP
  try:
    CM.PUSH_BELOW_MAX_DROP = True
    assert CM.push_floor(40, 10) == 30 and CM.push_floor(30, 10) == 25 and CM.push_floor(25, 6) == 25
    CM.PUSH_BELOW_MAX_DROP = False
    assert CM.push_floor(40, 10) == 40
  finally:
    CM.PUSH_BELOW_MAX_DROP = orig


# ---- the car side --------------------------------------------------------------------------------------------------------------
class BusCar:
  """The stock cruise unit on the bus. The button module sends its own unpressed 0x09D every 10 control frames (counter + 1). Our
  frames carry the module's last counter + 1. Modes (how the cruise unit might read OUR stream; the recorded drives cannot tell):
    hold      a frame repeating the last counter is ignored, so our stream is one unbroken press: 1 mph after 5 frames, the next
              multiple of 5 at 55, then 5 per 60 frames while pressed (pressed = one of our frames within the last 15 frames)
    each      every frame counts: each of our frames after the module's unpressed one is a new press (1 mph 4 frames later)
    once      only the first of our frames in a stream registers (1 mph), the rest are ignored
    taps_only a stream of more than one frame is ignored entirely (taps work)
  The speed follows the set speed as ti_cruise_model says."""

  def __init__(self, mph=45.0, set_mph=45, mode="hold"):
    self.k, self.counter, self.mph, self.set, self.mode = 0, 0, mph, set_mph, mode
    self.cruise = CM.Cruise(mph, set_mph, DT)
    self.ours = []                # (k, button, counter sent)
    self.pending = []
    self.stream = None            # [start k, button, last k, steps done]
    self.history = []

  def tick(self):
    self.k += 1
    if self.k % 10 == 3:
      self.counter = (self.counter + 1) % 16
    for p in [p for p in self.pending if p[0] <= self.k]:
      self.pending.remove(p)
      self.set = p[1]
    s = self.stream
    if s is not None and self.k - s[2] > 15:
      self.stream = None          # released
    elif s is not None and self.mode == "hold":
      d = -1 if s[1] == "SET_MINUS" else 1
      el = self.k - s[0]
      if el == 55:
        self.set = CM.hold_first_landing(self.set - d, d)
      elif el > 55 and (el - 55) % 60 == 0:
        self.set += 5 * d
    self.cruise.v = self.mph
    self.mph = self.cruise.step(self.set)
    self.history.append((self.k, self.set, self.mph))

  def press(self, button, counter_sent):
    self.ours.append((self.k, button, counter_sent))
    d = -1 if button == "SET_MINUS" else 1
    s = self.stream
    if s is None:
      self.stream = [self.k, button, self.k, 0]
      if self.mode in ("hold", "each", "once"):
        self.pending.append((self.k + 4, self.set + d))
      elif self.mode == "taps_only":
        self.pending.append((self.k + 4, self.set + d))
      return
    s[2] = self.k
    if self.mode == "each":
      self.pending.append((self.k + 4, self.set + d))
    elif self.mode == "taps_only" and self.k - s[0] > 15:
      pass


def drive(curve, car, frames, plan_fn, **over):
  outs = []
  for f in range(frames):
    car.tick()
    kw = dict(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, brake=False, gas=False,
              lead=False, driver_button=False, counter=car.counter, plan=plan_fn(f, car), plan_age=0.1, mono=f * DT)
    for k, v in over.items():
      kw[k] = v(f, car) if callable(v) else v
    b = curve.step(f, C.Inputs(**kw))
    if b is not None:
      assert b in ("SET_MINUS", "SET_PLUS")
      car.press(b, (car.counter + 1) % 16)
    outs.append((f, b, car.set, curve.state))
  return outs


def streams(car):
  out = []
  for k, b, c in car.ours:
    if out and k - out[-1][-1][0] <= 15 and out[-1][-1][1] == b:
      out[-1].append((k, b, c))
    else:
      out.append([(k, b, c)])
  return out


def _caps_hold(car):
  for s in streams(car):
    ks = [k for k, _, _ in s]
    assert len(s) <= TT.HOLD_MAX_FRAMES and (ks[-1] - ks[0]) * DT <= TT.HOLD_MAX_S + 1e-9
    assert all((b - a) * DT >= TT.HOLD_FRAME_MIN_S - 1e-9 for a, b in zip(ks, ks[1:]))
  ss = streams(car)
  for a, b in zip(ss, ss[1:]):
    assert (b[0][0] - a[-1][0]) * DT >= TT.MIN_TAP_GAP_S - 1e-9


def test_hold_off_by_default():
  assert TT.HOLD_ENABLED is False and C.Curve().hold_ok is False
  car = BusCar(45.0, 45, "hold")
  c = C.Curve()
  drive(c, car, 1500, lambda f, car: plan(35))
  assert c.holds == 0 and all(len(s) <= TT.MAX_PRESSED_FRAMES for s in streams(car)) and car.set == 35


def test_hold_reaches_the_target_faster_and_never_past_it():
  res = {}
  for hold in (False, True):
    car = BusCar(45.0, 45, "hold")
    c = C.Curve(hold=hold)
    out = drive(c, car, 1500, lambda f, car: plan(30))
    first = next(f for f, b, _, _ in out if b is not None)
    res[hold] = next(f for f, _, s, _ in out if s == 30) - first
    assert min(s for _, _, s, _ in out) == 30 and c.state != "stopped"
    _caps_hold(car)
  assert res[True] <= 200 and res[False] >= 400, res      # held: 45 -> 44 -> 40 -> 35 -> 30 in about 1.8 s; taps: 15 x 0.3 s


def test_hold_frames_one_per_car_frame_with_its_counter_plus_one():
  car = BusCar(45.0, 45, "hold")
  c = C.Curve(hold=True)
  frames = []
  orig = car.press

  def press(b, cs):
    frames.append((car.k, car.counter, cs))
    orig(b, cs)
  car.press = press
  drive(c, car, 300, lambda f, car: plan(35))
  s = streams(car)[0]
  assert len(s) >= 10
  assert all(cs == (cc + 1) % 16 for _, cc, cs in frames)
  assert len({cc for _, cc, _ in frames[:len(s)]}) == len(s)          # never two held frames on one car frame


@pytest.mark.parametrize("mode", ["each", "once", "taps_only"])
def test_hold_not_read_as_measured_falls_back_to_taps_without_a_stop(mode):
  car = BusCar(45.0, 45, mode)
  c = C.Curve(hold=True)
  out = drive(c, car, 2500, lambda f, car: plan(33))
  assert car.set == 33 and min(s for _, _, s, _ in out) >= 33
  assert not any(e["kind"] == "stop" for e in c.events), [e for e in c.events if e["kind"] == "stop"]
  _caps_hold(car)
  if mode != "each":
    assert c.hold_off or any(e["kind"] == "hold" and e["why"] != "landed" for e in c.events)


def test_driver_press_during_a_hold_wins_at_once():
  car = BusCar(55.0, 55, "hold")
  c = C.Curve(hold=True)
  t_press = {}

  def btn(f, car):
    if c.hold is not None and c.hold.frames and "f" not in t_press and len(c.hold.frames) >= 4:
      t_press["f"] = f
    return "f" in t_press and f < t_press["f"] + 30

  out = drive(c, car, 800, lambda f, car: plan(35), driver_button=btn, driver_set=btn)
  f0 = t_press["f"]
  assert all(b is None for f, b, _, _ in out if f >= f0)              # not one more frame from the press's own frame on
  assert c.state == "paused" and c.hold is None


@pytest.mark.parametrize("field", ["brake", "gas", "cancel_req", "resume_req", "standstill"])
def test_pedals_and_openpilot_requests_end_a_hold(field):
  car = BusCar(55.0, 55, "hold")
  c = C.Curve(hold=True)
  on = {}

  def v(f, car):
    if c.hold is not None and len(c.hold.frames) >= 3 and "f" not in on:
      on["f"] = f
    return "f" in on

  out = drive(c, car, 400, lambda f, car: plan(35), **{field: v})
  assert all(b is None for f, b, _, _ in out if f >= on["f"])


def test_planner_late_mid_hold_releases_and_holds():
  car = BusCar(55.0, 55, "hold")
  c = C.Curve(hold=True)
  on = {}

  def age(f, car):
    if c.hold is not None and len(c.hold.frames) >= 3 and "f" not in on:
      on["f"] = f
    return 2.0 if "f" in on else 0.1

  out = drive(c, car, 600, lambda f, car: plan(35), plan_age=age)
  assert all(b is None for f, b, _, _ in out if f >= on["f"])
  assert c.state == "holding"


def test_never_cancel_or_resume_and_raise_back_uses_the_hold_up():
  car = BusCar(55.0, 55, "hold")
  c = C.Curve(hold=True)
  out = drive(c, car, 2500, lambda f, car: plan(35, corner=45) if f < 900 else plan(45, corner=45))
  assert {b for _, b, _, _ in out if b} <= {"SET_MINUS", "SET_PLUS"}
  assert car.set == 45 and c.state != "stopped"
  ups = [s for s in streams(car) if s[0][1] == "SET_PLUS"]
  assert ups and max(len(s) for s in ups) > TT.MAX_PRESSED_FRAMES          # 35 -> 45 was held (35 -> 36 -> 40 -> 45)


def test_planner_dies_mid_push_with_the_dash_10_below_target_restores(monkeypatch):
  monkeypatch.setattr(CM, "PUSH_BELOW_MAX_DROP", True)
  monkeypatch.setattr(P, "DRIVE_GAP_MPH", 10)
  car = BusCar(55.0, 55, "hold")
  c = C.Curve(hold=True)
  # corner 40 (the floor: 55 - 15), push to 30: 10 below the target, below the floor, above the absolute floor
  out = drive(c, car, 700, lambda f, car: plan(30, corner=40, dist=60.0))
  assert car.set == 30 and min(s for _, _, s, _ in out) == 30 and c.floor == 40 and c.press_floor == 30
  # the planner dies: hold the set speed until past the corner's end, then restore to the driver's 55 (existing rule, T1)
  out = drive_more(c, car, 4000, lambda f, car: None, plan_age=None)
  assert car.set == 55 and not c.stop_latched


def drive_more(curve, car, frames, plan_fn, **over):
  start = car.k
  outs = []
  for f in range(start, start + frames):
    car.tick()
    kw = dict(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, brake=False, gas=False,
              lead=False, driver_button=False, counter=car.counter, plan=plan_fn(f, car), plan_age=0.1, mono=f * DT)
    for k, v in over.items():
      kw[k] = v(f, car) if callable(v) else v
    b = curve.step(f, C.Inputs(**kw))
    if b is not None:
      car.press(b, (car.counter + 1) % 16)
    outs.append((f, b, car.set, curve.state))
  return outs


def test_push_floor_never_below_the_absolute_floor(monkeypatch):
  monkeypatch.setattr(CM, "PUSH_BELOW_MAX_DROP", True)
  monkeypatch.setattr(P, "DRIVE_GAP_MPH", 10)
  car = BusCar(38.0, 38, "hold")
  c = C.Curve(hold=True)
  out = drive(c, car, 1500, lambda f, car: plan(20, corner=28))
  assert min(s for _, _, s, _ in out) == C.ABS_FLOOR_MPH
  # without the switch: the push stops at the floor (max(25, 38 - 15) = 25 here too; a higher start shows it)
  monkeypatch.setattr(CM, "PUSH_BELOW_MAX_DROP", False)
  car = BusCar(55.0, 55, "hold")
  c = C.Curve(hold=True)
  out = drive(c, car, 1500, lambda f, car: plan(30, corner=40))
  assert min(s for _, _, s, _ in out) == 40


# ---- our frames never reach the driver-press classification --------------------------------------------------------------------
IDLE = bytes.fromhex("0001ffe8")          # recorded: the module's unpressed frame
SET_M = bytes.fromhex("2001bfe8")         # recorded: SET- pressed (driver's own frame)
BITS = {"CAN_OFF": (0, 0x01), "RES": (0, 0x04), "SET_P": (0, 0x10), "SET_M": (0, 0x20), "DISTANCE_MORE": (0, 0x40),
        "DISTANCE_LESS": (0, 0x80), "MODE_Y": (1, 0x20), "MODE_X": (1, 0x40)}


def decode(dat):
  return {k: int(bool(dat[i] & m)) for k, (i, m) in BITS.items()}


def bus_frames(hold_frames, driver_at=None):
  """A recorded-pattern interleave over 3 s: (k, src, data). src 0: the module's frames (pressed only from `driver_at`); src 128:
  the transmit echo of ours; src 1: bus 1 sees both (recorded: 394 = 333 + 61)."""
  out = []
  for k in range(300):
    if k % 10 == 3:
      d = SET_M if driver_at is not None and k >= driver_at else IDLE
      out += [(k, 0, d), (k, 1, d)]
    if k % 10 == 4 and k < 10 * hold_frames:
      out += [(k, 128, SET_M), (k, 1, SET_M)]
  return out


@pytest.mark.parametrize("driver_at", [None, 153])
def test_held_frames_never_classify_as_a_driver_press(driver_at):
  # the bus-0 parser keeps the last src-0 frame; a 2.5 s hold of ours (25 frames) never makes it a press; the driver's own pressed
  # frame does, on the first update after it
  last = IDLE
  seen = []
  for k, src, dat in bus_frames(25, driver_at):
    if src == 0:
      last = dat
    seen.append((k, CSm.crz_btns_flags(decode(last))))
  if driver_at is None:
    assert not any(f[0] for _, f in seen)
  else:
    assert not any(f[0] for k, f in seen if k < driver_at) and all(f[0] and f[1] for k, f in seen if k >= driver_at)


def test_no_parser_but_bus_0_reads_the_button_frame(monkeypatch):
  # bus 1 carries copies of OUR frames (recorded): no parser there (or on the camera bus) may read 0x09D
  from openpilot.selfdrive.car.mazda.values import CAR, MazdaFlags
  made = []

  class FakeParser:
    def __init__(self, dbc, messages, bus):
      made.append((bus, [m[0] for m in messages]))
  monkeypatch.setattr(CSm, "CANParser", FakeParser)

  class CP:
    carFingerprint = CAR.MAZDA_CX5
    flags = MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR
  CSm.CarState.get_can_parser(CP, None)
  CSm.CarState.get_cam_can_parser(CP, None)
  CSm.CarState.get_body_can_parser(CP)
  buses = {bus for bus, msgs in made if "CRZ_BTNS" in msgs}
  assert buses == {0}, made


def test_buttonevents_never_carry_set_presses():
  # the only cruise-button consumers downstream of carState (controlsd resume, frogpilot accel / decel) look for accelCruise /
  # decelCruise / resumeCruise events, which the Mazda interface never creates (only gapAdjustCruise and lkas)
  import inspect
  from openpilot.selfdrive.car.mazda import interface
  src = inspect.getsource(interface)
  assert "accelCruise" not in src and "decelCruise" not in src and "resumeCruise" not in src
