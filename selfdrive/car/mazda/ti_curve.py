"""Curve slowing, the car side (ti_curve.py): lowers the stock cruise's SET SPEED before a remembered corner the steering cannot hold at
the current speed, one button tap at a time, and puts it back afterwards. The planner (ti_curve_plan.py, its own 5 Hz process) says
how low the set speed may be (a ceiling); this module only does the taps, under the rules below. It cannot brake: the stock cruise
slows the car at its own rate (about 1-2 mph per second).

Switched on ONLY by its own control on the Tests pages (PARAM_ON, tmpfs, written by the page), for the drive: the car controller clears
it when it starts, so it is OFF after every controls restart. While OFF nothing is sent (the planner still runs and logs what it
would have done: shadow mode).

Every frame (the car controller calls `step`, 100 Hz; the planner's output is read at 10 Hz by the car controller and passed in):
  ready      no intervention. The driver's set speed (`ref`) is adopted whenever it changes (any change while idle is the driver's).
  slowing    the planner's ceiling is below the set speed: SET_MINUS taps toward the target, one at a time, each read back exactly 1 mph
             lower before the next. The target is the ceiling, but never below the FLOOR of this intervention:
               floor = max(ABS_FLOOR_MPH, (the road's known speed limit, else the speed when the intervention started) - MAX_SHED_MPH)
             (owner, 2026-11-23: "never shed more than about 15 miles per hour beneath the known speed limit of a road. If the speed
             limit is unknown, we should just use current speed."). A corner that needs more: the set speed goes to the floor and the
             page says so ("CORNER NEEDS MORE THAN I MAY SLOW: be ready"); logged.
  holding    at the target through the corner.
  restoring  the planner says past the corner AND the driving model's own preview ahead is gentle (model_la below MODEL_LA_RESTORE)
             AND the lead flag is readable AND the speed has settled near the set speed (a lead car does not block it: the stock cruise
             keeps its gap, and staying lowered behind a lead that speeds up only opens a gap for following traffic): SET_PLUS taps, at most one per RESTORE_GAP_S,
             never above `ref` (never on memory alone: a fresh planner output with the model's preview is required).
  paused     the driver pressed a cruise button: the intervention ends at once, nothing is restored, their new set speed becomes `ref`,
             and nothing is done for PAUSE_S.
  stopped    brake; gas held longer than GAS_MAX_S; openpilot disengaged; cruise off, cancelled or unavailable; a set-speed change
             that was not the step asked for (no restore: the page says "set it yourself"); a tap not read back within ACCEPT_WAIT_S;
             the planner stale or dead while the set speed is lowered. Nothing more is sent until the driver presses a cruise button or
             re-engages the cruise (then `ref` is the set speed as it is); only a pedal / engagement stop that lowered nothing clears
             by itself, once the cause has cleared for CLEAR_S.
Low-confidence planner output while lowered: restore gently (same rules as restoring) if the model's preview is gentle, else hold.

Never: a SET_PLUS above `ref`; a SET_MINUS below the floor; taps closer than the reviewed tap machinery allows (ti_cruise_taps.py:
2 pressed frames 50 ms apart, PressGuard caps at 3 frames / 0.15 s, 0.25 s from one tap's last pressed frame to the next); a press on a
frame where openpilot asks for cancel or resume, or while the driver's own cruise button frame shows a press, or while the cruise check
runs; CANCEL or RESUME (this module only ever returns SET_MINUS / SET_PLUS). A lead car: lowering toward a ceiling still happens (the
car does not surge into the corner when the lead leaves).

Pure: no clock, no params; the car controller does the I/O (carcontroller.py `curve_step`). Events go to the `ti_curve` log channel.
"""
from typing import NamedTuple, Optional

from openpilot.selfdrive.car.mazda.ti_cruise_taps import DT, PressGuard, Tap

PARAM_ON = "TiCurveOn"           # tmpfs, the page: "1" on for this drive, "0" / absent off
PARAM_PLAN = "TiCurvePlan"       # tmpfs, the planner (ti_curve_plannerd.py): its output, 5 Hz
PARAM_STATUS = "TiCurveStatus"   # tmpfs, the car controller: what the page shows, on change (at most 10 Hz) and every second
STATUS_EVERY = 100

PRESSED_FRAMES = 2               # a real tap (the cruise check's recipe A, accepted 3 of 3 on the car)
ACCEPT_WAIT_S = 1.0              # first pressed frame to the set speed reading the step (measured 0.08-0.09 s)
ABS_FLOOR_MPH = 25               # never a set speed below this (the car's own floor is 19)
MAX_SHED_MPH = 15                # never more than this below the known speed limit, else below the speed at the start
MIN_MPH = 20.0                   # no taps below this speed
GAS_MAX_S = 1.0                  # gas held longer than this stops the intervention
PAUSE_S = 30.0                   # after the driver's own cruise button
CLEAR_S = 2.0
PLAN_STALE_S = 1.0               # planner output older than this is not used (it publishes at 5 Hz)
PLAN_DEAD_S = 3.0                # ... older than this: the planner is taken as dead
MODEL_LA_RESTORE = 1.0           # m/s^2: the model's largest predicted lateral acceleration ahead must be below this to restore
SETTLED_MPH = 3.0                # restore only with the speed within this of the set speed
RESTORE_GAP_S = 1.0              # at most one SET_PLUS per this (the car speeds up 5 mph in 7-9 s)
TEXT_STALE_S = 3.0               # the page calls a status older than this stale

STATES = ("off", "ready", "slowing", "holding", "restoring", "paused", "stopped", "waiting")


class Inputs(NamedTuple):
  on: bool = False                # the page's switch (PARAM_ON)
  engaged: bool = False
  cruise_on: bool = False
  cruise_available: bool = False
  mph: float = 0.0
  set_mph: Optional[int] = None
  brake: bool = False
  gas: bool = False
  lead: Optional[bool] = None
  driver_button: Optional[bool] = None
  counter: Optional[int] = None
  cancel_req: bool = False
  resume_req: bool = False
  check_active: bool = False      # the cruise set-speed check is running
  plan: Optional[dict] = None     # the planner's last output (ti_curve_plan.plan_json)
  plan_age: Optional[float] = None   # seconds since the planner wrote it (None: never)
  mono: Optional[float] = None


class Curve:
  def __init__(self):
    self.state = "off"
    self.why = ""
    self.guard = PressGuard()
    self.events = []
    self.ref = None               # the driver's set speed
    self.floor = None             # this intervention's floor
    self.target = None
    self.needs_more = False
    self.tap = None
    self.tap_from = None
    self.tap_to = None
    self.tap_first = None
    self.last_plus = -10 ** 9
    self.taps_down = 0
    self.taps_up = 0
    self.gas_since = None
    self.pause_until = None
    self.stop_lowered = False
    self.stop_latched = False
    self.clear_since = None
    self.cruise_was_on = False
    self.btn_was = False
    self.set_last = None
    self.set_stable_since = None
    self.lowered_from = None      # ref at the start of the intervention, for the page
    self.line1, self.line2 = "OFF", ""

  # ---- helpers ----------------------------------------------------------------------------------------------------------------------
  def _ev(self, kind, frame, **kw):
    self.events.append({"kind": kind, "frame": int(frame), **kw})

  def _to(self, frame, state, why=""):
    if state != self.state or why != self.why:
      self._ev("state", frame, state=state, why=why, set_mph=self.set_last, ref=self.ref, target=self.target, floor=self.floor)
    self.state, self.why = state, why

  def intervening(self):
    return self.state in ("slowing", "holding", "restoring")

  def _plan_ok(self, inp):
    return inp.plan is not None and inp.plan_age is not None and inp.plan_age <= PLAN_STALE_S

  def _cut_tap(self):
    if self.tap is not None:
      self.guard.end()
    self.tap = None

  def _stop(self, frame, inp, why):
    self._cut_tap()
    lowered = self.ref is not None and inp.set_mph is not None and inp.set_mph < self.ref and self.intervening()
    self.stop_lowered = lowered
    # a fault of the button path or the planner latches until the driver acts (a cruise button or re-engaging the cruise)
    self.stop_latched = lowered or why.startswith(("set speed", "planner", "cruise check"))
    self._ev("stop", frame, why=why, set_mph=inp.set_mph, ref=self.ref, taps_down=self.taps_down, taps_up=self.taps_up)
    self.clear_since = None
    self._to(frame, "stopped", why)
    self.line1 = f"STOPPED: {why}"
    self.line2 = (f"Set speed {inp.set_mph} mph (was {self.ref}): set it yourself" if lowered else "Nothing was changed")
    self.target = None

  def _end_intervention(self, frame):
    self.target, self.floor, self.needs_more, self.lowered_from = None, None, False, None
    self.taps_down = self.taps_up = 0

  # ---- one frame ----------------------------------------------------------------------------------------------------------------
  def step(self, frame, inp):
    """The button to send this frame: None, "SET_MINUS" or "SET_PLUS"."""
    t = frame * DT
    btn = bool(inp.driver_button)
    set_changed = inp.set_mph != self.set_last
    if set_changed:
      self.set_last, self.set_stable_since = inp.set_mph, frame
    cruise_rose = inp.cruise_on and not self.cruise_was_on
    self.cruise_was_on = inp.cruise_on
    btn_rose = btn and not self.btn_was
    self.btn_was = btn

    if not inp.on:
      if self.state != "off":
        self._cut_tap()
        self._ev("switched_off", frame, set_mph=inp.set_mph, ref=self.ref)
        self._end_intervention(frame)
        self._to(frame, "off")
      self.line1, self.line2 = "CURVE SLOWING OFF", "shadow: the planner only logs"
      return None
    if self.state == "off":
      self.ref = inp.set_mph if inp.cruise_on else None
      self._to(frame, "ready")

    # the driver's own cruise button: ends any intervention, adopts their set speed, pauses
    if btn:
      if btn_rose:
        self._cut_tap()
        if self.intervening() or self.state in ("ready", "stopped", "waiting"):
          self._ev("driver_button", frame, set_mph=inp.set_mph, ref=self.ref, during=self.state)
        self._end_intervention(frame)
        self.pause_until = t + PAUSE_S
        self._to(frame, "paused", "you changed the set speed")
      self.line1, self.line2 = "PAUSED: you changed the set speed", f"back in {PAUSE_S:.0f} s"
      return None
    if self.state == "paused":
      if inp.set_mph is not None and inp.cruise_on and (frame - (self.set_stable_since or frame)) * DT >= 1.0:
        self.ref = inp.set_mph                  # their new set speed, once it has settled
      if t < (self.pause_until or 0):
        self.line1 = "PAUSED: you changed the set speed"
        self.line2 = f"back in {max(0.0, self.pause_until - t):.0f} s"
        return None
      self.ref = inp.set_mph if inp.cruise_on else None
      self._to(frame, "ready")

    # the stop conditions
    if inp.gas:
      self.gas_since = frame if self.gas_since is None else self.gas_since
    else:
      self.gas_since = None
    gas_long = self.gas_since is not None and (frame - self.gas_since) * DT > GAS_MAX_S
    why = ""
    if inp.brake:
      why = "brake pressed"
    elif gas_long:
      why = "gas held"
    elif not inp.engaged:
      why = "openpilot disengaged"
    elif not inp.cruise_available:
      why = "cruise unavailable"
    elif not inp.cruise_on:
      why = "cruise off"
    elif inp.set_mph is None:
      why = "set speed unreadable"

    if self.state == "stopped":
      if btn_rose or (cruise_rose and inp.set_mph is not None):
        self.ref = inp.set_mph
        self._to(frame, "ready")
      elif not why and not self.stop_latched:
        self.clear_since = frame if self.clear_since is None else self.clear_since
        if (frame - self.clear_since) * DT >= CLEAR_S:
          self.ref = inp.set_mph
          self._to(frame, "ready")
          self.clear_since = None
      else:
        self.clear_since = None
      if self.state == "stopped":
        return None

    if why:
      if self.intervening():
        self._stop(frame, inp, why)
        return None
      self._cut_tap()
      self.ref = inp.set_mph if inp.cruise_on else self.ref
      self._to(frame, "waiting", why)
      self.line1, self.line2 = "ON: waiting", why
      return None
    if self.state == "waiting":
      self.ref = inp.set_mph
      self._to(frame, "ready")

    # a tap in flight: the set speed must be the value it started from or the step it asks for; anything else is unexpected
    if self.tap is not None or self.tap_to is not None:
      if inp.set_mph not in (self.tap_from, self.tap_to):
        self._stop(frame, inp, f"set speed changed to {inp.set_mph} mph unexpectedly")
        self.tap_from = self.tap_to = None
        return None
    elif set_changed and self.set_last is not None and inp.set_mph is not None:
      if self.intervening():
        self._stop(frame, inp, f"set speed changed to {inp.set_mph} mph unexpectedly")
        return None
      self.ref = inp.set_mph                      # idle: any change is the driver's

    if self.ref is None:
      self.ref = inp.set_mph

    # the planner
    plan_ok = self._plan_ok(inp)
    dead = inp.plan_age is None or inp.plan_age > PLAN_DEAD_S
    if inp.check_active:
      self._cut_tap()
      if self.intervening():
        self._stop(frame, inp, "cruise check started")
        return None
      self.line1, self.line2 = "ON: waiting", "the cruise check is running"
      return None
    if not plan_ok and self.intervening() and inp.set_mph < self.ref:
      self._stop(frame, inp, "planner stopped" if dead else "planner output late")
      return None

    out = self._send(frame, inp)
    if out is not None or self.tap is not None:
      return out
    if self.tap_to is not None:
      # waiting for the read back of the last tap
      if inp.set_mph == self.tap_to:
        self._ev("step", frame, set_mph=inp.set_mph, ms=int(round((frame - self.tap_first) * DT * 1000)))
        self.tap_from = self.tap_to = None
      elif (frame - self.tap_first) * DT >= ACCEPT_WAIT_S:
        self._stop(frame, inp, "set speed did not respond")
        self.tap_from = self.tap_to = None
        return None
      else:
        return None

    plan = inp.plan if plan_ok else None
    ceil = plan.get("ceil") if plan else None
    trusted = bool(plan.get("trusted")) if plan else False
    if ceil is not None and trusted and inp.mph >= MIN_MPH:
      if not self.intervening():
        if ceil >= inp.set_mph:
          return self._idle_text(plan)
        # a new intervention: its floor from the known limit, else the speed now
        lim = plan.get("limit")
        base = int(lim) if lim else int(round(inp.mph))
        self.floor = max(ABS_FLOOR_MPH, base - MAX_SHED_MPH)
        self.lowered_from = self.ref
        self._ev("start", frame, ceil=ceil, floor=self.floor, limit=lim, mph=round(inp.mph, 1), set_mph=inp.set_mph, ref=self.ref,
                 dist=plan.get("dist"), lead=inp.lead)
      self.target = max(int(ceil), self.floor)
      needs_more = int(ceil) < self.floor
      if needs_more and not self.needs_more:
        self._ev("needs_more", frame, ceil=ceil, floor=self.floor)
      self.needs_more = needs_more
      if inp.set_mph > self.target:
        self._to(frame, "slowing")
        self._lines(inp)
        return self._start(frame, inp, "SET_MINUS")
      self._to(frame, "holding")
      self._lines(inp)
      return None

    # no ceiling (past the corner, or planner not trusted): restore if lowered, else idle
    if self.intervening() or (self.ref is not None and inp.set_mph is not None and inp.set_mph < self.ref and self.lowered_from is not None):
      if inp.set_mph >= self.ref:
        self._ev("restored", frame, set_mph=inp.set_mph, taps_down=self.taps_down, taps_up=self.taps_up)
        self._end_intervention(frame)
        self._to(frame, "ready")
        return self._idle_text(plan)
      self.target = self.ref
      ok, wait = self._restore_ok(inp, plan)
      self._to(frame, "restoring", wait)
      self._lines(inp, wait)
      if ok and (frame - self.last_plus) * DT >= RESTORE_GAP_S:
        out = self._start(frame, inp, "SET_PLUS")
        if out is not None:
          self.last_plus = frame
        return out
      return None
    self._to(frame, "ready")
    return self._idle_text(plan, dead=dead)

  def _restore_ok(self, inp, plan):
    if plan is None:
      return False, "planner output late"
    if plan.get("model_la") is None:
      return False, "no model preview"
    if plan["model_la"] >= MODEL_LA_RESTORE:
      return False, "the model sees a bend ahead"
    if inp.lead is None:
      return False, "lead flag unreadable"
    if abs(inp.mph - inp.set_mph) > SETTLED_MPH:
      return False, "speed settling"
    return True, ""

  def _idle_text(self, plan, dead=False):
    self.line1 = "ON: " + ("planner not running" if dead else "road not known here" if not plan or not plan.get("known")
                           else "matching the road" if not plan.get("trusted") else "no known corner ahead")
    self.line2 = (plan.get("why") or "") if plan else ""
    return None

  def _lines(self, inp, wait=""):
    if self.state == "slowing" or self.state == "holding":
      word = "SLOWING FOR A CORNER" if self.state == "slowing" else "HOLDING FOR A CORNER"
      self.line1 = f"{word}: {self.lowered_from} -> {self.target} mph"
      self.line2 = "CORNER NEEDS MORE THAN I MAY SLOW: be ready" if self.needs_more else f"set speed now {inp.set_mph} mph"
    else:
      self.line1 = f"RESTORING: {inp.set_mph} -> {self.ref} mph"
      self.line2 = wait or "speeding back up"

  # ---- taps ---------------------------------------------------------------------------------------------------------------------
  def _direction_ok(self, button, set_mph):
    if set_mph is None or self.ref is None:
      return False
    if button == "SET_PLUS":
      return set_mph + 1 <= self.ref
    return self.floor is not None and set_mph - 1 >= self.floor and set_mph - 1 >= ABS_FLOOR_MPH and \
      self.target is not None and set_mph > self.target

  def _can_press(self, inp):
    return not (inp.cancel_req or inp.resume_req or inp.driver_button or inp.driver_button is None or inp.counter is None
                or inp.check_active)

  def _start(self, frame, inp, button):
    if self.tap is not None or self.tap_to is not None or not self._can_press(inp) or not self._direction_ok(button, inp.set_mph):
      return None
    if not self.guard.begin(frame, inp.mono):
      return None                     # too soon after the previous tap
    self.tap = Tap(button, PRESSED_FRAMES, frame, inp.counter, inp.set_mph + (1 if button == "SET_PLUS" else -1))
    self.tap.first = frame            # this frame is the tap's first pressed frame (begun above)
    self.tap_from, self.tap_to, self.tap_first = inp.set_mph, self.tap.target, frame
    if button == "SET_MINUS":
      self.taps_down += 1
    else:
      self.taps_up += 1
    return self._press(frame, inp)

  def _press(self, frame, inp):
    tap = self.tap
    if not self._can_press(inp) or inp.set_mph == tap.target or not self._direction_ok(tap.button, self.tap_from):
      tap.n = len(tap.frames)         # the step has landed, or pressing is not allowed now: no further frame of this tap
    elif self.guard.allow(frame, inp.mono):
      tap.frames.append((int(frame), inp.counter, (inp.counter + 1) % 16))
      if tap.done:
        self._tap_done(frame, inp)
      return tap.button
    else:
      tap.n = len(tap.frames)
    if tap.done:
      self._tap_done(frame, inp)
    return None

  def _send(self, frame, inp):
    """The next pressed frame of the tap in flight, if one is due."""
    tap = self.tap
    if tap is None:
      return None
    if (frame - tap.first) % 5 != 0:
      return None
    return self._press(frame, inp)

  def _tap_done(self, frame, inp):
    self._ev("tap", frame, button=self.tap.button, frames=[list(f) for f in self.tap.frames], set_before=self.tap_from,
             set_now=inp.set_mph)
    self.guard.end()
    self.tap = None
    if inp.set_mph == self.tap_to:
      self._ev("step", frame, set_mph=inp.set_mph, ms=int(round((frame - self.tap_first) * DT * 1000)))
      self.tap_from = self.tap_to = None

  # ---- the page -------------------------------------------------------------------------------------------------------------------
  def status(self, wall):
    return {"t": round(wall, 1), "v": 1, "state": self.state, "why": self.why, "line1": self.line1, "line2": self.line2,
            "ref": self.ref, "target": self.target, "floor": self.floor, "set": self.set_last, "needs_more": self.needs_more,
            "down": self.taps_down, "up": self.taps_up}

  @property
  def active(self):
    return self.intervening()
