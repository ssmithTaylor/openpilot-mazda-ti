"""Curve slowing, the car side (ti_curve.py): lowers the stock cruise's SET SPEED before a remembered corner the steering cannot hold at
the current speed, one button tap at a time, and puts it back afterwards. The planner (ti_curve_plan.py, its own 5 Hz process) says
how low the set speed may be (a ceiling); this module only does the taps, under the rules below. It cannot brake: the stock cruise
slows the car at its own rate (about 1-2 mph per second).

Switched on ONLY by its own control on the Tests pages (PARAM_ON, tmpfs, written by the page), for the drive: the car controller clears
it when it starts, so it is OFF after every controls restart. While OFF nothing is sent (the planner still runs and logs what it
would have done: shadow mode).

Every frame (the car controller calls `step`, 100 Hz; the planner's output is read at 10 Hz by the car controller and passed in):
  ready      no intervention. The driver's set speed (`ref`) is adopted whenever it changes (any change while idle is the driver's).
  slowing    the planner's corner ceiling is below the set speed: SET_MINUS taps toward it, one at a time, each read back exactly 1 mph
             lower before the next; the planner first asks for a set speed up to 6 mph below the corner speed (the gap at which the
             cruise slows fastest), then for the corner speed itself as the car reaches it (SET_PLUS taps, never above `ref`). The
             target is never below the FLOOR of this intervention:
               floor = max(ABS_FLOOR_MPH, the speed when it started - MAX_SHED_MPH, the known speed limit - MAX_SHED_MPH)
             (a known limit can only make it slow LESS; the planner passes a limit only when it is fresh and plausible)
             (owner, 2026-11-23: "never shed more than about 15 miles per hour beneath the known speed limit of a road. If the speed
             limit is unknown, we should just use current speed."). A corner that needs more: the set speed goes to the floor and the
             page says so ("CORNER NEEDS MORE THAN I MAY SLOW: be ready"); logged.
  holding    at the target through the corner.
  restoring  the planner says past the corner (or gives a "hold" ceiling: back up only to what the NEXT corner allows, in steps of at
             least RESTORE_MIN_MPH) AND the driving model's sharpest predicted bend ahead fits at the restored speed (ti_curve_plan.fits)
             AND the lead flag is readable AND the speed has settled near the set speed (a lead car does not block it: the stock cruise
             keeps its gap, and staying lowered behind a lead that speeds up only opens a gap for following traffic): SET_PLUS taps at the tap machinery's pace,
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

from openpilot.selfdrive.car.mazda import ti_curve_plan as P
from openpilot.selfdrive.car.mazda.ti_cruise_taps import DT, FRAME_SPACING, PressGuard, Tap

PARAM_ON = "TiCurveOn"           # tmpfs, the page: "1" on for this drive, "0" / absent off
PARAM_PLAN = "TiCurvePlan"       # tmpfs, the planner (ti_curve_plannerd.py): its output, 5 Hz
PARAM_STATUS = "TiCurveStatus"   # tmpfs, the car controller: what the page shows, on change (at most 10 Hz) and every second
STATUS_EVERY = 100

PRESSED_FRAMES = 2               # a real tap (the cruise check's recipe A, accepted 3 of 3 on the car)
ACCEPT_WAIT_S = 1.0              # first pressed frame to the set speed reading the step (measured 0.08-0.09 s)
ABS_FLOOR_MPH = 25               # never a set speed below this (the car's own floor is 19)
MAX_SHED_MPH = 15                # never more than this below the known speed limit, else below the speed at the start
MIN_MPH = 20.0                   # no press of either kind below this speed, at standstill or with the cruise in its standstill hold
MPH = 0.44704
GAS_MAX_S = 1.0                  # gas held longer than this stops the intervention
PAUSE_S = 30.0                   # after the driver's own cruise button
CLEAR_S = 2.0
PLAN_STALE_S = 1.0               # planner output older than this is not used (it publishes at 5 Hz)
PLAN_DEAD_S = 3.0                # ... older than this: the planner is taken as dead
SETTLED_MPH = 3.0                # restore only with the speed within this of the set speed
RESTORE_MIN_MPH = 2              # a restore toward a "hold" ceiling only for 2 mph or more (no dithering)
RESTORE_LEAD_MPH = 3             # while restoring, the set speed is never more than this above the car's speed (a gentle catch-up)
SETTLED_RESUME_MPH = 2.0         # "speed settling" ends below this gap (it starts above SETTLED_MPH): hysteresis
HOLDING_AT_MPH = 1.0             # slowing -> holding within this of the corner speed; back to slowing above HOLDING_AT_MPH + 1
TRUST_HOLD_M = 30.0              # trust lost during an approach: hold the set speed until this far past where the corner ended
TRUST_HOLD_MAX_S = 30.0          # ... but never longer than this: then stop, leaving the set speed to the driver with a plain message
RESTORE_WAIT_S = 10.0            # a restore that cannot proceed for this long says WAITING TO RESTORE and why
LIMIT_PLAUSIBLE_MPH = 15.0       # a speed limit further than this from the car's speed is not used (stale or wrong, high or low)
STATE_LOG_MIN_S = 1.0            # the same state with a different reason is logged at most this often
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
  standstill: bool = False        # the car at standstill or the stock cruise in its standstill hold
  driver_res: bool = False        # the driver's pressed button is RES only (a RES returns to the stored set speed: not a new choice)


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
    self.shown = None             # the corner speed shown on the page (the target may be the push below it)
    self.line1, self.line2 = "OFF", ""
    self.odo = 0.0                # metres driven (from the speed), for holding after a loss of trust
    self.corner_end_odo = None    # where the corner being slowed for ends (from the planner's distance)
    self.correct_down = False     # the set speed ended ABOVE the reference: single aligned SET_MINUS taps back to it
    self.restart_note = ""        # set by the car controller when the previous run may have left the set speed lowered
    self.off_note = ""            # switched off mid-intervention: what is left for the driver
    self.settling = False
    self.tap_correcting = False
    self.log_state = (None, None, -10 ** 9)
    # the set speed this feature left lowered (stop, switch off) while the driver has not yet chosen a new one by their own SET+ / SET-
    # press: until they do, `ref` stays their ORIGINAL choice, so stops and resumes can never walk the set speed further down
    self.left_at = None
    self.own_press = False        # a driver press other than RES since the set speed was left lowered
    self.left_doubt = False       # the set speed moved in a way that is neither ours nor RES: leave it alone
    self.trust_hold_since = None
    self.restore_wait_since = None
    self.off_seen = False

  # ---- helpers ----------------------------------------------------------------------------------------------------------------------
  def _ev(self, kind, frame, **kw):
    self.events.append({"kind": kind, "frame": int(frame), **kw})

  def _to(self, frame, state, why=""):
    ls, lw, lf = self.log_state
    if state != ls or (why != lw and (frame - lf) * DT >= STATE_LOG_MIN_S):
      self._ev("state", frame, state=state, why=why, set_mph=self.set_last, ref=self.ref, target=self.target, floor=self.floor)
      self.log_state = (state, why, frame)
    self.state, self.why = state, why

  def intervening(self):
    return self.state in ("slowing", "holding", "restoring")

  def _plan_ok(self, inp):
    return inp.plan is not None and inp.plan_age is not None and inp.plan_age <= PLAN_STALE_S

  def _cut_tap(self):
    if self.tap is not None:
      self.guard.end()
      if not self.tap.frames:
        self.tap_from = self.tap_to = None   # nothing was sent: nothing to read back
    self.tap = None

  def _stop(self, frame, inp, why):
    self._cut_tap()
    lowered = self.ref is not None and inp.set_mph is not None and inp.set_mph < self.ref and self.intervening()
    self.stop_lowered = lowered
    if lowered:
      self.left_at, self.own_press, self.left_doubt = inp.set_mph, False, False
    if why.startswith("set speed changed") and inp.set_mph is not None and self.ref is not None and inp.set_mph < self.ref:
      # a change nobody asked for (possibly the driver's press the button flag missed): never restored, nothing more pressed until
      # the driver acts;  is kept only as the limit for how far a later intervention may go
      self.left_at = inp.set_mph if self.left_at is None else self.left_at
      self.left_doubt = True
    # a fault of the button path or the planner latches until the driver acts (a cruise button or re-engaging the cruise)
    self.stop_latched = lowered or why.startswith(("set speed", "planner", "cruise check"))
    self._ev("stop", frame, why=why, set_mph=inp.set_mph, ref=self.ref, taps_down=self.taps_down, taps_up=self.taps_up)
    self.clear_since = None
    self._to(frame, "stopped", why)
    self.line1 = f"STOPPED: {why}"
    self.line2 = self._left_text(inp.set_mph, why)
    self.target = None
    if inp.set_mph is not None and self.ref is not None and inp.set_mph > self.ref:
      self.correct_down = True      # above the driver's own set speed: put back down by single aligned taps (lowering is allowed)
      self.stop_latched = True

  def _left_text(self, set_mph, why=""):
    """What is left for the driver: never "nothing changed" when the set speed differs from theirs."""
    if set_mph is None or self.ref is None:
      return "Set speed unknown: check it"
    if set_mph < self.ref:
      word = "left at" if why == "gas held" else ""
      return f"Set speed {word + ' ' if word else ''}{set_mph} mph (was {self.ref}): set it yourself"
    if set_mph > self.ref:
      return f"Set speed {set_mph} mph is above yours ({self.ref}): lowering it"
    return f"Set speed {set_mph} mph, as you set it"

  def _end_intervention(self, frame):
    self.target, self.floor, self.needs_more = None, None, False
    if self.left_at is None:
      self.lowered_from = None
    self.taps_down = self.taps_up = 0
    self.trust_hold_since = self.restore_wait_since = None

  def _adopt(self, set_mph):
    """A set speed the driver may have chosen. While a set speed this feature left lowered is outstanding, only a value reached after
    the driver's own press other than RES is their new choice; a RES (back to the stored, lowered value) or anything unexplained is
    not, and `ref` stays their original choice."""
    if set_mph is None:
      return
    if self.left_at is None or self.ref is None:
      self.ref = set_mph
    elif self.own_press:
      self._ev("driver_chose", 0, set_mph=set_mph, was=self.ref)
      self.ref = set_mph
      self.left_at, self.own_press, self.left_doubt, self.lowered_from = None, False, False, None
    elif set_mph != self.left_at:
      self.left_doubt = True              # neither ours nor a RES to the stored value: leave it alone, keep the message

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
    self.odo += max(0.0, inp.mph) * MPH * DT

    if not inp.on:
      if self.state != "off":
        self._cut_tap()
        lowered = self.ref is not None and inp.set_mph is not None and inp.set_mph < self.ref and \
          (self.intervening() or self.stop_lowered)
        self.off_note = f"Set speed {inp.set_mph} mph (was {self.ref}): set it yourself" if lowered else ""
        if lowered:
          self.lowered_from = self.ref
          if self.left_at is None:
            self.left_at, self.own_press, self.left_doubt = inp.set_mph, False, False
        self._ev("switched_off", frame, set_mph=inp.set_mph, ref=self.ref, lowered=lowered)
        self._end_intervention(frame)
        self.tap_from = self.tap_to = None
        self.correct_down = False
        self._to(frame, "off")
      if btn_rose and not inp.driver_res:
        self.off_note = self.restart_note = ""
        self.own_press = True
      self.off_view(inp.plan, inp.plan_age, inp.set_mph, inp.mph)
      return None
    if self.state == "off":
      if inp.cruise_on:
        self._adopt(inp.set_mph)
      self.off_note = self.restart_note = ""
      self._to(frame, "ready")

    # the driver's own cruise button: ends any intervention, adopts their set speed, pauses
    if btn:
      if btn_rose:
        self._cut_tap()
        if not inp.driver_res:
          self.own_press = True
        if self.intervening() or self.state in ("ready", "stopped", "waiting"):
          self._ev("driver_button", frame, set_mph=inp.set_mph, ref=self.ref, during=self.state)
        self._end_intervention(frame)
        self.pause_until = t + PAUSE_S
        self._to(frame, "paused", "you changed the set speed")
      self.line1, self.line2 = "PAUSED: you changed the set speed", f"back in {PAUSE_S:.0f} s"
      return None
    if self.state == "paused":
      if inp.set_mph is not None and inp.cruise_on and (frame - (self.set_stable_since or frame)) * DT >= 1.0:
        self._adopt(inp.set_mph)                # their new set speed, once it has settled (a RES is not a new choice)
      if t < (self.pause_until or 0):
        self.line1 = "PAUSED: you changed the set speed"
        self.line2 = f"back in {max(0.0, self.pause_until - t):.0f} s"
        return None
      if inp.cruise_on:
        self._adopt(inp.set_mph)
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

    if self.state == "stopped" and self.correct_down and not btn and not why:
      return self._correct(frame, inp)
    if self.state == "stopped":
      if btn_rose or (cruise_rose and inp.set_mph is not None):
        self.correct_down = False
        self._adopt(inp.set_mph)
        self._to(frame, "ready")
      elif not why and not self.stop_latched and not self.correct_down:
        self.clear_since = frame if self.clear_since is None else self.clear_since
        if (frame - self.clear_since) * DT >= CLEAR_S:
          self._adopt(inp.set_mph)
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
      if inp.cruise_on:
        self._adopt(inp.set_mph)
      self._to(frame, "waiting", why)
      if self.left_at is not None and inp.set_mph is not None and self.ref is not None and inp.set_mph < self.ref:
        self.line1, self.line2 = "ON: waiting", self._left_text(inp.set_mph)
        return None
      self.line1, self.line2 = "ON: waiting", why
      return None
    if self.state == "waiting":
      self._adopt(inp.set_mph)
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
      self._adopt(inp.set_mph)                    # idle: the driver's (a RES to a set speed left lowered is not a new choice)

    if self.ref is None:
      self.ref = inp.set_mph
    if self.left_at is not None and inp.set_mph is not None and self.ref is not None and inp.set_mph >= self.ref:
      self.left_at, self.own_press, self.left_doubt = None, False, False     # back at the driver's own set speed

    if self.left_doubt:
      self._cut_tap()
      self._to(frame, "holding", "set speed changed: left to you")
      self.line1 = f"ON: set speed {inp.set_mph} mph"
      self.line2 = self._left_text(inp.set_mph)
      return None

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

    if self.tap is not None and self.tap.first is None and self.tap.button == "SET_MINUS" and not self.tap_correcting:
      p_ = inp.plan if plan_ok else None
      if not (p_ and p_.get("trusted") and p_.get("ceil") is not None and p_.get("kind", "corner") != "hold"):
        self._cut_tap()                     # a SET_MINUS not yet sent whose corner ceiling has gone: not sent
    out = self._send(frame, inp)
    if out is not None or self.tap is not None:
      return out
    if self.tap_to is not None and self.tap_first is None:
      self.tap_from = self.tap_to = None
    if self.tap_to is not None:
      # waiting for the read back of the last tap
      if inp.set_mph == self.tap_to:
        self._ev("step", frame, set_mph=inp.set_mph, ms=int(round((frame - self.tap_first) * DT * 1000)))
        self.tap_from = self.tap_to = None
        if self.left_at is not None:
          self.left_at = inp.set_mph
      elif (frame - self.tap_first) * DT >= ACCEPT_WAIT_S:
        self._stop(frame, inp, "set speed did not respond")
        self.tap_from = self.tap_to = None
        return None
      else:
        return None

    plan = inp.plan if plan_ok else None
    ceil = plan.get("ceil") if plan else None
    trusted = bool(plan.get("trusted")) if plan else False
    kind = plan.get("kind", "corner") if plan else ""
    if ceil is not None and trusted and kind != "hold" and (plan.get("end") is not None or plan.get("dist") is not None):
      # where the remembered tight stretch ENDS (the planner's "end"; "dist" is to its start)
      self.corner_end_odo = self.odo + max(0.0, float(plan["end"] if plan.get("end") is not None else plan["dist"]))
    if self.intervening() and not trusted and self.corner_end_odo is not None and self.odo < self.corner_end_odo + TRUST_HOLD_M:
      # trust lost during an approach (position, road left, road changed): hold what has been set until past where the corner ended
      self.trust_hold_since = frame if self.trust_hold_since is None else self.trust_hold_since
      if (frame - self.trust_hold_since) * DT > TRUST_HOLD_MAX_S:
        self._stop(frame, inp, "road not matched")
        return None
      self.target = self.shown = inp.set_mph
      self._to(frame, "holding", "road not matched: holding past the corner")
      self.line1 = f"HOLDING: {inp.set_mph} mph"
      self.line2 = "road not matched: holding past the corner"
      return None
    if trusted:
      self.trust_hold_since = None
    if ceil is not None and trusted and kind != "hold" and inp.mph >= MIN_MPH:
      if not self.intervening() and ceil >= inp.set_mph:
        return self._idle_text(plan)
      if not self.intervening() or self.floor is None:
        # a new intervention: never more than MAX_SHED_MPH below the DRIVER'S OWN set speed  (kept across stops and resumes, so
        # neither chained corners nor repeated stops can go further); below the car's speed instead only when a lead holds the car
        # under its set speed; a known limit within LIMIT_PLAUSIBLE_MPH of the speed only raises it:
        #   floor = max(ABS_FLOOR_MPH, (ref, or min(ref, speed) when a lead holds the car) - 15, limit - 15)
        lim = plan.get("limit")
        if lim and abs(float(lim) - inp.mph) > LIMIT_PLAUSIBLE_MPH:
          lim = None                          # stale or wrong, high or low: not used
        lead_held = bool(inp.lead) and inp.mph < inp.set_mph - 2.0
        base = min(self.ref, int(round(inp.mph))) if lead_held else self.ref
        self.floor = max(ABS_FLOOR_MPH, base - MAX_SHED_MPH, (int(lim) - MAX_SHED_MPH) if lim else 0)
        self.lowered_from = self.ref
        self.restore_wait_since = None
        self._ev("start", frame, ceil=ceil, corner=plan.get("corner"), floor=self.floor, limit=lim, mph=round(inp.mph, 1),
                 set_mph=inp.set_mph, ref=self.ref, dist=plan.get("dist"), lead=inp.lead)
      # the set speed the planner asks for now (the push below the corner speed, then the corner speed), within the floor and
      # never above the driver's own set speed
      self.target = min(max(int(ceil), self.floor), self.ref)
      corner = plan.get("corner") if plan.get("corner") is not None else ceil
      self.shown = min(max(int(corner), self.floor), self.ref)
      needs_more = bool(plan.get("capped")) or int(corner) < self.floor
      if needs_more and not self.needs_more:
        self._ev("needs_more", frame, ceil=ceil, corner=corner, floor=self.floor, mph=round(inp.mph, 1))
      self.needs_more = needs_more
      if inp.set_mph > self.target:
        self._to(frame, "slowing")
        self._lines(inp)
        return self._start(frame, inp, "SET_MINUS")
      near = inp.mph <= self.shown + HOLDING_AT_MPH + (1.0 if self.state == "holding" else 0.0)   # hysteresis
      self._to(frame, "holding" if near else "slowing")
      self._lines(inp)
      if inp.set_mph < self.target:
        # the planned raise from the push back to the corner speed as the car reaches it (never above `ref`)
        return self._start(frame, inp, "SET_PLUS")
      return None

    # no corner ceiling (past the corner, a "hold" for the next one, or the planner not trusted): restore if lowered, else idle
    if self.intervening() or (self.ref is not None and inp.set_mph is not None and inp.set_mph < self.ref and self.lowered_from is not None):
      if inp.set_mph >= self.ref:
        self._ev("restored", frame, set_mph=inp.set_mph, taps_down=self.taps_down, taps_up=self.taps_up)
        self._end_intervention(frame)
        self._to(frame, "ready")
        return self._idle_text(plan)
      cap = self.ref
      if plan is not None and trusted and ceil is not None and kind == "hold":
        cap = min(self.ref, int(ceil))           # the next corner is close: come back up only to what it allows
      if cap <= inp.set_mph or (cap - inp.set_mph < RESTORE_MIN_MPH and cap != self.ref and self.state != "restoring"):
        self.target = self.shown = inp.set_mph
        self._to(frame, "holding", "next corner close")
        self.line1 = f"HOLDING FOR THE NEXT CORNER: {inp.set_mph} mph"
        self.line2 = f"back to {self.ref} mph after it"
        return None
      self.target = self.shown = cap
      self.corner_end_odo = None
      if self.left_doubt:
        self._to(frame, "holding", "set speed changed: left to you")
        self.line1 = f"ON: set speed {inp.set_mph} mph"
        self.line2 = self._left_text(inp.set_mph)
        return None
      ok, wait = self._restore_ok(inp, plan)
      if ok:
        self.restore_wait_since = None
      elif self.restore_wait_since is None:
        self.restore_wait_since = frame
      self._to(frame, "restoring", wait)
      self._lines(inp, wait)
      if not ok and (frame - self.restore_wait_since) * DT >= RESTORE_WAIT_S:
        self.line1 = f"WAITING TO RESTORE: {inp.set_mph} -> {self.ref} mph"
      if ok:
        return self._start(frame, inp, "SET_PLUS")
      return None
    self._to(frame, "ready")
    return self._idle_text(plan, dead=dead)

  def _restore_ok(self, inp, plan):
    if inp.mph < MIN_MPH or inp.standstill:
      return False, f"below {MIN_MPH:.0f} mph: no presses"
    if plan is None:
      return False, "planner output late"
    if plan.get("model_k") is None:
      return False, "no model preview"
    if not P.fits(float(plan["model_k"]), inp.set_mph + 1):
      return False, "the model sees a bend ahead"
    if inp.lead is None:
      return False, "lead flag unreadable"
    gap = inp.mph - inp.set_mph
    self.settling = gap > (SETTLED_RESUME_MPH if self.settling else SETTLED_MPH)      # hysteresis
    if self.settling:
      return False, "speed settling"
    if inp.set_mph + 1 > inp.mph + RESTORE_LEAD_MPH:
      return False, "car is below the set speed"                                      # paced to the car: a gentle catch-up
    return True, ""

  def _idle_text(self, plan, dead=False):
    self.line1 = "ON: " + ("planner not running" if dead else "road not known here" if not plan or not plan.get("known")
                           else "matching the road" if not plan.get("trusted") else "no known corner ahead")
    self.line2 = (plan.get("why") or "") if plan else ""
    return None

  def _lines(self, inp, wait=""):
    if self.state == "slowing" or self.state == "holding":
      word = "SLOWING FOR A CORNER" if self.state == "slowing" else "HOLDING FOR A CORNER"
      self.line1 = f"{word}: {self.lowered_from} -> {self.shown} mph"
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
                or inp.check_active or inp.standstill or inp.mph < MIN_MPH)

  def _start(self, frame, inp, button, correcting=False):
    """Request one tap. Like the reviewed cruise check (ti_cruise_check._start_tap / _send): the tap waits for a new car button frame
    (a counter change, at most PHASE_WAIT_S), so both pressed frames fall between two of the car's own idle frames and register as
    ONE press; then PRESSED_FRAMES frames FRAME_SPACING apart, each re-checked, under PressGuard."""
    if self.tap is not None or self.tap_to is not None or not self._can_press(inp):
      return None
    if not (self._correct_ok(inp.set_mph) if correcting else self._direction_ok(button, inp.set_mph)):
      return None
    self.tap = Tap(button, PRESSED_FRAMES, frame, inp.counter, inp.set_mph + (1 if button == "SET_PLUS" else -1))
    self.tap_correcting = correcting
    self.tap_from, self.tap_to, self.tap_first = inp.set_mph, self.tap.target, None
    if button == "SET_MINUS":
      self.taps_down += 1
    else:
      self.taps_up += 1
    return self._send(frame, inp)

  def _correct_ok(self, set_mph):
    return set_mph is not None and self.ref is not None and set_mph - 1 >= self.ref

  def _send(self, frame, inp):
    """The next pressed frame of the tap in flight, or None (the cruise check's _send, frame for frame)."""
    tap = self.tap
    if tap is None:
      return None
    out = None
    ok = self._correct_ok(self.tap_from) if self.tap_correcting else self._direction_ok(tap.button, self.tap_from)
    if tap.due(frame, inp.counter) and (inp.set_mph == tap.target or not ok or not self._can_press(inp)):
      tap.n = len(tap.frames)         # landed, or pressing is not allowed now: no further frame of this tap
    elif tap.due(frame, inp.counter):
      if tap.first is None:
        if not self.guard.begin(frame, inp.mono):
          tap.counter0, tap.requested = inp.counter, frame   # too soon after the previous tap: wait, then for the next car frame
          return None
        tap.first = frame
        self.tap_first = frame
      if self.guard.allow(frame, inp.mono):
        tap.frames.append((int(frame), inp.counter, (inp.counter + 1) % 16))
        out = tap.button
      else:
        tap.n = len(tap.frames)       # the cap ends the tap: never a longer press
    if tap.done:
      if tap.first is None:
        tap.first = frame             # refused before its first frame: nothing was sent
        self.tap_first = frame
      self._tap_done(frame, inp)
    return out

  def _correct(self, frame, inp):
    """Stopped with the set speed above the driver's: single aligned SET_MINUS taps down to it, read back one by one."""
    if self.tap is not None:
      return self._send(frame, inp)
    if self.tap_to is not None:
      if inp.set_mph == self.tap_to:
        self.tap_from = self.tap_to = None
      elif inp.set_mph not in (self.tap_from, self.tap_to) or (frame - (self.tap_first or frame)) * DT >= ACCEPT_WAIT_S:
        self.correct_down = False     # it did not go as asked: nothing more
        self.tap_from = self.tap_to = None
        self.line2 = self._left_text(inp.set_mph)
        return None
      else:
        return None
    if not self._correct_ok(inp.set_mph):
      self.correct_down = False
      self.line2 = self._left_text(inp.set_mph)
      return None
    self.line2 = self._left_text(inp.set_mph)
    return self._start(frame, inp, "SET_MINUS", correcting=True)

  def _tap_done(self, frame, inp):
    self._ev("tap", frame, button=self.tap.button, frames=[list(f) for f in self.tap.frames], set_before=self.tap_from,
             set_now=inp.set_mph, correcting=bool(self.tap_correcting))
    sent = bool(self.tap.frames)
    self.guard.end()
    self.tap = None
    if not sent:
      self.tap_from = self.tap_to = None   # refused before any frame was sent: nothing to read back
      if self.tap_correcting:
        self.taps_down -= 1
      return
    if inp.set_mph == self.tap_to:
      self._ev("step", frame, set_mph=inp.set_mph, ms=int(round((frame - self.tap_first) * DT * 1000)))
      self.tap_from = self.tap_to = None
      if self.left_at is not None:
        self.left_at = inp.set_mph

  # ---- the page -------------------------------------------------------------------------------------------------------------------
  def status(self, wall):
    if self.state == "off" and not self.off_seen:   # before the first step / off_view
      self.off_view(None, None, None, 0.0)
    return {"t": round(wall, 1), "v": 1, "state": self.state, "why": self.why, "line1": self.line1, "line2": self.line2,
            "ref": self.ref, "target": self.target, "floor": self.floor, "set": self.set_last, "needs_more": self.needs_more,
            "down": self.taps_down, "up": self.taps_up}

  def off_view(self, plan, plan_age, set_mph, mph):
    """The OFF page: what the planner sees now, so the driver can check it is tracking before switching on. Words only: distances and
    speeds, never a place; nothing here is pressed ("would slow")."""
    self.off_seen = True
    self.line1 = "CURVE SLOWING OFF"
    if self.off_note or self.restart_note:
      self.line2 = self.off_note or self.restart_note
      return
    if plan is None or plan_age is None or plan_age > PLAN_DEAD_S:
      self.line2 = "OFF. Planner not running"
    elif not plan.get("known"):
      why = plan.get("why") or ""
      self.line2 = "OFF. No position yet" if why in ("position uncertain", "no position fix") else "OFF. Road not known here"
    elif not plan.get("trusted"):
      self.line2 = "OFF. No position yet" if plan.get("why") == "position uncertain" else "OFF. Matching the road"
    elif plan.get("ceil") is not None and (plan.get("corner") if plan.get("corner") is not None else plan["ceil"]) <         (set_mph if set_mph else int(round(mph))):
      corner = plan.get("corner") if plan.get("corner") is not None else plan["ceil"]
      d = plan.get("dist")
      where = "in the corner" if d is None or d <= 0 else f"corner in {d / 1609.344:.1f} mi"
      frm = set_mph if set_mph else int(round(mph))
      self.line2 = f"OFF. {where[0].upper() + where[1:]}: would slow {frm} -> {int(corner)} mph"
    else:
      self.line2 = "OFF. Road known here: no slowdown needed ahead"

  @property
  def active(self):
    return self.intervening()
