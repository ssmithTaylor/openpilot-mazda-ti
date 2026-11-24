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
             that was not the step asked for (no restore: the page says "set it yourself"); a tap not read back within ACCEPT_WAIT_S.
             Nothing more is sent until the driver presses a cruise button or re-engages the cruise (then `ref` is the set speed as it
             is); only a pedal / engagement stop that lowered nothing clears by itself, once the cause has cleared for CLEAR_S.
Planner output late (over PLAN_STALE_S) while lowered: hold the set speed, no tap either way (a tap not yet sent is dropped). Planner
dead (over PLAN_DEAD_S) while lowered: `restoring` ("planner stopped: restoring your set speed"): SET_PLUS back to `ref` once past
where the corner ends + TRUST_HOLD_M (the end last known; never known: PLAN_DEAD_WAIT_S after it died), under the restore guards that
do not need the planner (speed, lead flag, settled, paced) and the tap pace; no latch: when the planner is back it works as usual.
Gas pressed (any length): no tap starts, a tap not yet sent is dropped (one already pressing finishes and is read back); tapping
resumes GAS_RESUME_S after the gas is released. Gas held over GAS_MAX_S still stops the intervention, as above.
Low-confidence planner output while lowered: restore gently (same rules as restoring) if the model's preview is gentle, else hold.

Never: a SET_PLUS above `ref`; a SET_MINUS below the floor; taps closer than the reviewed tap machinery allows (ti_cruise_taps.py:
2 pressed frames 50 ms apart, PressGuard caps at 3 frames / 0.15 s, 0.25 s from one tap's last pressed frame to the next); a press on a
frame where openpilot asks for cancel or resume, or while the driver's own cruise button frame shows a press, or while the cruise check
runs; CANCEL or RESUME (this module only ever returns SET_MINUS / SET_PLUS). A lead car: lowering toward a ceiling still happens (the
car does not surge into the corner when the lead leaves).

Pure: no clock, no params; the car controller does the I/O (carcontroller.py `curve_step`). Events go to the `ti_curve` log channel.
"""
import math
from typing import NamedTuple, Optional

from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
from openpilot.selfdrive.car.mazda import ti_curve_plan as P
from openpilot.selfdrive.car.mazda.ti_cruise_taps import DT, FRAME_SPACING, HOLD_ENABLED, PHASE_WAIT_S, HoldGuard, PressGuard, Tap

PARAM_ON = "TiCurveOn"           # tmpfs, the page: "1" on for this drive, "0" / absent off
PARAM_PLAN = "TiCurvePlan"       # tmpfs, the planner (ti_curve_plannerd.py): its output, 5 Hz
PARAM_STATUS = "TiCurveStatus"   # tmpfs, the car controller: what the page shows, on change (at most 10 Hz) and every second
PARAM_DELIVER = "TiCurveDeliver"  # tmpfs, the car controller: stage 3's alert {"m", "need", "why"} at onset, clear, and every 0.2 s
GENTLE_TAP_S = 1.0               # stage 3 (ti_curve_monitor): at most one SET- per this, about 1 mph/s (plan A2)
STATUS_EVERY = 100

PRESSED_FRAMES = 2               # a real tap (the cruise check's recipe A, accepted 3 of 3 on the car)
ACCEPT_WAIT_S = 1.0              # first pressed frame to the set speed reading the step (measured 0.08-0.09 s)
ABS_FLOOR_MPH = 25               # never a set speed below this (the car's own floor is 19)
MAX_SHED_MPH = 15                # never more than this below the known speed limit, else below the speed at the start
MIN_MPH = 20.0                   # no press of either kind below this speed, at standstill or with the cruise in its standstill hold
MPH = 0.44704
GAS_MAX_S = 1.0                  # gas held longer than this stops the intervention
GAS_RESUME_S = 0.5               # shorter gas: no tap starts while pressed nor until this long after it is released
OTHER_BUTTON_PAUSE_S = 2.0       # after RES / distance / mode / cancel
PAUSE_S = 10.0                   # after the driver's own cruise button (owner, first drive: 30 s was too long, "like 10 seconds")
CLEAR_S = 2.0
PLAN_STALE_S = 1.0               # planner output older than this is not used (it publishes at 5 Hz)
PLAN_DEAD_S = 3.0                # ... older than this: the planner is taken as dead
PLAN_DEAD_WAIT_S = 30.0          # planner dead while lowered and the corner's end never known: restore this long after it died
PLAN_DEAD_WHY = "planner stopped: restoring your set speed"
SETTLED_MPH = 3.0                # restore only with the speed within this of the set speed
RESTORE_MIN_MPH = 2              # a restore toward a "hold" ceiling only for 2 mph or more (no dithering)
RESTORE_LEAD_MPH = 7             # while restoring, the set speed is at most this above the car's speed: enough gap for the cruise's
                                 # own natural acceleration (it saturates near 6 mph), without one jump to the old set speed
SETTLED_RESUME_MPH = 2.0         # "speed settling" ends below this gap (it starts above SETTLED_MPH): hysteresis
HOLDING_AT_MPH = 1.0             # slowing -> holding within this of the corner speed; back to slowing above HOLDING_AT_MPH + 1
TRUST_HOLD_M = 30.0              # trust lost during an approach: hold the set speed until this far past where the corner ended
TRUST_HOLD_MAX_S = 30.0          # ... but never longer than this: then stop, leaving the set speed to the driver with a plain message
RESTORE_WAIT_S = 10.0            # a restore that cannot proceed for this long says WAITING TO RESTORE and why
LIMIT_PLAUSIBLE_MPH = 15.0       # a speed limit further than this from the car's speed is not used (stale or wrong, high or low)
STATE_LOG_MIN_S = 1.0            # the same state with a different reason is logged at most this often
TEXT_STALE_S = 3.0               # the page calls a status older than this stale

STATES = ("off", "ready", "slowing", "holding", "restoring", "paused", "stopped", "waiting")

# ---- the held button (plan A7 / A10; ti_cruise_taps.HOLD_ENABLED, OFF until the on-car check) --------------------------------------
# Used only while slowing for a corner (the push down and the raise back to the corner speed; never stage 3's gentle taps, a restore
# or a correction), only where ti_cruise_model.hold_landing allows it (5 mph or more to go, no multiple-of-5 landing past the goal),
# and released the moment the dash reads its stop; the rest is tapped. One pressed frame per car button frame (counter + 1), capped
# by HoldGuard (HOLD_MAX_S, frames and real time). Read back: the dash must walk the hold's path (monotone, never past its stop: else
# "set speed changed ... unexpectedly", as for taps); it must move within HOLD_FIRST_WAIT_S, land on the multiple of 5 within
# HOLD_LAND_WAIT_S and step on within HOLD_STEP_WAIT_S of the last change, else the press is released and the rest of this
# intervention is tapped (no stop: a dash that has not moved HOLD_FALLBACK_S after the first frame is simply tapped from there).
HOLD_FIRST_WAIT_S = 0.3
HOLD_LAND_WAIT_S = 0.8
HOLD_STEP_WAIT_S = 0.7
HOLD_FALLBACK_S = 1.5
HOLD_SETTLE_S = 0.3              # after a release: no press until the dash has been still this long (and this long after the release)

# ---- the push net (owner, 2026-11-24: the SET speed may pass the 15 mph line, the car's speed may not) ------------------------------
# While the dash is below this intervention's floor (a push below the 15 mph line), this side keeps its own model car (the ONE cruise
# model, fed with the dash and synced to the speed every NET_DT_S) and raises the dash back to the floor BY ITSELF the moment the model
# says the car, raised from now with the first tap NET_DELAY_S late, would bottom out within NET_MARGIN_MPH of the floor: whatever the
# planner says, late or dead. Once it has acted, no push below the floor for the rest of the intervention ("push_net" event).
NET_DT_S = 0.05
NET_CHECK_S = 0.1
NET_DELAY_S = 0.0                # the raise modelled from now, as the planner's raise_due (swept: 0.1 / 0.2 s fire before an on-time planner)
NET_MARGIN_MPH = 0.0             # ... and acting when it would bottom out AT the floor: the planner's raise (T + 0.1) always comes first

# ---- in a bend: no raise toward a higher ceiling (review C1 (c), 2026-11-24) ----------------------------------------------------------
# Defensive, on the car's OWN signals (whatever the planner publishes): the car is "in a bend" while max(|measured|, |desired|) lateral
# acceleration is at least BEND_IN_LA (the planner's corner-entry demand, ti_curve_plan.ENTRY_LA); it is out again once that has been
# under BEND_IN_LA - BEND_HYST_LA for BEND_OUT_S. In a bend a SET+ may only bring the dash up to the lowest corner speed this side has
# shown since the bend began (the raise back to the target after a push), or be the push net / the safe exit (back to the floor);
# a restore, or a raise toward a ceiling a planner raised inside the bend, waits until the bend is over.
BEND_IN_LA = P.ENTRY_LA
BEND_HYST_LA = 0.3
BEND_OUT_S = 1.0


class HoldRun:
  """One hold in flight (or settling after its release)."""

  def __init__(self, button, set0, stop, frame, counter):
    self.button, self.s0, self.stop = button, int(set0), int(stop)
    self.d = 1 if button == "SET_PLUS" else -1
    self.requested, self.counter0 = frame, counter
    self.first = None
    self.frames = []
    self.last_counter = None
    self.last_v = int(set0)
    self.last_change = frame
    self.released = None
    self.why = ""

  def on_path(self, v):
    """The dash may only walk from where it last was toward the stop, never past it."""
    if v is None:
      return False
    return (self.stop <= v <= self.last_v) if self.d < 0 else (self.last_v <= v <= self.stop)


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
  driver_set: bool = False        # the driver's pressed button is SET+ or SET- (the buttons that CHOOSE a speed); RES, DISTANCE, MODE
                                  # and CANCEL never are
  driver_set_plus: bool = False   # ... and it is SET+ (a SET+ from a lowered set speed never makes a LOWER speed their choice)
  deliver: bool = False           # stage 3 (ti_curve_monitor): the steering is at its limit and losing the bend (plan T7)
  deliver_need: Optional[int] = None   # ... the speed it walks toward, gently (GENTLE_TAP_S per tap), within the floor
  deliver_why: str = ""           # ... why ("limit", "dropout", "assist off: 0x11 stuck request"), for the page
  la_now: Optional[float] = None       # the car's measured lateral acceleration (m/s^2, the monitor's input), None unknown
  desired_la: Optional[float] = None   # the controller's desired lateral acceleration now (v^2 x desired curvature), None unknown


class Curve:
  def __init__(self, hold=None):
    self.state = "off"
    self.why = ""
    self.floor_now = None         # the floor a new intervention would take now (published for the planner)
    self.guard = PressGuard()
    self.hold_ok = HOLD_ENABLED if hold is None else bool(hold)   # the held button may be used (tests pass hold=True)
    self.hold_guard = HoldGuard()
    self.hold = None              # HoldRun in flight or settling
    self.hold_off = False         # a hold did not behave as measured: taps only for the rest of this intervention
    self.holds = 0
    self.press_floor = None       # the lowest set speed a SET_MINUS may give now: the floor, or the push floor (plan A10)
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
    self.deliver_tap_f = -10 ** 9     # stage 3's last SET- (gentle pacing)
    self.taps_up = 0
    self.gas_since = None
    self.gas_off_at = None            # frame the gas was last released
    self.gas_hold = False             # the gas pressed or released under GAS_RESUME_S ago: no tap may start
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
    self.last_set_press = -10 ** 9    # frame of the driver's last SET+ / SET- press
    self.press_plus = False           # the driver's last SET press was SET+
    self.alert_mph = None             # the on-road warning's corner speed (published; frogpilot shows it)
    self.chose_note = ""              # "Your set speed is now 45 (was 55)": a driver SET press that lowered their own speed
    self.after_brake = False          # a pedal / cruise-off stop left the set speed lowered: restore promptly once the cruise is back
    self.trust_past = False           # trust was lost during an approach and the car has since passed where the corner ended
    self.off_seen = False
    self.dead_since = None            # frame the planner was first seen dead while the set speed was lowered (restoring after it)
    self.net_car = None               # the push net's model car (CM.Cruise), while intervening
    self.net_f = -10 ** 9             # its last update / check
    self.net_chk = -10 ** 9
    self.net_up = False               # the push net acted: the dash back to the floor, no push below it for this intervention
    self.in_bend = False              # review C1 (c): the car's own signals say it is in a bend (BEND_*)
    self.bend_low_f = None            # frame the lateral acceleration first fell under the exit level
    self.bend_lo = None               # the highest a SET+ may bring the dash while in this bend (None: no limit known)
    self.bend_held = False            # a SET+ was held back by the bend rule (logged once per bend)

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
    self._cut_hold()
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
      # a pedal or the cruise going off left it lowered: once the cruise is back (RES or SET), the driver's own set speed is put back
      # promptly (owner, first drive: a brake for a traffic light must never leave the lowered speed as theirs)
      self.after_brake = why in ("brake pressed", "gas held", "cruise off", "openpilot disengaged", "cruise unavailable")
    if why.startswith("set speed changed") and inp.set_mph is not None and self.ref is not None and inp.set_mph < self.ref:
      # a change nobody asked for (possibly the driver's press the button flag missed): never restored, nothing more pressed until
      # the driver acts; `ref` is kept only as the limit for how far a later intervention may go
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

  def _cut_hold(self, frame=None, why="cut"):
    """End a hold at once: no further held frame (the dash is then read as it is; nothing more is awaited from it)."""
    h = self.hold
    if h is None:
      return
    self.hold_guard.end()
    if h.released is None:
      self._ev("hold", h.requested if frame is None else frame, button=h.button, set_before=h.s0, stop=h.stop, why=why,
               frames=[list(f) for f in h.frames], set_now=h.last_v)
    self.hold = None

  def _end_intervention(self, frame):
    self._cut_hold(frame, "intervention ended")
    self.hold_off = False
    self.press_floor = None
    self.target, self.floor, self.needs_more = None, None, False
    if self.left_at is None:
      self.lowered_from = None
    self.taps_down = self.taps_up = 0
    self.trust_hold_since = self.restore_wait_since = None
    self.dead_since = None
    self.net_car, self.net_up = None, False
    if self.left_at is None:
      self.corner_end_odo = None

  def _adopt(self, set_mph):
    """A set speed the driver may have chosen. While a set speed this feature left lowered is outstanding, only a value reached after
    the driver's own press other than RES is their new choice; a RES (back to the stored, lowered value) or anything unexplained is
    not, and `ref` stays their original choice."""
    if set_mph is None:
      return
    if self.left_at is None or self.ref is None:
      self.ref = set_mph
    elif self.own_press:
      # any SET+ / SET- of the driver defines their speed, full stop (the reviewed rule); when it is LOWER than before, say so
      self._ev("driver_chose", 0, set_mph=set_mph, was=self.ref)
      if self.ref is not None and set_mph < self.ref:
        self.chose_note = f"Your set speed is now {set_mph} (was {self.ref})"
      self.ref = set_mph
      self.left_at, self.own_press, self.left_doubt, self.lowered_from = None, False, False, None
      self.after_brake = False
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
    self._bend_track(frame, inp)
    if inp.gas:
      self.gas_off_at = None
    elif self.gas_hold and self.gas_off_at is None:
      self.gas_off_at = frame
    self.gas_hold = bool(inp.gas) or (self.gas_off_at is not None and (frame - self.gas_off_at) * DT < GAS_RESUME_S - 1e-9)
    if inp.driver_button and inp.driver_set:
      self.last_set_press = frame
    if cruise_rose and (frame - self.last_set_press) * DT <= 1.5:
      self.own_press = True               # the cruise set afresh with SET (by the button, whatever the value): the driver's new choice

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
      if btn_rose and inp.driver_set:
        self.off_note = self.restart_note = ""
        self.own_press = True
      self.off_view(inp.plan, inp.plan_age, inp.set_mph, inp.mph)
      return None
    self._update_alert(inp)
    self.floor_now = self._floor_for(inp, inp.plan if self._plan_ok(inp) else None)
    self._net_track(frame, inp)
    if self.state == "off":
      if inp.cruise_on:
        self._adopt(inp.set_mph)
      self.off_note = self.restart_note = ""
      self._to(frame, "ready")

    # the driver's own cruise button: ends any intervention, adopts their set speed, pauses
    if btn:
      if btn_rose:
        self._cut_tap()
        self.tap_from = self.tap_to = None       # the driver's own press supersedes any read-back still awaited
        if inp.set_mph is not None and self.ref is not None and inp.set_mph < self.ref and self.left_at is None and \
           (self.intervening() or self.lowered_from is not None):
          # the set speed is below the driver's own: remember it as left lowered, so only their SET+ / SET- can make it the new choice
          self.left_at, self.left_doubt = inp.set_mph, False
          self.lowered_from = self.ref
          self.own_press = False
        if inp.driver_set:
          self.own_press = True
          self.press_plus = bool(inp.driver_set_plus)
          self.chose_note = ""
        if self.intervening() or self.state in ("ready", "stopped", "waiting"):
          self._ev("driver_button", frame, set_mph=inp.set_mph, ref=self.ref, during=self.state, set_button=bool(inp.driver_set))
        self._end_intervention(frame)
        # a SET+ / SET- pauses PAUSE_S; RES, distance, mode, cancel only briefly (after brake + RES the driver's own set speed is
        # put back promptly, and a warning is never held back)
        self.pause_until = t + (PAUSE_S if inp.driver_set else OTHER_BUTTON_PAUSE_S)
        self._to(frame, "paused", "you pressed a cruise button")
      self.line1 = "PAUSED: you pressed a cruise button"
      self.line2 = self.chose_note or f"back in {max(0.0, (self.pause_until or t) - t):.0f} s"
      return None
    if self.state == "paused":
      if inp.set_mph is not None and inp.cruise_on and (frame - (self.set_stable_since or frame)) * DT >= 1.0:
        self._adopt(inp.set_mph)                # their new set speed, once it has settled (a RES is not a new choice)
      if t < (self.pause_until or 0):
        self.line1 = "PAUSED: you pressed a cruise button"
        self.line2 = self.chose_note or f"back in {max(0.0, self.pause_until - t):.0f} s"
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
      elif not why and self.why == "gas held" and self.after_brake and not self.correct_down and not self.left_doubt:
        # the gas override is over and the cruise is still on: put the driver's set speed back like after brake + RES
        self.clear_since = frame if self.clear_since is None else self.clear_since
        if (frame - self.clear_since) * DT >= CLEAR_S:
          self._to(frame, "ready")
          self.clear_since = None
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

    # a hold in flight or settling: the set speed may only walk its path (from where it was toward its stop, never past it)
    if self.hold is not None:
      h = self.hold
      if not h.on_path(inp.set_mph):
        self._cut_hold(frame, "off its path")
        self._stop(frame, inp, f"set speed changed to {inp.set_mph} mph unexpectedly")
        return None
      if inp.set_mph != h.last_v:
        h.last_v, h.last_change = inp.set_mph, frame
    # a tap in flight: the set speed must be the value it started from or the step it asks for; anything else is unexpected
    elif self.tap is not None or self.tap_to is not None:
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
    if plan_ok:
      self.dead_since = None
    late_hold = dead_restore = False
    if not plan_ok and self.intervening() and inp.set_mph < self.ref:
      if dead:
        # the planner dead while lowered: restore the driver's set speed once past the corner (never latched; when the planner
        # comes back the feature works as usual)
        if self.dead_since is None:
          self.dead_since = frame
          self._ev("planner_dead", frame, set_mph=inp.set_mph, ref=self.ref, corner_end_known=self.corner_end_odo is not None)
        dead_restore = True
      else:
        late_hold = True                    # merely late: hold the set speed, no tap either way
      if self.tap is not None and self.tap.first is None and not self.tap_correcting and not self.net_up and \
         (late_hold or not self._dead_past(frame)):
        self._cut_tap()                     # a tap not yet sent: not sent (one already pressing finishes and is read back)
      if self.hold is not None and self.hold.released is None:
        self._release_hold(frame, "planner output late" if late_hold else "planner stopped")   # a hold never runs on

    if self.tap is not None and self.tap.first is None and self.tap.button == "SET_MINUS" and not self.tap_correcting:
      p_ = inp.plan if plan_ok else None
      corner_on = bool(p_ and p_.get("trusted") and p_.get("ceil") is not None and p_.get("kind", "corner") != "hold")
      if not corner_on and not (inp.deliver and inp.deliver_need is not None):
        self._cut_tap()                     # a SET_MINUS not yet sent whose corner ceiling has gone: not sent
    if self.hold is not None:
      done, out = self._hold_step(frame, inp)
      if not done:
        return out
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

    net = self._push_net(frame, inp)
    if net is not None or self.tap is not None:
      return net

    if late_hold:
      self.target = self.shown = inp.set_mph
      self._to(frame, "holding", "planner output late: holding")
      self.line1 = f"HOLDING: {inp.set_mph} mph"
      self.line2 = "planner output late: holding"
      return None
    if dead_restore:
      return self._dead_restore(frame, inp)

    plan = inp.plan if plan_ok else None
    ceil = plan.get("ceil") if plan else None
    trusted = bool(plan.get("trusted")) if plan else False
    kind = plan.get("kind", "corner") if plan else ""
    gentle = False
    if inp.deliver and inp.deliver_need is not None and plan is not None:
      # stage 3 (plan T7): the steering is losing the bend: walk toward its need, gently, as one more corner ceiling (the lower wins;
      # floors and the driver's set speed apply as for any corner)
      dn = int(inp.deliver_need)
      if ceil is None or not trusted or kind == "hold" or dn < int(ceil):
        end = plan.get("end") if (trusted and kind != "hold" and plan.get("end") is not None) else 0.0
        plan = dict(plan, ceil=dn, corner=dn, need=dn, trusted=True, kind="corner", end=end)
        ceil, trusted, kind, gentle = dn, True, "corner", True
    self.gentle_why = inp.deliver_why if gentle else ""
    if ceil is not None and trusted and kind != "hold" and (plan.get("end") is not None or plan.get("dist") is not None):
      # where the remembered tight stretch ENDS (the planner's "end"; "dist" is to its start)
      self.corner_end_odo = self.odo + max(0.0, float(plan["end"] if plan.get("end") is not None else plan["dist"]))
    if self.intervening() and not trusted and self.corner_end_odo is not None and self.odo >= self.corner_end_odo + TRUST_HOLD_M:
      self.trust_past = True              # trust lost, but the car is now past where the remembered corner ended
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
        self.floor = self._floor_for(inp, plan)
        self.lowered_from = self.ref
        self.restore_wait_since = None
        self.trust_past = False
        self._ev("start", frame, ceil=ceil, corner=plan.get("corner"), floor=self.floor, limit=lim, mph=round(inp.mph, 1),
                 set_mph=inp.set_mph, ref=self.ref, dist=plan.get("dist"), lead=inp.lead)
      # the set speed the planner asks for now (the push below the corner speed, then the corner speed), within the floor and
      # never above the driver's own set speed
      if not (bool(inp.lead) and inp.mph < self.ref - 2.0):
        lim_ = plan.get("limit") if plan.get("limit") and abs(float(plan["limit"]) - inp.mph) <= LIMIT_PLAUSIBLE_MPH else None
        self.floor = max(self.floor, ABS_FLOOR_MPH, self.ref - MAX_SHED_MPH, (int(lim_) - MAX_SHED_MPH) if lim_ else 0)
      corner = plan.get("corner") if plan.get("corner") is not None else ceil
      if int(ceil) < int(corner) and not gentle:
        # a PUSH below the corner speed (plan A10): the maximum drop binds the corner speed, the push may go P.DRIVE_GAP_MPH below
        # it but never below the push floor (ABS_FLOOR_MPH, the car's minimum)
        self.press_floor = CM.push_floor(self.floor, P.DRIVE_GAP_MPH)
        self.target = min(max(int(ceil), self.press_floor, max(int(corner), self.floor) - P.DRIVE_GAP_MPH), self.ref)
      else:
        self.press_floor = self.floor
        self.target = min(max(int(ceil), self.floor), self.ref)
      if self.net_up:
        self.press_floor = self.floor           # the push net acted: no push below the floor for the rest of this intervention
        self.target = max(self.target, min(self.floor, self.ref))
      self.shown = min(max(int(corner), self.floor), self.ref)
      if self.in_bend:
        self.bend_lo = self.shown if self.bend_lo is None else min(self.bend_lo, self.shown)
      need_true = plan.get("need") if plan.get("need") is not None else corner
      needs_more = int(need_true) < self.floor        # ONE meaning: the corner's TRUE need is below the floor (never "behind")
      self.behind = bool(plan.get("behind"))
      if needs_more and not self.needs_more:
        self._ev("needs_more", frame, ceil=ceil, corner=corner, floor=self.floor, mph=round(inp.mph, 1))
      self.needs_more = needs_more
      if inp.set_mph > self.target:
        self._to(frame, "slowing")
        self._lines(inp)
        if gentle and (frame - self.deliver_tap_f) * DT < GENTLE_TAP_S - 1e-9:
          return None                          # stage 3: gently (about 1 mph/s)
        b = self._start(frame, inp, "SET_MINUS", allow_hold=not gentle)
        if gentle and self.tap is not None:
          self.deliver_tap_f = frame
        return b
      near = inp.mph <= self.shown + HOLDING_AT_MPH + (1.0 if self.state == "holding" else 0.0)   # hysteresis
      self._to(frame, "holding" if near else "slowing")
      self._lines(inp)
      if inp.set_mph < self.target:
        # the planned raise from the push back to the corner speed as the car reaches it (never above `ref`)
        return self._start(frame, inp, "SET_PLUS", allow_hold=not gentle)
      return None

    # no corner ceiling (past the corner, a "hold" for the next one, or the planner not trusted): restore if lowered, else idle
    if self.intervening() or (self.ref is not None and inp.set_mph is not None and inp.set_mph < self.ref and self.lowered_from is not None):
      if inp.set_mph >= self.ref:
        self._ev("restored", frame, set_mph=inp.set_mph, taps_down=self.taps_down, taps_up=self.taps_up, after_brake=self.after_brake)
        self.after_brake = False
        self.chose_note = ""
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

  def _net_track(self, frame, inp):
    """The push net's model car, fed every NET_DT_S of an intervention (every frame is seen, whatever else waits)."""
    if not self.intervening() or self.floor is None or inp.set_mph is None or inp.mph is None:
      self.net_car = None
      return
    if self.net_car is None:
      self.net_car, self.net_f = CM.Cruise(inp.mph, inp.set_mph, NET_DT_S), frame
    elif (frame - self.net_f) * DT >= NET_DT_S - 1e-9:
      self.net_car.v = inp.mph                 # synced to the real speed; the gap history and the lag are the model's
      self.net_car.step(inp.set_mph)
      self.net_f = frame

  def _push_net(self, frame, inp):
    """The push net (see NET_*): SET_PLUS toward the floor once the model says so, or None. Runs only while intervening."""
    if self.net_car is None or self.floor is None or inp.set_mph is None:
      return None
    fl = min(self.floor, self.ref) if self.ref is not None else self.floor
    if inp.set_mph >= fl:
      return None
    if not self.net_up and (frame - self.net_chk) * DT >= NET_CHECK_S - 1e-9:
      self.net_chk = frame
      lo = CM.min_after_raise(self.net_car, inp.set_mph, fl, CM.TAP_S, False, NET_DELAY_S)
      if lo <= fl + NET_MARGIN_MPH:
        self.net_up = True
        self._ev("push_net", frame, set_mph=inp.set_mph, floor=fl, mph=round(inp.mph, 1), model_low=round(lo, 2),
                 plan_age=None if inp.plan_age is None else round(inp.plan_age, 2))
    if not self.net_up:
      return None
    self.target = fl
    self._to(frame, "slowing", "raising to the floor")
    return self._start(frame, inp, "SET_PLUS", exempt=True)

  def _floor_for(self, inp, plan):
    """The floor a new intervention would take NOW: max(ABS_FLOOR_MPH, (ref, or min(ref, speed) when a lead holds the car under the
    driver's set speed) - MAX_SHED_MPH, a plausible limit - MAX_SHED_MPH). ONE definition, also published (status "floor_now") so
    the planner plans and warns with the same floor (fourth review, item 4)."""
    if self.ref is None:
      return None
    lim = (plan or {}).get("limit")
    if lim and abs(float(lim) - inp.mph) > LIMIT_PLAUSIBLE_MPH:
      lim = None
    lead_held = bool(inp.lead) and inp.mph < self.ref - 2.0       # a lead holds the car below the DRIVER's set speed
    base = min(self.ref, int(round(inp.mph))) if lead_held else self.ref
    return max(ABS_FLOOR_MPH, base - MAX_SHED_MPH, (int(lim) - MAX_SHED_MPH) if lim else 0)

  def _update_alert(self, inp):
    """The warning is the PLANNER's one rule (ti_curve_plan.Planner._warn, from this side's floor and state relayed by the daemon);
    this side only relays it for the page, every frame, and drops it only when the planner's output is late or the feature is off."""
    plan = inp.plan if self._plan_ok(inp) else None
    a = plan.get("alert") if plan else None
    self.alert_mph = int(a) if a else None

  def _dead_past(self, frame):
    """Planner dead while lowered: past where the corner it was slowing for ends, plus TRUST_HOLD_M (the end the car side last knew);
    never knew one: PLAN_DEAD_WAIT_S after the planner died."""
    if self.corner_end_odo is not None:
      return self.odo >= self.corner_end_odo + TRUST_HOLD_M
    return self.dead_since is not None and (frame - self.dead_since) * DT >= PLAN_DEAD_WAIT_S - 1e-9

  def _dead_restore(self, frame, inp):
    """The planner dead with the set speed lowered: SET_PLUS taps back to `ref` once past the corner, under the normal restore's
    guards that do not need the planner (speed, standstill, lead flag readable, settled, paced to the car) and the tap machinery's
    pace; the driver's buttons, brake and gas are handled before this, as always."""
    self.target = self.shown = self.ref
    if not self._dead_past(frame):
      ok, wait = False, "planner stopped: waiting until past the corner"
    elif inp.mph < MIN_MPH or inp.standstill:
      ok, wait = False, f"below {MIN_MPH:.0f} mph: no presses"
    else:
      ok, wait = self._restore_common(inp)
    self._to(frame, "restoring", PLAN_DEAD_WHY)
    self.line1 = f"RESTORING: {inp.set_mph} -> {self.ref} mph"
    self.line2 = wait or PLAN_DEAD_WHY
    if ok:
      return self._start(frame, inp, "SET_PLUS")
    return None

  def _restore_ok(self, inp, plan):
    if inp.mph < MIN_MPH or inp.standstill:
      return False, f"below {MIN_MPH:.0f} mph: no presses"
    if plan is None:
      return False, "planner output late"
    past_end = self.corner_end_odo is not None and self.odo >= self.corner_end_odo + TRUST_HOLD_M
    if not plan.get("trusted") and not self.trust_past and not past_end and not self.after_brake:
      return False, "road not matched: not restoring"
    if plan.get("model_k") is None:
      return False, "no model preview"
    if not P.fits(float(plan["model_k"]), inp.set_mph + 1):
      return False, "the model sees a bend ahead"
    return self._restore_common(inp)

  def _restore_common(self, inp):
    """The restore guards that do not need the planner: lead flag readable, speed settled, paced to the car."""
    if inp.lead is None:
      return False, "lead flag unreadable"
    gap = inp.mph - inp.set_mph
    self.settling = gap > (SETTLED_RESUME_MPH if self.settling else SETTLED_MPH)      # hysteresis
    if self.settling:
      return False, "speed settling"
    if inp.set_mph + 1 > inp.mph + RESTORE_LEAD_MPH and not self.after_brake:
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
      gw = getattr(self, "gentle_why", "")
      self.line2 = (("assist off: slowing gently" if gw.startswith("assist off") else "steering at its limit: slowing gently") if gw else
                    "CORNER NEEDS MORE THAN I MAY SLOW: be ready" if self.needs_more else
                    "slowing as fast as the cruise allows" if getattr(self, "behind", False) else f"set speed now {inp.set_mph} mph")
    elif self.after_brake:
      self.line1 = f"RESTORING YOUR SET SPEED: {inp.set_mph} -> {self.ref} mph"
      self.line2 = wait or "after the brake: back to your own set speed"
    else:
      self.line1 = f"RESTORING: {inp.set_mph} -> {self.ref} mph"
      self.line2 = wait or "speeding back up"

  # ---- taps ---------------------------------------------------------------------------------------------------------------------
  def _direction_ok(self, button, set_mph):
    if set_mph is None or self.ref is None:
      return False
    if button == "SET_PLUS":
      return set_mph + 1 <= self.ref
    lo = self.press_floor if self.press_floor is not None else self.floor
    return lo is not None and set_mph - 1 >= lo and set_mph - 1 >= ABS_FLOOR_MPH and \
      self.target is not None and set_mph > self.target

  def _can_press(self, inp):
    return not (inp.cancel_req or inp.resume_req or inp.driver_button or inp.driver_button is None or inp.counter is None
                or inp.check_active or inp.standstill or inp.mph < MIN_MPH)

  def _bend_track(self, frame, inp):
    """Review C1 (c): in a bend or not, from the car's own lateral acceleration (measured and desired); see BEND_*."""
    vals = [abs(x) for x in (inp.la_now, inp.desired_la) if x is not None]
    la = max(vals) if vals else None
    if la is None:
      self.in_bend, self.bend_low_f, self.bend_lo, self.bend_held = False, None, None, False
      return
    if la >= BEND_IN_LA:
      self.bend_low_f = None
      if not self.in_bend:
        self.in_bend, self.bend_held = True, False
        # the dash may come up to the corner speed being slowed for (a raise after a push), else it stays where it is
        self.bend_lo = self.shown if (self.state in ("slowing", "holding") and self.shown is not None) else inp.set_mph
      return
    if self.in_bend and la < BEND_IN_LA - BEND_HYST_LA:
      self.bend_low_f = frame if self.bend_low_f is None else self.bend_low_f
      if (frame - self.bend_low_f) * DT >= BEND_OUT_S - 1e-9:
        self.in_bend, self.bend_low_f, self.bend_lo, self.bend_held = False, None, None, False
    else:
      self.bend_low_f = None

  def _bend_blocks(self, frame, inp, button, exempt):
    """True: a SET+ that would take the dash above what the bend allows (BEND_*); the push net and the safe exit are exempt."""
    if button != "SET_PLUS" or exempt or not self.in_bend or inp.set_mph is None:
      return False
    lo = self.bend_lo if self.bend_lo is not None else inp.set_mph
    if inp.set_mph + 1 <= lo:
      return False
    if not self.bend_held:
      self.bend_held = True
      self._ev("bend_hold", frame, set_mph=inp.set_mph, allowed=lo, target=self.target, state=self.state)
    return True

  def _start(self, frame, inp, button, correcting=False, allow_hold=False, exempt=False):
    """Request one tap. Like the reviewed cruise check (ti_cruise_check._start_tap / _send): the tap waits for a new car button frame
    (a counter change, at most PHASE_WAIT_S), so both pressed frames fall between two of the car's own idle frames and register as
    ONE press; then PRESSED_FRAMES frames FRAME_SPACING apart, each re-checked, under PressGuard. With `allow_hold` (slowing for a
    corner only) a hold is started instead where it is allowed (`_start_hold`)."""
    if self.tap is not None or self.tap_to is not None or self.hold is not None or not self._can_press(inp) or self.gas_hold:
      return None
    if not (self._correct_ok(inp.set_mph) if correcting else self._direction_ok(button, inp.set_mph)):
      return None
    if not correcting and self._bend_blocks(frame, inp, button, exempt):
      return None
    if self.in_bend and button == "SET_PLUS":
      allow_hold = False                  # in a bend a raise is tapped, one step at a time, each checked against the bend's limit
    if allow_hold and not correcting and self._start_hold(frame, inp, button):
      _, out = self._hold_step(frame, inp)
      return out
    self.tap = Tap(button, PRESSED_FRAMES, frame, inp.counter, inp.set_mph + (1 if button == "SET_PLUS" else -1))
    self.tap_correcting = correcting
    self.tap_from, self.tap_to, self.tap_first = inp.set_mph, self.tap.target, None
    if button == "SET_MINUS":
      self.taps_down += 1
    else:
      self.taps_up += 1
    return self._send(frame, inp)

  def _start_hold(self, frame, inp, button):
    """A hold instead of a tap where it is allowed (see HOLD_*); None when it is not (the caller taps)."""
    if not self.hold_ok or self.hold_off or self.hold is not None or self.target is None or inp.set_mph is None:
      return None
    hl = CM.hold_landing(inp.set_mph, self.target)
    if hl is None:
      return None
    stop = hl[0]
    if button == "SET_MINUS":
      lo = self.press_floor if self.press_floor is not None else self.floor
      if lo is None or stop < lo or stop < ABS_FLOOR_MPH or stop < self.target or stop >= inp.set_mph:
        return None
    elif self.ref is None or stop > self.ref or stop > self.target or stop <= inp.set_mph:
      return None
    self.hold = HoldRun(button, inp.set_mph, stop, frame, inp.counter)
    self.holds += 1
    return True

  def _release_hold(self, frame, why):
    h = self.hold
    if h is None or h.released is not None:
      return
    self.hold_guard.end()
    h.released, h.why = frame, why
    if why != "landed":
      self.hold_off = True                # not as measured (or cut short): taps for the rest of this intervention
    self._ev("hold", frame, button=h.button, set_before=h.s0, stop=h.stop, why=why, frames=[list(f) for f in h.frames],
             set_now=h.last_v, ms=None if h.first is None else int(round((frame - h.first) * DT * 1000)))

  def _hold_step(self, frame, inp):
    """The hold in flight: -> (done, button). done False: the caller returns `button` (a held frame or None); done True: the hold is
    over and settled, the caller goes on as usual this frame."""
    h = self.hold
    if h.released is None:
      why = ""
      t_on = None if h.first is None else (frame - h.first) * DT
      moved = abs(inp.set_mph - h.s0)
      if not self._can_press(inp) or self.gas_hold:
        why = "pressing not allowed"
      elif inp.set_mph == h.stop:
        why = "landed"
      elif self.target is None or (h.d < 0 and h.stop < self.target) or (h.d > 0 and h.stop > self.target):
        why = "target changed"
      elif t_on is not None and moved == 0 and t_on >= HOLD_FIRST_WAIT_S - 1e-9:
        why = "no response"
      elif t_on is not None and moved <= 1 and t_on >= HOLD_LAND_WAIT_S - 1e-9:
        why = "no landing"
      elif t_on is not None and moved > 1 and (frame - h.last_change) * DT >= HOLD_STEP_WAIT_S - 1e-9:
        why = "no step"
      if why:
        if h.first is None:
          self.hold = None                # nothing was sent
          return True, None
        self._release_hold(frame, why)
        return False, None
      if h.first is None:
        if inp.counter == h.counter0 and (frame - h.requested) * DT < PHASE_WAIT_S - 1e-9:
          return False, None              # wait for a new car button frame, as a tap does
        if not self.hold_guard.begin(frame, inp.mono, other=self.guard):
          h.counter0, h.requested = inp.counter, frame
          return False, None
        h.first, h.last_change = frame, frame
      elif inp.counter == h.last_counter:
        return False, None                # one held frame per car button frame
      if not self.hold_guard.allow(frame, inp.mono):
        self._release_hold(frame, "cap")
        return False, None
      h.frames.append((int(frame), inp.counter, (inp.counter + 1) % 16))
      h.last_counter = inp.counter
      return False, h.button
    # released: settle before anything else is pressed (a late step on the path is accepted, never a press over it)
    if h.why == "no response" and inp.set_mph == h.s0 and (frame - h.first) * DT < HOLD_FALLBACK_S - 1e-9:
      return False, None
    if (frame - h.released) * DT < HOLD_SETTLE_S - 1e-9 or (frame - h.last_change) * DT < HOLD_SETTLE_S - 1e-9:
      return False, None
    self.hold = None
    if self.left_at is not None and inp.set_mph is not None:
      self.left_at = inp.set_mph
    return True, None

  def _correct_ok(self, set_mph):
    return set_mph is not None and self.ref is not None and set_mph - 1 >= self.ref

  def _send(self, frame, inp):
    """The next pressed frame of the tap in flight, or None (the cruise check's _send, frame for frame)."""
    tap = self.tap
    if tap is None:
      return None
    if tap.first is None and self.gas_hold:
      self._cut_tap()                 # the gas: a tap not yet sent is not sent (one already pressing finishes and is read back)
      return None
    out = None
    ok = self._correct_ok(self.tap_from) if self.tap_correcting else self._direction_ok(tap.button, self.tap_from)
    if tap.due(frame, inp.counter) and (inp.set_mph == tap.target or not ok or not self._can_press(inp)):
      tap.n = len(tap.frames)         # landed, or pressing is not allowed now: no further frame of this tap
    elif tap.due(frame, inp.counter):
      if tap.first is None:
        hg = self.hold_guard
        hold_gap = (hg.last_frame is None or (frame - hg.last_frame) * DT >= self.guard.min_gap_s - 1e-9) and \
          (inp.mono is None or hg.last_mono is None or inp.mono - hg.last_mono >= self.guard.min_gap_s - 1e-6)
        if not hold_gap or not self.guard.begin(frame, inp.mono):
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
    if self.state != "off" and self.alert_mph:
      # the page agrees with the on-road warning, whatever the state (never "no known corner ahead" while it shows)
      self.line1 = f"SLOW DOWN: corner needs {self.alert_mph} mph"
      self.line2 = "more than curve slowing can do: brake"
    return {"t": round(wall, 1), "v": 1, "state": self.state, "why": self.why, "line1": self.line1, "line2": self.line2,
            "ref": self.ref, "target": self.target, "floor": self.floor, "set": self.set_last, "needs_more": self.needs_more,
            "floor_now": self.floor_now,
            "alert": self.alert_mph if self.state != "off" else None,
            "acting": self.state in ("ready", "slowing", "holding", "restoring") and not self.left_doubt,
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


# ---- the SAFE EXIT (review C2, 2026-11-24) -----------------------------------------------------------------------------------------
# Whenever curve slowing stops for any reason other than the driver's own cruise button (their value stands) or the cruise itself
# going off, with the dash set speed below the EXIT LINE (max(ABS_FLOOR_MPH, ref - MAX_SHED_MPH, the intervention's floor), never above
# the driver's reference `ref`), SET+ taps raise it back to that line and stop. Independent of the Curve object (which may have
# failed): it keeps the minimal state itself (ref, floor, last dash value), has its own tap sender on the reviewed machinery
# (ti_cruise_taps Tap / PressGuard: aligned 2-frame taps, 0.25 s gap, read back within ACCEPT_WAIT_S), and the car controller keeps it
# outside the Curve. It waits (never fights) while the brake or gas is pressed, openpilot is disengaged, openpilot asks for cancel or
# resume, the driver's button frame shows a press or is unreadable, the cruise check runs, at standstill or under MIN_MPH; it ends at
# once on a driver cruise button or the cruise going off; a set speed moving in a way it did not ask is left to the driver.
# A tap not read back ("set speed did not respond") is the one case it cannot fix: it says so on the page and on the road (I4).
# Its marker ("exit" in the status key TiCurveStatus, params memory) is published whenever the dash is below the line while curve
# slowing has it lowered, so a CONTROLS RESTART finds it (the car controller reads the old status before clearing it, as for the
# restart note): the exit then runs from the marker's ref and floor (no ref in it: the car's speed rounded up, never above it).
EXIT_RAISE_ON_SWITCH_OFF = True  # switching the feature off mid-push still sends these few taps (the owner is to confirm)
EXIT_START_S = 0.3               # after arming: no tap before this, and the dash still this long (a tap in flight lands first)
EXIT_WAIT_MAX_S = 60.0           # waiting longer than this (pedal, disengaged, ...) gives up: "set it yourself" (and the alert)
EXIT_EXEMPT = ("cruise off", "cruise unavailable")


def exit_line(ref, floor):
  """The line the safe exit raises the dash back to: max(ABS_FLOOR_MPH, ref - MAX_SHED_MPH, floor), never above ref."""
  if ref is None:
    return None
  return int(min(int(ref), max(ABS_FLOOR_MPH, int(ref) - MAX_SHED_MPH, int(floor) if floor else 0)))


class SafeExit:
  def __init__(self):
    self.ref = self.floor = self.cmd = None   # held from the Curve every frame it runs fine (never cleared by a switch-off)
    self.active = False
    self.why = ""
    self.line = None
    self.ref_unknown = False
    self.armed_f = None
    self.wait_since = None
    self.result = None            # None, "done", "driver", "cruise off", "changed", "failed", "gave up", "curve slowing back in charge"
    self.guard = PressGuard()
    self.tap = None
    self.tap_to = None
    self.tap_first = None
    self.set_last, self.set_since = None, None
    self.events = []
    self.line1 = self.line2 = ""

  def _ev(self, kind, frame, **kw):
    self.events.append({"kind": kind, "frame": int(frame), **kw})

  # ---- the state it keeps (outside the Curve) -----------------------------------------------------------------------------------
  def hold_from(self, cv, inp):
    """Every frame the Curve ran without error: keep its reference and floor (a switch-off cannot clear them here)."""
    if cv is None or cv.ref is None or cv.state == "off":
      return
    self.ref = int(cv.ref)
    fl = cv.floor if cv.floor is not None else cv.floor_now
    if fl is not None:
      self.floor = int(fl)
    if inp.set_mph is not None:
      self.cmd = int(inp.set_mph)
    if cv.guard.last_frame is not None:
      self.guard.last_frame, self.guard.last_mono = cv.guard.last_frame, cv.guard.last_mono   # the reviewed gap spans both senders

  def marker(self, cv, set_mph):
    """The persisted "pushed below the line" marker (for the status key), or None."""
    line = self.line if self.active else exit_line(self.ref, self.floor)
    lowered = self.active or (cv is not None and (cv.intervening() or cv.state == "stopped"))
    if line is None or set_mph is None or set_mph >= line or not lowered:
      return None
    return {"ref": self.ref, "floor": self.floor, "line": line}

  # ---- arming ------------------------------------------------------------------------------------------------------------------
  def observe(self, frame, cv, inp):
    """After the Curve's step (its events not yet cleared): arm on a stop / switch-off that left the dash below the line; end when
    the Curve takes charge again (it intervenes) or the driver pressed a cruise button."""
    for ev in cv.events:
      k = ev.get("kind")
      if k == "driver_button":
        self.cancel(frame, "driver")
      elif k == "stop" and ev.get("why") not in EXIT_EXEMPT:
        self.arm(frame, inp, ev.get("why") or "stopped")
      elif k == "switched_off" and EXIT_RAISE_ON_SWITCH_OFF:
        self.arm(frame, inp, "switched off")
    if self.active and cv.intervening():
      self.cancel(frame, "curve slowing back in charge")

  def arm(self, frame, inp, why, ref=None, floor=None, restart=False):
    """Start the exit (if the dash is below the line). restart: from a persisted marker (ref / floor given, ref may be unknown)."""
    if restart:
      self.ref, self.floor = (None if ref is None else int(ref)), (None if floor is None else int(floor))
    self.ref_unknown = self.ref is None
    line = exit_line(self.ref, self.floor)
    if line is None and not restart:
      return False
    if line is not None and inp is not None and inp.set_mph is not None and inp.set_mph >= line:
      return False
    if not restart and (inp is None or inp.set_mph is None):
      return False
    self.active, self.why, self.line, self.result = True, why, line, None
    self.armed_f, self.wait_since = frame, None
    self.tap = self.tap_to = self.tap_first = None
    self._ev("exit_armed", frame, why=why, set_mph=None if inp is None else inp.set_mph, ref=self.ref, floor=self.floor, line=line)
    return True

  def cancel(self, frame, result):
    if self.active:
      self._finish(frame, result)

  def _finish(self, frame, result, set_mph=None):
    self.active = False
    self.result = result
    if self.tap is not None:
      self.guard.end()
    self.tap = self.tap_to = self.tap_first = None
    self._ev("exit_end", frame, result=result, why=self.why, set_mph=set_mph, line=self.line, ref=self.ref)

  # ---- one frame ----------------------------------------------------------------------------------------------------------------
  @staticmethod
  def _blocked(inp, gas_recent):
    if inp.brake:
      return "brake pressed"
    if inp.gas or gas_recent:
      return "gas pressed"
    if not inp.engaged:
      return "openpilot disengaged"
    if inp.cancel_req or inp.resume_req or inp.driver_button is None or inp.counter is None:
      return "buttons busy"
    if inp.check_active:
      return "the cruise check is running"
    if inp.standstill or inp.mph < MIN_MPH:
      return f"below {MIN_MPH:.0f} mph"
    return ""

  def step(self, frame, inp, gas_recent=False):
    """The button to send this frame (only ever "SET_PLUS") or None."""
    if inp.set_mph != self.set_last:
      self.set_last, self.set_since = inp.set_mph, frame
    if not self.active:
      return None
    if inp.driver_button:
      self._finish(frame, "driver", inp.set_mph)
      return None
    if not inp.cruise_on or not inp.cruise_available:
      self._finish(frame, "cruise off", inp.set_mph)
      return None
    if inp.set_mph is None:
      return None
    if self.line is None:
      # restarted without the driver's set speed: the car's speed now rounded up to whole mph (the dash never goes above that, nor
      # ever above a known reference), once the speed is above MIN_MPH
      if inp.mph < MIN_MPH:
        return None
      self.line = int(math.ceil(inp.mph - 1e-6))
      self._ev("exit_line", frame, line=self.line, ref_unknown=True)
    cap = self.line if self.ref is None else min(self.line, self.ref)
    self.line1 = f"RAISING SET SPEED: {inp.set_mph} -> {cap} mph"
    self.line2 = ("your set speed is unknown: to the car's speed" if self.ref_unknown else f"curve slowing stopped: back to {cap}")
    if self.tap is not None:
      return self._send(frame, inp)
    if self.tap_to is not None:
      if inp.set_mph == self.tap_to:
        self.tap_to = self.tap_first = None
      elif inp.set_mph != self.tap_to - 1:
        self._finish(frame, "changed", inp.set_mph)        # moved in a way it did not ask: left to the driver
        return None
      elif (frame - self.tap_first) * DT >= ACCEPT_WAIT_S:
        self._finish(frame, "failed", inp.set_mph)         # "set speed did not respond": nothing more the code can do (I4 alert)
        return None
      else:
        return None
    if inp.set_mph >= cap:
      self._finish(frame, "done", inp.set_mph)
      return None
    blocked = self._blocked(inp, gas_recent)
    if not blocked and ((frame - self.armed_f) * DT < EXIT_START_S - 1e-9 or
                        (frame - (self.set_since if self.set_since is not None else frame)) * DT < EXIT_START_S - 1e-9):
      blocked = "the set speed settling"
    if blocked:
      self.wait_since = frame if self.wait_since is None else self.wait_since
      self.line2 = ("set speed unknown: " if self.ref_unknown else "") + f"waiting: {blocked}"
      if (frame - self.wait_since) * DT >= EXIT_WAIT_MAX_S:
        self._finish(frame, "gave up", inp.set_mph)
      return None
    self.wait_since = None
    self.tap = Tap("SET_PLUS", PRESSED_FRAMES, frame, inp.counter, inp.set_mph + 1)
    self.tap_to, self.tap_first = inp.set_mph + 1, None
    return self._send(frame, inp)

  def _send(self, frame, inp):
    tap = self.tap
    out = None
    if tap.due(frame, inp.counter) and (inp.set_mph == tap.target or self._blocked(inp, False)):
      tap.n = len(tap.frames)
    elif tap.due(frame, inp.counter):
      if tap.first is None:
        if not self.guard.begin(frame, inp.mono):
          tap.counter0, tap.requested = inp.counter, frame
          return None
        tap.first = frame
        self.tap_first = frame
      if self.guard.allow(frame, inp.mono):
        tap.frames.append((int(frame), inp.counter, (inp.counter + 1) % 16))
        out = tap.button
      else:
        tap.n = len(tap.frames)
    if tap.done:
      self.guard.end()
      self._ev("exit_tap", frame, frames=[list(f) for f in tap.frames], set_before=self.tap_to - 1, set_now=inp.set_mph)
      if not tap.frames:
        self.tap_to = self.tap_first = None
      elif self.tap_first is None:
        self.tap_first = frame
      self.tap = None
    return out


def run_frame(cv, ex, frame, inp):
  """The car controller's per-frame composition (pure; carcontroller.curve_step uses it): the Curve's step, then the safe exit (which
  reads the Curve's events of this frame before the caller logs and clears them). cv None (after an internal error): the exit alone.
  -> the button to send (the Curve's first; the exit only presses when the Curve does not)."""
  button = None
  if cv is not None:
    button = cv.step(frame, inp)
    if ex is not None:
      ex.hold_from(cv, inp)
      ex.observe(frame, cv, inp)
  if ex is None:
    return button
  b2 = ex.step(frame, inp, gas_recent=bool(cv.gas_hold) if cv is not None else False)
  return button if button is not None else b2


ALERT_FRESH_S = 1.0               # the planner writes every 0.2 s: a warning older than this is dropped (no flicker, no re-chime)


DELIVER_FRESH_S = 1.0             # stage 3's record is rewritten every 0.2 s while it alerts: older than this, it is gone


def read_deliver_alert(params_memory, now):
  """(stage 3's need mph, its monotonic time) from the car controller's TiCurveDeliver while it alerts and is fresh, or (None, 0).
  The car controller writes it only while curve slowing is on (TiCurveOn). `now` is time.monotonic()."""
  import json
  try:
    raw = params_memory.get(PARAM_DELIVER)
    d = json.loads(raw) if raw else None
    if not isinstance(d, dict) or not d.get("need"):
      return None, 0.0
    if str(d.get("why") or "limit") != "limit":
      return None, 0.0                 # review I1: an interceptor dropout / assist off is never "corner needs N mph" (its own text)
    m = float(d.get("m", 0.0))
    if now - m > DELIVER_FRESH_S:
      return None, 0.0
    return int(d["need"]), m
  except Exception:
    return None, 0.0


# ---- fixed on-road texts carried by frogpilotPlan.steerAdvisorySpeed (review I1 / I4, 2026-11-24) ---------------------------------
# The on-road alert path (frogpilot_planner -> frogpilotPlan.steerAdvisorySpeed -> events.steer_authority_advisory_alert) carries one
# float: > 0 the model's corner advisory (m/s), < 0 curve slowing's corner warning (m/s). A value at or below -ALERT_CODE_UNIT carries
# a FIXED text instead: -(ALERT_CODE_UNIT x code + 1000 x a + b), a and b two whole numbers under 1000 (exact in float32 up to code 15).
# No schema change; events.py decodes it (its constants pinned to these by test_mazda_ti_alert_text.py).
ALERT_CODE_UNIT = 1_000_000
CODE_ASSIST_OFF = 1               # stage 3, the interceptor OFF in a bend
CODE_ASSIST_STUCK = 2             # ... with its stuck-request code (0x11)
CODE_DROPOUT = 3                  # stage 3, the steering command saturated with no effect (lane-keep path blocked)
CODE_SET_LEFT = 4                 # I4: curve slowing (or a test) stopped and the cruise set speed is lower than the driver set it


def alert_code_value(code, a=0, b=0):
  return -float(ALERT_CODE_UNIT * int(code) + 1000 * max(0, min(999, int(a))) + max(0, min(999, int(b))))


def deliver_code(why):
  """The fixed-text code of a stage-3 alert reason other than the limit path, or None."""
  if not why or why == "limit":
    return None
  if why == "dropout":
    return CODE_DROPOUT
  return CODE_ASSIST_STUCK if "stuck" in why else CODE_ASSIST_OFF


def read_curve_code(params_memory, now):
  """(steerAdvisorySpeed value of a fixed-text curve alert, its monotonic time) or (None, 0): stage 3's dropout / assist off
  (TiCurveDeliver, fresh). I4's set-speed-left alert is added in its own commit."""
  import json
  try:
    raw = params_memory.get(PARAM_DELIVER)
    d = json.loads(raw) if raw else None
    if isinstance(d, dict) and d.get("need"):
      code = deliver_code(str(d.get("why") or "limit"))
      m = float(d.get("m", 0.0))
      if code is not None and now - m <= DELIVER_FRESH_S:
        return alert_code_value(code), m
  except Exception:
    pass
  return None, 0.0


def curve_alert_mph(params_memory, now):
  """The Level 1 curve warning's need (mph) and time: the planner's pre-bend warning or stage 3's, the LOWER need wins; (None, 0)."""
  a, ta = read_curve_alert(params_memory, now)
  b, tb = read_deliver_alert(params_memory, now)
  if b is not None and (a is None or b < a):
    return b, tb
  return a, ta


def read_curve_alert(params_memory, now):
  """(corner speed mph to warn about, its monotonic time) from the PLANNER's output (TiCurvePlan, written at 5 Hz) while curve slowing
  is on, or (None, 0). `now` is time.monotonic()."""
  import json
  try:
    raw = params_memory.get(PARAM_PLAN)
    d = json.loads(raw) if raw else None
    if not isinstance(d, dict) or d.get("mode") != "act" or not d.get("alert"):
      return None, 0.0
    m = float(d.get("m", 0.0))
    if now - m > ALERT_FRESH_S:
      return None, 0.0
    return int(d["alert"]), m
  except Exception:
    return None, 0.0
