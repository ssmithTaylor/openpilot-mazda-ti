"""Cruise set-speed check (ti_cruise_check.py): a one-off on-car check of whether the stock radar cruise accepts set-speed button
presses (CRZ_BTNS 0x09D SET_M / SET_P) sent by the comma. It answers one question: can openpilot change the cruise set speed?

Started ONLY by its own control on the Tests pages (param TiCruiseCheck, tmpfs, written by the page): never part of the plan
START runs, never by anything else, off after every controls restart (the car controller clears the trigger and the status when
it starts, and ignores a trigger written before it started), and refused while the drive plan, the A/B switch or any test is on.
One press arms one run; it never repeats or resumes by itself.

A run (the car controller calls `step` every 10 ms frame, and sends the button this returns):
  waiting    until every precondition holds (openpilot engaged, stock cruise active, 30-70 mph and within 1.5 mph of the set speed
             for 3 s, set speed at least 25 mph, no brake or gas, no lead being followed, steering not near its limit, cruise
             available, no cruise button pressed by the driver); gives up after START_WAIT_S with the reason
  rounds     ROUNDS times: ONE set-minus tap; wait up to ACCEPT_WAIT_S for the set speed to read exactly one lower; then ONE
             set-plus tap and wait for it to return to the original; pause PAUSE_S. Recipe A (2 pressed frames per tap); only if
             none of recipe A's three rounds was accepted, three more rounds with recipe B (3 pressed frames per tap)
  restore    after the last round or an abort, while cruise is still active and nothing says the driver took over the set speed: at
             most RESTORE_TAPS single taps toward the original, then stop regardless
  abort      at once on brake or gas, disengagement, cruise cancelled / off / unavailable, openpilot asking for cancel or resume, a
             set-speed change that is not the step asked for (a jump to a multiple of 5 = read as a hold, or a change the check did
             not ask for), a driver's cruise button, the speed leaving the band, any precondition lost, a stop from the screen, the
             plan / A/B / a test switched on, or RUN_LIMIT_S elapsed in a recipe
It never sends CANCEL or RESUME, and sends nothing on a frame where openpilot asks for either (the existing cancel / resume logic of
the car controller is untouched and always wins).

The tap recipe (owner's study of the recorded button frames, E:/openpilot-mazda-ti-data/cruise-buttons-20261122/RESULTS.txt):
  - the car's button module sends 0x09D at 10 Hz, counter +1 per frame; real taps are mostly 2 pressed frames; ONE pressed frame was
    enough for the set speed to move (0.02-0.07 s after the first pressed frame)
  - the car reads a press as a HOLD at 0.52-0.57 s after the first pressed frame (then jumps to a multiple of 5); the longest press
    without a hold step was 5 frames (0.31 s)
  - every comma frame recorded so far used counter = the car's last counter + 1 (605 of 606), which is what create_button_cmd does
  So: a tap starts on the first control frame after a new car button frame (a counter change), sends PRESSED_FRAMES[recipe] pressed
  frames FRAME_SPACING frames (50 ms) apart, each with the car's last counter + 1, and is capped in code (PressGuard) at
  MAX_PRESSED_FRAMES frames and MAX_TAP_SPAN_S from first to last pressed frame (well under the 0.3 s cap), with taps at least
  MIN_TAP_GAP_S apart. Acceptance of injected SET frames has never been observed: that is what this check finds out.

Everything here is a pure function of the inputs and the frame number (no clock, no params): the car controller does the I/O
(carcontroller.py `cruise_check_step`) and the tests drive it frame by frame (selfdrive/car/tests/test_mazda_ti_cruise_check.py).
Every step is logged (`events`) in the `ti_cruise` log channel: frames sent with their counters, set speed before and after, timings,
abort reason, result.
"""
from typing import NamedTuple, Optional

DT = 0.01                       # the car controller's frame
PARAM_TRIGGER = "TiCruiseCheck"            # tmpfs, the page: {"cmd": "run" | "stop", "id": text, "t": wall seconds}
PARAM_STATUS = "TiCruiseCheckStatus"       # tmpfs, the car controller: what the page shows (`status`), on change and every second
STATUS_EVERY = 100              # frames between status writes when nothing changed (the page calls a status older than 3 s stale)

# ---- the tap recipe (see the module docstring for where each number comes from) --------------------------------------------------
PRESSED_FRAMES = {"A": 2, "B": 3}   # pressed frames per tap: A = a real tap; B = the fallback (still well under the hold)
FRAME_SPACING = 5               # control frames between pressed frames: 50 ms (the module itself sends at 10 Hz)
MAX_PRESSED_FRAMES = 3          # hard cap per tap, enforced by PressGuard whatever the recipe says
MAX_TAP_SPAN_S = 0.15           # hard cap, first to last pressed frame of a tap (a hold is read at 0.52 s; 0.31 s never was one)
MIN_TAP_GAP_S = 0.25            # last pressed frame of one tap to the first of the next
PHASE_WAIT_S = 0.2              # a tap waits at most this long for a new car button frame to start after (else it starts anyway)

# ---- the sequence ----------------------------------------------------------------------------------------------------------------
ROUNDS = 3
ACCEPT_WAIT_S = 1.0             # from the first pressed frame to the set speed reading the expected value
PAUSE_S = 3.0                   # after each round
RUN_LIMIT_S = 20.0              # per recipe, from its first tap: abort when exceeded
START_WAIT_S = 60.0             # waiting for the preconditions before the first tap; then the run is given up
RESTORE_TAPS = 3
SPEED_MIN_MPH, SPEED_MAX_MPH = 30.0, 70.0
STEADY_TOL_MPH, STEADY_S = 1.5, 3.0      # within this of the set speed for this long before each round
RUN_TOL_MPH = 3.0               # during a round: the speed may not leave the original set speed by more than this
SET_MIN_MPH = 25                # a 1 mph drop stays far from the car's 19 mph floor
STEER_LIMIT_SHARE = 0.9         # |TI command| / TI_STEER_MAX at or above this is "near its limit"
TRIGGER_MAX_AGE_S = 15.0        # a trigger older than this when first seen is ignored (a stale write)

STATES = ("idle", "refused", "waiting", "running", "restoring", "finished", "aborted")
ACTIVE_STATES = ("waiting", "running", "restoring")
BUTTONS = ("SET_MINUS", "SET_PLUS")


class Inputs(NamedTuple):
  """One frame, as the car controller reads it. None means unknown (treated as a precondition not met)."""
  engaged: bool = False           # openpilot engaged (CarControl.enabled)
  cruise_on: bool = False         # stock cruise active (carState.cruiseState.enabled)
  cruise_available: bool = False  # cruise main on and no fault (carState.cruiseState.available)
  mph: float = 0.0
  set_mph: Optional[int] = None   # the set speed, whole mph (carState.cruiseState.speed)
  brake: bool = False
  gas: bool = False
  lead: Optional[bool] = None     # the stock cruise is following a lead (CRZ_CTRL 0x21C RADAR_HAS_LEAD, bit 23)
  steer_share: float = 0.0        # |TI command| / TI_STEER_MAX
  driver_button: Optional[bool] = None   # the car's own 0x09D frame shows a cruise button pressed (our frames are never read back)
  counter: Optional[int] = None   # the car's last 0x09D counter
  cancel_req: bool = False        # openpilot asks for cruise cancel this frame (CarControl.cruiseControl.cancel)
  resume_req: bool = False
  others_on: str = ""             # why another mode excludes the check ("" when nothing else is on)
  wall: float = 0.0               # wall seconds (only to compare with the trigger's stamp)


class Trigger(NamedTuple):
  cmd: str                        # "run" or "stop"
  id: str
  t: float


def parse_trigger(raw):
  """The page's trigger, or None for anything unreadable."""
  import json
  try:
    if isinstance(raw, (bytes, bytearray)):
      raw = raw.decode("utf-8", "replace")
    d = json.loads(raw) if raw else None
    if not isinstance(d, dict) or d.get("cmd") not in ("run", "stop") or not isinstance(d.get("id"), str):
      return None
    t = d.get("t")
    return Trigger(d["cmd"], d["id"][:40], float(t) if isinstance(t, (int, float)) else 0.0)
  except Exception:
    return None


def precondition(inp, orig=None, steady_s=None, need_steady=False):
  """Why the check cannot press now, in plain words; "" when every precondition holds. `orig`: the original set speed during a run
  (the speed is compared with it); `steady_s`: how long the speed has been within STEADY_TOL_MPH of the set speed."""
  if inp.others_on:
    return inp.others_on
  if inp.cancel_req or inp.resume_req:
    return "openpilot is using the cruise buttons"
  if not inp.engaged:
    return "openpilot not engaged"
  if not inp.cruise_available:
    return "cruise unavailable or faulted"
  if not inp.cruise_on:
    return "stock cruise not active"
  if inp.brake:
    return "brake pressed"
  if inp.gas:
    return "gas pressed"
  if inp.driver_button is None:
    return "no cruise button frames from the car"
  if inp.driver_button:
    return "a cruise button is pressed"
  if inp.counter is None:
    return "no cruise button frames from the car"
  if inp.set_mph is None:
    return "set speed unreadable"
  if orig is None and inp.set_mph < SET_MIN_MPH:
    return f"set speed below {SET_MIN_MPH} mph"
  if inp.lead is None:
    return "lead flag unreadable"
  if inp.lead:
    return "cruise is following a car"
  if not (SPEED_MIN_MPH <= inp.mph <= SPEED_MAX_MPH):
    return f"speed outside {SPEED_MIN_MPH:.0f}-{SPEED_MAX_MPH:.0f} mph"
  if orig is not None and abs(inp.mph - orig) > RUN_TOL_MPH:
    return "speed moved away from the set speed"
  if inp.steer_share >= STEER_LIMIT_SHARE:
    return "steering near its limit"
  if need_steady and (steady_s is None or steady_s < STEADY_S):
    return "speed not steady at the set speed"
  return ""


class PressGuard:
  """The hard cap on a tap, independent of the recipe: at most MAX_PRESSED_FRAMES pressed frames, at most MAX_TAP_SPAN_S from the
  first to the last, and a new tap only MIN_TAP_GAP_S after the last pressed frame of the previous one. `allow` is called for every
  pressed frame before it is sent; a refused frame is not sent."""

  def __init__(self):
    self.tap_first = None
    self.tap_count = 0
    self.last_frame = None

  def begin(self, frame):
    if self.last_frame is not None and (frame - self.last_frame) * DT < MIN_TAP_GAP_S - 1e-9:
      return False
    self.tap_first, self.tap_count = frame, 0
    return True

  def allow(self, frame):
    if self.tap_first is None:
      return False
    if self.tap_count >= MAX_PRESSED_FRAMES or (frame - self.tap_first) * DT > MAX_TAP_SPAN_S + 1e-9:
      return False
    self.tap_count += 1
    self.last_frame = frame
    return True

  def end(self):
    self.tap_first = None


class Tap:
  """One tap being sent: waits for a new car button frame (counter change) for at most PHASE_WAIT_S, then sends `n` pressed frames
  FRAME_SPACING apart. `frames`: [(frame, car counter, counter sent)]."""

  def __init__(self, button, n, frame, counter):
    self.button, self.n = button, n
    self.requested = frame
    self.counter0 = counter
    self.first = None
    self.frames = []

  def due(self, frame, counter):
    """True when a pressed frame is to be sent on this frame."""
    if self.first is None:
      if counter != self.counter0 or (frame - self.requested) * DT >= PHASE_WAIT_S - 1e-9:
        return True
      return False
    return len(self.frames) < self.n and (frame - self.first) % FRAME_SPACING == 0

  @property
  def done(self):
    return len(self.frames) >= self.n


class CruiseCheck:
  """The sequence. `step(frame, inp, trigger)` returns the button to send this frame (None, "SET_MINUS" or "SET_PLUS"); the car
  controller sends it with the car's last counter (create_button_cmd adds one). `status()` is what the page shows; `events` collects
  the log events (the caller takes and clears them)."""

  def __init__(self):
    self.state = "idle"
    self.why = ""
    self.run_id = ""
    self.consumed = set()
    self.guard = PressGuard()
    self.events = []
    self._reset_run()

  def _reset_run(self):
    self.phase, self.round = "A", 0
    self.sub = ""                     # tap_minus wait_minus tap_plus wait_plus pause (running); tap wait (restoring)
    self.results = {"A": [], "B": []}
    self.orig = None
    self.armed_at = None
    self.phase_t0 = None
    self.sub_t0 = None
    self.tap = None
    self.hit = None                   # the frame the expected set speed was first read, during the tap
    self.cur = None                   # the round being run
    self.steady_since = None
    self.restore = {"taps": 0, "ok": None, "why": ""}
    self.restore_target = None
    self.final = ""                   # "finished" / "aborted" once the rounds are over (kept through restoring)
    self.abort_why = ""
    self.set_now = None
    self.lat = []

  # ---- events ------------------------------------------------------------------------------------------------------------------
  def _ev(self, kind, frame, **kw):
    self.events.append({"kind": kind, "frame": int(frame), "id": self.run_id, **kw})

  # ---- the run ---------------------------------------------------------------------------------------------------------------
  def step(self, frame, inp, trigger=None):
    self.set_now = inp.set_mph
    if trigger is not None and trigger.id not in self.consumed:
      self.consumed.add(trigger.id)
      self._trigger(frame, inp, trigger)
    if inp.set_mph is not None and abs(inp.mph - inp.set_mph) <= STEADY_TOL_MPH:
      if self.steady_since is None:
        self.steady_since = frame
    else:
      self.steady_since = None
    steady_s = None if self.steady_since is None else (frame - self.steady_since) * DT
    if self.state == "waiting":
      return self._waiting(frame, inp, steady_s)
    if self.state == "running":
      return self._running(frame, inp, steady_s)
    if self.state == "restoring":
      return self._restoring(frame, inp)
    return None

  def _trigger(self, frame, inp, trig):
    if trig.cmd == "stop":
      if self.state in ACTIVE_STATES:
        self._ev("stop_pressed", frame)
        if self.state == "restoring":
          self._end_restore(frame, "stopped on the screen")
        else:
          self._abort(frame, inp, "stopped on the screen")
      return
    if self.state in ACTIVE_STATES:
      return                          # one press arms one run: a second press while it runs does nothing
    if inp.wall and trig.t and inp.wall - trig.t > TRIGGER_MAX_AGE_S:
      self._ev("trigger_stale", frame, age_s=round(inp.wall - trig.t, 1))
      return
    self._reset_run()
    self.run_id = trig.id
    if inp.others_on:
      self.state, self.why = "refused", inp.others_on
      self._ev("refused", frame, why=self.why)
      return
    self.state, self.why = "waiting", ""
    self.armed_at = frame
    self._ev("armed", frame, set_mph=inp.set_mph, mph=round(inp.mph, 1))

  def _waiting(self, frame, inp, steady_s):
    why = precondition(inp, steady_s=steady_s, need_steady=True)
    if why != self.why:
      self._ev("waiting", frame, why=why, set_mph=inp.set_mph, mph=round(inp.mph, 1))
    self.why = why
    if why:
      if (frame - self.armed_at) * DT >= START_WAIT_S:
        self.final = "aborted"
        self.abort_why = f"not started: {why}"
        self.state, self.why = "aborted", self.abort_why
        self._ev("aborted", frame, why=self.abort_why, taps=0)
      return None
    self.orig = int(inp.set_mph)
    self.state = "running"
    self._ev("start", frame, orig=self.orig, mph=round(inp.mph, 1))
    self._begin_phase(frame, "A")
    return self._running(frame, inp, steady_s)

  def _begin_phase(self, frame, phase):
    self.phase, self.round = phase, 0
    self.phase_t0 = frame
    self._begin_round(frame)

  def _begin_round(self, frame):
    self.round += 1
    self.cur = {"round": self.round, "recipe": self.phase, "minus": None, "minus_ms": None, "plus": None, "plus_ms": None}
    self.sub, self.sub_t0 = "pre", frame
    self.tap = None

  def _expected(self):
    """The set speeds that are not a surprise in the current sub-step."""
    o = self.orig
    if self.sub in ("tap_minus", "wait_minus", "tap_plus", "wait_plus"):
      return (o, o - 1)
    return (o,)

  def _running(self, frame, inp, steady_s):
    # the abort conditions, every frame
    why = precondition(inp, orig=self.orig)
    if not why and inp.set_mph not in self._expected():
      why = f"set speed changed to {inp.set_mph} mph (expected {' or '.join(str(x) for x in self._expected())})"
    if not why and (frame - self.phase_t0) * DT > RUN_LIMIT_S:
      why = f"time limit {RUN_LIMIT_S:.0f} s reached"
    if why:
      self._abort(frame, inp, why)
      return self._restoring(frame, inp) if self.state == "restoring" else None
    t_sub = (frame - self.sub_t0) * DT
    if self.sub == "pre":
      # before each round: the steady-speed precondition (it waits here, inside the recipe's time limit)
      w = precondition(inp, orig=self.orig, steady_s=steady_s, need_steady=True)
      self.why = w
      if w:
        return None
      return self._start_tap(frame, inp, "SET_MINUS", "tap_minus")
    if self.sub in ("tap_minus", "tap_plus"):
      target = self.orig - 1 if self.sub == "tap_minus" else self.orig
      if inp.set_mph == target and self.hit is None:
        self.hit = frame                # read before the tap's last frame (one pressed frame is often enough)
      return self._send(frame, inp)
    if self.sub == "wait_minus":
      if inp.set_mph == self.orig - 1:
        self._accepted(frame, inp, "minus")
        return self._start_tap(frame, inp, "SET_PLUS", "tap_plus")
      if self._since_first(frame) >= ACCEPT_WAIT_S:
        self.cur["minus"] = False
        self._ev("not_accepted", frame, button="SET_MINUS", recipe=self.phase, round=self.round, set_mph=inp.set_mph)
        return self._end_round(frame, inp)
      return None
    if self.sub == "wait_plus":
      if inp.set_mph == self.orig:
        self._accepted(frame, inp, "plus")
        return self._end_round(frame, inp)
      if self._since_first(frame) >= ACCEPT_WAIT_S:
        self.cur["plus"] = False
        self._ev("not_accepted", frame, button="SET_PLUS", recipe=self.phase, round=self.round, set_mph=inp.set_mph)
        self.results[self.phase].append(self.cur)
        self.cur = None
        self._abort(frame, inp, "set-plus not accepted: set speed left 1 mph low")
        return self._restoring(frame, inp) if self.state == "restoring" else None
      return None
    if self.sub == "pause":
      if t_sub >= PAUSE_S:
        if self.round >= ROUNDS:
          return self._end_phase(frame, inp)
        self._begin_round(frame)
      return None
    return None

  def _since_first(self, frame):
    return (frame - self.tap.first) * DT if self.tap is not None and self.tap.first is not None else 0.0

  def _start_tap(self, frame, inp, button, sub):
    self.sub, self.sub_t0 = sub, frame
    self.tap = Tap(button, PRESSED_FRAMES[self.phase if self.state == "running" else "A"], frame, inp.counter)
    self.hit = None
    self._tap_set_before = inp.set_mph
    return self._send(frame, inp)

  def _send(self, frame, inp):
    """A pressed frame of the tap in progress, or None; moves to the wait once the tap is complete."""
    tap = self.tap
    out = None
    if tap.due(frame, inp.counter):
      if tap.first is None:
        if not self.guard.begin(frame):
          return None                 # too soon after the previous tap: wait (the gap is far shorter than any wait here)
        tap.first = frame
      if self.guard.allow(frame):
        tap.frames.append((int(frame), inp.counter, (inp.counter + 1) % 16))
        out = tap.button
      else:
        tap.n = len(tap.frames)       # the cap ends the tap: never a longer press
    if tap.done:
      self.guard.end()
      self._ev("tap", frame, button=tap.button, recipe=self.phase if self.state == "running" else "restore",
               round=self.round, frames=[list(f) for f in tap.frames], requested=int(tap.requested),
               set_before=self._tap_set_before, set_now=inp.set_mph)
      if self.state == "running":
        self.sub = "wait_minus" if tap.button == "SET_MINUS" else "wait_plus"
      else:
        self.sub = "wait"
    return out

  def _accepted(self, frame, inp, which):
    at = self.hit if self.hit is not None else frame
    ms = int(round((at - self.tap.first) * DT * 1000)) if self.tap is not None and self.tap.first is not None else 0
    self.cur[which], self.cur[which + "_ms"] = True, ms
    self._ev("accepted", frame, button="SET_MINUS" if which == "minus" else "SET_PLUS", recipe=self.phase, round=self.round,
             latency_ms=ms, set_mph=inp.set_mph)

  def _end_round(self, frame, inp):
    self.results[self.phase].append(self.cur)
    self._ev("round", frame, **self.cur)
    self.cur = None
    if self.round >= ROUNDS:
      return self._end_phase(frame, inp)
    self.sub, self.sub_t0 = "pause", frame
    return None

  def _end_phase(self, frame, inp):
    n = self.n_ok(self.phase)
    if self.phase == "A" and n == 0 and len(self.results["A"]) == ROUNDS:
      self._ev("fallback", frame, why="recipe A: 0 of 3 accepted; trying recipe B (3 pressed frames)")
      self._begin_phase(frame, "B")
      self.sub, self.sub_t0 = "pause", frame   # the pause between the recipes, then recipe B's first round
      self.round = 0
      return None
    self.final, self.abort_why = "finished", ""
    self._ev("rounds_done", frame, **self._summary())
    self._to_restore(frame, inp)
    return self._restoring(frame, inp) if self.state == "restoring" else None

  def n_ok(self, phase):
    return sum(1 for r in self.results[phase] if r.get("minus") and r.get("plus"))

  def _summary(self):
    return {"recipe_a": f"{self.n_ok('A')} of {len(self.results['A'])}", "recipe_b": f"{self.n_ok('B')} of {len(self.results['B'])}"
            if self.results["B"] else "", "orig": self.orig, "result": self.result()}

  def result(self):
    """YES / PARTLY / NO from the rounds run (only once the rounds are over, else "")."""
    if self.final != "finished":
      return ""
    best = max(self.n_ok("A"), self.n_ok("B"))
    return "YES" if best == ROUNDS else ("NO" if best == 0 else "PARTLY")

  def _abort(self, frame, inp, why):
    if self.cur is not None and self.cur.get("minus") is not None:
      self.results[self.phase].append(self.cur)
    self.cur = None
    if self.tap is not None:
      self.guard.end()
      if self.tap.frames and self.sub in ("tap_minus", "tap_plus", "wait"):
        self._ev("tap", frame, button=self.tap.button, recipe=self.phase, round=self.round, frames=[list(f) for f in self.tap.frames],
                 requested=int(self.tap.requested), set_before=getattr(self, "_tap_set_before", None), set_now=inp.set_mph, cut=True)
    self.tap = None
    self.final, self.abort_why = "aborted", why
    self._ev("aborted", frame, why=why, set_mph=inp.set_mph, **self._summary())
    self._to_restore(frame, inp)

  # ---- restore -------------------------------------------------------------------------------------------------------------------
  def _restore_blocked(self, inp):
    """Why no corrective tap may be sent now ("" when one may)."""
    # the driver's own button (now, or the reason of the abort): the set speed is theirs, never pressed against
    if inp.driver_button or self.abort_why == "a cruise button is pressed":
      return "the driver is using the cruise buttons"
    if inp.cancel_req or inp.resume_req:
      return "openpilot is using the cruise buttons"
    if not inp.engaged:
      return "openpilot not engaged"
    if not (inp.cruise_on and inp.cruise_available):
      return "stock cruise not active"
    if inp.brake or inp.gas:
      return "brake or gas pressed"
    if inp.counter is None or inp.driver_button is None or inp.set_mph is None:
      return "cruise frames unreadable"
    # the speed band, a lead, the steering limit or another mode switched on do not stop the check putting back its own change
    return ""

  def _to_restore(self, frame, inp):
    if self.orig is None or inp.set_mph is None or inp.set_mph == self.orig or not inp.cruise_on:
      self._finish(frame, "nothing to restore" if self.orig is not None else "")
      return
    blocked = self._restore_blocked(inp)
    if blocked:
      self.restore.update(ok=False, why=f"not restored: {blocked}")
      self._finish(frame, f"not restored: {blocked}")
      return
    self.state = "restoring"
    self.sub, self.sub_t0, self.tap = "next", frame, None
    self._ev("restore_start", frame, set_mph=inp.set_mph, orig=self.orig)

  def _restoring(self, frame, inp):
    blocked = self._restore_blocked(inp)
    if blocked:
      self._end_restore(frame, f"not restored: {blocked}")
      return None
    if self.sub == "next":
      if inp.set_mph == self.orig:
        self._end_restore(frame, "restored")
        return None
      if self.restore["taps"] >= RESTORE_TAPS:
        self._end_restore(frame, f"not restored after {RESTORE_TAPS} taps")
        return None
      self.restore["taps"] += 1
      self.restore_target = inp.set_mph + (1 if inp.set_mph < self.orig else -1)
      self._restore_from = inp.set_mph
      return self._start_tap(frame, inp, "SET_PLUS" if inp.set_mph < self.orig else "SET_MINUS", "tap")
    if self.sub == "tap":
      out = self._send(frame, inp)
      if inp.set_mph not in (self._restore_from, self.restore_target):
        self._end_restore(frame, f"not restored: set speed changed to {inp.set_mph} mph")
      return out
    if self.sub == "wait":
      if inp.set_mph == self.restore_target:
        self._ev("restore_step", frame, set_mph=inp.set_mph, latency_ms=int(round(self._since_first(frame) * 1000)))
        self.sub = "next"
        return self._restoring(frame, inp)
      if inp.set_mph != self._restore_from:
        self._end_restore(frame, f"not restored: set speed changed to {inp.set_mph} mph")
        return None
      if self._since_first(frame) >= ACCEPT_WAIT_S:
        self._ev("restore_step", frame, set_mph=inp.set_mph, accepted=False)
        self.sub = "next"
      return None
    return None

  def _end_restore(self, frame, note):
    if self.tap is not None:
      self.guard.end()
      self.tap = None
    self.restore.update(ok=(note == "restored"), why=note)
    self._ev("restore_end", frame, note=note, taps=self.restore["taps"])
    self._finish(frame, note)

  def _finish(self, frame, note):
    self.state = self.final or "finished"
    self.why = self.abort_why if self.state == "aborted" else ""
    self.restore["why"] = self.restore["why"] or note
    self.sub, self.tap = "", None
    self._ev("result", frame, state=self.state, why=self.why, restore=dict(self.restore), set_mph=self.set_now, **self._summary())

  # ---- what the page shows ---------------------------------------------------------------------------------------------------------
  def status(self, wall):
    """{t, v, id, state, why, phase, round, rounds, a: [..], b: [..], orig, set, result, restore}. `state` is the controller's own
    state; "running" only while the rounds are being run (a tap, a wait or a pause between them)."""
    def rows(phase):
      return [[1 if r.get("minus") else 0, 1 if r.get("plus") else 0, r.get("minus_ms") or 0, r.get("plus_ms") or 0]
              for r in self.results[phase]]
    return {"t": round(wall, 1), "v": 1, "id": self.run_id, "state": self.state, "why": self.why, "phase": self.phase,
            "round": self.round, "rounds": ROUNDS, "a": rows("A"), "b": rows("B"), "orig": self.orig,
            "set": self.set_now, "result": self.result(), "restore": self.restore.get("why", "")}

  @property
  def active(self):
    return self.state in ACTIVE_STATES
