"""Cruise button tests (ti_button_tests.py): two driver-armed on-car tests of how the stock cruise reads set-speed presses SENT BY THE
COMMA, to choose between a held button and faster taps for curve slowing (owner, 2026-11-24: "to see which one's best").

  HOLD TEST  one SET- hold, then (once the dash has settled) one SET+ hold back up. Each is one pressed frame per car button frame
             (counter + 1), at most HOLD_TEST_S of pressed time (HoldGuard and TestHoldGuard: frames AND real time), released by
             simply stopping. The dash set speed is traced (50 Hz) and each direction classified: HOLD (1 mph at about 0.05 s, then
             the next multiple of 5 at about 0.55 s), TAPS (1 mph per frame, about 0.1 s apart), ONE TAP (1 mph only), NOTHING, or
             OTHER (the raw steps are shown).
  TAP TEST   for each cycle in CYCLES (0.3, 0.2, 0.1 s): PRESSES presses of SET- then PRESSES of SET+, so the set speed returns to
             where it started. 0.3 / 0.2 s: the recorded recipe (up to 2 pressed frames 50 ms apart, the second only while no step
             has been read back since the first); 0.1 s: ONE pressed frame per press (two would leave only the switch's own unpressed
             frame between presses 0.1 s apart: a nearly unbroken pressed stream, i.e. a hold, not taps). Per cycle: presses sent,
             steps seen, steps per press, median time from press to step, missed / double steps. The page names the fastest cycle
             that stepped 1 mph on every press. A step of 2 mph or more (the car took the presses for a hold) ends the cycle at
             once ("JUMP: the car read it as a hold"): no further press, the faster cycles are skipped, and the set speed is put back.

The held stream and the 0.1 s presses are the same frames on the bus (one pressed frame of ours after each of the switch's frames,
its unpressed frame with our counter about 0.097 s later): only the car can say whether it reads them as a hold or as taps, and that
is exactly what the two tests ask.

Started ONLY by the driver from the page (param TiButtonTest, tmpfs, {"cmd": "hold" | "taps" | "stop", "id", "t"}); one press arms one
run; off after every controls restart (the car controller clears the trigger and the status when it starts, and ignores a trigger
written before it started); refused while the drive plan, the A/B switch, a test, the dither, curve slowing or the cruise check is on.

Conditions before the first press (and before each tap cycle): openpilot engaged with lateral control active, stock cruise on and
available, no brake, no gas, no cruise button, openpilot not asking for cancel / resume, no lead being followed (CRZ_CTRL bit 23, as
the cruise check), 30-70 mph and STEADY: within STEADY_TOL_MPH of the dash set speed for STEADY_S, set speed at least SET_MIN_MPH (the
car's 19 mph minimum + 15, so a 6 mph excursion is harmless), steering command under STEER_LIMIT_SHARE of its limit; and a set speed
that is a multiple of 5, for both tests (else a hold, or fast taps the car reads as one, would land past it on the way back up:
42 -> 37 down, 37 -> 38 -> 40 -> 41 -> 45 up). Every press of either test is sent only if neither a 1 mph step nor a jump to the
next multiple of 5 could take the set speed above the original (or below the test's lower bound). A driver's cruise button while
the test waits disarms it.

Aborts at once (and puts the set speed back, see below) on: brake, gas, a cruise button of the driver's (then NOTHING is put back: the
driver's value stands), cancel / resume from openpilot, disengagement, cruise off / unavailable, lateral control inactive, the car's
speed more than DROP_ABORT_MPH below where the test (or the tap cycle) started, a set speed the test did not expect on
UNEXPECTED_FRAMES control frames in a row (two 50 Hz readings), another mode switched on, STOP on the screen, or the run's time limit.

Expected loss of real speed (the measured braking against the set-speed gap: 0.5 mph/s at 1-2 mph, 1.3 at 4-6, 1.8 at 8-10; 0.5 s
delay + 0.5 s lag): a 1.2 s hold each way (10 mph below for about 2.6 s) loses about 2.8 mph, too close to the 3 mph abort; so each
hold is HOLD_TEST_S = 0.7 s (one 5-mph jump: 45 -> 44 -> 40, back 40 -> 41 -> 45; about 1.1 mph lost). The tap test loses about
2.1 / 1.7 / 1.3 mph at 0.3 / 0.2 / 0.1 s (5 mph below for 3.2 / 2.4 / 1.6 s). E:/openpilot-mazda-ti-data/button-tests/scratch/
excursion.py has the arithmetic.

Putting the set speed back (after the run or an abort, never after a driver's cruise button): once the dash has been still SETTLE_S,
ordinary taps (the reviewed 0.3 s recipe) toward the original, never past it, at most RESTORE_TAPS and only from at most
RESTORE_MAX_MPH away (what the test itself could have caused); each must give exactly the 1 mph step toward it within ACCEPT_WAIT_S,
else nothing more is sent. While the brake, gas, cancel / resume, a disengaged or inactive steering, a disengaged cruise or another
mode blocks a press, it waits up to RESTORE_WAIT_S. If that attempt ends short, ONE more attempt follows when the conditions allow
again within RESTORE_RETRY_S. Then it checks: the dash must read the original for VERIFY_S. Otherwise the page's large line says
"SET IT YOURSELF: NOW X MPH, WAS Y" until the driver presses a cruise button, and the log has its own `set_speed_not_restored`
event. A set speed that cannot be read (cruise cancelled / standby) is never put back and never reported as unchanged: "CRUISE OFF:
CHECK SET SPEED (WAS Y)". There is no on-road alert for this: the fork's only car-side alert path (curve slowing's, through
frogpilot_planner's steerAdvisorySpeed) carries one corner speed with fixed corner texts.

Never CANCEL or RESUME; every frame goes through mazdacan.create_button_cmd from carcontroller.py (the same sender and place as the
cruise check's taps). Pure (no clock, no params): the car controller does the I/O (`button_tests_step`), the tests drive it frame by
frame (selfdrive/car/tests/test_mazda_ti_button_tests.py). Every step is logged (`events`, channel `ti_button`).
"""
from typing import NamedTuple, Optional

from openpilot.selfdrive.car.mazda.ti_cruise_taps import (DT, MIN_TAP_GAP_S, TAP_CYCLE_SLOW_S, HoldGuard, PressGuard, Tap,
                                                          frames_for)

PARAM_TRIGGER = "TiButtonTest"            # tmpfs, the page: {"cmd": "hold" | "taps" | "stop", "id": text, "t": wall seconds}
PARAM_STATUS = "TiButtonTestStatus"       # tmpfs, the car controller: what the page shows (`status`), on change and every second
STATUS_EVERY = 100

TESTS = ("hold", "taps")
NAMES = {"hold": "HOLD TEST", "taps": "TAP TEST"}
BUTTONS = ("SET_MINUS", "SET_PLUS")
STATES = ("idle", "refused", "waiting", "running", "restoring", "finished", "aborted")
ACTIVE_STATES = ("waiting", "running", "restoring")

# ---- conditions ---------------------------------------------------------------------------------------------------------------
SPEED_MIN_MPH, SPEED_MAX_MPH = 30.0, 70.0
STEADY_TOL_MPH, STEADY_S = 2.0, 3.0       # steady: within this of the dash set speed for this long
CAR_MIN_SET_MPH = 19                      # the car's own lowest set speed
SET_MIN_MPH = CAR_MIN_SET_MPH + 15        # 34
STEER_LIMIT_SHARE = 0.9
DROP_ABORT_MPH = 3.0                      # the car's speed this far below where the test / the cycle started: abort
UNEXPECTED_FRAMES = 4                     # a set speed not expected on this many control frames in a row (two 50 Hz readings): abort
START_WAIT_S = 60.0                       # waiting for the conditions before the first press; then given up
RUN_LIMIT_S = {"hold": 20.0, "taps": 60.0}   # from the first press to the end of the restore
TRIGGER_MAX_AGE_S = 15.0
WAIT_LOG_MIN_S = 1.0

# ---- the hold test -------------------------------------------------------------------------------------------------------------
HOLD_TEST_S = 0.7                         # first to last pressed frame of one hold, at most (one 5-mph jump at 0.55 s)
HOLD_TEST_MAX_FRAMES = 8                  # one per car frame over HOLD_TEST_S, at most
HOLD_DROP_MPH = 6                         # the dash never pressed below the original - this (46 -> 45 -> 40 is the deepest hold)
HOLD_JUMP_S = (0.40, 0.80)                # where the multiple-of-5 jump of a hold is accepted (recorded 0.52-0.57 s)
HOLD_FIRST_S = 0.30                       # the first 1 mph step of a hold by this
SETTLE_S = 0.5                            # after a release: the dash still this long (and this long since our last pressed frame)

# ---- the tap test --------------------------------------------------------------------------------------------------------------
CYCLES = (0.3, 0.2, 0.1)
PRESSES = 5
STEP_LATE_S = 0.02                        # a step is counted for a press from this long after its first frame (to this long after the next)
INFLIGHT_S = 0.12                         # a press with no step read back yet may still land within this

# ---- the restore ---------------------------------------------------------------------------------------------------------------
RESTORE_TAPS = 14                         # the test's own reach: 5 + 5 mph of a cycle, or a jump past it, with room to spare
RESTORE_MAX_MPH = 12
RESTORE_WAIT_S = 10.0
RESTORE_RETRY_S = 30.0                    # one more attempt when a first one ends short and the conditions allow again within this
ACCEPT_WAIT_S = 1.0
VERIFY_S = 1.0

# ---- the page's line lengths (selfdrive/ui/qt/ti_pages_logic.h, BUTTONS_*: tested on both sides) ------------------------------------
MAX_LINE1 = 26
MAX_LINE2 = 55
MAX_RESULT = 38
MAX_DETAIL = 74


class Inputs(NamedTuple):
  """One frame, as the car controller reads it. None means unknown (a condition not met)."""
  engaged: bool = False           # openpilot engaged (CarControl.enabled)
  lat_active: bool = False        # lateral control active (CarControl.latActive)
  cruise_on: bool = False
  cruise_available: bool = False
  mph: float = 0.0
  set_mph: Optional[int] = None
  brake: bool = False
  gas: bool = False
  lead: Optional[bool] = None
  steer_share: float = 0.0
  driver_button: Optional[bool] = None   # the car's own 0x09D frame shows a cruise button (ours are never read back: carstate)
  counter: Optional[int] = None
  cancel_req: bool = False
  resume_req: bool = False
  others_on: str = ""
  wall: float = 0.0
  mono: Optional[float] = None


class Trigger(NamedTuple):
  cmd: str                        # "hold", "taps" or "stop"
  id: str
  t: float


def parse_trigger(raw):
  import json
  try:
    if isinstance(raw, (bytes, bytearray)):
      raw = raw.decode("utf-8", "replace")
    d = json.loads(raw) if raw else None
    if not isinstance(d, dict) or d.get("cmd") not in ("hold", "taps", "stop") or not isinstance(d.get("id"), str):
      return None
    t = d.get("t")
    return Trigger(d["cmd"], d["id"][:40], float(t) if isinstance(t, (int, float)) else 0.0)
  except Exception:
    return None


class TestHoldGuard(HoldGuard):
  """HoldGuard (curve slowing's cap, unchanged) plus this test's own tighter cap: at most HOLD_TEST_MAX_FRAMES frames and HOLD_TEST_S
  from the first to the last, on frames AND real time."""

  def allow(self, frame, mono=None):
    if self.first is not None:
      if self.count >= HOLD_TEST_MAX_FRAMES or (frame - self.first) * DT > HOLD_TEST_S + 1e-9:
        return False
      if mono is not None and self.first_mono is not None and mono - self.first_mono > HOLD_TEST_S + 1e-6:
        return False
    return super().allow(frame, mono)


def cut(text, n):
  text = text or ""
  return text if len(text) <= n else text[:max(0, n - 3)] + "..."


def upper_first(t):
  return t[:1].upper() + t[1:] if t else t


def next_mult(v, d):
  """The multiple of 5 strictly beyond v in direction d (+1 up, -1 down)."""
  return (v // 5 + 1) * 5 if d > 0 else ((v - 1) // 5) * 5


def precondition(inp, test, steady_s=None, need_steady=True):
  """Why the test cannot press now, in plain words ("" when every condition holds)."""
  if inp.others_on:
    return inp.others_on
  if inp.cancel_req or inp.resume_req:
    return "openpilot is using the cruise buttons"
  if not inp.engaged:
    return "openpilot not engaged"
  if not inp.lat_active:
    return "steering not active"
  if not inp.cruise_available:
    return "cruise unavailable or faulted"
  if not inp.cruise_on:
    return "stock cruise not active"
  if inp.brake:
    return "brake pressed"
  if inp.gas:
    return "gas pressed"
  if inp.driver_button is None or inp.counter is None:
    return "no cruise button frames from the car"
  if inp.driver_button:
    return "a cruise button is pressed"
  if inp.set_mph is None:
    return "set speed unreadable"
  if inp.set_mph < SET_MIN_MPH:
    return f"set speed below {SET_MIN_MPH} mph"
  if inp.set_mph % 5 != 0:
    return "needs a set speed ending in 0 or 5"
  if inp.lead is None:
    return "lead flag unreadable"
  if inp.lead:
    return "cruise is following a car"
  if not (SPEED_MIN_MPH <= inp.mph <= SPEED_MAX_MPH):
    return f"speed outside {SPEED_MIN_MPH:.0f}-{SPEED_MAX_MPH:.0f} mph"
  if inp.steer_share >= STEER_LIMIT_SHARE:
    return "steering near its limit"
  if need_steady and (steady_s is None or steady_s < STEADY_S):
    return "speed not steady at the set speed"
  return ""


def run_abort(inp):
  """Why a run in progress must stop now ("" when it may go on). The speed band, a lead and the steering limit are start conditions
  only: a lowered set speed only slows the car, and the stock cruise keeps following a lead whatever its set speed."""
  if inp.driver_button:
    return "you pressed a cruise button"
  if inp.others_on:
    return inp.others_on
  if inp.cancel_req or inp.resume_req:
    return "openpilot is using the cruise buttons"
  if not inp.engaged:
    return "openpilot not engaged"
  if not inp.lat_active:
    return "steering not active"
  if not (inp.cruise_on and inp.cruise_available):
    return "stock cruise not active"
  if inp.brake:
    return "brake pressed"
  if inp.gas:
    return "gas pressed"
  if inp.driver_button is None or inp.counter is None or inp.set_mph is None:
    return "cruise frames unreadable"
  return ""


# ---- classification ---------------------------------------------------------------------------------------------------------------
def classify_hold(s0, d, changes):
  """One direction of the hold test. changes: [(seconds after the first pressed frame, dash value)] in order (every change seen while
  held and until settled). -> (word, text): HOLD, TAPS, ONE TAP, NOTHING or OTHER."""
  path = [s0] + [v for _, v in changes]
  steps = " -> ".join(str(v) for v in (path if len(path) <= 7 else path[:4] + ["..."] + path[-2:]))
  if not changes:
    return "NOTHING", f"{s0}: no change"
  deltas = [(t, b - a) for (t, b), a in zip(changes, path)]
  if (len(changes) >= 2 and deltas[0][1] == d and deltas[0][0] <= HOLD_FIRST_S and changes[1][1] == next_mult(s0 + d, d)
      and HOLD_JUMP_S[0] <= changes[1][0] <= HOLD_JUMP_S[1] and all(dd == 5 * d for _, dd in deltas[2:])):
    times = ", ".join(f"{t:.2f}" for t, _ in changes[:3])
    return "HOLD", f"{steps} ({times} s): a hold"
  if len(changes) >= 3 and all(dd == d for _, dd in deltas):
    gaps = [b[0] - a[0] for a, b in zip(changes, changes[1:])]
    gaps.sort()
    return "TAPS", f"{steps}: taps, 1 mph per {gaps[len(gaps) // 2]:.2f} s"
  if len(changes) == 1 and deltas[0][1] == d:
    return "ONE TAP", f"{steps} ({changes[0][0]:.2f} s): one step only"
  times = ", ".join(f"{t:.2f}" for t, _ in changes[:4])
  return "OTHER", f"{steps} ({times} s): other"


def press_stats(presses, changes, d, end_frame):
  """One block of the tap test. presses: [first frame of each press]; changes: [(frame, delta)]; d: the direction. -> dict."""
  per, lat = [], []
  jump = 0
  wrong = 0
  for i, f0 in enumerate(presses):
    lo = f0 + int(round(STEP_LATE_S / DT))
    hi = (presses[i + 1] if i + 1 < len(presses) else end_frame) + int(round(STEP_LATE_S / DT))
    mine = [(f, dd) for f, dd in changes if lo <= f < hi]
    per.append(sum(dd * d for _, dd in mine))
    if mine:
      lat.append(int(round((mine[0][0] - f0) * DT * 1000)))
  for _, dd in changes:
    if abs(dd) >= 2:
      jump += 1
    if dd * d < 0:
      wrong += 1
  steps = sum(dd * d for _, dd in changes)
  lat.sort()
  return {"sent": len(presses), "steps": steps, "missed": sum(1 for p in per if p == 0), "double": sum(1 for p in per if p >= 2),
          "jump": jump, "wrong": wrong, "ms": lat[len(lat) // 2] if lat else None, "per": per}


class ButtonTests:
  """The two tests. `step(frame, inp, trigger)` returns the button to send this frame (None, "SET_MINUS" or "SET_PLUS"); the car
  controller sends it with the car's last counter (create_button_cmd adds one). `status()` is what the page shows; `events` the log."""

  def __init__(self):
    self.state, self.test, self.why = "idle", "", ""
    self.run_id = ""
    self.consumed = set()
    self.guard = PressGuard()
    self.hold_guard = TestHoldGuard()
    self.events = []
    self.steady_since = None
    self._reset_run()

  def _reset_run(self):
    self.orig = None
    self.armed_at = None
    self.run_t0 = None
    self.final, self.abort_why = "", ""
    self.sub, self.sub_t0 = "", None
    self.set_now = None
    self.set_last, self.set_changed_at = None, None
    self.unexpected = 0
    self.start_mph = None
    self.lowest_mph = None
    self.top_mph = None
    self.last_wait_ev = -10 ** 9
    self.no_restore = False
    self.jump_at = None               # the tap test: the frame a step of 2 mph or more was read (the car took it for a hold)
    self.answered = False             # a set speed left short: the driver has since pressed a cruise button
    # the hold test
    self.hold = None                  # the hold in flight: {"button","d","s0","first","frames","last_counter","changes","released"}
    self.dirs = {}                    # "down" / "up": {"word","text","s0","changes","frames"}
    # the tap test
    self.cycle_i = 0
    self.cycles = []                  # per cycle: {"cycle","down","up","clean","start_mph","lowest_mph","cut"}
    self.block = None                 # the block in flight: {"button","d","s0","presses","changes","sent","last_first"}
    self.tap = None
    self.tap_set_first = None
    # the restore
    self.restore = {"taps": 0, "ok": None, "note": "", "verified": False, "attempts": 1}
    self.restore_t0 = None
    self.restore_from = None
    self.retry_t0 = None
    self.retry_why = ""

  def _ev(self, kind, frame, **kw):
    self.events.append({"kind": kind, "frame": int(frame), "id": self.run_id, "test": self.test, **kw})

  @property
  def active(self):
    return self.state in ACTIVE_STATES

  # ---- the frame ---------------------------------------------------------------------------------------------------------------
  def step(self, frame, inp, trigger=None):
    self.set_now = inp.set_mph
    changed = inp.set_mph != self.set_last
    prev = self.set_last
    if changed:
      self.set_last, self.set_changed_at = inp.set_mph, frame
    if inp.set_mph is not None and abs(inp.mph - inp.set_mph) <= STEADY_TOL_MPH:
      if self.steady_since is None:
        self.steady_since = frame
    else:
      self.steady_since = None
    if trigger is not None and trigger.id not in self.consumed:
      self.consumed.add(trigger.id)
      self._trigger(frame, inp, trigger)
    if self.state in ("running", "restoring"):
      self.lowest_mph = inp.mph if self.lowest_mph is None else min(self.lowest_mph, inp.mph)
      if changed and prev is not None and inp.set_mph is not None:
        self._record_change(frame, prev, inp.set_mph)
        if (self.state == "running" and self.test == "taps" and self.block is not None and abs(inp.set_mph - prev) >= 2
                and self.jump_at is None):
          self.jump_at = frame        # acted on below, before any send decision of this frame
    if self.state in ("finished", "aborted") and self.not_back() and not self.answered and inp.driver_button:
      self.answered = True
      self._ev("driver_set_after", frame, set_mph=inp.set_mph, orig=self.orig)
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
      if self.active:
        self._ev("stop_pressed", frame)
        if self.state == "restoring":
          self._end_restore(frame, inp, False, "stopped on the screen")
        else:
          self._abort(frame, inp, "stopped on the screen")
      return
    if self.active:
      return                          # one press arms one run
    if inp.wall and trig.t and inp.wall - trig.t > TRIGGER_MAX_AGE_S:
      self._ev("trigger_stale", frame, age_s=round(inp.wall - trig.t, 1))
      return
    self._reset_run()
    self.run_id, self.test = trig.id, trig.cmd
    if inp.others_on:
      self.state, self.why = "refused", inp.others_on
      self._ev("refused", frame, why=self.why)
      return
    self.state, self.why = "waiting", ""
    self.armed_at = frame
    self._ev("armed", frame, set_mph=inp.set_mph, mph=round(inp.mph, 1))

  def _waiting(self, frame, inp, steady_s):
    if inp.driver_button:
      self.final, self.abort_why = "aborted", "you pressed a cruise button"   # their press wins: disarmed, nothing was pressed
      self.state, self.why = "aborted", self.abort_why
      self._ev("aborted", frame, why=self.abort_why, set_mph=inp.set_mph, mph=round(inp.mph, 1), waiting=True)
      self._ev("result", frame, **self.summary())
      return None
    why = precondition(inp, self.test, steady_s)
    if why != self.why and (frame - self.last_wait_ev) * DT >= WAIT_LOG_MIN_S:
      self.last_wait_ev = frame
      self._ev("waiting", frame, why=why, set_mph=inp.set_mph, mph=round(inp.mph, 1))
    self.why = why
    if why:
      if (frame - self.armed_at) * DT >= START_WAIT_S:
        self.final, self.abort_why = "aborted", f"not started: {why}"
        self.state, self.why = "aborted", self.abort_why
        self._ev("result", frame, **self.summary())
      return None
    self.orig = int(inp.set_mph)
    self.start_mph = self.top_mph = self.lowest_mph = inp.mph
    self.state, self.run_t0 = "running", frame
    self._ev("start", frame, orig=self.orig, mph=round(inp.mph, 1))
    if self.test == "hold":
      self._begin_hold(frame, inp, "SET_MINUS")
    else:
      self.cycle_i = 0
      self.sub, self.sub_t0 = "pre", frame
    return self._running(frame, inp, steady_s)

  # ---- what is expected of the dash ------------------------------------------------------------------------------------------------
  def _bounds(self):
    """(low, high) the dash may read now, inclusive."""
    o = self.orig
    if self.test == "hold":
      if self.sub in ("down", "down_settle"):
        return o - HOLD_DROP_MPH, o
      if self.sub in ("up", "up_settle"):
        return self.dirs["down"]["end"], o
      return o, o
    b = self.block
    if b is not None:
      n = max(1, len(b["presses"]))
      # a step per press (one more in flight), or the car's hold-like jump to the next multiple of 5 (then the cycle ends: JUMP)
      if b["d"] < 0:
        return max(o - PRESSES - 2, min(b["s0"] - 2 * n, next_mult(b["s0"], -1))), b["s0"]
      return b["s0"], min(o, max(b["s0"] + 2 * n, next_mult(b["s0"], 1)))
    return o, o

  def _record_change(self, frame, prev, v):
    if self.state == "running" and self.test == "hold" and self.hold is not None and self.hold["first"] is not None:
      self.hold["changes"].append(((frame - self.hold["first"]) * DT, v))
    if self.state == "running" and self.test == "taps" and self.block is not None:
      self.block["changes"].append((frame, v - prev))

  # ---- running -----------------------------------------------------------------------------------------------------------------
  def _running(self, frame, inp, steady_s):
    why = run_abort(inp)
    if not why and inp.mph < self.start_mph - DROP_ABORT_MPH:
      why = f"speed fell {DROP_ABORT_MPH:.0f} mph below the start"
    if not why:
      lo, hi = self._bounds()
      self.unexpected = self.unexpected + 1 if not (lo <= inp.set_mph <= hi) else 0
      if self.unexpected >= UNEXPECTED_FRAMES:
        why = f"set speed {inp.set_mph} mph not expected ({lo}-{hi})" if lo != hi else f"set speed {inp.set_mph} mph not expected ({lo})"
    if not why and (frame - self.run_t0) * DT > RUN_LIMIT_S[self.test]:
      why = f"time limit {RUN_LIMIT_S[self.test]:.0f} s reached"
    if why:
      self._abort(frame, inp, why)
      return self._restoring(frame, inp) if self.state == "restoring" else None
    if self.test == "taps" and self.jump_at is not None and not self.unexpected:
      self._taps_jumped(frame, inp)       # a jump outside the bounds is left to the unexpected-set-speed abort (nothing sent meanwhile)
      return self._restoring(frame, inp) if self.state == "restoring" else None
    if self.unexpected:
      return None                     # a set speed outside what is expected: no pressed frame while it is checked again
    return self._hold_running(frame, inp) if self.test == "hold" else self._taps_running(frame, inp, steady_s)

  def _settled(self, frame):
    last = max(x for x in (self.guard.last_frame, self.hold_guard.last_frame, self.set_changed_at, -10 ** 9) if x is not None)
    return (frame - last) * DT >= SETTLE_S - 1e-9

  # ---- the hold test -----------------------------------------------------------------------------------------------------------
  def _begin_hold(self, frame, inp, button):
    d = -1 if button == "SET_MINUS" else 1
    self.hold = {"button": button, "d": d, "s0": int(inp.set_mph), "first": None, "frames": [], "last_counter": None, "changes": [],
                 "requested": frame, "counter0": inp.counter, "released": None, "why": ""}
    self.sub, self.sub_t0 = "down" if d < 0 else "up", frame
    self._ev("hold_start", frame, button=button, set_mph=int(inp.set_mph))

  def _hold_may_press(self, v):
    """No pressed frame of the hold may move the dash past what is allowed, whatever the car makes of it: down, never below
    orig - HOLD_DROP_MPH (a frame read as a tap: 1 mph, with one more in flight; read as a hold: the next multiple of 5); up, never
    past the original."""
    h, o = self.hold, self.orig
    if h["d"] < 0:
      lo = o - HOLD_DROP_MPH
      return v - 2 >= lo and next_mult(v, -1) >= lo
    return v + 2 <= o and next_mult(v, 1) <= o

  def _hold_running(self, frame, inp):
    h = self.hold
    if self.sub in ("down", "up"):
      v = inp.set_mph
      if h["released"] is None:
        why = ""
        if not self._hold_may_press(v):
          why = "limit"
        elif h["first"] is not None and (frame - h["first"]) * DT >= HOLD_TEST_S - 1e-9:
          why = "time"
        if why:
          self._release(frame, why)
          return None
        if h["first"] is None:
          if inp.counter == h["counter0"] and (frame - h["requested"]) * DT < 0.2 - 1e-9:
            return None               # a new car button frame first, as a tap
          if not self.hold_guard.begin(frame, inp.mono, other=self.guard):
            h["counter0"], h["requested"] = inp.counter, frame
            return None
          h["first"] = frame
        elif inp.counter == h["last_counter"]:
          return None                 # one pressed frame per car button frame
        if not self.hold_guard.allow(frame, inp.mono):
          self._release(frame, "cap")
          return None
        h["frames"].append((int(frame), inp.counter, (inp.counter + 1) % 16))
        h["last_counter"] = inp.counter
        return h["button"]
    if self.sub in ("down_settle", "up_settle"):
      if not self._settled(frame):
        return None
      key = "down" if self.sub == "down_settle" else "up"
      word, text = classify_hold(h["s0"], h["d"], h["changes"])
      self.dirs[key] = {"word": word, "text": text, "s0": h["s0"], "end": int(inp.set_mph), "frames": len(h["frames"]),
                        "changes": [[int(round(t * 1000)), v] for t, v in h["changes"]], "why": h["why"]}
      self._ev("direction", frame, dir=key, button=h["button"], word=word, text=text, s0=h["s0"], end=int(inp.set_mph),
               trace=self.dirs[key]["changes"], frames=[list(f) for f in h["frames"]], release=h["why"])
      self.hold = None
      if key == "down":
        v = int(inp.set_mph)
        if v <= self.orig - 2 and next_mult(v, 1) <= self.orig:
          self._begin_hold(frame, inp, "SET_PLUS")
          return self._hold_running(frame, inp)
        self.dirs["up"] = {"word": "NOT TRIED", "text": "not tried: " + ("SET- changed nothing" if v == self.orig else
                           f"a hold up from {v} could pass {self.orig}"), "s0": v, "end": v, "frames": 0, "changes": [], "why": ""}
        self._ev("direction", frame, dir="up", word="NOT TRIED", text=self.dirs["up"]["text"], s0=v, end=v)
      self.final = "finished"
      self._to_restore(frame, inp)
      return self._restoring(frame, inp) if self.state == "restoring" else None
    return None

  def _release(self, frame, why):
    h = self.hold
    if h is None or h["released"] is not None:
      return
    self.hold_guard.end()
    h["released"], h["why"] = frame, why
    self._ev("hold_release", frame, button=h["button"], why=why, frames=len(h["frames"]),
             ms=None if h["first"] is None else int(round((frame - h["first"]) * DT * 1000)))
    self.sub = "down_settle" if h["d"] < 0 else "up_settle"

  # ---- the tap test ------------------------------------------------------------------------------------------------------------
  def _taps_running(self, frame, inp, steady_s):
    if self.sub == "pre":
      w = precondition(inp, "taps", steady_s)
      self.why = w
      if w or not self._settled(frame):
        return None
      self.start_mph = self.lowest_mph = inp.mph
      c = CYCLES[self.cycle_i]
      self.cycles.append({"cycle": c, "down": None, "up": None, "clean": False, "start_mph": round(inp.mph, 1), "lowest_mph": None})
      self._ev("cycle_start", frame, cycle=c, set_mph=int(inp.set_mph), mph=round(inp.mph, 1))
      self.guard.set_cycle(c)
      self._begin_block(frame, inp, "SET_MINUS")
    if self.sub in ("down", "up"):
      return self._block_press(frame, inp)
    if self.sub in ("down_settle", "up_settle"):
      if not self._settled(frame):
        return None
      b = self.block
      key = "down" if b["d"] < 0 else "up"
      st = press_stats(b["presses"], b["changes"], b["d"], frame)
      st["cycle_ms"] = (int(round(sorted(y - x for x, y in zip(b["presses"], b["presses"][1:]))[(len(b["presses"]) - 1) // 2] * DT * 1000))
                        if len(b["presses"]) >= 2 else None)
      st["frames"] = b["frames"]
      cyc = self.cycles[-1]
      cyc[key] = st
      self._ev("block", frame, cycle=cyc["cycle"], dir=key, s0=b["s0"], end=int(inp.set_mph),
               **{k: v for k, v in st.items() if k != "frames"}, frames=[list(f) for f in b["frames"]],
               trace=[[int(round((f - b["presses"][0]) * DT * 1000)) if b["presses"] else 0, dd] for f, dd in b["changes"]])
      self.block = None
      if key == "down":
        self._begin_block(frame, inp, "SET_PLUS")
        return self._block_press(frame, inp)
      cyc["lowest_mph"] = round(self.lowest_mph, 1)
      cyc["clean"] = all(cyc[k] is not None and cyc[k]["sent"] == PRESSES and cyc[k]["per"] == [1] * PRESSES and not cyc[k]["jump"]
                         and not cyc[k]["wrong"] for k in ("down", "up"))
      self._ev("cycle", frame, cycle=cyc["cycle"], clean=cyc["clean"], start_mph=cyc["start_mph"], lowest_mph=cyc["lowest_mph"])
      self.cycle_i += 1
      if self.cycle_i < len(CYCLES) and inp.set_mph == self.orig:
        self.sub, self.sub_t0 = "pre", frame
        return None
      if self.cycle_i < len(CYCLES):
        self._ev("cycles_stopped", frame, why=f"set speed {inp.set_mph} mph after the cycle (was {self.orig})")
      self.final = "finished"
      self._to_restore(frame, inp)
      return self._restoring(frame, inp) if self.state == "restoring" else None
    return None

  def _begin_block(self, frame, inp, button):
    d = -1 if button == "SET_MINUS" else 1
    self.block = {"button": button, "d": d, "s0": int(inp.set_mph), "presses": [], "changes": [], "frames": []}
    self.sub, self.sub_t0 = "down" if d < 0 else "up", frame
    self.tap = None

  def _block_press(self, frame, inp):
    b = self.block
    c = CYCLES[self.cycle_i]
    if self.tap is None:
      if len(b["presses"]) >= PRESSES or not self._press_ok(frame, inp):
        self.sub = "down_settle" if b["d"] < 0 else "up_settle"
        return None
      self.tap = Tap(b["button"], frames_for(c), frame, inp.counter)
      self.tap_set_first = None
    tap = self.tap
    out = None
    if tap.due(frame, inp.counter):
      if tap.first is not None and inp.set_mph != self.tap_set_first:
        tap.n = len(tap.frames)         # the step is read back: no second frame
      else:
        if tap.first is None:
          if not self._press_ok(frame, inp):
            self.tap = None             # checked again on the frame it would start: the dash may have moved since
            self.sub = "down_settle" if b["d"] < 0 else "up_settle"
            return None
          if not self._tap_begin(frame, inp.mono):
            tap.counter0, tap.requested = inp.counter, frame
            return None
          tap.first = frame
          self.tap_set_first = inp.set_mph
          b["presses"].append(frame)
        if self.guard.allow(frame, inp.mono):
          tap.frames.append((int(frame), inp.counter, (inp.counter + 1) % 16))
          b["frames"].append((int(frame), inp.counter, (inp.counter + 1) % 16))
          out = tap.button
        else:
          tap.n = len(tap.frames)
    if tap.done:
      self.guard.end()
      self.tap = None
    return out

  def _tap_begin(self, frame, mono):
    """A tap's first frame: the press guard (its cycle) and MIN_TAP_GAP_S after any held frame (HoldGuard.begin checks the other way)."""
    hg = self.hold_guard
    if hg.last_frame is not None and (frame - hg.last_frame) * DT < MIN_TAP_GAP_S - 1e-9:
      return False
    if mono is not None and hg.last_mono is not None and mono - hg.last_mono < MIN_TAP_GAP_S - 1e-6:
      return False
    return self.guard.begin(frame, mono)

  def _press_ok(self, frame, inp):
    """A press of the tap test only while it cannot take the dash past its bound, a step of the previous press still in flight
    included, whether the car reads it as a tap (1 mph) or as a hold (a jump to the next multiple of 5, as the hold test allows for):
    SET+ never past the original; SET- never below the original - PRESSES - 1."""
    b = self.block
    inflight = 0
    if b["presses"]:
      last = b["presses"][-1]
      if (frame - last) * DT < INFLIGHT_S and not any(f >= last for f, _ in b["changes"]):
        inflight = 1
    v, d = inp.set_mph, b["d"]
    if d > 0:
      return max(v + inflight + 1, next_mult(v + inflight, 1)) <= self.orig      # the jump comes after the first step of a hold
    return min(v - inflight - 1, next_mult(v - inflight, -1)) >= self.orig - PRESSES - 1

  def _taps_jumped(self, frame, inp):
    """A step of 2 mph or more in a tap cycle: the car read the presses as a hold. The cycle ends here (no further press), the
    faster cycles are skipped (a faster rate can only be more hold-like) and the set speed is put back."""
    if self.tap is not None:
      self.guard.end()
      self.tap = None
    b, cyc = self.block, self.cycles[-1]
    key = "down" if b["d"] < 0 else "up"
    st = press_stats(b["presses"], b["changes"], b["d"], frame)
    st["frames"] = b["frames"]
    st["cycle_ms"] = None
    cyc[key] = st
    cyc["jump_stop"] = True
    cyc["path"] = [b["s0"]] + [int(b["s0"] + sum(dd for _, dd in b["changes"][:i + 1])) for i in range(len(b["changes"]))]
    cyc["lowest_mph"] = round(self.lowest_mph, 1) if self.lowest_mph is not None else None
    self._ev("cycle_jump", frame, cycle=cyc["cycle"], dir=key, s0=b["s0"], set_mph=inp.set_mph, path=cyc["path"],
             **{k: v for k, v in st.items() if k != "frames"}, frames=[list(f) for f in b["frames"]])
    skipped = [c for c in CYCLES if c < cyc["cycle"]]
    if skipped:
      self._ev("cycles_stopped", frame, why="a jump: the car read the presses as a hold", skipped=skipped)
    self.block = None
    self.final = "finished"
    self._to_restore(frame, inp)

  # ---- abort and restore -------------------------------------------------------------------------------------------------------
  def _abort(self, frame, inp, why):
    if self.hold is not None and self.hold["released"] is None:
      self.hold_guard.end()
      self._ev("hold_release", frame, button=self.hold["button"], why="abort", frames=len(self.hold["frames"]))
    if self.tap is not None:
      self.guard.end()
      self.tap = None
    if self.test == "taps" and self.block is not None and self.cycles:
      b = self.block
      st = press_stats(b["presses"], b["changes"], b["d"], frame)
      st["frames"] = b["frames"]
      st["cycle_ms"] = None
      self.cycles[-1]["down" if b["d"] < 0 else "up"] = st
      self.cycles[-1]["cut"] = True
      self.cycles[-1]["lowest_mph"] = round(self.lowest_mph, 1) if self.lowest_mph is not None else None
      self._ev("block", frame, cycle=self.cycles[-1]["cycle"], dir="down" if b["d"] < 0 else "up", s0=b["s0"], cut=True,
               **{k: v for k, v in st.items() if k != "frames"}, frames=[list(f) for f in b["frames"]])
    if self.test == "hold" and self.hold is not None:
      h = self.hold
      key = "down" if h["d"] < 0 else "up"
      word, text = classify_hold(h["s0"], h["d"], h["changes"])
      self.dirs[key] = {"word": "CUT", "text": text + " (cut short)", "s0": h["s0"], "end": inp.set_mph, "frames": len(h["frames"]),
                        "changes": [[int(round(t * 1000)), v] for t, v in h["changes"]], "why": "abort"}
      self._ev("direction", frame, dir=key, word="CUT", seen=word, text=text, trace=self.dirs[key]["changes"],
               frames=[list(f) for f in h["frames"]])
    self.hold, self.block = None, None
    self.final, self.abort_why = "aborted", why
    self.no_restore = why == "you pressed a cruise button"
    self._ev("aborted", frame, why=why, set_mph=inp.set_mph, mph=round(inp.mph, 1))
    self._to_restore(frame, inp)

  def not_restored_text(self, v):
    return f"Set speed is now {v} mph (was {self.orig}): set it yourself"

  def _to_restore(self, frame, inp):
    self.state, self.sub, self.sub_t0 = "restoring", "settle", frame
    self.restore_t0 = frame
    self.guard.set_cycle(TAP_CYCLE_SLOW_S)     # the reviewed recipe and gap for every restore tap
    self._ev("restore_start", frame, set_mph=inp.set_mph, orig=self.orig, driver=self.no_restore)

  def _restore_blocked(self, inp):
    if inp.others_on:
      return inp.others_on
    if inp.cancel_req or inp.resume_req:
      return "openpilot is using the cruise buttons"
    if not inp.engaged:
      return "openpilot not engaged"
    if not inp.lat_active:
      return "steering not active"
    if not (inp.cruise_on and inp.cruise_available):
      return "stock cruise not active"
    if inp.brake or inp.gas:
      return "brake or gas pressed"
    if inp.counter is None or inp.driver_button is None or inp.set_mph is None:
      return "cruise frames unreadable"
    return ""

  def _restoring(self, frame, inp):
    if inp.driver_button or self.no_restore:
      self.no_restore = True
      self._end_restore(frame, inp, None, "you pressed a cruise button: your set speed stands")
      return None
    v = inp.set_mph
    blocked = self._restore_blocked(inp)
    if self.sub == "retry":
      if (frame - self.retry_t0) * DT >= RESTORE_RETRY_S:
        self._end_restore(frame, inp, False, self.retry_why, final=True)
        return None
      if blocked or not self._settled(frame):
        return None
      self.restore["attempts"] += 1
      self._ev("restore_retry", frame, set_mph=v, orig=self.orig, taps=self.restore["taps"])
      self.restore_t0 = frame
      self.sub, self.sub_t0 = "settle", frame
    if self.sub == "settle":
      if not self._settled(frame):
        return None
      if v == self.orig:
        self.sub, self.sub_t0 = "verify", frame
        return None
      if blocked:
        if (frame - self.restore_t0) * DT >= RESTORE_WAIT_S:
          self._end_restore(frame, inp, False, blocked)
        return None
      if v is None or abs(v - self.orig) > RESTORE_MAX_MPH or self.restore["taps"] >= RESTORE_TAPS:
        self._end_restore(frame, inp, False, "too far or too many taps")
        return None
      button = "SET_PLUS" if v < self.orig else "SET_MINUS"
      self.restore["taps"] += 1
      self.restore_from = v
      self.tap = Tap(button, frames_for(TAP_CYCLE_SLOW_S), frame, inp.counter, v + (1 if v < self.orig else -1))
      self.sub, self.sub_t0 = "tap", frame
    if self.sub == "tap":
      if blocked:
        self.guard.end()
        self.tap = None
        self.sub, self.sub_t0 = "settle", frame       # nothing more while blocked; the wait counts toward RESTORE_WAIT_S
        return None
      if v not in (self.restore_from, self.tap.target):
        self._end_restore(frame, inp, False, f"set speed went to {v} mph")
        return None
      return self._restore_send(frame, inp)
    if self.sub == "wait":
      if v == self.tap.target:
        self._ev("restore_step", frame, set_mph=v, ms=int(round((frame - self.tap.first) * DT * 1000)))
        self.tap = None
        self.sub, self.sub_t0 = "settle", frame
        return None
      if v != self.restore_from or (frame - self.tap.first) * DT >= ACCEPT_WAIT_S:
        self._ev("restore_step", frame, set_mph=v, accepted=False)
        self._end_restore(frame, inp, False, "a tap did not give its 1 mph step")
      return None
    if self.sub == "verify":
      if v != self.orig:
        self._end_restore(frame, inp, False, f"set speed went to {v} mph")
        return None
      if (frame - self.sub_t0) * DT >= VERIFY_S:
        self.restore["verified"] = True
        self._end_restore(frame, inp, True, "")
      return None
    return None

  def _restore_send(self, frame, inp):
    tap = self.tap
    out = None
    if tap.due(frame, inp.counter) and inp.set_mph == tap.target:
      tap.n = len(tap.frames)
    elif tap.due(frame, inp.counter):
      if tap.first is None:
        if not self._tap_begin(frame, inp.mono):
          tap.counter0, tap.requested = inp.counter, frame
          return None
        tap.first = frame
      if self.guard.allow(frame, inp.mono):
        tap.frames.append((int(frame), inp.counter, (inp.counter + 1) % 16))
        out = tap.button
      else:
        tap.n = len(tap.frames)
    if tap.done:
      if tap.first is None:
        tap.first = frame
      self.guard.end()
      self._ev("restore_tap", frame, button=tap.button, frames=[list(f) for f in tap.frames], set_before=self.restore_from,
               set_now=inp.set_mph)
      self.sub, self.sub_t0 = "wait", frame
    return out

  def _end_restore(self, frame, inp, ok, why, final=False):
    if self.tap is not None:
      self.guard.end()
      self.tap = None
    v = inp.set_mph
    if (ok is False and not final and self.retry_t0 is None and why != "stopped on the screen" and v and self.orig is not None
            and v != self.orig and abs(v - self.orig) <= RESTORE_MAX_MPH and self.restore["taps"] < RESTORE_TAPS):
      self.retry_t0, self.retry_why = frame, why          # ONE more attempt when the conditions allow again
      self.sub, self.sub_t0 = "retry", frame
      self._ev("restore_retry_wait", frame, why=why, set_mph=v, orig=self.orig, taps=self.restore["taps"])
      return
    unknown = not v
    if ok is None:
      note = "Your set speed stands (you pressed a cruise button)"
    elif unknown:
      ok = False
      note = f"Cruise went off: check your set speed (was {self.orig})"
    elif ok:
      note = f"Set speed back at {self.orig} mph and checked"
    elif v == self.orig:
      note = f"Set speed {self.orig} mph, not checked: {why}"
    else:
      note = self.not_restored_text(v)
    self.restore.update(ok=ok, note=note, left_at=v if v else None, unknown=unknown)
    self._ev("restore_end", frame, ok=ok, why=why, note=note, taps=self.restore["taps"], set_mph=v if v else None, unknown=unknown,
             attempts=self.restore["attempts"])
    if ok is False and not unknown and self.orig is not None and v != self.orig:
      self._ev("set_speed_not_restored", frame, set_mph=v, orig=self.orig, why=why, taps=self.restore["taps"])
    self.state = self.final or "finished"
    self.why = self.abort_why if self.state == "aborted" else ""
    self.sub = ""
    self._ev("result", frame, **self.summary())

  # ---- results and the page ------------------------------------------------------------------------------------------------------
  def fastest_clean(self):
    clean = [c["cycle"] for c in self.cycles if c.get("clean")]
    return min(clean) if clean else None

  def summary(self):
    out = {"state": self.state, "why": self.abort_why, "orig": self.orig, "set_mph": self.set_now, "restore": dict(self.restore),
           "start_mph": None if self.start_mph is None else round(self.start_mph, 1),
           "lowest_mph": None if self.lowest_mph is None else round(self.lowest_mph, 1)}
    if self.test == "hold":
      out["hold"] = {k: {kk: vv for kk, vv in v.items()} for k, v in self.dirs.items()}
    else:
      out["taps"] = [{k: ({kk: vv for kk, vv in v.items() if kk != "frames"} if isinstance(v, dict) else v) for k, v in c.items()}
                     for c in self.cycles]
      out["fastest_clean"] = self.fastest_clean()
    return out

  def not_back(self):
    """The run ended with the set speed readable and not at the original (the driver is to set it)."""
    v = self.restore.get("left_at")
    return (self.state in ("finished", "aborted") and self.restore.get("ok") is False and bool(v) and self.orig is not None
            and v != self.orig)

  def cycle_row(self, c):
    """"0.2 s: 10 sent, 10 steps, 1.00 per press, 60 ms, lost 1.7 mph: every press 1 mph" """
    if c is None:
      return ""
    if c.get("jump_stop"):
      parts = [x for x in (c.get("down"), c.get("up")) if x]
      path = c.get("path") or []
      steps = " -> ".join(str(x) for x in (path if len(path) <= 4 else path[:1] + ["..."] + path[-2:]))
      return cut(f"{c['cycle']:.1f} s: JUMP: the car read it as a hold ({sum(x['sent'] for x in parts)} sent, {steps})", MAX_DETAIL)
    parts = [x for x in (c.get("down"), c.get("up")) if x]
    sent = sum(x["sent"] for x in parts)
    steps = sum(x["steps"] for x in parts)
    ms = sorted(x["ms"] for x in parts if x.get("ms") is not None)
    issues = []
    missed = sum(x["missed"] for x in parts)
    double = sum(x["double"] for x in parts)
    jump = sum(x["jump"] for x in parts)
    if jump:
      issues.append("JUMP")            # a step of 2 mph or more: read as a hold
    if missed:
      issues.append(f"{missed} missed")
    if double:
      issues.append(f"{double} double")
    if c.get("cut"):
      issues.append("cut short")
    done = c.get("up") is not None or c.get("cut")
    word = "all 1 mph" if c.get("clean") else (", ".join(issues) if issues else ("not clean" if done else "so far"))
    per = f"{steps / sent:.2f}" if sent else "0.00"
    med = f"{ms[len(ms) // 2]} ms" if ms else "no step"
    return cut(f"{c['cycle']:.1f} s: {sent} sent, {steps} steps ({per}/press), {med}: {word}", MAX_DETAIL)

  def lines(self):
    """(line1, line2, result, d1, d2, d3): the page's text, written here to fit (MAX_*)."""
    name = NAMES.get(self.test, "BUTTON TESTS")
    st = self.state
    d = ["", "", ""]
    result = ""
    if self.test == "hold":
      for i, key in enumerate(("down", "up")):
        x = self.dirs.get(key)
        if x is not None:
          d[i] = cut(f"{'SET-' if key == 'down' else 'SET+'}: {x['text']}", MAX_DETAIL)
      if self.start_mph is not None and self.lowest_mph is not None:
        d[2] = f"Speed {self.start_mph:.1f} mph at the start, lowest {self.lowest_mph:.1f} mph"
    elif self.test == "taps":
      for i, c in enumerate(CYCLES):
        row = next((x for x in self.cycles if x["cycle"] == c), None)
        d[i] = self.cycle_row(row) if row is not None else f"{c:.1f} s: not run yet" if st in ACTIVE_STATES else f"{c:.1f} s: not run"
    if st == "idle":
      return ("READY: PICK A TEST", "Hold: 0.7 s each way. Taps: 5 down, 5 up, 3 rates",
              "", "Needs: engaged, cruise on, steady 30-70 mph, set 34 mph+, no car ahead",
              "Both also need a set speed ending in 0 or 5",
              "Brake, gas or a cruise button stops it; set speed is put back")
    if st == "refused":
      return cut("REFUSED: " + name, MAX_LINE1), cut(upper_first(self.why), MAX_LINE2), "", "", "", ""
    if st == "waiting":
      return (cut("WAITING: " + name, MAX_LINE1), cut(upper_first(self.why) or "Checking the conditions", MAX_LINE2), "",
              "Needs: engaged, cruise on, steady 30-70 mph, set 34 mph+, no car ahead",
              "Also needs a set speed ending in 0 or 5", "Gives up after 60 s of waiting")
    if st == "running":
      if self.test == "hold":
        l1 = "HOLD TEST: SET- HELD" if self.sub.startswith("down") else "HOLD TEST: SET+ HELD"
      else:
        l1 = (f"TAP TEST: {CYCLES[min(self.cycle_i, len(CYCLES) - 1)]:.1f} S " +
              ("WAITS" if self.sub == "pre" else ("SET-" if self.sub.startswith("down") else "SET+")))
      l2 = ("Waiting: " + self.why) if (self.sub == "pre" and self.why) else "Hands off the cruise buttons. Brake stops it."
      return cut(l1, MAX_LINE1), cut(l2, MAX_LINE2), "", d[0], d[1], d[2]
    if st == "restoring":
      if self.sub == "retry":
        return ("PUTTING SET SPEED BACK", cut(f"Will try once more: {self.retry_why}", MAX_LINE2), "", d[0], d[1], d[2])
      return ("PUTTING SET SPEED BACK", cut(f"To {self.orig} mph with single taps, then checked", MAX_LINE2), "", d[0], d[1], d[2])
    # finished / aborted
    v = self.set_now
    restored = self.restore.get("ok")
    if self.test == "hold":
      dn, up = self.dirs.get("down", {}).get("word", "-"), self.dirs.get("up", {}).get("word", "-")
      result = cut(f"SET-: {dn}   SET+: {up}", MAX_RESULT)
    else:
      f = self.fastest_clean()
      result = f"FASTEST CLEAN RATE: {f:.1f} S" if f is not None else "NO RATE STEPPED 1 MPH EVERY PRESS"
    not_back = self.not_back()
    v = self.restore.get("left_at", v)                                     # where the run left it
    unknown = bool(self.restore.get("unknown")) and self.orig is not None    # cruise off / standby: never "unchanged"
    if unknown:
      result = cut(f"CRUISE OFF: CHECK SET SPEED (WAS {self.orig})", MAX_RESULT)
    elif not_back and self.answered:
      result = "YOUR SET SPEED STANDS"
    elif not_back:
      result = cut(f"SET IT YOURSELF: NOW {v} MPH, WAS {self.orig}", MAX_RESULT)
    if st == "aborted":
      # the stop's reason on the second line; the large line says where the set speed is (the results so far are below)
      if self.orig is None:
        result = ""                   # never started: nothing was pressed
      elif restored is None:
        result = "YOUR SET SPEED STANDS"
      elif not not_back and not unknown:
        result = cut(f"SET SPEED {'PUT BACK' if self.restore.get('taps') else 'UNCHANGED'}: {self.orig} MPH", MAX_RESULT)
      if self.orig is None and self.abort_why.startswith("not started: "):
        return (cut(name + " NOT RUN", MAX_LINE1), cut("Gave up: " + self.abort_why[len("not started: "):], MAX_LINE2),
                "", "Nothing was pressed", "", "")
      return cut(name + " STOPPED", MAX_LINE1), cut("Stopped: " + self.abort_why, MAX_LINE2), result, d[0], d[1], d[2]
    return cut(name + " DONE", MAX_LINE1), cut(self.restore.get("note", ""), MAX_LINE2), result, d[0], d[1], d[2]

  def status(self, wall):
    l1, l2, res, d1, d2, d3 = self.lines()
    return {"t": round(wall, 1), "v": 1, "id": self.run_id, "state": self.state, "test": self.test, "line1": l1, "line2": l2,
            "result": res, "d1": d1, "d2": d2, "d3": d3, "orig": self.orig, "set": self.set_now}
