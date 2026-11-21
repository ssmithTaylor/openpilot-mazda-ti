"""Drive plan (ti_plan.py): one switch (param TiPlanOn, tmpfs, off at every boot, off by default) that sequences the A/B alternation
(ti_ab.py) and the identification tests (ti_ident.py, run-all mode) so that exactly one of them is allowed at any moment and
never both. With the switch off nothing here acts and both modes work exactly as without it.

What is allowed is a pure function of state (`decide`): the alternation where it is configured, not complete and the speed is
inside PLAN_ALT_MIN_MPH..PLAN_ALT_MAX_MPH (a mile of hysteresis, and a minimum dwell, so hovering at a boundary does not flip);
the tests everywhere else, and everywhere once the alternation is complete (TARGET_MIN_PER_SIDE engaged, scored minutes on each side,
counted in ti_ab only in valid, non-voided, non-wash-in block time).

Hand-over (the scheduler runs in controlsd, the tests in the car controller; they talk through tmpfs):
  tests -> A/B: the plan publishes "ab"; the car controller stops arming the tests (they fade out as always) and acknowledges
                with {"seq", "quiet", "t"} once its signal is exactly zero; the alternation is allowed to START only after the
                acknowledgement of THIS plan sequence has been quiet for QUIET_SETTLE_S (and, as always, only through the
                straight-road switch gate of ti_ab).
  A/B -> tests: the plan publishes "hold" (neither is allowed); the alternation returns to A by its normal ramp; the plan
                publishes "tests" only once the blend is exactly A and has been for the wash-in time.
A plan that is stale (not refreshed for STALE_S), unreadable, in error ("dead") or missing allows no test at all.
Plan events (kind "plan") go in the `ti_ab` log channel: `mode` (off, hold, ab, tests), `why`, `seq`.
"""
import json
from typing import NamedTuple

DT = 0.01
H = 1.0                              # mph of hysteresis, as ti_ab's own speed gate
PLAN_ALT_MIN_MPH = 35.0
PLAN_ALT_MAX_MPH = 56.0              # first trial: replay evidence covers 35-56 mph only
TARGET_MIN_PER_SIDE = 20.0           # engaged, scored minutes per side before the alternation is complete
MIN_DWELL_S = 10.0                   # a speed-driven change of mode waits this long after the last change
QUIET_SETTLE_S = 1.0                 # the tests' signal has been zero this long before the alternation may start
STALE_S = 3.0                        # a plan not refreshed for this long allows no test
ACK_STALE_S = 2.5
PARAM_PLAN_ON, PARAM_PLAN, PARAM_PLAN_ACK = "TiPlanOn", "TiPlan", "TiPlanAck"
MODES = ("off", "hold", "ab", "tests")


def in_range(prev, mph, ceiling=PLAN_ALT_MAX_MPH):
  """The speed inside the alternation's range, with hysteresis: off below 35 and from the ceiling, on again from 36 and below
  ceiling - 1 mph."""
  if mph < PLAN_ALT_MIN_MPH or mph >= ceiling:
    return False
  if PLAN_ALT_MIN_MPH + H <= mph < ceiling - H:
    return True
  return prev


def decide(cfg_valid, complete, speed_in_range):
  """(mode, why) the plan wants: a pure function of state."""
  if complete:
    return "tests", "A/B complete"
  if not cfg_valid:
    return "tests", "A/B not configured"
  if not speed_in_range:
    return "tests", "speed outside the A/B range"
  return "ab", "A/B in its range"


class PlanOut(NamedTuple):
  mode: str            # off, hold, ab, tests: what is published
  ab_allowed: bool     # the alternation may run (and start) now
  why: str
  next: str
  seq: int


class Plan:
  def __init__(self, log=None):
    self.log = log
    self.mode = "off"
    self.seq = 0
    self.why = ""
    self.dwell = 10 ** 6
    self.in_range = False
    self.seen_off = False
    self.quiet = 0
    self.n = 0

  def _set(self, mode, why):
    if mode != self.mode:
      self.mode, self.why, self.dwell = mode, why, 0
      self.seq += 1
      self.quiet = 0
      if self.log is not None:
        try:
          self.log("plan", mode=mode, why=why, seq=self.seq)
        except Exception:
          pass
    else:
      self.why = why

  def update(self, *, on, cfg_valid, complete, mph, ab_at_a, ack, now, tests_toggles_on, ceiling=PLAN_ALT_MAX_MPH):
    """One control cycle. `ab_at_a`: the alternation's blend is exactly A and has been for its wash-in time. `ack`: the car
    controller's acknowledgement dict (or None); `now`: wall seconds."""
    self.n += 1
    self.dwell += 1
    if not on:
      self.seen_off = True
      self._set("off", "plan off")
      return self._out(False, "")
    self.in_range = in_range(self.in_range, mph, ceiling)
    if not self.seen_off:
      self._set("hold", "the plan was on when controls started: switch it off and on again")
      return self._out(False, "")
    if tests_toggles_on:
      self._set("hold", "single tests are on: the plan waits")
      return self._out(False, "")
    want, why = decide(cfg_valid, complete, self.in_range)
    forced = complete or not cfg_valid
    cur = self.mode
    if want == "tests":
      if cur != "tests":
        if cur == "ab" and not forced and self.dwell < MIN_DWELL_S / DT:
          pass                                        # a speed-driven change waits out the dwell
        elif cur != "hold":
          self._set("hold", why)                      # the alternation returns to A first
        elif ab_at_a:
          self._set("tests", why)
    else:
      if cur == "tests" and self.dwell < MIN_DWELL_S / DT:
        pass
      elif cur != "ab":
        self._set("ab", why)                          # the tests fade out; the alternation waits for their quiet
    # the acknowledgement of this sequence, fresh and quiet, for QUIET_SETTLE_S
    t_ack = ack.get("t") if isinstance(ack, dict) else None       # (stamps are rounded to 0.1 s: a hair "in the future" is fine)
    fresh = (isinstance(ack, dict) and ack.get("seq") == self.seq and ack.get("quiet") is True
             and isinstance(t_ack, (int, float)) and -0.5 <= now - t_ack <= ACK_STALE_S)
    self.quiet = self.quiet + 1 if fresh else 0
    allowed = self.mode == "ab" and self.quiet >= QUIET_SETTLE_S / DT
    return self._out(allowed, why)

  def _out(self, allowed, why):
    nxt = {"off": "", "hold": "waiting to hand over", "ab": "tests when the speed leaves the A/B range or A/B completes",
           "tests": "A/B when it is configured, incomplete and the speed is in its range"}[self.mode]
    return PlanOut(self.mode, bool(allowed), self.why or why, nxt, self.seq)


def encode_plan(out, now, dead=False):
  return json.dumps({"seq": out.seq, "mode": "dead" if dead else out.mode, "t": round(now, 1)})


def tests_gate(plan_on, raw_plan, now):
  """(hold, allowed) for the car controller. hold: no test may run. allowed: the plan is on and says "tests" (run-all mode is
  then forced). Anything unreadable, stale or not "tests" holds. A plan in error ("dead") holds whatever the switch says."""
  try:
    if isinstance(raw_plan, (bytes, bytearray)):
      raw_plan = raw_plan.decode("utf-8", "replace")
    plan = json.loads(raw_plan) if raw_plan else {}
    if not isinstance(plan, dict):
      plan = {}
  except Exception:
    plan = {}
  if plan.get("mode") == "dead":
    return True, False
  if not plan_on:
    return False, False
  t = plan.get("t")
  ok = (plan.get("mode") == "tests" and isinstance(t, (int, float)) and -0.5 <= now - t <= STALE_S)
  return (not ok), ok


def plan_seq(raw_plan):
  try:
    if isinstance(raw_plan, (bytes, bytearray)):
      raw_plan = raw_plan.decode("utf-8", "replace")
    seq = json.loads(raw_plan).get("seq")
    return seq if isinstance(seq, int) else None
  except Exception:
    return None


def make_ack(seq, ident_running, ident_last, dither_envelope, now):
  """The car controller's acknowledgement: quiet only when no test signal is in progress (fade included) and the last value sent
  was zero."""
  quiet = (not ident_running) and ident_last == 0 and dither_envelope == 0.0
  return {"seq": seq, "quiet": bool(quiet), "t": round(now, 1)}
