"""Stuck-request guard for the Mazda torque interceptor (TI) command (CAM_LKAS2 / 0x249).

The fault. The interceptor compares each received request with the previous one for EXACT equality. A request that stays
unchanged, nonzero and not exactly +600 or -600 for about 1.9 s makes the unit report violation 0x11 (LKAS_STUK, "LKAS request
input does not change when not 0 or +/-600") and drop to OFF: interceptor steering is gone for about 31 s (the fault clears
about 29 s later and RUN returns about 2 s after that). Zero and exactly +-600 are exempt. Observed: -650 held 1.90 s and -570
held 1.90 s on this car (application version 16); another fork's CX-9 with the same unit tripped 22 of 22 times after a constant
command held 1.84-1.99 s, including -1 held 2 s. The exemption of exactly +-600 is the firmware's fixed request clip
(TI_FW_CLIP), not the tunable TiSteerMax (100-1200): a command saturated at any other TiSteerMax is the likely real trigger.

The guard. Called on the FINAL interceptor command, after every limiter, dither and test, just before it is packed, so it sees
exactly what the unit will receive. It counts consecutive frames on which the SENT value was identical; when a nonzero value other
than exactly +-600 has been sent STUCK_HOLD_FRAMES times in a row and the command is still that value, it sends the value moved
STUCK_NUDGE counts toward zero for STUCK_NUDGE_FRAMES frames (|value| <= STUCK_NUDGE: zero), then the command again, which starts
a new count. It never increases |command|, never crosses zero, never acts while lateral control is inactive or on zero / +-600,
and a command that changes on its own is passed through untouched.

Constants (the reasons are in the constants' comments). The caller feeds the SENT value back to the rate limiter as its history,
so the step back up from the nudged value is the limiter's own (inside every rate limit, in any configuration).
"""
# RUNTIME: runs in card at 100 Hz; gc disabled (see docs/RUNTIME_CONSTRAINTS.md).
# Per frame: create no class or closure, keep no container without a bound; params / JSON at most 10 Hz.

DT = 0.01                    # s per car-controller frame (100 Hz)

# The interceptor firmware's fixed request clip: a request of exactly +-600 is exempt from the stuck check (v6 image; on this car
# a saturated -600 was held about 5 s and the unit stayed in RUN). Fixed in the firmware: never the tunable TiSteerMax.
TI_FW_CLIP = 600

# Hold before the nudge. The shortest observed trip is 1.84 s. 0.8 s keeps even a WHOLE lost nudge burst safe: the unit would see
# 2 x 0.8 s + 0.02 s = 1.62 s of one value, 0.22 s under the shortest trip. (1.0 s would leave 2.02 s on a lost burst: over it.)
STUCK_HOLD_S = 0.8
STUCK_HOLD_FRAMES = int(round(STUCK_HOLD_S / DT))

# Size of the nudge, counts toward zero. The firmware resets its streak on any change of the raw request (exact equality, v6), so
# 1 count would already do; 2 (as proposed) costs nothing more: far below the unit's own +-6-count request dead band (v6: requests
# -6..+6 give zero internal bias; bias = trunc(10 p / 67), so 2 request counts are 0.3 internal counts) and inside every rate limit
# (TI_STEER_DELTA_DOWN >= 10, the firmware's own slew 3 up / 8 down per 5 ms tick). Not felt.
STUCK_NUDGE = 2

# Frames the nudged value is sent. 2: one dropped or late CAN frame still leaves a changed value on the bus (with 1, a single drop
# would leave the unit a 1.6 s streak, safe only by the hold margin above). Both together: a lost frame costs nothing, a lost burst
# is still under the trip.
STUCK_NUDGE_FRAMES = 2


def toward_zero(value):
  """value moved STUCK_NUDGE counts toward zero, never across it (|value| <= STUCK_NUDGE: 0, itself exempt and a change)."""
  if value > 0:
    return max(value - STUCK_NUDGE, 0)
  return min(value + STUCK_NUDGE, 0)


# Replayability (owner, 2026-11-26 item 6). The ti_stuck_guard event stays at most one per TI_STUCK_LOG_EVERY_FRAMES (carcontroller.py):
# lifting that limit would log every burst (one per 0.82 s through a long held command, ~11 kB/min and a log call on the control core
# each time); two fields cost nothing per frame (two numbers kept on a burst frame only). The count of every burst is the cumulative
# `bursts`; every value sent, nudges included, is in the rlog's sendcan (0x249) with latActive in carControl, from which the guard's
# run is replayed. StuckGuard attributes: logged (event key) or exempt (why). Guarded by test_mazda_ti_alert_replayable.py.
STUCK_LOGGED = {"bursts": "bursts", "fired_run": "run", "fired_s": "held_s", "held": "value"}
STUCK_EXEMPT = {
  "last_out": "the previous value sent: sendcan (0x249) in the rlog",
  "run": "frames last_out has been sent: counted from sendcan",
  "since": "when last_out was first sent: sendcan's log time",
  "nudge_left": "nudge frames still to send: follows from sendcan and STUCK_NUDGE_FRAMES",
  "nudged": "this frame's value is a nudge: sendcan",
  "burst_started": "a burst began this frame: the event itself",
}


class StuckGuard:
  """One per car controller. update() returns the value to send; `nudged` says whether this frame's value is a nudge and
  `burst_started` whether a new nudge burst began on this frame; `bursts` counts them since start."""

  def __init__(self):
    self.last_out = None       # the value sent on the previous frame (None: lateral was inactive)
    self.run = 0               # consecutive frames last_out has been sent
    self.since = None          # clock time (s) last_out was first sent; None without a clock
    self.held = None           # the value being nudged
    self.nudge_left = 0        # nudge frames still to send for `held`
    self.nudged = False
    self.burst_started = False
    self.bursts = 0
    self.fired_run = 0         # frames the value had been sent unchanged when the latest burst fired (logged as "run")
    self.fired_s = None        # ... and by the clock, s (None without a clock; logged as "held_s")

  def reset(self):
    self.last_out, self.run, self.held, self.nudge_left, self.since = None, 0, None, 0, None

  def update(self, cmd, active, now=None):
    """cmd: the final command about to be packed (int). active: lateral control active and the interceptor taking commands.
    now: a monotonic clock in seconds (optional). With it the hold is ALSO judged in real time, so a stalled control loop
    (frames arriving late) cannot stretch an unchanged value past STUCK_HOLD_S: whichever of frames or time is reached first."""
    self.nudged = self.burst_started = False
    if not active:
      self.reset()
      return cmd
    out = cmd
    if self.nudge_left > 0 and cmd == self.held:
      out = toward_zero(cmd)
      self.nudge_left -= 1
      self.nudged = True
    else:
      self.nudge_left = 0      # the command moved on its own during a burst: sent as it is, the burst is over
      held_long = self.run >= STUCK_HOLD_FRAMES or (now is not None and self.since is not None and now - self.since >= STUCK_HOLD_S)
      if cmd == self.last_out and held_long and cmd != 0 and cmd != TI_FW_CLIP and cmd != -TI_FW_CLIP:
        self.held = cmd
        self.nudge_left = STUCK_NUDGE_FRAMES - 1
        out = toward_zero(cmd)
        self.nudged = self.burst_started = True
        self.bursts += 1
        self.fired_run, self.fired_s = self.run, (now - self.since) if (now is not None and self.since is not None) else None
    if out == self.last_out:
      self.run += 1
    else:
      self.last_out, self.run, self.since = out, 1, now
    return out
