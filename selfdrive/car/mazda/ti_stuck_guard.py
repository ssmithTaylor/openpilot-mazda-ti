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


class StuckGuard:
  """One per car controller. update() returns the value to send; `nudged` says whether this frame's value is a nudge and
  `burst_started` whether a new nudge burst began on this frame; `bursts` counts them since start."""

  def __init__(self):
    self.last_out = None       # the value sent on the previous frame (None: lateral was inactive)
    self.run = 0               # consecutive frames last_out has been sent
    self.held = None           # the value being nudged
    self.nudge_left = 0        # nudge frames still to send for `held`
    self.nudged = False
    self.burst_started = False
    self.bursts = 0

  def reset(self):
    self.last_out, self.run, self.held, self.nudge_left = None, 0, None, 0

  def update(self, cmd, active):
    """cmd: the final command about to be packed (int). active: lateral control active and the interceptor taking commands."""
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
      if cmd == self.last_out and self.run >= STUCK_HOLD_FRAMES and cmd != 0 and cmd != TI_FW_CLIP and cmd != -TI_FW_CLIP:
        self.held = cmd
        self.nudge_left = STUCK_NUDGE_FRAMES - 1
        out = toward_zero(cmd)
        self.nudged = self.burst_started = True
        self.bursts += 1
    if out == self.last_out:
      self.run += 1
    else:
      self.last_out, self.run = out, 1
    return out
