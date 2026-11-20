"""Identification dither on the Mazda torque-interceptor (TI) command. OFF unless the `TiDither` toggle is on.

Why. Normal driving cannot separate the two paths that steer this car: the TI command and the stock LKAS command are
both computed from the same `actuators.steer`, so the power steering's response to the TI's injected sensor torque at
a few sensor counts (the "small-signal assist"), and how quickly that response builds, are not identifiable from
ordinary logs. A small, known, zero-mean signal added to the TI command ONLY (never to the stock command) decorrelates
the two paths, and its response can be measured by synchronous detection over minutes of driving.

What. Two sine tones at frequencies chosen from the train-log noise spectra and the simulator's car models
(tools/mazda_sim/README.md, "TI identification dither"): above the controller's band and the slow lane weave
(~0.2 Hz), off the natural steering mode near 1.5-1.7 Hz, and slow enough that the dither's own slew stays well
inside TI_STEER_DELTA_UP. Amplitudes are in TI command counts; 1 count is ~0.054 sensor counts at the EPS, so preset 1
peaks at ~1.2 sensor counts, below the rack's measured friction (~2.5-4 sensor counts).

Where. carcontroller.py adds the integer returned by `TiDither.update` to the TI request BEFORE
`apply_ti_steer_torque_limits`, so the magnitude, driver-torque and rate limits bound the total exactly as before.
The stock command, CAN definitions and panda safety are untouched.

When. Only while lateral control is active and the TI is in RUN, the toggle is on, speed >= MIN_SPEED_MPS, the driver
is not touching the wheel, no blinker or lane change, the undithered TI request is at most MAX_BASE_FRACTION of
TI_STEER_MAX and the desired lateral acceleration is at most MAX_LAT_DEMAND_MPS2. After a driver touch or a blinker
it stays off for HOLDOFF_S (a touch: steeringPressed, a firm torque reading, or a small one that persists for
TOUCH_CONFIRM_FRAMES frames; see the constants). It fades in over FADE_IN_S and out over FADE_OUT_S with a raised-cosine envelope; a driver
touch, lateral control dropping or the TI leaving RUN zero it at once, and so does a live rate limit with too little
room for it (`rate_room`).

Reconstruction from logs (no new log fields).
  requested dither = mazdaDiagnostics.tiRequested - round(carControl.actuators.steer * tiMax), for the carControl
                     whose logMonoTime is carOutput.appliedCarControlMonoTime (exact);
  applied dither   = carOutput.actuatorsOutput.steerOutputCan - round(actuatorsOutput.steer * mazdaDiagnostics.tiMax)
                     (carcontroller reports `steer` as the limiter's output for the UNDITHERED request; steerOutputCan
                     and tiLimited are the command sent). It equals the requested dither except while the limiter
                     is clipping or rate-limiting the command: the sent and undithered limiter histories then
                     differ, so frame by frame the applied dither can be larger than the requested one or of the
                     opposite sign, but its magnitude never exceeds the preset's peak (22 counts for preset 1).
The unit waveform is `waveform(frame)` with frame = carOutput.applySequence - 1 (the car controller's frame counter).
"""
import math

DT = 0.01

# (frequency Hz, amplitude TI counts). Preset 1 is the one wired to the toggle; preset 2 is the stronger alternative
# described in the README and is selected only by changing ACTIVE_PRESET in code.
# Both presets keep their largest per-frame slew under half of the default TI_STEER_DELTA_UP (6): the dither is
# switched off whenever the live rate room is below RATE_ROOM_FACTOR times its slew (see `rate_room`).
PRESETS = {
  1: ((1.1, 12.0), (2.9, 10.0)),
  2: ((1.1, 24.0), (2.9, 6.0)),
}
ACTIVE_PRESET = 1

MIN_SPEED_MPS = 15.0          # above the LKAS_BLOCK clear speed (14.5 m/s): stock path consistently unblocked
MAX_BASE_FRACTION = 0.5       # undithered TI request at most half of TI_STEER_MAX: far from the clip and the knee
MAX_LAT_DEMAND_MPS2 = 1.0     # desired lateral acceleration v^2 * |curvature|: gentle curves and straights only
DRIVER_QUIET_COUNTS = 4.0     # |driver torque| at or above this for TOUCH_CONFIRM_FRAMES frames running is a touch
DRIVER_FIRM_COUNTS = 6.0      # |driver torque| at or above this is a touch on the first frame (as is steeringPressed)
# The first dither drive (2026-11-19) treated every reading at or above DRIVER_QUIET_COUNTS as a touch: 110 of its 139
# touch trips above the gate speed lasted one or two frames, 90 % of them a reading of exactly 4 counts, hands off,
# and each cost the hold-off and the fade-in. A reading has to persist now unless it is firm or the pressed flag is up.
TOUCH_CONFIRM_FRAMES = 3
HOLDOFF_S = 3.0               # off for this long after a driver touch, a blinker or a lane change
FADE_IN_S = 1.0
FADE_OUT_S = 0.25
# The rate limiter must leave room for the dither: with too little, rate limiting clips its rising edges and leaves a
# mean bias (review: -2.6 counts at TI_STEER_DELTA_UP 1, none at 6). Required: room >= this factor x max slew.
RATE_ROOM_FACTOR = 2.0


def max_slew_per_frame(preset=ACTIVE_PRESET):
  """Largest change of the unit waveform between two frames (TI counts): sum of 2*pi*f*A*DT."""
  return sum(2.0 * math.pi * f * a * DT for f, a in PRESETS[preset])


def peak_counts(preset=ACTIVE_PRESET):
  return sum(a for _, a in PRESETS[preset])


def waveform(frame, preset=ACTIVE_PRESET):
  """The full-amplitude dither at a frame (float TI counts). Deterministic in the frame counter, zero mean."""
  t = frame * DT
  return sum(a * math.sin(2.0 * math.pi * f * t) for f, a in PRESETS[preset])


def rate_room(limits):
  """The smallest per-frame step the live TI limiter allows in the range the dither works in (counts): the climb
  rate, the high-range climb rate when its knee lies inside that range, and the release rate."""
  up = limits.TI_STEER_DELTA_UP
  knee = getattr(limits, 'TI_STEER_DELTA_UP_KNEE', limits.TI_STEER_MAX)
  if knee <= MAX_BASE_FRACTION * limits.TI_STEER_MAX + max(peak_counts(p) for p in PRESETS):
    up = min(up, getattr(limits, 'TI_STEER_DELTA_UP_HIGH', up))
  return min(up, limits.TI_STEER_DELTA_DOWN)


def rate_room_ok(room, preset=ACTIVE_PRESET):
  return room >= RATE_ROOM_FACTOR * max_slew_per_frame(preset)


def gate_open(*, v_ego, steering_pressed, driver_torque, blinker, base_request, ti_max, lat_demand):
  """True when every condition that allows the dither this frame holds (the toggle and holdoff aside)."""
  return (v_ego >= MIN_SPEED_MPS and not steering_pressed and abs(driver_torque) < DRIVER_QUIET_COUNTS
          and not blinker and abs(base_request) <= MAX_BASE_FRACTION * ti_max
          and abs(lat_demand) <= MAX_LAT_DEMAND_MPS2)


def gate_reason(*, engaged, enabled, v_ego, steering_pressed, driver_torque, blinker, base_request, ti_max,
                lat_demand, holdoff, room=None, touch=None):
  """Display only: why the dither is or is not running, as one short code for the on-car Tests screen. Not used by
  `TiDither.update` or anywhere in the command path; the car controller calls it only when it publishes test
  progress (every 5 s, and only while a test is on). Same conditions, in the same order of precedence, as `update`.
  `touch`: the dither's own confirmed verdict (`TiDither.touch`); None: every reading at or above
  DRIVER_QUIET_COUNTS counts."""
  if not engaged:
    return "inactive"
  if not enabled:
    return "off"
  if (steering_pressed or abs(driver_torque) >= DRIVER_QUIET_COUNTS) if touch is None else touch:
    return "touch"
  if room is not None and not rate_room_ok(room):
    return "rate"
  if blinker:
    return "blinker"
  if holdoff > 0:
    return "holdoff"
  if v_ego < MIN_SPEED_MPS:
    return "speed"
  if abs(base_request) > MAX_BASE_FRACTION * ti_max:
    return "command"
  if abs(lat_demand) > MAX_LAT_DEMAND_MPS2:
    return "curve"
  return "ok"


class TiDither:
  def __init__(self, preset=ACTIVE_PRESET, touch_confirm_frames=TOUCH_CONFIRM_FRAMES):
    self.preset = preset
    self.touch_confirm_frames = touch_confirm_frames   # 1: the first drive's gate (a replay of that drive)
    self.loud = 0             # frames running with |driver torque| >= DRIVER_QUIET_COUNTS
    self.touch = False        # the latest update's verdict: the driver is touching the wheel
    self.envelope = 0.0       # 0..1, linear; the applied gain is its raised-cosine shape
    self.holdoff = 0          # frames left before the dither may fade in again
    self.last = 0             # the integer dither of the latest update

  def reset(self):
    """Lateral control off or TI not in RUN: zero at once, start any later fade-in from nothing."""
    self._zero()
    self.loud = 0
    self.touch = False

  def _zero(self):
    self.envelope = 0.0
    self.last = 0

  def gain(self):
    return 0.5 - 0.5 * math.cos(math.pi * self.envelope)

  def update(self, enabled, frame, *, v_ego, steering_pressed, driver_torque, blinker, base_request, ti_max,
             lat_demand, room=None):
    """The integer dither (TI counts) to add to this frame's TI request. Call only while lateral control is active
    and the TI is allowed; otherwise call `reset`. `room`: `rate_room` of the live limits (None: not checked).

    A driver touch and too little rate room zero the dither at once; every other exit fades it out over FADE_OUT_S.
    A touch is steeringPressed, |driver torque| >= DRIVER_FIRM_COUNTS, or |driver torque| >= DRIVER_QUIET_COUNTS on
    `touch_confirm_frames` updates running; a shorter small reading changes nothing."""
    magnitude = abs(driver_torque)
    self.loud = self.loud + 1 if magnitude >= DRIVER_QUIET_COUNTS else 0
    touch = self.touch = bool(steering_pressed or magnitude >= DRIVER_FIRM_COUNTS
                              or self.loud >= self.touch_confirm_frames)
    if touch or blinker:
      self.holdoff = int(round(HOLDOFF_S / DT))
    elif self.holdoff > 0:
      self.holdoff -= 1
    if touch or (room is not None and not rate_room_ok(room, self.preset)):
      self._zero()
      return 0
    # Not a touch (above): the gate's own touch terms are given quiet values so an unconfirmed reading does not fade.
    want = enabled and self.holdoff == 0 and gate_open(
      v_ego=v_ego, steering_pressed=False, driver_torque=0.0, blinker=blinker,
      base_request=base_request, ti_max=ti_max, lat_demand=lat_demand)
    if want:
      self.envelope = min(1.0, self.envelope + DT / FADE_IN_S)
    else:
      self.envelope = max(0.0, self.envelope - DT / FADE_OUT_S)
    if self.envelope <= 0.0:
      self.last = 0
    else:
      self.last = int(round(self.gain() * waveform(frame, self.preset)))
    return self.last
