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
it stays off for HOLDOFF_S. It fades in over FADE_IN_S and out over FADE_OUT_S with a raised-cosine envelope; a driver
touch, lateral control dropping or the TI leaving RUN zero it at once, and so does a live rate limit with too little
room for it (`rate_room`).

Reconstruction from logs (no new log fields).
  requested dither = mazdaDiagnostics.tiRequested - round(carControl.actuators.steer * tiMax), for the carControl
                     whose logMonoTime is carOutput.appliedCarControlMonoTime (exact);
  applied dither   = carOutput.actuatorsOutput.steerOutputCan - round(actuatorsOutput.steer * mazdaDiagnostics.tiMax)
                     (carcontroller reports `steer` as the limiter's output for the UNDITHERED request; steerOutputCan
                     and tiLimited are the command sent). It equals the requested dither except where the limiter
                     clipped, and its magnitude never exceeds it.
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
DRIVER_QUIET_COUNTS = 4.0     # |driver torque| must stay under this (and steeringPressed false)
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


class TiDither:
  def __init__(self, preset=ACTIVE_PRESET):
    self.preset = preset
    self.envelope = 0.0       # 0..1, linear; the applied gain is its raised-cosine shape
    self.holdoff = 0          # frames left before the dither may fade in again
    self.last = 0             # the integer dither of the latest update

  def reset(self):
    """Lateral control off or TI not in RUN: zero at once, start any later fade-in from nothing."""
    self.envelope = 0.0
    self.last = 0

  def gain(self):
    return 0.5 - 0.5 * math.cos(math.pi * self.envelope)

  def update(self, enabled, frame, *, v_ego, steering_pressed, driver_torque, blinker, base_request, ti_max,
             lat_demand, room=None):
    """The integer dither (TI counts) to add to this frame's TI request. Call only while lateral control is active
    and the TI is allowed; otherwise call `reset`. `room`: `rate_room` of the live limits (None: not checked).

    A driver touch (steeringPressed or |driver torque| >= DRIVER_QUIET_COUNTS) and too little rate room zero the
    dither at once; every other exit fades it out over FADE_OUT_S."""
    touch = steering_pressed or abs(driver_torque) >= DRIVER_QUIET_COUNTS
    if touch or blinker:
      self.holdoff = int(round(HOLDOFF_S / DT))
    elif self.holdoff > 0:
      self.holdoff -= 1
    if touch or (room is not None and not rate_room_ok(room, self.preset)):
      self.reset()
      return 0
    want = enabled and self.holdoff == 0 and gate_open(
      v_ego=v_ego, steering_pressed=steering_pressed, driver_torque=driver_torque, blinker=blinker,
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
