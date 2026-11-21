"""Steering identification tests on the Mazda torque-interceptor (TI) command: frequency sweep, amplitude ladder and
short pulses. OFF unless one of the toggles TiSweep / TiLadder / TiPulses is on (Tests screen, default off).

Same principle and the same safety envelope as the identification dither (ti_dither.py), which stays exactly as it
was: a small, known, zero-mean, always-moving signal is added to the TI request ONLY, BEFORE
apply_ti_steer_torque_limits, so the magnitude, driver-torque and rate limits bound the total exactly as before. The
stock command, CAN definitions and panda safety are untouched. At most one injection runs at a time: these tests run
only while the dither toggle is off and the dither is silent, and only when exactly one of the three toggles is on.

Signal. Each test is a table of STEPS. A step is captured in CHUNKS: a whole number of cycles (sweep, ladder) or one
pulse period (pulses). A chunk counts only when every one of its frames was clean (gate open, full amplitude, one
speed band); a paused chunk is repeated, nothing else is. The step to run is chosen at every chunk start: the first
step of the table that is not complete IN THE BAND THE CAR IS IN NOW (so mixed-speed driving fills every band, and a
completed step is never repeated). Tone and amplitude changes happen at a zero of the waveform, so a chunk follows
the previous one with no gap. After a pause the test resumes at once: sweep and ladder with a short raised-cosine
lead-in (LEAD_IN_S) into a fresh chunk, pulses with the next pulse (a pulse starts at zero and ramps anyway).

Gate (all must hold, else the signal fades out in FADE_OUT_S; a driver touch or too little rate room zero it at
once; after a touch or a blinker it stays off for HOLDOFF_S). Touch, blinker, hold-off, base-request, lateral-demand
and rate-room rules are the dither's (ti_dither constants). Added for these tests:
  - the speed is inside one of BANDS (12-28, 31-45, 45-60, 60-80 mph);
  - the steering regime is the band's: below 28 mph the EPS blocks the stock lane-keep path and the TI steers alone
    (LKAS_BLOCK set); from 31 mph both paths steer (LKAS_BLOCK clear). The regime must have been unchanged for
    REGIME_SETTLE_S (the stock path clears about 3 s after rising through 50 km/h);
  - |desired curvature| <= MAX_CURVATURE (straight roads and gentle bends only), as well as v^2 |curvature| <= 1;
  - the band still has a step to capture.

Reconstruction. The requested added counts are exact from logs, as for the dither:
  added = mazdaDiagnostics.tiRequested - round(carControl.actuators.steer * tiMax)
(the dither is zero whenever these run). The regime of every sample is frogpilotCarState.lkasBlocked. Which test,
step, band and phase produced it: the generator logs an event (cloudlog "ti_ident", JSON) at every run start, chunk
start and stop; `reconstruct(events, n)` rebuilds the added counts frame by frame from those events alone (exactly;
test_mazda_ti_ident.py checks it). The run's stats config carries {"TiIdent": {test: {"version", "table", "frames",
"touch_confirm"}}} for every test that was on, `table` being TABLE_HASH[test] of the step tables below.
"""
import hashlib
import json
import math

from openpilot.selfdrive.car.mazda.ti_dither import (DRIVER_FIRM_COUNTS, DRIVER_QUIET_COUNTS, FADE_OUT_S, HOLDOFF_S,
                                                     MAX_BASE_FRACTION, MAX_LAT_DEMAND_MPS2, RATE_ROOM_FACTOR,
                                                     TOUCH_CONFIRM_FRAMES)

VERSION = 1
DT = 0.01
MS_TO_MPH = 1.0 / 0.44704

SWEEP, LADDER, PULSES = "TiSweep", "TiLadder", "TiPulses"
KEYS = (SWEEP, LADDER, PULSES)
TOGGLES = {SWEEP: "ti_sweep", LADDER: "ti_ladder", PULSES: "ti_pulses"}

# (low mph, high mph, label, TI alone): half-open [low, high). 28-31 mph is left out on purpose: the stock path
# switches there (blocked below 45 km/h, cleared above 50 km/h with a lag).
BANDS = ((12.0, 28.0, "12-28", True), (31.0, 45.0, "31-45", False), (45.0, 60.0, "45-60", False),
         (60.0, 80.0, "60-80", False))

MAX_CURVATURE = 0.004       # 1/m: radius 250 m or more
REGIME_SETTLE_S = 3.0
MAX_SLEW = 3.0              # counts per frame, lead-in included: half of the default TI_STEER_DELTA_UP (6)

# Sweep: one tone per step, (Hz, peak TI counts). 10 s chunks (a whole number of cycles of every tone), 2 per step.
# Amplitudes in the dither's class (its preset 1 peaks at 22 counts, 12 at its 1.1 Hz tone); the high tones are
# reduced so the slew stays under MAX_SLEW.
SWEEP_TONES = ((0.3, 12.0), (0.6, 12.0), (1.0, 16.0), (1.6, 16.0), (2.4, 16.0), (3.5, 12.0), (5.0, 9.0))
SWEEP_CHUNK = 1000
SWEEP_LEAD_IN = 50

# Ladder: one frequency, stepped amplitudes (TI counts; 1 count ~ 0.0544 EPS sensor counts): 0.3 to 5.0 sensor counts,
# from below the 1.2 sc at which the dither already moved the wheel to above the rack's friction (2.5-4 sc). 0.4 Hz
# keeps the 92-count step under MAX_SLEW. 5 s chunks (2 cycles), 2 per step. Per band only the steps up to the band's
# cap run (LADDER_CAPS: TI counts by band label).
LADDER_FREQ = 0.4
LADDER_AMPS = (6.0, 11.0, 22.0, 37.0, 55.0, 74.0, 92.0)
LADDER_CAPS = {"12-28": 92.0, "31-45": 55.0, "45-60": 37.0, "60-80": 37.0}
LADDER_CHUNK = 500
LADDER_LEAD_IN = 100

# Pulses: a bipolar trapezoid doublet (+A then -A, ramps at PULSE_RAMP counts per frame, PULSE_FLAT frames flat),
# then quiet to the end of a PULSE_PERIOD-frame chunk for the settling; the doublet's sign alternates pulse by pulse.
PULSE_AMPS = (22.0, 44.0)
PULSE_PERIOD = 200
PULSE_RAMP = 2.75
PULSE_FLAT = 10

STEP_SECONDS = {SWEEP: 20.0, LADDER: 10.0, PULSES: 40.0}


def _table(key):
  if key == SWEEP:
    return {"tones": SWEEP_TONES, "chunk": SWEEP_CHUNK, "lead_in": SWEEP_LEAD_IN}
  if key == LADDER:
    return {"freq": LADDER_FREQ, "amps": LADDER_AMPS, "caps": LADDER_CAPS, "chunk": LADDER_CHUNK,
            "lead_in": LADDER_LEAD_IN}
  return {"amps": PULSE_AMPS, "period": PULSE_PERIOD, "ramp": PULSE_RAMP, "flat": PULSE_FLAT}


TABLE_HASH = {k: hashlib.sha1(json.dumps([VERSION, _table(k), STEP_SECONDS[k], BANDS], sort_keys=True).encode())
              .hexdigest()[:12] for k in KEYS}


def n_steps(key):
  return len(SWEEP_TONES) if key == SWEEP else len(LADDER_AMPS) if key == LADDER else len(PULSE_AMPS)


def steps_in_band(key, label):
  """Number of steps captured in a band (the first ones of the table)."""
  if key == LADDER:
    return sum(1 for a in LADDER_AMPS if a <= LADDER_CAPS[label])
  return n_steps(key)


def step_params(key, step):
  """(frequency Hz or None, peak counts, chunk frames, lead-in frames)."""
  if key == SWEEP:
    f, a = SWEEP_TONES[step]
    return f, a, SWEEP_CHUNK, SWEEP_LEAD_IN
  if key == LADDER:
    return LADDER_FREQ, LADDER_AMPS[step], LADDER_CHUNK, LADDER_LEAD_IN
  return None, PULSE_AMPS[step], PULSE_PERIOD, 0


def doublet(amp):
  """The pulse shape (float counts per frame): +lobe then its exact negative; starts and ends at zero."""
  r = int(math.ceil(amp / PULSE_RAMP))
  half = [amp * k / r for k in range(1, r + 1)] + [amp] * PULSE_FLAT + [amp * (r - k) / r for k in range(1, r + 1)]
  return half + [-x for x in half]


_DOUBLETS = {a: doublet(a) for a in PULSE_AMPS}


def unit(key, step, ph, sign=1):
  """The full-amplitude signal (float counts) of a step at chunk phase `ph` (frames; negative: lead-in). Sweep and
  ladder: antisymmetric about the chunk's middle by construction, so a whole chunk sums to exactly zero."""
  f, a, n, _ = step_params(key, step)
  if f is None:
    shape = _DOUBLETS[a]
    return sign * shape[ph] if 0 <= ph < len(shape) else 0.0
  if 0 <= ph < n and 2 * ph > n:
    return -a * math.sin(2.0 * math.pi * f * (n - ph) * DT)
  return a * math.sin(2.0 * math.pi * f * ph * DT)


def gain(env):
  return 0.5 - 0.5 * math.cos(math.pi * env)


def value(key, step, ph, env, sign=1):
  return int(round(gain(env) * unit(key, step, ph, sign)))


def band_for(v_ego):
  mph = v_ego * MS_TO_MPH
  for low, high, label, ti_alone in BANDS:
    if low <= mph < high:
      return label, ti_alone
  return None, None


def peak_counts(key):
  return max(step_params(key, s)[1] for s in range(n_steps(key)))


def rate_room(limits, key):
  """The smallest per-frame step the live TI limiter allows where this test works (as ti_dither.rate_room)."""
  up = limits.TI_STEER_DELTA_UP
  knee = getattr(limits, 'TI_STEER_DELTA_UP_KNEE', limits.TI_STEER_MAX)
  if knee <= MAX_BASE_FRACTION * limits.TI_STEER_MAX + peak_counts(key):
    up = min(up, getattr(limits, 'TI_STEER_DELTA_UP_HIGH', up))
  return min(up, limits.TI_STEER_DELTA_DOWN)


def rate_room_ok(room):
  return room >= RATE_ROOM_FACTOR * MAX_SLEW


def check_definition(key, bins, steps, target):
  """None when a step_seconds definition for `key` matches this module, else the reason (ti_tests validation)."""
  if key not in KEYS:
    return f"setting {key} does not report steps"
  if tuple((lo, hi, lab) for lo, hi, lab, _ in bins) != tuple((lo, hi, lab) for lo, hi, lab, _ in BANDS):
    return f"bins must be the {key} bands " + ", ".join(b[2] for b in BANDS)
  if tuple(steps) != tuple(steps_in_band(key, b[2]) for b in BANDS):
    return f"steps must be {[steps_in_band(key, b[2]) for b in BANDS]}"
  if target != STEP_SECONDS[key]:
    return f"target_seconds must be {STEP_SECONDS[key]:g}"
  return None


class TiIdent:
  """The three tests' generator. `update` runs every frame while lateral control is active and the TI is in RUN
  (else `reset`), and returns the integer counts to add to this frame's TI request."""

  def __init__(self, touch_confirm_frames=TOUCH_CONFIRM_FRAMES, log=None):
    self.touch_confirm_frames = touch_confirm_frames
    self.log = log               # callable(dict) for the events; None: kept in self.events only
    self.events = []             # the latest events (bounded), for tests and the replay
    self.loud = 0
    self.touch = False
    self.holdoff = 0
    self.regime = None           # last LKAS_BLOCK value seen
    self.regime_frames = 0       # frames it has been unchanged
    self.reason = "off"
    self.credit = None           # (test key, band, step, seconds) credited on this frame, else None
    self.frames = {}             # test key -> frames with a non-zero output (stats stamp)
    self.emitted = None          # the test whose waveform produced this frame's output (even if it rounded to 0)
    self._stop()

  def _stop(self):
    self.test = None             # the test whose waveform is running (fading included)
    self.step = 0
    self.ph = 0
    self.env = 0.0
    self.fading = False
    self.sign = 1
    self.band = None             # the band of the current chunk
    self.clean = False           # every frame of the current chunk so far was clean
    self.last = 0

  def _event(self, frame, kind, **kw):
    e = {"frame": frame, "kind": kind, **kw}
    self.events.append(e)
    if len(self.events) > 2000:
      del self.events[:1000]
    if self.log is not None:
      try:
        self.log(e)
      except Exception:
        pass

  def reset(self, frame=None):
    """Lateral control off or TI not in RUN: zero at once."""
    if self.test is not None and frame is not None:
      self._event(frame, "stop", mode="zero")
    self._stop()
    self.loud = 0
    self.touch = False
    self.credit = None
    self.emitted = None

  @property
  def running(self):
    return self.test is not None

  def _next_step(self, key, band, remaining, credit=None):
    """The first step of the table still to capture in `band`, counting this frame's credit; None: band complete."""
    for s in range(steps_in_band(key, band)):
      left = remaining(key, band, s)
      if credit is not None and credit[:3] == (key, band, s):
        left -= credit[3]
      if left > 1e-6:
        return s
    return None

  def _start_chunk(self, frame, key, band, step, lead_in):
    _, a, _, lead = step_params(key, step)
    if key == PULSES and self.test == key:
      self.sign = -self.sign
    self.test, self.step, self.band, self.fading = key, step, band, False
    self.ph = -lead if lead_in else 0
    self.env = 0.0 if lead_in and lead else 1.0
    self.clean = True
    self._event(frame, "run" if lead_in else "chunk", test=key, step=step, band=band, ph=self.ph, env=self.env,
                sign=self.sign, amp=a)

  def update(self, armed, frame, *, v_ego, steering_pressed, driver_torque, blinker, base_request, ti_max,
             lat_demand, curvature, lkas_blocked, room=None, remaining=None):
    """`armed`: the one test key to run (None: none). `room`: rate_room of the live limits for the armed test.
    `remaining(key, band, step)`: seconds still to capture for that step in that band (the accountant's progress;
    None: nothing captured yet)."""
    if remaining is None:
      remaining = lambda k, b, s: STEP_SECONDS[k]
    self.credit = None
    magnitude = abs(driver_torque)
    self.loud = self.loud + 1 if magnitude >= DRIVER_QUIET_COUNTS else 0
    touch = self.touch = bool(steering_pressed or magnitude >= DRIVER_FIRM_COUNTS
                              or self.loud >= self.touch_confirm_frames)
    if touch or blinker:
      self.holdoff = int(round(HOLDOFF_S / DT))
    elif self.holdoff > 0:
      self.holdoff -= 1
    if lkas_blocked is None or lkas_blocked != self.regime:
      self.regime, self.regime_frames = lkas_blocked, 0
    else:
      self.regime_frames += 1

    # A chunk that ended on the previous frame is credited first (its frames were 0..n-1), whatever happens now
    if self.test is not None and not self.fading and self.ph >= step_params(self.test, self.step)[2] and self.clean:
      self.credit = (self.test, self.band, self.step, round(step_params(self.test, self.step)[2] * DT, 2))

    band, ti_alone = band_for(v_ego)
    key = armed if armed in KEYS else None
    # Why not (display), and the gate, in the dither's order of precedence
    if key is None:
      reason = "off"
    elif touch:
      reason = "touch"
    elif room is not None and not rate_room_ok(room):
      reason = "rate"
    elif blinker:
      reason = "blinker"
    elif self.holdoff > 0:
      reason = "holdoff"
    elif band is None:
      reason = "band"
    elif lkas_blocked is None or bool(lkas_blocked) != ti_alone or self.regime_frames < REGIME_SETTLE_S / DT:
      reason = "regime"
    elif abs(base_request) > MAX_BASE_FRACTION * ti_max:
      reason = "command"
    elif abs(lat_demand) > MAX_LAT_DEMAND_MPS2 or abs(curvature) > MAX_CURVATURE:
      reason = "curve"
    elif self._next_step(key, band, remaining, self.credit) is None:
      reason = "band_done"
    elif key == LADDER and self.test == key and step_params(key, self.step)[1] > LADDER_CAPS[band]:
      reason = "band"             # the band changed under a step above the new band's cap
    else:
      reason = "ok"
    self.reason = reason
    open_ = reason == "ok"

    if self.test is not None and key is not None and reason in ("touch", "rate"):
      self._event(frame, "stop", mode="zero")      # a touch or too little rate room: zero at once
      self._stop()
    elif self.test is not None and key is None and touch:
      self._event(frame, "stop", mode="zero")
      self._stop()
    if self.test is not None and not self.fading and (not open_ or key != self.test):
      self.fading = True
      self.clean = False
      self._event(frame, "stop", mode="fade", env=self.env)

    if self.test is None:
      if open_:
        self._start_chunk(frame, key, band, self._next_step(key, band, remaining), lead_in=True)
    elif not self.fading:
      if self.ph >= step_params(self.test, self.step)[2]:
        # The next chunk at once; the gate is open, so the band has a step left
        self._start_chunk(frame, key, band, self._next_step(key, band, remaining, self.credit), lead_in=False)
      elif self.ph >= 0 and (band != self.band or self.env < 1.0):
        self.clean = False

    if self.test is None:
      self.last = 0
      self.emitted = None
      return 0
    self.emitted = self.test
    if self.fading and self.ph >= step_params(self.test, self.step)[2]:
      self.ph = 0                 # the waveform keeps going (whole cycles, next pulse) while it fades
      if self.test == PULSES:
        self.sign = -self.sign
    self.last = value(self.test, self.step, self.ph, self.env, self.sign)
    if self.last != 0:
      self.frames[self.test] = self.frames.get(self.test, 0) + 1
    # Advance to the next frame
    self.ph += 1
    lead = step_params(self.test, self.step)[3]
    if self.fading:
      self.env = max(0.0, self.env - DT / FADE_OUT_S)
      if self.env <= 0.0:
        self._stop()
    elif self.ph <= 0 and lead:
      self.env = (self.ph + lead) / lead
    else:
      self.env = 1.0
    return self.last


def reconstruct(events, n_frames):
  """The added counts of frames 0..n_frames-1 from the logged events alone (the same state machine, no inputs)."""
  by_frame = {}
  for e in events:
    by_frame.setdefault(e["frame"], []).append(e)
  out = []
  test = None
  step = ph = sign = 0
  env, fading = 0.0, False
  for k in range(n_frames):
    for e in by_frame.get(k, ()):
      if e["kind"] in ("run", "chunk"):
        test, step, ph, env, sign, fading = e["test"], e["step"], e["ph"], e["env"], e["sign"], False
      elif e["kind"] == "stop" and e["mode"] == "zero":
        test = None
      elif e["kind"] == "stop" and test is not None and not fading:
        fading = True
        if e.get("env") is not None:
          env = e["env"]
    if test is None:
      out.append(0)
      continue
    n, lead = step_params(test, step)[2], step_params(test, step)[3]
    if fading and ph >= n:
      ph = 0
      if test == PULSES:
        sign = -sign
    out.append(value(test, step, ph, env, sign))
    ph += 1
    if fading:
      env = max(0.0, env - DT / FADE_OUT_S)
      if env <= 0.0:
        test = None
    elif ph <= 0 and lead:
      env = (ph + lead) / lead
    else:
      env = 1.0
  return out
