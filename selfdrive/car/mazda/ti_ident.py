"""Steering identification tests on the Mazda torque-interceptor (TI) command: frequency sweep, amplitude ladder and
short pulses. OFF unless one of the toggles TiSweep / TiLadder / TiPulses, or the run-all toggle TiIdentAll, is on
(Tests screen, default off, off at ignition off).

Same principle and the same safety envelope as the identification dither (ti_dither.py), which stays exactly as it
was: a small, known, zero-mean, always-moving signal is added to the TI request ONLY, BEFORE
apply_ti_steer_torque_limits, so the magnitude, driver-torque and rate limits bound the total exactly as before. The
stock command, CAN definitions and panda safety are untouched. At most one signal runs at a time: these tests run only
while the dither toggle is off and the dither is silent, and one generator runs one test at a time.

Signal. Each test is a table of STEPS. Progress is credited per UNIT: for sweep and ladder the smallest whole number
of periods of the tone that is at least UNIT_MIN_PERIODS periods and UNIT_MIN_S long, for pulses one doublet with its
settling gap. A unit counts only when every one of its frames was clean (gate open, full amplitude, one speed band);
a pause loses only the unit in progress, and the same step resumes afterwards and keeps what it has. The step to run
is chosen at every unit start: the first step of the table that is not complete IN THE BAND THE CAR IS IN NOW, so
mixed-speed driving fills every band. When every step of the band is complete the test keeps running there and
collects EXTRA, least-collected step first; completion is judged on the required amounts only. Units of the same step
follow each other with no gap; between two steps of the sweep or ladder the old tone fades out and the new one fades
in (about 0.5-2 s); pulses follow at once.

Run all (TiIdentAll). `schedule` picks the test: in the band the car is in, RUN_ALL_ORDER (sweep, pulses, ladder:
smallest signals first), staying on the current test until its requirement for this band is complete; after a band
change, the first test not complete in the new band; with every test complete in the band, extra, least-collected
test first (with hysteresis). A test change fades the old signal out completely, then SWITCH_QUIET_S of quiet, then
the next one starts.

Heading. Each sweep and ladder tone starts at the phase (PHASES) that makes the heading it produces zero-mean over a
unit, its lead-in included, so the car does not drift sideways (a sine from zero gives a one-sided heading and a
steady drift); each period has an even number of frames, so a unit still sums to exactly zero. Units of a step that
follow each other start where the last one ended (whole periods), so every unit has the same zero-mean heading.
Pulse doublets alternate sign, so each pair leaves no heading.

Gate. A driver touch (steeringPressed, |driver torque| >= 6 counts, or >= 4 counts for 3 frames) or too little rate
room cut the signal to zero AT ONCE. Everything else that closes the gate FADES the signal out: blinker or lane
change, the hold-off after a touch or a blinker, a bend, a large command, leaving the speed bands, a steering-mode
change, a test change, and the toggle switched off. A fade lasts fade_frames(step): long enough that the signal never
changes by more than MAX_SLEW counts per frame; a pulse in progress finishes its doublet instead. The thresholds are
the named constants below; steps of at most SMALL_AMP counts (the dither's size) get looser bend, command and touch
hold-off limits, the larger steps keep the strict ones. Also:
  - the speed is inside one of BANDS (19-28, 31-45, 45-60, 60-80 mph); in the 28-31 mph gap the logged regime decides
    which of the two neighbouring bands a sample belongs to;
  - the steering regime is the band's: below 28 mph the EPS blocks the stock lane-keep path and the TI steers alone
    (LKAS_BLOCK set); from 31 mph both paths steer (LKAS_BLOCK clear); it must have been unchanged REGIME_SETTLE_S.

Reconstruction. The requested added counts are exact from logs, as for the dither:
  added = mazdaDiagnostics.tiRequested - round(carControl.actuators.steer * tiMax)
(the dither is zero whenever these run). The regime of every sample is frogpilotCarState.lkasBlocked. Which test,
step, band and phase produced it: the generator logs an event (cloudlog "ti_ident", JSON) at every run start, unit
start and stop; `reconstruct(events, n)` rebuilds the added counts frame by frame from those events alone (exactly;
test_mazda_ti_ident.py checks it). The run's stats config carries {"TiIdent": {test: {"version", "table", "frames",
"touch_confirm", "frames_by": {band: {reason: frames}}}, "run_all": bool}}; `table` is TABLE_HASH[test].
"""
import hashlib
import json
import math

from openpilot.selfdrive.car.mazda.ti_dither import (DRIVER_FIRM_COUNTS, DRIVER_QUIET_COUNTS, FADE_OUT_S, HOLDOFF_S,
                                                     MAX_BASE_FRACTION, MAX_LAT_DEMAND_MPS2, RATE_ROOM_FACTOR,
                                                     TOUCH_CONFIRM_FRAMES)

VERSION = 3
DT = 0.01
MS_TO_MPH = 1.0 / 0.44704

SWEEP, LADDER, PULSES = "TiSweep", "TiLadder", "TiPulses"
KEYS = (SWEEP, LADDER, PULSES)
TOGGLES = {SWEEP: "ti_sweep", LADDER: "ti_ladder", PULSES: "ti_pulses"}
RUN_ALL_KEY, RUN_ALL_TOGGLE = "TiIdentAll", "ti_ident_all"

# (low mph, high mph, label, TI alone): half-open [low, high). Between the first two bands (28-31 mph) the stock path
# switches (blocked below 45 km/h, cleared above 50 km/h with a lag); samples there count by the logged regime. The
# lowest band starts at 19 mph: the car only lets cruise control (and so openpilot) engage from about 19 mph.
BANDS = ((19.0, 28.0, "19-28", True), (31.0, 45.0, "31-45", False), (45.0, 60.0, "45-60", False),
         (60.0, 80.0, "60-80", False))

MAX_SLEW = 3.0              # counts per frame, lead-in and fades included: half of the default TI_STEER_DELTA_UP (6)

# ---- gates (named thresholds) ------------------------------------------------------------------------------------
# A step of at most SMALL_AMP counts (the dither's peak) is "small": the dither ran at that size with only the
# lateral-demand gate and measured <= 0.006 m/s^2 and <= 0.12 mm sideways at its tones. Larger steps (ladder 37 and
# 55 counts, the 44-count pulse) keep the strict limits.
SMALL_AMP = 22.0
# Bends: keep the larger signals off where the controller is busy; the analysis fits the response relative to the
# base command, so a steady bend does not spoil a small-signal measurement.
MAX_CURVATURE = 0.004                  # 1/m (radius 250 m), larger steps
MAX_CURVATURE_SMALL = 0.01             # 1/m (radius 100 m), small steps
MAX_LAT_DEMAND = MAX_LAT_DEMAND_MPS2   # v^2 |desired curvature|, m/s^2 (the dither's 1.0), larger steps
MAX_LAT_DEMAND_SMALL = 1.5             # small steps
# Base command: stay clear of the 600 clip (a clipped signal leaves a bias; a flat command at the clip stalls the rack)
MAX_BASE_FRACTION_STRICT = MAX_BASE_FRACTION   # |undithered request| <= 0.5 x TI_STEER_MAX (300), larger steps
MAX_BASE_FRACTION_SMALL = 0.75                 # 450 + 22 leaves 128 counts to the clip, small steps
# Hold-off after a confirmed touch (the driver may still be steering); a blinker keeps the full time (lane change)
TOUCH_HOLDOFF_S = HOLDOFF_S            # 3 s, larger steps
TOUCH_HOLDOFF_SMALL_S = 1.5            # small steps
BLINKER_HOLDOFF_S = HOLDOFF_S          # 3 s, all steps
# Steering-mode settle: a sample belongs to one regime; the regime is the logged LKAS_BLOCK flag, so this only lets
# the response settle after a change. Measurement only.
REGIME_SETTLE_S = 1.0
# Rate room: the live limiter's smallest step must be >= RATE_ROOM_FACTOR (2) x the step's own largest slew
# (STEP_SLEW), the dither's rule; with less, rate limiting clips the signal and leaves a bias.

# ---- credited units ----------------------------------------------------------------------------------------------
UNIT_MIN_PERIODS = 2
UNIT_MIN_S = 2.0

# Sweep: one tone per step, (period in frames, even; peak TI counts): 0.30, 0.60, 1.0, 1.6, 2.4, 3.6 and 5.0 Hz.
# Amplitudes in the dither's class (its preset 1 peaks at 22 counts, 12 at its 1.1 Hz tone); the high tones are
# reduced so the slew stays under MAX_SLEW.
SWEEP_TONES = ((334, 12.0), (166, 12.0), (100, 16.0), (62, 16.0), (42, 16.0), (28, 12.0), (20, 9.0))
SWEEP_LEAD_IN = 50

# Ladder: one frequency (0.4 Hz, a 250-frame period), stepped amplitudes (TI counts; 1 count ~ 0.0544 EPS sensor
# counts): 0.3 to 5.0 sensor counts, from below the 1.2 sc at which the dither already moved the wheel to above the
# rack's friction (2.5-4 sc). Per band only the steps up to the band's cap run. The 19-28 mph cap is 55 counts until a
# drive has shown the measured wheel angle at 37 and 55 counts and no 0x25 fault; LADDER_LOW_BAND_FULL_CAP (92) then
# brings back the 74- and 92-count steps (and the ti_tests.json steps for that band go from 5 to 7).
LADDER_PERIOD = 250
LADDER_AMPS = (6.0, 11.0, 22.0, 37.0, 55.0, 74.0, 92.0)
LADDER_LOW_BAND_CAP = 55.0
LADDER_LOW_BAND_FULL_CAP = 92.0
LADDER_CAPS = {"19-28": LADDER_LOW_BAND_CAP, "31-45": 37.0, "45-60": 37.0, "60-80": 37.0}
LADDER_LEAD_IN = 100

# Pulses: a bipolar trapezoid doublet (+A then -A, ramps at PULSE_RAMP counts per frame, PULSE_FLAT frames flat),
# then quiet to the end of a PULSE_PERIOD-frame unit for the settling; the doublet's sign alternates pulse by pulse.
PULSE_AMPS = (22.0, 44.0)
PULSE_PERIOD = 200
PULSE_RAMP = 2.75
PULSE_FLAT = 10

STEP_SECONDS = {SWEEP: 20.0, LADDER: 10.0, PULSES: 40.0}

# Run all
RUN_ALL_ORDER = (SWEEP, PULSES, LADDER)
SWITCH_QUIET_S = 0.5        # quiet between one test's complete fade-out and the next test's start
EXTRA_HYSTERESIS = 0.25     # extra collection: leave the current test once it leads the least-collected by this share


def unit_periods(p):
  return max(UNIT_MIN_PERIODS, int(math.ceil(UNIT_MIN_S / (p * DT) - 1e-9)))


def _table(key):
  if key == SWEEP:
    return {"tones": SWEEP_TONES, "lead_in": SWEEP_LEAD_IN, "unit": [UNIT_MIN_PERIODS, UNIT_MIN_S]}
  if key == LADDER:
    return {"period": LADDER_PERIOD, "amps": LADDER_AMPS, "caps": LADDER_CAPS, "lead_in": LADDER_LEAD_IN,
            "unit": [UNIT_MIN_PERIODS, UNIT_MIN_S]}
  return {"amps": PULSE_AMPS, "period": PULSE_PERIOD, "ramp": PULSE_RAMP, "flat": PULSE_FLAT}


TABLE_HASH = {k: hashlib.sha1(json.dumps([VERSION, _table(k), STEP_SECONDS[k], BANDS, MAX_SLEW],
                                         sort_keys=True).encode()).hexdigest()[:12] for k in KEYS}


def n_steps(key):
  return len(SWEEP_TONES) if key == SWEEP else len(LADDER_AMPS) if key == LADDER else len(PULSE_AMPS)


def steps_in_band(key, label):
  """Number of steps captured in a band (the first ones of the table)."""
  if key == LADDER:
    return sum(1 for a in LADDER_AMPS if a <= LADDER_CAPS[label])
  return n_steps(key)


def step_params(key, step):
  """(period frames or None for pulses, peak counts, unit frames, lead-in frames)."""
  if key == SWEEP:
    p, a = SWEEP_TONES[step]
    return p, a, p * unit_periods(p), SWEEP_LEAD_IN
  if key == LADDER:
    return LADDER_PERIOD, LADDER_AMPS[step], LADDER_PERIOD * unit_periods(LADDER_PERIOD), LADDER_LEAD_IN
  return None, PULSE_AMPS[step], PULSE_PERIOD, 0


def units_per_step(key, step):
  """Units that make up one step's STEP_SECONDS (each credited as its nominal share)."""
  return max(1, int(round(STEP_SECONDS[key] / (step_params(key, step)[2] * DT))))


def unit_credit(key, step):
  return STEP_SECONDS[key] / units_per_step(key, step)


def small(key, step):
  return step_params(key, step)[1] <= SMALL_AMP


def freq(key, step):
  p = step_params(key, step)[0]
  return None if p is None else 1.0 / (p * DT)


def gain(env):
  return 0.5 - 0.5 * math.cos(math.pi * env)


def fade_frames(key, step):
  """Frames of a fade-out: at least FADE_OUT_S, and long enough that the raised-cosine fade of the tone never moves the
  signal by more than MAX_SLEW counts per frame (bound: A * DT * sqrt((pi / 2F)^2 + w^2) <= MAX_SLEW, with 10 %
  margin). Pulses do not fade: a doublet in progress finishes, see TiIdent."""
  p, a, _, _ = step_params(key, step)
  base = int(round(FADE_OUT_S / DT))
  if p is None:
    return base
  w = 2.0 * math.pi / (p * DT)
  room = (MAX_SLEW / (a * DT)) ** 2 - w ** 2
  need = math.pi / (2.0 * math.sqrt(room)) / DT * 1.1
  return max(base, int(math.ceil(need)))


def doublet(amp):
  """The pulse shape (float counts per frame): +lobe then its exact negative; starts and ends at zero."""
  r = int(math.ceil(amp / PULSE_RAMP))
  half = [amp * k / r for k in range(1, r + 1)] + [amp] * PULSE_FLAT + [amp * (r - k) / r for k in range(1, r + 1)]
  return half + [-x for x in half]


_DOUBLETS = {a: doublet(a) for a in PULSE_AMPS}


def _tone(p, a, phi, ph):
  """a * sin(2 pi ph / p + phi), built so that the second half of every period is the exact negative of the first
  (each period, and so each unit, sums to exactly zero after rounding)."""
  j = ph % p
  half = p // 2
  if j >= half:
    return -a * math.sin(2.0 * math.pi * (j - half) / p + phi)
  return a * math.sin(2.0 * math.pi * j / p + phi)


def _zero_heading_phase(key, step):
  """The tone's starting phase that makes the heading it produces zero-mean over a unit, the lead-in included: the
  heading is the running sum of the signal, and its unit mean is linear in (cos phi, sin phi), so phi follows from
  two sums. A sine from zero (phi = 0) would give a one-sided heading and a steady sideways drift."""
  p, a, n, lead = step_params(key, step)
  sums = []
  for phi in (0.0, math.pi / 2):
    heading, total = 0.0, 0.0
    for ph in range(-lead, n):
      g = gain((ph + lead) / lead) if ph < 0 and lead else 1.0
      heading += g * _tone(p, 1.0, phi, ph)
      if ph >= 0:
        total += heading
    sums.append(total)
  return math.atan2(-sums[0], sums[1])


PHASES = {(k, s): _zero_heading_phase(k, s) for k in (SWEEP, LADDER) for s in range(n_steps(k))}


def unit(key, step, ph, sign=1):
  """The full-amplitude signal (float counts) of a step at unit phase `ph` (frames; negative: lead-in). Sweep and
  ladder: a tone at the phase PHASES gives it (zero-mean heading); pulses: the doublet, then quiet."""
  p, a, _, _ = step_params(key, step)
  if p is None:
    shape = _DOUBLETS[a]
    return sign * shape[ph] if 0 <= ph < len(shape) else 0.0
  return _tone(p, a, PHASES[(key, step)], ph)


def value(key, step, ph, env, sign=1):
  return int(round(gain(env) * unit(key, step, ph, sign)))


def _step_slew(key, step):
  """The largest per-frame change of the step's signal (lead-in, two units, and a fade from any point)."""
  p, a, n, lead = step_params(key, step)
  if p is None:
    shape = [0.0] + _DOUBLETS[a] + [0.0]
    return max(abs(y - x) for x, y in zip(shape, shape[1:]))
  x = [unit(key, step, ph) * (gain((ph + lead) / lead) if ph < 0 and lead else 1.0) for ph in range(-lead, 2 * n)]
  worst = max(abs(y - v) for v, y in zip(x, x[1:]))
  f = fade_frames(key, step)
  for start in range(0, n, max(1, n // 23)):
    seq = [gain(max(0.0, 1.0 - i / f)) * unit(key, step, start + i) for i in range(f + 1)]
    worst = max(worst, max(abs(y - v) for v, y in zip(seq, seq[1:])))
  return worst


STEP_SLEW = {(k, s): _step_slew(k, s) for k in KEYS for s in range(n_steps(k))}


def band_for(v_ego, lkas_blocked=None):
  """(band label, TI alone) for a speed. In the 28-31 mph gap between the TI-alone band and the next one the logged
  regime decides: stock path blocked counts in the band below, stock path active in the band above."""
  mph = v_ego * MS_TO_MPH
  for low, high, label, ti_alone in BANDS:
    if low <= mph < high:
      return label, ti_alone
  lo_band, hi_band = BANDS[0], BANDS[1]
  if lkas_blocked is not None and lo_band[1] <= mph < hi_band[0]:
    return (lo_band[2], True) if lkas_blocked else (hi_band[2], False)
  return None, None


def peak_counts(key):
  return max(step_params(key, s)[1] for s in range(n_steps(key)))


def rate_room(limits, key):
  """The smallest per-frame step the live TI limiter allows where this test works (as ti_dither.rate_room)."""
  up = limits.TI_STEER_DELTA_UP
  knee = getattr(limits, 'TI_STEER_DELTA_UP_KNEE', limits.TI_STEER_MAX)
  if knee <= MAX_BASE_FRACTION_SMALL * limits.TI_STEER_MAX + peak_counts(key):
    up = min(up, getattr(limits, 'TI_STEER_DELTA_UP_HIGH', up))
  return min(up, limits.TI_STEER_DELTA_DOWN)


def rate_room_ok(room, key=None, step=None):
  """Room for the step's own largest slew (lead-in and fade included); without a step, for MAX_SLEW."""
  need = MAX_SLEW if key is None else STEP_SLEW[(key, step)]
  return room >= RATE_ROOM_FACTOR * need


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


def band_share(key, band, remaining):
  """Collected / required for one test in one band, extra included (> 1 when it has extra); None when unknown."""
  required, collected = 0.0, 0.0
  for s in range(steps_in_band(key, band)):
    left = remaining(key, band, s)
    if left is None:
      return None
    required += STEP_SECONDS[key]
    collected += STEP_SECONDS[key] - left
  return collected / required


def schedule(current, current_band, band, remaining):
  """Run all: the test to arm now. `current`: the test armed so far; `current_band`: the band it was armed for;
  `band`: the band now (None: keep the current test; its own gate pauses it). None when progress is unknown."""
  if band is None:
    return current
  shares = {}
  for k in RUN_ALL_ORDER:
    shares[k] = band_share(k, band, remaining)
    if shares[k] is None:
      return None
  if current in shares and band == current_band and shares[current] < 1.0 - 1e-9:
    return current                       # stay until its requirement for this band is complete
  for k in RUN_ALL_ORDER:
    if shares[k] < 1.0 - 1e-9:
      return k                           # the first test not complete in this band
  least = min(RUN_ALL_ORDER, key=lambda k: (shares[k], RUN_ALL_ORDER.index(k)))
  if current in shares and band == current_band and shares[current] - shares[least] < EXTRA_HYSTERESIS:
    return current                       # extra: keep going until it leads by the hysteresis
  return least


def _advance(st):
  """One frame of the state machine shared by TiIdent and reconstruct: st = dict(test, step, ph, env, fading)."""
  test, step = st["test"], st["step"]
  p, _, n, lead = step_params(test, step)
  st["ph"] += 1
  if st["fading"]:
    if p is None:
      if st["ph"] >= len(_DOUBLETS[PULSE_AMPS[step]]):
        st["test"] = None           # the doublet is finished: stopped
    else:
      st["env"] = max(0.0, st["env"] - 1.0 / fade_frames(test, step))
      if st["env"] <= 0.0:
        st["test"] = None
  elif st["ph"] <= 0 and lead:
    st["env"] = (st["ph"] + lead) / lead
  else:
    st["env"] = 1.0


def _wrap(st):
  """While fading, the tone keeps going past the unit end (it is continuous there)."""
  p, _, n, _ = step_params(st["test"], st["step"])
  if p is not None and st["fading"] and st["ph"] >= n:   # (a pulse never wraps: it stops after its doublet)
    st["ph"] -= n


class TiIdent:
  """The three tests' generator. `update` runs every frame while lateral control is active and the TI is in RUN
  (else `reset`), and returns the integer counts to add to this frame's TI request."""

  def __init__(self, touch_confirm_frames=TOUCH_CONFIRM_FRAMES, log=None):
    self.touch_confirm_frames = touch_confirm_frames
    self.log = log               # callable(dict) for the events; None: kept in self.events only
    self.events = []             # the latest events (bounded), for tests and the replay
    self.loud = 0
    self.touch = False
    self.touch_age = 10 ** 6     # frames since the last confirmed touch
    self.blink_age = 10 ** 6     # frames since the last blinker
    self.regime = None           # last LKAS_BLOCK value seen
    self.regime_frames = 0       # frames it has been unchanged
    self.reason = "off"
    self.extra = False           # the running unit is extra collection in a complete band
    self.credit = None           # (test key, band, step, seconds) credited on this frame, else None
    self.frames = {}             # test key -> frames with a non-zero output (stats stamp)
    self.counts = {}             # (test key, band or "none", reason) -> frames armed (stats stamp; reset per run)
    self.emitted = None          # the test whose waveform produced this frame's output (even if it rounded to 0)
    self.prev_test = None        # the test that ran last, and frames since it stopped (the quiet between tests)
    self.stopped_age = 10 ** 6
    self._stop()

  def _stop(self):
    if getattr(self, "test", None) is not None:
      self.prev_test, self.stopped_age = self.test, 0
    self.test = None             # the test whose waveform is running (fading included)
    self.step = 0
    self.ph = 0
    self.env = 0.0
    self.fading = False
    self.sign = 1
    self.band = None             # the band of the current unit
    self.clean = False           # every frame of the current unit so far was clean
    self.last = 0

  @property
  def holdoff(self):
    """Frames of the strict (3 s) hold-off still to run (display / compatibility)."""
    return max(0, int(round(TOUCH_HOLDOFF_S / DT)) - min(self.touch_age, self.blink_age))

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

  def _pick(self, key, band, remaining, credit=None):
    """(step, extra): the first step of the table still to capture in `band` (this frame's credit counted), else the
    least-collected one as extra (ties: the first); (None, False) when progress is unknown."""
    best, best_left = None, None
    for s in range(steps_in_band(key, band)):
      left = remaining(key, band, s)
      if left is None:
        return None, False
      if credit is not None and credit[:3] == (key, band, s):
        left -= credit[3]
      if left > 1e-6:
        return s, False
      if best_left is None or left > best_left + 1e-9:
        best, best_left = s, left
    return best, True

  def _start_chunk(self, frame, key, band, step, lead_in, extra):
    _, a, _, lead = step_params(key, step)
    if key == PULSES and self.test == key:
      self.sign = -self.sign
    self.test, self.step, self.band, self.fading, self.extra = key, step, band, False, extra
    self.ph = -lead if lead_in else 0
    self.env = 0.0 if lead_in and lead else 1.0
    self.clean = True
    self._event(frame, "run" if lead_in else "chunk", test=key, step=step, band=band, ph=self.ph, env=self.env,
                sign=self.sign, amp=a, extra=extra)

  def _fade(self, frame, why):
    if self.test == PULSES and self.ph >= len(_DOUBLETS[PULSE_AMPS[self.step]]):
      self._event(frame, "stop", mode="zero", why=why)   # between doublets: the signal is already zero
      self._stop()
      return
    self.fading = True
    self.clean = False
    self._event(frame, "stop", mode="fade", env=self.env, why=why)

  def update(self, armed, frame, *, v_ego, steering_pressed, driver_torque, blinker, base_request, ti_max,
             lat_demand, curvature, lkas_blocked, room=None, remaining=None):
    """`armed`: the one test key to run (None: none). `room`: rate_room of the live limits for the armed test.
    `remaining(key, band, step)`: seconds still to capture for that step in that band, negative when it has extra
    (the accountant's progress; None from it: unknown, nothing runs; None here: nothing captured yet)."""
    if remaining is None:
      remaining = lambda k, b, s: STEP_SECONDS[k]
    self.credit = None
    self.stopped_age += 1
    magnitude = abs(driver_torque)
    self.loud = self.loud + 1 if magnitude >= DRIVER_QUIET_COUNTS else 0
    touch = self.touch = bool(steering_pressed or magnitude >= DRIVER_FIRM_COUNTS
                              or self.loud >= self.touch_confirm_frames)
    self.touch_age = 0 if touch else self.touch_age + 1
    self.blink_age = 0 if blinker else self.blink_age + 1
    if lkas_blocked is None or lkas_blocked != self.regime:
      self.regime, self.regime_frames = lkas_blocked, 0
    else:
      self.regime_frames += 1

    # A unit that ended on the previous frame is credited first (its frames were 0..n-1), whatever happens now
    if self.test is not None and not self.fading and self.ph >= step_params(self.test, self.step)[2] and self.clean:
      self.credit = (self.test, self.band, self.step, unit_credit(self.test, self.step))

    band, ti_alone = band_for(v_ego, lkas_blocked)
    key = armed if armed in KEYS else None
    # The step whose limits apply: the running one, else the one that would start
    step, extra = (None, False)
    if key is not None and band is not None:
      if self.test == key and not self.fading:
        step, extra = self.step, self.extra
      else:
        step, extra = self._pick(key, band, remaining, self.credit)
    strict = step is None or not small(key, step)
    touch_hold = (TOUCH_HOLDOFF_S if strict else TOUCH_HOLDOFF_SMALL_S) / DT
    # Why not (display), and the gate, in the dither's order of precedence
    if key is None:
      reason = "off"
    elif touch:
      reason = "touch"
    elif room is not None and not rate_room_ok(room, key, step if step is not None else 0):
      reason = "rate"
    elif blinker:
      reason = "blinker"
    elif self.touch_age < touch_hold or self.blink_age < BLINKER_HOLDOFF_S / DT:
      reason = "holdoff"
    elif band is None:
      reason = "band"
    elif lkas_blocked is None or bool(lkas_blocked) != ti_alone or self.regime_frames < REGIME_SETTLE_S / DT:
      reason = "regime"
    elif abs(base_request) > (MAX_BASE_FRACTION_STRICT if strict else MAX_BASE_FRACTION_SMALL) * ti_max:
      reason = "command"
    elif (abs(lat_demand) > (MAX_LAT_DEMAND if strict else MAX_LAT_DEMAND_SMALL) or
          abs(curvature) > (MAX_CURVATURE if strict else MAX_CURVATURE_SMALL)):
      reason = "curve"
    elif step is None:
      reason = "off"              # progress unknown (no accountant): nothing runs
    elif key == LADDER and self.test == key and step_params(key, self.step)[1] > LADDER_CAPS[band]:
      reason = "band"             # the band changed under a step above the new band's cap
    elif self.test is None and key != self.prev_test and self.stopped_age <= SWITCH_QUIET_S / DT:
      reason = "switch"           # quiet between two tests
    else:
      reason = "extra" if extra else "ok"
    if self.test is not None and key != self.test and reason in ("ok", "extra"):
      reason = "switch"           # another test is next: this one fades out first
    self.reason = reason
    open_ = reason in ("ok", "extra")
    if key is not None:
      slot = (key, band or "none", reason)
      self.counts[slot] = self.counts.get(slot, 0) + 1

    if self.test is not None and (touch or (key is not None and reason == "rate")):
      self._event(frame, "stop", mode="zero")      # a touch or too little rate room: zero at once
      self._stop()
    if self.test is not None and not self.fading and (not open_ or key != self.test):
      self._fade(frame, reason)

    if self.test is None:
      if open_:
        self._start_chunk(frame, key, band, step, lead_in=True, extra=extra)
    elif not self.fading:
      if self.ph >= step_params(self.test, self.step)[2]:
        nxt, nxt_extra = self._pick(key, band, remaining, self.credit)
        if nxt == self.step or self.test == PULSES:
          self._start_chunk(frame, key, band, nxt, lead_in=False, extra=nxt_extra)   # straight on: continuous
        else:
          self._fade(frame, "step")     # another tone or amplitude: fade out, then a fresh lead-in
      elif self.ph >= 0 and (band != self.band or self.env < 1.0):
        self.clean = False

    if self.test is None:
      self.last = 0
      self.emitted = None
      return 0
    self.emitted = self.test
    st = self.__dict__
    _wrap(st)
    self.last = value(self.test, self.step, self.ph, self.env, self.sign)
    if self.last != 0:
      self.frames[self.test] = self.frames.get(self.test, 0) + 1
    _advance(st)
    if self.test is None:
      self.prev_test, self.stopped_age = self.emitted, 0
      self._stop()
    return self.last

  def frames_by(self, key):
    """{band: {reason: frames}} for one test since the counts were last cleared (stats stamp)."""
    out = {}
    for (k, band, reason), n in self.counts.items():
      if k == key:
        out.setdefault(band, {})[reason] = n
    return out

  def share(self, key):
    """Fraction of the armed, engaged frames that were running clean ("ok" or "extra"), or None without any."""
    total = sum(n for (k, _, _), n in self.counts.items() if k == key)
    ok = sum(n for (k, _, r), n in self.counts.items() if k == key and r in ("ok", "extra"))
    return ok / total if total else None


def reconstruct(events, n_frames):
  """The added counts of frames 0..n_frames-1 from the logged events alone (the same state machine, no inputs)."""
  by_frame = {}
  for e in events:
    by_frame.setdefault(e["frame"], []).append(e)
  out = []
  st = dict(test=None, step=0, ph=0, env=0.0, fading=False)
  sign = 1
  for k in range(n_frames):
    for e in by_frame.get(k, ()):
      if e["kind"] in ("run", "chunk"):
        st.update(test=e["test"], step=e["step"], ph=e["ph"], env=e["env"], fading=False)
        sign = e["sign"]
      elif e["kind"] == "stop" and e["mode"] == "zero":
        st["test"] = None
      elif e["kind"] == "stop" and st["test"] is not None and not st["fading"]:
        st["fading"] = True
        st["env"] = e["env"]
    if st["test"] is None:
      out.append(0)
      continue
    _wrap(st)
    out.append(value(st["test"], st["step"], st["ph"], st["env"], sign))
    _advance(st)
  return out
