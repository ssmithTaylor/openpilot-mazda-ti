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

Run all (TiIdentAll). `RunAll.pick` chooses the test: in the window the car is in (or the nearest one when between
windows), RUN_ALL_ORDER (sweep, pulses, ladder: smallest signals first), staying on the current test until its
requirement there is complete; a test that cannot run only because of a command / curve / lateral-demand limit for
longer than BLOCK_PATIENCE_S is passed over for one that can. With every test complete in the window the run goes on:
the test is drawn AT RANDOM among those whose total time in the window is within TIE_S of the least (seeded; the seed,
draw number, totals and candidates are in the logged "pick" event, so `verify_picks` recomputes every choice). A test
change fades the old signal out completely, then SWITCH_QUIET_S of quiet, then the next one starts.

Heading. Each sweep and ladder tone starts at the phase (PHASES) that makes the heading it produces zero-mean over a
unit, its lead-in included, so the car does not drift sideways (a sine from zero gives a one-sided heading and a
steady drift); each period has an even number of frames, so a unit still sums to exactly zero. Units of a step that
follow each other start where the last one ended (whole periods), so every unit has the same zero-mean heading. A
fade-out and the next lead-in leave a few hundred count-frames each; every fresh run takes the tone sign whose
lead-in pulls the running net of all counts sent back toward zero, and extra collection dwells EXTRA_DWELL_UNITS
units per step, so there is no long-run bias. Pulse doublets alternate sign, so each pair leaves no heading.

Gate. A driver touch (steeringPressed, |driver torque| >= 6 counts, or >= 4 counts for 3 frames) or too little rate
room cut the signal to zero AT ONCE. Everything else that closes the gate FADES the signal out: blinker or lane
change, the hold-off after a touch or a blinker, a bend, a large command, leaving the speed bands, a steering-mode
change, a test change, and the toggle switched off. A fade lasts fade_frames(step): long enough that the signal never
changes by more than MAX_SLEW counts per frame; a pulse in progress finishes its doublet instead. The thresholds are
the named constants below; steps of at most SMALL_AMP counts (the dither's size) get looser bend, command and touch
hold-off limits, the larger steps keep the strict ones. Also:
  - the speed is from MIN_TEST_MPH (19) up to MAX_TEST_MPH (80) and not in the 28-31 mph region where the EPS starts to
    allow the stock steering path (reason "transition": off there, the signal fades out);
  - the steering regime is the speed's: below 28 mph the EPS blocks the stock lane-keep path and the TI steers alone
    (LKAS_BLOCK set); from 31 mph both paths steer (LKAS_BLOCK clear); it must have been unchanged REGIME_SETTLE_S.

Windows (revision 3). Progress is credited ONLY while the car is inside one of WINDOWS (21-27, 35-41, 49-56, 64-80
mph); a unit counts only if every frame of it was inside the same window. Between the windows (and 19-21 mph) the tests
still run under every gate, with the amplitude cap, command limit and step choice of the nearest window of the same
steering regime (`Zone.profile`), and the data is logged and tagged with the actual speed (events carry `mph`, `band`
= the window or "gap", `profile`), but ticks no box.

Reconstruction. The requested added counts are exact from logs, as for the dither:
  added = mazdaDiagnostics.tiRequested - round(carControl.actuators.steer * tiMax)
(the dither is zero whenever these run). The regime of every sample is frogpilotCarState.lkasBlocked. Which test,
step, band and phase produced it: the generator logs an event (cloudlog "ti_ident", JSON) at every run start, unit
start and stop; `reconstruct(events, n)` rebuilds the added counts frame by frame from those events alone (exactly;
test_mazda_ti_ident.py checks it). The run's stats config carries {"TiIdent": {test: {"version", "table", "frames",
"touch_confirm", "frames_by": {band: {reason: frames}}}, "run_all": bool}}; `table` is TABLE_HASH[test].
"""
# RUNTIME: runs in card at 100 Hz; gc disabled (see docs/RUNTIME_CONSTRAINTS.md).
# Per frame: create no class or closure, keep no container without a bound; params / JSON at most 10 Hz.
import hashlib
import json
import math
import os
from typing import NamedTuple, Optional

from openpilot.selfdrive.car.mazda.ti_dither import (DRIVER_FIRM_COUNTS, DRIVER_QUIET_COUNTS, FADE_OUT_S, HOLDOFF_S,
                                                     MAX_BASE_FRACTION, MAX_LAT_DEMAND_MPS2, RATE_ROOM_FACTOR,
                                                     TOUCH_CONFIRM_FRAMES)

VERSION = 6
DT = 0.01
MS_TO_MPH = 1.0 / 0.44704

SWEEP, LADDER, PULSES = "TiSweep", "TiLadder", "TiPulses"
KEYS = (SWEEP, LADDER, PULSES)
TOGGLES = {SWEEP: "ti_sweep", LADDER: "ti_ladder", PULSES: "ti_pulses"}
RUN_ALL_KEY, RUN_ALL_TOGGLE = "TiIdentAll", "ti_ident_all"

# Speed windows (low mph, high mph, label, TI alone): half-open [low, high). Progress counts only inside them. The gaps
# between them keep clear of the 28-31 mph region (EPS: blocked below 45 km/h, cleared above 50 km/h with a lag), which
# no window straddles and where no test runs. The lowest window starts above 19 mph: the car only lets cruise control
# (and so openpilot) engage from about 19 mph.
WINDOWS = ((21.0, 27.0, "21-27", True), (35.0, 41.0, "35-41", False), (49.0, 56.0, "49-56", False),
           (64.0, 80.0, "64-80", False))
BANDS = WINDOWS             # the old name
MIN_TEST_MPH = 19.0         # nothing runs below this (no engaged driving there)
EPS_LOW_MPH, EPS_HIGH_MPH = 28.0, 31.0   # the EPS starts to allow the stock path here: no test from 28 up to 31 mph
MAX_TEST_MPH = 80.0
# Speed hysteresis. The tests are switched OFF at once at every edge of an off-region (below 19, 28-31, above 80 mph);
# they switch ON again only once the speed is this far clear of it. Between two windows the profile (whose caps and limits
# apply) changes only once the speed is this far beyond the midpoint of the gap. A car hovering at a boundary so makes no
# fade-and-switch churn and no flood of log events.
ZONE_HYSTERESIS_MPH = 1.0
# At most one "pick" event per this long (the number skipped since the last rides in the next one)
PICK_LOG_MIN_S = 0.5


class Zone(NamedTuple):
  """Where a speed is. state: inside (a window), between (a gap that runs tests), below, transition, above (no test).
  window: the window credited, or None. profile: the window whose caps, limits and step choice apply (the window itself,
  else the nearest of the same steering regime), or None where nothing runs."""
  window: Optional[str]
  profile: Optional[str]
  ti_alone: Optional[bool]
  state: str


def zone_for(v_ego):
  mph = v_ego * MS_TO_MPH
  if mph < MIN_TEST_MPH:
    return Zone(None, None, None, "below")
  if EPS_LOW_MPH <= mph < EPS_HIGH_MPH:
    return Zone(None, None, None, "transition")
  if mph >= MAX_TEST_MPH:
    return Zone(None, None, None, "above")
  ti_alone = mph < EPS_LOW_MPH
  for low, high, label, alone in WINDOWS:
    if low <= mph < high:
      return Zone(label, label, alone, "inside")
  same = [w for w in WINDOWS if w[3] == ti_alone]
  near = min(same, key=lambda w: max(w[0] - mph, mph - w[1]))
  return Zone(None, near[2], ti_alone, "between")


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
# The same limit PER WINDOW (strict steps, small steps), as shares of TI_STEER_MAX: the one place to change it for one
# window when evidence shows what the car does there. The values are the previous revision's; none may exceed it
# without the owner's decision (the unit tests pin the values). The gap between windows uses its nearest window's.
COMMAND_LIMIT_FRACTION = {w[2]: (MAX_BASE_FRACTION_STRICT, MAX_BASE_FRACTION_SMALL) for w in WINDOWS}
# Re-arm dwell: a gate that closed on the command (or a bend / lateral demand) opens again only after the value has been
# inside its limit for this long. Cutting stays immediate; only starting a unit waits, so a command hovering at the
# limit does not start and cut units (and lead-ins) over and over.
REARM_S = 0.3
# A test that is blocked only by one of those limits for this long is passed over in run all for one that can run.
BLOCK_PATIENCE_S = 1.0
# Run all, every test complete in the window: tests whose total time there is within this of the least are drawn from.
TIE_S = 10.0
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
# Amplitudes at the dither's peak (22 counts) where the slew allows it, so the slow tones, which the natural steering
# noise hides most, need as little time as possible; the high tones are reduced so the slew stays under MAX_SLEW.
SWEEP_TONES = ((334, 22.0), (166, 22.0), (100, 22.0), (62, 22.0), (42, 16.0), (28, 12.0), (20, 9.0))
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
LADDER_CAPS = {"21-27": LADDER_LOW_BAND_CAP, "35-41": 37.0, "49-56": 37.0, "64-80": 37.0}
LADDER_LEAD_IN = 100

# Pulses: a bipolar trapezoid doublet (+A then -A, ramps at PULSE_RAMP counts per frame, PULSE_FLAT frames flat),
# then quiet to the end of a PULSE_PERIOD-frame unit for the settling; the doublet's sign alternates pulse by pulse.
PULSE_AMPS = (22.0, 44.0)
PULSE_PERIOD = 200
PULSE_RAMP = 2.75
PULSE_FLAT = 10

# ---- what each band needs -----------------------------------------------------------------------------------------
# Clean seconds per step and band for a usable estimate of the EPS sensor response (z = 5) against the natural
# steering noise of 150 released segments (E:\openpilot-mazda-ti-data\ident-analysis-20261120\noise_needs.json),
# at the amplitudes above: time = 10-20 s x (noise / response)^2, rounded up to whole units, at least 2 units (pulses:
# 10), at most STEP_CAP_S (z then 3.4-5). A step that would need more than OPTIONAL_ABOVE_S is OPTIONAL (0 here): it
# runs only as extra once the band is complete. Optional: sweep 0.3 Hz in every band (148-776 s needed even at 22
# counts; the ladder's 0.4 Hz steps give the low-frequency point instead), ladder 6 and 11 counts everywhere
# (214-4861 s) and 22 and 37 counts below 28 mph (362 and 128 s), the 22-count pulse below 28 mph (211 s).
STEP_CAP_S = 60.0
OPTIONAL_ABOVE_S = 130.0
REQUIRED_S = {
  # 21-27 sweep: the noise-derived needs (190.5 s) cut to about 80 s (whole units per step, the first six units' shares kept roughly in proportion) by the owner's plan (ti_drive_plan.json gives the reason: two earlier drives cover the
  # sweep in the lowest band and the wheel's response to it is below the angle sensor's resolution); pulses and ladder are in full
  "21-27": {SWEEP: (0.0, 26.56, 18.0, 12.4, 12.6, 4.48, 6.0), LADDER: (0.0, 0.0, 0.0, 0.0, 60.0), PULSES: (0.0, 54.0)},
  "35-41": {SWEEP: (0.0, 26.56, 14.0, 9.92, 6.3, 4.48, 6.0), LADDER: (0.0, 0.0, 60.0, 40.0), PULSES: (60.0, 20.0)},
  "49-56": {SWEEP: (0.0, 26.56, 10.0, 7.44, 4.2, 4.48, 4.0), LADDER: (0.0, 0.0, 60.0, 25.0), PULSES: (52.0, 20.0)},
  "64-80": {SWEEP: (0.0, 13.28, 10.0, 4.96, 4.2, 4.48, 4.0), LADDER: (0.0, 0.0, 55.0, 20.0), PULSES: (54.0, 20.0)},
}
# Nominal per-step figure kept in ti_tests.json as target_seconds (the cap); the screen uses the per-step "needs".
STEP_SECONDS = {SWEEP: STEP_CAP_S, LADDER: STEP_CAP_S, PULSES: STEP_CAP_S}


def required_seconds(key, band, step):
  """Clean seconds the step needs in the band (0: optional, extra only)."""
  return REQUIRED_S[band][key][step]


def band_required(key, band):
  return sum(REQUIRED_S[band][key])

# Run all
RUN_ALL_ORDER = (SWEEP, PULSES, LADDER)
RUNNING_REASONS = ("ok", "extra", "gap")      # the gate is open: a signal is (or starts to be) sent
# The limits that can close the gate on a step's own size, in the gate chain's order: (what, unit) for the screen
GATE_WHAT = {"command": ("steering command", "counts"), "curve": ("road curvature", "1/m"),
             "lat": ("lateral demand", "m/s2")}
SWITCH_QUIET_S = 0.5        # quiet between one test's complete fade-out and the next test's start
EXTRA_HYSTERESIS = 0.25     # extra collection: leave the current test once it leads the least-collected by this share
EXTRA_DWELL_UNITS = 4       # extra collection: units on one step before moving to the next (fewer fade transitions)


def unit_periods(p):
  return max(UNIT_MIN_PERIODS, int(math.ceil(UNIT_MIN_S / (p * DT) - 1e-9)))


def _table(key):
  if key == SWEEP:
    return {"tones": SWEEP_TONES, "lead_in": SWEEP_LEAD_IN, "unit": [UNIT_MIN_PERIODS, UNIT_MIN_S]}
  if key == LADDER:
    return {"period": LADDER_PERIOD, "amps": LADDER_AMPS, "caps": LADDER_CAPS, "lead_in": LADDER_LEAD_IN,
            "unit": [UNIT_MIN_PERIODS, UNIT_MIN_S]}
  return {"amps": PULSE_AMPS, "period": PULSE_PERIOD, "ramp": PULSE_RAMP, "flat": PULSE_FLAT}


TABLE_HASH = {k: hashlib.sha1(json.dumps([VERSION, _table(k), {b: REQUIRED_S[b][k] for b in REQUIRED_S}, BANDS, MAX_SLEW],
                                         sort_keys=True).encode()).hexdigest()[:12] for k in KEYS}


def _signal(key):
  """What a credited second of this test means: the waveform of every step (not the needs, caps or gates), and the
  windows its seconds are counted in (revision 3: new windows start from empty, the old record is kept aside)."""
  t = dict(_table(key))
  t.pop("caps", None)
  return ["zero-heading phase, sign alternating per run", t, ["windows", WINDOWS]]


# Progress is kept per SIGNAL_HASH (ti_tests.py): seconds collected with another waveform do not count toward these.
SIGNAL_HASH = {k: hashlib.sha1(json.dumps(_signal(k), sort_keys=True).encode()).hexdigest()[:12] for k in KEYS}


def n_steps(key):
  return len(SWEEP_TONES) if key == SWEEP else len(LADDER_AMPS) if key == LADDER else len(PULSE_AMPS)


_STEPS_IN_BAND = {}


def steps_in_band(key, label):
  """Number of steps captured in a band (the first ones of the table). Constant per (test, band): kept once computed."""
  got = _STEPS_IN_BAND.get((key, label))
  if got is None:
    # per-frame-ok: keyed by (test, band): 3 tests x the LADDER_CAPS bands
    got = _STEPS_IN_BAND[(key, label)] = (sum(1 for a in LADDER_AMPS if a <= LADDER_CAPS[label]) if key == LADDER else n_steps(key))
  return got


_STEP_PARAMS = {}


def step_params(key, step):
  """(period frames or None for pulses, peak counts, unit frames, lead-in frames). Constant per (test, step): kept once computed
  (it is asked many times a frame)."""
  got = _STEP_PARAMS.get((key, step))
  if got is None:
    # per-frame-ok: keyed by (test, step): 3 tests x their step tables
    got = _STEP_PARAMS[(key, step)] = _step_params(key, step)
  return got


def _step_params(key, step):
  if key == SWEEP:
    p, a = SWEEP_TONES[step]
    return p, a, p * unit_periods(p), SWEEP_LEAD_IN
  if key == LADDER:
    return LADDER_PERIOD, LADDER_AMPS[step], LADDER_PERIOD * unit_periods(LADDER_PERIOD), LADDER_LEAD_IN
  return None, PULSE_AMPS[step], PULSE_PERIOD, 0


def unit_credit(key, step):
  """Seconds a clean unit is credited: its length."""
  return round(step_params(key, step)[2] * DT, 2)


_SMALL = {}


def small(key, step):
  got = _SMALL.get((key, step))
  if got is None:
    # per-frame-ok: keyed by (test, step): finite like _STEP_PARAMS
    got = _SMALL[(key, step)] = step_params(key, step)[1] <= SMALL_AMP
  return got


_SMALL_FLAGS = {}


def small_flags(key, band):
  """small(key, s) for each step captured in `band`, as a tuple. Constant per (test, band): kept once computed."""
  got = _SMALL_FLAGS.get((key, band))
  if got is None:
    # per-frame-ok: keyed by (test, band): finite like _STEPS_IN_BAND
    got = _SMALL_FLAGS[(key, band)] = tuple(small(key, s) for s in range(steps_in_band(key, band)))
  return got


# `feasible` for _pick when no step is allowed now (the limits close every step class of the window, or an input is not finite):
# the same answer as a test that is False for every step, without a call per step
NONE_FEASIBLE = object()


def limit_open(strict, profile, base_request, curvature, lat_demand, ti_max):
  """True when no limit closes a step of this class (strict: not small) in `profile` now: exactly `limit_state(...)[0] is None`
  (each value finite and not above its bound), without building the dicts (cost: asked every frame)."""
  b = abs(base_request)
  c = abs(curvature)
  la = abs(lat_demand)
  if strict:
    return (math.isfinite(b) and b <= COMMAND_LIMIT_FRACTION[profile][0] * ti_max and math.isfinite(c) and c <= MAX_CURVATURE
            and math.isfinite(la) and la <= MAX_LAT_DEMAND)
  return (math.isfinite(b) and b <= COMMAND_LIMIT_FRACTION[profile][1] * ti_max and math.isfinite(c) and c <= MAX_CURVATURE_SMALL
          and math.isfinite(la) and la <= MAX_LAT_DEMAND_SMALL)


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
  """The full-amplitude signal (float counts) of a step at unit phase `ph` (frames; negative: lead-in), times `sign`.
  Sweep and ladder: a tone at the phase PHASES gives it (zero-mean heading); its sign alternates from one run (lead-in)
  to the next, so the fade-out and lead-in at successive transitions cancel in pairs. Pulses: the doublet, its sign
  alternating pulse by pulse, then quiet."""
  p, a, _, _ = step_params(key, step)
  if p is None:
    shape = _DOUBLETS[a]
    return sign * shape[ph] if 0 <= ph < len(shape) else 0.0
  return sign * _tone(p, a, PHASES[(key, step)], ph)


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

# Net counts of a step's lead-in at sign +1 (the raised-cosine ramp into the tone): the generator picks each new run's
# sign so that the lead-in pulls the running net of everything sent so far back toward zero.
LEAD_SUM = {(k, s): sum(value(k, s, ph, (ph + step_params(k, s)[3]) / step_params(k, s)[3])
                        for ph in range(-step_params(k, s)[3], 0))
            for k in (SWEEP, LADDER) for s in range(n_steps(k))}


def band_for(v_ego, lkas_blocked=None):
  """(window label, TI alone) for a speed: the window it is inside, else (None, TI alone) between windows, and
  (None, None) where nothing runs. Kept for callers of the old name; `zone_for` has the whole picture."""
  z = zone_for(v_ego)
  return z.window, z.ti_alone


_PEAK = {}


def peak_counts(key):
  got = _PEAK.get(key)
  if got is None:
    # per-frame-ok: keyed by test: 3 entries
    got = _PEAK[key] = max(step_params(key, s)[1] for s in range(n_steps(key)))
  return got


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


def check_definition(key, bins, steps, target, needs=None):
  """None when a step_seconds definition for `key` matches this module, else the reason (ti_tests validation)."""
  if key not in KEYS:
    return f"setting {key} does not report steps"
  if tuple((lo, hi, lab) for lo, hi, lab, _ in bins) != tuple((lo, hi, lab) for lo, hi, lab, _ in BANDS):
    return f"bins must be the {key} bands " + ", ".join(b[2] for b in BANDS)
  if tuple(steps) != tuple(steps_in_band(key, b[2]) for b in BANDS):
    return f"steps must be {[steps_in_band(key, b[2]) for b in BANDS]}"
  if target != STEP_SECONDS[key]:
    return f"target_seconds must be {STEP_SECONDS[key]:g}"
  if needs is not None and tuple(tuple(float(x) for x in n) for n in needs) != tuple(REQUIRED_S[b[2]][key] for b in BANDS):
    return f"needs must be {[list(REQUIRED_S[b[2]][key]) for b in BANDS]}"
  return None


def band_share(key, band, remaining, clip=True):
  """Collected / required for one test in one band; None when unknown. clip: each step counts at most its own need, so
  the share reaches 1 only when every step is complete (completion). Not clipped: extra included (choosing extra)."""
  required, collected = 0.0, 0.0
  for s in range(steps_in_band(key, band)):
    left = remaining(key, band, s)
    if left is None:
      return None
    need = required_seconds(key, band, s)
    required += need
    collected += min(need, need - left) if clip else need - left
  if required <= 0:
    return 1.0 if clip else 1.0 + collected
  return collected / required


def window_total(key, window, remaining):
  """Seconds collected by one test in a window, extra and optional steps included; None when unknown."""
  total = 0.0
  for s in range(steps_in_band(key, window)):
    left = remaining(key, window, s)
    if left is None:
      return None
    total += required_seconds(key, window, s) - left
  return round(total, 2)


def window_left(key, window, remaining):
  """(required seconds still to collect, required cells still incomplete) of one test in a window; None when unknown.
  A cell is one step (a tone, an amplitude, a pulse size); optional steps (need 0) are not cells."""
  secs, cells = 0.0, 0
  for s in range(steps_in_band(key, window)):
    need = required_seconds(key, window, s)
    left = remaining(key, window, s)
    if left is None:
      return None
    if need > 0 and left > 1e-6:
      secs += left
      cells += 1
  return secs, cells


def window_target_mph(window):
  """The speed the screen asks for in a window: its middle, in whole mph (24, 38, 52, 72)."""
  for low, high, label, _ in WINDOWS:
    if label == window:
      return int((low + high) / 2.0)
  return None


def window_distance(mph, window):
  """mph to drive to be inside the window (0 when inside)."""
  for low, high, label, _ in WINDOWS:
    if label == window:
      return max(low - mph, mph - high, 0.0)
  return None


def target_window(mph, remaining, keys=KEYS):
  """(window, seconds left, cells left, mph to change) of the nearest window that still needs something of `keys`:
  the least speed change first (whole mph), then the most still outstanding; None when every window is complete or
  progress is unknown."""
  best = None
  for _, _, label, _ in WINDOWS:
    secs, cells = 0.0, 0
    for k in keys:
      got = window_left(k, label, remaining)
      if got is None:
        return None
      secs += got[0]
      cells += got[1]
    if cells == 0:
      continue
    dist = window_distance(mph, label)
    rank = (int(math.floor(dist + 0.5)), -secs)     # whole mph, halves up (ti_tests_logic.h: std::lround)
    if best is None or rank < best[0]:
      best = (rank, (label, secs, cells, dist))
  return None if best is None else best[1]


def _uniform(seed, draw):
  """The draw-th number in [0, 1) of a seed: a hash, so any reader recomputes it without our random module."""
  h = hashlib.sha256(f"{seed}:{draw}".encode()).digest()
  return int.from_bytes(h[:6], "big") / float(1 << 48)


def draw_choice(seed, draw, cands):
  return cands[min(int(_uniform(seed, draw) * len(cands)), len(cands) - 1)]


class RunAll:
  """Run all: the test to arm now. `pick(frame, profile, remaining, can_run)`; `profile` is the window whose progress
  applies (Zone.profile; None: keep the current test, its own gate pauses it); `can_run(test, profile)`: False when
  that test's next step is closed only by a command / curve / lateral-demand limit right now. Every change of test is
  emitted as a "pick" event through `emit(frame, "pick", **fields)`."""

  def __init__(self, seed=None, emit=None):
    self.seed = int.from_bytes(os.urandom(4), "big") if seed is None else int(seed)
    self.draw = 0
    self.emit = emit
    self.current = None
    self.profile = None
    self.mode = None
    self.blocked = 0
    self.last_emit = None      # frame of the last pick event
    self.skipped = 0           # pick events held back since then (rate limit)

  def _choose(self, frame, profile, test, mode, totals, **kw):
    if test != self.current or mode != self.mode or profile != self.profile:
      if self.emit is not None and (test != self.current or profile != self.profile):
        if self.last_emit is None or frame - self.last_emit >= PICK_LOG_MIN_S / DT:
          extra = {"skipped": self.skipped} if self.skipped else {}
          self.emit(frame, "pick", test=test, mode=mode, window=profile, seed=self.seed, totals=totals, **extra, **kw)
          self.last_emit, self.skipped = frame, 0
        else:
          self.skipped += 1
    self.current, self.profile, self.mode, self.blocked = test, profile, mode, 0
    return test

  def pick(self, frame, profile, remaining, can_run=None, strict=None, stats=None):
    """`strict(k, profile)`: this test's next step is a strict-gated one (the 44-count pulse, the ladder's larger steps). Strict-open
    stretches are the scarce resource (the limits are tighter), so when one is open a test that can use it goes first instead of the
    current one finishing its small steps (which can run in any stretch). No limit changes."""
    if profile is None:
      return self.current
    run = (lambda k: True) if can_run is None else (lambda k: bool(can_run(k, profile)))
    if stats is not None:
      totals, shares = stats(profile)                    # the same window_total / band_share, kept while the progress is unchanged
      if totals is None:
        return None
    else:
      totals, shares = {}, {}
      for k in RUN_ALL_ORDER:
        totals[k] = window_total(k, profile, remaining)
        shares[k] = band_share(k, profile, remaining)
        if totals[k] is None or shares[k] is None:
          return None
    same = profile == self.profile
    cur = self.current
    patience = int(round(BLOCK_PATIENCE_S / DT))
    incomplete = [k for k in RUN_ALL_ORDER if shares[k] < 1.0 - 1e-9]
    if incomplete and strict is not None:
      scarce = [k for k in incomplete if run(k) and strict(k, profile)]
      if scarce and not (same and cur in scarce and self.mode in ("required", "blocked")):
        return self._choose(frame, profile, scarce[0], "required", totals)
    if incomplete:
      if same and cur in incomplete and self.mode in ("required", "blocked"):
        if run(cur):
          self.blocked = 0
          return cur
        self.blocked += 1
        others = [k for k in incomplete if k != cur and run(k)]
        if self.blocked < patience or not others:
          return cur
        return self._choose(frame, profile, others[0], "blocked", totals, passed_over=cur)
      runnable = [k for k in incomplete if run(k)]
      return self._choose(frame, profile, (runnable or incomplete)[0], "required", totals)
    # every test is complete in this window: keep going, drawing among the least collected
    lo = min(totals.values())
    tied = [k for k in RUN_ALL_ORDER if totals[k] - lo < TIE_S]
    if same and self.mode == "random" and cur in tied:
      if run(cur):
        self.blocked = 0
        return cur
      self.blocked += 1
      if self.blocked < patience:
        return cur
    cands = [k for k in tied if run(k)] or tied
    draw = self.draw
    self.draw += 1
    return self._choose(frame, profile, draw_choice(self.seed, draw, cands), "random", totals, draw=draw, tied=tied,
                        cands=cands, tie_s=TIE_S)


def verify_picks(events):
  """Recompute every logged random pick from its seed, draw number and totals; the list of mismatching events."""
  bad = []
  for e in events:
    if e.get("kind") != "pick" or e.get("mode") != "random":
      continue
    totals = e["totals"]
    lo = min(totals.values())
    tied = [k for k in RUN_ALL_ORDER if totals[k] - lo < e["tie_s"]]
    cands = e["cands"]
    if tied != e["tied"] or not set(cands) <= set(tied) or draw_choice(e["seed"], e["draw"], cands) != e["test"]:
      bad.append(e)
  return bad


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
    self.tone_sign = -1          # sweep and ladder: the sign of the current run's tone
    self.net = 0                 # running sum of every count sent (the heading the test has left); kept near zero
    self.units_on_step = 0       # units started on the current step in this run (extra dwell)
    self.gate_last = None        # (frame count, gate) of the latest frame a limit closed the gate (published at 1 Hz)
    self.gate = None             # while a limit closes the gate: {reason, what, value, limit, unit, window} (screen)
    self.profile = None          # the window whose caps and limits apply to the running unit
    self.n = 0                   # frames seen (re-arm dwell)
    self.viol_at = {}            # (test key, step) -> frame a limit last closed that step, and which
    self.inputs = None           # the last frame's limit inputs, for can_run
    self.zone = Zone(None, None, None, "below")
    self.zone_off = True         # the speed is (or was last) in an off-region and has not cleared it yet
    self.prof = None             # the profile held between windows (hysteresis)
    self.zone_n = -1             # the frame count advance_zone last ran for
    self.mph = 0.0
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

  def event_for_pick(self, frame, kind, **kw):
    """Run all's scheduler logs its choices through the same event stream as the signal's own events."""
    self._event(frame, kind, **kw)

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

  def advance_zone(self, v_ego):
    """The zone for this frame's speed, with the hysteresis of ZONE_HYSTERESIS_MPH: off at once at an off-region's edge,
    on again only once clear of it by the margin; between windows the profile changes only when the speed is the margin
    beyond the midpoint. A non-finite speed is no speed (off). Run once per frame, before `update` (which runs it itself
    when the caller did not) so run all and the gate see the same zone."""
    v = float(v_ego) if math.isfinite(v_ego) else 0.0
    mph = v * MS_TO_MPH
    raw = zone_for(v)
    h = ZONE_HYSTERESIS_MPH
    if raw.profile is None:
      self.zone_off, self.prof = True, None
      self.zone = raw
    else:
      if self.zone_off:
        clear = mph >= MIN_TEST_MPH + h and not (EPS_LOW_MPH - h <= mph < EPS_HIGH_MPH + h) and mph < MAX_TEST_MPH - h
        if not clear:
          state = "below" if mph < MIN_TEST_MPH + h else "transition" if mph < MAX_TEST_MPH - h else "above"
          self.zone = Zone(None, None, None, state)
          self.zone_n = self.n + 1
          return self.zone
        self.zone_off, self.prof = False, None
      prof = raw.profile
      if raw.window is None and self.prof is not None and self.prof != raw.profile:
        held = next(w for w in WINDOWS if w[2] == self.prof)
        if held[3] == raw.ti_alone and (window_distance(mph, self.prof) - window_distance(mph, raw.profile)) <= 2 * h:
          prof = self.prof               # not yet clearly past the midpoint of the gap
      self.prof = prof
      self.zone = Zone(raw.window, prof, raw.ti_alone, raw.state)
    self.zone_n = self.n + 1
    return self.zone

  def reset(self, frame=None):
    """Lateral control off or TI not in RUN: zero at once."""
    if self.test is not None and frame is not None:
      self._event(frame, "stop", mode="zero", why="inactive", step=self.step)
    self._stop()
    self.loud = 0
    self.touch = False
    self.credit = None
    self.emitted = None
    self.gate = None
    self.zone_off, self.prof = True, None

  @property
  def running(self):
    return self.test is not None

  def _pick(self, key, band, remaining, credit=None, feasible=None):
    """(step, extra): the first step of the table still to capture in `band` (this frame's credit counted) that the
    limits allow now, else the first still to capture; with everything captured, the least-collected one as extra
    (ties: the first), among those the limits allow if any; (None, False) when progress is unknown."""
    never = feasible is NONE_FEASIBLE
    lefts = []
    for s in range(steps_in_band(key, band)):
      left = remaining(key, band, s)
      if left is None:
        return None, False
      if credit is not None and credit[:3] == (key, band, s):
        left -= credit[3]
      lefts.append((s, left))
    todo = [s for s, left in lefts if left > 1e-6]
    allowed = [] if never else [(s, left) for s, left in lefts if feasible is None or feasible(s)]
    if todo:
      ok = [] if never else [s for s in todo if feasible is None or feasible(s)]
      if ok:
        return ok[0], False
      if not allowed:
        return todo[0], False      # nothing the limits allow: the first still to capture (its gate shows why)
      # every step still to capture is closed by a limit right now: the signal goes on with the least-collected step
      # that is allowed (extra), so a gate hovering at its limit does not leave the test idle
    pool = allowed or lefts
    best, best_left = None, None
    for s, left in pool:
      if best_left is None or left > best_left + 1e-9:
        best, best_left = s, left
    return best, True

  @staticmethod
  def limit_bounds(key, step, profile, ti_max):
    """The three limits that apply to a step in a window: command (counts of the undithered TI request, per window and
    step size, COMMAND_LIMIT_FRACTION), curve (|curvature|, 1/m) and lat (|v^2 curvature|, m/s^2)."""
    strict = not small(key, step)
    return {"command": COMMAND_LIMIT_FRACTION[profile][0 if strict else 1] * ti_max,
            "curve": MAX_CURVATURE if strict else MAX_CURVATURE_SMALL,
            "lat": MAX_LAT_DEMAND if strict else MAX_LAT_DEMAND_SMALL}

  def limit_state(self, key, step, profile, base_request, curvature, lat_demand, ti_max):
    """(reason, value, limit) of the first limit that closes this step now, else (None, None, None), in the gate
    chain's order."""
    bounds = self.limit_bounds(key, step, profile, ti_max)
    values = {"command": abs(base_request), "curve": abs(curvature), "lat": abs(lat_demand)}
    for reason in GATE_WHAT:
      if not math.isfinite(values[reason]) or values[reason] > bounds[reason]:   # NaN fails every comparison: closed
        return reason, values[reason], bounds[reason]
    return None, None, None

  def _limit_class(self, key, step, band, memo):
    """limit_state of this frame for a step, computed once per step class (small or not) and window."""
    sm = _SMALL.get((key, step))
    c = (small(key, step) if sm is None else sm, band)
    if c not in memo:
      memo[c] = self.limit_state(key, step, band, **self.inputs)
    return memo[c]

  def next_is_strict(self, key, profile, remaining):
    """True when the step this test would run next (one the limits allow now) is a strict-gated one."""
    step, _ = self._pick(key, profile, remaining, None, lambda s: self._feasible(key, s, profile))
    return step is not None and not small(key, step)

  def can_run(self, key, profile, remaining):
    """False when the step this test would run next is closed right now only by a command / curve / lateral-demand
    limit, or is still inside that limit's re-arm dwell (run all passes such a test over after BLOCK_PATIENCE_S)."""
    if self.inputs is None or profile is None:
      return True
    if not all(math.isfinite(x) for x in self.inputs.values()):
      return False
    step, _ = self._pick(key, profile, remaining, None, lambda s: self._feasible(key, s, profile))
    return step is not None and self._feasible(key, step, profile)

  def _feasible(self, key, step, profile):
    if self.inputs is None:
      return True
    if not all(math.isfinite(x) for x in self.inputs.values()):
      return False
    if self.limit_state(key, step, profile, **self.inputs)[0] is not None:
      return False
    return (self.n - self.viol_at.get((key, step), (-10 ** 9, None))[0]) > REARM_S / DT

  def _start_chunk(self, frame, key, band, step, lead_in, extra):
    p, a, _, lead = step_params(key, step)
    if key == PULSES and self.test == key:
      self.sign = -self.sign
    elif p is not None and lead_in:
      # Each fresh run's sign: the one whose lead-in pulls the running net back toward zero (a fade-out and the next
      # lead-in leave a few hundred count-frames each); with nothing to correct, alternate.
      lead_net = LEAD_SUM[(key, step)]
      if self.net != 0 and lead_net != 0:
        self.tone_sign = -1 if (self.net > 0) == (lead_net > 0) else 1
      else:
        self.tone_sign = -self.tone_sign
      self.sign = self.tone_sign
    self.units_on_step = self.units_on_step + 1 if (not lead_in and self.test == key and self.step == step) else 1
    window = self.zone.window
    self.test, self.step, self.band, self.fading, self.extra = key, step, window, False, extra
    self.profile = band
    self.ph = -lead if lead_in else 0
    self.env = 0.0 if lead_in and lead else 1.0
    self.clean = True
    # `band`: the window the unit starts in ("gap" between windows: logged, not counted); `profile`: whose caps and
    # limits apply; `mph`: the speed now
    self._event(frame, "run" if lead_in else "chunk", test=key, step=step, band=window or "gap", profile=band,
                mph=round(self.mph, 1), ph=self.ph, env=self.env, sign=self.sign, amp=a, extra=extra)

  def _fade(self, frame, why):
    if self.test == PULSES and self.ph >= len(_DOUBLETS[PULSE_AMPS[self.step]]):
      self._event(frame, "stop", mode="zero", why=why, step=self.step)   # between doublets: already zero
      self._stop()
      return
    self.fading = True
    self.clean = False
    self._event(frame, "stop", mode="fade", env=self.env, why=why, step=self.step)

  def update(self, armed, frame, *, v_ego, steering_pressed, driver_torque, blinker, base_request, ti_max,
             lat_demand, curvature, lkas_blocked, room=None, remaining=None):
    """`armed`: the one test key to run (None: none). `room`: rate_room of the live limits for the armed test.
    `remaining(key, band, step)`: seconds still to capture for that step in that window, negative when it has extra
    (the accountant's progress; None from it: unknown, nothing runs; None here: nothing captured yet)."""
    if remaining is None:
      remaining = lambda k, b, s: required_seconds(k, b, s)
    self.credit = None
    self.gate = None
    self.n += 1
    self.stopped_age += 1
    # A non-finite input (NaN or infinity) closes the gate: nothing comparable can be trusted, and a NaN passes every
    # "greater than the limit" test. Zero at once, like a touch.
    isfin = math.isfinite
    bad = not (isfin(v_ego) and isfin(base_request) and isfin(curvature) and isfin(lat_demand) and isfin(driver_torque)
               and isfin(ti_max))
    magnitude = abs(driver_torque) if math.isfinite(driver_torque) else 0.0
    self.loud = self.loud + 1 if magnitude >= DRIVER_QUIET_COUNTS else 0
    touch = self.touch = bool(steering_pressed or magnitude >= DRIVER_FIRM_COUNTS
                              or self.loud >= self.touch_confirm_frames)
    self.touch_age = 0 if touch else self.touch_age + 1
    self.blink_age = 0 if blinker else self.blink_age + 1
    if lkas_blocked is None or lkas_blocked != self.regime:
      self.regime, self.regime_frames = lkas_blocked, 0
    else:
      self.regime_frames += 1

    if self.zone_n != self.n:
      self.advance_zone(v_ego)
      self.zone_n = self.n          # (advance_zone stamps n + 1 for a caller that runs it before this frame's update)
    zone = self.zone
    self.mph = v_ego * MS_TO_MPH if math.isfinite(v_ego) else 0.0
    band, ti_alone = zone.profile, zone.ti_alone      # the window whose caps, limits and needs apply
    self.inputs = dict(base_request=base_request, curvature=curvature, lat_demand=lat_demand, ti_max=ti_max)

    # A unit that ended on the previous frame is credited first (its frames were 0..n-1), whatever happens now. Only
    # a unit that stayed inside one window counts (self.band is None for a unit that started between windows).
    if (self.test is not None and not self.fading and self.ph >= step_params(self.test, self.step)[2] and self.clean
        and self.band is not None):
      self.credit = (self.test, self.band, self.step, unit_credit(self.test, self.step))
      left = remaining(self.test, self.band, self.step)
      # a compact progress snapshot for the analysis: seconds still to capture in this cell after this credit
      self._event(frame, "credit", test=self.test, band=self.band, step=self.step, s=self.credit[3],
                  left=None if left is None else round(left - self.credit[3], 2))

    key = armed if armed in KEYS else None
    if key is None and self.test is None:
      # nothing armed and nothing running: what the full path below does then, without its work (cost: every cycle with the tests
      # off): the reason is "off", no gate, no count, no signal
      self.reason = "off"
      self.last = 0
      self.emitted = None
      return 0
    # This frame's limit checks, one per step class (limit_state depends on the step only through small(); cost: it ran for every step
    # of the window two or three times a frame)
    lim_memo = {}
    finite_in = isfin(base_request) and isfin(curvature) and isfin(lat_demand) and isfin(ti_max)

    def lim(k, s_):
      return self._limit_class(k, s_, band, lim_memo)

    viol_at, n_now, rearm_frames = self.viol_at, self.n, REARM_S / DT

    def feasible_now(k, s_):              # = self._feasible(k, s_, band) on this frame's inputs
      if not finite_in:
        return False
      sm_ = _SMALL.get((k, s_))
      why_ = lim_memo.get((small(k, s_) if sm_ is None else sm_, band))
      if (lim(k, s_) if why_ is None else why_)[0] is not None:
        return False
      return (n_now - viol_at.get((k, s_), (-10 ** 9, None))[0]) > rearm_frames
    # Which steps the limits close right now: when each last did, for the re-arm dwell (the limit result is one per step class)
    any_open = False                      # some step class of the window open now (else no step is feasible this frame)
    if key is not None and band is not None:
      why_small = why_strict = None       # the limit result of each step class, looked up once
      for st_, sm in enumerate(small_flags(key, band)):
        if sm:
          if why_small is None:
            why_small = lim(key, st_)
          why = why_small
        else:
          if why_strict is None:
            why_strict = lim(key, st_)
          why = why_strict
        if why[0] is not None:
          viol_at[(key, st_)] = (n_now, why[0])
        else:
          any_open = True
    # The step whose limits apply: the running one, else the one that would start
    step, extra = (None, False)
    if key is not None and band is not None:
      if self.test == key and not self.fading:
        step, extra = self.step, self.extra
      else:
        step, extra = self._pick(key, band, remaining, self.credit,
                                 (lambda s: feasible_now(key, s)) if finite_in and any_open else NONE_FEASIBLE)
    strict = step is None or not small(key, step)
    touch_hold = (TOUCH_HOLDOFF_S if strict else TOUCH_HOLDOFF_SMALL_S) / DT
    limit = (None, None, None)
    rearming = None
    if step is not None and band is not None:
      limit = lim(key, step)
      # A step about to START (nothing running) waits out the re-arm dwell after a limit closed it; cutting is immediate
      if limit[0] is None and self.test is None:
        last = self.viol_at.get((key, step))
        if last is not None and self.n - last[0] <= REARM_S / DT:
          rearming = last[1]
    # Why not (display), and the gate, in the dither's order of precedence
    if key is None:
      reason = "off"
    elif bad:
      reason = "input"
    elif touch:
      reason = "touch"
    elif room is not None and not rate_room_ok(room, key, step if step is not None else 0):
      reason = "rate"
    elif blinker:
      reason = "blinker"
    elif self.touch_age < touch_hold or self.blink_age < BLINKER_HOLDOFF_S / DT:
      reason = "holdoff"
    elif zone.state == "transition":
      reason = "transition"
    elif band is None:
      reason = "band"
    elif lkas_blocked is None or bool(lkas_blocked) != ti_alone or self.regime_frames < REGIME_SETTLE_S / DT:
      reason = "regime"
    elif limit[0] is not None:
      reason = limit[0]
    elif rearming is not None:
      reason = rearming
    elif step is None:
      reason = "off"              # progress unknown (no accountant): nothing runs
    elif key == LADDER and self.test == key and step_params(key, self.step)[1] > LADDER_CAPS[band]:
      reason = "band"             # the band changed under a step above the new band's cap
    elif self.test is None and key != self.prev_test and self.stopped_age <= SWITCH_QUIET_S / DT:
      reason = "switch"           # quiet between two tests
    else:
      reason = "extra" if extra else "ok"
      if zone.window is None:
        reason = "gap"            # between the windows: runs, logged with its speed, counts toward nothing
    if self.test is not None and key != self.test and reason in RUNNING_REASONS:
      reason = "switch"           # another test is next: this one fades out first
    self.reason = reason
    open_ = reason in RUNNING_REASONS
    if reason in GATE_WHAT and step is not None and band is not None:
      val = {"command": abs(base_request), "curve": abs(curvature), "lat": abs(lat_demand)}[reason]
      bound = self.limit_bounds(key, step, band, ti_max)[reason]
      what, unit = GATE_WHAT[reason]
      self.gate = {"reason": reason, "what": what, "value": round(val, 3), "limit": round(bound, 3), "unit": unit,
                   "window": band, "rearm": limit[0] is None}
    if self.gate is None and key is not None and band is not None and self.test == key and extra and not self.fading:
      # running extra only because a limit closes the step still to capture: say which limit, and that it waits
      for st_ in range(steps_in_band(key, band)):
        left = remaining(key, band, st_)
        if left is not None and left > 1e-6 and required_seconds(key, band, st_) > 0:
          why = lim(key, st_)
          if why[0] is not None:
            what, unit = GATE_WHAT[why[0]]
            self.gate = {"reason": why[0], "what": what, "value": round(why[1], 3), "limit": round(why[2], 3),
                         "unit": unit, "window": band, "rearm": False, "running": True}
            break
    if self.gate is not None:
      self.gate_last = (self.n, self.gate)
    if key is not None:
      slot = (key, zone.window or ("gap" if band is not None else "none"), reason)
      self.counts[slot] = self.counts.get(slot, 0) + 1

    if self.test is not None and (touch or bad or (key is not None and reason == "rate")):
      self._event(frame, "stop", mode="zero", why=reason, step=self.step)   # a touch or too little rate room
      self._stop()
    if self.test is not None and not self.fading and (not open_ or key != self.test):
      self._fade(frame, reason)

    if self.test is None:
      if open_:
        self._start_chunk(frame, key, band, step, lead_in=True, extra=extra)
    elif not self.fading:
      if self.ph >= step_params(self.test, self.step)[2]:
        nxt, nxt_extra = self._pick(key, band, remaining, self.credit, lambda s: self._feasible(key, s, band))
        if (nxt_extra and self.extra and nxt != self.step and self.units_on_step < EXTRA_DWELL_UNITS
            and self.step < steps_in_band(key, band) and band == self.profile):
          nxt = self.step                 # extra: a few units on one step before moving on (fewer transitions)
        if nxt == self.step or self.test == PULSES:
          self._start_chunk(frame, key, band, nxt, lead_in=False, extra=nxt_extra)   # straight on: continuous
        else:
          self._fade(frame, "step")     # another tone or amplitude: fade out, then a fresh lead-in
      elif self.ph >= 0 and (zone.window != self.band or self.env < 1.0):
        self.clean = False

    if self.test is None:
      self.last = 0
      self.emitted = None
      return 0
    self.emitted = self.test
    st = self.__dict__
    _wrap(st)
    self.last = value(self.test, self.step, self.ph, self.env, self.sign)
    self.net += self.last
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
    ok = sum(n for (k, _, r), n in self.counts.items() if k == key and r in RUNNING_REASONS)
    return ok / total if total else None


def frame_scheduler(ident, remaining):
  """(remaining, can_run, strict) for ONE frame of run all's pick: the same answers as `remaining`, `ident.can_run` and
  `ident.next_is_strict`, with each progress lookup, each test's next step and each step's feasibility computed once in the frame
  instead of once per question (pick asks can_run up to four times per test and strict once per test, and every answer re-ran
  the step search). Exact, not rate-limited: nothing that the answers depend on (progress, the generator's limit inputs and frame
  count) changes between pick's questions, which all come before the generator's update. Make a new one every frame."""
  rem_memo, feas_memo, step_memo, limit_memo = {}, {}, {}, {}
  inputs = ident.inputs
  finite = inputs is None or all(math.isfinite(x) for x in inputs.values())
  rearm = REARM_S / DT

  def rem(k, b, s):
    key = (k, b, s)
    if key not in rem_memo:
      rem_memo[key] = remaining(k, b, s)
    return rem_memo[key]

  def feasible(k, s, w):
    # ident._feasible, with its limit check made once per (strict or small step, window): limit_state depends on the step only
    # through limit_bounds, i.e. through small(k, s) (tested equal frame by frame)
    key = (k, s, w)
    if key not in feas_memo:
      if inputs is None:
        ok = True
      elif not finite:
        ok = False
      else:
        strict_ = not small(k, s)
        lk = (strict_, w)
        if lk not in limit_memo:
          limit_memo[lk] = limit_open(strict_, w, inputs["base_request"], inputs["curvature"], inputs["lat_demand"], inputs["ti_max"])
        ok = limit_memo[lk] and (ident.n - ident.viol_at.get((k, s), (-10 ** 9, None))[0]) > rearm
      feas_memo[key] = ok
    return feas_memo[key]

  def next_step(k, w):
    key = (k, w)
    if key not in step_memo:
      step_memo[key] = ident._pick(k, w, rem, None, lambda s: feasible(k, s, w))[0]
    return step_memo[key]

  run_memo, strict_memo = {}, {}

  def can_run(k, w):
    got = run_memo.get((k, w))
    if got is None:
      if inputs is None or w is None:
        got = True
      elif not finite:
        got = False
      else:
        step = next_step(k, w)
        got = step is not None and feasible(k, step, w)
      run_memo[(k, w)] = got
    return got

  def strict(k, w):
    got = strict_memo.get((k, w))
    if got is None:
      step = next_step(k, w)
      got = strict_memo[(k, w)] = step is not None and not small(k, step)
    return got

  stats_memo = {}

  def stats(w):
    """(totals, shares) of RunAll.pick for window w: window_total / band_share of every test, or (None, None) when unknown."""
    if w not in stats_memo:
      totals, shares = {}, {}
      for k in RUN_ALL_ORDER:
        totals[k] = window_total(k, w, rem)
        shares[k] = band_share(k, w, rem)
        if totals[k] is None or shares[k] is None:
          stats_memo[w] = (None, None)
          return stats_memo[w]
      stats_memo[w] = (totals, shares)
    return stats_memo[w]

  can_run.stats = stats
  return rem, can_run, strict


_REP = {}


def _class_rep():
  """One (test, step) per step class (small or not): limit_state depends on the step only through small()."""
  if not _REP:
    for k in KEYS:
      for s in range(n_steps(k)):
        # per-frame-ok: filled once while empty, keyed by small() (2 values)
        _REP.setdefault(small(k, s), (k, s))
  return _REP


class SchedulerCache:
  """frame_scheduler's answers kept from frame to frame while nothing they depend on has changed: the window, whether the limit
  inputs exist and are finite, whether each step class's limits are open in the window, which steps are inside their re-arm dwell,
  and the progress (the accountant's `version`). Exact: when that key is unchanged every answer is the same, and anything not yet
  asked is computed on the current frame's state. Without a version (None) it is per frame only (frame_scheduler).
  `frame(ident, remaining, profile, version)` -> (remaining, can_run, strict)."""

  def __init__(self):
    self.key = None
    self.fns = None

  def frame(self, ident, remaining, profile, version):
    if version is None or version[1] is None:
      self.key = None
      return frame_scheduler(ident, remaining)
    inputs = ident.inputs
    isfin = math.isfinite
    if inputs is None:
      finite = True
    else:
      base, curv, lat, tmax = inputs["base_request"], inputs["curvature"], inputs["lat_demand"], inputs["ti_max"]
      finite = isfin(base) and isfin(curv) and isfin(lat) and isfin(tmax)
    lims = ()
    if inputs is not None and finite and profile is not None:
      lims = (limit_open(False, profile, base, curv, lat, tmax), limit_open(True, profile, base, curv, lat, tmax))
    rearm = REARM_S / DT
    n = ident.n
    dwell = frozenset({ks for ks, v in ident.viol_at.items() if not (n - v[0]) > rearm}) if ident.viol_at else frozenset()
    key = (id(ident), profile, inputs is None, finite, lims, dwell, version)
    if key != self.key or self.fns is None:
      self.key, self.fns = key, frame_scheduler(ident, remaining)
    return self.fns


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
