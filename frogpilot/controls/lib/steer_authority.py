"""How much steering this car can actually hold, and how much slower a corner needs to be taken.

The Mazda + Torque Interceptor drives through two actuators that are both hard-clipped: the
interceptor at 600 counts (a limit inside the unit -- a TiSteerMax of 650 delivered no more) and the
stock LKAS path at the EPS's own +-308. With both saturated the rack winds up until self-aligning
torque plus static friction balances the assist, and then it stops. Measured across the log corpus
that latch sits at roughly 205 counts of column torque, which at 73 km/h is 21-24 degrees of
steering and about 2.6-2.7 m/s^2 of lateral acceleration. Nothing in the command path moves it.

The useful part is what does not change with speed. For a corner of a given radius the steering
angle required is geometry -- slowing down does not reduce it. What slowing reduces is the torque
needed to *hold* that angle, because self-aligning torque falls with speed. So the whole question
becomes: at what speed does the torque this corner demands drop under the ceiling?

SAT per degree, fitted on 87k hands-off quasi-static frames above 10 degrees of steering, using a
strict driver filter (no hand anywhere in the previous 4 s -- the rack holds what a hand put it at,
so an instantaneous filter admits hysteresis and overstates what the car can do alone):

    45-60 km/h    7.07 counts/deg
    60-70         9.39
    70-80        11.83
    80-95        14.08
    95-135       19.15                 r^2 0.995, scaling as v^1.28

The fit is worth trusting because it predicts something it was not fitted to: a 205-count ceiling at
70-80 km/h implies a 22.0 degree limit, and the observed latch in that band is 21-24 degrees.

What this module does NOT know is how much lane there is. A corner that demands more than the car
can hold is not automatically a corner the car leaves -- five recorded passes completed a 3.0 m/s^2
corner while delivering only the 2.65 ceiling, because the lane absorbed the ~0.5 m of path error.
So the advisory here is the conservative one: the speed at which the car can actually track the
plan. Expect it to ask for more slowing than strictly needed until the margin model earns its place;
`TARGET_MARGIN` is the knob, and the logged prediction-versus-outcome is what should move it.
"""
# RUNTIME: runs in frogpilot_process at 20 Hz (each modelV2, on-road, engaged or not); gc disabled (see docs/RUNTIME_CONSTRAINTS.md).
# Per call: create no class or closure, keep no container without a bound, nothing that grows with drive time.

# NOTE (2026-11-22): the 0x75 field named STEER_TORQUE / columnTorque is NOT column torque. It is the car's own
# lateral acceleration sensor, about 96-103 counts per m/s^2 (so 202 counts is roughly 2.1 m/s^2), and it includes road
# banking. Checked on recorded drives: parked, it does not move with driver torque; moving, it follows speed x yaw
# rate. The names and the thresholds below are kept as tuned; read "column torque" / "column load" as lateral
# acceleration. No motor-current or unsaturated torque signal has been found on the bus.

import time
from collections import deque

import numpy as np

from openpilot.selfdrive.controls.lib import lane_drift as LD

# Fitted SAT slope and intercept against speed. Torque = slope * |angle_deg| + intercept.
SAT_SPEED_KPH = [52.5, 65.0, 75.0, 87.5, 115.0]
SAT_SLOPE = [7.07, 9.39, 11.83, 14.08, 19.15]
SAT_INTERCEPT = [-37.3, -42.3, -54.7, -54.7, -67.4]

CEILING_COUNTS = 205.0     # where the rack stops answering, both actuators saturated

# Debounce for the "steering has stopped mid-corner" latch, in 20 Hz planner frames. It should
# sit at the duration that DEFINES a stall -- 0.30 s sustained; shorter is a direction reversal --
# not above it. Replayed over the corpus (~14 h, 12 ceiling-limited corners): the original 15
# frames (0.75 s) latched in 9 of the 12 and 24 times overall; 6 frames (0.30 s) latches in 12 of
# 12 and 72 times overall, every one at the clip with the wheel stopped and the column loaded,
# i.e. moments the driver already feels. The driver's report was silence on hard right-handers;
# part of that is this margin, and if it persists after this change the remainder is presentation
# (how long the alert stays on screen), not detection.
LATCH_DEBOUNCE_FRAMES = 6

# "Running wide" -- the deficit-based out-of-torque alert. Raw deficit alone fires on every entry
# ramp (61 times in 42 min: transient plant lag is not torque exhaustion). The discriminator is
# the ceiling: deficit while the command is already at max IS being out of torque.
#
# RECALIBRATED 2026-08-22 on route 0000027a (the 92-min drive that road-tested the first cut).
# As shipped (deficit 0.4, sustain 0.8 s) the alert fired 1.9-10.5 s AFTER the command pinned --
# by the time 0.4 m/s^2 of shortfall accrues the driver is already correcting -- and missed one
# episode outright. Replayed over 26 saturation episodes on 3 routes (3.9 engaged hours):
# deficit > 0.15 sustained 0.4 s hits 25/26 with median lead +2.0 s after pin (the physics floor
# for a deficit-confirmed alert: the shortfall takes ~2 s to become measurable) and ZERO fires
# outside episode windows. Saturation alone cannot replace the deficit term: the corpus's best
# corner pass held the clip 1.3 s with no deficit at all.
RUNWIDE_DEFICIT = 0.15     # m/s^2 the car is short of the ask
RUNWIDE_CEIL = 0.96        # of max command: only counts as out-of-torque at the ceiling
RUNWIDE_HOLD_FRAMES = 8    # 0.4 s at the 20 Hz planner rate
RUNWIDE_MIN_KPH = 40.0
# Once triggered, the EVENT stays asserted this long so the alert is readable. Without it the
# trigger flaps with the stick-slip cycle: measured on route 0000027a the displayed alert came in
# 0.2-0.9 s bursts alternating with the latch text several times per corner, each replaying its
# chime -- unreadable chatter, reported by the driver as "not being alerted" despite 20 logged
# event transitions.
RUNWIDE_DISPLAY_HOLD_FRAMES = 60   # 3 s at 20 Hz
LATCH_DISPLAY_HOLD_FRAMES = 40     # 2 s; the latch condition itself clears on one frame of rack motion

# TI dropout: the interceptor leaving RUN mid-drive removes most of the car's steering authority
# instantly (measured on the 0x11 event: achieved lateral accel 2.6 -> 0.3 within half a second,
# 32 s to self-recover). The alert latches for a few seconds so it cannot flash unseen.
TI_DROPOUT_HOLD_FRAMES = 60   # 3 s at 20 Hz
TARGET_MARGIN = 15.0       # counts of headroom to aim for, so the advice is not knife-edge

MIN_ADVISORY_KPH = 30.0    # below this the plant is a different animal and the fit does not reach
# The search floor is the bottom of the fitted range, not an arbitrary low speed. Below 52.5 km/h
# sat_torque() clips its interpolation and stops measuring anything -- so a search allowed to go
# there will happily converge on a confident-looking number the data does not support. Refusing is
# the honest output: an advisory that says nothing is recoverable, one that says 35 is not.
MIN_MODELLED_KPH = 52.5
MAX_SLOWDOWN_KPH = 40.0    # refuse to display an implausible number; means the model is out of range
MIN_ANGLE_DEG = 5.0        # below this there is no meaningful corner


def sat_torque(angle_deg, v_kph):
  """Column torque needed to hold a steering angle at a speed, in interceptor counts."""
  v = float(np.clip(v_kph, SAT_SPEED_KPH[0], SAT_SPEED_KPH[-1]))
  slope = float(np.interp(v, SAT_SPEED_KPH, SAT_SLOPE))
  intercept = float(np.interp(v, SAT_SPEED_KPH, SAT_INTERCEPT))
  return max(0.0, slope * abs(angle_deg) + intercept)


def required_angle_deg(curvature, wheelbase, steer_ratio):
  """Steering angle a curvature needs. Geometry, so speed does not enter."""
  return abs(float(curvature)) * float(wheelbase) * float(steer_ratio) * 180.0 / np.pi


def advisory_speed_kph(curvature, v_kph, wheelbase, steer_ratio,
                       ceiling=CEILING_COUNTS, margin=TARGET_MARGIN):
  """The speed at which this corner comes inside the car's steering authority.

  Returns 0.0 when the corner is already within it, or when the answer is outside the range the
  fit covers -- better to show nothing than a number the model cannot support."""
  angle = required_angle_deg(curvature, wheelbase, steer_ratio)
  if angle < MIN_ANGLE_DEG or v_kph <= MIN_ADVISORY_FLOOR_KPH:
    return 0.0
  target = ceiling - margin
  if sat_torque(angle, v_kph) <= target:
    return 0.0
  # Torque rises monotonically with speed at fixed angle, so bisect. 40 iterations is exact to
  # far below display resolution and costs nothing at 20 Hz.
  lo, hi = MIN_ADVISORY_FLOOR_KPH, float(v_kph)
  if sat_torque(angle, lo) > target:
    return 0.0     # even the slowest speed we model will not fit: out of range, say nothing
  for _ in range(40):
    mid = 0.5 * (lo + hi)
    if sat_torque(angle, mid) > target:
      hi = mid
    else:
      lo = mid
  advised = 0.5 * (lo + hi)
  if v_kph - advised > MAX_SLOWDOWN_KPH:
    return 0.0
  if v_kph - advised < MIN_MEANINGFUL_SLOWDOWN_KPH:
    return 0.0
  return advised


def torque_headroom(angle_deg, v_kph, ceiling=CEILING_COUNTS):
  """Counts of column torque still available at this angle and speed. Negative means past the latch."""
  return ceiling - sat_torque(angle_deg, v_kph)

# CORRECTED 2026-08-19. This previously converted a ceiling ANGLE to lateral acceleration with a
# pure v^2 map, giving a ceiling that rose with speed (2.21 at 52 km/h to 3.29 at 90). That was a
# double-count: the fitted SAT slope already contains the understeer denominator (L + K*v^2), and
# the deviation of its exponent from 2 *is* that denominator, so converting again with v^2 applied
# it twice.
#
# Done consistently the torque ceiling is a front-axle FORCE ceiling, which makes the lateral
# acceleration ceiling flat in speed. Measured over 61 latch events (command pinned, rack stopped
# 0.3 s, no hand within 4 s): the slope of delivered lateral acceleration against speed is
# -0.003 m/s^2 per km/h -- flat -- against the +0.028 the old model assumed. The single 113 km/h
# latch delivers 2.62, sitting inside the 72-73 km/h cluster rather than the 3.9 the rising model
# predicted.
#
# The old model's failure mode was a FALSE NEGATIVE at speed: a corner demanding 3.2 m/s^2 at
# 90 km/h scored as fitting comfortably when in fact the car cannot hold it.
CEILING_LA = 2.55            # m/s^2, speed-flat, zero-bank. Observed range 2.4-2.7.

# Below the EPS's own LKAS gate (~45-52 km/h with hysteresis) the stock path drops out entirely and
# authority falls by 3-4x. That is the one regime where slowing genuinely costs more than it buys,
# so the advisory never recommends entering it.
MIN_ADVISORY_FLOOR_KPH = 55.0

# How much lateral room the shortfall is allowed to eat. This is what separates a corner the car
# tracks from a corner the car completes: five recorded passes ran 0.4-0.9 m wide of the planned
# line and finished comfortably, because the lane had 0.7-2.0 m of outside margin. Demanding zero
# shortfall would advise slowing on every one of them.
ALLOWED_DRIFT_M = 1.0
PEAK_DURATION_S = 2.0        # nominal time the shortfall acts for; see the caveat in the module doc

# The FIRING threshold for the spoken advisory, separate from the drift budget above and much
# lower. Requiring the full 1.0 m budget before speaking demands a forecast above ~3.05 m/s^2,
# and the model's far horizon under-forecasts its own escalation -- measured on 92 minutes of
# route 0000027a it exceeded that on none of 8 saturation episodes: the advisory spoke once, 1.5 s
# LATE. At 0.3 m (forecast ~2.7) it reaches 3.5-4.3 s of lead on the repeat-offender S-bend
# corners. Every fire at this threshold across 3.9 engaged hours / 3 routes was a genuine
# >=2.75 m/s^2 ask (the three "false" fires on route 00000277 were 3.2-3.4 asks the driver was
# already handling by hand). Coverage is structurally partial -- 11 of 26 episodes -- because the
# forecast is the model's own plan; the running-wide alert above is the reliable reactive layer.
#
# TRIGGERED ONLY AGAINST THE SPEED-FLAT CEILING AND ONLY AT/ABOVE THE ADVISORY FLOOR SPEED.
# Below 55 km/h la_at_ceiling() returns the uncalibrated 0.3x floor, and a momentary forecast
# blip against that floor reads as metres of drift: both recorded phantom spikes (5.0 m of
# "drift" on a straight road) were this branch at 49-55 km/h.
ADVISORY_FIRE_DRIFT_M = 0.3
# Occupancy trigger, not a plain sustain: the forecast crosses the threshold in sub-0.2 s blips
# during approach (a plain 0.75 s sustain erased every early lead in replay). Fire when the
# threshold held for ADVISORY_OCC_NEED of the last ADVISORY_OCC_WINDOW planner frames.
ADVISORY_OCC_WINDOW = 30     # 1.5 s at 20 Hz
ADVISORY_OCC_NEED = 4        # 0.2 s cumulative
# Once fired, the advisory speed stays displayed this long (refreshed while the trigger holds),
# and the displayed number may only fall during a hold -- the raw value dithered 42/41/40 at
# 10 Hz on the one occasion it displayed, re-chiming on every change.
ADVISORY_HOLD_FRAMES = 80    # 4 s at 20 Hz

# Do not display a slowdown too small to act on. Both constants above come from measurement, not
# from fitting: a joint sweep over ceiling and drift budget scored better on the 22-pass corpus, but
# it won by picking a ceiling below the measured range and a drift budget wider than the lane, and
# the winning cell then missed a 3.2 m/s^2 corner at 90 km/h entirely. The corpus holds no fast hard
# corners, so it cannot see that failure. Trimming trivial advisories at the display is honest;
# moving the physics to win a 22-point fit is not.
MIN_MEANINGFUL_SLOWDOWN_KPH = 2.0


def la_at_ceiling(v_kph=None, ceiling=None):
  """Lateral acceleration the car can hold, m/s^2.

  Speed-flat above the LKAS gate. The argument is accepted and ignored above the floor, kept so
  callers read naturally and so the speed dependence has one obvious place to come back if it is
  ever measured rather than assumed."""
  if v_kph is not None and v_kph < MIN_ADVISORY_FLOOR_KPH:
    return CEILING_LA * 0.3      # stock path gone; a floor, not a calibrated number
  return CEILING_LA

def predicted_drift_m(demand_la, v_kph, duration_s=PEAK_DURATION_S):
  """How far wide of the planned line the car ends up, if it simply runs out of steering."""
  deficit = max(0.0, float(demand_la) - la_at_ceiling(v_kph))
  return 0.5 * deficit * duration_s ** 2


class AdvisoryTrigger:
  """When to SAY the advisory, and what number to hold on screen. Pure 20 Hz logic so it can be
  replayed and tested off the car; the planner owns one and feeds it per frame.

  Fire on threshold occupancy (the forecast crosses in sub-0.2 s blips on approach -- a plain
  sustain erased every early lead in replay), hold the display ADVISORY_HOLD_FRAMES, and only
  ever revise the shown speed downward while it is showing. Trigger judged against the
  speed-flat ceiling at advisory speeds only; the sub-55 km/h ceiling branch turned forecast
  blips into 5 m phantom drifts on straight road, twice, in one drive."""

  def __init__(self):
    self.occ = deque(maxlen=ADVISORY_OCC_WINDOW)
    self.hold = 0
    self.shown_kph = 0.0

  def update(self, active, forecast_la, v_kph):
    """Returns the advisory speed to display in km/h, 0.0 for silence."""
    flat_drift = 0.5 * max(0.0, float(forecast_la) - CEILING_LA) * PEAK_DURATION_S ** 2
    self.occ.append(bool(active) and v_kph >= MIN_ADVISORY_FLOOR_KPH
                    and flat_drift > ADVISORY_FIRE_DRIFT_M)
    if sum(self.occ) >= ADVISORY_OCC_NEED:
      # a drift budget consistent with the firing threshold, not the 1.0 m completion budget --
      # the completion budget yields a speed the under-forecasting model can rarely justify
      advised = advisory_speed_margin(forecast_la, v_kph, allowed_drift=ADVISORY_FIRE_DRIFT_M)
      if advised > 0:
        self.hold = ADVISORY_HOLD_FRAMES
        if self.shown_kph == 0.0 or advised < self.shown_kph:
          self.shown_kph = advised
    if self.hold > 0:
      self.hold -= 1
      if self.hold == 0:
        self.shown_kph = 0.0
    return self.shown_kph


def advisory_speed_margin(demand_la, v_kph, allowed_drift=ALLOWED_DRIFT_M,
                          duration_s=PEAK_DURATION_S):
  """Speed at which the shortfall stops costing more lane than `allowed_drift`.

  This is the criterion that matches what actually happened on the road, rather than the stricter
  "the car tracks the plan exactly". Returns 0.0 when the corner already fits."""
  if v_kph <= MIN_ADVISORY_FLOOR_KPH or demand_la <= 0.0:
    return 0.0
  if predicted_drift_m(demand_la, v_kph, duration_s) <= allowed_drift:
    return 0.0
  lo, hi = MIN_ADVISORY_FLOOR_KPH, float(v_kph)
  scaled = lambda v: demand_la * (v / float(v_kph)) ** 2      # a corner is a fixed radius
  if predicted_drift_m(scaled(lo), lo, duration_s) > allowed_drift:
    return 0.0     # out of the range the fit covers; say nothing rather than guess
  # Invariant: lo is a speed that fits, hi is one that does not. Drift rises with speed, so an
  # unsafe midpoint means the answer is BELOW it. Getting this backwards collapses the search onto
  # MIN_ADVISORY_KPH, which MAX_SLOWDOWN_KPH then suppresses entirely -- the advisory silently never
  # fires, which is exactly how it failed its first acceptance run.
  for _ in range(40):
    mid = 0.5 * (lo + hi)
    if predicted_drift_m(scaled(mid), mid, duration_s) > allowed_drift:
      hi = mid
    else:
      lo = mid
  advised = 0.5 * (lo + hi)
  if v_kph - advised > MAX_SLOWDOWN_KPH:
    return 0.0
  if v_kph - advised < MIN_MEANINGFUL_SLOWDOWN_KPH:
    return 0.0
  return advised


# ---- the ti_authority record (replayability, owner 2026-11-26 item 6) ---------------------------------------------------------------
# The small counters behind "Running Wide - Slow Down" (frogpilot_events.py: runwide_frames, runwide_display_hold), "Steering Has
# Stopped Mid-Corner" (frogpilot_planner.py: latch_frames, latch_bits; frogpilot_events.py: latch_display_hold) and "Steering Assist
# Dropped" (ti_dropout_hold), with their gate decisions, in one record per CHANGE of the discrete state "s" (AUTHORITY_BITS, bit 0
# first), written by frogpilot_process (20 Hz) through lane_drift.OnChange: at most 5 a second, "sup" counts what the cap held back.
# The analogue values are carried as they were at the change (never compared): enough afterwards to tell WHY an alert fired -- the
# command (st, of max) and whether it was at the ceiling, wheel rate (sr, deg/s) and column load (col, counts) the latch used, asked
# and delivered lateral acceleration (des / act, m/s^2: delivering what was asked is not losing the bend), speed (v, m/s) and the car
# side's clearance to the outside line (oc, m, from the frogpilot_process lane gate).
AUTHORITY_BITS = ("lat_active", "ti_active", "runwide_base", "drift_open", "runwide_shown", "latch_cond", "latched", "latch_shown",
                  "dropout_shown", "advisory_on")
AUTHORITY_MAX_B = 260        # bytes of the record as JSON (about 200 in practice)
# FrogPilotEvents / FrogPilotPlanner state of these alerts: logged (record key) or exempt (why a replay does not need it).
# Guarded by selfdrive/car/tests/test_mazda_ti_alert_replayable.py.
AUTHORITY_EVENTS_STATE = {"runwide_frames": "rwf", "runwide_display_hold": "rwd", "latch_display_hold": "ltd", "ti_dropout_hold": "dd",
                          "ti_was_active": "bit ti_active (of the previous record)", "lane_drift_gate": "the lane_gate record (p f)"}
AUTHORITY_EVENTS_EXEMPT = {"authority_log": "the record gate itself (OnChange)", "lane_gate_log": "the lane_gate record gate (OnChange)"}
AUTHORITY_PLANNER_STATE = {"latch_frames": "ltf", "latch_bits": "lb", "steer_latched": "bit latched"}
AUTHORITY_PLANNER_EXEMPT = {
  "steer_advisory_speed": "published: frogpilotPlan.steerAdvisorySpeed in the rlog",
  "steer_predicted_drift": "published: frogpilotPlan.steerPredictedDrift in the rlog",
  "advisory_trigger": "AdvisoryTrigger: inputs (modelV2 road curvature, speed) in the rlog, output steerAdvisorySpeed in the rlog",
  "curve_tick": "a call counter (reads at 10 Hz)",
  "curve_alert_mph": "read from TiCurvePlan / TiCurveDeliver, logged by the planner (tick) and the car side (deliver_phase)",
  "curve_alert_t": "as curve_alert_mph",
  "curve_code": "as curve_alert_mph",
  "curve_code_t": "as curve_alert_mph",
}


def authority_key(lat_active, ti_now, base, drift, rw_shown, latch_cond, latched, lt_shown, dd_shown, adv_on):
  """The alerts' discrete state as an int (AUTHORITY_BITS): booleans only."""
  return ((1 if lat_active else 0) | (2 if ti_now else 0) | (4 if base else 0) | (8 if drift else 0) | (16 if rw_shown else 0)
          | (32 if latch_cond else 0) | (64 if latched else 0) | (128 if lt_shown else 0) | (256 if dd_shown else 0)
          | (512 if adv_on else 0))


def authority_bits(s):
  """The record's "s" as {name: bool} (for the drive review)."""
  return {n: bool(s >> i & 1) for i, n in enumerate(AUTHORITY_BITS)}


def log_authority(log, key, events, planner, v_ego, des_la, act_la, steer, steer_rate, column):
  """Once per frogpilot_process frame, after the alerts' blocks: one ti_authority record when `key` changed (its latch_cond bit is
  set here from planner.latch_bits). Never raises; after an error, off for this process."""
  if log.dead:
    return
  try:
    if planner.latch_bits == 7:
      key |= 32                  # latch_cond: at the clip, wheel stopped and column loaded this frame
    if log.due(key):
      oc = events.lane_drift_gate.oc
      LD.emit_event("ti_authority", {
        "m": round(time.monotonic(), 3), "s": key, "rwf": events.runwide_frames, "rwd": events.runwide_display_hold,
        "ltf": planner.latch_frames, "lb": planner.latch_bits, "ltd": events.latch_display_hold, "dd": events.ti_dropout_hold,
        "ceil": 1 if abs(steer) >= RUNWIDE_CEIL else 0, "v": round(float(v_ego), 2), "des": round(float(des_la), 2),
        "act": round(float(act_la), 2), "st": round(float(steer), 3), "sr": round(float(steer_rate), 1),
        "col": round(float(column), 1), "oc": None if oc is None else round(float(oc), 2), "sup": log.take_sup()})
  except Exception:
    log.dead = True
