"""Curve slowing planner (pure logic): where the car is on the remembered roads, and the set-speed ceiling for the corners ahead.

Runs in its own low-rate process (ti_curve_plannerd.py, 5 Hz, normal priority, never on the control core); the car controller only
reads its small output (PARAM_PLAN, tmpfs) and does the button taps (ti_curve.py). Pure: no clock, no params, no messaging; the
process and the tests feed it one tick at a time.

1. Localise on the road memory (ti_curve_memory.py): nearest remembered point within MATCH_RADIUS_M whose direction of travel agrees
   with the car's within MATCH_HEADING_DEG; between fixes the position advances by the distance driven. A match is trusted only after
   it has stayed consistent (same trace, along-road position agreeing with the distance driven) for STABLE_S. Trust is dropped when
   the fixes stop agreeing for MISS_S, when the fix has been missing for FIX_LOST_S, when the remembered road ends, and when the curvature
   the car measures over the last CHANGE_WINDOW_M stops matching the remembered curvature (the road may have changed: no trust for the
   next CHANGE_HOLDOFF_M).
2. Look ahead along the trace (following its continuation links) LOOKAHEAD_S of driving, at least LOOKAHEAD_MIN_M, at most
   LOOKAHEAD_MAX_M. For every remembered point the speed the steering can hold (ti_curve_demand.need_mph): the highest whole mph at
   which the MEDIAN pass's curvature (memory kap_med; never the sharpest-pass kap; a file without kap_med has no memory stage) asks no
   more than the measured capacity of its direction (CAP_RAW: no bank term until the memory's bank is corrected), checked over
   +-0.75 mph. Graded (plan A6): the target is where demand equals capacity, so the drop follows the excess.
3. Slowing for a corner starts at the LATEST moment that still arrives at its speed ENTRY_M before it, given the measured response
   (see the constants below); closed loop every tick against the planned approach (push / let off / behind plan); held through the
   corner and released HOLD_AFTER_M past it. Between corners the output is a "hold" ceiling: the highest set speed a restore may reach
   from which the NEXT corner can still be made, so chained corners keep only what the next one needs. The ceiling is the lowest due speed; it holds through the
   corner and is released HOLD_AFTER_M after the last tight point has been passed.

4. Stage 2 (late, forecast-confirmed; `_stage2`, constants FC_LO_S .. FORK_NEAR_M): the driving model's preview 1.5-6 s ahead,
   calibrated, sets a ceiling at most LATE_SHED_MPH below the speed once confirmed, on any road (trusted memory or not); the lower
   of the memory ramp (stage 1) and stage 2 wins.

Output (`Plan`): ceiling in whole mph (None: no ceiling), confidence, trusted, known road, reason, distance to the limiting point, and the
model's own preview (largest predicted lateral acceleration in the next MODEL_LOOK_S), which the car side needs before it restores.
No position ever leaves this module: logs carry trace / point indices, distances and curvatures only.
"""
import math
from typing import NamedTuple, Optional

import numpy as np

from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
from openpilot.selfdrive.car.mazda import ti_curve_demand as D
from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.selfdrive.car.mazda import ti_curve_monitor as TM

MPH = 0.44704
fits = D.fits                   # re-exported: the car side's restore check (ti_curve.py)

MODEL_K_SIGN = -1.0             # the model preview (ti_curve_plannerd.model_curvature: orientationRate.z / velocity.x) is openpilot's
                                # curvature, RIGHT positive (latcontrol_angle / _pid steer from -desired_curvature); the planner's yaw_k
                                # and the memory are LEFT positive: preview curvature times this is left positive
AHEAD_S = 3.0                   # Plan.ahead_la: the signed peak calibrated forecast demand within this lead
ASK_LEADS = TM.ASK_LEADS        # Plan.ask: the calibrated signed forecast demand at these leads (stage 3's MON_REMAIN_S)
# ---- stage 2: late, forecast-confirmed (plan T5). The model preview between FC_LO_S and CONFIRM_LEAD_S ahead, calibrated by FC_RATIO
# (ti_curve_demand.forecast_need), judged against the capacity of its direction at the speed now; a bend it shows needing at least
# MIN_DROP_MPH below the speed, CONFIRM_TICKS planner ticks in a row (FORK_CONFIRM_S while trusted memory says the road ahead is
# straight: at a fork the model may be showing the other road), sets a ceiling at most LATE_SHED_MPH below the speed it began at
# (never below the floor); a bend needing more gets that plus the warning. Held until the forecast has not shown it for HOLD_AFTER_M
# past where it last did; suppressed while a blinker is on and BLINKER_HOLD_S after (a turn or a lane change).
FC_LO_S = 1.5
CONFIRM_LEAD_S = 6.0
CONFIRM_TICKS = 2
FORK_CONFIRM_S = 3.0
LATE_SHED_MPH = 5
S1_MIN_PASSES = 1               # the owner's standing rule (RULES.md, 2026-11-23): "One pass is enough for full slowing; phantom
#                                 protection comes from the pass-quality filters only". R15 (A15: at least 3 passes) contradicted it
#                                 and is undone; the per-point mechanism stays so the alternative can be measured and switched here:
#                                 above 1, stage 1 acts only on remembered points with at least this many passes (Memory.npass), and a
#                                 bend whose constraining points have fewer is left to stage 2 and the warning
BLINKER_HOLD_S = 2.0            # assumption (not in the plan's table)
S2_TAP_LAG_S = 0.5             # the warning's stage-2 prediction: plan read and first tap after confirmation (assumption)
S2_MIN_MPH = 20.0               # never act below 19-20 mph (the cruise and the comma engage from ~19)
FORK_NEAR_M = 30.0              # "memory straight" around the forecast's bend: no tight remembered point within this of its distance
MODEL_CONFIRM_SHARE = 0.6       # the model confirms a remembered bend when its sharpest curvature is at least this share of it
# ---- stage 1: early geometry, with release (plan T6, A3, A6). Acts only on a bend needing MORE than LATE_SHED_MPH below the speed
# (smaller ones are stage 2's). Its ceiling is T + LATE_SHED_MPH: the FIRST part of ONE ramp, timed (approach_m, the latest start) to
# arrive at the entry; stage 1 hands over to stage 2 (Plan.stage 2, same source) as the dash comes within one tap of T + 5, so the
# walk never pauses there (no early dwell, no step, no re-acceleration). Once per bend, CONFIRM_LEAD_S before its entry (distance /
# speed), with a preview REACHING the entry, the bend is RELEASED when the calibrated forecast demand (peak over FC_LO_S ..
# MODEL_LOOK_S, scaled to the target speed) is below CONFIRM_SHARE x its capacity (the bend is not there: a wrong map, a changed road):
# the ceiling goes, the set speed is restored, and a released bend stays released until passed. Without such a preview it is never
# released. Sources: trusted memory ("mem"); where no memory is known the map ("map": unsigned, so the LOWER side's capacity), a map
# bend acting only once it has persisted map_persist_s (0 s up to MAP_PERSIST_LO_MPH, rising linearly to MAP_PERSIST_S at
# MAP_PERSIST_HI_MPH and above: phantoms last <= 2.1 s); a missing or stale map (plannerd passes None) has no map stage.
CONFIRM_SHARE = 0.6
# R16 (A15): before the forecast confirms a stage-1 bend, stage 1 sheds at most S1_UNCONFIRMED_MAX_MPH (the dash no lower than the
# speed at the start minus this); the rest only after confirmation. ONE decision per bend, ONE definition: the bend is CONFIRMED at
# the first tick a preview REACHING its entry (and no further than MODEL_LOOK_S) shows it asking at least CONFIRM_SHARE x its
# capacity at T (the release check's own test, passed); it is RELEASED at the first tick within CONFIRM_LEAD_S of the entry at which
# such a preview does not show it (as before); a bend whose preview never reaches the entry by CONFIRM_LEAD_S is UNCONFIRMED and keeps
# the cap to its end ("s1_unconfirmed"; the warning covers the rest). A bend started with no preview able to confirm it (none: the
# model not running; or one shorter than CONFIRM_LEAD_S, which could never reach the entry at the decision) has nothing to wait for:
# no cap, decided at CONFIRM_LEAD_S as before.
# ONE switch (the owner is to be asked): None = no cap (stage 1 walks its full graded ramp from the start, as before R16).
S1_UNCONFIRMED_MAX_MPH = 5


def s1_unconfirmed_ceiling(u0_mph):
  """R16: the lowest dash set speed stage 1 asks for before its bend is confirmed, for a start at u0_mph; None with the cap off."""
  if S1_UNCONFIRMED_MAX_MPH is None:
    return None
  return int(math.ceil(float(u0_mph) - 1e-6)) - S1_UNCONFIRMED_MAX_MPH


MAP_PERSIST_S = 2.5
MAP_PERSIST_LO_MPH = 45.0
MAP_PERSIST_HI_MPH = 50.0
MAP_KEY_M = 40.0                # a map bend is the same bend from tick to tick while its end stays within this (odometer)
HOLD_GAP_S = 6.0                # between corners: hold a reduced speed only if the next slowdown would start within this of now
# The on-road warning (first drive, 2026-11-23: a corner needing ~35 mph was reached at 40 and the car crossed the lane line, with no
# alert). Raised, once per corner, when the corner's own speed is below what the feature may (the floor) or can (behind plan) reach,
# early enough for the driver to brake calmly: lead ALERT_LEAD_S plus the distance to brake to the corner speed at CALM_BRAKE_MS2
ALERT_LEAD_S = 4.0
CALM_BRAKE_MS2 = 1.5
ALERT_MARGIN_MPH = 2
LATCH_EXTRA_S = 10.0            # a corner on trusted road this many seconds of driving inside the warning horizon is LATCHED (odometer)
LATCH_PAST_M = 30.0             # ... and kept until the car is this far past its end, whatever the trust (fourth review, MUST 1)
OBS_WINDOW_S = 2.0              # the cruise's OBSERVED deceleration over this, against what it was measured to give for the same gap
OBS_MIN_EXPECT = 0.5            # ... judged only while it should be slowing at least this much (mph/s)
OBS_PRE_S = 1.5                 # ... the model is run from this long before the window (dead time + lag: 0.5 + 2 x 0.5 s)
OBS_SIM_DT_S = 0.05             # ... in steps of this
OBS_KEEP_S = 10.0               # ... and the last judgement kept this long (to the corner) once the gap closes
RAISE_SLACK_MPH = 0.5           # a warning starts only above need + ALERT_MARGIN_MPH + this (speed noise at the margin); ends at the margin
SETTLE_MPH = 0.3                # within this of its set speed the cruise is taken to be at it
SIM_DT_S = 0.2                  # the arrival prediction's time step
ACCEL_MPH_S = CM.ACCEL_MPH_S    # the stock cruise speeding up toward its set speed (measured: 5 mph in 7-9 s)
SPEED_WINDOW_MPH = D.SPEED_WINDOW_MPH   # a speed fits only if every speed within this of it fits (band edges, speed error)
ENTRY_LA = 1.3                  # a corner's entry (where the approach aims) is its first point whose demand reaches this (m/s^2) at the
                                # corner's own target speed (owner, 2026-11-24: arrive just in time). None reaches it: its sharpest point
CAR_MIN_MPH = 19                # the car's own lowest set speed
MAX_MPH = 90

# ---- when to start: as LATE as still reaches the corner speed (owner, 2026-11-23: "we should ensure that we're not slowing down
# further ahead than we need to ... We only want to do what we need to, speed change wise, and speed of speed change wise.")
# How the car answers our taps is ti_cruise_model (the ONE measured cruise model, plan amendment A1, 2026-11-24): the dash walks 1 mph
# per tap, one tap per 0.30 s; the cruise acts on the gap (speed minus set speed) 0.5 s late through a 0.5 s first-order lag, with the
# measured, grade-corrected deceleration for that gap (0.21 m/s^2 at 1-2 mph rising to 0.90 at 10-14 mph, capped there). The same
# model is used by the warning's arrival prediction, the replay tools and every test car. The approach is: PUSH (set DRIVE_GAP_MPH
# below the corner speed, never below the floor), then set the corner speed itself at the moment the same model says the car will
# bottom out at it (CM.raise_due; plan A9-R2a: no dip below the corner speed and re-acceleration). With several constraints ahead the
# lowest allowed speed decides: a lower one appearing after the push ended starts a new push (A9-R2b).
# The start is the latest moment that still arrives at T + ARRIVED_MPH by ENTRY_M before the corner, with BUFFER_S of margin. CLOSED
# LOOP every tick: the car's speed is compared with the highest speed from which it can still make it (`v_allow`); a car slowing less
# than planned (downhill or anything else) is pushed to the floor; one slowing more (uphill) is let off to the corner speed.
REACTION_S = CM.DELAY_S         # the model's dead time on the gap (0.5 s); the measured ~1.4 s from the first tap to the car slowing
                                # 0.5 mph is not a separate constant: it comes out of the model (dead time, lag, weak small-gap decel)
TAP_S = CM.TAP_S                # one set-speed tap per this (measured walk: 0.30 s per tap, 0 of 293 taps missed)
DRIVE_GAP_MPH = 10 if CM.HOLD else 6   # push this far below the corner speed (plan A10: 10 with the held button, 6 with taps only)
RAISE_AT_MPH = 1.5              # only when the dash is NOT below the corner speed (no push under way): "on target" within this
RAISE_SIM_DT_S = 0.1            # the planner's raise prediction steps (CM.raise_due)
ARRIVED_MPH = 0.5               # plan A8-R1: at the corner speed + this (was 1.5, the measured manoeuvre definition CM.ARRIVED_MPH;
                                # 1.0 ruled; the largest value <= 1.0 passing every suite on a 0.1 grid is 0.5: the lead-held e2e cases)
RESP_FACTOR = 1.0               # plan with the measured deceleration
BUFFER_S = 0.3                  # extra seconds of start-early margin (was 1.0; 2026-11-24, arrive just in time)
ENTRY_M = 0.0                   # be at the corner speed this far before the aim point (was 10 m; 2026-11-24)
COAST_BELOW_MPH = 3.0           # pushing and this far below v_allow: let off to the corner speed
REPUSH_MPH = 1.0                # ... and push again when back within this of v_allow
BEHIND_GRACE_S = 3.0            # not "behind" in the first seconds of a start: a start fires AT the latest-start speed and the cruise
                                # needs its reaction plus the taps before the speed falls (2026-11-23: "behind" at 6 of 10 starts)
BEHIND_MPH = 2.0                # above v_allow by this: behind plan (the gap is already at its most useful: nothing more to ask of the
                                # set speed; the corner is flagged "needs more than I can slow" and logged)
MIN_SWITCH_S = 3.0              # no push / let-off reversal within this unless behind plan
GRADE_STEEP_DEG = 3.0           # remembered approach steeper downhill than this: start earlier
GRADE_BUFFER_S_PER_DEG = 0.5    # ... by this per degree beyond GRADE_STEEP_DEG (no data beyond +-3 degrees: a guess, logged)
MIN_DROP_MPH = 2                # a point needing less than this below the speed is ignored (the car can hold it at its speed)
HOLD_AFTER_M = 30.0             # keep the ceiling this far past the last tight point


def decel_mph_s(gap):
  """The stock cruise's measured settled deceleration (mph/s) for a gap (mph): ti_cruise_model, timing variant."""
  return CM.decel_mph_s(gap)


_APPROACH = {}


def approach_m(u, T, S1, dt=0.05, set0=None):
  """Metres from the first press until the speed is within ARRIVED_MPH of T, starting at u mph with the set speed at `set0` (default u;
  A8-R3: above u when a lead holds the car below it), tapping down to S1 (one tap per TAP_S), raising to T when the model says (A9-R2);
  the car answers as ti_cruise_model says. Cached by whole mph."""
  key = (int(u), int(T), int(S1), int(u) if set0 is None else max(int(u), int(round(set0))))
  r = _APPROACH.get(key)
  if r is not None:
    return r
  _, dist = CM.manoeuvre(float(key[0]), key[1], key[2], raise_at=None, arrived=ARRIVED_MPH, dt=dt, tap_s=TAP_S,
                         set0=float(key[3]), hold=CM.HOLD)
  _APPROACH[key] = dist
  return dist


def map_persist_s(mph):
  """Plan A3: how long a map bend must have been shown before stage 1 acts on it."""
  return MAP_PERSIST_S * min(1.0, max(0.0, (mph - MAP_PERSIST_LO_MPH) / (MAP_PERSIST_HI_MPH - MAP_PERSIST_LO_MPH)))


def v_allow(D, T, S1, buf_s):
  """The highest speed (mph, fractional) from which the car can still be at T by distance D with the push approach and buf_s of
  margin at that speed; T when even T + 1 cannot."""
  def need(u):
    return approach_m(u, T, S1) + buf_s * u * MPH
  lo = int(T)
  if need(lo + 1) > D:
    return float(T)
  hi = MAX_MPH
  if need(hi) <= D:
    return float(hi)
  while hi - lo > 1:
    mid = (lo + hi) // 2
    if need(mid) <= D:
      lo = mid
    else:
      hi = mid
  a, b = need(lo), need(hi)
  return lo + (0.0 if b <= a else min(1.0, max(0.0, (D - a) / (b - a))))


def slow_distance_m(from_mph, to_mph, floor_mph=CAR_MIN_MPH):
  """Metres before the corner at which slowing from `from_mph` to `to_mph` must start (approach + buffer + ENTRY_M)."""
  if from_mph - to_mph <= 0:
    return ENTRY_M
  S1 = max(CM.push_floor(floor_mph, DRIVE_GAP_MPH), to_mph - DRIVE_GAP_MPH)
  return approach_m(from_mph, to_mph, S1) + BUFFER_S * from_mph * MPH + ENTRY_M


# ---- look-ahead (long-horizon study: warning beyond 25-30 s stops helping) -----------------------------------------------------
LOOKAHEAD_S = 45.0
LOOKAHEAD_MIN_M = 300.0
LOOKAHEAD_MAX_M = 2000.0

# ---- localisation ----------------------------------------------------------------------------------------------------------------
MATCH_RADIUS_M = 25.0
MATCH_HEADING_DEG = 30.0
# WHICH road, by geometry (owner, 2026-11-23: "an on-ramp shouldn't look the same as the highway itself"). Remembered roads overlap where
# they share tarmac (a ramp's record starts on the highway it leaves). Two rules: (1) while another remembered road lies as near the
# car sideways as the locked one (within AMBIG_M: a lane, the position error), the car may be on either, and a corner counts only if
# EVERY such road has one about there (within AMBIG_SHIFT_M along the road); (2) once another road is nearer by SWITCH_MARGIN_M for
# SWITCH_S the lock is on the wrong one and is dropped (the nearer road is then matched in the ordinary way).
AMBIG_M = 4.0
AMBIG_SHIFT_M = 50.0
AMBIG_MAX = 3
SWITCH_MARGIN_M = 3.0
SWITCH_S = 1.0
TRACK_WINDOW_M = 40.0           # while locked, a fix must land within this of the predicted along-road position
FIX_GAIN = 0.3                  # how far a fix pulls the dead-reckoned position
STABLE_S = 3.0                  # a new match must stay consistent this long before it is trusted
MISS_S = 2.0                    # fixes that do not agree for this long: lock lost
FIX_STALE_S = 1.0               # no valid fix for this long: confidence reduced
FIX_LOST_S = 5.0                # ... for this long: trust dropped
MIN_TRUST_CONF = 0.7

# ---- the road may have changed (study's change detection, kept simple) -----------------------------------------------------------
CHANGE_WINDOW_M = 300.0
CHANGE_MIN_M = 200.0            # judged only once this much has been compared
CHANGE_BEND_K = 0.0015          # rms curvature (1/m) below which the window is "straight" in both: nothing to judge
CHANGE_SHIFT_M = 50.0           # the comparison allows the two curvature traces to be offset by up to this along the road
CHANGE_MIN_CORR = 0.3           # their best correlation below this: the road's shape differs
CHANGE_FLAT_STD = 0.0006        # curvature spread (1/m) below which a window has no shape: compare levels instead
CHANGE_LEVEL_SHARE = 0.35       # ... a level less than this share of the other (or the opposite way) differs
CHANGE_STRIKES = 20             # ... on this many consecutive planner ticks (4 s at 5 Hz) before trust is dropped
CHANGE_HOLDOFF_M = 150.0        # a distrust costs this much road (was 500 m: 16.5 % of a drive lost to false changes)
LIVE_K_TAU_S = 0.5              # smoothing of the live curvature (the memory is smoothed over ~10 m)

# ---- the model's own preview -----------------------------------------------------------------------------------------------------
MODEL_LOOK_S = 8.0

def entry_point(run, kabs, T):
  """The point of a tight run (array of look-ahead indices) to aim the approach at: the first whose demand (remembered median |k|, no
  bank term) reaches ENTRY_LA at the corner's target T (mph); none does: the run's sharpest point (the first of equals)."""
  la = kabs[run] * (T * MPH) ** 2
  hit = np.nonzero(la >= ENTRY_LA)[0]
  return int(run[hit[0]]) if len(hit) else int(run[int(np.argmax(kabs[run]))])


class Fix(NamedTuple):
  valid: bool
  x: float = 0.0                  # local metres (Memory.to_local)
  y: float = 0.0
  hdg: float = 0.0                # radians, east = 0, counter-clockwise


class Plan(NamedTuple):
  ceil: Optional[int]             # set-speed ceiling, whole mph; None: no ceiling
  conf: float
  trusted: bool
  known: bool                     # a remembered road is matched here (trusted or not yet)
  why: str
  dist_m: Optional[float]         # to the limiting point
  corner_mph: Optional[int]       # the speed the limiting point allows
  past: bool                      # no ceiling and no corner due: past any corner
  model_la: Optional[float]       # largest |predicted lateral accel| in the model's next MODEL_LOOK_S (None: no model)
  capped: bool                    # the corner wants more than the car side may shed (logged for the owner)
  limit_mph: Optional[int] = None # the road's known speed limit (frogpilotPlan.slcMapSpeedLimit), None when unknown
  model_k: Optional[float] = None # the sharpest |curvature| in the model's next MODEL_LOOK_S (the car side's restore check)
  kind: str = ""                  # "corner": slow to the ceiling; "hold": the highest set speed a restore may reach (never slows)
  end_m: Optional[float] = None   # a corner in progress: metres to the END of its tight stretch (dist_m is to its start)
  behind: bool = False            # the car is slowing less than planned (not the same as capped)
  alert_mph: Optional[int] = None # warn the driver: the corner ahead needs this speed and the feature will not get there
  need_mph: Optional[int] = None  # the corner's TRUE need (demand against capacity), NOT clamped to the floor (corner_mph is)
  trace: int = -1
  point: int = -1
  stage: int = 0                  # what set the ceiling: 0 none, 1 memory ramp (early geometry), 2 late forecast-confirmed
  src: str = ""                   # "mem" / "map" / "fc" (forecast); "" with no ceiling
  ahead_la: Optional[float] = None  # signed (left +) peak calibrated forecast demand within AHEAD_S, m/s^2 (None: no preview)
  cap: Optional[float] = None     # the capacity (m/s^2) the limiting bend was judged against, at its target and side
  released: bool = False          # a bend ahead was released by the forecast (plan T6; kept until passed)
  diag: Optional[dict] = None     # small per-tick decision record for the tick log (plan T8); never a position
  ask: Optional[tuple] = None     # calibrated signed (left +) forecast demand at ASK_LEADS, m/s^2 (stage 3, plan T7); None: no preview


def bearing_to_hdg(bearing_deg):
  """Compass bearing (degrees, north = 0, clockwise) to the memory's heading (radians, east = 0, counter-clockwise)."""
  return M.wrap(math.radians(90.0 - bearing_deg))


class Planner:
  MAX_SHED_MPH = 15               # mirrors ti_curve.MAX_SHED_MPH, only to flag in the log when a corner needs more
  ABS_FLOOR_MPH = 25              # mirrors ti_curve.ABS_FLOOR_MPH

  def __init__(self, memory):
    self.mem = memory
    self.need = None
    self.need_hi = None
    self.hi = True                      # R14: this tick plans on D.need_hi (the car at or above STEP_MPH, or a bend started there)
    self.mem_stage = False
    if memory is not None:
      # plan from the MEDIAN pass only (plan T4: never the sharpest-pass kap); the same figure serves the road-change check
      self.kmed = np.asarray(getattr(memory, "kap_med", memory.kap), np.float64)
      self.mem_stage = bool(getattr(memory, "has_kap_med", True))
      # bank = 0 and CAP_RAW until the memory carries the corrected bank (Memory.bank_v >= 2; plan section 5, default 4)
      self.need = D.need_mph(self.kmed).astype(np.float64) if self.mem_stage else np.full(memory.n, float(MAX_MPH))
      # R14: the need of a car at or above D.STEP_MPH (never planned into the low-capacity zone merely because capacity steps there)
      self.need_hi = D.need_hi(self.kmed).astype(np.float64) if self.mem_stage else self.need
      self.kabs = np.abs(self.kmed)
    self.limit = None
    self.reset()

  def reset(self):
    self.t = None
    self.trace, self.pos = -1, 0.0      # pos: fractional global point index on the trace
    self.locked = False
    self.stable_s = 0.0
    self.cand = None                    # (trace, along metres, odometer) of the tentative match
    self.miss_s = 0.0
    self.alts, self.off_s = [], 0.0     # other remembered roads as near as the locked one; seconds one has been nearer
    self.fix_age = 1e9
    self.odo = 0.0
    self.live_k = 0.0
    self.cmp = []                       # (odometer, live k, remembered k), every spacing of travel while locked
    self.cmp_next = 0.0
    self.change_strikes = 0
    self.distrust_until = -1.0
    self.distrust_why = ""
    self.active = {}                    # point -> corner in progress: T (corner speed), at (odometer of the point), mode, since
    self.int_floor = None
    self.int_base = None
    self.end_m = None
    self.behind = False
    self.alert = None
    self.alerting = set()
    self.alerted = set()
    self.hist = []                      # (t, mph, set mph) over the last OBS_WINDOW_S + REACTION_S
    self.resp = RESP_FACTOR             # the share of the measured deceleration the cruise is giving now (never above RESP_FACTOR)
    self.resp_t = -1e9
    self.latch = {}                     # key -> corner (need, entry / low / end on the odometer): warned for whatever the trust
    self.model_k = None
    self.events = []
    self.fc = None
    self.ahead_la = None
    self.s2 = None                      # stage 2 in force: T, need, u0 (the speed it began at), until (odometer), alert
    self.s2_run, self.s2_t0 = 0, 0.0    # consecutive ticks the forecast has shown a bend, and since when
    self.blink_t = -1e9
    self.released = {}                  # plan T6: bend key -> its end (odometer); released by the forecast, kept until passed
    self.map_active, self.map_bends, self.map_seq = {}, {}, 0   # the map stage's bends in progress / seen (A3 persistence)
    self.dr_why = ""                    # why the corners in progress are being dead-reckoned (the last lock loss), for the log
    # review C1 (2026-11-24): the published ceiling per corner in progress, whatever its source: key -> {cap (the lowest non-push
    # ceiling published for it), until (its end + HOLD_AFTER_M, odometer), entry (odometer), ref (the driver's set speed then)}; while
    # one is alive the published ceiling is never above its cap (_guard_apply)
    self.guard = {}
    self.s2_seq = 0

  # ---- localisation ----------------------------------------------------------------------------------------------------------------
  def _candidates(self, fix):
    mem = self.mem
    idx = mem.near(fix.x, fix.y, MATCH_RADIUS_M)
    if len(idx) == 0:
      return []
    dh = np.abs((mem.hdg[idx] - fix.hdg + math.pi) % (2 * math.pi) - math.pi)
    idx = idx[dh <= math.radians(MATCH_HEADING_DEG)]
    out = []
    for i in idx:
      along, lat = mem.project(int(i), fix.x, fix.y)
      if abs(along) <= mem.spacing:
        out.append((abs(lat), int(mem.trace_of[i]), int(i) + along / mem.spacing))
    out.sort()
    return out

  def _lose(self, why, keep_active=True):
    """Drop the lock. The corners in progress (slowing started, or the car inside the bend) are KEPT whatever the reason (a lost
    position fix, another road, a changed road, the remembered road ending, the fixes leaving it): they run on by DEAD RECKONING on the
    odometer to their end + HOLD_AFTER_M (bounded: their entry / end are odometer figures), with the reason recorded (review C1,
    2026-11-24: a lock loss inside a bend being slowed for must never raise the set speed there). A corner is only dropped early by
    the explicit release decision (_ramp) or by the driver (the guard, `_guard_apply`). keep_active=False is kept for a caller that
    must start afresh (none in the planner itself)."""
    if self.locked or self.cand is not None:
      self.events.append({"kind": "lock_lost", "why": why, "trace": self.trace})
    self.locked, self.cand, self.stable_s, self.miss_s = False, None, 0.0, 0.0
    self.trace = -1
    self.alts, self.off_s = [], 0.0
    self.cmp = []
    if not keep_active:
      self.active = {}
    elif self.active:
      self.dr_why = why
      self.events.append({"kind": "dead_reckon", "why": why, "n": len(self.active),
                          "T": int(min(a_["T"] for a_ in self.active.values()))})

  def _advance(self, dist):
    """Move along the trace by `dist` metres; follows the continuation link at the end, or loses the lock."""
    mem = self.mem
    self.pos += dist / mem.spacing
    end = mem.trace_end(self.trace) - 1
    while self.pos > end:
      nxt = int(mem.trace_next[self.trace])
      if nxt < 0:
        self._lose("remembered road ends")
        return
      over = self.pos - end - 1
      self.trace = nxt
      self.pos = mem.trace_start[nxt] + max(0.0, over)
      end = mem.trace_end(nxt) - 1

  def _localise(self, dt, fix, v):
    mem = self.mem
    if self.locked:
      self._advance(v * dt)
    if self.cand is not None:
      self.cand = (self.cand[0], self.cand[1] + v * dt / mem.spacing)
    if fix.valid:
      self.fix_age = 0.0
      cands = self._candidates(fix)
      if self.locked:
        best = None
        for lat, tr, p in cands:
          dpos = self._gap(tr, p)
          if dpos is not None and abs(dpos) * mem.spacing <= TRACK_WINDOW_M:
            best = (lat, tr, p, dpos)
            break
        if best is not None:
          self.miss_s = 0.0
          if best[1] == self.trace:
            self.pos += FIX_GAIN * best[3]
          # the other remembered roads here (not this trace nor its continuation): as near sideways, or nearer
          alts, seen = [], set()
          for lat, tr, p in cands:
            if tr in seen or self._gap(tr, p) is not None:
              continue
            seen.add(tr)
            if lat <= best[0] + AMBIG_M and len(alts) < AMBIG_MAX:
              alts.append((lat, tr, p))
          self.alts = [(tr, p) for _, tr, p in alts]
          if alts and alts[0][0] + SWITCH_MARGIN_M < best[0]:
            self.off_s += dt
            if self.off_s >= SWITCH_S:
              self._lose("a nearer remembered road")
          else:
            self.off_s = 0.0
        else:
          self.miss_s += dt
          if self.miss_s >= MISS_S:
            self._lose("fixes left the remembered road")
      if not self.locked:
        if not cands:
          self.cand, self.stable_s = None, 0.0
        else:
          lat, tr, p = cands[0]
          if self.cand is not None and self.cand[0] == tr and abs(self.cand[1] - p) * mem.spacing <= TRACK_WINDOW_M / 2:
            self.stable_s += dt
            self.cand = (tr, p)
          else:
            self.cand, self.stable_s = (tr, p), 0.0
          if self.stable_s >= STABLE_S:
            self.locked, self.trace, self.pos = True, tr, p
            self.miss_s = 0.0
            self.cmp = []
            self.cmp_next = self.odo
            self.events.append({"kind": "lock", "trace": tr, "point": int(p)})
    else:
      self.fix_age += dt
      self.alts = []
      if self.cand is not None and not self.locked:
        self.cand, self.stable_s = None, 0.0
      if self.locked and self.fix_age >= FIX_LOST_S:
        self._lose("no position fix")

  def _either_road(self, need, dist, look, mph):
    """While the car may as well be on another remembered road (self.alts), a point's allowed speed is at least the lowest the other
    road allows within AMBIG_SHIFT_M of the same distance ahead: a corner only one of them has is not slowed for."""
    tight = np.nonzero(need < mph)[0]
    if not len(tight):
      return need
    for tr, p in self.alts:
      i0 = int(math.floor(p))
      idx2, d2 = self.mem.ahead(tr, i0, look + AMBIG_SHIFT_M)
      if not len(idx2):
        continue
      d2 = d2 - (p - i0) * self.mem.spacing
      eff = (self.need_hi if self.hi else self.need)[idx2]
      for j in tight:
        if dist[j] > d2[-1]:
          continue                                   # the other road's record ends before here: nothing known against the corner
        w = np.abs(d2 - dist[j]) <= AMBIG_SHIFT_M
        if w.any():
          need[j] = max(need[j], float(eff[w].min()))
    return need

  def _gap(self, tr, p):
    """Points from the current position to candidate (tr, p) along the trace or its continuation; None if unrelated."""
    mem = self.mem
    if tr == self.trace:
      return p - self.pos
    nxt = int(mem.trace_next[self.trace])
    if tr == nxt:
      return (mem.trace_end(self.trace) - self.pos) + (p - mem.trace_start[nxt])
    for prev in range(mem.n_traces):
      if mem.trace_next[prev] == self.trace and tr == prev:
        return -((self.pos - mem.trace_start[self.trace]) + (mem.trace_end(prev) - p))
    return None

  def _check_change(self):
    """Live versus remembered curvature over the last CHANGE_WINDOW_M; True on a mismatch."""
    if not self.locked:
      return False
    while self.odo >= self.cmp_next:
      i = int(round(self.pos))
      i = min(max(i, 0), self.mem.n - 1)
      self.cmp.append((self.odo, self.live_k, float(self.kmed[i])))
      self.cmp_next += self.mem.spacing
    while self.cmp and self.odo - self.cmp[0][0] > CHANGE_WINDOW_M:
      self.cmp.pop(0)
    if not self.cmp or self.odo - self.cmp[0][0] < CHANGE_MIN_M:
      return False
    a = np.array(self.cmp)
    live, rem = a[:, 1], a[:, 2]
    rl, rr = math.sqrt(float(np.mean(live ** 2))), math.sqrt(float(np.mean(rem ** 2)))
    scale = max(rl, rr)
    if scale < CHANGE_BEND_K:
      self.change_strikes = 0
      return False
    # tolerant of where along the road the two line up (passes differ by 15 m typically, 50 m at worst) and of how sharp the stored
    # figure is (it is the second-sharpest pass, not this one): the SHAPE must disagree at every shift within CHANGE_SHIFT_M
    # a long even bend (or a straight) has no shape to correlate: compare LEVELS there (the same bend on both, within a factor)
    if live.std() < CHANGE_FLAT_STD or rem.std() < CHANGE_FLAT_STD:
      ml, mr = float(np.mean(live)), float(np.mean(rem))
      big = max(abs(ml), abs(mr))
      bad = big >= CHANGE_BEND_K and (ml * mr < 0 or min(abs(ml), abs(mr)) < CHANGE_LEVEL_SHARE * big)
      self.change_strikes = self.change_strikes + 1 if bad else 0
      return self.change_strikes >= CHANGE_STRIKES
    best = -1.0
    n = len(live)
    k = int(CHANGE_SHIFT_M / self.mem.spacing)
    # Pearson r by dot products (np.corrcoef costs ~50 us a call, and this ran up to 2k+1 times a tick: 60 % of the planner's time).
    # Shift 0 first, then outward; stop at the first shift that matches, since only `best < CHANGE_MIN_CORR` is used.
    for sh in sorted(range(-k, k + 1), key=abs):
      x = live[max(0, sh):n + min(0, sh)]
      y = rem[max(0, -sh):n - max(0, sh)]
      if len(x) < 20:
        continue
      xm, ym = x - x.mean(), y - y.mean()
      sxx, syy = float(xm @ xm), float(ym @ ym)
      if sxx < (1e-9 ** 2) * len(x) or syy < (1e-9 ** 2) * len(y):
        continue
      best = max(best, float(xm @ ym) / math.sqrt(sxx * syy))
      if best >= CHANGE_MIN_CORR:
        break
    bad = best < CHANGE_MIN_CORR
    self.change_strikes = self.change_strikes + 1 if bad else 0
    return self.change_strikes >= CHANGE_STRIKES

  # ---- one tick ------------------------------------------------------------------------------------------------------------------
  def step(self, t, fix, v, set_mph=None, yaw_k=None, model_k=None, limit_mph=None, ref_mph=None, floor_mph=None, acting=True,
           map_k=None, blinker=False):
    """t: seconds (monotonic); fix: Fix; v: m/s; set_mph: the cruise set speed or None; yaw_k: live curvature (1/m, left +, from the
    car's yaw sensor) or None; model_k: (seconds ahead array, curvature array) of the driving model's preview as it publishes it (RIGHT
    positive, see MODEL_K_SIGN), or None; map_k: the map's (metres ahead, |curvature|) profile (ti_curve_plannerd.map_profile; None
    when absent or stale), used by stage 1 only where no memory is known (plan T6);
    blinker: either turn signal on (suppresses stage 2). -> Plan."""
    dt = 0.0 if self.t is None else max(0.0, min(1.0, t - self.t))
    self.t = t
    self.ref_now = int(ref_mph) if ref_mph else None
    self.limit = int(round(limit_mph)) if limit_mph and limit_mph > 0 else None
    v = max(0.0, float(v))
    self.odo += v * dt
    self._observe(t, v / MPH, set_mph)
    if yaw_k is not None and math.isfinite(yaw_k):
      a = 1.0 if dt <= 0 else min(1.0, dt / LIVE_K_TAU_S)
      self.live_k += a * (yaw_k - self.live_k)
    model_la = None
    self.model_k = None
    self.fc = None                      # (lead times, LEFT-positive curvature) of the model preview this tick
    self.ahead_la = None
    self.ask = None
    if model_k is not None:
      ts, ks = model_k
      ts, ks = np.asarray(ts, np.float64), np.asarray(ks, np.float64)
      sel = ts <= MODEL_LOOK_S
      if len(ts) == len(ks) and sel.any() and np.all(np.isfinite(ks[sel])):
        kk = np.abs(ks[sel])
        j = int(np.argmax(kk))
        model_la = float(kk[j] * max(v, 1.0) ** 2)
        self.model_k = float(kk[j])
        self.fc = (ts, MODEL_K_SIGN * ks)
        self.fc_v = v
        _, _, la3, _ = D.forecast_need(ts, self.fc[1], v, 0.0, AHEAD_S)
        self.ahead_la = None if la3 is None else round(la3, 2)
        la_all = D.calibrate(ts, max(v, 1.0) ** 2 * self.fc[1])
        self.ask = tuple(round(float(x), 2) for x in np.interp(ASK_LEADS, ts, la_all))
    mph = v / MPH
    # R14: plan on the capacity above the step (D.need_hi) while the car is at or above D.STEP_MPH, or while a bend (stage 1, map or
    # stage 2) that started with it there is still in progress (so a car slowed to 32 that dips under it is not re-planned to 25); a
    # car already below STEP_MPH otherwise plans on the stepped, low-speed capacity
    self.hi = bool(mph >= D.STEP_MPH or any(a_.get("hi") for a_ in list(self.active.values()) + list(self.map_active.values()))
                   or (self.s2 is not None and self.s2.get("hi")))
    if blinker:
      self.blink_t = t
    self.released = {k: at for k, at in self.released.items() if self.odo <= at + HOLD_AFTER_M}
    if self.mem is None:
      floor = self._floor(floor_mph, ref_mph, set_mph, mph)
      self._stage2(t, v, mph, set_mph, floor, None)
      return self._map_out(t, v, mph, set_mph, floor, ref_mph, map_k, model_la, 0.0, False, "no road memory on the device")
    self._localise(dt, fix, v)
    if self.locked and yaw_k is not None and self.odo >= self.distrust_until and self._check_change():
      self.distrust_until = self.odo + CHANGE_HOLDOFF_M
      self.distrust_why = "road differs from memory"
      self.events.append({"kind": "change", "trace": self.trace, "point": int(self.pos)})
      self._lose("road differs from memory")
    conf = 0.0
    if self.locked:
      conf = 1.0 if self.fix_age < FIX_STALE_S else max(0.0, 1.0 - (self.fix_age - FIX_STALE_S) / (FIX_LOST_S - FIX_STALE_S))
    elif self.cand is not None:
      conf = 0.5 * min(1.0, self.stable_s / STABLE_S)
    known = self.locked or self.cand is not None
    mph = v / MPH
    floor = self._floor(floor_mph, ref_mph, set_mph, mph)
    untrusted = None
    if self.odo < self.distrust_until:
      untrusted = (0.0, known, self.distrust_why)
    elif not self.locked:
      untrusted = (conf, known, "matching the remembered road" if self.cand is not None else "road not known here")
    elif conf < MIN_TRUST_CONF:
      untrusted = (conf, True, "position uncertain")
    if untrusted is not None:
      # a corner latched while the road was trusted is still warned for, by the odometer, until it is passed: a GPS gap or a matcher
      # miss never drops (or delays) a warning (fourth review, MUST 1). Nothing more is tapped down without trust: predicted so.
      self.latch = {k: c for k, c in self.latch.items() if self.odo <= c["end"] + LATCH_PAST_M}
      self.alert = self._warn(list(self.latch.values()), mph, v, floor, acting, set_mph, lowering=False)
      self._stage2(t, v, mph, set_mph, floor, None)
      if self.active:
        # a remembered corner in progress when the position fix went (a GPS gap: "position uncertain", then "no position fix", then
        # "matching the remembered road" until the lock is back): it runs on by DEAD RECKONING on the odometer (its entry / end are
        # odometer figures) until passed, with no fresh constraint and nothing new started; its ceiling is valid (trusted), and
        # stage 2 merges below it in _out (the lower one wins), so a second source's higher opinion never raises the set speed
        # inside the bend (Job 2, 2026-11-24)
        self.map_active, self.map_bends = {}, {}
        return self._dead_reckoned(t, v, mph, set_mph, floor, ref_mph, model_la, untrusted)
      if untrusted[1]:
        self.map_active, self.map_bends = {}, {}  # a remembered road is being matched here: no map (plan T6)
        return self._out(None, untrusted[0], False, untrusted[1], untrusted[2], None, None, model_la)
      return self._map_out(t, v, mph, set_mph, floor, ref_mph, map_k, model_la, untrusted[0], untrusted[1], untrusted[2])

    # ---- the corners ahead -------------------------------------------------------------------------------------------------------
    self.map_active, self.map_bends = {}, {}     # trusted memory: the map is not used (plan T6: map only where no memory is known)
    u = max(mph, float(set_mph)) if set_mph else mph
    look = min(LOOKAHEAD_MAX_M, max(LOOKAHEAD_MIN_M, v * LOOKAHEAD_S))
    i0 = int(math.floor(self.pos))
    idx, dist = self.mem.ahead(self.trace, i0, look)
    dist = dist - (self.pos - i0) * self.mem.spacing
    need = self._either_road((self.need_hi if self.hi else self.need)[idx].astype(np.float64), dist, look, mph)
    # the floor the car side keeps for an intervention (the known limit, else the speed when it starts, minus MAX_SHED_MPH)
    # (kept from the first corner of a chain until the car is back near the speed it started from, as the car side does)
    # ONE source with the car side (2026-11-23 drive: a floor kept here from an earlier 55 mph episode clamped every corner for 11 min
    # while the driver's set speed was 45): the car side's published floor while it intervenes, else the floor it WOULD use now from
    # the driver's own set speed (`ref_mph`, its published reference); nothing is kept here between ticks
    self.int_floor = floor
    self.int_base = float(ref_mph) if ref_mph else None
    shed_to = np.maximum(need, floor)
    grade = getattr(self.mem, "grade", None)
    # tight against the speed now, the set speed, and (within a chain) the speed the chain started from, so a corner just after one
    # already slowed for still bounds the way back up (a "hold")
    u_ref = max(u, self.int_base or 0.0)
    sign = np.sign(self.kmed[idx])
    cands = self._bend_cands(lambda run: int(idx[run[-1]]), dist, need, self.kabs[idx], sign, shed_to, u_ref, mph, set_mph, floor,
                             self.active, grade_of=None if grade is None else (lambda j: self._grade(grade, idx, dist, j)),
                             npass=self.mem.npass[idx])
    r = self._ramp(t, v, mph, set_mph, floor, cands, self.active, u_ref)
    ceil, cdist, cpt, kind, corner = r["ceil"], r["dist"], r["point"], r["kind"], r["corner"]
    cap, need_true, cend, behind, stage = r["cap"], r["need"], r["end"], r["behind"], r["stage"]
    corners = self._corners(idx, dist, need, mph)
    # latch every corner within reach of the warning horizon (plus LATCH_EXTRA_S of driving) while the road is trusted, and keep it,
    # by the odometer, until the car is LATCH_PAST_M past its end: also through a wrong re-match (a trusted position that no longer
    # shows it). A corner seen now replaces its latched copy.
    keys = {c["key"] for c in corners}
    for c in corners:
      if c["entry"] - self.odo <= self._horizon(v, c["need"]) + v * LATCH_EXTRA_S:
        self.latch[c["key"]] = c
    self.latch = {k: c for k, c in self.latch.items() if self.odo <= c["end"] + LATCH_PAST_M}
    corners = corners + [c for k, c in self.latch.items() if k not in keys]
    self.alert = self._warn(corners, mph, v, floor, acting, set_mph, lowering=True, u_ref=u_ref)
    self._stage2(t, v, mph, set_mph, floor, (need, dist, u_ref))
    # a warned corner whose need is below the floor is "capped" too, also on the tick before its slowdown starts
    warned_below_floor = self.alert is not None and self.alert < floor
    if ceil is None:
      return self._out(None, conf, True, True, "no known corner ahead", None, None, model_la, past=True, capped=warned_below_floor)
    # "capped" means ONE thing: a corner's target is below the floor (more than the feature may shed); a car slowing less than planned
    # is "behind", reported separately and never as capped
    capped = (bool(self.active) and min(a_["need"] for a_ in self.active.values()) < floor) or warned_below_floor
    why = ("next corner: holding what it needs" if kind == "hold" else "corner ahead" if (cdist or 0) > 0 else "in the corner")
    self.end_m = cend
    self.behind = behind
    self.need_out = need_true
    self.cap_out = cap
    return self._out(int(ceil), conf, True, True, why, cdist, corner, model_la, capped=capped, point=cpt, kind=kind,
                     past=kind == "hold", stage=stage, src="mem", gkey=r["key"], guntil=r["until"])

  def _dead_reckoned(self, t, v, mph, set_mph, floor, ref_mph, model_la, untrusted):
    """The memory corners in progress through a position gap (step): the ramp with no fresh candidates, by the odometer."""
    u = max(mph, float(set_mph)) if set_mph else mph
    u_ref = max(u, float(ref_mph) if ref_mph else 0.0)
    r = self._ramp(t, v, mph, set_mph, floor, {}, self.active, u_ref)
    if r["ceil"] is None:
      return self._out(None, untrusted[0], False, untrusted[1], untrusted[2], None, None, model_la)
    self.end_m, self.behind, self.need_out, self.cap_out = r["end"], r["behind"], r["need"], r["cap"]
    capped = (bool(self.active) and min(a_["need"] for a_ in self.active.values()) < floor) or (
      self.alert is not None and self.alert < floor)
    why = "corner ahead" if (r["dist"] or 0) > 0 else "in the corner"
    return self._out(int(r["ceil"]), untrusted[0], True, untrusted[1], why, r["dist"], r["corner"], model_la, capped=capped,
                     point=r["point"], kind=r["kind"], stage=r["stage"], src="mem", gkey=r["key"], guntil=r["until"])

  def _map_out(self, t, v, mph, set_mph, floor, ref_mph, map_k, model_la, conf, known, why):
    """The output where no memory is known: the map stage's ramp (plan T6) if it has a ceiling, else none (stage 2 merges in _out)."""
    r = self._map_stage(t, v, mph, set_mph, floor, ref_mph, map_k)
    if r is None or r["ceil"] is None:
      return self._out(None, conf, False, known, why, None, None, model_la)
    self.end_m, self.behind, self.need_out, self.cap_out = r["end"], r["behind"], r["need"], r["cap"]
    capped = bool(self.map_active) and min(a_["need"] for a_ in self.map_active.values()) < floor
    why = "next bend (map): holding what it needs" if r["kind"] == "hold" else "bend ahead (map)"
    # trusted: this ceiling's source (the map, persisted, matched by plannerd) is valid
    return self._out(int(r["ceil"]), conf, True, known, why, r["dist"], r["corner"], model_la, capped=capped, point=-1,
                     kind=r["kind"], past=r["kind"] == "hold", stage=r["stage"], src="map", gkey=r["key"], guntil=r["until"])

  @staticmethod
  def _grade(grade, idx, dist, j):
    """Extra start-early buffer (s) for a remembered approach steeper downhill than GRADE_STEEP_DEG up to look-ahead point j."""
    if dist[j] <= 0:
      return 0.0
    sel = idx[(dist >= 0) & (dist <= dist[j]) & (idx >= 0)]
    g = float(np.degrees(np.nanmean(grade[sel]))) if len(sel) else 0.0
    if math.isfinite(g) and g < -GRADE_STEEP_DEG:
      return GRADE_BUFFER_S_PER_DEG * (-g - GRADE_STEEP_DEG)
    return 0.0

  def _bend_cands(self, key_of, dist, need, kab, sign, shed_to, u_ref, mph, set_mph, floor, active, grade_of=None, npass=None):
    """Stage 1's bends ahead (plan T6): runs of consecutive points needing at least MIN_DROP_MPH below u_ref; a run whose lowest
    target is not MORE than LATE_SHED_MPH below u_ref is stage 2's (not started here; kept while already started). key_of(run) -> the
    bend's key, or None (a map bend not yet persisted). Per bend its binding constraint: {T, need, va (v_allow), D, due, at, entry,
    aim_at, S1, buf, cap, side}. npass: the passes behind each look-ahead point (memory; None: the map); with S1_MIN_PASSES above 1
    (R15, off by default: the owner's one-pass rule) only points with at least that many constrain stage 1, and a bend with none such
    is not stage 1's."""
    cands = {}
    tight = np.nonzero((u_ref - shed_to) >= MIN_DROP_MPH)[0]
    if not len(tight):
      return cands
    breaks = np.nonzero(np.diff(tight) > 1)[0]
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, len(tight) - 1]
    for s0, s1 in zip(starts, ends):
      run = tight[s0:s1 + 1]
      # a corner is known by the END of its tight stretch: it stays the same while the car drives into the stretch (its first and
      # lowest points ahead change as they are passed), so a corner is started once
      key = key_of(run)
      if key is None or key in self.released:
        continue
      if npass is not None and S1_MIN_PASSES > 1:
        ok = npass[run] >= S1_MIN_PASSES          # R15 (off at 1): only remembered points with the evidence constrain stage 1
        if not ok.any():
          continue
      else:
        ok = np.ones(len(run), bool)
      if key not in active and u_ref - float(shed_to[run[ok]].min()) <= LATE_SHED_MPH:
        continue                                   # plan T6 / A6: a bend needing at most LATE_SHED_MPH off is stage 2's
      lowest = run[ok][int(np.argmin(need[run[ok]]))]
      # aimed at the corner's ENTRY (entry_point: the first point whose demand reaches ENTRY_LA at the corner's speed), not at the
      # run's first point (a gentle lead-in). GUARD: every tight point before the entry is a constraint of its own at its own speed
      # and distance, so the car is never planned to arrive at an earlier point faster than the capacity there allows. The lowest
      # point (the first of equals at or after the entry) is always a constraint.
      aim = entry_point(run, kab, int(shed_to[run].min()))
      tied = run[ok & (need[run] <= need[lowest]) & (run >= aim)]
      lowest = int(tied[0]) if len(tied) else int(lowest)
      pts = {int(aim), int(lowest)}
      pts.update(int(j) for j in run[ok & (run < aim)])
      side = int(sign[lowest])
      for j in sorted(pts):
        T = int(shed_to[j])
        D_ = float(dist[j]) - ENTRY_M
        buf = BUFFER_S + (grade_of(j) if grade_of is not None else 0.0)
        S1 = max(CM.push_floor(floor, DRIVE_GAP_MPH), T - DRIVE_GAP_MPH)
        va = v_allow(D_, T, S1, buf)
        # A8-R3: the set speed above the car's speed (a lead holds it there): the taps from the REAL set speed come first, so the
        # start is due when the model's manoeuvre from (speed, set speed) needs the remaining distance
        # (the car is never faster than its set speed, so the manoeuvre from (set, set) bounds it: the exact one only within that)
        due = bool(set_mph and set_mph > mph + SETTLE_MPH and
                   approach_m(math.ceil(set_mph), T, S1) + buf * mph * MPH >= D_ and
                   approach_m(math.ceil(mph), T, S1, set0=set_mph) + buf * mph * MPH >= D_)
        c = cands.get(key)
        if c is None or va < c["va"] or (due and not c["due"]):
          cands[key] = {"T": T, "need": int(need[j]), "Tc": int(need[j]), "va": va, "D": D_, "due": due,
                        "at": self.odo + float(dist[run[-1]]), "entry": self.odo + float(dist[run[0]]),
                        "aim_at": self.odo + float(dist[aim]), "S1": S1, "buf": buf, "cap": D.capacity(T, side), "side": side}
    return cands

  def _release_check(self, v, T, cap, lead_s):
    """Plan T6: True when the preview, REACHING the bend's entry (lead_s ahead), shows it asking less than CONFIRM_SHARE x its
    capacity: the calibrated peak demand over FC_LO_S .. MODEL_LOOK_S, scaled from the speed now to the target T. None: no preview,
    or one that does not reach the entry (never released then)."""
    if self.fc is None or float(np.max(self.fc[0])) < lead_s - 1e-6:
      return None
    _, _, la, _ = D.forecast_need(self.fc[0], self.fc[1], v, FC_LO_S, MODEL_LOOK_S)
    if la is None:
      return None
    la_T = abs(la) * (T * MPH) ** 2 / max(v, 1.0) ** 2
    return bool(la_T < CONFIRM_SHARE * cap)

  def _ramp(self, t, v, mph, set_mph, floor, cands, active, u_ref):
    """The closed-loop ramp for the bends in `cands` (started, pushed, let off, behind, raised, handed over, released, passed);
    `active` holds the bends in progress. -> dict(ceil, dist, point, kind, corner, need, cap, end, behind, stage)."""
    # start: the car is at or above the highest speed from which it can still make it (or, below its set speed, the start is due)
    for key, c in cands.items():
      if key not in active and (mph >= c["va"] or c["due"]) and mph - c["T"] >= MIN_DROP_MPH:
        active[key] = {"T": c["T"], "need": c["need"], "Tc": c["Tc"], "at": c["at"], "entry": c["entry"], "mode": "push",
                       "since": t, "start_t": t, "start_over": mph - c["va"], "cap": c["cap"], "phase": 1, "aim_at": c["aim_at"],
                       "side": c["side"], "hi": self.hi,
                       "cap_c": s1_unconfirmed_ceiling(mph) if (self.fc is not None and
                                                                 float(np.max(self.fc[0])) >= CONFIRM_LEAD_S - 1e-6) else None}  # R16
        self.events.append({"kind": "start", "point": key if isinstance(key, int) else -1, "T": c["T"], "need": c["need"],
                            "floor": floor, "mph": round(mph, 1), "planned": round(c["va"], 1), "dist": round(c["D"], 1)})
    for key in [k for k, a_ in active.items() if self.odo > a_["at"] + HOLD_AFTER_M]:
      self.events.append({"kind": "passed", "point": key if isinstance(key, int) else -1, "T": active[key]["T"], "mph": round(mph, 1)})
      del active[key]
    out = {"ceil": None, "dist": None, "point": -1, "kind": "", "corner": None, "need": None, "cap": None, "end": None,
           "behind": False, "stage": 1, "key": None, "until": None}
    for key, a_ in list(active.items()):
      c = cands.get(key)
      mode = a_["mode"]
      if c is not None and c["T"] < a_["T"]:
        # A9-R2b: a lower constraint of this corner now binds: a corner "on target" for the higher one is pushed again
        a_["T"] = c["T"]
        if mode == "target":
          mode = "push"
      T = a_["T"]
      S1 = max(CM.push_floor(floor, DRIVE_GAP_MPH), T - DRIVE_GAP_MPH)
      va = c["va"] if c is not None else float(T)
      late, on_plan = mph > va + BEHIND_MPH, mph <= va
      if c is not None and set_mph and mode in ("push", "behind"):
        # mid-approach the dash is already below the speed and the car is answering it: v_allow (a FRESH start from here) would call
        # an on-plan car behind. Judge with the same model instead: where it will be at the constraint with the taps it is getting,
        # with the SAME push-and-raise manoeuvre as the warning (R8, plan A12)
        pred = self._arrive(v, max(0.0, c["D"]), float(set_mph), float(S1), True, T=T)
        late, on_plan = pred > T + ARRIVED_MPH + BEHIND_MPH, pred <= T + ARRIVED_MPH
      d_aim = a_["aim_at"] - self.odo
      early = (not a_.get("decided") and a_.get("cap_c") is not None and self.fc is not None and
               CONFIRM_LEAD_S * max(v, 1.0) < d_aim <= MODEL_LOOK_S * max(v, 1.0))
      if early and self._release_check(v, T, a_["cap"], d_aim / max(v, 1.0)) is False:
        # R16: confirmed early: a preview reaching the entry already shows the bend (the one decision, taken now)
        a_["decided"], a_["cap_c"] = True, None
        self.events.append({"kind": "s1_confirm", "point": key if isinstance(key, int) else -1, "T": T, "mph": round(mph, 1),
                            "preview": True})
      if not a_.get("decided") and d_aim <= CONFIRM_LEAD_S * max(v, 1.0):
        # the release decision, once per bend, CONFIRM_LEAD_S before its entry (also when the hand-over came earlier)
        a_["decided"] = True
        rel = self._release_check(v, T, a_["cap"], d_aim / max(v, 1.0))
        if rel is None and a_.get("cap_c") is not None:
          # R16: no preview reaching the entry: never confirmed; the cap stays to the bend's end
          self.events.append({"kind": "s1_unconfirmed", "point": key if isinstance(key, int) else -1, "T": T, "mph": round(mph, 1)})
          rel = "unconfirmed"
        if rel is True:
          self.released[key] = a_["at"]
          self.guard.pop(key, None)               # the explicit release decision: the one early drop of a corner in progress
          self.events.append({"kind": "s1_release", "point": key if isinstance(key, int) else -1, "T": T, "mph": round(mph, 1)})
          del active[key]
          continue
        if rel != "unconfirmed":
          a_["cap_c"] = None
          self.events.append({"kind": "s1_confirm", "point": key if isinstance(key, int) else -1, "T": T, "mph": round(mph, 1),
                              "preview": rel is not None})
      if a_["phase"] == 1 and (a_.get("decided") or (set_mph is not None and set_mph <= T + LATE_SHED_MPH + 1.5)):
        # stage 1 -> 2 hand-over: at the release decision, or as soon as the dash is within one tap of T + LATE_SHED_MPH (judged a
        # planner tick early, so the walk never pauses there while the car side waits for the next plan): ONE ramp, no early dwell
        a_["phase"] = 2
        self.events.append({"kind": "s1_handover", "point": key if isinstance(key, int) else -1, "T": T, "mph": round(mph, 1)})
      if mode != "target":
        if self._raise_due(mph, set_mph, T):
          mode = "target"
        elif late and (t - a_["start_t"] >= BEHIND_GRACE_S or a_["start_over"] > BEHIND_MPH):
          mode = "behind"
        elif mode == "push" and mph < va - COAST_BELOW_MPH and t - a_["since"] >= MIN_SWITCH_S:
          mode = "coast"
        elif mode == "behind" and on_plan:
          mode = "push"
        elif mode == "coast" and mph >= va - REPUSH_MPH:
          mode = "push"
      if mode != a_["mode"]:
        self.events.append({"kind": "mode", "point": key if isinstance(key, int) else -1, "mode": mode, "mph": round(mph, 1),
                            "planned": round(va, 1), "T": T})
        a_["mode"], a_["since"] = mode, t
      cc = S1 if mode in ("push", "behind") else T
      if a_["phase"] == 1:
        cc = max(cc, T + LATE_SHED_MPH)           # stage 1's part of the ramp
      if a_.get("cap_c") is not None:
        cc = max(cc, a_["cap_c"])                 # R16: not yet confirmed: at most S1_UNCONFIRMED_MAX_MPH off the start speed
      if mode == "behind":
        out["behind"] = True
      if out["ceil"] is None or cc < out["ceil"]:
        out.update(ceil=cc, dist=round(max(0.0, a_["entry"] - self.odo), 1), point=key if isinstance(key, int) else -1,
                   kind="corner", corner=T, need=a_["Tc"], cap=a_.get("cap"), end=round(a_["at"] - self.odo, 1),
                   stage=a_["phase"], key=key, until=a_["at"] + HOLD_AFTER_M)
    if not active:
      # no corner in progress: hold a reduced set speed for the NEXT corner only when its slowdown would have to start within
      # HOLD_GAP_S anyway (from the speed the chain started from); otherwise restore fully and slow again later (traffic behind)
      hold = None
      for key, c in cands.items():
        need_m = approach_m(int(u_ref), c["T"], c["S1"]) + c["buf"] * u_ref * MPH
        if (c["D"] - need_m) / max(v, 1.0) > HOLD_GAP_S:
          continue
        if hold is None or c["va"] < hold[0]:
          hold = (c["va"], key, c)
      if hold is not None and int(math.floor(hold[0])) < MAX_MPH:
        out.update(ceil=int(math.floor(hold[0])), dist=round(hold[2]["D"], 1), point=hold[1] if isinstance(hold[1], int) else -1,
                   kind="hold", corner=hold[2]["T"], cap=hold[2]["cap"])
    return out

  def _map_stage(self, t, v, mph, set_mph, floor, ref_mph, map_k):
    """Stage 1 from the map (plan T6, A3) where no memory is known: map_k = (metres ahead, |curvature|) from
    ti_curve_plannerd.map_profile, or None (absent / stale: no map stage). Unsigned: judged on the LOWER side's capacity. A bend acts
    only once it has persisted map_persist_s(mph). -> the _ramp dict, or None."""
    ok = map_k is not None
    if ok:
      d, k = np.asarray(map_k[0], np.float64), np.abs(np.asarray(map_k[1], np.float64))
      ok = len(d) >= 2 and len(d) == len(k) and bool(np.all(np.isfinite(d)) and np.all(np.isfinite(k)))
    if not ok:
      self.map_active, self.map_bends = {}, {}
      return None
    nf = D.need_hi if self.hi else D.need_mph                                # R14
    need = np.minimum(nf(k), nf(-k)).astype(np.float64)                     # side unknown: the lower capacity
    u = max(mph, float(set_mph)) if set_mph else mph
    u_ref = max(u, float(ref_mph) if ref_mph else 0.0)
    shed_to = np.maximum(need, floor)
    seen = set()
    hold_s = map_persist_s(mph)

    def key_of(run):
      end = self.odo + float(d[run[-1]])
      key = None
      for kk, b in self.map_bends.items():
        if kk not in seen and abs(b["end"] - end) <= MAP_KEY_M:
          key = kk
          break
      if key is None:
        self.map_seq += 1
        key = ("map", self.map_seq)
        self.map_bends[key] = {"first": t}
      self.map_bends[key]["end"] = end
      seen.add(key)
      return key if (key in self.map_active or t - self.map_bends[key]["first"] >= hold_s - 1e-6) else None
    cands = self._bend_cands(key_of, d, need, k, np.zeros(len(k)), shed_to, u_ref, mph, set_mph, floor, self.map_active)
    self.map_bends = {kk: b for kk, b in self.map_bends.items() if kk in seen or kk in self.map_active}
    return self._ramp(t, v, mph, set_mph, floor, cands, self.map_active, u_ref)

  @staticmethod
  def _mem_tight(mem_ahead, lo_m, hi_m):
    """Trusted memory (mem_ahead: (need, dist, u_ref) of the look-ahead, None when not trusted) has a point between lo_m and hi_m
    ahead needing at least MIN_DROP_MPH below u_ref."""
    if mem_ahead is None:
      return False
    need, dist, u_ref = mem_ahead
    near = (dist >= lo_m) & (dist <= hi_m)
    return bool(np.any(near & ((u_ref - need) >= MIN_DROP_MPH)))

  def _stage2(self, t, v, mph, set_mph, floor, mem_ahead):
    """Stage 2 (plan T5): late, forecast-confirmed, at most LATE_SHED_MPH. Updates self.s2 (merged into the output by _out).
    mem_ahead: the trusted memory's look-ahead (need, dist, u_ref), or None (no memory, not trusted): with it, "memory straight" over
    the forecast window asks FORK_CONFIRM_S of confirmation, and a bend memory knows (a tight remembered point within FORK_NEAR_M of
    the forecast's bend) is the memory stage's to warn for: stage 2 adds no warning there."""
    u = max(mph, float(set_mph)) if set_mph else mph
    quiet = (self.fc is None or mph < S2_MIN_MPH or t - self.blink_t < BLINKER_HOLD_S)
    need, lead, side = None, None, D.UNKNOWN
    if not quiet:
      need, lead, _, side = D.forecast_need(self.fc[0], self.fc[1], v, FC_LO_S, CONFIRM_LEAD_S, hi=self.hi)
    base = self.s2["u0"] if self.s2 is not None else u
    hit = need is not None and need <= base - MIN_DROP_MPH
    if hit:
      if self.s2_run == 0:
        self.s2_t0 = t
      self.s2_run += 1
    else:
      self.s2_run = 0
    if self.s2 is not None and (t - self.blink_t < BLINKER_HOLD_S or mph < S2_MIN_MPH):
      # a blinker (or the speed under 20 mph) drops stage 2 at once; a missing preview only stops refreshing it (held to `until`)
      self.events.append({"kind": "s2_off", "why": "blinker" if t - self.blink_t < BLINKER_HOLD_S else "slow", "T": self.s2["T"]})
      if t - self.blink_t < BLINKER_HOLD_S:
        self.guard.pop(self.s2.get("key"), None)   # the driver's blinker: their decision (a turn off the road), the corner is theirs
      self.s2 = None
    mem_straight = mem_ahead is not None and not self._mem_tight(mem_ahead, FC_LO_S * v - FORK_NEAR_M, CONFIRM_LEAD_S * v + FORK_NEAR_M)
    confirmed = hit and self.s2_run >= CONFIRM_TICKS and (not mem_straight or t - self.s2_t0 >= FORK_CONFIRM_S - 1e-6)
    if confirmed:
      T = int(max(need, int(round(base)) - LATE_SHED_MPH, floor))
      until = self.odo + lead * v + HOLD_AFTER_M
      knew = self._mem_tight(mem_ahead, lead * v - FORK_NEAR_M, lead * v + FORK_NEAR_M)
      if self.s2 is None:
        self.s2_seq += 1
        self.s2 = {"T": T, "need": int(need), "u0": base, "until": until, "lead": lead, "knew": knew, "side": side, "hi": self.hi,
                   "key": ("s2", self.s2_seq)}
        self.events.append({"kind": "s2_start", "T": T, "need": int(need), "mph": round(mph, 1), "lead": round(lead, 1),
                            "floor": int(floor), "fork": bool(mem_straight)})
      else:
        if int(need) < self.s2["need"]:
          self.s2["side"] = side
        self.s2.update(T=min(self.s2["T"], T), need=min(self.s2["need"], int(need)), until=max(self.s2["until"], until), lead=lead,
                       knew=self.s2["knew"] or knew)
    elif hit and self.s2 is not None:
      self.s2["until"] = max(self.s2["until"], self.odo + lead * v + HOLD_AFTER_M)
    if self.s2 is not None and self.odo > self.s2["until"]:
      self.events.append({"kind": "s2_off", "why": "passed", "T": self.s2["T"]})
      self.s2 = None

  @staticmethod
  def _raise_due(mph, set_mph, T):
    """Switch a corner to its target: with the dash pushed below T, when the model says the car raised now bottoms out at T
    (CM.raise_due from the car settled on its current gap); otherwise within RAISE_AT_MPH of T."""
    if set_mph is not None and set_mph < T:
      return CM.raise_due(CM.Cruise(mph, float(set_mph), RAISE_SIM_DT_S), float(set_mph), T, hold=CM.HOLD)
    return mph <= T + RAISE_AT_MPH

  def _floor(self, floor_mph, ref_mph, set_mph, mph):
    """The floor, ONE source with the car side (fourth review, item 4): the car side's published floor (while it intervenes, or the
    one it would take now: lead-aware, limit-aware); only without a car-side status, the same rule from the set speed."""
    if floor_mph is not None:
      return int(floor_mph)
    base_ = ref_mph if ref_mph else (set_mph if set_mph else round(mph))
    return max(self.ABS_FLOOR_MPH, int(round(base_)) - self.MAX_SHED_MPH, (self.limit - self.MAX_SHED_MPH) if self.limit else 0)

  @staticmethod
  def _horizon(v, need):
    """The calm-braking horizon (m): ALERT_LEAD_S at this speed plus the distance to brake to the need at CALM_BRAKE_MS2."""
    vt = need * MPH
    return v * ALERT_LEAD_S + max(0.0, (v * v - vt * vt) / (2.0 * CALM_BRAKE_MS2))

  def _corners(self, idx, dist, need_true, mph):
    """The corners ahead the car is too fast for now: runs of points (at most 2 spacings apart) whose TRUE need (demand against capacity;
    speed; never the floor) is below mph - ALERT_MARGIN_MPH, up to HOLD_AFTER_M behind. Each: key (its last point, or the model's
    point), need (whole mph, the run's lowest), entry / low / end on the odometer."""
    pts = np.nonzero((need_true < mph - ALERT_MARGIN_MPH) & (dist >= -HOLD_AFTER_M))[0]
    if not len(pts):
      return []
    order = pts[np.argsort(dist[pts])]
    runs, cur = [], [order[0]]
    for j in order[1:]:
      if dist[j] - dist[cur[-1]] <= 2 * self.mem.spacing + 1e-6:
        cur.append(j)
      else:
        runs.append(cur)
        cur = [j]
    runs.append(cur)
    out = []
    for run in runs:
      run = np.array(run)
      lo = run[int(np.argmin(need_true[run]))]
      key = int(idx[run[-1]]) if idx[run[-1]] >= 0 else ("model", round(self.odo + float(dist[run[-1]]), -1))
      out.append({"key": key, "need": int(need_true[lo]), "entry": self.odo + float(dist[run[0]]),
                  "low": self.odo + float(dist[lo]), "end": self.odo + float(dist[run[-1]])})
    return out

  def _observe(self, t, mph, set_mph):
    """How much of its measured deceleration the cruise is ACTUALLY giving (a weaker cruise than measured, a downhill, a heavy load):
    observed speed drop over OBS_WINDOW_S against the drop ti_cruise_model predicts for the same recorded set speeds (the model run
    from the oldest kept sample, OBS_PRE_S before the window, settled there), while it should be slowing. The prediction (`_arrive`)
    uses min(RESP_FACTOR, that share); fourth review, item 3."""
    self.hist.append((t, mph, set_mph))
    while self.hist and t - self.hist[0][0] > OBS_WINDOW_S + OBS_PRE_S + 1e-6:
      self.hist.pop(0)
    if t - self.resp_t > OBS_KEEP_S:
      self.resp = RESP_FACTOR
    old = [h for h in self.hist if t - h[0] >= OBS_WINDOW_S - 1e-6]
    if not old or set_mph is None or any(h[2] is None for h in self.hist):
      return
    t0, m0, _ = old[-1]
    car = CM.Cruise(self.hist[0][1], self.hist[0][2], OBS_SIM_DT_S)
    v_t0 = car.v
    for (ta, _, sa), (tb, _, _) in zip(self.hist, self.hist[1:]):
      for _ in range(max(1, int(round((tb - ta) / OBS_SIM_DT_S)))):
        car.step(sa)
      if tb <= t0 + 1e-6:
        v_t0 = car.v
    exp = (v_t0 - car.v) / max(t - t0, 1e-3)
    if exp < OBS_MIN_EXPECT:
      return
    obs = (m0 - mph) / max(t - t0, 1e-3)
    self.resp = min(RESP_FACTOR, max(0.2, obs / exp))
    self.resp_t = t

  def _arrive(self, v, d, set0, s1, lowering, T=None):
    """The speed (mph) at `d` metres from what curve slowing will actually do: the set speed tapped down from `set0` one mph per TAP_S
    to `s1` when `lowering` (else it stays), the car answering as ti_cruise_model says (the same model the approach is planned with),
    its slowing scaled by the share of it the car is observed to give now if less (`_observe`; kept under plan amendment A1 because
    it is shown to help: without it the weak-cruise scenario test_f_behind_plan_warns_early warns 3.9 s ahead instead of 5). With T
    (above s1): the planner's own manoeuvre, a push to s1 and the model-timed raise back to T (plan A11)."""
    return CM.speed_at(v / MPH, d, set0, s1, lowering, dt=SIM_DT_S, scale=self.resp, T=T, hold=CM.HOLD)

  def _warn(self, corners, mph, v, floor, acting, set_mph, lowering, u_ref=None):
    """THE warning, one rule. For each corner the car is too fast for (`_corners` on trusted road; the LATCHED ones, by the odometer,
    while it is not trusted): predict the speed at the corner's tightest point from what curve slowing will actually do (`_arrive`;
    acting on trusted road: taps down to the push speed max(floor, need - DRIVE_GAP_MPH), never below the floor; otherwise the set
    speed stays where it is). If that is above need + ALERT_MARGIN_MPH and the corner's entry is within the calm-braking horizon
    (`_horizon`), warn with the TRUE need. A warning holds until the speed is down to need + ALERT_MARGIN_MPH or the corner is
    passed, and re-arms if the car speeds up again before it. Independent of whether an intervention was started, held, paused,
    stopped, floored, behind or decided against. -> the lowest need warned for (whole mph) or None."""
    seen = set()
    out = None
    if u_ref is None:
      u_ref = max(mph, float(set_mph)) if set_mph else mph
    for c in corners:
      key, need = c["key"], c["need"]
      seen.add(key)
      d_low = max(0.0, c["low"] - self.odo)
      d_entry = max(0.0, c["entry"] - self.odo)
      if key in self.alerted and key not in self.alerting and mph > need + ALERT_MARGIN_MPH + 3:
        self.alerted.discard(key)               # sped up again before the corner: may warn again
      if key not in self.alerting and (key in self.alerted or d_entry > self._horizon(v, need)):
        continue
      if not set_mph:
        arrive = mph                              # no cruise: nothing changes the speed
      elif acting and lowering and u_ref - max(need, int(floor)) <= LATE_SHED_MPH and self.fc is not None:
        # a bend stage 1 leaves to stage 2 (plan T6): nothing changes until the preview's window reaches it, then a straight walk
        # to at most LATE_SHED_MPH below (stage 2 does not push)
        # confirmed on the 2nd planner tick (5 Hz), then the car side reads the plan and sends the first tap (S2_TAP_LAG_S)
        win = max(0.0, min(CONFIRM_LEAD_S, float(np.max(self.fc[0]))) - CONFIRM_TICKS / 5.0 - S2_TAP_LAG_S)
        d1 = max(0.0, d_low - win * v)
        T2 = float(max(need, int(round(u_ref)) - LATE_SHED_MPH, int(floor)))
        arrive = self._arrive(v, d_low - d1, float(set_mph), min(float(set_mph), T2), True)
      elif acting and lowering and u_ref - max(need, int(floor)) > LATE_SHED_MPH:
        T = max(need, int(floor))
        arrive = self._arrive(v, d_low, float(set_mph), float(max(CM.push_floor(floor, DRIVE_GAP_MPH), T - DRIVE_GAP_MPH)), True, T=T)
      else:
        arrive = self._arrive(v, d_low, float(set_mph), float(set_mph), False)
      if key in self.alerting:
        # released only once the car IS at need + ALERT_MARGIN_MPH and is not going to be above it there (a car speeding up toward a
        # set speed above the corner's keeps its warning)
        if mph <= need + ALERT_MARGIN_MPH and arrive <= need + ALERT_MARGIN_MPH:
          self.alerting.discard(key)
          self.alerted.add(key)
          continue
      else:
        if arrive <= need + ALERT_MARGIN_MPH + RAISE_SLACK_MPH:
          continue
        self.events.append({"kind": "alert", "point": key if isinstance(key, int) else -1, "need": need, "floor": int(floor),
                            "acting": bool(acting), "trusted": bool(lowering), "mph": round(mph, 1), "arrive": round(arrive, 1),
                            "dist": round(d_entry, 1)})
        self.alerting.add(key)
      out = need if out is None else min(out, need)
    for key in [k for k in self.alerting if k not in seen]:
      self.alerting.discard(key)                  # passed (or no longer ahead)
      self.alerted.add(key)
    return out

  def _guard_apply(self, ceil, trusted, kind, cmph, dist, end, gkey, guntil=None):
    """Review C1 (b), 2026-11-24: the invariant on the PUBLISHED ceiling. A corner ceiling published (trusted, not a "hold") is
    recorded for its corner (`gkey`: the memory point, the map bend, or stage 2's run) as its cap: the lowest NON-PUSH value published
    for it (max(ceiling, corner speed): the push below the corner speed and the raise back to it stay allowed), kept to the corner's
    end + HOLD_AFTER_M. While any is alive, the published ceiling is never HIGHER than the lowest cap, whatever source proposes it
    (stage 2, the map, a new match, nothing at all). Dropped early only by the explicit release decision (_ramp), the driver's blinker
    (stage 2) or the driver choosing a new set speed (their reference changes). -> (ceil, trusted, kind, cmph, dist, end, bound)."""
    odo, ref = self.odo, getattr(self, "ref_now", None)
    for k in [k for k, g in self.guard.items() if odo > g["until"] or (ref is not None and g["ref"] is not None and ref != g["ref"])]:
      g = self.guard.pop(k)
      self.events.append({"kind": "guard_off", "why": "passed" if odo > g["until"] else "driver set speed", "cap": g["cap"]})
    if ceil is not None and trusted and kind != "hold" and gkey is not None:
      cap = int(max(int(ceil), int(cmph) if cmph is not None else int(ceil)))
      # the corner's own release point (its end + HOLD_AFTER_M, exact), else from the published end
      until = float(guntil) if guntil is not None else odo + max(0.0, float(end or 0.0)) + HOLD_AFTER_M
      g = self.guard.get(gkey)
      if g is None:
        self.guard[gkey] = {"cap": cap, "until": until, "entry": odo + max(0.0, float(dist or 0.0)), "ref": ref}
      else:
        g["cap"], g["until"] = min(g["cap"], cap), max(g["until"], until)
    if not self.guard:
      self._gb = False
      return ceil, trusted, kind, cmph, dist, end, False
    g = min(self.guard.values(), key=lambda g_: g_["cap"])
    if ceil is not None and trusted and int(ceil) <= g["cap"]:
      self._gb = False
      return ceil, trusted, kind, cmph, dist, end, False
    if not getattr(self, "_gb", False):
      self.events.append({"kind": "guard_bind", "cap": g["cap"], "proposed": None if ceil is None else int(ceil), "src_kind": kind,
                          "why": self.dr_why})
    self._gb = True
    return (g["cap"], True, "corner", g["cap"], round(max(0.0, g["entry"] - odo), 1),
            round(max(0.0, g["until"] - HOLD_AFTER_M - odo), 1), True)

  def _out(self, ceil, conf, trusted, known, why, dist, cmph, model_la, past=False, capped=False, point=-1, kind="", stage=0, src="",
           gkey=None, guntil=None):
    """The Plan of this tick; stage 2 (self.s2) is merged here (the lower ceiling wins), so it acts on every path, trusted or not."""
    end, self.end_m = self.end_m, None
    behind, self.behind = self.behind, False
    alert, self.alert = self.alert, None
    need, self.need_out = getattr(self, "need_out", None), None
    cap, self.cap_out = getattr(self, "cap_out", None), None
    s2 = getattr(self, "s2", None)
    if s2 is not None and (ceil is None or s2["T"] < ceil or (kind == "hold" and s2["T"] <= ceil)):
      # stage 2 binds: its source (the forecast) is valid whatever the memory's trust
      ceil, cmph, kind, stage, src, trusted, past = s2["T"], s2["T"], "corner", 2, "fc", True, False
      dist = round(max(0.0, s2["lead"] * self.fc_v), 1) if getattr(self, "fc_v", None) is not None else 0.0
      end = round(max(0.0, s2["until"] - HOLD_AFTER_M - self.odo), 1)
      need = s2["need"]
      cap = round(D.capacity(s2["T"], s2.get("side", D.UNKNOWN)), 2)   # R9: the raw capacity of the forecast's side at the target
      capped = capped or s2["need"] < s2["T"]          # the bend wants more than stage 2 may shed
      why = "bend ahead (forecast)"
      gkey, guntil = s2.get("key"), s2["until"]
    ceil, trusted, kind, cmph, dist, end, bound = self._guard_apply(ceil, trusted, kind, cmph, dist, end, gkey, guntil)
    if bound:
      # review C1 (b): a corner in progress holds the published ceiling (no source may raise it inside the bend)
      stage, src, past = max(int(stage or 0), 1), (src or "guard"), False
      why = "in the corner: holding" + (f" ({self.dr_why})" if self.dr_why and not self.locked else "")
    if s2 is not None and s2["need"] < s2["T"] and not s2["knew"]:
      # a bend needing more than stage 2 may shed (LATE_SHED_MPH, or the floor): the 5 mph plus the warning
      alert = s2["need"] if alert is None else min(alert, s2["need"])
    if ceil is None:
      stage, src = 0, ""
    ahead = getattr(self, "ahead_la", None)
    diag = {"stage": stage, "ceil": ceil, "kind": kind, "need": need if isinstance(need, (int, np.integer)) else None,
            "cap": cap, "ahead": ahead, "behind": behind, "alert": alert, "src": src,
            "rel": len(getattr(self, "released", {}) or {})}
    return Plan(ceil, round(conf, 2), trusted, known, why, None if dist is None else round(dist, 1), cmph, past, model_la, capped,
                self.limit, self.model_k, kind, end, behind, alert, need if isinstance(need, (int, np.integer)) else None, self.trace,
                point, stage, src, ahead, cap, bool(getattr(self, "released", None)), diag, getattr(self, "ask", None))


def plan_json(plan, t_mono, seq, mode):
  """The output the car controller reads (PARAM_PLAN): small, whole numbers where possible, no position."""
  return {"m": round(t_mono, 2), "seq": seq, "mode": mode, "ceil": plan.ceil, "conf": plan.conf, "trusted": plan.trusted,
          "known": plan.known, "why": plan.why, "dist": plan.dist_m, "past": plan.past,
          "model_la": None if plan.model_la is None else round(plan.model_la, 2), "capped": plan.capped,
          "limit": plan.limit_mph, "model_k": None if plan.model_k is None else round(plan.model_k, 5), "kind": plan.kind,
          "corner": plan.corner_mph, "end": plan.end_m, "behind": plan.behind, "alert": plan.alert_mph,
          "need": None if plan.need_mph is None else int(plan.need_mph), "stage": plan.stage, "src": plan.src,
          "ahead": plan.ahead_la, "cap": plan.cap, "rel": plan.released, "ask": None if plan.ask is None else list(plan.ask)}
