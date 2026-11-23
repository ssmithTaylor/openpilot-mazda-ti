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
   LOOKAHEAD_MAX_M. For every remembered point the speed the steering can hold (`corner_mph`): the highest whole mph at which the
   remembered curvature needs at most MARGIN of the measured steering envelope (ENVELOPE, conservative end of the two measurements),
   checked over +-0.75 mph; where a remembered pass reached the steering limit or had a firm takeover, at least FLAGGED_DROP_MPH below
   the speed that pass was driven at. The driving model's own preview (when it shows a sharper bend than memory in the next few
   seconds) gives a second figure; the lower wins.
3. Slowing for a corner starts at the LATEST moment that still arrives at its speed ENTRY_M before it, given the measured response
   (see the constants below); closed loop every tick against the planned approach (push / let off / behind plan); held through the
   corner and released HOLD_AFTER_M past it. Between corners the output is a "hold" ceiling: the highest set speed a restore may reach
   from which the NEXT corner can still be made, so chained corners keep only what the next one needs. The ceiling is the lowest due speed; it holds through the
   corner and is released HOLD_AFTER_M after the last tight point has been passed.

Output (`Plan`): ceiling in whole mph (None: no ceiling), confidence, trusted, known road, reason, distance to the limiting point, and the
model's own preview (largest predicted lateral acceleration in the next MODEL_LOOK_S), which the car side needs before it restores.
No position ever leaves this module: logs carry trace / point indices, distances and curvatures only.
"""
import math
from typing import NamedTuple, Optional

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_memory as M

MPH = 0.44704

# ---- what the steering can hold (lateral-sensors-20261122 and the codex corner design; the LOWER of the two per band) -------------
# (lowest mph of the band, m/s^2 achieved at the steering limit). Measured: 1.41 / 2.20 / 2.42 / 2.13 at 19-27 / 28-41 / 42-56 / 57+;
# design study episode medians: 1.14 / 1.14 (28-31, the stock-steering handover) / 2.12 (32-41) / 2.42 / 2.24.
ENVELOPE = ((19.0, 1.14), (28.0, 1.14), (32.0, 2.12), (42.0, 2.42), (57.0, 2.13))
MARGIN = 0.85                   # use at most 85 % of the envelope (the design proposed 88 %; the conservative start)
SPEED_WINDOW_MPH = 0.75         # a speed fits only if every speed within this of it fits (band edges, speed error)
FLAGGED_DROP_MPH = 10           # where a pass hit the limit or had a takeover: at least this far below the speed it was driven at
CAR_MIN_MPH = 19                # the car's own lowest set speed
MAX_MPH = 90

# ---- when to start: as LATE as still reaches the corner speed (owner, 2026-11-23: "we should ensure that we're not slowing down
# further ahead than we need to ... We only want to do what we need to, speed change wise, and speed of speed change wise.")
# The stock cruise's measured response to a lower set speed (cruise-slowing-20261122): reaction ~0.5 s, then a deceleration that
# depends on the gap between speed and set speed: ~0.34 m/s^2 (0.76 mph/s) for a gap up to 3 mph, rising to ~0.9-1.0 m/s^2 (cap ~2 mph/s)
# at 6 mph and more. So the approach is: PUSH (set DRIVE_GAP_MPH below the corner speed, never below the floor) so the car slows at the
# rate it can, then set the corner speed itself once within RAISE_AT_MPH of it (no undershoot). The start is the latest moment that
# still arrives by ENTRY_M before the corner with RESP_FACTOR of the measured deceleration and BUFFER_S of margin. CLOSED LOOP every
# tick: the car's speed is compared with the highest speed from which it can still make it (`v_allow`); a car slowing less than planned
# (downhill or anything else) is pushed to the floor; one slowing more (uphill) is let off to the corner speed (with hysteresis).
REACTION_S = 0.5
TAP_S = 0.3                     # one set-speed tap per this (2 pressed frames + the reviewed 0.25 s gap)
DRIVE_GAP_MPH = 6                # a gap of 6 mph or more gives the most deceleration the cruise has; more does nothing
RAISE_AT_MPH = 1.5
RESP_FACTOR = 0.9               # plan with 90 % of the measured deceleration
BUFFER_S = 1.0                  # extra seconds of start-early margin (chosen from the replay's buffer / early / late curve)
ENTRY_M = 10.0                  # be at the corner speed this far before the first point that needs it
COAST_BELOW_MPH = 3.0           # pushing and this far below v_allow: let off to the corner speed
REPUSH_MPH = 1.0                # ... and push again when back within this of v_allow
BEHIND_MPH = 1.0                # above v_allow by this: behind plan (the gap is already at its most useful: nothing more to ask of the
                                # set speed; the corner is flagged "needs more than I can slow" and logged)
MIN_SWITCH_S = 3.0              # no push / let-off reversal within this unless behind plan
GRADE_STEEP_DEG = 3.0           # remembered approach steeper downhill than this: start earlier
GRADE_BUFFER_S_PER_DEG = 0.5    # ... by this per degree beyond GRADE_STEEP_DEG (no data beyond +-3 degrees: a guess, logged)
MIN_DROP_MPH = 2                # a point needing less than this below the speed is ignored (the car can hold it at its speed)
HOLD_AFTER_M = 30.0             # keep the ceiling this far past the last tight point


SAT_MPH_S = 1.6                 # the saturated rate at a 6+ mph gap: ~0.7 m/s^2 mean (peaks 0.9-1.0)


def decel_mph_s(gap):
  """The stock cruise's measured deceleration (mph/s) for a gap (mph) between speed and set speed."""
  if gap <= 0.0:
    return 0.0
  if gap < 1.0:
    return 0.76 * gap
  if gap <= 3.0:
    return 0.76
  if gap >= 6.0:
    return SAT_MPH_S
  return 0.76 + (SAT_MPH_S - 0.76) * (gap - 3.0) / 3.0


_APPROACH = {}


def approach_m(u, T, S1, dt=0.05):
  """Metres from the first press until the speed is within 0.5 mph of T, starting at u mph with the set speed at u, tapping down to S1
  (one tap per TAP_S), raising to T once within RAISE_AT_MPH of it; the cruise acts REACTION_S after the set speed changes, with
  RESP_FACTOR of the measured deceleration. Cached by whole mph."""
  key = (int(u), int(T), int(S1))
  r = _APPROACH.get(key)
  if r is not None:
    return r
  v = float(key[0])
  T, S1 = key[1], key[2]
  hist = []                       # set speed by step, for the reaction delay
  setn = v
  t = dist = 0.0
  tap_t = 0.0
  raised = False
  lag = int(round(REACTION_S / dt))
  while v > T + 0.5 and t < 90.0:
    if not raised and v <= T + RAISE_AT_MPH:
      raised = True
    goal = T if raised else S1
    if t >= tap_t and setn != goal:
      setn += 1 if goal > setn else -1
      tap_t = t + TAP_S
    hist.append(setn)
    eff = hist[-1 - lag] if len(hist) > lag else float(key[0])
    v = max(0.0, v - RESP_FACTOR * decel_mph_s(v - eff) * dt)
    dist += v * MPH * dt
    t += dt
  _APPROACH[key] = dist
  return dist


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
  S1 = max(floor_mph, to_mph - DRIVE_GAP_MPH)
  return approach_m(from_mph, to_mph, S1) + BUFFER_S * from_mph * MPH + ENTRY_M


# ---- look-ahead (long-horizon study: warning beyond 25-30 s stops helping) -----------------------------------------------------
LOOKAHEAD_S = 45.0
LOOKAHEAD_MIN_M = 300.0
LOOKAHEAD_MAX_M = 2000.0

# ---- localisation ----------------------------------------------------------------------------------------------------------------
MATCH_RADIUS_M = 25.0
MATCH_HEADING_DEG = 30.0
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
CHANGE_RATIO = 0.7              # rms(live - remembered) above this share of the larger rms: mismatch
CHANGE_HOLDOFF_M = 500.0
LIVE_K_TAU_S = 0.5              # smoothing of the live curvature (the memory is smoothed over ~10 m)

# ---- the model's own preview -----------------------------------------------------------------------------------------------------
MODEL_LOOK_S = 8.0

_RUNGS = np.arange(MAX_MPH, CAR_MIN_MPH - 1, -1, dtype=np.float64)   # highest first


def envelope(mph):
  a = ENVELOPE[0][1]
  for lo, acc in ENVELOPE:
    if mph >= lo:
      a = acc
  return a


def _limit_table():
  """For every rung: the lowest envelope within +-SPEED_WINDOW_MPH, times MARGIN, over (rung + window) squared in m/s:
  the largest curvature that fits at that rung."""
  out = []
  for r in _RUNGS:
    lo, hi = max(CAR_MIN_MPH, r - SPEED_WINDOW_MPH), r + SPEED_WINDOW_MPH
    env = min(envelope(s) for s in (lo, r, hi))
    out.append(MARGIN * env / ((hi * MPH) ** 2))
  return np.array(out)


_KMAX = _limit_table()


def fits(k, mph):
  """True when |k| fits at `mph` (same rule as corner_mph)."""
  lo, hi = max(CAR_MIN_MPH, mph - SPEED_WINDOW_MPH), mph + SPEED_WINDOW_MPH
  env = min(envelope(s) for s in (lo, mph, hi))
  return abs(k) * (hi * MPH) ** 2 <= MARGIN * env + 1e-12


def corner_mph(k):
  """Highest whole mph (CAR_MIN_MPH .. MAX_MPH) at which |k| fits; CAR_MIN_MPH when none does. Vectorised over k."""
  k = np.abs(np.atleast_1d(np.asarray(k, np.float64)))
  ok = k[:, None] <= _KMAX[None, :] + 1e-12
  first = np.where(ok.any(axis=1), ok.argmax(axis=1), len(_RUNGS) - 1)
  return _RUNGS[first]


def needed_mph(kap, flags, v_flag):
  """The speed each remembered point allows: corner_mph, and at least FLAGGED_DROP_MPH below a flagged pass's speed."""
  need = corner_mph(kap)
  flagged = (np.asarray(flags) & (M.FLAG_LIMIT | M.FLAG_TAKEOVER)) != 0
  vf = np.asarray(v_flag, np.float64)
  cap = np.where(flagged & np.isfinite(vf), np.floor(vf / MPH) - FLAGGED_DROP_MPH, np.inf)
  return np.minimum(need, np.maximum(cap, CAR_MIN_MPH))


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
  trace: int = -1
  point: int = -1


def bearing_to_hdg(bearing_deg):
  """Compass bearing (degrees, north = 0, clockwise) to the memory's heading (radians, east = 0, counter-clockwise)."""
  return M.wrap(math.radians(90.0 - bearing_deg))


class Planner:
  MAX_SHED_MPH = 15               # mirrors ti_curve.MAX_SHED_MPH, only to flag in the log when a corner needs more
  ABS_FLOOR_MPH = 25              # mirrors ti_curve.ABS_FLOOR_MPH

  def __init__(self, memory):
    self.mem = memory
    self.need = needed_mph(memory.kap, memory.flags, memory.v_flag) if memory is not None else None
    self.limit = None
    self.reset()

  def reset(self):
    self.t = None
    self.trace, self.pos = -1, 0.0      # pos: fractional global point index on the trace
    self.locked = False
    self.stable_s = 0.0
    self.cand = None                    # (trace, along metres, odometer) of the tentative match
    self.miss_s = 0.0
    self.fix_age = 1e9
    self.odo = 0.0
    self.live_k = 0.0
    self.cmp = []                       # (odometer, live k, remembered k), every spacing of travel while locked
    self.cmp_next = 0.0
    self.distrust_until = -1.0
    self.distrust_why = ""
    self.active = {}                    # point -> corner in progress: T (corner speed), at (odometer of the point), mode, since
    self.int_floor = None
    self.int_base = None
    self.end_m = None
    self.model_k = None
    self.events = []

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

  def _lose(self, why):
    if self.locked or self.cand is not None:
      self.events.append({"kind": "lock_lost", "why": why, "trace": self.trace})
    self.locked, self.cand, self.stable_s, self.miss_s = False, None, 0.0, 0.0
    self.trace = -1
    self.cmp = []
    self.active = {}

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
      if self.cand is not None and not self.locked:
        self.cand, self.stable_s = None, 0.0
      if self.locked and self.fix_age >= FIX_LOST_S:
        self._lose("no position fix")

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
      self.cmp.append((self.odo, self.live_k, float(self.mem.kap[i])))
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
      return False
    err = math.sqrt(float(np.mean((live - rem) ** 2)))
    return err > CHANGE_RATIO * scale

  # ---- one tick ------------------------------------------------------------------------------------------------------------------
  def step(self, t, fix, v, set_mph=None, yaw_k=None, model_k=None, limit_mph=None):
    """t: seconds (monotonic); fix: Fix; v: m/s; set_mph: the cruise set speed or None; yaw_k: live curvature (1/m, left +, from the
    car's yaw sensor) or None; model_k: (seconds ahead array, curvature array) of the driving model's preview, or None. -> Plan."""
    dt = 0.0 if self.t is None else max(0.0, min(1.0, t - self.t))
    self.t = t
    self.limit = int(round(limit_mph)) if limit_mph and limit_mph > 0 else None
    v = max(0.0, float(v))
    self.odo += v * dt
    if yaw_k is not None and math.isfinite(yaw_k):
      a = 1.0 if dt <= 0 else min(1.0, dt / LIVE_K_TAU_S)
      self.live_k += a * (yaw_k - self.live_k)
    model_la, model_need, model_dist = None, None, None
    self.model_k = None
    if model_k is not None:
      ts, ks = model_k
      ts, ks = np.asarray(ts, np.float64), np.asarray(ks, np.float64)
      sel = ts <= MODEL_LOOK_S
      if sel.any() and np.all(np.isfinite(ks[sel])):
        kk = np.abs(ks[sel])
        j = int(np.argmax(kk))
        model_la = float(kk[j] * max(v, 1.0) ** 2)
        self.model_k = float(kk[j])
        model_need, model_dist = float(corner_mph(kk[j])[0]), float(ts[sel][j] * v)
    if self.mem is None:
      return self._out(None, 0.0, False, False, "no road memory on the device", None, None, model_la)
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
    if self.odo < self.distrust_until:
      return self._out(None, 0.0, False, known, self.distrust_why, None, None, model_la)
    if not self.locked:
      why = "matching the remembered road" if self.cand is not None else "road not known here"
      return self._out(None, conf, False, known, why, None, None, model_la)
    if conf < MIN_TRUST_CONF:
      return self._out(None, conf, False, True, "position uncertain", None, None, model_la)

    # ---- the corners ahead -------------------------------------------------------------------------------------------------------
    mph = v / MPH
    u = max(mph, float(set_mph)) if set_mph else mph
    look = min(LOOKAHEAD_MAX_M, max(LOOKAHEAD_MIN_M, v * LOOKAHEAD_S))
    i0 = int(math.floor(self.pos))
    idx, dist = self.mem.ahead(self.trace, i0, look)
    dist = dist - (self.pos - i0) * self.mem.spacing
    need = self.need[idx]
    if model_need is not None:
      # the model's own preview, where it shows a sharper bend than memory around the same distance
      near = np.abs(dist - model_dist) <= 30.0
      if near.any() and model_need < float(need[near].min()):
        idx = np.r_[idx, -1]
        dist = np.r_[dist, model_dist]
        need = np.r_[need, model_need]
    # the floor the car side keeps for an intervention (the known limit, else the speed when it starts, minus MAX_SHED_MPH)
    # (kept from the first corner of a chain until the car is back near the speed it started from, as the car side does)
    if self.active or (self.int_base is not None and mph < self.int_base - 2.0):
      floor = self.int_floor
    else:
      floor = max(self.ABS_FLOOR_MPH, int(round(mph)) - self.MAX_SHED_MPH, (self.limit - self.MAX_SHED_MPH) if self.limit else 0)
      self.int_floor, self.int_base = floor, None
    shed_to = np.maximum(need, floor)
    grade = getattr(self.mem, "grade", None)
    # tight against the speed now, the set speed, and (within a chain) the speed the chain started from, so a corner just after one
    # already slowed for still bounds the way back up (a "hold")
    u_ref = max(u, self.int_base or 0.0)
    tight = np.nonzero((u_ref - shed_to) >= MIN_DROP_MPH)[0]
    # corners: runs of consecutive tight points; per run, its first point and its lowest-speed point
    cands = {}
    if len(tight):
      breaks = np.nonzero(np.diff(tight) > 1)[0]
      starts = np.r_[0, breaks + 1]
      ends = np.r_[breaks, len(tight) - 1]
      for s0, s1 in zip(starts, ends):
        run = tight[s0:s1 + 1]
        lowest = run[int(np.argmin(need[run]))]
        # a corner is known by the END of its tight stretch: it stays the same while the car drives into the stretch (its first and
        # lowest points ahead change as they are passed), so a corner is started once
        key = int(idx[run[-1]])
        for j in {int(run[0]), int(lowest)}:
          T = int(shed_to[j])
          D = float(dist[j]) - ENTRY_M
          buf = BUFFER_S
          if grade is not None and dist[j] > 0:
            sel = idx[(dist >= 0) & (dist <= dist[j]) & (idx >= 0)]
            g = float(np.degrees(np.nanmean(grade[sel]))) if len(sel) else 0.0
            if math.isfinite(g) and g < -GRADE_STEEP_DEG:
              buf += GRADE_BUFFER_S_PER_DEG * (-g - GRADE_STEEP_DEG)
          va = v_allow(D, T, max(floor, T - DRIVE_GAP_MPH), buf)
          c = cands.get(key)
          if c is None or va < c["va"]:
                cands[key] = {"T": T, "need": int(need[j]), "va": va, "D": D, "at": self.odo + float(dist[run[-1]]),
                              "entry": self.odo + float(dist[run[0]])}
    # start: the car is at or above the highest speed from which it can still make it
    for key, c in cands.items():
      if key not in self.active and mph >= c["va"] and mph - c["T"] >= MIN_DROP_MPH:
        if not self.active and self.int_base is None:
          self.int_base = mph
        self.active[key] = {"T": c["T"], "need": c["need"], "at": c["at"], "entry": c["entry"], "mode": "push", "since": t}
        self.events.append({"kind": "start", "point": key, "T": c["T"], "need": c["need"], "floor": floor, "mph": round(mph, 1),
                            "planned": round(c["va"], 1), "dist": round(c["D"], 1)})
    for key in [k for k, a_ in self.active.items() if self.odo > a_["at"] + HOLD_AFTER_M]:
      self.events.append({"kind": "passed", "point": key, "T": self.active[key]["T"], "mph": round(mph, 1)})
      del self.active[key]
    ceil, cdist, cpt, kind, corner = None, None, -1, "", None
    cend = None
    behind = False
    for key, a_ in self.active.items():
      c = cands.get(key)
      va = c["va"] if c is not None else float(a_["T"])
      if c is not None:
        a_["T"] = min(a_["T"], c["T"])
      T = a_["T"]
      S1 = max(floor, T - DRIVE_GAP_MPH)
      mode = a_["mode"]
      if mode != "target":
        if mph <= T + RAISE_AT_MPH:
          mode = "target"
        elif mph > va + BEHIND_MPH:
          mode = "behind"
        elif mode == "push" and mph < va - COAST_BELOW_MPH and t - a_["since"] >= MIN_SWITCH_S:
          mode = "coast"
        elif mode == "behind" and mph <= va:
          mode = "push"
        elif mode == "coast" and mph >= va - REPUSH_MPH:
          mode = "push"
      if mode != a_["mode"]:
        self.events.append({"kind": "mode", "point": key, "mode": mode, "mph": round(mph, 1), "planned": round(va, 1), "T": T})
        a_["mode"], a_["since"] = mode, t
      cc = S1 if mode in ("push", "behind") else T
      if mode == "behind":
        behind = True
      if ceil is None or cc < ceil:
        ceil, cdist, cpt, kind, corner = cc, round(max(0.0, a_["entry"] - self.odo), 1), key, "corner", T
        cend = round(a_["at"] - self.odo, 1)
    if not self.active:
      # no corner in progress: the highest set speed a restore may reach so the NEXT corner can still be made (chained corners hold
      # only what the next one needs; a corner far enough away lets the speed come back fully)
      hold = None
      for key, c in cands.items():
        if hold is None or c["va"] < hold[0]:
          hold = (c["va"], key, c)
      if hold is not None and int(math.floor(hold[0])) < MAX_MPH:
        ceil, cdist, cpt, kind, corner = int(math.floor(hold[0])), round(hold[2]["D"], 1), hold[1], "hold", hold[2]["T"]
    if ceil is None:
      return self._out(None, conf, True, True, "no known corner ahead", None, None, model_la, past=True)
    capped = (bool(self.active) and min(a_["need"] for a_ in self.active.values()) < floor) or behind
    why = ("next corner: holding what it needs" if kind == "hold" else "corner ahead" if (cdist or 0) > 0 else "in the corner")
    self.end_m = cend
    return self._out(int(ceil), conf, True, True, why, cdist, corner, model_la, capped=capped, point=cpt, kind=kind,
                     past=kind == "hold")

  def _out(self, ceil, conf, trusted, known, why, dist, cmph, model_la, past=False, capped=False, point=-1, kind=""):
    end, self.end_m = self.end_m, None
    return Plan(ceil, round(conf, 2), trusted, known, why, None if dist is None else round(dist, 1), cmph, past, model_la, capped,
                self.limit, self.model_k, kind, end, self.trace, point)


def plan_json(plan, t_mono, seq, mode):
  """The output the car controller reads (PARAM_PLAN): small, whole numbers where possible, no position."""
  return {"m": round(t_mono, 2), "seq": seq, "mode": mode, "ceil": plan.ceil, "conf": plan.conf, "trusted": plan.trusted,
          "known": plan.known, "why": plan.why, "dist": plan.dist_m, "past": plan.past,
          "model_la": None if plan.model_la is None else round(plan.model_la, 2), "capped": plan.capped,
          "limit": plan.limit_mph, "model_k": None if plan.model_k is None else round(plan.model_k, 5), "kind": plan.kind,
          "corner": plan.corner_mph, "end": plan.end_m}
