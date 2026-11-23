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
3. A point is "due" at the LATEST distance that still sheds the speed by ENTRY_M before it with the measured response (SHED_TIME,
   REACTION_S) plus BUFFER_S; only the speed the point needs is shed (no extra beyond MARGIN). The ceiling is the lowest due speed; it holds through the
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
# Measured from the first set-speed press (cruise-slowing-20261122 and the corner design's response table): reaction ~0.5 s;
# 5 mph shed in ~4.4 s (planning allowance 5.2 s), 10 mph 7.5-9 s (8.5), 15 mph 12-14 s (13.5); peak ~1.35-1.5 mph/s for 5 mph, cap
# ~2 mph/s. SHED_TIME is the planning allowance (the slower end of the measurements: no more deceleration than measured is assumed);
# beyond 15 mph it continues at 1 mph/s. The set speed goes straight to the corner speed when due: the car's own response to that step
# is what the allowance measures (its deceleration grows with the gap only up to ~6 mph, so a smaller step would be slower, not gentler).
REACTION_S = 0.5
SHED_TIME = ((0.0, 0.5), (5.0, 5.2), (10.0, 8.5), (15.0, 13.5))   # (mph shed, seconds from the first press)
BUFFER_S = 1.0                  # the small stated buffer on top (to be chosen from the replay's "late" and "early" figures)
ENTRY_M = 10.0                  # be at the corner speed this far before the first point that needs it
MIN_DROP_MPH = 2                # a point needing less than this below the speed is ignored (the car can hold it at its speed)
HOLD_AFTER_M = 30.0             # keep the ceiling this far past the last tight point

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


def shed_time_s(drop_mph):
  """Seconds from the first press until `drop_mph` has been shed (the measured planning allowance; 1 mph/s beyond 15 mph)."""
  d = max(0.0, float(drop_mph))
  pts = SHED_TIME
  if d >= pts[-1][0]:
    return pts[-1][1] + (d - pts[-1][0]) / 1.0
  for (d0, t0), (d1, t1) in zip(pts, pts[1:]):
    if d <= d1:
      return t0 + (t1 - t0) * (d - d0) / (d1 - d0)
  return pts[-1][1]


def slow_distance_m(from_mph, to_mph):
  """Metres driven from the first press until the speed is down from `from_mph` to `to_mph` (speed falls roughly linearly after the
  reaction), plus BUFFER_S at the start speed and ENTRY_M: the latest distance at which slowing may start."""
  drop = max(0.0, from_mph - to_mph)
  if drop <= 0.0:
    return ENTRY_M
  t = shed_time_s(drop)
  avg = 0.5 * (from_mph + to_mph)
  return from_mph * MPH * (REACTION_S + BUFFER_S) + avg * MPH * max(0.0, t - REACTION_S) + ENTRY_M


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
    self.committed = {}                 # point -> (speed it allows, odometer where it is): corners slowing has started for
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
    self.committed = {}

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
    if model_k is not None:
      ts, ks = model_k
      ts, ks = np.asarray(ts, np.float64), np.asarray(ks, np.float64)
      sel = ts <= MODEL_LOOK_S
      if sel.any() and np.all(np.isfinite(ks[sel])):
        kk = np.abs(ks[sel])
        j = int(np.argmax(kk))
        model_la = float(kk[j] * max(v, 1.0) ** 2)
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
    # what the car side will actually shed: never below its floor (the known limit, else the speed now, minus MAX_SHED_MPH), so the
    # timing is for the drop that will really be asked for (a corner needing more is flagged, not started earlier)
    floor = max(self.ABS_FLOOR_MPH, (self.limit if self.limit else int(round(mph))) - self.MAX_SHED_MPH)
    shed_to = np.maximum(need, floor)
    tight = (u - shed_to) >= MIN_DROP_MPH
    if tight.any():
      ti = np.nonzero(tight)[0]
      dd = np.array([slow_distance_m(u, n) for n in shed_to[ti]])
      for j in ti[dist[ti] <= dd]:
        # due: committed until HOLD_AFTER_M past it (a corner once started is held, never dropped because the car has slowed)
        key = int(idx[j])
        at = self.odo + float(dist[j])
        old = self.committed.get(key)
        if old is None or need[j] < old[0]:
          self.committed[key] = (float(need[j]), at if old is None else old[1])
    for key in [k for k, (_, at) in self.committed.items() if self.odo > at + HOLD_AFTER_M]:
      del self.committed[key]
    ceil, cdist, cpt = None, None, -1
    if self.committed:
      key, (n_, at) = min(self.committed.items(), key=lambda kv: (kv[1][0], kv[1][1]))
      ceil, cdist, cpt = int(n_), round(at - self.odo, 1), key
    if ceil is None:
      return self._out(None, conf, True, True, "no known corner ahead", None, None, model_la, past=True)
    capped = ceil < floor
    why = "model preview: sharper bend" if cpt == -1 else ("corner ahead" if cdist > 0 else "in the corner")
    return self._out(ceil, conf, True, True, why, cdist, ceil, model_la, capped=capped, point=cpt)

  def _out(self, ceil, conf, trusted, known, why, dist, cmph, model_la, past=False, capped=False, point=-1):
    return Plan(ceil, round(conf, 2), trusted, known, why, None if dist is None else round(dist, 1), cmph, past, model_la, capped,
                self.limit, self.trace, point)


def plan_json(plan, t_mono, seq, mode):
  """The output the car controller reads (PARAM_PLAN): small, whole numbers where possible, no position."""
  return {"m": round(t_mono, 2), "seq": seq, "mode": mode, "ceil": plan.ceil, "conf": plan.conf, "trusted": plan.trusted,
          "known": plan.known, "why": plan.why, "dist": plan.dist_m, "past": plan.past,
          "model_la": None if plan.model_la is None else round(plan.model_la, 2), "capped": plan.capped,
          "limit": plan.limit_mph}
