"""What the driving screen shows for curve slowing (the owner's approved design, 2026-11-26: mock-up round 5): the road ahead for the
next 45 s as a ribbon with the bends coloured by what the feature will do, and a chip beside the MAX box only on approach. This module
builds the screen's input from the planner's own look-ahead (ti_curve_plan.Planner.view: the tick's point distances and needs) and
decides the one ding; the UI (selfdrive/ui/qt/onroad/ti_curve_view_logic.h) fits, smooths and draws it.

Published to the tmpfs param PARAM_VIEW (compact JSON, written by ti_curve_plannerd at 5 Hz):
  {"v": 1, "m": tick time (monotonic s), "seq": tick, "on": curve slowing switched on (0 / 1), "mph": speed x10, "fl": floor mph,
   "r": [[metres along the path, signed curvature x1e5 (left +)], ...] (<= VIEW_PTS points, the next VIEW_LOOK_S s; [] where no trusted
   road memory: no ribbon), "re": 1 when the remembered road ends inside the window (the ribbon fades out),
   "b": [[start, tightest, end, dir, need, target, cls, approach, resume, lead x10], ...]}
Bends (<= MAX_BENDS, nearest first): start / tightest / end / approach / resume in whole metres along the path from the car; dir +1 left,
-1 right, 0 unknown (a map bend: the map's curvature is unsigned); need = the bend's needed speed, target = what the car slows to
(max(need, floor)); cls 1 yellow (the car will slow here), 2 red (need below the floor: the feature cannot reach it); approach = where
the approach tint starts; resume = where the car speeds back up (end + HOLD_AFTER_M); lead = the chip's lead time (s x10).
Never a position: distances along the path, speeds and curvatures only (test_mazda_ti_curve_view.py scans the schema).

The ding (PARAM_DING, written only at the onset): ONE per red bend, when its chip first appears (seconds to the bend <= its lead) while
curve slowing is on and the car is at or above MIN_MPH; never again for the same bend (known by the odometer of its end, kept until
passed). frogpilot_planner reads it (read_curve_ding) and raises the size-none event tiCurveDing once: a sound, no banner, no loop.
"""
# RUNTIME: runs in ti_curve_plannerd at 5 Hz (gc frozen after load, collector on); read_curve_ding in frogpilot_process at 10 Hz (gc
# disabled). Per tick: bounded lists only (MAX_BENDS, VIEW_PTS, DING_KEEP); never a position in a log or param.
import json
import math

import numpy as np

MPH = 0.44704
PARAM_VIEW = "TiCurveView"        # tmpfs: the screen element's input (this module), 5 Hz
PARAM_DING = "TiCurveDing"        # tmpfs: {"m": onset time (monotonic s), "n": onsets so far, "need": mph}, written at a red chip's onset
VIEW_VERSION = 1
VIEW_LOOK_S = 45.0                # the ribbon: the next 45 s of road at the speed now
VIEW_PTS = 100                    # ... decimated to at most this many points
MAX_BENDS = 8
MIN_DROP_MPH = 2                  # = ti_curve_plan.MIN_DROP_MPH: a point needing less than this below the speed is no bend
HOLD_AFTER_M = 30.0               # = ti_curve_plan.HOLD_AFTER_M: the planner holds the ceiling this far past a bend's last tight point
YELLOW_LEAD_S = 10.0              # the yellow chip: from this long before the bend
RED_BASE_S = 6.0                  # the red chip: from the driver's own braking time to the need at CALM_MS2, plus this ...
RED_MIN_S = 13.0                  # ... never less than this before the bend
CALM_MS2 = 1.5
YELLOW_DECEL_MS2 = 1.0            # the yellow approach tint: where the cruise starts slowing (mock-up round 2)
MIN_MPH = 19.0                    # cruise and the comma engage from about 19 mph: nothing shown or dinged below
DING_MATCH_M = 60.0               # a red bend is the same bend while its end stays within this (odometer)
DING_FORGET_M = 300.0             # ... and remembered until the car is this far past its end
DING_KEEP = 16
DING_FRESH_S = 1.0                # frogpilot_planner acts on an onset only this fresh
CLS_NEUTRAL, CLS_YELLOW, CLS_RED = 0, 1, 2
VIEW_KEYS = ("v", "m", "seq", "on", "mph", "fl", "r", "re", "b")
BEND_FIELDS = ("start", "low", "end", "dir", "need", "target", "cls", "approach", "resume", "lead")


def red_lead_s(mph, need):
  """The red chip's lead (s): the driver's own braking time from the speed now to the need at CALM_MS2, plus RED_BASE_S; at least
  RED_MIN_S."""
  return max(RED_MIN_S, RED_BASE_S + max(0.0, (float(mph) - float(need)) * MPH) / CALM_MS2)


def bends(dist, need, ksign, mph, set_mph, floor):
  """The bends of the look-ahead (the mock-up's rule, render2.bends): runs of consecutive points with (u - max(need, floor)) >=
  MIN_DROP_MPH or need < floor, u = max(speed, set speed), within VIEW_LOOK_S s. -> list of dicts (BEND_FIELDS, floats for distances),
  nearest first, at most MAX_BENDS."""
  dist = np.asarray(dist, np.float64)
  need = np.asarray(need, np.float64)
  v = max(float(mph) * MPH, 1.0)
  sel = dist <= VIEW_LOOK_S * v
  d, n, sg = dist[sel], need[sel], np.asarray(ksign, np.float64)[sel]
  u = max(float(mph), float(set_mph or 0.0))
  fl = float(floor)
  t = np.nonzero(((u - np.maximum(n, fl)) >= MIN_DROP_MPH) | (n < fl))[0]
  out = []
  if not len(t):
    return out
  uu = u * MPH
  for run in np.split(t, np.nonzero(np.diff(t) > 1)[0] + 1):
    j = int(run[int(np.argmin(n[run]))])
    nd = int(n[j])
    tgt = int(max(nd, fl))
    red = nd < fl
    vt = tgt * MPH
    if red:
      app = 2.0 * uu + max(0.0, uu * uu - (nd * MPH) ** 2) / (2.0 * CALM_MS2)
    else:
      app = max(0.0, uu * uu - vt * vt) / (2.0 * YELLOW_DECEL_MS2)
    start = max(float(d[run[0]]), 0.0)
    end = float(d[run[-1]])
    out.append({"start": start, "low": max(float(d[j]), 0.0), "end": end, "dir": int(np.sign(sg[j])), "need": nd, "target": tgt,
                "cls": CLS_RED if red else CLS_YELLOW, "approach": max(start - app, 0.0), "resume": end + HOLD_AFTER_M,
                "lead": red_lead_s(mph, nd) if red else YELLOW_LEAD_S})
    if len(out) >= MAX_BENDS:
      break
  return out


def ribbon(dist, k, mph):
  """The ribbon: points with 0 <= distance <= VIEW_LOOK_S s, decimated to at most VIEW_PTS; each point's curvature is the mean over
  the stretch it closes (the heading change along the ribbon is kept). -> ([[metres, curvature x1e5], ...], ends early)."""
  dist = np.asarray(dist, np.float64)
  k = np.asarray(k, np.float64)
  look = VIEW_LOOK_S * max(float(mph) * MPH, 1.0)
  m = (dist >= 0.0) & (dist <= look)
  d, kk = dist[m], k[m]
  n = len(d)
  if n < 2:
    return [], False
  stride = max(1, int(math.ceil((n - 1) / (VIEW_PTS - 1))))
  ends = list(range(0, n, stride))
  if ends[-1] != n - 1:
    ends.append(n - 1)
  starts = [0] + [e + 1 for e in ends[:-1]]
  sums = np.add.reduceat(kk, starts)
  cnt = np.diff(np.r_[starts, n])
  km = sums / np.maximum(cnt, 1)
  pts = [[int(round(float(d[e]))), int(round(float(km[i]) * 1e5))] for i, e in enumerate(ends)]
  return pts, bool(float(d[-1]) < look - 5.0)


def plan_bend(plan, side, floor, mph):
  """Where no trusted road memory gives a ribbon (map only, the forecast, a corner dead-reckoned through a position gap): the plan's
  limiting corner as one bend, its direction from the stage in force (side: +1 left, -1 right, 0 unknown: a map bend), or None."""
  if plan is None or plan.ceil is None or plan.kind != "corner" or plan.dist_m is None:
    return None
  need = plan.need_mph if plan.need_mph is not None else plan.corner_mph
  if need is None:
    return None
  need = int(need)
  red = need < int(floor)
  start = max(float(plan.dist_m), 0.0)
  end = max(start, float(plan.end_m)) if plan.end_m is not None else start
  return {"start": start, "low": start, "end": end, "dir": int(side or 0), "need": need, "target": int(max(need, floor)),
          "cls": CLS_RED if red else CLS_YELLOW, "approach": start, "resume": end + HOLD_AFTER_M,
          "lead": red_lead_s(mph, need) if red else YELLOW_LEAD_S}


def chip(bl, mph):
  """The chip the screen shows at the tick (the UI's own rule, ti_curve_view_logic.h chipFor, at age 0): the nearest red bend within its
  lead, else the nearest yellow within YELLOW_LEAD_S. -> (cls, index in bl) or (0, -1)."""
  v = max(float(mph) * MPH, 1.0)
  for i, b in enumerate(bl):
    if b["cls"] == CLS_RED and b["start"] / v <= b["lead"]:
      return CLS_RED, i
  for i, b in enumerate(bl):
    if b["cls"] == CLS_YELLOW and b["start"] / v <= b["lead"]:
      return CLS_YELLOW, i
  return CLS_NEUTRAL, -1


def payload(t_mono, seq, on, mph, floor, rib, ends, bl):
  """The PARAM_VIEW dict (VIEW_KEYS only; ints except the tick time)."""
  return {"v": VIEW_VERSION, "m": round(float(t_mono), 2), "seq": int(seq), "on": int(bool(on)), "mph": int(round(float(mph) * 10)),
          "fl": int(floor), "r": rib, "re": int(bool(ends)),
          "b": [[int(round(b["start"])), int(round(b["low"])), int(round(b["end"])), int(b["dir"]), int(b["need"]), int(b["target"]),
                 int(b["cls"]), int(round(b["approach"])), int(round(b["resume"])), int(round(b["lead"] * 10))] for b in bl]}


class View:
  """ti_curve_plannerd's screen state: the red bends already dinged (odometer of their end; bounded) and the onset count."""

  def __init__(self):
    self.dinged = []                  # odometer of each dinged red bend's end, at most DING_KEEP, dropped DING_FORGET_M past it
    self.n = 0                        # red-chip onsets (dings) so far in this process
    self.last = None                  # the last onset {"m", "n", "need"} (PARAM_DING)

  def step(self, planner, plan, t_mono, seq, on, mph, set_mph, floor, side):
    """One tick after Planner.step. -> (payload dict, ding dict or None, log list of ints)."""
    view = getattr(planner, "view", None) if on else None
    bl, rib, ends = [], [], False
    if on:
      if view is not None:
        idx, dist, need = view
        k = planner.kmed[idx]
        bl = bends(dist, need, np.sign(k), mph, set_mph, floor)
        rib, ends = ribbon(dist, k, mph)
      else:
        b = plan_bend(plan, side, floor, mph)
        bl = [b] if b is not None else []
    odo = float(getattr(planner, "odo", 0.0))
    self.dinged = [e for e in self.dinged if odo <= e + DING_FORGET_M]
    ck, ci = chip(bl, mph) if bl else (CLS_NEUTRAL, -1)
    ding = None
    if on and ck == CLS_RED and float(mph) >= MIN_MPH:
      key = odo + bl[ci]["end"]
      if not any(abs(key - e) <= DING_MATCH_M for e in self.dinged):
        self.dinged.append(key)
        del self.dinged[:-DING_KEEP]
        self.n += 1
        ding = self.last = {"m": round(float(t_mono), 2), "n": self.n, "need": int(bl[ci]["need"])}
    p = payload(t_mono, seq, on, mph, floor, rib, ends, bl)
    log = [self.n, int(ck), len(bl)]
    for b in p["b"][:3]:
      log += [b[0], b[2], b[3], b[4], b[5], b[6]]
    return p, ding, log


def view_json(p):
  return json.dumps(p, separators=(",", ":"))


def read_curve_ding(params_memory, now):
  """(onset time, need) of the latest red-chip onset when it is fresher than DING_FRESH_S, else (None, None). `now` is time.monotonic().
  frogpilot_planner raises tiCurveDing once per new onset time (never again for the same one)."""
  try:
    raw = params_memory.get(PARAM_DING)
    d = json.loads(raw) if raw else None
    if not isinstance(d, dict):
      return None, None
    m = float(d.get("m", -1e9))
    if not (0.0 <= now - m <= DING_FRESH_S):
      return None, None
    return m, int(d.get("need", 0))
  except Exception:
    return None, None


def curve_ding_due(params_memory, now, last_m, engaged):
  """frogpilot_planner's 10 Hz read: -> (the onset time to remember, True when tiCurveDing is to be raised now). An onset is raised
  once, only while engaged at the read that first sees it; an onset seen while not engaged is remembered and never raised later."""
  m, _ = read_curve_ding(params_memory, now)
  if m is None or m == last_m:
    return last_m, False
  return m, bool(engaged)
