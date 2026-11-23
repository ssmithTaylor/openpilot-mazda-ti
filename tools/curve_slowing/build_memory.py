#!/usr/bin/env python3
"""Build the road memory for curve slowing (selfdrive/car/mazda/ti_curve_memory.py format) OFF the car, from recorded drives.

PRIVACY: everything this tool writes holds positions and goes ONLY under DATA_DIR (E:\\openpilot-mazda-ti-data\\curve-slowing\\memory).
It prints counts, sizes and position-free statistics only. Nothing is written into the repository, and nothing is sent anywhere.

Drive set: by default every drive of the gate-checked 20 Hz extraction (seg20: train split, released, not reserved, not held), whose
rlog paths come from open_segs.json. `--drives` narrows it, `--exclude` removes drives.
`--source all` (the ON-CAR memory; owner 2026-11-23, RULES.md "Curve slowing from remembered roads": "Correct, all drives.", for
this feature only) uses every recorded drive found on this PC under RLOG_ROOT (and DATA_DIR/logs), held-out routes included,
EXCEPT the segments the release records reserve for a benchmark or keep back as test data (reserved_segments, KEPT_BACK); those
are never opened. Segments outside seg20 are read straight from their rlogs (read_rlog_seg: the same signals and decoding as the
seg20 extraction x1_extract.py, yaw from CAN 0x075 bytes 4-5) into the private cache DATA_DIR/cache/rl20/<tag>.npz.

How the memory is built (reusing the remembered-roads study, s2_build / s5_pieces):
  * each run of consecutive segments is resampled every SPACING_M of distance driven (wheel speed); heading from the positions +-15 m;
  * a chronological library of roads: drive by drive, a pass point joins the existing road point within TOL_M whose heading is within
    HDG_TOL; runs of unmatched points start a new road (direction-specific); later passes project onto the roads (continuous distance);
  * every pass contributes, on each road point it covers, the yaw-based curvature (left +), the speed and the bank (interpolated by
    distance); each pass's curvature is smoothed along the road (~10 m Gaussian) first. Curvature samples from unrepresentative
    stretches are EXCLUDED (they still count for position, speed and npass): blinker on +-5 s, the driver steering by hand
    (steeringPressed or |driver torque| >= TK_DRV counts) while lateral control is active +-2 s, below 19 mph, slowing to a stop (from a sustained aEgo < -0.8 m/s^2
    when the speed falls below 10 mph within 150 m) through the stop and the pull-away (back above 19 mph), the last 150 m before
    the pass leaves the trace (moves onto a road that is not this trace's trace_next continuation, or stops) and the first 150 m
    after it joins one (from a road that is not its predecessor, or from its start). kap is ROBUST (meta kap_rule "second<="):
    one usable pass -> its value; two or more -> the SECOND sharpest |kap|, signed by the median's sign, and the LOWEST |kap| where
    the usable passes disagree by more than DISAGREE x (phantom corners at traffic lights, swerves and lane changes cannot set a
    corner on a straight road). No usable pass -> kap = 0 and FLAG_NOKAP. EXCEPT where a pass had a steering-limit event or a
    real takeover near the point (within the flag windows): there kap is the sharpest of those event passes, from all their samples,
    exclusions not applied (the trouble is the evidence), and hand steering within +-3 s of such an event is never excluded; kap_med = the plain median of the same usable passes
    (0 where none), for the planner's road-change check;
    v_proven = the highest speed of a pass through the point with lateral control active, not braking to a stop or pulling away,
    and no steering-limit event, firm takeover or lane-line touch from PROVEN_M before to PROVEN_M after it (nan where none);
    v = median speed, bank = median (nan where unknown), npass = passes; grade = median over passes of the road grade (radians,
    + uphill in the direction of travel; nan where unknown): liveLocationKalman calibratedOrientationNED.value[1] (pitch in the
    calibrated car frame, nose up +, so no further calibration), low-passed with a Gaussian of sigma GRADE_SIGMA_S (~3 s window);
  * flags: FLAG_LIMIT where a pass had the steering at its limit (study's run_events: |ti_cmd| >= 590 for >= 0.3 s, lateral active,
    driver not pressing), spread from LIMIT_BEFORE_M before to LIMIT_AFTER_M after the event; FLAG_TAKEOVER for a FIRM takeover
    (lateral active, |driver torque| >= TK_DRV counts for >= 0.5 s, then within 2 s lateral control inactive or openpilot disengaged;
    steeringPressed alone is not enough; never a brake to a stop: not counted with the brake pressed in that window or a stop below
    10 mph within 150 m after it) +-TK_M, spread only over points of a real bend (stored |kap| >= BEND_KAP); v_flag = lowest speed at the event of a pass that set the flag; a flag window that
    runs past a road's end / start is carried onto the first / last points of the linked next / previous trace (one hop);
    FLAG_LANE (+-LANE_M) where, with lateral control active, the nearest modelV2 lane line (laneLineProbs > 0.5) came within
    LANE_HALF_W of the car's centre at x = 0 for >= 0.3 s with no blinker within +-5 s (cache lane/<tag>.npz, position-free),
    spread only over points of a real bend like FLAG_TAKEOVER;
  * roads shorter than MIN_TRACE_M are dropped; roads are then cut where a pass moved from one road to another (so a trace starts or
    ends at every join and fork, cuts at least MIN_CUT_PTS apart) and trace_next links a trace to the single trace whose first point is
    within LINK_DIST_M / LINK_HDG of its last point and ahead of it (-1 if none or several).
  All leave-one-out files share one local frame (lat0/lon0 from the FULL default drive set, kept in DATA_DIR/cache/origin.json).

Private caches (DATA_DIR/cache, positions allowed there only):
  pos/<tag>.npz        per segment, raw liveLocationKalman rows: llk[:, 0..7] = t (s from the segment's first carState), latitude,
                       longitude, positionGeodetic.valid, gpsOK, calibratedOrientationNED.value[2] (rad, NED yaw = bearing),
                       calibratedOrientationNED.value[1] (rad, pitch, nose up +), positionGeodetic.value[2] (altitude, m)
  passes.pkl           the prepared runs (local x/y on the 20 Hz grid + the per-run signals) used by the builds
  origin.json          lat0, lon0, kx, ky
  drive_<drive>.npz    REPLAY cache, one per drive. 20 Hz arrays, all runs of the drive concatenated:
                         t (s from the run start), lat, lon (float64 deg, nan where not pos_ok), pos_ok (bool: positionGeodetic.valid
                         and gpsOK), bearing_deg (deg clockwise from north, from calibratedOrientationNED.value[2]), v (m/s),
                         kap (1/m, yaw-based, left +), cr_en, cr_set (as logged, m/s), lead, en (openpilot enabled),
                         lat_act (lateral active; named lat_act because `lat` is the latitude), brk, gas, pressed, blink,
                         ti (ti_cmd), model_la5 (max |predicted lateral accel| over the model forecast up to 5 s, m/s^2),
                         grade (rad, + uphill, the same low-passed pitch as the memory; nan where unknown)
                       run_off int64 [n_runs + 1]: run r is samples run_off[r] .. run_off[r+1]-1
                       seg_tags str [n_segs], seg_off int64 [n_segs] (sample index where each segment starts), seg_run int [n_segs]
                       corners (position-free, the study's corners.pkl entries of this drive): c_id str, c_run, c_i20 (sample index
                       within the run), c_sat, c_cruise, c_v_mph, c_la, c_kap, c_dir ('left'/'right'), c_nprev

Run (worktree venv python, PYTHONPATH with the cereal shim + this worktree, below-normal priority, up to 6 workers):
  python tools/curve_slowing/build_memory.py                       # memory/curve_memory_v1.npz from the default drive set
  python tools/curve_slowing/build_memory.py --loo-all             # memory/loo/<drive>.npz, each from every other drive
  python tools/curve_slowing/build_memory.py --replay-caches       # memory/cache/drive_<drive>.npz
  python tools/curve_slowing/build_memory.py --check               # position-free validation printout
"""
import argparse
import collections
import glob
import json
import math
import os
import pickle
import sys
import time

import numpy as np

try:
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M
except ImportError:
  sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
  from selfdrive.car.mazda import ti_curve_memory as M

DATA_DIR = r'E:\openpilot-mazda-ti-data\curve-slowing\memory'
STUDY = 'E:/openpilot-mazda-ti-data/remembered-roads-20261123'
SEG20 = 'E:/openpilot-mazda-ti-data/codex-designs-20261123/_work/seg20'
OPEN_SEGS = 'E:/openpilot-mazda-ti-data/cruise-slowing-20261122/open_segs.json'
RLOG_ROOT = 'E:/openpilot-mazda-ti-data'
# the simulator's release records (the reserved / kept-back segments the all-drives build must never open); --releases or the
# CURVE_RELEASES environment variable (worker processes inherit it) when they are not beside this tool
RELEASES = os.environ.get('CURVE_RELEASES') or os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'mazda_sim', 'releases'))
KEPT_BACK = {'0000033b': lambda s: s % 3 == 1, '0000033c': lambda s: s % 3 == 1}   # releases g / h: kept back as test data, unread

DT = 0.05
SP = M.SPACING_M
TOL_M = 15.0
HDG_TOL = math.radians(35.0)
HD_HALF = 3                    # heading baseline: +-3 points
MIN_ROAD_PTS = 2
MIN_TRACE_M = 100.0
MIN_CUT_PTS = 10               # cuts at least 50 m apart and from a trace's ends
LIMIT_BEFORE_M, LIMIT_AFTER_M = 80.0, 80.0
TK_M = 60.0
TK_DRV = 5.0                   # driver-torque counts
TK_SUSTAIN = 10               # 0.5 s at 20 Hz
TK_FOLLOW = 40                # 2 s
TK_STOP_MPH, TK_STOP_M = 19.0, 150.0   # a takeover that ends in a stop or below 19 mph within 150 m is a stop, not trouble
BEND_KAP = 0.002               # takeover and lane-line flags are spread only over points of a real bend (stored |kap| >= this)
EVENT_SLACK = 20               # 20 Hz samples: an event belongs to a piece within 1 s of it (s5_pieces)
LINK_DIST_M = 20.0
LINK_HDG = math.radians(30.0)
KAP_SIGMA_M = 10.0
GRADE_SIGMA_S = 1.0            # Gaussian sigma of the grade low-pass (about a 3 s window)
LLK_COLS = 8
SAT_TI = 590.0
SAT_MIN = 6


def wrap(a):
  return (a + np.pi) % (2 * np.pi) - np.pi


def low_priority():
  try:
    import ctypes
    ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x00004000)   # BELOW_NORMAL
  except Exception:
    pass


def gauss(a, sigma_pts):
  from scipy.ndimage import gaussian_filter1d
  if len(a) < 2 or sigma_pts <= 0:
    return np.asarray(a, float)
  return gaussian_filter1d(np.asarray(a, float), sigma_pts, mode='nearest')


# ---- events (s5_pieces.run_events) --------------------------------------------------------------------------------------------------
def run_events(ti, lat_act, pressed, en, drv, brk=None, v=None, s=None):
  """steering-limit events (request at the clamp >= 0.3 s, lateral active, driver not pressing; s5_pieces) and firm takeovers
  (lateral active at the onset of |driver torque| >= TK_DRV sustained >= 0.5 s, then within 2 s of its end lateral control inactive
  or openpilot disengaged) as 20 Hz indices (the onset). A brake to a stop is never a takeover: not counted when the brake is
  pressed in that window (openpilot disengaged by the brake) or the car is below TK_STOP_MPH within TK_STOP_M after the onset."""
  sat = (np.abs(ti) >= SAT_TI) & (lat_act > .5) & (pressed < .5)
  m = np.r_[0, sat.astype(np.int8), 0]
  ed = np.where(np.diff(m) != 0)[0].reshape(-1, 2)
  sats = [int(a) for a, b in ed if (b - a) >= SAT_MIN]
  hard = np.abs(drv) >= TK_DRV
  m = np.r_[0, hard.astype(np.int8), 0]
  ed = np.where(np.diff(m) != 0)[0].reshape(-1, 2)
  la, e = lat_act > .5, en > .5
  tk = []
  for a, b in ed:
    if b - a < TK_SUSTAIN or not la[a]:
      continue
    w = slice(a, min(len(la), b + TK_FOLLOW))
    if not ((~la[w]).any() or (~e[w]).any()):
      continue
    # M4: a brake in the window is the owner's protocol for taking over (brake to disengage), so it does NOT discard a takeover; only
    # a brake that goes on to a stop or below TK_STOP_MPH (the stop rule below) does, and the flag is spread over real bends only
    if v is not None and s is not None:
      ahead = (np.asarray(s) >= s[a]) & (np.asarray(s) <= s[a] + TK_STOP_M)
      if (np.asarray(v)[ahead] < TK_STOP_MPH * 0.44704).any():
        continue
    tk.append(int(a))
  return np.array(sats, int), np.array(tk, int)


# ---- the core: passes -> memory arrays (no file access; the synthetic tests use this) -------------------------------------------------
def resample(p):
  """5 m points of one pass: 20 Hz index, x, y, heading (nan where unknown), position ok."""
  s = p['s']
  npts = int(s[-1] // SP) + 1 if len(s) else 0
  idx20 = np.searchsorted(s, np.arange(npts) * SP).clip(0, max(len(s) - 1, 0))
  px, py = p['X'][idx20], p['Y'][idx20]
  okp = np.isfinite(px) & np.isfinite(py)
  a = np.clip(np.arange(npts) - HD_HALF, 0, npts - 1)
  b = np.clip(np.arange(npts) + HD_HALF, 0, npts - 1)
  dx, dy = px[b] - px[a], py[b] - py[a]
  hd = np.where(okp[a] & okp[b] & (b > a) & (np.hypot(dx, dy) > 2.0), np.arctan2(dy, dx), np.nan)
  return idx20, px, py, hd, okp & np.isfinite(hd)


def _project(Lx, Ly, Ls, Lroad, j, x, y):
  """continuous along-road distance of (x, y) projected on the polyline pieces next to library point j (vectorised s2 project_on_road)"""
  bs = Ls[j].copy()
  bd = np.hypot(x - Lx[j], y - Ly[j])
  NL = len(Lx)
  for sh in (-1, 1):
    jj = j + sh
    ok = (jj >= 0) & (jj < NL)
    jj = np.clip(jj, 0, NL - 1)
    ok &= Lroad[jj] == Lroad[j]
    dx, dy = Lx[jj] - Lx[j], Ly[jj] - Ly[j]
    L2 = np.maximum(dx * dx + dy * dy, 1e-6)
    u = ((x - Lx[j]) * dx + (y - Ly[j]) * dy) / L2
    d = np.hypot(x - (Lx[j] + u * dx), y - (Ly[j] + u * dy))
    better = ok & (u >= 0) & (u <= 1) & (d < bd)
    bs = np.where(better, Ls[j] + u * (Ls[jj] - Ls[j]), bs)
    bd = np.where(better, d, bd)
  return bs


def build_library(passes):
  """Chronological library (drive by drive, in the order given). -> (lib dict, per-pass alignment list)."""
  from scipy.spatial import cKDTree
  Lx, Ly, Lh, Lroad, Ls = (np.zeros(0) for _ in range(5))
  Lroad = Lroad.astype(int)
  road_base, road_len = [], []
  aligns = [None] * len(passes)
  order = list(range(len(passes)))
  drives = list(collections.OrderedDict.fromkeys(passes[i]['drive'] for i in order))
  by_drive = collections.defaultdict(list)
  for i in order:
    by_drive[passes[i]['drive']].append(i)
  for d in drives:
    NL = len(Lx)
    tree = cKDTree(np.c_[Lx, Ly]) if NL else None
    newpts = []
    for pi in by_drive[d]:
      idx20, px, py, hd, okp = resample(passes[pi])
      n = len(px)
      lj = np.full(n, -1)
      sl = np.full(n, np.nan)
      if tree is not None and okp.any():
        q = np.where(okp)[0]
        dd, jj = tree.query(np.c_[px[q], py[q]], k=4, distance_upper_bound=TOL_M)
        dd, jj = np.atleast_2d(dd), np.atleast_2d(jj)
        good = np.isfinite(dd) & (jj < NL)
        jc = np.clip(jj, 0, max(NL - 1, 0))
        good &= np.abs(wrap(hd[q][:, None] - Lh[jc])) < HDG_TOL
        anyg = good.any(1)
        first = np.argmax(good, 1)
        sel = q[anyg]
        j = jc[anyg, first[anyg]]
        lj[sel] = j
        sl[sel] = _project(Lx, Ly, Ls, Lroad, j, px[sel], py[sel])
      aligns[pi] = dict(idx20=idx20, lj=lj, sl=sl, n=n)
      newpts.append((pi, px, py, hd, okp))
    ins = [[], [], [], [], []]
    for pi, px, py, hd, okp in newpts:
      a = aligns[pi]
      lj = a['lj']
      valid = (lj < 0) & okp
      k = 0
      n = len(px)
      while k < n:
        if not valid[k]:
          k += 1
          continue
        k2 = k
        while k2 + 1 < n and valid[k2 + 1]:
          k2 += 1
        if k2 - k + 1 >= MIN_ROAD_PTS:
          rd = len(road_base)
          base = len(Lx) + len(ins[0])
          road_base.append(base)
          road_len.append(k2 - k + 1)
          for qq, kk in enumerate(range(k, k2 + 1)):
            ins[0].append(px[kk]); ins[1].append(py[kk]); ins[2].append(hd[kk]); ins[3].append(rd); ins[4].append(qq * SP)
            lj[kk] = base + qq
            a['sl'][kk] = qq * SP
        k = k2 + 1
    if ins[0]:
      Lx = np.r_[Lx, ins[0]]; Ly = np.r_[Ly, ins[1]]; Lh = np.r_[Lh, ins[2]]
      Lroad = np.r_[Lroad, ins[3]].astype(int); Ls = np.r_[Ls, ins[4]]
  lib = dict(x=Lx, y=Ly, h=Lh, road=Lroad, s=Ls, base=np.array(road_base, int), len=np.array(road_len, int))
  return lib, aligns


# rows columns (one per road point covered by a pass)
C_PT, C_PASS, C_KAP, C_V, C_BANK, C_GRADE, C_EXCL, C_PIECE, C_TOEND, C_CLEAN, C_FROMSTART, C_EVENT = range(12)
EVENT_PROTECT_S = 3.0          # hand steering within this of a steering-limit event or a real takeover is never excluded
EX_BLINK, EX_HAND, EX_SLOW, EX_LEAVE, EX_STOP, EX_JOIN = 1, 2, 4, 8, 16, 32
EX_NAMES = {EX_BLINK: 'blinker +-5 s', EX_HAND: 'steering by hand +-2 s', EX_SLOW: 'below 19 mph', EX_LEAVE: 'last 150 m before leaving',
            EX_STOP: 'slowing to a stop / pulling away', EX_JOIN: 'first 150 m after joining'}
BLINK_PAD_S, HAND_PAD_S, SLOW_MPH, LEAVE_M = 5.0, 2.0, 19.0, 150.0
STOP_DECEL, STOP_DECEL_S, STOP_MPH, STOP_LOOK_M = -0.8, 0.5, 10.0, 150.0
PROVEN_M = 80.0
LANE_HALF_W = 0.9              # m: the car's half width; a lane line nearer than this to the centre is touched or crossed
LANE_PROB = 0.5
LANE_MIN = 6                   # 0.3 s at 20 Hz
LANE_BLINK_S = 5.0
LANE_M = 60.0
DISAGREE = 2.0                 # usable passes disagreeing by more than this factor in |kap|: the lower value


def dilate(mask, pad):
  """mask widened by pad samples each way"""
  if pad <= 0 or not mask.any():
    return mask.copy()
  c = np.cumsum(np.r_[0, mask.astype(np.int64)])
  n = len(mask)
  i = np.arange(n)
  return (c[np.minimum(n, i + pad + 1)] - c[np.maximum(0, i - pad)]) > 0


def stop_mask(v, s, aego=None):
  """slowing to a stop and pulling away: from where a sustained deceleration (aEgo < STOP_DECEL for STOP_DECEL_S) starts, when the speed
  falls below STOP_MPH within STOP_LOOK_M of it, to the stop, then until the speed is back above SLOW_MPH"""
  v = np.asarray(v, float)
  n = len(v)
  out = np.zeros(n, bool)
  if n < 3:
    return out
  a = np.asarray(aego, float) if aego is not None else gauss(np.gradient(v) / DT, 0.25 / DT)
  dec = a < STOP_DECEL
  m = np.r_[0, dec.astype(np.int8), 0]
  ed = np.where(np.diff(m) != 0)[0].reshape(-1, 2)
  slow = v < STOP_MPH * 0.44704
  slow_idx = np.where(slow)[0]
  fast_idx = np.where(v >= SLOW_MPH * 0.44704)[0]
  for i0, i1 in ed:
    if i1 - i0 < int(STOP_DECEL_S / DT):
      continue
    j = slow_idx[np.searchsorted(slow_idx, i0):]
    if not len(j) or s[j[0]] > s[i1 - 1] + STOP_LOOK_M:
      continue
    j = int(j[0])
    f = fast_idx[np.searchsorted(fast_idx, j):]
    j2 = int(f[0]) if len(f) else n
    out[i0:j2] = True
  return out


def excl_bits(p):
  """20 Hz exclusion bits of a pass (curvature only): blinker +-5 s, driver steering by hand while lateral control is active +-2 s,
  below 19 mph, slowing to a stop and pulling away"""
  n = len(p['s'])
  out = np.zeros(n, np.uint8)
  if 'blink' in p:
    out |= np.where(dilate(np.asarray(p['blink']) > .5, int(BLINK_PAD_S / DT)), EX_BLINK, 0).astype(np.uint8)
  hand = np.zeros(n, bool)
  if 'pressed' in p:
    hand |= np.asarray(p['pressed']) > .5
  if 'drv' in p:
    hand |= np.abs(np.asarray(p['drv'])) >= TK_DRV
  if 'lat_act' in p:
    hand &= np.asarray(p['lat_act']) > .5      # only while openpilot steers: manual driving is the car's real path
  ev = np.zeros(n, bool)
  for k in ('sat_i', 'tk_i'):
    idx = np.asarray(p.get(k, ()), int)
    ev[idx[(idx >= 0) & (idx < n)]] = True
  hand = dilate(hand, int(HAND_PAD_S / DT)) & ~dilate(ev, int(EVENT_PROTECT_S / DT))   # the trouble is the evidence: never excluded near it
  out |= np.where(hand, EX_HAND, 0).astype(np.uint8)
  out |= np.where(np.asarray(p['v']) < SLOW_MPH * 0.44704, EX_SLOW, 0).astype(np.uint8)
  out |= np.where(stop_mask(p['v'], p['s'], p.get('aego')), EX_STOP, 0).astype(np.uint8)
  return out


def lane_events(lane_d, lat_act, blink):
  """lane-line touches: lateral control active, the nearest lane line (prob > LANE_PROB) within LANE_HALF_W of the centre for at least
  0.3 s, no blinker within +-5 s. -> 20 Hz onset indices"""
  d = np.asarray(lane_d, float)
  hit = np.isfinite(d) & (d <= LANE_HALF_W) & (np.asarray(lat_act) > .5)
  hit &= ~dilate(np.asarray(blink) > .5, int(LANE_BLINK_S / DT))
  m = np.r_[0, hit.astype(np.int8), 0]
  ed = np.where(np.diff(m) != 0)[0].reshape(-1, 2)
  return np.array([int(a) for a, b in ed if b - a >= LANE_MIN], int)


def collect(passes, lib, aligns):
  """Per-pass samples on road points, flag intervals, road-to-road transitions (cut points) and the pieces (a pass's stretch on
  one road) with how each one ends: ('same',) the pass carries on along the road after a position gap, ('road', rb, qb) it moved
  onto road rb at point qb, ('end',) the pass stopped there."""
  rows = []
  flags = []         # (road, g_lo, g_hi, flag, v)
  pieces = []        # (road, last point index, exit)
  cuts = collections.defaultdict(set)
  trans = set()      # (road left, its last point index, road entered, its first point index) as driven
  Lroad, base = lib['road'], lib['base']
  for pi, (p, a) in enumerate(zip(passes, aligns)):
    lj, sl, idx20 = a['lj'], a['sl'], a['idx20']
    ok = lj >= 0
    rd_of = np.where(ok, Lroad[np.where(ok, lj, 0)], -1)
    s = p['s']
    sats, tks = p['sat_i'], p['tk_i']
    lns = np.asarray(p.get('lane_i', np.zeros(0, int)), int)
    ev_s = np.sort(np.r_[s[sats], s[tks], s[lns]]) if len(sats) + len(tks) + len(lns) else np.zeros(0)
    xb = excl_bits(p)
    la = (np.asarray(p['lat_act']) > .5) if 'lat_act' in p else np.ones(len(s), bool)
    for k in np.where(ok[:-1] & ok[1:] & (rd_of[:-1] != rd_of[1:]))[0]:
      ra, rb = rd_of[k], rd_of[k + 1]
      cuts[rb].add(int(lj[k + 1] - base[rb]))
      cuts[ra].add(int(lj[k] - base[ra]) + 1)
      trans.add((int(ra), int(lj[k] - base[ra]), int(rb), int(lj[k + 1] - base[rb])))
    k, n = 0, a['n']
    while k < n:
      if rd_of[k] < 0:
        k += 1
        continue
      k2 = k
      while k2 + 1 < n and rd_of[k2 + 1] == rd_of[k]:
        k2 += 1
      rd = rd_of[k]
      ks = np.arange(k, k2 + 1)
      s_lib = sl[ks]
      keep = np.r_[True, np.diff(np.maximum.accumulate(s_lib)) > 1e-6]
      ks, s_lib = ks[keep], s_lib[keep]
      if len(ks) >= 2:
        q0 = int(math.ceil(s_lib[0] / SP - 1e-9))
        q1 = min(int(math.floor(s_lib[-1] / SP + 1e-9)), int(lib['len'][rd]) - 1)
        if q1 >= q0:
          # how the pass leaves this stretch
          k3 = k2 + 1
          while k3 < n and rd_of[k3] < 0:
            k3 += 1
          k0 = k - 1
          while k0 >= 0 and rd_of[k0] < 0:
            k0 -= 1
          if k0 < 0:
            entry = ('start',)
          elif rd_of[k0] == rd:
            entry = ('same',) if sl[k0] < s_lib[0] else ('start',)
          else:
            entry = ('road', int(rd_of[k0]), int(lj[k0] - base[rd_of[k0]]))
          if k3 >= n:
            exit_ = ('end',)
          elif rd_of[k3] == rd:
            exit_ = ('same',) if sl[k3] > s_lib[-1] else ('end',)
          else:
            exit_ = ('road', int(rd_of[k3]), int(lj[k3] - base[rd_of[k3]]))
          piece = len(pieces)
          pieces.append((int(rd), int(q1), exit_, int(q0), entry))
          q = np.arange(q0, q1 + 1)
          g = q * SP
          sr = np.interp(g, s_lib, s[idx20[ks]])
          i20 = np.clip(np.searchsorted(s, sr), 0, len(s) - 1)
          kap = gauss(np.interp(sr, s, p['kap']), KAP_SIGMA_M / SP)     # ~10 m along the road, per pass
          gr = p['grade'][i20] if 'grade' in p else np.full(len(q), np.nan)
          if len(ev_s):
            lo_i = np.searchsorted(ev_s, sr - PROVEN_M, side='left')
            hi_i = np.searchsorted(ev_s, sr + PROVEN_M, side='right')
            clean = (hi_i == lo_i) & la[i20]
          else:
            clean = la[i20].copy()
          clean &= (xb[i20] & EX_STOP) == 0            # never proven while braking to a stop or pulling away
          evt = np.zeros(len(q), bool)                   # this pass had a steering-limit event / real takeover near the point
          for ix, w in ((sats, max(LIMIT_BEFORE_M, LIMIT_AFTER_M)), (tks, TK_M)):
            if len(ix):
              es = np.sort(s[ix])
              evt |= np.searchsorted(es, sr + w, side='right') > np.searchsorted(es, sr - w, side='left')
          rows.append(np.c_[base[rd] + q, np.full(len(q), pi), kap, p['v'][i20], p['bank'][i20], gr, xb[i20], np.full(len(q), piece),
                            (q1 - q) * SP, clean, (q - q0) * SP, evt])
          for evs, fl, lo, hi in ((sats, M.FLAG_LIMIT, LIMIT_BEFORE_M, LIMIT_AFTER_M), (tks, M.FLAG_TAKEOVER, TK_M, TK_M),
                                  (lns, M.FLAG_LANE, LANE_M, LANE_M)):
            for e in evs:
              if i20[0] - EVENT_SLACK <= e <= i20[-1] + EVENT_SLACK:
                ge = float(np.interp(s[e], sr, g)) if len(sr) > 1 else float(g[0])
                flags.append((rd, ge - lo, ge + hi, fl, float(p['v'][e])))
      k = k2 + 1
  rows = np.concatenate(rows) if rows else np.zeros((0, 12))
  return rows, flags, cuts, trans, pieces


def group_median(key, val, n):
  """median of val per integer key in [0, n) ignoring nan; nan where none."""
  ok = np.isfinite(val)
  key, val = key[ok], val[ok]
  out = np.full(n, np.nan)
  if not len(key):
    return out
  o = np.lexsort((val, key))
  key, val = key[o], val[o]
  uk, start, cnt = np.unique(key, return_index=True, return_counts=True)
  out[uk] = 0.5 * (val[start + (cnt - 1) // 2] + val[start + cnt // 2])
  return out


def robust_kap(key, val, n):
  """per key: 1 sample -> it; 2+ -> the SECOND sharpest |val|, signed by the median's sign, and where the samples disagree by more
  than DISAGREE x in |val| the LOWEST |val| instead (meta kap_rule "second<="). nan if none."""
  ok = np.isfinite(val)
  key, val = key[ok], val[ok]
  out = np.full(n, np.nan)
  if not len(key):
    return out
  o = np.lexsort((np.abs(val), key))
  key, val = key[o], val[o]
  uk, start, cnt = np.unique(key, return_index=True, return_counts=True)
  second = val[start + np.maximum(cnt - 2, 0)]
  lowest = np.abs(val[start])
  sharpest = np.abs(val[start + cnt - 1])
  sgn = np.sign(group_median(key, val, n)[uk])
  sgn = np.where(sgn == 0, np.sign(second), sgn)
  mag = np.abs(second)
  disagree = (cnt >= 2) & (sharpest > DISAGREE * lowest)
  mag = np.where(disagree, np.minimum(mag, lowest), mag)
  out[uk] = np.where(cnt < 2, second, sgn * mag)
  return out


def merge(rows, NL, usable=None):
  """per library point: robust kap from the usable samples, median v, bank and grade, npass, and v_proven (all samples)"""
  if len(rows):
    key = rows[:, C_PT].astype(np.int64) * 100000 + rows[:, C_PASS].astype(np.int64)
    _, first = np.unique(key, return_index=True)      # one sample per (point, pass)
    first = np.sort(first)
    rows = rows[first]
    usable = usable[first] if usable is not None else np.ones(len(rows), bool)
  else:
    usable = np.zeros(0, bool)
  pt = rows[:, C_PT].astype(np.int64)
  kap = robust_kap(pt[usable], rows[usable, C_KAP], NL)
  kap_med = group_median(pt[usable], rows[usable, C_KAP], NL)
  # where a pass had a steering-limit event or a real takeover near the point, the curvature comes from ALL passes' samples there
  # (exclusions do not apply): the sharpest of the passes that had the event
  ev = rows[:, C_EVENT] > .5
  if ev.any():
    pe, ke = pt[ev], rows[ev, C_KAP]
    o = np.lexsort((np.abs(ke), pe))
    pe, ke = pe[o], ke[o]
    uk, start, cnt = np.unique(pe, return_index=True, return_counts=True)
    kap[uk] = ke[start + cnt - 1]
  v = group_median(pt, rows[:, C_V], NL)
  bank = group_median(pt, rows[:, C_BANK], NL)
  grade = group_median(pt, rows[:, C_GRADE], NL)
  npass = np.bincount(pt, minlength=NL)[:NL]
  v_proven = np.full(NL, np.nan)
  cl = rows[:, C_CLEAN] > .5
  np.fmax.at(v_proven, pt[cl], rows[cl, C_V])
  return kap, v, bank, npass, grade, v_proven, kap_med


def spread_flags(flags, lib, kap=None):
  """flag windows onto road points; takeover and lane-line flags only onto points of a real bend (|kap| >= BEND_KAP)"""
  NL = len(lib['x'])
  fl = np.zeros(NL, np.uint8)
  vf = np.full(NL, np.nan)
  bend = np.abs(kap) >= BEND_KAP if kap is not None else np.ones(NL, bool)
  for rd, lo, hi, f, v in flags:
    b, L = int(lib['base'][rd]), int(lib['len'][rd])
    q0, q1 = max(0, int(math.ceil(lo / SP - 1e-9))), min(L - 1, int(math.floor(hi / SP + 1e-9)))
    if q1 < q0:
      continue
    idx = np.arange(b + q0, b + q1 + 1)
    if f & (M.FLAG_TAKEOVER | M.FLAG_LANE):
      idx = idx[bend[idx]]
    fl[idx] |= np.uint8(f)
    vf[idx] = np.fmin(vf[idx], v)
  return fl, vf


def fill_nan(a):
  a = np.asarray(a, float).copy()
  ok = np.isfinite(a)
  if ok.all():
    return a
  if not ok.any():
    return np.zeros_like(a)
  i = np.arange(len(a))
  a[~ok] = np.interp(i[~ok], i[ok], a[ok])
  return a


def split_points(L, cut_set):
  """accepted cut positions (a new trace starts at each) of a road of L points"""
  out, last = [], 0
  for c in sorted(cut_set):
    if c - last >= MIN_CUT_PTS and L - c >= MIN_CUT_PTS:
      out.append(c)
      last = c
  return out


def link_traces(x, y, hdg, trace_start, observed=None):
  """trace_next: the single trace whose first point is within LINK_DIST_M of this trace's last point, not behind it, and either
  within LINK_HDG of its heading or seen driven straight after it (observed: {trace: {next traces}}, from the passes; a pass that
  turned onto a road joins it mid-turn, where the headings still differ). -1 when none or several (a fork)."""
  nt = len(trace_start) - 1
  first = np.asarray(trace_start[:-1])
  last = np.asarray(trace_start[1:]) - 1
  out = np.full(nt, -1, np.int64)
  if nt == 0:
    return out
  from scipy.spatial import cKDTree
  tree = cKDTree(np.c_[x[first], y[first]])
  for t in range(nt):
    e = last[t]
    cands = []
    seen = (observed or {}).get(t, set())
    for u in tree.query_ball_point([x[e], y[e]], LINK_DIST_M):
      if u == t:
        continue
      f = first[u]
      if abs(wrap(hdg[f] - hdg[e])) > LINK_HDG and u not in seen:
        continue
      along = (x[f] - x[e]) * math.cos(hdg[e]) + (y[f] - y[e]) * math.sin(hdg[e])
      if along < -2.0:
        continue
      cands.append(u)
    if len(cands) == 1:
      out[t] = cands[0]
  return out


def carry_flags(arrays, flags, lib, tid):
  """a flag window running past a kept road's last (first) point continues on the linked next (previous) trace, one hop"""
  ts, nx = arrays['trace_start'], arrays['trace_next']
  prev = collections.defaultdict(list)
  for t, u in enumerate(nx):
    if u >= 0:
      prev[int(u)].append(t)
  for rd, lo, hi, f, v in flags:
    L = int(lib['len'][rd])
    t_last, t_first = tid.get((rd, L - 1, 'end')), tid.get((rd, 0))
    end_m = (L - 1) * SP
    if t_last is not None and hi > end_m + SP / 2 and nx[t_last] >= 0:
      u = int(nx[t_last])
      k = min(int(ts[u + 1] - ts[u]), int(math.floor((hi - end_m) / SP)))
      sl = np.arange(int(ts[u]), int(ts[u]) + k)
      if f & (M.FLAG_TAKEOVER | M.FLAG_LANE):
        sl = sl[np.abs(arrays['kap'][sl]) >= BEND_KAP]
      arrays['flags'][sl] |= np.uint8(f)
      arrays['v_flag'][sl] = np.fmin(arrays['v_flag'][sl], v)
    if t_first is not None and lo < -SP / 2:
      for u in prev.get(t_first, []):
        k = min(int(ts[u + 1] - ts[u]), int(math.floor(-lo / SP)))
        sl = np.arange(int(ts[u + 1]) - k, int(ts[u + 1]))
        if f & (M.FLAG_TAKEOVER | M.FLAG_LANE):
          sl = sl[np.abs(arrays['kap'][sl]) >= BEND_KAP]
        arrays['flags'][sl] |= np.uint8(f)
        arrays['v_flag'][sl] = np.fmin(arrays['v_flag'][sl], v)


def build(passes, info=None):
  """passes (chronological) -> (arrays for ti_curve_memory.save, stats). Each pass: dict(drive, s, X, Y, v, kap, bank, sat_i, tk_i and,
  optional, grade, blink, pressed, drv, lat_act). `info` (a dict) receives position-free diagnostics."""
  passes = [p for p in passes if len(p['s']) > 1]
  lib, aligns = build_library(passes)
  rows, flags, cuts, trans, pieces = collect(passes, lib, aligns)
  NL = len(lib['x'])
  # ---- layout: kept roads cut into traces (geometry only) ----
  hdg_lib = np.zeros(NL)
  order, starts, tid = [], [0], {}
  trace_lib = np.full(NL, -1, np.int64)
  n_roads = 0
  for rd in range(len(lib['base'])):
    b, L = int(lib['base'][rd]), int(lib['len'][rd])
    if L * SP < MIN_TRACE_M:
      continue
    n_roads += 1
    xs, ys = gauss(lib['x'][b:b + L], 1.0), gauss(lib['y'][b:b + L], 1.0)
    hdg_lib[b:b + L] = np.arctan2(np.gradient(ys), np.gradient(xs))
    cps = [0] + split_points(L, cuts.get(rd, ())) + [L]
    for a, c in zip(cps[:-1], cps[1:]):
      t = len(starts) - 1
      tid[(rd, a)] = t
      tid[(rd, c - 1, 'end')] = t
      trace_lib[b + a:b + c] = t
      order.append(np.arange(b + a, b + c))
      starts.append(starts[-1] + c - a)
  if len(starts) == 1:
    raise ValueError('no road long enough to keep')
  order = np.concatenate(order)
  ts = np.array(starts, np.int64)
  x, y, hdg = lib['x'][order], lib['y'][order], hdg_lib[order]
  observed = collections.defaultdict(set)
  for ra, qa, rb, qb in trans:
    ta, tb = tid.get((ra, qa, 'end')), tid.get((rb, qb))
    if ta is not None and tb is not None:
      observed[ta].add(tb)
  nxt = link_traces(x, y, hdg, ts, observed)
  # ---- which pieces leave their trace (the pass's own continuation is not this trace's) ----
  pos_in_trace = np.full(NL, -1, np.int64)
  pos_in_trace[order] = np.arange(len(order)) - np.repeat(ts[:-1], np.diff(ts))
  tlen = np.diff(ts)
  leaving = np.zeros(len(pieces), bool)
  joining = np.zeros(len(pieces), bool)
  prev_of = collections.defaultdict(set)
  for t_, u_ in enumerate(nxt):
    if u_ >= 0:
      prev_of[int(u_)].add(t_)
  for i, (rd, q1, ex, q0, en) in enumerate(pieces):
    j = int(lib['base'][rd]) + q0
    t = trace_lib[j]
    if t < 0 or en[0] == 'same':
      continue
    if en[0] == 'start':
      joining[i] = True
      continue
    ja = int(lib['base'][en[1]]) + min(en[2], int(lib['len'][en[1]]) - 1)
    ta = trace_lib[ja]
    at_start = pos_in_trace[j] <= 1
    at_end = ta >= 0 and pos_in_trace[ja] >= tlen[ta] - 2
    joining[i] = not (at_start and at_end and ta in prev_of.get(int(t), set()))
  for i, (rd, q1, ex, _q0, _en) in enumerate(pieces):
    j = int(lib['base'][rd]) + q1
    t = trace_lib[j]
    if t < 0 or ex[0] == 'same':
      continue
    if ex[0] == 'end':
      leaving[i] = True
      continue
    jb = int(lib['base'][ex[1]]) + min(ex[2], int(lib['len'][ex[1]]) - 1)
    tb = trace_lib[jb]
    at_end = pos_in_trace[j] >= tlen[t] - 2
    at_start = tb >= 0 and pos_in_trace[jb] <= 1
    leaving[i] = not (at_end and at_start and nxt[t] == tb)
  excl = rows[:, C_EXCL].astype(np.uint8) if len(rows) else np.zeros(0, np.uint8)
  if len(rows):
    pc = rows[:, C_PIECE].astype(np.int64)
    excl = excl | np.where(leaving[pc] & (rows[:, C_TOEND] <= LEAVE_M), EX_LEAVE, 0).astype(np.uint8)
    excl = excl | np.where(joining[pc] & (rows[:, C_FROMSTART] <= LEAVE_M), EX_JOIN, 0).astype(np.uint8)
  usable = excl == 0
  kap, v, bank, npass, grade, v_proven, kap_med = merge(rows, NL, usable)
  nokap = ~np.isfinite(kap)
  kap = np.where(nokap, 0.0, kap)
  kap_med = np.where(np.isfinite(kap_med), kap_med, 0.0)
  fl, vf = spread_flags(flags, lib, kap)
  fl = fl | np.where(nokap, M.FLAG_NOKAP, 0).astype(np.uint8)
  v_f = np.zeros(NL)
  for t in range(len(ts) - 1):
    idx = order[ts[t]:ts[t + 1]]
    v_f[idx] = fill_nan(v[idx])
  arrays = dict(x=x, y=y, hdg=hdg, kap=kap[order], v=v_f[order], flags=fl[order], v_flag=vf[order], npass=npass[order],
                bank=bank[order], grade=grade[order], v_proven=v_proven[order], kap_med=kap_med[order], trace_start=ts, trace_next=nxt)
  carry_flags(arrays, flags, lib, tid)
  if info is not None:
    info['rows'] = int(len(rows))
    if len(rows):                                   # per output point (private diagnostics, never written to the file)
      stp = (excl & EX_STOP) > 0
      key = np.unique(rows[stp, C_PT].astype(np.int64) * 100000 + rows[stp, C_PASS].astype(np.int64))
      nstop = np.bincount((key // 100000).astype(np.int64), minlength=NL)[:NL]
      info['_n_stop_pass'] = nstop[order]
    info['event_points'] = int(np.isin(np.arange(NL), rows[rows[:, C_EVENT] > .5, C_PT].astype(np.int64)).sum()) if len(rows) else 0
    info['excluded_rows'] = int((excl > 0).sum())
    info['excluded_by_reason'] = {nm: int(((excl & b) > 0).sum()) for b, nm in EX_NAMES.items()}
    info['pieces_leaving'] = int(leaving.sum())
    info['pieces_joining'] = int(joining.sum())
    info['lane_events'] = int(sum(len(p.get('lane_i', ())) for p in passes))
    info['takeover_events'] = int(sum(len(p['tk_i']) for p in passes))
    info['limit_events'] = int(sum(len(p['sat_i']) for p in passes))
    info['pieces'] = len(pieces)
  stats = dict(passes=len(passes), library_points=NL, roads=len(lib['base']), roads_kept=n_roads, traces=len(ts) - 1, points=len(x))
  return arrays, stats


# ---- data access (real drives; private) ---------------------------------------------------------------------------------------------
def _p(*a):
  return os.path.join(DATA_DIR, *a)


def seg_key(t):
  return int(t.split('-')[0], 16), int(t.split('-')[1])


def default_tags():
  """gate-checked segments (seg20 ∩ open_segs), chronological -> {tag: rlog path}"""
  have = {os.path.basename(p)[:-4] for p in glob.glob(SEG20 + '/*.npz')}
  out = {}
  for route, seg, path in json.load(open(OPEN_SEGS)):
    tag = f"{route.split('--')[0]}-{seg}"
    if tag in have:
      out[tag] = path
  return dict(sorted(out.items(), key=lambda kv: seg_key(kv[0])))


def select_tags(drives=None, exclude=()):
  tags = default_tags()
  if drives:
    want = set(drives)
    unknown = want - {t.split('-')[0] for t in tags}
    if unknown:
      raise SystemExit(f'drives not in the gate-checked set: {sorted(unknown)}')
    tags = {t: p for t, p in tags.items() if t.split('-')[0] in want}
  ex = set(exclude or ())
  return {t: p for t, p in tags.items() if t.split('-')[0] not in ex}


def read_positions(args):
  """one rlog -> raw liveLocationKalman rows (t, lat, lon, valid, gpsOK, yaw, pitch, altitude), saved to the private cache"""
  tag, path = args
  outp = _p('cache', 'pos', tag + '.npz')
  if pos_cached(tag):
    return tag, 'cached'
  from cereal import log as clog
  with open(path, 'rb') as f:
    evs = list(clog.Event.read_multiple_bytes(f.read()))
  t0 = None
  for e in evs:
    if e.which() == 'carState':
      t0 = e.logMonoTime
      break
  rows = []
  if t0 is not None:
    for e in evs:
      if e.which() == 'liveLocationKalman':
        lk = e.liveLocationKalman
        p = lk.positionGeodetic
        o = lk.calibratedOrientationNED.value
        if len(p.value) < 2:
          continue
        rows.append(((e.logMonoTime - t0) * 1e-9, p.value[0], p.value[1], float(p.valid), float(lk.gpsOK), o[2] if len(o) > 2 else np.nan,
                     o[1] if len(o) > 2 else np.nan, p.value[2] if len(p.value) > 2 else np.nan))
  np.savez_compressed(outp, llk=np.array(rows, float).reshape(-1, LLK_COLS))
  return tag, 'read'


def read_lanes(args):
  """one rlog -> lane/<tag>.npz: t (s from the first carState), d = |lateral offset| at x = 0 of the nearest modelV2 lane line with
  laneLineProbs > LANE_PROB (nan when none). Position-free."""
  tag, path = args
  outp = _p('cache', 'lane', tag + '.npz')
  if os.path.exists(outp):
    return tag, 'cached'
  from cereal import log as clog
  with open(path, 'rb') as f:
    evs = list(clog.Event.read_multiple_bytes(f.read()))
  t0 = None
  for e in evs:
    if e.which() == 'carState':
      t0 = e.logMonoTime
      break
  rows = []
  if t0 is not None:
    for e in evs:
      if e.which() == 'modelV2':
        m = e.modelV2
        probs = list(m.laneLineProbs)
        best = np.nan
        for i, ln in enumerate(m.laneLines):
          if i < len(probs) and probs[i] > LANE_PROB and len(ln.y):
            dd = abs(float(ln.y[0]))
            if not (dd >= best):
              best = dd
        rows.append(((e.logMonoTime - t0) * 1e-9, best))
  np.savez_compressed(outp, lane=np.array(rows, float).reshape(-1, 2))
  return tag, 'read'


def ensure_lanes(tags, workers):
  os.makedirs(_p('cache', 'lane'), exist_ok=True)
  todo = [(t, p) for t, p in tags.items() if not os.path.exists(_p('cache', 'lane', t + '.npz'))]
  if todo:
    t0 = time.time()
    from multiprocessing import Pool
    with Pool(min(workers, 6), initializer=low_priority) as pool:
      for i, _ in enumerate(pool.imap_unordered(read_lanes, todo, chunksize=4)):
        if (i + 1) % 200 == 0:
          print(f'  lanes {i + 1}/{len(todo)}', flush=True)
    print(f'read lane lines from {len(todo)} rlogs in {time.time() - t0:.0f} s', flush=True)


def lane_grid(tag, n):
  """nearest lane-line distance on the segment's 20 Hz grid (last model frame within 0.2 s; nan otherwise or when not cached)"""
  f = _p('cache', 'lane', tag + '.npz')
  out = np.full(n, np.nan)
  if not os.path.exists(f):
    return out
  l = np.load(f)['lane']
  if len(l) < 2:
    return out
  l = l[np.argsort(l[:, 0])]
  tg = np.arange(n) * DT
  k = np.clip(np.searchsorted(l[:, 0], tg, side='right') - 1, 0, len(l) - 1)
  return np.where(np.abs(tg - l[k, 0]) < 0.2, l[k, 1], np.nan)


def pos_cached(tag):
  f = _p('cache', 'pos', tag + '.npz')
  if not os.path.exists(f):
    return False
  with np.load(f) as z:
    return z['llk'].ndim == 2 and z['llk'].shape[1] >= LLK_COLS


def ensure_positions(tags, workers):
  os.makedirs(_p('cache', 'pos'), exist_ok=True)
  todo = [(t, p) for t, p in tags.items() if not pos_cached(t)]
  if todo:
    t0 = time.time()
    from multiprocessing import Pool
    with Pool(min(workers, 6), initializer=low_priority) as pool:
      for i, _ in enumerate(pool.imap_unordered(read_positions, todo, chunksize=4)):
        if (i + 1) % 200 == 0:
          print(f'  read {i + 1}/{len(todo)} rlogs', flush=True)
    print(f'read {len(todo)} rlogs in {time.time() - t0:.0f} s', flush=True)


def origin():
  """the shared local frame, computed once from the FULL default drive set"""
  path = _p('cache', 'origin.json')
  if os.path.exists(path):
    return json.load(open(path))
  tags = default_tags()
  ensure_positions(tags, 6)
  ll = []
  for t in tags:
    l = np.load(_p('cache', 'pos', t + '.npz'))['llk']
    ok = (l[:, 3] > .5) & (l[:, 4] > .5) & (np.abs(l[:, 1]) > 1) if len(l) else np.zeros(0, bool)
    ll.append(l[ok, 1:3])
  ll = np.concatenate(ll)
  lat0, lon0 = float(np.median(ll[:, 0])), float(np.median(ll[:, 1]))
  o = dict(lat0=lat0, lon0=lon0, kx=111320.0 * math.cos(math.radians(lat0)), ky=110574.0)
  json.dump(o, open(path, 'w'))
  return o


def seg_grid(tag, n):
  """positions of one segment on its 20 Hz grid: lat, lon (nan where not ok), pos_ok, bearing_deg, pitch (rad), altitude (m)"""
  l = np.load(_p('cache', 'pos', tag + '.npz'))['llk']
  tg = np.arange(n) * DT
  lat = np.full(n, np.nan); lon = np.full(n, np.nan); pok = np.zeros(n, bool); brg = np.full(n, np.nan)
  pit = np.full(n, np.nan); alt = np.full(n, np.nan)
  if len(l) < 2:
    return lat, lon, pok, brg, pit, alt
  l = l[np.argsort(l[:, 0])]
  ok = (l[:, 3] > .5) & (l[:, 4] > .5) & (np.abs(l[:, 1]) > 1)
  j = np.clip(np.searchsorted(l[:, 0], tg), 1, len(l) - 1)
  nn = np.where(np.abs(l[j - 1, 0] - tg) < np.abs(l[j, 0] - tg), j - 1, j)
  pok = ok[nn] & (np.abs(l[nn, 0] - tg) < 0.2)
  if ok.sum() >= 2:
    lat = np.where(pok, np.interp(tg, l[ok, 0], l[ok, 1]), np.nan)
    lon = np.where(pok, np.interp(tg, l[ok, 0], l[ok, 2]), np.nan)
  else:
    pok[:] = False
  yo = np.isfinite(l[:, 5])
  if yo.sum() >= 2:
    yaw = np.interp(tg, l[yo, 0], np.unwrap(l[yo, 5]))
    brg = np.where(np.abs(l[nn, 0] - tg) < 0.2, np.degrees(yaw) % 360.0, np.nan)
  near = np.abs(l[nn, 0] - tg) < 0.2
  po = np.isfinite(l[:, 6])
  if po.sum() >= 2:
    pit = np.where(near & po[nn], np.interp(tg, l[po, 0], l[po, 6]), np.nan)
  if ok.sum() >= 2:
    ao = ok & np.isfinite(l[:, 7])
    if ao.sum() >= 2:
      alt = np.where(pok, np.interp(tg, l[ao, 0], l[ao, 7]), np.nan)
  return lat, lon, pok, brg, pit, alt


def lowpass_nan(a, sigma_samples):
  """Gaussian low-pass that ignores nan (normalised convolution); nan stays nan"""
  from scipy.ndimage import gaussian_filter1d
  a = np.asarray(a, float)
  ok = np.isfinite(a)
  if not ok.any():
    return a.copy()
  num = gaussian_filter1d(np.where(ok, a, 0.0), sigma_samples, mode='nearest')
  den = gaussian_filter1d(ok.astype(float), sigma_samples, mode='nearest')
  return np.where(ok & (den > 0.2), num / np.maximum(den, 1e-9), np.nan)


def runs_of(tags):
  out = []
  for t in tags:
    d, s = t.split('-')
    s = int(s)
    if out and out[-1][0] == d and out[-1][2] + 1 == s:
      out[-1][1].append(t)
      out[-1][2] = s
    else:
      out.append([d, [t], s])
  return [(d, ts) for d, ts, _ in out]


def prepare_runs(tags, org, replay=False):
  """runs of consecutive segments -> pass dicts (positions in local metres; private, in memory or the private cache)"""
  sys.path.insert(0, STUDY + '/scripts')
  import rr_lib as R
  out = []
  for d, ts in runs_of(list(tags)):
    zs = [load_seg(t) for t in ts]
    ders = [R.derived(z) for z in zs]
    n = [len(z['t']) for z in zs]
    off = np.r_[0, np.cumsum(n)].astype(np.int64)
    cat = lambda f: np.concatenate([np.asarray(f(z, q)) for z, q in zip(zs, ders)])
    v = cat(lambda z, q: q['v'])
    grids = [seg_grid(t, k) for t, k in zip(ts, n)]
    lat = np.concatenate([g[0] for g in grids]); lon = np.concatenate([g[1] for g in grids])
    p = dict(drive=d, tags=list(ts), off=off, v=v.astype(np.float32), kap=cat(lambda z, q: q['kap']).astype(np.float32),
             bank=cat(lambda z, q: q['bank']).astype(np.float32))
    p['s'] = np.r_[0, np.cumsum(v[:-1].astype(float) * DT)]
    p['grade'] = lowpass_nan(np.concatenate([g[4] for g in grids]), GRADE_SIGMA_S / DT).astype(np.float32)
    p['alt'] = np.concatenate([g[5] for g in grids]).astype(np.float32)
    p['X'] = (lon - org['lon0']) * org['kx']
    p['Y'] = (lat - org['lat0']) * org['ky']
    sig = {nm: cat(lambda z, q, f=f: z[f]).astype(np.float32) for nm, f in (('en', 'enabled'), ('lat_act', 'lat_act'), ('ti', 'ti_cmd'),
                                                                           ('pressed', 'pressed'), ('drv', 'drv_trq'))}
    brk = cat(lambda z, q: z['brk']).astype(np.float32)
    p['sat_i'], p['tk_i'] = run_events(sig['ti'], sig['lat_act'], sig['pressed'], sig['en'], sig['drv'], brk=brk, v=v, s=p['s'])
    p['blink'] = cat(lambda z, q: z['blink']).astype(np.float32)
    for nm in ('pressed', 'drv', 'lat_act'):
      p[nm] = sig[nm]
    if all('aego' in z for z in zs):
      p['aego'] = cat(lambda z, q: z['aego']).astype(np.float32)
    p['lane_d'] = np.concatenate([lane_grid(t, k) for t, k in zip(ts, n)]).astype(np.float32)
    p['lane_i'] = lane_events(p['lane_d'], p['lat_act'], p['blink'])
    if replay:
      p.update(sig)
      p['lat'], p['lon'] = lat, lon
      p['pos_ok'] = np.concatenate([g[2] for g in grids])
      p['bearing_deg'] = np.concatenate([g[3] for g in grids])
      for nm, f in (('cr_en', 'cr_en'), ('cr_set', 'cr_set'), ('lead', 'lead'), ('brk', 'brk'), ('gas', 'gas'), ('blink', 'blink')):
        p[nm] = cat(lambda z, q, f=f: z[f]).astype(np.float32)
      la5 = []
      for z in zs:
        m = z['pred_t'] <= 5.0 + 1e-6
        la5.append(np.nanmax(np.abs(z['pred_la'][:, m]), axis=1) if m.any() else np.full(len(z['t']), np.nan))
      p['model_la5'] = np.concatenate(la5).astype(np.float32)
    out.append(p)
  return out


def load_seg(tag):
  """20 Hz signals of one segment: the seg20 extraction when it has the segment, else the private rlog cache"""
  f = os.path.join(SEG20, tag + '.npz')
  if not os.path.exists(f):
    f = _p('cache', 'rl20', tag + '.npz')
  with np.load(f, allow_pickle=False) as d:
    return {k: d[k] for k in d.files}


def reserved_segments():
  """{(route counter, segment)} that release records reserve for a benchmark (segments_reserved); never opened here"""
  out = set()
  for f in glob.glob(os.path.join(RELEASES, '*.json')):
    try:
      d = json.load(open(f))
    except Exception:
      continue
    routes = d.get('routes')
    if isinstance(routes, dict):
      for r, v in routes.items():
        for sg in ((v.get('segments_reserved') or {}) if isinstance(v, dict) else {}):
          out.add((r[:8], int(sg)))
  if not out:
    raise SystemExit('no release records found: refusing to read held-out drives without the reservation list')
  return out


def recorded_tags():
  """every recorded segment on this PC -> rlog path (the largest copy when there are several), minus reserved / kept-back ones.
  -> (tags, counts)"""
  import re
  pat = re.compile(r'^([0-9a-f]{8})--([0-9a-f]{10})--(\d+)$')
  skip = {'.git', 'node_modules', '__pycache__', 'site-packages', 'seg20', 'pulses', 'cache', 'loo'}
  found = collections.defaultdict(list)
  for root in (RLOG_ROOT, _p('logs')):
    if not os.path.isdir(root):
      continue
    for dp, dn, fn in os.walk(root, followlinks=True):
      dn[:] = [d for d in dn if d not in skip and not d.startswith('.venv')]
      m = pat.match(os.path.basename(dp))
      if m:
        if 'rlog' in fn:
          found[(m.group(1), int(m.group(3)))].append(os.path.join(dp, 'rlog'))
        dn[:] = []
  res = reserved_segments()
  seg20 = default_tags()
  logs = os.path.abspath(_p('logs')).lower()
  out, cnt = {}, collections.Counter()
  for (r, sg), paths in found.items():
    tag = f'{r}-{sg}'
    if (r, sg) in res or (r in KEPT_BACK and KEPT_BACK[r](sg)):
      cnt['reserved_or_kept_back_skipped'] += 1
      continue
    if tag in seg20:
      out[tag] = seg20[tag]
      cnt['gate_set'] += 1
      continue
    out[tag] = max(paths, key=os.path.getsize)
    cnt['held_out_0x332_up' if int(r, 16) >= 0x332 else 'below_0x332_not_in_gate_set'] += 1
    if os.path.abspath(out[tag]).lower().startswith(logs):
      cnt['collector_copied'] += 1
  return dict(sorted(out.items(), key=lambda kv: seg_key(kv[0]))), cnt


def read_rlog_seg(args):
  """one rlog -> private caches: rl20/<tag>.npz (20 Hz signals as x1_extract.py) and pos/<tag>.npz (liveLocationKalman rows)"""
  tag, path = args
  f20, fpos = _p('cache', 'rl20', tag + '.npz'), _p('cache', 'pos', tag + '.npz')
  if os.path.exists(os.path.join(SEG20, tag + '.npz')) or (os.path.exists(f20) and os.path.exists(fpos)):
    return tag, 'cached'
  try:
    from cereal import log as clog
    with open(path, 'rb') as fh:
      evs = list(clog.Event.read_multiple_bytes(fh.read()))
    cs, co, cc, lp, rd, llk, f75 = [], [], [], [], [], [], ([], [])
    for e in evs:
      w = e.which()
      t = e.logMonoTime
      if w == 'carState':
        c = e.carState
        cs.append((t, c.vEgo, c.steeringPressed, c.steeringTorque, c.brakePressed, c.gasPressed, c.cruiseState.enabled, c.cruiseState.speed,
                   c.leftBlinker or c.rightBlinker))
      elif w == 'carOutput':
        co.append((t, e.carOutput.actuatorsOutput.steerOutputCan))
      elif w == 'carControl':
        cc.append((t, e.carControl.latActive, e.carControl.enabled))
      elif w == 'livePose':
        lp.append((t, e.livePose.angularVelocityDevice.z))
      elif w == 'radarState':
        rd.append((t, e.radarState.leadOne.status))
      elif w == 'liveLocationKalman':
        lk = e.liveLocationKalman
        pg = lk.positionGeodetic
        o = lk.calibratedOrientationNED.value
        if len(pg.value) >= 2:
          llk.append((t, pg.value[0], pg.value[1], float(pg.valid), float(lk.gpsOK), o[2] if len(o) > 2 else np.nan,
                      o[1] if len(o) > 2 else np.nan, pg.value[2] if len(pg.value) > 2 else np.nan))
      elif w == 'can':
        for fr in e.can:
          if fr.src == 0 and fr.address == 0x75 and len(fr.dat) >= 8:
            f75[0].append(t)
            f75[1].append(bytes(fr.dat)[:8])
    if len(cs) < 500:
      return tag, 'short'
    cs = np.array(cs, float)
    t0 = cs[0, 0]
    g = np.arange(0, (cs[-1, 0] - t0) * 1e-9, DT)
    rel = lambda a: (a[:, 0] - t0) * 1e-9
    A = lambda x: np.array(x, float) if len(x) else None

    def H(a, i):
      if a is None or len(a) == 0:
        return np.full(len(g), np.nan)
      k = np.clip(np.searchsorted(rel(a), g, side='right') - 1, 0, len(a) - 1)
      return a[k, i]
    co, cc, lp, rd = A(co), A(cc), A(lp), A(rd)
    d = dict(t=g, v=np.interp(g, rel(cs), cs[:, 1]), pressed=H(cs, 2), drv_trq=np.interp(g, rel(cs), cs[:, 3]), brk=H(cs, 4), gas=H(cs, 5),
             cr_en=H(cs, 6), cr_set=H(cs, 7), blink=H(cs, 8), ti_cmd=H(co, 1), lat_act=H(cc, 1), enabled=H(cc, 2), lead=H(rd, 1),
             gz=np.interp(g, rel(lp), lp[:, 1]) if lp is not None and len(lp) > 10 else np.full(len(g), np.nan))
    if f75[0]:
      tt = (np.array(f75[0], np.int64) - t0) * 1e-9
      dat = np.frombuffer(b''.join(f75[1]), np.uint8).reshape(-1, 8).astype(np.int64)
      k = np.clip(np.searchsorted(tt, g, side='right') - 1, 0, len(tt) - 1)
      d['ay_raw'] = (((dat[:, 2] & 0x0F) << 8) | dat[:, 3])[k].astype(np.float32)
      d['yaw_raw'] = ((dat[:, 4] << 8) | dat[:, 5])[k].astype(np.float32)
    with open(f20, 'wb') as fh:
      np.savez_compressed(fh, **{k: np.asarray(v, np.float32) for k, v in d.items()})
    llk = np.array(llk, float).reshape(-1, LLK_COLS)
    llk[:, 0] = (llk[:, 0] - t0) * 1e-9
    with open(fpos, 'wb') as fh:
      np.savez_compressed(fh, llk=llk)
    return tag, 'read'
  except Exception as ex:
    return tag, 'error ' + type(ex).__name__


def ensure_rlog_segs(tags, workers):
  """read the non-seg20 segments once (private caches); -> the tags that have 20 Hz data"""
  os.makedirs(_p('cache', 'rl20'), exist_ok=True)
  os.makedirs(_p('cache', 'pos'), exist_ok=True)
  have20 = lambda t: os.path.exists(os.path.join(SEG20, t + '.npz'))
  todo = [(t, p) for t, p in tags.items() if not have20(t)
          and not (os.path.exists(_p('cache', 'rl20', t + '.npz')) and os.path.exists(_p('cache', 'pos', t + '.npz')))]
  c = collections.Counter()
  if todo:
    t0 = time.time()
    from multiprocessing import Pool
    with Pool(min(workers, 6), initializer=low_priority) as pool:
      for i, (_, st) in enumerate(pool.imap_unordered(read_rlog_seg, todo, chunksize=2)):
        c[st.split()[0]] += 1
        if (i + 1) % 100 == 0:
          print(f'  rlogs {i + 1}/{len(todo)} {dict(c)} {time.time() - t0:.0f} s', flush=True)
    print(f'read {len(todo)} rlogs in {time.time() - t0:.0f} s: {dict(c)}', flush=True)
  return {t: p for t, p in tags.items() if have20(t) or (os.path.exists(_p('cache', 'rl20', t + '.npz'))
                                                         and os.path.exists(_p('cache', 'pos', t + '.npz')))}


def load_passes(tags, workers):
  """prepared passes of the full default set, cached privately; filtered to `tags`"""
  path = _p('cache', 'passes.pkl')
  allt = default_tags()
  if os.path.exists(path):
    P = pickle.load(open(path, 'rb'))
  else:
    ensure_positions(allt, workers)
    ensure_lanes(allt, workers)
    org = origin()
    t0 = time.time()
    P = prepare_runs(allt, org)
    pickle.dump(P, open(path, 'wb'), protocol=4)
    print(f'prepared {len(P)} runs in {time.time() - t0:.0f} s', flush=True)
  keep = set(tags)
  return [p for p in P if all(t in keep for t in p['tags'])] if keep != set(allt) else P


def write_memory(passes, path, meta_extra):
  org = origin()
  info = {}
  arrays, stats = build(passes, info)
  stats.update({k: v for k, v in info.items() if not k.startswith('_')})
  meta = dict(version=M.VERSION, spacing_m=SP, lat0=org['lat0'], lon0=org['lon0'], kx=org['kx'], ky=org['ky'],
              built=time.strftime('%Y-%m-%d %H:%M'), builder='tools/curve_slowing/build_memory.py', **meta_extra,
              params=dict(tol_m=TOL_M, hdg_tol_deg=math.degrees(HDG_TOL), min_trace_m=MIN_TRACE_M, limit_before_m=LIMIT_BEFORE_M,
                          limit_after_m=LIMIT_AFTER_M, takeover_m=TK_M, link_dist_m=LINK_DIST_M, link_hdg_deg=math.degrees(LINK_HDG),
                          kap_sigma_m=KAP_SIGMA_M, tk_drv_counts=TK_DRV, tk_sustain_s=TK_SUSTAIN * DT, tk_follow_s=TK_FOLLOW * DT),
              kap_rule='second<=', stats=stats)
  os.makedirs(os.path.dirname(path), exist_ok=True)
  M.save(path, arrays, meta)
  m = M.load(path)                   # validate
  return m, stats


def _loo_one(drive):
  low_priority()
  P = _LOO_PASSES
  drives = list(collections.OrderedDict.fromkeys(p['drive'] for p in P))
  path = _p('loo', f'{drive}.npz')
  m, st = write_memory([p for p in P if p['drive'] != drive], path,
                       dict(drives=[d for d in drives if d != drive], excluded=[drive], loo_of=drive))
  return drive, m.n, m.n_traces


_LOO_PASSES = None


def _loo_init(tags):
  global _LOO_PASSES
  low_priority()
  _LOO_PASSES = load_passes(tags, 1)


# ---- replay caches ------------------------------------------------------------------------------------------------------------------
def write_replay_caches(tags, workers):
  ensure_positions(tags, workers)
  org = origin()
  corners = pickle.load(open(STUDY + '/cache/corners.pkl', 'rb'))
  RU = pickle.load(open(STUDY + '/cache/runs.pkl', 'rb'))['RUNS']
  study_off = {}
  for run in RU.values():
    for i, t in enumerate(run['tags']):
      study_off[t] = int(run['off'][i])
  del RU
  by_drive = collections.OrderedDict()
  for t in tags:
    by_drive.setdefault(t.split('-')[0], []).append(t)
  F = ('v', 'kap', 'cr_en', 'cr_set', 'lead', 'en', 'lat_act', 'brk', 'gas', 'pressed', 'blink', 'ti', 'model_la5', 'lat', 'lon', 'pos_ok',
       'bearing_deg', 'grade')
  for d, ts in by_drive.items():
    runs = prepare_runs(ts, org, replay=True)
    tag_pos = {}
    for r, p in enumerate(runs):
      for i, t in enumerate(p['tags']):
        tag_pos[t] = (r, int(p['off'][i]))
    out = {f: np.concatenate([p[f] for p in runs]) for f in F}
    out['lat'] = out['lat'].astype(np.float64); out['lon'] = out['lon'].astype(np.float64); out['pos_ok'] = out['pos_ok'].astype(bool)
    out['t'] = np.concatenate([np.arange(len(p['v'])) * DT for p in runs])
    run_off = np.r_[0, np.cumsum([len(p['v']) for p in runs])].astype(np.int64)
    out['run_off'] = run_off
    out['seg_tags'] = np.array([t for p in runs for t in p['tags']])
    out['seg_off'] = np.array([run_off[r] + p['off'][i] for r, p in enumerate(runs) for i in range(len(p['tags']))], np.int64)
    out['seg_run'] = np.array([r for r, p in enumerate(runs) for _ in p['tags']], np.int64)
    cs = [c for c in corners if c['tag'] in tag_pos]
    rows = []
    for c in cs:
      r, o = tag_pos[c['tag']]
      rows.append((c['corner_id'], r, o + (c['i20'] - study_off[c['tag']]), c['sat'], c['cruise'], c['v_mph'], c['la'], c['kap'], c['direction'],
                   c['nprev']))
    cols = list(zip(*rows)) if rows else [[]] * 10
    for name, col, dt in zip(('c_id', 'c_run', 'c_i20', 'c_sat', 'c_cruise', 'c_v_mph', 'c_la', 'c_kap', 'c_dir', 'c_nprev'), cols,
                             ('U8', np.int64, np.int64, np.int8, np.int8, np.float32, np.float32, np.float32, 'U5', np.int32)):
      out[name] = np.array(col, dtype=dt)
    with open(_p('cache', f'drive_{d}.npz'), 'wb') as f:
      np.savez_compressed(f, **out)
  print(f'replay caches: {len(by_drive)} drives', flush=True)


# ---- position-free validation -------------------------------------------------------------------------------------------------------
def _coverage(m, x, y, h, dist=15.0, hdg=math.radians(30)):
  """fraction-ready bool per sample: within dist and hdg of some memory point"""
  from scipy.spatial import cKDTree
  tree = cKDTree(np.c_[m.x, m.y])
  out = np.zeros(len(x), bool)
  if not len(x):
    return out
  dd, jj = tree.query(np.c_[x, y], k=16, distance_upper_bound=dist)
  ok = np.isfinite(dd) & (jj < m.n)
  jc = np.clip(jj, 0, m.n - 1)
  ok &= np.abs(wrap(h[:, None] - m.hdg[jc])) <= hdg
  return ok.any(1)


def check(tags):
  full = M.load(_p('curve_memory_v1.npz'))
  size = os.path.getsize(_p('curve_memory_v1.npz')) / 1024
  print(f'full memory: traces {full.n_traces}, points {full.n}, length {full.n * SP / 1609.344:.1f} mi, file {size:.0f} KB')
  print(f'  FLAG_LIMIT points {int(((full.flags & M.FLAG_LIMIT) > 0).sum())}, FLAG_TAKEOVER points {int(((full.flags & M.FLAG_TAKEOVER) > 0).sum())}')
  print(f'  trace_next links {int((full.trace_next >= 0).sum())} of {full.n_traces}')
  bins = [1, 2, 3, 5, 10, 20, 256]
  h, _ = np.histogram(full.npass, bins=bins)
  print('  npass distribution: ' + ', '.join(f'{a}-{b - 1}: {c}' for a, b, c in zip(bins[:-1], bins[1:], h)))
  loo = sorted(glob.glob(_p('loo', '*.npz')))
  if loo:
    ns = [M.load(f).n for f in loo]
    print(f'LOO files {len(loo)}, points min/median/max {min(ns)}/{int(np.median(ns))}/{max(ns)}')
  drives = list(collections.OrderedDict.fromkeys(t.split('-')[0] for t in tags))
  pick = [drives[int(round(i))] for i in np.linspace(len(drives) * 0.15, len(drives) * 0.85, 3)]
  for d in pick:
    c = _p('cache', f'drive_{d}.npz')
    lp = _p('loo', f'{d}.npz')
    if not (os.path.exists(c) and os.path.exists(lp)):
      continue
    z = np.load(c)
    sel = np.where(z['pos_ok'] & (z['v'] > 5) & np.isfinite(z['bearing_deg']))[0][::20]
    x, y = full.to_local(z['lat'][sel], z['lon'][sel])
    hh = np.pi / 2 - np.radians(z['bearing_deg'][sel])
    a = _coverage(full, x, y, hh).mean()
    lm = M.load(lp)
    b = _coverage(lm, x, y, hh).mean()
    print(f'drive {d}: {len(sel)} samples at 1 Hz; within 15 m / 30 deg of memory: full {a:.3f}, leave-one-out {b:.3f}')
  # the 100 saturating-under-cruise corners against their drive's LOO memory
  n = known = 0
  for d in drives:
    c = _p('cache', f'drive_{d}.npz')
    lp = _p('loo', f'{d}.npz')
    if not (os.path.exists(c) and os.path.exists(lp)):
      continue
    z = np.load(c)
    sel = (z['c_sat'] == 1) & (z['c_cruise'] == 1)
    if not sel.any():
      continue
    i = z['run_off'][z['c_run'][sel]] + z['c_i20'][sel]
    ok = z['pos_ok'][i] & np.isfinite(z['bearing_deg'][i])
    lm = M.load(lp)
    x, y = lm.to_local(z['lat'][i], z['lon'][i])
    cov = _coverage(lm, np.nan_to_num(x), np.nan_to_num(y), np.pi / 2 - np.radians(np.nan_to_num(z['bearing_deg'][i]))) & ok
    n += int(sel.sum())
    known += int(cov.sum())
  print(f'saturating cruise corners: {n}; on road known to their leave-one-out memory: {known} (study, chronological: 58)')


def main():
  ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
  ap.add_argument('--drives', nargs='*', help='drive ids (route counter hex); default: every drive of the gate-checked set')
  ap.add_argument('--exclude', nargs='*', default=[], help='drive ids to leave out')
  ap.add_argument('--out', default=None, help='memory file (default DATA_DIR/curve_memory_v1.npz)')
  ap.add_argument('--workers', type=int, default=6)
  ap.add_argument('--loo-all', action='store_true', help='write DATA_DIR/loo/<drive>.npz for every drive of the set')
  ap.add_argument('--replay-caches', action='store_true', help='write DATA_DIR/cache/drive_<drive>.npz')
  ap.add_argument('--check', action='store_true', help='position-free validation printout')
  ap.add_argument('--source', choices=('gate', 'all'), default='gate',
                  help="gate: the seg20 gate-checked set (default; LOO and replay); all: every recorded drive on this PC (on-car memory)")
  ap.add_argument('--releases', default=None, help='folder of the simulator release records (default: tools/mazda_sim/releases)')
  a = ap.parse_args()
  if a.releases:
    global RELEASES
    RELEASES = os.environ['CURVE_RELEASES'] = a.releases
  low_priority()
  os.makedirs(_p('cache'), exist_ok=True)
  workers = max(1, min(a.workers, 6))
  tags = select_tags(a.drives, a.exclude)
  drives = list(collections.OrderedDict.fromkeys(t.split('-')[0] for t in tags))
  print(f'drive set: {len(drives)} drives, {len(tags)} segments', flush=True)
  if a.check:
    check(tags)
    return
  if a.replay_caches:
    write_replay_caches(tags, workers)
    return
  if a.loo_all:
    load_passes(tags, workers)              # make the shared cache before the workers start
    os.makedirs(_p('loo'), exist_ok=True)
    from multiprocessing import Pool
    t0 = time.time()
    with Pool(workers, initializer=_loo_init, initargs=(list(tags),)) as pool:
      for i, (d, n, nt) in enumerate(pool.imap_unordered(_loo_one, drives)):
        print(f'  loo {i + 1}/{len(drives)}: {n} points, {nt} traces', flush=True)
    print(f'LOO files: {len(drives)} in {time.time() - t0:.0f} s')
    return
  t0 = time.time()
  if a.source == 'all':
    if a.loo_all or a.replay_caches or a.check:
      raise SystemExit('--source all builds the on-car memory only (LOO, replay and the check stay on the gate set)')
    rt, cnt = recorded_tags()
    if a.drives:
      rt = {t: p for t, p in rt.items() if t.split('-')[0] in set(a.drives)}
    rt = {t: p for t, p in rt.items() if t.split('-')[0] not in set(a.exclude)}
    gate = default_tags()
    ensure_positions({t: p for t, p in rt.items() if t in gate}, workers)
    rt = ensure_rlog_segs(rt, workers)
    ensure_lanes(rt, workers)
    passes = prepare_runs(rt, origin())
    drives = list(collections.OrderedDict.fromkeys(t.split('-')[0] for t in rt))
    cnt = dict(cnt, segments_used=len(rt), drives_used=len(drives), gate_drives_used=len({t.split('-')[0] for t in rt if t in gate}))
    print('all-drives source (counts only):', cnt, flush=True)
    out = os.path.abspath(a.out or _p('curve_memory_v1.npz'))
    if not out.lower().startswith(os.path.abspath(DATA_DIR).lower() + os.sep):
      raise SystemExit('the memory holds positions: --out must be under ' + DATA_DIR)
    m, st = write_memory(passes, out, dict(drives=drives, excluded=sorted(a.exclude), source='all recorded drives (owner 2026-11-23)',
                                           source_counts=cnt))
    print(f'memory: {m.n_traces} traces, {m.n} points, {os.path.getsize(out) / 1024:.0f} KB, {time.time() - t0:.0f} s; {st}')
    return
  passes = load_passes(tags, workers)
  out = os.path.abspath(a.out or _p('curve_memory_v1.npz'))
  if not out.lower().startswith(os.path.abspath(DATA_DIR).lower() + os.sep):
    raise SystemExit('the memory holds positions: --out must be under ' + DATA_DIR)
  m, st = write_memory(passes, out, dict(drives=drives, excluded=sorted(a.exclude)))
  print(f'memory: {m.n_traces} traces, {m.n} points, {os.path.getsize(out) / 1024:.0f} KB, {time.time() - t0:.0f} s; {st}')


if __name__ == '__main__':
  main()
