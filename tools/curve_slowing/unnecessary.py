"""Curve slowing replay against ground truth (plan T9, amendments A5 / A6, rulings R10-R12; off the car, open loop).

The planner (ti_curve_plan.Planner, 5 Hz; all three stages) runs on the recorded drives: the curve-governor's position-free runs
(ground_truth.RUNS_DIR; made after the governor's segment filter: labelled, corpus-excluded and reserved segments never opened). The
planner's fix is the road-memory skeleton point the governor placed each sample on (`pt`: the memory's own x / y / heading; no
recorded coordinate is read here), its speed, set speed, yaw curvature, blinker and driving-model preview are the recorded ones. No
map list and no speed limit are recorded: stage 1 runs from the memory only; the floor is the set speed - 15 (no limit). Stage 3
(ti_curve_monitor, 100 Hz on the car; here 5 monitor frames per 20 Hz sample) runs where the governor's steering cache has the sent
command, steering rate, interceptor state and the stock lane-keep flags (cache/steer/<tag>.npz); the controller's desired lateral
acceleration is not recorded (None: the monitor uses the forecast and the measured lateral acceleration).

Every slowdown the replay STARTS (stage 1: "start"; stage 2: "s2_start"; stage 3: an alert onset, whose gentle taps go to its need)
is matched to the recorded bend pass it is for (ground_truth.passes_of_run; the pass overlapping the planned bend). Against ground
truth (trouble = car-steered pass, >= 2 s at the limit, firm takeover or a lane-line excursion): precision / recall (slowdowns of
5 mph+, and alerts), unnecessary slowdowns of 3 mph+ per engaged hour (on a clean car-steered pass, or where no bend loads the car
at all while engaged), and SPEED LOST (A6): the predicted speed (early_dwell.Profile: the ONE measured cruise model, the planner's
push and model-timed raise) below the start speed from the start to the bend's end, plus the way back up at ACCEL_MPH_S, in
mph-seconds per engaged hour; and the largest drop on a clean pass. Early dwell (R11: arrived at T + the planner's ARRIVED_MPH)
from the same profile to where the planned bend first loads the car at T (searched from the planned bend's approach: plan A5).
Evidence rules: car-steered passes only; headline figures on standard builds (the governor's per-pass provenance), all builds
alongside. Output is POSITION-FREE: route, segment, seconds, speeds, counts.

  python tools/curve_slowing/unnecessary.py [--mem FILE] [--drives D ...] [--workers 3] [--confirm-share 0.6] [--cap-scale 1.0]
                                            [--min-drop 2] [--min-passes 1] [--unconfirmed-cap 5|none] [--planner FILE]
                                            [--out JSON] [--list]
"""
import argparse
import importlib.util
import inspect
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

try:
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P, ti_curve_demand as CD, ti_curve_monitor as TM
except ImportError:
  sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
  from selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P, ti_curve_demand as CD, ti_curve_monitor as TM  # type: ignore
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import early_dwell as ED   # noqa: E402
import ground_truth as G   # noqa: E402

MEM_DIR = 'E:/openpilot-mazda-ti-data/curve-slowing/memory'
OUT_DIR = 'E:/openpilot-mazda-ti-data/curve-stages/replay'      # private outputs (and must_catch.json): never in the repository
STEER_DIR = os.path.join(G.GOV_DIR, 'cache', 'steer')
MPH = 0.44704
DT = G.DT
TICK = 4                        # samples per planner tick (5 Hz)
MON_SUB = 5                     # monitor frames per 20 Hz sample (100 Hz)
TI_MAX = 600.0
MATCH_PRE_M = 60.0              # a slowdown is for the pass overlapping [planned entry - this, planned end + MATCH_POST_M]
MATCH_POST_M = 30.0
S2_WIN_M = 60.0                 # stage 2: +- this around the forecast's bend
S3_MATCH_S = 2.0                # stage 3: the pass the car is in (or within this of)
BIG_MPH, UNNEC_MPH = 5, 3
ACCEL_MPH_S = 0.7               # the stock cruise speeding back up (ti_cruise_model.ACCEL_MPH_S)
LOAD_PRE_S = 3.0                # the planned bend's load is searched from this long (at the start speed) before its planned entry

# corrected bank (road tilt) from the builder's cached one: the governor's common.bank_fixed (reader fix: sens = (ay_raw - 2000 +
# 14.31) / 93.857, left +); the measured lateral acceleration la_now = v^2 kap + bank (what the accelerometer reads)
_AY_SCALE_OLD, _AY_OFF_OLD = 0.010394, -20.598
_ACC_ZERO, _ACC_K, _ACC_RAW0 = -14.311256405649797, 93.85730392231883, 2000.0
_ALPHA = -1.0 / (_AY_SCALE_OLD * _ACC_K)
_BETA = (-_AY_OFF_OLD / _AY_SCALE_OLD - _ACC_RAW0 - _ACC_ZERO) / _ACC_K


def bank_fixed(bank_old, kap, v):
  from scipy.ndimage import gaussian_filter1d
  la_s = np.asarray(kap, float) * np.maximum(np.asarray(v, float), 3.0) ** 2
  g1 = gaussian_filter1d(la_s, 1.0 / DT, mode='nearest')
  return _ALPHA * np.asarray(bank_old, float) + (_ALPHA - 1.0) * g1 + _BETA


_MEM = {}


def _mem(path):
  if path not in _MEM:
    _MEM[path] = M.load(path)
  return _MEM[path]


def planner_module(path=None):
  """The planner: this checkout's ti_curve_plan, or (A5) an older one saved as a file (e.g. the build on the car, 6213aea)."""
  if not path:
    return P
  spec = importlib.util.spec_from_file_location('plan_replayed', path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


_UNSET = object()


def apply_options(Pm, confirm_share=None, cap_scale=1.0, min_drop=None, min_passes=None, unconfirmed_cap=_UNSET):
  """The sweep's settings (in this worker process only): CONFIRM_SHARE (stage 1's release), a scale on capacity (every need and
  release judgement), MIN_DROP_MPH, S1_MIN_PASSES (R15's mechanism), S1_UNCONFIRMED_MAX_MPH (R16's switch; None = no cap)."""
  if confirm_share is not None and hasattr(Pm, 'CONFIRM_SHARE'):
    Pm.CONFIRM_SHARE = float(confirm_share)
  if min_passes is not None and hasattr(Pm, 'S1_MIN_PASSES'):
    Pm.S1_MIN_PASSES = int(min_passes)
  if unconfirmed_cap is not _UNSET and hasattr(Pm, 'S1_UNCONFIRMED_MAX_MPH'):
    Pm.S1_UNCONFIRMED_MAX_MPH = None if unconfirmed_cap is None else int(unconfirmed_cap)
  if min_drop is not None:
    Pm.MIN_DROP_MPH = int(min_drop)
  if cap_scale and abs(cap_scale - 1.0) > 1e-9 and Pm is P:
    base = getattr(CD, '_capacity_base', CD.capacity)
    CD._capacity_base = base
    CD.capacity = lambda mph, side, banked=False, hold_step=False, _b=base, _f=float(cap_scale): _f * _b(mph, side, banked, hold_step)
    CD._KMAX = {k: CD._kmax_table(*k) for k in CD._KMAX}
    if hasattr(CD, '_KMAX_HOLD'):
      CD._KMAX_HOLD = {k: CD._kmax_table(*k, True) for k in CD._KMAX_HOLD}


# ---- raw position fixes (road matching, 2026-11-24) ----------------------------------------------------------------------------------
# The replay's own fix is the memory point the governor snapped each sample onto (`pt`): it always lies ON a remembered trace, so a
# rule that judges the car's offset from the trace (road matching, ti_curve_plan RM_*) cannot see anything with it. --raw-fixes
# feeds the planner the RAW position instead (the llk position and heading, as ti_curve_plannerd feeds the car), read ONLY through
# the governor's admitted segment list (common.allowed_tags) and the builder's own position reader, exactly as the road-matching
# investigation's measure.py does. PRIVATE: held in memory for one drive, never printed, logged or written.
GOV_CODE = os.path.join(G.GOV_DIR, 'wt', 'tools', 'curve_slowing', 'governor')
_ALLOWED = {}


def raw_positions(drive):
  """The drive's runs (in G.load_runs order) as dicts of local X / Y (the memory's frame), hdg (rad, east = 0, ccw) and ok, from the
  governor's admitted segments only. In memory only."""
  if GOV_CODE not in sys.path:
    sys.path.insert(0, GOV_CODE)
  import common as C   # the governor's
  B = C.builder()
  if 'keep' not in _ALLOWED:
    _ALLOWED['keep'] = C.allowed_tags(B)[0]
  ts = {t: p for t, p in _ALLOWED['keep'].items() if t.split('-')[0] == drive}
  # only tags whose 20 Hz data are already cached (nothing new is read)
  have = lambda t: os.path.exists(os.path.join(B.SEG20, t + '.npz')) or (
    os.path.exists(B._p('cache', 'rl20', t + '.npz')) and os.path.exists(B._p('cache', 'pos', t + '.npz')))
  ts = {t: p for t, p in ts.items() if have(t)}
  org = B.origin()
  out = []
  for _, tl in B.runs_of(list(ts)):
    grids = []
    for t in tl:
      f = os.path.join(B.SEG20, t + '.npz')
      if not os.path.exists(f):
        f = B._p('cache', 'rl20', t + '.npz')
      with np.load(f, allow_pickle=False) as z:
        k = len(z['t'])
      grids.append(B.seg_grid(t, k))
    x = (np.concatenate([g[1] for g in grids]) - org['lon0']) * org['kx']
    y = (np.concatenate([g[0] for g in grids]) - org['lat0']) * org['ky']
    hdg = np.radians(90.0 - np.concatenate([g[3] for g in grids]))
    ok = np.isfinite(x) & np.isfinite(y) & np.concatenate([g[2] for g in grids]).astype(bool) & np.isfinite(hdg)
    out.append(dict(X=x, Y=y, hdg=hdg, ok=ok))
  return out


def fix_at(Pm, mem, r, i, raw=None):
  """The planner's fix for sample i of run r: the governor's snapped memory point (default), or the raw position of the same sample
  (`raw`: this run's raw_positions entry)."""
  if raw is not None:
    return Pm.Fix(True, float(raw['X'][i]), float(raw['Y'][i]), float(raw['hdg'][i])) if raw['ok'][i] else Pm.Fix(False)
  pt = int(r['pt'][i])
  return Pm.Fix(True, float(mem.x[pt]), float(mem.y[pt]), float(mem.hdg[pt])) if pt >= 0 else Pm.Fix(False)


def steer_arrays(r):
  """The steering cache on the run's grid: rate (deg/s), ti_out, ti_state, lkas_eff, lkas_block; NaN (state -1) where absent."""
  n = len(r['v'])
  out = {k: np.full(n, np.nan) for k in ('rate', 'ti_out', 'ti_state', 'lkas_eff', 'lkas_block')}
  have = np.zeros(n, bool)
  off = list(r['off'])
  for i, tag in enumerate(r['tags']):
    f = os.path.join(STEER_DIR, f'{tag}.npz')
    if not os.path.exists(f):
      continue
    a, b = int(off[i]), int(off[i + 1]) if i + 1 < len(off) else n
    with np.load(f, allow_pickle=False) as z:
      k = min(b - a, len(z['rate']))
      for key in out:
        out[key][a:a + k] = z[key][:k]
      have[a:a + k] = True
  return out, have


def _seg_sec(r, i):
  off = np.asarray(r['off'])
  sg = int(np.searchsorted(off, i, side='right') - 1)
  return str(r['tags'][sg]), round(float((i - off[sg]) * DT), 1)


def _match(passes, s, lo_m, hi_m):
  """The pass whose recorded stretch overlaps [lo_m, hi_m] (distance along the run), the sharpest if several; None."""
  best = None
  for p in passes:
    if s[p['b'] - 1] >= lo_m and s[p['a']] <= hi_m and (best is None or p['la'] > best['la']):
      best = p
  return best


# ---- the car side's own rules for when a slowdown can act (R13) ------------------------------------------------------------------
# Mirrored from selfdrive/car/mazda/ti_curve.py WITHOUT importing it (another builder edits that file): its constants are read from the
# source text, and the rules below restate its update(): (1) the gas pressed, or released less than GAS_RESUME_S ago (gas_hold): no tap
# starts; (2) the gas held longer than GAS_MAX_S, the brake, openpilot not engaged, the cruise off (or no set speed): the intervention
# stops (`_stop`; the set speed is walked back up). A slowdown the planner starts while (1) or (2) holds acts from the first sample
# they clear, if that is before its bend's end (the planner's ceiling is still there and the car side taps toward it whenever it may);
# one that is acting stops at the first sample (2) holds. Not modelled: a fresh intervention after such a stop (CLEAR_S later) on the
# same bend, and the pause of taps during a short (< GAS_MAX_S) gas press mid-slowdown (the driver is accelerating then).
TI_CURVE_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'selfdrive', 'car', 'mazda', 'ti_curve.py')


def car_side_constants(path=TI_CURVE_SRC):
  """{'GAS_MAX_S', 'GAS_RESUME_S'} as ti_curve.py defines them (read from its text; never imported)."""
  import re
  txt = open(path, encoding='utf-8').read()
  out = {}
  for name in ('GAS_MAX_S', 'GAS_RESUME_S'):
    m = re.search(rf'^{name}\s*=\s*([0-9.]+)', txt, re.M)
    if m is None:
      raise RuntimeError(f'{name} not found in ti_curve.py: the replay cannot mirror the car side')
    out[name] = float(m.group(1))
  return out


def car_gate(gas, brake, engaged, cruise_on, dt=DT, consts=None):
  """Per sample: (can_start, stop) as the car side decides them (see above). Inputs are per-sample truthy arrays."""
  c = consts or car_side_constants()
  gas, brake = np.asarray(gas) > .5, np.asarray(brake) > .5
  ok = (np.asarray(engaged) > .5) & (np.asarray(cruise_on) > .5)
  n = len(gas)
  hold, long_ = np.zeros(n, bool), np.zeros(n, bool)
  since, off_at, h = None, None, False
  for i in range(n):
    if gas[i]:
      since = i if since is None else since
      off_at = None
    else:
      since = None
      if h and off_at is None:
        off_at = i
    h = bool(gas[i]) or (off_at is not None and (i - off_at) * dt < c['GAS_RESUME_S'] - 1e-9)
    hold[i] = h
    long_[i] = since is not None and (i - since) * dt > c['GAS_MAX_S']
  stop = brake | long_ | ~ok
  return ~stop & ~hold, stop


_CONSTS = {}


def _consts():
  if not _CONSTS:
    _CONSTS.update(car_side_constants())
  return _CONSTS


def act_window(can_start, stop, i0, i_end):
  """(j, k): the first sample in [i0, i_end) the slowdown can act from (None: never before its bend's end), and the first sample
  after j (before i_end) the car side stops it (None: it runs to the end)."""
  i_end = max(int(i_end), int(i0) + 1)
  w = np.nonzero(can_start[i0:i_end])[0]
  if not len(w):
    return None, None
  j = int(i0 + w[0])
  ks = np.nonzero(stop[j + 1:i_end])[0]
  return j, (int(j + 1 + ks[0]) if len(ks) else None)


def speed_lost(prof, u, t_end):
  """mph-seconds below the start speed u: the profile from the start to t_end, then the way back up at ACCEL_MPH_S."""
  t = np.arange(0.0, max(t_end, 0.0) + 1e-9, 0.1)
  vv = prof.speed(t)
  lost = float(np.sum(np.maximum(0.0, u - vv)) * 0.1)
  d_end = max(0.0, u - float(prof.speed(t_end)))
  return lost + d_end * d_end / (2.0 * ACCEL_MPH_S)


def account_row(x, Pm, s, v, kap, can_start, stop, s_end, opts):
  """R13 (and R16 / R17) accounting of one slowdown row (stage 1, 2 or 3) in place: when the car side acts on it (act_window), the
  speed it actually sheds under the ONE cruise model until its bend's end, its release (stage 1) or the car side's stop, the speed lost,
  and the dwell. -> x. `x['T_target']` keeps the planner's target; `x['T']` becomes u - shed for a stopped / released one."""
  x['T_target'], x['u_plan'] = x['T'], x['u']
  # R13: when (and whether) the car side acts on it, and where it stops it
  j, k_stop = act_window(can_start, stop, x['i'], int(np.searchsorted(s, s_end)) + 1)
  stop_t = None if (j is None or k_stop is None) else (k_stop - j) * DT
  if j is not None and x['stage'] == 1 and x.get('rel'):
    t_rel = (x['i_dec'] - j) * DT
    j = None if t_rel <= 0 else j                    # released before the car side could act on it: nothing happens
    stop_t = t_rel if stop_t is None else min(stop_t, t_rel)
  if j is None:
    x['acts'], x['lost'], x['shed'] = False, 0.0, 0.0
    return x
  x['acts'] = x.get('acts', True)
  x['defer_s'] = round((j - x['i']) * DT, 2)
  u = round(float(v[j] / MPH), 1)
  T = x['T']
  floor = x.get('floor') or max(25, int(round(u)) - 15)
  # the planner's own push floor (hold-branch merge: the push may pass the 15 mph line by its depth, CM.push_floor; an older planner
  # file without it: the floor)
  pf = Pm.CM.push_floor(int(floor), Pm.DRIVE_GAP_MPH) if hasattr(getattr(Pm, 'CM', None), 'push_floor') else int(floor)
  S1 = max(pf, T - Pm.DRIVE_GAP_MPH) if x['stage'] == 1 else None
  cap = None
  s1_cap = getattr(Pm, 'S1_UNCONFIRMED_MAX_MPH', None) if x['stage'] == 1 else None
  if s1_cap is not None and x.get('capped'):
    # R16: before the decision (confirmed) the dash goes no lower than the planner's start speed - S1_UNCONFIRMED_MAX_MPH
    t_c = (x['i_dec'] - j) * DT if (x.get('i_dec') is not None and not x.get('rel')) else 1e9
    cap = (Pm.s1_unconfirmed_ceiling(x['u_plan']), t_c)
  x['u'] = u
  prof = ED.Profile(u, T, S1=S1, cap=cap, stop_t=stop_t, tap_s=opts.get('tap_s')) if u > T else None
  if prof is not None:
    t_end = prof.t_at(max(0.0, s_end - s[j]))
    x['lost'] = round(speed_lost(prof, u, t_end), 1)
    x['shed'] = round(prof.shed(t_end), 1)
  else:
    x['lost'], x['shed'] = 0.0, 0.0
  if stop_t is not None:
    x['T'] = round(u - x['shed'], 1)                  # scored with the speed actually shed by its release / stop
  elif cap is not None and cap[1] > 1e8:
    x['T'] = max(T, cap[0])                          # never confirmed: the dash went no lower than the cap
  if x['stage'] in (1, 2) and u > x['T'] and stop_t is None:
    i_from = int(np.searchsorted(s, x['aim'] - LOAD_PRE_S * u * MPH))
    kw = dict(cap=cap, tap_s=opts.get('tap_s'))
    x['dwell'] = ED.dwell(u, T, s, kap, j, s_end + 100.0, DT, S1=S1, i_from=i_from, **kw)
    x['dwell0'] = ED.dwell(u, T, s, kap, j, s_end + 100.0, DT, S1=S1, **kw)   # the old search, from the start (A5 comparison)
  return x


def replay_drive(args):
  drive, mempath, opts = args
  Pm = planner_module(opts.get('planner'))
  apply_options(Pm, opts.get('confirm_share'), opts.get('cap_scale', 1.0), opts.get('min_drop'), opts.get('min_passes'),
                opts['unconfirmed_cap'] if 'unconfirmed_cap' in opts else _UNSET)
  mem = _mem(mempath)
  new = Pm is P
  sig = inspect.signature(Pm.Planner.step).parameters
  prov = opts.get('_prov', {})
  res = dict(drive=drive, eng_s=0.0, eng_std_s=0.0, passes=[], slow=[], alerts=[], s3=[], s3_cov_s=0.0, signs=[], events={})
  runs = G.load_runs(drive)
  raw = raw_positions(drive) if opts.get('raw_fixes') else None
  if raw is not None:
    assert len(raw) == len(runs), (drive, len(raw), len(runs))
  for ri, r in enumerate(runs):
    v = np.asarray(r['v'], np.float64)
    n = len(v)
    kap = np.asarray(r['kap'], np.float64)
    s = np.r_[0.0, np.cumsum(v[:-1] * DT)]
    passes = G.passes_of_run(r)
    for p in passes:
      p['std'] = prov.get((p['tag'], p['t_seg']))
      p['run'] = ri
    # per-segment standard build: from its passes, else the drive's majority
    seg_std = {}
    for p in passes:
      seg_std.setdefault(p['seg'], []).append(bool(p['std']))
    allp = [bool(p['std']) for p in passes if p['std'] is not None]
    drive_std = (sum(allp) * 2 >= len(allp)) if allp else True
    off = list(r['off']) + [n]
    eng = (np.asarray(r['en']) > .5) & (v > 8.0)
    res['eng_s'] += float(eng.sum() * DT)
    for sg in range(len(r['tags'])):
      st = seg_std.get(sg)
      if (all(st) if st else drive_std):
        res['eng_std_s'] += float(eng[off[sg]:off[sg + 1]].sum() * DT)
    lat = np.asarray(r['lat_act']) > .5
    can_start, stop = car_gate(r['gas'], r['brk'], r['en'], (np.asarray(r['cr_en']) > .5) & (np.asarray(r['cr_set']) > .5),
                               consts=_consts())
    st_arr, have = steer_arrays(r) if (new and not opts.get('no_s3')) else ({}, np.zeros(n, bool))
    la_now = kap * v * v + np.nan_to_num(bank_fixed(r['bank'], kap, v)) if have.any() else None
    pred, pred_t = r.get('pred_la'), r.get('pred_t')
    planner = Pm.Planner(mem)
    mon = TM.DeliveryMonitor() if new else None
    plan = None
    mon_alert = False
    for i in range(n):
      if i % TICK == 0:
        fix = fix_at(Pm, mem, r, i, None if raw is None else raw[ri])
        set_mph = float(r['cr_set'][i]) / MPH if r['cr_en'][i] > .5 and r['cr_set'][i] > .5 else None
        mk = None
        if pred is not None and pred_t is not None:
          row = np.asarray(pred[i], np.float64)
          if np.all(np.isfinite(row)):
            mk = (np.asarray(pred_t, np.float64), row / max(v[i], 3.0) ** 2)   # pred_la = z * vx: curvature z / vx with vx ~ v
        kw = dict(set_mph=set_mph, yaw_k=float(kap[i]) if v[i] > 3 else None, model_k=mk)
        if 'blinker' in sig:
          kw['blinker'] = bool(r['blink'][i] > .5)
        plan = planner.step(i * DT, fix, float(v[i]), **kw)
        mph = v[i] / MPH
        # (whether and when the car side acts on a start: car_gate / act_window at scoring, R13)
        for e in planner.events:
          k = e.get('kind')
          if k in ('rm_off', 'rm_on', 'lock', 'lock_lost', 'dead_reckon'):
            res['events'][k] = res['events'].get(k, 0) + 1      # counts only (road matching's firing rate)
          if k in ('s1_release', 's1_confirm'):
            # the stage-1 bend's one decision (R13: a released bend is scored only up to its release; R16: the rest only after it)
            kw_ = f"{k}:{e.get('why', '')}"
            res['events'][kw_] = res['events'].get(kw_, 0) + 1     # counts by reason (the planner's release rules, REL_*)
            for x in reversed(res['slow']):
              if x['run'] == ri and x['stage'] == 1 and x['key'] == str(e.get('point')) and 'i_dec' not in x:
                x['i_dec'], x['rel'], x['why'] = i, k == 's1_release', e.get('why', '')
                break
            continue
          if k == 'start' and mph - e['T'] >= (opts.get('min_drop') or 2):
            a_ = (planner.map_active if (new and not isinstance(e.get('point'), int)) else planner.active)
            key = e['point'] if e['point'] in a_ else next((kk for kk, aa in a_.items() if aa.get('start_t') == i * DT), None)
            aa = a_.get(key)
            if aa is None:
              continue
            ent = s[i] + (aa['entry'] - planner.odo)
            end = s[i] + (aa['at'] - planner.odo)
            aim = s[i] + (aa.get('aim_at', aa['entry']) - planner.odo)
            res['slow'].append(dict(stage=1, run=ri, i=i, u=round(mph, 1), T=int(e['T']), floor=e.get('floor'), ent=ent, end=end, aim=aim,
                                    key=str(key), capped=aa.get('cap_c') is not None))
          elif k == 's2_start':
            lead = float(e.get('lead') or 0.0)
            at = s[i] + lead * v[i]
            res['slow'].append(dict(stage=2, run=ri, i=i, u=round(mph, 1), T=int(e['T']), floor=e.get('floor'), ent=at, end=at + 30.0,
                                    aim=at, key='s2'))
          elif k == 'alert':
            res['alerts'].append(dict(stage=0, run=ri, i=i, u=round(mph, 1), need=int(e['need']), ent=s[i] + float(e.get('dist') or 0.0)))
        planner.events.clear()
      if mon is not None and have[i] and plan is not None:
        res['s3_cov_s'] += DT if eng[i] else 0.0
        ahead = plan.ahead_la
        need3 = int(CD.need_mph(float(ahead) / (v[i] * v[i]))[0]) if (ahead and v[i] > 1.0) else None
        tst = st_arr['ti_state'][i]
        # (stage 3 alerts whatever the cruise; its taps need the cruise: scored as slowdowns only while engaged, below)
        mi = TM.MonInputs(mph=v[i] / MPH, ti_cmd=float(np.nan_to_num(st_arr['ti_out'][i])), ti_max=TI_MAX, lat_active=bool(lat[i]),
                          ti_active=bool(lat[i] and tst == 3), steering_pressed=bool(r['pressed'][i] > .5),
                          angle_rate_deg=float(np.nan_to_num(st_arr['rate'][i])), la_now=float(la_now[i]), desired_la=None,
                          ahead_la=ahead, ahead_need=need3, lkas_blocked=bool(st_arr['lkas_block'][i] > .5),
                          lkas_effective=float(st_arr['lkas_eff'][i]) if np.isfinite(st_arr['lkas_eff'][i]) else None,
                          ti_state=int(tst) if np.isfinite(tst) else None, ti_viol=0,
                          ahead_prof=(TM.ASK_LEADS, plan.ask) if plan.ask else None)
        for q in range(MON_SUB):
          out = mon.step(i * MON_SUB + q, mi)
        for ev in mon.events:
          if ev['kind'] == 'deliver_onset':
            res['s3'].append(dict(stage=3, run=ri, i=i, u=round(v[i] / MPH, 1), T=int(ev['need']), why=ev['why'], s=s[i],
                                  acts=bool(r['cr_en'][i] > .5 and r['en'][i] > .5)))
        mon.events.clear()
        mon_alert = out.phase == 'alert'
      elif mon is not None and not have[i] and mon_alert:
        mon, mon_alert = TM.DeliveryMonitor(), False
    # match and score this run's slowdowns
    for x in [x for x in res['slow'] + res['alerts'] + res['s3'] if x['run'] == ri and 'p' not in x]:
      if x['stage'] == 3:
        p = _match(passes, s, x['s'] - S3_MATCH_S * v[x['i']], x['s'] + 1.0)
      elif x['stage'] == 0:
        p = _match(passes, s, x['ent'] - MATCH_PRE_M, x['ent'] + 200.0)
      elif x['stage'] == 2:
        p = _match(passes, s, x['ent'] - S2_WIN_M, x['ent'] + S2_WIN_M)
      else:
        p = _match(passes, s, x['ent'] - MATCH_PRE_M, x['end'] + MATCH_POST_M)
      x['p'] = None if p is None else passes.index(p)
      x['seg'], x['sec'] = _seg_sec(r, x['i'])
      w0, w1 = x['i'], int(np.searchsorted(s, (s[p['b'] - 1] if p else x.get('end', x.get('ent', s[x['i']]))) + 1.0))
      x['eng'] = float(lat[w0:max(w0 + 1, min(n, w1))].mean())
      if x['stage'] in (1, 2, 3):
        account_row(x, Pm, s, v, kap, can_start, stop, s[p['b'] - 1] if p else x.get('end', s[x['i']]), opts)
    for p in passes:
      p['drive'] = drive
      p['s3cov'] = bool(have[p['a']:p['b']].all())
      res['passes'].append({k: p[k] for k in ('s3cov', 'drive', 'run', 'tag', 't_seg', 't_a', 'a', 'b', 'car', 'trouble', 'la', 'sign', 'std',
                                              'limit_s', 'limit_on', 'v_entry', 'v_peak')})
      # sign checks (R10 / R12) on unambiguous car-steered bends: the preview's side through MODEL_K_SIGN, and the steering rate
      if p['car'] and p['la'] >= 1.8 and pred is not None:
        j = max(0, p['a'] - 20)
        row = np.asarray(pred[j], np.float64)
        if np.all(np.isfinite(row)) and pred_t is not None:
          _, _, _, side = CD.forecast_need(np.asarray(pred_t, np.float64), Pm.MODEL_K_SIGN * row / max(v[j], 3.0) ** 2, v[j], 0.0, 3.0) \
            if hasattr(Pm, 'MODEL_K_SIGN') else (None, None, None, 0)
          rate = st_arr['rate'][p['a']:p['i']] if have[p['a']:p['i']].all() and p['i'] > p['a'] else None
          ln = la_now[p['i']] if (la_now is not None and have[p['i']]) else None
          res['signs'].append(dict(sign=p['sign'], fc_side=int(side), rate=None if rate is None else float(np.nanmedian(rate)),
                                   la_now=None if ln is None else float(ln)))
  # strip the run-local pass index into a drive-wide one
  base = {}
  for k, p in enumerate(res['passes']):
    base.setdefault(p['run'], k)
  for x in res['slow'] + res['alerts'] + res['s3']:
    if x.get('p') is not None:
      x['p'] = base[x['run']] + x['p']
  return res


# ---- scoring (each figure says how many passes and engaged hours it rests on) ------------------------------------------------------------------------------------------------------------------------
def _rows(R, std_only):
  """All slowdowns / alerts with their pass, the passes, and engaged hours (standard builds only, or all)."""
  rows, passes, eng = [], [], 0.0
  for res in R:
    eng += res['eng_std_s' if std_only else 'eng_s'] / 3600.0
    off = len(passes)
    ps = res['passes']
    passes += ps
    for x in res['slow'] + res['alerts'] + res['s3']:
      p = None if x.get('p') is None else ps[x['p']]
      if std_only and p is not None and p['std'] is False:
        continue
      rows.append(dict(x, drive=res['drive'], P=None if p is None else off + x['p']))
  if std_only:
    keep = [p['std'] is not False for p in passes]
  else:
    keep = [True] * len(passes)
  return rows, passes, keep, eng


def _cls(x, passes):
  """'trouble' / 'clean' (car-steered pass) / 'hand' (not car-steered: no evidence) / 'nobend' (no bend loads the car; engaged) /
  'unknown' (no bend, not engaged)."""
  if x['P'] is None:
    return 'nobend' if x.get('eng', 0.0) >= 0.9 else 'unknown'
  p = passes[x['P']]
  if not p['car']:
    return 'hand'
  return 'trouble' if p['trouble'] else 'clean'


def _dedupe(sl):
  """One slowdown per stage per bend: a bend re-started (its tight run's end moved as the recorded speed changed) counts once, at
  its first start, with the largest drop of its starts (`restarts` counts the rest). No bend: starts within 20 s of each other."""
  out, seen = [], {}
  for x in sorted(sl, key=lambda x: (x['drive'], x['run'], x['i'])):
    key = (x['stage'], 'p', x['P']) if x['P'] is not None else None
    if key is None:
      prev = seen.get((x['stage'], 'i', x['drive'], x['run']))
      if prev is not None and (x['i'] - prev['i_last']) * DT <= 20.0:
        prev['drop'] = max(prev['drop'], x['drop'])
        prev['restarts'] += 1
        prev['i_last'] = x['i']
        continue
      y = dict(x, restarts=0, i_last=x['i'])
      seen[(x['stage'], 'i', x['drive'], x['run'])] = y
      out.append(y)
      continue
    if key in seen:
      seen[key]['drop'] = max(seen[key]['drop'], x['drop'])
      seen[key]['restarts'] += 1
      continue
    y = dict(x, restarts=0, i_last=x['i'])
    seen[key] = y
    out.append(y)
  return out


def score(R, std_only=True, stages=(1, 2, 3)):
  rows, passes, keep, eng = _rows(R, std_only)
  car = [k for k, p in enumerate(passes) if keep[k] and p['car']]
  trouble = [k for k in car if passes[k]['trouble']]
  out = {'engaged_h': round(eng, 2), 'car_passes': len(car), 'trouble_passes': len(trouble)}
  s3h = sum(r['s3_cov_s'] for r in R) / 3600.0
  trouble3 = [k for k in trouble if passes[k].get('s3cov')]
  out['stage3_basis'] = {'covered_engaged_h_all_builds': round(s3h, 2), 'trouble_passes_covered': len(trouble3)}
  for name, st in [(f'stage{k}', (k,)) for k in stages] + [('overall', tuple(stages))]:
    sl = [x for x in rows if x['stage'] in st and x.get('acts', True)]
    for x in sl:
      x['cls'] = _cls(x, passes)
      x['drop'] = x['u'] - x['T']
    sl = _dedupe([x for x in sl if x['drop'] >= 1])
    if name == 'overall':
      # one slowdown per bend: the largest drop among the stages (no bend: each its own)
      by = {}
      for x in sl:
        key = ('p', x['P']) if x['P'] is not None else ('i', x['drive'], x['run'], x['i'])
        if key not in by or x['drop'] > by[key]['drop']:
          by[key] = dict(x, lost=sum(y.get('lost', 0.0) for y in sl if y['P'] == x['P']) if x['P'] is not None else x.get('lost', 0.0))
      sl = list(by.values())
    big = [x for x in sl if x['drop'] >= BIG_MPH and x['cls'] in ('trouble', 'clean', 'nobend')]
    tp = sum(1 for x in big if x['cls'] == 'trouble')
    caught = {x['P'] for x in sl if x['drop'] >= BIG_MPH and x['cls'] == 'trouble'}
    un = [x for x in sl if x['drop'] >= UNNEC_MPH and x['cls'] in ('clean', 'nobend')]
    lost_un = sum(x.get('lost', 0.0) for x in sl if x['cls'] in ('clean', 'nobend'))
    clean_drops = [x['drop'] for x in sl if x['cls'] == 'clean']
    dw = [x['dwell'] for x in sl if x.get('dwell') is not None]
    sm = ED.summary(dw)
    de = [x['dwell']['dwell_entry_s'] for x in sl if x.get('dwell') and x['dwell'].get('dwell_entry_s') is not None]
    if de:
      sm['entry_def_median'] = round(float(np.median(de)), 1)        # the measured episodes' definition (A5)
    sm['late_from_start_search'] = sum(1 for x in sl if x.get('dwell0') and x['dwell0']['late'])
    h = s3h if name == 'stage3' else eng
    tr = trouble3 if name == 'stage3' else trouble
    caught = caught & set(tr) if name == 'stage3' else caught
    d = dict(slowdowns=len(sl), per_h=round(len(sl) / max(h, 1e-9), 2), restarts=sum(x.get('restarts', 0) for x in sl), big=len(big),
             precision5=None if not big else round(tp / len(big), 2), recall5=None if not tr else round(len(caught) / len(tr), 2),
             unnec3=len(un), unnec3_per_h=round(len(un) / max(h, 1e-9), 2), lost_unnec_mphs_per_h=round(lost_un / max(h, 1e-9), 0),
             lost_all_mphs_per_h=round(sum(x.get('lost', 0.0) for x in sl) / max(h, 1e-9), 0),
             max_clean_drop=max(clean_drops) if clean_drops else 0, dwell=sm,
             cls={c: sum(1 for x in sl if x['cls'] == c) for c in ('trouble', 'clean', 'hand', 'nobend', 'unknown')})
    out[name] = d
  # alerts: the pre-bend warning (planner) and stage 3 onsets
  for name, st in (('warning', (0,)), ('stage3_alerts', (3,))):
    al = [x for x in rows if x['stage'] in st]
    for x in al:
      x['cls'] = _cls(x, passes)
    h, tr = (s3h, trouble3) if name == 'stage3_alerts' else (eng, trouble)
    ev = [x for x in al if x['cls'] in ('trouble', 'clean', 'nobend')]
    tp = sum(1 for x in ev if x['cls'] == 'trouble')
    caught = {x['P'] for x in al if x['cls'] == 'trouble'} & set(tr)
    out[name] = dict(n=len(al), per_h=round(len(al) / max(h, 1e-9), 2), precision=None if not ev else round(tp / len(ev), 2),
                     recall=None if not tr else round(len(caught) / len(tr), 2),
                     false=sum(1 for x in al if x['cls'] in ('clean', 'nobend')),
                     false_per_h=round(sum(1 for x in al if x['cls'] in ('clean', 'nobend')) / max(h, 1e-9), 2))
  return out


def must_catch(R, path=None):
  """Each private must-catch entry ({route, segment, t0, t1}: trouble seconds in the segment): did stage 3 alert within 1 s of the
  limit's onset on that bend? -> list of dicts (route, segment, covered, onset_s, alert_s)."""
  path = path or os.path.join(OUT_DIR, 'must_catch.json')
  if not os.path.exists(path):
    return None
  out = []
  for e in json.load(open(path)):
    tag = f"{e['route']}-{e['segment']}"
    res = next((x for x in R if x['drive'] == e['route']), None)
    if res is None:
      out.append(dict(tag=tag, covered=None, why='route not replayed'))
      continue
    ps = [p for p in res['passes'] if p['tag'] == tag and e['t0'] - 3 <= p['t_seg'] <= e['t1'] + 3 and p['trouble']]
    if not ps:
      out.append(dict(tag=tag, covered=None, why='no trouble pass there'))
      continue
    p = ps[0]
    k = res['passes'].index(p)
    on = p['limit_on']
    al = [x for x in res['s3'] if x.get('p') == k]
    first = min((x['i'] for x in al), default=None)
    lag = None if first is None or on is None else round((first - on) * DT, 2)
    out.append(dict(tag=tag, covered=bool(lag is not None and lag <= 1.0), onset_s=round(p['t_a'] + (on - p['a']) * DT, 2) if on else None,
                    alert_lag_s=lag, warned=any(x.get('p') == k for x in res['alerts']),
                    slowed=max([x['u'] - x['T'] for x in res['slow'] if x.get('p') == k], default=0)))
  return out


def run(drives, mempath, opts, workers=3):
  opts = dict(opts)
  opts['_prov'] = G.provenance()
  with ProcessPoolExecutor(workers, initializer=_low) as ex:
    return list(ex.map(replay_drive, [(d, mempath, opts) for d in drives]))


def _low():
  try:
    import ctypes
    ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x00004000)
  except Exception:
    pass


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--mem', default=f'{MEM_DIR}/curve_memory_all_v6.npz')
  ap.add_argument('--drives', nargs='*')
  ap.add_argument('--workers', type=int, default=3)
  ap.add_argument('--confirm-share', type=float, default=None)
  ap.add_argument('--cap-scale', type=float, default=1.0)
  ap.add_argument('--min-drop', type=int, default=None)
  ap.add_argument('--planner', default='', help='an older ti_curve_plan saved as a file (A5: the build on the car)')
  ap.add_argument('--no-s3', action='store_true')
  ap.add_argument('--tap-s', type=float, default=None, help='the tap cycle the replay cruise model walks the dash with (R17; the planner is unchanged)')
  ap.add_argument('--min-passes', type=int, default=None, help='S1_MIN_PASSES (default: the planner value, 1)')
  ap.add_argument('--unconfirmed-cap', default=None, help='S1_UNCONFIRMED_MAX_MPH: a whole mph, or "none" for no cap (default: the planner value)')
  ap.add_argument('--raw-fixes', action='store_true', help='feed the planner the RAW position fixes (road matching), not the snapped ones')
  ap.add_argument('--out', default='')
  a = ap.parse_args()
  drives = a.drives or G.drives()
  opts = dict(confirm_share=a.confirm_share, cap_scale=a.cap_scale, min_drop=a.min_drop, planner=a.planner or None, no_s3=a.no_s3,
              tap_s=a.tap_s, min_passes=a.min_passes, raw_fixes=a.raw_fixes)
  if a.unconfirmed_cap is not None:
    opts['unconfirmed_cap'] = None if a.unconfirmed_cap.lower() == 'none' else int(a.unconfirmed_cap)
  R = run(drives, a.mem, opts, a.workers)
  rep = dict(options={k: v for k, v in opts.items()}, memory=os.path.basename(a.mem), drives=len(drives),
             standard=score(R, True), all_builds=score(R, False), must_catch=must_catch(R))
  print(json.dumps(rep, indent=1, default=str))
  if a.out:
    json.dump(dict(report=rep, raw=R), open(a.out, 'w'), default=lambda o: o.item() if hasattr(o, 'item') else str(o))


if __name__ == '__main__':
  main()
