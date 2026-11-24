"""Ground truth for curve slowing's replay (plan T9): bend passes and their trouble, ported from the curve-governor measurement
(tools/curve_slowing/governor/passes.py on branch curve-governor; REPORT.md there). Pure and position-free.

Bend pass: 20-sample smoothed |v^2 kap| >= BEND_LA for >= 1 s at >= 19 mph (one run of consecutive allowed segments). Car-steered:
lateral control active on >= CAR_STEERED of the bend up to its first stretch of 1 s or more without it, that part >= 1 s long.
Per pass:
  limit_s     seconds with the interceptor request at its clamp (|ti| >= SAT_TI, lateral active, driver not pressing) in the bend
  takeover    a firm takeover (the memory builder's run_events: firm hand torque then lateral off, or a brake disengagement at speed
              with steering evidence) from the bend's start to 2 s after its end
  excursion   the car's SIDE (nearest confident lane line's centre distance minus HALF_W) at or over the line for >= 0.3 s inside the
              bend to 2 s after it, lateral active, no blinker within +-5 s
  trouble     car-steered AND limit_s >= 2 AND (takeover OR excursion)

A run is the governor's position-free run dict (cache/runs/<drive>.pkl: 20 Hz arrays v, kap, lat_act, ti, pressed, blink, lane_d,
tk_i, off, tags, ...). Those caches were made after the governor's segment filter (common.allowed_tags: labelled, corpus-excluded and
reserved segments dropped before any segment was read); this module opens nothing else.
"""
import os
import pickle

import numpy as np

GOV_DIR = 'E:/openpilot-mazda-ti-data/curve-governor'
RUNS_DIR = os.path.join(GOV_DIR, 'cache', 'runs')
DT = 0.05
MPH = 0.44704
HALF_W = 0.92
SAT_TI = 590.0
V_MIN = 19 * MPH
BEND_LA = 1.3
BEND_MIN = 20                   # samples (1 s)
CAR_STEERED = 0.9
POST = 40                       # samples (2 s) after the bend for takeover / excursion
EXC_MIN = 6                     # samples (0.3 s)
LIMIT_S = 2.0
LIMIT_ON_S = 0.5                # the limit's onset: its first stretch lasting this (stage 3's MON_HOLD_S)
BLINK_PAD_S = 5.0


def runs(mask):
  d = np.diff(np.concatenate(([0], np.asarray(mask).astype(np.int8), [0])))
  return np.flatnonzero(d == 1), np.flatnonzero(d == -1)


def smooth20(x):
  return np.convolve(np.nan_to_num(np.asarray(x, np.float64)), np.ones(20) / 20, 'same')


def dilate(m, pad):
  c = np.cumsum(np.r_[0, np.asarray(m).astype(np.int64)])
  n = len(m)
  i = np.arange(n)
  return (c[np.minimum(n, i + pad + 1)] - c[np.maximum(0, i - pad)]) > 0


def longest(mask):
  s, e = runs(mask)
  return int((e - s).max()) if len(s) else 0


def passes_of_run(r):
  """-> list of pass dicts (indices into the run, seconds, flags; never a position)."""
  v = np.asarray(r['v'], np.float64)
  kap = np.asarray(r['kap'], np.float64)
  n = len(v)
  las = smooth20(kap * v * v)
  lat = np.asarray(r['lat_act']) > .5
  pressed = np.asarray(r['pressed']) > .5
  blink = dilate(np.asarray(r['blink']) > .5, int(BLINK_PAD_S / DT))
  side = np.asarray(r['lane_d'], np.float64) - HALF_W
  sat = (np.abs(np.asarray(r['ti'], np.float64)) >= SAT_TI) & lat & ~pressed
  tk_i = np.asarray(r.get('tk_i', []), int)
  off = np.asarray(r['off'])
  s, e = runs((np.abs(las) >= BEND_LA) & (v >= V_MIN))
  out = []
  for a, b in zip(s, e):
    if b - a < BEND_MIN:
      continue
    i = a + int(np.argmax(np.abs(las[a:b])))
    w2 = slice(a, min(n, b + POST))
    so, eo = runs(~lat[a:b])
    long_off = [x for x, y in zip(so, eo) if y - x >= 20 or y == b - a]
    j = a + (long_off[0] if long_off else b - a)
    steered = float(lat[a:j].mean()) if j - a >= BEND_MIN else 0.0
    tk = bool(((tk_i >= a) & (tk_i < min(n, b + POST))).any())
    okside = lat[w2] & ~blink[w2] & np.isfinite(side[w2])
    sd = np.where(okside, side[w2], np.inf)
    exc = longest(sd <= 0.0) >= EXC_MIN
    seg = int(np.searchsorted(off, i, side='right') - 1)
    car = steered >= CAR_STEERED
    lim = float(sat[a:b].sum() * DT)
    # the limit's onset in the bend: the first stretch at the limit lasting LIMIT_ON_S (a 0.1 s touch is not it; stage 3 must alert
    # within 1 s of the onset on a must-catch bend)
    ls, le = runs(sat[a:b])
    js = [x for x, y in zip(ls, le) if (y - x) * DT >= LIMIT_ON_S - 1e-9]
    out.append(dict(a=int(a), b=int(b), i=int(i), seg=seg, tag=str(r['tags'][seg]), t_seg=round(float((i - off[seg]) * DT), 2),
                    t_a=round(float((a - off[seg]) * DT), 2), dur_s=(b - a) * DT, steered=steered, car=car,
                    la=float(abs(las[i])), sign=float(np.sign(las[i])), v_peak=float(v[i] / MPH), v_entry=float(v[a] / MPH),
                    limit_s=lim, limit_on=int(a + js[0]) if len(js) else None, takeover=tk, excursion=exc,
                    trouble=bool(car and lim >= LIMIT_S and (tk or exc))))
  return out


def load_runs(drive):
  """The governor's position-free runs of one drive (list of dicts), or []."""
  f = os.path.join(RUNS_DIR, f'{drive}.pkl')
  if not os.path.exists(f):
    return []
  with open(f, 'rb') as fh:
    return pickle.load(fh)


def drives():
  return sorted(os.path.basename(f)[:-4] for f in os.listdir(RUNS_DIR) if f.endswith('.pkl')) if os.path.isdir(RUNS_DIR) else []


def provenance():
  """(tag, peak second in segment) -> standard build (bool), from the governor's passes.pkl (step-one provenance per pass)."""
  f = os.path.join(GOV_DIR, 'passes.pkl')
  if not os.path.exists(f):
    return {}
  with open(f, 'rb') as fh:
    P = pickle.load(fh)
  return {(p['tag'], round(float(p['t_seg']), 2)): bool(p.get('standard')) for p in P}
