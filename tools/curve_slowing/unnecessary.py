"""Unnecessary curve slowdowns, measured against what the very pass did (off the car, open loop).

For every cached drive the planner (ti_curve_plan.Planner, 5 Hz) runs on the recorded location stream; every slowdown it would START
(target more than 3 mph below the car's speed) is compared with what the recorded pass did through that bend: its lowest speed from the
corner's entry to its end, and whether it had trouble there (steering-limit event, firm takeover, lane-line event: the memory builder's
own detectors, +-3 s around the bend).
  NEEDED       the real pass had trouble, or it was already at or below target + 3 mph
  UNNECESSARY  the real pass went through at >= target + 5 mph with no trouble
  MARGINAL     the rest
Memory: one file for every drive (default v6, the file on the car; the drive is IN it, so this is in-sample: the old leave-one-out
files predate v6 and are not used). Output is POSITION-FREE: segment+seconds, speeds, counts.

  python tools/curve_slowing/unnecessary.py [--mem FILE] [--drives D ...] [--worst 20]
"""
import argparse
import glob
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

try:
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P
except ImportError:
  sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
  from selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P   # type: ignore
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_memory as B   # noqa: E402

DATA_DIR = 'E:/openpilot-mazda-ti-data/curve-slowing/memory'
MPH = 0.44704
DT = 0.05
TROUBLE_S = 3.0
_MEM = {}


def _mem(path):
  if path not in _MEM:
    _MEM[path] = M.load(path)
  return _MEM[path]


def rule_of(planner, mem, k):
  """which rule set the target at memory point k"""
  fl = int(mem.flags[k])
  env = int(P.corner_mph(mem.kap[k])[0])
  need = int(planner.need[k])
  pv = mem.v_proven[k] / MPH if np.isfinite(mem.v_proven[k]) else float('nan')
  vf = mem.v_flag[k] / MPH if np.isfinite(mem.v_flag[k]) else float('nan')
  names = [n for b, n in ((M.FLAG_LIMIT, 'limit'), (M.FLAG_TAKEOVER, 'takeover'), (M.FLAG_LANE, 'lane')) if fl & b]
  if need < env:
    why = f"trouble cap ({'+'.join(names)} at {vf:.0f}) -> {need}"
  else:
    sharp = abs(mem.kap[k]) > 1.3 * abs(mem.kap_med[k]) + 1e-9
    why = f"envelope {env}" + (" (conservative curvature, median pass allows %d)" % int(P.corner_mph(mem.kap_med[k])[0]) if sharp else "")
  if int(mem.npass[k]) <= 1:
    why += ", single pass"
  pr = int(planner.proven[k])
  if names and np.isfinite(pv):
    why += f", proven {pv:.0f} ZEROED by flag"
  elif pr < need + 2:
    why += f", proven {pr if pr else 'none'}"
  return why


def one(args):
  drive, mempath = args
  mem = _mem(mempath)
  R = np.load(f'{DATA_DIR}/cache/drive_{drive}.npz', allow_pickle=False)
  n = len(R['v'])
  v = R['v'].astype(float)
  s = np.r_[0.0, np.cumsum(v[:-1] * DT)]
  # trouble events in this pass (the builder's detectors)
  lane_d = np.full(n, np.nan)
  offs = list(R['seg_off']) + [n]
  for tag, a, b in zip(R['seg_tags'], offs[:-1], offs[1:]):
    lane_d[a:b] = B.lane_grid(str(tag), int(b - a))
  lane_i = B.lane_events(lane_d, R['lat_act'], R['blink'])
  drv = np.where(R['pressed'] > .5, 10.0 * B.TK_DRV, 0.0)    # no torque in the cache: the steering-pressed bit stands in
  sats, tks = B.run_events(R['ti'], R['lat_act'], R['pressed'], R['en'], drv, brk=R['brk'], v=v, s=s, blink=R['blink'],
                           lead=R['lead'], kap=R['kap'], lane_i=lane_i)
  ev = {'limit': np.asarray(sats, int), 'takeover': np.asarray(tks, int), 'lane': np.asarray(lane_i, int)}
  planner = P.Planner(mem)
  out = []
  eng_s = float(((R['en'] > .5) & (v > 8)).sum() * DT)
  runs = list(R['run_off']) if len(R['run_off']) else [0, n]
  seg_of = np.searchsorted(np.asarray(R['seg_off']), np.arange(n), side='right') - 1
  for ra, rb in zip(runs[:-1], runs[1:]):
    planner.reset()
    odo_at = {}
    for i in range(int(ra), int(rb), 4):
      t = float(R['t'][i])
      ok = bool(R['pos_ok'][i])
      fix = P.Fix(False)
      if ok:
        x, y = mem.to_local(float(R['lat'][i]), float(R['lon'][i]))
        fix = P.Fix(True, float(x), float(y), P.bearing_to_hdg(float(R['bearing_deg'][i])))
      set_mph = float(R['cr_set'][i]) / MPH if R['cr_en'][i] > .5 and R['cr_set'][i] > .5 else None
      yk = float(R['kap'][i]) if v[i] > 3 else None
      planner.step(t, fix, float(v[i]), set_mph=set_mph, yaw_k=yk)
      for e in planner.events:
        if e.get('kind') != 'start' or e['mph'] - e['T'] <= 3:
          continue
        a_ = planner.active.get(e['point'])
        if a_ is None:
          continue
        s_ent = s[i] + (a_['entry'] - planner.odo) - P.ENTRY_M
        s_end = s[i] + (a_['at'] - planner.odo)
        w = (s >= s_ent) & (s <= max(s_end, s_ent + 5)) & (np.arange(n) >= i)
        if not w.any():
          continue
        ww = np.nonzero(w)[0]
        act = float(v[ww].min() / MPH)
        lo, hi = ww[0] - int(TROUBLE_S / DT), ww[-1] + int(TROUBLE_S / DT)
        tr = [k for k, ix in ev.items() if ((ix >= lo) & (ix <= hi)).any()]
        T = int(e['T'])
        cls = 'NEEDED' if (tr or act <= T + 3) else ('UNNECESSARY' if act >= T + 5 else 'MARGINAL')
        # the limiting point: lowest need in the corner's memory stretch
        k = int(e['point'])
        t0 = int(np.searchsorted(mem.trace_start, k, side='right') - 1)
        span = int(max(1, (a_['at'] - a_['entry']) / mem.spacing)) + 2
        ks = np.arange(max(int(mem.trace_start[t0]), k - span), k + 1)
        kk = int(ks[np.argmin(planner.need[ks])])
        sg = int(seg_of[i])
        tag = str(R['seg_tags'][sg])
        sec = (i - int(R['seg_off'][sg])) * DT
        dur = float(len(ww) * DT)
        out.append(dict(drive=drive, seg=tag, sec=round(sec, 1), mph=e['mph'], T=T, act=round(act, 1), cls=cls, tr='+'.join(tr),
                        waste=max(0.0, act - T) * dur if cls == 'UNNECESSARY' else 0.0, npass=int(mem.npass[kk]),
                        vmed=float(mem.v[kk] / MPH), vpro=float(mem.v_proven[kk] / MPH), rule=rule_of(planner, mem, kk)))
      planner.events.clear()
  return drive, eng_s, out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--mem', default=f'{DATA_DIR}/curve_memory_all_v6.npz')
  ap.add_argument('--drives', nargs='*')
  ap.add_argument('--worst', type=int, default=20)
  ap.add_argument('--workers', type=int, default=6)
  ap.add_argument('--list', default='', help='also print every slowdown in these segments (prefix match)')
  a = ap.parse_args()
  drives = a.drives or sorted(os.path.basename(p)[6:-4] for p in glob.glob(f'{DATA_DIR}/cache/drive_*.npz'))
  with ProcessPoolExecutor(a.workers) as ex:
    res = list(ex.map(one, [(d, a.mem) for d in drives]))
  eng = sum(r[1] for r in res) / 3600.0
  allr = [x for r in res for x in r[2]]
  cnt = {c: sum(1 for x in allr if x['cls'] == c) for c in ('NEEDED', 'UNNECESSARY', 'MARGINAL')}
  waste = sum(x['waste'] for x in allr)
  print(f"memory {os.path.basename(a.mem)}; drives {len(drives)}; engaged {eng:.1f} h; slowdowns {len(allr)}: {cnt}")
  print(f"UNNECESSARY per engaged hour {cnt['UNNECESSARY'] / max(eng, 1e-9):.2f}; NEEDED per hour {cnt['NEEDED'] / max(eng, 1e-9):.2f}; "
        f"mph-seconds wasted {waste:.0f}")
  print("worst unnecessary (segment+s | start mph -> target | actual clean min mph | memory passes, median, best clean | rule):")
  for x in sorted([x for x in allr if x['cls'] == 'UNNECESSARY'], key=lambda x: -(x['act'] - x['T']))[:a.worst]:
    print(f"  {x['seg']}+{x['sec']:.0f}s | {x['mph']:.0f}->{x['T']} | {x['act']:.0f} | n={x['npass']} med {x['vmed']:.0f} best {x['vpro']:.0f} | {x['rule']}")
  if a.list:
    for x in allr:
      if x['seg'].startswith(a.list):
        print(f"  [{x['cls']}] {x['seg']}+{x['sec']:.0f}s | {x['mph']:.0f}->{x['T']} | act {x['act']:.0f} trouble={x['tr'] or '-'} | {x['rule']}")


if __name__ == '__main__':
  main()
