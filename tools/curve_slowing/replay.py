"""Closed-loop leave-one-out replay of curve slowing over recorded drives (off the car).

For every drive: the planner (ti_curve_plan.Planner, 5 Hz) runs on that drive's recorded location stream with a memory built WITHOUT
that drive (memory/loo/<drive>.npz); the car side (ti_curve.Curve, 100 Hz, the on-car code) taps a SIMULATED stock cruise; the
simulated car follows the recorded speed except where the lowered set speed holds it lower. Because the car may be slower than
recorded, the recording is indexed by distance driven (not time): position, curvature, lead, pedals and the driving model's preview
are taken at the recorded sample where the recorded distance equals the simulated distance (by time while stopped).

Simulated cruise (cruise-slowing-20261122): a set-speed tap is read back 0.08 s after its first frame; the speed responds 0.5 s after
the set speed changes, slowing at min(MAX_RATE, 0.6 + 0.25 * gap) mph/s (5 mph in ~5.1 s, 10 in ~8.5, 15 in ~11.8 from the press;
measured 4.4, 7.5-9, 12-14), speeding back up at 0.7 mph/s (5 mph in ~7 s; measured 7-9). The driver's own set-speed changes in the
recording are replayed as a cruise button press. No speed limit in the caches: the floor is (speed at the start - 15 mph).

Output is POSITION-FREE: per corner (the remembered-roads study's corner ids) and per drive, times, speeds, counts. The private caches
and memories are read from DATA_DIR (positions stay there and in memory).

  python tools/curve_slowing/replay.py [--drives D ...] [--workers 8] [--out DIR] [--max-rate 1.5]
"""
import argparse
import collections
import glob
import json
import math
import os
import sys
import time

import numpy as np

try:
  from openpilot.selfdrive.car.mazda import ti_curve as C, ti_curve_memory as M, ti_curve_plan as P
except ImportError:
  sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
  from selfdrive.car.mazda import ti_curve as C, ti_curve_memory as M, ti_curve_plan as P   # type: ignore

DATA_DIR = 'E:/openpilot-mazda-ti-data/curve-slowing/memory'
MPH = 0.44704
DT = 0.01
REC_DT = 0.05
READ_S = 0.08
REACT_S = 0.5
ACCEL_MPH_S = 0.7


def low_priority():
  try:
    import ctypes
    ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x00004000)
  except Exception:
    pass


class SimCruise:
  def __init__(self, max_rate):
    self.max_rate = max_rate
    self.counter = 0
    self.set = None          # the car's set speed (mph) as read
    self.eff = None          # the set speed the speed controller acts on (REACT_S after a change)
    self.pending = []        # (time, new set)
    self.eff_pending = []
    self.last_press = -1e9
    self.press_start = -1e9

  def change_set(self, t, new):
    self.set = new
    self.eff_pending.append((t + REACT_S, new))

  def tick(self, t, k):
    if k % 10 == 3:
      self.counter = (self.counter + 1) % 16
    for p in [p for p in self.pending if p[0] <= t]:
      self.pending.remove(p)
      self.change_set(t, p[1])
    for p in [p for p in self.eff_pending if p[0] <= t]:
      self.eff_pending.remove(p)
      self.eff = p[1]

  def press(self, t, button):
    new = t - self.last_press > 0.15
    self.last_press = t
    if new:
      self.press_start = t
      if self.set is not None:
        self.pending.append((t + READ_S, self.set + (-1 if button == "SET_MINUS" else 1)))
    elif t - self.press_start >= 0.52:
      raise AssertionError("a press was held long enough to be read as a hold")


def replay_drive(drive, max_rate=1.5, feature_on=True):
  cache = np.load(f'{DATA_DIR}/cache/drive_{drive}.npz', allow_pickle=False)
  mem = M.load(f'{DATA_DIR}/loo/{drive}.npz')
  R = {k: cache[k] for k in cache.files}
  run_off = R['run_off']
  corners = collections.defaultdict(list)
  for j in range(len(R['c_id'])):
    corners[int(R['c_run'][j])].append(j)
  out_corners, runs = [], []
  plan_times = []
  for r in range(len(run_off) - 1):
    a, b = int(run_off[r]), int(run_off[r + 1])
    if b - a < 200:
      continue
    sl = slice(a, b)
    v_rec = R['v'][sl].astype(float)
    S = np.r_[0.0, np.cumsum(v_rec[:-1] * REC_DT)]
    lat, lon, pos_ok = R['lat'][sl], R['lon'][sl], R['pos_ok'][sl].astype(bool)
    brg, kap = R['bearing_deg'][sl].astype(float), R['kap'][sl].astype(float)
    cr_en, cr_set = R['cr_en'][sl] > 0.5, R['cr_set'][sl].astype(float)
    lead, en, brk, gas = R['lead'][sl] > 0.5, R['en'][sl] > 0.5, R['brk'][sl] > 0.5, R['gas'][sl] > 0.5
    mla = R['model_la5'][sl].astype(float)
    n = b - a
    xs, ys = mem.to_local(lat, lon)
    rec_set_mph = np.where(cr_set > 0.5, np.round(cr_set / MPH), np.nan)
    planner = P.Planner(mem)
    curve = C.Curve()
    cru = SimCruise(max_rate)
    p = 0
    s_sim, v_sim = 0.0, float(v_rec[0])
    t, k = 0.0, 0
    plan, plan_t = None, None
    last_rec_set = rec_set_mph[0]
    btn_until = -1.0
    cr_was = False
    hist = []          # per recorded index passed: (p, t, v_sim, set, state, ceil, trusted)
    taps = {"SET_MINUS": 0, "SET_PLUS": 0}
    taps_untrusted = 0
    events = collections.Counter()
    seq = 0
    last_btn_k = -10 ** 9
    while p < n - 1:
      # recorded index for the simulated distance (by time while stopped)
      if v_rec[p] < 0.5 and v_sim < 0.5:
        if t >= (hist[-1][1] if hist else 0) + REC_DT - 1e-9 or not hist:
          p += 1
          s_sim = S[p]
      else:
        p2 = int(np.searchsorted(S, s_sim, side='right') - 1)
        p = max(p, min(n - 1, p2))
      # the driver's cruise and set speed, from the recording
      if cr_en[p] and not cr_was:
        cru.set = int(rec_set_mph[p]) if np.isfinite(rec_set_mph[p]) else int(round(v_sim / MPH))
        cru.eff = cru.set
        cru.pending.clear()
        cru.eff_pending.clear()
        last_rec_set = rec_set_mph[p]
      if not cr_en[p]:
        cru.set = cru.eff = None
      cr_was = bool(cr_en[p])
      if cr_en[p] and np.isfinite(rec_set_mph[p]) and np.isfinite(last_rec_set) and rec_set_mph[p] != last_rec_set:
        cru.change_set(t, int(rec_set_mph[p]))          # the driver's own press, as recorded
        btn_until = t + 0.2
        events['driver_set_changes'] += 1
      last_rec_set = rec_set_mph[p]
      cru.tick(t, k)
      # the planner, 5 Hz
      if k % 20 == 0:
        fix = P.Fix(bool(pos_ok[p]) and np.isfinite(xs[p]), float(xs[p]) if pos_ok[p] else 0.0, float(ys[p]) if pos_ok[p] else 0.0,
                    P.bearing_to_hdg(float(brg[p])) if np.isfinite(brg[p]) else 0.0)
        t0 = time.perf_counter()
        pl = planner.step(t, fix, v_sim, set_mph=cru.set, yaw_k=float(kap[p]), model_k=None)
        plan_times.append(time.perf_counter() - t0)
        d = P.plan_json(pl, t, seq, "act")
        d["model_la"] = float(mla[p]) if np.isfinite(mla[p]) else None    # the model's preview (5 s) for the restore check
        plan, plan_t, seq = d, t, seq + 1
      inp = C.Inputs(on=feature_on, engaged=bool(en[p]), cruise_on=bool(cr_en[p]) and cru.set is not None, cruise_available=True,
                     mph=v_sim / MPH, set_mph=cru.set, brake=bool(brk[p]), gas=bool(gas[p]), lead=bool(lead[p]),
                     driver_button=t < btn_until, counter=cru.counter, plan=plan,
                     plan_age=None if plan_t is None else t - plan_t, mono=t)
      btn = curve.step(k, inp)
      for ev in curve.events:
        events['ev_' + ev['kind'] + ('_' + ev.get('why', '').split(' ')[0] if ev['kind'] == 'stop' else '')] += 1
      curve.events.clear()
      if btn is not None and (k - last_btn_k) > 10:
        taps[btn] += 1
        if btn == "SET_MINUS" and (plan is None or not plan.get("trusted") or plan.get("ceil") is None):
          taps_untrusted += 1
      if btn is not None:
        last_btn_k = k
        if not feature_on:
          raise AssertionError("a press with the feature off")
        cru.press(t, btn)
      # the simulated car
      vr = float(v_rec[p])
      cap = cru.eff * MPH if (cru.eff is not None and cr_en[p] and np.isfinite(rec_set_mph[p]) and cru.eff < rec_set_mph[p] - 0.01) else None
      target = vr if cap is None else min(vr, cap)
      if v_sim > target:
        if cap is not None and target == cap:
          gap = (v_sim - target) / MPH
          v_sim = max(target, v_sim - min(max_rate, 0.6 + 0.25 * gap) * MPH * DT)     # the cruise slowing to the lowered set speed
        else:
          v_sim = target                                                               # the recording is slower (lead, brake): follow
      elif target - v_sim > 0.3 * MPH:
        v_sim = min(target, v_sim + ACCEL_MPH_S * MPH * DT)                            # catching up after a release
      else:
        v_sim = target
      s_sim += v_sim * DT
      if not hist or hist[-1][0] != p:
        hist.append((p, t, v_sim, cru.set, curve.state, plan.get("ceil") if plan else None, bool(plan.get("trusted")) if plan else False,
                     curve.target, curve.why, curve.needs_more))
      t += DT
      k += 1
    rec_T = n * REC_DT
    runs.append(dict(run=r, rec_s=rec_T, sim_s=t, taps_down=taps["SET_MINUS"], taps_up=taps["SET_PLUS"], taps_untrusted=taps_untrusted,
                     events=dict(events)))
    # corners
    H = {h[0]: h for h in hist}
    hp = np.array([h[0] for h in hist])
    for j in corners.get(r, []):
      ci = int(R['c_i20'][j])
      if ci >= n:
        continue
      q = int(hp[np.searchsorted(hp, ci).clip(0, len(hp) - 1)])
      h = H[q]
      v_c = h[2] / MPH
      kc = abs(float(R['c_kap'][j]))
      tc = h[1]
      # slowing start: first index before the corner, within 60 s, where the feature was slowing
      before = [x for x in hist if tc - 60 <= x[1] <= tc]
      started = next((x for x in before if x[4] == 'slowing'), None)
      tgt = next((x[7] for x in reversed(before) if x[7] is not None and x[4] in ('slowing', 'holding')), None)
      at_red = None
      if started is not None and tgt is not None:
        reached = next((x for x in before if x[1] >= started[1] and x[2] / MPH <= tgt + 1.0), None)
        at_red = None if reached is None else tc - reached[1]
      after = [x for x in hist if x[1] >= tc and x[1] <= tc + 120]
      restored = next((x for x in after if x[4] == 'ready' and x[1] > tc), None)
      out_corners.append(dict(
        c_id=str(R['c_id'][j]), sat=int(R['c_sat'][j]), cruise=int(R['c_cruise'][j]), nprev=int(R['c_nprev'][j]),
        v_rec=float(R['c_v_mph'][j]), v_sim=round(v_c, 1), trusted=h[6], ceil=h[5], target=tgt,
        started_s_before=None if started is None else round(tc - started[1], 1),
        at_reduced_s=None if at_red is None else round(at_red, 1),
        late=bool(started is not None and tgt is not None and v_c > tgt + 1.0),
        over_target=None if tgt is None else round(v_c - tgt, 1), state_at=h[4], why_at=h[8], needs_more=bool(h[9]),
        paused_or_stopped_before=any(x[4] in ('paused', 'stopped') for x in before),
        fits_raw=bool(kc * (v_c * MPH) ** 2 <= P.envelope(v_c)), fits_raw_rec=bool(kc * (float(R['c_v_mph'][j]) * MPH) ** 2 <= P.envelope(float(R['c_v_mph'][j]))),
        fits_margin=bool(P.fits(kc, v_c)), shed=round(max(0.0, float(R['c_v_mph'][j]) - v_c), 1),
        restored_s_after=None if restored is None else round(restored[1] - tc, 1)))
  return dict(drive=drive, runs=runs, corners=out_corners, plan_ms=[float(np.mean(plan_times) * 1e3) if plan_times else None,
                                                                    float(np.percentile(plan_times, 99) * 1e3) if plan_times else None])


def _work(args):
  low_priority()
  d, rate, on = args
  try:
    return replay_drive(d, rate, on)
  except Exception:
    import traceback
    return dict(drive=d, error=traceback.format_exc()[-1500:])


def summarise(res):
  C_ = [c for r in res if 'corners' in r for c in r['corners']]
  sat = [c for c in C_ if c['sat'] and c['cruise']]
  known = [c for c in sat if c['trusted']]
  acted = [c for c in sat if c['started_s_before'] is not None]
  cov = [c for c in sat if c['trusted'] and c['shed'] >= 1.0 and c['fits_raw']]
  famil = [c for c in sat if c['nprev'] >= 3]
  cov_f = [c for c in famil if c in cov]
  runs = [x for r in res if 'runs' in r for x in r['runs']]
  rec_h = sum(x['rec_s'] for x in runs) / 3600
  pct = lambda xs, q: None if not xs else round(float(np.percentile(xs, q)), 1)
  early = [c['at_reduced_s'] for c in acted if c['at_reduced_s'] is not None]
  starts = [c['started_s_before'] for c in acted]
  rest = [c['restored_s_after'] for c in acted if c['restored_s_after'] is not None]
  ev = collections.Counter()
  for x in runs:
    ev.update(x['events'])
  return dict(
    drives=len([r for r in res if 'runs' in r]), errors=[r['drive'] for r in res if 'error' in r], recorded_hours=round(rec_h, 2),
    saturating_cruise_corners=len(sat), on_trusted_known_road_at_corner=len(known), slowing_started_for=len(acted),
    covered_inside_envelope=len(cov), familiar_3plus=len(famil), covered_familiar=len(cov_f),
    inside_envelope_without_feature=sum(c['fits_raw_rec'] for c in sat),
    late_arrivals=sum(c['late'] for c in acted), late_over_2mph=sum(1 for c in acted if (c['over_target'] or 0) > 2.0),
    over_target_mph=dict(p50=pct([c['over_target'] for c in acted if c['over_target'] is not None], 50),
                         p90=pct([c['over_target'] for c in acted if c['over_target'] is not None], 90)),
    not_covered_reasons=dict(collections.Counter(
      'not on trusted known road' if not c['trusted'] else 'driver button / pedal / stop before it' if c['paused_or_stopped_before']
      else 'needs more than 15 mph below' if c['needs_more'] else 'not started' if c['started_s_before'] is None
      else 'still too fast for the envelope' for c in sat if not (c['trusted'] and c['shed'] >= 1.0 and c['fits_raw']))), seconds_started_before=dict(p10=pct(starts, 10), p50=pct(starts, 50), p90=pct(starts, 90)),
    seconds_at_reduced_speed_before_corner=dict(p10=pct(early, 10), p50=pct(early, 50), p90=pct(early, 90), n=len(early)),
    restored_seconds_after_corner=dict(p50=pct(rest, 50), p90=pct(rest, 90), n=len(rest)),
    mph_shed_at_corner=dict(p50=pct([c['shed'] for c in acted], 50), p90=pct([c['shed'] for c in acted], 90)),
    time_cost_s=round(sum(x['sim_s'] - x['rec_s'] for x in runs), 1),
    time_cost_s_per_hour=round(sum(x['sim_s'] - x['rec_s'] for x in runs) / max(rec_h, 1e-9), 1),
    taps_down=sum(x['taps_down'] for x in runs), taps_up=sum(x['taps_up'] for x in runs),
    taps_down_per_hour=round(sum(x['taps_down'] for x in runs) / max(rec_h, 1e-9), 1),
    minus_taps_without_trusted_ceiling=sum(x['taps_untrusted'] for x in runs), events=dict(ev),
    planner_ms_mean=round(float(np.mean([r['plan_ms'][0] for r in res if r.get('plan_ms') and r['plan_ms'][0]])), 3),
    planner_ms_p99_max=round(float(np.max([r['plan_ms'][1] for r in res if r.get('plan_ms') and r['plan_ms'][1]])), 3))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--drives', nargs='*')
  ap.add_argument('--workers', type=int, default=8)
  ap.add_argument('--out', default='E:/openpilot-mazda-ti-data/curve-slowing/replay')
  ap.add_argument('--max-rate', type=float, default=1.5)
  ap.add_argument('--off', action='store_true', help='feature OFF: asserts nothing is ever sent')
  a = ap.parse_args()
  drives = a.drives or sorted(os.path.basename(p)[6:-4] for p in glob.glob(f'{DATA_DIR}/cache/drive_*.npz'))
  os.makedirs(a.out, exist_ok=True)
  from multiprocessing import Pool
  t0 = time.time()
  with Pool(a.workers) as pool:
    res = pool.map(_work, [(d, a.max_rate, not a.off) for d in drives], chunksize=1)
  s = summarise(res)
  s['wall_s'] = round(time.time() - t0)
  s['max_rate'] = a.max_rate
  s['feature_on'] = not a.off
  tag = ('off' if a.off else 'on') + f'_r{a.max_rate}'
  json.dump(res, open(f'{a.out}/replay_{tag}.json', 'w'), indent=0)       # position-free
  json.dump(s, open(f'{a.out}/summary_{tag}.json', 'w'), indent=1)
  print(json.dumps(s, indent=1))


if __name__ == '__main__':
  main()
