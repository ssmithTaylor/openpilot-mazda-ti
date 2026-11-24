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
GRADE_CAP_MS2 = 1.25     # ASSUMPTION: the most deceleration the cruise can produce on the flat (engine and any brake use together); with
                         # gravity along the slope taken from it, the measured 0.7 m/s^2 is unaffected within +-3 degrees (as measured)
                         # and falls short beyond about 3 degrees downhill; nothing is known beyond +-3 degrees
REACT_S = P.CM.DELAY_S          # the ONE measured cruise model (ti_cruise_model): 0.5 s dead time ...
LAG_ALPHA = 1.0 - math.exp(-DT / P.CM.TAU_S)   # ... then a 0.5 s first-order lag on the deceleration
ACCEL_MPH_S = 0.7
DRIVER_ACCEL_MPH_S = 3.0   # the recorded driver on the gas or with the cruise off (they were not reacting to this feature)


def low_priority():
  try:
    import ctypes
    ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x00004000)
  except Exception:
    pass


class SimCruise:
  def __init__(self, max_rate):
    self.max_rate = max_rate
    self.dec_f = 0.0         # the deceleration actually given (m/s^2), lagging the demand by ti_cruise_model.TAU_S
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


def replay_drive(drive, max_rate=2.0, feature_on=True):
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
    grade = R['grade'][sl].astype(float) if 'grade' in R else np.zeros(b - a)
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
    state_time = collections.Counter()
    why_ticks = collections.Counter()
    correction_taps = 0
    loss_by_state = collections.Counter()
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
        # the car side's reference, floor (while it intervenes, else the one it would take now) and whether it acts: as the daemon
        # relays them (ti_curve_plannerd.car_side)
        acting = curve.state in ("ready", "slowing", "holding", "restoring") and not curve.left_doubt
        pl = planner.step(t, fix, v_sim, set_mph=cru.set, yaw_k=float(kap[p]), model_k=None, ref_mph=curve.ref,
                          floor_mph=(curve.floor if curve.intervening() and curve.floor else curve.floor_now), acting=acting)
        if en[p] and v_sim > 9.0:
          why_ticks[pl.why] += 1
        for pev in planner.events:
          events['plan_' + pev['kind']] += 1
          if pev['kind'] == 'alert':
            events['plan_alert_' + ('behind' if pev.get('behind') else 'floor')] += 1
            if pev.get('npass') == 1:
              events['plan_alert_single'] += 1
        planner.events.clear()
        plan_times.append(time.perf_counter() - t0)
        d = P.plan_json(pl, t, seq, "act")
        d["model_k"] = float(mla[p]) / max(float(v_rec[p]), 5.0) ** 2 if np.isfinite(mla[p]) else None   # the model's sharpest bend (5 s)
        plan, plan_t, seq = d, t, seq + 1
      inp = C.Inputs(on=feature_on, engaged=bool(en[p]), cruise_on=bool(cr_en[p]) and cru.set is not None, cruise_available=True,
                     mph=v_sim / MPH, set_mph=cru.set, brake=bool(brk[p]), gas=bool(gas[p]), lead=bool(lead[p]),
                     driver_button=t < btn_until, counter=cru.counter, plan=plan,
                     plan_age=None if plan_t is None else t - plan_t, mono=t)
      btn = curve.step(k, inp)
      state_time[(curve.state, curve.why if curve.state in ('restoring', 'holding', 'stopped', 'paused', 'waiting') else '')] += DT
      if v_rec[p] > 2.0:
        loss_by_state[(curve.state, curve.why if curve.state in ('restoring', 'holding', 'stopped', 'paused', 'waiting') else '')] +=           DT * max(0.0, 1.0 - v_sim / float(v_rec[p]))
      for ev in curve.events:
        events['ev_' + ev['kind'] + ('_' + ev.get('why', '').split(' ')[0] if ev['kind'] == 'stop' else '')] += 1
      curve.events.clear()
      if btn is not None and (k - last_btn_k) > 10:
        taps[btn] += 1
        correction_taps += curve.state == "stopped"
        if btn == "SET_MINUS" and curve.state != "stopped" and (plan is None or not plan.get("trusted") or plan.get("ceil") is None):
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
          dec = min(max_rate, P.decel_mph_s(gap)) * MPH                                 # ti_cruise_model's measured table (timing)
          cru.dec_f += (dec - cru.dec_f) * LAG_ALPHA
          g = float(grade[p]) if np.isfinite(grade[p]) else 0.0
          avail = GRADE_CAP_MS2 + 9.81 * math.sin(g)                                   # downhill takes from what the cruise can do
          v_sim = max(target, v_sim - max(0.0, min(cru.dec_f, avail)) * DT)
        else:
          cru.dec_f = 0.0
          v_sim = target                                                               # the recording is slower (lead, brake): follow
      elif target - v_sim > 0.3 * MPH:
        # catching up: the cruise at its measured rate; the driver (gas, or driving with the cruise off) at DRIVER_ACCEL_MPH_S
        acc = ACCEL_MPH_S if (cr_en[p] and not gas[p] and cru.set is not None) else DRIVER_ACCEL_MPH_S
        v_sim = min(target, v_sim + acc * MPH * DT)
      else:
        v_sim = target
      s_sim += v_sim * DT
      if not hist or hist[-1][0] != p:
        hist.append((p, t, v_sim, cru.set, curve.state, plan.get("ceil") if plan else None, bool(plan.get("trusted")) if plan else False,
                     curve.shown, curve.why, curve.needs_more, float(grade[p]) if np.isfinite(grade[p]) else 0.0))
      t += DT
      k += 1
    rec_T = n * REC_DT
    runs.append(dict(run=r, rec_s=rec_T, sim_s=t, taps_down=taps["SET_MINUS"], taps_up=taps["SET_PLUS"], taps_untrusted=taps_untrusted, correction_taps=correction_taps,
                     events=dict(events), state_time={f'{a_}|{b_}': round(v_, 1) for (a_, b_), v_ in state_time.items()}, why_ticks=dict(why_ticks),
                     loss={f'{a_}|{b_}': round(v_, 2) for (a_, b_), v_ in loss_by_state.items()}))
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
        grade_deg=round(float(np.degrees(np.mean([x[10] for x in before[-60:]]))) if before else 0.0, 2),
        grade_min_deg=round(float(np.degrees(np.min([x[10] for x in before[-60:]]))) if before else 0.0, 2),
        fits_raw=bool(kc * (v_c * MPH) ** 2 <= P.envelope(v_c)), fits_raw_rec=bool(kc * (float(R['c_v_mph'][j]) * MPH) ** 2 <= P.envelope(float(R['c_v_mph'][j]))),
        fits_margin=bool(P.fits(kc, v_c)), shed=round(max(0.0, float(R['c_v_mph'][j]) - v_c), 1),
        restored_s_after=None if restored is None else round(restored[1] - tc, 1)))
  return dict(drive=drive, runs=runs, corners=out_corners, plan_ms=[float(np.mean(plan_times) * 1e3) if plan_times else None,
                                                                    float(np.percentile(plan_times, 99) * 1e3) if plan_times else None])


def _work(args):
  low_priority()
  d, rate, on, buf = args
  if buf is not None:
    P.BUFFER_S = buf
  try:
    return replay_drive(d, rate, on)
  except Exception:
    import traceback
    return dict(drive=d, error=traceback.format_exc()[-1500:])


def summarise(res):
  C_ = [c for r in res if 'corners' in r for c in r['corners']]
  sat = [c for c in C_ if c['sat'] and c['cruise']]
  acted = [c for c in sat if c['started_s_before'] is not None]
  clean = [c for c in acted if not c['paused_or_stopped_before']]
  covered = lambda c: c['trusted'] and c['shed'] >= 1.0 and c['fits_raw']
  famil = [c for c in sat if c['nprev'] >= 3]
  artefact = lambda c: c['paused_or_stopped_before']          # the recorded driver's own button / pedal: not reacting to this feature
  runs = [x for r in res if 'runs' in r for x in r['runs']]
  rec_h = sum(x['rec_s'] for x in runs) / 3600
  pct = lambda xs, q: None if not xs else round(float(np.percentile(xs, q)), 1)
  early = [c['at_reduced_s'] for c in clean if c['at_reduced_s'] is not None]
  starts = [c['started_s_before'] for c in clean]
  rest = [c['restored_s_after'] for c in acted if c['restored_s_after'] is not None]
  over = [c['over_target'] for c in clean if c['over_target'] is not None]
  st = collections.Counter()
  ev = collections.Counter()
  lo = collections.Counter()
  wt = collections.Counter()
  for x in runs:
    wt.update(x.get('why_ticks', {}))
  nwt = max(1, sum(wt.values()))
  for x in runs:
    for k_, v_ in x.get('loss', {}).items():
      lo[k_] += v_
    ev.update(x['events'])
    for k_, v_ in x['state_time'].items():
      st[k_] += v_
  steep = [c for c in clean if c['grade_min_deg'] <= -3.0 or c['grade_min_deg'] >= 3.0 or abs(c['grade_deg']) >= 2.0]

  def reason(c):
    if covered(c):
      return 'covered'
    if not c['trusted']:
      return 'road not trusted at the corner (unknown / trust not yet established)'
    if artefact(c):
      return "recording artefact: the recorded driver's own button / pedal / stop"
    if c['started_s_before'] is None:
      return 'not started'
    if c['needs_more']:
      return 'needs more than 15 mph below (designed limit)'
    if (c['over_target'] or 0) > 1.0:
      return 'arrived late'
    return 'slowed to target but still outside the raw envelope (memory understated or target too high)'
  return dict(
    drives=len([r for r in res if 'runs' in r]), errors=[r['drive'] for r in res if 'error' in r], recorded_hours=round(rec_h, 2),
    saturating_cruise_corners=len(sat), slowing_started_for=len(acted), uninterrupted_interventions=len(clean),
    covered=sum(covered(c) for c in sat), covered_excluding_artefacts=f"{sum(covered(c) for c in sat if not artefact(c))} of {sum(not artefact(c) for c in sat)}",
    covered_familiar=f"{sum(covered(c) for c in famil)} of {len(famil)}",
    covered_familiar_excluding_artefacts=f"{sum(covered(c) for c in famil if not artefact(c))} of {sum(not artefact(c) for c in famil)}",
    inside_envelope_without_feature=sum(c['fits_raw_rec'] for c in sat),
    reasons_all=dict(collections.Counter(reason(c) for c in sat)), reasons_familiar=dict(collections.Counter(reason(c) for c in famil)),
    arrival_within_1mph=f"{sum(1 for x in over if x <= 1.0)} of {len(over)} uninterrupted", arrival_over_target_mph=dict(p50=pct(over, 50), p90=pct(over, 90), max=pct(over, 100)),
    seconds_started_before_peak=dict(p10=pct(starts, 10), p50=pct(starts, 50), p90=pct(starts, 90)),
    seconds_at_reduced_speed_before_peak=dict(p10=pct(early, 10), p50=pct(early, 50), p90=pct(early, 90), n=len(early)),
    restored_seconds_after_peak=dict(p50=pct(rest, 50), p90=pct(rest, 90), n=len(rest)),
    mph_shed_at_corner=dict(p50=pct([c['shed'] for c in acted], 50), p90=pct([c['shed'] for c in acted], 90)),
    time_cost_s_per_hour=round(sum(x['sim_s'] - x['rec_s'] for x in runs) / max(rec_h, 1e-9), 1),
    taps_down_per_hour=round(sum(x['taps_down'] for x in runs) / max(rec_h, 1e-9), 1),
    taps_up_per_hour=round(sum(x['taps_up'] for x in runs) / max(rec_h, 1e-9), 1),
    interventions_per_hour=round(ev.get('ev_start', 0) / max(rec_h, 1e-9), 1),
    planner_state_share_engaged={k_: round(v_ / nwt, 3) for k_, v_ in wt.most_common(8)},
    matched_share_engaged=round(sum(v_ for k_, v_ in wt.items() if k_ not in ("road not known here", "matching the remembered road",
                                    "road differs from memory", "position uncertain")) / nwt, 3),
    trusted_share_of_known_road=round(sum(v_ for k_, v_ in wt.items() if k_ not in ("road not known here", "matching the remembered road",
                                      "road differs from memory", "position uncertain")) / max(1, nwt - wt.get("road not known here", 0)), 3),
    warnings_per_hour=round(ev.get('plan_alert', 0) / max(rec_h, 1e-9), 2),
    chose_not_to_slow_per_hour=round(ev.get('plan_not_slowing', 0) / max(rec_h, 1e-9), 1),
    minus_taps_without_trusted_ceiling=sum(x['taps_untrusted'] for x in runs),
    correction_taps_down_to_reference=sum(x.get('correction_taps', 0) for x in runs),
    time_lost_s_per_hour_by_state={k_: round(v_ / max(rec_h, 1e-9), 1) for k_, v_ in lo.most_common(12)},
    seconds_by_state={k_: round(v_) for k_, v_ in st.most_common(14) if not k_.startswith('ready|') and not k_.startswith('off|')},
    steep_approaches=[dict(grade=c['grade_deg'], grade_min=c['grade_min_deg'], over=c['over_target']) for c in steep],
    events=dict(ev),
    planner_ms_mean=round(float(np.mean([r['plan_ms'][0] for r in res if r.get('plan_ms') and r['plan_ms'][0]])), 3),
    planner_ms_p99_max=round(float(np.max([r['plan_ms'][1] for r in res if r.get('plan_ms') and r['plan_ms'][1]])), 3))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--drives', nargs='*')
  ap.add_argument('--workers', type=int, default=8)
  ap.add_argument('--out', default='E:/openpilot-mazda-ti-data/curve-slowing/replay')
  ap.add_argument('--max-rate', type=float, default=2.0)
  ap.add_argument('--off', action='store_true', help='feature OFF: asserts nothing is ever sent')
  ap.add_argument('--buffer', type=float, default=None, help='planner BUFFER_S override (the buffer sweep)')
  a = ap.parse_args()
  drives = a.drives or sorted(os.path.basename(p)[6:-4] for p in glob.glob(f'{DATA_DIR}/cache/drive_*.npz'))
  drives = [d for d in drives if os.path.exists(f'{DATA_DIR}/loo/{d}.npz')]     # caches of every drive exist; LOO only for the gate set
  os.makedirs(a.out, exist_ok=True)
  from multiprocessing import Pool
  t0 = time.time()
  with Pool(a.workers) as pool:
    res = pool.map(_work, [(d, a.max_rate, not a.off, a.buffer) for d in drives], chunksize=1)
  s = summarise(res)
  s['wall_s'] = round(time.time() - t0)
  s['max_rate'] = a.max_rate
  s['feature_on'] = not a.off
  tag = ('off' if a.off else 'on') + f'_r{a.max_rate}' + ('' if a.buffer is None else f'_b{a.buffer}')
  s['buffer_s'] = P.BUFFER_S if a.buffer is None else a.buffer
  json.dump(res, open(f'{a.out}/replay_{tag}.json', 'w'), indent=0)       # position-free
  json.dump(s, open(f'{a.out}/summary_{tag}.json', 'w'), indent=1)
  print(json.dumps(s, indent=1))


if __name__ == '__main__':
  main()
