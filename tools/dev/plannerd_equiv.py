#!/usr/bin/env python3
"""Equivalence of ti_curve_plannerd's per-tick helpers before / after a cost change (perf 2026-11-25): map_profile (incl. its parse
cache, fed repeated, alternating and malformed lists at moving positions), can_reads against yaw_from_can / lat_accel_from_can /
ti_from_can, and tick_log on a planner run along a road memory's traces. Prints counts of differing results only (no positions).

  python tools/dev/plannerd_equiv.py <old ti_curve_plannerd.py> <new ti_curve_plannerd.py> [<memory.npz>]
"""
import importlib.util
import json
import math
import random
import sys
from types import SimpleNamespace as NS


def load(path, name):
  sp = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(sp)
  sp.loader.exec_module(m)
  return m


def same(a, b):
  """Exact equality, NaN equal to NaN (bit-identical floats otherwise)."""
  if isinstance(a, float) and isinstance(b, float):
    return (math.isnan(a) and math.isnan(b)) or (a == b and math.copysign(1, a) == math.copysign(1, b))
  if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
    return type(a) is type(b) and len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
  if isinstance(a, dict) and isinstance(b, dict):
    return list(a) == list(b) and all(same(a[k], b[k]) for k in a)
  return type(a) is type(b) and a == b


def polyline(rng, n, step):
  pts, lat, lon, h = [], rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(0, 6.28)
  for i in range(n):
    h += rng.gauss(0, 0.08)
    lat += step * math.cos(h) / 111195.0
    lon += step * math.sin(h) / 111195.0
    pts.append({"latitude": lat, "longitude": lon, "curvature": rng.choice([1, -1]) * abs(rng.gauss(0, 0.003))})
  return pts


def raws(rng):
  out = []
  for _ in range(60):
    p = polyline(rng, rng.choice([2, 3, 10, 40, 150, 300]), rng.choice([5.0, 25.0, 60.0]))
    out.append((json.dumps(p), p))
  bad = ['', 'null', '[]', '[{"latitude": 1}]', '{"a": 1}', 'not json', '[1, 2]',
         json.dumps([{"latitude": 0.0, "longitude": 0.0, "curvature": float("nan")}] * 3),
         json.dumps([{"latitude": "x", "longitude": 0.0, "curvature": 0.0}] * 3),
         json.dumps([{"latitude": 0.0, "longitude": 0.0, "curvature": None}] * 3)]
  return out, bad


def check_map(O, N, rng):
  good, bad = raws(rng)
  n = diff = 0
  seq = []
  for raw, p in good:
    for _ in range(rng.choice([1, 2, 5, 8])):          # the same text several ticks running, the car moving along it
      seq.append((raw, p))
    if rng.random() < 0.3:
      seq.append((rng.choice(bad), None))
    if rng.random() < 0.3:
      seq.append(rng.choice(good))                       # an earlier list again (alternating)
  for raw, p in seq:
    for _ in range(3):
      brg = rng.uniform(0, 360)
      if p:
        j = rng.randrange(len(p) - 1)
        q, r = p[j], p[j + 1]
        lat, lon = q["latitude"] + rng.gauss(0, 1e-4), q["longitude"] + rng.gauss(0, 1e-4)
        if rng.random() < 0.8:                           # mostly along the road (a match), sometimes any way
          brg = math.degrees(math.atan2(r["longitude"] - q["longitude"], r["latitude"] - q["latitude"])) + rng.gauss(0, 25)
      else:
        lat, lon = rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01)
      age = rng.choice([0.2, 1.0, 2.9, 3.5, None, -0.1])
      for a in ((raw, age, lat, lon, brg), (None, age, lat, lon, brg), (raw, age, None, lon, brg)):
        n += 1
        diff += not same(O.map_profile(*a), N.map_profile(*a))
  return n, diff


def check_can(O, N, rng):
  n = diff = 0
  if not hasattr(N, "can_reads"):
    return 0, 0
  for _ in range(20000):
    frames = []
    for _ in range(rng.choice([0, 1, 5, 30, 80])):
      frames.append(NS(address=rng.choice([0x75, 0x75, 0x24A, 0x24A, 0x100, 0x21B]), src=rng.choice([0, 1, 2]),
                       dat=bytes(rng.randrange(256) for _ in range(rng.choice([0, 3, 4, 5, 6, 8])))))
    a = (O.yaw_from_can(frames), O.lat_accel_from_can(frames), O.ti_from_can(frames))
    n += 1
    diff += not same(a, N.can_reads(frames))
  return n, diff


def check_ticklog(O, N, mem_path):
  if not mem_path:
    return 0, 0
  import numpy as np
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P
  mem = M.load(mem_path)
  km = np.asarray(mem.kap_med, np.float64)
  pl = P.Planner(mem)
  n = diff = 0
  t = 0.0
  for tr in np.argsort(-np.diff(mem.trace_start))[:2]:
    s0, s1 = int(mem.trace_start[tr]), int(mem.trace_start[tr + 1])
    pl.reset()
    q = float(s0)
    while q < s1 - 1 and n < 6000:
      i = int(q)
      ahead = [10.0 * (k / 32) ** 2 for k in range(33)]
      kk = [P.MODEL_K_SIGN * float(km[min(i + int(x * 25.0 / mem.spacing), s1 - 1)]) for x in ahead]
      p = pl.step(t, P.Fix(True, float(mem.x[i]), float(mem.y[i]), float(mem.hdg[i])), 25.0, set_mph=55, yaw_k=float(km[i]),
                  model_k=(ahead, kk), ref_mph=55, floor_mph=40, acting=True, map_k=None, blinker=False)
      s1_, s2_, side = O.stage_summary(pl)
      ex = dict(seq=n, v=55.0 + (n % 7) * 0.123, set=55, gap=0.31, a=-0.2, dec=0.1, mem=O.mem_summary(pl), fc=O.fc_summary(pl), s1=s1_,
                s2=s2_, side=side, mon=["x" * (n % 50), 3] if n % 3 else None, car=["slowing" * (n % 9), 40, 1, 0], lay=0.1, lacc=None,
                des=float("nan") if n % 11 == 0 else 0.2, rate=1.0, cmd=0, ti=[3, 0], ln=[0.5, 0.9, 0.5, 0.9], drv=n % 64, tq=True,
                cpu=5000, mdl="m" * (n % 45), step_us=900, lacc_ms=0, ti_ms=None)
      if n % 5 == 0:
        ex["map"] = [[k * 40.0, 0.002 * k] for k in range(n % 20)]
      n += 1
      diff += not same(O.tick_log(p, ex), N.tick_log(p, ex))
      q += 25.0 * 0.2 / mem.spacing
      t += 0.2
  return n, diff


def main():
  O, N = load(sys.argv[1], "plannerd_old"), load(sys.argv[2], "plannerd_new")
  rng = random.Random(20261125)
  total = 0
  for name, fn in (("map_profile", lambda: check_map(O, N, rng)), ("can_reads", lambda: check_can(O, N, rng)),
                   ("tick_log", lambda: check_ticklog(O, N, sys.argv[3] if len(sys.argv) > 3 else None))):
    n, d = fn()
    total += d
    print(f"{name}: {n} cases, {d} differ")
  sys.exit(1 if total else 0)


if __name__ == "__main__":
  main()
