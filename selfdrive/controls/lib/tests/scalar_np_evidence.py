#!/usr/bin/env python3
"""Evidence for selfdrive/controls/lib/scalar_np.py: its clip / sign / Interp against numpy's np.clip / np.sign / np.interp, bit for
bit, over edge cases and N random cases each (default 1.2 million), with the call sites' own tables and bounds plus random ones.

    python3 scalar_np_evidence.py [N] [path/to/scalar_np.py]

Run it with the device's numpy (1.26.4; e.g. in an aarch64 python:3.11 container with numpy==1.26.4) -- that is the reference.
Prints one JSON line: numpy version, machine, cases per function, mismatches (with the first few), and the numpy result types.
Exit 0 when nothing differs. Standalone: needs only numpy and the module file.
"""
import importlib.util
import json
import math
import platform
import random
import struct
import sys

import numpy as np

N = int(sys.argv[1]) if len(sys.argv) > 1 else 1_200_000
path = sys.argv[2] if len(sys.argv) > 2 else __file__.replace("tests/scalar_np_evidence.py", "scalar_np.py")
spec = importlib.util.spec_from_file_location("scalar_np", path)
snp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(snp)

V_BP = [7.0, 12.5, 15.0, 17.5, 20.0, 25.0, 30.0, 35.0]
Q_BP = [2.79e-6, 3.14e-6, 3.49e-6, 4.00e-6, 4.51e-6, 4.96e-6, 4.96e-6, 4.96e-6]
GS_BP = [1.21e-3, 1.21e-3, 1.31e-3, 1.88e-3, 2.14e-3, 2.34e-3, 2.41e-3, 2.41e-3]
LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]
TABLES = [(V_BP, Q_BP), (V_BP, GS_BP), (LOW_SPEED_X, LOW_SPEED_Y)]


def bits(v):
  return struct.pack("<d", float(v)).hex()          # every bit, NaN payload and sign included


def same(a, b):
  return bits(a) == bits(b)


def specials():
  out = [0.0, -0.0, 1.0, -1.0, 0.5, -0.5, float("inf"), float("-inf"), float("nan"), 1e-300, -1e-300, 5e-324, -5e-324,
         1e300, -1e300, 2.0 ** 53, 0.1, 1e-9, 600.0, -600.0, 1200.0, 100.0, 0.75, 1.25, 0.6, -0.6]
  return out


def neighbours(v):
  return [v, math.nextafter(v, math.inf), math.nextafter(v, -math.inf)] if math.isfinite(v) else [v]


def rnd_float(r):
  k = r.random()
  if k < 0.4:
    return r.uniform(-700.0, 700.0)
  if k < 0.6:
    return r.uniform(-2.0, 2.0)
  if k < 0.75:
    return math.copysign(10.0 ** r.uniform(-12, 6), r.random() - 0.5)
  if k < 0.85:
    return struct.unpack("<d", struct.pack("<Q", r.getrandbits(64)))[0]       # any bit pattern: NaNs, infinities, subnormals
  if k < 0.9:
    return float(r.randint(-1000, 1000))
  return r.choice(specials())


def check_clip(r, n):
  bad, types = [], set()
  bounds = [(-1.0, 1.0), (0.0, 1.0), (-0.6, 0.6), (1, 100), (0.75, 1.25), (100.0, 1200.0), (0.0, 600.0), (-600.0, 600.0),
            (-0.0, 0.0), (0.0, 0.0), (-0.0, 1.0), (0.0, -0.0)]
  cases = []
  for lo, hi in bounds:
    for x in specials() + [v for b in (lo, hi) for v in neighbours(float(b))] + [lo, hi, 0, 1, -1, 50, np.float32(0.3), np.float32(-0.0)]:
      cases.append((x, lo, hi))
  for _ in range(n):
    k = r.random()
    if k < 0.5:
      lo, hi = r.choice(bounds)
    else:
      a, b = rnd_float(r), rnd_float(r)
      if not (a <= b):
        a, b = b, a
      if a != a or b != b:
        a, b = -1.0, 1.0
      lo, hi = a, b
    x = rnd_float(r)
    if r.random() < 0.05:
      x = r.choice([lo, hi, -0.0, 0.0])
    if r.random() < 0.02:
      x = np.float32(x) if math.isfinite(x) and abs(x) < 3e38 else x
    if r.random() < 0.02 and math.isfinite(x) and abs(x) < 1e15:
      x = int(x)
    cases.append((x, lo, hi))
  for x, lo, hi in cases:
    want = np.clip(x, lo, hi)
    types.add(type(want).__name__)
    got = snp.clip(x, lo, hi)
    if not same(got, want):
      bad.append([repr(x), repr(lo), repr(hi), bits(want), bits(got)])
  return len(cases), bad, sorted(types)


def check_sign(r, n):
  bad, types = [], set()
  cases = specials() + [0, 1, -3, np.float32(-0.0), np.float32(2.5)] + [rnd_float(r) for _ in range(n)]
  for x in cases:
    want = np.sign(x)
    types.add(type(want).__name__)
    got = snp.sign(x)
    if not same(got, want):
      bad.append([repr(x), bits(want), bits(got)])
  return len(cases), bad, sorted(types)


def check_interp(r, n):
  bad, types = [], set()
  tables = list(TABLES)
  for _ in range(40):                                  # random tables: increasing breakpoints, any values (repeats, zero, huge)
    m = r.randint(2, 12)
    xs = sorted({round(r.uniform(-50, 50), r.randint(0, 6)) for _ in range(m)} | {r.uniform(-50, 50)})
    if len(xs) < 2:
      continue
    ys = [r.choice([rnd_float(r), r.uniform(-5, 5), 0.0, 1e308, -1e308, float("inf")]) for _ in xs]
    tables.append((xs, ys))
  fns = [(xp, fp, snp.Interp(xp, fp)) for xp, fp in tables]
  count = 0
  for xp, fp, f in fns:
    edge = specials() + [v for b in xp for v in neighbours(float(b))] + [int(xp[0]), np.float32(xp[-1])]
    for x in edge:
      want = np.interp(x, xp, fp)
      types.add(type(want).__name__)
      if not same(f(x), want):
        bad.append([repr(x), repr(xp)[:60], bits(want), bits(f(x))])
      count += 1
  for _ in range(n):
    xp, fp, f = fns[r.randrange(3)] if r.random() < 0.7 else r.choice(fns)
    k = r.random()
    if k < 0.8:
      x = r.uniform(float(xp[0]) - 5.0, float(xp[-1]) + 5.0)
    elif k < 0.9:
      x = float(r.choice(xp)) + r.choice([0.0, 1e-12, -1e-12, 1e-6])
    else:
      x = rnd_float(r)
    want = np.interp(x, xp, fp)
    got = f(x)
    count += 1
    if not same(got, want):
      bad.append([repr(x), repr(xp)[:60], bits(want), bits(got)])
  return count, bad, sorted(types)


def main():
  r = random.Random(20261122)
  res = {"numpy": np.__version__, "machine": platform.machine(), "python": platform.python_version()}
  ok = True
  for name, fn in [("clip", check_clip), ("sign", check_sign), ("interp", check_interp)]:
    n, bad, types = fn(r, N)
    res[name] = {"cases": n, "mismatches": len(bad), "first": bad[:5], "numpy_types": types}
    ok = ok and not bad
  print(json.dumps(res))
  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
