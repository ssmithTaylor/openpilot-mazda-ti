#!/usr/bin/env python3
"""Profile ti_curve_plan.Planner on a real road-memory file without a drive log (PC or comma).

  python tools/dev/profile_curve_plan.py --memory <curve_memory.npz> [--traces 6] [--profile]

Drives the planner at 5 Hz along the memory's own longest traces (fixes taken from the trace points, heading from the trace, the
curvature preview from the trace), at 25 m/s with set speed 55, and prints tick times (mean / p99 / max ms) and, with --profile, the
top functions by cumulative time. Positions stay in memory: nothing but counts and timings is printed.
"""
import argparse
import cProfile
import pstats
import time

import numpy as np


def main():
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P
  ap = argparse.ArgumentParser()
  ap.add_argument("--memory", required=True)
  ap.add_argument("--traces", type=int, default=6)
  ap.add_argument("--mps", type=float, default=25.0)
  ap.add_argument("--profile", action="store_true")
  a = ap.parse_args()
  t0 = time.perf_counter()
  mem = M.load(a.memory)
  print(f"memory: {mem.n} points, {mem.n_traces} traces, loaded in {time.perf_counter() - t0:.2f} s")
  lengths = np.diff(mem.trace_start)
  traces = np.argsort(-lengths)[:a.traces]
  pl = P.Planner(mem)
  ticks, trusted, t = [], 0, 0.0
  step_pts = a.mps * 0.2 / mem.spacing
  prof = cProfile.Profile() if a.profile else None
  for tr in traces:
    s0, s1 = int(mem.trace_start[tr]), int(mem.trace_start[tr + 1])
    pl.reset() if hasattr(pl, "reset") else None
    q = float(s0)
    while q < s1 - 1:
      i = int(q)
      ahead = np.arange(0, 4.01, 0.25)
      kk = mem.kap[np.minimum(i + (ahead * a.mps / mem.spacing).astype(int), s1 - 1)]
      fix = P.Fix(True, float(mem.x[i]), float(mem.y[i]), float(mem.hdg[i]))
      c = time.perf_counter()
      if prof:
        prof.enable()
      p = pl.step(t, fix, a.mps, set_mph=55, yaw_k=float(mem.kap[i]), model_k=(ahead, kk), ref_mph=55, floor_mph=None, acting=True)
      if prof:
        prof.disable()
      ticks.append((time.perf_counter() - c) * 1e3)
      trusted += bool(getattr(p, "trusted", False))
      q += step_pts
      t += 0.2
  ms = np.array(ticks)
  print(f"ticks {len(ms)}, trusted {100.0 * trusted / max(1, len(ms)):.0f} %, mean {ms.mean():.2f} ms, p50 {np.percentile(ms, 50):.2f}, "
        f"p99 {np.percentile(ms, 99):.2f}, max {ms.max():.2f} ms")
  if prof:
    pstats.Stats(prof).sort_stats("cumulative").print_stats(18)


if __name__ == "__main__":
  main()
