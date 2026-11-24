#!/usr/bin/env python3
"""Profile ti_curve_plan.Planner on a real road-memory file without a drive log (PC or comma).

  python tools/dev/profile_curve_plan.py --memory <curve_memory.npz> [--traces 6] [--profile]

Drives the planner (all stages: memory ramp, forecast stage 2) at 5 Hz along the memory's own longest traces (fixes taken from the
trace points, heading from the trace; the driving-model preview from the trace's MEDIAN curvature kap_med at the model's 33 leads 0-10
s, published RIGHT positive as plannerd passes it), at 25 m/s with set speed 55, and prints tick times (mean / p99 / max ms), the
plan-T8 tick log's own cost (mem / forecast / stage summaries, tick_log and its JSON) and, with --profile, the top functions by
cumulative time. Positions stay in memory: nothing but counts and timings is printed.
"""
import argparse
import cProfile
import pstats
import time

import numpy as np


def main():
  import json
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P, ti_curve_plannerd as PD
  ap = argparse.ArgumentParser()
  ap.add_argument("--memory", required=True)
  ap.add_argument("--traces", type=int, default=6)
  ap.add_argument("--mps", type=float, default=25.0)
  ap.add_argument("--profile", action="store_true")
  a = ap.parse_args()
  t0 = time.perf_counter()
  mem = M.load(a.memory)
  print(f"memory: {mem.n} points, {mem.n_traces} traces, kap_med {mem.has_kap_med}, bank_v {mem.bank_v}, "
        f"loaded in {time.perf_counter() - t0:.2f} s")
  km = np.asarray(mem.kap_med, np.float64)
  logs, sizes = [], []
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
      ahead = np.array([10.0 * (k / 32) ** 2 for k in range(33)])     # the model's preview times (0 .. 10 s, denser near)
      kk = P.MODEL_K_SIGN * km[np.minimum(i + (ahead * a.mps / mem.spacing).astype(int), s1 - 1)]
      fix = P.Fix(True, float(mem.x[i]), float(mem.y[i]), float(mem.hdg[i]))
      c = time.perf_counter()
      if prof:
        prof.enable()
      p = pl.step(t, fix, a.mps, set_mph=55, yaw_k=float(km[i]), model_k=(ahead, kk), ref_mph=55, floor_mph=None, acting=True,
                  map_k=None, blinker=False)
      if prof:
        prof.disable()
      ticks.append((time.perf_counter() - c) * 1e3)
      c = time.perf_counter()
      s1_, s2_, side = PD.stage_summary(pl)
      d = PD.tick_log(p, dict(seq=len(ticks), v=a.mps / P.MPH, set=55, gap=a.mps / P.MPH - 55, mem=PD.mem_summary(pl),
                              fc=PD.fc_summary(pl), s1=s1_, s2=s2_, side=side))
      sizes.append(len(json.dumps(d)))
      logs.append((time.perf_counter() - c) * 1e3)
      trusted += bool(getattr(p, "trusted", False))
      q += step_pts
      t += 0.2
  ms = np.array(ticks)
  print(f"ticks {len(ms)}, trusted {100.0 * trusted / max(1, len(ms)):.0f} %, mean {ms.mean():.2f} ms, p50 {np.percentile(ms, 50):.2f}, "
        f"p99 {np.percentile(ms, 99):.2f}, max {ms.max():.2f} ms")
  lg = np.array(logs)
  print(f"tick log: mean {1000 * lg.mean():.0f} us, p99 {1000 * np.percentile(lg, 99):.0f} us; planner fields {np.mean(sizes):.0f} B mean, "
        f"{max(sizes)} B max (plannerd adds the car-side fields)")
  if prof:
    pstats.Stats(prof).sort_stats("cumulative").print_stats(18)


if __name__ == "__main__":
  main()
