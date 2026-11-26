#!/usr/bin/env python3
"""Record the curve planner install dry run's real-drive inputs (selfdrive/car/mazda/ti_curve_dryrun_inputs.npz) from a drive's
POSITION-FREE run arrays (the governor's run cache: 20 Hz speed, yaw curvature, cruise set speed / state, blinker, the driving model's
lateral preview), PC only:

  python tools/dev/record_dryrun_inputs.py --drive <index> [--run 0] [--t0 2515] [--minutes 8]

What is written: the road as the car's own yaw curvature per 5 m of the distance driven (the dry run rebuilds a synthetic road and its
road memory from it: a shape relative to its own start, never a place), and per planner tick (5 Hz) the distance along it, speed,
set speed (float32 m/s, as carState carries it), yaw curvature, cruise / blinker flags and the preview (float16). No position, no
name, no VIN, no route or drive name. The car side's values and the map list ("cs", "mpr" + map_list in the new tick log) are not in
the run cache: arrays for them are written only when given (--with-logged, not yet available: no drive with that build)."""
import argparse
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(REPO, "selfdrive", "car", "mazda", "ti_curve_dryrun_inputs.npz")
DT = 0.05            # the run cache's step
TICK = 4             # planner ticks at 5 Hz
SPACING = 5.0


def record(run, t0, minutes, out=OUT):
  i0 = int(round(t0 / DT))
  i1 = min(len(run["v"]), i0 + int(round(minutes * 60.0 / DT)))
  v = np.asarray(run["v"], np.float64)[i0:i1]
  kap = np.asarray(run["kap"], np.float64)[i0:i1]
  s = np.r_[0.0, np.cumsum(v[:-1] * DT)]
  nb = int(s[-1] // SPACING) + 1
  b = np.minimum((s // SPACING).astype(int), nb - 1)
  road_k = np.bincount(b, weights=kap, minlength=nb) / np.maximum(1, np.bincount(b, minlength=nb))
  t = np.arange(0, i1 - i0, TICK)
  cr = (np.asarray(run["cr_en"])[i0:i1] > .5) & (np.asarray(run["cr_set"])[i0:i1] > .5)
  arrays = dict(
    meta=np.array(json.dumps({"version": 1, "spacing_m": SPACING, "tick_s": TICK * DT, "source": "run cache (position-free)"})),
    road_k=road_k.astype(np.float16), s=s[t].astype(np.float32), v=v[t].astype(np.float16),
    set_ms=np.where(cr, np.asarray(run["cr_set"], np.float32)[i0:i1], 0.0).astype(np.float32)[t],
    yaw_k=kap[t].astype(np.float16), blink=(np.asarray(run["blink"])[i0:i1][t] > .5).astype(np.int8),
    pred_t=np.asarray(run["pred_t"], np.float32), pred_la=np.asarray(run["pred_la"], np.float16)[i0:i1][t])
  np.savez_compressed(out, **arrays)
  return out, len(t), len(road_k)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--drive", type=int, required=True, help="index into ground_truth.drives() (the 132-drive set)")
  ap.add_argument("--run", type=int, default=0)
  ap.add_argument("--t0", type=float, default=0.0)
  ap.add_argument("--minutes", type=float, default=8.0)
  a = ap.parse_args()
  sys.path.insert(0, os.path.join(REPO, "tools", "curve_slowing"))
  import ground_truth as G
  run = G.load_runs(G.drives()[a.drive])[a.run]
  out, n, nk = record(run, a.t0, a.minutes)
  print(f"{n} ticks, road {nk * SPACING / 1000:.1f} km -> {out} ({os.path.getsize(out) / 1024:.0f} KB)")


if __name__ == "__main__":
  main()
