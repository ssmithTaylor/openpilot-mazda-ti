#!/usr/bin/env python3
"""Build selfdrive/car/mazda/ti_curve_approach.npz: the curve planner's approach_m values for its common keys, computed with
ti_curve_plan.approach_m itself (the same values the planner would compute when first needed), keyed to the sources they depend on
(ti_curve_plan.approach_src_hash), and the raise thresholds (ti_curve_plan._raise_threshold) for the recorded cruise set speeds
(ti_curve_plan.raise_table_keys, keyed to ti_curve_plan.raise_src_hash). Rebuild after any change to the cruise model or the approach settings; the planner ignores a
table built from other sources, and test_mazda_ti_curve_approach_table.py fails until it is rebuilt.

  python tools/dev/build_approach_table.py
"""
import time

import numpy as np


def main():
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  t0 = time.perf_counter()
  keys = P.approach_table_keys()
  P._APPROACH.clear()
  dist = [P.approach_m(u, T, S1, set0=s0) for u, T, S1, s0 in keys]
  t1 = time.perf_counter()
  rk = P.raise_table_keys()
  P._RAISE_THR.clear()
  thr = [P._raise_threshold(sm, T) for sm, T in rk]
  np.savez_compressed(P.APPROACH_TABLE, keys=np.array(keys, np.int16), dist=np.array(dist, np.float64),
                      src=np.array(P.approach_src_hash()),
                      rthr_set=np.array([sm for sm, _ in rk], np.float64), rthr_T=np.array([T for _, T in rk], np.int16),
                      rthr=np.array([np.nan if x is None else x for x in thr], np.float64), rsrc=np.array(P.raise_src_hash()))
  print(f"{len(keys)} keys in {t1 - t0:.1f} s, {len(rk)} raise thresholds in {time.perf_counter() - t1:.1f} s -> {P.APPROACH_TABLE}")


if __name__ == "__main__":
  main()
