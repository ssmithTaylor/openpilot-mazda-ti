#!/usr/bin/env python3
"""Build selfdrive/car/mazda/ti_curve_approach.npz: the curve planner's approach_m values for its common keys, computed with
ti_curve_plan.approach_m itself (the same values the planner would compute when first needed), keyed to the sources they depend on
(ti_curve_plan.approach_src_hash). Rebuild after any change to the cruise model or the approach settings; the planner ignores a
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
  np.savez_compressed(P.APPROACH_TABLE, keys=np.array(keys, np.int16), dist=np.array(dist, np.float64),
                      src=np.array(P.approach_src_hash()))
  print(f"{len(keys)} keys in {time.perf_counter() - t0:.1f} s -> {P.APPROACH_TABLE}")


if __name__ == "__main__":
  main()
