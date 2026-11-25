"""The precomputed approach table (ti_curve_approach.npz, tools/dev/build_approach_table.py) matches the current sources and holds
exactly what ti_curve_plan.approach_m computes: it is a cache only, never a different answer."""
import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_plan as P


def test_table_is_built_from_the_current_sources():
  z = np.load(P.APPROACH_TABLE, allow_pickle=False)
  assert str(z["src"]) == P.approach_src_hash(), "stale: run tools/dev/build_approach_table.py"
  assert [tuple(k) for k in z["keys"].tolist()] == P.approach_table_keys()


def test_table_values_are_the_computed_ones():
  z = np.load(P.APPROACH_TABLE, allow_pickle=False)
  keys, dist = z["keys"].tolist(), z["dist"]
  saved = dict(P._APPROACH)
  try:
    for i in range(0, len(keys), 37):                    # a spread sample, computed fresh
      P._APPROACH.clear()
      u, T, S1, s0 = keys[i]
      assert P.approach_m(u, T, S1, set0=s0) == float(dist[i]), keys[i]
  finally:
    P._APPROACH.clear()
    P._APPROACH.update(saved)


def test_planner_loads_it():
  P.Planner._table_tried = False
  P.Planner(None)
  assert P.Planner.table_keys == len(P.approach_table_keys())
