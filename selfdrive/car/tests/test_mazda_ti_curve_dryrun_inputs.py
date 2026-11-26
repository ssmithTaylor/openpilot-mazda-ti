"""The install dry run over recorded real-drive inputs (perf 2026-11-26; owner's yes: "real-drive dry run at install"). The shipped
inputs are small and position-free (curvature / distance / speed series only); the dry run runs the FULL tick and fails on any of the
three budgets: no bend in play p99 <= 20 ms, bend in play p99 <= 100 ms, no tick over 500 ms."""
import json
import time

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_dryrun as DR
from openpilot.selfdrive.car.mazda import ti_curve_plan as P

KEYS = {"meta", "road_k", "s", "v", "set_ms", "yaw_k", "blink", "pred_t", "pred_la"}


def test_shipped_inputs_are_small_and_position_free():
  import os
  assert os.path.getsize(DR.INPUTS) <= 250 * 1024
  z = np.load(DR.INPUTS, allow_pickle=False)
  assert set(z.files) == KEYS, set(z.files) ^ KEYS
  meta = json.loads(str(z["meta"]))
  assert set(meta) <= {"version", "spacing_m", "tick_s", "source"}
  text = json.dumps(meta).lower()
  for bad in ("lat", "lon", "vin", "route", "name", "000"):
    assert bad not in text
  assert len(z["s"]) >= 2000 and len(z["road_k"]) * float(meta["spacing_m"]) >= 5000.0
  # bend-rich: the stretch was chosen for its slowdowns
  assert float(np.max(np.abs(z["road_k"].astype(float)))) > 0.01


def test_dry_run_passes_on_the_pc_and_plans_bends(capsys):
  assert DR.run_inputs(DR.INPUTS) == 0
  out = capsys.readouterr().out
  assert "DRY RUN OK" in out
  started = int(out.split("bends started ")[1].split(";")[0])
  assert started >= 20
  cold = int(out.split("cold simulations ")[1].split(";")[0])
  assert cold <= 5, cold                     # the table covers what a real drive asks (121 with the 2026-11-25 table)


def test_budgets_each_fail_on_their_own():
  ok, _ = DR.budgets([0.001] * 200, [False] * 200, 20.0, 100.0, 500.0)
  assert ok
  assert not DR.budgets([0.001] * 190 + [0.03] * 10, [False] * 200, 20.0, 100.0, 500.0)[0]          # no-bend p99
  assert not DR.budgets([0.001] * 100 + [0.15] * 100, [False] * 100 + [True] * 100, 20.0, 100.0, 500.0)[0]   # bend p99
  assert not DR.budgets([0.001] * 199 + [0.6], [True] * 200, 20.0, 100.0, 500.0)[0]                 # one slow step


def test_a_slow_step_fails_the_dry_run(monkeypatch):
  orig = P.Planner.step
  calls = [0]

  def slow(self, *a, **k):
    calls[0] += 1
    if calls[0] == 30:
      time.sleep(0.55)
    return orig(self, *a, **k)
  monkeypatch.setattr(P.Planner, "step", slow)
  assert DR.run_inputs(DR.INPUTS, limit=60) == 1
