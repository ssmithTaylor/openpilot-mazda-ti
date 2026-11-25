"""No per-frame leaks or cost growth in the code this fork runs every frame (inventory: tools/dev/PER_FRAME_INVENTORY.md).

card and controlsd run with the cycle collector off (common/realtime.py config_realtime_process -> gc.disable()), so one cycle left per
frame is a leak that grows for the whole drive: on 2026-11-25 lane_drift._with_line_probs left a class per call (about 1-2 GB per
engaged hour, "low memory" after about an hour). Short drives never show it. Each scenario here drives one entry point through a
repeating input sequence that starts and ends everything that keeps state, then (selfdrive/car/mazda/ti_mem_check.py, also the
install check on the comma) asserts: (a) no cyclic garbage after the warm-up, (b) no retained growth in the second window beyond 1 %
of the first plus 8 KB and under 5 MB/h at the entry point's rate, (c) the second window's per-frame time within 1.5x of the first
(skipped when the timing is too noisy to judge on a busy PC).
"""
import gc

import pytest

from openpilot.selfdrive.car.mazda import ti_mem_check as MC

NAMES = list(MC.SCENARIOS)
# What each scenario must have exercised inside its measured run (else the "no growth" is about an idle path): the states that create
# and drop state. Keys are counted by the scenario (ti_mem_check: `seen`).
REQUIRED = {
  "lane_drift": ("gate open", "latched", "lane change"),
  "ab_runner": ("ev:block_start", "ev:block_end", "plan:ab", "plan:tests"),
  "lateral": ("lane valid", "lane confidence", "lane lane_change", "inactive", "diagnostics"),
  "card_tests": ("ident running", "lag held", "plan tests", "ev:run", "ev:stop"),
  "card_curve": ("curve:tap", "curve:restored", "curve:deliver_onset", "curve:deliver_clear", "state:stopped", "curve:switched_off"),
  "card_buttons": ("buttons:running", "buttons:finished", "check:running", "check:finished", "frames sent"),
  "planner": ("ceiling", "ev:alert", "ev:lock_lost", "ev:lock"),
}


@pytest.fixture(scope="module")
def results():
  return {}


@pytest.mark.parametrize("name", NAMES)
def test_no_cyclic_garbage_and_no_retained_growth(name, results):
  r = MC.run(name)
  results[name] = r
  print(name, {k: v for k, v in r.items() if k not in ("seen", "top")}, "seen:", r["seen"])
  missing = [k for k in REQUIRED[name] if not r["seen"].get(k)]
  assert not missing, f"{name}: the input sequence no longer exercises {missing} (seen {r['seen']})"
  assert r["cyclic"] == 0, f"{name}: {r['fails']}"
  assert not r["fails"], f"{name} ({MC.SCENARIOS[name][5]}, {MC.SCENARIOS[name][1]} Hz): {r['fails']}"
  if r["noisy"]:
    pytest.skip(f"{name}: memory ok; per-frame timing too noisy to judge cost growth "
                f"(window medians {r['us1']} / {r['us2']} us, chunk spread over {MC.NOISY_SPREAD})")
  assert not r["cost_fail"], f"{name}: per-frame cost grew {r['cost_ratio']}x ({r['us1']} -> {r['us2']} us)"


@pytest.mark.parametrize("mod,cls,cap", MC.BOUNDED_EVENT_BUFFERS)
def test_the_event_buffers_the_harness_drains_stay_capped(mod, cls, cap):
  """The scenarios drain these buffers (they take hours to fill); their own trim is what bounds them on the car: keep it."""
  import importlib
  m = importlib.import_module(f"openpilot.selfdrive.car.mazda.{mod}")
  obj = getattr(m, cls)(log=None)
  for k in range(3 * cap):
    obj._event("hb", s=0.0) if mod == "ti_ab" else obj._event(k, "hb")
  assert 0 < len(obj.events) <= cap


# ---- the guard itself must fail on the two leak shapes it exists for ----------------------------------------------------------------
def test_guard_catches_a_class_defined_per_call():
  """The 2026-11-25 lane-drift shape: a class made inside a per-frame function, holding the frame's input."""
  def step(i):
    class View:            # per-frame-ok: deliberately the leak, to prove the guard sees it
      src = i
    return View.src
  r = MC.measure(step, 200, 2000, 100)
  assert r["cyclic"] > 0 and r["fails"]


def test_guard_catches_an_append_without_a_trim():
  hist = []

  def step(i):
    hist.append((i, float(i)))
  r = MC.measure(step, 200, 4000, 100)
  assert r["cyclic"] == 0 and any("retained growth" in f for f in r["fails"])


def test_guard_passes_a_bounded_history_and_restores_the_collector():
  import collections
  hist = collections.deque(maxlen=50)
  was = gc.isenabled()
  r = MC.measure(lambda i: hist.append((i, float(i))), 200, 4000, 100)
  assert not r["fails"], r
  assert gc.isenabled() == was


def test_readiness_line_warns_under_20_percent_free(monkeypatch, tmp_path):
  import shutil
  monkeypatch.setattr(shutil, "disk_usage", lambda p: shutil._ntuple_diskusage(100 * 10**9, 85 * 10**9, 15 * 10**9))
  line = MC.readiness(str(tmp_path))
  assert "free disk 15 % (15.0 GB)" in line and "WARNING" in line
  monkeypatch.setattr(shutil, "disk_usage", lambda p: shutil._ntuple_diskusage(100 * 10**9, 50 * 10**9, 50 * 10**9))
  assert "WARNING" not in MC.readiness(str(tmp_path))


def test_install_check_quick_mode_runs_every_scenario_and_passes(capsys):
  """The same entry point comma_checks.sh calls on the device (--quick, strided inputs), here on the PC harness: exit 0, one line per
  scenario."""
  assert MC.main(["--quick"]) == 0
  out = capsys.readouterr().out
  for name in NAMES:
    assert name in out
  assert "0 failure(s)" in out
