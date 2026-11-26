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
  "frogpilot_events": ("running wide", "gate latched", "ti dropout", "advisory", "latched", "curve alert", "curve code"),
}


def test_frogpilot_process_is_guarded_and_runs_with_the_collector_off():
  """frogpilot_process runs FrogPilotEvents.update (and through it LaneDriftGate.update) every model frame on-road, engaged or not,
  with gc disabled: the 2026-11-25 drive's second leak (~1-1.4 GB/h, the same lane_drift class per call). Keep a scenario for it,
  and keep the collector state it is measured under true to the process."""
  from pathlib import Path
  src = (Path(__file__).resolve().parents[3] / "frogpilot" / "frogpilot_process.py").read_text()
  assert "config_realtime_process(" in src and "Ratekeeper(1 / DT_MDL" in src
  assert MC.SCENARIOS["frogpilot_events"][1] == 20 and MC.SCENARIOS["frogpilot_events"][5] == "frogpilot_process"


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
  assert any("retained growth" in f for f in r["fails"]), r   # the cycles stay counted as growth too (kept in gc.garbage)


def test_guard_catches_an_append_without_a_trim():
  hist = []

  def step(i):
    hist.append((i, float(i)))
  r = MC.measure(step, 200, 4000, 100)
  assert r["cyclic"] == 0 and any("retained growth" in f for f in r["fails"])


def test_guard_catches_a_slow_append_at_about_5_mb_per_hour_in_a_quick_window():
  """A float kept every other frame: 16 B/frame, 5.8 MB/h at 100 Hz; 1500 frames is the device's quick card_tests window."""
  hist = []

  def step(i):
    if i % 2:
      hist.append(float(i))
  r = MC.measure(step, 200, 1500, 100)
  assert r["cyclic"] == 0 and any("retained growth" in f for f in r["fails"]), r


def test_guard_does_not_count_memory_a_full_collection_returns_without_finding_garbage():
  """Python 3.11 on the comma (2026-11-25 install: card_tests at carcontroller.py:1485, 48 B 1-tuples from pycapnp 1.3's new_message;
  planner at ti_curve_plan.py:1094, 88 B 6-tuples from tuple(genexpr)): a freed tuple goes onto the interpreter's free list (at most
  2000 per size) and keeps its tracemalloc trace until a full collection empties the lists. Bounded (176,000 B for the planner's
  6-tuples, reached after 2000 ticks), but inside a one-period window it reads as growth. Simulated here by a cache that only a full
  collection empties; real retention and cycles must still show (the tests above)."""
  cache = []

  def emptied_by_full_collection(phase, info):
    if phase == "stop" and info["generation"] == 2:
      cache.clear()
  gc.callbacks.append(emptied_by_full_collection)
  try:
    r = MC.measure(lambda i: cache.append(bytes(100 + i % 2)), 200, 2000, 100)
  finally:
    gc.callbacks.remove(emptied_by_full_collection)
  assert not r["fails"], r


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


def test_install_script_runs_the_guard_and_prints_readiness_on_every_exit():
  from pathlib import Path
  sh = (Path(__file__).resolve().parents[3] / "tools" / "dev" / "comma_checks.sh").read_text()
  assert "ti_mem_check.py --quick" in sh and "trap 'python3 selfdrive/car/mazda/ti_mem_check.py --readiness" in sh
  assert MC.main(["--readiness"]) == 0
