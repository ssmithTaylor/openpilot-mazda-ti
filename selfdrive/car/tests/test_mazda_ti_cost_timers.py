"""ti_cost_timers.HookTimers (core-4 cost, 2026-11-26): the 10 s summary's numbers (mean / p99 to the 20 us bin / max / count, gauges),
the in-place reset, the fields the drive review reads, the timers' own cost, and the wiring in controlsd / card / the car controller."""
import ast
import time
from pathlib import Path

from openpilot.selfdrive.car.mazda import ti_cost_timers as T

ROOT = Path(__file__).resolve().parents[3]


def test_summary_numbers_and_reset():
  logged = []
  tc = T.HookTimers("card", ("step", "cc"), gauges=("plan_age_s",), log=logged.append, every=200)
  for k in range(200):
    tc.add(0, 1_000_000 + (5_000_000 if k == 7 else 0))      # 1 ms, one 6 ms outlier
    if k % 2 == 0:
      tc.add(1, 100_000 if k < 196 else 900_000)              # 100 us, the last two 900 us
    tc.gauge_max(0, None)
    tc.gauge_max(0, 0.2 + (0.5 if k == 50 else 0.0))
    out = tc.tick()
    assert (out is None) == (k < 199)
  assert len(logged) == 1
  s = logged[0]
  assert set(s) == {"proc", "frames", "win_s", "h", "g"} and s["proc"] == "card" and s["frames"] == 200
  n, mean, p99, mx = s["h"]["step"]
  assert n == 200 and abs(mean - 1025.0) < 0.1 and mx == 6000.0 and p99 == 1020.0     # the outlier is the top 0.5 %: p99 is the 1 ms bin (upper edge)
  n, mean, p99, mx = s["h"]["cc"]
  assert n == 100 and mx == 900.0 and p99 == 920.0                                     # 2 of 100 above: their bin's upper edge
  assert s["g"] == {"plan_age_s": 0.7}
  assert tc.count == [0, 0] and tc.sum_ns == [0, 0] and tc.max_ns == [0, 0] and tc.gauge == [None] and tc.frames == 0
  assert all(sum(h) == 0 and len(h) == T.N_BINS for h in tc.hist)


def test_empty_hook_and_overflow_bin():
  tc = T.HookTimers("controlsd", ("step", "drift"), log=None, every=10)
  for _ in range(10):
    tc.add(0, 50_000_000)                                       # 50 ms: the last bin
  s = tc.summary()
  assert s["h"]["drift"] == [0, 0.0, 0.0, 0.0]
  assert s["h"]["step"][3] == 50000.0 and s["h"]["step"][2] == T.N_BINS * T.BIN_NS / 1000


def test_own_cost_is_small():
  """add + tick per hook-cycle; the comma is ~20x slower than the PC, and the budget there is 10 us a cycle for all hooks (5 adds)."""
  tc = T.HookTimers("controlsd", ("step", "lat", "ab", "diag", "drift"), log=None)
  pc = time.perf_counter_ns
  t0 = pc()
  for _ in range(20000):
    a = pc()
    tc.add(1, pc() - a)
    tc.add(2, 1000)
    tc.add(3, 1000)
    tc.add(4, 1000)
    tc.add(0, pc() - a)
    tc.tick()
  per_cycle_us = (pc() - t0) / 20000 / 1000
  assert per_cycle_us < 2.0, per_cycle_us


def _src(rel):
  return (ROOT / rel).read_text(encoding="utf-8")


def test_hooks_are_wired_where_the_review_expects_them():
  ctl, card, cc = _src("selfdrive/controls/controlsd.py"), _src("selfdrive/car/card.py"), _src("selfdrive/car/mazda/carcontroller.py")
  assert 'HookTimers("controlsd", ("step", "lat", "ab", "diag", "drift")' in ctl and "tc.tick()" in ctl
  assert 'HookTimers("card", ("step", "cc", "curve", "buttons"), gauges=("plan_age_s",)' in card and "tc.tick()" in card
  assert "ti_cc.ti_cost = self.ti_cost" in card
  assert "tc.gauge_max(0, age)" in cc and "tc.add(2," in cc and "tc.add(3," in cc
  for src in (ctl, card):
    ast.parse(src)
