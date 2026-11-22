"""Lag guard (ti_lag.py): the tests stand down by themselves while the control loop is short of time, with the reason on the pages and in
the log; resumes only after 10 s back under the release level. An injected clock drives it; the car controller's hold is checked through
its own apply_ti_tuning, and the pages' words for the reason through the header."""
import ast
import importlib.util
import json
import re
import time
from pathlib import Path
from types import SimpleNamespace as NS

from openpilot.selfdrive.car.mazda import ti_lag as tl
from openpilot.selfdrive.car.mazda import ti_plan as tp

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def feed(g, t, n, dt):
  held = []
  for _ in range(n):
    t += dt
    held.append(g.tick(t))
  return t, held


def test_on_time_it_never_holds():
  ev = []
  g = tl.LagGuard(log=ev.append)
  t, held = feed(g, 0.0, 20000, 0.0100)
  assert not any(held) and ev == []
  t, held = feed(g, t, 5000, 0.0102)            # a slightly slow loop under the hold level
  assert not any(held)


def test_holds_when_the_mean_of_100_cycles_is_over_10_3_ms_and_resumes_after_10_s_under_10_1():
  ev = []
  g = tl.LagGuard(log=ev.append)
  t, _ = feed(g, 0.0, 200, 0.0100)
  t, held = feed(g, t, 100, 0.0110)
  first = held.index(True)
  assert 25 <= first <= 60                       # the mean crosses 10.3 ms after about 30 slow cycles: quick, not on one spike
  assert ev[-1]["kind"] == "lag_hold"
  t, held = feed(g, t, 300, 0.0102)              # between the levels: stays held
  assert all(held)
  t, held = feed(g, t, 999, 0.0100)              # under 10.1 ms, but not yet 10 s
  assert all(held)
  t, held = feed(g, t, 200, 0.0100)
  assert held[-1] is False and ev[-1]["kind"] == "lag_release"


def test_a_brief_return_over_the_release_level_restarts_the_10_s():
  g = tl.LagGuard()
  t, _ = feed(g, 0.0, 101, 0.0120)
  assert g.held
  t, _ = feed(g, t, 800, 0.0100)                 # 8 s under
  t, _ = feed(g, t, 30, 0.0130)                  # the mean goes back over 10.1 ms
  t, held = feed(g, t, 700, 0.0100)
  assert all(held)                               # the 10 s start again once it is under
  t, held = feed(g, t, 600, 0.0100)
  assert held[-1] is False


def test_a_single_spike_does_not_hold_and_a_pause_clears_the_window():
  g = tl.LagGuard()
  t, _ = feed(g, 0.0, 300, 0.0100)
  t, held = feed(g, t, 1, 0.025)                 # one 25 ms cycle: the mean of 100 rises 0.15 ms
  t, held2 = feed(g, t, 100, 0.0100)
  assert not any(held + held2)
  t, _ = feed(g, t, 1, 5.0)                      # the process paused: not one long cycle
  assert not g.held and len(g.card.q) == 0


def test_the_cost_of_a_tick_is_constant():
  g = tl.LagGuard()
  t0 = time.perf_counter()
  feed(g, 0.0, 100000, 0.01)
  assert (time.perf_counter() - t0) / 100000 < 20e-6


class Mem:
  def __init__(self, **d):
    self.d = dict(d)

  def get_bool(self, k):
    return bool(self.d.get(k, False))

  def get(self, k, encoding=None):
    return self.d.get(k)

  def put(self, k, v):
    self.d[k] = v


HOLD_LOG = []


def _apply():
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  cls = next(n for n in ast.parse(src).body if isinstance(n, ast.ClassDef) and n.name == "CarController")
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "apply_ti_tuning")
  from openpilot.selfdrive.car.mazda import ti_ident as ti
  env = {"ti_ident_mod": ti, "ti_plan_mod": tp, "time": time, "json": json, "ti_hold_log": HOLD_LOG.append, "ti_lag_mod": tl}
  exec(compile(ast.Module(body=[fn], type_ignores=[]), "apply_ti_tuning", "exec"), env)
  return env["apply_ti_tuning"]


def _plan(mode, seq=1, t=None, at_a=True):
  return tp.encode_plan(tp.PlanOut(mode, mode == "ab", "", "", seq), time.time() if t is None else t, False, "n1", at_a)


def test_the_car_controller_holds_the_tests_with_the_reason_while_the_loop_is_busy_and_lets_them_go_after():
  apply = _apply()
  mem = Mem(TiPlanOn=True, TiPlan=_plan("tests", 4))
  h = NS(ccp=NS(TI_STEER_DELTA_UP=6, TI_STEER_DELTA_UP_HIGH=6), _ti_limit=lambda *a: 6, frame=0, params_memory=mem,
         ti_steer_threshold=None, ti_dither_enabled=False, ti_ident_on=(), ti_ident_all=False, ti_ab_on=False, ti_plan_on=False,
         ti_plan_allowed=False, ti_plan_seq=None, ti_plan_nonce=None, ti_plan_ack_key=None, ti_hold_key=None, ti_hold_published=None,
         ti_lag_held=False)
  toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
  apply(h, toggles)
  assert h.ti_ab_on is False and json.loads(mem.d["TiTestHold"])["held"] is False
  h.ti_lag_held, h.frame = True, 10
  mem.d["TiPlan"] = _plan("tests", 4)
  apply(h, toggles)
  hold = json.loads(mem.d["TiTestHold"])
  assert h.ti_ab_on is True and hold == dict(hold, held=True, why="control loop busy")
  assert HOLD_LOG[-1]["why"] == "control loop busy" and HOLD_LOG[-1]["held"] is True
  assert h.ti_plan_on is True                    # the plan stays on: it is the tests that stand down
  h.ti_lag_held, h.frame = False, 20
  mem.d["TiPlan"] = _plan("tests", 4)
  apply(h, toggles)
  assert h.ti_ab_on is False and json.loads(mem.d["TiTestHold"])["held"] is False


def test_the_pages_word_the_reason_plainly():
  assert tl.WHY in tp.HOLD_WHYS
  ab_h = (ROOT / "selfdrive/ui/qt/ti_ab_logic.h").read_text(encoding="utf-8")
  pages_h = (ROOT / "selfdrive/ui/qt/ti_pages_logic.h").read_text(encoding="utf-8")
  assert '{"control loop busy", "Tests paused: control loop busy"}' in ab_h
  assert re.search(r'\{"control loop busy", "PAUSED: LOOP BUSY", Tone::Held\}', pages_h)


def test_the_guard_runs_every_cycle_before_the_hold_decision():
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  i_tick = src.index("self.ti_lag_held = lag.tick(time.monotonic(), getattr(self, \"ti_lag_ctl\", None))")
  i_apply = src.index("      self.apply_ti_tuning(frogpilot_toggles)")
  assert i_tick < i_apply


def test_one_stall_however_long_never_holds_but_three_do():
  g = tl.LagGuard()
  t, _ = feed(g, 0.0, 300, 0.0100)
  for stall in (0.031, 0.2, 0.9):
    t, held = feed(g, t, 1, stall)
    t, held2 = feed(g, t, 150, 0.0100)
    assert not any(held + held2), stall
  for _ in range(4):
    t, _ = feed(g, t, 1, 0.040)
    t, _ = feed(g, t, 5, 0.0100)
  assert g.held


def test_controlsd_lag_holds_while_card_is_on_time():
  g = tl.LagGuard()
  t, lag, stamp = 0.0, 50.0, 0.0
  for k in range(300):                                  # controlsd on time: its lag stays flat
    t += 0.01
    stamp += 0.01
    g.tick(t, (stamp, lag + (0.3 if k % 2 else -0.3)))
  assert not g.held
  for k in range(60):                                   # controlsd 1 ms late every cycle: a mean period of 11 ms
    t += 0.01
    stamp += 0.011
    lag += 1.0
    g.tick(t, (stamp, lag))
  assert g.held and g.signal == "controls" and g.card_ms < 10.1
  assert g.ctl_ms > tl.HOLD_MS


def test_the_resume_wait_doubles_on_a_quick_re_trigger_up_to_a_minute_and_resets_after_a_quiet_minute():
  g = tl.LagGuard()
  t, _ = feed(g, 0.0, 200, 0.0100)
  waits = []
  for _ in range(4):
    t, _ = feed(g, t, 120, 0.0130)                      # busy: held
    assert g.held
    waits.append(g.wait_s)
    t, held = feed(g, t, int((g.wait_s + 3.0) / 0.01), 0.0100)
    assert not held[-1]                                 # released after its wait
  assert waits == [10.0, 20.0, 40.0, 60.0]
  t, _ = feed(g, t, 7000, 0.0100)                       # a quiet minute
  t, _ = feed(g, t, 120, 0.0130)
  assert g.held and g.wait_s == 10.0


def test_a_gap_clears_the_release_timer():
  g = tl.LagGuard()
  t, _ = feed(g, 0.0, 200, 0.0130)
  assert g.held
  t, _ = feed(g, t, 900, 0.0100)                        # 9 s under
  assert g.under_since is not None
  t, _ = feed(g, t, 1, 3.0)                             # the process paused
  assert g.under_since is None and g.held


def test_card_hands_controlsd_lag_to_the_car_controller_and_the_hold_holds_without_the_plan_module():
  card = (ROOT / "selfdrive/car/card.py").read_text(encoding="utf-8")
  assert "ti_cc.ti_lag_ctl = (self.sm.logMonoTime['controlsState'] * 1e-9, self.sm['controlsState'].cumLagMs)" in card
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  assert 'self.ti_ab_on = ab_on or bool(getattr(self, "ti_lag_held", False))' in src
