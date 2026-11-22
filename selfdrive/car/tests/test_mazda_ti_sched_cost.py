"""The run-all scheduler's per-frame cost reduction (ti_ident.frame_scheduler): the new per-frame answers against the old calls,
side by side over the randomised drives of test_mazda_ti_ident.py, frame by frame and event by event (exact equivalence), and the
work it saves. The time-per-simulated-second regression against d20104c is measured off the car by
tools/mazda_ti_accept (see the cost report); here only machine-independent counts are asserted."""
import importlib.util
from pathlib import Path

import pytest

from openpilot.selfdrive.car.mazda import ti_ident as ti
from openpilot.selfdrive.car.mazda.ti_ident import TiIdent

HERE = Path(__file__).resolve().parent


def _ident_tests():
  spec = importlib.util.spec_from_file_location("ti_ident_tests_for_sched", HERE / "test_mazda_ti_ident.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


T = _ident_tests()


def _loop(rows, prog, new, counter=None):
  """The car controller's run-all loop (carcontroller.py: advance_zone, pick with can_run and strict, update), old or new."""
  g = TiIdent(log=None)
  logged = []
  g.log = logged.append
  if counter is not None:
    feas = g._feasible

    def counting(*a):
      counter[0] += 1
      return feas(*a)
    g._feasible = counting
  runner = ti.RunAll(seed=11, emit=g.event_for_pick)
  armed_seq, out = [], []
  for k, row in enumerate(rows):
    profile = g.advance_zone(row["v_ego"]).profile
    if new:
      rem, can_run, strict = ti.frame_scheduler(g, prog.remaining)
      armed = runner.pick(k, profile, rem, can_run, strict)
    else:
      armed = runner.pick(k, profile, prog.remaining, lambda t, w: g.can_run(t, w, prog.remaining),
                          lambda t, w: g.next_is_strict(t, w, prog.remaining))
    armed_seq.append(armed)
    out.append(g.update(armed, k, **row, remaining=prog.remaining))
    prog.add(g.credit)
  return armed_seq, out, logged, runner


@pytest.mark.parametrize("seed,done", [(0, ()), (1, ((ti.SWEEP, "35-41"), (ti.SWEEP, "49-56"), (ti.PULSES, "49-56"))),
                                       (2, tuple((k, w) for k in ti.KEYS for w in ("35-41", "21-27")))])
def test_the_new_scheduler_is_frame_for_frame_identical_to_the_old_one(seed, done):
  rows = T._random_rows(seed, 12000)
  res = []
  for new in (False, True):
    prog = T.Progress()
    for k, w in done:
      T.complete(prog, k, w)
    res.append(_loop(rows, prog, new))
  (a_arm, a_out, a_ev, a_r), (b_arm, b_out, b_ev, b_r) = res
  assert a_arm == b_arm
  assert a_out == b_out
  assert a_ev == b_ev                                   # picks (mode, totals, draw), runs, chunks, stops, credits
  assert (a_r.draw, a_r.current, a_r.mode, a_r.blocked) == (b_r.draw, b_r.current, b_r.mode, b_r.blocked)
  assert len({x for x in a_arm if x}) >= 2 and any(e["kind"] == "pick" for e in a_ev)


def test_the_new_scheduler_checks_each_step_far_fewer_times():
  rows = T._random_rows(3, 4000)
  old, new = [0], [0]
  _loop(rows, T.Progress(), False, old)
  _loop(rows, T.Progress(), True, new)
  assert new[0] * 2 <= old[0], (old[0], new[0])


def _plan_tests():
  spec = importlib.util.spec_from_file_location("ti_plan_tests_for_sched", HERE / "test_mazda_ti_plan.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


@pytest.mark.parametrize("seed", range(2))
def test_the_status_key_is_the_key_of_the_full_status_in_every_cycle(seed, monkeypatch):
  P = _plan_tests()
  from openpilot.selfdrive.car.mazda import ti_ab as ab
  real = ab.TiAb.status_key
  seen = {"n": 0, "states": set()}

  def checked(self):
    k = real(self)
    st = self.status()
    want = (st["state"], st["msg"], st["arm"], st["pin"], st["block"] and st["block"]["idx"], st["wait"] and st["wait"]["reason"],
            st["complete"])
    assert k == want, (k, want)
    seen["n"] += 1
    seen["states"].add((st["state"], st["block"] is not None, st["wait"] is not None))
    return k
  monkeypatch.setattr(ab.TiAb, "status_key", checked)
  P.simulate(seed, seconds=300)
  assert seen["n"] > 20000 and len(seen["states"]) >= 3, seen


def test_the_shares_are_not_computed_while_the_tests_do_not_interleave(monkeypatch):
  from openpilot.selfdrive.car.mazda import ti_ab as ab, ti_plan as tp
  assert tp.INTERLEAVE_TESTS_IN_RANGE is False
  r = ab.TiAbRunner.__new__(ab.TiAbRunner)
  r.ti_tests = None
  r.tests_behind = True
  called = []
  monkeypatch.setattr(tp, "tests_behind", lambda *a: called.append(a) or True)
  assert r._tests_behind(20.0) is False and r.tests_behind is False and called == []
