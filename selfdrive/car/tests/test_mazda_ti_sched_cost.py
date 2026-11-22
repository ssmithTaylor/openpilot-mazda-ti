"""The run-all scheduler's per-frame cost reduction (ti_ident.frame_scheduler): the new per-frame answers against the old calls,
side by side over the randomised drives of test_mazda_ti_ident.py, frame by frame and event by event (exact equivalence), and the
work it saves. The time-per-simulated-second regression against d20104c is measured off the car by
tools/mazda_ti_accept (see the cost report); here only machine-independent counts are asserted."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

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


class VProgress(T.Progress):
  """The test accountant's `remaining` with its `version` (ti_tests.TestAccountant: +1 on every change of progress)."""
  def __init__(self):
    super().__init__()
    self.version = 0

  def add(self, credit):
    if credit is not None:
      self.version += 1
    super().add(credit)


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
  cache = ti.SchedulerCache()
  armed_seq, out = [], []
  for k, row in enumerate(rows):
    profile = g.advance_zone(row["v_ego"]).profile
    if new == "cached":
      rem, can_run, strict = cache.frame(g, prog.remaining, profile, (id(prog), prog.version))
      armed = runner.pick(k, profile, rem, can_run, strict)
    elif new:
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
  for k in range(1500, 12000, 2900):             # a few frames of unusable inputs (non-finite: every limit closed)
    for j in range(k, k + 40):
      rows[j] = dict(rows[j], lat_demand=float("nan"))
  res = []
  for new in (False, True, "cached"):
    prog = VProgress()
    for k, w in done:
      T.complete(prog, k, w)
    prog.version += 1
    res.append(_loop(rows, prog, new))
  (a_arm, a_out, a_ev, a_r) = res[0]
  for (b_arm, b_out, b_ev, b_r) in res[1:]:
    assert a_arm == b_arm
    assert a_out == b_out
    assert a_ev == b_ev                                 # picks (mode, totals, draw), runs, chunks, stops, credits
    assert (a_r.draw, a_r.current, a_r.mode, a_r.blocked) == (b_r.draw, b_r.current, b_r.mode, b_r.blocked)
  assert len({x for x in a_arm if x}) >= 2 and any(e["kind"] == "pick" for e in a_ev)


def test_the_new_scheduler_checks_each_step_far_fewer_times():
  rows = T._random_rows(3, 4000)
  old, new = [0], [0]
  _loop(rows, VProgress(), False, old)
  _loop(rows, VProgress(), True, new)
  assert new[0] * 2 <= old[0], (old[0], new[0])


def test_the_cache_reuses_the_answers_across_frames_while_nothing_changes():
  rows = T._random_rows(4, 3000)
  calls = [0]
  real = ti.frame_scheduler

  def counting(*a):
    calls[0] += 1
    return real(*a)
  ti_frame = ti.frame_scheduler
  try:
    ti.frame_scheduler = counting
    _loop(rows, VProgress(), "cached")
  finally:
    ti.frame_scheduler = ti_frame
  assert calls[0] < len(rows) / 2, calls[0]


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


ROOT = HERE.parents[2]


def _block(path, start, n_lines):
  import textwrap
  lines = (ROOT / path).read_text(encoding="utf-8").splitlines()
  i = next(k for k, line in enumerate(lines) if line.strip() == start)
  return textwrap.dedent("\n".join(lines[i:i + n_lines]))


@pytest.mark.parametrize("path,start,n", [("selfdrive/controls/controlsd.py", "toggles_updated = bool(self.sm['frogpilotPlan'].togglesUpdated)", 6),
                                          ("selfdrive/car/card.py", "toggles_updated = bool(self.sm['frogpilotPlan'].togglesUpdated)", 5)])
def test_the_toggles_are_read_on_the_edges_and_once_a_second_while_the_flag_is_on(path, start, n):
  code = compile(_block(path, start, n), path, "exec")

  def run(seq, value_at=lambda k: k):
    host = NS(toggles_updated_last=False, toggles_on_cycles=0, select_lat_control=lambda: None, frogpilot_toggles=None)
    reads = []
    for k, flag in enumerate(seq):
      host.sm = {"frogpilotPlan": NS(togglesUpdated=flag)}
      exec(code, {"self": host, "get_frogpilot_toggles": lambda k=k: reads.append(k) or value_at(k)})
    return reads, host
  # an ordinary window (the flag on for about a second): its two edges and at most one more
  reads, _ = run([False] * 5 + [True] * 100 + [False] * 50)
  assert reads[0] == 5 and reads[-1] == 105 and len(reads) <= 3
  # the flag stuck on (a clock step, a stalled plan): a toggle changed underneath is picked up within about a second
  reads, host = run([False] * 5 + [True] * 1000, value_at=lambda k: "new" if k >= 400 else "old")
  assert host.frogpilot_toggles == "new" and any(400 <= r <= 505 for r in reads) and len(reads) <= 12


class OldLimits(TiIdent):
  """The generator as before: every limit check made in full for every step (no per-frame class memo)."""
  def _limit_class(self, key, step, band, memo):
    return self.limit_state(key, step, band, **self.inputs)


@pytest.mark.parametrize("seed", range(3))
def test_the_generator_with_the_per_frame_limit_memo_is_identical_to_the_full_checks(seed):
  rows = T._random_rows(seed, 12000)
  for k in range(1500, 12000, 2900):
    for j in range(k, k + 40):
      rows[j] = dict(rows[j], base_request=float("nan"))
  res = []
  for cls in (OldLimits, TiIdent):
    g = cls(log=None)
    logged = []
    g.log = logged.append
    prog = VProgress()
    runner = ti.RunAll(seed=5, emit=g.event_for_pick)
    out, gates = [], []
    for k, row in enumerate(rows):
      profile = g.advance_zone(row["v_ego"]).profile
      armed = runner.pick(k, profile, prog.remaining, lambda t, w: g.can_run(t, w, prog.remaining),
                          lambda t, w: g.next_is_strict(t, w, prog.remaining))
      out.append(g.update(armed, k, **row, remaining=prog.remaining))
      gates.append((g.reason, None if g.gate is None else dict(g.gate)))
      prog.add(g.credit)
    res.append((out, logged, gates, dict(g.viol_at)))
  assert res[0] == res[1]


def test_the_on_device_cost_check_runs_and_a_test_runs_in_its_running_mode():
  spec = importlib.util.spec_from_file_location("ti_cost_check_for_test", HERE.parents[0] / "mazda" / "ti_cost_check.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  run = mod.run_mode("tests mode, run", "tests", 38.0, 0.0003, 400)
  idle = mod.run_mode("tests mode, idle", "tests", 38.0, 0.02, 200)
  assert run["test_running_share"] > 0.2 and idle["test_running_share"] == 0.0 and run["mean_us"] > 0


# ---- against the literal pre-change code (95e17e3), with the real test accountant and every kind of progress change ------------------
def _ref_module():
  spec = importlib.util.spec_from_file_location("ti_ident_ref_95e17e3", HERE / "ti_ident_ref_95e17e3.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


REF = _ref_module()


class DictParams:
  def __init__(self):
    self.d = {}

  def get(self, k, encoding=None):
    v = self.d.get(k)
    return v if (v is None or encoding) else (v.encode() if isinstance(v, str) else v)

  def put(self, k, v):
    self.d[k] = v.decode() if isinstance(v, bytes) else v

  put_nonblocking = put

  def get_bool(self, k):
    return self.d.get(k) in ("1", True)

  def remove(self, k):
    self.d.pop(k, None)


def _accountant_pipeline(mod, mode, rows, actions):
  """The car controller's loop (advance_zone, pick, update, the accountant's update and publish) with `mod`'s generator; mode: "old"
  (the 95e17e3 calls on the 95e17e3 generator), "cached" (SchedulerCache with the accountant's version) or "noversion"."""
  import json
  from openpilot.selfdrive.car.mazda import ti_tests as tt
  params, mem = DictParams(), DictParams()
  acc = tt.make_accountant(params, mem)
  g = mod.TiIdent(log=None)
  logged = []
  g.log = logged.append
  runner = mod.RunAll(seed=7, emit=g.event_for_pick)
  cache = ti.SchedulerCache()
  toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
  armed_seq, out, versions = [], [], set()
  for k, row in enumerate(rows):
    act = actions.get(k)
    if act == "publish":
      acc.publish(1000.0 + k * 0.01)
    elif act == "reload":
      acc.doc = None                                   # the next read re-reads the stored progress
    elif act == "reset":                               # a progress reset: one test's stored entry gone, then re-read
      acc.publish(1000.0 + k * 0.01)
      doc = json.loads(params.get(tt.PROGRESS_KEY, encoding="utf8") or "{}")
      doc.pop(next(iter(acc.step_tests.values())).id, None)
      params.put(tt.PROGRESS_KEY, json.dumps(doc))
      mem.remove(tt.PROGRESS_KEY)
      acc.doc = None
    elif act == "fail":
      acc._fail("test")
    profile = g.advance_zone(row["v_ego"]).profile
    if mode == "old":
      armed = runner.pick(k, profile, acc.remaining, lambda t, w: g.can_run(t, w, acc.remaining),
                          lambda t, w: g.next_is_strict(t, w, acc.remaining))
    else:
      rem, can_run, strict = cache.frame(g, acc.remaining, profile, (id(acc), None if mode == "noversion" else acc.version))
      armed = runner.pick(k, profile, rem, can_run, strict)
    armed_seq.append(armed)
    out.append(g.update(armed, k, **row, remaining=acc.remaining))
    acc.update(toggles, row["v_ego"], True, (), credit=g.credit, run_all_forced=True)
    versions.add(getattr(acc, "version", None))
  return armed_seq, out, logged, (runner.draw, runner.current, runner.mode), versions


@pytest.mark.parametrize("seed", range(2))
def test_the_current_code_is_frame_for_frame_the_95e17e3_code_through_publish_reload_reset_and_failure(seed):
  rows = T._random_rows(seed, 14000)
  for k in range(2500, 14000, 3100):
    for j in range(k, k + 40):
      rows[j] = dict(rows[j], lat_demand=float("nan"))
  actions = {k: "publish" for k in range(500, 14000, 500)}
  actions.update({3333: "reload", 7777: "reset", 12500: "fail"})
  old = _accountant_pipeline(REF, "old", rows, actions)
  for mode in ("cached", "noversion"):
    new = _accountant_pipeline(ti, mode, rows, actions)
    assert new[0] == old[0] and new[1] == old[1] and new[2] == old[2] and new[3] == old[3], mode
  assert len(new[4]) > 10                              # the version moved (credits, publishes, the reload, the reset, the failure)
  assert any(a is not None for a in old[0][:12000]) and all(a is None for a in old[0][12501:])   # ran, and nothing after the failure
