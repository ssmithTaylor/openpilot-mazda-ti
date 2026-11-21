"""Drive plan (ti_plan.py): sequences the A/B alternation and the identification tests so that exactly one is allowed at any
moment: the pure decision, the hand-over in both directions, completion of the alternation, absolute exclusion over long
randomised drives with the real test generator, and the failure paths."""
import ast
import json
import random
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.mazda import ti_ab as ab
from openpilot.selfdrive.car.mazda import ti_ident as ti
from openpilot.selfdrive.car.mazda import ti_plan as tp
from openpilot.selfdrive.car.mazda.ti_ident import TiIdent

ROOT = Path(__file__).resolve().parents[3]
MPH = 0.44704
F = lambda s: int(round(s / 0.01))


def v(m):
  return m * MPH


class Mem:
  def __init__(self, **kv):
    self.d = dict(kv)

  def get_bool(self, k):
    return bool(self.d.get(k))

  def get(self, k):
    return self.d.get(k)

  def put(self, k, val):
    self.d[k] = val

  def remove(self, k):
    self.d.pop(k, None)


STORED_KP = 0.8


def toggles(**over):
  t = NS(steerKp=[[0], [STORED_KP]], lat_commit_setpoint=True, lat_damping=False, lat_friction_comp=True, lat_output_filter=False,
         lat_no_friction_relay=True, ti_dither=False, ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False)
  for k, x in over.items():
    setattr(t, k, x)
  return t


CONFIG = json.dumps({"setting": "LatDamping", "b": True, "block_min": 4, "seed": 11})
QUIET = dict(engaged=True, v_ego=v(45.0), curvature=0.0003, steer=0.02, saturated=False, steering_pressed=False, driver_torque=0.0,
             lane_change=False, blinker=False, ti_ok=True, fault=False, controller_ok=True, mono_ns=1)


# ---- the pure decision and its speed hysteresis ---------------------------------------------------------------------------

def test_the_first_trial_constants_and_defaults():
  assert (tp.PLAN_ALT_MIN_MPH, tp.PLAN_ALT_MAX_MPH, tp.TARGET_MIN_PER_SIDE) == (35.0, 56.0, 20.0)
  assert tp.MIN_DWELL_S > 0 and tp.QUIET_SETTLE_S > 0 and tp.STALE_S > 0


@pytest.mark.parametrize("cfg_valid,complete,rng,mode", [
  (True, False, True, "ab"), (True, False, False, "tests"), (False, False, True, "tests"), (False, False, False, "tests"),
  (True, True, True, "tests"), (True, True, False, "tests"), (False, True, True, "tests"),
])
def test_decide_is_a_pure_function_of_state(cfg_valid, complete, rng, mode):
  assert tp.decide(cfg_valid, complete, rng)[0] == mode
  assert tp.decide(cfg_valid, complete, rng) == tp.decide(cfg_valid, complete, rng)


def test_the_range_has_a_mile_of_hysteresis_at_both_ends():
  prev = False
  seen = []
  for mph in (30, 34.9, 35.5, 35.9, 36.1, 40, 54.9, 55.5, 55.9, 56.0, 55.5, 54.9, 40, 35.5, 34.9, 35.5):
    prev = tp.in_range(prev, mph)
    seen.append(prev)
  assert seen == [False, False, False, False, True, True, True, True, True, False, False, True, True, True, False, False]
  assert tp.in_range(True, 80) is False and tp.in_range(False, 80) is False


# ---- the scheduler ---------------------------------------------------------------------------------------------------------

def ack_for(plan, quiet=True, now=100.0):
  return {"seq": plan.seq, "quiet": quiet, "t": now}


def run(plan, n, **kw):
  base = dict(on=True, cfg_valid=True, complete=False, mph=45.0, ab_at_a=True, ack=None, now=100.0, tests_toggles_on=False)
  base.update(kw)
  out = None
  for _ in range(n):
    if "ack_quiet" in base:
      base["ack"] = ack_for(plan, base["ack_quiet"], base["now"])
    kw2 = {k: x for k, x in base.items() if k != "ack_quiet"}
    out = plan.update(**kw2)
  return out


def fresh_plan():
  p = tp.Plan()
  p.update(on=False, cfg_valid=True, complete=False, mph=45.0, ab_at_a=True, ack=None, now=100.0, tests_toggles_on=False)
  return p


def test_plan_off_allows_nothing_and_publishes_off():
  p = tp.Plan()
  out = run(p, 5, on=False)
  assert out.mode == "off" and not out.ab_allowed


def test_a_stale_on_at_start_holds_both_until_the_switch_is_cycled():
  p = tp.Plan()
  assert run(p, 50).mode == "hold"
  run(p, 1, on=False)
  assert run(p, 5, ack_quiet=True).mode in ("ab", "hold")


def test_tests_to_ab_the_alternation_waits_for_the_tests_to_be_quiet_and_settled():
  p = fresh_plan()
  assert run(p, 1, mph=22.0).mode == "hold"
  run(p, 3, mph=22.0, ab_at_a=True)
  assert p.mode == "tests"
  p.dwell = 10 ** 6
  out = run(p, 1, mph=45.0)                                   # now the alternation's range: "ab" published at once
  assert out.mode == "ab" and not out.ab_allowed
  out = run(p, 200, mph=45.0, ack=None)                       # no acknowledgement: it never starts
  assert not out.ab_allowed
  out = run(p, 200, mph=45.0, ack_quiet=False)               # tests still sending: it never starts
  assert not out.ab_allowed
  out = run(p, 50, mph=45.0, ack_quiet=True)
  assert not out.ab_allowed                                    # quiet, but not yet for QUIET_SETTLE_S
  out = run(p, 60, mph=45.0, ack_quiet=True)
  assert out.ab_allowed
  stale = dict(ack_for(p), seq=p.seq - 1)                      # an acknowledgement of the previous sequence does not count
  q = fresh_plan()
  run(q, 3, mph=22.0)
  q.dwell = 10 ** 6
  run(q, 1, mph=45.0)
  assert not run(q, 300, mph=45.0, ack=stale).ab_allowed
  old = ack_for(q, True, now=50.0)                             # an old acknowledgement does not count either
  assert not run(q, 300, mph=45.0, ack=old, now=100.0).ab_allowed


def test_ab_to_tests_the_tests_start_only_when_the_blend_is_at_a_for_its_wash_in():
  p = fresh_plan()
  run(p, 3, mph=45.0)
  run(p, 200, mph=45.0, ack_quiet=True)
  assert p.mode == "ab"
  p.dwell = 10 ** 6
  out = run(p, 3, mph=60.0, ab_at_a=False)                    # speed leaves the range
  assert out.mode == "hold" and not out.ab_allowed
  out = run(p, 500, mph=60.0, ab_at_a=False)                  # the alternation is still ramping back or washing in
  assert out.mode == "hold"
  out = run(p, 2, mph=60.0, ab_at_a=True)
  assert out.mode == "tests"


def test_a_speed_change_waits_out_the_dwell_but_completion_and_a_lost_configuration_do_not():
  p = fresh_plan()
  run(p, 3, mph=45.0)
  run(p, 200, mph=45.0, ack_quiet=True)
  p.dwell = 0
  assert run(p, 100, mph=60.0).mode == "ab"                    # hovering: no flip inside the dwell
  assert run(p, F(tp.MIN_DWELL_S) + 5, mph=60.0).mode in ("hold", "tests")
  q = fresh_plan()
  run(q, 3, mph=45.0)
  q.dwell = 0
  assert run(q, 1, mph=45.0, complete=True).mode == "hold"     # forced: at once
  r = fresh_plan()
  run(r, 3, mph=45.0)
  r.dwell = 0
  assert run(r, 1, mph=45.0, cfg_valid=False).mode == "hold"


def test_hovering_at_a_boundary_makes_no_flip_flop():
  p = fresh_plan()
  run(p, 3, mph=45.0)
  run(p, 200, mph=45.0, ack_quiet=True)
  changes = p.seq
  for k in range(6000):
    mph = 56.0 + (0.4 if (k // 7) % 2 else -0.4)               # across 56 mph
    p.update(on=True, cfg_valid=True, complete=False, mph=mph, ab_at_a=True, ack=ack_for(p), now=100.0, tests_toggles_on=False)
  assert p.seq - changes <= 2 * (6000 // F(tp.MIN_DWELL_S)) + 2


def test_single_test_toggles_on_hold_the_plan():
  p = fresh_plan()
  out = run(p, 5, tests_toggles_on=True)
  assert out.mode == "hold" and not out.ab_allowed


def test_an_unconfigured_alternation_leaves_the_tests_running_everywhere():
  p = fresh_plan()
  for mph in (22.0, 45.0, 52.0, 70.0):
    p.dwell = 10 ** 6
    run(p, 3, mph=mph, cfg_valid=False)
    run(p, 3, mph=mph, cfg_valid=False)
    assert p.mode == "tests"


def test_plan_events_carry_mode_why_and_seq():
  ev = []
  p = tp.Plan(log=lambda kind, **kw: ev.append(dict(kind=kind, **kw)))
  run(p, 2, on=False)
  run(p, 3, mph=22.0)
  run(p, 3, mph=22.0)
  assert [e["mode"] for e in ev] == ["hold", "tests"] and [e["seq"] for e in ev] == [1, 2]
  assert all(e["kind"] == "plan" and e["why"] for e in ev)


# ---- the car controller's side ----------------------------------------------------------------------------------------------

def plan_json(mode, seq=1, t=100.0):
  return json.dumps({"seq": seq, "mode": mode, "t": t})


def test_tests_gate_allows_only_a_fresh_tests_word_of_a_plan_that_is_on():
  assert tp.tests_gate(True, plan_json("tests"), 101.0) == (False, True)
  for mode in ("ab", "hold", "off"):
    assert tp.tests_gate(True, plan_json(mode), 101.0) == (True, False)
  assert tp.tests_gate(True, plan_json("tests"), 110.0) == (True, False)          # stale
  assert tp.tests_gate(True, plan_json("tests", t=200.0), 101.0) == (True, False)  # from the future
  for bad in (None, "", b"\xff", "{", "[1]", json.dumps({"mode": "tests"}), json.dumps({"mode": "tests", "t": "x"})):
    assert tp.tests_gate(True, bad, 101.0) == (True, False), bad
  assert tp.tests_gate(False, plan_json("tests"), 101.0) == (False, False)        # plan off: the tests are as without it
  assert tp.tests_gate(False, None, 101.0) == (False, False)
  assert tp.tests_gate(False, plan_json("dead"), 101.0) == (True, False)          # a plan in error holds whatever the switch says


def test_the_acknowledgement_is_quiet_only_when_no_test_signal_is_in_progress():
  assert tp.make_ack(5, False, 0, 0.0, 1.0)["quiet"] is True
  assert tp.make_ack(5, True, 0, 0.0, 1.0)["quiet"] is False                       # fading
  assert tp.make_ack(5, False, 3, 0.0, 1.0)["quiet"] is False
  assert tp.make_ack(5, False, 0, 0.3, 1.0)["quiet"] is False                      # the dither's envelope
  assert tp.make_ack(5, False, 0, 0.0, 1.0)["seq"] == 5 and tp.plan_seq(plan_json("tests", seq=9)) == 9 and tp.plan_seq("x") is None


def _method(name):
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  cls = next(n for n in ast.parse(src).body if isinstance(n, ast.ClassDef) and n.name == "CarController")
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
  env = {"ti_ident_mod": ti, "ti_plan_mod": tp, "time": time, "json": json}
  exec(compile(ast.Module(body=[fn], type_ignores=[]), name, "exec"), env)
  return env[name]


def cc_host(mem, frame=0):
  return NS(ccp=NS(TI_STEER_DELTA_UP=6, TI_STEER_DELTA_UP_HIGH=6), _ti_limit=lambda *a: 6, frame=frame, params_memory=mem,
            ti_steer_threshold=None, ti_dither_enabled=False, ti_ident_on=(), ti_ident_all=False, ti_ab_on=False,
            ti_plan_on=False, ti_plan_allowed=False, ti_plan_seq=None, ti_plan_ack_key=None)


def test_the_car_controller_forces_run_all_only_while_the_plan_says_tests_and_holds_otherwise():
  apply = _method("apply_ti_tuning")
  now = time.time()
  mem = Mem(TiPlanOn=True, TiPlan=plan_json("tests", 4, now))
  h = cc_host(mem)
  apply(h, toggles())
  assert h.ti_plan_on and h.ti_plan_allowed and h.ti_ident_all and h.ti_ident_on == () and h.ti_ab_on is False and h.ti_plan_seq == 4
  mem.d["TiPlan"] = plan_json("ab", 5, now)
  h.frame = 10
  apply(h, toggles())
  assert not h.ti_plan_allowed and h.ti_ab_on is True and not h.ti_ident_all
  mem.d["TiPlan"] = plan_json("tests", 6, now - 10)             # stale
  h.frame = 20
  apply(h, toggles())
  assert h.ti_ab_on is True and not h.ti_ident_all
  mem.d["TiPlanOn"] = False                                     # the plan off: the separate toggles work as always
  mem.d["TiPlan"] = plan_json("off", 7, now)
  h.frame = 30
  apply(h, toggles(ti_sweep=True))
  assert h.ti_ab_on is False and h.ti_ident_on == (ti.SWEEP,) and not h.ti_ident_all
  mem.d["TiAbOn"] = True                                        # ... and the plain A/B switch still holds the tests
  h.frame = 40
  apply(h, toggles(ti_sweep=True))
  assert h.ti_ab_on is True


def test_a_plan_in_error_holds_the_tests_even_with_the_switch_off_and_an_unreadable_plan_holds():
  apply = _method("apply_ti_tuning")
  mem = Mem(TiPlanOn=False, TiPlan=plan_json("dead", 1, time.time()))
  h = cc_host(mem)
  apply(h, toggles(ti_sweep=True))
  assert h.ti_ab_on is True
  class Bad(Mem):
    def get(self, k):
      raise OSError("x")
  h2 = cc_host(Bad(), frame=10)
  h2.ti_plan_on = True
  apply(h2, toggles())
  assert h2.ti_ab_on is True and not h2.ti_plan_allowed


def test_the_car_controller_acknowledges_the_plan_sequence_it_read():
  pub = _method("publish_plan_ack")
  mem = Mem()
  ident = NS(running=False, last=0)
  h = NS(ti_ident=ident, ti_dither=NS(envelope=0.0), ti_plan_seq=7, params_memory=mem, frame=0, ti_plan_ack_key=None)
  pub(h)
  a = json.loads(mem.d["TiPlanAck"])
  assert a["seq"] == 7 and a["quiet"] is True
  ident.running = True
  h.frame = 1
  pub(h)
  assert json.loads(mem.d["TiPlanAck"])["quiet"] is False
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  assert "self.publish_plan_ack()" in src and "ti_plan_mod.tests_gate" in src


# ---- completion of the alternation -----------------------------------------------------------------------------------------

def rig_ab(target_s=30.0, block_s=8.0, seed=11):
  ev = []
  m = ab.TiAb(log=ev.append, block_s=block_s)
  m.set_plan_limits(tp.PLAN_ALT_MAX_MPH, target_s)
  cfg = json.dumps({"setting": "LatDamping", "b": True, "block_min": 4, "seed": seed})
  base = dict(on=True, pin=None, config=cfg, stored={"LatDamping": False}, tests_active=False, **{k: x for k, x in QUIET.items()})
  m.update(**{**base, "on": False})
  return m, ev, base


def test_completion_counts_only_scored_time_of_non_voided_blocks_then_returns_to_a_and_stays():
  m, ev, base = rig_ab(target_s=30.0)
  outs = []
  for _ in range(F(300)):
    outs.append(m.update(**base))
    if m.complete:
      break
  assert m.complete
  c = [e for e in ev if e["kind"] == "complete"]
  assert len(c) == 1 and c[0]["target_s"] == 30.0 and min(c[0]["committed"].values()) >= 30.0
  # each committed second is scored, non-wash, in a non-voided block: it is below the block time on each arm
  ends = [e for e in ev if e["kind"] == "block_end" and not e["voided"]]
  for arm in "AB":
    assert m.committed[arm] <= sum(e["engaged_s"] for e in ends if e["arm"] == arm) + 1e-9
  for _ in range(F(20)):
    outs.append(m.update(**base))
  assert outs[-1].s == 0.0 and not outs[-1].active and m.mode == "complete"
  assert "complete" in m.message or "stays" in m.message
  for _ in range(F(120)):                                        # the rest of the drive: no block, no switch
    o = m.update(**base)
    assert o.s == 0.0 and not o.active
  assert [e["kind"] for e in ev].count("block_start") == len([e for e in ev if e["kind"] == "block_end"])
  # the return to A was by the normal ramp
  sw = [e for e in ev if e["kind"] == "switch"][-1]
  assert sw["why"] == "release:complete" and sw["s_to"] == 0.0 and sw["ramp_frames"] > 0 or sw["immediate"] is False


def test_voided_blocks_wash_in_and_pinned_time_never_count_toward_completion():
  m, ev, base = rig_ab(target_s=1e9)
  for _ in range(F(14)):
    m.update(**base)
  for _ in range(F(3)):
    m.update(**{**base, "engaged": False})                      # voided
  assert m.committed == {"A": 0.0, "B": 0.0} or all(e["voided"] for e in ev if e["kind"] == "block_end" and e["reason"] != "done")
  m2, ev2, base2 = rig_ab(target_s=1e9, seed=next(s for s in range(100) if ab.arm_for(s, 0) == "B"))
  for _ in range(F(40)):
    m2.update(**{**base2, "pin": "B"})
  assert m2.committed == {"A": 0.0, "B": 0.0}
  # a completed arm B block's scored time is less than its length by at least the wash-in
  m3, ev3, base3 = rig_ab(target_s=1e9, seed=next(s for s in range(100) if ab.arm_for(s, 0) == "B"), block_s=20.0)
  for _ in range(F(60)):
    m3.update(**base3)
  assert 0 < m3.committed["B"] <= 20.0 - ab.SETTINGS["LatDamping"].wash_in_s + 1e-9


def test_the_plan_ceiling_is_56_mph_and_without_the_plan_it_is_70():
  m, ev, base = rig_ab()
  for _ in range(F(3)):
    m.update(**{**base, "v_ego": v(57.0)})
  assert m.mode == "idle" and "56" in m.message
  m.clear_plan_limits()
  for _ in range(F(3)):
    m.update(**{**base, "v_ego": v(57.0)})
  assert m.mode == "running"


# ---- the runner with the plan ---------------------------------------------------------------------------------------------

def make_runner(plan_on=False, target_min=0.5, **kw):
  mem = Mem(TiPlanOn=plan_on)
  params = Mem(TiAbConfig=CONFIG)
  rn = ab.TiAbRunner(params, mem, block_s=8.0, plan_target_min=target_min, **kw)
  return rn, mem


def test_with_the_plan_off_the_runner_is_unchanged_and_publishes_no_plan():
  rn, mem = make_runner(False)
  t = toggles()
  for _ in range(300):
    assert rn.step(t, **QUIET) is t
  assert json.loads(mem.d.get("TiAbStatus", "{}") or "{}").get("plan") in (None, {})
  assert rn.plan.mode == "off" and rn.ab.target_s is None and rn.ab.speed_off_high == ab.SPEED_OFF_HIGH


def test_a_plan_with_the_tests_not_acknowledging_never_starts_the_alternation():
  rn, mem = make_runner(False)
  t = toggles()
  for _ in range(30):
    rn.step(t, **QUIET)
  mem.d["TiPlanOn"] = True
  for _ in range(F(60)):
    rn.step(t, **QUIET)
  assert json.loads(mem.d["TiPlan"])["mode"] == "ab" and rn.ab.block is None and not rn.ab.at_a_settled is False


def test_a_scheduler_exception_turns_both_modes_off_and_leaves_the_owners_toggles(monkeypatch):
  rn, mem = make_runner(True)
  t = toggles()
  for _ in range(30):
    rn.step(t, **QUIET)
  monkeypatch.setattr(rn.plan, "update", lambda **kw: (_ for _ in ()).throw(RuntimeError("boom")))
  for _ in range(20):
    assert rn.step(t, **QUIET) is t
  assert rn.plan_dead and rn.ab.dead
  assert json.loads(mem.d["TiPlan"])["mode"] == "dead"
  assert tp.tests_gate(True, mem.d["TiPlan"], time.time()) == (True, False) and tp.tests_gate(False, mem.d["TiPlan"], time.time()) == (True, False)
  assert json.loads(mem.d["TiAbStatus"])["plan"]["mode"] == "dead"


# ---- absolute exclusion over long randomised drives, with the real test generator ---------------------------------------------

def simulate(seed, seconds=1500, target_min=1.0, block_s=8.0, seg=(4, 25)):
  rng = random.Random(seed)
  mem = Mem(TiPlanOn=False)
  params = Mem(TiAbConfig=json.dumps({"setting": "LatDamping", "b": True, "block_min": 4, "seed": seed}))
  rn = ab.TiAbRunner(params, mem, block_s=block_s, plan_target_min=target_min)
  gen = TiIdent()
  t = toggles()
  hold, allowed, seq = True, False, None
  ack_key = None
  state = dict(mph=45.0, curv=0.0003, touch=False, blink=False, engaged=True)
  stats = dict(test_frames=0, ab_frames=0, both=0, modes=set(), complete_at=None, ab_after_complete=0, ident_while_blend=0, switches=0)
  for k in range(F(seconds)):
    if k % F(rng.uniform(*seg)) == 0 or k == 0:
      state["mph"] = rng.choice([15, 22, 25, 30, 36, 38, 45, 52, 55, 56.5, 60, 66, 72, 85])
      state["curv"] = rng.choice([0.0003, 0.0003, 0.0003, 0.002, 0.006])
      state["engaged"] = rng.random() > 0.08
    state["touch"] = rng.random() < 0.002 if not state["touch"] else rng.random() < 0.97
    state["blink"] = rng.random() < 0.001 if not state["blink"] else rng.random() < 0.99
    if k == F(0.3):
      mem.d["TiPlanOn"] = True
    v_ego = v(state["mph"])
    inputs = dict(engaged=state["engaged"], v_ego=v_ego, curvature=state["curv"], steer=0.03, saturated=False,
                  steering_pressed=state["touch"], driver_torque=-1.0, lane_change=False, blinker=state["blink"], ti_ok=True,
                  fault=False, controller_ok=True, mono_ns=k)
    view = rn.step(t, **inputs)
    # the car controller: reads the plan at 10 Hz, arms the tests only on an allowed plan, acknowledges
    if k % 10 == 0:
      plan_on = bool(mem.get_bool("TiPlanOn"))
      raw = mem.get("TiPlan")
      hold, allowed = tp.tests_gate(plan_on, raw, time.time())
      seq = tp.plan_seq(raw)
    armed = ti.SWEEP if (allowed and not hold) else None
    if state["engaged"]:
      out = gen.update(armed, k, v_ego=v_ego, steering_pressed=state["touch"], driver_torque=-1.0, blinker=state["blink"],
                       base_request=40, ti_max=600, lat_demand=v_ego ** 2 * state["curv"], curvature=state["curv"],
                       lkas_blocked=state["mph"] < 28.0, room=6)
    else:
      gen.reset(k)
      out = 0
    if k % 10 == 0 and mem.get_bool("TiPlanOn"):
      ack = tp.make_ack(seq, gen.running, gen.last, 0.0, time.time())
      if (ack["seq"], ack["quiet"]) != ack_key or k % 50 == 0:
        mem.put("TiPlanAck", json.dumps(ack))
        ack_key = (ack["seq"], ack["quiet"])
    blend = rn.ab.ramp.s != 0.0 or rn.ab.ramp.moving or rn.ab.ramp.to != 0.0
    stats["modes"].add(rn.plan.mode)
    stats["test_frames"] += out != 0
    stats["ab_frames"] += blend
    # the invariant: no test output while the alternation's blend is non-zero, and no running test signal either
    assert not (out != 0 and blend), (seed, k)
    assert not (gen.running and blend), (seed, k)
    assert not (view is not t and gen.running), (seed, k)
    if rn.ab.complete and stats["complete_at"] is None:
      stats["complete_at"] = k
  done = next((e["frame"] for e in rn.ab.events if e["kind"] == "complete"), None)
  if done is not None:                       # after completion no switch ever moves toward B again (the return to A is the release)
    stats["ab_after_complete"] = sum(1 for e in rn.ab.events if e["kind"] == "switch" and e["frame"] > done and e["s_to"] == 1.0)
  stats["switches"] = rn.plan.seq
  return stats, rn


@pytest.mark.parametrize("seed", range(6))
def test_in_no_cycle_is_a_test_output_non_zero_while_the_alternation_blend_is_non_zero(seed):
  stats, rn = simulate(seed)
  assert stats["test_frames"] > 300 and stats["ab_frames"] > 300, stats          # both really ran
  assert {"ab", "tests"} <= stats["modes"] and stats["switches"] >= 6
  assert stats["ab_after_complete"] == 0


def test_the_randomised_runs_include_a_completion_and_the_tests_taking_over_afterwards():
  done = 0
  for seed in range(6):
    stats, rn = simulate(seed, seconds=4000, target_min=0.3, block_s=20.0, seg=(70, 160))
    if stats["complete_at"] is not None:
      done += 1
      assert rn.plan.mode in ("tests", "hold") and stats["ab_after_complete"] == 0
      assert rn.ab.mode == "complete"
  assert done >= 2


def test_the_alternation_never_starts_while_any_test_signal_is_in_progress():
  """Every alternation block start (and every switch) happens on a cycle where the generator has been fully quiet."""
  rng_seed = 3
  stats, rn = simulate(rng_seed)
  starts = [e for e in rn.ab.events if e["kind"] == "switch" and e["why"] in ("block", "pin")]
  assert starts and all(e["s_from"] == 0.0 for e in starts if e["s_to"] == 1.0)


def test_plan_events_let_the_analysis_tell_alternation_time_from_test_time():
  stats, rn = simulate(1, seconds=900)
  plans = [e for e in rn.ab.events if e["kind"] == "plan"]
  assert plans and {"mode", "why", "seq", "frame"} <= set(plans[0])
  assert [e["seq"] for e in plans] == sorted(e["seq"] for e in plans)
  assert {e["mode"] for e in plans} >= {"hold", "ab", "tests"}


def test_all_tests_off_on_the_screen_also_ends_the_drive_plan():
  text = (ROOT / 'selfdrive/ui/qt/sidebar.cc').read_text(encoding='utf-8')
  body = text.split('void TiTests::allOff() {', 1)[1].split('}', 1)[0]
  assert 'putBool("TiPlanOn", false)' in body
