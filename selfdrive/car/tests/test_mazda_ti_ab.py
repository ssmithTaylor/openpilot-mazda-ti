"""On-car A/B alternation (ti_ab.py): the allowlist and its ranges, configuration refusals, the seeded balanced schedule and its
reproduction from the log, the switch gates, waiting and skipping, pins, fail-safe returns, mutual exclusion with the tests,
and the exact reconstruction of the value in force from the logged events."""
import ast
import json
import random
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.mazda import ti_ab as ab
from openpilot.selfdrive.car.mazda import ti_ident as ti

ROOT = Path(__file__).resolve().parents[3]
MPH = 0.44704
FRAMES = lambda s: int(round(s / ab.DT))


def v_mph(m):
  return m * MPH


STORED = {"SteerKP": 0.8, "LatCommitSetpoint": True, "LatDamping": False, "LatFrictionComp": True, "LatOutputFilter": False,
          "LatNoFrictionRelay": True}
BASE = dict(on=True, pin=None, stored=STORED, engaged=True, v_ego=v_mph(38.0), curvature=0.0003, steer=0.02, saturated=False,
            steering_pressed=False, driver_torque=0.0, lane_change=False, blinker=False, ti_ok=True, fault=False,
            controller_ok=True, tests_active=False)


def cfg(**over):
  c = {"setting": "LatDamping", "b": True, "block_min": 4, "seed": 5}
  c.update(over)
  return json.dumps(c)


class Rig:
  """A TiAb with short blocks, fed 100 Hz of quiet straight driving unless told otherwise."""

  def __init__(self, config=None, block_s=10.0, **over):
    self.events = []
    self.ab = ab.TiAb(log=self.events.append, block_s=block_s)
    self.config = cfg() if config is None else config
    self.over = dict(over)
    self.outs = []
    self.step(1, on=False)             # the switch is seen off once: a stale "on" never starts it

  def step(self, n=1, **kw):
    for _ in range(n):
      args = {**BASE, "config": self.config, **self.over, **kw}
      self.outs.append(self.ab.update(**args))
    return self.outs[-1]

  def secs(self, s, **kw):
    return self.step(FRAMES(s), **kw)

  def kinds(self, kind):
    return [e for e in self.events if e["kind"] == kind]


def quiet_until_block_b(r, limit=60.0):
  """Run until the first block with arm B has started (a switch to B done)."""
  for _ in range(FRAMES(limit)):
    r.step()
    if r.ab.block is not None and r.ab.block[1] == "B":
      return
  raise AssertionError("never reached a B block")


# ---- the allowlist -----------------------------------------------------------------------------------------------

def test_the_15_settings_are_each_allowed_or_excluded_with_a_reason():
  fifteen = {"SteerKP", "LatCommitSetpoint", "LatDamping", "LatFrictionComp", "LatOutputFilter", "LatNoFrictionRelay",
             "TiSteerMax", "TiSteerDeltaUp", "TiSteerDeltaDown", "TiSteerDriverAllowance", "TiSteerDriverMultiplier",
             "TiSteerDeltaUpKnee", "TiSteerDeltaUpHigh", "TiSteerThreshold", "ForceAutoTune"}
  assert set(ab.SETTINGS) | set(ab.EXCLUDED) == fifteen and not set(ab.SETTINGS) & set(ab.EXCLUDED)
  assert all(why.strip() for why in ab.EXCLUDED.values())
  # nothing that sets torque authority or a limit, or is read at start-up, is allowed
  assert not any(n.startswith("TiSteer") for n in ab.SETTINGS) and "ForceAutoTune" not in ab.SETTINGS


def test_ranges_are_inside_what_the_product_already_clips_to_and_steps_inside_ranges():
  kp = ab.SETTINGS["SteerKP"]
  car_kp = 0.8
  assert car_kp * 0.5 <= kp.lo < kp.hi <= car_kp * 1.5            # frogpilot_variables clips SteerKP to 0.5x-1.5x the car's
  for s in ab.SETTINGS.values():
    assert s.lo < s.hi and 0 < s.max_step <= s.hi - s.lo
    assert s.wash_in_s >= s.ramp_s > -1 and s.ramp_s >= 0
    assert (s.kind == "bool") == (s.lo == 0 and s.hi == 1)
    assert s.weighted <= (s.kind == "bool")


def test_every_allowlisted_attribute_is_read_live_each_cycle_by_the_controller():
  src = (ROOT / "selfdrive/controls/lib/latcontrol_torque.py").read_text(encoding="utf-8")
  ctl = (ROOT / "selfdrive/controls/controlsd.py").read_text(encoding="utf-8")
  for s in ab.SETTINGS.values():
    if s.name == "SteerKP":
      assert "self.ab_toggles.steerKp" in ctl
    else:
      assert f'"{s.attr}"' in src, s.attr
    if s.weighted:
      assert f'ab_weight(frogpilot_toggles, "{s.attr}")' in src


# ---- configuration ----------------------------------------------------------------------------------------------

@pytest.mark.parametrize("raw,part", [
  (None, "not configured"), ("", "not configured"), ("{not json", "unreadable"), ("[1]", "unreadable"),
  (cfg(setting="TiSteerMax"), "authority"), (cfg(setting="ForceAutoTune"), "learned"), (cfg(setting="Nope"), "not an A/B setting"),
  (cfg(b=None), "on or off"), (cfg(b=0.5), "on or off"), (cfg(b=False), "same as your stored"),
  (cfg(setting="SteerKP", b=1.5), "outside the SteerKP range"), (cfg(setting="SteerKP", b=0.3), "outside"),
  (cfg(setting="SteerKP", b=True), "must be a number"), (cfg(setting="SteerKP", b=float("nan")), "must be a number"),
  (cfg(setting="SteerKP", b=0.8), "same as your stored"), (cfg(setting="SteerKP", b=0.805), "same as your stored"),
  (cfg(a=True), "A is always your stored"), (cfg(setting="SteerKP", a=0.9, b=0.9), "A is always your stored"),
  (cfg(block_min=1), "block length"), (cfg(block_min=11), "block length"), (cfg(block_min="4"), "block length"),
  (cfg(seed=-1), "seed"), (cfg(seed=2 ** 32), "seed"), (cfg(seed=1.5), "seed"),
])
def test_a_configuration_outside_the_allowlist_ranges_or_steps_is_refused_with_a_message(raw, part):
  c, msg = ab.validate(raw, STORED)
  assert c is None and part in msg and len(msg) < 140, msg


def test_a_step_larger_than_the_limit_is_refused_and_one_at_the_limit_is_not():
  assert ab.SETTINGS["SteerKP"].max_step == 0.10 and (ab.SETTINGS["SteerKP"].lo, ab.SETTINGS["SteerKP"].hi) == (0.70, 1.00)
  for b in (0.70, 0.85, 0.9):                                            # around a stored 0.8
    assert ab.validate(cfg(setting="SteerKP", b=b), STORED)[0] is not None, b
  for b in (0.91, 1.0, 0.65, 0.69, 1.05):
    c, msg = ab.validate(cfg(setting="SteerKP", b=b), STORED)
    assert c is None and ("step" in msg or "outside" in msg), (b, msg)


def test_a_stored_value_outside_the_range_or_unreadable_is_refused():
  c, msg = ab.validate(cfg(setting="SteerKP", b=1.0), {**STORED, "SteerKP": 1.2})
  assert c is None and "your stored SteerKP" in msg
  assert ab.validate(cfg(), {**STORED, "LatDamping": None})[0] is None
  assert ab.validate(cfg(), {})[0] is None


def test_value_a_is_the_owners_stored_value_whatever_the_config_says_and_accepts_the_same_value():
  c, _ = ab.validate(cfg(a=False), STORED)
  assert c is not None and c.a is False and c.b is True
  c, _ = ab.validate(cfg(setting="SteerKP", a=0.8, b=0.9), STORED)
  assert c.a == 0.8


def test_an_accepted_configuration_carries_the_default_block_and_a_drawn_seed():
  raw = json.dumps({"setting": "SteerKP", "b": 0.9})
  c, _ = ab.validate(raw, STORED)
  assert c.block_s == ab.DEFAULT_BLOCK_MIN * 60 and c.seed is None
  assert ab.MIN_BLOCK_MIN <= ab.DEFAULT_BLOCK_MIN <= ab.MAX_BLOCK_MIN


# ---- the schedule -------------------------------------------------------------------------------------------------

def test_every_pair_of_blocks_has_one_of_each_arm_and_both_orders_occur():
  firsts = set()
  for seed in range(50):
    for p in range(20):
      pair = (ab.arm_for(seed, 2 * p), ab.arm_for(seed, 2 * p + 1))
      assert sorted(pair) == ["A", "B"]
      firsts.add(pair)
  assert firsts == {("A", "B"), ("B", "A")}


def test_the_order_is_unbiased_over_many_pairs_and_depends_on_the_seed():
  n = 4000
  ab_first = sum(ab.arm_for(1234, 2 * p) == "A" for p in range(n))
  assert abs(ab_first / n - 0.5) < 0.04
  assert [ab.arm_for(1, i) for i in range(40)] != [ab.arm_for(2, i) for i in range(40)]
  assert [ab.arm_for(7, i) for i in range(40)] == [ab.arm_for(7, i) for i in range(40)]


def test_the_draw_is_the_same_hash_the_identification_tests_log_so_a_reader_needs_no_random_module():
  for seed in (0, 5, 2 ** 31):
    for draw in range(10):
      assert ab._uniform(seed, draw) == ti._uniform(seed, draw)


def test_the_logged_blocks_follow_the_seeded_schedule_and_can_be_verified_from_the_log():
  r = Rig(config=cfg(seed=None))
  for _ in range(FRAMES(75.0)):
    r.step()
  starts = r.kinds("block_start")
  assert len(starts) >= 5
  assert ab.verify_schedule(r.events) == []
  seed = r.kinds("config")[0]["seed"]
  assert [e["arm"] for e in starts] == [ab.arm_for(seed, e["idx"]) for e in starts]
  # a tampered log is caught
  bad = [dict(e, arm="A" if e["arm"] == "B" else "B") if e["kind"] == "block_start" and e["idx"] == 2 else e for e in r.events]
  assert [e["idx"] for e in ab.verify_schedule(bad)] == [2]


def test_a_given_seed_is_used_and_logged_and_a_missing_one_is_drawn_and_logged():
  r = Rig(config=cfg(seed=99))
  r.secs(1)
  c = r.kinds("config")[0]
  assert c["seed"] == 99 and c["seed_given"] and c["setting"] == "LatDamping" and c["table"] == ab.TABLE_HASH
  r2 = Rig(config=cfg(seed=None))
  r2.secs(1)
  c2 = r2.kinds("config")[0]
  assert not c2["seed_given"] and 0 <= c2["seed"] < 2 ** 32


def test_pairs_complete_in_engaged_time_on_a_clean_drive_and_blocks_are_the_configured_length():
  r = Rig(block_s=10.0)
  r.secs(95.0)
  ends = r.kinds("block_end")
  assert len(ends) >= 6 and all(e["reason"] == "done" and not e["voided"] for e in ends)
  assert all(abs(e["engaged_s"] - 10.0) < 0.02 for e in ends)
  arms = [e["arm"] for e in ends[:6]]
  assert all(sorted(arms[i:i + 2]) == ["A", "B"] for i in (0, 2, 4))


# ---- value A and "off" ------------------------------------------------------------------------------------------

def test_the_value_in_force_is_exactly_the_owners_stored_value_whenever_arm_a_is_in_force():
  r = Rig()
  seen = 0
  for k in range(FRAMES(60.0)):
    stored = {**STORED, "SteerKP": 0.8 + 0.001 * (k % 7)}
    out = r.step(stored=stored, config=cfg(setting="SteerKP", b=0.8 + 0.05), )
    if out.arm == "A" and not r.ab.ramp.moving and out.setting == "SteerKP":
      assert not out.active and out.s == 0.0     # the controller then gets the owner's own object
      assert out.value == stored["SteerKP"]
      seen += 1
  assert seen > 1000


def test_off_by_default_and_a_stale_on_at_start_never_starts_it():
  ev = []
  m = ab.TiAb(log=ev.append, block_s=10.0)
  outs = [m.update(**{**BASE, "config": cfg()}) for _ in range(FRAMES(30.0))]          # on from the very first frame
  assert not any(o.active for o in outs) and m.mode == "off" and "off and on again" in m.message
  m.update(**{**BASE, "config": cfg(), "on": False})
  outs = [m.update(**{**BASE, "config": cfg()}) for _ in range(FRAMES(30.0))]
  assert m.mode == "running"


def test_a_refused_configuration_runs_nothing_and_says_why_on_the_screen_status():
  r = Rig(config=cfg(setting="SteerKP", b=5.0))
  r.secs(5)
  assert not any(o.active for o in r.outs) and r.ab.mode == "off"
  st = r.ab.status()
  assert st["state"] == "off" and "outside the SteerKP range" in st["msg"]
  assert [e["kind"] for e in r.events if e["kind"] == "refuse"] == ["refuse"]


# ---- switch gates -----------------------------------------------------------------------------------------------

def hold_pending_switch(r, **bad):
  """Start a run whose first block is arm B (so a switch is pending) with the given gate violated."""
  r.step(**bad)


def first_b_seed():
  return next(s for s in range(100) if ab.arm_for(s, 0) == "B")


def first_a_seed():
  return next(s for s in range(100) if ab.arm_for(s, 0) == "A")


def r_pending(**over):
  r = Rig(config=cfg(seed=first_b_seed()), **over)
  return r


GATE_CASES = [
  ("touch", dict(steering_pressed=True)), ("touch", dict(driver_torque=7.0)),
  ("lane_change", dict(lane_change=True)), ("lane_change", dict(blinker=True)),
  ("speed", dict(v_ego=v_mph(18.0))), ("speed", dict(v_ego=v_mph(85.0))),
  ("speed", dict(v_ego=v_mph(34.0))), ("speed", dict(v_ego=v_mph(71.0))),
  ("curve", dict(curvature=0.003)), ("lat", dict(curvature=0.0015, v_ego=v_mph(60.0))),
  ("request", dict(steer=0.4)), ("saturated", dict(saturated=True)), ("input", dict(curvature=float("nan"))),
]


@pytest.mark.parametrize("name,bad", GATE_CASES)
def test_a_switch_waits_while_each_gate_is_violated_and_goes_ahead_once_it_clears(name, bad):
  r = r_pending()
  r.secs(15.0, **bad)
  # nothing but A has been in force: the speed gates are release conditions too, the others only delay the switch
  assert all(o.s == 0.0 and not o.active for o in r.outs)
  if name in ("speed", "input"):
    return
  w = r.ab.status()["wait"]
  assert w is not None and w["reason"] == name and w["arm"] == "B", w
  assert r.kinds("wait_start") and r.kinds("wait_start")[0]["why"] in ("margin", name)       # the first 3 s: the speed margin
  r.secs(10.0)
  assert r.ab.block is not None and r.ab.block[1] == "B" and r.outs[-1].arm == "B"
  assert r.kinds("wait_end")[-1]["outcome"] == "switched" and name in r.kinds("wait_end")[-1]["reasons"]


def test_the_wait_is_logged_and_the_switch_is_skipped_after_the_bound_then_the_next_block_is_tried(monkeypatch):
  monkeypatch.setattr(ab, "MAX_WAIT_S", 5.0)
  r = r_pending()
  r.secs(5.2, curvature=0.003)
  sk = r.kinds("skip")
  assert len(sk) == 1 and sk[0]["idx"] == 0 and sk[0]["arm"] == "B" and sk[0]["kept"] == "A" and "curve" in sk[0]["reasons"]
  assert all(not o.active for o in r.outs)
  r.secs(2.0)                                           # block 1 is arm A: no change is needed, it starts at once
  assert r.ab.block == (1, "A") and r.kinds("block_start")[-1]["switched"] is False
  assert r.ab.status()["skipped"] == 1


def test_the_gates_have_hysteresis_and_a_condition_that_closes_resets_the_settle_time():
  r = r_pending()
  r.secs(2.8)                                           # (the speed has been well inside the range for MARGIN_HOLD_S only from 3.0 s)
  r.step(curvature=0.003)                               # one bad frame
  r.secs(0.9)                                           # not yet settled for SETTLE_S again
  assert r.ab.block is None
  r.secs(0.3)
  assert r.ab.block is not None
  # lateral demand v^2 |k|: open at 0.4, close at 0.5 m/s^2 (at 40 mph: k 0.00125 and 0.00156)
  fast = dict(v_ego=v_mph(40.0))
  r2 = r_pending()
  r2.secs(4.2, curvature=0.0014, **fast)                # between open and close: was never closed, so open (after the margin hold and the settle)
  assert r2.ab.block is not None
  r3 = r_pending()
  r3.step(curvature=0.003, **fast)                      # closes
  r3.secs(5.0, curvature=0.0014, **fast)                # inside the hysteresis band: stays closed
  assert r3.ab.block is None
  r3.secs(3.0, curvature=0.0011, **fast)
  assert r3.ab.block is not None


def test_after_a_touch_a_blinker_or_a_lane_change_the_switch_waits_the_holdoff():
  for kw in (dict(steering_pressed=True), dict(blinker=True), dict(lane_change=True)):
    r = r_pending()
    r.secs(1.0, **kw)
    r.secs(2.5)
    assert r.ab.block is None and r.ab.status()["wait"]["reason"] == "holdoff"
    r.secs(2.0)
    assert r.ab.block is not None


def test_a_touch_needs_confirming_unless_firm_as_in_the_test_gates():
  r = r_pending()
  r.step(driver_torque=4.0)
  r.step(driver_torque=4.0)
  r.step()
  assert r.ab.touch_age > 0                              # two readings of 4 counts are not a touch
  r.step(driver_torque=4.0, n=1)
  r.step(driver_torque=4.0)
  r.step(driver_torque=4.0)
  assert r.ab.touch_age == 0


def test_a_block_already_running_is_not_cut_by_a_gate_only_a_hard_release_ends_it():
  r = r_pending()
  quiet_until_block_b(r)
  r.secs(6.0)
  r.secs(2.0, curvature=0.004, steering_pressed=True, lane_change=True)
  assert r.ab.block is not None and r.ab.block[1] == "B" and r.outs[-1].s == 1.0


def test_the_mode_is_active_only_between_35_and_70_mph_with_a_mile_of_hysteresis():
  assert (ab.ALT_MIN_MPH, ab.ALT_MAX_MPH) == (35.0, 70.0)
  r = running_b()
  r.secs(2.0, v_ego=v_mph(34.5))                         # below 35: released by a ramp
  r.secs(4.0, v_ego=v_mph(34.5))
  assert r.outs[-1].s == 0.0 and r.ab.mode == "idle"
  r.secs(5.0, v_ego=v_mph(35.5))                         # a mile of hysteresis: not back yet
  assert r.ab.mode == "idle"
  r.secs(2.0, v_ego=v_mph(36.5))
  assert r.ab.mode == "running"
  r.secs(2.0, v_ego=v_mph(70.2))
  assert r.ab.mode == "idle"
  r.secs(2.0, v_ego=v_mph(69.5))
  assert r.ab.mode == "idle"
  r.secs(2.0, v_ego=v_mph(68.5))
  assert r.ab.mode == "running"
  for mph in (28.0, 29.5, 31.0):                          # the EPS region is below the band
    q = r_pending()
    q.secs(10.0, v_ego=v_mph(mph))
    assert not any(o.active for o in q.outs)


def test_the_first_trial_defaults_are_named_constants():
  assert (ab.KP_LO, ab.KP_HI, ab.KP_MAX_STEP) == (0.70, 1.00, 0.10)
  assert ab.MIN_BLOCK_MIN == 4.0 and ab.DEFAULT_BLOCK_MIN == 4.0
  assert ab.validate(cfg(block_min=3.5), STORED)[0] is None and ab.validate(cfg(block_min=4), STORED)[0] is not None


# ---- hard releases, fail-safe -----------------------------------------------------------------------------------------

def running_b(**over):
  r = r_pending(**over)
  quiet_until_block_b(r)
  r.secs(6.0)
  assert r.outs[-1].s == 1.0 and r.outs[-1].arm == "B"
  return r


def test_disengaging_returns_to_a_at_once_and_a_short_block_is_voided_and_repeated():
  r = running_b()
  r.step(engaged=False)
  assert r.outs[-1].s == 0.0 and not r.outs[-1].active
  e = r.kinds("block_end")[-1]
  assert e["reason"] == "disengaged" and e["voided"] and r.ab.idx == 0
  assert r.kinds("switch")[-1]["immediate"] and r.kinds("switch")[-1]["why"] == "release:disengaged"
  r.secs(5.0)
  assert r.ab.block == (0, "B") and r.kinds("block_start")[-1]["repeat"] is True


def test_a_long_block_cut_short_is_kept_and_the_next_block_follows(monkeypatch):
  monkeypatch.setattr(ab, "MIN_SCORED_KEEP_S", 3.0)
  r = running_b()
  r.secs(10.0)                                                  # scored time after the wash-in
  r.step(engaged=False)
  assert not r.kinds("block_end")[-1]["voided"] and r.ab.idx == 1


@pytest.mark.parametrize("kw,why", [(dict(fault=True), "fault"), (dict(ti_ok=False), "fault"),
                                    (dict(v_ego=v_mph(18.0)), "speed"), (dict(v_ego=v_mph(80.5)), "speed"),
                                    (dict(tests_active=True), "tests"), (dict(controller_ok=False), "controller"),
                                    (dict(curvature=float("nan")), "input")])
def test_a_fault_or_hard_condition_returns_to_the_owners_value_by_a_ramp_with_no_step(kw, why):
  r = running_b()
  r.step(**kw)
  r.secs(4.0, **kw)
  assert r.outs[-1].s == 0.0 and r.ab.mode == "idle"
  sw = r.kinds("switch")[-1]
  assert sw["why"] == f"release:{why}" and sw["s_to"] == 0.0 and sw["ramp_frames"] > 0 and not sw["immediate"]
  ss = [o.s for o in r.outs[-FRAMES(4.0) - 1:]]
  assert max(abs(y - x) for x, y in zip(ss, ss[1:])) <= 1.0 / sw["ramp_frames"] + 1e-12


def test_switching_the_toggle_off_returns_the_value_by_a_ramp_and_forgets_the_run():
  r = running_b()
  r.secs(0.3, on=False)
  assert r.outs[-1].active and 0 < r.outs[-1].s < 1                      # still blending back
  r.secs(3.0, on=False)
  assert r.outs[-1].s == 0.0 and not r.outs[-1].active and r.ab.mode == "off" and r.ab.cfg is None
  assert r.kinds("switch")[-1]["why"] == "release:off"


def test_a_missing_or_corrupt_config_while_running_releases_and_says_why():
  for bad in (None, "", "{broken", b"\xff\xfe", cfg(b=None)):
    r = running_b()
    r.secs(4.0, config=bad)
    assert r.outs[-1].s == 0.0 and r.ab.mode == "off" and r.ab.message


def test_a_changed_configuration_releases_and_needs_the_switch_cycled():
  r = running_b()
  r.secs(4.0, config=cfg(block_min=5, seed=first_b_seed()))
  assert r.outs[-1].s == 0.0 and r.ab.mode == "off" and "off and on again" in r.ab.message
  r.secs(5.0, config=cfg(block_min=5, seed=first_b_seed()))
  assert r.outs[-1].s == 0.0                               # still off: it did not restart by itself
  r.step(on=False)
  r.secs(5.0, config=cfg(block_min=5, seed=first_b_seed()))
  assert r.ab.mode == "running"


def test_the_owner_changing_his_stored_value_during_a_run_moves_a_and_leaves_the_config_valid_or_stops_it():
  r = Rig(config=cfg(setting="SteerKP", b=0.9, seed=first_a_seed()))
  r.secs(2.0)
  assert r.outs[-1].value == 0.8
  r.secs(1.0, stored={**STORED, "SteerKP": 0.82})
  assert r.outs[-1].value == 0.82                           # A follows what he stored
  r.secs(2.0, stored={**STORED, "SteerKP": 1.2})            # now outside the A/B range: refused
  assert r.ab.mode == "off" and "stored SteerKP" in r.ab.message


def test_any_exception_returns_a_at_once_and_disables_the_mode_for_the_rest_of_the_process(monkeypatch):
  r = running_b()

  def boom(*a, **k):
    raise RuntimeError("boom")
  monkeypatch.setattr(r.ab, "_gate", boom)
  out = r.step()
  assert out.s == 0.0 and not out.active and r.ab.dead and r.kinds("error")
  assert r.ab.status()["state"] == "error" and "boom" in r.ab.status()["msg"]
  r.step(on=False)
  r.secs(10.0)
  assert not any(o.active for o in r.outs[-FRAMES(10.0):])


def test_a_log_callback_that_raises_never_stops_the_controller():
  def bad_log(e):
    raise OSError("disk")
  m = ab.TiAb(log=bad_log, block_s=10.0)
  m.update(**{**BASE, "config": cfg(), "on": False})
  for _ in range(FRAMES(30.0)):
    m.update(**{**BASE, "config": cfg(seed=first_b_seed())})
  assert m.mode == "running" and not m.dead


# ---- pins ----------------------------------------------------------------------------------------------------------

def test_pin_b_waits_for_the_same_gates_as_an_automatic_switch_and_says_why():
  r = r_pending()
  r.secs(0.2, curvature=0.003, steering_pressed=True)
  r.secs(5.0, pin="B", curvature=0.003)                           # a bend: no switch
  assert r.outs[-1].s == 0.0 and not r.outs[-1].active and r.ab.mode == "pinned"
  w = r.ab.status()["wait"]
  assert w["reason"] == "curve" and w["why"].startswith("pin waiting: ") and w["pin"] is True
  assert "curve" in [e["why"] for e in r.kinds("pin_wait")]
  r.secs(0.5, pin="B")                                            # straight again, but not settled for a second
  assert r.outs[-1].s == 0.0
  r.secs(1.0, pin="B")
  assert 0 < r.outs[-1].s < 1 and r.kinds("switch")[-1]["why"] == "pin"
  r.secs(4.0, pin="B")
  assert r.outs[-1].s == 1.0 and r.ab.status()["wait"] is None
  n_blocks = len(r.kinds("block_start"))
  r.secs(40.0, pin="B")
  assert len(r.kinds("block_start")) == n_blocks and r.outs[-1].s == 1.0          # alternation stopped
  assert [e["arm"] for e in r.kinds("pin")] == ["B"]
  r.secs(0.5)                                                   # released: alternation resumes with the next block
  assert r.kinds("unpin") and r.ab.mode == "running"
  r.secs(30.0)
  assert len(r.kinds("block_start")) > n_blocks


def test_pin_a_is_immediate_by_ramp_even_with_the_gates_closed():
  r = running_b()
  r.secs(1.0, pin="A", curvature=0.004, steering_pressed=True)
  assert r.outs[-1].s < 1.0 and r.kinds("switch")[-1]["why"] == "pin"
  r.secs(3.0, pin="A", curvature=0.004)
  assert r.outs[-1].s == 0.0


def test_a_touch_or_blinker_during_a_ramp_toward_b_turns_it_back_but_a_ramp_toward_a_goes_on():
  for kw in (dict(steering_pressed=True), dict(blinker=True), dict(lane_change=True), dict(driver_torque=9.0)):
    r = r_pending()
    for _ in range(FRAMES(6.0)):
      r.step()
      if r.ab.ramp.moving and r.ab.ramp.to == 1.0:
        break
    r.secs(0.4)
    s_before = r.outs[-1].s
    assert 0.0 < s_before < 1.0
    r.secs(0.2, **kw)
    sw = r.kinds("switch")[-1]
    assert sw["why"].startswith("reverse:") and sw["s_to"] == 0.0
    r.secs(3.0, **kw)
    assert r.outs[-1].s == 0.0 and r.ab.ramp.to == 0.0
    be = r.kinds("block_end")[-1]
    assert be["reason"] == "reversed" and be["voided"]
    r.secs(8.0)                                                  # hands off: the block is tried again
    assert r.ab.block is not None
    # per-frame slew never exceeds the ramp's own rate, also at the turn
    ss = [o.s for o in r.outs]
    assert max(abs(y - x) for x, y in zip(ss, ss[1:])) <= 1.0 / (ab.SETTINGS["LatDamping"].ramp_s / ab.DT) + 1e-12
  # a ramp toward A is not interrupted
  r = running_b()
  r.secs(0.5, engaged=True, pin="A")
  s1 = r.outs[-1].s
  r.secs(0.3, pin="A", steering_pressed=True)
  assert r.outs[-1].s < s1 and not any(e["why"].startswith("reverse") for e in r.kinds("switch"))


def test_a_pin_to_b_with_the_touch_during_its_ramp_reverses_and_waits_again():
  r = r_pending()
  r.secs(4.2, pin="B")                                  # the pin waits for the gates, the speed margin included
  r.secs(0.3, pin="B")
  assert 0 < r.outs[-1].s < 1
  r.secs(0.2, pin="B", steering_pressed=True)
  r.secs(3.0, pin="B", steering_pressed=True)
  assert r.outs[-1].s == 0.0 and r.ab.status()["wait"]["why"].startswith("pin waiting")


def test_the_controls_constructor_failure_leaves_the_mode_off():
  src = (ROOT / "selfdrive/controls/controlsd.py").read_text(encoding="utf-8")
  i = src.index("self.ti_ab = None")
  assert "try:" in src[i:i + 120] and "TiAbRunner(self.params, params_memory, log=ti_ab_log)" in src[i:i + 400]
  assert "except Exception:" in src[i:i + 500] and "ti_ab disabled" in src[i:i + 600]


def test_the_commit_arm_is_scored_only_where_it_can_act_and_the_page_says_so():
  assert ab.ACTS_ABOVE_LAT == {"LatCommitSetpoint": 1.0}
  r = Rig(config=cfg(setting="LatCommitSetpoint", b=False, seed=first_b_seed()), block_s=20.0)
  r.secs(15.0)                                                    # straight: nothing scored
  assert sum(sum(d.values()) for d in r.ab.progress.values()) == 0.0 and r.ab.arm_in_force == "B"
  r.secs(10.0, curvature=0.0045, v_ego=v_mph(40.0), saturated=False)   # lateral demand 1.4 m/s^2: scored (gates only delay switches)
  assert sum(r.ab.progress["B"].values()) > 5.0
  d = Rig(config=cfg(seed=first_b_seed()), block_s=20.0)
  d.secs(30.0)
  assert sum(sum(x.values()) for x in d.ab.progress.values()) > 5.0   # other settings are scored everywhere
  note = (ROOT / "selfdrive/car/mazda/TI_AB.md").read_text(encoding="utf-8")
  ui = (ROOT / "selfdrive/ui/qt/sidebar.cc").read_text(encoding="utf-8")
  assert "acts only in bends" in ui and "only in bends" in note


def test_pin_a_returns_the_owners_value_by_a_ramp_and_holds_it():
  r = running_b()
  r.secs(1.0, pin="A")
  assert r.outs[-1].s < 1.0
  r.secs(4.0, pin="A")
  assert r.outs[-1].s == 0.0 and not r.outs[-1].active
  r.secs(30.0, pin="A")
  assert r.outs[-1].s == 0.0


def test_an_unreadable_pin_holds_a():
  r = running_b()
  r.secs(5.0, pin="Q")
  assert r.outs[-1].s == 0.0 and r.ab.pin == "A" and "pin" in r.ab.message
  r2 = running_b()
  r2.secs(5.0, pin=b"\x00")
  assert r2.outs[-1].s == 0.0


def test_a_pin_while_disengaged_changes_nothing_and_applies_when_engaged_again():
  r = r_pending()
  r.secs(2.0, engaged=False, pin="B")
  assert all(o.s == 0.0 for o in r.outs)
  r.secs(6.0, pin="B")
  assert r.outs[-1].s == 1.0


# ---- progress and wash-in -----------------------------------------------------------------------------------------

def test_progress_counts_engaged_seconds_per_arm_and_window_and_leaves_out_the_wash_in():
  r = Rig(config=cfg(seed=first_b_seed()), block_s=20.0)
  r.secs(30.0, v_ego=v_mph(38.0))
  r.secs(25.0, v_ego=v_mph(52.0))
  r.secs(25.0, v_ego=v_mph(60.0))                              # between windows: "gap"
  p = r.ab.status()["progress"]
  total = sum(sum(d.values()) for d in p.values())
  assert total > 40 and set(p["A"]) == set(ab.WINDOW_LABELS) and ab.WINDOW_LABELS[-1] == "gap"
  assert p["A"]["64-80"] == 0 and p["B"]["64-80"] == 0
  assert p["B"]["35-41"] + p["A"]["35-41"] > 0 and p["B"]["49-56"] + p["A"]["49-56"] + p["A"]["gap"] + p["B"]["gap"] > 0
  # wash-in frames are not in it: total is less than the engaged time by at least the first wash-in
  engaged_s = 80.0
  assert total <= engaged_s - r.ab.spec.wash_in_s + 1.0


def test_the_first_seconds_after_each_switch_are_marked_wash_in_in_the_log_and_the_output():
  r = r_pending()
  r.secs(15.0)
  sw = r.kinds("switch")[0]
  assert sw["wash_frames"] == FRAMES(ab.SETTINGS["LatDamping"].wash_in_s) and sw["why"] == "block"
  i = sw["frame"]
  flags = [o.wash_in for o in r.outs[i - 1:i + sw["wash_frames"] + 5]]
  assert all(flags[:sw["wash_frames"]]) and not flags[sw["wash_frames"] + 1]


def test_every_setting_has_a_wash_in_at_least_as_long_as_its_ramp():
  for s in ab.SETTINGS.values():
    assert s.wash_in_s >= s.ramp_s + 2.0


# ---- the log ---------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("seed", range(4))
def test_the_value_in_force_is_rebuilt_exactly_from_the_logged_events_alone(seed):
  rng = random.Random(seed)
  r = Rig(config=cfg(setting="SteerKP", b=0.9, seed=seed), block_s=8.0)
  for _ in range(90):
    r.secs(rng.uniform(1.0, 12.0), engaged=rng.random() > 0.15, steering_pressed=rng.random() < 0.1,
           curvature=rng.choice([0.0003, 0.0003, 0.004]), pin=rng.choice([None, None, None, "A", "B"]),
           fault=rng.random() < 0.05, v_ego=v_mph(rng.choice([15, 38, 38, 45, 52, 60, 52, 70, 90])))
  r.step(on=False)
  r.secs(5.0, on=False)
  rec = ab.reconstruct(r.events, len(r.outs))
  live = [(o.s, o.arm if o.arm else "A", o.wash_in) for o in r.outs]
  assert len(rec) == len(live)
  for k, (x, y) in enumerate(zip(rec, live)):
    assert x[0] == pytest.approx(y[0], abs=1e-12), k
    assert x[2] == y[2], k
    if r.outs[k].arm is not None:
      assert x[1] == y[1], k
  assert len(r.kinds("switch")) > 6 and {e["why"].split(":")[0] for e in r.kinds("switch")} >= {"block", "pin", "release"}


def test_events_are_plain_json_with_a_frame_and_the_documented_fields():
  r = Rig(config=cfg(seed=first_b_seed()), block_s=6.0)
  r.over["mono_ns"] = 123
  r.secs(40.0)
  r.secs(2.0, engaged=False)
  kinds = {e["kind"] for e in r.events}
  assert {"config", "block_start", "block_end", "switch", "hb"} <= kinds
  for e in r.events:
    json.dumps(e)
    assert isinstance(e["frame"], int) and e["frame"] >= 1
  assert all("mono_ns" in e for e in r.events if e["frame"] > FRAMES(1.0))
  assert {"idx", "arm", "switched", "waited_s", "block_s", "first", "repeat"} <= set(r.kinds("block_start")[0])
  assert {"idx", "arm", "engaged_s", "reason", "voided"} <= set(r.kinds("block_end")[0])
  assert {"setting", "why", "s_from", "s_to", "ramp_frames", "wash_frames", "a", "b", "arm", "immediate"} <= set(r.kinds("switch")[0])
  assert {"s", "arm", "wash_left", "idx", "block", "pin", "window", "progress"} <= set(r.kinds("hb")[0])


# ---- the runner (what controlsd calls) ----------------------------------------------------------------------------------

class Mem:
  def __init__(self, **kv):
    self.d = dict(kv)
    self.bad = set()

  def get_bool(self, k):
    if k in self.bad:
      raise OSError("params")
    return bool(self.d.get(k))

  def get(self, k):
    if k in self.bad:
      raise OSError("params")
    return self.d.get(k)

  def put(self, k, v):
    self.d[k] = v


def toggles(**over):
  t = NS(steerKp=[[0], [0.8]], lat_commit_setpoint=True, lat_damping=False, lat_friction_comp=True, lat_output_filter=False,
         lat_no_friction_relay=True, ti_dither=False, ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False,
         nnff=False, ti_steer_max=600)
  for k, v in over.items():
    setattr(t, k, v)
  return t


INPUTS = {k: v for k, v in BASE.items() if k not in ("on", "pin", "stored", "tests_active")}


def runner(mem, config, **kw):
  params = Mem(TiAbConfig=config)
  rn = ab.TiAbRunner(params, mem, block_s=10.0, **kw)
  return rn, params


def test_the_runner_hands_the_controller_the_owners_own_object_whenever_a_is_in_force():
  mem = Mem(TiAbOn=False)
  rn, _ = runner(mem, cfg())
  t = toggles()
  for _ in range(300):
    assert rn.step(t, **INPUTS) is t
  mem.d["TiAbOn"] = True
  seen_view = False
  for _ in range(FRAMES(60.0)):
    view = rn.step(t, **INPUTS)
    if view is not t:
      seen_view = True
      assert isinstance(view, ab.AbToggles) and view.lat_damping is not None and view.ti_steer_max == 600
      with pytest.raises(AttributeError):
        view.lat_damping = True
  assert seen_view
  mem.d["TiAbOn"] = False
  for _ in range(FRAMES(5.0)):
    view = rn.step(t, **INPUTS)
  assert view is t
  assert t.lat_damping is False and t.steerKp == [[0], [0.8]]          # the owner's object is never written


def test_the_runner_never_writes_the_owners_param_and_publishes_a_status():
  mem = Mem(TiAbOn=True)
  rn, params = runner(mem, cfg(setting="SteerKP", b=0.9, seed=first_b_seed()))
  rn.step(toggles(), **INPUTS)
  mem.d["TiAbOn"] = False
  for _ in range(20):
    rn.step(toggles(), **INPUTS)
  mem.d["TiAbOn"] = True
  for _ in range(FRAMES(40.0)):
    rn.step(toggles(), **INPUTS)
  assert set(params.d) <= {"TiAbConfig", "TiAbProgress"}                # the persistent store: the set-up and the experiment's progress only
  assert set(mem.d) <= {"TiAbOn", "TiAbStatus", "TiPlan", "TiPlanOn"}
  st = json.loads(mem.d["TiAbStatus"])
  assert st["setting"] == "SteerKP" and st["a"] == 0.8 and st["b"] == 0.9 and st["state"] == "running"


def test_the_steerkp_view_keeps_the_shape_the_controller_expects_and_blends_a_to_b():
  mem = Mem(TiAbOn=False)
  rn, _ = runner(mem, cfg(setting="SteerKP", b=0.9, seed=first_b_seed()))
  t = toggles()
  for _ in range(20):
    rn.step(t, **INPUTS)
  mem.d["TiAbOn"] = True
  vals = []
  for _ in range(FRAMES(30.0)):
    v = rn.step(t, **INPUTS)
    if v is not t:
      vals.append(v.steerKp)
  assert vals and all(len(x) == 2 and len(x[1]) == 1 and x[0] == [0] for x in vals)
  ks = [x[1][0] for x in vals]
  assert min(ks) >= 0.8 and max(ks) == 0.9 and any(0.8 < k < 0.9 for k in ks)
  rise = ks[:ks.index(0.9) + 1]
  assert all(y >= x for x, y in zip(rise, rise[1:])) and max(y - x for x, y in zip(rise, rise[1:])) < 0.002   # smooth, no overshoot


def test_a_weighted_boolean_view_carries_the_blend_weight():
  mem = Mem(TiAbOn=False)
  rn, _ = runner(mem, cfg(setting="LatDamping", b=True, seed=first_b_seed()))
  t = toggles()
  for _ in range(20):
    rn.step(t, **INPUTS)
  mem.d["TiAbOn"] = True
  ws = []
  for _ in range(FRAMES(30.0)):
    v = rn.step(t, **INPUTS)
    if v is not t:
      ws.append(v.ab_weights["lat_damping"])
  assert ws[0] < 0.1 and ws[-1] > 0.99 and all(0.0 <= w <= 1.0 for w in ws) and any(0.3 < w < 0.7 for w in ws)


def test_tests_on_hold_the_mode_off_and_the_runner_returns_the_owners_toggles():
  for name in ab.TEST_TOGGLES:
    mem = Mem(TiAbOn=False)
    rn, _ = runner(mem, cfg(seed=first_b_seed()))
    t = toggles(**{name: True})
    for _ in range(20):
      rn.step(t, **INPUTS)
    mem.d["TiAbOn"] = True
    for _ in range(FRAMES(40.0)):
      assert rn.step(t, **INPUTS) is t
    assert rn.ab.mode in ("idle", "off") and "tests" in rn.ab.message


def test_a_param_that_cannot_be_read_turns_the_mode_off_and_returns_the_owners_value():
  mem = Mem(TiAbOn=False)
  rn, params = runner(mem, cfg(seed=first_b_seed()))
  t = toggles()
  for _ in range(20):
    rn.step(t, **INPUTS)
  mem.d["TiAbOn"] = True
  for _ in range(FRAMES(30.0)):
    v = rn.step(t, **INPUTS)
  assert rn.ab.mode == "running"
  mem.bad.add("TiAbOn")
  for _ in range(FRAMES(5.0)):
    v = rn.step(t, **INPUTS)
  assert v is t and rn.ab.mode == "off"
  mem.bad.clear()
  params.d["TiAbConfig"] = b"\xff\x00garbage"
  mem.d["TiAbOn"] = False
  for _ in range(100):
    rn.step(t, **INPUTS)
  mem.d["TiAbOn"] = True
  for _ in range(FRAMES(30.0)):
    v = rn.step(t, **INPUTS)
  assert v is t and rn.ab.mode == "off" and "unreadable" in rn.ab.message


def test_an_unreadable_steerkp_in_the_toggles_refuses_and_a_bad_object_never_raises():
  mem = Mem(TiAbOn=False)
  rn, _ = runner(mem, cfg(setting="SteerKP", b=0.9))
  t = toggles(steerKp="nope")
  for _ in range(20):
    rn.step(t, **INPUTS)
  mem.d["TiAbOn"] = True
  for _ in range(100):
    assert rn.step(t, **INPUTS) is t
  assert "unreadable" in rn.ab.message
  assert rn.step(NS(), **INPUTS) is not None


def test_a_module_error_inside_the_runner_returns_the_owners_toggles_forever(monkeypatch):
  mem = Mem(TiAbOn=False)
  rn, _ = runner(mem, cfg(seed=first_b_seed()))
  t = toggles()
  for _ in range(20):
    rn.step(t, **INPUTS)
  monkeypatch.setattr(rn, "_read", lambda: (_ for _ in ()).throw(ValueError("x")))
  for _ in range(40):
    assert rn.step(t, **INPUTS) is t
  assert rn.ab.dead


# ---- exclusion with the tests, car-controller side ------------------------------------------------------------------

def test_the_car_controller_holds_every_test_off_while_the_ab_switch_is_on():
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  tree = ast.parse(src)
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CarController")
  upd = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")
  text = ast.get_source_segment(src, upd)
  assert 'getattr(self, "ti_ab_on", False)' in text
  # both the dither's enable and the identification tests' `free` condition carry it
  assert text.count('getattr(self, "ti_ab_on", False)') == 2
  tune = ast.get_source_segment(src, next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "apply_ti_tuning"))
  assert "TiAbOn" in tune


def test_every_toggle_the_tests_use_is_in_the_exclusion_list_of_the_runner():
  assert set(ab.TEST_TOGGLES) == {ti.TOGGLES[k] for k in ti.KEYS} | {ti.RUN_ALL_TOGGLE, "ti_dither"}


def test_the_new_params_are_registered_in_both_the_native_table_and_the_python_defaults():
  cc = (ROOT / "common/params.cc").read_text(encoding="utf-8")
  py = (ROOT / "frogpilot/common/frogpilot_variables.py").read_text(encoding="utf-8")
  for key, flag in (("TiAbConfig", "PERSISTENT"), ("TiAbOn", "CLEAR_ON_MANAGER_START"), ("TiAbPin", "CLEAR_ON_MANAGER_START"),
                    ("TiAbStatus", "CLEAR_ON_MANAGER_START")):
    assert f'{{"{key}", {flag}}}' in cc
    assert f'("{key}", ' in py
  excluded = py[py.index("EXCLUDED_KEYS = {"):py.index("}", py.index("EXCLUDED_KEYS = {"))]
  for key in ("TiAbConfig", "TiAbOn", "TiAbPin", "TiAbStatus"):
    assert f'"{key}"' in excluded
  assert ab.PARAM_ON == "TiAbOn" and ab.PARAM_PIN == "TiAbPin" and ab.PARAM_CONFIG == "TiAbConfig" and ab.PARAM_STATUS == "TiAbStatus"


def test_a_reboot_starts_off_the_switch_is_a_clear_on_boot_param_and_a_stale_on_needs_cycling():
  cc = (ROOT / "common/params.cc").read_text(encoding="utf-8")
  assert '{"TiAbOn", CLEAR_ON_MANAGER_START}' in cc          # gone at every boot
  m = ab.TiAb(block_s=10.0)
  for _ in range(FRAMES(5.0)):
    m.update(**{**BASE, "config": cfg(seed=first_b_seed())})
  assert m.mode == "off"                                      # (a controls restart with the param still on)


# ---- the car controller with the switch off, on, and the installed build's own code -----------------------------------------
import importlib.util
import subprocess


def _load(name, rel):
  spec = importlib.util.spec_from_file_location(name, ROOT / rel)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


DITHER_TESTS = _load("td_tests_for_ab", "selfdrive/car/tests/test_mazda_ti_dither.py")
IDENT_TESTS = _load("ident_tests_for_ab", "selfdrive/car/tests/test_mazda_ti_ident.py")
INSTALLED = "5194601"


def _blocks_from(src):
  """The steering block of CarController.update as compiled by test_mazda_ti_dither, from any source text."""
  tree = ast.parse(src)
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CarController")
  body = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update").body
  begin = next(i for i, n in enumerate(body) if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == "apply_steer" for t in n.targets))
  reset = next(i for i, n in enumerate(body) if isinstance(n, ast.If)
               and any(isinstance(c, ast.Attribute) and c.attr == "reset" for c in ast.walk(n)))
  builder = next(n for n in body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "new_actuators" for t in n.targets))
  report = next(n for n in body if isinstance(n, ast.If) and any(isinstance(t, ast.Attribute) and t.attr == "steerOutputCan" for t in ast.walk(n)))
  diagnostic = next(n for n in body if isinstance(n, ast.If) and any(isinstance(t, ast.Attribute) and t.attr == "lateral_diagnostics" for t in ast.walk(n)))
  tail = [builder, report, diagnostic]
  return (compile(ast.Module(body=body[begin:reset + 1] + tail, type_ignores=[]), "installed-block", "exec"),
          compile(ast.Module(body=body[begin:reset] + tail, type_ignores=[]), "installed-bare", "exec"))


def _installed_source():
  return subprocess.run(["git", "-C", str(ROOT), "show", f"{INSTALLED}:selfdrive/car/mazda/carcontroller.py"], check=True,
                        capture_output=True, text=True, encoding="utf-8").stdout


def _drive_host(h, seed, n_s=60):
  rows = IDENT_TESTS._rows(seed, DITHER_TESTS.frames(n_s))
  return [IDENT_TESTS.step(h, **r) for r in rows]


@pytest.mark.parametrize("seed", range(2))
@pytest.mark.parametrize("kw", [dict(on=()), dict(on=(ti.SWEEP,)), dict(on=(ti.PULSES,)), dict(run_all=True)])
def test_with_the_switch_off_the_steering_block_is_the_installed_builds_for_the_same_inputs(seed, kw, monkeypatch):
  def mk(attr):
    h = IDENT_TESTS.host(**kw)
    if attr is not None:
      h.ti_ab_on = attr
    if kw.get("run_all"):
      IDENT_TESTS.complete(h.ti_tests.prog, ti.SWEEP, "35-41")
    return h
  new_absent, new_false = _drive_host(mk(None), seed), _drive_host(mk(False), seed)
  full, bare = _blocks_from(_installed_source())
  monkeypatch.setattr(DITHER_TESTS, "BLOCK", full)
  monkeypatch.setattr(DITHER_TESTS, "BARE_BLOCK", bare)
  old = _drive_host(mk(None), seed)
  assert new_absent == old and new_false == old
  if kw.get("on") or kw.get("run_all"):
    assert any(r["ident"] for r in old)                                  # the comparison covers frames where a test sends


@pytest.mark.parametrize("seed", range(2))
@pytest.mark.parametrize("kw", [dict(on=(ti.SWEEP,)), dict(on=(ti.LADDER,)), dict(on=(ti.PULSES,)), dict(run_all=True)])
def test_with_the_switch_on_no_test_sends_anything_and_the_request_is_the_undithered_one(seed, kw):
  h = IDENT_TESTS.host(**kw)
  h.ti_ab_on = True
  ref = DITHER_TESTS.host(False, False)
  rows = IDENT_TESTS._rows(seed, DITHER_TESTS.frames(60))
  for row in rows:
    a = IDENT_TESTS.step(h, **row)
    b = DITHER_TESTS.step(ref, **{k: v for k, v in row.items() if k != "blocked"})
    assert a["ident"] == 0 and h.ti_ident_armed is None
    assert {k: a[k] for k in b} == b


def test_with_the_switch_on_the_dither_is_held_off_too_and_comes_back_when_it_goes_off():
  h = IDENT_TESTS.host(dither=True, ident=False)
  h.ti_ab_on = True
  rows = [IDENT_TESTS.step(h, 0.05) for _ in range(DITHER_TESTS.frames(10))]
  assert all(r["dither"] == 0 for r in rows)
  h.ti_ab_on = False
  rows = [IDENT_TESTS.step(h, 0.05) for _ in range(DITHER_TESTS.frames(10))]
  assert any(r["dither"] for r in rows)


def test_a_test_running_when_the_switch_goes_on_fades_out_and_a_test_cannot_start_while_it_is_on():
  h = IDENT_TESTS.host(on=(ti.SWEEP,))
  rows = [IDENT_TESTS.step(h, 0.05) for _ in range(DITHER_TESTS.frames(10))]
  assert any(r["ident"] for r in rows)
  h.ti_ab_on = True
  tail = [IDENT_TESTS.step(h, 0.05) for _ in range(DITHER_TESTS.frames(10))]
  assert all(r["ident"] == 0 for r in tail[DITHER_TESTS.frames(3):])          # faded out inside the generator's own fade time
  assert max(abs(r["ident"]) for r in tail) <= ti.peak_counts(ti.SWEEP)
  assert h.ti_ident_armed is None


def _apply_ti_tuning():
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  cls = next(n for n in ast.parse(src).body if isinstance(n, ast.ClassDef) and n.name == "CarController")
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "apply_ti_tuning")
  from openpilot.selfdrive.car.mazda import ti_plan as tp
  import time as _time
  env = {"ti_ident_mod": ti, "ti_plan_mod": tp, "time": _time, "ti_hold_log": lambda e: None}
  exec(compile(ast.Module(body=[fn], type_ignores=[]), "apply_ti_tuning", "exec"), env)
  return env["apply_ti_tuning"]


def test_the_car_controller_reads_the_switch_at_10_hz_keeps_the_last_value_on_an_error_and_starts_off():
  apply = _apply_ti_tuning()
  mem = Mem(TiAbOn=False)
  h = NS(ccp=NS(TI_STEER_DELTA_UP=6, TI_STEER_DELTA_UP_HIGH=6), _ti_limit=lambda *a: 6, frame=0, params_memory=mem,
         ti_steer_threshold=None, ti_dither_enabled=False, ti_ident_on=(), ti_ident_all=False, ti_ab_on=False,
         ti_plan_on=False, ti_plan_allowed=False, ti_plan_seq=None, ti_plan_nonce=None, ti_hold_key=None, ti_hold_published=None)
  t = toggles()
  apply(h, t)
  assert h.ti_ab_on is False
  mem.d["TiAbOn"] = True
  h.frame = 5
  apply(h, t)
  assert h.ti_ab_on is False                                     # not yet: read every 10th frame
  h.frame = 10
  apply(h, t)
  assert h.ti_ab_on is True
  mem.bad.add("TiAbOn")
  h.frame = 20
  apply(h, t)
  assert h.ti_ab_on is True                                      # unreadable: keeps the last, the A/B side refuses on its own
  mem.bad.clear()
  mem.d["TiAbOn"] = False
  h.frame = 30
  apply(h, t)
  assert h.ti_ab_on is False


def test_the_format_note_names_every_event_and_every_allowed_and_excluded_setting():
  note = (ROOT / "selfdrive/car/mazda/TI_AB.md").read_text(encoding="utf-8")
  for name in list(ab.SETTINGS) + list(ab.EXCLUDED):
    assert name in note, name
  for kind in ("config", "block_start", "block_end", "wait_start", "wait_end", "skip", "switch", "pin", "pin_wait", "unpin", "release",
               "refuse", "error", "hb"):
    assert f"`{kind}`" in note, kind
  r = Rig(config=cfg(seed=first_b_seed()), block_s=6.0)
  r.secs(60.0, pin=None)
  r.secs(5.0, pin="B")
  r.secs(5.0, engaged=False)
  r.secs(1.0, config=cfg(setting="SteerKP", b=9.0))
  for e in r.events:
    assert e["kind"] in {"config", "block_start", "block_end", "wait_start", "wait_end", "skip", "switch", "pin", "pin_wait", "unpin",
                         "release", "refuse", "error", "hb"}, e["kind"]


def test_the_toggles_view_passes_everything_else_through_and_defaults_work():
  base = toggles()
  view = ab.AbToggles(base, {"lat_damping": True, "ab_weights": {"lat_damping": 0.4}})
  assert view.lat_damping is True and view.lat_commit_setpoint is True and view.ti_steer_max == 600
  assert getattr(view, "missing_thing", "d") == "d" and getattr(base, "ab_weights", None) is None
  assert view.ab_weights == {"lat_damping": 0.4}


def test_the_page_switches_the_tests_off_before_it_starts_and_a_test_switches_it_off():
  ui = (ROOT / "selfdrive/ui/qt/sidebar.cc").read_text(encoding="utf-8")
  assert ui.count('tests_params_memory.putBool("TiAbOn", false)') == 1                   # TiTests::toggle (on); startAll writes it through ti_pages::runAllOps
  hdr = (ROOT / "selfdrive/ui/qt/ti_pages_logic.h").read_text(encoding="utf-8")
  assert hdr.count("memBool(KEY_AB_ON, false)") >= 3                                      # RUN ALL, the plan start and STOP switch the plain A/B off
  start = ui.index("void TestsWindow::abSetOn(bool on)")
  body = ui[start:ui.index("void TestsWindow::abAct", start)]
  assert body.index("tests.allOff()") < body.index("putBool(TI_AB_ON_KEY, on)")
  assert 'TI_AB_ON_KEY = "TiAbOn"' in ui and 'TI_AB_PIN_KEY = "TiAbPin"' in ui and 'TI_AB_CONFIG_KEY = "TiAbConfig"' in ui
  # the page writes no tuning setting: its only writes are the three keys
  writes = set(__import__("re").findall(r"ab_(?:memory|params)\.(?:put|putBool|remove)\((\w+)", ui))
  assert writes <= {"TI_AB_ON_KEY", "TI_AB_PIN_KEY", "TI_AB_CONFIG_KEY", "TI_AB_PROGRESS_KEY", "TI_PLAN_ON_KEY"}, writes
  # the lock is the Tests page's own (the A/B page is a page of the same window)
  assert "class TestsWindow" in (ROOT / "selfdrive/ui/qt/sidebar.h").read_text(encoding="utf-8")
  assert "bool abStoredValue" in (ROOT / "selfdrive/ui/qt/sidebar.h").read_text(encoding="utf-8")


def test_a_cycle_of_the_runner_is_cheap_enough_for_the_100_hz_thread():
  import time
  mem = Mem(TiAbOn=True)
  rn, _ = runner(mem, cfg(setting="SteerKP", b=0.9, seed=first_b_seed()))
  t = toggles()
  rn.step(t, **INPUTS)
  mem.d["TiAbOn"] = False
  rn.step(t, **INPUTS)
  for _ in range(20):
    rn.step(t, **INPUTS)
  mem.d["TiAbOn"] = True
  t0 = time.perf_counter()
  n = 20000
  for _ in range(n):
    rn.step(t, **INPUTS)
  per = (time.perf_counter() - t0) / n
  assert per < 400e-6, per                       # on a PC; the device is several times slower: see the report for the figure


# ---- progress that survives an ignition cycle -------------------------------------------------------------------------------

def _persisting_rig(raw=None, target_s=None, **over):
  """A Rig whose TiAb has the persistent record `raw` and a save function that keeps what is written (None clears)."""
  r = Rig(config=cfg(seed=first_b_seed()), **over)
  r.written = []
  r.ab.stored_raw = raw
  r.ab.save_fn = lambda v: r.written.append(v)
  if target_s is not None:
    r.ab.set_plan_limits(56.0, target_s)
  return r


def test_scored_minutes_are_written_at_each_block_end_and_restored_for_the_same_experiment():
  r = _persisting_rig(target_s=600.0)
  r.secs(75.0)
  r.secs(75.0)
  assert r.written and r.written[-1] is not None
  doc = ab.parse_progress(r.written[-1])
  assert doc is not None and doc["committed"]["A"] + doc["committed"]["B"] > 0.0
  # a new process (an ignition cycle): same experiment, same stored record
  n = _persisting_rig(raw=r.written[-1], target_s=600.0)
  n.step(5)
  assert n.ab.committed == doc["committed"]
  assert all(n.ab.progress[a][w] >= doc["progress"][a][w] for a in "AB" for w in doc["progress"][a])


def test_restored_progress_is_judged_for_completion_and_never_starts_the_mode():
  r = _persisting_rig(target_s=30.0)
  r.secs(100.0)
  r.secs(100.0)
  raw = r.written[-1]
  doc = ab.parse_progress(raw)
  assert doc["committed"]["A"] >= 30.0 and doc["committed"]["B"] >= 30.0
  # a fresh process with the switch OFF: the stored progress fills nothing and nothing runs
  n = _persisting_rig(raw=raw, target_s=30.0)
  n.config = cfg(seed=first_b_seed())
  out = n.step(50, on=False)
  assert not out.active and n.ab.mode == "off" and n.ab.cfg_key is None and not n.ab.complete
  assert n.ab.committed == {"A": 0.0, "B": 0.0}
  # switched on (valid, engaged): completion is judged on the restored total at once, the value stays A and no block starts
  n.step(5, on=True)
  assert n.ab.complete and n.ab.block is None and all(not o.active for o in n.outs[-5:])
  assert n.ab.mode == "complete"


def test_a_corrupt_stored_record_falls_back_to_zero():
  for raw in ("{", "[]", "null", json.dumps({"v": 1}), json.dumps({"v": 2, "key": [], "committed": {}, "progress": {}}),
              b"\xff\xfe", ""):
    n = _persisting_rig(raw=raw, target_s=600.0)
    n.step(5)
    assert n.ab.committed == {"A": 0.0, "B": 0.0} and not n.ab.complete
  r = _persisting_rig(target_s=600.0)
  r.secs(150.0)
  good = json.loads(r.written[-1])
  for bad in (-5.0, float("nan"), 1e12, "7", True, None):
    d = json.loads(json.dumps(good))
    d["committed"]["A"] = bad
    n = _persisting_rig(raw=json.dumps(d), target_s=600.0)
    n.step(5)
    assert n.ab.committed == {"A": 0.0, "B": 0.0}


def test_a_different_experiment_starts_from_zero_and_clears_the_stored_record():
  r = _persisting_rig(target_s=600.0)
  r.secs(150.0)
  raw = r.written[-1]
  n = _persisting_rig(raw=raw, target_s=600.0)
  n.config = cfg(seed=first_b_seed(), setting="LatFrictionComp", b=False)   # another setting
  n.step(5)
  assert n.ab.committed == {"A": 0.0, "B": 0.0} and n.written == [None]
  m = _persisting_rig(raw=raw, target_s=600.0)
  m.over["stored"] = {**STORED, "LatDamping": True}                      # the owner changed his own value A
  m.config = cfg(seed=first_b_seed(), b=False)
  m.step(5)
  assert m.ab.committed == {"A": 0.0, "B": 0.0} and m.written == [None]


def test_the_progress_param_is_registered_persistent_and_the_runner_reads_and_writes_it():
  cc = (ROOT / "common/params.cc").read_text(encoding="utf-8")
  assert '{"TiAbProgress", PERSISTENT}' in cc and ab.PARAM_PROGRESS == "TiAbProgress"
  mem = Mem()
  par = Mem()
  par.remove = lambda k: par.d.pop(k, None)
  rn = ab.TiAbRunner(par, mem)
  assert rn.ab.save_fn is not None
  rn._save_progress("x")
  assert par.d["TiAbProgress"] == "x"
  rn._save_progress(None)
  assert "TiAbProgress" not in par.d


# ---- fewer wasted blocks: a start well inside the range, partial blocks kept by their scored time ---------------------------

def _started_within(mph, seconds=8.0, plan_ceiling=None):
  r = r_pending()
  if plan_ceiling is not None:
    r.ab.set_plan_limits(plan_ceiling, 1200.0)
  r.secs(seconds, v_ego=v_mph(mph))
  return r


def test_a_block_starts_only_when_the_speed_is_well_inside_either_edge_for_a_few_seconds():
  assert "margin" in ab.PRIORITY and ab.MARGIN_MPH == 3.0 and ab.MARGIN_HOLD_S == 3.0
  for mph in (36.0, 37.0, 37.9):                                  # ready for the plain gate (from 36 mph) but at the edge: no block
    r = _started_within(mph)
    assert r.ab.block is None and r.ab.status()["wait"]["reason"] == "margin", mph
  assert _started_within(38.0).ab.block is not None and _started_within(45.0).ab.block is not None
  for mph in (67.5, 68.5):                                        # the plain range ends at 70
    assert _started_within(mph).ab.block is None
  assert _started_within(66.0).ab.block is not None
  for mph in (53.5, 54.5):                                        # under the drive plan the range ends at 56
    assert _started_within(mph, plan_ceiling=56.0).ab.block is None, mph
  assert _started_within(52.0, plan_ceiling=56.0).ab.block is not None
  # held for the few seconds: not at once
  r = r_pending()
  r.secs(3.5, v_ego=v_mph(45.0))                                  # MARGIN_HOLD_S, then the usual SETTLE_S
  assert r.ab.block is None and r.ab.status()["wait"]["reason"] in ("margin", "settle")
  r.secs(1.0, v_ego=v_mph(45.0))
  assert r.ab.block is not None
  # leaving the comfortable band for a moment (still in range) restarts the hold
  r = r_pending()
  r.secs(2.0, v_ego=v_mph(45.0))
  r.secs(0.5, v_ego=v_mph(36.5))
  r.secs(2.0, v_ego=v_mph(45.0))
  assert r.ab.block is None
  r.secs(2.5, v_ego=v_mph(45.0))
  assert r.ab.block is not None


def test_the_margin_loosens_nothing_a_block_already_running_still_ends_at_the_old_edges():
  r = r_pending()
  r.secs(8.0, v_ego=v_mph(45.0))
  assert r.ab.block is not None
  r.secs(1.0, v_ego=v_mph(36.5))                                  # near the edge, still inside: the running block goes on
  assert r.ab.block is not None
  r.secs(1.0, v_ego=v_mph(34.0))                                  # below 35: released as before
  assert r.outs[-1].s < 1.0 and r.kinds("block_end")[-1]["reason"] == "speed"


def test_a_partial_block_keeps_its_scored_time_when_it_reached_the_minimum_and_is_voided_below_it(monkeypatch):
  assert ab.MIN_SCORED_KEEP_S == 30.0 and not hasattr(ab, "MIN_BLOCK_KEEP_S")
  r = running_b(block_s=240.0)
  r.secs(20.0)                                                    # after the wash-in: about 20 s scored
  r.secs(1.0, v_ego=v_mph(34.0))                                  # the speed leaves the range
  e = r.kinds("block_end")[-1]
  assert e["reason"] == "speed" and e["voided"] and 10.0 < e["scored_s"] < 30.0 and r.ab.committed["B"] == 0.0 and r.ab.idx == 0
  r2 = running_b(block_s=240.0)
  r2.secs(45.0)
  r2.secs(1.0, v_ego=v_mph(34.0))
  e2 = r2.kinds("block_end")[-1]
  assert e2["reason"] == "speed" and not e2["voided"] and e2["scored_s"] >= 30.0
  assert abs(r2.ab.committed["B"] - e2["scored_s"]) < 0.02 and r2.ab.idx == 1       # kept, and the next block follows
  # the same for a disengagement and a hand-over (the plan's tests flag)
  for kw in (dict(engaged=False), dict(tests_active=True)):
    r3 = running_b(block_s=240.0)
    r3.secs(45.0)
    r3.secs(1.0, **kw)
    e3 = r3.kinds("block_end")[-1]
    assert not e3["voided"] and e3["scored_s"] >= 30.0 and r3.ab.committed["B"] >= 30.0
  # events carry what the analysis needs: the time in force comes from the switch events, not the block label
  assert {"idx", "arm", "engaged_s", "scored_s", "reason", "voided"} <= set(e2)


def test_the_next_block_is_the_side_that_is_behind_when_the_sides_differ_and_the_seeded_one_otherwise():
  r = Rig(config=cfg(seed=first_b_seed()), block_s=10.0)
  arm, why = r.ab._next_arm(0)
  assert (arm, why) == (ab.arm_for(r.ab.seed if r.ab.seed else first_b_seed(), 0), "seed") or why == "seed"
  r.secs(1.0)
  r.ab.committed = {"A": 100.0, "B": 100.0 + ab.BALANCE_MARGIN_S}              # exactly at the margin: still the seeded order
  assert r.ab._next_arm(5)[1] == "seed"
  r.ab.committed = {"A": 100.0, "B": 100.0 + ab.BALANCE_MARGIN_S + 1.0}        # B is ahead: A next, whatever the seed says
  assert r.ab._next_arm(5) == ("A", "balance") and r.ab._next_arm(6) == ("A", "balance")
  r.ab.committed = {"A": 300.0, "B": 100.0}
  assert r.ab._next_arm(0) == ("B", "balance")


def test_a_balanced_block_is_logged_with_its_reason_and_the_schedule_check_skips_it():
  r = Rig(config=cfg(seed=first_a_seed()), block_s=10.0)
  r.secs(0.5)                                                                   # block 0 (A, by the seed) starts at once: A is in force
  r.ab.committed = {"A": 0.0, "B": 400.0}                                       # B far ahead: block 1 must be A although the seed says B
  r.secs(12.0)
  starts = r.kinds("block_start")
  assert len(starts) >= 2 and starts[0]["arm_why"] == "seed" and starts[1]["arm"] == "A" and starts[1]["arm_why"] == "balance"
  assert ab.arm_for(first_a_seed(), starts[1]["idx"]) == "B"
  assert ab.verify_schedule(r.events) == []


# ---- saturation: B in force and the request at its limit for a while goes back to A ----------------------------------------------

def test_a_request_at_its_limit_for_half_a_second_on_b_returns_to_a_by_a_ramp_and_never_the_other_way():
  r = running_b()
  r.secs(0.4, steer=1.0)                                                # a moment at the limit: nothing happens
  assert r.outs[-1].s == 1.0 and not r.kinds("switch")[-1]["why"].startswith("release")
  r.step(steer=0.1)                                                     # one cycle off the limit resets the count
  r.secs(0.4, steer=1.0)
  assert r.outs[-1].s == 1.0
  r.secs(0.2, steer=1.0)                                                # half a second in a row: back to A
  assert r.outs[-1].s < 1.0
  sw = r.kinds("switch")[-1]
  assert sw["why"] == "release:saturation" and sw["s_to"] == 0.0 and sw["ramp_frames"] > 0 and not sw["immediate"]
  r.secs(4.0, steer=1.0)
  ss = [o.s for o in r.outs[-FRAMES(4.0) - 1:]]
  assert r.outs[-1].s == 0.0 and max(abs(y - x) for x, y in zip(ss, ss[1:])) <= 1.0 / sw["ramp_frames"] + 1e-12
  assert r.kinds("block_end")[-1]["reason"] == "saturation"
  r.secs(20.0, steer=1.0)                                               # saturated: B is not blended in again
  assert all(o.s == 0.0 for o in r.outs[-FRAMES(20.0):])


def test_saturation_on_the_a_side_changes_nothing():
  r = r_pending()
  r.secs(0.2)
  n = len(r.kinds("switch"))
  r.secs(5.0, steer=1.0)
  assert len(r.kinds("switch")) == n and r.outs[-1].s == 0.0
