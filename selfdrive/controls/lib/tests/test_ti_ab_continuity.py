"""A/B alternation and the lateral controller (selfdrive/car/mazda/ti_ab.py, latcontrol_torque.py).

Three things are proved with the real `LatControlTorque` plant branch on identical inputs:
  1. Continuity. For every allowlisted setting, the change a switch makes to the steering request has a bounded per-frame
     slew (CONTINUITY_COUNTS, a third of the default TI rate limit), and where a plain step of the setting WOULD make a
     visible jump the test shows the jump (so the bound is not vacuous).
  2. Off means identical. With no A/B view, with a view that overrides nothing, with the mode switched off, or with the
     blend at 0 (arm A), the controller's outputs and its whole log are bit-for-bit those of the installed build
     (commit 5194601's latcontrol_torque.py, read from git).
  3. Blend weights at their ends equal the plain toggles (1.0 = on, 0.0 = off), so the blend is a true interpolation.
"""
import importlib.util
import math
import random
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from cereal import car, log
from openpilot.common.mock.generators import generate_liveLocationKalman
from openpilot.selfdrive.car.mazda import ti_ab as ab
from openpilot.selfdrive.car.mazda.lateral_plant import TiLateralPlant
from openpilot.selfdrive.controls.lib import latcontrol_torque as lct
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel

ROOT = Path(__file__).resolve().parents[4]
DT = 0.01
U_MAX = 600.0
COUNT = 1.0 / U_MAX
CONTINUITY_COUNTS = 2.0          # largest change per frame the switch itself may add to the request (default TI_STEER_DELTA_UP is 6)
INSTALLED = "5194601"            # the build on the car today


def make_cp():
  CP = car.CarParams.new_message()
  CP.steerRatio = 15.5
  CP.wheelbase = 2.7
  CP.mass = 1650.0
  CP.centerToFront = 1.2
  CP.tireStiffnessFactor = 0.7
  CP.tireStiffnessFront = 192150.0
  CP.tireStiffnessRear = 202500.0
  CP.lateralTuning.init("torque")
  CP.lateralTuning.torque.kp = 0.8
  CP.lateralTuning.torque.ki = 0.2
  CP.lateralTuning.torque.latAccelFactor = 1.4
  CP.lateralTuning.torque.friction = 0.05
  CP.steerActuatorDelay = 0.3
  return CP


class FakeCI:
  def __init__(self):
    self.lateral_plant = TiLateralPlant(U_MAX)

  def torque_from_lateral_accel(self):
    return lambda la, tp: la / tp.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda t, tp: t * tp.latAccelFactor


def owner_toggles(**over):
  t = SimpleNamespace(steerKp=[[0], [0.8]], lat_commit_setpoint=False, lat_damping=False, lat_friction_comp=False,
                      lat_output_filter=False, lat_no_friction_relay=False, ti_steer_max=U_MAX, ti_dither=False, ti_sweep=False,
                      ti_ladder=False, ti_pulses=False, ti_ident_all=False)
  for k, v in over.items():
    setattr(t, k, v)
  return t


def fp_state():
  return SimpleNamespace(lkasBlocked=False, lkasEffective=250.0, tiActive=True, columnTorque=0.0)


def make(module=lct, speed=20.0):
  CP = make_cp()
  c = module.LatControlTorque(CP, FakeCI(), DT)
  return c, VehicleModel(CP), speed


def one_step(c, VM, V, desired_la, measured_la, toggles):
  CS = car.CarState.new_message()
  CS.vEgo = V
  CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-measured_la / V ** 2, V, 0.0))
  params = log.LiveParametersData.new_message()
  out = c.update(True, CS, VM, params, False, desired_la / V ** 2, False, 0.3, generate_liveLocationKalman(), None, toggles,
                 fp_state())
  # controlsd.update_frogpilot_variables: the PID's kp is rewritten from the toggles at the end of every cycle
  if hasattr(c, "pid"):
    c.pid._k_p = toggles.steerKp
  return out


# ---- scenarios: open-loop (desired, measured) lateral acceleration in m/s^2, chosen so the term under test is alive --------
def ripple(k, amp):
  # deterministic pseudo-noise, identical in every run
  return amp * (math.sin(k * 0.37) + 0.6 * math.sin(k * 1.31 + 1.0) + 0.4 * math.sin(k * 2.9 + 2.0))


def lagged(f, k, lag=25):
  return f(max(k - lag, 0))


SCENARIOS = {
  # a standing error: P and the friction relay are non-zero
  "SteerKP": lambda k: (0.30, 0.10 + ripple(k, 0.01)),
  "LatNoFrictionRelay": lambda k: (0.30, 0.10 + ripple(k, 0.01)),
  # a swinging measurement beyond the rate deadzone: the D term is alive
  "LatDamping": lambda k: (0.30, 0.30 + 0.25 * math.sin(2 * math.pi * 1.0 * k * DT)),
  # a sustained demand: the compensation is alive
  "LatFrictionComp": lambda k: (0.80, 0.70 + ripple(k, 0.01)),
  # ripple on the measurement: the command has content for the filter to remove
  "LatOutputFilter": lambda k: (0.40, 0.30 + ripple(k, 0.04)),
  # a demand above the commitment gate that rises and falls slowly, the car following it 0.25 s behind
  "LatCommitSetpoint": lambda k: (1.9 + 0.6 * math.sin(2 * math.pi * 0.25 * k * DT),
                                  1.9 + 0.6 * math.sin(2 * math.pi * 0.25 * (k - 25) * DT) + ripple(k, 0.01)),
}
OWNER_ON = {"LatNoFrictionRelay": True}      # settings whose "off" (A) value is True in the owner's file; the others start False


def b_value(name):
  spec = ab.SETTINGS[name]
  return 0.8 + 0.2 if spec.kind == "float" else not OWNER_ON.get(name, False)


def drive(name, mode, n=900, switch_at=300, speed=20.0, scenario=None):
  """The controller's normalised steering output per frame. mode: base (A all along), ab (the module blends to B at
  `switch_at` by a pin), step (the setting jumps to B at `switch_at`, no blend), ab_off (the module's view with s = 0)."""
  spec = ab.SETTINGS[name]
  scenario = scenario or SCENARIOS[name]
  c, VM, V = make(speed=speed)
  stored = 0.8 if spec.kind == "float" else OWNER_ON.get(name, False)
  base = owner_toggles(**({} if spec.kind == "float" else {spec.attr: stored}))
  b = b_value(name)
  m = ab.TiAb(block_s=10.0)
  cfg = {"setting": name, "b": b, "block_min": 2, "seed": next(x for x in range(50) if ab.arm_for(x, 0) == "A")}
  outs, logs, ws = [], [], []
  m_kw = dict(engaged=True, v_ego=V, curvature=0.0, steer=0.0, saturated=False, steering_pressed=False, driver_torque=0.0,
              lane_change=False, blinker=False, ti_ok=True, fault=False, controller_ok=True, tests_active=False,
              stored={name: stored}, config=json_cfg(cfg))
  m.update(on=False, pin=None, **m_kw)
  for k in range(n):
    if mode in ("ab", "ab_off"):
      out = m.update(on=True, pin="B" if (mode == "ab" and k >= switch_at) else None, **m_kw)
      t = base if not out.active else ab.AbToggles(base, m.overrides(out, base))
      ws.append(out.s)
      if mode == "ab_off":
        assert t is base
    elif mode == "step" and k >= switch_at:
      ws.append(1.0)
      t = owner_toggles(**({"steerKp": [[0], [b]]} if spec.kind == "float" else {spec.attr: b}))
    else:
      t = base
      ws.append(0.0)
    d, mm = scenario(k)
    steer, _, lg = one_step(c, VM, V, d, mm, t)
    outs.append(steer)
    logs.append(lg)
  return outs, ws, logs


def json_cfg(c):
  import json
  return json.dumps(c)


def max_step_counts(diff, lo, hi):
  return max(abs(diff[i] - diff[i - 1]) for i in range(lo, hi)) / COUNT


def diag(lg):
  return lg.mazdaDiagnostics


def runs(name, switch_at=300):
  base, _, lbase = drive(name, "base", switch_at=switch_at)
  blend, w, lab = drive(name, "ab", switch_at=switch_at)
  step, _, lstep = drive(name, "step", switch_at=switch_at)
  allb, _, lallb = drive(name, "step", switch_at=0)
  return dict(base=base, blend=blend, w=w, step=step, allb=allb, lbase=lbase, lab=lab, lstep=lstep, lallb=lallb)


def diffs(r):
  d = lambda x: [a - b for a, b in zip(x, r["base"])]
  return d(r["blend"]), d(r["step"]), d(r["allb"])


# where a plain step of the setting visibly jumps in these scenarios (counts): the bound below is not vacuous there
ALIVE = {"SteerKP": 5.0, "LatDamping": 10.0, "LatFrictionComp": 20.0, "LatNoFrictionRelay": 10.0}


@pytest.mark.parametrize("name", list(ab.SETTINGS))
def test_the_switch_adds_no_step_to_the_steering_request(name):
  """The blend's per-frame change of the request, over the whole wash-in window, is no more than the setting's own natural
  variation (the same window with B in force from the start) plus CONTINUITY_COUNTS; before the switch the request is exactly
  the owner's; where a plain step of the setting does jump, it does so by far more."""
  r = runs(name)
  dab, dsw, dB = diffs(r)
  assert max(abs(x) for x in dab[:300]) == 0.0                           # arm A: bit-identical to the owner's setting
  window = int(round(ab.SETTINGS[name].wash_in_s / DT))
  blend = max_step_counts(dab, 300, 300 + window)
  natural = max_step_counts(dB, 300, 300 + window)
  assert blend <= natural + CONTINUITY_COUNTS, (blend, natural)
  jump = max_step_counts(dsw, 299, 303)
  if name in ALIVE:
    assert jump >= ALIVE[name] and jump > blend + CONTINUITY_COUNTS, (jump, blend)


def test_steerkp_the_proportional_term_follows_the_blended_gain_and_the_request_follows_it_linearly():
  r = runs("SteerKP")
  dab, dsw, dB = diffs(r)
  kps = [diag(lg).kp for lg in r["lab"]]
  assert kps[299] == pytest.approx(0.8) and kps[-1] == pytest.approx(1.0) and all(y >= x - 1e-6 for x, y in zip(kps, kps[1:]))
  assert max(y - x for x, y in zip(kps, kps[1:])) <= 0.2 / (ab.SETTINGS["SteerKP"].ramp_s / DT) + 2e-7   # |B - A| over RAMP frames
  # the output tracks w * (output with B throughout) to a fraction of a count: the term enters linearly through the error
  res = [(dab[k] - dab[k - 1]) - r["w"][k] * (dB[k] - dB[k - 1]) for k in range(1, len(dab))]
  assert max(abs(x) for x in res) / COUNT < 1.0


def _term(lg, term):
  return float(lg.d) if term == "d" else float(getattr(diag(lg), term))


@pytest.mark.parametrize("name,term", [("LatDamping", "d"), ("LatFrictionComp", "frictionCompensation"),
                                       ("LatNoFrictionRelay", "frictionRelay")])
def test_a_weighted_term_is_the_weight_times_the_full_term_so_it_fades_and_cannot_step(name, term):
  """w is the blend weight toward 'on' of the toggle; the term in force is always a convex mix of the term with the setting at
  A and with it at B, with a weight that moves by at most 1 / ramp frames per frame."""
  r = runs(name)
  w = r["w"]
  mix, full, base_term = ([_term(lg, term) for lg in r[k]] for k in ("lab", "lallb", "lbase"))
  alive = 0
  for k in range(300, len(w)):
    expect = w[k] * full[k] + (1.0 - w[k]) * base_term[k]
    assert mix[k] == pytest.approx(expect, abs=1e-6), (k, w[k])
    alive += abs(full[k] - base_term[k]) > 1e-3
  assert alive > 100
  slew = max(abs(y - x) for x, y in zip(w[299:], w[300:]))
  assert slew <= 1.0 / (ab.SETTINGS[name].ramp_s / DT) + 1e-12 and w[-1] == 1.0


@pytest.mark.parametrize("name,blend_key,rate", [("LatOutputFilter", "outputFilterBlend", DT / lct.OUT_FILTER_TAU),
                                                 ("LatCommitSetpoint", "commitBlend", DT / lct.COMMIT_BLEND_RAMP)])
def test_the_controllers_own_blend_takes_a_flip_of_the_toggle_without_a_step(name, blend_key, rate):
  """Output smoothing and the committed reference are blended by the controller's own ramp, over 0.25 s and 0.5 s: the module
  flips the toggle at once for them (ramp 0), and the controller's blend state moves by at most its own rate per frame."""
  assert ab.SETTINGS[name].ramp_s == 0.0 and not ab.SETTINGS[name].weighted
  r = runs(name)
  vals = [getattr(diag(lg), blend_key) for lg in r["lab"]]
  assert max(abs(y - x) for x, y in zip(vals, vals[1:])) <= rate + 1e-9
  assert max(vals[300:]) > 0.5
  dab, dsw, dB = diffs(r)
  window = int(round(ab.SETTINGS[name].wash_in_s / DT))
  assert max_step_counts(dsw, 299, 303) <= max_step_counts(dB, 300, 300 + window) + CONTINUITY_COUNTS


def test_the_block_path_on_a_straight_adds_no_step_for_any_setting_and_arm_a_is_the_owners_request():
  # a straight: small demand, small error, steering-angle noise at 1-3 Hz
  slow = lambda k: 0.004 * (math.sin(k * 0.05) + 0.6 * math.sin(k * 0.13 + 1.0) + 0.4 * math.sin(k * 0.21 + 2.0))
  straight = lambda k: (0.04 * math.sin(k * 0.01), 0.02 * math.sin(k * 0.01 - 0.2) + slow(k))
  for name in ab.SETTINGS:
    spec = ab.SETTINGS[name]
    stored = 0.8 if spec.kind == "float" else OWNER_ON.get(name, False)
    base_t = owner_toggles(**({} if spec.kind == "float" else {spec.attr: stored}))
    m = ab.TiAb(block_s=8.0)
    kw = dict(engaged=True, v_ego=20.0, curvature=0.0002, steer=0.01, saturated=False, steering_pressed=False, driver_torque=0.0,
              lane_change=False, blinker=False, ti_ok=True, fault=False, controller_ok=True, tests_active=False,
              stored={name: stored}, config=json_cfg({"setting": name, "b": b_value(name), "block_min": 2, "seed": 3}))
    m.update(on=False, pin=None, **kw)
    cb, VMb, V = make()
    ca, VMa, _ = make()
    base, ab_out, arms, moving = [], [], [], []
    for k in range(4800):
      out = m.update(on=True, pin=None, **kw)
      t = base_t if not out.active else ab.AbToggles(base_t, m.overrides(out, base_t))
      d, mm = straight(k)
      base.append(one_step(cb, VMb, V, d, mm, base_t)[0])
      ab_out.append(one_step(ca, VMa, V, d, mm, t)[0])
      arms.append(out.arm)
      moving.append(out.active)
    assert {"A", "B"} <= set(arms), name
    assert sum(e["kind"] == "switch" for e in m.events) >= 4
    dd = [x - y for x, y in zip(ab_out, base)]
    assert max(abs(dd[i] - dd[i - 1]) for i in range(1, len(dd))) / COUNT <= CONTINUITY_COUNTS, name
    # before the first switch the request is bit-identical to the owner's
    first = next(i for i, a in enumerate(moving) if a)
    assert first >= 90 and all(x == 0.0 for x in dd[:first]), name


# ---- off means identical to the installed build ----------------------------------------------------------------------------

def installed_module():
  src = subprocess.run(["git", "-C", str(ROOT), "show", f"{INSTALLED}:selfdrive/controls/lib/latcontrol_torque.py"], check=True,
                       capture_output=True, text=True, encoding="utf-8").stdout
  path = Path(__file__).with_name("_installed_latcontrol_torque_tmp.py")
  path.write_text(src, encoding="utf-8")
  try:
    spec = importlib.util.spec_from_file_location("installed_latcontrol_torque", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["installed_latcontrol_torque"] = mod
    spec.loader.exec_module(mod)
  finally:
    path.unlink()
  return mod


TOGGLE_SETS = [dict(), dict(lat_damping=True), dict(lat_friction_comp=True), dict(lat_no_friction_relay=True),
               dict(lat_output_filter=True), dict(lat_commit_setpoint=True),
               dict(lat_damping=True, lat_friction_comp=True, lat_no_friction_relay=True, lat_output_filter=True,
                    lat_commit_setpoint=True)]


def whole(controller_module, toggles_fn, scenario, n=1200):
  c, VM, V = make(module=controller_module)
  res = []
  for k in range(n):
    d, mm = scenario(k)
    steer, ang, lg = one_step(c, VM, V, d, mm, toggles_fn(k))
    res.append((steer, ang, lg.to_dict()))
  return res


def scenario_all(k):
  d = 1.2 + 1.0 * math.sin(k * 0.004) + ripple(k, 0.05)
  return d, d * 0.9 + 0.05 * math.sin(k * 0.02) + ripple(k, 0.02)


class NoMem:
  def get_bool(self, k):
    return False

  def get(self, k):
    return None

  def put(self, k, v):
    pass


@pytest.mark.parametrize("flags", range(len(TOGGLE_SETS)))
def test_with_the_mode_off_the_controller_is_bit_for_bit_the_installed_build(flags):
  inst = installed_module()
  over = TOGGLE_SETS[flags]
  want = whole(inst, lambda k: owner_toggles(**over), scenario_all)
  # 1. the new controller handed the owner's plain toggles
  assert whole(lct, lambda k: owner_toggles(**over), scenario_all) == want
  # 2. the new controller handed a view that overrides nothing
  base = owner_toggles(**over)
  assert whole(lct, lambda k: ab.AbToggles(base, {}), scenario_all) == want
  # 3. the mode switched off, through the runner (it hands back the owner's own object)
  rn = ab.TiAbRunner(NoMem(), NoMem())
  kw = dict(engaged=True, v_ego=20.0, curvature=0.0, steer=0.0, saturated=False, steering_pressed=False, driver_torque=0.0,
            lane_change=False, blinker=False, ti_ok=True, fault=False, controller_ok=True, mono_ns=None)
  assert whole(lct, lambda k: rn.step(base, **kw), scenario_all) == want
  assert rn.ab.events == [] and rn.ab.n == 1200


@pytest.mark.parametrize("name", ["LatDamping", "LatFrictionComp", "LatNoFrictionRelay"])
@pytest.mark.parametrize("weight,on", [(1.0, True), (0.0, False)])
def test_a_blend_weight_at_either_end_is_the_plain_toggle(name, weight, on):
  attr = ab.SETTINGS[name].attr
  plain = lambda k: owner_toggles(**{attr: on})
  base = owner_toggles(**{attr: not on})
  weighted = lambda k: ab.AbToggles(base, {attr: on, "ab_weights": {attr: weight}})
  assert whole(lct, weighted, scenario_all) == whole(lct, plain, scenario_all)


def test_pinning_a_gives_the_owners_value_exactly_for_every_setting():
  for name, spec in ab.SETTINGS.items():
    m = ab.TiAb(block_s=8.0)
    stored = 0.8 if spec.kind == "float" else False
    kw = dict(engaged=True, v_ego=20.0, curvature=0.0, steer=0.0, saturated=False, steering_pressed=False, driver_torque=0.0,
              lane_change=False, blinker=False, ti_ok=True, fault=False, controller_ok=True, tests_active=False,
              stored={name: stored}, config=json_cfg({"setting": name, "b": b_value(name), "seed": 1}))
    m.update(on=False, pin=None, **kw)
    out = m.update(on=True, pin="A", **kw)
    assert not out.active and out.value in (None, stored) and out.s == 0.0
