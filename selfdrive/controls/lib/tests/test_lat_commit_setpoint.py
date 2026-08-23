"""Tests for the committed setpoint ("Hold the Corner Line").

The mechanism being pinned: the reference the controller tracks (and the feedforward it holds)
deepens at the plan's pace but releases slowly, so one breakaway overshoot cannot talk the
controller out of a corner via the plan's sympathetic ease-off. The properties that matter are
the guards as much as the ratchet: it must not act on straights, must not delay entries, must
never hold an old direction through an S-transition, and must never exceed the live ask by more
than its fixed margin -- the replay's unguarded version held 4+ s of stale ask through exactly
one of those.
"""
import math

import numpy as np
import pytest

from cereal import car, log
from openpilot.common.mock.generators import generate_liveLocationKalman
from openpilot.selfdrive.controls.lib.latcontrol_torque import (
  LatControlTorque, CommitFilter, COMMIT_FLAG, COMMIT_MAX_EXTRA, COMMIT_TAU_DEEPEN,
  COMMIT_TAU_RELEASE, COMMIT_GATE_ON,
)
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.selfdrive.car.mazda.lateral_plant import TiLateralPlant
from types import SimpleNamespace

DT = 0.01
V = 20.0
U_MAX = 600.0


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


def toggles(commit):
  return SimpleNamespace(lat_commit_setpoint=commit, lat_friction_comp=False,
                         lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=U_MAX)


def fp_state():
  return SimpleNamespace(lkasBlocked=False, lkasEffective=250.0, tiActive=True, columnTorque=0.0)


def make_controller():
  CP = make_cp()
  c = LatControlTorque(CP, FakeCI(), DT)
  VM = VehicleModel(CP)
  return c, VM


def step(c, VM, desired_la, measured_la, commit):
  CS = car.CarState.new_message()
  CS.vEgo = V
  curv = measured_la / V ** 2
  CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-curv, V, 0.0))
  params = log.LiveParametersData.new_message()
  llk = generate_liveLocationKalman()
  return c.update(True, CS, VM, params, False, desired_la / V ** 2, False, 0.3,
                  llk, None, toggles(commit), fp_state())


class TestCommitFilter:
  """The pure ratchet, directly."""

  def test_deepens_fast_releases_slowly(self):
    f = CommitFilter(DT)
    for _ in range(100):
      f.update(2.5)
    assert f.x == pytest.approx(2.5, abs=0.05), "one deepen time-constant should be nearly there"
    # release toward a smaller ask, but bounded by the extra-ask clamp: the slow path only shows
    # inside the clamp window
    for _ in range(20):
      f.update(2.2)
    assert f.x > 2.4, "0.2 s into a release the committed ask should have barely moved"
    for _ in range(200):
      f.update(2.2)
    assert f.x == pytest.approx(2.2, abs=0.15)

  def test_never_exceeds_live_ask_plus_margin(self):
    f = CommitFilter(DT)
    for _ in range(200):
      f.update(3.0)
    f.update(1.0)
    assert abs(f.x) <= 1.0 + COMMIT_MAX_EXTRA + 1e-9, \
      "a released ask must clamp the committed one immediately"

  def test_sign_flip_is_fast(self):
    # The S-transition guard: committing against the current ask's direction held 4+ s of stale
    # ask in the unguarded replay. With the guard the crossing must be on the DEEPEN timescale.
    f = CommitFilter(DT)
    for _ in range(300):
      f.update(2.5)
    n = 0
    while f.x > 0 and n < 200:
      f.update(-2.5)
      n += 1
    assert n * DT < 3 * COMMIT_TAU_DEEPEN + COMMIT_TAU_DEEPEN, \
      f"took {n * DT:.2f}s to cross zero against an opposite ask"

  def test_straight_road_is_identity_within_noise(self):
    f = CommitFilter(DT)
    rng = np.random.default_rng(0)
    xs = 0.1 * rng.standard_normal(500)
    outs = np.array([f.update(v) for v in xs])
    assert np.max(np.abs(outs)) < 0.1 + COMMIT_MAX_EXTRA


class TestController:
  def test_off_is_identical(self):
    c1, VM1 = make_controller()
    c2, VM2 = make_controller()
    for _ in range(400):
      o1, _, l1 = step(c1, VM1, 2.2, 2.0, commit=False)
      o2, _, l2 = step(c2, VM2, 2.2, 2.0, commit=False)
    assert o1 == pytest.approx(o2, abs=1e-9)
    assert not (int(l1.plantState) & COMMIT_FLAG)

  def test_straights_untouched_and_unflagged(self):
    c1, VM1 = make_controller()
    c2, VM2 = make_controller()
    for _ in range(400):
      on, _, log_on = step(c1, VM1, 0.5, 0.45, commit=True)
      off, _, _ = step(c2, VM2, 0.5, 0.45, commit=False)
    assert on == pytest.approx(off, abs=1e-9), "below the gate the commitment must do nothing"
    assert not (int(log_on.plantState) & COMMIT_FLAG)

  def test_corner_plateau_flagged_but_converged(self):
    # Steady cornering: the ratchet converges onto the raw reference, so the flag is set while
    # the OUTPUT stays essentially the toggle-off output -- commitment is not a standing bias.
    c1, VM1 = make_controller()
    c2, VM2 = make_controller()
    for _ in range(800):
      on, _, log_on = step(c1, VM1, 2.4, 2.2, commit=True)
      off, _, _ = step(c2, VM2, 2.4, 2.2, commit=False)
    assert int(log_on.plantState) & COMMIT_FLAG
    assert on == pytest.approx(off, abs=0.02)

  def test_holds_through_a_sympathetic_release(self):
    # The scallop step itself: mid-corner the plan eases 2.6 -> 2.0 for half a second (the mirror
    # following the car's overshoot). Committed: the controller's ask must NOT follow it down by
    # more than the release physics allows; uncommitted it does.
    outs = {}
    for commit in (False, True):
      c, VM = make_controller()
      for _ in range(800):
        step(c, VM, 2.6, 2.4, commit)
      dips = []
      for _ in range(50):
        o, _, _ = step(c, VM, 2.0, 2.4, commit)
        dips.append(abs(o))
      outs[commit] = min(dips)
    assert outs[True] > outs[False] + 0.02, \
      f"committed min ask {outs[True]:.3f} should hold above uncommitted {outs[False]:.3f}"

  def test_logged_desired_is_the_raw_ask(self):
    # ts_des_la must stay the truth for analysis; the committed reference is reconstructable.
    c, VM = make_controller()
    for _ in range(800):
      _, _, pid_log = step(c, VM, 2.4, 2.2, commit=True)
    assert pid_log.desiredLateralAccel == pytest.approx(2.4, abs=0.05)

  def test_toggle_flip_mid_corner_never_steps(self):
    c, VM = make_controller()
    for _ in range(800):
      step(c, VM, 2.6, 2.1, commit=True)
    prev = None
    max_jump = 0.0
    for i in range(120):
      o, _, _ = step(c, VM, 2.6, 2.1, commit=False)
      if prev is not None:
        max_jump = max(max_jump, abs(o - prev))
      prev = o
    assert max_jump < 0.02, f"disabling mid-corner stepped the output by {max_jump:.3f}"


if __name__ == "__main__":
  pytest.main([__file__, "-q"])
