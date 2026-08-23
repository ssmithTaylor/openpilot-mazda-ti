"""Tests for the damping term ("Damp Steering Swings").

The scallop's engine is stick-slip with zero loop damping: breakaway overshoot, command
reversal, post-corner ringing, all carried by the P term (route 0000027e traces). The PID's
error_rate input was plumbed but k_d was always 0; this suite pins the turn-on: off must be
bit-identical, rest must be exactly untouched (the deadzone), and the D contribution must
oppose the measurement's motion with the replay-sized magnitude.
"""
import math

import pytest

from cereal import car, log
from openpilot.common.mock.generators import generate_liveLocationKalman
from openpilot.selfdrive.controls.lib.latcontrol_torque import (
  LatControlTorque, DAMP_KD, DAMP_RATE_DEADZONE, DAMP_FLAG,
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


def toggles(damp):
  return SimpleNamespace(lat_damping=damp, lat_commit_setpoint=False, lat_friction_comp=False,
                         lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=U_MAX)


def fp_state():
  return SimpleNamespace(lkasBlocked=False, lkasEffective=250.0, tiActive=True, columnTorque=0.0)


def make_controller():
  CP = make_cp()
  c = LatControlTorque(CP, FakeCI(), DT)
  VM = VehicleModel(CP)
  return c, VM


def step(c, VM, desired_la, measured_la, damp):
  CS = car.CarState.new_message()
  CS.vEgo = V
  curv = measured_la / V ** 2
  CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-curv, V, 0.0))
  params = log.LiveParametersData.new_message()
  llk = generate_liveLocationKalman()
  return c.update(True, CS, VM, params, False, desired_la / V ** 2, False, 0.3,
                  llk, None, toggles(damp), fp_state())


class TestDamping:
  def test_off_is_identical(self):
    # The toggle off must be bit-identical to the pre-damping controller: k_d is nonzero on the
    # PID, but the error_rate input is zeroed, so the D term contributes exactly nothing.
    c1, VM1 = make_controller()
    c2, VM2 = make_controller()
    for k in range(400):
      la = 1.5 + 0.4 * math.sin(2 * math.pi * k * DT)
      o1, _, l1 = step(c1, VM1, 1.5, la, damp=False)
      o2, _, l2 = step(c2, VM2, 1.5, la, damp=False)
    assert o1 == pytest.approx(o2, abs=1e-12)
    assert l1.d == 0.0
    assert not (int(l1.plantState) & DAMP_FLAG)

  def test_rest_is_exactly_untouched(self):
    # Constant measurement -> rate 0 -> under the deadzone -> D exactly zero, output equals the
    # undamped controller. This is the straight-road promise.
    c1, VM1 = make_controller()
    c2, VM2 = make_controller()
    for _ in range(400):
      on, _, log_on = step(c1, VM1, 0.4, 0.35, damp=True)
      off, _, _ = step(c2, VM2, 0.4, 0.35, damp=False)
    assert log_on.d == 0.0
    assert on == pytest.approx(off, abs=1e-12)
    assert int(log_on.plantState) & DAMP_FLAG

  def test_opposes_the_swing(self):
    # A measurement swinging at scallop frequency: the D term must be nonzero and oppose the
    # measurement's rate (rate positive -> d negative).
    c, VM = make_controller()
    d_when_rising = []
    for k in range(600):
      la = 2.0 + 0.4 * math.sin(2 * math.pi * 1.0 * k * DT)
      # strict mid-rise window: the measurement filter lags ~0.08 s, so the window edges can
      # carry a deadzoned 0.0 -- the claim is about the rise proper, not the turning points
      rising = math.cos(2 * math.pi * 1.0 * k * DT) > 0.8
      _, _, pid_log = step(c, VM, 2.0, la, damp=True)
      if k > 150 and rising:
        d_when_rising.append(pid_log.d)
    assert len(d_when_rising) > 50
    assert max(d_when_rising) <= 0.0, "D must never push WITH a rising measurement"
    assert min(d_when_rising) < -0.05, "D must actively oppose the rise"

  def test_magnitude_matches_the_replay_sizing(self):
    # 0.4 m/s^2 at 1 Hz -> peak rate ~2.5 m/s^3 -> |D| peak ~ KD * (2.5 - deadzone-ish). Pin the
    # order of magnitude so a retune is a conscious act.
    c, VM = make_controller()
    dmax = 0.0
    for k in range(600):
      la = 2.0 + 0.4 * math.sin(2 * math.pi * 1.0 * k * DT)
      _, _, pid_log = step(c, VM, 2.0, la, damp=True)
      if k > 100:
        dmax = max(dmax, abs(pid_log.d))
    assert 0.05 < dmax < 0.25, f"peak |D| {dmax:.3f} outside the replay-sized window"

  def test_stall_contributes_nothing(self):
    # Wheel stopped (measurement frozen mid-corner): rate decays under the deadzone and D goes to
    # zero -- the damping never fights the breaker in a stall.
    c, VM = make_controller()
    for k in range(200):
      la = 2.0 + 0.4 * math.sin(2 * math.pi * 1.0 * k * DT)
      step(c, VM, 2.5, la, damp=True)
    for _ in range(100):
      _, _, pid_log = step(c, VM, 2.5, 2.2, damp=True)
    assert pid_log.d == 0.0

  def test_constants_pinned(self):
    assert DAMP_KD == pytest.approx(0.06)
    assert 0.1 <= DAMP_RATE_DEADZONE <= 0.5


if __name__ == "__main__":
  pytest.main([__file__, "-q"])
