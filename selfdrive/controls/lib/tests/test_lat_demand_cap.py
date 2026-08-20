"""The demand cap: the plant branch never asks its PID to track lateral accel the plant cannot
deliver, and never lets the open-loop feedforward sit exactly on the absolute clip.

Why it exists, measured on the 00000273 curve pass (2026-08-20): the rack statically stalls almost
exclusively while the command sits flat at the ceiling -- 10.1 % of pinned frames against 0.03 %
of breathing frames over the whole corpus -- and the command only sits flat because the lat-accel
clip erases the loop's texture whenever the ask exceeds what the actuators deliver. Capping the ask
at authority keeps the loop linear; leaving feedback headroom above the feedforward keeps du/dt
alive at the top, which is what actually prevents the stall.
"""
import math
from types import SimpleNamespace

import pytest

from cereal import car, log
from openpilot.selfdrive.car.mazda.lateral_plant import TiLateralPlant
from openpilot.selfdrive.controls.lib.latcontrol_torque import (
  LatControlTorque, CAP_HEADROOM, DEMAND_CAP_FLAG,
)
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.common.mock.generators import generate_liveLocationKalman

DT = 0.01
V = 20.3                    # m/s, the 73 km/h the measured corner ran at
U_MAX = 600.0


def make_cp():
  CP = car.CarParams.new_message()
  CP.steerLimitTimer = 0.4
  CP.mass = 1909.0
  CP.rotationalInertia = 2813.0
  CP.wheelbase = 2.7
  CP.centerToFront = CP.wheelbase * 0.41
  CP.steerRatio = 15.5
  CP.tireStiffnessFront = 80000.0
  CP.tireStiffnessRear = 90000.0
  lt = CP.lateralTuning
  lt.init("torque")
  lt.torque.kp = 0.8
  lt.torque.ki = 0.2
  lt.torque.friction = 0.1
  lt.torque.latAccelFactor = 2.0
  lt.torque.latAccelOffset = 0.0
  lt.torque.useSteeringAngle = True
  return CP


class FakeCI:
  """Only what LatControlTorque touches. The plant is the real one."""
  def __init__(self):
    self.lateral_plant = TiLateralPlant(U_MAX)

  def torque_from_lateral_accel(self):
    return lambda la, tp: la / tp.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda t, tp: t * tp.latAccelFactor


def toggles(cap):
  return SimpleNamespace(lat_demand_cap=cap, lat_output_filter=False,
                         lat_no_friction_relay=False, lat_stall_modulation=False,
                         ti_steer_max=U_MAX)


def fp_state():
  # Stock LKAS alive and pushing, TI active: the settled two-actuator state the corner ran in.
  return SimpleNamespace(lkasBlocked=False, lkasEffective=250.0, tiActive=True, columnTorque=0.0)


def run(controller, VM, desired_la, measured_la, n, cap, roll=0.0):
  """Run n frames at constant demand and constant measured lateral accel; return the last frame."""
  CS = car.CarState.new_message()
  CS.vEgo = V
  CS.steeringPressed = False
  # angle that makes the controller's own measurement come out at measured_la
  curv = measured_la / V ** 2
  sa = VM.get_steer_from_curvature(-curv, V, roll)
  CS.steeringAngleDeg = math.degrees(sa)
  params = log.LiveParametersData.new_message()
  params.roll = roll
  llk = generate_liveLocationKalman()
  out = None
  for _ in range(n):
    out = controller.update(True, CS, VM, params, False, desired_la / V ** 2, False, 0.3,
                            llk, None, toggles(cap), fp_state())
  return out


def settled(cap, desired_la, measured_la, roll=0.0):
  CP = make_cp()
  controller = LatControlTorque(CP, FakeCI(), DT)
  VM = VehicleModel(CP)
  out = run(controller, VM, desired_la, measured_la, 800, cap, roll)
  la_max = controller.plant.la_max(V)
  return controller, out, la_max


class TestDemandCap:
  def test_harness_measures_what_it_says(self):
    controller, (_, _, pid_log), la_max = settled(False, 1.0, 1.0)
    assert la_max == pytest.approx(2.3, abs=0.2)
    assert pid_log.actualLateralAccel == pytest.approx(1.0, abs=0.1)
    assert pid_log.desiredLateralAccel == pytest.approx(1.0, abs=0.05)

  def test_pins_flat_without_cap_when_delivering_the_ceiling(self):
    """Today's pathology, preserved when the toggle is off: ask beyond authority with the car
    delivering all it has, and the command sits exactly on the clip with no texture."""
    controller, (output, _, pid_log), la_max = settled(False, 3.4, 2.25)
    assert abs(output) == pytest.approx(1.0, abs=1e-6)

  def test_cap_restores_headroom_when_delivering(self):
    """With the cap on, the same state leaves the feedforward short of the clip, so the loop
    still owns du/dt at the top. Feedback may use the rest, so assert on the ff, not the sum."""
    controller, (output, _, pid_log), la_max = settled(True, 3.4, 2.25)
    ff_counts = abs(controller.plant.inverse(pid_log.f, V))
    cap_counts = abs(controller.plant.inverse(la_max - CAP_HEADROOM, V))
    assert ff_counts == pytest.approx(cap_counts, abs=2.0)
    assert cap_counts < U_MAX - 8.0
    # error is measured against what the car can do, not against the sky: the standing error is
    # the shortfall to authority (~0.05), not to the 3.4 ask (~1.15)
    assert abs(pid_log.error) < 0.35

  def test_cap_keeps_full_authority_under_real_error(self):
    """A car far from its target must still get everything: capping demand is not capping torque."""
    controller, (output, _, pid_log), la_max = settled(True, 3.4, 0.0)
    assert abs(output) == pytest.approx(1.0, abs=1e-6)

  def test_cap_inert_below_authority(self):
    """Asking within authority, the cap changes nothing at all."""
    _, (out_off, _, log_off), _ = settled(False, 1.2, 1.1)
    _, (out_on, _, log_on), _ = settled(True, 1.2, 1.1)
    assert out_on == pytest.approx(out_off, abs=1e-9)
    assert log_on.error == pytest.approx(log_off.error, abs=1e-9)
    assert not (int(log_on.plantState) & DEMAND_CAP_FLAG)

  def test_flag_set_only_while_binding(self):
    _, (_, _, pid_log), _ = settled(True, 3.4, 2.25)
    assert int(pid_log.plantState) & DEMAND_CAP_FLAG
    _, (_, _, pid_log), _ = settled(True, 1.2, 1.1)
    assert not (int(pid_log.plantState) & DEMAND_CAP_FLAG)

  def test_logged_desired_stays_uncapped(self):
    """ts_des_la keeps meaning 'what was asked' -- every analysis of demand depends on it."""
    _, (_, _, pid_log), _ = settled(True, 3.4, 2.25)
    assert pid_log.desiredLateralAccel == pytest.approx(3.4, abs=0.05)

  def test_camber_shifts_the_cap_with_the_road(self):
    """A favorable camber raises what the road-frame ask may be before it is capped, by exactly
    the gravity component -- the plant's authority is about the tire, not the road frame."""
    roll = 0.06                    # rad, ~0.59 m/s^2 of help
    _, (_, _, log_flat), la_max = settled(True, 3.4, 2.25)
    _, (_, _, log_bank), _ = settled(True, 3.4, 2.25, roll=roll)
    # same tire-frame ff cap either way
    assert log_bank.f == pytest.approx(log_flat.f, abs=0.05)
    # but the road-frame error target moved by ~g*sin(roll): the banked road absorbs more ask
    # before the cap binds, so the standing (capped) error grows by that amount here, where the
    # measurement was held fixed
    g_component = 9.81 * math.sin(roll)
    assert abs(log_bank.error) - abs(log_flat.error) == pytest.approx(g_component, abs=0.25)

  def test_cap_never_opposes_demonstrated_capability(self):
    """The plant's top end is conservative (600 counts models ~2.30 where the car measures ~2.5
    delivered). When the car is measurably doing MORE than the plant's la_max, the bound follows
    the measurement: the capped error must never go negative and drag the car back down."""
    la_meas = 2.55
    controller, (_, _, pid_log), la_max = settled(True, 3.4, la_meas)
    assert la_meas > la_max          # the premise: measured beyond the plant's estimate
    # error is (tracked setpoint - measurement); with the bound widened to the measurement it is
    # ~zero, never negative-signed against the turn
    assert abs(pid_log.error) < 0.15

  def test_no_cap_without_authority(self):
    """With no actuator in play there is nothing to cap against: identical to the toggle being
    off (including the stiction breaker's standing push, which predates the cap)."""
    outs = {}
    for cap in (False, True):
      CP = make_cp()
      controller = LatControlTorque(CP, FakeCI(), DT)
      VM = VehicleModel(CP)
      CS = car.CarState.new_message()
      CS.vEgo = V
      params = log.LiveParametersData.new_message()
      llk = generate_liveLocationKalman()
      dead = SimpleNamespace(lkasBlocked=True, lkasEffective=0.0, tiActive=False, columnTorque=0.0)
      out = None
      for _ in range(400):
        out = controller.update(True, CS, VM, params, False, 3.4 / V ** 2, False, 0.3,
                                llk, None, toggles(cap), dead)
      outs[cap] = out
    assert outs[True][0] == pytest.approx(outs[False][0], abs=1e-9)
    assert not (int(outs[True][2].plantState) & DEMAND_CAP_FLAG)
