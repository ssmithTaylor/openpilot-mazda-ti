"""The lookahead feedforward: the plant branch's FF follows the model plan at t+lead instead of
the instantaneous setpoint, so torque arrives when the corner does instead of a plant-lag late.

Measured basis (2026-08-20, routes 270/272/273 re-extracted with modelV2.acceleration.y):
command->achieved lat-accel lag is 0.67-0.93 s (r~0.92) -- the friction-dominated plant lag, ~2.5x
the learned 0.34 s steer delay -- and the plan's horizons track later desired at r=0.97 with real
lead. acceleration.y is same-sign, same-frame, unit-slope against desired_curvature*v^2 (corr
0.98, slope 0.99), verified on this car's own logs, so no runtime sign guard is needed.

Contracts under test: the FF leads, the setpoint/error path is untouched, the plan is bounded by
the same ISO envelope clip_curvature enforces (timing may lead, amplitude may not), everything
falls back to today's behavior without a good plan, and the flip is continuous.
"""
import math
from types import SimpleNamespace

import pytest

from cereal import car, log
from openpilot.selfdrive.car.mazda.lateral_plant import TiLateralPlant
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MAX_LATERAL_ACCEL_NO_ROLL
from openpilot.selfdrive.controls.lib.latcontrol_torque import (
  LatControlTorque, FF_LOOKAHEAD_EXTRA, FF_LOOKAHEAD_MIN, FF_LOOKAHEAD_MAX, FF_LOOKAHEAD_FLAG,
)
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.mock.generators import generate_liveLocationKalman

DT = 0.01
V = 20.3
U_MAX = 600.0
LAT_DELAY = 0.3


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
  def __init__(self):
    self.lateral_plant = TiLateralPlant(U_MAX)

  def torque_from_lateral_accel(self):
    return lambda la, tp: la / tp.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda t, tp: t * tp.latAccelFactor


def toggles(look, cap=False):
  return SimpleNamespace(lat_ff_lookahead=look, lat_demand_cap=cap, lat_output_filter=False,
                         lat_no_friction_relay=False, lat_stall_modulation=False,
                         ti_steer_max=U_MAX)


def fp_state():
  return SimpleNamespace(lkasBlocked=False, lkasEffective=250.0, tiActive=True, columnTorque=0.0)


def plan(fn):
  """model_data whose acceleration.y is fn(t) over the T_IDXS grid."""
  return SimpleNamespace(acceleration=SimpleNamespace(y=[fn(t) for t in ModelConstants.T_IDXS]))


def run(controller, VM, desired_la, model_data, n, look, cap=False, measured_la=None):
  CS = car.CarState.new_message()
  CS.vEgo = V
  CS.steeringPressed = False
  if measured_la is not None:
    curv = measured_la / V ** 2
    CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-curv, V, 0.0))
  params = log.LiveParametersData.new_message()
  llk = generate_liveLocationKalman()
  out = None
  for _ in range(n):
    out = controller.update(True, CS, VM, params, False, desired_la / V ** 2, False, LAT_DELAY,
                            llk, model_data, toggles(look, cap), fp_state())
  return out


def fresh():
  CP = make_cp()
  return LatControlTorque(CP, FakeCI(), DT), VehicleModel(CP)


def expected_lead():
  return min(max(LAT_DELAY + FF_LOOKAHEAD_EXTRA, FF_LOOKAHEAD_MIN), FF_LOOKAHEAD_MAX)


class TestFFLookahead:
  def test_ff_leads_the_plan(self):
    """Desired is zero now; the plan says the corner arrives soon. With lookahead the FF reflects
    the plan at t+lead; without it the FF stays near zero."""
    ramp = plan(lambda t: min(2.0, max(0.0, (t - 0.2) * 4.0)))   # 0 until 0.2 s, 2.0 by 0.7 s
    c_on, VM = fresh()
    _, _, log_on = run(c_on, VM, 0.0, ramp, 800, look=True)
    c_off, VM2 = fresh()
    _, _, log_off = run(c_off, VM2, 0.0, ramp, 800, look=False)
    import numpy as np
    want = float(np.interp(expected_lead(), ModelConstants.T_IDXS[:CONTROL_N],
                           [min(2.0, max(0.0, (t - 0.2) * 4.0)) for t in ModelConstants.T_IDXS[:CONTROL_N]]))
    assert log_on.f == pytest.approx(want, abs=0.15)
    assert abs(log_off.f) < 0.05

  def test_fallback_without_model(self):
    """No plan (or a short one): outputs identical to the toggle being off, frame for frame."""
    for bad in (None, SimpleNamespace(acceleration=SimpleNamespace(y=[0.0] * (CONTROL_N - 1)))):
      c_on, VM = fresh()
      out_on = run(c_on, VM, 1.2, bad, 400, look=True, measured_la=1.1)
      c_off, VM2 = fresh()
      out_off = run(c_off, VM2, 1.2, bad, 400, look=False, measured_la=1.1)
      assert out_on[0] == pytest.approx(out_off[0], abs=1e-9)
      assert not (int(out_on[2].plantState) & FF_LOOKAHEAD_FLAG)

  def test_plan_bounded_by_iso_envelope(self):
    """The plan may lead in time, never in amplitude: a plan beyond the clip_curvature envelope
    feeds the FF only up to +-MAX_LATERAL_ACCEL_NO_ROLL (tire frame, flat road here)."""
    wild = plan(lambda t: 6.0)
    c, VM = fresh()
    _, _, pid_log = run(c, VM, 2.0, wild, 800, look=True, measured_la=2.0)
    assert abs(pid_log.f) <= MAX_LATERAL_ACCEL_NO_ROLL + 1e-6

  def test_setpoint_path_untouched(self):
    """The delayed setpoint (and so ts_des_la and the error clock) must not see the plan."""
    wild = plan(lambda t: 3.0)
    c_on, VM = fresh()
    _, _, log_on = run(c_on, VM, 1.0, wild, 600, look=True, measured_la=0.9)
    c_off, VM2 = fresh()
    _, _, log_off = run(c_off, VM2, 1.0, wild, 600, look=False, measured_la=0.9)
    assert log_on.desiredLateralAccel == pytest.approx(log_off.desiredLateralAccel, abs=1e-9)
    assert log_on.f != pytest.approx(log_off.f, abs=0.2)   # while the FF genuinely differs

  def test_flag_marks_lookahead_frames(self):
    steady = plan(lambda t: 1.0)
    c, VM = fresh()
    _, _, pid_log = run(c, VM, 1.0, steady, 400, look=True, measured_la=1.0)
    assert int(pid_log.plantState) & FF_LOOKAHEAD_FLAG
    c2, VM2 = fresh()
    _, _, pid_log = run(c2, VM2, 1.0, steady, 400, look=False, measured_la=1.0)
    assert not (int(pid_log.plantState) & FF_LOOKAHEAD_FLAG)

  def test_no_step_on_flip(self):
    """Live toggle convention: enabling ramps the plan in; the command never steps."""
    divergent = plan(lambda t: 2.5)
    c, VM = fresh()
    run(c, VM, 0.3, divergent, 400, look=False, measured_la=0.3)
    CS = car.CarState.new_message()
    CS.vEgo = V
    CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-(0.3 / V ** 2), V, 0.0))
    params = log.LiveParametersData.new_message()
    llk = generate_liveLocationKalman()
    prev = None
    max_step = 0.0
    for i in range(300):
      out, _, _ = c.update(True, CS, VM, params, False, 0.3 / V ** 2, False, LAT_DELAY,
                           llk, divergent, toggles(True), fp_state())
      if prev is not None:
        max_step = max(max_step, abs(out - prev))
      prev = out
    assert max_step < 0.02          # normalized torque per frame: ~12 counts, no step

  def test_composes_with_demand_cap(self):
    """Lookahead feeds the FF, the cap still bounds it at the plant's authority."""
    wild = plan(lambda t: 6.0)
    c, VM = fresh()
    _, _, pid_log = run(c, VM, 2.0, wild, 800, look=True, cap=True, measured_la=2.0)
    la_max = c.plant.la_max(V)
    ff_counts = abs(c.plant.inverse(pid_log.f, V))
    cap_counts = abs(c.plant.inverse(la_max - 0.10, V))
    assert ff_counts <= cap_counts + 2.0
