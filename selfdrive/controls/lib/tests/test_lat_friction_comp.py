"""The friction compensation: the command carries the rack's measured steady friction in the
direction of demand, so the feedforward stops arriving ~40 counts short everywhere.

Identified 2026-08-20 on stall-excluded quasi-steady frames (1.05M state-3 frames, four plant
routes): implied command-domain friction is route-stable (47/45/45/48 counts median, warm, cold,
wet), direction-symmetric (47 left / 50 right), speed-flat (40-50 across 45-120 km/h), and grows
with load (~36 counts at light command to ~75-80 at heavy) -- Coulomb plus a load term,
u_fric ~= 30 + 0.09*|u|. The same number appears in two other instruments: the breaker study's
"median 30 counts more than it was getting" (p75 63-91), and the integrator's standing +/-0.08
trim (~ comp x local slope). The steady map has no friction term; this supplies it on the
command path, not inside the model (la_max, forward, and the NNFF gain-correction reference stay
pure).

Contracts: magnitude follows the identified curve, direction follows the demand through a smooth
zero gate (straights untouched), the term tapers out approaching the clip (it must never re-pin
the command the demand cap keeps breathing), and off means byte-identical to before.
"""
import math
from types import SimpleNamespace

import pytest

from cereal import car, log
from openpilot.selfdrive.car.mazda.lateral_plant import TiLateralPlant
from openpilot.selfdrive.controls.lib.latcontrol_torque import (
  LatControlTorque, FRIC_COMP_BASE, FRIC_COMP_LOAD, FRIC_COMP_MAX, FRIC_COMP_KEEPOUT,
  FRIC_COMP_LA_SOFT, FRIC_COMP_LA_DEAD, FRIC_COMP_GATE_TAU, FRIC_COMP_FLAG,
)
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.common.mock.generators import generate_liveLocationKalman

DT = 0.01
V = 20.3
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
  def __init__(self):
    self.lateral_plant = TiLateralPlant(U_MAX)

  def torque_from_lateral_accel(self):
    return lambda la, tp: la / tp.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda t, tp: t * tp.latAccelFactor


def toggles(fric, cap=False):
  return SimpleNamespace(lat_friction_comp=fric, lat_demand_cap=cap, lat_ff_lookahead=False,
                         lat_output_filter=False, lat_no_friction_relay=True,
                         lat_stall_modulation=False, ti_steer_max=U_MAX)


def fp_state():
  return SimpleNamespace(lkasBlocked=False, lkasEffective=250.0, tiActive=True, columnTorque=0.0)


def run(controller, VM, desired_la, measured_la, n, fric, cap=False):
  CS = car.CarState.new_message()
  CS.vEgo = V
  curv = measured_la / V ** 2
  CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-curv, V, 0.0))
  params = log.LiveParametersData.new_message()
  llk = generate_liveLocationKalman()
  out = None
  for _ in range(n):
    out = controller.update(True, CS, VM, params, False, desired_la / V ** 2, False, 0.3,
                            llk, None, toggles(fric, cap), fp_state())
  return out


def settled(fric, desired_la, measured_la, cap=False):
  CP = make_cp()
  c = LatControlTorque(CP, FakeCI(), DT)
  VM = VehicleModel(CP)
  out = run(c, VM, desired_la, measured_la, 800, fric, cap)
  return c, out


class TestFrictionComp:
  def test_adds_the_identified_friction_in_the_demand_direction(self):
    """Tracking cleanly at moderate demand, the command with compensation exceeds the command
    without it by the identified curve's value, in the demand's direction."""
    c_on, (out_on, _, log_on) = settled(True, 1.2, 1.2)
    c_off, (out_off, _, log_off) = settled(False, 1.2, 1.2)
    delta = (abs(out_on) - abs(out_off)) * U_MAX
    u0 = abs(c_off.plant.inverse(1.2, V))
    want = min(FRIC_COMP_BASE + FRIC_COMP_LOAD * u0, FRIC_COMP_MAX)
    assert delta == pytest.approx(want, abs=6.0)
    assert int(log_on.plantState) & FRIC_COMP_FLAG
    # same magnitude the other way
    c_l, (out_l, _, _) = settled(True, -1.2, -1.2)
    assert abs(out_l) == pytest.approx(abs(out_on), abs=1e-3)

  def test_zero_gate_keeps_straights_untouched(self):
    c_on, (out_on, _, log_on) = settled(True, 0.0, 0.0)
    c_off, (out_off, _, _) = settled(False, 0.0, 0.0)
    assert abs(out_on - out_off) * U_MAX < 2.0
    assert not (int(log_on.plantState) & FRIC_COMP_FLAG)

  def test_gate_deadzone_covers_straight_dither(self):
    """Demand inside the deadzone -- straight-road dither territory -- gets exactly nothing."""
    _, (out_small_on, _, log_small) = settled(True, 0.05, 0.05)
    _, (out_small_off, _, _) = settled(False, 0.05, 0.05)
    assert abs(out_small_on - out_small_off) * U_MAX < 1.0
    assert not (int(log_small.plantState) & FRIC_COMP_FLAG)

  def test_dithering_straight_stays_calm(self):
    """A straight with +-0.1 m/s^2 of demand dither at ~1 Hz: the low-passed gate keeps the
    compensation's contribution to wheel movement negligible."""
    import math as _m
    CP = make_cp()
    c_on = LatControlTorque(CP, FakeCI(), DT)
    c_off = LatControlTorque(make_cp(), FakeCI(), DT)
    VM = VehicleModel(CP)
    CS = car.CarState.new_message()
    CS.vEgo = V
    params = log.LiveParametersData.new_message()
    llk = generate_liveLocationKalman()
    deltas = []
    for i in range(600):
      des = 0.1 * _m.sin(2 * _m.pi * i * DT * 1.0)
      o_on, _, _ = c_on.update(True, CS, VM, params, False, des / V ** 2, False, 0.3,
                               llk, None, toggles(True), fp_state())
      o_off, _, _ = c_off.update(True, CS, VM, params, False, des / V ** 2, False, 0.3,
                                 llk, None, toggles(False), fp_state())
      deltas.append(abs(o_on - o_off) * U_MAX)
    assert max(deltas[200:]) < 4.0

  def test_keepout_never_repins_the_clip(self):
    """Near the ceiling the compensation tapers out: with the demand cap holding the FF off the
    clip, adding friction compensation must not push it back on."""
    c, (out, _, pid_log) = settled(True, 3.4, 2.25, cap=True)
    assert abs(out) < 1.0 - 1e-6 or abs(out) * U_MAX <= FRIC_COMP_KEEPOUT * U_MAX + 1e-6
    # and the compensation near the clip is (almost) nothing: command matches cap-only run
    c2, (out2, _, _) = settled(False, 3.4, 2.25, cap=True)
    assert (abs(out) - abs(out2)) * U_MAX < 8.0

  def test_off_means_identical(self):
    _, (out_on, _, log_on) = settled(False, 1.2, 1.1)
    _, (out_ref, _, log_ref) = settled(False, 1.2, 1.1)
    assert out_on == pytest.approx(out_ref, abs=1e-9)
    assert not (int(log_on.plantState) & FRIC_COMP_FLAG)

  def test_constants_match_identification(self):
    """Pin the constants to the measured identification so a retune is a conscious act."""
    assert FRIC_COMP_BASE == pytest.approx(30.0)
    assert FRIC_COMP_LOAD == pytest.approx(0.09)
    assert FRIC_COMP_MAX == pytest.approx(85.0)
    assert 0.1 <= FRIC_COMP_LA_SOFT <= 0.25
    assert 0.05 <= FRIC_COMP_LA_DEAD <= 0.15
    assert 0.3 <= FRIC_COMP_GATE_TAU <= 1.0
    assert 0.9 <= FRIC_COMP_KEEPOUT <= 0.99
