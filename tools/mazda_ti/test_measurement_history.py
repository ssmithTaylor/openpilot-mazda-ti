# ruff: noqa: TID251
"""Plant-controller damping must observe elapsed inactive samples, not skip them."""

from types import SimpleNamespace as NS

import pytest

from .runtime import bootstrap, TestInterface as PlantInterface

controller_module = bootstrap()
from cereal import car, log


def controller():
  cp = car.CarParams.new_message(steerLimitTimer=1.0)
  tune = cp.lateralTuning.init('torque')
  tune.kp, tune.ki, tune.latAccelFactor = 0.8, 0.2, 3.0
  return controller_module.LatControlTorque(cp, PlantInterface(), 0.01)


def step(ctrl, measurement, active):
  cs = car.CarState.new_message(vEgo=20.0, steeringPressed=False)
  params = log.LiveParametersData.new_message()
  vm = NS(calc_curvature=lambda *_: -measurement / 400.0)
  toggles = NS(lat_commit_setpoint=True, lat_damping=True, lat_friction_comp=True,
               lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=600.0)
  fp = NS(lkasBlocked=False, lkasEffective=0.0, tiActive=True, columnTorque=0.0)
  # controlsd calls reset on each inactive iteration, then still calls update.
  if not active:
    ctrl.reset()
  return ctrl.update(active, cs, vm, params, False, measurement / 400.0, False, 0.49,
                     None, None, toggles, fp)[2]


@pytest.mark.parametrize('direction', [-1, 1])
def test_reengagement_does_not_treat_inactive_motion_as_one_control_tick(direction):
  ctrl = controller()
  for _ in range(100):
    step(ctrl, direction * 1.8, True)
  for _ in range(1800):
    inactive = step(ctrl, 0.0, False)
    assert not inactive.active and inactive.mazdaDiagnostics.command == 0
  resumed = step(ctrl, 0.0, True)
  assert abs(resumed.mazdaDiagnostics.measurementRate) < 1e-6
  assert abs(resumed.d) < 1e-6


@pytest.mark.parametrize('active', [False, True])
def test_first_measurement_has_no_invented_rate_from_constructor_zero(active):
  ctrl = controller()
  step(ctrl, 1.8, active)
  next_sample = step(ctrl, 1.8, True)
  assert next_sample.mazdaDiagnostics.measurementRate == 0.0


def test_real_motion_during_disengagement_is_available_to_damping_on_engage():
  ctrl = controller()
  step(ctrl, 0.0, False)
  for i in range(1, 201):
    step(ctrl, i * 0.002, False)
  resumed = step(ctrl, 201 * 0.002, True)
  assert resumed.mazdaDiagnostics.measurementRate == pytest.approx(0.2, abs=1e-6)
