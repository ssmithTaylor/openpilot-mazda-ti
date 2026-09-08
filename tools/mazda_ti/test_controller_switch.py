# ruff: noqa: TID251
"""Candidate activation must not discard a valid measurement derivative."""
from .controller_replay import switch_controller
from .runtime import controller_source, load_controller_source, TestInterface as PlantInterface
from .test_measurement_history import controller, controller_module, step
from cereal import car


def car_params():
  cp = car.CarParams.new_message(steerLimitTimer=1.0)
  tune = cp.lateralTuning.init('torque')
  tune.kp, tune.ki, tune.latAccelFactor = 0.8, 0.2, 3.0
  return cp


def test_switch_from_active_legacy_observer_preserves_measurement_rate():
  legacy = load_controller_source(controller_source('9c6c8abf2d85330c701284c7d6122bda3c0edf5f'), 'legacy_switch_test')
  reference = controller()
  old = legacy.LatControlTorque(car_params(), PlantInterface(), .01)
  for i in range(300):
    step(old, i*.002, True)
    step(reference, i*.002, True)
  before = old.measurement_rate_filter
  switch_controller(old, controller_module.LatControlTorque, car_params(), True)
  assert old.measurement_rate_filter is before
  actual = step(old, .6, True).mazdaDiagnostics.measurementRate
  expected = step(reference, .6, True).mazdaDiagnostics.measurementRate
  assert abs(actual - expected) < 1e-10


def test_switch_keeps_existing_observer_initialized_during_inactivity():
  ctrl = controller()
  for i in range(300):
    step(ctrl, i*.002, False)
  switch_controller(ctrl, controller_module.LatControlTorque, car_params(), False)
  assert ctrl._plant_measurement_initialized
  assert abs(step(ctrl,.6,True).mazdaDiagnostics.measurementRate - .2) < 1e-10


def test_legacy_inactive_observer_has_no_valid_rate_history_to_inherit():
  legacy = load_controller_source(controller_source('9c6c8abf2d85330c701284c7d6122bda3c0edf5f'), 'legacy_inactive_switch_test')
  old = legacy.LatControlTorque(car_params(), PlantInterface(), .01)
  for i in range(100):
    step(old, i*.02, True)
  for _ in range(100):
    step(old, 0, False)
  assert old.measurement_rate_filter.x > 1
  switch_controller(old, controller_module.LatControlTorque, car_params(), False)
  assert step(old,0,True).mazdaDiagnostics.measurementRate == 0
