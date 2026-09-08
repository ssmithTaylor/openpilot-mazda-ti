# ruff: noqa: TID251
"""Release-state logging, including absent data and actual controller transitions."""
from types import SimpleNamespace as NS

import pytest

from .runtime import TestInterface as PlantInterface, controller_source, load_controller_source
from .test_diagnostics import synthetic_controller, car, log, LaneContext


FIELDS = {'frictionReleaseVersion', 'frictionWithdrawal', 'frictionReleaseCompleted', 'frictionReleaseDirection'}


@pytest.mark.parametrize('direction', [-1, 1])
def test_release_logging_matches_state_and_preserves_previous_controller(direction):
  current = synthetic_controller()
  previous_module = load_controller_source(controller_source('eca093327d9d60b6c1e2fd13c82494891b58befc'), 'before_release_logging')
  cp = car.CarParams.new_message(steerLimitTimer=1.0)
  tune = cp.lateralTuning.init('torque')
  tune.kp, tune.ki, tune.latAccelFactor = .8, .1, 3.0
  previous = previous_module.LatControlTorque(cp, PlantInterface(), .01)
  cs = car.CarState.new_message(vEgo=25.0)
  params = log.LiveParametersData.new_message()
  vm = NS(calc_curvature=lambda *_: -direction * 3.0 / 625)
  toggles = NS(lat_commit_setpoint=True, lat_damping=True, lat_friction_comp=True,
               lat_output_filter=True, lat_no_friction_relay=False, ti_steer_max=600.0)
  fp = NS(lkasBlocked=False, lkasEffective=-direction * 308.0, tiActive=True, columnTorque=160.0)
  for ctrl in [current, previous]:
    ctrl._release_context = LaneContext(True, direction * .5, 0, 0, .1, 'valid', direction * .0032, direction * .0032)
  withdrew = completed = rearmed = False
  for frame in range(500):
    request = 3.0 if frame < 300 else (2.4 if frame < 410 else 3.4)
    cs.steeringRateDeg = direction * (3 if 360 <= frame < 380 else 0)
    toggles.lat_friction_comp = frame < 460
    active = frame < 480
    if not active:
      current.reset()
      previous.reset()
    args = (active, cs, vm, params, False, direction * request / 625, False, .48, None, None, toggles, fp)
    steer, angle, msg = current.update(*args)
    old_steer, old_angle, old_msg = previous.update(*args)
    assert (steer, angle) == (old_steer, old_angle)
    with log.ControlsState.LateralTorqueState.from_bytes(msg.to_bytes()) as reader:
      d = reader.mazdaDiagnostics
      state = current.friction_release
      assert d.frictionReleaseVersion == 1
      assert d.frictionWithdrawal == state.removed
      assert d.frictionReleaseCompleted == state.completed
      assert d.frictionReleaseDirection == int(state.direction)
      assert vars(state) == vars(previous.friction_release)
      now, old = reader.to_dict(), old_msg.to_dict()
      for values in [now, old]:
        for field in FIELDS:
          values['mazdaDiagnostics'].pop(field, None)
      assert now == old
      withdrew |= d.frictionWithdrawal > 0
      completed |= d.frictionReleaseCompleted
      rearmed |= completed and frame >= 410 and not d.frictionReleaseCompleted
      if not active or not toggles.lat_friction_comp:
        assert d.frictionWithdrawal == 0
        assert not d.frictionReleaseCompleted
        assert d.frictionReleaseDirection == 0
  assert withdrew and completed and rearmed


def test_absent_extension_is_not_observed_reset_state():
  msg = log.ControlsState.LateralTorqueState.new_message(active=True)
  msg.init('mazdaDiagnostics').version = 1
  with log.ControlsState.LateralTorqueState.from_bytes(msg.to_bytes()) as reader:
    assert reader.mazdaDiagnostics.version == 1
    assert reader.mazdaDiagnostics.frictionReleaseVersion == 0


@pytest.mark.parametrize('active,enabled', [(False, True), (True, False)])
def test_logged_reset_replaces_nonzero_retained_state(active, enabled):
  ctrl = synthetic_controller()
  ctrl.friction_release.removed = 34.0
  ctrl.friction_release.direction = -1.0
  ctrl.friction_release.completed = True
  cs = car.CarState.new_message(vEgo=25.0)
  params = log.LiveParametersData.new_message()
  vm = NS(calc_curvature=lambda *_: 0.0)
  toggles = NS(lat_friction_comp=enabled, ti_steer_max=600.0)
  _, _, msg = ctrl.update(active, cs, vm, params, False, 0.0, False, .48, None, None, toggles, None)
  with log.ControlsState.LateralTorqueState.from_bytes(msg.to_bytes()) as reader:
    d = reader.mazdaDiagnostics
    assert d.frictionReleaseVersion == 1
    assert d.frictionWithdrawal == 0
    assert not d.frictionReleaseCompleted
    assert d.frictionReleaseDirection == 0
