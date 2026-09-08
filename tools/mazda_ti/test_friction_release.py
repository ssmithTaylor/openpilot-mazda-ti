# ruff: noqa: TID251
"""Bounded compensation withdrawal and completed-episode behavior."""
import pytest
from types import SimpleNamespace

from .runtime import bootstrap
bootstrap()
from openpilot.selfdrive.car.mazda.friction_release import FrictionRelease
from .test_measurement_history import car, log, controller


@pytest.mark.parametrize('direction', [-1, 1])
def test_rate_chatter_cannot_restart_until_a_fresh_tightening_request(direction):
  state = FrictionRelease()

  def call(rate=0, latest=1.4):
    return state.update(60*direction, 1.5*direction, latest*direction, 1.8*direction, True, .2, rate*direction, .01)

  for _ in range(10):
    call()
  call(3)
  assert state.completed
  for rate in [0, 3, 0, 0, 5]*50:
    call(rate)
  assert state.removed == 0 and call() == 60*direction
  call(latest=1.75)
  assert state.completed
  call(latest=2.0)
  assert not state.completed
  assert call() == pytest.approx(56.6*direction)


def test_direction_change_and_reset_clear_the_episode():
  state = FrictionRelease()
  state.update(60, 1.5, 1.4, 1.8, True, .2, 0, .01)
  state.update(60, 1.5, 1.4, 1.8, True, .2, 3, .01)
  assert state.completed
  state.update(-60, -1.5, -1.4, -1.8, True, .2, 0, .01)
  assert not state.completed and state.removed == pytest.approx(3.4)
  state.reset()
  assert state.removed == 0 and not state.completed


@pytest.mark.parametrize('permitted,dwell', [(False, .2), (True, .19)])
def test_lane_permission_and_current_request_dwell_are_required(permitted, dwell):
  state = FrictionRelease()
  for _ in range(100):
    assert state.update(60, 1.5, 1.4, 1.8, permitted, dwell, 0, .01) == 60


def test_compensation_bound_tracks_reduced_available_extra():
  state = FrictionRelease()
  for _ in range(30):
    state.update(60, 1.5, 1.4, 1.8, True, .2, 0, .01)
  assert state.removed == 60
  assert state.update(5, 1.5, 1.4, 1.8, True, .2, 0, .01) == 0
  assert state.removed == 5
  assert state.update(-5, 1.5, 1.4, 1.8, True, .2, 0, .01) == -5
  assert state.removed == 0


@pytest.mark.parametrize('active,friction_enabled', [(False, True), (True, False)])
def test_controller_clears_withdrawal_when_inactive_or_compensation_disabled(active, friction_enabled):
  ctrl = controller()
  ctrl.friction_release.removed = 60
  ctrl.friction_release.completed = True
  cs = car.CarState.new_message(vEgo=20.0)
  params = log.LiveParametersData.new_message()
  toggles = SimpleNamespace(lat_commit_setpoint=True, lat_damping=True, lat_friction_comp=friction_enabled,
                            lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=600.0)
  fp = SimpleNamespace(lkasBlocked=False, lkasEffective=0.0, tiActive=True, columnTorque=0.0)
  vm = SimpleNamespace(calc_curvature=lambda *_: 0.0)
  ctrl.update(active, cs, vm, params, False, 0.0, False, .49, None, None, toggles, fp)
  assert ctrl.friction_release.removed == 0 and not ctrl.friction_release.completed
