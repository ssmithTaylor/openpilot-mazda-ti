"""Breaker direction contracts through the controller, without simulated motion."""
from collections import deque
import math

import numpy as np
import pytest

from cereal import car, log
from openpilot.selfdrive.controls.lib.latcontrol_torque import BREAK_MAX, BREAK_RAMP
from openpilot.selfdrive.controls.lib.tests.test_lat_breaker import DT, V, U_MAX, fp_state, make_controller, toggles


def prepare(desired, measured, commit=False, friction=False):
  controller, vm = make_controller()
  controller.plant.ramp_in = False
  controller.ff_filter.x = desired
  controller.requested_lateral_accel_buffer = deque([desired] * 100, maxlen=100)
  controller.previous_measurement = measured
  controller.commit_ff_filter.x = desired
  controller.commit_sp_filter.x = desired
  controller.commit_gate_filter.x = abs(desired)
  controller.commit_blend = float(commit)
  settings = toggles()
  settings.lat_commit_setpoint = commit
  settings.lat_friction_comp = friction
  return controller, vm, settings


def step(controller, vm, settings, desired, measured):
  state = car.CarState.new_message(vEgo=V)
  state.steeringAngleDeg = math.degrees(vm.get_steer_from_curvature(-measured / V ** 2, V, 0.0))
  params = log.LiveParametersData.new_message()
  steer, _, trace = controller.update(True, state, vm, params, False, desired / V ** 2,
                                      False, 0.48, None, None, settings, fp_state(160.0))
  assert abs(steer) <= 1.0
  return trace


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("commit,friction", [(False, False), (True, True)])
def test_no_new_turn_boost_when_both_references_request_less(sign, commit, friction):
  desired, measured = sign * 1.5, sign * 2.1
  controller, vm, settings = prepare(desired, measured, commit, friction)
  for _ in range(80):
    trace = step(controller, vm, settings, desired, measured)
    assert trace.error * sign < 0
    assert controller.break_boost == 0.0


@pytest.mark.parametrize("sign", [-1, 1])
def test_new_tightening_can_get_help_before_delayed_error_catches_up(sign):
  controller, vm, settings = prepare(sign * 0.9, sign * 1.5)
  helped_with_old_error = False
  for _ in range(35):
    trace = step(controller, vm, settings, sign * 2.0, sign * 1.5)
    if controller.break_boost * sign > 0 and trace.error * sign < 0:
      helped_with_old_error = True
  # A guard using only the delayed error waits about 0.7 s in this fixture.
  # The current reference can justify help while the old target still says less.
  assert helped_with_old_error


@pytest.mark.parametrize("sign", [-1, 1])
def test_tracking_reference_can_permit_boost_when_current_reference_eases(sign):
  measured = sign * 1.8
  controller, vm, settings = prepare(sign * 2.0, measured)
  permitted_with_easing_current_reference = False
  for _ in range(30):
    trace = step(controller, vm, settings, sign * 1.2, measured)
    if (controller.break_boost * sign > 0 and trace.error * sign > 0
        and (controller.ff_filter.x - measured) * sign < 0):
      permitted_with_easing_current_reference = True
  # Both references participate in this policy. Using only the current one
  # would remove the existing delayed-reference permission in this transition.
  assert permitted_with_easing_current_reference


@pytest.mark.parametrize("sign", [-1, 1])
def test_existing_boost_withdraws_at_original_bound_when_both_references_ease(sign):
  desired, measured = sign * 1.5, sign * 2.1
  controller, vm, settings = prepare(desired, measured)
  controller.break_boost = sign * BREAK_MAX * U_MAX
  magnitudes = [abs(controller.break_boost)]
  for _ in range(math.ceil(BREAK_RAMP / DT) + 1):
    step(controller, vm, settings, desired, measured)
    magnitudes.append(abs(controller.break_boost))
  assert np.all(np.diff(magnitudes) <= 0)
  assert np.max(np.abs(np.diff(magnitudes))) <= BREAK_MAX * U_MAX * DT / BREAK_RAMP + 1e-9
  assert magnitudes[-1] == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("extra,allowed", [(-1e-6, False), (0.0, True), (1e-6, True)])
def test_neutral_current_reference_boundary_is_explicit(sign, extra, allowed):
  controller, vm, settings = prepare(sign * 0.5, float(sign))
  # Preserve an older target asking for less while controlling the current
  # reference exactly. Isolate the zero boundary from VehicleModel rounding.
  current = sign * (1.0 + extra)
  controller.ff_filter.x = current
  vm.calc_curvature = lambda *args: -sign / V ** 2
  for _ in range(30):
    step(controller, vm, settings, current, float(sign))
  assert (controller.break_boost * sign > 0) == allowed
