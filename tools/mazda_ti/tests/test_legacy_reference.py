import pytest

from tools.mazda_ti.legacy_input_constraints import float32
from tools.mazda_ti.legacy_reference import plant_feedback_reference_interval, reference_from_observations


def test_recovers_hidden_reference_after_two_float32_roundings():
  measured, tracked = 2.36691231, 2.43671317
  factor = 1+(7.65/24.7)**2/1.
  lo, hi = plant_feedback_reference_interval(measured_accel=float32(measured),
      logged_error=float32((tracked-measured)*factor), speed=24.7, low_speed_value=7.65,
      torque_params_kp=1., minimum_speed=.3)
  assert lo <= tracked <= hi
  assert hi-lo < 1e-6


def test_negative_error_and_zero_error_are_preserved():
  args = dict(measured_accel=2., speed=20., low_speed_value=10., torque_params_kp=1., minimum_speed=.3)
  lo, hi = plant_feedback_reference_interval(logged_error=-.5, **args)
  assert lo <= 1.6 <= hi < 2.
  lo, hi = plant_feedback_reference_interval(logged_error=0., **args)
  assert lo <= 2. <= hi


@pytest.mark.parametrize('key,value', [('speed', 0.), ('low_speed_value', -1.),
                                    ('torque_params_kp', 0.), ('minimum_speed', 0.), ('speed', float('nan'))])
def test_invalid_inputs_fail_closed(key, value):
  args = dict(measured_accel=2., logged_error=.5, speed=20., low_speed_value=10., torque_params_kp=1., minimum_speed=.3)
  args[key] = value
  with pytest.raises(ValueError):
    plant_feedback_reference_interval(**args)


@pytest.mark.parametrize('sign', [-1, 1])
def test_joint_observations_enclose_actual_speed_and_references(sign):
  speed, kp = 24.569469451904297, float32(.8)
  measured, tracked, delayed, request = [sign*x for x in (2.36640981, 2.425, 2.3014369, 1.943)]
  low = 10-.5*(speed-20)
  result = reference_from_observations(curvature=float32(measured/speed**2),
      measured_accel=float32(measured), desired_curvature=float32(request/speed**2),
      delayed_setpoint=float32(delayed), logged_error=float32((tracked-measured)*(1+(low/speed)**2/kp)),
      torque_params_kp=kp, low_speed_x=[0,10,20,30], low_speed_y=[15,13,10,5], minimum_speed=1.)
  for key, true in [('speed_mps',speed), ('tracked_setpoint_mps2',tracked),
                   ('current_request_mps2',request), ('commitment_minus_delayed_mps2',tracked-delayed),
                   ('tracked_minus_current_mps2',tracked-request)]:
    lo, hi = result[key]
    assert lo <= true <= hi
    assert hi-lo < 1e-5


@pytest.mark.parametrize('changes', [{'curvature':0.}, {'measured_accel':-2.},
                                   {'low_speed_y':[0,1]}, {'low_speed_x':[1,0]}])
def test_joint_inversion_rejects_unresolved_speed_or_nonmonotonic_table(changes):
  args = dict(curvature=float32(.004), measured_accel=2., desired_curvature=float32(.003),
              delayed_setpoint=2., logged_error=float32(.1), torque_params_kp=.8,
              low_speed_x=[0,30], low_speed_y=[15,5], minimum_speed=1.)
  args.update(changes)
  with pytest.raises(ValueError):
    reference_from_observations(**args)
