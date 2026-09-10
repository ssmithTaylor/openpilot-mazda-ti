"""Algebraic reference intervals for a source-verified plant-domain PID log.

This does not apply to the NNFF controller's neural torque-domain error.
"""
import math

from tools.mazda_ti.legacy_input_constraints import rounding_interval


def _product(a, b):
  values = [x*y for x in a for y in b]
  return min(values), max(values)


def _difference(a, b):
  return a[0]-b[1], a[1]-b[0]


def reference_from_observations(*, curvature, measured_accel, desired_curvature,
                                delayed_setpoint, logged_error, torque_params_kp,
                                low_speed_x, low_speed_y, minimum_speed):
  """Conservative inversion of the verified plant equations, without receipt joins.

  actual_accel = curvature * speed**2 in both source locations. Infer a speed
  interval from these two Float32 observations, then enclose the reference and
  current request. This is serialization uncertainty only, not physical truth.
  Zero-crossing curvature and inconsistent product signs cannot resolve speed.
  The low-speed table must be nonnegative and nonincreasing so its ratio to
  positive speed is monotonic between endpoint evaluations.
  """
  from bisect import bisect_right

  if (len(low_speed_x) != len(low_speed_y) or len(low_speed_x) < 2
      or not all(math.isfinite(x) for x in [*low_speed_x, *low_speed_y])
      or any(b <= a for a, b in zip(low_speed_x, low_speed_x[1:]))
      or any(b > a for a, b in zip(low_speed_y, low_speed_y[1:]))
      or min(low_speed_y) < 0):
    raise ValueError('Require increasing x and nonnegative nonincreasing low-speed y')
  k, a = rounding_interval(curvature), rounding_interval(measured_accel)
  if k[0] <= 0 <= k[1]:
    raise ValueError('Curvature interval crosses zero; speed unresolved')
  squared = _product(a, (1/k[1], 1/k[0]))
  if squared[0] <= 0:
    raise ValueError('Acceleration/curvature cannot establish positive speed squared')
  speed = tuple(math.sqrt(x) for x in squared)

  def low(v):
    i = bisect_right(low_speed_x, v)
    if i == 0:
      return low_speed_y[0]
    if i == len(low_speed_x):
      return low_speed_y[-1]
    w = (v-low_speed_x[i-1])/(low_speed_x[i]-low_speed_x[i-1])
    return low_speed_y[i-1]*(1-w)+low_speed_y[i]*w

  candidates = [plant_feedback_reference_interval(measured_accel=measured_accel,
      logged_error=logged_error, speed=v, low_speed_value=low(v),
      torque_params_kp=torque_params_kp, minimum_speed=minimum_speed) for v in speed]
  tracked = (min(x[0] for x in candidates), max(x[1] for x in candidates))
  current = _product(rounding_interval(desired_curvature), squared)
  delayed = rounding_interval(delayed_setpoint)
  return {'speed_mps': speed, 'current_request_mps2': current,
          'delayed_setpoint_mps2': delayed, 'tracked_setpoint_mps2': tracked,
          'history_minus_current_mps2': _difference(delayed, current),
          'commitment_minus_delayed_mps2': _difference(tracked, delayed),
          'tracked_minus_current_mps2': _difference(tracked, current)}


def plant_feedback_reference_interval(*, measured_accel: float, logged_error: float,
                                      speed: float, low_speed_value: float,
                                      torque_params_kp: float, minimum_speed: float) -> tuple[float, float]:
  """Invert error_lsf=(tracked-measured)*(1+(low_speed/max(v,min))**2/kp).

  Use torque_params.kp from the original equation, not the possibly overridden
  PID proportional gain. Inputs measured_accel/error are serialized Float32.
  Bounds account only for their serialization, not sensor or receipt uncertainty.
  """
  values = [speed, low_speed_value, torque_params_kp, minimum_speed]
  if not all(math.isfinite(v) for v in values) or speed <= 0 or low_speed_value < 0 or torque_params_kp <= 0 or minimum_speed <= 0:
    raise ValueError('Invalid source parameters or speed')
  factor = 1 + (low_speed_value/max(speed, minimum_speed))**2/torque_params_kp
  ml, mh = rounding_interval(measured_accel)
  el, eh = rounding_interval(logged_error)
  return ml+el/factor, mh+eh/factor
