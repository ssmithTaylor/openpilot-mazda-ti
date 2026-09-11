"""Retrospective planar lane-motion consistency, not candidate vehicle simulation."""
import math


def instantaneous(*, speed, offset, heading, road_curvature, yaw_rate):
  """Right-positive Frenet rates under planar/no-sideslip assumptions.

  v*cos(heading)=(1-k*offset)*road_progress_rate; lateral_rate=v*sin(heading).
  Relative heading changes at yaw_rate minus the rotating road tangent rate.
  Model-fitted offset/curvature only approximate these Frenet coordinates.
  """
  if not all(math.isfinite(v) for v in (speed, offset, heading, road_curvature, yaw_rate)) or speed <= 0:
    raise ValueError('Require finite inputs and positive speed')
  denominator = 1-road_curvature*offset
  if denominator <= 0:
    raise ValueError('Invalid lane-coordinate denominator')
  road_rate = road_curvature*speed*math.cos(heading)/denominator
  return {'offset_rate_mps': speed*math.sin(heading),
          'relative_heading_rate_rps': yaw_rate-road_rate,
          'road_tangent_rate_rps': road_rate}


def integrate_observations(rows, *, maximum_gap_s):
  """Trapezoidal balance over supplied observations; never interpolate gaps.

  Compare integrated rates with endpoint observations. Uses recorded states
  throughout, so this cannot predict motion after changing a steering command.
  The yaw sensitivity integrates coherent one-reported-std perturbation. It
  excludes all other uncertainties and has no confidence-interval meaning.
  """
  if len(rows) < 2 or not math.isfinite(maximum_gap_s) or maximum_gap_s <= 0:
    raise ValueError('Require at least two rows and a positive finite gap limit')
  rates = []
  for row in rows:
    if not row['available']:
      raise ValueError('Unavailable observation; do not bridge its gap')
    if not math.isfinite(row['yaw_std_rps']) or row['yaw_std_rps'] < 0:
      raise ValueError('Invalid reported yaw uncertainty')
    rates.append(instantaneous(**{k:row[k] for k in ('speed','offset','heading','road_curvature','yaw_rate')}))
  offset_integral = heading_integral = yaw_sensitivity = 0.
  intervals = []
  for i, (a,b) in enumerate(zip(rows,rows[1:])):
    dt = (b['mono_ns']-a['mono_ns'])/1e9
    if not 0 < dt <= maximum_gap_s:
      raise ValueError('Nonincreasing time or excessive observation gap')
    dy = dt*(rates[i]['offset_rate_mps']+rates[i+1]['offset_rate_mps'])/2
    dh = dt*(rates[i]['relative_heading_rate_rps']+rates[i+1]['relative_heading_rate_rps'])/2
    offset_integral += dy
    heading_integral += dh
    yaw_sensitivity += dt*(a['yaw_std_rps']+b['yaw_std_rps'])/2
    intervals.append({'start_ns':a['mono_ns'],'end_ns':b['mono_ns'],
                      'observed_offset_delta_m':b['offset']-a['offset'],'integrated_offset_delta_m':dy,
                      'observed_heading_delta_rad':b['heading']-a['heading'],'integrated_heading_delta_rad':dh})
  actual_y = rows[-1]['offset']-rows[0]['offset']
  actual_h = rows[-1]['heading']-rows[0]['heading']
  return {'start_ns':rows[0]['mono_ns'],'end_ns':rows[-1]['mono_ns'],'rows':len(rows),
          'observed_offset_delta_m':actual_y,'integrated_offset_delta_m':offset_integral,
          'offset_closure_residual_m':actual_y-offset_integral,
          'observed_heading_delta_rad':actual_h,'integrated_heading_delta_rad':heading_integral,
          'heading_closure_residual_rad':actual_h-heading_integral,
          'coherent_reported_yaw_std_sensitivity_rad':yaw_sensitivity,'intervals':intervals}
