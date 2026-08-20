"""How much steering this car can actually hold, and how much slower a corner needs to be taken.

The Mazda + Torque Interceptor drives through two actuators that are both hard-clipped: the
interceptor at 600 counts (a limit inside the unit -- a TiSteerMax of 650 delivered no more) and the
stock LKAS path at the EPS's own +-308. With both saturated the rack winds up until self-aligning
torque plus static friction balances the assist, and then it stops. Measured across the log corpus
that latch sits at roughly 205 counts of column torque, which at 73 km/h is 21-24 degrees of
steering and about 2.6-2.7 m/s^2 of lateral acceleration. Nothing in the command path moves it.

The useful part is what does not change with speed. For a corner of a given radius the steering
angle required is geometry -- slowing down does not reduce it. What slowing reduces is the torque
needed to *hold* that angle, because self-aligning torque falls with speed. So the whole question
becomes: at what speed does the torque this corner demands drop under the ceiling?

SAT per degree, fitted on 87k hands-off quasi-static frames above 10 degrees of steering, using a
strict driver filter (no hand anywhere in the previous 4 s -- the rack holds what a hand put it at,
so an instantaneous filter admits hysteresis and overstates what the car can do alone):

    45-60 km/h    7.07 counts/deg
    60-70         9.39
    70-80        11.83
    80-95        14.08
    95-135       19.15                 r^2 0.995, scaling as v^1.28

The fit is worth trusting because it predicts something it was not fitted to: a 205-count ceiling at
70-80 km/h implies a 22.0 degree limit, and the observed latch in that band is 21-24 degrees.

What this module does NOT know is how much lane there is. A corner that demands more than the car
can hold is not automatically a corner the car leaves -- five recorded passes completed a 3.0 m/s^2
corner while delivering only the 2.65 ceiling, because the lane absorbed the ~0.5 m of path error.
So the advisory here is the conservative one: the speed at which the car can actually track the
plan. Expect it to ask for more slowing than strictly needed until the margin model earns its place;
`TARGET_MARGIN` is the knob, and the logged prediction-versus-outcome is what should move it.
"""
import numpy as np

# Fitted SAT slope and intercept against speed. Torque = slope * |angle_deg| + intercept.
SAT_SPEED_KPH = [52.5, 65.0, 75.0, 87.5, 115.0]
SAT_SLOPE = [7.07, 9.39, 11.83, 14.08, 19.15]
SAT_INTERCEPT = [-37.3, -42.3, -54.7, -54.7, -67.4]

CEILING_COUNTS = 205.0     # where the rack stops answering, both actuators saturated
TARGET_MARGIN = 15.0       # counts of headroom to aim for, so the advice is not knife-edge

MIN_ADVISORY_KPH = 30.0    # below this the plant is a different animal and the fit does not reach
# The search floor is the bottom of the fitted range, not an arbitrary low speed. Below 52.5 km/h
# sat_torque() clips its interpolation and stops measuring anything -- so a search allowed to go
# there will happily converge on a confident-looking number the data does not support. Refusing is
# the honest output: an advisory that says nothing is recoverable, one that says 35 is not.
MIN_MODELLED_KPH = 52.5
MAX_SLOWDOWN_KPH = 40.0    # refuse to display an implausible number; means the model is out of range
MIN_ANGLE_DEG = 5.0        # below this there is no meaningful corner


def sat_torque(angle_deg, v_kph):
  """Column torque needed to hold a steering angle at a speed, in interceptor counts."""
  v = float(np.clip(v_kph, SAT_SPEED_KPH[0], SAT_SPEED_KPH[-1]))
  slope = float(np.interp(v, SAT_SPEED_KPH, SAT_SLOPE))
  intercept = float(np.interp(v, SAT_SPEED_KPH, SAT_INTERCEPT))
  return max(0.0, slope * abs(angle_deg) + intercept)


def required_angle_deg(curvature, wheelbase, steer_ratio):
  """Steering angle a curvature needs. Geometry, so speed does not enter."""
  return abs(float(curvature)) * float(wheelbase) * float(steer_ratio) * 180.0 / np.pi


def advisory_speed_kph(curvature, v_kph, wheelbase, steer_ratio,
                       ceiling=CEILING_COUNTS, margin=TARGET_MARGIN):
  """The speed at which this corner comes inside the car's steering authority.

  Returns 0.0 when the corner is already within it, or when the answer is outside the range the
  fit covers -- better to show nothing than a number the model cannot support."""
  angle = required_angle_deg(curvature, wheelbase, steer_ratio)
  if angle < MIN_ANGLE_DEG or v_kph <= MIN_MODELLED_KPH:
    return 0.0
  target = ceiling - margin
  if sat_torque(angle, v_kph) <= target:
    return 0.0
  # Torque rises monotonically with speed at fixed angle, so bisect. 40 iterations is exact to
  # far below display resolution and costs nothing at 20 Hz.
  lo, hi = MIN_MODELLED_KPH, float(v_kph)
  if sat_torque(angle, lo) > target:
    return 0.0     # even the slowest speed we model will not fit: out of range, say nothing
  for _ in range(40):
    mid = 0.5 * (lo + hi)
    if sat_torque(angle, mid) > target:
      hi = mid
    else:
      lo = mid
  advised = 0.5 * (lo + hi)
  if v_kph - advised > MAX_SLOWDOWN_KPH:
    return 0.0
  return advised


def torque_headroom(angle_deg, v_kph, ceiling=CEILING_COUNTS):
  """Counts of column torque still available at this angle and speed. Negative means past the latch."""
  return ceiling - sat_torque(angle_deg, v_kph)

# Lateral acceleration available at the ceiling angle. Calibrated at one point -- the observed
# 22.0 deg / 2.65 m/s^2 latch at 73 km/h -- and scaled as v^2, since lateral acceleration for a
# given steering angle is kinematic. Note this RISES with speed: slowing reduces what the car can
# hold as well as what the corner demands, which is why the two do not cancel and why a naive
# "demand scales as v^2" estimate asks for far too little slowing.
LA_PER_DEG_AT_73 = 0.1205    # m/s^2 per degree of steering at 73 km/h

# How much lateral room the shortfall is allowed to eat. This is the number that separates a corner
# the car tracks from a corner the car completes: five recorded passes ran 0.4-0.9 m wide of the
# planned line and finished comfortably, because the lane had 0.7-2.0 m of outside margin. Demanding
# zero shortfall would advise slowing on every one of them.
ALLOWED_DRIFT_M = 1.0
PEAK_DURATION_S = 2.0        # nominal time the shortfall acts for; see the caveat in the module doc


def la_at_ceiling(v_kph, ceiling=CEILING_COUNTS):
  """Lateral acceleration the car can hold at a speed, m/s^2."""
  v = float(np.clip(v_kph, SAT_SPEED_KPH[0], SAT_SPEED_KPH[-1]))
  slope = float(np.interp(v, SAT_SPEED_KPH, SAT_SLOPE))
  intercept = float(np.interp(v, SAT_SPEED_KPH, SAT_INTERCEPT))
  angle_max = max(0.0, (ceiling - intercept) / max(slope, 1e-3))
  return angle_max * LA_PER_DEG_AT_73 * (v / 73.0) ** 2


def predicted_drift_m(demand_la, v_kph, duration_s=PEAK_DURATION_S):
  """How far wide of the planned line the car ends up, if it simply runs out of steering."""
  deficit = max(0.0, float(demand_la) - la_at_ceiling(v_kph))
  return 0.5 * deficit * duration_s ** 2


def advisory_speed_margin(demand_la, v_kph, allowed_drift=ALLOWED_DRIFT_M,
                          duration_s=PEAK_DURATION_S):
  """Speed at which the shortfall stops costing more lane than `allowed_drift`.

  This is the criterion that matches what actually happened on the road, rather than the stricter
  "the car tracks the plan exactly". Returns 0.0 when the corner already fits."""
  if v_kph <= MIN_MODELLED_KPH or demand_la <= 0.0:
    return 0.0
  if predicted_drift_m(demand_la, v_kph, duration_s) <= allowed_drift:
    return 0.0
  lo, hi = MIN_MODELLED_KPH, float(v_kph)
  scaled = lambda v: demand_la * (v / float(v_kph)) ** 2      # a corner is a fixed radius
  if predicted_drift_m(scaled(lo), lo, duration_s) > allowed_drift:
    return 0.0     # out of the range the fit covers; say nothing rather than guess
  # Invariant: lo is a speed that fits, hi is one that does not. Drift rises with speed, so an
  # unsafe midpoint means the answer is BELOW it. Getting this backwards collapses the search onto
  # MIN_ADVISORY_KPH, which MAX_SLOWDOWN_KPH then suppresses entirely -- the advisory silently never
  # fires, which is exactly how it failed its first acceptance run.
  for _ in range(40):
    mid = 0.5 * (lo + hi)
    if predicted_drift_m(scaled(mid), mid, duration_s) > allowed_drift:
      hi = mid
    else:
      lo = mid
  advised = 0.5 * (lo + hi)
  if v_kph - advised > MAX_SLOWDOWN_KPH:
    return 0.0
  return advised
