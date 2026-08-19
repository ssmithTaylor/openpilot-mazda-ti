import math
import numpy as np
from collections import deque

from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.car.interfaces import FRICTION_THRESHOLD
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED, apply_center_deadzone, get_friction
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]

MAX_LAT_JERK_UP = 2.5            # m/s^3

# Cars that ship a lateral plant model (CI.lateral_plant) use it instead of a single
# torque-to-lateral-accel gain. Tunables for that path:
FF_SMOOTH_SECONDS = 0.06   # low-pass on the request feeding the feedforward; the model's desired
                           # lateral accel steps at 20 Hz and the feedforward put every step on the wire
FRICTION_DEADZONE = 0.04   # m/s^2 of error below which the friction term is off, so it stops
                           # chattering about noise (measured 250-330 sign flips/min without it)
FRICTION_TORQUE = 0.10     # normalized torque of breakaway help at full error

# Stiction breaker. The rack does not answer a torque command until static friction lets go, and
# measured over 482 stick-then-move events it wanted a median 30 counts more than it was getting
# (p90 91) after sitting still for a median 0.8 s. These fire it only while that is happening.
BREAK_ERR = 0.10           # m/s^2 of error worth unsticking the wheel for
BREAK_STICK_RATE = 1.0     # deg/s: below this the wheel is not moving
BREAK_FREE_RATE = 2.0      # deg/s: above this it is moving and the boost lets go
BREAK_DEBOUNCE = 0.20      # s the condition must hold -- the wheel passes through zero rate on every
                           # direction reversal, and that is not the same thing as being stuck
BREAK_MAX = 0.15           # normalized torque, ~90 counts at a 600 ceiling: the p90 of what was needed
BREAK_RAMP = 0.25          # s to reach full boost, so it is never a step onto the wheel

class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES = int(1 / self.dt)
    self.requested_lateral_accel_buffer = deque([0.] * self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES , maxlen=self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * (MAX_LAT_JERK_UP - 0.5)), self.dt)

    # A car whose torque -> lateral acceleration relationship is worth inverting properly (more
    # than one actuator, or non-linear) hands us a model of it; see CarInterfaceBase.lateral_plant.
    self.plant = getattr(CI, "lateral_plant", None)
    self.ff_filter = FirstOrderFilter(0.0, FF_SMOOTH_SECONDS, self.dt)
    self.friction_torque = FRICTION_TORQUE
    self.break_frames = 0        # how long the wheel has been stuck with an error worth acting on
    self.break_boost = 0.0       # counts, ramped, signed in the command's frame

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    if self.plant is not None:
      # The plant model carries the shape, so the learned single gain and friction do not apply to
      # it -- a straight line fitted through a curved, clipped, state-dependent plant drifts with
      # whatever roads were driven last. The offset is a different quantity (the lateral accel the
      # car has at zero torque, from device roll misalignment) and is well identified, so keep it.
      self.torque_params.latAccelOffset = float(np.clip(latAccelOffset, -0.6, 0.6))
      return
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def reset(self):
    super().reset()
    self.break_frames = 0
    self.break_boost = 0.0
    if self.plant is not None:
      self.plant.reset()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def _plant_state(self, active, CS, fp_car_state, frogpilot_toggles):
    """Tell the plant model which actuators are live. Unknown state (the car state message is
    stale) is treated as 'everything is working', which under-commands rather than over-steers."""
    ti_max = getattr(frogpilot_toggles, "ti_steer_max", None)
    if ti_max is not None and abs(ti_max - self.plant.ti_steer_max) > 0.5:
      self.plant.set_ti_steer_max(ti_max)
    if fp_car_state is None:
      return self.plant.update_state(CS.vEgo, False, 0.0, True, active=active)
    return self.plant.update_state(CS.vEgo, fp_car_state.lkasBlocked, fp_car_state.lkasEffective,
                                   fp_car_state.tiActive, active=active)

  def _update_with_plant(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature,
                         curvature_limited, lat_delay, frogpilot_toggles, fp_car_state):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 2

    # The request is filtered and buffered even while we are not steering, so that engaging mid-corner
    # starts from the truth rather than from a second-old buffer (controlsd feeds the current
    # curvature while latActive is false).
    future_lateral_accel = self.ff_filter.update(desired_curvature * CS.vEgo ** 2)
    self.requested_lateral_accel_buffer.append(future_lateral_accel)
    plant_state = self._plant_state(active, CS, fp_car_state, frogpilot_toggles)
    pid_log.plantState = int(plant_state)

    if not active:
      self.plant.u_prev = 0.0
      self.break_frames = 0
      self.break_boost = 0.0
      pid_log.active = False
      return 0.0, 0.0, pid_log

    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
    self.previous_measurement = measurement
    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY

    # Compare like with like: the request from lat_delay ago is the one the car has had time to
    # answer, so that is what the error is against (upstream master does the same).
    delay_frames = int(np.clip(lat_delay / self.dt, 1, self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES))
    setpoint = self.requested_lateral_accel_buffer[-delay_frames]
    error = setpoint - measurement
    low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, MIN_SPEED)) ** 2
    error_lsf = error + low_speed_factor / self.torque_params.kp * error

    # Feedforward is the demand at the tire, in lateral accel; the model turns it into counts below.
    # latAccelOffset is what the car does at zero torque (device roll misalignment).
    ff = future_lateral_accel - roll_compensation - self.torque_params.latAccelOffset

    # Wind the integrator against the authority this car actually has in its current state -- at
    # low speed, or with the stock LKAS path blocked, that is a fraction of what it has on the
    # highway, and winding past it only delays the recovery.
    la_max = self.plant.la_max(CS.vEgo)
    self.pid.set_limits(la_max, -la_max)

    freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
    output_lataccel = self.pid.update(error_lsf,
                                      -measurement_rate,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

    u_max = self.plant.ti_steer_max
    command = self.plant.inverse(output_lataccel, CS.vEgo)

    # Breakaway friction, in torque space: what it compensates is stiction in the rack, which is a
    # torque, and in lateral-accel space the same relay becomes hundreds of counts wherever the
    # plant is soft. The dead-zone keeps it from chattering on measurement noise. With no actuator
    # in play there is nothing to break away, so send nothing rather than a standing offset that
    # would step onto the wheel the moment an actuator comes back.
    friction_torque = 0.0
    if la_max > 0.0:
      friction_torque = self.friction_torque * u_max * float(np.clip(
        apply_center_deadzone(error, FRICTION_DEADZONE) / FRICTION_THRESHOLD, -1.0, 1.0))
    command = float(np.clip(command + friction_torque, -u_max, u_max))

    # Stiction breaker. Only while the wheel is genuinely stuck: the driver is not steering, the
    # error is worth acting on, the wheel is not moving, and it has been that way long enough that a
    # zero-crossing mid-reversal cannot trigger it. Ramped in and out so it never steps, and it lets
    # go the moment the wheel moves -- the plant is a full 2x quicker once it is already sliding.
    wheel_rate = abs(float(CS.steeringRateDeg))
    stuck = (abs(error) > BREAK_ERR and wheel_rate < BREAK_STICK_RATE
             and not CS.steeringPressed and not steer_limited_by_safety)
    self.break_frames = self.break_frames + 1 if stuck else 0
    engaged_long_enough = self.break_frames * self.dt >= BREAK_DEBOUNCE
    if engaged_long_enough and wheel_rate < BREAK_FREE_RATE:
      direction = np.sign(command) if command != 0.0 else np.sign(error)
      target = BREAK_MAX * u_max * float(direction)
    else:
      target = 0.0
    step = BREAK_MAX * u_max * self.dt / BREAK_RAMP
    self.break_boost += float(np.clip(target - self.break_boost, -step, step))
    command = float(np.clip(command + self.break_boost, -u_max, u_max))

    output_torque = command / u_max
    self.plant.u_prev = -command   # wire convention: actuators.steer is positive left

    pid_log.active = True
    pid_log.error = float(error_lsf)
    pid_log.p = float(self.pid.p)
    pid_log.i = float(self.pid.i)
    pid_log.d = float(self.pid.d)
    pid_log.f = float(self.pid.f)
    pid_log.output = float(-output_torque)
    pid_log.actualLateralAccel = float(measurement)
    pid_log.desiredLateralAccel = float(setpoint)
    pid_log.frictionTorque = float((friction_torque + self.break_boost) / u_max)
    pid_log.saturated = bool(self._check_saturation(abs(command) >= u_max - 1.0, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay, llk, model_data, frogpilot_toggles, fp_car_state=None):
    if self.plant is not None:
      return self._update_with_plant(active, CS, VM, params, steer_limited_by_safety, desired_curvature,
                                     curvature_limited, lat_delay, frogpilot_toggles, fp_car_state)

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES))
      expected_lateral_accel = self.requested_lateral_accel_buffer[-delay_frames]
      # TODO factor out lateral jerk from error to later replace it with delay independent alternative
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.requested_lateral_accel_buffer.append(future_desired_lateral_accel)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      desired_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / lat_delay

      measurement = measured_curvature * CS.vEgo ** 2
      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      self.previous_measurement = measurement

      low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, MIN_SPEED)) ** 2
      setpoint = lat_delay * desired_lateral_jerk + expected_lateral_accel
      error = setpoint - measurement
      error_lsf = error + low_speed_factor / self.torque_params.kp * error

      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error_lsf)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset
      # TODO jerk is weighted by lat_delay for legacy reasons, but should be made independent of it
      ff += get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error,
                                       -measurement_rate,
                                        feedforward=ff,
                                        speed=CS.vEgo,
                                        freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)  # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
