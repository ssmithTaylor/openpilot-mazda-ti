#!/usr/bin/env python3
# Twilsonco's Lateral Neural Network Feedforward Controller
import json
import math
import numpy as np
import os

from collections import deque
from difflib import SequenceMatcher

from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.modeld.constants import ModelConstants

from openpilot.frogpilot.common.frogpilot_variables import NNFF_MODELS_PATH, get_nnff_model_files, get_nnff_substitutes

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

# dict used to rename activation functions whose names aren't valid python identifiers
ACTIVATION_FUNCTION_NAMES = {"σ": "sigmoid"}

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [12, 3, 1, 0]

LAT_PLAN_MIN_IDX = 5

class FluxModel:
  def __init__(self, params_file):
    with open(params_file, "r") as f:
      params = json.load(f)

    self.input_size = params["input_size"]
    self.output_size = params["output_size"]

    self.input_mean = np.array(params["input_mean"], dtype=np.float32).T
    self.input_std = np.array(params["input_std"], dtype=np.float32).T

    self.layers = []
    for layer_params in params["layers"]:
      bias_array = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith("_b"))], dtype=np.float32).T
      weight_array = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith("_W"))], dtype=np.float32).T

      activation = layer_params["activation"]
      for name, replacement in ACTIVATION_FUNCTION_NAMES.items():
        activation = activation.replace(name, replacement)

      self.layers.append((weight_array, bias_array, activation))

    self.validate_layers()
    self.check_for_friction_override()

  # Begin activation functions.
  # These are called by name using the keys in the model json file
  @staticmethod
  def identity(x):
    return x

  @staticmethod
  def sigmoid(x):
    return 1 / (1 + np.exp(-x))
  # End activation functions

  def check_for_friction_override(self):
    self.friction_override = self.evaluate([10.0, 0.0, 0.2]) < 0.1

  def evaluate(self, input_array):
    if len(input_array) != self.input_size:
      # If the input is length 2-4, then it's a simplified evaluation.
      # In that case, need to add on zeros to fill out the input array to match the correct length.
      if len(input_array) >= 2:
        input_array += [0] * (self.input_size - len(input_array))
      else:
        raise ValueError(f"Input length {len(input_array)} must be 2 or greater")

    # Rescale the input array using the input_mean and input_std
    output = self.forward((np.array(input_array, dtype=np.float32) - self.input_mean) / self.input_std)
    return float(output[0, 0])

  def forward(self, x):
    for weights, bias, activation in self.layers:
      x = getattr(self, activation)(x @ weights + bias)
    return x

  def validate_layers(self):
    for _, _, activation in self.layers:
      if not hasattr(self, activation):
        raise ValueError(f"Unknown activation function: {activation}")

def get_lookahead_value(future_values, current_value):
  if not future_values:
    return current_value

  if any(sign(value) != sign(current_value) for value in future_values):
    return 0.0

  return min(future_values + [current_value], key=abs)

def get_nn_model(car, eps_firmware) -> FluxModel | None:
  model_path = get_nn_model_path(car, eps_firmware)
  if model_path:
    Params().put("NNFFModelName", car.replace("_", " "))
    return FluxModel(model_path)
  return None

def get_nn_model_path(car, eps_firmware) -> str | None:
  def best_model_path(query):
    candidates = get_nnff_model_files()
    if not candidates:
      return None, 0.0

    best = max(candidates, key=lambda model: similarity(model, query))
    return os.path.join(NNFF_MODELS_PATH, f"{best}.json"), similarity(best, query)

  def find_valid_model(*queries_with_candidates):
    for query, candidate in queries_with_candidates:
      path, score = best_model_path(query)
      if path and candidate in path and score >= 0.9:
        return path
    return None

  substitutes = get_nnff_substitutes()
  sub_candidate = substitutes.get(car, car)

  candidates_to_check = [car]
  if car != sub_candidate:
    candidates_to_check.append(sub_candidate)

  queries = []
  for candidate in candidates_to_check:
    query_with_fw = f"{candidate} {eps_firmware}" if len(eps_firmware) > 3 else candidate
    queries.append((query_with_fw, candidate))
    queries.append((candidate, candidate))

  return find_valid_model(*queries)

def get_predicted_lateral_jerk(lat_accels, t_diffs):
  # compute finite difference between subsequent model_data.acceleration.y values
  # this is just two calls of np.diff followed by an element-wise division
  return (np.diff(lat_accels) / t_diffs).tolist()

# At a given roll, if pitch magnitude increases, the
# gravitational acceleration component starts pointing
# in the longitudinal direction, decreasing the lateral
# acceleration component. Here we do the same thing
# to the roll value itself, then passed to nnff.
def roll_pitch_adjust(roll, pitch):
  return roll * math.cos(pitch)

def sign(x):
  return 1.0 if x > 0 else -1.0 if x < 0 else 0.0

def similarity(s1: str, s2: str) -> float:
  return SequenceMatcher(None, s1, s2).ratio()

# Added to plantState on every frame NNFF produced, so a drive with the controller flipped part
# way through splits cleanly into NNFF and plant-branch populations. Without it NNFF frames log as
# plantState 0, which is indistinguishable from 'no actuator state known' -- the two arms of a
# comparison were previously separable only by which drive they came from. See lateral_plant.py
# for the full bit allocation.
NNFF_FLAG = 32
NNFF_GAIN_FLAG = 64  # added while the gain correction is actually moving the command

# Gain correction. The model is trained on another car, and its own steady-state gain is not this
# car's: MAZDA_CX9_2021 asks for a given lateral accel as though 1 unit of normalized torque buys
# 2.0-2.5 m/s^2 (rising with speed), while this car's measured plant buys considerably less. That
# is a pure scale error and it is the easy kind to fix -- but NOT with a constant. Every attempt to
# pin one down this session compared quantities that turned out not to be the same quantity: a
# torqued FRICTION carrying a 1.5 factor that params.toml does not, and a TLS small-signal slope
# read against a network's steady-state gain, 24 % apart on the same car and the same data.
#
# So no constants. Both sides are measured on the same frame, in normalized torque, for the same
# lateral accel: what the network asks for, against what the plant model -- fitted on THIS car,
# and aware of speed and of which actuators are live -- says that lateral accel actually needs.
# Agreement gives exactly 1.0 and changes nothing.
GAIN_TAU = 1.0             # s, so a change in actuator state ramps rather than steps
GAIN_MIN, GAIN_MAX = 0.7, 1.5
# The ratio is taken at the lateral accel actually being asked for, because neither side is a
# straight line: the network's own gain climbs with both speed and lateral accel (2.04 to 2.51
# across the range for MAZDA_CX9_2021) and the plant is explicitly curved and clipped. A fixed
# reference would correct the gain somewhere the car is not. Floored so it is never the ratio of
# two near-zero numbers, and capped inside present authority so the inverse is not reading back
# the ceiling.
GAIN_REF_MIN = 0.5         # m/s^2
GAIN_REF_CEIL = 0.9        # of la_max

class LatControlNNFF(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.lat_torque_nn_model = get_nn_model(CP.carFingerprint, str(next((fw.fwVersion for fw in CP.carFw if fw.ecu == "eps"), "")).replace("\\", ""))
    self.nnff_loaded = self.lat_torque_nn_model is not None

    # Observed, never steered by: NNFF computes its own feedforward. This is here only so NNFF
    # frames carry the same actuator-state label the plant branch writes, using the same
    # classifier -- a comparison between the two is worthless if each labels its frames its own way.
    self.plant = getattr(CI, "lateral_plant", None)
    # 1.0 is 'no correction', which is also what it holds whenever the correction cannot be
    # computed, so a missing plant or an unloaded model is a no-op rather than a surprise.
    self.gain_ratio = FirstOrderFilter(1.0, GAIN_TAU, dt)

    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    # Instantaneous lateral jerk changes very rapidly, making it not useful on its own,
    # however, we can "look ahead" to the future planned lateral jerk in order to gauge
    # whether the current desired lateral jerk will persist into the future, i.e.
    # whether it's "deliberate" or not. This lets us simply ignore short-lived jerk.
    # Note that LAT_PLAN_MIN_IDX is defined above and is used in order to prevent
    # using a "future" value that is actually planned to occur before the "current" desired
    # value, which is offset by the steerActuatorDelay.
    self.friction_look_ahead_bp = [9.0, 30.0]  # corresponding speeds in m/s in [0, ~40] in 1.0 increments
    self.friction_look_ahead_v = [1.4, 2.0]  # how many seconds in the future to look ahead in [0, ~2.1] in 0.1 increments

    # Scaling the lateral acceleration "friction response" could be helpful for some.
    # Increase for a stronger response, decrease for a weaker response.
    self.lat_accel_friction_factor = 0.7  # in [0, 3], in 0.05 increments. 3 is arbitrary safety limit
    self.lat_jerk_friction_factor = 0.4

    # precompute time differences between ModelConstants.T_IDXS
    self.t_diffs = np.diff(ModelConstants.T_IDXS)

    self.pitch = FirstOrderFilter(0.0, 0.5, 0.01)

    # NN model takes current v_ego, lateral_accel, lat accel/jerk error, roll, and past/future/planned data
    # of lat accel and roll
    # Past value is computed using previous desired lat accel and observed roll
    self.nn_friction_override = self.lat_torque_nn_model.friction_override if self.lat_torque_nn_model else False

    # setup future time offsets
    self.future_times = [0.3, 0.6, 1.0, 1.5]  # seconds in the future
    self.nn_future_times = [time + CP.steerActuatorDelay for time in self.future_times]

    # setup past time offsets
    self.past_times = [-0.3, -0.2, -0.1]
    history_check_frames = [int(abs(time) * 100) for time in self.past_times]
    self.history_frame_offsets = [history_check_frames[0] - time for time in history_check_frames]
    self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
    self.past_future_len = len(self.past_times) + len(self.nn_future_times)
    self.roll_deque = deque(maxlen=history_check_frames[0])

  def reset(self):
    super().reset()
    if self.plant is not None:
      # re-arm ramp-in for the next engagement, exactly as the plant branch does
      self.plant.reset()
    self.gain_ratio.x = 1.0

  def _model_steady_ff(self, v_ego, la, roll):
    """The model's own steady-state feedforward for `la`, in normalized torque: no lateral jerk,
    and every past and future lateral-accel and roll slot set to the present value. The 18 inputs
    are v_ego, lateral_accel, lateral_jerk, roll, then lateral accel at t-0.3/-0.2/-0.1 and
    t+0.3/+0.6/+1.0/+1.5, then roll at the same seven offsets. Holding them flat is what separates
    the model's gain from its shape -- the lookahead is the part worth keeping, so it must not be
    what the correction measures."""
    return self.lat_torque_nn_model.evaluate([v_ego, la, 0.0, roll] + [la] * 3 + [la] * 4 +
                                             [roll] * 3 + [roll] * 4)

  def _gain_ratio(self, CS, roll, la_target, frogpilot_toggles):
    """How much to scale the network's output so it lands where this car actually is."""
    if (self.plant is None or not self.nnff_loaded
        or not getattr(frogpilot_toggles, "nnff_gain_correction", False)):
      return self.gain_ratio.update(1.0)

    la_max = self.plant.la_max(CS.vEgo)
    if not np.isfinite(la_max) or la_max <= 0.0:
      return self.gain_ratio.update(1.0)
    hi = max(GAIN_REF_CEIL * la_max, GAIN_REF_MIN)
    ref = float(np.clip(abs(la_target), GAIN_REF_MIN, hi))

    model_ff = float(self._model_steady_ff(CS.vEgo, ref, roll))
    plant_ff = float(self.plant.inverse(ref, CS.vEgo)) / max(self.plant.ti_steer_max, 1.0)
    # inverse() returns 0 when neither actuator is in play, and the network can sit near zero at
    # low reference; either way there is no ratio to take, so hold at no-correction.
    if abs(model_ff) < 1e-3 or abs(plant_ff) < 1e-3:
      return self.gain_ratio.update(1.0)
    return self.gain_ratio.update(float(np.clip(plant_ff / model_ff, GAIN_MIN, GAIN_MAX)))

  def _plant_state(self, active, CS, fp_car_state, frogpilot_toggles):
    """Byte-for-byte the plant branch's classifier (LatControlTorque._plant_state). Unknown state
    (frogpilotCarState stale) is read as 'everything is working', which under-labels authority
    rather than over-claiming it."""
    if self.plant is None:
      return 0
    ti_max = getattr(frogpilot_toggles, "ti_steer_max", None)
    if ti_max is not None and abs(ti_max - self.plant.ti_steer_max) > 0.5:
      self.plant.set_ti_steer_max(ti_max)
    if fp_car_state is None:
      return self.plant.update_state(CS.vEgo, False, 0.0, True, active=active)
    return self.plant.update_state(CS.vEgo, fp_car_state.lkasBlocked, fp_car_state.lkasEffective,
                                   fp_car_state.tiActive, active=active)

  def update_live_delay(self, lateral_delay):
    self.nn_future_times = [time + lateral_delay for time in self.future_times]
    self.past_future_len = len(self.past_times) + len(self.nn_future_times)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay, llk, model_data, frogpilot_toggles, fp_car_state=None):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    # version 2 marks the struct as carrying plantState; the plant branch sets the same value, so
    # any filter that selects on it now keeps NNFF frames instead of silently dropping them.
    pid_log.version = 2
    plant_state = self._plant_state(active, CS, fp_car_state, frogpilot_toggles)
    pid_log.plantState = int(plant_state) + NNFF_FLAG
    if not active:
      output_torque = 0.0
      pid_log.active = False
      if self.plant is not None:
        self.plant.u_prev = 0.0
    else:
      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation
      if self.nnff_loaded and frogpilot_toggles.nnff or frogpilot_toggles.nnff_lite:
        if self.use_steering_angle:
          actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
          actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
        else:
          actual_lateral_jerk = 0.0

        model_good = model_data is not None and len(model_data.orientation.x) >= CONTROL_N

        if model_good:
          # prepare "look-ahead" desired lateral jerk
          lookahead = interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
          friction_upper_idx = next((idxs for idxs, value in enumerate(ModelConstants.T_IDXS) if value > lookahead), 16)

          predicted_lateral_jerk = get_predicted_lateral_jerk(model_data.acceleration.y, self.t_diffs)
          desired_lateral_jerk = (interp(lat_delay, ModelConstants.T_IDXS, model_data.acceleration.y) - desired_lateral_accel) / lat_delay

          lookahead_lateral_jerk = get_lookahead_value(predicted_lateral_jerk[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_lateral_jerk)

          if not self.use_steering_angle or lookahead_lateral_jerk == 0.0:
            lookahead_lateral_jerk = 0.0
            actual_lateral_jerk = 0.0
            self.lat_accel_friction_factor = 1.0

          lateral_jerk_setpoint = self.lat_jerk_friction_factor * lookahead_lateral_jerk
          lateral_jerk_measurement = self.lat_jerk_friction_factor * actual_lateral_jerk
        else:
          lateral_jerk_setpoint = 0
          lateral_jerk_measurement = 0
          lookahead_lateral_jerk = 0

        if self.nnff_loaded and model_good and frogpilot_toggles.nnff:
          # update past data
          pitch = 0
          roll = params.roll
          if len(llk.calibratedOrientationNED.value) > 1:
            pitch = self.pitch.update(llk.calibratedOrientationNED.value[1])
            roll = roll_pitch_adjust(roll, pitch)

          self.roll_deque.append(roll)
          self.lateral_accel_desired_deque.append(desired_lateral_accel)

          # prepare past and future values
          # adjust future times to account for longitudinal acceleration
          adjusted_future_times = [time + 0.5 * CS.aEgo * (time / max(CS.vEgo, 1.0)) for time in self.nn_future_times]
          past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, offset)] for offset in self.history_frame_offsets]
          future_rolls = [roll_pitch_adjust(interp(time, ModelConstants.T_IDXS, model_data.orientation.x) + roll, interp(time, ModelConstants.T_IDXS, model_data.orientation.y) + pitch) for time in adjusted_future_times]

          past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque)-1, offset)] for offset in self.history_frame_offsets]
          future_lateral_accels = [interp(time, ModelConstants.T_IDXS[:CONTROL_N], model_data.acceleration.y) for time in adjusted_future_times]

          base_input = [CS.vEgo, roll]
          nnff_common = past_rolls + future_rolls

          # compute NNFF error response
          nnff_setpoint_input = base_input[:1] + [setpoint, lateral_jerk_setpoint] + base_input[1:] + [setpoint] * self.past_future_len + nnff_common
          nnff_measurement_input = base_input[:1] + [measurement, lateral_jerk_measurement] + base_input[1:] + [measurement] * self.past_future_len + nnff_common

          torque_from_setpoint = self.lat_torque_nn_model.evaluate(nnff_setpoint_input)
          torque_from_measurement = self.lat_torque_nn_model.evaluate(nnff_measurement_input)

          pid_log.error = torque_from_setpoint - torque_from_measurement

          error_blend = interp(abs(desired_lateral_accel), [1.0, 2.0], [0.0, 1.0])
          if error_blend > 0.0:  # blend in stronger error response when in high lat accel
            torque_from_error = self.lat_torque_nn_model.evaluate([CS.vEgo, setpoint - measurement, lateral_jerk_setpoint - lateral_jerk_measurement, 0.0])
            if sign(pid_log.error) == sign(torque_from_error) and abs(pid_log.error) < abs(torque_from_error):
              pid_log.error = pid_log.error * (1.0 - error_blend) + torque_from_error * error_blend

          # compute feedforward (same as nn setpoint output)
          error = setpoint - measurement
          friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
          # latAccelOffset (torqued) is the lateral accel the car has at zero torque beyond what roll
          # explains; LatControlTorque subtracts it from the feedforward's lateral accel, do the same
          # for every lateral-accel input of the feedforward evaluation (0.0 unless live params apply)
          lat_accel_offset = self.torque_params.latAccelOffset
          nn_input = [CS.vEgo, desired_lateral_accel - lat_accel_offset, friction_input, roll] + \
                     [x - lat_accel_offset for x in past_lateral_accels_desired] + \
                     [x - lat_accel_offset for x in future_lateral_accels] + nnff_common
          ff = self.lat_torque_nn_model.evaluate(nn_input)

          # apply friction override for cars with low NN friction response
          if self.nn_friction_override:
            pid_log.error += self.torque_from_lateral_accel(0.0, self.torque_params)

          # Scale the network's output to this car. Both the feedforward and the error term come
          # out of the same network in the same torque scale, so scaling only one would quietly
          # change the loop gain by the same factor it was meant to correct.
          gain = self._gain_ratio(CS, roll, desired_lateral_accel - lat_accel_offset, frogpilot_toggles)
          if abs(gain - 1.0) > 1e-3:
            ff *= gain
            pid_log.error = float(pid_log.error) * gain
            pid_log.plantState = int(pid_log.plantState) + NNFF_GAIN_FLAG
        else:
          torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params)
          torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params)

          pid_log.error = float(torque_from_setpoint - torque_from_measurement)

          error = desired_lateral_accel - actual_lateral_accel
          friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
          # same offset treatment as LatControlTorque
          ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel - self.torque_params.latAccelOffset, self.torque_params)
      else:
        torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params)
        torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params)

        pid_log.error = float(torque_from_setpoint - torque_from_measurement)

        # same offset treatment as LatControlTorque
        ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel - self.torque_params.latAccelOffset, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(actual_lateral_accel)
      pid_log.desiredLateralAccel = float(desired_lateral_accel)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))
      if self.plant is not None:
        # Same wire convention as the plant branch (actuators.steer is positive left), in counts.
        # The dead-stock and ramp-settle tests compare LKAS_EFFECTIVE against the command that
        # asked for it, so leaving this at zero would make the stock path look dead under NNFF.
        self.plant.u_prev = -float(output_torque) * self.plant.ti_steer_max

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
