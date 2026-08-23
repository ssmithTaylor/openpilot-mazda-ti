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
# A stalled rack is not still. Through a real stall the steering rate exceeds 1.0 deg/s on 39 % of
# frames (p75 1.5, p90 3.8) -- it creeps and jitters against stiction rather than stopping dead.
# With the threshold at 1.0 the `stuck` flag had a median run length of TWO frames and only 1 % of
# its runs (52 of 4243) survived the 0.20 s debounce, so the breaker fired at 34 episodes/hr
# against a 317/hr design point and the saturation modulation ran 0.1 s in an entire drive.
#
# Raising the threshold to 2.0 lifts coverage of genuine saturated stalls from 5.8 % to 29.8 % of
# their duration, at a straight-road firing rate of 0.08 % against 0.02 % before -- still, in
# practice, nothing. A leaky counter tolerating 20 % non-stuck frames reaches 42.8 % but triples
# the straight-road rate to 0.27 %; that trade is available if 30 % proves too little, but ripple
# on a straight is what the driver actually feels, so it is not taken by default.
#
# FREE moves with STICK to keep the hysteresis: equal thresholds would chatter the boost on and off.
BREAK_STICK_RATE = 2.0     # deg/s: below this the wheel counts as not moving
BREAK_FREE_RATE = 4.0      # deg/s: above this it is moving and the boost lets go
BREAK_DEBOUNCE = 0.20      # s the condition must hold -- the wheel passes through zero rate on every
                           # direction reversal, and that is not the same thing as being stuck
BREAK_MAX = 0.15           # normalized torque, ~90 counts at a 600 ceiling: the p90 of what was needed
BREAK_RAMP = 0.25          # s to reach full boost, so it is never a step onto the wheel

# Optional smoothing of the finished command ("Smooth Steering Output"). Measured on straight
# road above 60 km/h, the command carries 33 counts RMS above 0.5 Hz -- a third of its total
# variation -- which the rack turns into 0.45 deg of wheel motion and 0.061 m/s^2 of car motion.
# The plan it is following has only 0.6 % of its power up there, so this is the controller's own
# content, not something the road asked for. Whether removing it feels better or just vague is a
# question for the driver, hence the toggle.
OUT_FILTER_TAU = 0.25      # s
OUT_FILTER_FLAG = 8        # added to plantState while active, so a drive can be split on it

# Breakaway prediction, from the unsaturated column torque on 0x75. Across 23 recorded stalls
# (demand over 3.0 m/s^2, command at the clip, wheel stopped for at least 0.30 s) this separates
# the ones that free themselves from the ones that do not, with no overlap: 123-202 counts broke
# free and went on to make the corner, 203-337 stayed locked. Wheel angle does not separate them.
# The margin below 202 is deliberate -- being wrong in the "predict it will recover" direction
# suppresses a warning the driver wanted, so only claim recovery well inside the boundary.
BREAKAWAY_TORQUE = 202.0   # counts of column torque at which the rack stops answering
BREAKAWAY_MARGIN = 25.0    # counts of headroom before claiming the wheel will move

# plantState bits 16, 128 and 256 are RETIRED, never to be reused: stall modulation (bit 16 --
# armed for months, never once triggered in 5 h of corner-heavy logs), the demand cap (bit 128 --
# its sustained sub-ceiling command tripped the interceptor's 0x11 watchdog mid-corner, route
# 00000277), and the lookahead feedforward (bit 256 -- wandered on the road in both its constant
# and demand-gated forms, because it swaps the FF source onto the near-field model plan, which
# mirrors the car's own motion). Old rlogs carry these bits, so their meaning is frozen; the
# decision docs in Stash hold the full evidence and the only terms on which each could return.

# Friction compensation ("Compensate Steering Friction"). The steady map has no friction term,
# and the rack eats a measured, stable slice of every command before anything moves: identified
# on 1.05M stall-excluded quasi-steady state-3 frames across four plant routes (warm, cold, wet),
# the implied command-domain friction is route-stable (47/45/45/48 counts median), direction-
# symmetric (47 left / 50 right), speed-flat (40-50 across 45-120 km/h), and load-dependent --
# ~36 counts at light command rising to ~75-80 at heavy, u_fric ~= 30 + 0.09*|u|. Two other
# instruments independently produce the same number: the breaker study's "median 30 counts more
# than it was getting" (p75 63-91), and the integrator's standing +/-0.08 la trim (~ comp times
# the local slope), which is the loop today paying for this term reactively on every curve.
#
# Applied on the COMMAND after the plant inverse, in the demand's direction, smoothly gated
# through zero so straights stay untouched, and tapered out approaching the clip so it can never
# re-pin the command the demand cap keeps breathing. Deliberately NOT inside the plant model:
# forward(), la_max and the NNFF gain-correction reference stay pure measured maps.
FRIC_COMP_BASE = 30.0      # counts, the Coulomb part
FRIC_COMP_LOAD = 0.09      # counts per count of command, the load-dependent part
FRIC_COMP_MAX = 85.0       # cap near the identified p75; beyond is breaker/stall territory
# The zero gate works on a LOW-PASSED demand with a deadzone, not the instantaneous one: straight
# road carries +-0.1 m/s^2 of demand dither, and a bare tanh gate would turn that into 15-30
# counts of direction-flipping torque at the rim (measured on route 276's straights) -- on
# exactly the roads whose calm is the plant branch's best quality. Filtered and deadzoned, a
# dithering straight reads ~0 while sustained cornering passes the gate within half a second;
# the reactive relay and breaker still own the transients the filter delays.
FRIC_COMP_GATE_TAU = 0.5   # s, low-pass on the demand feeding the gate
FRIC_COMP_LA_DEAD = 0.08   # m/s^2 of filtered demand below which the compensation is exactly zero
FRIC_COMP_LA_SOFT = 0.15   # m/s^2, tanh softness above the deadzone (full by ~0.4 sustained)
FRIC_COMP_KEEPOUT = 0.95   # of u_max: the compensation contributes nothing above this -- near
                           # the ceiling, friction is the breaker's problem, and headroom there
                           # belongs to feedback
FRIC_COMP_MIN_AUTHORITY = 0.5  # m/s^2: below this the actuator set is absent or mid-change and
                               # the inverse already commands almost nothing
FRIC_COMP_FLAG = 512       # added to plantState while the compensation is actually contributing

# Committed setpoint ("Hold the Corner Line"). Measured on routes 0000027a/00000276: in the
# corner-oscillation band (0.4-3 s) the model plan is not an independent target -- it TRAILS the
# car's own motion by 0.1-0.3 s and keeps only ~half its promises at 1 s (self-commitment
# r~0.47). The scallop cycle needs that: rack sticks -> error grows -> breakaway overshoot ->
# the plan eases IN SYMPATHY with the car's swing -> command unloads -> rack re-sticks. This
# filter attacks exactly the sympathetic-release step: the reference the controller tracks (and
# the feedforward it holds) may DEEPEN at the plan's own pace but RELEASES slowly, so one
# overshoot cannot talk the controller out of the corner.
#
# The obvious alternative -- a symmetric low-pass on the reference -- was replayed over the same
# corners and falsified before it reached the car: the mirror and the road overlap in frequency,
# so tau 0.3-0.6 s bought only 3-13 % oscillation-band rejection while adding 250-910 ms to
# every corner entry. The ratchet replays at 70-100 ms median entry cost.
COMMIT_TAU_DEEPEN = 0.10   # s: toward a deeper ask (or any sign conflict) the reference is fast
COMMIT_TAU_RELEASE = 0.60  # s: easing off is slow -- the commitment
# Two hard guards, both from the replay's tail: without them the worst exit overhang was 4.2-4.5 s
# (the ratchet holding an old direction through an S-transition); with them, max 120-160 ms and
# p90 <= 128 ms over 92 corner exits on three routes, while the in-corner commitment stays
# ~+0.045 m/s^2.
COMMIT_MAX_EXTRA = 0.5     # m/s^2 the committed reference may exceed the live ask by, at most
                           #   (the sign-conflict fast path is the other guard, in the filter)
COMMIT_GATE_TAU = 0.5      # s low-pass on |request| feeding the gate, as the friction comp does
COMMIT_GATE_ON = 1.0       # m/s^2 where the commitment starts blending in
COMMIT_GATE_FULL = 1.8     # fully committed here -- real corners, never straights
COMMIT_BLEND_RAMP = 0.5    # s, so the gate's motion and the mid-drive toggle never step
COMMIT_FLAG = 1024         # added to plantState while the commitment is blended in


class CommitFilter:
  """The asymmetric ratchet: fast toward a deeper (or opposite-signed) ask, slow easing off,
  never more than COMMIT_MAX_EXTRA beyond the live ask. Pure, so it can be replayed and tested."""

  def __init__(self, dt):
    self.a_deepen = 1.0 - math.exp(-dt / COMMIT_TAU_DEEPEN)
    self.a_release = 1.0 - math.exp(-dt / COMMIT_TAU_RELEASE)
    self.x = 0.0

  def update(self, v):
    fast = (v * self.x) < 0.0 or abs(v) > abs(self.x)
    self.x += (self.a_deepen if fast else self.a_release) * (v - self.x)
    lim = abs(v) + COMMIT_MAX_EXTRA
    self.x = float(np.clip(self.x, -lim, lim))
    return self.x

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
    self.fric_gate_filter = FirstOrderFilter(0.0, FRIC_COMP_GATE_TAU, self.dt)
    # Committed reference: one ratchet for the feedforward source, one for the delayed setpoint
    # (the same signal at two delays -- sharing one instance would double-step its state), plus
    # the gate and the blend. All kept warm while inactive so engaging mid-corner is continuous.
    self.commit_ff_filter = CommitFilter(self.dt)
    self.commit_sp_filter = CommitFilter(self.dt)
    self.commit_gate_filter = FirstOrderFilter(0.0, COMMIT_GATE_TAU, self.dt)
    self.commit_blend = 0.0
    self.friction_torque = FRICTION_TORQUE
    self.break_frames = 0        # how long the wheel has been stuck with an error worth acting on
    self.break_boost = 0.0       # counts, ramped, signed in the command's frame
    self.out_filter = FirstOrderFilter(0.0, OUT_FILTER_TAU, self.dt)
    self.out_filter_on = 0.0     # blend weight, ramped, so both directions are continuous

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
    self.out_filter.x = 0.0
    self.out_filter_on = 0.0
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
    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY

    # Committed reference, see COMMIT_TAU_DEEPEN above. Both ratchets, the gate and the blend run
    # while inactive too (same rationale as the request filter above: engaging mid-corner must
    # start from the truth). The setpoint is read here rather than below for the same reason.
    delay_frames = int(np.clip(lat_delay / self.dt, 1, self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES))
    setpoint = self.requested_lateral_accel_buffer[-delay_frames]
    commit_ff = self.commit_ff_filter.update(future_lateral_accel)
    commit_sp = self.commit_sp_filter.update(setpoint)
    commit_on = bool(getattr(frogpilot_toggles, "lat_commit_setpoint", False))
    commit_gate = float(np.clip((self.commit_gate_filter.update(abs(future_lateral_accel)) - COMMIT_GATE_ON) /
                                (COMMIT_GATE_FULL - COMMIT_GATE_ON), 0.0, 1.0))
    commit_step = self.dt / COMMIT_BLEND_RAMP
    self.commit_blend += float(np.clip((commit_gate if commit_on else 0.0) - self.commit_blend,
                                       -commit_step, commit_step))
    ff_lat_accel = future_lateral_accel + self.commit_blend * (commit_ff - future_lateral_accel)
    tracked_setpoint = setpoint + self.commit_blend * (commit_sp - setpoint)

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

    # Wind the integrator against the authority this car actually has in its current state -- at
    # low speed, or with the stock LKAS path blocked, that is a fraction of what it has on the
    # highway, and winding past it only delays the recovery.
    la_max = self.plant.la_max(CS.vEgo)
    self.pid.set_limits(la_max, -la_max)

    # The error tracks the delay-aligned request (the one the car has had time to answer, as
    # upstream does) -- committed in corners, raw otherwise; both computed above.
    lat_accel_offset = self.torque_params.latAccelOffset
    error = tracked_setpoint - measurement
    low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, MIN_SPEED)) ** 2
    error_lsf = error + low_speed_factor / self.torque_params.kp * error

    # Feedforward is the demand at the tire, in lateral accel; the model turns it into counts
    # below. latAccelOffset is what the car does at zero torque (device roll misalignment).
    ff = ff_lat_accel - roll_compensation - lat_accel_offset

    freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
    output_lataccel = self.pid.update(error_lsf,
                                      -measurement_rate,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

    u_max = self.plant.ti_steer_max
    command = self.plant.inverse(output_lataccel, CS.vEgo)

    # Friction compensation, see FRIC_COMP_BASE above. Proactive (demand-direction), unlike the
    # error-direction relay below; with the feedforward no longer arriving short, the relay and
    # the integrator both have less to make up.
    gate_la = self.fric_gate_filter.update(output_lataccel)
    fric_comp = 0.0
    if bool(getattr(frogpilot_toggles, "lat_friction_comp", False)) and la_max > FRIC_COMP_MIN_AUTHORITY:
      comp_mag = min(FRIC_COMP_BASE + FRIC_COMP_LOAD * abs(command), FRIC_COMP_MAX)
      comp_mag = min(comp_mag, max(0.0, FRIC_COMP_KEEPOUT * u_max - abs(command)))
      over = max(abs(gate_la) - FRIC_COMP_LA_DEAD, 0.0)
      fric_comp = comp_mag * math.tanh(over / FRIC_COMP_LA_SOFT) * (1.0 if gate_la >= 0 else -1.0)
      command = float(np.clip(command + fric_comp, -u_max, u_max))

    # Breakaway friction, in torque space: what it compensates is stiction in the rack, which is a
    # torque, and in lateral-accel space the same relay becomes hundreds of counts wherever the
    # plant is soft. The dead-zone keeps it from chattering on measurement noise. With no actuator
    # in play there is nothing to break away, so send nothing rather than a standing offset that
    # would step onto the wheel the moment an actuator comes back.
    friction_torque = 0.0
    relay_off = bool(getattr(frogpilot_toggles, "lat_no_friction_relay", False))
    if la_max > 0.0 and not relay_off:
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

    # Below the breakaway threshold the rack frees itself; this feeds the saturation report so a
    # lightly-loaded clip does not raise a warning. A missing reading (0, e.g. gen2) reads as
    # not-recovering, which falls through to the old always-warn behaviour.
    col_trq = abs(float(getattr(fp_car_state, "columnTorque", 0.0) or 0.0))
    will_recover = 0.0 < col_trq < (BREAKAWAY_TORQUE - BREAKAWAY_MARGIN)

    # Optional output smoothing. The filter tracks the command even while it is switched off, so
    # turning it on mid-corner continues from where the command already is instead of stepping
    # down to a stale value. It cannot raise anything: the clip below and the rate limiter and
    # the interceptor's own bounds all still apply.
    smooth = bool(getattr(frogpilot_toggles, "lat_output_filter", False))
    filtered = self.out_filter.update(command)
    # Blend rather than switch. Enabling is continuous either way because the filter tracks the
    # command while it is off, but DISABLING would hand back the raw command in one frame -- a step
    # the size of whatever ripple was being removed. Ramping the blend makes both directions
    # continuous, which is the point of a control meant to be flipped while moving.
    self.out_filter_on += float(np.clip((1.0 if smooth else 0.0) - self.out_filter_on,
                                        -self.dt / OUT_FILTER_TAU, self.dt / OUT_FILTER_TAU))
    command = float(np.clip(command + self.out_filter_on * (filtered - command), -u_max, u_max))

    output_torque = command / u_max
    self.plant.u_prev = -command   # wire convention: actuators.steer is positive left

    if self.out_filter_on > 0.5:
      pid_log.plantState = int(plant_state) + OUT_FILTER_FLAG   # so a drive can be split on it
    if abs(fric_comp) > 1.0:
      pid_log.plantState = int(pid_log.plantState) + FRIC_COMP_FLAG
    if self.commit_blend > 0.5:
      pid_log.plantState = int(pid_log.plantState) + COMMIT_FLAG
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
    # The rack is loaded well below where it stops answering, so it is going to move: sitting at the
    # clip here is not a loss of control and does not deserve a warning. 0 means the car did not
    # publish a reading, which falls through to the old behaviour.
    pid_log.saturated = bool(self._check_saturation(abs(command) >= u_max - 1.0, CS, steer_limited_by_safety,
                                                    curvature_limited, expected_to_recover=will_recover))

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
