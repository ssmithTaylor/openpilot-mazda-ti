import json

from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.mazda import mazdacan
from openpilot.selfdrive.car.mazda.values import CarControllerParams, Buttons, MazdaFlags, TI_STATE
from openpilot.common.realtime import ControlsTimer as Timer, DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.ti_apply_steer_last = 0
    self.packer = CANPacker(dbc_name)
    self.brake_counter = 0
    self.frame = 0
    self.ccp = CarControllerParams(CP)
    self.hold_timer = Timer(6.0)
    self.hold_delay = Timer(.5) # delay before we start holding as to not hit the brakes too hard
    self.resume_timer = Timer(0.5)
    self.cancel_delay = Timer(0.07) # 70ms delay to try to avoid a race condition with stock system
    self.acc_filter = FirstOrderFilter(0.0, .1, DT_CTRL, initialized=False)
    self.filtered_acc_last = 0
    self.long_active_last = False
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.ti_live = {}
    self.reset_ti_stats()

  def reset_ti_stats(self):
    self.ti_stats = {k: 0 for k in ("engaged", "short", "rate_limited", "driver_limited",
                                    "at_clip", "peak_cmd", "peak_bias", "not_run", "viol", "ramp")}

  def record_ti_stats(self, CS, desired, sent):
    # Counters for A/B-ing a tuning change over a chosen stretch of road. "short" is how often the
    # command openpilot wanted got cut, and rate_limited/driver_limited attribute why -- that split
    # is what says whether to touch the ramp rate or the driver-torque backoff.
    s = self.ti_stats
    s["engaged"] += 1

    # Health first, and unconditionally -- these are the frames where the TI stopped cooperating.
    if CS.ti_state != TI_STATE.RUN:
      s["not_run"] += 1
    if CS.ti_violation:
      s["viol"] = int(CS.ti_violation)
    if CS.ti_ramp_down:
      s["ramp"] += 1
    s["peak_bias"] = max(s["peak_bias"], abs(int(CS.eps_torque_sensor - CS.out.steeringTorque)))

    # Command-side attribution only means something while the TI is actually taking commands.
    if not CS.ti_lkas_allowed:
      return

    if abs(desired) - abs(sent) > 5:
      s["short"] += 1
      # Compare the signed step so a sign crossing still registers, and only call it rate limiting
      # when the command was climbing -- a command collapsing under driver torque moves at
      # DELTA_DOWN, which would otherwise be miscounted as a rate limit and point at the wrong knob.
      if abs(sent) > abs(self.ti_apply_steer_last) and \
         abs(sent - self.ti_apply_steer_last) >= self.ccp.TI_STEER_DELTA_UP:
        s["rate_limited"] += 1
      # Only torque OPPOSING the command narrows the cap -- openpilot's driver-torque limit is
      # signed, and torque in the command's own direction widens the bound instead. Counting both
      # directions would blame the driver term for frames it had nothing to do with.
      if abs(CS.out.steeringTorque) > self.ccp.TI_STEER_DRIVER_ALLOWANCE and \
         (CS.out.steeringTorque * desired) < 0:
        s["driver_limited"] += 1
    if abs(sent) >= self.ccp.TI_STEER_MAX:
      s["at_clip"] += 1
    s["peak_cmd"] = max(s["peak_cmd"], abs(sent))

  def publish_ti_stats(self):
    # Clearing keeps the outgoing run under a second key so the two can be compared side by side.
    if self.params.get_bool("ClearTiStats"):
      self.params.put_bool("ClearTiStats", False)
      # Snapshot the persisted figures rather than the in-memory ones. This process restarts every
      # ignition cycle, so clearing while parked would otherwise stash a set of zeros as "previous"
      # and lose the run the user actually wanted to compare against.
      persisted = self.params.get("TiTuningStats")
      self.params.put_nonblocking("TiTuningStatsPrevious", persisted or json.dumps(self.ti_stats))
      self.reset_ti_stats()
    self.params.put_nonblocking("TiTuningStats", json.dumps({**self.ti_stats, "live": self.ti_live}))

  def apply_ti_tuning(self, frogpilot_toggles):
    # The TI limits live on self.ccp, which the rate/driver-torque limiters read every frame, so
    # writing them here takes effect immediately. Falls back to the compiled-in value whenever a
    # toggle is absent, which keeps older FrogPilot params files working.
    self.ccp.TI_STEER_MAX = getattr(frogpilot_toggles, "ti_steer_max", self.ccp.TI_STEER_MAX)
    self.ccp.TI_STEER_DELTA_UP = getattr(frogpilot_toggles, "ti_steer_delta_up", self.ccp.TI_STEER_DELTA_UP)
    self.ccp.TI_STEER_DELTA_DOWN = getattr(frogpilot_toggles, "ti_steer_delta_down", self.ccp.TI_STEER_DELTA_DOWN)
    self.ccp.TI_STEER_DRIVER_ALLOWANCE = getattr(frogpilot_toggles, "ti_steer_driver_allowance", self.ccp.TI_STEER_DRIVER_ALLOWANCE)
    self.ccp.TI_STEER_DRIVER_MULTIPLIER = getattr(frogpilot_toggles, "ti_steer_driver_multiplier", self.ccp.TI_STEER_DRIVER_MULTIPLIER)

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    can_sends = []

    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      self.apply_ti_tuning(frogpilot_toggles)
      # Live TI state, refreshed every frame regardless of engagement. The mode and violation are
      # kept on the CarState object rather than published in cereal, so nothing outside this
      # process can see them unless we forward them. "Is the interceptor healthy right now" should
      # not require having already driven.
      self.ti_live = {"mode": int(CS.ti_state), "viol": int(CS.ti_violation),
                      "ramp": bool(CS.ti_ramp_down), "version": int(CS.ti_version)}

    apply_steer = 0
    ti_apply_steer = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_steer = int(round(CC.actuators.steer * self.ccp.STEER_MAX))
      apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                     CS.out.steeringTorque, self.ccp)
      if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
        ti_new_steer = 0
        if CS.ti_lkas_allowed:
          ti_new_steer = int(round(CC.actuators.steer * self.ccp.TI_STEER_MAX))
          ti_apply_steer = apply_ti_steer_torque_limits(ti_new_steer, self.ti_apply_steer_last,
                                                    CS.out.steeringTorque, self.ccp)
        # Recorded outside the ti_lkas_allowed gate on purpose: that flag is false precisely when
        # the TI has left RUN or is ramping down, which are the frames the health counters exist
        # to catch. Gating on it would make them structurally unreachable.
        self.record_ti_stats(CS, ti_new_steer, ti_apply_steer)
    self.apply_steer_last = apply_steer
    self.ti_apply_steer_last = ti_apply_steer

    if self.CP.flags & MazdaFlags.GEN1:
      if CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
          # Send Resume button when planner wants car to move
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

      # send HUD alerts
      if self.frame % 50 == 0:
        ldw = CC.hudControl.visualAlert == VisualAlert.ldw
        steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
        # TODO: find a way to silence audible warnings so we can add more hud alerts
        steer_required = steer_required and CS.lkas_allowed_speed
        if not self.CP.flags & MazdaFlags.NO_FSC:
          can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

      if self.CP.flags & MazdaFlags.RADAR_INTERCEPTOR:
        hold = False
        if CS.out.standstill:
          hold = self.hold_timer.active()
        else:
          self.hold_timer.reset()

        if CC.longActive:
          raw_acc_output = CC.actuators.accel * 1150
          raw_acc_output = max(-1000, min(raw_acc_output, 1000))

          if self.params.get_bool("BlendedACC"):
            if self.params_memory.get_int("CEStatus"):
              self.acc_filter.update_alpha(abs(raw_acc_output-self.filtered_acc_last)/1000)
              filtered_acc_output = int(self.acc_filter.update(raw_acc_output))
            else:
              # we want to use the stock value in this case but we need a smooth transition.
              self.acc_filter.update_alpha(abs(CS.crz_info["ACCEL_CMD"]-self.filtered_acc_last)/1000)
              filtered_acc_output = int(self.acc_filter.update(CS.crz_info["ACCEL_CMD"]))

            CS.crz_info["ACCEL_CMD"] = int(filtered_acc_output)
            self.filtered_acc_last = filtered_acc_output
          else:
            acc_output = raw_acc_output

        if self.frame % 2 == 0:
          can_sends.extend(mazdacan.create_radar_command(self.packer, self.frame, CC.longActive, CS, hold))

    elif self.CP.flags & MazdaFlags.GEN2:
      raw_acc_output = (CC.actuators.accel * 200) + 2000
      if CC.longActive:
        if self.params.get_bool("BlendedACC"):
          if not self.long_active_last:
            # reset the filter when we start ACC
            self.acc_filter.initialized = False

          if self.params_memory.get_int("CEStatus"):
            self.acc_filter.update_alpha(abs(raw_acc_output-self.filtered_acc_last)/1000)
            filtered_acc_output = int(self.acc_filter.update(raw_acc_output))
          else:
            # we want to use the stock value in this case but we need a smooth transition.
            self.acc_filter.update_alpha(abs(CS.acc["ACCEL_CMD"]-self.filtered_acc_last)/1000)
            filtered_acc_output = int(self.acc_filter.update(CS.acc["ACCEL_CMD"]))

          acc_output = filtered_acc_output
          self.filtered_acc_last = filtered_acc_output

        else:
          acc_output = raw_acc_output

        if self.params.get_bool("ExperimentalLongitudinalEnabled"):
          CS.acc["ACCEL_CMD"] = acc_output

      self.long_active_last = CC.longActive
      resume = False
      hold = False
      if Timer.interval(2): # send ACC command at 50hz
        """
        Without this hold/resum logic, the car will only stop momentarily.
        It will then start creeping forward again. This logic allows the car to
        apply the electric brake to hold the car. The hold delay also fixes a
        bug with the stock ACC where it sometimes will apply the brakes too early
        when coming to a stop.
        """
        if CS.out.standstill: # if we're stopped
          if not self.hold_delay.active(): # and we have been stopped for more than hold_delay duration. This prevents a hard brake if we aren't fully stopped.
            if ((CC.cruiseControl.resume and CC.actuators.longControlState != LongCtrlState.stopping) or
                CC.cruiseControl.override or CS.out.gasPressed or
                (CC.actuators.longControlState == LongCtrlState.starting) or CS.acc["RESUME"]): # if we are resuming or overriding, we want to release the brake
              self.resume_timer.reset() # reset the resume timer so its active
            else: # otherwise we're holding
              hold = self.hold_timer.active() # hold for 6s. This allows the electric brake to hold the car.

        else: # if we're moving
          self.hold_timer.reset() # reset the hold timer so its active when we stop
          self.hold_delay.reset() # reset the hold delay

        resume = self.resume_timer.active() # stay on for 0.5s to release the brake. This allows the car to move.
        can_sends.append(mazdacan.create_acc_cmd(self, self.packer, CS.acc, hold, resume))

    # send steering command
    can_sends.extend(mazdacan.create_steering_control(
      self.packer, self.CP,
      self.frame, apply_steer, CS.cam_lkas,
      ti_apply_steer if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else None
    ))

    new_actuators = CC.actuators.as_builder()
    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR and CS.ti_lkas_allowed:
      # While the TI is in RUN it is the actuator that actually steers the car, so report its
      # command rather than the stock LKAS one. torqued fits its model to actuatorsOutput.steer,
      # and the two commands are limited differently (TI_STEER_DELTA_UP vs STEER_DELTA_UP, driver
      # multiplier 40 vs 1), so reporting the stock command trains it on a signal the car is not
      # following. Falls back to the stock command whenever the TI is bypassed, which is correct
      # because the stock LKAS path is then the only thing steering.
      new_actuators.steer = ti_apply_steer / self.ccp.TI_STEER_MAX
      new_actuators.steerOutputCan = ti_apply_steer
    else:
      new_actuators.steer = apply_steer / self.ccp.STEER_MAX
      new_actuators.steerOutputCan = apply_steer

    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR and self.frame % 100 == 0:
      self.publish_ti_stats()

    self.frame += 1
    Timer.tick()
    return new_actuators, can_sends
