import json
import time

from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.mazda import mazdacan
from openpilot.selfdrive.car.mazda.values import CarControllerParams, Buttons, MazdaFlags, TI_STATE, TI_LIMIT_BOUNDS
from openpilot.common.realtime import ControlsTimer as Timer, DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params

# Below this the bias-to-command ratio is dominated by sensor quantisation -- both torque sensors
# are 8-bit -- so accumulating it would only add noise to the slope estimate.
BIAS_MIN_COMMAND = 100

# Flags kept. Enough for a long tuning drive, small enough that the param stays a few kilobytes.
FLAG_HISTORY = 40
# ...and how long any one of them is kept. A flag exists to find a moment in a drive again, so
# it has to outlive the drive -- but not the tuning campaign. A week covers "look at Tuesday's
# flag at the weekend"; older ones are pruned when the next flag is written. The rlog still
# holds the moment itself; the flag is only the pointer to it.
FLAG_MAX_AGE_S = 7 * 24 * 3600

# Completed measurement runs retained. Enough to hold a session's worth of A/B steps.
RUN_HISTORY = 5

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
    self.ti_steer_threshold = None
    self.ti_route = ""
    self.ti_route_seen = ""
    self.ti_route_started = 0.0
    self.reset_ti_stats()

  def reset_ti_stats(self):
    self.ti_stats = {k: 0 for k in ("engaged", "short", "rate_limited", "rate_limited_low",
                                    "rate_limited_high", "driver_limited",
                                    "at_clip", "peak_cmd", "peak_desired", "deficit",
                                    "peak_bias", "not_run", "viol", "ramp",
                                    "bias_cmd_sum", "bias_sum", "bias_frames",
                                    "bias_ratio_min", "bias_ratio_max", "bias_wrong_sign")}
    self.ti_stats_started = time.time()

  def ti_config(self):
    """The limits these counters were produced under. Without it a saved run has no identity: both
    the previous/current comparison and analyze_segment otherwise assume the params as they are
    NOW, which is wrong from the moment you change one -- which is the entire point of the run."""
    cfg = {
      "TiSteerMax": self.ccp.TI_STEER_MAX,
      "TiSteerDeltaUp": self.ccp.TI_STEER_DELTA_UP,
      "TiSteerDeltaDown": self.ccp.TI_STEER_DELTA_DOWN,
      "TiSteerDriverAllowance": self.ccp.TI_STEER_DRIVER_ALLOWANCE,
      "TiSteerDriverMultiplier": self.ccp.TI_STEER_DRIVER_MULTIPLIER,
      "TiSteerDeltaUpKnee": self.ccp.TI_STEER_DELTA_UP_KNEE,
      "TiSteerDeltaUpHigh": self.ccp.TI_STEER_DELTA_UP_HIGH,
    }
    if self.ti_steer_threshold is not None:
      cfg["TiSteerThreshold"] = self.ti_steer_threshold
    return cfg

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

    s["peak_desired"] = max(s["peak_desired"], abs(desired))

    if abs(desired) - abs(sent) > 5:
      s["short"] += 1
      # How far behind, not just how often. A frame cut by 6 counts and a frame cut by 150 both
      # increment "short", but only the second one misses an apex. Summed over frames this is
      # torque-frames of missing command; divided by engaged it is the single number that should
      # go down when a tuning change actually helps.
      s["deficit"] += abs(desired) - abs(sent)
      # Test against the rate the limiter actually applied this frame. Above the knee the step is
      # DELTA_UP_HIGH, which is the smaller of the two, so testing against DELTA_UP would fail to
      # recognise a high-magnitude rate limit at all -- under-reporting precisely the regime the
      # knee exists to control, and making the knee look free.
      above_knee = abs(self.ti_apply_steer_last) >= self.ccp.TI_STEER_DELTA_UP_KNEE
      delta_up = self.ccp.TI_STEER_DELTA_UP_HIGH if above_knee else self.ccp.TI_STEER_DELTA_UP
      # Compare the signed step so a sign crossing still registers, and only call it rate limiting
      # when the command was climbing -- a command collapsing under driver torque moves at
      # DELTA_DOWN, which would otherwise be miscounted as a rate limit and point at the wrong knob.
      if abs(sent) > abs(self.ti_apply_steer_last) and \
         abs(sent - self.ti_apply_steer_last) >= delta_up:
        s["rate_limited"] += 1
        # Which side of the knee did the cutting. With a knee configured, this is what says whether
        # the remaining shortfall is in the range we chose to open up or the range we chose to keep
        # cautious -- and so whether the next move is the low rate, the high rate, or the knee.
        s["rate_limited_high" if above_knee else "rate_limited_low"] += 1
      # Only torque OPPOSING the command narrows the cap -- openpilot's driver-torque limit is
      # signed, and torque in the command's own direction widens the bound instead. Counting both
      # directions would blame the driver term for frames it had nothing to do with.
      if abs(CS.out.steeringTorque) > self.ccp.TI_STEER_DRIVER_ALLOWANCE and \
         (CS.out.steeringTorque * desired) < 0:
        s["driver_limited"] += 1
    if abs(sent) >= self.ccp.TI_STEER_MAX:
      s["at_clip"] += 1
    s["peak_cmd"] = max(s["peak_cmd"], abs(sent))

    # Closed-loop check on whether the command is reaching the car. Both torque sensors are
    # declared identically in the DBC -- 8 bits, offset -127, range [-85,85] -- so their difference
    # is the bias the interceptor is actually injecting, in the same units. Accumulating it against
    # the command gives the conversion slope for this run, live, without waiting for a log parse.
    #
    # This deliberately only counts. It does not touch the command, and there is no threshold that
    # makes it start doing so. The unreliability the guard would defend against has never been
    # reproduced on this unit, we have no measured slope to set a threshold from, and a guard that
    # pulls assist down mid-corner on a false positive is a worse failure than the one it watches
    # for. Establish that decoupling happens, and what it looks like, before giving it authority.
    # ti_response does the real characterisation offline; this is the cheap in-drive version.
    if abs(sent) >= BIAS_MIN_COMMAND:
      bias = int(CS.eps_torque_sensor - CS.out.steeringTorque)
      s["bias_cmd_sum"] += abs(sent)
      s["bias_sum"] += abs(bias)
      # Both tails, kept apart. The sums above fold them together, and they mean opposite things:
      # bias below the command asked for is assist fading, which the driver simply steers through;
      # bias above it is torque nobody requested. Only the second would ever justify letting this
      # touch the command, so it has to be visible separately. Ratio in thousandths to stay integer.
      ratio = int(1000 * abs(bias) / abs(sent))
      s["bias_ratio_max"] = max(s["bias_ratio_max"], ratio)
      s["bias_ratio_min"] = ratio if not s["bias_frames"] else min(s["bias_ratio_min"], ratio)
      # Bias opposing the command. Never expected this far above the noise floor.
      if bias * sent < 0:
        s["bias_wrong_sign"] += 1
      s["bias_frames"] += 1

  def load_param_list(self, key):
    """A JSON list from params, or an empty one. Anything unparseable is treated as empty rather
    than raising: this runs inside the 100Hz car controller, where an exception is a dead process
    and a lost drive, and a corrupt history is not worth that."""
    try:
      raw = self.params.get(key, encoding="utf8")
      value = json.loads(raw) if raw else []
      return value if isinstance(value, list) else []
    except (ValueError, TypeError):
      return []

  def check_flag_request(self, CS, CC, sent):
    """The driver tapped "Flag This Moment" on the tuning panel because something felt wrong.

    Records enough to find the spot again without trawling a whole drive: when, which route and
    segment, and what the interceptor was doing at that instant. Polled at 1Hz alongside the other
    trigger, so the recorded moment can trail the tap by up to a second -- immaterial, since the
    thing being marked is a stretch of road and the driver is reacting to something already a
    second or two old anyway."""
    if not self.params_memory.get_bool("TiFlagMoment"):
      return
    # tmpfs, and the clear is blocking there because tmpfs writes do not reach flash. On /data this
    # was Params::put -- temp-file fsync plus directory fsync, two ext4 journal commits -- executed
    # inside the 100Hz thread that builds the steering frame, at the exact moment the driver taps
    # the button mid-drive. That is the commIssue mechanism again, reduced to once per tap. A
    # trigger flag is ephemeral by nature and has no business on flash.
    self.params_memory.put_bool("TiFlagMoment", False)

    now = time.time()
    if self.ti_route and self.ti_route != self.ti_route_seen:
      self.ti_route_seen, self.ti_route_started = self.ti_route, now

    segment = None
    if self.ti_route and self.ti_route_started:
      # loggerd rotates a segment a minute, so elapsed time in the route gives the index. Slightly
      # approximate -- this process comes up a moment after loggerd opens the route -- so a flag
      # landing near a minute boundary may name the neighbouring segment. Both are worth reading.
      segment = f"{self.ti_route}--{int((now - self.ti_route_started) // 60)}"

    entry = {
      "at": round(now, 1),
      "route": self.ti_route or None,
      "segment": segment,
      "engaged": bool(CC.latActive),
      "speed_ms": round(float(CS.out.vEgo), 1),
      "steering_angle_deg": round(float(CS.out.steeringAngleDeg), 1),
      "command": int(sent),
      "driver_torque": int(CS.out.steeringTorque),
      "bias": int(CS.eps_torque_sensor - CS.out.steeringTorque),
      "ti_mode": int(CS.ti_state),
      "ti_viol": int(CS.ti_violation),
      "ti_ramp": bool(CS.ti_ramp_down),
      "config": self.ti_config(),
    }

    existing = self.load_param_list("TiFlaggedMoments")
    # Drop anything past its keep-time on the way through, so flags do not pile up across weeks
    # of drives. Done here rather than on a timer: the only moment this list is worth touching
    # from the control loop is when a tap is already writing it. Entries without a usable
    # timestamp are kept rather than guessed at.
    cutoff = now - FLAG_MAX_AGE_S
    existing = [f for f in existing
                if not isinstance(f, dict) or not isinstance(f.get("at"), (int, float))
                or f["at"] >= cutoff]
    existing.append(entry)
    # Persisted to flash rather than tmpfs: the entire point is to still have these after the
    # drive. One write per tap is nothing, unlike the per-second counters.
    self.params.put_nonblocking("TiFlaggedMoments", json.dumps(existing[-FLAG_HISTORY:]))

  def maybe_refresh_route(self):
    """Read CurrentRoute once, not every second.

    loggerd only rewrites it at an onroad transition, and this process is restarted at every
    ignition cycle -- so one read per process lifetime is all the information there is. Doing it
    at 1Hz put a /data read inside the 100Hz realtime thread for a string that never changes."""
    if not self.ti_route:
      self.refresh_route()

  def refresh_route(self):
    """Identity for everything recorded this second. loggerd writes CurrentRoute at every onroad
    transition, so stamping it is what ties a saved run, or a flagged moment, back to the segments
    that produced it -- otherwise they can only be matched by guessing at wall-clock.

    encoding matters: Params.get returns bytes without it, and bytes are not JSON serialisable, so
    the dumps downstream would raise every second and take the car controller down with it."""
    self.ti_route = self.params.get("CurrentRoute", encoding="utf8") or self.ti_route

  def ti_payload(self):
    return json.dumps({**self.ti_stats, "live": self.ti_live, "config": self.ti_config(),
                       "started_at": self.ti_stats_started, "route": self.ti_route})

  def check_clear_request(self):
    """The driver tapped "Start A New Measurement". Bank the run as it stands -- counters and
    flagged moments alike -- and start the next one clean.

    Polled at 1Hz next to the flag check, not inside the 0.2Hz publish. When it lived in the
    publish it was consumed at most once every five seconds, so a second tap inside that window
    found the flag already set and did nothing, and two runs collapsed into one -- the button
    only worked once per drive in practice. A tmpfs read costs microseconds; a tap now lands
    within a second and banks exactly the run that was showing when it was pressed."""
    # Clearing keeps the outgoing run under a second key so the two can be compared side by side.
    # Same reasoning as the flag trigger: ephemeral, so tmpfs, so no journal commit in the control
    # loop. This one predates the flag work and had the same defect.
    if self.params_memory.get_bool("ClearTiStats"):
      self.params_memory.put_bool("ClearTiStats", False)
      payload = self.ti_payload()
      # Bank this session's counters when it has any, and the persisted ones otherwise. The process
      # restarts every ignition cycle, so clearing while parked would otherwise stash a set of
      # zeros as "previous" and lose the run the user actually meant to keep.
      # tmpfs first. The flash copy is only written once a minute, so closing a run while parked
      # would bank a snapshot up to 59 seconds behind -- and for an A/B that ends at the corner you
      # cared about, the missing minute is the part you wanted. The tmpfs copy is a second old and
      # survives ignition-off (the device stays powered); it is only lost at reboot, where the
      # flash copy is the correct fallback.
      closing = payload if self.ti_stats["engaged"] else \
                (self.params_memory.get("TiTuningStats", encoding="utf8") or
                 self.params.get("TiTuningStats", encoding="utf8") or payload)
      # The moments the driver flagged belong to the run they were taken in, so they close with it
      # and the new measurement starts with none. Otherwise the list is a week of drives deep and
      # "the markers from this run" means matching them up by route and wall clock afterwards.
      # They ride on the run payload rather than in a key of their own: a new params key also has
      # to be declared in common/params.cc, and that mistake does not fail loudly, it stops the
      # device booting (RUNBOOK section 2). Banked, not dropped -- same reason the counters are.
      flagged = self.load_param_list("TiFlaggedMoments")
      if flagged:
        try:
          closing = json.dumps({**json.loads(closing), "flags": flagged})
        except (ValueError, TypeError):
          pass          # a payload we cannot parse still banks, just without the flags attached
      self.params.put_nonblocking("TiTuningStatsPrevious", closing)
      # Keep a short history as well as the single previous run. Two slots meant one stray tap on
      # "Start A New Measurement" destroyed the baseline you were comparing against, which is easy
      # to do mid-drive and impossible to undo. Now that every run carries the limits it was taken
      # under, a handful of them is a usable record of a tuning session rather than just a pair.
      try:
        history = self.load_param_list("TiTuningStatsHistory")
        history.append(json.loads(closing))
        self.params.put_nonblocking("TiTuningStatsHistory",
                                    json.dumps(history[-RUN_HISTORY:]))
      except (ValueError, TypeError):
        pass
      if flagged:
        self.params.put_nonblocking("TiFlaggedMoments", "[]")
      self.reset_ti_stats()
      cleared = self.ti_payload()
      self.params_memory.put("TiTuningStats", cleared)
      self.params.put_nonblocking("TiTuningStats", cleared)

  def publish_ti_stats(self):
    payload = self.ti_payload()

    # Live copy on tmpfs, written BLOCKING on purpose. put_nonblocking hands the write to a
    # background thread, and Params spawns that thread through std::async on nearly every call --
    # which is the expensive part here, not the write. A tmpfs write is a memcpy and its fsync is a
    # no-op, so doing it inline is cheaper than creating a thread to avoid it. put_nonblocking is
    # still right for the flash copy below, where the write genuinely is slow.
    self.params_memory.put("TiTuningStats", payload)

    # Flash copy once a minute, not once a second. Params::put fsyncs the temp file and then the
    # params directory, and each of those is an ext4 journal commit the whole filesystem queues
    # behind -- including loggerd streaming camera video to the same eMMC. At 1Hz this put ~60 such
    # barriers a minute underneath the camera pipeline. openpilot persists its own derived state at
    # a minute (LiveTorqueParameters); match that. A reboot now costs at most a minute of counters,
    # and anything watching live should read the tmpfs copy above.
    if self.frame % 6000 == 0:
      self.params.put_nonblocking("TiTuningStats", payload)

  def _ti_limit(self, frogpilot_toggles, attr, toggle_name):
    """One live limit, clamped. Deliberately repeats the clip in frogpilot_variables rather than
    trusting it: the panda applies no steering checks at all to MAZDA_TI_LKAS on gen1, so this
    process is the last thing between a bad number and the CAN bus. A stale params file, a
    half-written value or a bug upstream has nothing downstream to catch it."""
    value = getattr(frogpilot_toggles, toggle_name, None)
    if value is None:
      return getattr(self.ccp, attr)
    lo, hi = TI_LIMIT_BOUNDS[attr]
    try:
      return int(min(max(value, lo), hi))
    except (TypeError, ValueError):
      return getattr(self.ccp, attr)

  def apply_ti_tuning(self, frogpilot_toggles):
    # The TI limits live on self.ccp, which the rate/driver-torque limiters read every frame, so
    # writing them here takes effect immediately. Falls back to the compiled-in value whenever a
    # toggle is absent, which keeps older FrogPilot params files working.
    self.ccp.TI_STEER_MAX = self._ti_limit(frogpilot_toggles, "TI_STEER_MAX", "ti_steer_max")
    self.ccp.TI_STEER_DELTA_UP = self._ti_limit(frogpilot_toggles, "TI_STEER_DELTA_UP", "ti_steer_delta_up")
    self.ccp.TI_STEER_DELTA_DOWN = self._ti_limit(frogpilot_toggles, "TI_STEER_DELTA_DOWN", "ti_steer_delta_down")
    self.ccp.TI_STEER_DRIVER_ALLOWANCE = self._ti_limit(frogpilot_toggles, "TI_STEER_DRIVER_ALLOWANCE", "ti_steer_driver_allowance")
    self.ccp.TI_STEER_DRIVER_MULTIPLIER = self._ti_limit(frogpilot_toggles, "TI_STEER_DRIVER_MULTIPLIER", "ti_steer_driver_multiplier")
    self.ccp.TI_STEER_DELTA_UP_KNEE = self._ti_limit(frogpilot_toggles, "TI_STEER_DELTA_UP_KNEE", "ti_steer_delta_up_knee")
    # The high-magnitude rate is the REDUCED one by construction. Letting it exceed the base rate
    # would inverse the whole point -- fast where the unit is suspect, slow where it is proven --
    # so it is held at or below it rather than trusted to be set sensibly.
    self.ccp.TI_STEER_DELTA_UP_HIGH = min(
      self._ti_limit(frogpilot_toggles, "TI_STEER_DELTA_UP_HIGH", "ti_steer_delta_up_high"),
      self.ccp.TI_STEER_DELTA_UP)
    # Not a ccp field -- carstate applies it to steeringPressed -- but it belongs in the run's
    # config stamp, because it changes when openpilot decides the driver is overriding and so
    # changes what the driver_limited counter means.
    self.ti_steer_threshold = getattr(frogpilot_toggles, "ti_steer_threshold", None)

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

    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      # Trigger polls stay at 1Hz -- they are tmpfs reads costing microseconds, and the flag button
      # needs to feel responsive.
      if self.frame % 100 == 0:
        self.maybe_refresh_route()
        self.check_flag_request(CS, CC, ti_apply_steer)
        self.check_clear_request()

      # Publishing moved from 1Hz to 0.2Hz. Params.put_nonblocking spawns a fresh thread through
      # std::async on nearly every call, and this is a SCHED_FIFO process under mlockall where a
      # new thread's stack has to be allocated and faulted in. At 1Hz that cost landed in the
      # 100Hz thread that builds the steering frame once a second, on a fixed phase -- visible in
      # the logs as a skipped carState frame at exactly 1Hz, which then made the 20Hz consumers
      # polling carState fail their all_checks and flag their own output invalid, which controlsd
      # reports as commIssue. Five seconds of staleness costs the counters nothing.
      if self.frame % 500 == 0:
        self.publish_ti_stats()

    self.frame += 1
    Timer.tick()
    return new_actuators, can_sends
