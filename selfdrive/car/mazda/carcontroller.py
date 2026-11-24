import json
import time

from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.mazda import mazdacan
from openpilot.selfdrive.car.mazda.values import CarControllerParams, Buttons, MazdaFlags, TI_STATE, TI_LIMIT_BOUNDS
from openpilot.selfdrive.car.mazda.ti_dither import ACTIVE_PRESET as TI_DITHER_PRESET, TiDither, gate_reason, rate_room
from openpilot.selfdrive.car.mazda.ti_stuck_guard import StuckGuard
try:
  # On-car tests: optional. If the module cannot be imported, the tests are off and this controller runs exactly as
  # without them (self.ti_tests is None).
  from openpilot.selfdrive.car.mazda.ti_tests import make_accountant
except Exception:
  make_accountant = None
try:
  # Steering identification tests (sweep, ladder, pulses): optional in the same way. Without the module they are off
  # and this controller runs exactly as without them (self.ti_ident is None).
  from openpilot.selfdrive.car.mazda import ti_ident as ti_ident_mod
except Exception:
  ti_ident_mod = None
try:
  # Drive plan (ti_plan.py): sequences the A/B alternation and the tests. Optional: without it nothing here changes.
  from openpilot.selfdrive.car.mazda import ti_plan as ti_plan_mod
except Exception:
  ti_plan_mod = None
try:
  # Lag guard (ti_lag.py): the tests stand down by themselves while the control loop is short of time. Optional like the rest.
  from openpilot.selfdrive.car.mazda import ti_lag as ti_lag_mod
except Exception:
  ti_lag_mod = None
try:
  # Cruise set-speed check (ti_cruise_check.py): started only by its own control on the Tests pages. Optional: without the module
  # nothing here changes (self.cruise_check is None).
  from openpilot.selfdrive.car.mazda import ti_cruise_check as cruise_check_mod
except Exception:
  cruise_check_mod = None
try:
  # Curve slowing (ti_curve.py): lowers the cruise set speed before remembered corners; switched on only by its own control on the
  # Tests pages, for the drive. Optional: without the module nothing here changes (self.curve is None).
  from openpilot.selfdrive.car.mazda import ti_curve as curve_mod
  from openpilot.selfdrive.car.mazda import ti_curve_monitor as curve_mon_mod, ti_curve_demand as curve_demand_mod   # stage 3
except Exception:
  curve_mod = curve_mon_mod = curve_demand_mod = None
from openpilot.common.realtime import ControlsTimer as Timer, DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV

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

# What the dither reports to the test accountant on a frame it was applied (ti_tests.ACTIVE_SOURCES).
TI_DITHER_ACTIVE = ("TiDither",)


def ti_hold_log(event):
  """Why the car controller holds the tests (plan stale, dead, alternation not at A, ...): on every change, in the `ti_ab` channel."""
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event("ti_ab", **event)
  except Exception:
    pass


def ti_ident_log(event):
  """The identification tests' events (run, chunk, stop, credit, pick) into the log, so the signal can be rebuilt exactly
  (ti_ident.reconstruct) and every run-all choice replayed. A run, chunk or stop per unit or pause (seconds apart); the
  speed hysteresis (ti_ident.ZONE_HYSTERESIS_MPH) keeps a car hovering at a boundary from making them in bursts, and
  pick events are limited to one per ti_ident.PICK_LOG_MIN_S."""
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event("ti_ident", **event)
  except Exception:
    pass

# Stuck-request guard events: at most one per this many frames (10 s), each carrying the running burst count.
TI_STUCK_LOG_EVERY_FRAMES = 1000


def ti_stuck_log(event):
  """A stuck-request guard nudge (ti_stuck_guard.py) into the log, rate limited by the caller: {kind, frame, value, sent, bursts}."""
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event("ti_stuck_guard", **event)
  except Exception:
    pass


def ti_curve_log(event):
  """Curve slowing's steps (start, tap with its frames, step read back, stop with why, restored, needs_more, state) into the log."""
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event("ti_curve", **event)
  except Exception:
    pass


def _finite_or_none(x):
  """A float that is finite, else None (curve slowing's car-side inputs never carry a NaN or a non-number)."""
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  return x if x == x and abs(x) != float("inf") else None


def ti_cruise_log(event):
  """The cruise set-speed check's steps (armed, waiting, tap with its frames and counters, accepted, aborted, result) into the log."""
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event("ti_cruise", **event)
  except Exception:
    pass


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
    # CP.flags tests cached once (an IntFlag `&` costs several Python calls; update asked about a dozen a cycle)
    self.flag_ti = bool(CP.flags & MazdaFlags.TORQUE_INTERCEPTOR)
    self.flag_gen1 = bool(CP.flags & MazdaFlags.GEN1)
    self.flag_gen2 = bool(CP.flags & MazdaFlags.GEN2)
    self.flag_no_fsc = bool(CP.flags & MazdaFlags.NO_FSC)
    self.flag_radar = bool(CP.flags & MazdaFlags.RADAR_INTERCEPTOR)
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
    # Identification dither (ti_dither.py). Off unless the TiDither toggle is on; read every frame from the live
    # toggles, so switching it off fades the dither out within FADE_OUT_S of the toggles reaching card.
    self.ti_dither = TiDither()
    self.ti_dither_enabled = False
    self.ti_reported_last = 0
    # Stuck-request guard (ti_stuck_guard.py): on the final TI command, just before it is packed. Interceptor cars only.
    self.ti_stuck_guard = StuckGuard() if self.flag_ti else None
    self.ti_stuck_logged = None        # frame of the last ti_stuck_guard event (rate limit)
    # On-car tests (ti_tests.py, definitions in ti_tests.json): counts each enabled test's metric for the Tests
    # screen. Outside the command path: it only reads what this controller already computed. None when the file
    # holds no valid test or the module is missing, and on every car without the interceptor.
    self.ti_tests = (make_accountant(self.params, self.params_memory)
                     if make_accountant is not None and CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else None)
    # Identification tests (ti_ident.py): off unless exactly one of TiSweep / TiLadder / TiPulses is on, the dither is
    # off and silent, and the tests accountant is loaded (it says which steps are still to capture). None on every
    # car without the interceptor or without the module.
    self.ti_ident = (ti_ident_mod.TiIdent(log=ti_ident_log)
                     if ti_ident_mod is not None and CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else None)
    self.ti_ident_on = ()
    self.ti_ident_armed = None
    self.ti_ident_all = False          # run all (TiIdentAll): the scheduler picks the test, window by window
    # The run-all scheduler: seeded, every pick logged as a "pick" ti_ident event (ti_ident.RunAll, verify_picks)
    self.ti_run_all = (ti_ident_mod.RunAll(emit=self.ti_ident.event_for_pick)
                       if self.ti_ident is not None else None)
    self.ti_plan_on = False            # the drive plan switch (TiPlanOn); with it on the plan's word decides whether the tests may run
    self.ti_plan_allowed = False
    self.ti_plan_seq = None
    self.ti_plan_nonce = None
    self.ti_plan_ack_key = None
    self.ti_hold_key = None
    self.ti_hold_published = None
    self.ti_ab_on = False              # the A/B alternation switch (TiAbOn, tmpfs, read at 10 Hz): while on, no test runs
    self.ti_gate_published = None      # the last gate block written to TiTestGate (screen: reason, value, limit)
    self.ti_gate_frame = 0
    self.ti_ab_switch = False          # TiAbOn as read (the plain A/B switch), for the cruise check's exclusion
    self.ti_lag = (ti_lag_mod.LagGuard(log=ti_hold_log)
                   if ti_lag_mod is not None and CP.flags & MazdaFlags.TORQUE_INTERCEPTOR else None)
    self.ti_lag_held = False           # the loop has been short of time: the tests are held (reason "control loop busy")
    self.ti_lag_ctl = None             # (log time s, cumLagMs) of the latest controlsState, set by card before each update
    # Cruise set-speed check: gen1 with the interceptor only (the car it was studied on). Off after every start of this process: the
    # trigger and the last status are cleared here, and a trigger stamped before this moment is ignored.
    self.cruise_check = (cruise_check_mod.CruiseCheck()
                         if cruise_check_mod is not None and CP.flags & MazdaFlags.TORQUE_INTERCEPTOR and CP.flags & MazdaFlags.GEN1 else None)
    self.cruise_started_wall = time.time()
    self.cruise_trigger = None
    self.cruise_published = None
    # Curve slowing: gen1 with the interceptor only. OFF after every start of this process: the switch and the status are cleared here.
    self.curve = (curve_mod.Curve()
                  if curve_mod is not None and CP.flags & MazdaFlags.TORQUE_INTERCEPTOR and CP.flags & MazdaFlags.GEN1 else None)
    self.curve_on = False
    self.curve_plan = None
    self.curve_plan_age = None
    self.curve_plan_read = None
    self.curve_published = None
    # review C2: the safe exit lives OUTSIDE the Curve (an internal error there must not take it down), with the minimal state it needs
    self.curve_exit = curve_mod.SafeExit() if self.curve is not None else None
    self.curve_inp_last = None
    if self.curve is not None:
      # the previous run of this process may have left the set speed lowered (a restart mid-intervention): say so on the page
      try:
        old = json.loads(self.params_memory.get(curve_mod.PARAM_STATUS) or b"{}")
        if old.get("state") in ("slowing", "holding", "restoring") or (old.get("state") == "stopped" and
                                                                       (old.get("set") or 0) < (old.get("ref") or 0)):
          self.curve.restart_note = "Restarted: set speed may be lowered: check it"
        mk = old.get("exit")
        if isinstance(mk, dict):
          # pushed below the line when the previous run ended: the safe exit raises the dash back to it (review C2)
          ref_, fl_ = mk.get("ref"), mk.get("floor")
          self.curve_exit.arm(0, None, "controls restarted", ref=ref_ if isinstance(ref_, int) else None,
                              floor=fl_ if isinstance(fl_, int) else None, restart=True)
          self.curve.restart_note = ("Restarted: raising the set speed back" if isinstance(ref_, int) else
                                     "Restarted: set speed unknown: raising to the car's speed")
      except Exception:
        pass
      for key in (curve_mod.PARAM_ON, curve_mod.PARAM_STATUS):
        try:
          self.params_memory.remove(key)
        except Exception:
          pass
    if self.cruise_check is not None:
      for key in (cruise_check_mod.PARAM_TRIGGER, cruise_check_mod.PARAM_STATUS):
        try:
          self.params_memory.remove(key)
        except Exception:
          pass
    self.reset_ti_stats()

  def reset_ti_stats(self):
    self.reset_ti_counters()
    # Frames of this measurement run on which the dither was non-zero. Only stamped into the run's config when the
    # toggle was on at some point, so a run without the dither carries exactly the payload it always did.
    self.ti_dither_seen = False
    self.ti_dither_frames = 0
    # Identification tests switched on during this run, and their frames with a non-zero signal (stats stamp)
    self.ti_ident_seen = set()
    self.ti_ident_frames = {}
    self.ti_ident_all_seen = False
    ident = getattr(self, "ti_ident", None)
    if ident is not None:
      ident.counts = {}
    tests = getattr(self, "ti_tests", None)
    if tests is not None:
      tests.run_seen = set()

  def reset_ti_counters(self):
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
    # Identification dither (ti_dither.py): present only for a run during which the TiDither toggle was on, so the
    # run is recognisable as dithered and kept out of A/B comparisons; absent otherwise (payload unchanged).
    if getattr(self, "ti_dither_seen", False):
      cfg["TiDither"] = {"preset": TI_DITHER_PRESET, "frames": self.ti_dither_frames,
                         "touch_confirm": self.ti_dither.touch_confirm_frames}
    # On-car tests (ti_tests.json) that were on at any point of this run, by id; absent otherwise (payload unchanged).
    tests = getattr(self, "ti_tests", None)
    if tests is not None and tests.run_seen:
      cfg["Tests"] = sorted(tests.run_seen)
    # Identification tests (ti_ident.py) that were on during this run; absent otherwise (payload unchanged). With the
    # logged "ti_ident" events this is what an offline reader needs to rebuild the signal.
    seen = getattr(self, "ti_ident_seen", None)
    if seen and ti_ident_mod is not None:
      ident = getattr(self, "ti_ident", None)
      cfg["TiIdent"] = {k: {"version": ti_ident_mod.VERSION, "table": ti_ident_mod.TABLE_HASH[k],
                            "frames": self.ti_ident_frames.get(k, 0),
                            "frames_by": ident.frames_by(k) if ident is not None else {},
                            "touch_confirm": ident.touch_confirm_frames if ident is not None else None}
                        for k in sorted(seen)}
      cfg["TiIdent"]["run_all"] = bool(getattr(self, "ti_ident_all_seen", False))
    return cfg

  def ti_ident_failed(self):
    """The identification tests raised: off for the rest of this process (the drive), logged once."""
    self.ti_ident = None
    self.ti_ident_armed = None
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception("ti_ident disabled")
    except Exception:
      pass

  def publish_ti_tests(self, CS, CC, ti_request):
    """Test progress, at the stats cadence. `ti_request` is this frame's undithered TI request. The dither's gate
    reason is display only (ti_dither.gate_reason) and computed only while a test is on."""
    tests = getattr(self, "ti_tests", None)
    if tests is None:
      return
    reasons = {}
    if tests.wants_reasons():
      try:
        reasons["TiDither"] = gate_reason(
          engaged=bool(CC.latActive and CS.ti_lkas_allowed), enabled=self.ti_dither_enabled, v_ego=CS.out.vEgo,
          steering_pressed=CS.out.steeringPressed, driver_torque=CS.out.steeringTorque,
          blinker=bool(CS.out.leftBlinker or CS.out.rightBlinker or CC.leftBlinker or CC.rightBlinker),
          base_request=ti_request, ti_max=self.ccp.TI_STEER_MAX,
          lat_demand=CS.out.vEgo ** 2 * CC.actuators.curvature, holdoff=self.ti_dither.holdoff,
          room=rate_room(self.ccp), touch=self.ti_dither.touch)
      except Exception:
        pass
    ident = getattr(self, "ti_ident", None)
    if ident is not None and tests.wants_reasons():
      notes, run_all = {}, None
      keys = ti_ident_mod.KEYS if self.ti_ident_all else getattr(self, "ti_ident_on", ())
      for key in keys:
        reasons[key] = ident.reason if key == self.ti_ident_armed else "other"
        share = ident.share(key)
        if share is not None:
          notes[key] = f" · {int(round(100 * share))}% captured"
      if self.ti_ident_all:
        run_all = (self.ti_ident_armed, ident.reason)
      tests.publish(time.monotonic(), reasons, notes=notes, run_all=run_all)
      return
    tests.publish(time.monotonic(), reasons)

  def publish_plan_ack(self):
    """Tell the drive plan whether any test signal is still in progress (ti_plan.make_ack); tmpfs, on change and every 0.5 s."""
    try:
      ident, dither = getattr(self, "ti_ident", None), getattr(self, "ti_dither", None)
      ack = ti_plan_mod.make_ack(self.ti_plan_nonce, self.ti_plan_seq, bool(ident is not None and ident.running), 0 if ident is None else ident.last,
                                 0.0 if dither is None else dither.envelope, time.time())
      key = (ack["nonce"], ack["seq"], ack["quiet"])
      if key != self.ti_plan_ack_key or self.frame % 50 == 0:
        self.params_memory.put("TiPlanAck", json.dumps(ack))
        self.ti_plan_ack_key = key
    except Exception:
      pass

  def publish_ti_gate(self):
    """What closes the armed identification test's gate right now, once a second, for the Tests screen: {t, test, reason,
    what, value, limit, unit, window} while a limit (command, curve, lateral demand) blocks it, else nothing (the
    screen reads an empty or stale block as no limit). tmpfs only; written when it changes, never raises."""
    ident = getattr(self, "ti_ident", None)
    if ident is None or self.ti_tests is None:
      return
    try:
      last = ident.gate_last     # the latest closure within the last second, so a value hovering at its limit shows
      gate = last[1] if (last is not None and ident.n - last[0] <= 100 and self.ti_ident_armed is not None) else None
      if gate is None and self.ti_gate_published is None:
        return
      block = None if gate is None else {**gate, "test": self.ti_ident_armed, "t": round(time.time(), 1)}
      self.params_memory.put("TiTestGate", json.dumps(block if block is not None else {"t": round(time.time(), 1)}))
      self.ti_gate_published = block
    except Exception:
      pass

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
    # Re-read only when the toggles object is a new one (controlsd / card replace it on every reload; it is never changed in place):
    # the limits are a pure function of it and of the values set here last time (cost: seven clipped reads every cycle).
    if frogpilot_toggles is not getattr(self, "ti_tuned_for", None):
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
      # Absent (older toggles JSON) reads as off.
      self.ti_dither_enabled = bool(getattr(frogpilot_toggles, "ti_dither", False))
      self.ti_tuned_for = frogpilot_toggles
    if self.frame % 10 == 0:
      # A/B alternation (ti_ab.py) and the tests exclude each other: the switch is tmpfs, off at every boot. An unreadable
      # switch keeps the last value read; the A/B side independently refuses to run while any test toggle is on.
      try:
        ab_on = bool(self.params_memory.get_bool("TiAbOn"))
        self.ti_ab_switch = ab_on
        if ti_plan_mod is not None:
          plan_on = bool(self.params_memory.get_bool("TiPlanOn"))
          raw_plan = self.params_memory.get("TiPlan")
          hold, allowed, why = ti_plan_mod.tests_gate(plan_on, raw_plan, time.time())
          self.ti_plan_on, self.ti_plan_allowed = plan_on, allowed
          self.ti_plan_nonce, self.ti_plan_seq = ti_plan_mod.plan_ident(raw_plan)
          if not hold and not plan_on and ab_on:
            why = "A/B switch on"
          ab_on = hold if plan_on else (hold or ab_on)
          if getattr(self, "ti_lag_held", False):
            ab_on, why = True, ti_lag_mod.WHY      # the loop is short of time: no test or dither signal until it has recovered
          key = (bool(ab_on), why)
          if key != self.ti_hold_key:                 # every change of the hold decision, and why, into the log
            self.ti_hold_key = key
            ti_hold_log({"kind": "tests_hold", "frame": self.frame, "held": bool(ab_on), "why": why, "plan_on": plan_on,
                         "nonce": self.ti_plan_nonce, "seq": self.ti_plan_seq})
          if key != self.ti_hold_published or self.frame % 100 == 0:   # the same reason for the screens, on change and every second
            try:
              self.params_memory.put("TiTestHold", json.dumps({"held": bool(ab_on), "why": why, "t": round(time.time(), 1)}))
              self.ti_hold_published = key
            except Exception:
              pass
        self.ti_ab_on = ab_on or bool(getattr(self, "ti_lag_held", False))   # held by the lag guard whatever else (also without ti_plan)
      except Exception:
        if self.ti_plan_on:                  # an unreadable plan allows no test
          self.ti_ab_on, self.ti_plan_allowed = True, False
    if ti_ident_mod is not None:
      self.ti_ident_on = tuple(k for k in ti_ident_mod.KEYS
                               if getattr(frogpilot_toggles, ti_ident_mod.TOGGLES[k], False) is True)
      self.ti_ident_all = getattr(frogpilot_toggles, ti_ident_mod.RUN_ALL_TOGGLE, False) is True
      if self.ti_plan_allowed:
        self.ti_ident_all, self.ti_ident_on = True, ()    # the plan runs the tests in run-all mode, under all their own gates

  def cruise_others_on(self):
    """Why another mode excludes the cruise check ("" when none is on)."""
    if self.ti_plan_on:
      return "the drive plan is on"
    if self.ti_ab_switch:
      return "the A/B switch is on"
    if self.ti_ident_all or self.ti_ident_on:
      return "a test is on"
    if self.ti_dither_enabled:
      return "the dither test is on"
    if getattr(self, "curve_on", False):
      return "curve slowing is on"
    return ""

  def cruise_check_step(self, CC, CS, ti_apply_steer):
    """One frame of the cruise set-speed check: the button frame to send, or None. Reads the trigger at 10 Hz (and consumes it),
    publishes the status on change and every second, logs every step. Never raises: after an error the check is off for the rest of
    this process and says so."""
    chk = self.cruise_check
    if chk is None:
      return None
    if not chk.active and self.frame % 10 != 0:
      return None                      # nothing to do between the 10 Hz trigger reads while no check runs (cost: every frame)
    try:
      mod = cruise_check_mod
      wall = time.time()
      trigger = None
      if self.frame % 10 == 0:
        raw = self.params_memory.get(mod.PARAM_TRIGGER)
        if raw:
          self.params_memory.remove(mod.PARAM_TRIGGER)        # one press arms one run: the trigger is consumed
          trigger = mod.parse_trigger(raw)
          if trigger is not None and trigger.t and trigger.t < self.cruise_started_wall:
            ti_cruise_log({"kind": "trigger_before_start", "frame": self.frame, "id": trigger.id})
            trigger = None                                     # written before this process started: never acted on
      out = CS.out
      cruise = getattr(out, "cruiseState", None)
      speed = float(getattr(cruise, "speed", 0.0) or 0.0)
      set_mph = int(round(speed * CV.MS_TO_MPH)) if speed > 0.5 else None
      counter = getattr(CS, "crz_btns_counter", None)
      inp = mod.Inputs(
        engaged=bool(getattr(CC, "enabled", False)), cruise_on=bool(getattr(cruise, "enabled", False)),
        cruise_available=bool(getattr(cruise, "available", False)), mph=float(out.vEgo) * CV.MS_TO_MPH, set_mph=set_mph,
        brake=bool(out.brakePressed), gas=bool(out.gasPressed), lead=getattr(CS, "crz_lead", None),
        steer_share=abs(ti_apply_steer) / max(1.0, float(self.ccp.TI_STEER_MAX)), driver_button=getattr(CS, "crz_btns_driver", None),
        counter=None if counter is None else int(counter), cancel_req=bool(CC.cruiseControl.cancel), resume_req=bool(CC.cruiseControl.resume),
        others_on=self.cruise_others_on(), wall=wall, mono=time.monotonic())
      button = chk.step(self.frame, inp, trigger)
      for ev in chk.events:
        ti_cruise_log(ev)
      chk.events.clear()
      st = chk.status(wall)
      key = (st["state"], st["why"], st["phase"], st["round"], len(st["a"]), len(st["b"]), st["set"], st["restore"], st["id"])
      # on change at most every 0.1 s (a flickering condition cannot write at 100 Hz), and once a second
      if (key != self.cruise_published and self.frame % 10 == 0) or self.frame % mod.STATUS_EVERY == 0:
        self.params_memory.put(mod.PARAM_STATUS, json.dumps(st))
        self.cruise_published = key
      if button is None or inp.cancel_req or inp.resume_req:
        return None
      return mazdacan.create_button_cmd(self.packer, self.CP, inp.counter, Buttons.SET_MINUS if button == "SET_MINUS" else Buttons.SET_PLUS)
    except Exception:
      self.cruise_check = None
      try:
        self.params_memory.put(cruise_check_mod.PARAM_STATUS, json.dumps({"t": round(time.time(), 1), "v": 1, "state": "aborted",
                                                                          "why": "internal error: check off until restart"}))
      except Exception:
        pass
      try:
        from openpilot.common.swaglog import cloudlog
        cloudlog.exception("ti_cruise disabled")
      except Exception:
        pass
      return None

  def _curve_monitor(self, CC, CS, ti_apply_steer):
    """Curve slowing stage 3 (ti_curve_monitor, plan T7): every frame, on the SENT interceptor command. While curve slowing is on,
    writes TiCurveDeliver {"m", "need", "why"} at the alert's onset, every 0.2 s while it lasts, and at its clear. -> MonOut."""
    TM = curve_mon_mod
    mon = getattr(self, "curve_mon", None)
    if mon is None:
      mon = self.curve_mon = TM.DeliveryMonitor()
      self.curve_deliver, self.curve_need_key, self.curve_need_val = None, None, None
    out = CS.out
    v = float(out.vEgo)
    plan = self.curve_plan or {}
    ahead, ask = plan.get("ahead"), plan.get("ask")
    if ahead and v > 1.0:
      key = (ahead, int(v * CV.MS_TO_MPH))
      if key != self.curve_need_key:      # at most when the plan (5 Hz) or the whole mph changes
        self.curve_need_key, self.curve_need_val = key, int(curve_demand_mod.need_mph(float(ahead) / (v * v))[0])
    else:
      self.curve_need_key = self.curve_need_val = None
    r = mon.step(self.frame, TM.MonInputs(
      mph=v * CV.MS_TO_MPH, ti_cmd=float(ti_apply_steer), ti_max=float(self.ccp.TI_STEER_MAX), lat_active=bool(CC.latActive),
      ti_active=bool(getattr(CS, "ti_lkas_allowed", False)), steering_pressed=bool(out.steeringPressed),
      angle_rate_deg=float(getattr(out, "steeringRateDeg", 0.0) or 0.0), la_now=getattr(CS, "lat_accel", None),
      desired_la=-v * v * float(CC.actuators.curvature),    # openpilot's curvature is RIGHT positive (ti_curve_plan.MODEL_K_SIGN)
      ahead_la=ahead, ahead_need=self.curve_need_val, lkas_blocked=getattr(CS, "lkas_blocked", None),
      lkas_effective=getattr(CS, "lkas_effective", None), ti_state=int(CS.ti_state) if hasattr(CS, "ti_state") else None,
      ti_viol=int(getattr(CS, "ti_violation", 0) or 0), ahead_prof=(TM.ASK_LEADS, ask) if ask else None,
      floor_mph=getattr(self.curve, "floor_now", None) if self.curve is not None else None))
    for ev in mon.events:
      ti_curve_log(ev)
    mon.events.clear()
    alerting = r.phase == "alert" and self.curve_on
    if alerting and (self.curve_deliver is None or self.frame % 20 == 0):
      self.curve_deliver = {"m": round(time.monotonic(), 2), "need": r.need_mph, "why": r.why}
      self.params_memory.put(curve_mod.PARAM_DELIVER, json.dumps(self.curve_deliver))
    elif not alerting and self.curve_deliver is not None:
      self.curve_deliver = None
      self.params_memory.put(curve_mod.PARAM_DELIVER, json.dumps({"m": round(time.monotonic(), 2), "need": None, "why": ""}))
    return r

  def curve_step(self, CC, CS, ti_apply_steer=0):
    """One frame of curve slowing: the button frame to send, or None. Reads the switch and the planner's output at 10 Hz (frames
    ending in 5, apart from the other 10 Hz reads), publishes the status on change (at most 10 Hz) and every second, logs every step.
    Runs stage 3's monitor every frame on the sent command (ti_apply_steer). While the switch is off and nothing is in progress it
    does only that and the 10 Hz read. Never raises: after an error curve slowing is off for the rest of this process and says so."""
    cv = self.curve
    ex = getattr(self, "curve_exit", None)
    if cv is None:
      if ex is not None and (ex.active or getattr(self, "curve_left", None) is not None):
        return self._curve_exit_only(CC, CS)   # the exit, then the set-speed-left alert until the driver acts (review I4)
      return None
    mon = None
    if getattr(self, "curve_mon_ok", True) and (self.curve_on or cv.state != "off"):   # OFF: no stage-3 work, no events (review)
      try:
        mon = self._curve_monitor(CC, CS, ti_apply_steer)
      except Exception:
        self.curve_mon_ok, mon = False, None    # stage 3 off for the rest of this process; the rest of curve slowing is unchanged
        try:
          from openpilot.common.swaglog import cloudlog
          cloudlog.exception("ti_curve monitor disabled")
        except Exception:
          pass
    read = self.frame % 10 == 5
    exiting = ex is not None and ex.active
    if not read and not self.curve_on and cv.state == "off" and not exiting:
      return None
    try:
      mod = curve_mod
      now = time.monotonic()
      if read:
        self.curve_on = self.params_memory.get(mod.PARAM_ON) in (b"1", "1")
        # while OFF the planner's output is read once a second, only for the page ("OFF. Corner in 0.4 mi: would slow ...")
        off_idle = not self.curve_on and cv.state == "off"
        raw = self.params_memory.get(mod.PARAM_PLAN) if (not off_idle or self.frame % 100 == 5) else None
        plan = None
        if raw:
          try:
            plan = json.loads(raw)
          except Exception:
            plan = None
        if isinstance(plan, dict) and isinstance(plan.get("m"), (int, float)):
          self.curve_plan, self.curve_plan_read = plan, now
          self.curve_plan_age = max(0.0, now - float(plan["m"]))
        elif raw is None and not off_idle:
          self.curve_plan = None
      if not self.curve_on and cv.state == "off" and not exiting:
        if read and getattr(CS, "crz_btns_set", False) and (cv.restart_note or cv.off_note):
          cv.restart_note = cv.off_note = ""    # the driver's own SET+ / SET-: the "check it" note has been answered
          self.curve_published = None
        if read and (self.curve_published is None or self.frame % 100 == 5):
          age = None if self.curve_plan_read is None else self.curve_plan_age + (now - self.curve_plan_read)
          out = CS.out
          speed = float(out.cruiseState.speed or 0.0)
          cv.off_view(self.curve_plan, age, int(round(speed * CV.MS_TO_MPH)) if speed > 0.5 else None, float(out.vEgo) * CV.MS_TO_MPH)
          self._curve_publish(cv, force=True)
        return None
      age = None
      if self.curve_plan is not None and self.curve_plan_read is not None:
        age = self.curve_plan_age + (now - self.curve_plan_read)
      inp = self._curve_inputs(CC, CS, now, age, mon)
      self.curve_inp_last = inp
      button = mod.run_frame(cv, ex, self.frame, inp)
      for evs in (cv.events, ex.events if ex is not None else []):
        for ev in evs:
          ti_curve_log(ev)
        evs.clear()
      self._curve_publish(cv)
      if button is None or inp.cancel_req or inp.resume_req:
        return None
      return mazdacan.create_button_cmd(self.packer, self.CP, inp.counter, Buttons.SET_MINUS if button == "SET_MINUS" else Buttons.SET_PLUS)
    except Exception:
      self.curve = None
      if ex is not None:
        try:
          # review C2: the safe exit does not depend on the object that failed: it runs from its own state from the next frame
          ex.arm(self.frame, self.curve_inp_last, "internal error")
        except Exception:
          pass
      try:
        mk = None if ex is None else ex.marker(None, getattr(self.curve_inp_last, "set_mph", None))
        self.params_memory.put(curve_mod.PARAM_STATUS, json.dumps({"t": round(time.time(), 1), "v": 1, "state": "stopped",
                                                                   "line1": "CURVE SLOWING STOPPED", "line2": "internal error: off until restart",
                                                                   "exit": mk}))
      except Exception:
        pass
      try:
        from openpilot.common.swaglog import cloudlog
        cloudlog.exception("ti_curve disabled")
      except Exception:
        pass
      return None

  def _curve_inputs(self, CC, CS, now, age, mon):
    """Curve slowing's car-side Inputs for this frame (the safe exit's too)."""
    mod = curve_mod
    out = CS.out
    cruise = out.cruiseState
    speed = float(cruise.speed or 0.0)
    counter = getattr(CS, "crz_btns_counter", None)
    chk = self.cruise_check
    return mod.Inputs(
      on=self.curve_on, engaged=bool(CC.enabled), cruise_on=bool(cruise.enabled), cruise_available=bool(cruise.available),
      mph=float(out.vEgo) * CV.MS_TO_MPH, set_mph=int(round(speed * CV.MS_TO_MPH)) if speed > 0.5 else None,
      brake=bool(out.brakePressed), gas=bool(out.gasPressed), lead=getattr(CS, "crz_lead", None),
      driver_button=getattr(CS, "crz_btns_driver", None), counter=None if counter is None else int(counter),
      cancel_req=bool(CC.cruiseControl.cancel), resume_req=bool(CC.cruiseControl.resume),
      check_active=bool(chk is not None and chk.active), plan=self.curve_plan, plan_age=age, mono=now,
      standstill=bool(out.standstill or getattr(cruise, "standstill", False)), driver_set=bool(getattr(CS, "crz_btns_set", False)),
      driver_set_plus=bool(getattr(CS, "crz_btns_set_plus", False)),
      deliver=bool(mon is not None and mon.phase == "alert"), deliver_need=None if mon is None else mon.need_mph,
      deliver_why="" if mon is None else mon.why,
      la_now=_finite_or_none(getattr(CS, "lat_accel", None)),
      desired_la=_finite_or_none(-float(out.vEgo) ** 2 * float(CC.actuators.curvature)))   # curvature RIGHT positive: left + here

  def _curve_exit_only(self, CC, CS):
    """After an internal error (self.curve is None): the safe exit alone, from its own state, until it ends; never raises."""
    ex = self.curve_exit
    try:
      inp = self._curve_inputs(CC, CS, time.monotonic(), None, None)
      b = curve_mod.run_frame(None, ex, self.frame, inp)
      self._curve_left(None, ex, inp.set_mph)
      for ev in ex.events:
        ti_curve_log(ev)
      ex.events.clear()
      if self.frame % 10 == 5 or not ex.active:
        self.params_memory.put(curve_mod.PARAM_STATUS, json.dumps({
          "t": round(time.time(), 1), "v": 1, "state": "stopped", "why": "internal error",
          "line1": ex.line1 if ex.active else "CURVE SLOWING STOPPED",
          "line2": ex.line2 if ex.active else "internal error: off until restart",
          "ref": ex.ref, "set": inp.set_mph, "exit": ex.marker(None, inp.set_mph)}))
      if b is None or inp.cancel_req or inp.resume_req:
        return None
      return mazdacan.create_button_cmd(self.packer, self.CP, inp.counter, Buttons.SET_PLUS)
    except Exception:
      try:
        ex.cancel(self.frame, "failed")
      except Exception:
        self.curve_exit = None
      return None

  def _curve_left(self, cv, ex, set_mph):
    """Review I4: write / clear the generic TiSetSpeedLeft record (who, now, was) for the on-road alert; at most every 0.5 s."""
    left = curve_mod.set_left(cv, ex, set_mph)
    if left is not None and (self.frame % 50 == 5 or getattr(self, "curve_left", None) != left):
      self.params_memory.put(curve_mod.PARAM_LEFT, json.dumps({"m": round(time.monotonic(), 2), "who": "curve slowing",
                                                               "now": left[0], "was": left[1]}))
    elif left is None and getattr(self, "curve_left", None) is not None:
      self.params_memory.put(curve_mod.PARAM_LEFT, json.dumps({"m": round(time.monotonic(), 2), "who": "curve slowing",
                                                               "now": None, "was": None}))
    self.curve_left = left

  def _curve_publish(self, cv, force=False):
    ex = getattr(self, "curve_exit", None)
    inp = getattr(self, "curve_inp_last", None)
    self._curve_left(cv, ex, getattr(inp, "set_mph", None))
    mk = None if ex is None else ex.marker(cv, getattr(inp, "set_mph", None))
    ex_lines = (ex.line1, ex.line2) if ex is not None and ex.active else None
    key = (cv.state, cv.why, cv.line1, cv.line2, cv.alert_mph, json.dumps(mk), ex_lines)
    # the exit marker is published at once when it changes (a controls restart must find it)
    marker_changed = self.curve_published is not None and len(self.curve_published) > 5 and key[5] != self.curve_published[5]
    if force or marker_changed or (key != self.curve_published and self.frame % 10 == 5) or self.frame % curve_mod.STATUS_EVERY == 5:
      st = cv.status(time.time())
      st["exit"] = mk
      if ex_lines is not None:
        st["line1"], st["line2"] = ex_lines
      self.params_memory.put(curve_mod.PARAM_STATUS, json.dumps(st))
      self.curve_published = key

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    can_sends = []

    if self.flag_ti:
      lag = getattr(self, "ti_lag", None)
      if lag is not None:
        try:
          self.ti_lag_held = lag.tick(time.monotonic(), getattr(self, "ti_lag_ctl", None))
        except Exception:
          self.ti_lag, self.ti_lag_held = None, False
      self.apply_ti_tuning(frogpilot_toggles)
      # Live TI state, refreshed every frame regardless of engagement. The mode and violation are
      # kept on the CarState object rather than published in cereal, so nothing outside this
      # process can see them unless we forward them. "Is the interceptor healthy right now" should
      # not require having already driven.
      self.ti_live = {"mode": int(CS.ti_state), "viol": int(CS.ti_violation),
                      "ramp": bool(CS.ti_ramp_down), "version": int(CS.ti_version)}

    apply_steer = 0
    is_ti = getattr(self, "flag_ti", None)        # the cached flag (a host without it: the steering-block tests)
    if is_ti is None:
      is_ti = bool(self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR)
    ti_apply_steer = 0
    ti_unguarded = 0     # the TI command before the stuck-request guard (equal to the sent one except on a nudge frame)
    new_steer = ti_new_steer = 0
    ti_dither = 0
    ti_ident = 0         # the identification tests' counts (ti_ident.py); zero whenever none runs
    ti_reported = None   # the TI command reported to controlsd when it differs from the one sent (dither on)
    stock_previous, ti_previous = self.apply_steer_last, self.ti_apply_steer_last

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_steer = int(round(CC.actuators.steer * self.ccp.STEER_MAX))
      apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                     CS.out.steeringTorque, self.ccp)
      if is_ti:
        ti_new_steer = 0
        if CS.ti_lkas_allowed:
          ti_new_steer = int(round(CC.actuators.steer * self.ccp.TI_STEER_MAX))
          # Identification dither, TI path only, added BEFORE the limiter so every existing bound still applies to
          # the total. Zero whenever the toggle is off (ti_dither.py). getattr, here and at the reset below:
          # tools/mazda_ti/test_diagnostics.py executes this block on a minimal host that has no dither, which must
          # keep the undithered path.
          dither = getattr(self, "ti_dither", None)
          if dither is not None:
            # One signal at a time: switched on while an identification test is still sending (fading included),
            # the dither waits until that signal has ended. Without a running test this is the toggle as before.
            ident_now = getattr(self, "ti_ident", None)
            ti_dither = dither.update(
              self.ti_dither_enabled and not getattr(self, "ti_ab_on", False) and not (ident_now is not None and ident_now.running), self.frame, v_ego=CS.out.vEgo, steering_pressed=CS.out.steeringPressed,
              driver_torque=CS.out.steeringTorque,
              blinker=bool(CS.out.leftBlinker or CS.out.rightBlinker or CC.leftBlinker or CC.rightBlinker),
              base_request=ti_new_steer, ti_max=self.ccp.TI_STEER_MAX,
              lat_demand=CS.out.vEgo ** 2 * CC.actuators.curvature, room=rate_room(self.ccp))
            ti_new_steer += ti_dither
          # Identification tests: the same place and the same rules as the dither, never both at once (armed only
          # while the dither toggle is off and the dither silent, and only one test toggle on). getattr: as above.
          ident = getattr(self, "ti_ident", None)
          if ident is not None:
            tests = getattr(self, "ti_tests", None)
            on = self.ti_ident_on
            free = (tests is not None and not self.ti_dither_enabled and (dither is None or dither.envelope == 0.0)
                    and not getattr(self, "ti_ab_on", False))
            rem_now = tests.remaining if tests is not None else None
            try:
              if free and self.ti_ident_all and not on:
                # Run all: the scheduler picks the test for the window the car is in, or the nearest one between
                # windows (ti_ident.RunAll); a seeded random choice once the window is complete
                profile = ident.advance_zone(CS.out.vEgo).profile      # with the speed hysteresis
                # one frame's answers computed once (ti_ident.frame_scheduler: the same answers as ident.can_run /
                # ident.next_is_strict on tests.remaining, without re-running the step search for every question)
                sched = getattr(self, "ti_sched", None)
                if sched is None:
                  sched = self.ti_sched = ti_ident_mod.SchedulerCache()
                rem_now, can_run, strict = sched.frame(ident, tests.remaining, profile, (id(tests), getattr(tests, "version", None)))
                armed = self.ti_run_all.pick(self.frame, profile, rem_now, can_run, strict, stats=getattr(can_run, "stats", None))
              else:
                armed = on[0] if (len(on) == 1 and free and not self.ti_ident_all) else None
              self.ti_ident_armed = armed
              ti_ident = int(ident.update(
                armed, self.frame, v_ego=CS.out.vEgo, steering_pressed=CS.out.steeringPressed,
                driver_torque=CS.out.steeringTorque,
                blinker=bool(CS.out.leftBlinker or CS.out.rightBlinker or CC.leftBlinker or CC.rightBlinker),
                base_request=ti_new_steer - ti_dither, ti_max=self.ccp.TI_STEER_MAX,
                lat_demand=CS.out.vEgo ** 2 * CC.actuators.curvature, curvature=CC.actuators.curvature,
                lkas_blocked=getattr(CS, "lkas_blocked", None),
                room=ti_ident_mod.rate_room(self.ccp, armed) if armed is not None else None,
                remaining=rem_now))
            except Exception:
              # Never take the controller down: no signal from this frame on, tests off for the rest of the drive
              ti_ident, ident = 0, None
              self.ti_ident_failed()
            ti_new_steer += ti_ident
          ti_apply_steer = apply_ti_steer_torque_limits(ti_new_steer, self.ti_apply_steer_last,
                                                    CS.out.steeringTorque, self.ccp)
          if (dither is not None and dither.envelope > 0.0) or (ident is not None and ident.emitted):
            # What the car would be sent had there been no dither: the same pure limiter on the undithered request,
            # run on its own (undithered) history, ti_reported_last. Running it on the sent history instead would
            # see every dither swing as a step and report rate limiting that is not there. This is what controlsd
            # compares with its requests, so its limited flag (and the integrator freeze) see the real limiting of
            # the controller's request and never the dither.
            ti_reported = apply_ti_steer_torque_limits(ti_new_steer - ti_dither - ti_ident, self.ti_reported_last,
                                                       CS.out.steeringTorque, self.ccp)
            self.ti_dither_frames += ti_dither != 0
            if ti_ident != 0:
              self.ti_ident_frames[ident.emitted] = self.ti_ident_frames.get(ident.emitted, 0) + 1
        # Recorded outside the ti_lkas_allowed gate on purpose: that flag is false precisely when
        # the TI has left RUN or is ramping down, which are the frames the health counters exist
        # to catch. Gating on it would make them structurally unreachable.
        self.record_ti_stats(CS, ti_new_steer, ti_apply_steer)
    ti_unguarded = ti_apply_steer
    guard = getattr(self, "ti_stuck_guard", None)   # getattr: the steering-block tests' minimal hosts have no guard
    if is_ti and guard is not None:
      # Stuck-request guard, on the FINAL command (after the limiters, the dither and the tests): a value other than 0 / +-600 sent
      # unchanged for STUCK_HOLD_S is moved STUCK_NUDGE counts toward zero for STUCK_NUDGE_FRAMES frames, so the interceptor never
      # sees the ~1.9 s unchanged request that trips its 0x11 fault. The value sent becomes the limiter's history below, so the step
      # back up is the limiter's own.
      ti_apply_steer = guard.update(ti_apply_steer, bool(CC.latActive and CS.ti_lkas_allowed), now=now_nanos * 1e-9)
      if guard.burst_started and (self.ti_stuck_logged is None or self.frame - self.ti_stuck_logged >= TI_STUCK_LOG_EVERY_FRAMES):
        self.ti_stuck_logged = self.frame
        ti_stuck_log({"kind": "nudge", "frame": self.frame, "value": int(ti_unguarded), "sent": int(ti_apply_steer),
                      "bursts": guard.bursts})
    self.apply_steer_last = apply_steer
    self.ti_apply_steer_last = ti_apply_steer
    if is_ti:
      # The undithered limiter history; equal to the sent one whenever the dither is idle.
      self.ti_reported_last = ti_apply_steer if ti_reported is None else ti_reported
      self.ti_dither_seen |= self.ti_dither_enabled
      dither = getattr(self, "ti_dither", None)   # same guard as in the steering block above
      if dither is not None and not (CC.latActive and CS.ti_lkas_allowed):
        dither.reset()   # lateral control off or TI not in RUN: zero at once, next fade-in from nothing
      ident = getattr(self, "ti_ident", None)
      if ident is not None:
        self.ti_ident_seen |= set(self.ti_ident_on) | (set(ti_ident_mod.KEYS) if self.ti_ident_all else set())
        self.ti_ident_all_seen |= self.ti_ident_all
        if not (CC.latActive and CS.ti_lkas_allowed):
          try:
            ident.reset(self.frame)   # the same for the identification tests
          except Exception:
            self.ti_ident_failed()

    if self.flag_gen1:
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

      # Cruise set-speed check: SET_M / SET_P only, never CANCEL or RESUME, and nothing on a frame where openpilot asks for either
      # (checked again inside). After the existing cancel / resume logic, which it never changes.
      if self.flag_ti:
        cruise_msg = self.cruise_check_step(CC, CS, ti_apply_steer)
        if cruise_msg is not None:
          can_sends.append(cruise_msg)
        # Curve slowing: SET_M / SET_P only, never on a frame the cruise check sends (one button frame per control frame at most)
        curve_msg = self.curve_step(CC, CS, ti_apply_steer)
        if curve_msg is not None and cruise_msg is None:
          can_sends.append(curve_msg)

      # send HUD alerts
      if self.frame % 50 == 0:
        ldw = CC.hudControl.visualAlert == VisualAlert.ldw
        steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
        # TODO: find a way to silence audible warnings so we can add more hud alerts
        steer_required = steer_required and CS.lkas_allowed_speed
        if not self.flag_no_fsc:
          can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

      if self.flag_radar:
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

    elif self.flag_gen2:
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
      ti_apply_steer if self.flag_ti else None
    ))

    new_actuators = CC.actuators.as_builder()
    if is_ti and CS.ti_lkas_allowed:
      # While the TI is in RUN it is the actuator that actually steers the car, so report its
      # command rather than the stock LKAS one. torqued fits its model to actuatorsOutput.steer,
      # and the two commands are limited differently (TI_STEER_DELTA_UP vs STEER_DELTA_UP, driver
      # multiplier 40 vs 1), so reporting the stock command trains it on a signal the car is not
      # following. Falls back to the stock command whenever the TI is bypassed, which is correct
      # because the stock LKAS path is then the only thing steering.
      # With the identification dither on, `steer` is the limiter's output for the UNDITHERED request: controlsd
      # flags the output as limited (and freezes the integrator) when it matches none of the recent requests to 1e-2,
      # i.e. 6 counts, and a dither of that size is not a limit. steerOutputCan stays the command actually sent, so
      # steerOutputCan - round(steer * TI_STEER_MAX) is the dither as applied. ti_reported is None, and this is the
      # old expression, whenever the dither's envelope is zero (toggle off, gate closed and faded out, or a touch);
      # while the envelope is above zero it is set on every frame, including frames where the dither itself rounds
      # to zero.
      # The stuck-request guard's nudge is treated like the dither: `steer` is the command before it (the nudge is not a limit for
      # controlsd to react to), steerOutputCan and mazdaDiagnostics.tiLimited are the value actually sent.
      new_actuators.steer = (ti_unguarded if ti_reported is None else ti_reported) / self.ccp.TI_STEER_MAX
      new_actuators.steerOutputCan = ti_apply_steer
    else:
      new_actuators.steer = apply_steer / self.ccp.STEER_MAX
      new_actuators.steerOutputCan = apply_steer

    if self.flag_ti:
      # Trigger polls stay at 1Hz -- they are tmpfs reads costing microseconds, and the flag button
      # needs to feel responsive.
      if self.frame % 100 == 0:
        self.maybe_refresh_route()
        self.check_flag_request(CS, CC, ti_apply_steer)
        self.check_clear_request()

      # On-car tests: count this frame (ti_dither is non-zero exactly on the frames ti_dither_frames counts), after
      # every output of the frame has been computed; the accountant only reads, and never raises.
      tests = getattr(self, "ti_tests", None)
      if tests is not None:
        ident = getattr(self, "ti_ident", None)
        active = TI_DITHER_ACTIVE if ti_dither != 0 else ()
        if ident is not None and ident.emitted and ident.reason in ti_ident_mod.RUNNING_REASONS:
          active = active + (ident.emitted,)    # an identification test running clean (screen: ON · ACTIVE)
        tests.update(frogpilot_toggles, CS.out.vEgo, CC.latActive, active,
                     credit=ident.credit if ident is not None else None,
                     run_all_forced=bool(self.ti_ident_all))

      # Publishing moved from 1Hz to 0.2Hz. Params.put_nonblocking spawns a fresh thread through
      # std::async on nearly every call, and this is a SCHED_FIFO process under mlockall where a
      # new thread's stack has to be allocated and faulted in. At 1Hz that cost landed in the
      # 100Hz thread that builds the steering frame once a second, on a fixed phase -- visible in
      # the logs as a skipped carState frame at exactly 1Hz, which then made the 20Hz consumers
      # polling carState fail their all_checks and flag their own output invalid, which controlsd
      # reports as commIssue. Five seconds of staleness costs the counters nothing. The test progress rides on the
      # same cadence (a tmpfs write; its flash copy at most once a minute, ti_tests.py).
      if self.frame % 500 == 0:
        self.publish_ti_stats()
        self.publish_ti_tests(CS, CC, ti_new_steer - ti_dither - ti_ident)
      if self.frame % 100 == 0:
        self.publish_ti_gate()
      if self.frame % 10 == 0 and self.ti_plan_on and ti_plan_mod is not None:
        self.publish_plan_ack()

    self.frame += 1
    Timer.tick()
    if is_ti:
      diag = car.CarOutput.MazdaActuatorDiagnostics.new_message()
      diag.version = 1
      diag.latActive = CC.latActive
      diag.tiAllowed = bool(CS.ti_lkas_allowed)
      diag.stockRequested = int(new_steer)
      diag.stockLimited = int(apply_steer)
      diag.tiRequested = int(ti_new_steer)
      diag.tiLimited = int(ti_apply_steer)
      diag.stockPrevious = int(stock_previous)
      diag.tiPrevious = int(ti_previous)
      diag.driverTorque = float(CS.out.steeringTorque)
      diag.tiMax = float(self.ccp.TI_STEER_MAX)
      diag.tiDeltaUp = float(self.ccp.TI_STEER_DELTA_UP)
      diag.tiDeltaDown = float(self.ccp.TI_STEER_DELTA_DOWN)
      diag.tiDriverAllowance = float(self.ccp.TI_STEER_DRIVER_ALLOWANCE)
      diag.tiDriverMultiplier = float(self.ccp.TI_STEER_DRIVER_MULTIPLIER)
      diag.tiDeltaUpKnee = float(self.ccp.TI_STEER_DELTA_UP_KNEE)
      diag.tiDeltaUpHigh = float(self.ccp.TI_STEER_DELTA_UP_HIGH)
      self.lateral_diagnostics = diag

    return new_actuators, can_sends
