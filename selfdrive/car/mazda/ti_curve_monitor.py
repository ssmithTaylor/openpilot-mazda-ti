"""Curve slowing stage 3: the in-bend delivery monitor (plan T7, A2, A4). Pure: no clock, no params, no messaging; the car controller
feeds it one control frame (100 Hz) at a time and relays its output (tmpfs TiCurveDeliver, the Level 1 curve warning, gentle taps).

It fires when the steering is AT ITS LIMIT AND LOSING THE BEND, never for the limit alone (owner: steering at the limit by itself is
not trouble; 66 % of passes at 2.3-2.5 m/s^2 touch it and are held). All of these for MON_HOLD_S:
  * the SENT interceptor command (after every limiter: A4, the unit slews its own working request) is at SAT_SHARE x its maximum or
    more, lateral control active, the interceptor in RUN, the driver not steering;
  * the steering angle is no longer winding into the turn: its rate toward the turn below ANGLE_STALL;
  * the bend asks more than the car is doing: max(forecast peak `ahead_la`, the controller's `desired_la`) toward the turn at least
    `la_now` + ASK_MARGIN, and the forecast keeps asking that for at least MON_REMAIN_S more (`ahead_prof`; without a profile the
    condition itself must have lasted MON_HOLD_S + MON_REMAIN_S).
Two more paths: DROPOUT (speed at least DROPOUT_MIN_MPH, the stock lane-keep path blocked with no effect, the interceptor saturated,
for MON_HOLD_S) and, A2, the interceptor leaving RUN while lateral control is active (TI_FEEDBACK STATE != 3, for MON_HOLD_S), naming
its violation code when there is one (0x11: stuck request).

Alert discipline (plan "Alerts combined"): one continuous event per bend, held at least ALERT_MIN_S and until the path's limit has been
off CLEAR_OFF_S; re-armed only after REARM_M driven with the limit off; no onset for the same need within CHIME_REPEAT_S. Its need is
latched at onset: max(floor, min(ahead_need, mph - MON_SHED_MPH)) (the floor is applied by the car side); the car side walks toward it
GENTLY (ti_curve: at most one tap per GENTLE_TAP_S, about 1 mph/s; A2).

Signs: every lateral figure here is LEFT positive (m/s^2), the steering angle rate too (deg/s, openpilot's steeringRateDeg).
"""
import math
from typing import NamedTuple, Optional

DT = 0.01
MON_HOLD_S = 0.5
SAT_SHARE = 0.983               # 590 / 600: "at the limit" (measured definition)
ANGLE_STALL = 2.0               # deg/s toward the turn
ASK_MARGIN = 0.3                # m/s^2 (per-pass scatter)
MON_REMAIN_S = 1.5              # the trouble definition (2 s at the limit) minus the hold
DROPOUT_MIN_MPH = 32.0
TI_RUN = 3                      # TI_FEEDBACK STATE: RUN
TI_DISCOVER, TI_OFF, TI_DRIVER_OVER = 0, 1, 2   # the other states (DRIVER_OVER: firm driver torque, never "assist off")
MIN_MON_MPH = 20.0              # review I1: no stage-3 alert of any path below this (junction turns; the engage floor is ~19 mph)
STEER_QUIET_S = 1.0             # review I1: no "assist off" while the driver steers nor within this after it
ASSIST_TURN_LA = 0.8            # review I1: "assist off" only with a turn in progress or imminent: |desired| or |measured| lateral
                                # acceleration, or the forecast (ahead_la / its profile), at least this (m/s^2): the limit path's
                                # "bend asks" evidence at a lower bar (no margin over what the car does: the assist is gone)
MIN_NEED_MPH = 25               # review I1: a need is never below the feature's absolute floor (ti_curve.ABS_FLOOR_MPH)
ALERT_MIN_S = 2.0
CLEAR_OFF_S = 1.0
REARM_M = 100.0
CHIME_REPEAT_S = 10.0
MON_SHED_MPH = 5
GENTLE_TAP_S = 1.0              # the car side's stage-3 tap pacing (about 1 mph/s)
TURN_MIN_LA = 0.3               # below this |desired| (else |measured|) lateral acceleration there is no turn to lose
MPH = 0.44704
VIOL_NAMES = {0x11: "stuck request"}
# The stuck-request guard (ti_stuck_guard, carcontroller: on the FINAL command, before this monitor reads it) moves an unchanged sent
# value STUCK_NUDGE counts toward zero for STUCK_NUDGE_FRAMES frames. Exactly +-600 is never nudged, but a saturated value at the
# band's edge (590 / 591 at TiSteerMax 600: 588 / 589) and, below TiSteerMax 118, the clipped command itself would leave the band and
# reset the hold. A dip of at most SAT_GRACE_COUNTS below the band lasting at most SAT_GRACE_FRAMES, straight after a saturated frame,
# is therefore still "at the limit" (Job 4, 2026-11-24); a real fall off the limit (larger or longer) resets as before.
try:
  from openpilot.selfdrive.car.mazda import ti_stuck_guard as _SG
  SAT_GRACE_COUNTS, SAT_GRACE_FRAMES = _SG.STUCK_NUDGE, _SG.STUCK_NUDGE_FRAMES
except ImportError:                                   # the guard's module absent: its constants as merged (2 counts, 2 frames)
  SAT_GRACE_COUNTS, SAT_GRACE_FRAMES = 2, 2
ASK_LEADS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)   # the planner's Plan.ask profile leads (s)


class MonInputs(NamedTuple):
  mph: float = 0.0
  ti_cmd: float = 0.0             # the SENT interceptor command (counts)
  ti_max: float = 600.0
  lat_active: bool = False
  ti_active: bool = False         # the interceptor steering (RUN and not ramping down)
  steering_pressed: bool = False
  angle_rate_deg: float = 0.0     # steering angle rate, left positive
  la_now: Optional[float] = None  # measured lateral acceleration, left positive (includes bank)
  desired_la: Optional[float] = None   # the controller's desired lateral acceleration now (v^2 x desired curvature), left positive
  ahead_la: Optional[float] = None     # the planner's signed peak calibrated forecast demand within 3 s (Plan.ahead_la)
  ahead_need: Optional[int] = None     # the speed that forecast bend needs (mph), or None
  lkas_blocked: Optional[bool] = None
  lkas_effective: Optional[float] = None
  ti_state: Optional[int] = None  # TI_FEEDBACK STATE (A2), None unknown
  ti_viol: int = 0                # TI_FEEDBACK VIOL
  ahead_prof: Optional[tuple] = None   # (leads s, calibrated signed demand m/s^2) of the forecast, for MON_REMAIN_S
  floor_mph: Optional[int] = None      # the car side's floor now (ti_curve floor_now); the need never goes below it (review I1)


class MonOut(NamedTuple):
  phase: str                      # "idle", "watch" (a path's conditions building), "alert", "cool" (cleared, not yet re-armed)
  why: str                        # "" / "limit" / "dropout" / "assist off" / "assist off: 0x11 stuck request"
  need_mph: Optional[int]         # latched at onset while alerting
  sat_s: float                    # seconds the sent command has been at the limit


def _sign(x):
  return 1.0 if x > 0 else -1.0 if x < 0 else 0.0


def remain_s(prof, floor_la, turn):
  """Seconds from now the forecast keeps asking at least floor_la toward `turn` (the first lead where it stops), or None."""
  if prof is None:
    return None
  ts, la = prof
  out = 0.0
  for t, a in zip(ts, la):
    if a is None or not math.isfinite(a) or turn * a < floor_la:
      return out
    out = float(t)
  return out


class DeliveryMonitor:
  def __init__(self):
    self.phase = "idle"
    self.why = ""
    self.need = None
    self.sat_s = 0.0
    self.grace_n = 0              # consecutive frames bridged by the guard-nudge grace
    self.lim_s = 0.0
    self.drop_s = 0.0
    self.off_s_ti = 0.0
    self.alert_s = 0.0
    self.clear_s = 0.0
    self.cool_m = 0.0
    self.last_onset = None        # (frame, need)
    self.onsets = 0
    self.events = []
    self.was_run = False          # the interceptor has been in RUN during this drive (DISCOVER counts as "off" only after that)
    self.steer_f = None           # last frame the driver was steering

  def _paths(self, inp, frame=0):
    """(limit, dropout, ti_off) raw conditions this frame, and the remaining forecast seconds."""
    sat = inp.lat_active and inp.ti_max > 0 and abs(inp.ti_cmd) >= SAT_SHARE * inp.ti_max
    if sat:
      self.grace_n = 0
    elif (inp.lat_active and inp.ti_max > 0 and self.sat_s > 0.0 and self.grace_n < SAT_GRACE_FRAMES and
          abs(inp.ti_cmd) >= SAT_SHARE * inp.ti_max - SAT_GRACE_COUNTS - 1e-9):
      sat, self.grace_n = True, self.grace_n + 1      # a stuck-guard nudge, not a fall off the limit
    else:
      self.grace_n = 0
    des = inp.desired_la if inp.desired_la is not None else 0.0
    now = inp.la_now if inp.la_now is not None else 0.0
    turn = _sign(des) if abs(des) >= TURN_MIN_LA else (_sign(now) if abs(now) >= TURN_MIN_LA else 0.0)
    limit, rem = False, None
    if sat and inp.ti_active and not inp.steering_pressed and turn != 0.0 and inp.la_now is not None:
      stall = inp.angle_rate_deg * turn < ANGLE_STALL
      ask = max(turn * des, turn * inp.ahead_la if inp.ahead_la is not None else -1e9)
      floor_la = turn * now + ASK_MARGIN
      if stall and ask >= floor_la:
        limit = True
        rem = remain_s(inp.ahead_prof, floor_la, turn)
    # R18 (A15): the dropout path also needs the stalled wheel (the angle rate toward the turn under ANGLE_STALL; turn unknown: its
    # magnitude): a wheel still winding into the bend is the steering delivering, not a dropout (5 of its 11 false alerts)
    wheel_stall = (inp.angle_rate_deg * turn < ANGLE_STALL) if turn != 0.0 else (abs(inp.angle_rate_deg) < ANGLE_STALL)
    dropout = bool(sat and inp.mph >= DROPOUT_MIN_MPH and inp.lkas_blocked and inp.lkas_effective is not None
                   and inp.lkas_effective == 0 and wheel_stall)
    # review I1: the interceptor really OFF (state 1, or DISCOVER after RUN this drive), never DRIVER_OVER; never while the driver
    # steers or within STEER_QUIET_S of it; and only with a turn in progress or imminent
    if inp.ti_state == TI_RUN:
      self.was_run = True
    if inp.steering_pressed:
      self.steer_f = frame
    quiet = self.steer_f is None or (frame - self.steer_f) * DT >= STEER_QUIET_S - 1e-9
    prof = inp.ahead_prof[1] if inp.ahead_prof is not None and inp.ahead_prof[1] is not None else ()
    turning = max([abs(x) for x in (inp.desired_la, inp.la_now, inp.ahead_la) if x is not None and math.isfinite(x)] +
                  [abs(x) for x in prof if x is not None and math.isfinite(x)] + [0.0]) >= ASSIST_TURN_LA
    off_state = inp.ti_state == TI_OFF or (inp.ti_state == TI_DISCOVER and self.was_run)
    fast = inp.mph >= MIN_MON_MPH
    ti_off = bool(inp.lat_active and off_state and not inp.steering_pressed and quiet and turning and fast)
    limit, dropout = limit and fast, dropout and fast          # review I1: the same speed gate on the other paths
    return sat, limit, dropout, ti_off, rem

  def step(self, frame, inp):
    sat, limit, dropout, ti_off, rem = self._paths(inp, frame)
    self.sat_s = self.sat_s + DT if sat else 0.0
    self.lim_s = self.lim_s + DT if limit else 0.0
    self.drop_s = self.drop_s + DT if dropout else 0.0
    self.off_s_ti = self.off_s_ti + DT if ti_off else 0.0
    if self.phase == "alert":
      self.alert_s += DT
      on = {"limit": sat, "dropout": dropout}.get(self.why, ti_off)
      self.clear_s = 0.0 if on else self.clear_s + DT
      if self.alert_s >= ALERT_MIN_S - 1e-9 and self.clear_s >= CLEAR_OFF_S - 1e-9:
        self.events.append({"kind": "deliver_clear", "why": self.why, "need": self.need, "s": round(self.alert_s, 2)})
        self.phase, self.cool_m = "cool", 0.0
      return self._out()
    if self.phase == "cool":
      self.cool_m = 0.0 if sat or ti_off else self.cool_m + max(0.0, inp.mph) * MPH * DT
      if self.cool_m < REARM_M:
        return self._out()
      self.phase, self.why, self.need = "idle", "", None
    why = None
    if self.off_s_ti >= MON_HOLD_S - 1e-9:
      code = int(inp.ti_viol or 0)
      why = "assist off" + (f": 0x{code:02X}" + (f" {VIOL_NAMES[code]}" if code in VIOL_NAMES else "") if code else "")
    elif self.drop_s >= MON_HOLD_S - 1e-9:
      why = "dropout"
    elif self.lim_s >= MON_HOLD_S - 1e-9 and ((rem is not None and rem >= MON_REMAIN_S - 1e-9) or
                                             (rem is None and inp.ahead_prof is None and self.lim_s >= MON_HOLD_S + MON_REMAIN_S - 1e-9)):
      why = "limit"
    if why is None:
      self.phase = "watch" if (limit or dropout or ti_off) else "idle"
      return self._out()
    need = int(math.floor(inp.mph)) - MON_SHED_MPH
    if inp.ahead_need is not None:
      need = min(need, int(inp.ahead_need))
    need = max(need, MIN_NEED_MPH, int(inp.floor_mph) if inp.floor_mph else 0)   # review I1: never below the feature's floor
    if self.last_onset is not None and self.last_onset[1] == need and (frame - self.last_onset[0]) * DT < CHIME_REPEAT_S:
      self.phase = "watch"
      return self._out()                       # the same need chimed within CHIME_REPEAT_S: no new onset
    self.phase, self.why, self.need = "alert", why, need
    self.alert_s = self.clear_s = 0.0
    self.last_onset = (frame, need)
    self.onsets += 1
    self.events.append({"kind": "deliver_onset", "why": why, "need": need, "mph": round(inp.mph, 1), "sat_s": round(self.sat_s, 2),
                        "viol": int(inp.ti_viol or 0)})
    return self._out()

  def _out(self):
    alerting = self.phase == "alert"
    return MonOut(self.phase, self.why if alerting else "", self.need if alerting else None, round(self.sat_s, 2))
