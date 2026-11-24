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
ALERT_MIN_S = 2.0
CLEAR_OFF_S = 1.0
REARM_M = 100.0
CHIME_REPEAT_S = 10.0
MON_SHED_MPH = 5
GENTLE_TAP_S = 1.0              # the car side's stage-3 tap pacing (about 1 mph/s)
TURN_MIN_LA = 0.3               # below this |desired| (else |measured|) lateral acceleration there is no turn to lose
MPH = 0.44704
VIOL_NAMES = {0x11: "stuck request"}
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
    self.lim_s = 0.0
    self.drop_s = 0.0
    self.off_s_ti = 0.0
    self.alert_s = 0.0
    self.clear_s = 0.0
    self.cool_m = 0.0
    self.last_onset = None        # (frame, need)
    self.onsets = 0
    self.events = []

  def _paths(self, inp):
    """(limit, dropout, ti_off) raw conditions this frame, and the remaining forecast seconds."""
    sat = inp.lat_active and inp.ti_max > 0 and abs(inp.ti_cmd) >= SAT_SHARE * inp.ti_max
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
    dropout = bool(sat and inp.mph >= DROPOUT_MIN_MPH and inp.lkas_blocked and inp.lkas_effective is not None
                   and inp.lkas_effective == 0)
    ti_off = bool(inp.lat_active and inp.ti_state is not None and inp.ti_state != TI_RUN)
    return sat, limit, dropout, ti_off, rem

  def step(self, frame, inp):
    sat, limit, dropout, ti_off, rem = self._paths(inp)
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
