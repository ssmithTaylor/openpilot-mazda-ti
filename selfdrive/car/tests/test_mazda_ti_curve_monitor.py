"""Curve slowing stage 3: the in-bend delivery monitor (ti_curve_monitor.py) and its wiring (plan T7, A2, A4). Synthetic inputs only."""
import json
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.mazda import ti_curve as C
from openpilot.selfdrive.car.mazda import ti_curve_monitor as TM
from openpilot.selfdrive.car.tests import test_mazda_ti_curve as TC

DT = TM.DT


def bend(**kw):
  """A left bend at 45 mph the steering is losing: the sent command at 600 of 600, the angle stalled, the controller asking 2.2 m/s^2
  where the car does 1.6, the forecast asking 2.2 for the next 3 s."""
  d = dict(mph=45.0, ti_cmd=600.0, ti_max=600.0, lat_active=True, ti_active=True, steering_pressed=False, angle_rate_deg=0.0,
           la_now=1.6, desired_la=2.2, ahead_la=2.2, ahead_need=40, lkas_blocked=False, lkas_effective=100.0, ti_state=3, ti_viol=0,
           ahead_prof=(TM.ASK_LEADS, [2.2] * len(TM.ASK_LEADS)))
  d.update(kw)
  return TM.MonInputs(**d)


def run(mon, seconds, fn, start=0):
  """fn(t) -> MonInputs; -> [(t, MonOut)]"""
  out = []
  for i in range(int(round(seconds / DT))):
    f = start + i
    out.append((f * DT, mon.step(f, fn(f * DT))))
  return out


def onsets(res):
  return sum(1 for (_, a), (_, b) in zip(res, res[1:]) if a.phase != "alert" and b.phase == "alert") + (res[0][1].phase == "alert")


def test_fires_after_the_hold_on_a_lost_bend():
  mon = TM.DeliveryMonitor()
  res = run(mon, 2.0, lambda t: bend())
  first = next(t for t, o in res if o.phase == "alert")
  assert first == pytest.approx(TM.MON_HOLD_S - DT, abs=1.5 * DT)
  o = res[-1][1]
  assert o.why == "limit" and o.need_mph == min(40, 45 - TM.MON_SHED_MPH) and o.sat_s > 1.9
  assert mon.events[0]["kind"] == "deliver_onset"
  assert (TM.SAT_SHARE, TM.ANGLE_STALL, TM.ASK_MARGIN, TM.MON_REMAIN_S, TM.MON_HOLD_S) == (0.983, 2.0, 0.3, 1.5, 0.5)


def test_held_bend_no_alert():
  # plan section 4 risk 1 / owner's rule: steering at the limit ALONE is not trouble. A normal hard bend held at the limit: command
  # at 600 but the wheel still winding in; or the bend not asking more than the car does; or the forecast asking more for under 1.5 s;
  # or below 590 of 600; or the driver steering; none alerts
  cases = [dict(angle_rate_deg=5.0),                                    # the angle still rising toward the turn
           dict(desired_la=1.8, ahead_la=1.8),                          # asks only 0.2 more than the car does
           dict(ahead_prof=(TM.ASK_LEADS, [2.2, 2.2, 1.6, 1.0, 0.5, 0.0])),   # the bend ends within 1 s
           dict(ti_cmd=589.0),
           dict(steering_pressed=True),
           dict(lat_active=False)]
  for kw in cases:
    mon = TM.DeliveryMonitor()
    res = run(mon, 6.0, lambda t, kw=kw: bend(**kw))
    assert not any(o.phase == "alert" for _, o in res), kw
  # the right-hand mirror of the lost bend does fire (signs: left positive throughout)
  mon = TM.DeliveryMonitor()
  res = run(mon, 1.0, lambda t: bend(ti_cmd=-600.0, la_now=-1.6, desired_la=-2.2, ahead_la=-2.2,
                                     ahead_prof=(TM.ASK_LEADS, [-2.2] * 6), angle_rate_deg=-5.0 if t < 0.2 else 1.0))
  assert any(o.phase == "alert" for _, o in res)


def test_no_profile_needs_the_whole_two_seconds():
  mon = TM.DeliveryMonitor()
  res = run(mon, 3.0, lambda t: bend(ahead_prof=None))
  first = next(t for t, o in res if o.phase == "alert")
  assert first == pytest.approx(TM.MON_HOLD_S + TM.MON_REMAIN_S - DT, abs=1.5 * DT)


def test_dropout_alert():
  # the dropout path: speed >= 32 mph, the stock lane-keep path blocked with no effect, the interceptor saturated 0.5 s; fires even
  # when the ask would not. R18 (A15): it also needs the stalled wheel (was: fired with the wheel still winding, angle rate 5.0 deg/s
  # toward the turn; now 0.5 here, and 5.0 is in the no-alert list below)
  kw = dict(lkas_blocked=True, lkas_effective=0.0, angle_rate_deg=0.5, desired_la=1.0, ahead_la=1.0)
  res = run(TM.DeliveryMonitor(), 1.0, lambda t: bend(**kw))
  first = next(t for t, o in res if o.phase == "alert")
  assert first == pytest.approx(TM.MON_HOLD_S - DT, abs=1.5 * DT) and res[-1][1].why == "dropout"
  for bad in (dict(mph=31.0), dict(lkas_effective=12.0), dict(lkas_blocked=False), dict(ti_cmd=500.0), dict(angle_rate_deg=5.0)):
    res = run(TM.DeliveryMonitor(), 2.0, lambda t, bad=bad: bend(**dict(kw, **bad)))
    assert not any(o.phase == "alert" for _, o in res), bad


def test_ti_not_in_run_alert_names_the_code():
  # A2: the interceptor leaves RUN while lateral control is active: Level 1, naming the violation code (0x11 = stuck request)
  res = run(TM.DeliveryMonitor(), 1.0, lambda t: bend(ti_state=1, ti_viol=0x11, ti_active=False, ti_cmd=0.0))
  o = res[-1][1]
  assert o.phase == "alert" and o.why == "assist off: 0x11 stuck request"
  # review I1 (2026-11-24) changed this (was: ti_state=2 alerted "assist off"): DRIVER_OVER is firm driver torque, never "assist off"
  res = run(TM.DeliveryMonitor(), 1.0, lambda t: bend(ti_state=2, ti_viol=0, ti_active=False, ti_cmd=0.0))
  assert not any(o.phase == "alert" for _, o in res)
  res = run(TM.DeliveryMonitor(), 1.0, lambda t: bend(ti_state=1, ti_viol=0, ti_active=False, ti_cmd=0.0))
  assert res[-1][1].why == "assist off"
  res = run(TM.DeliveryMonitor(), 2.0, lambda t: bend(ti_state=1, ti_viol=0x11, ti_active=False, ti_cmd=0.0, lat_active=False))
  assert not any(o.phase == "alert" for _, o in res)
  res = run(TM.DeliveryMonitor(), 2.0, lambda t: bend(ti_state=1 if t < 0.3 else 3, ti_viol=0x11, ti_active=t >= 0.3, ti_cmd=0.0))
  assert not any(o.phase == "alert" for _, o in res)           # under MON_HOLD_S: a blip


def test_alert_held_2s_and_until_the_limit_is_off_1s():
  mon = TM.DeliveryMonitor()
  res = run(mon, 0.7, lambda t: bend())                             # onset at 0.49 s, then the limit goes at 0.7 s
  res += run(mon, 4.0, lambda t: bend(ti_cmd=300.0), start=70)
  on = [t for t, o in res if o.phase == "alert"]
  assert on[-1] - on[0] >= TM.ALERT_MIN_S - 2 * DT
  # held until the limit has been off 1 s, when that is later than the 2 s minimum
  mon = TM.DeliveryMonitor()
  res = run(mon, 3.0, lambda t: bend())
  res += run(mon, 3.0, lambda t: bend(ti_cmd=300.0), start=300)
  last = max(t for t, o in res if o.phase == "alert")
  assert last == pytest.approx(3.0 + TM.CLEAR_OFF_S - DT, abs=2 * DT)


def test_max_two_chimes_per_bend():
  # plan "Alerts combined": one continuous event per bend, re-armed only after 100 m driven with the limit off; with the pre-bend
  # warning that makes at most 2 chimes (Level 1 starts) per bend, however the limit flickers
  mon = TM.DeliveryMonitor()
  # a 40 s bend at 45 mph (800 m): the limit on 0.8 s, off 1.2 s, all along
  res = run(mon, 40.0, lambda t: bend() if (t % 2.0) < 0.8 else bend(ti_cmd=300.0))
  assert onsets(res) == 1
  # the Level 1 signal as the frogpilot planner forms it: the planner's pre-bend warning (need 38, the 5 s before the bend) and stage
  # 3's, the lower need wins; chimes = starts of the alert or changes of its number
  pre = [(t, 38 if t < 5.0 else None) for t in [i * DT for i in range(500)]]
  lvl = [n for _, n in pre] + [o.need_mph for _, o in res]
  chimes = sum(1 for a, b in zip([None] + lvl, lvl) if b is not None and b != a)
  assert chimes <= 2, chimes
  # 100 m with the limit off re-arms it: the next bend alerts again
  res2 = run(mon, 6.0, lambda t: bend(ti_cmd=300.0), start=4000)
  res2 += run(mon, 2.0, lambda t: bend(ahead_need=36), start=4600)
  assert onsets(res2) == 1


def test_same_need_within_10s_no_new_onset():
  mon = TM.DeliveryMonitor()
  run(mon, 3.0, lambda t: bend())
  run(mon, 1.5, lambda t: bend(ti_cmd=300.0, mph=150.0), start=300)     # 100 m off the limit (fast) inside 10 s
  res = run(mon, 2.0, lambda t: bend(), start=450)
  assert not any(o.phase == "alert" for _, o in res)                # the same need chimed within CHIME_REPEAT_S


# ---- wiring ---------------------------------------------------------------------------------------------------------------------
def _cost_mod():
  return TC._controller()


def test_alert_within_1s_of_limit_onset():
  # plan T7: end to end, the car controller's monitor (on the SENT command) -> TiCurveDeliver -> the 10 Hz read the frogpilot planner
  # does (ti_curve.curve_alert_mph): the Level 1 warning within 0.8 s of the limit's onset (the plan's bound: 1 s)
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  mod = _cost_mod()
  cc, mem = mod.make_controller()
  clock = {"k": 0}
  ccm.time = NS(monotonic=lambda: clock["k"] * 0.01, time=lambda: 1.0e9 + clock["k"] * 0.01, perf_counter=lambda: 0.0)
  logged = []
  ccm.ti_curve_log = logged.append
  try:
    CC, CS = mod.inputs(45.0, -0.004, steer=1.0)                       # openpilot's curvature right +: a LEFT bend; full command
    CS.lat_accel, CS.lkas_effective = 1.4, 100.0
    CS.out.steeringRateDeg = 0.0
    toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
    onset, seen = None, None
    for k in range(600):
      clock["k"] = k
      if k % 20 == 0:
        mem.put(C.PARAM_ON, b"1")
        mem.put(C.PARAM_PLAN, json.dumps({"m": k * 0.01, "seq": k, "mode": "act", "ceil": None, "conf": 1.0, "trusted": True,
                                          "known": True, "why": "", "dist": None, "past": True, "model_la": 2.0, "model_k": 0.005,
                                          "capped": False, "limit": None, "kind": "", "corner": None, "ahead": 2.0,
                                          "ask": [2.0] * 6, "alert": None}))
      if k % 10 == 3:
        CS.crz_btns_counter = (CS.crz_btns_counter + 1) % 16
      cc.update(CC, CS, k * 10_000_000, toggles)
      if onset is None and abs(cc.ti_apply_steer_last) >= TM.SAT_SHARE * cc.ccp.TI_STEER_MAX:
        onset = k * 0.01
      if k % 2 == 0 and seen is None:                                  # the frogpilot planner: 20 Hz, reading every 2nd tick
        need, _ = C.curve_alert_mph(mem, clock["k"] * 0.01)
        if need is not None:
          seen = k * 0.01
    assert onset is not None and seen is not None, (onset, seen, cc.curve_mon.phase)
    assert seen - onset <= 0.8, seen - onset
    d = json.loads(mem.d[C.PARAM_DELIVER])
    assert d["need"] == 40 and d["why"] == "limit" and set(d) == {"m", "need", "why"}
    assert any(e.get("kind") == "deliver_onset" for e in logged)
    # switched off: nothing is written (Running Wide / Take Control are not this module's)
    cc2, mem2 = mod.make_controller()
    for k in range(300):
      clock["k"] = k
      if k % 20 == 0:
        mem2.put(C.PARAM_ON, b"0")
      cc2.update(CC, CS, k * 10_000_000, toggles)
    assert C.PARAM_DELIVER not in mem2.d or not json.loads(mem2.d[C.PARAM_DELIVER]).get("need")
  finally:
    import time as _t
    ccm.time = _t


def test_read_deliver_alert_and_lower_need_wins():
  store = _cost_mod().MemParams()
  assert C.read_deliver_alert(store, 10.0) == (None, 0.0) and C.curve_alert_mph(store, 10.0) == (None, 0.0)
  store.put(C.PARAM_DELIVER, json.dumps({"m": 10.0, "need": 41, "why": "limit"}))
  assert C.read_deliver_alert(store, 10.5) == (41, 10.0)
  assert C.read_deliver_alert(store, 10.0 + C.DELIVER_FRESH_S + 0.01) == (None, 0.0)      # stale
  store.put(C.PARAM_PLAN, json.dumps({"m": 10.2, "mode": "act", "alert": 38}))
  assert C.curve_alert_mph(store, 10.5) == (38, 10.2)
  store.put(C.PARAM_PLAN, json.dumps({"m": 10.2, "mode": "act", "alert": 44}))
  assert C.curve_alert_mph(store, 10.5) == (41, 10.0)
  store.put(C.PARAM_DELIVER, json.dumps({"m": 10.4, "need": None, "why": ""}))            # cleared
  assert C.curve_alert_mph(store, 10.5) == (44, 10.2)


def test_gentle_tap_rate():
  # A2: stage 3's taps are gentle: at most one SET- per GENTLE_TAP_S (about 1 mph/s), toward its need within the floor
  c, car = C.Curve(), TC.Car(55, 55)
  TC.run(c, car, 300, lambda f, car: TC.plan(None))                      # ready, no corner
  TC.run(c, car, 1200, lambda f, car: TC.plan(None), start=300, deliver=True, deliver_need=45, deliver_why="limit")
  downs = [f for f, b in car.sent if b == "SET_MINUS"]
  firsts = [f for i, f in enumerate(downs) if i == 0 or f - downs[i - 1] > 5]   # a tap's first pressed frame
  assert len(firsts) >= 8
  assert all(b - a >= int(C.GENTLE_TAP_S / C.DT) - 1 for a, b in zip(firsts, firsts[1:])), firsts
  assert car.set >= 45 and "slowing gently" in c.line2
  # the planner's own corner ceiling (not stage 3) still taps at the normal pace
  c, car = C.Curve(), TC.Car(55, 55)
  TC.run(c, car, 300, lambda f, car: TC.plan(None))
  TC.run(c, car, 400, lambda f, car: TC.plan(45, corner=45), start=300)
  assert car.set <= 50                                                  # 5+ mph in 4 s: not held to 1 mph/s


def test_monitor_cost_within_budget():
  # plan T7: the monitor <= 300 us per controller cycle (ti_cost_check.py mode "curve: monitor"); a PC under load, so the mean
  r = _cost_mod().run_monitor("curve: monitor", 1500)
  assert r["alert_share"] > 0 and r["written"]
  assert r["mean_us"] <= 300.0, r


def test_stage3_text_is_short_and_fits():
  # the on-road Level 1 text is the curve warning's own ("SLOW DOWN: corner needs N mph", width-checked at every speed by
  # test_mazda_ti_alert_text.py); the page's second line for stage 3 fits the page whole (MAX_CURVE_SECOND_CHARS), and the
  # TiCurveDeliver reason is short
  from openpilot.selfdrive.car.tests.test_mazda_ti_curve_screen import MAX_SECOND
  for why in ("limit", "dropout", "assist off: 0x11 stuck request"):
    c, car = C.Curve(), TC.Car(55, 55)
    TC.run(c, car, 300, lambda f, car: TC.plan(None))
    TC.run(c, car, 200, lambda f, car: TC.plan(None), start=300, deliver=True, deliver_need=45, deliver_why=why)
    assert "slowing gently" in c.line2 and len(c.line2) <= MAX_SECOND, c.line2
    assert len(why) <= 32


# ---- the stuck-request guard (ti_stuck_guard) nudges the SENT command; the monitor reads the sent command (Job 4) ---------------
def _guarded(ti_max, cmd, seconds=3.0):
  """The monitor fed the guard's output for a constant command; -> (sat_s per frame, first alert frame, nudge frames)."""
  from openpilot.selfdrive.car.mazda.ti_stuck_guard import StuckGuard
  g, mon, sat, first, nudges = StuckGuard(), TM.DeliveryMonitor(), [], None, 0
  for f in range(int(round(seconds / DT))):
    sent = g.update(int(cmd), True, now=f * DT)
    nudges += g.nudged
    o = mon.step(f, bend(ti_cmd=float(sent), ti_max=float(ti_max)))
    sat.append(o.sat_s)
    if first is None and o.phase == "alert":
      first = f
  return sat, first, nudges


def test_guard_never_nudges_exactly_600_so_a_clipped_command_stays_saturated():
  # the firmware exempts exactly +-600 (TI_FW_CLIP); the guard therefore never touches it: with TiSteerMax 600 a clipped command is
  # sent unchanged and the monitor's hold runs uninterrupted
  for cmd in (600, -600):
    sat, first, nudges = _guarded(600, cmd)
    assert nudges == 0 and all(b > a for a, b in zip(sat, sat[1:])) and first is not None


@pytest.mark.parametrize("ti_max", [100, 110, 117, 118, 300, 600, 800, 1200])
def test_a_guard_nudge_never_resets_the_monitor_hold(ti_max):
  # a nudge of STUCK_NUDGE (2) counts toward zero for STUCK_NUDGE_FRAMES (2) frames, every STUCK_HOLD_S of an unchanged value. At
  # TiSteerMax 600 the only saturated values it can move out of the monitor's band (>= 0.983 x 600 = 589.8) are 590 / 591 (-> 588 /
  # 589); at a TiSteerMax below 118 even the clipped command (ti_max - 2 < 0.983 x ti_max) leaves it. The monitor bridges a dip of at
  # most that size and length (SAT_GRACE_*), so its hold timer is never reset by a nudge: sat_s rises on every frame and the alert
  # comes on the same frame as with no guard
  from openpilot.selfdrive.car.mazda import ti_stuck_guard as SG
  lo = math.ceil(TM.SAT_SHARE * ti_max - 1e-9)
  for cmd in sorted({ti_max, lo, lo + 1}):
    sat, first, nudges = _guarded(ti_max, cmd)
    if ti_max != SG.TI_FW_CLIP or cmd != SG.TI_FW_CLIP:
      assert nudges >= 2 * SG.STUCK_NUDGE_FRAMES, (ti_max, cmd)       # the guard really acted, twice, in 3 s
    assert all(b > a for a, b in zip(sat, sat[1:])), (ti_max, cmd, [i for i, (a, b) in enumerate(zip(sat, sat[1:])) if b <= a][:3])
    mon, ref = TM.DeliveryMonitor(), None
    for f in range(300):
      if mon.step(f, bend(ti_cmd=float(cmd), ti_max=float(ti_max))).phase == "alert":
        ref = f
        break
    assert first == ref, (ti_max, cmd, first, ref)


def test_grace_is_only_the_nudge():
  # a real fall off the limit (more than STUCK_NUDGE counts, or longer than STUCK_NUDGE_FRAMES) still resets the hold
  mon = TM.DeliveryMonitor()
  seq = [600.0] * 30 + [589.0] * 3 + [600.0] * 10 + [587.0] + [600.0] * 5      # 589: inside the nudge band, but 3 frames; 587: below it
  sat = [mon.step(f, bend(ti_cmd=c)).sat_s for f, c in enumerate(seq)]
  assert sat[31] > sat[30] > sat[29] and sat[32] == 0.0 and sat[43] == 0.0



# ---- review I1 (2026-11-24): the assist-off path is not the driver steering; nothing below 20 mph; the need never below the floor ---
def test_i1_driver_over_on_a_straight_at_70_never_alerts():
  # the reviewer's probe 3a: a firm steer on a straight at 70 mph gave "SLOW DOWN: corner needs 65 mph"
  inp = TM.MonInputs(mph=70.0, ti_cmd=120.0, ti_max=600.0, lat_active=True, ti_active=False, steering_pressed=True,
                     angle_rate_deg=8.0, la_now=0.1, desired_la=0.0, ahead_la=0.0, ahead_need=None, lkas_blocked=False,
                     lkas_effective=100.0, ti_state=2, ti_viol=0)
  assert not any(o.phase == "alert" for _, o in run(TM.DeliveryMonitor(), 2.0, lambda t: inp))


def test_i1_limit_path_at_a_junction_at_12_mph_never_alerts():
  # the reviewer's probe 3b: a junction turn at 12 mph gave "corner needs 7 mph"
  inp = TM.MonInputs(mph=12.0, ti_cmd=600.0, ti_max=600.0, lat_active=True, ti_active=True, steering_pressed=False,
                     angle_rate_deg=0.0, la_now=1.0, desired_la=1.6, ahead_la=1.6, ahead_need=None, lkas_blocked=True,
                     lkas_effective=0.0, ti_state=3, ti_viol=0, ahead_prof=(TM.ASK_LEADS, (1.6,) * 6))
  assert not any(o.phase == "alert" for _, o in run(TM.DeliveryMonitor(), 2.0, lambda t: inp))


@pytest.mark.parametrize("path", ["limit", "dropout", "assist off"])
def test_i1_no_path_below_20_mph(path):
  kw = {"limit": {}, "dropout": dict(lkas_blocked=True, lkas_effective=0.0),
        "assist off": dict(ti_state=1, ti_active=False, ti_cmd=0.0)}[path]
  for mph, fires in ((19.5, False), (45.0, True)):
    res = run(TM.DeliveryMonitor(), 3.0, lambda t, mph=mph: bend(mph=mph, **kw))
    assert any(o.phase == "alert" for _, o in res) == fires, (path, mph)


def test_i1_assist_off_needs_no_driver_steering_within_1_s():
  res = run(TM.DeliveryMonitor(), 3.0, lambda t: bend(ti_state=1, ti_active=False, ti_cmd=0.0, steering_pressed=t < 0.5))
  first = next(t for t, o in res if o.phase == "alert")
  assert first >= 0.5 + TM.STEER_QUIET_S + TM.MON_HOLD_S - 0.03, first


def test_i1_assist_off_needs_a_turn_and_discover_only_after_run():
  straight = dict(ti_state=1, ti_active=False, ti_cmd=0.0, la_now=0.1, desired_la=0.1, ahead_la=0.1,
                  ahead_prof=(TM.ASK_LEADS, [0.1] * len(TM.ASK_LEADS)))
  assert not any(o.phase == "alert" for _, o in run(TM.DeliveryMonitor(), 2.0, lambda t: bend(**straight)))
  # DISCOVER (0) from the start of the drive: not "off"; after RUN this drive: it is
  assert not any(o.phase == "alert" for _, o in run(TM.DeliveryMonitor(), 2.0, lambda t: bend(ti_state=0, ti_active=False, ti_cmd=0.0)))
  res = run(TM.DeliveryMonitor(), 2.0, lambda t: bend(ti_state=3 if t < 0.2 else 0, ti_active=t < 0.2, ti_cmd=0.0))
  assert any(o.phase == "alert" and o.why.startswith("assist off") for _, o in res)


def test_i1_need_never_below_the_floor():
  res = run(TM.DeliveryMonitor(), 3.0, lambda t: bend(mph=28.0, ahead_need=20))
  assert min(o.need_mph for _, o in res if o.phase == "alert") == TM.MIN_NEED_MPH
  res = run(TM.DeliveryMonitor(), 3.0, lambda t: bend(mph=50.0, ahead_need=30, floor_mph=40))
  assert min(o.need_mph for _, o in res if o.phase == "alert") == 40


@pytest.mark.parametrize("why,code", [("assist off", 1), ("assist off: 0x11 stuck request", 2), ("dropout", 3), ("limit", None)])
def test_i1_dropout_text_is_not_a_corner_speed(why, code):
  import time as _t
  class M:
    def __init__(self, d):
      self.d = d
    def get(self, k):
      return self.d.get(k)
  now = _t.monotonic()
  m = M({C.PARAM_DELIVER: json.dumps({"m": now, "need": 40, "why": why}).encode()})
  need, _ = C.read_deliver_alert(m, now)
  val, _ = C.read_curve_code(m, now)
  if code is None:
    assert need == 40 and val is None
  else:
    assert need is None and val == C.alert_code_value(code)


def _run_logged(frames, per_frame, steer=0.3):
  """CarController.update for `frames` frames with curve slowing on and a quiet plan; per_frame(k, CS) sets the car state; the
  ti_curve records logged."""
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  mod = _cost_mod()
  cc, mem = mod.make_controller()
  clock = {"k": 0}
  saved = (ccm.time, ccm.ti_curve_log)
  ccm.time = NS(monotonic=lambda: clock["k"] * 0.01, time=lambda: 1.0e9 + clock["k"] * 0.01, perf_counter=lambda: 0.0)
  logged = []
  ccm.ti_curve_log = logged.append
  try:
    CC, CS = mod.inputs(45.0, 0.0003, steer=steer)
    CS.lat_accel, CS.lkas_effective, CS.lkas_blocked = 0.2, 100.0, False
    CS.out.steeringRateDeg = 0.0
    toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
    for k in range(frames):
      clock["k"] = k
      if k % 20 == 0:
        mem.put(C.PARAM_ON, b"1")
        mem.put(C.PARAM_PLAN, json.dumps({"m": k * 0.01, "seq": k, "mode": "act", "ceil": None, "conf": 1.0, "trusted": True,
                                          "known": True, "why": "", "dist": None, "past": True, "model_la": 0.1, "model_k": 0.0003,
                                          "capped": False, "limit": None, "kind": "", "corner": None, "ahead": 0.2,
                                          "ask": [0.2] * 6, "alert": None}))
      if k % 10 == 3:
        CS.crz_btns_counter = (CS.crz_btns_counter + 1) % 16
      per_frame(k, CS)
      cc.update(CC, CS, k * 10_000_000, toggles)
  finally:
    ccm.time, ccm.ti_curve_log = saved
  return logged


def test_deliver_phase_logged_when_the_assist_turns_on_or_off_not_on_every_torque_count():
  # review 2026-11-25: lkas_effective is a torque count that changes almost every frame above the stock assist speed; compared raw,
  # the "on change" record logged at a steady 10 Hz (~180 KB/min). Only its crossing zero (the assist acting or not) is state.
  def varying(k, CS):
    CS.lkas_effective = 60.0 + 37.0 * math.sin(k * 0.37)           # never zero
  recs = [e for e in _run_logged(1000, varying) if e.get("kind") == "deliver_phase"]
  assert len(recs) == 1 and len({r["phase"] for r in recs}) == 1, recs

  def crossing(k, CS):
    CS.lkas_effective = 0.0 if k >= 600 else 60.0 + 37.0 * math.sin(k * 0.37)
  recs = [e for e in _run_logged(1000, crossing) if e.get("kind") == "deliver_phase"]
  assert len(recs) == 2 and recs[1]["lkas_effective"] == 0.0, recs

