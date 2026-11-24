#!/usr/bin/env python3
"""On-device cost check of the car-side TI code, for the install (run on the PARKED comma; not part of the app, needs no display):

    python3 selfdrive/car/mazda/ti_cost_check.py [--cycles 3000] [--budget-us 300]

Times CarController.update (the real modules: carcontroller, ti_ident, ti_tests, ti_plan, ti_dither, ti_lag, ti_cruise_check,
ti_curve, mazdacan) for a few thousand simulated 10 ms cycles in four modes, with an in-memory params store (nothing is read from or written to
the device's params, no CAN is sent, no message queue is opened):
  plan off           TiPlanOn off: the controller as without the tests
  plan on, ab        the plan in its A/B mode: the tests held
  tests mode, idle   the plan in its tests mode, the car in a test window, the road too curved for any test: the scheduler runs, no signal
  tests mode, run    the plan in its tests mode on a straight road in a window: a test signal runs
Then curve slowing (ti_curve.py, plan off) in three modes: off, on with no corner, on and tapping (exit 1 over budget, 2 when no tap
was sent in the working mode). Then the controls side (controlsd's TiAbRunner.step, the A/B alternation and the plan) in three modes: plan off, plan on in its A/B mode
with a block running, plan on in its tests mode. Then controlsd's lateral controller (plan off) with its scalar helpers in plain
Python and, for comparison, with the numpy calls they replaced; the two must give bit-identical outputs (exit 1 otherwise). Prints mean and p99 microseconds per cycle and the cost each mode adds to the plan off. Exits 1 when a tests mode adds more than the
budget on average (default 300 us per cycle), 2 when the measurement is not valid (the running mode never ran a test).

Budget: the drives of 2026-11-22 left about 1.5 ms per 10 ms cycle free on the controls core with the plan off (card about 32 % of the
core, controlsd about 56 %), and a lag soft-disable needs the 1-second mean above 11.1 ms. The TI code's added cost in any mode should
therefore stay well under 0.5 ms per cycle; 0.3 ms leaves room for the jitter of a busy core. Runs in well under a minute.
"""
import argparse
import json
import statistics
import sys
import time
from types import SimpleNamespace as NS


class MemParams:
  """The Params API subset the car controller uses, in memory."""

  def __init__(self, d=None):
    self.d = dict(d or {})

  def get(self, k, block=False, encoding=None):
    v = self.d.get(k)
    if v is None:
      return None
    if isinstance(v, bytes):
      return v.decode(encoding) if encoding else v
    return v if encoding else str(v).encode()

  def get_bool(self, k, block=False):
    v = self.d.get(k)
    return v in (True, b"1", "1")

  def get_int(self, k, block=False):
    try:
      return int(self.d.get(k) or 0)
    except (TypeError, ValueError):
      return 0

  def put(self, k, v):
    self.d[k] = v

  put_nonblocking = put

  def put_bool(self, k, v):
    self.d[k] = b"1" if v else b"0"

  put_bool_nonblocking = put_bool

  def remove(self, k):
    self.d.pop(k, None)

  def check_key(self, k):
    return True


def make_controller():
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  from openpilot.selfdrive.car.mazda.values import MazdaFlags
  stores = {}

  def params(path=None):
    key = "mem" if path and "shm" in str(path) else "disk"
    stores.setdefault(key, MemParams())
    return stores[key]
  ccm.Params = params                                   # this process only: the device's params are never touched
  # One car controller per measurement, as on the car: ControlsTimer keeps every timer ever made in a class list that its tick walks
  # each cycle, so the timers of controllers made earlier in this process (other modes, other tests) are dropped first.
  from openpilot.common.realtime import ControlsTimer
  ControlsTimer.objects.clear()
  CP = NS(flags=int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR), carFingerprint="MAZDA_CX5", carName="mazda")
  cc = ccm.CarController("mazda_2017", CP, None)

  class Packer:                                         # no CAN is built or sent
    def make_can_msg(self, *a, **k):
      return (0, 0, b"\0" * 8, 0)
  cc.packer = Packer()
  return cc, stores["mem"]


def inputs(mph, curvature, steer=0.03):
  from openpilot.selfdrive.car.mazda.values import TI_STATE
  act = NS(steer=steer, curvature=curvature, accel=0.0)
  act.as_builder = lambda a=act: NS(steer=a.steer, curvature=a.curvature, steerOutputCan=0, accel=0.0)
  CC = NS(latActive=True, enabled=True, actuators=act, leftBlinker=False, rightBlinker=False, longActive=False,
          cruiseControl=NS(cancel=False, resume=False, override=False), hudControl=NS(visualAlert=0))
  v = mph * 0.44704
  out = NS(vEgo=v, steeringPressed=False, steeringTorque=-1.0, leftBlinker=False, rightBlinker=False, standstill=False,
           brakePressed=False, gasPressed=False, steeringAngleDeg=0.0, cruiseState=NS(enabled=True, available=True, speed=v))
  from collections import defaultdict
  CS = NS(out=out, ti_lkas_allowed=True, ti_state=TI_STATE.RUN, ti_violation=0, ti_ramp_down=False, ti_version=1, eps_torque_sensor=0,
          lkas_blocked=mph < 28.0, cam_lkas=defaultdict(int), cam_laneinfo=defaultdict(int), lkas_allowed_speed=True, crz_btns_counter=0,
          crz_btns_driver=False, crz_lead=False, crz_info={}, acc={})
  return CC, CS


def run_mode(name, plan_mode, mph, curvature, cycles):
  from openpilot.selfdrive.car.mazda import ti_plan as tp
  cc, mem = make_controller()
  toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
  CC, CS = inputs(mph, curvature)
  times, ran = [], 0
  for k in range(cycles + 300):
    if k % 50 == 0:
      mem.put_bool("TiPlanOn", plan_mode is not None)
      if plan_mode is not None:
        mem.put("TiPlan", tp.encode_plan(tp.PlanOut(plan_mode, plan_mode == "ab", "", "", 1), time.time(), False, "costcheck", True))
    t0 = time.perf_counter_ns()
    cc.update(CC, CS, k * 10_000_000, toggles)
    dt = time.perf_counter_ns() - t0
    if k >= 300:                                        # after the warm-up (accountant load, run-all's first picks)
      times.append(dt / 1000.0)
      ident = getattr(cc, "ti_ident", None)
      ran += bool(ident is not None and ident.running)
  times.sort()
  return {"mode": name, "mean_us": round(statistics.fmean(times), 1), "p99_us": round(times[int(0.99 * (len(times) - 1))], 1),
          "max_us": round(times[-1], 1), "test_running_share": round(ran / len(times), 3)}


def run_curve(name, mode, cycles, count=False, warm=300):
  """Curve slowing (ti_curve.py) in the car controller, plan off: "off" (the switch off: the 10 Hz read only), "on, no corner" (on, the
  planner says no corner ahead), "on, working" (on, a corner ceiling 15 mph below the set speed that is lifted and given again every
  10 s, so taps are sent and read back all along). The planner's output is written to the in-memory store as the planner would (5 Hz).
  count: calls per cycle (as count_mode) instead of times."""
  from openpilot.selfdrive.car.mazda import ti_curve as tc
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  cc, mem = make_controller()
  seen = {"taps": 0}
  ccm.ti_curve_log = lambda ev: seen.__setitem__("taps", seen["taps"] + (ev.get("kind") == "tap"))   # this process only
  clock = {"k": 0}
  # the cycles run back to back here, faster than 10 ms apart: the car controller's monotonic clock is the simulated one (the tap
  # machinery enforces its gaps on real time too, which would otherwise hold every tap after the first)
  ccm.time = NS(monotonic=lambda: clock["k"] * 0.01, time=time.time, perf_counter=time.perf_counter)
  toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
  CC, CS = inputs(55.0, 0.0003)
  cs = CS.out.cruiseState
  times, py, c = [], 0, 0
  for k in range(cycles + warm):
    clock["k"] = k
    if k % 20 == 0:
      mem.put(tc.PARAM_ON, b"1" if mode != "off" else b"0")
      corner = mode == "on, working" and (k // 1000) % 2 == 0
      mem.put(tc.PARAM_PLAN, json.dumps({"m": k * 0.01, "seq": k, "mode": "act", "ceil": 40 if corner else None, "conf": 1.0,
                                         "trusted": True, "known": True, "why": "", "dist": 100.0, "past": not corner,
                                         "model_la": 0.1, "model_k": 0.0003, "capped": False, "limit": None, "kind": "corner",
                                         "corner": 40 if corner else None}))
    if k % 10 == 3:
      CS.crz_btns_counter = (CS.crz_btns_counter + 1) % 16
    cv = cc.curve
    if cv is not None and cv.tap_to is not None and cv.tap_first is not None and k - cv.tap_first >= 8:
      cs.speed = cv.tap_to * 0.44704                    # the car reads the step back 80 ms after the tap
      CS.out.vEgo = cs.speed
    if count and k >= warm:
      _, p_, c_ = _counting(lambda: cc.update(CC, CS, k * 10_000_000, toggles))
      py, c = py + p_, c + c_
    else:
      t0 = time.perf_counter_ns()
      cc.update(CC, CS, k * 10_000_000, toggles)
      dt = time.perf_counter_ns() - t0
      if k >= warm:
        times.append(dt / 1000.0)
  ccm.time = time
  if count:
    return {"mode": name, "py": round(py / cycles, 1), "c": round(c / cycles, 1), "total": round((py + c) / cycles, 1), "taps": seen["taps"]}
  times.sort()
  return {"mode": name, "mean_us": round(statistics.fmean(times), 1), "p99_us": round(times[int(0.99 * (len(times) - 1))], 1),
          "max_us": round(times[-1], 1), "taps": seen["taps"], "state": cc.curve.state if cc.curve is not None else "absent"}


def run_monitor(name, cycles, warm=300):
  """Curve slowing stage 3 (plan T7): the car controller's per-frame delivery monitor (CarController._curve_monitor, on the sent
  command), curve slowing on, through bends that saturate (the command at its maximum, the angle stalled, the bend asking more) and
  release, so it watches, alerts, clears and re-arms all along. Times only the monitor's own call."""
  from openpilot.selfdrive.car.mazda import ti_curve as tc
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  cc, mem = make_controller()
  ccm.ti_curve_log = lambda ev: None
  clock = {"k": 0}
  ccm.time = NS(monotonic=lambda: clock["k"] * 0.01, time=time.time, perf_counter=time.perf_counter)
  CC, CS = inputs(45.0, -0.004)
  CS.lat_accel, CS.lkas_effective = 1.0, 100.0
  CS.out.steeringRateDeg = 0.0
  cc.curve_on = True
  times, alerts = [], 0
  for k in range(cycles + warm):
    clock["k"] = k
    phase = (k // 400) % 3                    # 4 s at the limit, 8 s free
    if k % 20 == 0:
      cc.curve_plan = {"m": k * 0.01, "ahead": 2.0 + 0.01 * (k // 20 % 3), "ask": [2.0] * 6, "mode": "act"}
    ti = 600 if phase == 0 else 200
    t0 = time.perf_counter_ns()
    r = cc._curve_monitor(CC, CS, ti)
    dt = time.perf_counter_ns() - t0
    alerts += r.phase == "alert"
    if k >= warm:
      times.append(dt / 1000.0)
  ccm.time = time
  times.sort()
  return {"mode": name, "mean_us": round(statistics.fmean(times), 1), "p99_us": round(times[int(0.99 * (len(times) - 1))], 1),
          "max_us": round(times[-1], 1), "alert_share": round(alerts / (cycles + warm), 3), "written": tc.PARAM_DELIVER in mem.d}


def _counting(fn):
  """(result, python calls, C calls) of fn() under sys.setprofile: the operation count, deterministic for one Python version."""
  n = [0, 0]

  def prof(frame, event, arg):
    if event == "call":
      n[0] += 1
    elif event == "c_call":
      n[1] += 1
  sys.setprofile(prof)
  try:
    out = fn()
  finally:
    sys.setprofile(None)
  return out, n[0], n[1]


def count_mode(plan_mode, mph, curvature, cycles=200, warm=300):
  """Calls per car-controller cycle in one mode, after the warm-up: {"py": Python function calls, "c": builtin calls, "total"}."""
  from openpilot.selfdrive.car.mazda import ti_plan as tp
  cc, mem = make_controller()
  toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
  CC, CS = inputs(mph, curvature)
  py = c = 0
  for k in range(warm + cycles):
    if k % 50 == 0:
      mem.put_bool("TiPlanOn", plan_mode is not None)
      if plan_mode is not None:
        mem.put("TiPlan", tp.encode_plan(tp.PlanOut(plan_mode, plan_mode == "ab", "", "", 1), time.time(), False, "costcheck", True))
    if k < warm:
      cc.update(CC, CS, k * 10_000_000, toggles)
    else:
      _, p_, c_ = _counting(lambda: cc.update(CC, CS, k * 10_000_000, toggles))
      py, c = py + p_, c + c_
  return {"py": round(py / cycles, 1), "c": round(c / cycles, 1), "total": round((py + c) / cycles, 1),
          "test_running": bool(getattr(cc, "ti_ident", None) is not None and cc.ti_ident.running)}


def run_controls(name, plan_on, mph, cycles, count=False):
  """controlsd's side: TiAbRunner.step per cycle (the A/B alternation and the plan), with the friction relay set up. count: the calls
  per step (as count_mode) instead of the times."""
  from openpilot.selfdrive.car.mazda import ti_ab, ti_plan as tp
  clock = [time.time()]
  fake = NS(time=lambda: clock[0], monotonic=lambda: clock[0], perf_counter=time.perf_counter, perf_counter_ns=time.perf_counter_ns)
  real = (ti_ab.time, getattr(tp, "time", None))
  ti_ab.time = fake                                     # the plan's dwell and hand-over run on simulated 10 ms cycles, not this loop's speed
  if real[1] is not None:
    tp.time = fake
  try:
    return _run_controls(name, plan_on, mph, cycles, clock, count)
  finally:
    ti_ab.time = real[0]
    if real[1] is not None:
      tp.time = real[1]


def _run_controls(name, plan_on, mph, cycles, clock, count=False):
  from openpilot.selfdrive.car.mazda import ti_ab, ti_plan as tp
  warm = 2000
  disk = MemParams({"TiAbConfig": json.dumps({"setting": "LatNoFrictionRelay", "b": True, "block_min": 4})})
  mem = MemParams({"TiPlanOn": b"0"})
  rn = ti_ab.TiAbRunner(disk, mem)                     # controls starts with the plan off; START comes after it is running
  toggles = NS(lat_no_friction_relay=False, steerKp=[[0.0], [1.0]])
  times, calls = [], [0, 0]
  for k in range(cycles + warm):
    clock[0] += 0.01
    if k == 50:
      mem.put_bool("TiPlanOn", plan_on)
    if plan_on and k % 10 == 0:                         # the car side's acknowledgement: quiet, as when no test signal runs
      raw = mem.get("TiPlan", encoding="utf8")
      if raw:
        nonce, seq = tp.plan_ident(raw)
        mem.put("TiPlanAck", json.dumps(tp.make_ack(nonce, seq, False, 0, 0.0, clock[0])))
    def step(k=k):
      rn.step(toggles, engaged=True, v_ego=mph * 0.44704, curvature=0.0003, steer=0.03, saturated=False, steering_pressed=False,
              driver_torque=0.0, lane_change=False, blinker=False, ti_ok=True, fault=False, controller_ok=True, mono_ns=k * 10_000_000)
    if count and k >= warm:
      _, p_, c_ = _counting(step)
      calls[0], calls[1] = calls[0] + p_, calls[1] + c_
      continue
    t0 = time.perf_counter_ns()
    step()
    if k >= warm:
      times.append((time.perf_counter_ns() - t0) / 1000.0)
  if count:
    return {"mode": name, "py": round(calls[0] / cycles, 1), "c": round(calls[1] / cycles, 1), "total": round(sum(calls) / cycles, 1),
            "ab_state": rn.ab.mode, "plan_mode": rn.plan.mode}
  times.sort()
  return {"mode": name, "mean_us": round(statistics.fmean(times), 1), "p99_us": round(times[int(0.99 * (len(times) - 1))], 1),
          "max_us": round(times[-1], 1), "ab_state": rn.ab.mode, "plan_mode": rn.plan.mode, "why": (rn.ab.message or rn.plan.why)[:80]}


def run_lateral(name, use_numpy, cycles, seed=7):
  """controlsd's lateral controller (LatControlTorque with the Mazda TI plant, plan off) per cycle, on a seeded drive-like input
  sequence. use_numpy: the scalar helpers (scalar_np's clip / sign / interp) swapped back for the numpy calls they replaced, i.e. the
  controller as before that change. Also returns the outputs' exact bits, so the two modes can be checked identical on the device."""
  import math
  import random
  from cereal import car, log
  import numpy as np
  from openpilot.selfdrive.car.mazda import lateral_plant as lp_mod
  from openpilot.selfdrive.controls.lib import latcontrol_torque as ltq
  from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
  saved = {(m, k): getattr(m, k) for m, k in ((ltq, "_clip"), (ltq, "_sign"), (ltq, "_LOW_SPEED"), (lp_mod, "_clip"), (lp_mod, "_sign"),
                                               (lp_mod, "_Q_OF_V"), (lp_mod, "_GS_OF_V"))}
  if use_numpy:
    ltq._clip = lp_mod._clip = lambda x, lo, hi: np.clip(x, lo, hi)
    ltq._sign = lp_mod._sign = np.sign
    ltq._LOW_SPEED = lambda v: np.interp(v, ltq.LOW_SPEED_X, ltq.LOW_SPEED_Y)
    lp_mod._Q_OF_V = lambda v: float(np.interp(v, lp_mod.V_BP, lp_mod.Q_BP))
    lp_mod._GS_OF_V = lambda v: float(np.interp(v, lp_mod.V_BP, lp_mod.GS_BP))
  try:
    CP = car.CarParams.new_message()
    CP.steerLimitTimer, CP.mass, CP.rotationalInertia, CP.wheelbase = 0.4, 1909.0, 2813.0, 2.7
    CP.centerToFront, CP.steerRatio, CP.tireStiffnessFront, CP.tireStiffnessRear = 2.7 * 0.41, 15.5, 80000.0, 90000.0
    lt = CP.lateralTuning
    lt.init("torque")
    lt.torque.kp, lt.torque.ki, lt.torque.friction, lt.torque.latAccelFactor = 0.8, 0.2, 0.1, 2.0
    lt.torque.useSteeringAngle = True
    ci = NS(lateral_plant=lp_mod.TiLateralPlant(600.0, 0.01), torque_from_lateral_accel=lambda: (lambda la, tp: la / tp.latAccelFactor),
            lateral_accel_from_torque=lambda: (lambda t, tp: t * tp.latAccelFactor))
    ctrl = ltq.LatControlTorque(CP, ci, 0.01)
    VM = VehicleModel(CP)
    params = log.LiveParametersData.new_message()
    toggles = NS(lat_damping=True, lat_commit_setpoint=True, lat_friction_comp=True, lat_output_filter=False, lat_no_friction_relay=True,
                 ti_steer_max=600.0)
    r = random.Random(seed)
    CS = car.CarState.new_message()
    v, curv, times, out = 20.0, 0.0, [], []
    for k in range(cycles + 300):
      v = min(max(v + r.gauss(0, 0.05), 8.0), 32.0)
      curv = min(max(curv + r.gauss(0, 1e-4), -0.01), 0.01)
      CS.vEgo = v
      CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-curv * (1 + r.gauss(0, 0.05)), v, 0.0))
      CS.steeringRateDeg = r.gauss(0, 5)
      fp = NS(lkasBlocked=v < 12.5, lkasEffective=r.uniform(0, 300), tiActive=True, columnTorque=r.gauss(0, 20))
      t0 = time.perf_counter_ns()
      steer, _, pid_log = ctrl.update(True, CS, VM, params, False, curv, False, 0.25, None, None, toggles, fp)
      dt = time.perf_counter_ns() - t0
      if k >= 300:
        times.append(dt / 1000.0)
        out.append((float(steer).hex(), pid_log.to_bytes()))
  finally:
    for (m, k), v in saved.items():
      setattr(m, k, v)
  times.sort()
  return {"mode": name, "mean_us": round(statistics.fmean(times), 1), "p99_us": round(times[int(0.99 * (len(times) - 1))], 1),
          "max_us": round(times[-1], 1)}, out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--cycles", type=int, default=3000)
  ap.add_argument("--budget-us", type=float, default=300.0)
  a = ap.parse_args()
  t0 = time.time()
  res = [run_mode("plan off", None, 38.0, 0.0003, a.cycles),
         run_mode("plan on, ab", "ab", 45.0, 0.0003, a.cycles),
         run_mode("tests mode, idle", "tests", 38.0, 0.02, a.cycles),
         run_mode("tests mode, run", "tests", 38.0, 0.0003, a.cycles)]
  base = res[0]["mean_us"]
  bad = []
  for r in res:
    r["added_mean_us"] = round(r["mean_us"] - base, 1)
    print(f"{r['mode']:18s} mean {r['mean_us']:8.1f} us  p99 {r['p99_us']:8.1f} us  max {r['max_us']:8.1f} us  added {r['added_mean_us']:7.1f} us"
          f"  test running {100 * r['test_running_share']:.0f} %")
    if r["mode"].startswith("tests") and r["added_mean_us"] > a.budget_us:
      bad.append(f"{r['mode']} adds {r['added_mean_us']} us per cycle (budget {a.budget_us:g})")
  cur = [run_curve("curve: off", "off", a.cycles), run_curve("curve: on, no corner", "on, no corner", a.cycles),
         run_curve("curve: on, working", "on, working", max(a.cycles, 4000))]
  for r in cur:
    r["added_mean_us"] = round(r["mean_us"] - base, 1)
    print(f"{r['mode']:22s} mean {r['mean_us']:8.1f} us  p99 {r['p99_us']:8.1f} us  max {r['max_us']:8.1f} us  added {r['added_mean_us']:7.1f} us"
          f"  taps {r['taps']}  state {r['state']}")
    if r["added_mean_us"] > a.budget_us:
      bad.append(f"{r['mode']} adds {r['added_mean_us']} us per cycle (budget {a.budget_us:g})")
  if cur[2]["taps"] < 10:
    print("NOT VALID: curve slowing sent no taps in its working mode")
    return 2
  mon = run_monitor("curve: monitor", max(a.cycles, 4000))      # stage 3 (plan T7): its own cost, budget per cycle
  print(f"{mon['mode']:22s} mean {mon['mean_us']:8.1f} us  p99 {mon['p99_us']:8.1f} us  max {mon['max_us']:8.1f} us"
        f"  alerting {100 * mon['alert_share']:.0f} %")
  cur.append(mon)
  if mon["mean_us"] > a.budget_us:
    bad.append(f"{mon['mode']} costs {mon['mean_us']} us per cycle (budget {a.budget_us:g})")
  if mon["alert_share"] <= 0.0 or not mon["written"]:
    print("NOT VALID: the monitor never alerted")
    return 2
  ctl = [run_controls("controls: plan off", False, 45.0, a.cycles),
         run_controls("controls: plan on, ab", True, 45.0, a.cycles),
         run_controls("controls: plan on, tests", True, 70.0, a.cycles)]
  for r in ctl:
    r["added_mean_us"] = round(r["mean_us"] - ctl[0]["mean_us"], 1)
    print(f"{r['mode']:26s} mean {r['mean_us']:8.1f} us  p99 {r['p99_us']:8.1f} us  added {r['added_mean_us']:7.1f} us"
          f"  (A/B {r['ab_state']}, plan {r['plan_mode']})")
    if r["added_mean_us"] > a.budget_us:
      bad.append(f"{r['mode']} adds {r['added_mean_us']} us per cycle (budget {a.budget_us:g})")
  lat = []
  try:
    new, out_new = run_lateral("lateral controller (plain-Python helpers)", False, a.cycles)
    old, out_old = run_lateral("lateral controller (numpy helpers, as before)", True, a.cycles)
    same = out_new == out_old
    for r in (old, new):
      print(f"{r['mode']:46s} mean {r['mean_us']:8.1f} us  p99 {r['p99_us']:8.1f} us")
    print(f"lateral controller: saved {old['mean_us'] - new['mean_us']:.1f} us per cycle; outputs bit-identical: {same}")
    lat = [old, new, {"identical": same}]
    if not same:
      bad.append("the lateral controller's outputs differ between the numpy and the plain-Python helpers")
  except Exception as e:                                     # the car-side numbers stand without it
    print(f"lateral controller not timed: {type(e).__name__}: {e}")
  print(json.dumps({"results": res, "curve": cur, "controls": ctl, "lateral": lat, "budget_us": a.budget_us, "seconds": round(time.time() - t0, 1)}))
  if res[3]["test_running_share"] < 0.2:
    print("NOT VALID: no test signal ran in the running mode")
    return 2
  if bad:
    print("OVER BUDGET: " + "; ".join(bad))
    return 1
  print("WITHIN BUDGET")
  return 0


if __name__ == "__main__":
  sys.exit(main())
