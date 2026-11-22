#!/usr/bin/env python3
"""On-device cost check of the car-side TI code, for the install (run on the PARKED comma; not part of the app, needs no display):

    python3 selfdrive/car/mazda/ti_cost_check.py [--cycles 3000] [--budget-us 300]

Times CarController.update (the real modules: carcontroller, ti_ident, ti_tests, ti_plan, ti_dither, ti_lag, ti_cruise_check,
mazdacan) for a few thousand simulated 10 ms cycles in four modes, with an in-memory params store (nothing is read from or written to
the device's params, no CAN is sent, no message queue is opened):
  plan off           TiPlanOn off: the controller as without the tests
  plan on, ab        the plan in its A/B mode: the tests held
  tests mode, idle   the plan in its tests mode, the car in a test window, the road too curved for any test: the scheduler runs, no signal
  tests mode, run    the plan in its tests mode on a straight road in a window: a test signal runs
Then the controls side (controlsd's TiAbRunner.step, the A/B alternation and the plan) in three modes: plan off, plan on in its A/B mode
with a block running, plan on in its tests mode. Prints mean and p99 microseconds per cycle and the cost each mode adds to the plan off. Exits 1 when a tests mode adds more than the
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


def run_controls(name, plan_on, mph, cycles):
  """controlsd's side: TiAbRunner.step per cycle (the A/B alternation and the plan), with the friction relay set up."""
  from openpilot.selfdrive.car.mazda import ti_ab, ti_plan as tp
  clock = [time.time()]
  fake = NS(time=lambda: clock[0], monotonic=lambda: clock[0], perf_counter=time.perf_counter, perf_counter_ns=time.perf_counter_ns)
  real = (ti_ab.time, getattr(tp, "time", None))
  ti_ab.time = fake                                     # the plan's dwell and hand-over run on simulated 10 ms cycles, not this loop's speed
  if real[1] is not None:
    tp.time = fake
  try:
    return _run_controls(name, plan_on, mph, cycles, clock)
  finally:
    ti_ab.time = real[0]
    if real[1] is not None:
      tp.time = real[1]


def _run_controls(name, plan_on, mph, cycles, clock):
  from openpilot.selfdrive.car.mazda import ti_ab, ti_plan as tp
  warm = 2000
  disk = MemParams({"TiAbConfig": json.dumps({"setting": "LatNoFrictionRelay", "b": True, "block_min": 4})})
  mem = MemParams({"TiPlanOn": b"0"})
  rn = ti_ab.TiAbRunner(disk, mem)                     # controls starts with the plan off; START comes after it is running
  toggles = NS(lat_no_friction_relay=False, steerKp=[[0.0], [1.0]])
  times = []
  for k in range(cycles + warm):
    clock[0] += 0.01
    if k == 50:
      mem.put_bool("TiPlanOn", plan_on)
    if plan_on and k % 10 == 0:                         # the car side's acknowledgement: quiet, as when no test signal runs
      raw = mem.get("TiPlan", encoding="utf8")
      if raw:
        nonce, seq = tp.plan_ident(raw)
        mem.put("TiPlanAck", json.dumps(tp.make_ack(nonce, seq, False, 0, 0.0, clock[0])))
    t0 = time.perf_counter_ns()
    rn.step(toggles, engaged=True, v_ego=mph * 0.44704, curvature=0.0003, steer=0.03, saturated=False, steering_pressed=False,
            driver_torque=0.0, lane_change=False, blinker=False, ti_ok=True, fault=False, controller_ok=True, mono_ns=k * 10_000_000)
    if k >= warm:
      times.append((time.perf_counter_ns() - t0) / 1000.0)
  times.sort()
  return {"mode": name, "mean_us": round(statistics.fmean(times), 1), "p99_us": round(times[int(0.99 * (len(times) - 1))], 1),
          "max_us": round(times[-1], 1), "ab_state": rn.ab.mode, "plan_mode": rn.plan.mode, "why": (rn.ab.message or rn.plan.why)[:80]}


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
  ctl = [run_controls("controls: plan off", False, 45.0, a.cycles),
         run_controls("controls: plan on, ab", True, 45.0, a.cycles),
         run_controls("controls: plan on, tests", True, 70.0, a.cycles)]
  for r in ctl:
    r["added_mean_us"] = round(r["mean_us"] - ctl[0]["mean_us"], 1)
    print(f"{r['mode']:26s} mean {r['mean_us']:8.1f} us  p99 {r['p99_us']:8.1f} us  added {r['added_mean_us']:7.1f} us"
          f"  (A/B {r['ab_state']}, plan {r['plan_mode']})")
    if r["added_mean_us"] > a.budget_us:
      bad.append(f"{r['mode']} adds {r['added_mean_us']} us per cycle (budget {a.budget_us:g})")
  print(json.dumps({"results": res, "controls": ctl, "budget_us": a.budget_us, "seconds": round(time.time() - t0, 1)}))
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
