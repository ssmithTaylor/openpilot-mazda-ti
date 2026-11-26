#!/usr/bin/env python3
"""Per-frame memory and cost guard for the code this fork runs every frame (inventory: tools/dev/PER_FRAME_INVENTORY.md).

    python3 selfdrive/car/mazda/ti_mem_check.py [--quick] [--only NAME ...] [--json]

Not part of the app: no process imports it. It drives each per-frame entry point with a varied, repeating synthetic input
sequence (bends starting / released / passed, alerts raised and cleared, driver presses, plan gaps, lock loss, a changing map list,
lane lines appearing and disappearing, lane changes, the test features starting and finishing) and measures, with the cycle
collector OFF as card and controlsd run (common/realtime.py config_realtime_process -> gc.disable()):

  a. cyclic garbage: objects gc.collect() finds after the measured windows (warm-up excluded): must be at most `max_cycles` (0);
  b. retained growth: traced memory (tracemalloc) grown over window 1 and window 2 of the same length; window 2 must grow under
     1 % of window 1 plus SLACK_B, and under MAX_MB_PER_H at the entry point's real rate;
  c. cost growth: median per-frame time of window 2 within COST_RATIO of window 1 (reported as noisy, not failed, when the
     chunks of one window disagree by more than NOISY_SPREAD).

Each window is a whole number of the scenario's input periods, so window 2 is as busy as window 1. Nothing is read from or written
to the device's params (in-memory stores), no message queue is opened, no CAN is sent. The full set runs in about 20-30 s on the
comma with --quick (the install check, tools/dev/comma_checks.sh); the PC tests (selfdrive/car/tests/test_no_per_frame_leaks.py)
run the same scenarios. Exit 1 when any scenario fails a or b, 0 otherwise.
"""
import argparse
import gc
import json
import math
import statistics
import sys
import time
import tracemalloc
from types import SimpleNamespace as NS

MAX_MB_PER_H = 5.0        # retained growth that fails a scenario, at the entry point's own rate
SLACK_B = 16384           # bytes of window-2 growth always allowed (allocator noise; the in-memory params store keeps the latest JSON
                          # values, whose length changes with the state)
GROWTH_FRAC = 0.01        # window 2 may grow by this share of window 1's growth on top of the slack
COST_RATIO = 1.5          # window 2 median per-frame time over window 1's
NOISY_SPREAD = 0.5        # (p75 - p25) / median of the pairwise chunk ratios above this: timing too noisy to judge
MIN_US = 2.0              # per-frame times under this are not judged (timer resolution)


# ---- the measurement ---------------------------------------------------------------------------------------------------------------
def _window(step, start, n, chunk):
  per = []
  i = start
  while i < start + n:
    m = min(chunk, start + n - i)
    t0 = time.perf_counter()
    for j in range(i, i + m):
      step(j)
    per.append((time.perf_counter() - t0) * 1e6 / m)
    i += m
  return per


def _spread(xs):
  if len(xs) < 4:
    return 0.0
  q = statistics.quantiles(xs, n=4)
  med = statistics.median(xs)
  return (q[2] - q[0]) / med if med > 0 else 0.0


def _settled_traced():
  """Traced bytes after a full collection with DEBUG_SAVEALL. The collection frees nothing the code made: every unreachable object
  (a cycle) is moved to gc.garbage, so it stays traced as growth and is counted as cyclic garbage. What it does return is the
  interpreter's free lists (CPython 3.11, the comma's Python: up to 2000 freed tuples per size, 100 floats, 80 lists / dicts ...
  keep their tracemalloc trace at the line that first allocated them). Those are bounded by the interpreter, yet fill one entry per
  frame for a while (2026-11-25 install: 48 B 1-tuples at carcontroller.py:1485 from pycapnp 1.3's new_message; 88 B 6-tuples at
  ti_curve_plan.py:1094 from tuple(genexpr), flat at 176,000 B after 2000 ticks) and read as growth in a one-period window."""
  gc.set_debug(gc.DEBUG_SAVEALL)
  gc.collect()
  return tracemalloc.get_traced_memory()[0]


def measure(step, warm, n, rate_hz, max_cycles=0, chunks=8):
  """Run step(i) for i in [0, warm) (not measured), then two windows of n frames with gc off and tracemalloc on; each window's
  growth is read after _settled_traced (free lists emptied, cycles kept and counted).
  -> dict with the cyclic garbage, both windows' growth, per-frame growth, MB/h, the cost ratio and a list of failures."""
  was = gc.isenabled()
  gc.collect()
  gc.disable()
  tracing = tracemalloc.is_tracing()
  garbage_info, top = [], []
  try:
    for i in range(warm):
      step(i)
    gc.collect()                         # cycles made during the warm-up (start-up state, first config read) are not counted
    gc.garbage.clear()
    if not tracing:
      tracemalloc.start(1)
    m0 = _settled_traced()
    chunk = max(1, n // chunks)
    t1 = _window(step, warm, n, chunk)
    m1 = _settled_traced()
    s1 = tracemalloc.take_snapshot()
    t2 = _window(step, warm + n, n, chunk)
    m2 = _settled_traced()
    s2 = tracemalloc.take_snapshot()
    top = [str(d) for d in s2.compare_to(s1, "lineno")[:5] if d.size_diff > 0]
    gc.collect()
    cyc = len(gc.garbage)                # every cycle made in either window (the settles above saved them all)
    if cyc:
      import collections
      where = collections.Counter()
      for o in gc.garbage[:20000]:
        tb = tracemalloc.get_object_traceback(o)
        where[f"{type(o).__name__} @ {tb[0].filename.replace(chr(92), '/').split('/')[-1]}:{tb[0].lineno}" if tb else
              type(o).__name__] += 1
      garbage_info = [f"{k} x{v}" for k, v in where.most_common(6)]
    gc.garbage.clear()
    gc.set_debug(0)
  finally:
    gc.set_debug(0)
    if not tracing:
      tracemalloc.stop()
    if was:
      gc.enable()
  g1, g2 = m1 - m0, m2 - m1
  per_frame = g2 / n
  mb_h = per_frame * rate_hz * 3600 / 1e6
  med1, med2 = statistics.median(t1), statistics.median(t2)
  # chunk j of window 2 is the same stretch of the input period as chunk j of window 1: compare them pairwise
  ratios = [b / a for a, b in zip(t1, t2) if a > 0]
  ratio = statistics.median(ratios) if ratios else 1.0
  noisy = _spread(ratios) > NOISY_SPREAD or med1 < MIN_US
  bound = GROWTH_FRAC * max(g1, 0) + SLACK_B
  fails = []
  if cyc > max_cycles:
    fails.append(f"cyclic garbage {cyc} objects over {2 * n} frames (allowed {max_cycles}): {garbage_info}")
  if g2 > bound:
    fails.append(f"retained growth in window 2 {g2} B > {bound:.0f} B (1 % of window 1's {g1} B + {SLACK_B} B); top: {top}")
  if mb_h > MAX_MB_PER_H:
    fails.append(f"retained growth {per_frame:.1f} B/frame = {mb_h:.2f} MB/h at {rate_hz:g} Hz (limit {MAX_MB_PER_H:g}); top: {top}")
  return {"cyclic": cyc, "garbage": garbage_info, "growth1_B": g1, "growth2_B": g2, "B_per_frame": round(per_frame, 2),
          "MB_per_h": round(mb_h, 3), "us1": round(med1, 1), "us2": round(med2, 1), "cost_ratio": round(ratio, 2), "noisy": noisy, "top": top,
          "cost_fail": (not noisy) and ratio > COST_RATIO, "fails": fails, "frames": 2 * n, "warm": warm}


# The event buffers the code keeps for itself and trims (bounded, verified by reading; their caps are also guarded by
# test_no_per_frame_leaks.py): they take hours to fill on the car, so inside a short window they look like growth. The harness drains
# them as a reader would; an unbounded list anywhere else still shows.
#   ti_ab.TiAb.events       trimmed to 2000 past 4000 (ti_ab.py _event): one heartbeat every HB_S = 10 s, so full after ~11 h
#   ti_ident.TiIdent.events trimmed to 1000 past 2000 (ti_ident.py _event)
BOUNDED_EVENT_BUFFERS = (("ti_ab", "TiAb", 4000), ("ti_ident", "TiIdent", 2000))


def _drain(seen, *objs):
  """Count the drained events' kinds into `seen` (what the scenario exercised), then clear the buffer."""
  for o in objs:
    ev = getattr(o, "events", None) if o is not None else None
    if ev:
      for e in ev:
        k = "ev:" + str(e.get("kind"))
        seen[k] = seen.get(k, 0) + 1
      ev.clear()


def _count(seen, key, n=1):
  seen[key] = seen.get(key, 0) + n


# ---- shared fakes ------------------------------------------------------------------------------------------------------------------
def _lane_model(i, period=6000):
  """A modelV2-like object, new every 5 frames (the 20 Hz model held for 5 control frames), through: centred straight; a right bend
  running wide onto and over the outside line (the model re-indexes the lines); lines fading away; a lane change; a left bend with
  flickering low-confidence lines."""
  ph = i % period
  k = ph // 5
  lcs = 0
  if ph < 1500:
    ly, ry, p, curv = -1.8, 1.8, 0.9, 0.0
  elif ph < 3000:
    f = (ph - 1500) / 1500.0
    ly = -1.8 - 1.8 * f + (3.6 if f > 0.7 else 0.0)
    ry = ly + 3.6
    p, curv = 0.85, 0.004
  elif ph < 3600:
    ly, ry, p, curv = -1.8, 1.8, 0.1 + 0.05 * (k % 3), 0.0
  elif ph < 4200:
    f = (ph - 3600) / 600.0
    ly, ry, p, curv, lcs = -1.8 + 3.0 * f, 1.8 + 3.0 * f, 0.8, 0.0, 1 if f < 0.9 else 0
  else:
    ly, ry, p, curv = -1.2, 2.4, 0.3 + 0.6 * ((k // 7) % 2), -0.003
  xs = [0.0, 5.0, 10.0, 15.0, 20.0, 30.0]
  lines = [NS(x=xs, y=[ly - 3.6] * 6), NS(x=xs, y=[ly + curv * x * x / 2 for x in xs]),
           NS(x=xs, y=[ry + curv * x * x / 2 for x in xs]), NS(x=xs, y=[ry + 3.6] * 6)]
  m = NS(laneLines=lines, laneLineProbs=[0.1, p, p * (0.9 if k % 11 else 0.5), 0.1], laneLineStds=[0.5, 0.1, 0.1, 0.5],
         roadEdges=[NS(x=xs, y=[ly - 4.0] * 6), NS(x=xs, y=[ry + 4.0] * 6)], roadEdgeStds=[0.3, 0.3 if k % 13 else 9.0],
         meta=NS(laneChangeState=lcs), action=NS(desiredCurvature=curv), timestampEof=0)
  return m, curv, lcs


class _ModelFeed:
  def __init__(self, period=6000):
    self.period, self.k, self.cur = period, None, None

  def get(self, i):
    if i // 5 != self.k:
      self.k = i // 5
      self.cur = _lane_model(i, self.period)
    return self.cur


# ---- scenarios -----------------------------------------------------------------------------------------------------------------------
# Each: name -> (setup() -> (step(i), teardown()), rate Hz, period frames, process). Windows are whole periods.
def setup_lane_drift():
  from openpilot.selfdrive.controls.lib import lane_drift as LD
  g = LD.LaneDriftGate(0.01)
  feed = _ModelFeed()
  seen = {}

  def step(i):
    m, curv, lcs = feed.get(i)
    ph = i % 6000
    if 5800 <= ph < 6000:               # controlsd resets the gate while steering is not active
      g.reset()
      return
    at_limit = 1500 <= ph < 3000 or (4200 <= ph and (i // 40) % 3 == 0)
    lc = LD.lane_change_active(m, lcs == 1, False)
    if g.update(m, m.action.desiredCurvature, at_limit, lc):
      _count(seen, "gate open")
    if lc:
      _count(seen, "lane change")
    if g.latch_left > 0:
      _count(seen, "latched")
  step.seen = seen
  return step, lambda: None


def _controls_clock(mods):
  clock = [1_800_000_000.0]
  fake = NS(time=lambda: clock[0], monotonic=lambda: clock[0], perf_counter=time.perf_counter, perf_counter_ns=time.perf_counter_ns,
            monotonic_ns=lambda: int(clock[0] * 1e9), time_ns=lambda: int(clock[0] * 1e9))
  saved = [(m, getattr(m, "time", None)) for m in mods]
  for m in mods:
    if getattr(m, "time", None) is not None:
      m.time = fake

  def restore():
    for m, t in saved:
      if t is not None:
        m.time = t
  return clock, restore


def setup_ab_runner():
  """controlsd: TiAbRunner.step every cycle (the A/B alternation and the drive plan): the A/B switch and the plan switched on and off,
  short blocks so several start and end in each window, the driver steering, lane changes, blinkers, faults, engagement lost."""
  from openpilot.selfdrive.car.mazda import ti_ab, ti_plan as tp
  from openpilot.selfdrive.car.mazda.ti_cost_check import MemParams
  clock, restore = _controls_clock([ti_ab, tp])
  disk = MemParams({"TiAbConfig": json.dumps({"setting": "LatNoFrictionRelay", "b": True, "block_min": 4})})
  mem = MemParams({"TiPlanOn": b"0"})
  rn = ti_ab.TiAbRunner(disk, mem, log=lambda ev: None, block_s=20.0, plan_target_min=1.0)
  toggles = NS(lat_no_friction_relay=False, steerKp=[[0.0], [1.0]])
  P = 12000
  seen = {}

  def step(i):
    clock[0] += 0.01
    ph = i % P
    if ph == 0:
      mem.put_bool("TiAbOn", True)
      mem.put_bool("TiPlanOn", False)
    if ph == 5000:
      mem.put_bool("TiAbOn", False)
      mem.put_bool("TiPlanOn", True)
    if ph == 11000:
      mem.put_bool("TiPlanOn", False)
    if i % 10 == 0 and mem.get_bool("TiPlanOn"):
      raw = mem.get("TiPlan", encoding="utf8")
      if raw:
        nonce, seq = tp.plan_ident(raw)
        mem.put("TiPlanAck", json.dumps(tp.make_ack(nonce, seq, (ph // 700) % 2 == 0, 0, 0.0, clock[0])))
    engaged = not (3000 <= ph < 3400 or 9000 <= ph < 9200)
    v = (20.0 + 8.0 * math.sin(i * 2e-4)) * 0.44704 + 8.0
    press = (ph // 250) % 9 == 4
    rn.step(toggles, engaged=engaged, v_ego=v, curvature=0.0003 + 0.002 * math.sin(i * 1e-3), steer=0.03 + 0.5 * press,
            saturated=(ph // 600) % 7 == 3, steering_pressed=press, driver_torque=60.0 if press else 0.0,
            lane_change=(ph // 1000) % 6 == 2, blinker=(ph // 1000) % 6 == 2, ti_ok=not (7000 <= ph < 7100), fault=ph % 5000 == 76,
            controller_ok=True, mono_ns=i * 10_000_000)
    _count(seen, "ab:" + str(rn.ab.mode))
    _count(seen, "plan:" + str(rn.plan.mode))
    _drain(seen, rn.ab)
  step.seen = seen
  return step, restore


def setup_lateral():
  """controlsd: LatControlTorque with the TI plant (lateral_plant, lateral_reference's LaneObserver / LaneRelease, friction_release),
  update_model_context on a new model every 5 frames, and lateral_diagnostics.record_inputs on its log, as controlsd calls them."""
  import random
  from cereal import car, log
  from openpilot.selfdrive.car.mazda import lateral_plant as lp_mod
  from openpilot.selfdrive.car.mazda.lateral_diagnostics import INPUT_SERVICES, record_inputs
  from openpilot.selfdrive.controls.lib import latcontrol_torque as ltq
  from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
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
  r = random.Random(7)
  CS = car.CarState.new_message()
  feed = _ModelFeed()
  svc = {s: 0 for s in INPUT_SERVICES}
  sm = NS(logMonoTime=svc, valid=dict.fromkeys(INPUT_SERVICES, True), alive=dict.fromkeys(INPUT_SERVICES, True),
          freq_ok=dict.fromkeys(INPUT_SERVICES, True), updated=dict.fromkeys(INPUT_SERVICES, True),
          seen=dict.fromkeys(INPUT_SERVICES, True), all_checks=lambda s=None: True)
  ev = NS(logMonoTime=0, valid=True, carState=NS(canValid=True))
  st = {"v": 20.0}
  seen = {}

  def step(i):
    m, curv, lcs = feed.get(i)
    now = 10_000_000_000 + i * 10_000_000
    m.timestampEof = now - 50_000_000
    lcs_s = "off" if lcs == 0 else "preLaneChange"
    m.meta = NS(laneChangeState=lcs_s)
    ph = i % 6000
    active = not (5800 <= ph)
    st["v"] = min(max(st["v"] + r.gauss(0, 0.05), 8.0), 32.0)
    v = st["v"]
    CS.vEgo = v
    CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-curv * (1 + r.gauss(0, 0.05)), v, 0.0))
    CS.steeringRateDeg = r.gauss(0, 5)
    CS.steeringPressed = (ph // 300) % 10 == 7
    fp = NS(lkasBlocked=v < 12.5, lkasEffective=r.uniform(0, 300), tiActive=not (4000 <= ph < 4100), columnTorque=r.gauss(0, 20))
    ctrl.update_model_context(m, now - 30_000_000 - (i // 5) % 2, (i // 500) % 9 != 4, now, now)
    _, _, lac_log = ctrl.update(active, CS, VM, params, False, curv, False, 0.25, None, None, toggles, fp)
    for s in INPUT_SERVICES:
      svc[s] = now
    ev.logMonoTime = now
    if hasattr(lac_log, "mazdaDiagnostics"):
      record_inputs(lac_log.mazdaDiagnostics, sm, ev if i % 3 else None, i % 3 != 0)
      _count(seen, "diagnostics")
    _count(seen, "lane " + str(getattr(ctrl, "_release_context", NS(reason="none")).reason))
    _count(seen, "active" if active else "inactive")
  step.seen = seen
  return step, lambda: None


def _card(curve_on_fn=None):
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  from openpilot.selfdrive.car.mazda import ti_cost_check as K
  clock = {"k": 0}
  saved = (ccm.time, ccm.ti_curve_log, getattr(ccm, "Params", None))
  ccm.time = NS(monotonic=lambda: clock["k"] * 0.01, time=lambda: 1_800_000_000.0 + clock["k"] * 0.01, perf_counter=time.perf_counter,
                perf_counter_ns=time.perf_counter_ns, monotonic_ns=lambda: clock["k"] * 10_000_000)
  seen = {}
  ccm.ti_curve_log = lambda ev: _count(seen, "curve:" + str(ev.get("kind")))
  cc, mem = K.make_controller()
  disk = ccm.Params()

  def restore():
    ccm.time, ccm.ti_curve_log = saved[0], saved[1]
    if saved[2] is not None:
      ccm.Params = saved[2]
  return ccm, K, cc, mem, disk, clock, restore, seen


def setup_card_tests():
  """card: CarController.update with the plan cycling off / A/B / tests, the identification tests and the dither starting and
  stopping, the lag guard holding and releasing, the driver steering and braking, Flag This Moment and Start A New Measurement taps."""
  from openpilot.selfdrive.car.mazda import ti_plan as tp
  ccm, K, cc, mem, disk, clock, restore, seen = _card()
  CC, CS = K.inputs(38.0, 0.0003)
  out = CS.out
  P = 9000
  lag = [0.0]
  tog = [NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False),
         NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=True, ti_dither=True),
         NS(ti_sweep=True, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=True)]

  def step(i):
    clock["k"] = i
    ph = i % P
    mode = (None, "ab", "tests")[ph // 3000]
    if i % 50 == 0:
      mem.put_bool("TiPlanOn", mode is not None)
      if mode is not None:
        mem.put("TiPlan", tp.encode_plan(tp.PlanOut(mode, mode == "ab", "", "", 1 + ph // 3000),
                                         1_800_000_000.0 + i * 0.01, False, "memcheck", True))
    if ph == 1236:
      mem.put_bool("TiFlagMoment", True)
    if ph == 8100:
      mem.put_bool("ClearTiStats", True)
    curved = (ph // 700) % 3 == 1
    CC.actuators.curvature = 0.02 if curved else 0.0003
    CC.latActive = not (2500 <= ph < 2700)
    out.vEgo = (38.0 + 6.0 * math.sin(i * 5e-4)) * 0.44704
    out.steeringPressed = (ph // 400) % 11 == 5
    out.steeringTorque = 80.0 if out.steeringPressed else -1.0
    out.brakePressed = 6000 <= ph < 6050
    CS.lkas_blocked = out.vEgo < 28.0 * 0.44704
    lag[0] += 2.0 if (ph // 1500) % 4 == 2 else 0.0      # cumLagMs: controlsd 2 ms late per cycle for 15 s, then on time
    cc.ti_lag_ctl = (i * 0.01, lag[0])
    cc.update(CC, CS, i * 10_000_000, tog[(ph // 1000) % 3])
    _count(seen, "ident running" if cc.ti_ident is not None and cc.ti_ident.running else "ident idle")
    _count(seen, "plan " + str(mode))
    _count(seen, "lag held" if cc.ti_lag_held else "lag ok")
    _drain(seen, cc.ti_ident)
  step.seen = seen
  return step, restore


def setup_card_curve():
  """card: curve slowing (ti_curve Curve + SafeExit, the delivery monitor ti_curve_monitor, ti_curve_demand) inside
  CarController.update: the switch on and off, bends given / lifted / passed, plan gaps, the driver's own SET presses, gas, brake."""
  from openpilot.selfdrive.car.mazda import ti_curve as tc
  ccm, K, cc, mem, disk, clock, restore, seen = _card()
  CC, CS = K.inputs(55.0, 0.0003)
  cs = CS.out.cruiseState
  CS.lat_accel, CS.lkas_effective = 1.0, 100.0
  CS.out.steeringRateDeg = 0.0
  P = 12000
  st = {"set": 55}
  tog = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)

  def step(i):
    clock["k"] = i
    ph = i % P
    on = not (10500 <= ph < 11000 or 4600 <= ph < 4700)  # off: once with the set speed left lowered in a bend (the `left` log)
    if i % 20 == 0 and not (7000 <= ph < 7300):         # 3 s plan gap
      corner = (ph // 1000) % 2 == 0 and ph < 9000
      ceil = 40 + (ph // 2000) % 3 * 3
      mem.put(tc.PARAM_ON, b"1" if on else b"0")
      mem.put(tc.PARAM_PLAN, json.dumps({"m": i * 0.01, "seq": i, "mode": "act", "ceil": ceil if corner else None, "conf": 1.0,
                                         "trusted": (ph // 3000) != 2, "known": True, "why": "", "dist": 100.0, "past": not corner,
                                         "model_la": 0.1, "model_k": 0.0003, "capped": False, "limit": None, "kind": "corner",
                                         "corner": ceil if corner else None, "ahead": 2.0 + (ph // 400) % 3 * 0.5,
                                         "ask": [2.0, 2.2, 2.5, 2.2, 2.0, 1.8]}))
    CS.crz_btns_counter = ((i - 3) // 10) % 16          # the switch module's counter steps at frames 3, 13, 23, ...
    cv = cc.curve
    if cv is not None and cv.tap_to is not None and cv.tap_first is not None and i - cv.tap_first >= 8:
      st["set"] = int(cv.tap_to)
    if ph == 8500:
      st["set"] = 53                                   # the driver's own SET- press
    CS.crz_btns_driver = 8497 <= ph < 8500             # ... as the car shows it (0x09D): the `drv_btn` log, 2026-11-25
    CS.crz_btns_set, CS.crz_btns_set_plus = CS.crz_btns_driver, False
    if ph == 0:
      st["set"] = 55
    cs.speed = st["set"] * 0.44704
    CS.out.vEgo = (st["set"] + 1.5 * math.sin(i * 0.01)) * 0.44704
    CS.out.gasPressed = 9500 <= ph < 9600
    # the stock assist's torque count: changing every frame, zero for a while (review 2026-11-25: a constant hid deliver_phase
    # logging every check)
    CS.lkas_effective = 0.0 if 3000 <= ph < 3500 else 60.0 + 37.0 * math.sin(i * 0.37)
    CS.out.brakePressed = 9800 <= ph < 9850
    cs.enabled = not (9850 <= ph < 10000)
    CC.actuators.steer = 1.0 if (ph // 400) % 3 == 0 else 0.3
    cc.update(CC, CS, i * 10_000_000, tog)
    _count(seen, "state:" + str(cc.curve.state if cc.curve is not None else None))
    _drain(seen, cc.ti_ident)
  step.seen = seen
  return step, restore


def setup_card_buttons():
  """card: the cruise button tests (ti_button_tests: hold and taps, start to finish, then stopped) and the cruise set-speed check
  (ti_cruise_check) inside CarController.update, one after the other, on a cruise that takes the presses."""
  from openpilot.selfdrive.car.mazda import ti_button_tests as tb, ti_cruise_check as ck
  ccm, K, cc, mem, disk, clock, restore, seen = _card()
  sent = []

  class Packer:
    def make_can_msg(self, name_, bus, values):
      if name_ == "CRZ_BTNS":
        sent.append((clock["k"], 1 if values.get("SET_P") else (-1 if values.get("SET_M") else 0)))
      return (0, 0, b"\0" * 8, 0)
  cc.packer = Packer()
  CC, CS = K.inputs(45.0, 0.0003)
  cs = CS.out.cruiseState
  st = {"set": 45, "pending": [], "last": None}
  P = 12000
  tog = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)

  def step(i):
    clock["k"] = i
    ph = i % P
    wall = 1_800_000_000.0 + i * 0.01
    if ph == 100:
      mem.put(tb.PARAM_TRIGGER, json.dumps({"cmd": "hold", "id": f"m{i}", "t": wall}))
    if ph == 4000:
      mem.put(tb.PARAM_TRIGGER, json.dumps({"cmd": "taps", "id": f"m{i}", "t": wall}))
    if ph == 7500:
      mem.put(tb.PARAM_TRIGGER, json.dumps({"cmd": "stop", "id": f"m{i}", "t": wall}))
    if ph == 8000:
      mem.put(ck.PARAM_TRIGGER, json.dumps({"cmd": "run", "id": f"m{i}", "t": wall}))
    if ph == 11800:
      st["set"] = 45
    CS.crz_btns_counter = ((i - 3) // 10) % 16
    for p in [p for p in st["pending"] if p[0] <= i]:
      st["pending"].remove(p)
      st["set"] = min(max(st["set"] + p[1], 25), 70)
    cs.speed = st["set"] * 0.44704
    CS.out.vEgo = 45.0 * 0.44704
    n0 = len(sent)
    cc.update(CC, CS, i * 10_000_000, tog)
    for kk, d in sent[n0:]:
      if d and (st["last"] is None or (st["last"] - 3) // 10 != (kk - 3) // 10):
        st["pending"].append((kk + 4, d))
      st["last"] = kk
    _count(seen, "frames sent", len(sent) - n0)
    del sent[:]
    _count(seen, "buttons:" + str(cc.button_tests.state if cc.button_tests is not None else None))
    _count(seen, "check:" + str(cc.cruise_check.state if cc.cruise_check is not None else None))
    _drain(seen, cc.ti_ident)
  step.seen = seen
  return step, restore


def _lap(n_bends, seed):
  import numpy as np
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M
  rng = np.random.default_rng(seed)
  segs = [(300.0, 0.0)]
  for b in range(n_bends):
    sign = 1 if b % 2 else -1
    for _ in range(1 + b % 2):
      segs.append((float(rng.integers(60, 141)), sign * float(rng.uniform(0.006, 0.02))))
      sign = -sign
    segs.append((float(rng.integers(150, 320)), 0.0))
  return M.synthetic_trace(segs)


def setup_planner():
  """ti_curve_plannerd's tick (5 Hz): the car side's status read (car_side), the map list read (map_profile_ref) from a list rewritten
  every second, its `map_list` record once per list, Planner.step, plan_json, the events and the tick log through log_payload / tick_log (JSON-dumped, as cloudlog would),
  fresh_limit; over a lap whose first part is remembered and whose second part is known only from the map list, with lock lost and
  regained, the driver's set speed changing, cruise off and on, the forecast missing for a few seconds, and the car-side floor
  binding (alerts raised and cleared); the driving screen's element (ti_curve_view.View.step: look-ahead, bends, the ding at a red
  chip's onset, its tick field). One period is one lap; the position jumps back to the start (lock lost, re-lock)."""
  import numpy as np
  from openpilot.selfdrive.car.mazda import ti_curve as C, ti_curve_memory as M, ti_curve_plan as P, ti_curve_plannerd as D
  from openpilot.selfdrive.car.mazda import ti_curve_view as V
  from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
  from openpilot.selfdrive.car.mazda.ti_cost_check import MemParams
  view = V.View()
  trA, trB = _lap(4, 11), _lap(3, 12)
  mem = M.synthetic([trA])
  nA = len(trA["x"])
  mem.npass[:] = 12
  mem.npass[nA * 2 // 3:] = 2
  road = {k: np.concatenate([trA[k], np.asarray(trB[k]) + (np.asarray(trA[k])[-1] + M.SPACING_M if k == "x" else 0.0)])
          if k == "x" else np.concatenate([trA[k], trB[k]]) for k in ("x", "y", "hdg", "kap")}
  n = len(road["x"])
  s_road = np.arange(n) * M.SPACING_M
  kap = road["kap"]
  R = 6371000.0
  DT = 1.0 / D.RATE_HZ if hasattr(D, "RATE_HZ") else 0.2
  pl = P.Planner(mem)
  store = MemParams()
  st = {"s": 0.0, "v": 45.0, "set": 45, "seq": 0, "map_t": -1e9, "tap_t": 0.0, "lim": {"t": None}}
  car = CM.Cruise(st["v"], st["set"], DT / 4)
  lap_m = float(s_road[-1] - 20.0)
  lap_ticks = int(lap_m / (st["v"] * P.MPH * DT))
  ahead = np.arange(0, 8.01, 0.25)
  seen = {}

  def kat(q):
    return float(kap[min(max(int(q // M.SPACING_M), 0), n - 1)])

  def step(i):
    t = i * DT
    lap_i = i % lap_ticks
    if lap_i == 0:
      st["s"], st["v"] = 0.0, 45.0
      car.v = st["v"]
    s = st["s"]
    x, y = float(np.interp(s, s_road, road["x"])), float(np.interp(s, s_road, road["y"]))
    h = float(road["hdg"][min(int(s // M.SPACING_M), n - 1)])
    lat, lon, bearing = math.degrees(y / R), math.degrees(x / R), (90.0 - math.degrees(h)) % 360.0
    if t - st["map_t"] >= 1.0:
      m = 12 + (i // 5) % 48
      qs = s + np.linspace(-20, 25 * m, m)
      store.d[D.MAP_PARAM] = json.dumps([{"latitude": math.degrees(float(np.interp(q, s_road, road["y"])) / R),
                                          "longitude": math.degrees(float(np.interp(q, s_road, road["x"])) / R),
                                          "curvature": kat(q)} for q in qs])
      st["map_t"] = t
    mk_map, map_ref = D.map_profile_ref(store.d.get(D.MAP_PARAM), t - st["map_t"], lat, lon, bearing)
    tick_x = {}
    if map_ref is not None:                       # 2026-11-25: the list once per id (map_list_log) and [id, s0_dm] per tick
      if map_ref[0][0] != st.get("list_id"):
        for rec in D.map_list_log(map_ref[0]):
          json.dumps(D.log_payload(rec))
        st["list_id"] = map_ref[0][0]
      tick_x["mpr"] = [map_ref[0][0], map_ref[1]]
    floor_binds = (lap_i // 60) % 4 == 1
    store.d[C.PARAM_STATUS] = json.dumps({"t": 1_800_000_000.0 + t, "state": "slowing" if floor_binds else "ready", "ref": st["set"],
                                          "floor": st["set"] - 2 if floor_binds else None, "floor_now": st["set"] - 4,
                                          "acting": lap_i % 300 < 270}).encode()
    ref, floor, acting = D.car_side(store, 1_800_000_000.0 + t, C)
    v = st["v"] * P.MPH
    kk = P.MODEL_K_SIGN * np.array([kat(q) for q in s + ahead * v]) * np.interp(ahead, P.D.FC_LEAD_S, P.D.FC_RATIO)
    lock = not (lap_ticks // 2 <= lap_i < lap_ticks // 2 + 40)
    model = None if (lap_i // 25) % 9 == 5 else (ahead, kk)
    cruise_on = not (lap_ticks // 3 <= lap_i < lap_ticks // 3 + 30)
    try:
      limit = D.fresh_limit(True, 26.8 if (lap_i // 100) % 2 else 0.0, v, lock, t, st["lim"])
    except Exception:
      limit = None
    p = pl.step(t, P.Fix(lock, x, y, h), v, set_mph=st["set"] if cruise_on else None, yaw_k=kat(s), model_k=model, limit_mph=limit,
                ref_mph=ref, floor_mph=floor, acting=acting, map_k=mk_map)
    pj = P.plan_json(p, t, st["seq"], "act")
    st["seq"] += 1
    store.d[C.PARAM_PLAN] = json.dumps(pj)
    _, _, side = D.stage_summary(pl)
    vp, ding, scr = view.step(pl, p, t, st["seq"], True, st["v"], st["set"] if cruise_on else None,
                              pl._floor(floor, ref, st["set"] if cruise_on else None, st["v"]), side)
    store.d[V.PARAM_VIEW] = V.view_json(vp)
    if ding is not None:
      store.d[V.PARAM_DING] = json.dumps(ding)
      _count(seen, "ding")
    if vp["r"]:
      _count(seen, "ribbon")
    tick_x["scr"] = scr
    if p.ceil is not None:
      _count(seen, "ceiling")
    for ev in pl.events:
      _count(seen, "ev:" + str(ev.get("kind")))
      json.dumps(D.log_payload(ev, feature="act"), default=str)
    pl.events.clear()
    json.dumps(D.log_payload(D.tick_log(p, dict(tick_x, mph=round(st["v"], 1), map=list(zip(*mk_map)) if mk_map else [],
                                                cs=[ref, floor, int(bool(acting))])), kind="tick"), default=str)
    if lap_i % 200 == 150:
      st["set"] = 47 if st["set"] == 45 else 45       # the driver's own set-speed change
    goal = st["set"] if p.ceil is None else min(st["set"], p.ceil)
    setv = getattr(car, "set_mph", st["set"])
    for j in range(4):
      tj = t + j * DT / 4
      if setv != goal and tj >= st["tap_t"] - 1e-9:
        setv += 1 if goal > setv else -1
        st["tap_t"] = tj + P.TAP_S
      car.step(setv)
    car.set_mph = setv
    st["v"] = car.v
    st["s"] = min(s + st["v"] * P.MPH * DT, lap_m)
  step.period = lap_ticks
  step.seen = seen
  return step, lambda: None


def _decode(b):
  """Event bytes -> reader, as cereal.messaging.log_from_bytes does in SubMaster (a fresh buffer per message: a reader kept alive keeps
  its whole message, so a leaked reference costs the message's size, not just the Python objects)."""
  from cereal import log
  r = log.Event.from_bytes(b, traversal_limit_in_words=2**63 - 1)
  if hasattr(r, "__enter__"):
    with r as m:
      return m
  return r


def _fp_messages(i, period):
  """frogpilot_process's inputs for model frame i, as fresh capnp readers: modelV2 from _lane_model (straight; a right bend running wide
  over the outside line; lines fading; a lane change; a left bend with flickering lines), with the steering at its limit and short of
  the ask in the bends (running wide), the interceptor dropping out once, a blinker, a stop, alerts raised and cleared."""
  from cereal import log
  ph = i % period
  nm, curv, lcs = _lane_model(5 * ph)        # one model frame per call (_lane_model's schedule is in 100 Hz frames)
  out = {}

  def ev(svc):
    e = log.Event.new_message()
    e.valid = True
    return e, e.init(svc)
  e, m = ev("modelV2")
  m.init("laneLines", 4)
  for k, ln in enumerate(nm.laneLines):
    m.laneLines[k].x, m.laneLines[k].y = ln.x, ln.y
  m.laneLineProbs, m.laneLineStds = nm.laneLineProbs, nm.laneLineStds
  m.init("roadEdges", 2)
  for k, ln in enumerate(nm.roadEdges):
    m.roadEdges[k].x, m.roadEdges[k].y = ln.x, ln.y
  m.roadEdgeStds = nm.roadEdgeStds
  m.position.x = [float(5 * k) for k in range(33)]
  m.action.desiredCurvature = curv
  m.meta.laneChangeState = "preLaneChange" if lcs else "off"
  out["modelV2"] = e.to_bytes()
  v = 0.0 if 900 <= ph < 960 else 25.0
  bend = 300 <= ph < 600 or ph >= 840
  e, c = ev("controlsState")
  c.desiredCurvature, c.curvature = curv, curv * (0.6 if bend else 1.0)
  c.vCruise = c.vCruiseCluster = 100.0
  c.alertText1 = "TAKE CONTROL" if (ph // 50) % 7 == 3 else ""
  c.alertType = "steerSaturated/warning" if c.alertText1 else ""
  out["controlsState"] = e.to_bytes()
  e, f = ev("frogpilotControlsState")
  f.alertText1 = "" if ph % 400 else "startup"
  out["frogpilotControlsState"] = e.to_bytes()
  e, cc = ev("carControl")
  cc.latActive = not (700 <= ph < 760)
  cc.actuators.steer = 1.0 if bend else 0.2
  cc.actuators.accel = 0.5
  out["carControl"] = e.to_bytes()
  e, cs = ev("carState")
  cs.vEgo, cs.standstill, cs.gearShifter = v, v == 0.0, "drive"
  cs.leftBlinker = 720 <= ph < 840
  cs.steeringRateDeg = 0.5 if bend else 8.0
  out["carState"] = e.to_bytes()
  e, fc = ev("frogpilotCarState")
  fc.tiActive = not (450 <= ph < 470)        # an interceptor dropout while steering
  fc.columnTorque = 200.0 if bend else 30.0
  fc.trafficModeEnabled = (ph // 600) % 2 == 1
  out["frogpilotCarState"] = e.to_bytes()
  e, fm = ev("frogpilotModelV2")
  fm.turnDirection = "turnLeft" if 1000 <= ph < 1020 else "none"
  out["frogpilotModelV2"] = e.to_bytes()
  return {s: getattr(_decode(b), s) for s, b in out.items()}


def setup_frogpilot_events():
  """frogpilot_process (20 Hz, gc disabled), on-road, engaged or not: FrogPilotEvents.update (whole, the planner's state faked), which
  runs LaneDriftGate.update every model frame (the running-wide gate; 2026-11-25 second leak: lane_drift's per-call class kept every
  modelV2 message, about 13.6 KB per cycle, ~1 GB/h on-road), and the TI block FrogPilotPlanner.update adds per cycle
  (steer_authority.predicted_drift_m / AdvisoryTrigger.update, ti_curve.curve_alert_mph / read_curve_code every 2nd cycle, the
  stopped-steering latch count). Fresh capnp messages per cycle, as SubMaster gives them. The module's params / params_memory are
  swapped for in-memory stores for the run (nothing is read from or written to the device's)."""
  # sentry first: frogpilot_variables -> car_helpers -> sentry -> frogpilot_variables is circular when frogpilot_variables is the
  # first of them imported (the comma: ImportError ERROR_LOGS_PATH); the processes import sentry first through their own imports
  import openpilot.system.sentry  # noqa: F401
  from openpilot.frogpilot.controls.lib import frogpilot_events as FE, steer_authority as SA
  from openpilot.selfdrive.car.mazda import ti_curve as TC
  from openpilot.selfdrive.car.mazda.ti_cost_check import MemParams
  from openpilot.selfdrive.controls.lib.events import FrogPilotEventName as FEN
  saved = (FE.params, FE.params_memory)
  mem = MemParams()
  FE.params, FE.params_memory = MemParams(), mem
  P = 1200                                   # 60 s of model frames
  planner = NS(frogpilot_vcruise=NS(forcing_stop=False, slc=NS(speed_limit_changed_timer=0.0)), tracking_lead=False, model_stopped=False,
               cem=NS(stop_light_detected=False), lead_one=NS(dRel=10.0, vLead=0.0), lateral_acceleration=0.0, steer_advisory_speed=0.0,
               steer_latched=False)
  toggles = NS(random_events=False, green_light_alert=True, current_holiday_theme="stock", lead_departing_alert=True, nnff=False,
               speed_limit_changed_alert=True, startup_alert_top="startup", startup_alert_bottom="", steer_authority_advisory=True,
               wheel_image="stock", is_metric=False)
  fe = FE.FrogPilotEvents(planner, NS(is_file=lambda: False), NS(update_wheel_image=lambda *a, **k: None))
  trig = SA.AdvisoryTrigger()
  st = {"latch": 0, "tick": 0, "alert": (None, 0.0), "code": (None, 0.0)}
  names = {FEN.steerRunningWide: "running wide", FEN.tiDropout: "ti dropout", FEN.steerAuthorityAdvisory: "advisory",
           FEN.steerLatchedWarning: "latched", FEN.trafficModeActive: "traffic mode", FEN.turningLeft: "turning"}
  seen = {}

  def step(i):
    now = i * 0.05
    ph = i % P
    if i % 4 == 0:                           # the planner (5 Hz) and the car controller (stage 3, the set speed left) write these
      mem.put(TC.PARAM_PLAN, json.dumps({"m": now, "mode": "act", "alert": 38 if 300 <= ph < 500 else None}))
      mem.put(TC.PARAM_DELIVER, json.dumps({"m": now, "need": 36 if 440 <= ph < 560 else None,
                                            "why": "dropout" if 450 <= ph < 470 else "limit"}))
      mem.put(TC.PARAM_LEFT, json.dumps({"m": now, "who": "curve", "now": 40, "was": 45}) if 1100 <= ph < 1150 else "")
    sm = _fp_messages(i, P)
    v_ego = sm["carState"].vEgo
    # FrogPilotPlanner.update's TI block (frogpilot_planner.py), in its order
    forecast_la = abs(sm["modelV2"].action.desiredCurvature) * 1.6 * max(v_ego, 1.0) ** 2
    SA.predicted_drift_m(forecast_la, v_ego * 3.6)
    adv = trig.update(True, forecast_la, v_ego * 3.6)
    planner.steer_advisory_speed = adv / 3.6 if adv > 0 else 0.0
    st["tick"] += 1
    if st["tick"] % 2 == 0:
      st["alert"] = TC.curve_alert_mph(mem, now)
      st["code"] = TC.read_curve_code(mem, now)
    if st["alert"][0] and now - st["alert"][1] < 1.0:
      planner.steer_advisory_speed = -st["alert"][0] * 0.44704
      _count(seen, "curve alert")
    if st["code"][0] and now - st["code"][1] < 1.0:
      planner.steer_advisory_speed = st["code"][0]
      _count(seen, "curve code")
    loaded = abs(sm["frogpilotCarState"].columnTorque) > SA.CEILING_COUNTS - 25.0
    at_clip = abs(sm["carControl"].actuators.steer) >= 0.99
    st["latch"] = st["latch"] + 1 if (at_clip and abs(sm["carState"].steeringRateDeg) < 2.0 and loaded) else 0
    planner.steer_latched = st["latch"] >= SA.LATCH_DEBOUNCE_FRAMES
    fe.update(100.0 * 0.2778, sm, toggles)
    for n in fe.events.names:
      if n in names:
        _count(seen, names[n])
    if fe.lane_drift_gate.latch_left > 0:
      _count(seen, "gate latched")

  def teardown():
    FE.params, FE.params_memory = saved
  step.seen = seen
  return step, teardown


# name: (setup, rate Hz, period frames, (warm periods, window periods) full, quick stride, process). The quick run (the install check on
# the comma) runs one period of warm-up and one per window with the input schedule strided: step(i * stride), the whole sequence in
# 1/stride of the frames (the simulated clock jumps stride frames per call: coarser, still every state); the PC tests run it unstrided.
SCENARIOS = {
  "lane_drift": (setup_lane_drift, 100, 6000, (1, 2), 4, "controlsd"),
  "ab_runner": (setup_ab_runner, 100, 12000, (1, 1), 8, "controlsd"),
  "lateral": (setup_lateral, 100, 6000, (1, 1), 6, "controlsd"),
  "card_tests": (setup_card_tests, 100, 9000, (1, 1), 6, "card"),
  "card_curve": (setup_card_curve, 100, 12000, (1, 1), 6, "card"),
  "card_buttons": (setup_card_buttons, 100, 12000, (1, 1), 6, "card"),
  "planner": (setup_planner, 5, None, (1, 2), 4, "ti_curve_plannerd"),
  "frogpilot_events": (setup_frogpilot_events, 20, 1200, (1, 2), 4, "frogpilot_process"),
}


def run(name, quick=False, max_cycles=0):
  setup, rate, period, full, stride, _ = SCENARIOS[name]
  step, teardown = setup()
  try:
    period = period or step.period
    if quick:
      fn = step
      step = lambda i: fn(i * stride)              # noqa: E731
      res = measure(step, period // stride, period // stride, rate, max_cycles=max_cycles)
    else:
      w, k = full
      res = measure(step, w * period, k * period, rate, max_cycles=max_cycles)
    res["seen"] = dict(getattr(fn if quick else step, "seen", {}))
  finally:
    teardown()
  res["name"] = name
  return res


def readiness(path="/data"):
  """'free disk X % (Y GB) on /data, memory used Z %' and a WARNING when the free disk is under 20 %."""
  import shutil
  parts = []
  warn = False
  try:
    du = shutil.disk_usage(path)
    free_pct = 100.0 * du.free / du.total
    parts.append(f"free disk {free_pct:.0f} % ({du.free / 1e9:.1f} GB) on {path}")
    warn = free_pct < 20.0
  except OSError as e:
    parts.append(f"free disk on {path}: unknown ({type(e).__name__})")
  try:
    info = {}
    with open("/proc/meminfo") as fh:
      for line in fh:
        k, v = line.split(":", 1)
        info[k] = int(v.split()[0])
    parts.append(f"memory used {100.0 * (1 - info['MemAvailable'] / info['MemTotal']):.0f} %")
  except (OSError, KeyError, ValueError):
    parts.append("memory used: unknown")
  line = "readiness: " + ", ".join(parts)
  if warn:
    line += "\nWARNING: free disk under 20 %: logs will be deleted early or recording may stop"
  return line


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--quick", action="store_true", help="one-period windows (the install check, about 20-30 s on the comma)")
  ap.add_argument("--only", nargs="*", default=None)
  ap.add_argument("--json", action="store_true")
  ap.add_argument("--readiness", action="store_true", help="print only the readiness line (free disk on /data, memory used)")
  a = ap.parse_args(argv)
  if a.readiness:
    print(readiness())
    return 0
  t0 = time.time()
  bad = []
  for name in (a.only or list(SCENARIOS)):
    r = run(name, quick=a.quick)
    if a.json:
      print(json.dumps(r))
    else:
      print(f"{name:13s} {SCENARIOS[name][5]:18s} cyclic {r['cyclic']:4d}  growth w1 {r['growth1_B']:8d} B  w2 {r['growth2_B']:7d} B  "
            f"{r['B_per_frame']:7.2f} B/frame = {r['MB_per_h']:6.2f} MB/h  cost {r['us1']:7.1f} -> {r['us2']:7.1f} us"
            f"{' (noisy)' if r['noisy'] else ''}  {'FAIL' if r['fails'] else 'ok'}")
    for f in r["fails"]:
      print(f"  FAIL {name}: {f}")
    if r["cost_fail"]:
      print(f"  note {name}: cost grew {r['cost_ratio']}x (window 2 over window 1); not failed on the device (timing)")
    bad += r["fails"]
  print(f"ti_mem_check: {len(bad)} failure(s), {time.time() - t0:.1f} s")
  return 1 if bad else 0


if __name__ == "__main__":
  sys.exit(main())
