#!/usr/bin/env python3
"""Old vs new lateral controller, side by side, bit for bit: the torque controller and the Mazda lateral plant of an OLD source tree
and of the current one get the same inputs every cycle, and every cycle's outputs (steer, angle, and the serialized
LateralTorqueState log) and the controllers' numeric state must be identical.

    python3 ltq_side_by_side.py --old <old tree> [--rlogs rlog ...] [--random N] [--seed S]

--rlogs: recorded drives (each segment's carState / controlsState / liveParameters / liveTorqueParameters / liveDelay /
  frogpilotCarState fed as controlsd does; the toggles are varied per segment so the output filter, the friction relay and the
  commit / damping / friction-compensation branches all run).
--random N: N randomised cycles (random walks with injected edge values: NaN, +-inf, -0.0, exact table breakpoints, out of range,
  tiny and huge values), half through the plant path and half through the plain torque path; a cycle that raises must raise the same
  exception in both, after which both controllers are rebuilt.
Run with the device's numpy (1.26.4). Prints one JSON line; exit 0 when nothing differed.
"""
import argparse
import importlib.util
import json
import math
import random
import sys
import time
from bisect import bisect_right
from types import SimpleNamespace as NS

DT = 0.01


def load(path, name):
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  sys.modules[name] = mod
  spec.loader.exec_module(mod)
  return mod


def modules(old_root):
  from openpilot.selfdrive.controls.lib import latcontrol_torque as new_ltq
  from openpilot.selfdrive.car.mazda import lateral_plant as new_plant
  old_ltq = load(f"{old_root}/selfdrive/controls/lib/latcontrol_torque.py", "old_latcontrol_torque")
  old_plant = load(f"{old_root}/selfdrive/car/mazda/lateral_plant.py", "old_lateral_plant")
  return (old_ltq, old_plant), (new_ltq, new_plant)


class CI:
  def __init__(self, plant_mod, with_plant=True):
    if with_plant:
      self.lateral_plant = plant_mod.TiLateralPlant(600.0, DT)

  def torque_from_lateral_accel(self):
    return lambda la, tp: la / tp.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda t, tp: t * tp.latAccelFactor


def fbits(v):
  if isinstance(v, bool) or v is None or isinstance(v, str):
    return v
  try:
    f = float(v)
  except (TypeError, ValueError):
    return repr(v)
  return "nan" if f != f else f.hex() + ("-" if math.copysign(1.0, f) < 0 else "+")


def state(ctrl, depth=0):
  """The controller's numeric state, recursively (floats as exact bits, deques and lists element-wise)."""
  out = {}
  for k, v in sorted(vars(ctrl).items()):
    if k in ("CP", "VM", "plant_mod") or callable(v):
      continue
    if isinstance(v, (int, float, bool)) or type(v).__name__ in ("float64", "float32", "int64", "bool_"):
      out[k] = fbits(v)
    elif isinstance(v, (list, tuple)) or type(v).__name__ == "deque":
      out[k] = [fbits(x) if not hasattr(x, "__dict__") else "obj" for x in v]
    elif hasattr(v, "__dict__") and depth < 3:
      out[k] = state(v, depth + 1)
  return out


def compare(a, b):
  (sa, ga, la), (sb, gb, lb) = a, b
  return fbits(sa) == fbits(sb) and fbits(ga) == fbits(gb) and la.to_bytes() == lb.to_bytes()


class Pair:
  def __init__(self, mods, CP, with_plant=True):
    (self.old_ltq, self.old_plant), (self.new_ltq, self.new_plant) = mods
    self.old = self.old_ltq.LatControlTorque(CP, CI(self.old_plant, with_plant), DT)
    self.new = self.new_ltq.LatControlTorque(CP, CI(self.new_plant, with_plant), DT)

  def each(self, fn):
    """fn(controller) on both; (same?, results). An exception counts as a result (its type and message)."""
    res = []
    for c in (self.old, self.new):
      try:
        res.append(("ok", fn(c)))
      except Exception as e:                                  # noqa: BLE001 - the exception itself is compared
        res.append(("raise", f"{type(e).__name__}: {e}"))
    (ka, va), (kb, vb) = res
    if ka != kb:
      return False, res
    if ka == "raise":
      return va == vb, res
    if isinstance(va, tuple) and len(va) == 3:
      return compare(va, vb), res
    return fbits(va) == fbits(vb), res


# ---- recorded drives --------------------------------------------------------------------------------------------------------------
def replay(mods, paths, report):
  from cereal import log
  from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
  topics = {"carState", "controlsState", "liveParameters", "liveTorqueParameters", "liveDelay", "frogpilotCarState", "carParams"}
  for si, path in enumerate(paths):
    rows = {k: [] for k in topics}
    with open(path, "rb") as f:
      raw = f.read()
    for ev in log.Event.read_multiple_bytes(raw):
      w = ev.which()
      if w in rows:
        rows[w].append((ev.logMonoTime, getattr(ev, w), ev.valid))
    if not rows["carParams"] or not rows["controlsState"]:
      report["segments_skipped"] += 1
      continue
    times = {k: [t for t, _, _ in v] for k, v in rows.items()}

    def latest(k, t):
      i = bisect_right(times[k], t) - 1
      return rows[k][i] if i >= 0 else None
    CP = rows["carParams"][0][1].as_builder()
    pair = Pair(mods, CP)
    VM = VehicleModel(CP)
    toggles = NS(lat_damping=si % 2 == 0, lat_commit_setpoint=si % 3 != 2, lat_friction_comp=si % 4 != 3,
                 lat_output_filter=si % 2 == 1, lat_no_friction_relay=si % 5 != 4, ti_steer_max=600.0)
    for mono, c, _ in rows["controlsState"]:
      if c.lateralControlState.which() != "torqueState":
        continue
      got = [latest(k, mono) for k in ("carState", "liveParameters", "liveTorqueParameters", "liveDelay", "frogpilotCarState")]
      if any(g is None for g in got):
        continue
      (_, cs, _), (_, lp, _), (_, ltp, ltp_valid), (_, delay, _), (_, fp, _) = got
      active = bool(c.lateralControlState.torqueState.active)
      VM.update_params(max(lp.stiffnessFactor, 0.1), max(lp.steerRatio, 0.1))
      if ltp_valid and ltp.useParams:
        ok, res = pair.each(lambda x: x.update_live_torque_params(ltp.latAccelFactorFiltered, ltp.latAccelOffsetFiltered,
                                                                   ltp.frictionCoefficientFiltered))
      if not active:
        pair.each(lambda x: x.reset())
      ok, res = pair.each(lambda x: x.update(active, cs, VM, lp, False, c.desiredCurvature, False, delay.lateralDelay + 0.1, None,
                                             None, toggles, fp))
      report["replay_cycles"] += 1
      report["replay_active"] += active
      if ok and state(pair.old) != state(pair.new):
        ok = False
        res = "state differs"
      if not ok:
        report["replay_mismatches"] += 1
        if len(report["first"]) < 5:
          report["first"].append({"segment": str(path), "mono": mono, "res": str(res)[:400]})
    report["segments"] += 1


# ---- randomised cycles ------------------------------------------------------------------------------------------------------------
EDGE = [0.0, -0.0, float("nan"), float("inf"), float("-inf"), 1e-300, -1e-300, 5e-324, 1e300, -1e300]


def random_cycles(mods, n, seed, report):
  from cereal import car, log
  from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
  from openpilot.selfdrive.car.mazda.lateral_plant import V_BP
  r = random.Random(seed)
  speeds = list(V_BP) + [0.0, -0.0, 10.0, 20.0, 30.0, 0.5, 1.0]          # the plant's and the low-speed table's breakpoints
  CP = car.CarParams.new_message()
  CP.steerLimitTimer, CP.mass, CP.rotationalInertia, CP.wheelbase = 0.4, 1909.0, 2813.0, 2.7
  CP.centerToFront, CP.steerRatio, CP.tireStiffnessFront, CP.tireStiffnessRear = 2.7 * 0.41, 15.5, 80000.0, 90000.0
  lt = CP.lateralTuning
  lt.init("torque")
  lt.torque.kp, lt.torque.ki, lt.torque.friction, lt.torque.latAccelFactor = 0.8, 0.2, 0.1, 2.0
  lt.torque.useSteeringAngle = True
  VM = VehicleModel(CP)
  block = 2000
  done = 0
  while done < n:
    with_plant = (done // block) % 2 == 0
    pair = Pair(mods, CP, with_plant)
    v, ang, curv, rate, col, eff = r.uniform(5, 35), 0.0, 0.0, 0.0, 0.0, 0.0
    for _ in range(block):
      if done >= n:
        break
      done += 1
      v = min(max(v + r.gauss(0, 0.05), 0.0), 40.0)
      ang = ang + r.gauss(0, 0.5)
      curv = min(max(curv + r.gauss(0, 2e-4), -0.02), 0.02)
      rate, col, eff = r.gauss(0, 20), r.gauss(0, 30), min(max(eff + r.gauss(0, 10), -320), 320)
      vv, cc, aa = v, curv, ang
      k = r.random()
      if k < 0.03:
        vv = r.choice(speeds)
      elif k < 0.035:
        vv = r.choice(EDGE)
      elif k < 0.04:
        cc = r.choice(EDGE)
      elif k < 0.045:
        aa = r.choice(EDGE)
      elif k < 0.05:
        cc = 0.0 if r.random() < 0.5 else -0.0
      CS = car.CarState.new_message()
      CS.vEgo, CS.steeringAngleDeg, CS.steeringRateDeg = vv, aa, rate if r.random() > 0.01 else r.choice(EDGE)
      CS.steeringPressed = r.random() < 0.02
      lp = log.LiveParametersData.new_message()
      lp.roll, lp.angleOffsetDeg = r.gauss(0, 0.02), r.gauss(0, 0.3)
      fp = NS(lkasBlocked=r.random() < 0.3 if r.random() < 0.05 else vv < 12.5, lkasEffective=eff if r.random() > 0.02 else r.choice([0, -0.0, 0.0, 1]),
              tiActive=r.random() > 0.01, columnTorque=col if r.random() > 0.02 else r.choice([0.0, None, -0.0]))
      toggles = NS(lat_damping=r.random() < 0.7, lat_commit_setpoint=r.random() < 0.7, lat_friction_comp=r.random() < 0.7,
                   lat_output_filter=r.random() < 0.5, lat_no_friction_relay=r.random() < 0.5, ti_steer_max=600.0)
      active = r.random() > 0.02
      delay = r.choice([0.1, 0.2, 0.25, 0.3, 0.5, 1e-9, 0.0]) if r.random() < 0.02 else 0.25 + r.gauss(0, 0.02)
      limited = r.random() < 0.01                            # drawn once: the lambdas below run once per controller
      if r.random() < 0.002:
        off = r.choice([0.6, -0.6, 0.7, -0.0, 0.1])
        pair.each(lambda x: x.update_live_torque_params(2.0, off, 0.1))
      if not active:
        pair.each(lambda x: x.reset())
      ok, res = pair.each(lambda x: x.update(active, CS, VM, lp, limited, cc, False, delay, None, None, toggles, fp))
      report["random_cycles"] += 1
      raised = res[0][0] == "raise"
      report["random_raised"] += raised
      if ok and not raised and state(pair.old) != state(pair.new):
        ok, res = False, "state differs"
      if not ok:
        report["random_mismatches"] += 1
        if len(report["first"]) < 5:
          report["first"].append({"cycle": done, "res": str(res)[:400]})
      if raised:
        break                                                # rebuild both (same exception in both: compared above)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--old", required=True)
  ap.add_argument("--rlogs", nargs="*", default=[])
  ap.add_argument("--random", type=int, default=0)
  ap.add_argument("--seed", type=int, default=20261122)
  a = ap.parse_args()
  import numpy as np
  t0 = time.time()
  mods = modules(a.old)
  report = {"numpy": np.__version__, "segments": 0, "segments_skipped": 0, "replay_cycles": 0, "replay_active": 0, "replay_mismatches": 0,
            "random_cycles": 0, "random_raised": 0, "random_mismatches": 0, "first": []}
  if a.rlogs:
    replay(mods, a.rlogs, report)
  if a.random:
    random_cycles(mods, a.random, a.seed, report)
  report["seconds"] = round(time.time() - t0, 1)
  print(json.dumps(report))
  return 0 if not report["replay_mismatches"] and not report["random_mismatches"] else 1


if __name__ == "__main__":
  sys.exit(main())
