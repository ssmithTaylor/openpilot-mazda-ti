#!/usr/bin/env python3
"""On-device dry run of the curve slowing planner, for the installer (run on the PARKED comma; sends nothing, no CAN, no params):

    python3 selfdrive/car/mazda/ti_curve_dryrun.py --rlog /data/media/0/realdata/<route>--<seg>/rlog [more rlogs] [--memory PATH]

Checks that the road memory file loads (time, traces, points) and runs ti_curve_plan.Planner at 5 Hz over a recorded drive's own
location stream (liveLocationKalman), speed and set speed (carState), the model's preview (modelV2) and the speed limit
(frogpilotPlan), exactly as ti_curve_plannerd feeds it, measuring the time per planner tick (wall clock on the parked device: mean, p99, max) and the
process's peak memory. Prints counts only: ticks, share trusted / known, ceilings, corners started, why-reasons; never a position.
Exit 0 when the memory loaded and the p99 tick stays under --budget-ms (default 20 ms of the 200 ms tick), else 1.
"""
import argparse
import collections
import json
import math
import sys
import time


def events(paths):
  try:
    from openpilot.tools.lib.logreader import LogReader
    for p in paths:
      yield from LogReader(p)
  except ImportError:
    from cereal import log
    for p in paths:
      with open(p, "rb") as f:
        yield from log.Event.read_multiple_bytes(f.read())


def main():
  from cereal import log
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P, ti_curve_plannerd as D
  ap = argparse.ArgumentParser()
  ap.add_argument("--rlog", nargs="+", required=True)
  ap.add_argument("--memory", default=D.MEMORY_PATH)
  ap.add_argument("--budget-ms", type=float, default=20.0)
  a = ap.parse_args()
  t0 = time.monotonic()
  mem = M.load(a.memory)
  load_s = time.monotonic() - t0
  print(f"memory: loaded in {load_s:.2f} s, {mem.n_traces} traces, {mem.n} points, grade {'present' if mem.grade.any() else 'absent'}")
  pl = P.Planner(mem)
  valid = log.LiveLocationKalman.Status.valid
  last = {}
  next_t, t_first = None, None
  cpu, why, ceils, starts, trusted, known, n = [], collections.Counter(), 0, 0, 0, 0, 0
  for e in events(a.rlog):
    w = e.which()
    if w in ("liveLocationKalman", "carState", "modelV2", "frogpilotPlan"):
      last[w] = getattr(e, w)
    if w != "carState" or "liveLocationKalman" not in last:
      continue
    t = e.logMonoTime * 1e-9
    t_first = t if t_first is None else t_first
    if next_t is not None and t < next_t:
      continue
    next_t = t + 1.0 / D.RATE_HZ
    c0 = time.perf_counter()
    llk, cs = last["liveLocationKalman"], last["carState"]
    fix = P.Fix(False)
    if llk.gpsOK and llk.positionGeodetic.valid and llk.status == valid:
      x, y = mem.to_local(llk.positionGeodetic.value[0], llk.positionGeodetic.value[1])
      fix = P.Fix(True, float(x), float(y), P.bearing_to_hdg(math.degrees(llk.calibratedOrientationNED.value[2])))
    v = float(cs.vEgo)
    set_mph = cs.cruiseState.speed / P.MPH if cs.cruiseState.enabled and cs.cruiseState.speed > 0.5 else None
    yaw_k = -float(llk.angularVelocityCalibrated.value[2]) / v if v > 3.0 else None
    lim = None
    if "frogpilotPlan" in last:
      sl = float(last["frogpilotPlan"].slcMapSpeedLimit)
      lim = sl / P.MPH if sl > 0.5 else None
    mk = D.model_curvature(last["modelV2"]) if "modelV2" in last else None
    p = pl.step(t, fix, v, set_mph=set_mph, yaw_k=yaw_k, model_k=mk, limit_mph=lim)
    json.dumps(P.plan_json(p, t, n, "shadow"))
    cpu.append(time.perf_counter() - c0)
    n += 1
    trusted += p.trusted
    known += p.known
    ceils += p.ceil is not None and p.kind == "corner"
    why[p.why] += 1
    starts += sum(1 for ev in pl.events if ev.get("kind") == "start")
    pl.events.clear()
  if not cpu:
    print("NOT VALID: no carState / liveLocationKalman in the logs given")
    return 1
  cpu.sort()
  p99 = cpu[int(0.99 * (len(cpu) - 1))] * 1000
  try:
    import resource
    rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
  except ImportError:                                  # not on the device (a PC check)
    rss_mb = float("nan")
  print(f"ticks {n} ({n / D.RATE_HZ / 60:.1f} min); trusted {100 * trusted / n:.0f} %, known {100 * known / n:.0f} %; ticks with a corner "
        f"ceiling {ceils}; corners started {starts}")
  print("reasons: " + ", ".join(f"{k} {v}" for k, v in why.most_common(8)))
  print(f"cpu per tick: mean {1000 * sum(cpu) / n:.2f} ms, p99 {p99:.2f} ms, max {1000 * cpu[-1]:.2f} ms "
        f"(= {100 * sum(cpu) / n * D.RATE_HZ:.2f} % of one core at {D.RATE_HZ:g} Hz); peak memory {rss_mb:.0f} MB")
  ok = p99 <= a.budget_ms
  print("DRY RUN OK" if ok else f"OVER BUDGET: p99 {p99:.2f} ms > {a.budget_ms} ms")
  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
