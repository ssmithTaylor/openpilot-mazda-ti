#!/usr/bin/env python3
"""On-device dry run of the curve slowing planner, for the installer (run on the PARKED comma; sends nothing, no CAN, no params):

    python3 selfdrive/car/mazda/ti_curve_dryrun.py                      # the shipped real-drive inputs (INPUTS), the FULL tick
    python3 selfdrive/car/mazda/ti_curve_dryrun.py --rlog /data/media/0/realdata/<route>--<seg>/rlog [more rlogs] [--memory PATH]

INPUTS (default, perf 2026-11-26): ti_curve_dryrun_inputs.npz, recorded by tools/dev/record_dryrun_inputs.py from a real drive's
position-free arrays (8 bend-rich minutes: the road as yaw curvature per 5 m, per tick the distance, speed, set speed, yaw, blinker
and the driving model's preview). The road and its memory are rebuilt synthetically from that curvature; every tick runs what
ti_curve_plannerd runs after reading its inputs (Planner.step, plan_json, the tick summaries, tick_log and their JSON), timed whole.
Budgets (wall clock on the device): ticks with no bend in play p99 <= --budget-ms (20), ticks with a bend in play (a stage-1 bend,
a map bend or stage 2 in progress) p99 <= --budget-bend-ms (100), and no tick over --max-ms (500). ~2 min on the device.

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
import os
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


INPUTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ti_curve_dryrun_inputs.npz")


def budgets(walls, bend, budget_ms, budget_bend_ms, max_ms):
  """-> (ok, text): p99 of the ticks without / with a bend in play and the slowest tick, against the budgets (ms)."""
  def p99(x):
    x = sorted(x)
    return x[int(0.99 * (len(x) - 1))] * 1000 if x else 0.0
  nb = [w for w, b in zip(walls, bend) if not b]
  wb = [w for w, b in zip(walls, bend) if b]
  mx = max(walls) * 1000 if walls else 0.0
  ok = p99(nb) <= budget_ms and p99(wb) <= budget_bend_ms and mx <= max_ms
  return ok, (f"no bend in play: {len(nb)} ticks, p99 {p99(nb):.2f} ms (budget {budget_ms:g}); bend in play: {len(wb)} ticks, p99 "
              f"{p99(wb):.2f} ms (budget {budget_bend_ms:g}); slowest tick {mx:.2f} ms (budget {max_ms:g})")


def run_inputs(path, budget_ms=20.0, budget_bend_ms=100.0, max_ms=500.0, limit=None):
  """The FULL tick over the recorded real-drive inputs (module docstring). -> 0 within every budget, else 1."""
  import numpy as np
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P, ti_curve_plannerd as D, ti_curve_view as V
  z = np.load(path, allow_pickle=False)
  meta = json.loads(str(z["meta"]))
  sp = float(meta["spacing_m"])
  tr = M.synthetic_trace([(sp, float(k)) for k in z["road_k"].astype(np.float64)], spacing=sp)
  mem = M.synthetic([tr])
  s_road = np.arange(len(tr["x"])) * sp
  P.Planner._table_tried = False
  t0 = time.monotonic()
  pl = P.Planner(mem)
  load_s = time.monotonic() - t0
  s_, v_, set_, yaw_, bl_ = (z[k].astype(np.float64) for k in ("s", "v", "set_ms", "yaw_k", "blink"))
  pt, pla = z["pred_t"].astype(np.float64), z["pred_la"].astype(np.float64)
  n = len(s_) if limit is None else min(limit, len(s_))
  walls, bend, starts, cold0 = [], [], 0, P.COLD[0]
  view, dings, vbytes = V.View(), 0, 0
  dtick = float(meta["tick_s"])
  for k in range(n):
    c0 = time.perf_counter()
    s, v = float(s_[k]), float(v_[k])
    j = min(int(s // sp), len(tr["x"]) - 1)
    fix = P.Fix(True, float(np.interp(s, s_road, tr["x"])), float(np.interp(s, s_road, tr["y"])), float(tr["hdg"][j]))
    set_mph = float(np.float32(set_[k])) / P.MPH if set_[k] > 0.5 else None
    row = pla[k]
    mk = (pt, row / max(v, 3.0) ** 2) if np.all(np.isfinite(row)) else None
    p = pl.step(k * dtick, fix, v, set_mph=set_mph, yaw_k=float(yaw_[k]) if v > 3.0 else None, model_k=mk, blinker=bool(bl_[k]))
    d = P.plan_json(p, k * dtick, k, "shadow")
    json.dumps(d)
    starts += sum(1 for ev in pl.events if ev.get("kind") == "start")
    pl.events.clear()
    s1, s2, side = D.stage_summary(pl)
    # the driving screen's element as plannerd publishes it while curve slowing is on (ti_curve_view.py): look-ahead, bends, ding
    vp, ding, scr = view.step(pl, p, k * dtick, k, True, v / P.MPH, set_mph, pl._floor(None, None, set_mph, v / P.MPH), side)
    V.view_json(vp)
    dings += ding is not None
    vbytes = max(vbytes, len(V.view_json(vp)))
    extras = dict(seq=k, v=v / P.MPH, set=None if set_mph is None else int(round(set_mph)), mem=D.mem_summary(pl),
                  fc=D.fc_summary(pl), s1=s1, s2=s2, side=side, step_us=1, cs=[None, None, 1], scr=scr)
    json.dumps({"event": "ti_curve_plan", **D.log_payload(D.tick_log(p, extras), kind="tick")})
    walls.append(time.perf_counter() - c0)
    bend.append(bool(pl.active or pl.map_active or pl.s2 is not None))
  ok, txt = budgets(walls, bend, budget_ms, budget_bend_ms, max_ms)
  print(f"inputs: {n} ticks ({n * dtick / 60:.1f} min of a real drive), road {len(s_road) * sp / 1000:.1f} km rebuilt from its curvature; "
        f"planner start {load_s:.2f} s (approach table {P.Planner.table_keys} keys, raise thresholds {P.Planner.raise_keys}); bends "
        f"started {starts}; cold simulations {P.COLD[0] - cold0}; total {sum(walls):.1f} s; screen: {dings} dings, largest "
        f"payload {vbytes} B")
  print(txt)
  print("DRY RUN OK" if ok else "OVER BUDGET")
  return 0 if ok else 1


def main(argv=None):
  from cereal import log
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P, ti_curve_plannerd as D
  ap = argparse.ArgumentParser()
  ap.add_argument("--rlog", nargs="+")
  ap.add_argument("--inputs", default=INPUTS)
  ap.add_argument("--memory", default=D.MEMORY_PATH)
  ap.add_argument("--budget-ms", type=float, default=20.0)
  ap.add_argument("--budget-bend-ms", type=float, default=100.0)
  ap.add_argument("--max-ms", type=float, default=500.0)
  a = ap.parse_args(argv)
  if not a.rlog:
    return run_inputs(a.inputs, a.budget_ms, a.budget_bend_ms, a.max_ms)
  t0 = time.monotonic()
  mem = M.load(a.memory)
  load_s = time.monotonic() - t0
  print(f"memory: loaded in {load_s:.2f} s, {mem.n_traces} traces, {mem.n} points, grade {'present' if mem.grade.any() else 'absent'}")
  pl = P.Planner(mem)
  valid = log.LiveLocationKalman.Status.valid
  last = {}
  next_t, t_first = None, None
  cpu, why, ceils, starts, trusted, known, n = [], collections.Counter(), 0, 0, 0, 0, 0
  bend = []
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
    bend.append(bool(pl.active or pl.map_active or pl.s2 is not None))
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
  ok2, txt2 = budgets(cpu, bend, a.budget_ms, a.budget_bend_ms, a.max_ms)
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
  print(txt2)
  ok = p99 <= a.budget_ms and ok2
  print("DRY RUN OK" if ok else f"OVER BUDGET: p99 {p99:.2f} ms (budget {a.budget_ms} ms) or a budget above")
  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
