"""Decision equivalence: the planner before / after the cost port, tick by tick on the profiler's traces (memory v10).
open loop (the profiler's: 25 m/s, set 55) and closed loop (the set speed walks to the published ceiling 1 mph per 0.3 s, the speed
follows it through the cruise model). Prints counts and first differences only (indices, no positions)."""
import importlib.util
import sys
import time
import numpy as np
from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_cruise_model as CM


def load(path, name):
  sp = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(sp)
  sp.loader.exec_module(m)
  return m


OLD = load(sys.argv[1], "plan_old")
NEW = load(sys.argv[2], "plan_new")
mem = M.load(sys.argv[3])
km = np.asarray(mem.kap_med, np.float64)
traces = np.argsort(-np.diff(mem.trace_start))[:6]


def run(P, closed):
  pl = P.Planner(mem)
  out, ev, ms = [], [], []
  t = 0.0
  for tr in traces:
    s0, s1 = int(mem.trace_start[tr]), int(mem.trace_start[tr + 1])
    pl.reset()
    q = float(s0)
    setm, car, tap_t = 55, CM.Cruise(55.0, 55, 0.05), 0.0
    while q < s1 - 1:
      i = int(q)
      v = car.v * P.MPH if closed else 25.0
      ahead = np.array([10.0 * (k / 32) ** 2 for k in range(33)])
      kk = P.MODEL_K_SIGN * km[np.minimum(i + (ahead * v / mem.spacing).astype(int), s1 - 1)]
      fix = P.Fix(True, float(mem.x[i]), float(mem.y[i]), float(mem.hdg[i]))
      n0 = len(pl.events)
      c0 = time.perf_counter()
      p = pl.step(t, fix, v, set_mph=setm if closed else 55, yaw_k=float(km[i]), model_k=(ahead, kk), ref_mph=55, floor_mph=None,
                  acting=True, map_k=None, blinker=False)
      ms.append((time.perf_counter() - c0) * 1e3)
      out.append((p.ceil, p.kind, p.corner_mph, p.behind, p.alert_mph, p.stage, p.need_mph))
      ev.append(tuple(sorted((e["kind"], e.get("mode", ""), e.get("T", -1)) for e in pl.events[n0:])))
      if closed:
        goal = 55 if p.ceil is None else min(55, int(p.ceil))
        for _ in range(4):                              # 0.2 s in 0.05 s steps
          if t >= tap_t and setm != goal:
            setm += 1 if goal > setm else -1
            tap_t = t + CM.TAP_S
          car.step(setm)
          t += 0.05
        q += car.v * P.MPH * 0.2 / mem.spacing
      else:
        q += 25.0 * 0.2 / mem.spacing
        t += 0.2
  return out, ev, np.array(ms)


for closed in (False, True):
  (o1, e1, m1), (o2, e2, m2) = run(OLD, closed), run(NEW, closed)
  n = min(len(o1), len(o2))
  d_out = [k for k in range(n) if o1[k] != o2[k]]
  d_ev = [k for k in range(n) if e1[k] != e2[k]]
  print(f"{'closed' if closed else 'open'} loop: ticks {len(o1)} / {len(o2)}; output differs at {len(d_out)} ticks, events at {len(d_ev)}")
  for k in (d_out + d_ev)[:6]:
    print("  tick", k, "old", o1[k], e1[k], "new", o2[k], e2[k])
  # starts and pushes: the new one never more than one tick after the old one
  def marks(e, kinds):
    return [k for k, x in enumerate(e) if any(a[0] == "start" or (a[0] == "mode" and a[1] in kinds) for a in x)]
  a, b = marks(e1, ("push",)), marks(e2, ("push",))
  late = [k for k in a if not any(k <= j <= k + 1 for j in b)]
  print(f"  starts/pushes old {len(a)} new {len(b)}; old ones without a new one within one tick: {len(late)}")
  for tag, m in (("old", m1), ("new", m2)):
    print(f"  {tag}: mean {m.mean():.2f} ms, p99 {np.percentile(m, 99):.2f}, max {m.max():.2f}")
