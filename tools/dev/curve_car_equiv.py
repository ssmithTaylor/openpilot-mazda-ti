#!/usr/bin/env python3
"""Frame-by-frame equivalence of the car controller's curve slowing (perf 2026-11-25): runs CarController.update (through
ti_cost_check's in-memory controller: no device params, no CAN) on simulated 100 Hz sequences with a deterministic clock and records,
every frame, the CAN frames returned, every params write (key, value) and every ti_curve log event. Run it once per tree and compare:

  python tools/dev/curve_car_equiv.py --out old.pkl      (PYTHONPATH on the old tree)
  python tools/dev/curve_car_equiv.py --out new.pkl      (PYTHONPATH on the new tree)
  python tools/dev/curve_car_equiv.py --compare old.pkl new.pkl

Scenarios: curve off; on with no corner; on and working (corners given and lifted, the set speed read back 80 ms after each tap); a
random walk (corners, stale and malformed planner messages incl. a string ceiling and a NaN, driver buttons, brake, gas, speed and
cruise changes, the switch toggled); stage 3's monitor saturated. Prints counts only. (Found 2026-11-25, both before and after: a string
`ceil` reaching Curve.off_view while the switch is OFF raises there and stops curve slowing until restart; the planner never sends one.)
"""
import argparse
import json
import math
import pickle
import random
import sys
from types import SimpleNamespace as NS


def run(scn, cycles, seed):
  from openpilot.selfdrive.car.mazda import ti_cost_check as K, ti_curve as tc, carcontroller as ccm
  cc, mem = K.make_controller()

  class Packer:                                          # every frame's name, bus and values (no CAN is built or sent)
    def make_can_msg(self, name, bus, values):
      return (name, bus, json.dumps(values, sort_keys=True, default=repr))
  cc.packer = Packer()
  writes, frames = [], []
  put0 = mem.put

  def put(k, v):
    rec = v if isinstance(v, (str, bytes)) else repr(v)
    try:
      d = json.loads(v)
      if isinstance(d, dict) and "started_at" in d:      # TiTuningStats' wall-clock start (another module's clock): not compared
        d.pop("started_at")
        rec = json.dumps(d, sort_keys=True)
    except Exception:
      pass
    writes.append((len(frames), k, rec))
    return put0(k, v)
  mem.put = put
  logs = []
  ccm.ti_curve_log = lambda ev: logs.append((len(frames), json.dumps(ev, sort_keys=True, default=repr)))
  clock = {"k": 0}
  ccm.time = NS(monotonic=lambda: clock["k"] * 0.01, time=lambda: 1_800_000_000.0 + clock["k"] * 0.01, perf_counter=lambda: 0.0)
  toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
  CC, CS = K.inputs(55.0, 0.0003)
  CS.lat_accel, CS.lkas_effective = 1.0, 100.0
  CS.out.steeringRateDeg = 0.0
  cs = CS.out.cruiseState
  rng = random.Random(seed)
  on, ceil, stale_until, bad = scn != "off", None, -1, None
  for k in range(cycles):
    clock["k"] = k
    if scn == "random":
      r = rng.random()
      if r < 0.002:
        on = not on
      if k % 400 == 0:
        ceil = rng.choice([None, None, 30, 40, 45, 50, 25])
      if rng.random() < 0.001:
        stale_until = k + rng.randrange(50, 400)
      bad = rng.choice([None] * 30 + ["ceil_str", "nan", "flag"]) if k % 20 == 0 else bad
      CS.out.brakePressed = rng.random() < 0.003 or (CS.out.brakePressed and rng.random() < 0.95)
      CS.out.gasPressed = rng.random() < 0.002 or (CS.out.gasPressed and rng.random() < 0.95)
      CS.crz_btns_driver = rng.random() < 0.002
      CS.crz_btns_set = CS.crz_btns_driver and rng.random() < 0.5
      CS.crz_lead = rng.random() < 0.3 if k % 300 == 0 else CS.crz_lead
      if rng.random() < 0.001:
        cs.enabled = not cs.enabled
      CC.actuators.curvature = rng.choice([0.0003, -0.004, 0.006, 0.0]) if k % 250 == 0 else CC.actuators.curvature
      CS.lat_accel = 1.0 + rng.gauss(0, 0.3)
    elif scn == "working":
      ceil = 40 if (k // 1000) % 2 == 0 else None
    elif scn == "monitor":
      CC.actuators.curvature = -0.004
    if k % 20 == 0 and k >= stale_until:
      mem.put(tc.PARAM_ON, b"1" if on else b"0")
      plan = {"m": k * 0.01, "seq": k, "mode": "act", "ceil": ceil, "conf": 1.0, "trusted": True, "known": True, "why": "",
              "dist": 100.0, "past": ceil is None, "model_la": 0.1, "model_k": 0.0003, "capped": False, "limit": None, "kind": "corner",
              "corner": ceil, "ahead": 2.0 + 0.01 * (k // 20 % 3) if scn in ("monitor", "random") else None,
              "ask": [2.0] * 6 if scn in ("monitor", "random") else None}
      if bad == "ceil_str":
        plan["ceil"] = "40"
      elif bad == "nan":
        plan["dist"] = float("nan")
      elif bad == "flag":
        plan["trusted"] = 2
      mem.put(tc.PARAM_PLAN, json.dumps(plan))
    if k % 10 == 3:
      CS.crz_btns_counter = (CS.crz_btns_counter + 1) % 16
    cv = cc.curve
    if cv is not None and cv.tap_to is not None and cv.tap_first is not None and k - cv.tap_first >= 8:
      cs.speed = cv.tap_to * 0.44704
      CS.out.vEgo = cs.speed
    elif scn == "random" and k % 100 == 50:
      CS.out.vEgo = max(5.0, CS.out.vEgo + rng.gauss(0, 0.5))
    out = cc.update(CC, CS, k * 10_000_000, toggles)
    sends = out[1] if isinstance(out, tuple) and len(out) > 1 else out
    frames.append(repr(sends))
  st = cc.curve.state if cc.curve is not None else "absent"
  return {"frames": frames, "writes": writes, "logs": logs, "state": st}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--out")
  ap.add_argument("--compare", nargs=2)
  ap.add_argument("--cycles", type=int, default=20000)
  a = ap.parse_args()
  if a.compare:
    A, B = (pickle.load(open(p, "rb")) for p in a.compare)
    bad = 0
    for name in A:
      x, y = A[name], B[name]
      for f in ("frames", "writes", "logs"):
        d = sum(1 for p, q in zip(x[f], y[f]) if p != q) + abs(len(x[f]) - len(y[f]))
        bad += d
        print(f"{name:10s} {f:7s}: {len(x[f])} / {len(y[f])}, differ {d}")
      print(f"{name:10s} end state {x['state']} / {y['state']}")
    sys.exit(1 if bad else 0)
  res = {}
  for scn, seed in (("off", 1), ("no_corner", 2), ("working", 3), ("monitor", 4), ("random", 5), ("random2", 6)):
    res[scn] = run("random" if scn.startswith("random") else scn.replace("no_corner", "on"), a.cycles, seed)
    r = res[scn]
    print(f"{scn}: frames {len(r['frames'])}, writes {len(r['writes'])}, logs {len(r['logs'])}, end state {r['state']}, "
          f"button frames {sum(f.count('CRZ_BTNS') for f in r['frames'])}")
  pickle.dump(res, open(a.out, "wb"))


if __name__ == "__main__":
  main()
