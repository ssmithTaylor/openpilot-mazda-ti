#!/usr/bin/env python3
"""Recorded-input equivalence of the control core's per-cycle code (core-4 CPU work, 2026-11-26): runs the per-frame scenarios of
selfdrive/car/mazda/ti_mem_check.py (the same seeded inputs every run, a simulated clock) and records every output, every cycle:

  lateral       LatControlTorque.update with the TI plant: the returned steer (exact bits) and the whole LateralTorqueState message
                (mazdaDiagnostics included) as bytes, after lateral_diagnostics.record_inputs
  card_tests / card_curve / card_buttons
                CarController.update: the returned actuators (bytes), every CAN frame (name, bus, values), the carOutput
                diagnostics message (bytes), every params write (key, value) and every curve log event
  ab_runner     TiAbRunner.step: the toggles it returns (repr) and its events
  lane_drift    LaneDriftGate.update: the gate's answer and its latch

Run it once per tree and compare (the first difference is printed with its scenario and cycle):

  python tools/dev/core_equiv.py --out old.pkl      (PYTHONPATH on the old tree)
  python tools/dev/core_equiv.py --out new.pkl      (PYTHONPATH on the new tree)
  python tools/dev/core_equiv.py --compare old.pkl new.pkl

Not per frame (a development tool): it wraps the classes it records.
"""
import argparse
import json
import pickle
import sys

CYCLES = {"lateral": 6000, "card_tests": 9000, "card_curve": 12000, "card_buttons": 6000, "ab_runner": 12000, "lane_drift": 6000}


def _drop_wall_clock(v):
  try:
    d = json.loads(v)
  except Exception:
    return v if isinstance(v, (str, bytes)) else repr(v)
  if isinstance(d, dict):
    d.pop("started_at", None)                       # TiTuningStats' wall-clock start (another module's clock): not compared
    if isinstance(d.get("_live"), dict):
      d["_live"].pop("t", None)                      # TiTestProgress' wall-clock stamp (ti_tests.py reads time.time itself)
  return json.dumps(d, sort_keys=True, default=repr)


def record(name, cycles):
  from openpilot.selfdrive.car.mazda import ti_mem_check as M
  out = []
  undo = []

  def wrap(cls, attr, fn):
    orig = getattr(cls, attr)
    setattr(cls, attr, fn(orig))
    undo.append((cls, attr, orig))

  if name == "lateral":
    from openpilot.selfdrive.controls.lib import latcontrol_torque as ltq
    from openpilot.selfdrive.car.mazda import lateral_diagnostics as ld
    last = []

    def upd(orig):
      def f(self, *a, **k):
        r = orig(self, *a, **k)
        last[:] = [r]
        out.append(("steer", float(r[0]).hex(), float(r[1]).hex(), type(r[0]).__name__))
        return r
      return f
    wrap(ltq.LatControlTorque, "update", upd)
    rec0 = ld.record_inputs

    def rec(diag, *a, **k):
      r = rec0(diag, *a, **k)
      out.append(("log", last[0][2].to_bytes()))
      return r
    import openpilot.selfdrive.car.mazda.lateral_diagnostics as ldm
    ldm.record_inputs = rec
    undo.append((ldm, "record_inputs", rec0))
  elif name.startswith("card_"):
    from openpilot.selfdrive.car.mazda import carcontroller as ccm, ti_cost_check as K

    def upd(orig):
      def f(self, CC, CS, now, tog):
        act, sends = orig(self, CC, CS, now, tog)
        diag = getattr(self, "lateral_diagnostics", None)
        out.append(("act", repr((act.steer, act.curvature, act.steerOutputCan, act.accel)) if not hasattr(act, "to_bytes") else act.to_bytes(),
                    repr(sends), None if diag is None else diag.to_bytes()))
        return act, sends
      return f
    wrap(ccm.CarController, "update", upd)

    def put(orig):
      def f(self, k, v):
        out.append(("put", k, _drop_wall_clock(v)))
        return orig(self, k, v)
      return f
    wrap(K.MemParams, "put", put)
    K.MemParams.put_nonblocking = K.MemParams.put
    undo.append((K.MemParams, "put_nonblocking", undo[-1][2]))
  elif name == "ab_runner":
    from openpilot.selfdrive.car.mazda import ti_ab

    def step(orig):
      def f(self, *a, **k):
        r = orig(self, *a, **k)
        out.append(("ab", repr(sorted(vars(r).items())) if hasattr(r, "__dict__") else repr(r), repr(self.ab.mode), repr(self.plan.mode)))
        return r
      return f
    wrap(ti_ab.TiAbRunner, "step", step)
  elif name == "lane_drift":
    from openpilot.selfdrive.controls.lib import lane_drift as LD

    def upd(orig):
      def f(self, *a, **k):
        r = orig(self, *a, **k)
        out.append(("gate", r, self.latch_left))
        return r
      return f
    wrap(LD.LaneDriftGate, "update", upd)
  setup = M.SCENARIOS[name][0]
  step, teardown = setup()
  try:
    for i in range(cycles):
      step(i)
  finally:
    teardown()
    for cls, attr, orig in reversed(undo):
      setattr(cls, attr, orig)
  return out


def main(argv=None):
  ap = argparse.ArgumentParser()
  ap.add_argument("--out")
  ap.add_argument("--compare", nargs=2)
  ap.add_argument("--only", default="")
  a = ap.parse_args(argv)
  if a.compare:
    old, new = (pickle.load(open(p, "rb")) for p in a.compare)
    bad = 0
    for name in sorted(set(old) | set(new)):
      o, n = old.get(name), new.get(name)
      if o is None or n is None:
        print(f"{name:14s} MISSING in {'old' if o is None else 'new'}")
        bad += 1
        continue
      diff = next((i for i, (x, y) in enumerate(zip(o, n)) if x != y), None)
      if diff is None and len(o) == len(n):
        print(f"{name:14s} identical: {len(o)} records")
      else:
        bad += 1
        i = diff if diff is not None else min(len(o), len(n))
        print(f"{name:14s} DIFFER at record {i} of {len(o)}/{len(n)}: old {str(o[i])[:200] if i < len(o) else None} new {str(n[i])[:200] if i < len(n) else None}")
    print("ALL IDENTICAL" if not bad else f"{bad} scenario(s) differ")
    return 1 if bad else 0
  names = [n for n in CYCLES if not a.only or n in a.only.split(",")]
  res = {n: record(n, CYCLES[n]) for n in names}
  for n in names:
    print(f"{n:14s} {len(res[n])} records")
  pickle.dump(res, open(a.out, "wb"))
  return 0


if __name__ == "__main__":
  sys.exit(main())
