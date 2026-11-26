"""Replayability guard (owner, 2026-11-25: "Let's make sure we're properly logging information about our features so that we can
enhance them after the fact"). Fails when the curve planner or the car side gains an input or a piece of state that is neither in
its log record nor in the commented exemption list (ti_curve.DIAG_STATE / DIAG_EXEMPT / DIAG_INPUTS / INPUTS_IN_RLOG;
ti_curve_plannerd.PLANNER_STEP_INPUTS / PLANNER_STATE). Adding one: log it, or exempt it with the reason a replay does not need it.
Synthetic inputs only."""
import inspect
import json

from openpilot.selfdrive.car.mazda import ti_curve as C, ti_curve_memory as M, ti_curve_plan as P, ti_curve_plannerd as D


def test_every_car_side_attribute_is_logged_or_exempt():
  cv = C.Curve()
  for f in range(0, 400):
    C.run_frame(cv, None, f, C.Inputs(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=55.0, set_mph=55,
                                      driver_button=False, counter=f // 10 % 16, plan=None, plan_age=None, mono=f * C.DT))
  names = set(vars(cv)) | {"bad_plan_logged", "_plan_checked"}
  missing = sorted(n for n in names if n not in C.DIAG_STATE and n not in C.DIAG_EXEMPT)
  assert not missing, f"Curve state not in the diag record nor exempt: {missing}"
  stale = sorted(n for n in list(C.DIAG_STATE) + list(C.DIAG_EXEMPT) if n not in names and n not in ("bad_plan_logged", "_plan_checked"))
  assert not stale, f"listed but no longer an attribute: {stale}"
  assert not set(C.DIAG_STATE) & set(C.DIAG_EXEMPT)


def test_every_car_side_input_is_logged():
  fields = set(C.Inputs._fields)
  missing = sorted(f for f in fields if f not in C.DIAG_INPUTS and f not in C.INPUTS_IN_RLOG)
  assert not missing, f"Inputs fields with no logged source: {missing}"
  assert set(C.DIAG_INPUTS) | set(C.INPUTS_IN_RLOG) <= fields


def test_diag_record_carries_every_listed_key_bounded_and_position_free():
  cv = C.Curve()
  plan = {"m": 0.0, "seq": 7, "mode": "act", "ceil": 40, "conf": 1.0, "trusted": True, "known": True, "why": "", "dist": 100.0,
          "past": False, "model_la": 0.1, "model_k": 0.0003, "capped": False, "limit": None, "kind": "corner", "corner": 40}
  rec = None
  for f in range(0, 600):
    inp = C.Inputs(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=55.0, set_mph=55, driver_button=False,
                   counter=f // 10 % 16, plan=dict(plan, m=f * C.DT), plan_age=0.05, mono=f * C.DT)
    C.run_frame(cv, None, f, inp)
    rec = cv.diag(f, inp)
    cv.events.clear()
  assert rec["kind"] == "cs" and rec["in"][-1] == 7
  keys = set(rec)
  assert set(C.DIAG_STATE.values()) <= keys, set(C.DIAG_STATE.values()) - keys
  text = json.dumps(rec)
  assert len(text) <= 700, len(text)
  for bad in ("lat", "lon", "latitude", "longitude", "vin"):
    assert bad not in text.lower()


def test_every_planner_input_and_attribute_is_logged_or_replayed():
  params = [p for p in inspect.signature(P.Planner.step).parameters if p != "self"]
  missing = sorted(p for p in params if p not in D.PLANNER_STEP_INPUTS)
  assert not missing, f"Planner.step inputs with no logged source: {missing}"
  tr = M.synthetic_trace([(200.0, 0.0), (150.0, 0.01), (300.0, 0.0)])
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  for k in range(80):
    s = 5.0 + k * 0.2 * 20.0
    j = min(int(s // M.SPACING_M), len(tr["x"]) - 1)
    pl.step(k * 0.2, P.Fix(True, float(tr["x"][j]), float(tr["y"][j]), float(tr["hdg"][j])), 20.0, set_mph=45.0,
            yaw_k=float(tr["kap"][j]), ref_mph=45.0, floor_mph=30, map_k=([0.0, 100.0], [0.0, 0.001]))
  names = set(vars(pl))
  missing = sorted(n for n in names if n not in D.PLANNER_STATE)
  assert not missing, f"Planner state neither logged nor replayable: {missing}"
  for n, where in D.PLANNER_STATE.items():
    if where in D.TICK_KEYS or where == "rm":
      continue
    assert where.startswith(("replayed", "fixed at start", "the planner events", "a cost cache")), (n, where)
