"""The planner process's helpers (ti_curve_plannerd.py): the yaw decode and the model curvature. Synthetic frames only."""
from types import SimpleNamespace as NS

from openpilot.selfdrive.car.mazda import ti_curve_plannerd as D


def frame(addr, dat, src=0):
  return NS(address=addr, dat=bytes(dat), src=src)


def test_yaw_decode_matches_the_study():
  # raw = 18000: -(18000 * -0.00017243 + 3.1032) = 3.1037 - 3.1032 ... left positive
  raw = 18000
  y = D.yaw_from_can([frame(0x75, [0, 0, 0, 0, raw >> 8, raw & 0xFF, 0, 0])])
  assert abs(y - (-(raw * D.YAW_SCALE + D.YAW_OFF))) < 1e-12
  zero = round(-D.YAW_OFF / D.YAW_SCALE)                 # the count that reads no yaw
  assert abs(D.yaw_from_can([frame(0x75, [0, 0, 0, 0, zero >> 8, zero & 0xFF, 0, 0])])) < 2e-4


def test_yaw_decode_takes_the_newest_bus0_frame_only():
  a = frame(0x75, [0, 0, 0, 0, 0x46, 0x50, 0, 0])
  b = frame(0x75, [0, 0, 0, 0, 0x47, 0x00, 0, 0])
  other_bus = frame(0x75, [0, 0, 0, 0, 0x10, 0x00, 0, 0], src=2)
  assert D.yaw_from_can([a, b, other_bus]) == D.yaw_from_can([b])
  assert D.yaw_from_can([frame(0x76, [0] * 8)]) is None


def test_model_curvature():
  m = NS(orientationRate=NS(z=[0.0, 0.1, 0.2], t=[0.0, 1.0, 2.0]), velocity=NS(x=[20.0, 20.0, 1.0]))
  t, k = D.model_curvature(m)
  assert t == [0.0, 1.0, 2.0] and abs(k[1] - 0.005) < 1e-12 and abs(k[2] - 0.2 / 3.0) < 1e-12
  assert D.model_curvature(NS(orientationRate=NS(z=[], t=[]), velocity=NS(x=[]))) is None


def test_speed_limit_only_when_known_and_plausible():
  st = {"t": None}
  assert D.fresh_limit(True, 55 * D.MPH, 50 * D.MPH, True, 0.0, st) == pytest_approx(55)
  assert D.fresh_limit(False, 55 * D.MPH, 50 * D.MPH, True, 0.0, st) is None      # planner message not alive (toggle off)
  assert D.fresh_limit(True, 55 * D.MPH, 50 * D.MPH, False, 0.0, st) is None      # no position
  assert D.fresh_limit(True, 0.0, 50 * D.MPH, True, 0.0, st) is None              # unknown
  assert D.fresh_limit(True, 25 * D.MPH, 55 * D.MPH, True, 0.0, st) is None       # 25 on a 55 mph drive: implausible


def pytest_approx(x):
  import pytest
  return pytest.approx(x, abs=1e-6)


def test_speed_limit_far_above_the_speed_is_not_used():
  assert D.fresh_limit(True, 70 * D.MPH, 45 * D.MPH, True, 0.0, {"t": None}) is None


def test_frogpilot_reads_the_curve_warning_from_the_car_side_when_fresh():
  import json
  from openpilot.selfdrive.car.mazda.ti_curve import read_curve_alert

  class P_:
    def __init__(self, d):
      self.d = d

    def get(self, k):
      return json.dumps(self.d).encode() if self.d is not None else None
  assert read_curve_alert(P_({"state": "holding", "alert": 35, "t": 10.0}), 10.5) == (35, 10.0)
  assert read_curve_alert(P_({"state": "off", "alert": None, "t": 10.0}), 10.5)[0] is None       # OFF: never an alert
  assert read_curve_alert(P_({"state": "holding", "alert": 35, "t": 10.0}), 12.0)[0] is None     # stale
  assert read_curve_alert(P_(None), 10.5)[0] is None


def test_h2_every_planner_log_call_with_real_events_does_not_raise():
  import json
  from openpilot.selfdrive.car.mazda import ti_curve_memory as M, ti_curve_plan as P
  calls = []

  def event(name, **kw):                     # cloudlog.event's signature: a duplicate keyword would raise TypeError at the call
    json.dumps(kw)
    calls.append(kw)
  tr = M.synthetic_trace([(1500, 0.0), (150, 1 / 150.0), (800, 0.0)])
  mem = M.synthetic([tr])
  pl = P.Planner(mem)
  s_road = np.arange(len(tr["x"])) * M.SPACING_M
  kinds = set()
  for k in range(500):
    s = 55 * D.MPH * k * 0.2
    if s > s_road[-1]:
      break
    x, y = np.interp(s, s_road, tr["x"]), np.interp(s, s_road, tr["y"])
    plan = pl.step(k * 0.2, P.Fix(True, float(x), float(y), float(tr["hdg"][int(s // 5)])), 55 * D.MPH, set_mph=55, yaw_k=0.0,
                   ref_mph=55)
    d = P.plan_json(plan, k * 0.2, k, "act")
    for ev in pl.events:
      kinds.add(ev["kind"])
      event("ti_curve_plan", **D.log_payload(ev, feature=d["mode"]))
    pl.events.clear()
    event("ti_curve_plan", **D.log_payload(d, kind="decision", feature=d["mode"], mph=55.0, set_mph=55, trace=plan.trace,
                                           point=plan.point))
  assert {"lock", "start", "mode"} <= kinds and calls
  # the installed pattern raised on every mode event (a duplicate "mode" keyword)
  ev = {"kind": "mode", "mode": "target"}
  import pytest
  with pytest.raises(TypeError):
    event("ti_curve_plan", mode="act", **ev)


import numpy as np  # noqa: E402
