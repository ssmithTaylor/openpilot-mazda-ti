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


def test_frogpilot_reads_the_planners_warning_when_on_and_fresh():
  import json
  from openpilot.selfdrive.car.mazda.ti_curve import read_curve_alert

  class P_:
    def __init__(self, d):
      self.d = d

    def get(self, k):
      return json.dumps(self.d).encode() if self.d is not None else None
  assert read_curve_alert(P_({"mode": "act", "alert": 35, "m": 10.0}), 10.5) == (35, 10.0)
  assert read_curve_alert(P_({"mode": "shadow", "alert": 35, "m": 10.0}), 10.5)[0] is None       # OFF: never an alert
  assert read_curve_alert(P_({"mode": "act", "alert": 35, "m": 10.0}), 11.2)[0] is None          # older than 1.0 s (5 Hz writes)
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


def test_car_side_relays_the_lead_aware_floor_when_not_intervening():
  # fourth review, item 4: one floor source; while not intervening the car side's floor_now (lead-aware) is the planner's floor
  import json
  from openpilot.selfdrive.car.mazda import ti_curve as C

  class S:
    def __init__(self, d):
      self.raw = json.dumps(d).encode()

    def get(self, k):
      return self.raw
  st = {"t": 100.0, "state": "ready", "ref": 55, "floor": None, "floor_now": 25, "acting": True}
  assert D.car_side(S(st), 100.5, C) == (55, 25, True)
  st.update(state="slowing", floor=40)
  assert D.car_side(S(st), 100.5, C) == (55, 40, True)             # while it intervenes: its intervention's floor
  assert D.car_side(S(st), 104.0, C) == (None, None, False)         # stale


def test_car_side_floor_now_is_lead_aware():
  from openpilot.selfdrive.car.mazda import ti_curve as C
  c = C.Curve()
  c.ref = 55
  inp = C.Inputs(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=40.0, set_mph=55, lead=True, driver_button=False,
                 counter=0, plan=None, plan_age=None, mono=0.0)
  assert c._floor_for(inp, None) == 25                               # min(55, 40) - 15
  assert c._floor_for(inp._replace(lead=False) if hasattr(inp, "_replace") else C.Inputs(**{**inp.__dict__, "lead": False}), None) == 40


# ---- the map service's curvature list (synthetic coordinates near 0, 0: never a recorded position) ------------------------------
import json  # noqa: E402
import math  # noqa: E402


def _map_json(n=40, step_m=20.0, k=0.004, north=True):
  """a straight north-going (or south-going) line of nodes from 100 m south of (0, 0), curvature k at every node"""
  deg = step_m / (D.EARTH_R * math.pi / 180.0)
  lats = [(-100.0 / step_m + i) * deg for i in range(n)]
  if not north:
    lats = lats[::-1]
  return json.dumps([{"latitude": a, "longitude": 0.0, "curvature": k} for a in lats])


def test_map_profile_distances_along_the_road_from_the_car():
  d, k = D.map_profile(_map_json(), 0.5, 0.0, 0.0, 0.0)
  assert abs(d[0] + 20.0) < 0.5                         # the first kept node: 20 m behind (nodes from 30 m behind)
  assert all(b > a for a, b in zip(d, d[1:]))
  assert abs(d[-1] - (39 * 20.0 - 100.0)) < 1.0
  assert all(abs(x - 0.004) < 1e-12 for x in k)


def test_map_profile_no_opinion_when_stale_missing_unmatched_or_wrong_way():
  good = _map_json()
  assert D.map_profile(good, D.MAP_STALE_S + 0.1, 0.0, 0.0, 0.0) is None        # stale
  assert D.map_profile(good, None, 0.0, 0.0, 0.0) is None                         # no file
  assert D.map_profile(None, 0.5, 0.0, 0.0, 0.0) is None
  assert D.map_profile("[]", 0.5, 0.0, 0.0, 0.0) is None                          # mapd's "no way" output
  assert D.map_profile("{}", 0.5, 0.0, 0.0, 0.0) is None
  assert D.map_profile("not json", 0.5, 0.0, 0.0, 0.0) is None
  assert D.map_profile(good, 0.5, 0.0, 0.001, 0.0) is None                        # ~111 m to the side: not this road
  assert D.map_profile(good, 0.5, 0.0, 0.0, 180.0) is None                        # the map's way runs the other way
  assert D.map_profile(_map_json(north=False), 0.5, 0.0, 0.0, 180.0) is not None
  bad = json.loads(good)
  bad[3]["curvature"] = float("nan")
  assert D.map_profile(json.dumps(bad), 0.5, 0.0, 0.0, 0.0) is None


def test_map_age_absent_file_is_none(tmp_path):
  assert D.map_age_s(str(tmp_path / "MapCurvatures")) is None
  p = tmp_path / "MapCurvatures"
  p.write_text("[]")
  assert 0.0 <= D.map_age_s(str(p)) < 5.0


def test_map_profile_never_returns_positions():
  # synthetic origin away from (0, 0) so a leaked coordinate would be recognisable (never a recorded position)
  lat0, lon0, step_m, n = 10.37, -71.73, 20.0, 40
  deg = step_m / (D.EARTH_R * math.pi / 180.0)
  pts = [{"latitude": lat0 + (-100.0 / step_m + i) * deg, "longitude": lon0, "curvature": 0.001 * (1 + i % 3)} for i in range(n)]
  out = D.map_profile(json.dumps(pts), 0.5, lat0, lon0, 0.0)
  assert out is not None
  assert isinstance(out, tuple) and len(out) == 2                   # (distances, curvatures) and nothing else
  d, k = out
  assert len(d) == len(k) >= 2
  coords = {p["latitude"] for p in pts} | {p["longitude"] for p in pts} | {lat0, lon0}
  for x in list(d) + list(k):
    assert isinstance(x, float) and math.isfinite(x)
    assert all(abs(x - c) > 1e-3 for c in coords)                   # no latitude / longitude passes through
  assert -30.0 <= d[0] and d[-1] <= n * step_m                      # metres along the road from the car
  assert all(0.0 <= x <= 0.0031 for x in k)                          # |curvature| as given
  assert "lat" not in repr(out).lower() and "lon" not in repr(out).lower()


def test_map_input_passes_the_profile_only_when_fresh_and_positioned(tmp_path):
  # plan T6: plannerd passes map_k = map_profile(...) of the map service's list; None without a valid position, when the list is
  # absent, or stale (older than MAP_STALE_S)
  f = tmp_path / "MapCurvatures"
  f.write_text(_map_json())

  class PM:
    def __init__(self, raw):
      self.raw = raw

    def get(self, key):
      assert key == D.MAP_PARAM
      return self.raw
  now = f.stat().st_mtime + 0.5
  mk = D.map_input(PM(_map_json().encode()), True, 0.0, 0.0, 0.0, now=now, path=str(f))
  assert mk is not None and mk == D.map_profile(_map_json(), 0.5, 0.0, 0.0, 0.0)
  assert D.map_input(PM(_map_json().encode()), False, 0.0, 0.0, 0.0, now=now, path=str(f)) is None
  assert D.map_input(PM(None), True, 0.0, 0.0, 0.0, now=now, path=str(f)) is None
  assert D.map_input(PM(_map_json().encode()), True, 0.0, 0.0, 0.0, now=now + D.MAP_STALE_S, path=str(f)) is None
  assert D.map_input(PM(_map_json().encode()), True, 0.0, 0.0, 0.0, now=now, path=str(tmp_path / "absent")) is None


# ---- plan T8: the per-tick log ---------------------------------------------------------------------------------------------------
from openpilot.selfdrive.car.mazda import ti_curve_memory as TM  # noqa: E402
from openpilot.selfdrive.car.mazda import ti_curve_plan as TP  # noqa: E402


def _worst_extras():
  """Every field of the tick log at its widest (signed, most digits) value."""
  return dict(seq=9999999, v=-88.88, set=-88, gap=-88.88, a=-8.888, dec=-8.888,
              mem={"pk": -9999, "pkd": -9999, "pkn": 255, "s": [-9999] * 8},
              fc=[-99.99] * 7, s1=[99, -99, 2, "behind", 1], s2=[-99, -99, -99.9],
              mon=["assist off: 0x11 stuck request", -99], car=["restoring", -99, -999, -999],
              lay=-99.99, lacc=-99.99, des=-99.99, rate=-9999.9, cmd=-9999, ti=[255, 255],
              ln=[-9.99, 0.99, -9.99, 0.99], drv=63, tq=-999, cpu=99999, side=-1)


def _worst_plan():
  diag = {"stage": 2, "ceil": 88, "kind": "corner", "need": -88, "cap": -8.88, "ahead": -88.88, "behind": True, "alert": -88,
          "src": "map", "rel": 99}
  return TP.Plan(88, 1.0, True, True, "next bend (map): holding what it needs", -9999.9, 88, False, -88.88, True, need_mph=-88,
                 alert_mph=-88, stage=2, src="map", ahead_la=-88.88, cap=-8.88, released=True, diag=diag, kind="corner")


def test_tick_log_says_why_the_car_is_beside_the_remembered_road():
  # road matching (2026-11-24): the tick log carries why the planner judged the car beside the locked trace; absent otherwise
  p = _worst_plan()
  assert "rm" not in D.tick_log(p, _worst_extras())
  p = p._replace(diag=dict(p.diag, rm="link"))
  d = D.tick_log(p, _worst_extras())
  assert d["rm"] == "link"
  assert len(json.dumps(d, separators=(",", ":"))) <= D.TICK_MAX_B


def test_tick_log_size_bound():
  # plan T8: <= TICK_MAX_B bytes per message (TICK_MAP_MAX_B on a tick whose map list changed), every field at its widest
  e = _worst_extras()
  d = D.tick_log(_worst_plan(), e)
  assert len(json.dumps(d, separators=(",", ":"))) <= D.TICK_MAX_B == 700
  e["map"] = [[-9999, -9999]] * 12
  d = D.tick_log(_worst_plan(), e)
  assert "map" in d and len(d["map"]) == 12
  assert len(json.dumps(d, separators=(",", ":"))) <= D.TICK_MAP_MAX_B == 900
  # more than the log may carry (a longer map list, a long string): trimmed to the bound, never over it
  e["map"] = [[-99999, -99999]] * 40
  e["mon"] = ["x" * 2000, 1]
  d = D.tick_log(_worst_plan(), e)
  assert len(json.dumps(d, separators=(",", ":"))) <= D.TICK_MAP_MAX_B
  assert len(d.get("map", [])) <= 12
  # the fields later analysis needs are there on a normal tick (A10: gap and deceleration after every drive)
  d = D.tick_log(_worst_plan(), _worst_extras())
  for k in ("gap", "dec", "a", "set", "v", "st", "ceil", "need", "cap", "side", "fc", "s1", "s2", "mon", "ti", "lay", "lacc", "ln",
            "drv", "tq", "cpu", "rate", "cmd", "des", "rel", "alert"):
    assert k in d, k


def _memory_far_from_origin():
  # synthetic road (never a recorded position): a straight then a left bend, at a recognisable origin
  tr = TM.synthetic_trace([(200.0, 0.0), (150.0, 0.01), (300.0, 0.0)])
  mem = TM.synthetic([tr])
  mem.lat0, mem.lon0 = 10.37, -71.73
  return mem, tr


def test_tick_log_has_no_position():
  mem, tr = _memory_far_from_origin()
  pl = TP.Planner(mem)
  for k in range(60):
    s = 5.0 + k * 0.2 * 20.0
    j = min(int(s // TM.SPACING_M), len(tr["x"]) - 1)
    p = pl.step(k * 0.2, TP.Fix(True, float(tr["x"][j]), float(tr["y"][j]), float(tr["hdg"][j])), 20.0, set_mph=45.0,
                yaw_k=float(tr["kap"][j]), ref_mph=45.0)
  ms = D.mem_summary(pl)
  assert ms is not None and set(ms) == {"pk", "pkd", "pkn", "s"} and len(ms["s"]) == 8
  e = _worst_extras()
  e.update(mem=ms, lat=mem.lat0, lon=mem.lon0, latitude=mem.lat0, longitude=mem.lon0, x=float(tr["x"][10]), y=float(tr["y"][10]),
           map=[[10, 5], [20, 7]])
  d = D.tick_log(p, e)
  coords = {mem.lat0, mem.lon0}
  text = json.dumps(d)
  for bad in ("lat", "lon", '"x"', '"y"', "hdg"):
    assert bad not in text, bad

  def walk(o):
    if isinstance(o, dict):
      for v in o.values():
        yield from walk(v)
    elif isinstance(o, (list, tuple)):
      for v in o:
        yield from walk(v)
    elif isinstance(o, (int, float)) and not isinstance(o, bool):
      yield float(o)
  for x in walk(d):
    assert all(abs(x - c) > 1e-3 for c in coords), x          # no latitude / longitude passes through
  assert set(d) <= set(D.TICK_KEYS)


def test_tick_log_decodes_the_interceptor_and_accelerometer():
  # TI_FEEDBACK (0x24A, body bus 1): STATE byte 3, VIOL byte 4; 0x075 bus 0 lateral acceleration (ay_raw - 2000 + 14.31) / 93.857
  assert D.ti_from_can([frame(0x24A, [0, 0, 0, 3, 0x11, 0, 0, 0], src=1)]) == (3, 0x11)
  assert D.ti_from_can([frame(0x24A, [0, 0, 0, 3, 0x11, 0, 0, 0], src=0)]) is None
  raw = 2000 + 188
  la = D.lat_accel_from_can([frame(0x75, [0, 0, (raw >> 8) & 0x0F, raw & 0xFF, 0, 0, 0, 0])])
  assert abs(la - (188 + 14.31) / 93.857) < 1e-9
  assert D.lat_accel_from_can([frame(0x76, [0] * 8)]) is None


# ---- 2026-11-25: a different driving model (heavier on the CPU): a stale preview is no preview -----------------------------------
def _model(drop=0.0):
  t = [0.0, 1.0, 2.0, 4.0]
  return NS(orientationRate=NS(z=[0.0, 0.01, 0.02, 0.02], t=t), velocity=NS(x=[20.0] * 4), frameDropPerc=drop)


def test_fresh_model_is_passed_through():
  mk, why = D.fresh_model(_model(), int(100.0 * 1e9), 100.1)
  assert mk is not None and why == ""


def test_old_model_is_no_preview():
  mk, why = D.fresh_model(_model(), int(100.0 * 1e9), 100.0 + D.MODEL_MAX_AGE_S + 0.05)
  assert mk is None and why == "old"
  assert D.fresh_model(_model(), int(100.0 * 1e9), 100.0 + D.MODEL_MAX_AGE_S - 0.05)[0] is not None


def test_dropping_model_is_no_preview():
  mk, why = D.fresh_model(_model(drop=D.MODEL_MAX_DROP_PERC + 1.0), int(100.0 * 1e9), 100.05)
  assert mk is None and why == "drops"
  assert D.fresh_model(_model(drop=D.MODEL_MAX_DROP_PERC), int(100.0 * 1e9), 100.05)[0] is not None
  assert D.fresh_model(None, 0, 100.0) == (None, "none")


def test_tick_log_carries_model_name_step_time_and_stale_reason():
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  plan = P.Plan(None, 1.0, True, True, "", None, None, False, None, False)
  d = D.tick_log(plan, dict(seq=1, fcx="old", mdl="dark-souls", step_us=812))
  assert d["fcx"] == "old" and d["mdl"] == "dark-souls" and d["step_us"] == 812


def test_model_name_read_from_the_param():
  class Pm:
    def get(self, k, encoding=None):
      return "dark-souls" if k == "Model" else None
  assert D.model_name(Pm()) == "dark-souls"
  assert D.model_name(object()) == ""
