#!/usr/bin/env python3
"""Curve slowing planner process (ti_curve_plannerd): runs ti_curve_plan.Planner at 5 Hz, OUTSIDE the two 100 Hz control processes, at
normal priority, kept off the control core (core 4: card and controlsd), and publishes its small output to the tmpfs param
ti_curve.PARAM_PLAN for the car controller (ti_curve.py). It always runs while onroad: with curve slowing OFF it is the SHADOW mode (it
plans and logs what it would have done; the car controller sends nothing).

Reads: liveLocationKalman (position, heading, validity), carState (speed, set speed), modelV2 (the driving model's predicted curvature
over the next seconds), the map service's curvature list (MAP_PARAM, as map_k: plan T6), frogpilotPlan.slcMapSpeedLimit (the road's speed limit from the offline maps; 0 = unknown), the car's yaw
sensor from CAN 0x075 (bytes 4-5, -0.00017243 rad/s per count, offset 3.1032, right positive; the newest frame in the latest CAN
batch, else liveLocationKalman's calibrated yaw rate). The road memory: MEMORY_PATH, copied to the device by hand (it is private: where
the owner has driven); without it the planner publishes "no road memory on the device" and nothing happens.

Logs (`ti_curve_plan` channel): every change of the decision (ceiling, kind, trusted, reason) and the planner's events (lock, lock lost,
road changed, corner start with the planned and actual speed and the grade, mode changes, corner passed), a summary every 5 s, and
(plan T8) one `kind="tick"` record every tick (tick_log: <= TICK_MAX_B bytes, TICK_MAP_MAX_B on a tick whose map list changed).
Never a position: trace and point indices, distances, speeds and curvatures only.
"""
import json
import math
import os
import time

MEMORY_PATH = "/data/ti_curve/curve_memory_v1.npz"
RATE_HZ = 5.0
SUMMARY_S = 5.0
CONTROL_CORE = 4
MPH = 0.44704
YAW_SCALE, YAW_OFF = -0.00017243215217366658, 3.103220229446863


def yaw_from_can(can_msg):
  """Left-positive yaw rate (rad/s) from the newest 0x075 frame on bus 0 of one CAN batch, or None."""
  for f in reversed(can_msg):
    if f.address == 0x75 and f.src == 0 and len(f.dat) >= 6:
      raw = (f.dat[4] << 8) | f.dat[5]
      return -(raw * YAW_SCALE + YAW_OFF)
  return None


def model_curvature(m):
  """(seconds, curvature 1/m) of the model's preview: orientation rate z / speed x, or None."""
  try:
    z, vx, t = m.orientationRate.z, m.velocity.x, m.orientationRate.t
    if len(z) < 2 or len(z) != len(vx) or len(z) != len(t):
      return None
    return [float(x) for x in t], [float(a) / max(float(b), 3.0) for a, b in zip(z, vx)]
  except Exception:
    return None


MODEL_MAX_AGE_S = 0.3          # a driving-model preview older than this (its log time against this tick) is no preview (2026-11-25:
MODEL_MAX_DROP_PERC = 20.0      # a different, heavier driving model on the CPU); nor one whose frameDropPerc is above this


def fresh_model(m, log_mono_ns, now_s):
  """The model preview for the planner, or None when it is stale: -> (model_curvature(m) or None, why: "" / "old" / "drops" /
  "none"). Stale counts as no preview everywhere (stage 2 does not act; stage 1's confirmation and release read nothing: the
  remembered bend is kept)."""
  if m is None or not log_mono_ns:
    return None, "none"
  if now_s - float(log_mono_ns) * 1e-9 > MODEL_MAX_AGE_S:
    return None, "old"
  try:
    if float(m.frameDropPerc) > MODEL_MAX_DROP_PERC:
      return None, "drops"
  except Exception:
    pass
  mk = model_curvature(m)
  return mk, "" if mk is not None else "none"


def model_name(params):
  """The driving model's name as the Model param holds it (read once at start), or ""."""
  try:
    return (params.get("Model", encoding="utf-8") or "")[:24]
  except Exception:
    return ""


MAP_PARAM = "MapCurvatures"     # written by the map service (pfeiferj mapd v1.12.0, ~1 Hz) to /dev/shm/params: a JSON list of
MAP_FILE = "/dev/shm/params/d/" + MAP_PARAM   # {"latitude", "longitude", "curvature" (1/m, unsigned, 3-node average)} for the nodes of the
MAP_STALE_S = 3.0               # current way from its first node in the direction of travel, then the ways it predicts next
MAP_MATCH_M = 30.0              # the car must be within this of the map's polyline ...
MAP_HDG_DEG = 45.0              # ... on a piece pointing within this of the car's heading, or the map has no opinion
EARTH_R = 6371000.0


def map_profile(raw, age_s, lat, lon, bearing_deg):
  """The map service's curvature list as (metres along the road from the car, |curvature|), for the planner; None (no map opinion) when
  missing, older than MAP_STALE_S, unreadable, or not matched to the car (no polyline piece within MAP_MATCH_M pointing its way).
  Positions are used here only and never leave this function."""
  try:
    if raw is None or age_s is None or not (0.0 <= age_s <= MAP_STALE_S):
      return None
    pts = json.loads(raw)
    if not isinstance(pts, list) or len(pts) < 2:
      return None
    la = [float(p["latitude"]) for p in pts]
    lo = [float(p["longitude"]) for p in pts]
    ks = [abs(float(p["curvature"])) for p in pts]
    if not all(math.isfinite(a) and math.isfinite(b) and math.isfinite(c) for a, b, c in zip(la, lo, ks)):
      return None
    cl = math.cos(math.radians(lat))
    xs = [math.radians(b - lon) * cl * EARTH_R for b in lo]
    ys = [math.radians(a - lat) * EARTH_R for a in la]
    hx, hy = math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))
    cum = [0.0]
    for i in range(len(xs) - 1):
      cum.append(cum[-1] + math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]))
    best = None
    for i in range(len(xs) - 1):
      dx, dy = xs[i + 1] - xs[i], ys[i + 1] - ys[i]
      L = math.hypot(dx, dy)
      if L < 0.1 or (dx * hx + dy * hy) / L < math.cos(math.radians(MAP_HDG_DEG)):
        continue
      tt = max(0.0, min(1.0, -(xs[i] * dx + ys[i] * dy) / (L * L)))
      lat_m = math.hypot(xs[i] + tt * dx, ys[i] + tt * dy)
      if lat_m <= MAP_MATCH_M and (best is None or lat_m < best[0]):
        best = (lat_m, cum[i] + tt * L)
    if best is None:
      return None
    d = [c - best[1] for c in cum]
    keep = [j for j in range(len(d)) if d[j] >= -30.0 and (j == 0 or cum[j] > cum[j - 1])]
    if len(keep) < 2:
      return None
    return [d[j] for j in keep], [ks[j] for j in keep]
  except Exception:
    return None


def map_age_s(path=MAP_FILE, now=None):
  """Seconds since the map service last wrote its list (file time on tmpfs); None when absent."""
  try:
    return (time.time() if now is None else now) - os.path.getmtime(path)
  except OSError:
    return None


def map_input(params_memory, ok, lat, lon, bearing_deg, now=None, path=MAP_FILE):
  """The planner's map_k (plan T6): map_profile of the map service's list when the position is valid (ok), else None. Absent, stale
  (older than MAP_STALE_S) or unmatched: None (no map stage). Positions stay here and in map_profile."""
  if not ok:
    return None
  try:
    raw = params_memory.get(MAP_PARAM)
  except Exception:
    return None
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", "replace")
  return map_profile(raw, map_age_s(path, now), lat, lon, bearing_deg)


LIMIT_MAX_AGE_S = 2.0           # frogpilotPlan must have been received within this (it is published at 20 Hz)
LIMIT_MAX_BELOW_MPH = 15.0      # a limit more than this below OR above the car's speed is implausible (a stale or wrong value)


def fresh_limit(alive, limit_ms, v_ms, position_ok, t, state):
  """The road's speed limit in mph when it is KNOWN: frogpilotPlan alive (the speed-limit toggle on: the planner fills it only then),
  a position fix now, a value above zero, and plausible against the car's speed (not more than LIMIT_MAX_BELOW_MPH below it). The
  car side uses a known limit only to slow LESS (floor = max(speed - 15, limit - 15, 25)), so a wrong value can never add slowing."""
  if not alive or not position_ok or limit_ms <= 0.5:
    state["t"] = None
    return None
  mph = limit_ms / MPH
  if abs(mph - v_ms / MPH) > LIMIT_MAX_BELOW_MPH:
    return None                   # more than 15 mph from the car's speed, either way: stale or wrong, not used
  state["t"] = t
  return mph


def log_payload(*parts, **extra):
  """One dict for cloudlog.event from several dicts: later keys win, so a planner event's own "kind" / "mode" can never collide with
  ours (2026-11-23: duplicate keywords raised on every call; 7,268 error lines and 4 planner log lines in the drive)."""
  out = {}
  for d in parts:
    out.update(d)
  out.update(extra)
  return out


def car_side(params_memory, now_wall, tc):
  """(ref, floor, acting) from the car side's published status (TiCurveStatus) when fresh: its reference (the driver's own set speed),
  its floor while it intervenes, and whether it will act on a corner now (ON and ready / intervening, not paused, stopped, waiting or
  holding a set speed left to the driver). (None, None, False) when stale or unreadable: then the warning assumes nothing is done."""
  try:
    raw = params_memory.get(tc.PARAM_STATUS)
    st = json.loads(raw) if raw else None
    if not isinstance(st, dict) or now_wall - float(st.get("t", 0.0)) > 3.0:
      return None, None, False
    ref = st.get("ref")
    # its floor while it intervenes, else the floor it would take now (lead-aware: a lead holding the car slower lowers it)
    floor = st.get("floor") if st.get("state") in ("slowing", "holding", "restoring") else None
    if not floor:
      floor = st.get("floor_now")
    return (int(ref) if ref else None), (int(floor) if floor else None), bool(st.get("acting"))
  except Exception:
    return None, None, False


# ---- plan T8: the per-tick log (5 Hz, `ti_curve_plan` channel, kind "tick"); never a position ----------------------------------------
TICK_MAX_B = 700                # bytes per message (json.dumps default separators: what cloudlog writes is no larger)
TICK_MAP_MAX_B = 900            # ... on a tick that carries a changed map list
TICK_FC_LEADS = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0)   # calibrated forecast demand at these leads (s)
TICK_MEM_STEP_M, TICK_MEM_N = 50.0, 8                  # the road memory's median curvature every 50 m to 400 m
TICK_MAP_N = 12
TICK_STR_MAX = 40
TI_FEEDBACK_ADDR, TI_FEEDBACK_BUS = 0x24A, 1
CAR_HALF_WIDTH_M = 0.92
# every key the tick log may carry (anything else given is dropped: a position can never pass through)
TICK_KEYS = ("seq", "v", "set", "gap", "a", "dec", "st", "src", "ceil", "kind", "need", "cap", "side", "ahead", "alert", "rel", "beh",
             "mem", "map", "fc", "s1", "s2", "mon", "car", "lay", "lacc", "des", "rate", "cmd", "ti", "ln", "drv", "tq", "cpu", "lacc_ms", "ti_ms",
             "fcx", "mdl", "step_us")
DRV_PRESSED, DRV_BRAKE, DRV_GAS, DRV_BUTTON, DRV_LBLINK, DRV_RBLINK = 1, 2, 4, 8, 16, 32


def lat_accel_from_can(can_msg):
  """Left-positive lateral acceleration (m/s^2) from the newest 0x075 frame on bus 0 (STEER_RELATED.STEER_TORQUE, the car's
  accelerometer: (ay_raw - 2000 + 14.31) / 93.857, as carstate.lat_accel), or None."""
  for f in reversed(can_msg):
    if f.address == 0x75 and f.src == 0 and len(f.dat) >= 4:
      raw = ((f.dat[2] & 0x0F) << 8) | f.dat[3]
      return (raw - 2000 + 14.31) / 93.857
  return None


def ti_from_can(can_msg):
  """(STATE, VIOL) of the interceptor from the newest TI_FEEDBACK frame (0x24A, body bus 1), or None."""
  for f in reversed(can_msg):
    if f.address == TI_FEEDBACK_ADDR and f.src == TI_FEEDBACK_BUS and len(f.dat) >= 5:
      return int(f.dat[3]), int(f.dat[4])
  return None


def mem_summary(planner, look_m=TICK_MEM_STEP_M * TICK_MEM_N):
  """The remembered road ahead while locked: peak |kap_med| (signed, x1e4), its distance (m) and pass count, and kap_med (x1e4) every
  TICK_MEM_STEP_M to look_m. None when not locked or there is no memory stage. Point indices and positions never leave here."""
  mem = getattr(planner, "mem", None)
  if mem is None or not getattr(planner, "locked", False) or not getattr(planner, "mem_stage", False) or planner.trace < 0:
    return None
  i0 = int(math.floor(planner.pos))
  idx, dist = mem.ahead(planner.trace, i0, look_m)
  if not len(idx):
    return None
  dist = dist - (planner.pos - i0) * mem.spacing
  k = planner.kmed[idx]
  j = int(abs(k).argmax())
  grid = [TICK_MEM_STEP_M * (n + 1) for n in range(TICK_MEM_N)]
  import numpy as np
  s = np.interp(grid, dist, k, right=float("nan"))
  return {"pk": int(round(float(k[j]) * 1e4)), "pkd": int(round(float(dist[j]))), "pkn": int(mem.npass[idx[j]]),
          "s": [None if not math.isfinite(x) else int(round(float(x) * 1e4)) for x in s]}


def fc_summary(planner):
  """The calibrated signed (left +) forecast demand (m/s^2) at TICK_FC_LEADS from this tick's preview, or None."""
  fc, v = getattr(planner, "fc", None), getattr(planner, "fc_v", None)
  if fc is None or v is None:
    return None
  import numpy as np
  from openpilot.selfdrive.car.mazda import ti_curve_demand as CD
  ts = np.asarray(fc[0], np.float64)
  la = CD.calibrate(ts, max(float(v), 1.0) ** 2 * np.asarray(fc[1], np.float64))
  return [round(float(x), 2) for x in np.interp(TICK_FC_LEADS, ts, la, right=float("nan"))]


def stage_summary(planner):
  """Stage 1 (bends in progress: count, lowest target, its phase, mode, release decided), stage 2 (T, need, lead s) and the
  limiting bend's side (+1 left, -1 right, 0 unknown)."""
  act = dict(getattr(planner, "active", {}) or {})
  act.update(getattr(planner, "map_active", {}) or {})
  s1, side = None, 0
  if act:
    a_ = min(act.values(), key=lambda a: a["T"])
    s1 = [len(act), int(a_["T"]), int(a_.get("phase", 1)), str(a_.get("mode", "")), int(bool(a_.get("decided")))]
    side = int(a_.get("side", 0) or 0)
  s2 = getattr(planner, "s2", None)
  s2o = None
  if s2 is not None:
    s2o = [int(s2["T"]), int(s2["need"]), round(float(s2.get("lead") or 0.0), 1)]
    if s1 is None or s2["T"] < s1[1]:
      side = int(s2.get("side", 0) or 0)
  return s1, s2o, side


def lane_summary(model):
  """[clearance left, probability left, clearance right, probability right]: the car's side to the near lane line (m, negative =
  over it; CAR_HALF_WIDTH_M from the centre) at the car, or None."""
  try:
    ll, pr = model.laneLines, model.laneLineProbs
    if len(ll) < 3 or len(pr) < 3 or not len(ll[1].y) or not len(ll[2].y):
      return None
    return [round(-float(ll[1].y[0]) - CAR_HALF_WIDTH_M, 2), round(float(pr[1]), 2),
            round(float(ll[2].y[0]) - CAR_HALF_WIDTH_M, 2), round(float(pr[2]), 2)]
  except Exception:
    return None


def _r(x, nd):
  try:
    if x is None or isinstance(x, bool):
      return x
    x = float(x)
    return round(x, nd) if math.isfinite(x) else None
  except (TypeError, ValueError):
    return None


def _clip_str(o):
  if isinstance(o, str):
    return o[:TICK_STR_MAX]
  if isinstance(o, list):
    return [_clip_str(x) for x in o]
  if isinstance(o, dict):
    return {k: _clip_str(v) for k, v in o.items()}
  return o


def tick_log(plan, extras):
  """One tick's record (plan T8): the plan's decision and the inputs later analysis needs. extras: the plannerd-side figures (speed,
  set speed, gap, deceleration, memory / map / forecast summaries, stage 1 / 2, stage 3's alert, car side, lateral accelerations,
  lane clearance, driver flags, interceptor state, CPU). Only TICK_KEYS pass; bounded to TICK_MAX_B (TICK_MAP_MAX_B with a map list):
  over it, the memory samples, then the forecast, then the map are dropped."""
  e = dict(extras or {})
  d = {k: e[k] for k in TICK_KEYS if k in e}
  d.update(st=plan.stage, src=plan.src, ceil=plan.ceil, kind=plan.kind, need=plan.need_mph, cap=_r(plan.cap, 2),
           ahead=_r(plan.ahead_la, 2), alert=plan.alert_mph, beh=int(bool(plan.behind)),
           rel=(plan.diag or {}).get("rel", int(bool(plan.released))))
  if (plan.diag or {}).get("rm"):
    d["rm"] = plan.diag["rm"]       # beside the remembered road (road matching): why, "far" / "dev" / "lead" / "link"
  for k, nd in (("v", 1), ("gap", 1), ("a", 2), ("dec", 2), ("lay", 2), ("lacc", 2), ("des", 2), ("rate", 1)):
    if k in d:
      d[k] = _r(d[k], nd)
  if "map" in d:
    # a changed list (an empty one: the map has no opinion any more)
    d["map"] = [[int(round(float(a))), int(round(float(b)))] for a, b in list(d["map"] or [])[:TICK_MAP_N]]
  d = _clip_str(d)
  bound = TICK_MAP_MAX_B if "map" in d else TICK_MAX_B
  for drop in ("mem.s", "fc", "map", "mem", "ln", "car", "s1", "s2"):
    if len(json.dumps(d)) <= bound:
      break
    if drop == "mem.s":
      if isinstance(d.get("mem"), dict):
        d["mem"] = {k: v for k, v in d["mem"].items() if k != "s"}
    else:
      d.pop(drop, None)
    bound = TICK_MAP_MAX_B if "map" in d else TICK_MAX_B
  return d


def side_status(params_memory, now_wall, now_mono, tc):
  """For the tick log: (car side [state, target, taps down, taps up] or None, stage 3 [why, need] while it alerts or None), from
  TiCurveStatus (fresh within 3 s) and TiCurveDeliver (fresh within 1 s)."""
  car = mon = None
  try:
    st = json.loads(params_memory.get(tc.PARAM_STATUS) or b"null")
    if isinstance(st, dict) and now_wall - float(st.get("t", 0.0)) <= 3.0:
      car = [str(st.get("state", "")), st.get("target"), st.get("down"), st.get("up")]
  except Exception:
    car = None
  try:
    dv = json.loads(params_memory.get(tc.PARAM_DELIVER) or b"null")
    if isinstance(dv, dict) and dv.get("need") is not None and now_mono - float(dv.get("m", -1e9)) <= 1.0:
      mon = [str(dv.get("why", "")), dv.get("need")]
  except Exception:
    mon = None
  return car, mon


def keep_off_control_core():
  """Cores 0-3 only (never the control core 4, nor 5-7 where plannerd / radard / modeld run real-time), a little below normal
  priority: a slow tick of this process can never delay a process the communication checks watch (it publishes no message)."""
  try:
    cores = [c for c in os.sched_getaffinity(0) if c in (0, 1, 2, 3)]
    if cores:
      os.sched_setaffinity(0, cores)
  except Exception:
    pass
  try:
    os.nice(5)
  except Exception:
    pass


def main():
  import cereal.messaging as messaging
  from cereal import log
  LLK_VALID = log.LiveLocationKalman.Status.valid
  from openpilot.common.params import Params
  from openpilot.common.realtime import Ratekeeper
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.car.mazda import ti_curve as tc, ti_curve_memory as M, ti_curve_plan as P

  keep_off_control_core()
  params_memory = Params("/dev/shm/params")
  mem, why_none = None, ""
  t0 = time.monotonic()
  try:
    if os.path.exists(MEMORY_PATH):
      mem = M.load(MEMORY_PATH)
    else:
      why_none = "no road memory on the device"
  except Exception as e:
    why_none = f"road memory unreadable: {type(e).__name__}"
    cloudlog.exception("ti_curve_plan memory")
  cloudlog.event("ti_curve_plan", kind="memory", loaded=mem is not None, why=why_none, seconds=round(time.monotonic() - t0, 2),
                 traces=getattr(mem, "n_traces", 0), points=getattr(mem, "n", 0))
  planner = P.Planner(mem)
  mdl = model_name(Params())          # the driving model (logs are split by it); read once
  cloudlog.event("ti_curve_plan", kind="model", mdl=mdl)
  sm = messaging.SubMaster(["liveLocationKalman", "carState", "modelV2", "frogpilotPlan", "can", "carOutput"])
  rk = Ratekeeper(RATE_HZ, print_delay_threshold=None)
  seq = 0
  last_key, last_summary = None, 0.0
  yaw, yaw_t = None, -1e9
  lacc, lacc_t, ti_fb, ti_t = None, -1e9, None, -1e9
  vhist = []                      # (t, m/s) over the last ~1.2 s: the measured deceleration (A10)
  last_map, cpu_us = None, None
  lim_state = {"t": None}
  cpu = []
  while True:
    sm.update(0)
    t = time.monotonic()
    c0 = time.process_time()
    try:
      if sm.updated["can"]:
        y = yaw_from_can(sm["can"])
        if y is not None:
          yaw, yaw_t = y, t
        la_ = lat_accel_from_can(sm["can"])
        if la_ is not None:
          lacc, lacc_t = la_, t
        tf = ti_from_can(sm["can"])
        if tf is not None:
          ti_fb, ti_t = tf, t
      llk = sm["liveLocationKalman"]
      ok = bool(sm.alive["liveLocationKalman"] and llk.gpsOK and llk.positionGeodetic.valid and llk.status == LLK_VALID)
      fix = P.Fix(False)
      if ok and mem is not None:
        x, y_ = mem.to_local(llk.positionGeodetic.value[0], llk.positionGeodetic.value[1])
        fix = P.Fix(True, float(x), float(y_), P.bearing_to_hdg(math.degrees(llk.calibratedOrientationNED.value[2])))
      cs = sm["carState"]
      set_mph = cs.cruiseState.speed / MPH if cs.cruiseState.enabled and cs.cruiseState.speed > 0.5 else None
      yaw_k = None
      v = float(cs.vEgo)
      if v > 3.0:
        if yaw is not None and t - yaw_t < 0.5:
          yaw_k = yaw / v
        else:
          try:
            yaw_k = -float(llk.angularVelocityCalibrated.value[2]) / v
          except Exception:
            yaw_k = None
      limit = None
      try:
        limit = fresh_limit(sm.alive["frogpilotPlan"], float(sm["frogpilotPlan"].slcMapSpeedLimit), v, ok, t, lim_state)
      except Exception:
        limit = None
      ref_mph, floor_mph, acting = car_side(params_memory, time.time(), tc)
      map_k = None
      if ok:
        map_k = map_input(params_memory, ok, llk.positionGeodetic.value[0], llk.positionGeodetic.value[1],
                          math.degrees(llk.calibratedOrientationNED.value[2]))     # plan T6: stage 1 where no memory is known
      mk_, fcx = fresh_model(sm["modelV2"] if sm.alive["modelV2"] else None, sm.logMonoTime["modelV2"], t)
      s0 = time.perf_counter()
      plan = planner.step(t, fix, v, set_mph=set_mph, yaw_k=yaw_k, model_k=mk_, limit_mph=limit,
                          ref_mph=ref_mph, floor_mph=floor_mph, acting=acting, map_k=map_k,
                          blinker=bool(cs.leftBlinker or cs.rightBlinker))     # stage 2 is off with a blinker (plan T5)
      step_us = int(round(1e6 * (time.perf_counter() - s0)))
      on = params_memory.get(tc.PARAM_ON) in (b"1", "1")
      d = P.plan_json(plan, t, seq, "act" if on else "shadow")
      params_memory.put(tc.PARAM_PLAN, json.dumps(d))
      seq += 1
      for ev in planner.events:
        cloudlog.event("ti_curve_plan", **log_payload(ev, feature=d["mode"]))
      planner.events.clear()
      key = (d["ceil"], d["kind"], d["trusted"], d["why"], d["capped"])
      if key != last_key:
        cloudlog.event("ti_curve_plan", **log_payload(d, kind="decision", feature=d["mode"], mph=round(v / MPH, 1), set_mph=set_mph,
                                                       trace=plan.trace, point=plan.point))
        last_key = key
      # plan T8: one tick record, 5 Hz (positions never: the helpers return distances, curvatures, speeds and flags only)
      vhist.append((t, v))
      while len(vhist) > 2 and t - vhist[1][0] >= 1.0:
        vhist.pop(0)
      dec = (vhist[0][1] - v) / (t - vhist[0][0]) if t - vhist[0][0] >= 0.8 else None
      mk = None
      if map_k is not None:
        dd, kk = map_k
        sel = [j for j in range(len(dd)) if dd[j] >= 0.0]
        if len(sel) > TICK_MAP_N:
          sel = [sel[int(round(q * (len(sel) - 1) / (TICK_MAP_N - 1)))] for q in range(TICK_MAP_N)]
        mk = [[int(round(dd[j])), int(round(kk[j] * 1e4))] for j in sel]
      s1, s2, side = stage_summary(planner)
      car_st, mon_st = side_status(params_memory, time.time(), t, tc)
      co = sm["carOutput"]
      diag_ = co.mazdaDiagnostics if sm.alive["carOutput"] else None
      extras = dict(seq=seq, v=v / MPH, set=None if set_mph is None else int(round(set_mph)),
                    gap=None if set_mph is None else v / MPH - set_mph, a=float(cs.aEgo), dec=dec,
                    mem=mem_summary(planner), fc=fc_summary(planner), s1=s1, s2=s2, side=side, mon=mon_st, car=car_st,
                    lay=None if yaw_k is None else yaw_k * v * v, lacc=lacc if t - lacc_t < 0.5 else None,
                    des=(-v * v * float(co.actuatorsOutput.curvature)) if sm.alive["carOutput"] else None,
                    rate=float(cs.steeringRateDeg),
                    cmd=int(diag_.tiLimited) if diag_ is not None and diag_.version else None,
                    ti=list(ti_fb) if ti_fb is not None and t - ti_t < 0.5 else None, ln=lane_summary(sm["modelV2"]),
                    drv=(DRV_PRESSED * bool(cs.steeringPressed) | DRV_BRAKE * bool(cs.brakePressed) | DRV_GAS * bool(cs.gasPressed) |
                         DRV_BUTTON * bool(len(cs.buttonEvents)) | DRV_LBLINK * bool(cs.leftBlinker) | DRV_RBLINK * bool(cs.rightBlinker)),
                    tq=int(round(float(cs.steeringTorque))), cpu=cpu_us, fcx=fcx or None, mdl=mdl, step_us=step_us,
                    # review (minor): the CAN frame lacc / ti were last read from may be up to 0.5 s old (only the latest can message
                    # of each update is read): their age in ms, so a tick never reports a stale value as current
                    lacc_ms=int(round((t - lacc_t) * 1000)) if t - lacc_t < 0.5 else None,
                    ti_ms=int(round((t - ti_t) * 1000)) if ti_fb is not None and t - ti_t < 0.5 else None)
      if mk != last_map:
        extras["map"] = mk or []
        last_map = mk
      cloudlog.event("ti_curve_plan", **log_payload(tick_log(plan, extras), kind="tick"))
      cpu_us = int(round(1e6 * (time.process_time() - c0)))     # logged on the next tick (this tick's own cost, log included)
      cpu.append(time.process_time() - c0)
      if t - last_summary >= SUMMARY_S:
        cpu.sort()
        cloudlog.event("ti_curve_plan", kind="summary", feature=d["mode"], trusted=d["trusted"], known=d["known"], why=d["why"],
                       cpu_ms_mean=round(1000 * sum(cpu) / max(1, len(cpu)), 3), cpu_ms_max=round(1000 * cpu[-1], 3) if cpu else None,
                       yaw_source="car" if (yaw is not None and t - yaw_t < 0.5) else "locationd")
        cpu, last_summary = [], t
    except Exception:
      cloudlog.exception("ti_curve_plan step")       # the car controller treats a missing / old output as planner stopped
    rk.keep_time()


if __name__ == "__main__":
  main()
