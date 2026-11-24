#!/usr/bin/env python3
"""Curve slowing planner process (ti_curve_plannerd): runs ti_curve_plan.Planner at 5 Hz, OUTSIDE the two 100 Hz control processes, at
normal priority, kept off the control core (core 4: card and controlsd), and publishes its small output to the tmpfs param
ti_curve.PARAM_PLAN for the car controller (ti_curve.py). It always runs while onroad: with curve slowing OFF it is the SHADOW mode (it
plans and logs what it would have done; the car controller sends nothing).

Reads: liveLocationKalman (position, heading, validity), carState (speed, set speed), modelV2 (the driving model's predicted curvature
over the next seconds), frogpilotPlan.slcMapSpeedLimit (the road's speed limit from the offline maps; 0 = unknown), the car's yaw
sensor from CAN 0x075 (bytes 4-5, -0.00017243 rad/s per count, offset 3.1032, right positive; the newest frame in the latest CAN
batch, else liveLocationKalman's calibrated yaw rate). The road memory: MEMORY_PATH, copied to the device by hand (it is private: where
the owner has driven); without it the planner publishes "no road memory on the device" and nothing happens.

Logs (`ti_curve_plan` channel): every change of the decision (ceiling, kind, trusted, reason) and the planner's events (lock, lock lost,
road changed, corner start with the planned and actual speed and the grade, mode changes, corner passed), and a summary every 5 s.
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
  sm = messaging.SubMaster(["liveLocationKalman", "carState", "modelV2", "frogpilotPlan", "can"])
  rk = Ratekeeper(RATE_HZ, print_delay_threshold=None)
  seq = 0
  last_key, last_summary = None, 0.0
  yaw, yaw_t = None, -1e9
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
      plan = planner.step(t, fix, v, set_mph=set_mph, yaw_k=yaw_k, model_k=model_curvature(sm["modelV2"]), limit_mph=limit,
                          ref_mph=ref_mph, floor_mph=floor_mph, acting=acting)
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
