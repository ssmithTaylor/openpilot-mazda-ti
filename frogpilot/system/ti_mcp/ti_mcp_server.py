#!/usr/bin/env python3
"""Read-only MCP server exposing Torque Interceptor tuning telemetry.

Lets a tuning assistant query what the TI and the lateral controller are doing without pulling
and parsing an rlog first. Everything here is read-only by design: no tool writes a param, and
none can be added without changing this file. Changing a steering torque limit should require
being in the car, looking at the settings screen.

Transport is JSON-RPC 2.0 over HTTP POST, implemented on the standard library so nothing new has
to be installed on the device. Point an MCP client at http://<device>:8756/mcp.

  python -m frogpilot.system.ti_mcp.ti_mcp_server

Started automatically by the manager when TiMcpEnabled is set, on and offroad.

Environment:
  TI_MCP_HOST  bind address, default 0.0.0.0 (all interfaces)
  TI_MCP_PORT  bind port, default 8756

There is no authentication. Anyone who can reach the port can read driving state -- speed,
steering, engagement. That is an accepted trade for a read-only service on a trusted home network;
on an untrusted or public network, turn the toggle off or set TI_MCP_HOST=127.0.0.1 and tunnel.
"""
import collections
import json
import os
import socket
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.common.params import Params

PROTOCOL_VERSION = "2024-11-05"
SERVER_INFO = {"name": "ti-tuning", "version": "1.0.0"}

TI_MODE = {0: "DISCOVER", 1: "OFF", 2: "DRIVER_OVER", 3: "RUN"}
TI_VIOL = {
  0x11: "LKAS_STUK (class 2) LKAS request stuck",
  0x12: "CAN_TMO (class 3) CAN offline >100ms",
  0x13: "LKAS_ERR (class 2) LKAS request out of window",
  0x14: "LOW_VOLT (class 3) CPU voltage low",
  0x15: "TMR_ERR (class 1) periodic timer IRQ missing",
  0x16: "MAIN_ERR (class 1) main loop timing",
  0x17: "TORQUE_H (class 2) computed torque over high limit",
  0x18: "TORQUE_L (class 2) computed torque under low limit",
  0x19: "WDT_ERR (class 1) internal watchdog",
  0x21: "CPU_REG (class 1) pin/peripheral config",
  0x22: "HIGH_TMP (class 3) die over 65C",
  0x23: "DATA_MEM (class 1) EEPROM CRC",
  0x25: "TK_TRNSF (class 2) calculated vs measured torque",
  0x26: "ADC_STUK (class 1) ADC channel failed",
  0x31: "HIN_ERR (class 2) EPS hall sensor input out of window",
  0x32: "TK_NLOAD (class 2) feedback torque, no load",
  0x33: "TK_WLOAD (class 2) feedback torque, with load",
  0x34: "CAN_ERR (class 3) CAN bus error",
  0x35: "STCK_ERR (class 1) stack over/underflow",
  0x36: "PANIC_MON (class 1) panic monitor high",
  0x37: "PANIC (class 1) panic test injection",
  0x41: "LOGIC (class 1) illegal state machine value",
  0x42: "DBG_STOP (class 1) debug console disabled ESC",
  0x43: "CLK_STOP (class 1) main oscillator stopped",
}

TI_STEER = 0x249     # openpilot -> TI, carries LKAS_REQUEST plus the discovery key
TI_FEEDBACK = 0x24A  # TI -> openpilot, byte 3 mode, byte 4 violation, byte 6 ramp-down

TUNING_PARAMS = ("TiSteerMax", "TiSteerDeltaUp", "TiSteerDeltaDown",
                 "TiSteerDriverAllowance", "TiSteerDriverMultiplier", "TiSteerThreshold")

# Baselines measured from route 8590bb6980c396f4_00000342, segments 42/55/56, so a caller can tell
# whether a number is normal without having to remember what normal looked like.
BASELINE = {
  "short_pct": "44-51% before tuning",
  "peak_bias": "36 observed, board limit 119",
  "curvature_err_mean": "0.00014 clean, 0.00054 while saturating",
}


class Snapshot:
  """Live values, refreshed on a background thread so HTTP handlers never block on messaging."""

  def __init__(self):
    self.lock = threading.Lock()
    self.data = {"available": False, "reason": "not started"}

  def get(self):
    with self.lock:
      return dict(self.data)

  def set(self, data):
    with self.lock:
      self.data = data

  def run(self):
    try:
      import cereal.messaging as messaging
    except Exception as e:
      self.set({"available": False, "reason": f"cereal unavailable: {e}"})
      return

    sm = messaging.SubMaster(["carState", "carControl", "controlsState",
                              "liveTorqueParameters", "liveParameters"])
    while True:
      try:
        sm.update(1000)
        cs, cc = sm["carState"], sm["carControl"]
        ltp, lp, ctl = sm["liveTorqueParameters"], sm["liveParameters"], sm["controlsState"]
        self.set({
          "available": True,
          "engaged": bool(cc.latActive),
          "v_ego_ms": round(float(cs.vEgo), 2),
          "steering_angle_deg": round(float(cs.steeringAngleDeg), 2),
          "driver_torque": float(cs.steeringTorque),
          "steer_fault_temporary": bool(cs.steerFaultTemporary),
          "commanded_steer": round(float(cc.actuators.steer), 4),
          "curvature": round(float(ctl.curvature), 6),
          "desired_curvature": round(float(getattr(ctl, "desiredCurvature", 0.0)), 6),
          "curvature_error": round(float(getattr(ctl, "desiredCurvature", 0.0)) - float(ctl.curvature), 6),
          "learned_lat_accel_factor": round(float(ltp.latAccelFactorFiltered), 4),
          "learned_friction": round(float(ltp.frictionCoefficientFiltered), 4),
          "learned_valid": bool(ltp.liveValid),
          "learned_in_use": bool(ltp.useParams),
          "learned_bucket_points": float(ltp.totalBucketPoints),
          "learned_decay": round(float(ltp.decay), 1),
          "learned_steer_ratio": round(float(lp.steerRatio), 3),
        })
      except Exception as e:
        self.set({"available": False, "reason": str(e)})


snapshot = Snapshot()
params = Params()
params_memory = Params("/dev/shm/params")


def _param_int(name):
  try:
    raw = params.get(name)
    return int(float(raw)) if raw not in (None, b"", "") else None
  except Exception:
    return None


def _param_json(name, live_first=False):
  """live_first reads /dev/shm before /data. The car controller keeps the current counters on
  tmpfs and only persists them once a minute, so the flash copy lags a live run by up to that --
  but it is the one that survives an ignition cycle, so it stays as the fallback."""
  for store in ((params_memory, params) if live_first else (params,)):
    try:
      raw = store.get(name)
      if raw:
        return json.loads(raw)
    except Exception:
      continue
  return None


def tool_ti_status(_args):
  """Live TI health. This is the first thing to check: everything else is meaningless if the
  interceptor is not in RUN, because it is then bypassed and the stock EPS is steering."""
  live = snapshot.get()
  stats = _param_json("TiTuningStats") or {}
  ti = stats.get("live") or {}
  mode, viol = ti.get("mode"), int(ti.get("viol") or stats.get("viol") or 0)
  out = {
    "live_available": live.get("available", False),
    # Current TI state, forwarded by the car controller -- it is not published in cereal.
    "ti_mode": TI_MODE.get(mode, mode) if mode is not None else "unknown",
    "ti_ramping_down": ti.get("ramp"),
    "ti_version": ti.get("version"),
    "violation_code": f"0x{viol:02X}" if viol else None,
    "violation_meaning": TI_VIOL.get(viol) if viol else None,
    "engaged": live.get("engaged"),
    "speed_ms": live.get("v_ego_ms"),
    "driver_torque": live.get("driver_torque"),
    "steer_fault_temporary": live.get("steer_fault_temporary"),
    "frames_not_in_run": stats.get("not_run"),
    "frames_ramping_down": stats.get("ramp"),
  }
  if mode is not None and mode != 3:
    out["warning_mode"] = ("TI is not in RUN right now -- it is bypassed and the stock EPS is "
                           "steering. Nothing tuned here takes effect until this reads RUN.")
  if not live.get("available"):
    out["reason"] = live.get("reason")
  if stats.get("not_run"):
    out["warning"] = ("TI left RUN during this measurement. While bypassed it injects nothing and "
                      "the stock EPS is steering, so any tuning conclusion from this run is void.")
  if stats.get("ramp"):
    out["warning_ramp"] = ("TI ramped its bias down, meaning it judged the commanded rate unsafe. "
                           "Lower TiSteerDeltaUp.")
  return out


def tool_ti_stats(_args):
  """Counters for the current measurement and the one before it, for A/B comparison."""
  current = _param_json("TiTuningStats", live_first=True) or {}
  previous = _param_json("TiTuningStatsPrevious") or {}

  def summarise(s):
    engaged = s.get("engaged", 0)
    if not engaged:
      return {"engaged_frames": 0, "note": "no engaged frames recorded"}
    return {
      "engaged_frames": engaged,
      "short_of_requested_pct": round(100.0 * s.get("short", 0) / engaged, 1),
      "rate_limited_pct": round(100.0 * s.get("rate_limited", 0) / engaged, 1),
      "driver_torque_limited_pct": round(100.0 * s.get("driver_limited", 0) / engaged, 1),
      "at_600_clip_pct": round(100.0 * s.get("at_clip", 0) / engaged, 1),
      "peak_command": s.get("peak_cmd"),
      "peak_bias_reaching_eps": s.get("peak_bias"),
      "frames_not_in_run": s.get("not_run"),
      "frames_ramping": s.get("ramp"),
    }

  return {
    "current": summarise(current),
    "previous": summarise(previous),
    "baselines": BASELINE,
    "how_to_read": ("short_of_requested_pct is how often openpilot cut its own command. The split "
                    "between rate_limited_pct and driver_torque_limited_pct says which knob to "
                    "turn: the former points at TiSteerDeltaUp, the latter at "
                    "TiSteerDriverMultiplier and TiSteerDriverAllowance."),
  }


def tool_ti_tuning(_args):
  """Current values of the six TI limits."""
  values = {name: _param_int(name) for name in TUNING_PARAMS}
  return {
    "enabled": bool(_param_int("TorqueInterceptorTune")),
    "values": values,
    "defaults": {"TiSteerMax": 600, "TiSteerDeltaUp": 6, "TiSteerDeltaDown": 15,
                 "TiSteerDriverAllowance": 15, "TiSteerDriverMultiplier": 40,
                 "TiSteerThreshold": 6},
    "notes": {
      "TiSteerMax": "TI clips its own input at 600; higher values are discarded by the unit.",
      "TiSteerDeltaUp": "Per 10ms frame. 6 means a full second from zero to maximum.",
      "TiSteerDriverMultiplier": "Command cap is 600 + (allowance + driver_torque) * this. At 40 "
                                 "the cap reaches zero about 30 counts past the allowance.",
    },
  }


def tool_torque_learning(_args):
  """State of openpilot's own lateral learning, which is separate from the TI limits."""
  live = snapshot.get()
  if not live.get("available"):
    return {"available": False, "reason": live.get("reason")}
  # controlsd applies the learned values when `useParams or force_auto_tune`, so useParams alone
  # does not say whether they are in effect -- forcing it is the other half of the condition.
  native = bool(live.get("learned_in_use"))
  forced = bool(_param_int("ForceAutoTune"))
  advanced = bool(_param_int("AdvancedLateralTune"))
  out = {
    "lat_accel_factor": live.get("learned_lat_accel_factor"),
    "friction": live.get("learned_friction"),
    "valid": live.get("learned_valid"),
    "bucket_points": live.get("learned_bucket_points"),
    "decay": live.get("learned_decay"),
    "learned_steer_ratio": live.get("learned_steer_ratio"),
    "curvature_error": live.get("curvature_error"),
    "car_supports_autotune": native,
    "force_auto_tune_param": forced,
    "advanced_lateral_tune_param": advanced,
    "effectively_in_use": native or (forced and advanced),
  }
  if not (native or forced):
    out["note"] = ("Neither the car's own auto-tune nor Force Auto-Tune is on, so these values are "
                   "computed and cached but never applied.")
  elif forced and not advanced:
    out["note"] = ("ForceAutoTune is set but AdvancedLateralTune is not, and the toggle is gated on "
                   "it -- the learned values are NOT being applied.")
  elif forced:
    out["note"] = ("Applied via Force Auto-Tune. Reset learned values after changing anything that "
                   "alters the command-to-response relationship, or these describe the old setup.")
  return out


# log_root() takes HD and konik flags and returns a different directory for each, so all three
# exist on a device and only one holds the recordings. TI_MCP_SEGMENTS overrides.
SEGMENT_ROOTS = ([os.environ["TI_MCP_SEGMENTS"]] if os.environ.get("TI_MCP_SEGMENTS") else
                 ["/data/media/0/realdata_konik", "/data/media/0/realdata_HD",
                  "/data/media/0/realdata"])


def _segment_dirs():
  """Returns (segments, diagnostics). Diagnostics matter: a missing directory, a permission error
  and an empty one are three different problems that all otherwise look like 'no segments'."""
  found, diags = [], []
  for root in SEGMENT_ROOTS:
    if not os.path.isdir(root):
      diags.append(f"{root}: does not exist")
      continue
    try:
      names = [n for n in os.listdir(root) if "--" in n]
    except PermissionError:
      diags.append(f"{root}: permission denied (ti_mcp runs as the manager's user)")
      continue
    except Exception as e:
      diags.append(f"{root}: {e}")
      continue
    if not names:
      diags.append(f"{root}: exists but holds no segments")
    found += [(root, n) for n in names]
  # Sort by route then segment NUMBER. Lexical order puts "--10" between "--1" and "--2", which
  # silently mis-orders any drive longer than ten segments.
  def key(rn):
    parts = rn[1].rsplit("--", 1)
    try:
      return (parts[0], int(parts[1]))
    except (IndexError, ValueError):
      return (rn[1], -1)
  found.sort(key=key, reverse=True)
  return found, diags


def _rlog_path(segment):
  for root in SEGMENT_ROOTS:
    for name in ("rlog.bz2", "rlog", "rlog.zst"):
      p = os.path.join(root, segment, name)
      if os.path.exists(p):
        return p
  return None


def tool_list_segments(args):
  """Recorded segments on the device, newest first."""
  limit = int(args.get("limit", 20))
  segments, diags = _segment_dirs()
  out = []
  for root, name in segments[:limit]:
    p = _rlog_path(name)
    out.append({
      "segment": name,
      "root": root,
      "rlog": os.path.basename(p) if p else None,
      "size_mb": round(os.path.getsize(p) / 1e6, 1) if p else None,
    })
  result = {"roots_checked": SEGMENT_ROOTS, "count": len(out), "segments": out,
            "note": "Pass a segment name to analyze_segment. Raw logs are not served; the analysis "
                    "is done here because an rlog is tens of megabytes."}
  if not out:
    result["why_empty"] = diags or ["no diagnostics"]
  return result


def tool_analyze_segment(args):
  """Decode one segment's TI and lateral behaviour. This is the authority the live counters are
  only an approximation of -- it has the full time series, so it can show curvature tracking and
  exactly when the TI changed state."""
  segment = args.get("segment")
  if not segment:
    return {"error": "segment required; call list_segments first"}
  path = _rlog_path(segment)
  if path is None:
    return {"error": f"no rlog found for {segment}"}

  try:
    import bz2
    from cereal import log as capnp_log
  except Exception as e:
    return {"error": f"cannot load decoder: {e}"}

  raw = open(path, "rb").read()
  if path.endswith(".bz2"):
    raw = bz2.decompress(raw)

  modes, viols = collections.Counter(), collections.Counter()
  ramp = engaged = 0
  ti_req, curv_err, speeds = [], [], []
  transitions = []
  last_sig = None
  truncated = False

  # Shortfall and the limiter split are recomputed here rather than read from the live counters,
  # which retain only two runs and cannot be recovered for an old segment. Limits come from the
  # params as they are NOW -- if they were changed since the drive, the attribution is off.
  steer_max = _param_int("TiSteerMax") or 600
  delta_up = _param_int("TiSteerDeltaUp") or 6
  allowance = _param_int("TiSteerDriverAllowance") or 15
  cmd_frames = short = rate_lim = drv_lim = at_clip = 0
  peak_cmd = 0
  last_steer, last_lat, last_drv, last_sent = 0.0, False, 0.0, 0
  static_tune, learned_tune, native_autotune = None, None, None

  events = capnp_log.Event.read_multiple_bytes(raw)
  t0 = None
  while True:
    try:
      msg = next(events)
    except StopIteration:
      break
    except Exception:
      truncated = True
      break
    if t0 is None:
      t0 = msg.logMonoTime
    ts = (msg.logMonoTime - t0) / 1e9
    w = msg.which()
    if w == "carControl":
      last_lat = bool(msg.carControl.latActive)
      last_steer = float(msg.carControl.actuators.steer)
      if last_lat:
        engaged += 1
    elif w == "controlsState":
      c = msg.controlsState
      curv_err.append(abs(float(getattr(c, "desiredCurvature", 0.0)) - float(c.curvature)))
    elif w == "carState":
      speeds.append(float(msg.carState.vEgo))
      last_drv = float(msg.carState.steeringTorque)
    elif w == "carParams" and static_tune is None:
      try:
        lt = msg.carParams.lateralTuning
        if lt.which() == "torque":
          static_tune = {"latAccelFactor": round(float(lt.torque.latAccelFactor), 4),
                         "friction": round(float(lt.torque.friction), 4)}
      except Exception:
        pass
    elif w == "liveTorqueParameters":
      q = msg.liveTorqueParameters
      native_autotune = bool(q.useParams)
      learned_tune = {"latAccelFactor": round(float(q.latAccelFactorFiltered), 4),
                      "friction": round(float(q.frictionCoefficientFiltered), 4),
                      "valid": bool(q.liveValid), "bucket_points": float(q.totalBucketPoints)}
    elif w == "can":
      for f in msg.can:
        if f.src == 0 and f.address == TI_FEEDBACK:
          d = bytes(f.dat)
          modes[d[3]] += 1
          viols[d[4]] += 1
          if d[6]:
            ramp += 1
          sig = (d[3], d[4], bool(d[6]))
          if sig != last_sig:
            transitions.append({"t": round(ts, 2), "mode": TI_MODE.get(d[3], d[3]),
                                "viol": f"0x{d[4]:02X}" if d[4] else None, "ramp": bool(d[6])})
            last_sig = sig
    elif w == "sendcan":
      for f in msg.sendcan:
        if f.address == TI_STEER:
          d = bytes(f.dat)
          req = ((d[0] << 8 | d[1]) & 0x0FFF) - 2048
          ti_req.append(req)
          if last_lat:
            cmd_frames += 1
            peak_cmd = max(peak_cmd, abs(req))
            if abs(req) >= steer_max:
              at_clip += 1
            desired = int(round(last_steer * steer_max))
            if abs(desired) - abs(req) > 5:
              short += 1
              # Rate limiting only counts while the command is climbing; a command collapsing under
              # driver torque moves at DELTA_DOWN and would otherwise be blamed on the ramp rate.
              if abs(req) > abs(last_sent) and abs(req - last_sent) >= delta_up:
                rate_lim += 1
              # Only torque OPPOSING the command narrows the cap. openpilot's driver-torque limit
              # is signed: the bound in the driver's own direction widens instead.
              if abs(last_drv) > allowance and (last_drv * desired) < 0:
                drv_lim += 1
          last_sent = req

  def pct(vals, p):
    return round(sorted(vals)[min(len(vals) - 1, int(len(vals) * p))], 6) if vals else None

  result = {
    "segment": segment,
    "truncated": truncated,
    "engaged_frames": engaged,
    "ti_mode": {TI_MODE.get(k, k): v for k, v in modes.most_common()},
    "ti_violations": {("none" if k == 0 else f"0x{k:02X} {TI_VIOL.get(k, '?')}"): v
                      for k, v in viols.most_common()},
    "ti_ramp_frames": ramp,
    "lkas_request": {"min": min(ti_req), "max": max(ti_req),
                     "at_clip": sum(1 for r in ti_req if abs(r) >= 600)} if ti_req else None,
    "curvature_error": {"mean": round(sum(curv_err) / len(curv_err), 6),
                        "p95": pct(curv_err, 0.95), "n": len(curv_err)} if curv_err else None,
    "command": {
      "frames": cmd_frames,
      "short_of_requested_pct": round(100.0 * short / cmd_frames, 1),
      "rate_limited_pct": round(100.0 * rate_lim / cmd_frames, 1),
      "driver_torque_limited_pct": round(100.0 * drv_lim / cmd_frames, 1),
      "at_clip_pct": round(100.0 * at_clip / cmd_frames, 1),
      "peak_command": peak_cmd,
      "limits_used": {"TiSteerMax": steer_max, "TiSteerDeltaUp": delta_up,
                      "TiSteerDriverAllowance": allowance},
      "caveat": "Limits are the CURRENT param values; if they changed since this drive the "
                "attribution is wrong. driver_torque_limited counts only torque opposing the "
                "command, since same-direction torque widens the cap rather than narrowing it.",
    } if cmd_frames else None,
    "speed_ms": {"mean": round(sum(speeds) / len(speeds), 2), "max": round(max(speeds), 2)} if speeds else None,
    "state_changes": transitions[:40],
    "lateral_tuning": {
      "static_from_car_port": static_tune,
      "learned": learned_tune,
      "car_native_autotune": native_autotune,
      "caveat": "Whether the learned values were actually APPLIED cannot be determined from an "
                "rlog. controlsd applies them when `useParams or force_auto_tune`, and the "
                "force_auto_tune toggle is never logged -- only car_native_autotune is. Use the "
                "torque_learning tool for the current config. These fields still let you spot a "
                "configuration change between segments.",
    },
    "baselines": BASELINE,
  }
  # A segment can look healthy -- TI in RUN, no violations -- and still say nothing about tuning,
  # because the car never moved or openpilot never steered. Say so rather than leaving it to be
  # inferred from a wall of zeros.
  if 3 not in modes and modes:
    result["verdict"] = ("UNUSABLE: the TI never reached RUN, so it was bypassed and the stock EPS "
                         "was steering. No tuning conclusion from this segment is valid.")
  elif speeds and max(speeds) < 1.0:
    result["verdict"] = ("NOT A DRIVE: the car never moved. Useful only to confirm the TI comes up "
                         "healthy; the zero curvature error and zero request are expected here and "
                         "say nothing about tuning.")
  elif engaged == 0:
    result["verdict"] = ("NO ENGAGED DRIVING: the car moved but openpilot never steered, so the "
                         "limiters were never exercised. Curvature error reflects your driving, "
                         "not openpilot's.")
  else:
    result["verdict"] = f"USABLE: {engaged} engaged frames (~{engaged / 100.0:.0f}s of openpilot steering)."
  return result


# ---------------------------------------------------------------------------------------------
# Diagnostics.
#
# Added after a drive where openpilot could not hold engagement while the interceptor sat in RUN
# the whole time. The tuning tools could rule the TI out but had nothing to say about what was
# actually failing: "communication issue between processes" means one of controlsd's inputs went
# stale, and the alert never says which. These answer that, from the log of the drive that already
# happened rather than from a reproduction.

SWAGLOG_LEVEL_ERROR = 40

# Services whose staleness controlsd turns into that alert. Used to rank gaps by relevance, not to
# exclude anything -- a stall in a service controlsd ignores can still name the guilty process.
CONTROLSD_INPUTS = {
  "deviceState", "pandaStates", "peripheralState", "modelV2", "liveCalibration", "carOutput",
  "driverMonitoringState", "longitudinalPlan", "liveLocationKalman", "managerState",
  "liveParameters", "radarState", "liveTorqueParameters", "liveDelay", "frogpilotCarState",
  "frogpilotPlan",
}

# Services controlsd never looks at directly but whose health decides whether the ones above call
# themselves valid -- plannerd, for one, publishes longitudinalPlan with
# valid = all_checks(['carState', 'controlsState', 'modelV2']). A stall here surfaces as some
# other service going invalid, which is why these are reported unconditionally rather than only
# when they cross a threshold: their absence from the list is itself the finding.
UPSTREAM_OF_VALIDITY = {
  "carState", "controlsState", "modelV2", "driverStateV2", "cameraOdometry", "sendcan", "can",
  "roadCameraState", "driverCameraState", "wideRoadCameraState", "livePose",
  "accelerometer", "gyroscope", "magnetometer", "gpsLocationExternal",
}


def _swaglog_dir():
  try:
    from openpilot.system.hardware.hw import Paths
    return Paths.swaglog_root()
  except Exception:
    return "/data/log"


def _unsuffix(key):
  """SwagLogFileFormatter tags every scalar key with its type ('event' -> 'event$s') so keys of
  different types cannot collide. Strip it so lookups can use the real name. The same records
  arrive over cereal without the tags, so both forms have to work."""
  return key.rsplit("$", 1)[0] if "$" in key else key


def _normalise(obj):
  if isinstance(obj, dict):
    return {_unsuffix(k): _normalise(v) for k, v in obj.items()}
  if isinstance(obj, list):
    return [_normalise(v) for v in obj]
  return obj


def _parse_log_line(text):
  """Log records are JSON, sometimes behind a context prefix. Find the object rather than assuming
  how wide the prefix is."""
  start = text.find("{")
  if start < 0:
    return None
  try:
    return _normalise(json.loads(text[start:]))
  except Exception:
    return None


def _comm_issue_from(record):
  """Pull the service lists out of a commIssue record. controlsd logs this only when the set
  changes, so every one of these is a distinct failure, not a repeat."""
  msg = record.get("msg") if isinstance(record, dict) else None
  if not isinstance(msg, dict) or msg.get("event") != "commIssue":
    return None
  return {
    "time": record.get("created"),
    "not_alive": msg.get("not_alive", []),
    "not_freq_ok": msg.get("not_freq_ok", []),
    "invalid": msg.get("invalid", []),
  }


def tool_device_errors(args):
  """Error-level device log plus the live process table."""
  limit = int(args.get("limit", 25))
  needle = str(args.get("match", "")).lower()
  result = {"swaglog_dir": _swaglog_dir()}

  # One-shot subscription rather than adding managerState to the background snapshot. This is only
  # interesting when something is already wrong, and a permanent extra subscriber to a running
  # control system is exactly the kind of thing worth not adding.
  try:
    import cereal.messaging as messaging
    sm = messaging.SubMaster(["managerState"])
    for _ in range(40):
      sm.update(100)
      if sm.seen["managerState"]:
        break
    if sm.seen["managerState"]:
      procs = [{"name": p.name, "running": bool(p.running),
                "should_be_running": bool(p.shouldBeRunning), "exit_code": int(p.exitCode)}
               for p in sm["managerState"].processes]
      result["process_count"] = len(procs)
      result["processes_not_running"] = [p for p in procs
                                         if p["should_be_running"] and not p["running"]]
      result["processes_with_nonzero_exit"] = [p for p in procs if p["exit_code"] != 0]
    else:
      result["processes_note"] = "managerState never arrived; manager may not be running"
  except Exception as e:
    result["processes_note"] = f"could not read managerState: {e}"

  errors, comm = [], []
  try:
    d = _swaglog_dir()
    # Rotation is every 60 seconds or 256KB, so a few dozen files covers the recent past without
    # reading the whole backlog (the handler keeps up to 2500).
    paths = sorted((os.path.join(d, f) for f in os.listdir(d) if f.startswith("swaglog.")),
                   key=os.path.getmtime, reverse=True)[:int(args.get("files", 40))]
    result["swaglog_files_read"] = len(paths)
    for path in paths:
      try:
        with open(path, errors="replace") as fh:
          lines = fh.readlines()
      except Exception:
        continue
      for line in lines:
        rec = _parse_log_line(line)
        if rec is None:
          continue
        issue = _comm_issue_from(rec)
        if issue is not None:
          comm.append(issue)
          continue
        msg = rec.get("msg")
        text = json.dumps(msg) if isinstance(msg, (dict, list)) else str(msg)
        if rec.get("levelnum", 0) >= SWAGLOG_LEVEL_ERROR or (needle and needle in text.lower()):
          errors.append({"time": rec.get("created"), "level": rec.get("level"),
                         "where": rec.get("filename"), "msg": text[:600]})
  except Exception as e:
    result["swaglog_note"] = f"could not read swaglog: {e}"

  # Sort by time rather than by file: files come newest-first but lines within one run oldest-first.
  comm.sort(key=lambda r: r.get("time") or 0, reverse=True)
  errors.sort(key=lambda r: r.get("time") or 0, reverse=True)
  result["comm_issues"] = comm[:limit]
  result["comm_issue_count"] = len(comm)
  result["errors"] = errors[:limit]
  result["error_count"] = len(errors)

  crashes = []
  for cd in ("/data/crashes", "/data/community/crashes", os.path.join(_swaglog_dir(), "crashes")):
    if not os.path.isdir(cd):
      continue
    try:
      for name in os.listdir(cd):
        p = os.path.join(cd, name)
        crashes.append({"file": p, "mtime": round(os.path.getmtime(p), 1),
                        "size": os.path.getsize(p)})
    except Exception:
      continue
  crashes.sort(key=lambda c: c["mtime"], reverse=True)
  result["crash_files"] = crashes[:10]

  result["how_to_read"] = (
    "comm_issues is the decisive field: not_alive names a service that stopped publishing "
    "entirely, not_freq_ok one that fell behind its rate, invalid one publishing but flagged bad. "
    "Map the service to its process (carOutput/frogpilotCarState -> card, liveTorqueParameters -> "
    "torqued, modelV2 -> modeld) and that is the process to investigate. Empty comm_issues with a "
    "populated errors list usually means a process crashed instead of stalling.")
  return result


def tool_segment_diagnostics(args):
  """Why a recorded drive misbehaved: events raised, processes that died, and which services
  stopped publishing on time. Independent of the TI entirely."""
  segment = args.get("segment")
  if not segment:
    return {"error": "segment required; call list_segments first"}
  path = _rlog_path(segment)
  if path is None:
    return {"error": f"no rlog found for {segment}"}

  try:
    import bz2
    from cereal import log as capnp_log
  except Exception as e:
    return {"error": f"cannot load decoder: {e}"}

  # The dict is SERVICE_LIST here; SERVICES is the newer upstream name. Without it every gap falls
  # back to an absolute threshold, which hides exactly the failure worth finding: a 100Hz service
  # missing a handful of frames is a tenth of a second and would never trip a one-second bar.
  freqs = {}
  for attr in ("SERVICE_LIST", "SERVICES"):
    try:
      mod = __import__("cereal.services", fromlist=[attr])
      freqs = {k: float(v.frequency) for k, v in getattr(mod, attr).items()}
      break
    except Exception:
      continue

  raw = open(path, "rb").read()
  if path.endswith(".bz2"):
    raw = bz2.decompress(raw)

  limit = int(args.get("limit", 25))

  ev_frames, ev_first, ev_last = collections.Counter(), {}, {}
  comm, errors = [], []
  proc_trouble = {}
  last_seen, max_gap = {}, {}
  truncated = False

  events = capnp_log.Event.read_multiple_bytes(raw)
  t0 = ts = None
  while True:
    try:
      msg = next(events)
    except StopIteration:
      break
    except Exception:
      truncated = True
      break
    if t0 is None:
      t0 = msg.logMonoTime
    ts = (msg.logMonoTime - t0) / 1e9
    w = msg.which()

    # Publication gaps straight from the log, per service. This does not depend on what controlsd
    # concluded, so it still points at the right process even if the alert blamed the wrong thing.
    prev = last_seen.get(w)
    if prev is not None and msg.logMonoTime > prev:
      gap = (msg.logMonoTime - prev) / 1e9
      if gap > max_gap.get(w, (0.0, 0.0))[0]:
        max_gap[w] = (gap, ts)
    last_seen[w] = msg.logMonoTime

    if w == "onroadEvents":
      for e in msg.onroadEvents:
        name = str(e.name)
        ev_frames[name] += 1
        ev_first.setdefault(name, ts)
        ev_last[name] = ts
    elif w == "managerState":
      for p in msg.managerState.processes:
        if p.shouldBeRunning and not p.running:
          d = proc_trouble.setdefault(p.name, {"frames_down": 0, "exit_codes": [],
                                               "first_s": round(ts, 1), "last_s": round(ts, 1)})
          d["frames_down"] += 1
          d["last_s"] = round(ts, 1)
          if int(p.exitCode) not in d["exit_codes"]:
            d["exit_codes"].append(int(p.exitCode))
    elif w in ("errorLogMessage", "logMessage"):
      rec = _parse_log_line(getattr(msg, w))
      if rec is None:
        continue
      issue = _comm_issue_from(rec)
      if issue is not None:
        issue["t"] = round(ts, 2)
        comm.append(issue)
      elif w == "errorLogMessage":
        body = rec.get("msg")
        text = json.dumps(body) if isinstance(body, (dict, list)) else str(body)
        errors.append({"t": round(ts, 2), "where": rec.get("filename"), "msg": text[:600]})

  def gap_row(name, gap, at):
    hz = freqs.get(name, 0.0)
    return {"service": name, "max_gap_s": round(gap, 3), "at_s": round(at, 1),
            "expected_hz": hz or None, "periods_missed": round(gap * hz, 1) if hz else None,
            "checked_by_controlsd": name in CONTROLSD_INPUTS}

  gaps, upstream = [], []
  for name, (gap, at) in max_gap.items():
    row = gap_row(name, gap, at)
    missed = row["periods_missed"]
    # A service is only interesting if it missed several of its own periods. Rate-less services
    # keep an absolute threshold so they are not silently dropped.
    if (missed is not None and missed >= 5) or (missed is None and gap >= 1.0):
      gaps.append(row)
    if name in UPSTREAM_OF_VALIDITY:
      upstream.append(row)
  gaps.sort(key=lambda g: (g["checked_by_controlsd"], g["periods_missed"] or 0), reverse=True)
  upstream.sort(key=lambda g: g["periods_missed"] or 0, reverse=True)

  ordered = sorted(ev_frames.items(), key=lambda kv: kv[1], reverse=True)
  onroad = [{"event": n, "frames": c, "first_s": round(ev_first[n], 1),
             "last_s": round(ev_last[n], 1)} for n, c in ordered]

  # The three lists mean genuinely different things and must not be merged: a service that stopped
  # publishing points at its own process, one that published on time and flagged itself bad points
  # at whatever it depends on. Conflating them sends the reader to the wrong place.
  stale = sorted({s for c in comm for s in (c["not_alive"] + c["not_freq_ok"])})
  flagged = sorted({s for c in comm for s in c["invalid"]})
  if proc_trouble:
    verdict = ("PROCESS FAILURE: " + ", ".join(sorted(proc_trouble)) +
               " stopped running while manager expected them up. Exit codes are in "
               "process_failures; a crash here explains any stale service downstream.")
  elif stale:
    verdict = ("STOPPED PUBLISHING: " + ", ".join(stale) + ". These missed their timing outright, "
               "so look at the process that publishes them and at whatever is starving it.")
  elif flagged:
    verdict = ("FLAGGED INVALID: " + ", ".join(flagged) + ". Nothing stalled and nothing died -- "
               "these published on schedule and marked their own output bad, which nearly always "
               "means something they consume did. Validity propagates: plannerd marks "
               "longitudinalPlan invalid when carState, controlsState or modelV2 fail ITS checks. "
               "upstream_gaps measures those inputs directly and is where to look next.")
  elif any(e["event"] == "commIssue" for e in onroad):
    verdict = ("commIssue was raised but no detail line was recorded in this segment -- the "
               "detail is logged only when the failing set changes, so it is likely in the "
               "segment before this one. Check worst_service_gaps here, and the earlier segment.")
  elif onroad:
    verdict = "No comm issue in this segment. Events raised are listed in onroad_events."
  else:
    verdict = "Nothing notable: no events, no process failures, no significant publication gaps."

  return {
    "segment": segment,
    "truncated": truncated,
    "duration_s": round(ts, 1) if ts else 0,
    "verdict": verdict,
    "onroad_events": onroad[:limit],
    "comm_issue_detail": comm[:limit],
    "comm_issue_count": len(comm),
    "process_failures": proc_trouble,
    "worst_service_gaps": gaps[:15],
    "upstream_gaps": upstream,
    "error_log": errors[:limit],
    "error_log_count": len(errors),
    "how_to_read": (
      "Read verdict first, then comm_issue_detail. Its three lists are not interchangeable: "
      "not_alive means a service stopped publishing, not_freq_ok that it fell behind its rate, "
      "invalid that it published on time but flagged its own output bad -- the last one points at "
      "that service's INPUTS, not at itself. worst_service_gaps and upstream_gaps measure real "
      "publication gaps from the log, independent of what controlsd concluded; upstream_gaps is "
      "reported in full even when nothing crosses a threshold, because a clean result there is "
      "itself informative. periods_missed expresses a gap in units of that service's own "
      "interval, so a 1Hz and a 100Hz service can be compared. process_failures is decisive when "
      "non-empty."),
  }


TOOLS = [
  {"name": "ti_status",
   "description":
     "Live health of the Torque Interceptor. CALL THIS FIRST -- if ti_mode is not RUN the device "
     "is bypassed, the stock EPS is steering, and no other reading means anything for tuning. "
     "Also returns violation code and meaning, the ramp-down flag, engagement, speed and driver "
     "torque. Reads 'unknown' when the car is parked, because the car controller only runs "
     "onroad; that is expected, not a fault.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_ti_status},

  {"name": "ti_stats",
   "description":
     "Live tuning counters for the current measurement and the one before it, for A/B comparison. "
     "short_of_requested_pct is how often openpilot cut its own command; the rate_limited vs "
     "driver_torque_limited split is the diagnostic that says WHICH parameter to change (rate -> "
     "TiSteerDeltaUp, driver -> TiSteerDriverMultiplier/Allowance). Only two runs are retained -- "
     "a third clear destroys the first, so save this output after every run. For an older drive "
     "use analyze_segment instead, which recomputes the same figures from the log.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_ti_stats},

  {"name": "ti_tuning",
   "description":
     "Current values of the six Torque Interceptor command limits, with their defaults and notes "
     "on what each does. Read-only: changing them is done by hand on the device under Settings -> "
     "STEERING -> Torque Interceptor Tuning. Use this to confirm what a drive was actually run "
     "with before drawing any conclusion from it.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_ti_tuning},

  {"name": "torque_learning",
   "description":
     "openpilot's learned lateral parameters (latAccelFactor, friction), whether they have "
     "converged, and crucially whether they are actually being APPLIED. controlsd applies them "
     "when the car natively supports auto-tune OR Force Auto-Tune is set, and the force toggle is "
     "itself gated on Advanced Lateral Tune -- all three are reported separately plus a combined "
     "effectively_in_use. This is the only reliable way to establish that configuration; it is "
     "not recoverable from a log. Reads zero when parked.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_torque_learning},

  {"name": "list_segments",
   "description":
     "Recorded driving segments on the device, newest first, sorted by route then segment number. "
     "Each entry gives the segment name to pass to analyze_segment, plus rlog size. If none are "
     "found, a why_empty field names the cause per directory searched (missing, permission "
     "denied, or genuinely empty) rather than silently returning nothing.",
   "inputSchema": {"type": "object", "properties": {
     "limit": {"type": "integer", "description": "How many segments to list (default 20)"}}},
   "fn": tool_list_segments},

  {"name": "analyze_segment",
   "description":
     "Parse one segment's rlog ON THE DEVICE and return the full picture for that drive: TI "
     "mode/violation/ramp timeline with timestamps, LKAS request range and time at the 600 clip, "
     "command shortfall with the rate vs driver-torque limiter split, and curvature tracking "
     "error (mean and p95) -- the outcome metric that says whether the car actually tracked "
     "better, which the live counters cannot answer. Starts with a verdict field stating whether "
     "the segment can support a tuning conclusion at all; a parked or unengaged segment reads as "
     "healthy but proves nothing. Only the summary crosses the wire, never the log itself.",
   "inputSchema": {"type": "object", "properties": {
     "segment": {"type": "string",
                 "description": "Segment name exactly as returned by list_segments, "
                                "e.g. 00000056--9892b4291f--5"}},
     "required": ["segment"]},
   "fn": tool_analyze_segment},

  {"name": "device_errors",
   "description":
     "Why the device itself is unhappy, as opposed to the interceptor. Returns every commIssue "
     "record from the device log with the SERVICE NAMES that went stale (not_alive stopped "
     "publishing, not_freq_ok fell behind its rate, invalid published but flagged bad) -- the "
     "detail the on-screen 'communication issue between processes' alert never shows. Also "
     "returns the live process table with exit codes, error-level log entries, and any crash "
     "files. Use this whenever openpilot will not engage or drops out and the TI reads healthy.",
   "inputSchema": {"type": "object", "properties": {
     "limit": {"type": "integer", "description": "Max records of each kind (default 25)"},
     "files": {"type": "integer", "description": "How many rotated log files to read (default 40, "
                                                 "each covers 60s or 256KB)"},
     "match": {"type": "string", "description": "Also return non-error lines containing this text, "
                                                "e.g. a process name"}}},
   "fn": tool_device_errors},

  {"name": "segment_diagnostics",
   "description":
     "The retrospective counterpart to device_errors: reads a recorded segment and reports why "
     "that drive misbehaved, without needing to reproduce it. Gives the onroadEvents timeline "
     "(what openpilot raised and when), any process manager expected to be running that was not, "
     "with exit codes, the commIssue detail lines, and an independent per-service publication gap "
     "analysis measured straight from the log -- so it can identify a stall even when controlsd "
     "attributed it wrongly. Gaps are reported as periods_missed, the gap in units of that "
     "service's own interval, so a 1Hz and a 100Hz service can be compared. Nothing to do with "
     "the TI; use analyze_segment for that.",
   "inputSchema": {"type": "object", "properties": {
     "segment": {"type": "string",
                 "description": "Segment name exactly as returned by list_segments"},
     "limit": {"type": "integer", "description": "Max records of each kind (default 25)"}},
     "required": ["segment"]},
   "fn": tool_segment_diagnostics},
]


class Handler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  def log_message(self, fmt, *args):
    pass

  def _send(self, payload, status=200):
    body = json.dumps(payload).encode()
    self.send_response(status)
    self.send_header("Content-Type", "application/json")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def do_GET(self):
    # Convenience for eyeballing it from a browser; not part of the MCP surface.
    if self.path.rstrip("/") == "/health":
      self._send({"ok": True, "live": snapshot.get().get("available", False)})
    else:
      self._send({"error": "POST JSON-RPC to /mcp"}, 404)

  def do_POST(self):
    try:
      length = int(self.headers.get("Content-Length", 0))
      req = json.loads(self.rfile.read(length) or b"{}")
    except Exception as e:
      self._send({"jsonrpc": "2.0", "id": None,
                  "error": {"code": -32700, "message": f"parse error: {e}"}}, 400)
      return

    method, req_id = req.get("method"), req.get("id")

    # Notifications carry no id and expect no response body.
    if req_id is None and method and method.startswith("notifications/"):
      self.send_response(202)
      self.send_header("Content-Length", "0")
      self.end_headers()
      return

    if method == "initialize":
      client_version = (req.get("params") or {}).get("protocolVersion")
      self._send({"jsonrpc": "2.0", "id": req_id, "result": {
        "protocolVersion": client_version or PROTOCOL_VERSION,
        "capabilities": {"tools": {}},
        "serverInfo": SERVER_INFO,
      }})
    elif method == "tools/list":
      self._send({"jsonrpc": "2.0", "id": req_id, "result": {
        "tools": [{k: t[k] for k in ("name", "description", "inputSchema")} for t in TOOLS],
      }})
    elif method == "tools/call":
      name = (req.get("params") or {}).get("name")
      args = (req.get("params") or {}).get("arguments") or {}
      tool = next((t for t in TOOLS if t["name"] == name), None)
      if tool is None:
        self._send({"jsonrpc": "2.0", "id": req_id,
                    "error": {"code": -32602, "message": f"unknown tool: {name}"}})
        return
      try:
        result = tool["fn"](args)
        self._send({"jsonrpc": "2.0", "id": req_id, "result": {
          "content": [{"type": "text", "text": json.dumps(result, indent=2)}],
          "isError": False,
        }})
      except Exception as e:
        self._send({"jsonrpc": "2.0", "id": req_id, "result": {
          "content": [{"type": "text", "text": f"tool failed: {e}"}],
          "isError": True,
        }})
    else:
      self._send({"jsonrpc": "2.0", "id": req_id,
                  "error": {"code": -32601, "message": f"unknown method: {method}"}})


def lan_ip():
  """Best guess at the address this device is reachable on. Opening a UDP socket toward a public
  address sends nothing but makes the kernel pick the interface it would route out of, which is
  the one a laptop on the same network can reach."""
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      s.settimeout(0.2)
      s.connect(("8.8.8.8", 53))
      return s.getsockname()[0]
  except Exception:
    return None


def publish_address(bind_host, port):
  """Write the connect URL where the settings panel can display it. The device's address is not
  something the driver can be expected to know, and it changes between networks."""
  ip = lan_ip() if bind_host in ("0.0.0.0", "::") else bind_host
  reachable = bind_host not in ("127.0.0.1", "localhost")
  url = f"http://{ip or bind_host}:{port}/mcp"
  while True:
    # On tmpfs. This is a liveness heartbeat that is regenerated at every startup and never wanted
    # across a boot, so it has no business on flash: at 5s each write was a pair of fsyncs, and an
    # fsync is an ext4 journal commit the whole filesystem queues behind -- loggerd streaming
    # camera video included. /dev/shm keeps the cadence and costs nothing.
    params_memory.put_nonblocking("TiMcpAddress", json.dumps({
      "url": url,
      "bind": bind_host,
      "port": port,
      "reachable_remotely": reachable,
      "heartbeat": int(time.time()),
    }))
    time.sleep(5)


def main():
  host = os.getenv("TI_MCP_HOST", "0.0.0.0")
  port = int(os.getenv("TI_MCP_PORT", "8756"))
  threading.Thread(target=snapshot.run, daemon=True).start()
  threading.Thread(target=publish_address, args=(host, port), daemon=True).start()
  print(f"ti-tuning MCP (read-only) on http://{host}:{port}/mcp")
  ThreadingHTTPServer((host, port), Handler).serve_forever()


if __name__ == "__main__":
  main()
