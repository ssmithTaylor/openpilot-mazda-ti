#!/usr/bin/env python3
"""Read-only MCP server exposing Torque Interceptor tuning telemetry.

Lets a tuning assistant query what the TI and the lateral controller are doing without pulling
and parsing an rlog first. Everything here is read-only by design: no tool writes a param, and
none can be added without changing this file. Changing a steering torque limit should require
being in the car, looking at the settings screen.

Transport is JSON-RPC 2.0 over HTTP POST, implemented on the standard library so nothing new has
to be installed on the device. Point an MCP client at http://<device>:8756/mcp.

  python -m frogpilot.system.ti_mcp.ti_mcp_server

Environment:
  TI_MCP_HOST  bind address, default 127.0.0.1 (use 0.0.0.0 to reach it from another machine)
  TI_MCP_PORT  bind port, default 8756

Binding to 0.0.0.0 exposes the telemetry to anyone on the network. It is read-only, but it does
reveal driving state, so prefer an SSH tunnel or a private network over an open bind.
"""
import json
import os
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


def _param_int(name):
  try:
    raw = params.get(name)
    return int(float(raw)) if raw not in (None, b"", "") else None
  except Exception:
    return None


def _param_json(name):
  try:
    raw = params.get(name)
    return json.loads(raw) if raw else None
  except Exception:
    return None


def tool_ti_status(_args):
  """Live TI health. This is the first thing to check: everything else is meaningless if the
  interceptor is not in RUN, because it is then bypassed and the stock EPS is steering."""
  live = snapshot.get()
  stats = _param_json("TiTuningStats") or {}
  viol = int(stats.get("viol", 0))
  out = {
    "live_available": live.get("available", False),
    "engaged": live.get("engaged"),
    "speed_ms": live.get("v_ego_ms"),
    "driver_torque": live.get("driver_torque"),
    "steer_fault_temporary": live.get("steer_fault_temporary"),
    "frames_not_in_run": stats.get("not_run"),
    "frames_ramping_down": stats.get("ramp"),
    "last_violation_code": f"0x{viol:02X}" if viol else None,
    "last_violation_meaning": TI_VIOL.get(viol) if viol else None,
  }
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
  current = _param_json("TiTuningStats") or {}
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
  out = {
    "lat_accel_factor": live.get("learned_lat_accel_factor"),
    "friction": live.get("learned_friction"),
    "valid": live.get("learned_valid"),
    "in_use": live.get("learned_in_use"),
    "bucket_points": live.get("learned_bucket_points"),
    "decay": live.get("learned_decay"),
    "learned_steer_ratio": live.get("learned_steer_ratio"),
    "curvature_error": live.get("curvature_error"),
  }
  if not live.get("learned_in_use"):
    out["note"] = ("useParams is false, so these learned values are being computed and cached but "
                   "not applied. FrogPilot's Force Auto-Tune On is what enables them.")
  return out


TOOLS = [
  {"name": "ti_status", "description": "Live Torque Interceptor health: engaged state, mode, "
                                       "violations and ramp-downs. Check this before trusting any "
                                       "other reading.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_ti_status},
  {"name": "ti_stats", "description": "Tuning counters for the current and previous measurement, "
                                      "with the limiter breakdown that says which parameter to "
                                      "change.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_ti_stats},
  {"name": "ti_tuning", "description": "Current values of the six Torque Interceptor limits, with "
                                       "defaults and what each one does.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_ti_tuning},
  {"name": "torque_learning", "description": "openpilot's learned lateral parameters and live "
                                             "curvature tracking error.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_torque_learning},
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


def main():
  threading.Thread(target=snapshot.run, daemon=True).start()
  host = os.getenv("TI_MCP_HOST", "127.0.0.1")
  port = int(os.getenv("TI_MCP_PORT", "8756"))
  print(f"ti-tuning MCP (read-only) on http://{host}:{port}/mcp")
  ThreadingHTTPServer((host, port), Handler).serve_forever()


if __name__ == "__main__":
  main()
