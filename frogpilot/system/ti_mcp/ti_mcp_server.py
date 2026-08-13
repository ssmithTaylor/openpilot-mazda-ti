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
import math
import os
import socket
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.common.params import Params

PROTOCOL_VERSION = "2024-11-05"
SERVER_INFO = {"name": "ti-tuning", "version": "1.0.0"}
MAX_REQUEST_BYTES = 64 * 1024

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
STEER_TORQUE = 0x240  # car -> openpilot, byte 0 is the EPS torque sensor

# Both torque sensors are declared identically in mazda_2017.dbc -- 8|0+ (1,-127), range [-85,85]
# -- so they are directly comparable and their difference is a real bias in sensor counts.
# STEER_TORQUE_SENSOR is what the EPS reads (driver plus whatever the TI is injecting);
# TI_TORQUE_SENSOR is the driver alone. That identity is the whole basis of this analysis.
TORQUE_SENSOR_OFFSET = 127
TORQUE_SENSOR_LIMIT = 85  # DBC range edge; readings pinned here are saturated, not measured

# Speeds at which the stock Mazda LKAS drops out and wakes up (values.py LKAS_LIMITS, kph).
# Above the enable speed the stock CAM_LKAS request is acted on as well as the TI's, so the plant
# sees two actuators; below the disable speed it sees only the TI. Comparing the fitted slope
# either side of that band is what tells us whether the dual path is real.
LKAS_DISABLE_MS = 45 / 3.6
LKAS_ENABLE_MS = 52 / 3.6

TUNING_PARAMS = ("TiSteerMax", "TiSteerDeltaUp", "TiSteerDeltaDown",
                 "TiSteerDriverAllowance", "TiSteerDriverMultiplier", "TiSteerThreshold",
                 "TiSteerDeltaUpKnee", "TiSteerDeltaUpHigh")

# Param name -> the CarControllerParams attribute the bounds are keyed by. TiSteerThreshold is not
# a ccp field (carstate applies it to steeringPressed) so it keeps its bound stated here.
PARAM_TO_LIMIT = {
  "TiSteerMax": "TI_STEER_MAX", "TiSteerDeltaUp": "TI_STEER_DELTA_UP",
  "TiSteerDeltaDown": "TI_STEER_DELTA_DOWN", "TiSteerDriverAllowance": "TI_STEER_DRIVER_ALLOWANCE",
  "TiSteerDriverMultiplier": "TI_STEER_DRIVER_MULTIPLIER",
  "TiSteerDeltaUpKnee": "TI_STEER_DELTA_UP_KNEE", "TiSteerDeltaUpHigh": "TI_STEER_DELTA_UP_HIGH",
}
THRESHOLD_BOUNDS = (1, 15)

# Taken from the car port rather than restated, so this cannot drift from what is actually
# enforced -- a copy that nothing executes against is the one that goes stale unnoticed. Guarded
# because the server should keep serving if the import breaks, and say so rather than quietly
# reporting numbers it made up.
try:
  from openpilot.selfdrive.car.mazda.values import TI_LIMIT_BOUNDS
  TI_LIMIT_SOURCE = "selfdrive/car/mazda/values.py"
except Exception as _e:  # pragma: no cover - only on a broken install
  TI_LIMIT_BOUNDS = {}
  TI_LIMIT_SOURCE = f"unavailable ({_e}) -- ranges below are unknown, not unrestricted"

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
                              "liveTorqueParameters", "liveParameters", "liveDelay"])
    while True:
      try:
        sm.update(1000)
        cs, cc = sm["carState"], sm["carControl"]
        ltp, lp, ctl = sm["liveTorqueParameters"], sm["liveParameters"], sm["controlsState"]
        self.set({
          "available": True,
          # Monotonic stamp so a consumer can tell a fresh reading from the last one taken before
          # this thread wedged. The park check refuses to act on a stale snapshot.
          "ts": time.monotonic(),
          "engaged": bool(cc.latActive),
          "v_ego_ms": round(float(cs.vEgo), 2),
          # Gear and standstill feed the park check in _refuse_if_driving. Mazda's carstate decodes
          # both, and openpilot raises wrongGear off the same signal, so this is the car's own
          # reading rather than something inferred from speed.
          "gear_shifter": str(cs.gearShifter),
          "standstill": bool(cs.standstill),
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
          # lagd's live estimate of steering actuator delay. The rate limiter looks exactly like
          # actuator lag from the controller's point of view, so a learned delay well above the
          # car port's static 0.1s is quantified evidence the ramp is starving the controller --
          # and it should re-converge downward if raising the ramp rate genuinely helps.
          "learned_lateral_delay": round(float(sm["liveDelay"].lateralDelay), 4),
          "learned_lateral_delay_estimate": round(float(sm["liveDelay"].lateralDelayEstimate), 4),
          "lateral_delay_status": str(sm["liveDelay"].status),
          "lateral_delay_valid_blocks": int(sm["liveDelay"].validBlocks),
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
    # config/route/started_at are reported even for an empty run, because "which limits was this
    # recorded under" is exactly what you need when a run turns out to be empty.
    ident = {"config": s.get("config"), "route": s.get("route") or None,
             "started_at": s.get("started_at")}
    if not engaged:
      return {"engaged_frames": 0, "note": "no engaged frames recorded", **ident}
    return {
      "engaged_frames": engaged,
      "short_of_requested_pct": round(100.0 * s.get("short", 0) / engaged, 1),
      # The magnitude, not just the frequency. A frame cut by 6 counts and one cut by 150 both
      # count as "short"; only the second misses an apex. This is the number to watch across an
      # A/B -- it should fall when a change actually helps, and it can fall even if the percentage
      # does not move.
      "mean_deficit_per_engaged_frame": round(s.get("deficit", 0) / engaged, 1),
      "rate_limited_pct": round(100.0 * s.get("rate_limited", 0) / engaged, 1),
      # Which side of the knee the rate limiting happened on. With the knee at its default of
      # TiSteerMax everything lands in _below_knee, which is correct: there is only one slope.
      "rate_limited_below_knee_pct": round(100.0 * s.get("rate_limited_low", 0) / engaged, 1),
      "rate_limited_above_knee_pct": round(100.0 * s.get("rate_limited_high", 0) / engaged, 1),
      "driver_torque_limited_pct": round(100.0 * s.get("driver_limited", 0) / engaged, 1),
      "at_clip_pct": round(100.0 * s.get("at_clip", 0) / engaged, 1),
      "peak_command": s.get("peak_cmd"),
      "peak_requested": s.get("peak_desired"),
      "peak_bias_reaching_eps": s.get("peak_bias"),
      # Bias counts delivered per count of command, accumulated live over frames with appreciable
      # command. This is the conversion the whole problem rests on: multiplied by TiSteerMax it is
      # the most bias this setup can ever produce. ti_response computes the same thing from a log
      # with linearity, lag and per-speed breakdown; this is the cheap version that needs no parse.
      "measured_bias_slope": (round(s["bias_sum"] / s["bias_cmd_sum"], 5)
                              if s.get("bias_cmd_sum") else None),
      "bias_sample_frames": s.get("bias_frames"),
      # The two tails kept apart. Ratios are bias per command in thousandths, so 61 is the 0.0607
      # working figure. A low min is assist fading, which is benign. A high max, or any wrong_sign
      # frames at all, is torque the command did not ask for -- the direction that would justify
      # ever letting the guard touch the command, and the only one worth acting on.
      "bias_ratio_min_per_1000": s.get("bias_ratio_min") or None,
      "bias_ratio_max_per_1000": s.get("bias_ratio_max") or None,
      "bias_wrong_sign_frames": s.get("bias_wrong_sign"),
      "frames_not_in_run": s.get("not_run"),
      "frames_ramping": s.get("ramp"),
      **ident,
    }

  cur, prev = summarise(current), summarise(previous)
  # Oldest first, so a session reads as a sequence of steps. Only completed runs land here -- the
  # one in progress is `current`.
  history = [summarise(h) for h in (_param_json("TiTuningStatsHistory") or [])]
  result = {
    "current": cur,
    "previous": prev,
    "history": history,
    "baselines": BASELINE,
    "how_to_read": ("short_of_requested_pct is how often openpilot cut its own command and "
                    "mean_deficit_per_engaged_frame is by how much -- the second is the one that "
                    "tracks whether the car actually got its command. The split between "
                    "rate_limited_pct and driver_torque_limited_pct says which knob to turn: the "
                    "former points at TiSteerDeltaUp, the latter at TiSteerDriverMultiplier and "
                    "TiSteerDriverAllowance. peak_requested vs peak_command shows how much of what "
                    "openpilot asked for ever survived the limiters."),
  }
  # Comparing two runs recorded under different limits is the easiest way to reach a wrong
  # conclusion, so say so rather than leaving it to be noticed.
  if cur.get("config") and prev.get("config"):
    changed = {k: [prev["config"].get(k), cur["config"].get(k)]
               for k in set(cur["config"]) | set(prev["config"])
               if prev["config"].get(k) != cur["config"].get(k)}
    result["config_changed_between_runs"] = changed or "identical -- this is a repeatability check, not an A/B"
  elif cur.get("engaged_frames") and prev.get("engaged_frames"):
    result["config_changed_between_runs"] = ("unknown: one of these runs predates config stamping, "
                                             "so the comparison cannot be trusted")
  return result


def tool_ti_flags(args):
  """Moments the driver marked from the tuning panel, newest first."""
  flags = _param_json("TiFlaggedMoments") or []
  limit = int(args.get("limit", 25))
  out = []
  for f in reversed(flags[-limit:]):
    entry = dict(f)
    seg = f.get("segment")
    entry["rlog_present"] = bool(seg) and _rlog_path(seg) is not None
    try:
      entry["at_local"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(float(f["at"])))
    except (KeyError, TypeError, ValueError):
      pass
    # The segment index is derived from elapsed time in the route, so a flag landing near a minute
    # boundary can name the neighbour. Offer the ones either side that actually exist rather than
    # letting a reader conclude the interesting frames are missing.
    if seg and "--" in seg:
      base, _, idx = seg.rpartition("--")
      try:
        n = int(idx)
        neighbours = [f"{base}--{n - 1}"] if n > 0 else []
        neighbours.append(f"{base}--{n + 1}")
        entry["also_check"] = [s for s in neighbours if _rlog_path(s)]
      except ValueError:
        pass
    out.append(entry)

  return {
    "count": len(flags),
    "returned": len(out),
    "flags": out,
    "how_to_read": (
      "Each flag is a moment the driver marked because something felt wrong. Pass its segment "
      "straight to ti_response, analyze_segment or segment_diagnostics -- and check also_check "
      "too, since the index is derived from elapsed time and a flag near a minute boundary can "
      "name the neighbouring segment. The recorded instant can trail the tap by up to a second, "
      "and the driver is reacting to something already a second or two old, so read the flag as "
      "marking a stretch of road rather than a frame. command, bias, driver_torque and ti_mode are "
      "what the interceptor was doing at the time; config is the limits in force, which is what "
      "makes a flag from an earlier tuning state still interpretable. at_local uses the device "
      "clock, which is wrong until it syncs after boot -- trust the route and segment over it."),
  }


def tool_ti_tuning(_args):
  """Current values of the six TI limits."""
  values = {name: _param_int(name) for name in TUNING_PARAMS}
  return {
    "enabled": bool(_param_int("TorqueInterceptorTune")),
    "values": values,
    "defaults": {"TiSteerMax": 600, "TiSteerDeltaUp": 6, "TiSteerDeltaDown": 15,
                 "TiSteerDriverAllowance": 15, "TiSteerDriverMultiplier": 40,
                 "TiSteerThreshold": 6, "TiSteerDeltaUpKnee": 600, "TiSteerDeltaUpHigh": 6},
    "allowed_range": {
      **{p: list(TI_LIMIT_BOUNDS[a]) for p, a in PARAM_TO_LIMIT.items() if a in TI_LIMIT_BOUNDS},
      "TiSteerThreshold": list(THRESHOLD_BOUNDS),
    },
    "allowed_range_source": TI_LIMIT_SOURCE,
    "notes": {
      "TiSteerMax": "TI clips its own input at 600; higher values are discarded by the unit.",
      "TiSteerDeltaUp": "Per 10ms frame, below the knee. 6 means a full second from zero to "
                        "maximum; the stock Mazda LKAS path uses 10 against the same EPS.",
      "TiSteerDeltaUpKnee": "Command magnitude above which the climb rate drops to "
                            "TiSteerDeltaUpHigh. At its default of 600 the rate never changes and "
                            "the limiter behaves exactly as a single slope -- lowering it is what "
                            "turns the two-stage ramp on.",
      "TiSteerDeltaUpHigh": "Climb rate above the knee. Held at or below TiSteerDeltaUp, since it "
                            "exists to be the more cautious of the two.",
      "TiSteerDriverMultiplier": "Command cap is 600 + (allowance + driver_torque) * this. At 40 "
                                 "the cap reaches zero about 30 counts past the allowance.",
      "ranges": "These bounds are enforced twice, once where the toggles are read and again in the "
                "car controller, because the panda applies no steering checks to MAZDA_TI_LKAS on "
                "gen1 -- there is no safety layer downstream of openpilot on this path.",
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
    # lagd's live estimate of steering actuator delay, against the 0.1s this car port declares
    # statically. The rate limiter is indistinguishable from actuator lag as far as the lateral
    # controller is concerned, so a learned delay well above the static figure is independent
    # evidence that the ramp is starving it -- and it should fall back toward the static value if
    # opening the ramp up genuinely helps. Only meaningful once status reads "estimated".
    "lateral_delay": {
      "learned": live.get("learned_lateral_delay"),
      "estimate": live.get("learned_lateral_delay_estimate"),
      "status": live.get("lateral_delay_status"),
      "valid_blocks": live.get("lateral_delay_valid_blocks"),
      "static_from_car_port": 0.1,
      "how_to_read": ("Compare learned against static_from_car_port. Materially higher means the "
                      "controller is compensating for lag the ramp limiter is creating. Watch it "
                      "across a TiSteerDeltaUp change: it falling is corroboration independent of "
                      "the shortfall counters."),
    },
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


def _is_onroad():
  """manager mirrors the started flag into this param every cycle (system/manager/helpers.py)."""
  try:
    return params.get_bool("IsOnroad")
  except Exception:
    # Fail closed. Being unable to tell whether the car is moving is not a reason to go ahead and
    # decompress a hundred megabytes on the SoC that is running the driving model.
    return True


# How stale a snapshot may be before the park check stops believing it. The snapshot thread turns
# over on a 1s SubMaster timeout, so more than a few seconds means the messaging side is not
# keeping up and the gear reading could predate a shift out of park.
SNAPSHOT_MAX_AGE_S = 3.0


def _is_parked():
  """True only when the car is sitting in Park.

  Deliberately not 'stopped'. Stopped in Drive is still a driving situation -- the car can move and
  openpilot can engage the instant it does, which is precisely when a background rlog parse must
  not be competing with camerad and modeld. Park is the one state where neither can happen without
  the driver moving the selector first, which is not something they do by accident.

  Fails closed on every uncertainty: no snapshot, a stale one, a car port that does not decode
  gear, or any reported motion at all.
  """
  live = snapshot.get()
  if not live.get("available"):
    return False
  ts = live.get("ts")
  if ts is None or (time.monotonic() - ts) > SNAPSHOT_MAX_AGE_S:
    return False
  if live.get("gear_shifter") != "park":
    return False
  # Belt and braces against a gear reading that is fresh by timestamp but wrong: a car in park is
  # not moving, so any speed at all means one of the two signals is lying and neither gets trusted.
  return bool(live.get("standstill")) and abs(float(live.get("v_ego_ms") or 0.0)) < 0.1


def _refuse_if_driving(tool):
  """Reading a segment means decompressing tens of megabytes into RAM and walking every message in
  it, on the same SoC as modeld and camerad. Any LAN client can call these, and a drive is exactly
  when nobody is watching what the tuning laptop is doing. This is the same class of failure as the
  params fsync regression -- background work starving the camera pipeline -- reached through CPU
  and memory instead of I/O, so it gets refused rather than merely discouraged.

  Onroad-and-in-park is exempt. The reason to refuse is contention with the driving pipeline, and
  in park there is no engagement to lose -- while sitting in a hot car with the engine running for
  air conditioning is exactly when someone wants to read a drive they just finished. Ignition off
  is not a reasonable thing to require for a read-only query.
  """
  if not _is_onroad() or _is_parked():
    return None
  return {"error": f"refused: {tool} does not run while the car is in motion",
          "why": "it decompresses and walks a whole rlog on the device, which competes with "
                 "camerad and modeld; doing that mid-drive can drop camera frames and cost you "
                 "engagement",
          "what_to_do": "the segment is already recorded and is not going anywhere -- put the car "
                        "in Park and call again; the engine can stay running. ti_status, ti_stats, "
                        "ti_tuning, torque_learning and list_segments are all cheap and stay "
                        "available even while driving."}


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
  refusal = _refuse_if_driving("analyze_segment")
  if refusal is not None:
    return refusal
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
  curv_err_engaged, curv_err_limited, curv_err_free = [], [], []
  rate_limited_now = False
  transitions = []
  last_sig = None
  truncated = False

  # Shortfall and the limiter split are recomputed here rather than read from the live counters,
  # which retain only two runs and cannot be recovered for an old segment. Limits come from the
  # params as they are NOW -- if they were changed since the drive, the attribution is off.
  steer_max = _param_int("TiSteerMax") or 600
  delta_up = _param_int("TiSteerDeltaUp") or 6
  # The knee pair has to be recomputed here too. Above the knee the limiter applies the smaller
  # DELTA_UP_HIGH, so testing every frame against the base rate under-reports rate limiting in
  # exactly the regime the knee governs -- and because rate_limited_now drives the curvature-error
  # split, those frames would land in "while_tracking_freely", making the knee look free in the
  # headline outcome metric. Same bug that was fixed in the live counter; it lived on here.
  delta_up_knee = _param_int("TiSteerDeltaUpKnee") or 600
  delta_up_high = _param_int("TiSteerDeltaUpHigh") or delta_up
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
      err = abs(float(getattr(c, "desiredCurvature", 0.0)) - float(c.curvature))
      curv_err.append(err)
      # Split by whether the limiter was cutting at the time. This is the outcome metric the
      # counters cannot supply: the counters say the command was cut, this says whether the car
      # missed its line because of it. Engaged-only, because unengaged curvature error is a
      # measure of the driver, not of openpilot -- reporting the two together is how a parked or
      # hand-driven segment ends up looking like a tracking result.
      if last_lat:
        curv_err_engaged.append(err)
        (curv_err_limited if rate_limited_now else curv_err_free).append(err)
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
            rate_limited_now = False
            if abs(desired) - abs(req) > 5:
              short += 1
              # Rate limiting only counts while the command is climbing; a command collapsing under
              # driver torque moves at DELTA_DOWN and would otherwise be blamed on the ramp rate.
              applied_up = delta_up_high if abs(last_sent) >= delta_up_knee else delta_up
              if abs(req) > abs(last_sent) and abs(req - last_sent) >= applied_up:
                rate_lim += 1
                rate_limited_now = True
              # Only torque OPPOSING the command narrows the cap. openpilot's driver-torque limit
              # is signed: the bound in the driver's own direction widens instead.
              if abs(last_drv) > allowance and (last_drv * desired) < 0:
                drv_lim += 1
          last_sent = req

  def pct(vals, p):
    return round(sorted(vals)[min(len(vals) - 1, int(len(vals) * p))], 6) if vals else None

  def stat(vals):
    # n is reported alongside every mean because these subsets can be small -- a p95 over eleven
    # frames is not a p95, and a reader needs to see that rather than infer it.
    return {"mean": round(sum(vals) / len(vals), 6), "p95": pct(vals, 0.95),
            "n": len(vals)} if vals else None

  result = {
    "segment": segment,
    "truncated": truncated,
    "engaged_frames": engaged,
    "ti_mode": {TI_MODE.get(k, k): v for k, v in modes.most_common()},
    "ti_violations": {("none" if k == 0 else f"0x{k:02X} {TI_VIOL.get(k, '?')}"): v
                      for k, v in viols.most_common()},
    "ti_ramp_frames": ramp,
    # Against the live TiSteerMax, not a hardcoded 600. They agree only while TiSteerMax is at its
    # default, and the command block below already uses steer_max -- two figures called at_clip
    # that disagree the moment the limit is lowered is worse than either alone.
    "lkas_request": {"min": min(ti_req), "max": max(ti_req),
                     "at_clip": sum(1 for r in ti_req if abs(r) >= steer_max)} if ti_req else None,
    "curvature_error": stat(curv_err),
    # The one that answers "did the car track better", which the counters cannot. Compare
    # while_rate_limited against while_tracking_freely within a single drive: a large gap means the
    # ramp limiter is costing real tracking, not just command headroom. Across an A/B, the figure
    # that should improve is engaged_only.
    "curvature_error_engaged": {
      "engaged_only": stat(curv_err_engaged),
      "while_rate_limited": stat(curv_err_limited),
      "while_tracking_freely": stat(curv_err_free),
      "note": ("curvature_error above covers the whole segment including unengaged driving, where "
               "it measures the driver rather than openpilot. These three are engaged frames only."),
    } if curv_err_engaged else None,
    "command": {
      "frames": cmd_frames,
      "short_of_requested_pct": round(100.0 * short / cmd_frames, 1),
      "rate_limited_pct": round(100.0 * rate_lim / cmd_frames, 1),
      "driver_torque_limited_pct": round(100.0 * drv_lim / cmd_frames, 1),
      "at_clip_pct": round(100.0 * at_clip / cmd_frames, 1),
      "peak_command": peak_cmd,
      "limits_used": {"TiSteerMax": steer_max, "TiSteerDeltaUp": delta_up,
                      "TiSteerDeltaUpKnee": delta_up_knee, "TiSteerDeltaUpHigh": delta_up_high,
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
  refusal = _refuse_if_driving("segment_diagnostics")
  if refusal is not None:
    return refusal
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
  # Why paramsd is declaring its own output invalid, term by term.
  lp_frames = lp_invalid = 0
  lp_fail = collections.Counter()
  declared_steer_ratio = None

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
    elif w == "carParams" and declared_steer_ratio is None:
      declared_steer_ratio = float(msg.carParams.steerRatio)
    elif w == "liveParameters":
      # paramsd sets valid = all() over six terms. liveParameters is the service that dominates
      # commIssue records on this car, so knowing WHICH term fails is the whole difference between
      # a sensor-noise threshold -- benign, and expected on a rough road -- and a real estimator
      # problem. Five of the six terms are published; roll_std lives only in filterState under
      # DEBUG. So if every published term passes on a frame that is nonetheless invalid, roll_std
      # is the cause by elimination, and roll_std is a noise threshold rather than a sanity bound.
      lp = msg.liveParameters
      lp_frames += 1
      if not lp.valid:
        lp_invalid += 1
        sr, failed_any = declared_steer_ratio, False
        for name, failed in (
          ("angle_offset_average_over_10deg", abs(float(lp.angleOffsetAverageDeg)) >= 10.0),
          ("angle_offset_over_10deg", abs(float(lp.angleOffsetDeg)) >= 10.0),
          ("roll_over_10deg", abs(float(lp.roll)) >= math.radians(10.0)),
          ("stiffness_outside_0.2_to_5.0", not 0.2 <= float(lp.stiffnessFactor) <= 5.0),
          ("steer_ratio_outside_half_to_double_declared",
           sr is not None and not (0.5 * sr <= float(lp.steerRatio) <= 2.0 * sr)),
        ):
          if failed:
            lp_fail[name] += 1
            failed_any = True
        if not failed_any:
          lp_fail["no_published_term_failed__roll_std_by_elimination"] += 1
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
    "live_parameters_invalid_why": {
      "frames": lp_frames,
      "invalid_frames": lp_invalid,
      "invalid_pct": round(100.0 * lp_invalid / lp_frames, 1) if lp_frames else None,
      "declared_steer_ratio": declared_steer_ratio,
      "steer_ratio_valid_band": ([round(0.5 * declared_steer_ratio, 2),
                                  round(2.0 * declared_steer_ratio, 2)]
                                 if declared_steer_ratio else None),
      "failing_terms": dict(lp_fail.most_common()),
      "how_to_read": (
        "paramsd publishes liveParameters.valid = all() over six terms (paramsd.py:232). This "
        "counts which ones actually fail. If the dominant entry is "
        "no_published_term_failed__roll_std_by_elimination, the cause is roll_std exceeding "
        "1.5 degrees -- a NOISE threshold on the road-roll estimate, not a sanity bound, and "
        "expected to trip intermittently on rough or cambered roads. That is benign and explains "
        "an invalid flicker with nothing wrong. Any other entry dominating is a real estimator "
        "problem worth chasing. Note the offset and roll terms carry hysteresis (they must return "
        "below 8 rather than 10 to clear), so these counts are a lower bound on failures."),
    } if lp_frames else None,
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


def _fit_through_origin(cmd, bias, np):
  """The physical relationship has no offset: zero command must mean zero injected bias. Fitting an
  intercept anyway and reporting it is still worth doing -- a non-zero one means either a sensor
  offset or that something other than the TI is moving the difference."""
  denom = float(cmd @ cmd)
  if denom <= 0:
    return None
  slope = float(cmd @ bias) / denom
  resid = bias - slope * cmd
  ss_tot = float(((bias - bias.mean()) ** 2).sum())
  return {
    "slope": round(slope, 5),
    "r2": round(1.0 - float((resid ** 2).sum()) / ss_tot, 4) if ss_tot > 0 else None,
    "n": int(len(cmd)),
    "command_range": [int(cmd.min()), int(cmd.max())],
  }


def tool_ti_response(args):
  """Characterise how the interceptor's command actually turns into torque at the EPS."""
  segment = args.get("segment")
  if not segment:
    return {"error": "segment required; call list_segments first"}
  refusal = _refuse_if_driving("ti_response")
  if refusal is not None:
    return refusal
  path = _rlog_path(segment)
  if path is None:
    return {"error": f"no rlog found for {segment}"}

  try:
    import bz2
    import numpy as np
    from cereal import log as capnp_log
  except Exception as e:
    return {"error": f"cannot load decoder: {e}"}

  raw = open(path, "rb").read()
  if path.endswith(".bz2"):
    raw = bz2.decompress(raw)

  rows = []
  srcs = collections.Counter()
  eps = ti = vego = sensor1 = None
  mode = ramp = None
  lat_active = False
  sat_eps = sat_ti = 0
  over_600 = 0
  truncated = False
  # Sensor-identity pairs, collected at TI_FEEDBACK cadence and NOT gated on engagement -- the
  # bypass frames that prove the two sensors agree happen precisely when openpilot is not driving.
  identity = []
  # First src seen per address wins, so a frame duplicated across buses is counted once.
  first_src = {}
  # Redundant sensor on the same message, for inter-channel skew.
  skew = []
  # (bias, lateral accel) at liveLocationKalman cadence, for the response ceiling.
  latacc = []

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

    if w == "carState":
      vego = float(msg.carState.vEgo)
    elif w == "carControl":
      lat_active = bool(msg.carControl.latActive)
    elif w == "liveLocationKalman":
      # Lateral acceleration exactly as torqued derives it. This is the outcome domain: what a
      # count of bias actually buys in cornering, which is the only ceiling that matters to the
      # goal. The internal clamp, the DAC range and the sensor range are all upstream of it.
      try:
        llk = msg.liveLocationKalman
        if vego is not None and eps is not None and ti is not None:
          yaw_rate = float(llk.angularVelocityCalibrated.value[2])
          roll = float(llk.orientationNED.value[0])
          lat_acc = (vego * yaw_rate) - (math.sin(roll) * 9.81)
          latacc.append((eps - ti, lat_acc, vego, mode, ramp, lat_active))
      except Exception:
        pass
    elif w == "can":
      for f in msg.can:
        # The same frame is logged under several src values (0, 1 and 128+bus have all been seen
        # for these addresses), so without this every message is processed two or three times.
        # The main fit is unaffected -- it samples last-known values -- but the identity and skew
        # lists would carry each sample repeatedly and report an n several times the truth.
        if first_src.setdefault(f.address, f.src) != f.src:
          continue
        if f.address == STEER_TORQUE:
          srcs[f"eps_bus{f.src}"] += 1
          d = bytes(f.dat)
          eps = d[0] - TORQUE_SENSOR_OFFSET
          sat_eps += abs(eps) >= TORQUE_SENSOR_LIMIT
          # SENSOR1 rides the same message: 39|8@0+ (1,-128), i.e. byte 4 with a different offset
          # and a different declared range from STEER_TORQUE_SENSOR. Whether the two are meant to
          # track each other is NOT established -- this is exploratory.
          sensor1 = d[4] - 128
          skew.append((eps, sensor1))
        elif f.address == TI_FEEDBACK:
          srcs[f"ti_bus{f.src}"] += 1
          d = bytes(f.dat)
          ti = d[0] - TORQUE_SENSOR_OFFSET
          sat_ti += abs(ti) >= TORQUE_SENSOR_LIMIT
          mode, ramp = d[3], bool(d[6])
          if eps is not None:
            identity.append((eps, ti, mode))
    elif w == "sendcan":
      # Sampled at the command's cadence against the most recent sensor readings, the same
      # last-value alignment analyze_segment uses. Both sensors arrive at 50-100Hz, so the
      # staleness is under a frame, and the lag search below is what actually resolves timing.
      for f in msg.sendcan:
        if f.address != TI_STEER:
          continue
        d = bytes(f.dat)
        cmd = ((d[0] << 8 | d[1]) & 0x0FFF) - 2048
        if abs(cmd) > 600:
          over_600 += 1
        if lat_active and eps is not None and ti is not None and vego is not None:
          rows.append((ts, cmd, eps - ti, vego, mode, ramp))

  usable = [r for r in rows if r[4] == 3 and not r[5]]
  result = {
    "segment": segment,
    "truncated": truncated,
    "frames_engaged": len(rows),
    "frames_usable": len(usable),
    "sources_seen": dict(srcs),
    "commands_above_600": over_600,
    "eps_sensor_saturated_frames": sat_eps,
    "ti_sensor_saturated_frames": sat_ti,
  }
  if len(usable) < 200:
    result["verdict"] = ("NOT ENOUGH DATA: fewer than 200 frames with the TI in RUN, lateral "
                         "active and both sensors present. This segment cannot characterise "
                         "anything; pick one with sustained engaged driving.")
    return result

  # Bias behaviour per TI mode, over ALL engaged frames rather than the RUN-only subset the fit
  # uses. The faulty unit's signature lived precisely in the frames where it left RUN, ramped down
  # or threw a violation -- the frames the fit discards -- so without this, running the tool
  # against an old log returns a clean fit over the survivors and reads as "no fault found", which
  # is the filter talking rather than the data.
  by_mode = {}
  for (_, c_i, b_i, _v, m_i, r_i) in rows:
    key = f"{TI_MODE.get(m_i, m_i)}{'+ramping' if r_i else ''}"
    d = by_mode.setdefault(key, {"frames": 0, "cmd_sum": 0.0, "bias_sum": 0.0})
    d["frames"] += 1
    d["cmd_sum"] += abs(c_i)
    d["bias_sum"] += abs(b_i)
  result["bias_by_ti_mode"] = {
    k: {"frames": v["frames"],
        "slope": round(v["bias_sum"] / v["cmd_sum"], 5) if v["cmd_sum"] else None}
    for k, v in sorted(by_mode.items(), key=lambda kv: -kv[1]["frames"])
  }
  result["bias_by_ti_mode_note"] = (
    "Everything below RUN is the interceptor not taking commands. A slope near zero there is "
    "correct and expected -- it is bypassed. A slope that collapses WITHIN RUN is the fault worth "
    "finding, and that is what response.r2 and anomalies address.")

  arr = np.array([(c, b, v) for (_, c, b, v, _, _) in usable], dtype=float)
  cmd, bias, spd = arr[:, 0], arr[:, 1], arr[:, 2]

  overall = _fit_through_origin(cmd, bias, np)
  result["response"] = overall
  # An intercept is not physical here, but reporting it catches a sensor offset or a second thing
  # moving the difference, either of which would bias the slope.
  try:
    m, c0 = np.polyfit(cmd, bias, 1)
    result["with_intercept"] = {"slope": round(float(m), 5), "intercept": round(float(c0), 3)}
  except Exception:
    pass

  # Lag: how many 10ms frames after a command change the bias follows. Correlating the command
  # against progressively delayed bias and taking the peak is the same idea lagd uses for the
  # steering actuator, and it is the number that says whether the ramp is starving the controller.
  best = {"lag_frames": None, "correlation": None}
  for lag in range(0, 26):
    a, b = cmd[:len(cmd) - lag or None], bias[lag:]
    if len(a) < 200:
      break
    if a.std() == 0 or b.std() == 0:
      continue
    corr = float(np.corrcoef(a, b)[0, 1])
    if best["correlation"] is None or corr > best["correlation"]:
      best = {"lag_frames": lag, "lag_ms": lag * 10, "correlation": round(corr, 4)}
  result["command_to_bias_lag"] = best

  # Per-speed slope. The stock LKAS request is sent every frame regardless of the TI, but the EPS
  # only acts on it above the enable speed -- so if the plant really is seeing two actuators up
  # there, the slope should step at the band rather than drift smoothly through it.
  bins, edges = [], [0, LKAS_DISABLE_MS, LKAS_ENABLE_MS, 20.0, 25.0, 99.0]
  for lo, hi in zip(edges, edges[1:]):
    sel = (spd >= lo) & (spd < hi)
    if int(sel.sum()) < 150:
      continue
    fit = _fit_through_origin(cmd[sel], bias[sel], np)
    if fit:
      bins.append({"speed_ms": [round(lo, 1), round(hi, 1)], **fit})
  result["by_speed"] = bins

  below = (spd < LKAS_DISABLE_MS)
  above = (spd >= LKAS_ENABLE_MS)
  lo_fit = _fit_through_origin(cmd[below], bias[below], np) if int(below.sum()) >= 150 else None
  hi_fit = _fit_through_origin(cmd[above], bias[above], np) if int(above.sum()) >= 150 else None
  dual = {"ti_only_below_45kph": lo_fit, "ti_plus_stock_above_52kph": hi_fit}
  if lo_fit and hi_fit and lo_fit["slope"]:
    ratio = hi_fit["slope"] / lo_fit["slope"]
    dual["slope_ratio_above_over_below"] = round(ratio, 3)
    dual["reading"] = (
      "Materially above 1.0 means the stock CAM_LKAS request is contributing real torque above the "
      "enable speed, i.e. two actuators, and torqued is fitting one line through both regimes. "
      "Near 1.0 means the TI accounts for the response on its own and the stock path is inert."
      if abs(ratio - 1.0) > 0.15 else
      "Close to 1.0: no evidence of a second actuator contributing above the enable speed.")
  else:
    dual["reading"] = ("This segment does not span both regimes with enough engaged frames. Needs "
                       "a drive with sustained time both below 45kph and above 52kph.")
  result["dual_actuator_check"] = dual

  # Frames where a large command did not produce the bias the fit predicts. This is the signature
  # the whole exercise is looking for -- but sensor saturation produces the same shape, so those
  # frames are counted separately rather than being allowed to masquerade as a fault.
  if overall and overall["slope"] > 0:
    # Compare each command against the bias it actually produced, lag included. Predicting
    # instantaneously while separately measuring a command-to-bias lag makes every ramp read as
    # under-delivery -- and ramps are corner entry, which is exactly where the large commands and
    # the high-torque question live. Left uncorrected this is a false-alarm generator aimed at the
    # frames the whole investigation is about.
    lag = int(best.get("lag_frames") or 0)
    end = len(cmd) - lag if lag else len(cmd)
    c, b, s_spd = cmd[:end], bias[lag:] if lag else bias, spd[:end]
    idx_of = (lambda i: i)  # rows align with c, and usable[] is indexed the same way

    predicted = overall["slope"] * np.abs(c)
    actual = np.abs(b)
    big = np.abs(c) >= max(100.0, 0.6 * float(np.abs(c).max()))

    # Two tails, and they mean opposite things. Under-delivery is the benign failure: assist fades
    # and the driver steers. OVER-delivery is the dangerous one -- bias larger than the command
    # asked for, or bias that does not follow the command back down, which is a latched output
    # producing steering nobody requested. The +3 keeps a near-zero prediction from making the
    # ratio explode, and the 5-count floor keeps 8-bit sensor quantisation out of it.
    under = big & (actual < 0.5 * predicted)
    over = (actual >= 5) & (actual > 2.0 * predicted + 3.0)
    # Sign disagreement: bias pushing the opposite way to the command. Never expected above the
    # noise floor, and unambiguous when it happens.
    wrong_sign = big & (c * b < 0)

    def worst_of(mask, key):
      order = sorted(np.where(mask)[0], key=key, reverse=True)[:10]
      return [{"t": round(usable[idx_of(i)][0], 2), "command": int(c[i]), "bias": int(b[i]),
               "predicted_bias": round(float(predicted[i]), 1),
               "speed_ms": round(float(s_spd[i]), 1)} for i in order]

    result["anomalies"] = {
      "lag_frames_applied": lag,
      "high_command_frames": int(big.sum()),
      "under_delivery": {
        "frames": int(under.sum()),
        "pct_of_high_command_frames": round(100.0 * int(under.sum()) / max(int(big.sum()), 1), 1),
        "worst": worst_of(under, lambda i: float(predicted[i] - actual[i])),
        "meaning": "bias below half what the fit predicts -- assist fading, the benign direction",
      },
      "over_delivery": {
        "frames": int(over.sum()),
        "worst": worst_of(over, lambda i: float(actual[i] - predicted[i])),
        "meaning": ("bias more than double the prediction, or bias that has not followed the "
                    "command back down. This is the direction that produces steering the driver "
                    "did not ask for, and the one worth acting on."),
      },
      "wrong_sign": {
        "frames": int(wrong_sign.sum()),
        "worst": worst_of(wrong_sign, lambda i: abs(float(b[i]))),
        "meaning": "bias opposing the command; never expected above the noise floor",
      },
      "caveat": ("Sensor saturation looks identical to under-delivery here. Check "
                 "eps_sensor_saturated_frames: if those are numerous the shortfall is the [-85,85] "
                 "DBC range running out, not the interceptor misbehaving."),
    }

  # --- Does eps - ti actually isolate the injected bias? -----------------------------------------
  # Every bias figure anywhere in this project assumes it does. That holds only if the TI's own
  # sensor report is calibrated to match what the EPS reads through the TI's output stage. Two
  # checks settle it from recordings that already exist, and if either fails, its deviation IS the
  # correction factor rather than merely bad news.
  identity_out = {}
  bypass = [(e, t) for (e, t, m) in identity if m != 3]
  if len(bypass) >= 200:
    be = np.array([p[0] for p in bypass], dtype=float)
    bt = np.array([p[1] for p in bypass], dtype=float)
    if bt.std() > 0:
      m_b, c_b = np.polyfit(bt, be, 1)
      ok = abs(float(m_b) - 1.0) < 0.1 and abs(float(c_b)) < 3.0
      identity_out["bypass_regression"] = {
        "n": len(bypass), "slope": round(float(m_b), 4), "intercept": round(float(c_b), 3),
        "expected": "slope 1.0, intercept 0",
        "verdict": ("the two sensors agree with the DAC out of the loop -- the subtraction is sound"
                    if ok else
                    "THEY DO NOT AGREE. The slope is the scale factor between the TI's report and "
                    "the EPS reading, and every bias figure in this project needs dividing by it."),
      }
  else:
    identity_out["bypass_regression"] = {
      "n": len(bypass),
      "note": ("needs >=200 frames with the TI out of RUN, where the sensor passes through "
               "untouched. A healthy unit never leaves RUN, so run this against an old recording "
               "from the faulty unit, which sat bypassed for whole segments."),
    }

  quiet = [(c, b) for (_, c, b, _v, m, r) in rows if m == 3 and not r and abs(c) <= 20]
  if len(quiet) >= 100:
    qb = np.array([p[1] for p in quiet], dtype=float)
    identity_out["near_zero_command_residual"] = {
      "n": len(quiet),
      "mean_bias": round(float(qb.mean()), 3),
      "abs_p95": round(float(np.percentile(np.abs(qb), 95)), 2),
      "expected": "≈ 0",
      "verdict": ("consistent with zero -- the full path including the DAC passes through cleanly"
                  if abs(float(qb.mean())) < 2.0 else
                  "NOT zero. A standing offset with no command means the subtraction is not "
                  "isolating what we think, or the TI injects at rest."),
    }
  result["measurement_identity"] = identity_out

  # --- What a count of bias is actually worth ----------------------------------------------------
  # The ceiling that matters is not the interceptor's internal clamp, the DAC range or the sensor
  # range -- it is wherever the EPS's response to sensor bias rolls off. That is observable here,
  # in the outcome domain, and it is what replaces the cross-unit "36 of 119" comparison that was
  # never a valid ratio.
  la = np.array([(b, a) for (b, a, v, m, r, act) in latacc
                 if act and m == 3 and not r and v > 5.0 and abs(b) >= 3], dtype=float)
  if len(la) >= 200:
    lb, ll = la[:, 0], la[:, 1]
    whole = _fit_through_origin(lb, ll, np)
    # Split at the median magnitude and compare. A materially lower slope in the upper half is
    # rolloff -- the EPS giving less per count as bias grows, which is exactly the shape the
    # high-torque suspicion predicts and which no amount of TI headroom would fix.
    cut = float(np.median(np.abs(lb)))
    lo_sel, hi_sel = np.abs(lb) < cut, np.abs(lb) >= cut
    lo_fit = _fit_through_origin(lb[lo_sel], ll[lo_sel], np) if int(lo_sel.sum()) >= 80 else None
    hi_fit = _fit_through_origin(lb[hi_sel], ll[hi_sel], np) if int(hi_sel.sum()) >= 80 else None
    out = {"overall": whole, "split_at_bias": round(cut, 1),
           "lower_half": lo_fit, "upper_half": hi_fit,
           "units": "m/s^2 of lateral acceleration per count of bias"}
    if lo_fit and hi_fit and lo_fit["slope"]:
      ratio = hi_fit["slope"] / lo_fit["slope"]
      out["upper_over_lower"] = round(ratio, 3)
      # Comparing two slopes is only meaningful if both fits are any good. Lateral acceleration is
      # driven by steering ANGLE through the whole vehicle, not by torque directly, so this fit is
      # confounded by speed and by how much cornering happened to occur -- and a slope ratio drawn
      # between a tight fit and a scattered one says more about the scatter than about the plant.
      worst_r2 = min(lo_fit["r2"] or 0.0, hi_fit["r2"] or 0.0)
      if worst_r2 < 0.5:
        out["verdict"] = (
          f"INCONCLUSIVE: the weaker half fits at r2={worst_r2:.2f}, too scattered to compare "
          f"slopes against. The ratio is reported but should not be read as rolloff or its "
          f"absence. Needs a segment with more sustained cornering.")
      else:
        out["verdict"] = (
          "ROLLOFF: the EPS returns materially less per count of bias at higher bias. If this "
          "holds up, the binding constraint is the EPS response rather than any interceptor limit, "
          "and raising command headroom will not buy proportional cornering." if ratio < 0.8 else
          "No rolloff over the range driven: response is proportional, so amplitude headroom is "
          "worth having if it can be obtained. Says nothing about bias levels never reached here.")
      out["caveat"] = (
        "torqued fits this same torque-to-lateral-acceleration relationship properly -- with "
        "buckets, lag compensation and point filtering -- and publishes it as latAccelFactor. "
        "Where the two disagree, believe torqued and treat this as a cheap cross-check. Its value "
        "is in comparing the SAME figure across drives, not in its absolute magnitude. The sign is "
        "negative by convention: torqued negates actuatorsOutput.steer for exactly this reason.")
    result["lateral_response"] = out
  else:
    result["lateral_response"] = {
      "n": int(len(la)),
      "note": ("needs >=200 engaged frames above 5 m/s with appreciable bias. liveLocationKalman "
               "publishes at 20Hz, so a segment of sustained engaged cornering is required."),
    }

  # --- Redundant sensor on the same message ------------------------------------------------------
  # The interceptor drives two DAC channels, and 0x240 carries a second sensor. Divergence between
  # them is the signature an EPS plausibility check would trip on. Exploratory: the DBC declares the
  # two signals with different offsets and ranges, so whether they are meant to track is unknown --
  # a stable relationship of ANY shape is the useful finding, and a drifting one is the interesting.
  if len(skew) >= 200:
    s0 = np.array([p[0] for p in skew], dtype=float)
    s1 = np.array([p[1] for p in skew], dtype=float)
    corr = float(np.corrcoef(s0, s1)[0, 1]) if s0.std() > 0 and s1.std() > 0 else None
    result["redundant_sensor"] = {
      "n": len(skew),
      "steer_torque_sensor": {"mean": round(float(s0.mean()), 2), "min": int(s0.min()),
                              "max": int(s0.max())},
      "sensor1": {"mean": round(float(s1.mean()), 2), "min": int(s1.min()), "max": int(s1.max())},
      "correlation": round(corr, 4) if corr is not None else None,
      "how_to_read": ("A high absolute correlation, positive or negative, means the two track and "
                      "the pair can be used as a consistency check going forward. Near zero means "
                      "SENSOR1 measures something else entirely and should be ignored. Correlation "
                      "that DROPS between segments, or within one, is the inter-channel divergence "
                      "worth chasing -- compare this figure across drives rather than judging it "
                      "in isolation."),
    }

  result["how_to_read"] = (
    "response.slope is bias counts per count of command -- the conversion the whole tuning problem "
    "rests on, and multiplying it by TiSteerMax gives the most bias this setup can ever deliver. "
    "r2 near 1 means the relationship is linear over the range driven; a low r2 with a healthy n "
    "is the first real evidence of the nonlinearity we have been assuming. command_to_bias_lag is "
    "how long the EPS takes to follow. by_speed and dual_actuator_check test whether the stock "
    "LKAS path is contributing torque above 52kph, which would mean torqued has been fitting one "
    "line through two different plants. anomalies is the decoder-glitch search; read its caveat "
    "before believing it. commands_above_600 tests whether anything above the clip was ever sent.")
  return result


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
     "short_of_requested_pct is how often openpilot cut its own command and "
     "mean_deficit_per_engaged_frame is by how much -- watch the second one across an A/B, since a "
     "change can shrink the deficit without moving the percentage. The rate_limited vs "
     "driver_torque_limited split says WHICH parameter to change (rate -> TiSteerDeltaUp, driver "
     "-> TiSteerDriverMultiplier/Allowance). Each run carries the limits it was recorded under, "
     "its route and start time, and config_changed_between_runs states plainly whether the two "
     "runs are even comparable. Only two runs are retained -- a third clear destroys the first, so "
     "save this output after every run. For an older drive use analyze_segment instead, which "
     "recomputes the same figures from the log.",
   "inputSchema": {"type": "object", "properties": {}}, "fn": tool_ti_stats},

  {"name": "ti_flags",
   "description":
     "Moments the driver flagged from the tuning panel while driving, newest first -- odd "
     "behaviour, a torque dropout, anything worth a second look. CHECK THIS EARLY in any "
     "investigation of a drive: it is the driver telling you where to look, and it turns 'trawl "
     "the whole route' into 'read these two segments'. Each flag carries the segment name to pass "
     "straight to ti_response, analyze_segment or segment_diagnostics, the neighbouring segments "
     "to check alongside it, and what the interceptor was doing at that instant (command, bias, "
     "driver torque, mode, violation) plus the tuning limits in force, so a flag from an earlier "
     "tuning state is still interpretable. Empty just means nothing has been flagged.",
   "inputSchema": {"type": "object", "properties": {
     "limit": {"type": "integer", "description": "How many to return, newest first (default 25)"}}},
   "fn": tool_ti_flags},

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
     "healthy but proves nothing. Only the summary crosses the wire, never the log itself. "
     "Refuses to run while the car is in motion, because the parse competes with camerad and "
     "modeld; put it in Park and call again, engine running is fine.",
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
     "the TI; use analyze_segment for that. Refuses to run while the car is in motion, because "
     "the parse competes with camerad and modeld; put it in Park, engine running is fine.",
   "inputSchema": {"type": "object", "properties": {
     "segment": {"type": "string",
                 "description": "Segment name exactly as returned by list_segments"},
     "limit": {"type": "integer", "description": "Max records of each kind (default 25)"}},
     "required": ["segment"]},
   "fn": tool_segment_diagnostics},

  {"name": "ti_response",
   "description":
     "Characterises what the interceptor's command actually DOES: fits bias (the EPS torque sensor "
     "minus the TI's driver-only sensor, both declared in identical DBC units) against the 0x249 "
     "command. Returns counts-of-bias-per-count-of-command with an r2 for linearity, the "
     "command-to-bias lag in frames, the slope broken down by speed, and the frames where a large "
     "command failed to produce the bias the fit predicts -- the signature of the high-torque "
     "unreliability this fork exists to chase, reported next to a sensor-saturation count because "
     "saturation looks identical and must not be mistaken for it. Also tests whether the stock "
     "Mazda LKAS request contributes torque above its 52kph enable speed, which would mean the car "
     "has two actuators and torqued has been fitting one line through both plants. "
     "THREE FIELDS DESERVE READING BEFORE THE SLOPE ITSELF. measurement_identity checks whether "
     "the eps-minus-ti subtraction isolates injected bias at all -- every bias number in this "
     "project rests on that and it had never been verified; if it fails, its deviation is the "
     "correction factor. lateral_response gives m/s^2 of cornering per count of bias, split high "
     "against low, which is the only ceiling that matters to the goal and the one that replaces "
     "comparing figures from different unit domains. redundant_sensor tracks a second sensor on "
     "the same CAN message against the first, since divergence between the interceptor's two DAC "
     "channels is what an EPS plausibility check would trip on. Use this to establish the plant "
     "BEFORE changing limits; use ti_stats and analyze_segment to measure the effect afterwards. "
     "Needs sustained engaged driving, ideally spanning both below 45kph and above 52kph; the "
     "bypass half of measurement_identity needs an OLD recording from the faulty unit, which sat "
     "out of RUN for whole segments. Refuses to run while the car is in motion; Park is enough.",
   "inputSchema": {"type": "object", "properties": {
     "segment": {"type": "string",
                 "description": "Segment name exactly as returned by list_segments"}},
     "required": ["segment"]},
   "fn": tool_ti_response},
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
      # Every request this server understands is a small JSON object. Reading an arbitrary length
      # into RAM on a device that is also running the driving model is not something to leave open
      # to whatever else is on the network.
      if length > MAX_REQUEST_BYTES:
        self._send({"jsonrpc": "2.0", "id": None,
                    "error": {"code": -32600,
                              "message": f"request too large ({length} bytes, "
                                         f"limit {MAX_REQUEST_BYTES})"}}, 413)
        return
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
