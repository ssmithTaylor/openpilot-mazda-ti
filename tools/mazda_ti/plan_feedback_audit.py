#!/usr/bin/env python3
"""Describe plan-to-measured-motion phase association in declared Mazda rlog windows.

This is deliberately an observational audit.  It does not reconstruct a controller's
exact consumed model input, identify a transport delay, or show that a plan caused a
vehicle motion (or the reverse).  Its purpose is to make the potentially confounded
"plan mirrors the car" claim falsifiable on a fixed, rider-labelled case set.
"""

from __future__ import annotations

import argparse
import cmath
import hashlib
import json
import math
import statistics
import sys
from bisect import bisect_right
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.mazda_ti.audit_diagnostics import read_events

HZ = 20.0
MAX_AGE_NS = 200_000_000
MAX_LAG_S = 1.0
BAND_HZ = (0.4, 3.0)
WELCH_SECONDS = 3.0
WELCH_HOP_SECONDS = 1.5


def sha256(path: Path) -> str:
  h = hashlib.sha256()
  with path.open("rb") as f:
    for block in iter(lambda: f.read(1 << 20), b""):
      h.update(block)
  return h.hexdigest()


def latest(rows: list[dict[str, Any]], t_ns: int) -> dict[str, Any] | None:
  i = bisect_right([r["t_ns"] for r in rows], t_ns) - 1
  if i < 0:
    return None
  return {**rows[i], "age_ns": t_ns - rows[i]["t_ns"]}


def pearson(a: list[float], b: list[float]) -> float | None:
  if len(a) < 3 or len(a) != len(b):
    return None
  ma, mb = statistics.fmean(a), statistics.fmean(b)
  da = math.sqrt(sum((x - ma) ** 2 for x in a))
  db = math.sqrt(sum((x - mb) ** 2 for x in b))
  return None if da == 0 or db == 0 else sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (da * db)


def derivative(values: list[float], dt: float) -> list[float]:
  # A centered difference would use future samples.  This backward difference is
  # intentionally causal within the recorded observations, while remaining only
  # a descriptive transform.
  return [(values[i] - values[i - 1]) / dt for i in range(1, len(values))]


def lag_scan(source: list[float], response: list[float], max_lag_rows: int) -> dict[str, Any]:
  scans: list[dict[str, Any]] = []
  for lag in range(-max_lag_rows, max_lag_rows + 1):
    pairs = [(source[i], response[i + lag]) for i in range(len(source)) if 0 <= i + lag < len(response)]
    corr = pearson([x for x, _ in pairs], [y for _, y in pairs])
    if corr is not None:
      scans.append({"lag_ms": lag * 1000.0 / HZ, "correlation": corr, "pairs": len(pairs)})
  if not scans:
    return {"status": "insufficient_rows"}
  # Positive lag means response is sampled later than source.  The maximum is
  # association only: a road curvature change can drive both series.
  best = max(scans, key=lambda x: x["correlation"])
  edge_limited = abs(best["lag_ms"]) >= max_lag_rows * 1000.0 / HZ
  return {"status": "edge_limited" if edge_limited else "descriptive_only",
          "edge_limited": edge_limited,
          "sign_convention": "positive_lag_means_measured_yaw_derivative_is_later_than_model_request_derivative",
          "best": best, "zero_lag": next(x for x in scans if x["lag_ms"] == 0.0), "scan": scans}


def dft(values: list[float], k: int) -> complex:
  n = len(values)
  return sum(values[i] * cmath.exp(-2j * math.pi * k * i / n) for i in range(n))


def cross_spectrum(source: list[float], response: list[float]) -> dict[str, Any]:
  """Welch CSD; phase > 0 means response trails source under our X*conj(Y) sign."""
  n = int(WELCH_SECONDS * HZ)
  hop = int(WELCH_HOP_SECONDS * HZ)
  if len(source) < n or len(response) < n:
    return {"status": "insufficient_rows", "required_rows": n, "available_rows": min(len(source), len(response))}
  windows = [(source[i:i+n], response[i:i+n]) for i in range(0, min(len(source), len(response)) - n + 1, hop)]
  if len(windows) < 3:
    return {"status": "insufficient_independent_windows", "windows": len(windows), "required_windows": 3}
  rows = []
  for k in range(1, n // 2):
    f = k * HZ / n
    if not BAND_HZ[0] <= f <= BAND_HZ[1]:
      continue
    csd = 0j
    pxx = 0.0
    pyy = 0.0
    for x, y in windows:
      # Remove per-window mean and apply a Hann taper before DFT.
      xw = [(v - statistics.fmean(x)) * (0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1))) for i, v in enumerate(x)]
      yw = [(v - statistics.fmean(y)) * (0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1))) for i, v in enumerate(y)]
      X, Y = dft(xw, k), dft(yw, k)
      csd += X * Y.conjugate()
      pxx += abs(X) ** 2
      pyy += abs(Y) ** 2
    csd /= len(windows)
    pxx /= len(windows)
    pyy /= len(windows)
    coherence = (abs(csd) ** 2 / (pxx * pyy)) if pxx and pyy else None
    rows.append({"frequency_hz": f, "coherence": coherence, "phase_rad": cmath.phase(csd),
                 "phase_delay_ms": cmath.phase(csd) * 1000.0 / (2 * math.pi * f)})
  qualified = [row for row in rows if row["coherence"] is not None and row["coherence"] >= 0.6]
  return {"status": "descriptive_only", "windows": len(windows), "window_seconds": WELCH_SECONDS,
          "hop_seconds": WELCH_HOP_SECONDS, "sign_convention": "positive_phase_delay_means_measured_yaw_derivative_trails_model_request_derivative",
          "minimum_coherence_for_phase_review": 0.6, "qualified_band_bins": qualified, "bins": rows}


def streams(paths: list[Path]) -> tuple[dict[str, list[dict[str, Any]]], list[dict[str, Any]]]:
  out = {"model": [], "car": [], "llk": []}
  init = []
  for event in read_events(paths):
    kind, t_ns = event.which(), int(event.logMonoTime)
    if kind == "initData":
      init.append({"git_commit": str(event.initData.gitCommit), "dirty": bool(event.initData.dirty)})
    elif kind == "modelV2":
      x = event.modelV2
      out["model"].append({"t_ns": t_ns, "valid": bool(event.valid), "desired_curvature": float(x.action.desiredCurvature)})
    elif kind == "carState":
      x = event.carState
      out["car"].append({"t_ns": t_ns, "valid": bool(event.valid), "can_valid": bool(x.canValid), "speed_mps": float(x.vEgo)})
    elif kind == "liveLocationKalman":
      x = event.liveLocationKalman
      out["llk"].append({"t_ns": t_ns, "valid": bool(event.valid), "yaw_valid": bool(x.angularVelocityCalibrated.valid),
                         "health": bool(x.sensorsOK and x.inputsOK and x.posenetOK), "yaw_rps": float(x.angularVelocityCalibrated.value[2])})
  for rows in out.values():
    rows.sort(key=lambda r: r["t_ns"])
  return out, init


def sample_case(case: dict[str, Any], root: Path) -> dict[str, Any]:
  # Preserve the caller's repository-relative spelling in the durable artifact.
  # The local raw directory is commonly a junction to the managed rlog store, so
  # resolving it before calculating a relative path would mislabel the evidence.
  paths = [root / p for p in case["rlogs"]]
  for path in paths:
    if not path.is_file():
      raise FileNotFoundError(path)
  start_ns, end_ns = int(case["start_ns"]), int(case["end_ns"])
  stream, init = streams(paths)
  rows = []
  rejected = {"stale_or_missing": 0, "health": 0}
  for model in stream["model"]:
    if not start_ns <= model["t_ns"] < end_ns:
      continue
    car, llk = latest(stream["car"], model["t_ns"]), latest(stream["llk"], model["t_ns"])
    if car is None or llk is None or car["age_ns"] > MAX_AGE_NS or llk["age_ns"] > MAX_AGE_NS:
      rejected["stale_or_missing"] += 1
      continue
    if not (model["valid"] and car["valid"] and car["can_valid"] and llk["valid"] and llk["yaw_valid"] and llk["health"]):
      rejected["health"] += 1
      continue
    rows.append({"t_ns": model["t_ns"], "plan_lateral_accel_mps2": model["desired_curvature"] * car["speed_mps"] ** 2,
                 "yaw_lateral_accel_mps2": llk["yaw_rps"] * car["speed_mps"], "speed_mps": car["speed_mps"],
                 "model_age_to_car_ms": car["age_ns"] / 1e6, "model_age_to_llk_ms": llk["age_ns"] / 1e6})
  if len(rows) < 4:
    return {"label": case["label"], "status": "insufficient_healthy_rows", "rows": len(rows), "rejected": rejected,
            "raw_sha256": {str(p.relative_to(root)).replace("\\", "/"): sha256(p) for p in paths}, "init_data": init}
  # A gap breaks the derivative sequence; avoid silently bridging a missing model publication.
  contiguous = [rows[0]]
  gap_rows = 0
  for row in rows[1:]:
    if row["t_ns"] - contiguous[-1]["t_ns"] > int(1.5e9 / HZ):
      gap_rows += 1
      continue
    contiguous.append(row)
  plan = [r["plan_lateral_accel_mps2"] for r in contiguous]
  yaw = [r["yaw_lateral_accel_mps2"] for r in contiguous]
  dt = statistics.median([(b["t_ns"] - a["t_ns"]) / 1e9 for a, b in zip(contiguous, contiguous[1:])])
  plan_d, yaw_d = derivative(plan, dt), derivative(yaw, dt)
  return {
    "label": case["label"], "status": "descriptive_only",
    "window_ns": [start_ns, end_ns], "rows": len(contiguous), "dropped_gap_rows": gap_rows, "sample_dt_ms": dt * 1000.0,
    "rejected": rejected, "raw_sha256": {str(p.relative_to(root)).replace("\\", "/"): sha256(p) for p in paths}, "init_data": init,
    "inputs": {
      "plan_lateral_accel": "as-of modelV2.action.desiredCurvature * as-of carState.vEgo^2",
      "measured_lateral_accel": "as-of liveLocationKalman angularVelocityCalibrated.z * as-of carState.vEgo",
      "exact_consumed_model_identity": "unavailable in these legacy logs",
    },
    "ranges": {"plan_lateral_accel_mps2": [min(plan), max(plan)], "yaw_lateral_accel_mps2": [min(yaw), max(yaw)],
               "speed_mps": [min(r["speed_mps"] for r in contiguous), max(r["speed_mps"] for r in contiguous)]},
    "derivative_phase": lag_scan(plan_d, yaw_d, round(MAX_LAG_S * HZ)),
    "cross_spectrum": cross_spectrum(plan_d, yaw_d),
  }


def build_markdown(result: dict[str, Any]) -> str:
  lines = ["# Plan-to-motion feedback audit", "", result["scope"], "",
           "## Frozen tests", "",
           "The time-domain test scans the correlation of backward-difference plan demand and measured yaw lateral acceleration over ±1.0 s. Positive lag means measured yaw is later than plan demand. The cross-spectral test is a 3.0 s Hann-windowed Welch estimate in 0.4–3.0 Hz; it requires three windows and reports phase only as an association.",
           "", "A plan-mirror mechanism would need a reproducible negative plan-to-yaw lag (the plan trailing measured motion) in rider-labelled symptoms, a materially different result from the smooth controls, and a causal experiment that changes the plan/reference while allowing its own feedback history to evolve. These observations alone cannot establish that mechanism.",
           "", "## Results", "", "| case | label | rows | derivative result | best lag | correlation | spectral result |", "|---|---|---:|---|---:|---:|---|"]
  for name, case in result["cases"].items():
    phase = case.get("derivative_phase", {}).get("best", {})
    lines.append(f"| {name} | {case['label']} | {case.get('rows', 0)} | {case.get('derivative_phase', {}).get('status', 'unavailable')} | {phase.get('lag_ms', 'unavailable')} ms | {phase.get('correlation', 'unavailable')} | {case.get('cross_spectrum', {}).get('status', 'unavailable')} |")
  lines += ["", "## Interpretation", "", result["interpretation"], "", "## Limits", ""]
  lines.extend(f"- {limit}" for limit in result["limits"])
  lines.append("")
  return "\n".join(lines)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--request", type=Path, required=True)
  parser.add_argument("--data-root", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--markdown", type=Path, required=True)
  args = parser.parse_args()
  if args.output.exists() or args.markdown.exists():
    parser.error("outputs already exist; preserve prior evidence and choose fresh paths")
  request = json.loads(args.request.read_text(encoding="utf-8"))
  root = args.data_root.resolve()
  source_paths = [Path(__file__).resolve(), ROOT / "tools/mazda_ti/audit_diagnostics.py", ROOT / "cereal/log.capnp"]
  result = {
    "format_version": 1,
    "scope": "Source-locked observational comparison of as-of model plan demand and measured yaw response in rider-labelled Mazda windows. It predeclares phase tests and keeps plan/motion association separate from causal attribution.",
    "request": request,
    "request_sha256": sha256(args.request),
    "source_sha256": {str(p.relative_to(ROOT)).replace("\\", "/"): sha256(p) for p in source_paths},
    "tests_predeclared": {
      "time_domain": "Backward-difference plan-vs-yaw lag scan ±1.0 s; report all lags, not only maximum.",
      "cross_spectral": "3.0 s Hann-windowed Welch CSD at 0.4–3.0 Hz; require >=3 windows; phase sign is explicitly recorded.",
      "causal_rule": "No causal conclusion from association. A mirror claim remains unqualified absent exact consumed identities and a candidate-responsive closed-loop test.",
    },
    "cases": {case["name"]: sample_case(case, root) for case in request["cases"]},
    "limits": [
      "modelV2.action.desiredCurvature is joined as-of the model publication, not proven to be the exact controller input consumed on the legacy drive.",
      "model demand and lane/motion estimates can share scene and estimator inputs; correlation or spectral phase can arise from road geometry or common observation timing.",
      "Yaw times speed is a localization-derived response proxy, not surveyed vehicle path or actuator delivery.",
      "Smooth 24d/261 controls are older NNFF-era recordings; their controller input path and source differ, so they are negative controls for a universal phase signature, not an isolated A/B.",
      "The route280 rider label is held-inward/over-release with low confidence for short scallops; it is not labelled as a repeated scallop positive.",
    ],
  }
  symptom = [v for k, v in result["cases"].items() if k in request.get("symptom_cases", [])]
  smooth = [v for k, v in result["cases"].items() if k in request.get("smooth_cases", [])]
  def lag_summary(case: dict[str, Any]) -> dict[str, Any]:
    phase = case.get("derivative_phase", {})
    return {"lag_ms": phase.get("best", {}).get("lag_ms"), "status": phase.get("status")}
  def spectral_bins(case: dict[str, Any]) -> int:
    return len(case.get("cross_spectrum", {}).get("qualified_band_bins", []))
  result["interpretation"] = (
    "This fixed audit does not support a causal plan-mirror conclusion. "
    "A negative best lag would be compatible with the plan trailing measured response, but is also compatible with common road/vision timing; a positive lag is compatible with ordinary plan-to-vehicle response. "
    f"Symptom best-lag summaries: {[lag_summary(c) for c in symptom]}; smooth-control summaries: {[lag_summary(c) for c in smooth]}. "
    f"Symptom spectral bins meeting the predeclared 0.6 coherence screen: {[spectral_bins(c) for c in symptom]}. "
    "The next discriminating evidence is an instrumented matched drive with exact consumed model and applied-command identities, followed by a source-locked closed-loop candidate/equivalent-control comparison."
  )
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.markdown.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
  args.markdown.write_text(build_markdown(result), encoding="utf-8", newline="\n")
  print(json.dumps({"output": str(args.output), "output_sha256": sha256(args.output), "markdown": str(args.markdown), "markdown_sha256": sha256(args.markdown)}, sort_keys=True))


if __name__ == "__main__":
  main()
