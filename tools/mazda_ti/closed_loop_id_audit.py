#!/usr/bin/env python3
"""Cross-validated, causal-predictor audit of published-TI response proxies.

The input is a compact, source-locked JSON object containing independently
named cases and uniformly sampled rows.  A row has only values available at
its timestamp: ``time_s``, published ``ti_command_counts``, measured
``wheel_angle_deg``, fused ``yaw_lateral_accel_mps2``, and ``speed_mps``.

This tool deliberately treats published TI as a command proxy.  It is not an
applied CarControl identity, delivered motor torque, or tire/rack force.  The
models predict one next-sample measured increment; they do not roll out a
vehicle path or select a controller change.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


FORMAT_VERSION = 1
ROOT = Path(__file__).resolve().parents[2]
LAG_SAMPLES = (0, 1, 2, 3, 4, 5, 6, 8)
RIDGE = 1e-6
MODELS = {
  "state_only": "Causal autoregression on current measured response state, prior response increment, and speed.",
  "delayed_command": "state_only plus one delayed published-TI command proxy.",
  "delayed_command_rate": "delayed_command plus the causally available command increment at that delay (a phase proxy, not physical damping).",
}


def sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as handle:
    for block in iter(lambda: handle.read(1 << 20), b""):
      digest.update(block)
  return digest.hexdigest()


def require(condition: bool, message: str) -> None:
  if not condition:
    raise ValueError(message)


def load_input(path: Path) -> dict[str, Any]:
  payload = json.loads(path.read_text(encoding="utf-8"))
  require(payload.get("format_version") == FORMAT_VERSION, "unsupported input format_version")
  cases = payload.get("cases")
  require(isinstance(cases, list) and len(cases) >= 4, "need at least four cases for leave-case-out evaluation")
  names: set[str] = set()
  for case in cases:
    name = case.get("case_id")
    require(isinstance(name, str) and name and name not in names, "case_id must be unique")
    names.add(name)
    require(case.get("outcome") in ("smooth", "settling", "scallop"), f"unsupported outcome for {name}")
    rows = case.get("rows")
    require(isinstance(rows, list) and len(rows) >= 30, f"{name} needs >=30 rows")
    previous_time = -math.inf
    for row in rows:
      for key in ("time_s", "ti_command_counts", "wheel_angle_deg", "yaw_lateral_accel_mps2", "speed_mps"):
        value = row.get(key)
        require(isinstance(value, (int, float)) and math.isfinite(float(value)), f"{name}: invalid {key}")
      require(float(row["time_s"]) > previous_time, f"{name}: time_s must be strictly increasing")
      previous_time = float(row["time_s"])
  return payload


def normalize_case(case: dict[str, Any]) -> list[dict[str, float]]:
  """Normalize a sustained bend to positive while retaining time and speed."""
  rows = [{key: float(value) for key, value in row.items()} for row in case["rows"]]
  direction = 1.0 if np.mean([row["ti_command_counts"] for row in rows]) >= 0.0 else -1.0
  for row in rows:
    for key in ("ti_command_counts", "wheel_angle_deg", "yaw_lateral_accel_mps2"):
      row[key] *= direction
  return rows


def feature_names(model: str) -> list[str]:
  names = ["intercept", "state", "state_increment", "speed_mps"]
  if model != "state_only":
    names.append("published_ti_counts")
  if model == "delayed_command_rate":
    names.append("published_ti_increment_counts")
  return names


def materialize(rows: list[dict[str, float]], response_key: str, model: str, lag: int) -> list[tuple[np.ndarray, float, float]]:
  """Build one-step rows with no future value in any predictor."""
  require(model in MODELS, f"unknown model {model}")
  result: list[tuple[np.ndarray, float, float]] = []
  first = max(1, lag + 1)
  for index in range(first, len(rows) - 1):
    current, previous, nxt = rows[index], rows[index - 1], rows[index + 1]
    if nxt["time_s"] - current["time_s"] > 0.11:
      continue
    delayed = rows[index - lag]
    previous_delayed = rows[index - lag - 1]
    values = [
      1.0,
      current[response_key],
      current[response_key] - previous[response_key],
      current["speed_mps"],
    ]
    if model != "state_only":
      values.append(delayed["ti_command_counts"])
    if model == "delayed_command_rate":
      values.append(delayed["ti_command_counts"] - previous_delayed["ti_command_counts"])
    result.append((np.asarray(values, dtype=float), nxt[response_key] - current[response_key], current["time_s"]))
  return result


def fit(entries: list[tuple[np.ndarray, float, float]]) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
  require(bool(entries), "cannot fit no entries")
  matrix = np.asarray([entry[0] for entry in entries], dtype=float)
  target = np.asarray([entry[1] for entry in entries], dtype=float)
  mean = matrix[:, 1:].mean(axis=0)
  scale = matrix[:, 1:].std(axis=0)
  scale[scale < 1e-9] = 1.0
  normalized = matrix.copy()
  normalized[:, 1:] = (normalized[:, 1:] - mean) / scale
  coefficients = np.linalg.solve(normalized.T @ normalized + RIDGE * np.eye(normalized.shape[1]), normalized.T @ target)
  return coefficients, mean, scale, float(np.linalg.cond(normalized))


def predict(fitted: tuple[np.ndarray, np.ndarray, np.ndarray, float], entries: list[tuple[np.ndarray, float, float]]) -> np.ndarray:
  coefficients, mean, scale, _ = fitted
  matrix = np.asarray([entry[0] for entry in entries], dtype=float)
  normalized = matrix.copy()
  normalized[:, 1:] = (normalized[:, 1:] - mean) / scale
  return normalized @ coefficients


def score(fitted: tuple[np.ndarray, np.ndarray, np.ndarray, float], entries: list[tuple[np.ndarray, float, float]]) -> dict[str, float | int]:
  observed = np.asarray([entry[1] for entry in entries], dtype=float)
  residual = predict(fitted, entries) - observed
  return {
    "rows": int(len(entries)),
    "mae": float(np.mean(np.abs(residual))),
    "rmse": float(np.sqrt(np.mean(residual * residual))),
    "residual_mean": float(np.mean(residual)),
  }


def blocks(entries: list[tuple[np.ndarray, float, float]], count: int = 3) -> list[list[tuple[np.ndarray, float, float]]]:
  return [list(chunk) for chunk in np.array_split(np.asarray(entries, dtype=object), count)]


def nested_config(model: str, train: dict[str, list[dict[str, float]]], response_key: str) -> tuple[int, float]:
  candidates = (0,) if model == "state_only" else LAG_SAMPLES
  candidate_scores: list[tuple[float, int]] = []
  for lag in candidates:
    per_case = {name: materialize(rows, response_key, model, lag) for name, rows in train.items()}
    weighted_absolute_error, total = 0.0, 0
    for held_block in range(3):
      fitting: list[tuple[np.ndarray, float, float]] = []
      validation: list[tuple[np.ndarray, float, float]] = []
      for entries in per_case.values():
        for index, block in enumerate(blocks(entries)):
          (validation if index == held_block else fitting).extend(block)
      result = score(fit(fitting), validation)
      weighted_absolute_error += float(result["mae"]) * int(result["rows"])
      total += int(result["rows"])
    candidate_scores.append((weighted_absolute_error / total, lag))
  return min(candidate_scores, key=lambda value: (value[0], value[1]))[1], min(candidate_scores, key=lambda value: (value[0], value[1]))[0]


def original_coefficients(fitted: tuple[np.ndarray, np.ndarray, np.ndarray, float], names: list[str]) -> dict[str, float]:
  coefficients, mean, scale, _ = fitted
  result = {"intercept": float(coefficients[0] - np.sum(coefficients[1:] * mean / scale))}
  result.update({name: float(coefficient / divisor) for name, coefficient, divisor in zip(names[1:], coefficients[1:], scale)})
  return result


def run(payload: dict[str, Any]) -> dict[str, Any]:
  raw_cases = payload["cases"]
  normalized = {case["case_id"]: normalize_case(case) for case in raw_cases}
  outcomes = {case["case_id"]: case["outcome"] for case in raw_cases}
  responses = {
    "wheel_angle_increment_deg": "wheel_angle_deg",
    "yaw_lateral_accel_increment_mps2": "yaw_lateral_accel_mps2",
  }
  results: dict[str, Any] = {}
  for response_name, response_key in responses.items():
    per_target: dict[str, Any] = {}
    for target in normalized:
      train = {name: rows for name, rows in normalized.items() if name != target}
      per_model: dict[str, Any] = {}
      for model in MODELS:
        lag, validation_mae = nested_config(model, train, response_key)
        training_rows = [entry for rows in train.values() for entry in materialize(rows, response_key, model, lag)]
        held_out_rows = materialize(normalized[target], response_key, model, lag)
        fitted = fit(training_rows)
        per_model[model] = {
          "selected_on_other_cases_only": {"lag_samples": lag, "lag_ms_at_20hz": lag * 50},
          "nested_block_validation_mae": validation_mae,
          "held_out": score(fitted, held_out_rows),
          "training_association_coefficients": original_coefficients(fitted, feature_names(model)),
          "standardized_design_condition_number": fitted[3],
          "limits": "Coefficients are closed-loop predictive associations, not physical plant gain, damping, torque, or friction parameters.",
        }
      baseline = float(per_model["state_only"]["held_out"]["mae"])
      for model in ("delayed_command", "delayed_command_rate"):
        candidate = float(per_model[model]["held_out"]["mae"])
        per_model[model]["held_out"]["mae_change_vs_state_only_pct"] = 100.0 * (candidate - baseline) / baseline
      per_target[target] = {"outcome": outcomes[target], "models": per_model}
    results[response_name] = per_target
  return {
    "responses": results,
    "predeclared": {
      "models": MODELS,
      "lags_samples": list(LAG_SAMPLES),
      "lag_grid_assumed_hz": 20,
      "validation": "Leave one named case out. For each model and target, choose lag only from three contiguous time blocks in the remaining cases, fit on all remaining cases, then score the target once.",
      "causal_predictors": "Current/past measured response state, current speed, and current/past published TI command only. No controller request/actual fields, future lane/model values, or fused yaw as a wheel predictor are used.",
    },
  }


def separation(report: dict[str, Any]) -> dict[str, Any]:
  """Apply the frozen signature gate after outcome-blind model selection."""
  response_results = report["responses"]
  summary: dict[str, Any] = {}
  stable_all = True
  separate_all = True
  for response, cases in response_results.items():
    model_summary: dict[str, Any] = {}
    for model in ("delayed_command", "delayed_command_rate"):
      smooth = []
      problem = []
      lags = []
      gain_signs = []
      for case in cases.values():
        item = case["models"][model]
        change = float(item["held_out"]["mae_change_vs_state_only_pct"])
        (smooth if case["outcome"] == "smooth" else problem).append(change)
        lags.append(int(item["selected_on_other_cases_only"]["lag_samples"]))
        coefficient = float(item["training_association_coefficients"].get("published_ti_counts", 0.0))
        gain_signs.append(0 if abs(coefficient) < 1e-12 else (1 if coefficient > 0 else -1))
      # A useful outcome signature must have nonoverlapping >=5 point command-value ranges.
      nonoverlap = min(problem) - max(smooth) >= 5.0 or min(smooth) - max(problem) >= 5.0
      stable = max(lags) - min(lags) <= 2 and len(set(gain_signs)) == 1
      model_summary[model] = {
        "smooth_held_out_command_value_pct": smooth,
        "problem_held_out_command_value_pct": problem,
        "lag_samples": lags,
        "command_gain_signs": gain_signs,
        "nonoverlapping_outcome_signature_at_least_5_points": nonoverlap,
        "lag_and_gain_sign_stable": stable,
      }
      stable_all = stable_all and stable
      separate_all = separate_all and nonoverlap
    summary[response] = model_summary
  return {
    "gate": "For each response and both command models, published-command value must separate both smooth from both rider-problem cases by at least five held-out MAE percentage points, with leave-case-out lags within two samples and one command-gain sign. This is a deliberately strict descriptive gate, not a causal proof.",
    "all_models_all_responses_pass": stable_all and separate_all,
    "details": summary,
  }


def markdown(report: dict[str, Any]) -> str:
  lines = [
    "# Closed-loop command-response identification audit", "",
    "This is a compact cross-validated audit of **published TI command** against measured steering and fused-yaw response. Published TI is a command proxy, not exact applied actuation, motor torque, rack force, or tire force. The models are one-step predictors and do not predict lane path or rider experience.", "",
    "The only time-*i* predictors are past/current measured response state, speed, and past/current published TI. Controller request, controller actual lateral acceleration, lane/model outputs, and future response values are excluded from predictors. Outcome labels are not used to choose models or lags.", "",
    "## Held-out results", "",
    "| response | held-out case | outcome | model | selected lag | MAE | change vs state-only |", "|---|---|---|---|---:|---:|---:|",
  ]
  for response, cases in report["responses"].items():
    for case_name, case in cases.items():
      for model, item in case["models"].items():
        held = item["held_out"]
        change = "—" if model == "state_only" else f"{float(held['mae_change_vs_state_only_pct']):+.2f}%"
        lines.append(f"| {response} | {case_name} | {case['outcome']} | {model} | {item['selected_on_other_cases_only']['lag_ms_at_20hz']} ms | {float(held['mae']):.6f} | {change} |")
  lines.extend(["", "## Predeclared stability/separation test", "", report["separation"]["gate"], "", f"Result: **{report['separation']['all_models_all_responses_pass']}**.", ""])
  for response, models in report["separation"]["details"].items():
    lines.append(f"### {response}")
    lines.append("")
    for model, item in models.items():
      lines.append(f"- `{model}`: smooth values `{item['smooth_held_out_command_value_pct']}`; problem values `{item['problem_held_out_command_value_pct']}`; lags `{item['lag_samples']}`; gain signs `{item['command_gain_signs']}`; outcome separation `{item['nonoverlapping_outcome_signature_at_least_5_points']}`; stability `{item['lag_and_gain_sign_stable']}`.")
    lines.append("")
  lines.extend([
    "## Falsifiers and limits", "",
    "- If the strict gate fails, this corpus does not support claiming a stable command-to-wheel/yaw delay, gain, or phase difference that separates smooth from settling/scallop outcomes.",
    "- Passing it would still be a cross-drive predictive association. Controller era, road geometry, speed/load, tire state, and closed-loop command feedback remain confounded.",
    "- Exact previous-applied TI/stock command identity is absent in these legacy records. A current instrumented VW repeat and matched smooth control are required before using this result to motivate a controller change.",
  ])
  return "\n".join(lines) + "\n"


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--input", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--markdown", type=Path, required=True)
  args = parser.parse_args()
  if args.output.exists() or args.markdown.exists():
    raise FileExistsError("outputs already exist; choose new paths to preserve prior evidence")
  payload = load_input(args.input)
  report = run(payload)
  report.update({
    "format_version": FORMAT_VERSION,
    "input": {"path": str(args.input.resolve()), "sha256": sha256(args.input)},
    "source_sha256": {str(Path(__file__).resolve().relative_to(ROOT)).replace("\\", "/"): sha256(Path(__file__))},
  })
  report["separation"] = separation(report)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.markdown.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
  args.markdown.write_text(markdown(report), encoding="utf-8", newline="\n")
  print(json.dumps({"output": str(args.output), "output_sha256": sha256(args.output), "gate": report["separation"]["all_models_all_responses_pass"]}, sort_keys=True))


if __name__ == "__main__":
  main()
