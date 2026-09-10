"""Deterministic, evidence-graded lateral-event detector.

Input is a normalized JSON row stream produced from one route's rlogs.  The
extractor is intentionally separate from scoring: callers must perform the
exact carOutput/carControl/controlsState/input identity joins first.
"""
from __future__ import annotations

import argparse, hashlib, json
from pathlib import Path
from typing import Any


REQUIRED = ("mono_ns", "command", "measurement", "yaw_rate", "speed")
STEERING = ("steering_angle", "steering_rate")


def sha256(path: Path) -> str:
  h = hashlib.sha256()
  with path.open("rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""): h.update(chunk)
  return h.hexdigest()


def _finite(row: dict[str, Any], key: str) -> bool:
  value = row.get(key)
  return isinstance(value, (int, float)) and value == value and abs(float(value)) != float("inf")


def score(rows: list[dict[str, Any]], start_ns: int, end_ns: int, *, min_persistence: int = 3,
         command_drop: float = 0.15, motion_tolerance: float = 0.35) -> dict[str, Any]:
  if start_ns >= end_ns: raise ValueError("window must be increasing")
  selected = [r for r in rows if start_ns <= int(r.get("mono_ns", -1)) < end_ns]
  selected.sort(key=lambda r: int(r["mono_ns"]))
  if not selected: raise ValueError("window has no rows")
  invalid = [r for r in selected if any(not _finite(r, key) for key in REQUIRED)]
  gaps = [b["mono_ns"] - a["mono_ns"] for a, b in zip(selected, selected[1:]) if b["mono_ns"] - a["mono_ns"] > 100_000_000]
  identity_joined = all("identity_joined" in r and r["identity_joined"] is True for r in selected)
  lane_fields_present = all("lane_valid" in r for r in selected)
  health_fields_present = all("health_ok" in r for r in selected)
  events = []
  for i in range(1, len(selected)):
    a, b = selected[i - 1], selected[i]
    if any(not _finite(x, k) for x in (a, b) for k in REQUIRED): continue
    if b["mono_ns"] - a["mono_ns"] > 100_000_000: continue
    command_drop_ok = abs(b["command"]) < abs(a["command"]) * (1 - command_drop)
    motion = b["yaw_rate"] * b["speed"]
    measured = b["measurement"]
    motion_agrees = abs(motion - measured) <= motion_tolerance * max(1.0, abs(measured))
    same_direction = motion * a["command"] > 0 and measured * a["command"] > 0
    lane_ok = lane_fields_present and b["lane_valid"] is True
    health_ok = health_fields_present and b["health_ok"] is True
    steering_present = all(k in b for k in STEERING) and all(_finite(b, k) for k in STEERING)
    steering_agrees = steering_present and b["steering_angle"] * a["command"] > 0 and b["steering_rate"] * a["command"] < 0
    if command_drop_ok:
      run = selected[i:i + min_persistence]
      persistent = len(run) == min_persistence and all(_finite(x, "command") and abs(x["command"]) <= abs(a["command"]) * (1 - command_drop) for x in run)
      if persistent:
        category = "motion-supported candidate" if same_direction and motion_agrees and steering_agrees and lane_ok and health_ok and identity_joined else "command-only"
        events.append({"mono_ns": b["mono_ns"], "category": category,
                       "motion_agrees": motion_agrees, "lane_valid": lane_ok, "health_ok": health_ok,
                       "clock": {"mono_ns": b["mono_ns"], "camera_eof_ns": b.get("camera_eof_ns")}})
  usable = not invalid and not gaps and identity_joined and lane_fields_present and health_fields_present
  if invalid or gaps or not usable: category = "insufficient evidence"
  elif events and all(e["category"] == "motion-supported candidate" for e in events): category = "motion-supported candidate"
  elif events: category = "command-only"
  else: category = "insufficient evidence"
  return {"category": category, "events": events, "rows": len(selected),
          "invalid_rows": len(invalid), "gaps_over_100ms": len(gaps),
          "gates": {"identity_join_required": True, "identity_joined": identity_joined,
                    "lane_truth": False, "driver_contact": False,
                    "usable": usable},
          "scope": "recorded command/motion evidence; not surveyed lane truth or physical outcome"}


def main() -> None:
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument("--rows", type=Path, required=True); p.add_argument("--rlog", type=Path, action="append", required=True)
  p.add_argument("--start-ns", type=int, required=True); p.add_argument("--end-ns", type=int, required=True); p.add_argument("--output", type=Path, required=True)
  args = p.parse_args()
  if args.output.exists(): raise FileExistsError(args.output)
  rows = json.loads(args.rows.read_text(encoding="utf-8"))
  result = score(rows, args.start_ns, args.end_ns)
  result["inputs"] = {"rows": {"path": str(args.rows.resolve()), "sha256": sha256(args.rows)},
                       "rlogs": [{"path": str(x.resolve()), "sha256": sha256(x)} for x in args.rlog]}
  args.output.parent.mkdir(parents=True, exist_ok=True); args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__": main()
