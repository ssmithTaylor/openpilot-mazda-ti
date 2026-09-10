"""Deterministically map logged roadEncodeIdx rows to generated clip PTS.

This module deliberately does not decode video or read rlogs.  A producer may
extract ``fullHEVC`` rows with the repository's cereal reader and pass them as
JSON.  Keeping extraction separate makes the mapping small, testable, and
auditable for both SOF and EOF camera clocks.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable


def sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def validate_rows(rows: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
  result = sorted((dict(row) for row in rows), key=lambda row: int(row["segment_id"]))
  if not result:
    raise ValueError("roadEncodeIdx rows are empty")
  expected = list(range(int(result[0]["segment_id"]), int(result[0]["segment_id"]) + len(result)))
  actual = [int(row["segment_id"]) for row in result]
  if actual != expected:
    raise ValueError("roadEncodeIdx segment_id values are not contiguous")
  for row in result:
    for key in ("frame_id", "encode_id", "timestamp_sof_ns", "timestamp_eof_ns"):
      if key not in row:
        raise ValueError(f"roadEncodeIdx row missing {key}")
  frames = [int(row["frame_id"]) for row in result]
  if frames != list(range(frames[0], frames[0] + len(frames))):
    raise ValueError("frame_id values are not contiguous decoded-frame ordinals")
  return result


def map_target(rows: list[dict[str, Any]], target_s: float, fps: float, start_frame: int) -> dict[str, Any]:
  if fps <= 0:
    raise ValueError("fps must be positive")
  if target_s < int(rows[0]["timestamp_eof_ns"]) / 1e9 or target_s > int(rows[-1]["timestamp_eof_ns"]) / 1e9:
    raise ValueError("target is outside roadEncodeIdx coverage")
  candidates = [(row, "before") for row in rows if int(row["timestamp_eof_ns"]) <= round(target_s * 1e9)]
  after = [(row, "after") for row in rows if int(row["timestamp_eof_ns"]) >= round(target_s * 1e9)]
  before = max(candidates, key=lambda item: int(item[0]["timestamp_eof_ns"]), default=(None, "before"))[0]
  after_row = min(after, key=lambda item: int(item[0]["timestamp_eof_ns"]), default=(None, "after"))[0]
  if before is None and after_row is None:
    raise ValueError("target is outside roadEncodeIdx coverage")
  chosen = min((row for row in (before, after_row) if row is not None),
               key=lambda row: abs(int(row["timestamp_eof_ns"]) / 1e9 - target_s))

  def bracket(row: dict[str, Any] | None) -> dict[str, Any] | None:
    if row is None:
      return None
    frame = int(row["frame_id"])
    return {**row,
            "delta_from_target_ms": (int(row["timestamp_eof_ns"]) / 1e9 - target_s) * 1000,
            "generated_clip_pts_s": (frame - start_frame) / fps}

  return {"target_mono_s": target_s, "target_mono_ns": round(target_s * 1e9),
          "before": bracket(before), "after": bracket(after_row), "chosen": bracket(chosen)}


def build_mapping(rows: Iterable[dict[str, Any]], targets: dict[str, float], fps: float = 20.0,
                  start_frame: int | None = None, decoded_frame_count: int | None = None) -> dict[str, Any]:
  checked = validate_rows(rows)
  if decoded_frame_count is not None and decoded_frame_count != len(checked):
    raise ValueError("decoded frame count does not match roadEncodeIdx row count")
  first = int(checked[0]["frame_id"]) if start_frame is None else start_frame
  if first < int(checked[0]["frame_id"]) or first > int(checked[-1]["frame_id"]):
    raise ValueError("start_frame is outside roadEncodeIdx coverage")
  return {"format_version": 1, "fps": fps, "row_count": len(checked),
          "source_frame_start": first, "source_frame_end_inclusive": int(checked[-1]["frame_id"]),
          "events": {name: map_target(checked, value, fps, first) for name, value in targets.items()},
          "scope": "camera SOF/EOF to generated clip PTS mapping; no physical or lane interpretation"}


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--road-encode-json", type=Path, required=True)
  parser.add_argument("--video", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--fps", type=float, default=20.0)
  parser.add_argument("--decoded-frame-count", type=int)
  parser.add_argument("--target", action="append", required=True, metavar="NAME=MONO_SECONDS")
  parser.add_argument("--enhancement", default=None, help="record-only transformation description")
  args = parser.parse_args()
  if args.output.exists():
    raise FileExistsError(args.output)
  rows = json.loads(args.road_encode_json.read_text(encoding="utf-8"))
  targets = {}
  for item in args.target:
    name, sep, value = item.partition("=")
    if not sep or not name or name in targets:
      raise ValueError(f"invalid target {item!r}")
    targets[name] = float(value)
  result = build_mapping(rows, targets, args.fps, decoded_frame_count=args.decoded_frame_count)
  result["inputs"] = {"road_encode_json": {"path": str(args.road_encode_json.resolve()), "sha256": sha256(args.road_encode_json)},
                       "video": {"path": str(args.video.resolve()), "sha256": sha256(args.video), "size_bytes": args.video.stat().st_size}}
  result["transformation"] = {"applied": args.enhancement is not None, "description": args.enhancement}
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
  main()
