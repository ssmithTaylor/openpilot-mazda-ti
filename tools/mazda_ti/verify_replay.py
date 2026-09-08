# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Verify replay equivalence and source identity; no vehicle or network operations.

Uses only the Python standard library. Input prefixes identify .jsonl control
traces and -sends.json integer-command traces. The source lock is the result
record emitted by the integration run, containing production_sha256.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

if __package__:
  from .provenance import REPLAY_SOURCE_FILES
else:
  from provenance import REPLAY_SOURCE_FILES

FIELDS = (
  "replay_output",
  "replay_i",
  "replay_p",
  "replay_d",
  "replay_f",
  "ff_lat_accel",
  "tracked_setpoint",
  "lane_removed",
  "lane_valid",
  "lane_reason",
  "lane_position_permitted",
  "lane_permitted",
  "lane_heading10",
  "lane_heading20",
  "lane_curvature10",
  "lane_curvature20",
)
REQUIRED_SOURCES = {
  "selfdrive/car/mazda/lateral_reference.py",
  "selfdrive/controls/lib/latcontrol_torque.py",
  "selfdrive/controls/controlsd.py",
}


class EvidenceError(ValueError):
  pass


def digest(path):
  value = hashlib.sha256()
  with path.open("rb") as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b""):
      value.update(block)
  return value.hexdigest()


def reject_constant(value):
  raise EvidenceError(f"Non-finite JSON value: {value}")


def read_json(path):
  return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)


def source_identity(root, lock_path):
  lock = read_json(lock_path)
  version2 = lock.get('format_version') == 2
  sources = lock.get('repository_sources', {}) if version2 else lock.get("production_sha256", {})
  required = REPLAY_SOURCE_FILES if version2 else REQUIRED_SOURCES
  if not required.issubset(sources):
    raise EvidenceError("Source lock must include controller, lane helper and controlsd hashes from the run")
  root = root.resolve()
  for name, expected in sorted(sources.items()):
    path = (root / name).resolve()
    if Path(name).is_absolute() or not path.is_relative_to(root):
      raise EvidenceError(f"Source path escapes repository: {name}")
    if digest(path) != expected:
      raise EvidenceError(f"Source changed since the integration run: {name}")
  return sources


def controls(path, start, end):
  previous = None
  with path.open(encoding="utf-8") as stream:
    for line in stream:
      row = json.loads(line, parse_constant=reject_constant)
      mono = row["mono"]
      if not isinstance(mono, (int, float)) or not math.isfinite(mono):
        raise EvidenceError("Control timestamp must be finite")
      if previous is not None and mono <= previous:
        raise EvidenceError("Control timestamps must increase strictly")
      previous = mono
      if start <= mono <= end and row["active"]:
        yield row


def compare_controls(baseline, candidate, start, end, additional_flag):
  from itertools import zip_longest

  count = 0
  for original, changed in zip_longest(controls(baseline, start, end), controls(candidate, start, end)):
    if original is None or changed is None:
      raise EvidenceError("Different active control-frame coverage")
    if original["mono"] != changed["mono"]:
      raise EvidenceError("Control-frame identities differ")
    for name in FIELDS:
      if name not in original or name not in changed or original[name] != changed[name]:
        raise EvidenceError(f"{name} differs at mono {original['mono']}")
    expected = int(original["replay_plantState"])
    if expected & additional_flag:
      raise EvidenceError("Additional flag already occurs in baseline; choose the correct comparison")
    if max(changed["lane_removed"]) > 1e-9:
      expected |= additional_flag
    if changed["replay_plantState"] != expected:
      raise EvidenceError(f"Plant-state flags differ at mono {original['mono']}")
    count += 1
  if count == 0:
    raise EvidenceError("No active control frames in requested window")
  return count


def compare_sends(baseline, candidate, start, end):
  def selected(path):
    previous = None
    result = []
    for row in read_json(path)["sends"]:
      mono = row["mono"]
      if not isinstance(mono, int) or isinstance(mono, bool) or (previous is not None and mono <= previous):
        raise EvidenceError("Send timestamps must be strictly increasing integer nanoseconds")
      previous = mono
      if round(start * 1e9) <= mono <= round(end * 1e9):
        if not isinstance(row['counts'], int) or isinstance(row['counts'], bool) or abs(row["counts"]) > 600:
          raise EvidenceError("Invalid or out-of-envelope integer TI send")
        result.append(row)
    return result

  original, changed = selected(baseline), selected(candidate)
  if not original or original != changed:
    raise EvidenceError("Integer sends or their coverage differ")
  if any(abs(b["counts"] - a["counts"]) > 15 for a, b in zip(changed, changed[1:], strict=False)):
    raise EvidenceError("Adjacent TI sends exceed the recorded 15-count limit")
  return len(changed)


def verify(root, baseline, candidate, source_lock, start, end, additional_flag=0):
  if not math.isfinite(start) or not math.isfinite(end) or start >= end:
    raise EvidenceError("Use a finite, non-empty monotonic-time window")
  if additional_flag not in (0, 4096):
    raise EvidenceError("Only the declared lane-release log flag (4096) can be added")
  sources = source_identity(root, source_lock)
  paths = {
    "baseline_controls": Path(str(baseline) + ".jsonl"),
    "candidate_controls": Path(str(candidate) + ".jsonl"),
    "baseline_sends": Path(str(baseline) + "-sends.json"),
    "candidate_sends": Path(str(candidate) + "-sends.json"),
    "source_lock": source_lock,
  }
  lock = read_json(source_lock)
  if lock.get('format_version') == 2:
    expected = lock.get('output_sha256', {})
    if set(expected) != {'trace.json', 'trace.jsonl', 'trace-sends.json'}:
      raise EvidenceError('Incomplete replay artifact manifest')
    for suffix in ('.json', '.jsonl', '-sends.json'):
      if digest(Path(str(candidate) + suffix)) != expected['trace' + suffix]:
        raise EvidenceError(f'Candidate artifact changed since replay: {suffix}')
  frames = compare_controls(paths["baseline_controls"], paths["candidate_controls"], start, end, additional_flag)
  sends = compare_sends(paths["baseline_sends"], paths["candidate_sends"], start, end)
  return {
    "format_version": 1,
    "status": "equivalent",
    "window_seconds": [start, end],
    "control_frames": frames,
    "sends": sends,
    "additional_log_flag": additional_flag,
    "production_sha256": sources,
    "artifact_sha256": {name: digest(path) for name, path in sorted(paths.items())},
    "scope": "Source identity and replay equivalence only; not vehicle-path validation or a deployment authorization.",
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
  parser.add_argument("--baseline", type=Path, required=True, help="Baseline trace prefix, without extension")
  parser.add_argument("--candidate", type=Path, required=True, help="Candidate trace prefix, without extension")
  parser.add_argument("--source-lock", type=Path, required=True, help="Result record captured by the integration run")
  parser.add_argument("--start", type=float, required=True, help="Monotonic seconds, inclusive")
  parser.add_argument("--end", type=float, required=True, help="Monotonic seconds, inclusive")
  parser.add_argument("--additional-log-flag", type=int, choices=(0, 4096), default=0)
  parser.add_argument("--output", type=Path, help="New deterministic JSON report; existing files are preserved")
  args = parser.parse_args()
  try:
    report = verify(args.repo, args.baseline, args.candidate, args.source_lock, args.start, args.end, args.additional_log_flag)
    encoded = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
      with args.output.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(encoded)
    print(encoded, end="")
  except (EvidenceError, OSError, KeyError, TypeError, json.JSONDecodeError) as error:
    print(f"Evidence verification failed: {error}", file=sys.stderr)
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
