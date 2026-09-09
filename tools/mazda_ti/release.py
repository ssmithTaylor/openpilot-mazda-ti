"""Prepare a portable Mazda release qualification record.

This module consumes retained evidence.  It does not replay a controller,
contact a device, publish CAN, reboot, or perform deployment work.
"""

import argparse
import json
import re
import time
from pathlib import Path

from .provenance import read_json, sha256, under, write_json


FORMAT_VERSION = 1
PROFILE_VERSION = 1
PROFILE = "mazda-release-v1"
EVIDENCE_KINDS = ("corpus", "scenario", "process", "comparison")
PROFILE_DEFINITION = {
  "name": PROFILE,
  "version": PROFILE_VERSION,
  "required_evidence": EVIDENCE_KINDS,
}
PREREQUISITES = (
  "remote_main_before_device",
  "change_dependent_build_and_parameter_sync",
  "settings_preservation",
  "post_reboot_running_source",
)
SCOPE = (
  "Local software and recorded-motion qualification only. A completed release "
  "record does not establish an acceptable corner path, driver contact, device "
  "readiness, or safe deployment."
)


class Unsupported(ValueError):
  """The release request or evidence version is not supported."""


def _fields(value, required, optional=()):
  if not isinstance(value, dict) or not set(required) <= set(value) or set(value) - set(required) - set(optional):
    raise ValueError("Release request contains missing or unexpected fields")
  return {key: value[key] for key in required}


def _commit(value):
  return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{40}", value) is not None


def _text(value, name, maximum=2000):
  if not isinstance(value, str) or not value.strip() or len(value) > maximum or any(ord(char) < 32 for char in value):
    raise ValueError(f"Invalid {name}")
  return value


def canonical_request(value):
  """Validate the small, portable release request before reading evidence."""
  request = _fields(value, ("format_version", "candidate_revision", "profile", "physical_question",
                             "settings", "evidence", "deployment_prerequisites"))
  if type(request["format_version"]) is not int or request["format_version"] != FORMAT_VERSION:
    raise Unsupported("Unsupported release request version")
  if not _commit(request["candidate_revision"]):
    raise ValueError("candidate_revision must be a full 40-character Git revision")
  if request["profile"] != PROFILE:
    raise Unsupported("Unsupported release qualification profile")
  _text(request["physical_question"], "physical evaluation question")
  if not isinstance(request["settings"], dict) or not request["settings"]:
    raise ValueError("Release settings must be a nonempty object")
  for name, value in request["settings"].items():
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,79}", name):
      raise ValueError("Invalid release setting name")
    if type(value) not in (int, float) or value != value or value in (float("inf"), float("-inf")):
      raise ValueError("Release settings must be finite numbers")
  if not isinstance(request["evidence"], list) or len(request["evidence"]) != len(EVIDENCE_KINDS):
    raise ValueError("Release profile requires exactly one record for each evidence kind")
  ids, kinds = set(), set()
  for item in request["evidence"]:
    evidence = _fields(item, ("id", "kind", "path", "sha256"))
    if (not isinstance(evidence["id"], str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", evidence["id"])
        or evidence["id"] in ids):
      raise ValueError("Evidence identities must be unique")
    if evidence["kind"] not in EVIDENCE_KINDS or evidence["kind"] in kinds:
      raise ValueError("Release profile requires one unique record for each evidence kind")
    if not isinstance(evidence["path"], str) or not evidence["path"]:
      raise ValueError("Evidence paths are required")
    if not isinstance(evidence["sha256"], str) or re.fullmatch(r"[a-f0-9]{64}", evidence["sha256"]) is None:
      raise ValueError("Invalid evidence artifact hash")
    ids.add(evidence["id"])
    kinds.add(evidence["kind"])
  if kinds != set(EVIDENCE_KINDS):
    raise ValueError("Release profile is missing a mandatory evidence kind")
  prereqs = request["deployment_prerequisites"]
  if not isinstance(prereqs, list) or len(prereqs) != len(PREREQUISITES):
    raise ValueError("All deployment prerequisites must be recorded")
  names = set()
  for item in prereqs:
    row = _fields(item, ("name", "status", "note"))
    if row["name"] not in PREREQUISITES or row["name"] in names or row["status"] != "pending":
      raise ValueError("Deployment prerequisites must remain pending records")
    _text(row["note"], "deployment prerequisite note")
    names.add(row["name"])
  if names != set(PREREQUISITES):
    raise ValueError("Deployment prerequisite coverage is incomplete")
  request["evidence"] = [_fields(item, ("id", "kind", "path", "sha256")) for item in request["evidence"]]
  request["deployment_prerequisites"] = [_fields(item, ("name", "status", "note")) for item in prereqs]
  request["settings"] = {key: request["settings"][key] for key in sorted(request["settings"])}
  return request


def _walk(value):
  if isinstance(value, dict):
    yield value
    for child in value.values():
      yield from _walk(child)
  elif isinstance(value, list):
    for child in value:
      yield from _walk(child)


def _candidate_identities(record, kind):
  identities = set()
  if _commit(record.get("candidate_revision")):
    identities.add(record["candidate_revision"])
  for obj in _walk(record):
    for key in ("candidate", "candidate_controller", "candidate_revision"):
      value = obj.get(key) if isinstance(obj, dict) else None
      if _commit(value):
        identities.add(value)
    if kind == "process" and _commit(obj.get("git_head")):
      identities.add(obj["git_head"])
  return sorted(identities)


def _input_identities(record):
  identities = {}
  for obj in _walk(record):
    declared = obj.get("input_sha256") if isinstance(obj, dict) else None
    if isinstance(declared, dict):
      for name, digest in declared.items():
        if isinstance(name, str) and isinstance(digest, str) and re.fullmatch(r"[a-f0-9]{64}", digest):
          identities[name] = digest
    rlogs = obj.get("rlogs") if isinstance(obj, dict) else None
    if isinstance(rlogs, list):
      for row in rlogs:
        if isinstance(row, dict) and isinstance(row.get("label"), str) and re.fullmatch(r"[a-f0-9]{64}", row.get("sha256", "")):
          identities[row["label"]] = row["sha256"]
    manifest = obj.get("manifest_sha256") if isinstance(obj, dict) else None
    if isinstance(manifest, str) and re.fullmatch(r"[a-f0-9]{64}", manifest):
      identities["manifest.json"] = manifest
  return dict(sorted(identities.items()))


def _concerns(record):
  concerns = []
  findings = record.get("findings")
  if isinstance(findings, list):
    concerns.extend(str(item) for item in findings)
  elif findings:
    concerns.append(json.dumps(findings, sort_keys=True, separators=(",", ":")))
  for case in record.get("cases", []):
    if isinstance(case, dict):
      concerns.extend(str(item) for item in case.get("findings", []) if item)
  return concerns


def _load_evidence(item, root, candidate):
  path = under(root, item["path"])
  if not path.is_file():
    raise FileNotFoundError(f"Missing evidence record: {item['id']}")
  if sha256(path) != item["sha256"]:
    raise ValueError(f"Evidence identity changed: {item['id']}")
  record = read_json(path)
  if record.get("format_version") not in (1, 2):
    raise Unsupported(f"Unsupported evidence version: {item['id']}")
  status = record.get("status")
  source = _candidate_identities(record, item["kind"])
  inputs = _input_identities(record)
  blockers = []
  if status != "completed_checks":
    blockers.append(f"Evidence status is {status or 'missing'}")
  if source != [candidate]:
    blockers.append("Candidate source identity does not match the release candidate")
  if not inputs:
    blockers.append("Evidence lacks input identity")
  return {
    "id": item["id"], "kind": item["kind"], "path": item["path"], "artifact_sha256": item["sha256"],
    "status": status or "missing", "source_identities": source, "input_sha256": inputs,
    "concerns": _concerns(record), "blockers": blockers,
  }


def _report(result):
  lines = [f"# Mazda release qualification: {result['candidate_revision']}", "", f"Status: **{result['status']}**.", "", SCOPE, "", "## Evidence", "", "| Kind | Status | Concerns |", "| --- | --- | --- |"]
  for row in result["evidence"]:
    lines.append(f"| {row['kind']} | {row['status']} | {len(row['concerns'])} |")
  lines += ["", "## Findings", ""]
  lines.extend(f"- {finding}" for finding in result["findings"] or ["None"])
  lines += ["", "## Proposed physical evaluation", "", result["physical_evaluation"]["question"], "", "Status: **pending separate authorization**.", "", "## Deployment prerequisites", ""]
  lines.extend(f"- {row['name']}: pending — {row['note']}" for row in result["deployment_prerequisites"])
  return "\n".join(lines) + "\n"


def qualify(request_path, evidence_root, output):
  """Create a new release record from already-retained evidence."""
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  started = time.perf_counter()
  result = {
    "format_version": FORMAT_VERSION, "profile": PROFILE_DEFINITION,
    "status": "failed_execution", "qualification": "unqualified", "candidate_revision": None,
    "evidence": [], "findings": [], "settings": {}, "physical_evaluation": {"status": "pending", "question": ""},
    "deployment_prerequisites": [], "deployment": {"performed": False, "status": "pending"}, "scope": SCOPE,
  }
  try:
    request = canonical_request(read_json(Path(request_path)))
    result.update(candidate_revision=request["candidate_revision"], settings=request["settings"],
                  physical_evaluation={"status": "pending", "question": request["physical_question"]},
                  deployment_prerequisites=request["deployment_prerequisites"])
    for item in request["evidence"]:
      try:
        row = _load_evidence(item, Path(evidence_root), request["candidate_revision"])
      except (FileNotFoundError, Unsupported, ValueError, OSError) as error:
        row = {"id": item["id"], "kind": item["kind"], "path": item["path"], "artifact_sha256": item["sha256"],
               "status": "unsupported" if isinstance(error, Unsupported) else "failed_check", "source_identities": [],
               "input_sha256": {}, "concerns": [], "blockers": [str(error)]}
      result["evidence"].append(row)
      result["findings"].extend(f"{row['id']}: {item}" for item in row["blockers"])
    result["status"] = "completed_checks" if not result["findings"] else "failed_check"
    result["qualification"] = "qualified_for_separate_authorization" if result["status"] == "completed_checks" else "unqualified"
  except Unsupported as error:
    result["status"], result["findings"] = "unsupported", [str(error)]
  except (FileNotFoundError, OSError) as error:
    result["findings"] = [str(error)]
  except (ValueError, KeyError, TypeError) as error:
    result["status"], result["findings"] = "failed_check", [str(error)]
  execution = {"elapsed_seconds": time.perf_counter() - started}
  write_json(output / "result.json", result)
  (output / "report.md").write_text(_report(result), encoding="utf-8", newline="\n")
  write_json(output / "execution.json", execution)
  return result


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--request", type=Path, required=True)
  parser.add_argument("--evidence-root", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  args = parser.parse_args(argv)
  result = qualify(args.request, args.evidence_root, args.output)
  print(f"{result['status']}: {args.output / 'report.md'}")
  return 0 if result["status"] == "completed_checks" else 1


if __name__ == "__main__":
  raise SystemExit(main())
