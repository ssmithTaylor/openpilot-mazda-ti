"""Contract tests for the portable release qualification boundary."""

import json

from .provenance import read_json, sha256, write_json
from .release import EVIDENCE_KINDS, PREREQUISITES, PROFILE, canonical_request, main, qualify


CANDIDATE = "a" * 40
OTHER = "b" * 40
DIGEST = "1" * 64


def _record(kind, candidate=CANDIDATE, status="completed_checks", version=1):
  if kind == "corpus":
    return {"format_version": version, "status": status, "candidate_revision": candidate,
            "manifest_sha256": DIGEST, "findings": [], "cases": [{"id": "corner", "findings": ["retained concern"]}]}
  if kind == "scenario":
    return {"format_version": version, "status": status,
            "source_identities": {"candidate_controller": candidate}, "input_sha256": {"scenario.json": DIGEST},
            "findings": []}
  if kind == "process":
    return {"format_version": 2, "status": status, "runtime_source": {"git_head": candidate},
            "input": {"rlogs": [{"label": "route/rlog", "sha256": DIGEST}]}, "findings": []}
  return {"format_version": version, "status": status,
          "provenance": {"candidate": {"source_identities": {"candidate_controller": candidate},
                                         "input_sha256": {"trace.json": DIGEST}}}, "findings": []}


def _request(evidence, candidate=CANDIDATE):
  return {
    "format_version": 1, "candidate_revision": candidate, "profile": PROFILE,
    "physical_question": "Does this candidate preserve an acceptable path through the selected difficult corner without a driver catch?",
    "settings": {"TiSteerMax": 600, "TorqueInterceptorEnabled": 1},
    "evidence": evidence,
    "deployment_prerequisites": [{"name": name, "status": "pending", "note": "Performed only at the separately authorized device boundary."}
                                  for name in PREREQUISITES],
  }


def _fixture(tmp_path, statuses=None, candidate=CANDIDATE):
  statuses = statuses or {}
  root = tmp_path / "evidence"
  root.mkdir()
  refs = []
  for kind in EVIDENCE_KINDS:
    path = root / f"{kind}.json"
    write_json(path, _record(kind, candidate, statuses.get(kind, "completed_checks")))
    refs.append({"id": kind, "kind": kind, "path": path.name, "sha256": sha256(path)})
  request = tmp_path / "request.json"
  write_json(request, _request(refs, candidate))
  return request, root


def _rewrite(path, value):
  path.write_text(json.dumps(value), encoding="utf-8")


def test_completed_release_keeps_concerns_and_pending_boundaries(tmp_path):
  request, root = _fixture(tmp_path)
  result = qualify(request, root, tmp_path / "release")
  assert result["status"] == "completed_checks"
  assert result["qualification"] == "qualified_for_separate_authorization"
  assert result["deployment"] == {"performed": False, "status": "pending"}
  assert all(row["status"] == "pending" for row in result["deployment_prerequisites"])
  assert result["evidence"][0]["concerns"] == ["retained concern"]
  assert "acceptable path" in result["physical_evaluation"]["question"]
  assert "elapsed_seconds" not in result
  assert read_json(tmp_path / "release/execution.json")["elapsed_seconds"] >= 0
  assert "pending" in (tmp_path / "release/report.md").read_text(encoding="utf-8")


def test_cli_writes_the_same_portable_record_boundary(tmp_path, capsys):
  request, root = _fixture(tmp_path)
  assert main(["--request", str(request), "--evidence-root", str(root), "--output", str(tmp_path / "release")]) == 0
  assert "completed_checks" in capsys.readouterr().out


def test_failed_integration_result_blocks_profile_and_retains_status(tmp_path):
  request, root = _fixture(tmp_path, {"process": "failed_execution"})
  result = qualify(request, root, tmp_path / "release")
  assert result["status"] == "failed_check"
  process = next(row for row in result["evidence"] if row["kind"] == "process")
  assert process["status"] == "failed_execution"
  assert "Evidence status is failed_execution" in result["findings"][0]
  assert result["qualification"] == "unqualified"


def test_missing_required_check_blocks_profile(tmp_path):
  request, root = _fixture(tmp_path)
  value = read_json(request)
  value["evidence"] = [row for row in value["evidence"] if row["kind"] != "comparison"]
  request.write_text(json.dumps(value), encoding="utf-8")
  result = qualify(request, root, tmp_path / "release")
  assert result["status"] == "failed_check"
  assert "exactly one record for each evidence kind" in result["findings"][0]


def test_mismatched_candidate_source_blocks_stale_evidence(tmp_path):
  request, root = _fixture(tmp_path)
  path = root / "scenario.json"
  path.write_text(json.dumps(_record("scenario", OTHER)), encoding="utf-8")
  value = read_json(request)
  value["evidence"] = [{**row, "sha256": sha256(path)} if row["kind"] == "scenario" else row for row in value["evidence"]]
  _rewrite(request, value)
  result = qualify(request, root, tmp_path / "release")
  assert result["status"] == "failed_check"
  assert any("Candidate source identity does not match" in finding for finding in result["findings"])


def test_changed_evidence_bytes_are_rejected(tmp_path):
  request, root = _fixture(tmp_path)
  (root / "comparison.json").write_text("{}", encoding="utf-8")
  result = qualify(request, root, tmp_path / "release")
  assert result["status"] == "failed_check"
  assert "Evidence identity changed: comparison" in result["findings"][0]


def test_unsupported_evidence_version_is_visible_and_unqualified(tmp_path):
  request, root = _fixture(tmp_path)
  path = root / "scenario.json"
  path.write_text(json.dumps(_record("scenario", version=99)), encoding="utf-8")
  value = read_json(request)
  value["evidence"] = [{**row, "sha256": sha256(path)} if row["kind"] == "scenario" else row for row in value["evidence"]]
  _rewrite(request, value)
  result = qualify(request, root, tmp_path / "release")
  row = next(row for row in result["evidence"] if row["kind"] == "scenario")
  assert result["status"] == "failed_check"
  assert row["status"] == "unsupported"
  assert "Unsupported evidence version" in result["findings"][0]


def test_evidence_path_cannot_escape_root(tmp_path):
  refs = [{"id": kind, "kind": kind, "path": "../outside.json", "sha256": DIGEST} for kind in EVIDENCE_KINDS]
  canonical_request(_request(refs))
  request, root = _fixture(tmp_path)
  value = read_json(request)
  value["evidence"][0]["path"] = "../outside.json"
  _rewrite(request, value)
  result = qualify(request, root, tmp_path / "release")
  assert result["status"] == "failed_check"
  assert "Data path escapes" in result["findings"][0]


def test_non_pending_deployment_record_is_rejected_before_evidence(tmp_path):
  request, root = _fixture(tmp_path)
  value = read_json(request)
  value["deployment_prerequisites"][0]["status"] = "completed"
  _rewrite(request, value)
  result = qualify(request, root, tmp_path / "release")
  assert result["status"] == "failed_check"
  assert "pending records" in result["findings"][0]
