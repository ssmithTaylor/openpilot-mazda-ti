"""Contract tests for the portable release qualification boundary."""

import json
import subprocess

import pytest

from .corpus import evaluate_corpus
from .example_fixture import create_example
from .provenance import read_json, sha256, write_json
from .release import EVIDENCE_KINDS, PREREQUISITES, PROFILE, canonical_request, main, qualify
from .scenarios import run as run_scenario
from .trace_compare import compare_trace_bundles
from .transition_eval import assess, run as run_transition


CANDIDATE = "a" * 40
OTHER = "b" * 40
DIGEST = "1" * 64


def _record(kind, candidate=CANDIDATE, status="completed_checks", version=1):
  if kind == "corpus":
    return {"format_version": version, "status": status, "profile_pass": status == "completed_checks",
            "corpus_id": "test-corpus", "corpus_version": "1", "candidate_revision": candidate,
            "profile": "command-regression", "manifest_sha256": DIGEST, "manifest_artifact_sha256": "2" * 64,
            "findings": [], "cases": [{"id": "corner", "findings": ["retained concern"]}]}
  if kind == "scenario":
    return {"format_version": version, "status": status, "case_id": "reference-transitions",
            "qualification": "synthetic_controller_limiter", "comparison": {}, "scope": "Synthetic command evidence.",
            "source_identities": {"candidate_controller": candidate}, "input_sha256": {"scenario.json": DIGEST},
            "runtime": {}, "limitations": [], "findings": []}
  if kind == "process":
    return {"format_version": 2, "profile": "full_process_transition", "status": status,
            "runtime_source": {"git_head": candidate, "identity": {"controlsd.py": DIGEST}},
            "input_sha256": {"route/rlog": DIGEST}, "source_schema_sha256": {"controlsd.py": DIGEST},
            "transition": {"process_boundary": {"interface": "selfdrive.test.process_replay.replay_process"}},
            "isolation": {}, "findings": []}
  arm = {"bundle": "bundle/candidate", "artifact_sha256": {"result.json": DIGEST},
         "identity_validation": "verified_full_bundle", "source_identities": {"candidate_controller": candidate},
         "input_sha256": {"trace.json": DIGEST}}
  return {"format_version": version, "status": status, "identity_validation": "verified_full_bundle",
          "provenance": {"reference": {**arm, "bundle": "bundle/reference"}, "candidate": arm,
                         "current_control": {**arm, "bundle": "bundle/current-control"}},
          "phases": [], "metrics": {}, "phase_effects": {}, "command_effects": {},
          "findings": {"invariants": [], "command_effects": [], "physical_observations": []}, "scope": "Recorded comparison."}


def _request(evidence, candidate=CANDIDATE):
  return {
    "format_version": 1, "candidate_revision": candidate, "profile": PROFILE,
    "physical_evaluation": {"route_segment": "route-28f:835-875", "maneuver": "left_curve", "target_speed_mps": 20,
                            "measurements": ["lane_position", "driver_steering_intervention"],
                            "maximum_driver_steering_interventions": 0},
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
  assert "route-28f:835-875" in result["physical_evaluation"]["question"]
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


def test_content_bound_comparison_cannot_qualify_for_release(tmp_path):
  request, root = _fixture(tmp_path)
  path = root / "comparison.json"
  comparison = _record("comparison")
  comparison["identity_validation"] = "content_bound_unqualified"
  _rewrite(path, comparison)
  value = read_json(request)
  value["evidence"] = [{**row, "sha256": sha256(path)} if row["kind"] == "comparison" else row
                       for row in value["evidence"]]
  _rewrite(request, value)

  result = qualify(request, root, tmp_path / "release")

  assert result["status"] == "failed_check"
  assert any("verified_full_bundle" in finding for finding in result["findings"])


def test_content_bound_reference_arm_cannot_hide_behind_a_verified_aggregate(tmp_path):
  request, root = _fixture(tmp_path)
  path = root / "comparison.json"
  comparison = _record("comparison")
  comparison["provenance"]["reference"]["identity_validation"] = "content_bound_unqualified"
  _rewrite(path, comparison)
  value = read_json(request)
  value["evidence"] = [{**row, "sha256": sha256(path)} if row["kind"] == "comparison" else row
                       for row in value["evidence"]]
  _rewrite(request, value)

  result = qualify(request, root, tmp_path / "release")

  assert result["status"] == "failed_check"
  assert any("reference arm requires verified_full_bundle" in finding for finding in result["findings"])


def test_scenario_record_relabelled_as_process_is_rejected(tmp_path):
  request, root = _fixture(tmp_path)
  path = root / "process.json"
  _rewrite(path, _record("scenario"))
  value = read_json(request)
  value["evidence"] = [{**row, "sha256": sha256(path)} if row["kind"] == "process" else row
                       for row in value["evidence"]]
  _rewrite(request, value)

  result = qualify(request, root, tmp_path / "release")

  assert result["status"] == "failed_check"
  assert any("process evidence schema" in finding for finding in result["findings"])


def test_shortened_process_interface_alias_is_rejected(tmp_path):
  request, root = _fixture(tmp_path)
  path = root / "process.json"
  process = _record("process")
  process["transition"]["process_boundary"]["interface"] = "process_replay"
  _rewrite(path, process)
  value = read_json(request)
  value["evidence"] = [{**row, "sha256": sha256(path)} if row["kind"] == "process" else row
                       for row in value["evidence"]]
  _rewrite(request, value)

  result = qualify(request, root, tmp_path / "release")

  assert result["status"] == "failed_check"
  assert any("process evidence schema" in finding for finding in result["findings"])


@pytest.mark.parametrize(("declared_kind", "actual_kind"),
                         [(declared, actual) for declared in EVIDENCE_KINDS for actual in EVIDENCE_KINDS
                          if declared != actual])
def test_every_cross_kind_evidence_substitution_is_rejected(tmp_path, declared_kind, actual_kind):
  request, root = _fixture(tmp_path)
  path = root / f"{declared_kind}.json"
  _rewrite(path, _record(actual_kind))
  value = read_json(request)
  value["evidence"] = [{**row, "sha256": sha256(path)} if row["kind"] == declared_kind else row
                       for row in value["evidence"]]
  _rewrite(request, value)

  result = qualify(request, root, tmp_path / "release")

  assert result["status"] == "failed_check"
  assert any(f"{declared_kind} evidence schema" in finding for finding in result["findings"])


def test_physical_evaluation_requires_a_concrete_structured_question():
  request = _request([])
  request["evidence"] = [{"id": kind, "kind": kind, "path": f"{kind}.json", "sha256": DIGEST}
                         for kind in EVIDENCE_KINDS]
  request["physical_evaluation"] = "Drive it?"

  with pytest.raises(ValueError, match="physical evaluation"):
    canonical_request(request)


def test_actual_producer_results_satisfy_the_release_contract(tmp_path):
  candidate = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
  root = tmp_path / "evidence"
  root.mkdir()

  raw = root / "raw"
  generated_request = create_example(raw)
  generated = read_json(generated_request)
  case = generated["case"]
  case["id"] = "release-corpus-case"
  manifest = {
    "format_version": 1, "id": "release-contract-corpus", "version": "1",
    "cases": [{"case": case, "control_revision": case["experiment"]["baseline_controller"],
               "categories": ["straight_gentle"], "group": "development",
               "exposures": ["Deterministic synthetic contract fixture"], "annotations": [],
               "expected": {"minimum_sends": 19, "stock_commands": False}, "exclusion": None}],
    "sources": [], "unavailable": {},
    "profiles": {"release": {"required_cases": ["release-corpus-case"],
                              "required_categories": ["straight_gentle"], "require_unexposed_withheld": False}},
  }
  manifest_path = root / "manifest.json"
  write_json(manifest_path, manifest)
  corpus_dir = root / "corpus"
  corpus = evaluate_corpus(manifest_path, candidate, "release", raw, root, root / "ledger.json", corpus_dir)
  assert corpus["status"] == "completed_checks", corpus

  scenario_request = root / "scenario-request.json"
  write_json(scenario_request, {
    "format_version": 1, "candidate_revision": candidate,
    "case": {"id": "release-scenario", "method": "synthetic", "origin": "synthetic",
             "scenario": "reference_transitions", "baseline_controller": candidate, "limiter": candidate},
  })
  scenario_dir = root / "scenario"
  assert run_scenario(scenario_request, scenario_dir)["status"] == "completed_checks"

  retained = root / "retained/rlog"
  retained.parent.mkdir()
  retained.write_bytes(b"release transition identity")

  def process_boundary(rlogs, _maximum, _all_segments, _transition_harness):
    transition = assess([
      {"mono_time_ns": mono, "active": active, "ti_allowed": ti, "health": health,
       "diagnostic_references": [], "state_before": state, "state_after": state,
       "state_before_id": f"opaque:{state}", "state_id": f"opaque:{state}"}
      for mono, active, ti, health, state in (
        (10, False, True, "valid", "a"), (20, True, True, "valid", "a"),
        (30, True, False, "valid", "a"), (40, True, True, "valid", "a"),
        (50, False, True, "missing", "a"), (60, False, True, "stale", "a"),
        (70, False, True, "invalid", "a"), (80, True, True, "valid", "a"),
      )
    ])
    return {**transition, "startup_boundary": {
      "interface": "selfdrive.test.process_replay.replay_process",
      "runtime_source": {"git_head": candidate, "identity": {"controlsd.py": DIGEST}},
      "input": {"rlogs": [{"label": "retained/rlog", "sha256": sha256(retained)}]},
      "no_vehicle_output": {"status": "passed"},
    }}

  process_dir = root / "process"
  process = run_transition(process_dir, rlog=[retained], all_segments=True, process_boundary=process_boundary,
                           capability=lambda: {"full_process_supported": True, "missing": []})
  assert process["status"] == "completed_checks", process

  arm = corpus_dir / "cases/release-corpus-case/candidate"
  comparison_dir = root / "comparison"
  comparison = compare_trace_bundles(arm, arm, arm, comparison_dir, data_root=raw, evidence_root=root)
  assert comparison["identity_validation"] == "verified_full_bundle"

  refs = [{"id": kind, "kind": kind, "path": f"{kind}/result.json", "sha256": sha256(root / kind / "result.json")}
          for kind in EVIDENCE_KINDS]
  release_request = root / "release-request.json"
  write_json(release_request, _request(refs, candidate))

  result = qualify(release_request, root, tmp_path / "release")

  assert result["status"] == "completed_checks", result["findings"]
  assert result["qualification"] == "qualified_for_separate_authorization"

  content_bound_dir = root / "content-bound-comparison"
  content_bound = compare_trace_bundles(arm, arm, arm, content_bound_dir, evidence_root=root)
  assert content_bound["identity_validation"] == "content_bound_unqualified"
  unqualified_refs = [{**row, "path": "content-bound-comparison/result.json",
                       "sha256": sha256(content_bound_dir / "result.json")} if row["kind"] == "comparison" else row
                      for row in refs]
  unqualified_request = root / "unqualified-request.json"
  write_json(unqualified_request, _request(unqualified_refs, candidate))

  rejected = qualify(unqualified_request, root, tmp_path / "unqualified-release")

  assert rejected["status"] == "failed_check"
  assert rejected["qualification"] == "unqualified"
