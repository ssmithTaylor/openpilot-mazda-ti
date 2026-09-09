"""The public acceptance workflow is reproducible without session-owned notes."""

import copy
from pathlib import Path

import pytest

from .fresh_fixture import create_fresh_fixture, fixture_tree_sha256
from .fresh_session import _acceptance_checks, _portable, run
from .provenance import read_json, write_json


def test_fixture_bytes_are_independent_of_location(tmp_path):
  candidate = "f60092de7b0292ad584cd880c5ff32404f9fd31d"
  first, second = tmp_path / "first", tmp_path / "relocated/second"
  create_fresh_fixture(first, candidate)
  create_fresh_fixture(second, candidate)

  assert fixture_tree_sha256(first) == fixture_tree_sha256(second)
  manifest = read_json(first / "corpus.json")
  excluded = next(row for row in manifest["cases"] if row["exclusion"])
  assert excluded["case"]["id"] == "retained-unavailable-corner"
  assert manifest["unavailable"]["matched_conditions"]


def test_complete_fresh_session_reproduces_canonical_release_and_rejects_incomplete_environment(tmp_path):
  result = run(tmp_path / "acceptance")

  assert result["status"] == "completed_checks", result
  assert all(result["checks"].values())
  assert all(result["invalidation"].values())
  assert result["component_statuses"]["process_contract_fixture"] == "completed_checks"
  assert result["component_statuses"]["process_declared_runtime"] == "unsupported"
  assert result["component_statuses"]["release_decision"] == "unqualified"
  assert result["incomplete_environment"]["process_status"] == "unsupported"
  assert result["incomplete_environment"]["release_decision"] == "unqualified"
  execution = read_json(tmp_path / "acceptance/execution.json")
  assert execution["runs"]["cold"]["batch"]["timing_kind"] == "cold"
  assert execution["runs"]["repeated"]["batch"]["timing_kind"] == "repeated"
  assert execution["cache_contribution"]["repeated_cache_hits"] == 2
  assert execution["runs"]["cold"]["worker_pid"] != execution["runs"]["repeated"]["worker_pid"]
  assert "elapsed_seconds" not in result
  assert "Full controller replay" in execution["bottlenecks"][0]
  report = (tmp_path / "acceptance/report.md").read_text(encoding="utf-8")
  assert "Every required acceptance check passed." in report
  assert all(name in report for name in result["checks"])
  assert all(digest in report for digest in result["canonical_components"].values())


@pytest.mark.parametrize("absolute", [r"C:\unrelated\secret\trace.json", "/unrelated/secret/trace.json",
                                      r"\\server\share\trace.json", r"failure at C:\unrelated\secret.py",
                                      "failure at /unrelated/secret.py"])
def test_portability_rejects_any_absolute_path_after_json_decoding(tmp_path, absolute):
  artifact = tmp_path / "canonical.json"
  write_json(artifact, {"unrelated": {"path": absolute}})

  assert _portable([artifact]) is False


def test_existing_acceptance_root_is_preserved(tmp_path):
  output = tmp_path / "existing"
  output.mkdir()
  sentinel = output / "keep.txt"
  sentinel.write_text("keep", encoding="utf-8")

  with pytest.raises(FileExistsError):
    run(output)

  assert sentinel.read_text(encoding="utf-8") == "keep"


def test_worktree_candidate_is_rejected_before_creating_output(tmp_path):
  output = tmp_path / "must-not-exist"

  with pytest.raises(ValueError, match="committed candidate"):
    run(output, "worktree")

  assert not output.exists()


def test_failed_required_component_cannot_hide_behind_expected_unqualified_release():
  statuses = {name: "completed_checks" for name in ("batch", "corpus", "scenario", "process_contract_fixture", "comparison")}
  statuses.update(release="failed_check", release_decision="unqualified")
  first = {
    "components": {"result": "same"}, "statuses": statuses, "release_findings": [],
    "execution": {"session_identity": "first", "batch": {"timing_kind": "cold", "cases": [{"reused": False}]}},
  }
  second = copy.deepcopy(first)
  first["statuses"]["process_declared_runtime"] = second["statuses"]["process_declared_runtime"] = "completed_checks"
  first["statuses"]["release"] = second["statuses"]["release"] = "completed_checks"
  first["statuses"]["release_decision"] = second["statuses"]["release_decision"] = "qualified"
  second["execution"]["session_identity"] = "second"
  second["execution"]["batch"] = {"timing_kind": "repeated", "cases": [{"reused": True}]}
  first["statuses"]["scenario"] = second["statuses"]["scenario"] = "failed_check"

  checks = _acceptance_checks(first, second, True, True, {"source": True}, {
    "process_status": "unsupported", "release_status": "failed_check", "release_decision": "unqualified",
  })

  assert checks["required_component_stages_completed"] is False
  assert all(value for name, value in checks.items() if name != "required_component_stages_completed")
