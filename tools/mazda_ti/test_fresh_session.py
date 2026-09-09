"""The public acceptance workflow is reproducible without session-owned notes."""

from pathlib import Path

import pytest

from .fresh_fixture import create_fresh_fixture, fixture_tree_sha256
from .fresh_session import _portable, run
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
  assert result["component_statuses"]["release_decision"] == "unqualified"
  assert result["incomplete_environment"]["process_status"] == "unsupported"
  assert result["incomplete_environment"]["release_decision"] == "unqualified"
  execution = read_json(tmp_path / "acceptance/execution.json")
  assert execution["runs"]["cold"]["batch"]["timing_kind"] == "cold"
  assert execution["runs"]["repeated"]["batch"]["timing_kind"] == "repeated"
  assert execution["cache_contribution"]["repeated_cache_hits"] == 2
  assert "elapsed_seconds" not in result
  assert "Full controller replay" in execution["bottlenecks"][0]


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
