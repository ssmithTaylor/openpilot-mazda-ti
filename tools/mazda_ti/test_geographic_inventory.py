"""Contract tests for the compact NPZ geographic inventory."""

import hashlib
import json
import os
from pathlib import Path

import numpy as np
import pytest

from . import geographic_inventory as subject


def request():
  return {
    "format_version": 1,
    "case_id": "straight-window",
    "reference": {
      "extract": {"path": "reference.npz", "sha256": "a" * 64},
      "rlogs": [{"path": "reference--0/rlog", "sha256": "a" * 64}],
      "start_mono": 0.1,
      "end_mono": 0.2,
      "tolerance_m": 20.0,
    },
  }


def write_extract(path, *, offset=0.0, mono_offset=0.0, outcome=None):
  path.parent.mkdir(parents=True, exist_ok=True)
  mono = np.asarray([0.0, 0.1, 0.15, 0.2, 0.25], dtype=np.float64) + mono_offset
  lat = np.asarray([35.0, 35.0002, 35.0005, 35.0008, 35.0012], dtype=np.float32)
  lon = np.full(5, -84.0 + offset, dtype=np.float32)
  lane = np.zeros(5, dtype=np.float32)
  if outcome is not None and outcome[0] == "bad":
    lane[:] = 1000
  data = np.column_stack((lat, lon, lane)).astype(np.float32)
  np.savez(path, data=data, columns=np.asarray(["lat", "lon", "lane"]), mono=mono)


def bound_request(root):
  reference = root / "reference.npz"
  if not reference.exists():
    write_extract(reference)
  value = request()
  value["reference"]["extract"]["sha256"] = hashlib.sha256(reference.read_bytes()).hexdigest()
  return value


def test_inventory_is_compact_deterministic_and_outcome_independent(tmp_path):
  write_extract(tmp_path / "miss.npz", offset=0.01, outcome=["bad"])
  write_extract(tmp_path / "hit.npz", outcome=["intervention"])
  first = subject.build_inventory(bound_request(tmp_path), tmp_path)
  write_extract(tmp_path / "miss.npz", offset=0.01, outcome=["excellent"])
  second = subject.build_inventory(bound_request(tmp_path), tmp_path)

  assert first["qualification"] == second["qualification"] == "coarse_candidate_only"
  assert first["searched_file_count"] == second["searched_file_count"] == 3
  assert first["hit_file_count"] == second["hit_file_count"] == 1
  assert [hit["path"] for hit in first["hits"]] == [hit["path"] for hit in second["hits"]] == ["hit.npz"]
  assert first["hits"][0]["traversals"] == second["hits"][0]["traversals"]
  assert first["hits"][0]["fragment"] is True
  assert first["manifest_sha256"] != second["manifest_sha256"]
  assert "miss.npz" not in json.dumps(first["hits"])
  assert first["raw_float64_verification"]["required"] is True
  assert first["candidate_raw_rlog_provenance"]["status"] == "unverified_not_supplied"


def test_manifest_binds_all_files_but_exposes_identity_only_for_hits(tmp_path):
  write_extract(tmp_path / "a.npz")
  write_extract(tmp_path / "nested" / "b.npz", offset=0.01)
  result = subject.build_inventory(bound_request(tmp_path), tmp_path)
  rows = []
  for name in ("a.npz", "nested/b.npz", "reference.npz"):
    path = tmp_path / name
    rows.append({"path": name, "size_bytes": path.stat().st_size, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
  encoded = b"".join((json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n").encode() for row in rows)
  assert result["manifest_sha256"] == hashlib.sha256(encoded).hexdigest()
  assert [hit["path"] for hit in result["hits"]] == ["a.npz"]


def test_candidate_mono_range_is_not_compared_to_reference_window(tmp_path):
  write_extract(tmp_path / "different-clock.npz", mono_offset=1000.0)
  result = subject.build_inventory(bound_request(tmp_path), tmp_path)
  assert result["hit_file_count"] == 1
  assert result["hits"][0]["traversals"][0]["mono"] == [1000.1, 1000.2]


def test_schema_precision_and_ambiguous_location_names_are_rejected(tmp_path):
  path = tmp_path / "bad.npz"
  write_extract(tmp_path / "reference.npz")
  np.savez(path, data=np.ones((2, 2), dtype=np.float64), columns=np.asarray(["lat", "lon"]), mono=np.arange(2, dtype=np.float64))
  with pytest.raises(ValueError, match="float32"):
    subject.build_inventory(bound_request(tmp_path), tmp_path)
  np.savez(path, data=np.ones((2, 3), dtype=np.float32), columns=np.asarray(["lat", "latitude", "lon"]),
           mono=np.arange(2, dtype=np.float64))
  with pytest.raises(ValueError, match="Ambiguous"):
    subject.build_inventory(bound_request(tmp_path), tmp_path)


def test_reference_source_is_derived_and_hash_mismatch_is_rejected(tmp_path):
  write_extract(tmp_path / "reference.npz")
  value = request()
  value["reference"]["extract"]["sha256"] = hashlib.sha256((tmp_path / "reference.npz").read_bytes()).hexdigest()
  result = subject.build_inventory(value, tmp_path)
  assert result["reference"]["extract"]["path"] == "reference.npz"
  assert result["hits"] == []
  value["reference"]["extract"]["sha256"] = "b" * 64
  with pytest.raises(ValueError, match="SHA-256"):
    subject.build_inventory(value, tmp_path)


def test_candidate_discovery_uses_verified_reference_extract_geometry(tmp_path):
  write_extract(tmp_path / "candidate.npz")
  value = bound_request(tmp_path)
  first = subject.build_inventory(value, tmp_path)
  assert [hit["path"] for hit in first["hits"]] == ["candidate.npz"]
  write_extract(tmp_path / "reference.npz", offset=0.01)
  value["reference"]["extract"]["sha256"] = hashlib.sha256((tmp_path / "reference.npz").read_bytes()).hexdigest()
  second = subject.build_inventory(value, tmp_path)
  assert second["hits"] == []


def test_reference_sample_hold_coordinates_are_collapsed(tmp_path):
  write_extract(tmp_path / "candidate.npz")
  reference = tmp_path / "reference.npz"
  write_extract(reference)
  data = np.asarray([
    [35.0, -84.0, 0.0],
    [35.0002, -84.0, 0.0],
    [35.0002, -84.0, 0.0],
    [35.0008, -84.0, 0.0],
    [35.0008, -84.0, 0.0],
  ], dtype=np.float32)
  np.savez(reference, data=data, columns=np.asarray(["lat", "lon", "lane"]),
           mono=np.asarray([0.0, 0.1, 0.15, 0.2, 0.25], dtype=np.float64))
  result = subject.build_inventory(bound_request(tmp_path), tmp_path)
  assert [hit["path"] for hit in result["hits"]] == ["candidate.npz"]


def test_partial_nonfinite_coordinate_rows_are_counted_on_hit(tmp_path):
  candidate = tmp_path / "candidate.npz"
  write_extract(candidate)
  with np.load(candidate) as archive:
    data = archive["data"].copy()
    columns = archive["columns"].copy()
    mono = archive["mono"].copy()
  data[2, 0] = np.nan
  np.savez(candidate, data=data, columns=columns, mono=mono)
  result = subject.build_inventory(bound_request(tmp_path), tmp_path)
  assert result["partial_file_count"] == 1
  assert result["dropped_coordinate_rows"] == 1
  assert result["hits"][0]["dropped_coordinate_rows"] == 1
  assert result["unscannable_files"] == []


def test_all_nonfinite_coordinate_rows_are_explicitly_unscannable(tmp_path):
  candidate = tmp_path / "empty-coordinates.npz"
  write_extract(candidate)
  with np.load(candidate) as archive:
    mono = archive["mono"].copy()
    columns = archive["columns"].copy()
  np.savez(candidate, data=np.full((len(mono), 3), np.nan, dtype=np.float32),
           columns=columns, mono=mono)
  result = subject.build_inventory(bound_request(tmp_path), tmp_path)
  assert result["hit_file_count"] == 0
  assert result["partial_file_count"] == 0
  assert result["unscannable_file_count"] == 1
  excluded = result["unscannable_files"][0]
  assert excluded["path"] == "empty-coordinates.npz"
  assert excluded["reason"] == "no_finite_coordinate_rows"
  assert excluded["dropped_coordinate_rows"] == len(mono)


def test_byte_string_columns_are_decoded(tmp_path):
  path = tmp_path / "bytes.npz"
  write_extract(path)
  with np.load(path) as archive:
    data, mono = archive["data"].copy(), archive["mono"].copy()
  np.savez(path, data=data, columns=np.asarray([b"lat", b"lon", b"lane"]), mono=mono)
  lat, lon, mono, valid = subject._load_location(path)
  assert len(lat) == len(lon) == len(mono) == len(valid) == 5


def test_reference_requires_boundaries_and_small_max_gap(tmp_path):
  write_extract(tmp_path / "candidate.npz")
  reference = tmp_path / "reference.npz"
  write_extract(reference)
  with np.load(reference) as archive:
    data, columns = archive["data"], archive["columns"]
  np.savez(reference, data=data, columns=columns,
           mono=np.asarray([0.0, 0.1, 0.5, 1.0, 1.1], dtype=np.float64))
  with pytest.raises(ValueError, match="gap"):
    subject.build_inventory(bound_request(tmp_path), tmp_path)
  write_extract(reference)
  with np.load(reference) as archive:
    data, columns = archive["data"], archive["columns"]
  np.savez(reference, data=data, columns=columns,
           mono=np.asarray([0.11, 0.12, 0.15, 0.2, 0.25], dtype=np.float64))
  with pytest.raises(ValueError, match="boundary"):
    subject.build_inventory(bound_request(tmp_path), tmp_path)


@pytest.mark.skipif(os.name == "nt", reason="Symlink creation requires host privilege on Windows")
def test_final_symlinked_extract_root_is_allowed_and_recorded(tmp_path):
  source = tmp_path / "source"
  source.mkdir()
  write_extract(source / "file.npz")
  link = tmp_path / "link"
  link.symlink_to(source, target_is_directory=True)
  result = subject.build_inventory(bound_request(source), link)
  assert result["caller_spelled_root"] == str(link)
  assert result["root_link_type"] == "symlink"
  assert result["root_is_reparse"] is False
  assert result["resolved_root"] == str(source.resolve())
  assert result["searched_file_count"] == 2


def test_final_reparse_root_is_allowed_and_recorded(tmp_path, monkeypatch):
  root = tmp_path / "mock-junction"
  root.mkdir()
  write_extract(root / "file.npz")
  monkeypatch.setattr(subject, "_is_reparse", lambda path: Path(path) == root)
  result = subject.build_inventory(bound_request(root), root)
  assert result["caller_spelled_root"] == str(root)
  assert result["root_link_type"] == "reparse_point"
  assert result["root_is_reparse"] is True
  assert result["resolved_root"] == str(root.resolve())


def test_reparse_ancestor_is_rejected_before_resolution(tmp_path, monkeypatch):
  ancestor = tmp_path / "mock-junction"
  root = ancestor / "extracts"
  root.mkdir(parents=True)
  write_extract(root / "file.npz")
  monkeypatch.setattr(subject, "_is_reparse", lambda path: Path(path) == ancestor)
  with pytest.raises(ValueError, match="symlinks and reparse points"):
    subject.build_inventory(request(), root)


def test_lexical_parent_bypass_is_rejected_before_reparse_resolution(tmp_path, monkeypatch):
  ancestor = tmp_path / "mock-junction"
  outside = tmp_path / "outside"
  ancestor.mkdir()
  outside.mkdir()
  bypass = os.path.join(str(ancestor), "..", "outside")
  monkeypatch.setattr(subject, "_is_reparse", lambda path: Path(path) == ancestor)
  with pytest.raises(ValueError, match="lexical"):
    subject.build_inventory(request(), bypass)


def test_nested_reparse_directory_is_rejected(tmp_path, monkeypatch):
  root = tmp_path / "root"
  nested = root / "nested"
  nested.mkdir(parents=True)
  write_extract(root / "reference.npz")
  monkeypatch.setattr(subject, "_is_reparse", lambda path: Path(path) == nested)
  with pytest.raises(ValueError, match="symlinks and reparse points"):
    subject.build_inventory(bound_request(root), root)


def test_nested_reparse_file_is_rejected(tmp_path, monkeypatch):
  root = tmp_path / "root"
  root.mkdir()
  write_extract(root / "reference.npz")
  nested = root / "nested.npz"
  write_extract(nested, offset=0.01)
  monkeypatch.setattr(subject, "_is_reparse", lambda path: Path(path) == nested)
  with pytest.raises(ValueError, match="symlinks and reparse points"):
    subject.build_inventory(bound_request(root), root)


def test_changed_path_set_is_rejected(tmp_path, monkeypatch):
  write_extract(tmp_path / "hit.npz")
  value = bound_request(tmp_path)
  original = subject._load_location

  def add_file(path):
    result = original(path)
    write_extract(tmp_path / "appeared.npz", offset=0.01)
    return result

  monkeypatch.setattr(subject, "_load_location", add_file)
  with pytest.raises(ValueError, match="changed"):
    subject.build_inventory(value, tmp_path)


def test_run_refuses_overwrite_and_writes_only_inventory(tmp_path):
  write_extract(tmp_path / "hit.npz")
  value = bound_request(tmp_path)
  request_path = tmp_path / "request.json"
  request_path.write_text(json.dumps(value), encoding="utf-8")
  output = tmp_path / "result"
  result = subject.run(request_path, tmp_path, output)
  assert json.loads((output / "inventory.json").read_text(encoding="utf-8")) == result
  with pytest.raises(FileExistsError):
    subject.run(request_path, tmp_path, output)


def test_reference_source_identity_is_retained_and_changes_request_identity(tmp_path):
  write_extract(tmp_path / "hit.npz")
  value = bound_request(tmp_path)
  first_request = tmp_path / "request-a.json"
  first_request.write_text(json.dumps(value), encoding="utf-8")
  first = subject.run(first_request, tmp_path, tmp_path / "first")
  changed = value
  changed["reference"]["rlogs"][0]["sha256"] = "b" * 64
  second_request = tmp_path / "request-b.json"
  second_request.write_text(json.dumps(changed), encoding="utf-8")
  second = subject.run(second_request, tmp_path, tmp_path / "second")
  assert first["request_sha256"] != second["request_sha256"]
  assert first["reference"]["rlogs"][0]["sha256"] == "a" * 64
  assert second["reference"]["rlogs"][0]["sha256"] == "b" * 64
