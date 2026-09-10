import pytest

from tools.mazda_ti.camera_timing import build_mapping


def rows():
  return [{"segment_id": i, "frame_id": 100 + i, "encode_id": 200 + i,
           "timestamp_sof_ns": (i * 50_000_000) - 5_000_000,
           "timestamp_eof_ns": i * 50_000_000} for i in range(4)]


def test_mapping_keeps_sof_eof_and_brackets():
  event = build_mapping(rows(), {"mid": .075})["events"]["mid"]
  assert event["before"]["segment_id"] == 1
  assert event["after"]["segment_id"] == 2
  assert event["chosen"]["timestamp_sof_ns"] == 45_000_000
  assert event["chosen"]["generated_clip_pts_s"] == .05


def test_rejects_gap_and_frame_count_mismatch():
  with pytest.raises(ValueError, match="contiguous"):
    build_mapping([*rows()[:2], {**rows()[3]}], {})
  with pytest.raises(ValueError, match="decoded frame"):
    build_mapping(rows(), {}, decoded_frame_count=3)


def test_uses_decoded_frame_id_when_segment_id_differs():
  shifted = [{**row, "segment_id": row["segment_id"] + 1000} for row in rows()]
  event = build_mapping(shifted, {"mid": .075})["events"]["mid"]
  assert event["chosen"]["frame_id"] == 101
  assert event["chosen"]["generated_clip_pts_s"] == .05


def test_rejects_invalid_target_coverage():
  with pytest.raises(ValueError, match="outside"):
    build_mapping(rows(), {"late": 1.0})
