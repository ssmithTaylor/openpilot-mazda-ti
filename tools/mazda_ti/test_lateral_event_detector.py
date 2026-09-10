import pytest
from tools.mazda_ti.lateral_event_detector import score

def base():
  return [{"mono_ns": i * 50_000_000, "command": 100 - i * 20, "measurement": 1.0,
           "yaw_rate": 1.0, "speed": 1.0, "steering_angle": 2.0, "steering_rate": -1.0,
           "lane_valid": True, "health_ok": True, "identity_joined": True} for i in range(6)]

def test_motion_supported_candidate():
  out = score(base(), 0, 300_000_000)
  assert out["category"] == "motion-supported candidate"

def test_disagreement_is_command_only():
  rows = base()
  for row in rows[1:]: row["measurement"] = -2.0
  assert score(rows, 0, 300_000_000)["category"] == "command-only"

def test_gap_and_invalid_are_insufficient():
  rows = base(); rows[3]["mono_ns"] = 300_000_000; rows[4]["mono_ns"] = 350_000_000
  assert score(rows, 0, 500_000_000)["category"] == "insufficient evidence"
  rows = base(); rows[2]["speed"] = float("nan")
  assert score(rows, 0, 300_000_000)["category"] == "insufficient evidence"

def test_clock_mapping_is_retained():
  rows = base(); rows[2]["camera_eof_ns"] = 123
  assert any(event["clock"]["camera_eof_ns"] == 123 for event in score(rows, 0, 300_000_000)["events"])

def test_missing_identity_join_is_insufficient():
  rows = base()
  for row in rows: row["identity_joined"] = False
  out = score(rows, 0, 300_000_000)
  assert out["category"] == "insufficient evidence"
  assert out["gates"]["identity_joined"] is False

def test_missing_health_or_lane_gate_is_insufficient():
  rows = base()
  for row in rows: row.pop("health_ok")
  assert score(rows, 0, 300_000_000)["category"] == "insufficient evidence"

def test_left_and_right_steering_response_signs_are_required_for_motion_support():
  rows = base()
  for row in rows: row["identity_joined"] = True
  assert score(rows, 0, 300_000_000)["category"] == "motion-supported candidate"
  left = base()
  for row in left:
    row["command"] *= -1; row["measurement"] *= -1; row["yaw_rate"] *= -1
    row["steering_angle"] *= -1; row["steering_rate"] *= -1
    row["identity_joined"] = True
  assert score(left, 0, 300_000_000)["category"] == "motion-supported candidate"
  disagreement = base()
  for row in disagreement: row["steering_rate"] = 1.0; row["identity_joined"] = True
  assert score(disagreement, 0, 300_000_000)["category"] == "command-only"
