from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from tools.mazda_ti.closed_loop_id_audit import materialize, run


def rows(offset: float = 0.0):
  return [
    {
      "time_s": i * 0.05,
      "ti_command_counts": offset + 3.0 * i,
      "wheel_angle_deg": offset + 0.1 * i,
      "yaw_lateral_accel_mps2": offset + 0.01 * i,
      "speed_mps": 20.0,
    }
    for i in range(60)
  ]


class ClosedLoopIdAuditTest(unittest.TestCase):
  def test_future_command_is_not_a_predictor(self):
    original = rows()
    changed = copy.deepcopy(original)
    changed[2]["ti_command_counts"] = 9999.0
    first_original = materialize(original, "wheel_angle_deg", "delayed_command_rate", 0)[0]
    first_changed = materialize(changed, "wheel_angle_deg", "delayed_command_rate", 0)[0]
    self.assertEqual(first_original[0].tolist(), first_changed[0].tolist())
    self.assertEqual(first_original[1], first_changed[1])

  def test_leave_case_out_result_has_both_response_proxies(self):
    payload = {
      "format_version": 1,
      "cases": [
        {"case_id": "smooth_a", "outcome": "smooth", "rows": rows(0.0)},
        {"case_id": "smooth_b", "outcome": "smooth", "rows": rows(1.0)},
        {"case_id": "settling", "outcome": "settling", "rows": rows(2.0)},
        {"case_id": "scallop", "outcome": "scallop", "rows": rows(3.0)},
      ],
    }
    result = run(payload)
    self.assertEqual(set(result["responses"]), {"wheel_angle_increment_deg", "yaw_lateral_accel_increment_mps2"})
    for response in result["responses"].values():
      self.assertEqual(set(response), {"smooth_a", "smooth_b", "settling", "scallop"})
      self.assertEqual(response["scallop"]["models"]["delayed_command"]["selected_on_other_cases_only"]["lag_samples"] in (0, 1, 2, 3, 4, 5, 6, 8), True)


if __name__ == "__main__":
  unittest.main()
