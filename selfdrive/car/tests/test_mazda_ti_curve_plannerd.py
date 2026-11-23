"""The planner process's helpers (ti_curve_plannerd.py): the yaw decode and the model curvature. Synthetic frames only."""
from types import SimpleNamespace as NS

from openpilot.selfdrive.car.mazda import ti_curve_plannerd as D


def frame(addr, dat, src=0):
  return NS(address=addr, dat=bytes(dat), src=src)


def test_yaw_decode_matches_the_study():
  # raw = 18000: -(18000 * -0.00017243 + 3.1032) = 3.1037 - 3.1032 ... left positive
  raw = 18000
  y = D.yaw_from_can([frame(0x75, [0, 0, 0, 0, raw >> 8, raw & 0xFF, 0, 0])])
  assert abs(y - (-(raw * D.YAW_SCALE + D.YAW_OFF))) < 1e-12
  zero = round(-D.YAW_OFF / D.YAW_SCALE)                 # the count that reads no yaw
  assert abs(D.yaw_from_can([frame(0x75, [0, 0, 0, 0, zero >> 8, zero & 0xFF, 0, 0])])) < 2e-4


def test_yaw_decode_takes_the_newest_bus0_frame_only():
  a = frame(0x75, [0, 0, 0, 0, 0x46, 0x50, 0, 0])
  b = frame(0x75, [0, 0, 0, 0, 0x47, 0x00, 0, 0])
  other_bus = frame(0x75, [0, 0, 0, 0, 0x10, 0x00, 0, 0], src=2)
  assert D.yaw_from_can([a, b, other_bus]) == D.yaw_from_can([b])
  assert D.yaw_from_can([frame(0x76, [0] * 8)]) is None


def test_model_curvature():
  m = NS(orientationRate=NS(z=[0.0, 0.1, 0.2], t=[0.0, 1.0, 2.0]), velocity=NS(x=[20.0, 20.0, 1.0]))
  t, k = D.model_curvature(m)
  assert t == [0.0, 1.0, 2.0] and abs(k[1] - 0.005) < 1e-12 and abs(k[2] - 0.2 / 3.0) < 1e-12
  assert D.model_curvature(NS(orientationRate=NS(z=[], t=[]), velocity=NS(x=[]))) is None


def test_speed_limit_only_when_known_and_plausible():
  st = {"t": None}
  assert D.fresh_limit(True, 55 * D.MPH, 50 * D.MPH, True, 0.0, st) == pytest_approx(55)
  assert D.fresh_limit(False, 55 * D.MPH, 50 * D.MPH, True, 0.0, st) is None      # planner message not alive (toggle off)
  assert D.fresh_limit(True, 55 * D.MPH, 50 * D.MPH, False, 0.0, st) is None      # no position
  assert D.fresh_limit(True, 0.0, 50 * D.MPH, True, 0.0, st) is None              # unknown
  assert D.fresh_limit(True, 25 * D.MPH, 55 * D.MPH, True, 0.0, st) is None       # 25 on a 55 mph drive: implausible


def pytest_approx(x):
  import pytest
  return pytest.approx(x, abs=1e-6)


def test_speed_limit_far_above_the_speed_is_not_used():
  assert D.fresh_limit(True, 70 * D.MPH, 45 * D.MPH, True, 0.0, {"t": None}) is None
