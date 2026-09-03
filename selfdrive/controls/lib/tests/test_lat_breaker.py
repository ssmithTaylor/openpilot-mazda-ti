"""Tests for the escalating stiction breaker.

The lesson being pinned (route 00000283): the release need is a distribution (median 30 /
p75 63-91 / p90 184 counts), and firing the p90 flat at every stuck moment kicked mild corners
with 180-count lurches ("trouble caused by recovery attempts"). The boost must start at the
p75-sized stage 1, escalate to the ceiling only while the wheel STAYS stuck, withdraw at the old
fast rate, and re-stick starts gentle again.
"""
import math

import numpy as np
import pytest

from cereal import car, log
from openpilot.common.mock.generators import generate_liveLocationKalman
from openpilot.selfdrive.controls.lib.latcontrol_torque import (
  LatControlTorque, BREAK_STAGE1, BREAK_MAX, BREAK_RAMP, BREAK_ESCALATE, BREAK_DEBOUNCE,
  BREAKAWAY_TORQUE,
)
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.selfdrive.car.mazda.lateral_plant import TiLateralPlant
from types import SimpleNamespace

DT = 0.01
V = 20.0
U_MAX = 600.0


def make_cp():
  CP = car.CarParams.new_message()
  CP.steerRatio = 15.5
  CP.wheelbase = 2.7
  CP.mass = 1650.0
  CP.centerToFront = 1.2
  CP.tireStiffnessFactor = 0.7
  CP.tireStiffnessFront = 192150.0
  CP.tireStiffnessRear = 202500.0
  CP.lateralTuning.init("torque")
  CP.lateralTuning.torque.kp = 0.8
  CP.lateralTuning.torque.ki = 0.2
  CP.lateralTuning.torque.latAccelFactor = 1.4
  CP.lateralTuning.torque.friction = 0.05
  CP.steerActuatorDelay = 0.3
  return CP


class FakeCI:
  def __init__(self):
    self.lateral_plant = TiLateralPlant(U_MAX)

  def torque_from_lateral_accel(self):
    return lambda la, tp: la / tp.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda t, tp: t * tp.latAccelFactor


def toggles():
  return SimpleNamespace(lat_damping=False, lat_commit_setpoint=False, lat_friction_comp=False,
                         lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=U_MAX)


def fp_state(column_torque=0.0):
  return SimpleNamespace(lkasBlocked=False, lkasEffective=250.0, tiActive=True,
                         columnTorque=column_torque)


def make_controller():
  CP = make_cp()
  c = LatControlTorque(CP, FakeCI(), DT)
  VM = VehicleModel(CP)
  return c, VM


def step_stuck(c, VM, desired_la, measured_la, rate_deg=0.0, column_torque=0.0):
  """One update with the wheel at a fixed angle (stuck): rate forced via steeringRateDeg."""
  CS = car.CarState.new_message()
  CS.vEgo = V
  curv = measured_la / V ** 2
  CS.steeringAngleDeg = math.degrees(VM.get_steer_from_curvature(-curv, V, 0.0))
  CS.steeringRateDeg = rate_deg
  params = log.LiveParametersData.new_message()
  llk = generate_liveLocationKalman()
  return c.update(True, CS, VM, params, False, desired_la / V ** 2, False, 0.3,
                  llk, None, toggles(), fp_state(column_torque))


def boost_of(c):
  return abs(c.break_boost)


class TestEscalatingBreaker:
  def _hold_stuck(self, c, VM, n):
    """Keep break_frames climbing regardless of how the boost moves the error, so escalation is
    exercised deterministically: the test is about the boost's TIME shape given sustained stuck,
    not about the plant's error feedback. We assert on the boost trajectory's peak within a stage."""
    boosts = []
    for _ in range(n):
      step_stuck(c, VM, 3.0, 0.5, rate_deg=0.0)   # big sustained error so `stuck` never clears
      boosts.append(abs(c.break_boost))
    return boosts

  def test_first_kick_is_stage1_not_ceiling(self):
    # Over the first escalation-window of stuck time, the boost must not yet have reached the
    # ceiling: the onset is the p75 kick and escalation is gradual. (The kick itself can twitch
    # the wheel and reset the detector, which only makes the early boost SMALLER -- so the
    # ceiling-avoidance bound is what matters here, checked over the whole early span.)
    c, VM = make_controller()
    early = int((BREAK_DEBOUNCE + BREAK_ESCALATE) / DT) - 5   # up to just before full escalation
    boosts = self._hold_stuck(c, VM, early)
    peak = max(boosts)
    assert peak < BREAK_MAX * U_MAX * 0.9, \
      f"boost hit {peak:.0f} counts before the escalation window elapsed -- not gradual"
    assert peak >= BREAK_STAGE1 * U_MAX * 0.5, \
      f"boost never reached a meaningful kick ({peak:.0f} counts)"

  def test_escalates_to_ceiling_only_while_still_stuck(self):
    c, VM = make_controller()
    boosts = self._hold_stuck(c, VM, int((BREAK_DEBOUNCE + BREAK_ESCALATE + BREAK_RAMP) / DT) + 80)
    assert max(boosts) == pytest.approx(BREAK_MAX * U_MAX, rel=0.06), \
      "a persistently stuck wheel must eventually reach the full ceiling"
    # and the ceiling is reached LATE, not on the first kick
    early = max(boosts[:int((BREAK_DEBOUNCE + BREAK_RAMP) / DT) + 5])
    assert early < BREAK_MAX * U_MAX * 0.75, "escalation must be gradual, not immediate"

  def test_release_is_fast(self):
    # Once the wheel moves, the whole boost -- even a fully escalated one -- sheds in ~BREAK_RAMP.
    c, VM = make_controller()
    for _ in range(int((BREAK_DEBOUNCE + BREAK_ESCALATE) / DT) + 100):
      step_stuck(c, VM, 1.5, 0.8, rate_deg=0.0)
    assert boost_of(c) > BREAK_STAGE1 * U_MAX * 1.2
    for _ in range(int(BREAK_RAMP / DT) + 5):
      step_stuck(c, VM, 1.5, 0.8, rate_deg=30.0)
    assert boost_of(c) < 10.0, "boost must withdraw at the old fast rate once the wheel moves"

  def test_restick_starts_gentle_again(self):
    # After a release, a NEW stick begins at stage 1, not at the previous escalation level.
    c, VM = make_controller()
    for _ in range(int((BREAK_DEBOUNCE + BREAK_ESCALATE) / DT) + 100):
      step_stuck(c, VM, 1.5, 0.8, rate_deg=0.0)
    for _ in range(60):
      step_stuck(c, VM, 1.5, 0.8, rate_deg=30.0)   # moving: releases and resets the counter
    n_stage1 = int((BREAK_DEBOUNCE + BREAK_RAMP) / DT) + 10
    for _ in range(n_stage1):
      step_stuck(c, VM, 1.5, 0.8, rate_deg=0.0)
    b = boost_of(c)
    assert b <= BREAK_STAGE1 * U_MAX * 1.3, \
      f"re-stick jumped straight to {b:.0f} counts instead of restarting at stage 1"

  def test_no_boost_without_error(self):
    c, VM = make_controller()
    for _ in range(200):
      step_stuck(c, VM, 0.5, 0.5, rate_deg=0.0)
    assert boost_of(c) < 5.0

  def test_stands_down_at_the_authority_latch(self):
    # The route-00000283 lesson: a rack wound to the ~202-count SAT ceiling is grip-limited, not
    # stiction-stuck. The breaker must NOT fire there -- boosting is useless and is the felt
    # disturbance. Same stuck geometry as the escalation tests, but with the column at the latch.
    c, VM = make_controller()
    for _ in range(int((BREAK_DEBOUNCE + BREAK_ESCALATE) / DT) + 100):
      step_stuck(c, VM, 3.0, 2.4, rate_deg=0.0, column_torque=BREAKAWAY_TORQUE + 20)
    assert boost_of(c) < 5.0, "breaker fired into a grip-limited (authority-latched) rack"

  def test_still_fires_below_the_latch(self):
    # Control for the above: the SAME stuck geometry with the column BELOW the latch is real
    # stiction and must still get a kick.
    c, VM = make_controller()
    boosts = self._hold_stuck(c, VM, int((BREAK_DEBOUNCE + BREAK_ESCALATE) / DT) + 50)
    # _hold_stuck uses column_torque=0 (missing reading -> treated as not-at-latch)
    assert max(boosts) > BREAK_STAGE1 * U_MAX * 0.5, "breaker failed to fire on genuine stiction"

  def test_constants_shape(self):
    assert BREAK_STAGE1 < BREAK_MAX
    assert BREAK_STAGE1 == pytest.approx(0.15)
    assert BREAK_MAX == pytest.approx(0.30)
    assert 0.5 <= BREAK_ESCALATE <= 3.0


if __name__ == "__main__":
  pytest.main([__file__, "-q"])
