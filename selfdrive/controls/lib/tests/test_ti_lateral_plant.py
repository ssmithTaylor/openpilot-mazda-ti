"""Unit tests for the Mazda + Torque Interceptor lateral plant model.

The properties that matter for control safety: the model is monotone in the command (so its inverse
exists and cannot fold), the table inverse agrees with the closed form, round trips are exact enough
to steer with, the actuator-state machine follows the EPS, and no input produces a command outside
the interceptor's ceiling.
"""
import numpy as np
import pytest

from openpilot.selfdrive.car.mazda.lateral_plant import (
  TiLateralPlant, EFF_MAX, K_EFF, U_KNEE, DEAD_STOCK_TIME,
  STATE_TI_STOCK, STATE_TI_ONLY, STATE_STOCK_ONLY, STATE_NONE, STATE_RAMP_FLAG,
)

SPEEDS = [3.0, 7.0, 10.0, 12.5, 15.0, 17.5, 20.0, 25.0, 30.0, 40.0]
STATES = [(True, True), (True, False), (False, True), (False, False)]   # (ti_active, stock_active)


def plant(ti_active=True, stock_active=True, ramp_in=False, e_used=0.0, ti_max=600.0):
  p = TiLateralPlant(ti_max)
  p.ti_active = ti_active
  p.stock_active = stock_active
  p.ramp_in = ramp_in
  p.e_used = e_used
  return p


class TestForward:
  def test_monotone_in_command(self):
    for v in SPEEDS:
      for ti, stock in STATES:
        p = plant(ti, stock)
        la = np.array([p.forward(u, v) for u in np.arange(0.0, 601.0, 5.0)])
        assert np.all(np.diff(la) >= -1e-12), f"not monotone at v={v} state={(ti, stock)}"

  def test_odd_symmetry(self):
    p = plant()
    for v in SPEEDS:
      for u in (0.0, 50.0, 250.0, 342.0, 600.0):
        assert p.forward(-u, v) == pytest.approx(-p.forward(u, v), abs=1e-12)

  def test_zero_command_zero_accel(self):
    for ti, stock in STATES:
      assert plant(ti, stock).forward(0.0, 25.0) == 0.0

  def test_no_actuators_no_accel(self):
    p = plant(ti_active=False, stock_active=False)
    assert p.forward(600.0, 25.0) == 0.0
    assert p.la_max(25.0) == 0.0
    assert p.inverse(2.0, 25.0) == 0.0

  def test_knee_is_continuous(self):
    p = plant(ti_active=True, stock_active=False)
    for v in SPEEDS:
      below = p.forward(U_KNEE - 1e-6, v)
      above = p.forward(U_KNEE + 1e-6, v)
      assert below == pytest.approx(above, abs=1e-6)

  def test_stock_path_clips(self):
    """Past 342 counts of request the EPS is at its clip; all further gain is the interceptor's."""
    p = plant(ti_active=False, stock_active=True)
    v = 25.0
    at_clip = p.forward(EFF_MAX / K_EFF, v)
    assert p.forward(600.0, v) == pytest.approx(at_clip, abs=1e-9)
    assert at_clip == pytest.approx(p._gs(v) * EFF_MAX, abs=1e-9)

  def test_authority_matches_measurements(self):
    """la_max against the drives. These are steady-state values -- what the car reaches if the
    command is held, not what it has done 0.3 s in -- so they sit above the numbers the first cut of
    these tables carried. Measured at the ceiling on interceptor-alone frames: 1.13 m/s2 at 45 km/h,
    1.6-1.8 above 70; with the stock path on, 2.3-2.5 at speed (model_check.py)."""
    ti_only = plant(ti_active=True, stock_active=False)
    assert ti_only.la_max(12.5) == pytest.approx(1.13, abs=0.15)
    assert ti_only.la_max(20.0) == pytest.approx(1.62, abs=0.15)
    both = plant()
    assert both.la_max(20.0) == pytest.approx(2.28, abs=0.2)
    assert both.la_max(25.0) == pytest.approx(2.51, abs=0.2)

  def test_where_each_actuator_matters(self):
    """The measured split: at small commands the stock path carries at least as much as the
    interceptor, and at the ceiling the interceptor carries most of it. (Below the knee the model
    over-states the interceptor a little -- it commands slightly less than needed there, which the
    PID covers; over-steer is the direction we refuse.)"""
    v = 21.0
    both, ti_only = plant(), plant(stock_active=False)
    small_ratio = ti_only.forward(200.0, v) / both.forward(200.0, v)
    ceiling_ratio = ti_only.forward(600.0, v) / both.forward(600.0, v)
    assert small_ratio < 0.5, small_ratio
    assert ceiling_ratio > 0.6, ceiling_ratio


class TestInverse:
  def test_round_trip(self):
    """Below saturation the inverse returns the command it came from. (At and above the plant's
    limit the model is flat -- many commands give the same accel -- and the inverse deliberately
    goes to the ceiling there so the saturation warning fires; test_saturation_goes_to_ceiling.)"""
    for v in SPEEDS:
      for ti, stock in STATES:
        p = plant(ti, stock)
        lm = p.la_max(v)
        if lm == 0.0:
          continue
        for u in np.arange(10.0, 601.0, 10.0):
          la = p.forward(u, v)
          if la >= lm - 1e-9:
            continue
          assert p.inverse(la, v) == pytest.approx(u, abs=6.0), (v, ti, stock, u)

  def test_saturation_goes_to_ceiling(self):
    """Asking for more than the car can do commands the ceiling, so the controller's saturation
    check reports it -- notably with the interceptor tripped off, where authority is ~0.3 m/s^2."""
    for v in SPEEDS:
      for ti, stock in STATES:
        p = plant(ti, stock)
        if p.la_max(v) == 0.0:
          continue
        assert p.inverse(p.la_max(v) + 0.5, v) == pytest.approx(600.0)
        assert p.inverse(p.la_max(v), v) == pytest.approx(600.0)

  def test_forward_of_inverse(self):
    for v in SPEEDS:
      for ti, stock in STATES:
        p = plant(ti, stock)
        lm = p.la_max(v)
        if lm == 0.0:
          continue
        for la in np.linspace(0.0, lm, 25):
          assert p.forward(p.inverse(la, v), v) == pytest.approx(la, abs=0.01)

  def test_table_matches_closed_form(self):
    for v in SPEEDS:
      for ti, stock in STATES:
        p = plant(ti, stock)
        lm = p.la_max(v)
        if lm == 0.0:
          continue
        for la in np.linspace(0.02, lm, 30):
          assert p.inverse(la, v) == pytest.approx(p.inverse_closed(la, v), abs=8.0), (v, ti, stock, la)

  def test_saturates_at_the_ceiling(self):
    for v in SPEEDS:
      for ti, stock in STATES:
        for ti_max in (600.0, 500.0):
          p = plant(ti, stock, ti_max=ti_max)
          for la in (5.0, 50.0, -5.0, 1e9):
            assert abs(p.inverse(la, v)) <= ti_max + 1e-9

  def test_sign_is_preserved(self):
    p = plant()
    for v in SPEEDS:
      assert p.inverse(1.0, v) > 0
      assert p.inverse(-1.0, v) < 0
      assert p.inverse(0.0, v) == 0.0

  def test_small_command_is_finite_at_zero(self):
    """Below the knee the model is linear in the command, so the inverse stays finite and smooth
    through zero instead of the square root a pure quadratic would give. The knee is 150 counts:
    small enough that the model does not credit the interceptor with torque it has not got down
    there (measured 0.85 of the model at 60-150 counts, against 0.68 at a 250-count knee), large
    enough that the feedforward is not chasing the last few counts around centre."""
    p = plant(ti_active=True, stock_active=False)
    v = 8.5
    assert p.inverse(0.05, v) < 130.0
    assert p.inverse(0.10, v) < 210.0

  def test_ramp_in_uses_the_measurement(self):
    """During ramp-in the stock share is what the EPS reports, so the command falls as it winds in."""
    v = 15.0
    p_ramp = plant(ramp_in=True, e_used=0.0)
    p_settled = plant()
    assert p_ramp.inverse(0.5, v) > p_settled.inverse(0.5, v)
    p_ramp.e_used = 300.0
    assert p_ramp.inverse(0.5, v) < p_settled.inverse(0.5, v) + 1e-6


class TestStateMachine:
  def test_blocked_means_ti_only(self):
    p = TiLateralPlant()
    code = p.update_state(12.0, lkas_blocked=True, eff_meas=0.0, ti_active=True)
    assert not p.stock_active and code == STATE_TI_ONLY
    assert p.forward(600.0, 12.0) == pytest.approx(plant(True, False).forward(600.0, 12.0))

  def test_ti_off_means_stock_only(self):
    p = TiLateralPlant()
    code = p.update_state(25.0, lkas_blocked=False, eff_meas=200.0, ti_active=False)
    assert p.stock_active and not p.ti_active
    assert code in (STATE_STOCK_ONLY, STATE_STOCK_ONLY + STATE_RAMP_FLAG)
    assert p.la_max(25.0) < 0.8      # the stock clip is all that is left

  def test_both_off(self):
    p = TiLateralPlant()
    code = p.update_state(25.0, lkas_blocked=True, eff_meas=0.0, ti_active=False)
    assert code == STATE_NONE and p.la_max(25.0) == 0.0

  def test_ramp_in_after_block_clears_then_settles(self):
    """LKAS_BLOCK clears at speed: the EPS winds LKAS_EFFECTIVE in at ~1.5 counts/frame and the model
    follows the measurement until it has caught up, then goes algebraic."""
    p = TiLateralPlant()
    p.update_state(15.0, lkas_blocked=True, eff_meas=0.0, ti_active=True)
    assert p.ramp_in
    p.u_prev = 400.0
    eff = 0.0
    codes = []
    for _ in range(300):
      eff = min(eff + 1.5, K_EFF * p.u_prev)
      codes.append(p.update_state(15.0, lkas_blocked=False, eff_meas=eff, ti_active=True))
      if not p.ramp_in:
        break
    assert not p.ramp_in, "ramp-in never settled"
    # ~2 s: the EPS reaches the 300-count "nothing left to ramp" mark at 1.5 counts/frame
    assert 150 <= len(codes) <= 260, len(codes)
    assert codes[0] == STATE_TI_STOCK + STATE_RAMP_FLAG
    assert codes[-1] == STATE_TI_STOCK

  def test_ramp_in_times_out(self):
    """If the EPS never catches up (e.g. it clips), ramp-in still ends within its timeout."""
    p = TiLateralPlant()
    p.update_state(15.0, lkas_blocked=True, eff_meas=0.0, ti_active=True)
    p.u_prev = 600.0
    for _ in range(int(3.0 / p.dt) + 5):
      p.update_state(15.0, lkas_blocked=False, eff_meas=0.0, ti_active=True)
    assert not p.ramp_in

  def test_ramp_in_ignores_opposite_sign_feedback(self):
    p = TiLateralPlant()
    p.update_state(15.0, lkas_blocked=True, eff_meas=0.0, ti_active=True)
    p.u_prev = 300.0
    p.update_state(15.0, lkas_blocked=False, eff_meas=-250.0, ti_active=True)
    assert p.e_used == 0.0

  def test_one_frame_block_glitch_is_harmless(self):
    """A 10 ms LKAS_BLOCK blip re-arms ramp-in, which immediately settles because the EPS is
    already caught up -- the command must not jump."""
    p = TiLateralPlant()
    p.u_prev = 300.0
    for _ in range(20):
      p.update_state(20.0, lkas_blocked=False, eff_meas=270.0, ti_active=True)
    before = p.inverse(1.0, 20.0)
    p.update_state(20.0, lkas_blocked=True, eff_meas=0.0, ti_active=True)
    for _ in range(6):
      p.update_state(20.0, lkas_blocked=False, eff_meas=270.0, ti_active=True)
    assert p.inverse(1.0, 20.0) == pytest.approx(before, abs=15.0)

  def test_dead_stock_path_detected_and_cleared(self):
    """LKAS_BLOCK clear but the EPS applies nothing (camera fault / LKAS off): after a second of
    asking, treat the stock path as gone."""
    p = TiLateralPlant()
    p.u_prev = 400.0
    for _ in range(20):
      p.update_state(20.0, lkas_blocked=False, eff_meas=360.0, ti_active=True)   # settle first
    assert not p.ramp_in and p.stock_active
    n = int(DEAD_STOCK_TIME / p.dt)
    for _ in range(n + 2):
      p.update_state(20.0, lkas_blocked=False, eff_meas=0.0, ti_active=True)
    assert p.dead_stock and not p.stock_active
    assert p.la_max(20.0) == pytest.approx(plant(True, False).la_max(20.0))
    p.update_state(20.0, lkas_blocked=False, eff_meas=100.0, ti_active=True)
    assert not p.dead_stock and p.ramp_in

  def test_dead_stock_not_declared_when_not_asking(self):
    p = TiLateralPlant()
    p.u_prev = 50.0        # below the threshold: the EPS reporting 0 proves nothing
    for _ in range(int(DEAD_STOCK_TIME / p.dt) * 3):
      p.update_state(20.0, lkas_blocked=False, eff_meas=0.0, ti_active=True)
    assert not p.dead_stock

  def test_inactive_rearms(self):
    p = TiLateralPlant()
    p.ramp_in = False
    p.update_state(20.0, lkas_blocked=False, eff_meas=200.0, ti_active=True, active=False)
    assert p.ramp_in and p.e_used == 0.0


class TestMultipliers:
  def test_bounded(self):
    p = TiLateralPlant()
    p.set_multipliers(10.0, -3.0)
    assert 0.75 <= p.alpha_ti <= 1.25 and 0.75 <= p.alpha_stock <= 1.25

  def test_scale_the_model(self):
    p, q = plant(), plant()
    q.set_multipliers(1.25, 1.25)
    assert q.forward(400.0, 25.0) == pytest.approx(1.25 * p.forward(400.0, 25.0), rel=1e-9)
    assert q.inverse(1.0, 25.0) < p.inverse(1.0, 25.0)


class TestCeiling:
  def test_ti_steer_max_respected(self):
    p = TiLateralPlant()
    p.set_ti_steer_max(500.0)
    assert p.inverse(9.0, 25.0) == pytest.approx(500.0)
    assert p.forward(600.0, 25.0) == pytest.approx(p.forward(500.0, 25.0))

  def test_ti_steer_max_is_clamped(self):
    p = TiLateralPlant()
    p.set_ti_steer_max(1e6)
    assert p.ti_steer_max <= 1200.0
    p.set_ti_steer_max(0.0)
    assert p.ti_steer_max >= 100.0


class TestNumerics:
  def test_no_nan_or_inf(self):
    p = TiLateralPlant()
    for v in (0.0, 0.5, 3.0, 25.0, 60.0, 200.0):
      for la in (0.0, 0.001, 1.0, 3.5, 1e6, -1e6, -0.001):
        u = p.inverse(la, v)
        assert np.isfinite(u) and abs(u) <= p.ti_steer_max
        assert np.isfinite(p.forward(u, v))

  def test_below_lowest_breakpoint_holds(self):
    p = plant(ti_active=True, stock_active=False)
    assert p.forward(600.0, 2.0) == pytest.approx(p.forward(600.0, 7.0))
    assert p.forward(600.0, 60.0) == pytest.approx(p.forward(600.0, 35.0))
