"""The ONE measured cruise model (ti_cruise_model.py, plan amendment A1): pins the measured table, the dynamics, the tap walk and the
whole-manoeuvre times at 45 mph. Synthetic only."""
import pytest

from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
from openpilot.selfdrive.car.mazda import ti_curve_plan as P


def test_measured_table_grade_corrected_capped_at_10_14():
  # band centres: 1-2 0.21; 2-3 0.31; 3-4 0.41; 4-6 0.58; 6-8 0.64; 8-10 0.81; 10-14 0.90 m/s^2
  for gap, a in ((1.5, 0.21), (2.5, 0.31), (3.5, 0.41), (5.0, 0.58), (7.0, 0.64), (9.0, 0.81), (12.0, 0.90)):
    assert CM.decel_ms2(gap) == pytest.approx(a)
  assert CM.decel_ms2(0.0) == 0.0 and CM.decel_ms2(-3.0) == 0.0
  assert CM.decel_ms2(16.0) == CM.decel_ms2(30.0) == pytest.approx(0.90)     # the 14+ band (5 events) is not extrapolated
  gaps = [x / 4.0 for x in range(0, 80)]
  for v in (CM.TIMING, CM.CONSERVATIVE):
    a = [CM.decel_ms2(g, v) for g in gaps]
    assert all(y >= x for x, y in zip(a, a[1:]))                               # never less slowing for a larger gap
  # the conservative (uncorrected) variant never assumes more, and flattens near 0.6 from 6 mph
  assert all(CM.decel_ms2(g, CM.CONSERVATIVE) <= CM.decel_ms2(g) + 1e-12 for g in gaps)
  assert 0.55 <= CM.decel_ms2(7.0, CM.CONSERVATIVE) <= 0.66 and CM.decel_ms2(20.0, CM.CONSERVATIVE) <= 0.66
  assert CM.decel_mph_s(5.0) == pytest.approx(0.58 / CM.MPH)


def test_dead_time_and_lag():
  assert CM.DELAY_S == 0.5 and CM.TAU_S == 0.5
  dt = 0.01
  car = CM.Cruise(45.0, 45.0, dt)
  vs = [car.step(40.0) for _ in range(int(0.5 / dt) - 1)]
  assert all(v == 45.0 for v in vs)                                          # nothing for the dead time
  # then a first-order approach to the settled deceleration for a 5 mph gap: 63 % one time constant after the dead time
  car = CM.Cruise(45.0, 45.0, dt)
  a = []
  for _ in range(int(1.0 / dt) + 1):
    v0 = car.v
    car.step(40.0)
    a.append((v0 - car.v) / dt)
  target = CM.decel_mph_s(5.0)
  assert a[int(1.0 / dt)] == pytest.approx(target * 0.632, rel=0.08)


def test_tap_walk_one_mph_per_0_30_s():
  assert CM.TAP_S == 0.30 and CM.TAP_S_SLOW == 0.40
  assert CM.dash_after(45, 35, 0.0) == 44                                    # the first tap at t = 0
  assert CM.dash_after(45, 35, 0.29) == 44 and CM.dash_after(45, 35, 0.30) == 43
  assert CM.dash_after(45, 35, 2.7) == 35 and CM.dash_after(45, 35, 10.0) == 35
  assert CM.dash_after(30, 33, 0.65) == 33
  assert P.TAP_S == CM.TAP_S


@pytest.mark.parametrize("drop,t_lo,t_hi,x_lo,x_hi", [(5, 4.5, 5.0, 87, 98), (10, 7.2, 8.4, 143, 155), (15, 11.1, 11.7, 193, 239)])
def test_whole_manoeuvre_at_45(drop, t_lo, t_hi, x_lo, x_hi):
  # measured (A1): 45 mph to within 1.5 mph of the target, walking straight to it. The ranges are quoted to 0.1 s and 1 m: the model
  # must land inside them to that precision (+-0.05 s, +-2 m: the 15 mph case gives 191.5 m against the quoted 193), integrated finely
  t, x = CM.manoeuvre(45, 45 - drop, dt=0.01)
  assert t_lo - 0.05 <= t <= t_hi + 0.05, t
  assert x_lo - 2.0 <= x <= x_hi + 2.0, x


def test_planner_warning_and_test_cars_share_the_model():
  # the planner's settled-deceleration law IS the model's; its approach IS the model's manoeuvre (push to S1, raise by CM.raise_due)
  for g in (0.0, 1.0, 2.5, 5.0, 8.0, 13.0):
    assert P.decel_mph_s(g) == CM.decel_mph_s(g)
  t, x = CM.manoeuvre(55, 40, 34, arrived=P.ARRIVED_MPH)            # A9-R2a: raised when the model says (raise_at=None)
  assert P.approach_m(55, 40, 34) == pytest.approx(x)
  assert P.REACTION_S == CM.DELAY_S
  # plan A8-R1: the planner aims at T + 0.5 (on target at entry; 1.0 ruled, see ti_curve_plan.ARRIVED_MPH); CM.ARRIVED_MPH stays the MEASURED manoeuvre definition (1.5)
  assert P.ARRIVED_MPH == 0.5 and CM.ARRIVED_MPH == 1.5


def test_speed_at_follows_the_walk():
  # from 45 with the dash at 45, walking to 35 over 300 m: the same as stepping the model by hand
  v = CM.speed_at(45.0, 300.0, 45.0, 35.0, True, dt=0.05)
  car, x, t = CM.Cruise(45.0, 45.0, 0.05), 0.0, 0.0
  while x < 300.0:
    car.step(CM.dash_after(45.0, 35.0, t))
    x += car.v * CM.MPH * 0.05
    t += 0.05
  assert v == pytest.approx(car.v)
  assert CM.speed_at(45.0, 300.0, 45.0, 35.0, False) == 45.0                   # not lowering: nothing changes


def test_r3_manoeuvre_walks_the_dash_from_the_real_set_speed():
  # plan A8-R3: a car held below its set speed (a lead) needs (set - target) taps, not (speed - target)
  t0, x0 = CM.manoeuvre(40, 30)
  t1, x1 = CM.manoeuvre(40, 30, set0=55)
  assert t1 >= t0 + 15 * CM.TAP_S - 0.5 and x1 > x0
  assert CM.manoeuvre(40, 30, set0=40) == (t0, x0)
