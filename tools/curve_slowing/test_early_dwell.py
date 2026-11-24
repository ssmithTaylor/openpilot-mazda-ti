"""tools/curve_slowing/early_dwell.py on synthetic passes only (no recorded drive, no position)."""
import numpy as np
import pytest

from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
from openpilot.tools.curve_slowing import early_dwell as E

DT = 0.05
MPH = E.MPH


def _pass(v_mph, bend_at_m, k, length_m=3000.0):
  n = int(length_m / (v_mph * MPH * DT))
  s = np.arange(n) * v_mph * MPH * DT
  return s, np.where(s >= bend_at_m, k, 0.0)


def test_profile_measured_response():
  # rewritten for plan amendment A1 (was the 1.5 s + 1.36 mph/s profile): the profile is ti_cruise_model's walk straight to T; R11:
  # the default "arrived" is now the planner's 0.5 mph, so this comparison with the measured manoeuvre definition passes 1.5 explicitly
  p = E.Profile(55, 40, arrived=CM.ARRIVED_MPH)
  t, x = CM.manoeuvre(55, 40)
  assert p.t_arr == pytest.approx(t, abs=0.051) and p.d_arr == pytest.approx(x, abs=1.5)
  t0, v0 = p.at(0.5 * 55 * MPH * 0.99)
  assert v0 == pytest.approx(55, abs=0.01) and t0 == pytest.approx(0.5, abs=0.06)          # nothing in the dead time
  t, v = p.at(p.d_arr)
  assert v == pytest.approx(41.5, abs=0.1) and t == pytest.approx(p.t_arr)
  t, v = p.at(p.d_arr + p.v_arr * MPH * 2.0)
  assert v == pytest.approx(p.v_arr) and t == pytest.approx(p.t_arr + 2.0)


def test_arrived_is_the_planners():
  # plan R11 / A11: early dwell's "arrived" is the planner's ARRIVED_MPH (0.5), not the measured manoeuvre definition (1.5)
  from openpilot.selfdrive.car.mazda import ti_curve_plan as P
  assert E.ARRIVED_MPH == P.ARRIVED_MPH == 0.5
  p = E.Profile(55, 40)
  assert p.v_arr <= 40.5 + 1e-9 and p.t_arr > E.Profile(55, 40, arrived=1.5).t_arr


def test_profile_push_and_raise_arrives_sooner_without_undershoot():
  # the planner's own manoeuvre (push to S1, model-timed raise): earlier arrival than the straight walk, the speed never more than
  # 0.5 mph below T afterwards (A9 / A10)
  straight, push = E.Profile(50, 35), E.Profile(50, 35, S1=29)
  assert push.t_arr < straight.t_arr - 0.5
  assert float(push.vs.min()) >= 35 - 0.5


def test_load_searched_from_the_planned_bend():
  # plan A5: a bend the car is still in at the start (or one before the planned bend) is not the planned bend's load: searched from
  # i_from (the planned bend's approach), the dwell is early; from the start it would read as LATE
  p = E.Profile(55, 40)
  k = 1.6 / (40 * MPH) ** 2
  s, kap = _pass(55, p.d_arr + 5.0 * p.v_arr * MPH, k)
  kap[s < 40.0] = k                                 # in a bend at the start
  assert E.dwell(55, 40, s, kap, 0, s[-1], DT)['late']
  i_from = int(np.searchsorted(s, p.d_arr))
  r = E.dwell(55, 40, s, kap, 0, s[-1], DT, i_from=i_from)
  assert not r['late'] and r['dwell_s'] == pytest.approx(5.0, abs=0.1)


def test_dwell_early_and_late():
  p = E.Profile(55, 40)
  k = 1.6 / (40 * MPH) ** 2                       # loads 1.6 m/s^2 at the target
  # bend 5 s (at the arrival speed) after the predicted arrival: dwell 5 s
  s, kap = _pass(55, p.d_arr + 5.0 * p.v_arr * MPH, k)
  r = E.dwell(55, 40, s, kap, 0, s[-1], DT)
  assert r['dwell_s'] == pytest.approx(5.0, abs=0.1) and not r['late']
  # bend a third of the way through the deceleration: late, still well above the target there
  s, kap = _pass(55, p.d_arr / 3, k)
  r = E.dwell(55, 40, s, kap, 0, s[-1], DT)
  assert r['late'] and r['dwell_s'] < -2 and 5 < r['over_mph'] < 15


def test_load_needs_1_3_at_the_target_sustained():
  s, kap = _pass(50, 500.0, 1.2 / (40 * MPH) ** 2)  # 1.2 m/s^2 at the target: never loads
  assert E.dwell(50, 40, s, kap, 0, s[-1], DT) is None
  s, kap = _pass(50, 500.0, 1.4 / (40 * MPH) ** 2)
  kap[(s > 300) & (s < 303)] = 1.0                 # a 0.15 s spike earlier is not the bend
  j = E.load_index(s, kap, 40, 0, s[-1], DT)
  assert s[j] == pytest.approx(500.0, abs=1.5)
  assert E.load_index(s, kap, 40, 0, 400.0, DT) is None   # beyond the window: not found


def test_summary():
  rows = [dict(dwell_s=d, over_mph=o, late=d < 0) for d, o in ((5, 1.5), (8, 1.5), (-1, 4.0), (-2, 6.5))] + [None]
  sm = E.summary(rows)
  assert sm['n'] == 4 and sm['late'] == 2 and sm['worst_over_mph'] == 6.5 and sm['median'] == 2.0
