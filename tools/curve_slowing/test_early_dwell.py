"""tools/curve_slowing/early_dwell.py on synthetic passes only (no recorded drive, no position)."""
import numpy as np
import pytest

from openpilot.tools.curve_slowing import early_dwell as E

DT = 0.05
MPH = E.MPH


def _pass(v_mph, bend_at_m, k, length_m=3000.0):
  n = int(length_m / (v_mph * MPH * DT))
  s = np.arange(n) * v_mph * MPH * DT
  return s, np.where(s >= bend_at_m, k, 0.0)


def test_profile_measured_response():
  p = E.Profile(55, 40)
  assert p.t_arr == pytest.approx(1.5 + (55 - 41.5) / 1.36)
  t, v = p.at(p.d_react)
  assert v == 55 and t == pytest.approx(1.5)
  t, v = p.at(p.d_arr)
  assert v == pytest.approx(41.5) and t == pytest.approx(p.t_arr)
  t, v = p.at(p.d_arr + 41.5 * MPH * 2.0)
  assert v == pytest.approx(41.5) and t == pytest.approx(p.t_arr + 2.0)


def test_dwell_early_and_late():
  p = E.Profile(55, 40)
  k = 1.6 / (40 * MPH) ** 2                       # loads 1.6 m/s^2 at the target
  # bend 5 s (at the arrival speed) after the predicted arrival: dwell 5 s
  s, kap = _pass(55, p.d_arr + 5.0 * 41.5 * MPH, k)
  r = E.dwell(55, 40, s, kap, 0, s[-1], DT)
  assert r['dwell_s'] == pytest.approx(5.0, abs=0.1) and not r['late']
  # bend half-way through the deceleration: late, still well above the target there
  s, kap = _pass(55, (p.d_react + p.d_arr) / 2, k)
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
