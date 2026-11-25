"""tools/curve_slowing/unnecessary.py replay accounting (R13) on SYNTHETIC rows only (no recorded drive, no position)."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.car.mazda import ti_curve as TC
from openpilot.tools.curve_slowing import early_dwell as E
from openpilot.tools.curve_slowing import unnecessary as U

DT = U.DT
MPH = U.MPH


def test_car_side_constants_are_read_from_ti_curve_not_imported():
  # R13: the replay mirrors the car side's gas rules from ti_curve.py's own constants (read from its text)
  c = U.car_side_constants()
  assert c == {'GAS_MAX_S': TC.GAS_MAX_S, 'GAS_RESUME_S': TC.GAS_RESUME_S}


def _arr(n, on=()):
  a = np.zeros(n)
  for a0, a1 in on:
    a[int(round(a0 / DT)):int(round(a1 / DT))] = 1.0
  return a


def test_gate_short_gas_pauses_starts_long_gas_brake_disengage_stop():
  n = int(20 / DT)
  ones = np.ones(n)
  # 0.6 s of gas at 2 s: no start while pressed nor GAS_RESUME_S after; no stop (not held over GAS_MAX_S)
  can, stop = U.car_gate(_arr(n, [(2.0, 2.6)]), np.zeros(n), ones, ones)
  t = np.arange(n) * DT
  assert can[t < 2.0 - 1e-9].all() and not can[(t >= 2.0) & (t < 2.6 + 0.5 - 1e-9)].any() and can[t >= 3.1 + 1e-9].all()
  assert not stop.any()
  # gas held 2 s: stops once held over GAS_MAX_S
  can, stop = U.car_gate(_arr(n, [(5.0, 7.0)]), np.zeros(n), ones, ones)
  assert not stop[t < 6.0 + 1e-9].any() and stop[(t > 6.0 + 1e-9) & (t < 7.0 - 1e-9)].all() and not stop[t >= 7.0].any()
  # brake, openpilot disengaged, cruise off: stop and no start
  for args in ((np.zeros(n), _arr(n, [(1, 2)]), ones, ones), (np.zeros(n), np.zeros(n), 1 - _arr(n, [(1, 2)]), ones),
               (np.zeros(n), np.zeros(n), ones, 1 - _arr(n, [(1, 2)]))):
    can, stop = U.car_gate(*args)
    w = (t >= 1.0) & (t < 2.0 - 1e-9)
    assert stop[w].all() and not can[w].any() and not stop[~w].any()


def _row(T=30, rel_at=None):
  x = dict(stage=1, run=0, i=0, u=45.0, T=T, floor=None, ent=600.0, end=750.0, aim=600.0, key='7')
  if rel_at is not None:
    x['i_dec'], x['rel'] = int(round(rel_at / DT)), True
  return x


def _run(gas=(), brake=(), seconds=40.0):
  n = int(seconds / DT)
  v = np.full(n, 45.0 * MPH)
  s = np.r_[0.0, np.cumsum(v[:-1] * DT)]
  can, stop = U.car_gate(_arr(n, gas), _arr(n, brake), np.ones(n), np.ones(n))
  return v, s, np.zeros(n), can, stop


PM = NS(DRIVE_GAP_MPH=6)                     # the planner of c9af11c (no R16 cap): a full ramp from the start


def test_slowdown_started_on_the_gas_is_not_scored():
  v, s, kap, can, stop = _run(gas=[(0.0, 40.0)])
  x = U.account_row(_row(), PM, s, v, kap, can, stop, 750.0, {})
  assert x['acts'] is False and x['lost'] == 0.0 and x['shed'] == 0.0


def test_slowdown_started_on_the_gas_acts_from_resume():
  # the car side taps toward the ceiling once the gas is released + GAS_RESUME_S (here a short press: 0.8 s)
  v, s, kap, can, stop = _run(gas=[(0.0, 0.8)])
  x = U.account_row(_row(), PM, s, v, kap, can, stop, 750.0, {})
  assert x['acts'] and x['defer_s'] == pytest.approx(0.8 + 0.5, abs=DT + 1e-6)
  v2, s2, k2, c2, st2 = _run()
  y = U.account_row(_row(), PM, s2, v2, k2, c2, st2, 750.0, {})
  assert y['defer_s'] == 0.0 and x['lost'] < y['lost']          # less road left after the deferral: less of the ramp before the end


def test_released_stage1_scored_only_up_to_its_release():
  # R13: a stage-1 bend released at 3 s sheds only what the model gives by then (plus its dead time and lag), not its target
  v, s, kap, can, stop = _run()
  x = U.account_row(_row(rel_at=3.0), PM, s, v, kap, can, stop, 750.0, {})
  full = U.account_row(_row(), PM, s, v, kap, can, stop, 750.0, {})
  prof = E.Profile(45.0, 30, S1=30, stop_t=3.0)   # floor 45 - 15 binds the push
  assert x['shed'] == pytest.approx(prof.shed(60.0), abs=0.11)
  assert 0.0 < x['shed'] < 5.0 and x['T'] == pytest.approx(45.0 - x['shed'], abs=0.11) and x['T_target'] == 30
  assert full['T'] == 30 and full['shed'] > 14.0 and x['lost'] < 0.3 * full['lost']
  # released before the car side acts on it (here it was on the gas until then): nothing at all
  v, s, kap, can, stop = _run(gas=[(0.0, 5.0)])
  x = U.account_row(_row(rel_at=3.0), PM, s, v, kap, can, stop, 750.0, {})
  assert x['acts'] is False


def test_gas_held_or_brake_mid_slowdown_stops_it():
  v, s, kap, can, stop = _run(gas=[(4.0, 8.0)])
  x = U.account_row(_row(), PM, s, v, kap, can, stop, 750.0, {})
  prof = E.Profile(45.0, 30, S1=30, stop_t=4.0 + 1.0 + DT)       # stopped once the gas is held over GAS_MAX_S
  assert x['shed'] == pytest.approx(prof.shed(60.0), abs=0.11) and x['T'] > 30
  v, s, kap, can, stop = _run(brake=[(2.0, 3.0)])
  y = U.account_row(_row(), PM, s, v, kap, can, stop, 750.0, {})
  assert y['shed'] < x['shed'] and y['T'] == pytest.approx(45.0 - y['shed'], abs=0.11)


def test_profile_stop_walks_back_up_and_tap_cycle():
  p = E.Profile(45.0, 35, stop_t=2.0)
  assert p.vs[-1] == pytest.approx(45.0, abs=0.1) and 0.0 < p.shed(1e9) < 3.0
  # R17: a faster tap cycle (0.20 s) sheds more in the same seconds
  a, b = E.Profile(45.0, 40), E.Profile(45.0, 40, tap_s=0.20)
  assert b.shed(3.0) > a.shed(3.0) and b.t_arr < a.t_arr
  # unchanged by default
  c = E.Profile(45.0, 30, S1=24)
  assert c.t_arr == pytest.approx(E.Profile(45.0, 30, S1=24, cap=None, stop_t=None, tap_s=None).t_arr)


def test_options_switch_min_passes_and_unconfirmed_cap():
  # Job 1 measurement: the replay switches S1_MIN_PASSES and S1_UNCONFIRMED_MAX_MPH (None = no cap) in a stand-in module only
  m = NS(S1_MIN_PASSES=1, S1_UNCONFIRMED_MAX_MPH=5, CONFIRM_SHARE=0.6, MIN_DROP_MPH=2)
  U.apply_options(m)
  assert (m.S1_MIN_PASSES, m.S1_UNCONFIRMED_MAX_MPH) == (1, 5)          # nothing given: unchanged
  U.apply_options(m, min_passes=3, unconfirmed_cap=None)
  assert (m.S1_MIN_PASSES, m.S1_UNCONFIRMED_MAX_MPH) == (3, None)
  U.apply_options(m, unconfirmed_cap=5)
  assert m.S1_UNCONFIRMED_MAX_MPH == 5


def test_fix_default_is_the_snapped_point_raw_option_feeds_the_raw_fix():
  # road matching (2026-11-24): by default the planner gets the governor's snapped memory point (unchanged); with --raw-fixes the
  # raw position of the same sample, and no fix where the raw one is invalid. Synthetic numbers only
  P = U.P
  mem = NS(x=np.array([10.0, 20.0]), y=np.array([1.0, 2.0]), hdg=np.array([0.1, 0.2]))
  r = {'pt': np.array([1, -1])}
  assert U.fix_at(P, mem, r, 0) == P.Fix(True, 20.0, 2.0, 0.2)
  assert U.fix_at(P, mem, r, 1) == P.Fix(False)
  raw = dict(X=np.array([21.5, 0.0]), Y=np.array([-3.0, 0.0]), hdg=np.array([0.25, 0.0]), ok=np.array([True, False]))
  assert U.fix_at(P, mem, r, 0, raw) == P.Fix(True, 21.5, -3.0, 0.25)
  assert U.fix_at(P, mem, r, 1, raw) == P.Fix(False)
  assert '--raw-fixes' in open(U.__file__).read()


def test_replay_push_uses_the_planner_push_floor():
  # hold-branch merge: the replay's stage-1 push goes as deep as the planner's (CM.push_floor: below the 15 mph line by the push
  # depth, never below 25), not clamped at the floor
  from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
  pm = NS(DRIVE_GAP_MPH=6, CM=CM)
  v, s, kap, can, stop = _run()
  x = U.account_row(_row(), pm, s, v, kap, can, stop, 750.0, {})
  y = U.account_row(_row(), PM, s, v, kap, can, stop, 750.0, {})
  assert CM.push_floor(30, 6) == 25 and x['T'] == y['T'] == 30
  # the deeper push reaches T sooner, so slightly more time is spent slowed before the bend's end (862.1 vs 850.9 mph-s here)
  assert x['shed'] == pytest.approx(y['shed'], abs=0.6) and x['lost'] != y['lost']
  assert E.Profile(45.0, 30, S1=25).t_arr < E.Profile(45.0, 30, S1=30).t_arr
