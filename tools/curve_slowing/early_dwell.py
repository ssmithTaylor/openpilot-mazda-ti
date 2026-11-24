"""Predicted early dwell of a curve slowdown (off the car; owner, 2026-11-24: "curve slowing must arrive just in time").

For one slowdown the planner STARTED (car at `u` mph, corner target `T` mph): the speed it would follow with the cruise as MEASURED on
this car: the ONE measured cruise model (ti_cruise_model, plan amendment A1), the same the planner plans with, done at T +
ARRIVED_MPH (the car then holds that speed in this figure). Against it, the moment the recorded pass first
LOADS the car: the first sample from the start on where the pass's own curvature, at the target speed, needs LOAD_LA for at least
SUSTAIN_S. The dwell is the time from arrival to that moment along the predicted speed: positive is early (time spent slow before the
bend needs it), negative is LATE (the car would still be above T + ARRIVED_MPH when the bend loads; `over_mph` is how far above T).

Pure functions; unnecessary.py --dwell applies them to every slowdown it replays. Position-free.
"""
import math
import os
import sys

import numpy as np

try:
  from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
except ImportError:
  sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
  from selfdrive.car.mazda import ti_cruise_model as CM   # type: ignore

MPH = 0.44704
try:
  from openpilot.selfdrive.car.mazda import ti_curve_plan as _P
except ImportError:
  from selfdrive.car.mazda import ti_curve_plan as _P   # type: ignore
ARRIVED_MPH = _P.ARRIVED_MPH   # arrived: at or below the target + this (plan R11: the planner's own 0.5; was 1.5)
LOAD_LA = 1.3             # the bend loads the car: |curvature| * (target speed)^2 at least this (m/s^2) ...
SUSTAIN_S = 0.5           # ... for at least this long
ENTRY_SLACK_MPH = 1.0     # the measured episodes' "slowed": within this of the speed at the bend's entry (A5 comparison only)


class Profile:
  """The speed after a start at u mph toward target T mph, as the ONE measured cruise model says (ti_cruise_model: our taps walk the
  dash 1 mph per TAP_S, 0.5 s dead time, 0.5 s lag, the measured deceleration by gap): straight to T, or (S1 below T) the planner's
  own manoeuvre: a push to S1 and the model-timed raise back to T (CM.raise_due, as CM.manoeuvre judges it). Arrived at T + `arrived`;
  `at` holds the arrival speed after it (the dwell figure), `speed(t)` follows the model on (to T: the speed-lost figure). `set0`: the
  dash when the taps start (above u: a car held below its set speed)."""

  def __init__(self, u, T, arrived=ARRIVED_MPH, dt=0.05, S1=None, set0=None, tail_s=20.0):
    self.u, self.goal = float(u), float(T) + arrived
    s0 = self.u if set0 is None else float(set0)
    car = CM.Cruise(self.u, s0, dt)
    setn, tap_t, raised, chk = int(round(s0)), 0.0, S1 is None or S1 >= T, 0.0
    ts, xs, vs = [0.0], [0.0], [self.u]
    t, x, t_arr, d_arr, v_arr = 0.0, 0.0, None, None, None
    if self.u <= self.goal:
      t_arr, d_arr, v_arr = 0.0, 0.0, self.u
    while t < 120.0 and (t_arr is None or t < t_arr + tail_s):
      if not raised and t >= tap_t - 1e-9 and t >= chk - 1e-9 and car.v <= T + CM.RAISE_CHECK_MPH:
        raised, chk = CM.raise_due(car, setn, T), t + CM.RAISE_CHECK_S
      goal = int(T) if raised else int(S1)
      if t >= tap_t - 1e-9 and setn != goal:
        setn += 1 if goal > setn else -1
        tap_t = t + CM.TAP_S
      car.step(setn)
      t += dt
      x += car.v * MPH * dt
      ts.append(t)
      xs.append(x)
      vs.append(car.v)
      if t_arr is None and car.v <= self.goal:
        t_arr, d_arr, v_arr = t, x, car.v
    if t_arr is None:
      t_arr, d_arr, v_arr = t, x, car.v
    self.ts, self.xs, self.vs = np.array(ts), np.array(xs), np.array(vs)
    self.t_arr, self.d_arr, self.v_arr = t_arr, d_arr, v_arr
    na = int(np.searchsorted(self.ts, t_arr)) + 1
    self._xa, self._ta, self._va = self.xs[:na], self.ts[:na], self.vs[:na]

  def at(self, d):
    """(seconds, mph) when the car has driven d metres from the start (the arrival speed held after arrival)."""
    if self.t_arr == 0.0:
      return d / max(self.u * MPH, 0.1), self.u
    if d <= self.d_arr:
      return float(np.interp(d, self._xa, self._ta)), float(np.interp(d, self._xa, self._va))
    return self.t_arr + (d - self.d_arr) / (self.v_arr * MPH), self.v_arr

  def speed(self, t):
    """mph at t seconds from the start, the model followed on after arrival (held at its last value beyond the simulation)."""
    return np.interp(t, self.ts, self.vs)


def load_index(s, kap, T, i0, s_max, dt):
  """First index >= i0 (with s <= s_max) where |kap| * (T mph)^2 >= LOAD_LA for at least SUSTAIN_S; None if none."""
  n_sus = max(1, int(round(SUSTAIN_S / dt)))
  ok = (np.abs(np.asarray(kap, np.float64)) * (T * MPH) ** 2 >= LOAD_LA) & (np.asarray(s) <= s_max)
  ok[:i0] = False
  run = 0
  for j in np.nonzero(ok)[0] if ok.any() else []:
    run = run + 1 if run and ok[j - 1] else 1
    if run >= n_sus:
      return int(j - n_sus + 1)
  return None


def dwell(u, T, s, kap, i0, s_max, dt, S1=None, set0=None, i_from=None):
  """-> dict(dwell_s, over_mph, late, t_load) for a slowdown started at sample i0, or None when the pass never loads the car at T.
  The load is searched from sample i_from (default i0) on: the replay passes the planned bend's approach, so a bend the car is
  still in at the start, or one between the start and the planned bend, is not taken for it (plan A5)."""
  j = load_index(s, kap, T, i0 if i_from is None else max(i0, i_from), s_max, dt)
  if j is None:
    return None
  p = Profile(u, T, S1=S1, set0=set0)
  t_bend, v_bend = p.at(float(s[j] - s[i0]))
  dw = t_bend - p.t_arr
  # the measured episodes' definition (arrival-20261124/episodes.py), for the A5 check: from the first moment the (model) speed is
  # within ENTRY_SLACK_MPH of its speed at the load point to that point, along the model followed on (not held at arrival)
  d = float(s[j] - s[i0])
  if d <= p.xs[-1]:
    t2 = float(np.interp(d, p.xs, p.ts))
    v2 = float(p.speed(t2))
    k = int(np.argmax(p.vs <= v2 + ENTRY_SLACK_MPH))
    dm = round(max(0.0, t2 - float(p.ts[k])), 2)
  else:
    dm = None
  return dict(dwell_s=round(dw, 2), over_mph=round(v_bend - T, 2), late=bool(dw < 0), t_load=round(t_bend, 2), dwell_entry_s=dm)


def summary(rows):
  """rows: dicts from dwell() (None dropped). -> median / p10 / p90 dwell, late count, worst overshoot over T (mph)."""
  d = [r for r in rows if r is not None]
  if not d:
    return dict(n=0)
  x = np.array([r['dwell_s'] for r in d])
  late = [r for r in d if r['late']]
  return dict(n=len(d), p10=round(float(np.percentile(x, 10)), 1), median=round(float(np.median(x)), 1),
              p90=round(float(np.percentile(x, 90)), 1), late=len(late),
              worst_over_mph=round(max(r['over_mph'] for r in late), 1) if late else None)
