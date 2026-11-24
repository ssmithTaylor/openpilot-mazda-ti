"""Predicted early dwell of a curve slowdown (off the car; owner, 2026-11-24: "curve slowing must arrive just in time").

For one slowdown the planner STARTED (car at `u` mph, corner target `T` mph): the speed it would follow with the cruise as MEASURED on
the drives 00000342 / 00000343 (arrival-20261124), not the planner's own assumptions: no change for DELAY_S after the first tap, then
RATE_MPH_S, done at T + ARRIVED_MPH (the car then holds that speed in this figure). Against it, the moment the recorded pass first
LOADS the car: the first sample from the start on where the pass's own curvature, at the target speed, needs LOAD_LA for at least
SUSTAIN_S. The dwell is the time from arrival to that moment along the predicted speed: positive is early (time spent slow before the
bend needs it), negative is LATE (the car would still be above T + ARRIVED_MPH when the bend loads; `over_mph` is how far above T).

Pure functions; unnecessary.py --dwell applies them to every slowdown it replays. Position-free.
"""
import math

import numpy as np

MPH = 0.44704
DELAY_S = 1.5             # measured: first tap to the car starting to slow, median 1.5 s (0.9-3.4)
RATE_MPH_S = 1.36         # measured: mean deceleration for drops of 8 mph or more
ARRIVED_MPH = 1.5         # arrived: at or below the target + this
LOAD_LA = 1.3             # the bend loads the car: |curvature| * (target speed)^2 at least this (m/s^2) ...
SUSTAIN_S = 0.5           # ... for at least this long


class Profile:
  """The measured-response speed after a start at u mph toward target T mph."""

  def __init__(self, u, T, delay_s=DELAY_S, rate=RATE_MPH_S, arrived=ARRIVED_MPH):
    self.u, self.goal, self.delay, self.rate = float(u), float(T) + arrived, delay_s, rate
    self.d_react = self.u * MPH * delay_s
    drop = max(0.0, self.u - self.goal)
    self.t_arr = (delay_s + drop / rate) if drop > 0 else 0.0
    self.d_arr = (self.d_react + MPH * (self.u ** 2 - self.goal ** 2) / (2.0 * rate)) if drop > 0 else 0.0

  def at(self, d):
    """(seconds, mph) when the car has driven d metres from the start."""
    if self.t_arr == 0.0:
      return d / max(self.u * MPH, 0.1), self.u
    if d <= self.d_react:
      return d / (self.u * MPH), self.u
    if d <= self.d_arr:
      v = math.sqrt(max(self.goal ** 2, self.u ** 2 - 2.0 * self.rate * (d - self.d_react) / MPH))
      return self.delay + (self.u - v) / self.rate, v
    return self.t_arr + (d - self.d_arr) / (self.goal * MPH), self.goal


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


def dwell(u, T, s, kap, i0, s_max, dt):
  """-> dict(dwell_s, over_mph, late) for a slowdown started at sample i0, or None when the pass never loads the car at T."""
  j = load_index(s, kap, T, i0, s_max, dt)
  if j is None:
    return None
  p = Profile(u, T)
  t_bend, v_bend = p.at(float(s[j] - s[i0]))
  dw = t_bend - p.t_arr
  return dict(dwell_s=round(dw, 2), over_mph=round(v_bend - T, 2), late=bool(dw < 0))


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
