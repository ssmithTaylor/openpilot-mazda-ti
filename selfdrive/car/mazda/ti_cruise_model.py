"""The stock cruise's MEASURED response to set-speed taps: the ONE cruise model (plan amendment A1, 2026-11-24).

Pure (no clock, no params, no messaging). Used by the curve planner's approach simulation and its warning's arrival prediction
(ti_curve_plan.py), the replay and early-dwell tools (tools/curve_slowing/replay.py, early_dwell.py) and every test scenario's
simulated car, so the planner, the warning and the test car can never again disagree about how the car slows.

Measured on this car (137 drives; E:/openpilot-mazda-ti-data/cruise-gap-20261124, q1_table.json, q1_sim.json, q2_summary.json):
- Settled deceleration against GAP (actual speed minus the dash set speed), medians per gap band, m/s^2. Two variants:
    "timing"        grade-corrected medians (used for timing: planner approach, warning arrival, replay, test cars)
    "conservative"  uncorrected medians, made non-decreasing (for safety checks; flattens near 0.6 from 6 mph)
  The 14+ mph band (5 events) is NOT extrapolated: the table is capped at the 10-14 mph value.
- Response to a gap change: DELAY_S of dead time on the gap, then a first-order lag of TAU_S on the deceleration.
- Dash walk under our taps: exactly 1 mph per tap, one tap per TAP_S (sometimes TAP_S_SLOW); 0 of 293 taps missed.
- Speeding back up toward a higher set speed: ACCEL_MPH_S (cruise-slowing-20261122: 5 mph in 7-9 s).
"""
import math
from collections import deque

MPH = 0.44704

DELAY_S = 0.5                    # dead time from a gap change to the cruise acting on it
TAU_S = 0.5                      # first-order time constant of the deceleration
TAP_S = 0.30                     # one of our taps per this: the dash walks 1 mph per tap
TAP_S_SLOW = 0.40                # ... sometimes this
ACCEL_MPH_S = 0.7                # speeding up toward a set speed above the speed
SETTLE_MPH = 0.3                 # within this of its set speed the cruise is taken to be on it (the dash shows whole mph)
ARRIVED_MPH = 1.5                # a manoeuvre is "done" within this of its target (the measured whole-manoeuvre definition)
RAISE_SLACK_MPH = 0.1            # after a push below the target: raise the dash back to it once the speed, raised NOW, would bottom
                                 # out within this of the target (plan A9-R2a: the car keeps slowing for the dead time plus the lag)

# gap nodes (mph): 0, 0.5, then the centres of the measured bands 1-2, 2-3, 3-4, 4-6, 6-8, 8-10, 10-14 (capped there)
GAP_MPH = (0.0, 0.5, 1.5, 2.5, 3.5, 5.0, 7.0, 9.0, 12.0)
DECEL_MS2 = {
  "timing": (0.0, 0.05, 0.21, 0.31, 0.41, 0.58, 0.64, 0.81, 0.90),
  "conservative": (0.0, 0.0, 0.01, 0.01, 0.17, 0.47, 0.57, 0.65, 0.65),
}
TIMING, CONSERVATIVE = "timing", "conservative"


def decel_ms2(gap_mph, variant=TIMING):
  """Settled deceleration (m/s^2, positive) the cruise gives for a gap (mph) between speed and set speed; 0 at no gap."""
  ys = DECEL_MS2[variant]
  g = float(gap_mph)
  if g <= 0.0:
    return 0.0
  if g >= GAP_MPH[-1]:
    return ys[-1]
  for i in range(1, len(GAP_MPH)):
    if g <= GAP_MPH[i]:
      x0, x1 = GAP_MPH[i - 1], GAP_MPH[i]
      return ys[i - 1] + (ys[i] - ys[i - 1]) * (g - x0) / (x1 - x0)
  return ys[-1]


def decel_mph_s(gap_mph, variant=TIMING):
  """The same in mph/s."""
  return decel_ms2(gap_mph, variant) / MPH


def dash_after(set0, goal, t, tap_s=TAP_S):
  """The dash set speed `t` seconds after the first of our taps walking from `set0` toward `goal` (first tap at t = 0)."""
  if t < 0.0 or set0 == goal:
    return set0
  n = int(math.floor(t / tap_s + 1e-9)) + 1
  step = min(n, int(round(abs(goal - set0))))
  return set0 + (step if goal > set0 else -step)


class Cruise:
  """Stepped car: speed (mph) answering the dash set speed with the measured dead time and lag. step(set_mph) -> speed.
  `scale` (default 1) multiplies the slowing only: the planner's warning passes the share of the model's deceleration the car is
  observed to give now (never above 1), the only correction to the model."""

  def __init__(self, v_mph, set_mph=None, dt=0.05, variant=TIMING, scale=1.0):
    self.dt = float(dt)
    self.variant = variant
    self.scale = float(scale)
    self.v = float(v_mph)
    s = self.v if set_mph is None else float(set_mph)
    gap = self.v - s
    self.lag = max(0, int(round(DELAY_S / self.dt)))
    self.gaps = deque([gap] * (self.lag + 1), maxlen=self.lag + 1)   # gap history: the cruise acts on the oldest
    self.a = self._demand(gap, s)        # mph/s, negative while slowing; assumed settled on the current gap
    self.alpha = 1.0 - math.exp(-self.dt / TAU_S)

  def _demand(self, gap, set_mph):
    if gap > 0.0:
      return -self.scale * decel_mph_s(gap, self.variant)
    if gap < -SETTLE_MPH:
      return ACCEL_MPH_S
    return 0.0

  def clone(self):
    c = Cruise.__new__(Cruise)
    c.__dict__.update(self.__dict__)
    c.gaps = deque(self.gaps, maxlen=self.gaps.maxlen)
    return c

  def step(self, set_mph):
    s = float(set_mph)
    self.gaps.append(self.v - s)
    g = self.gaps[0]
    self.a += (self._demand(g, s) - self.a) * self.alpha
    nv = self.v + self.a * self.dt
    if self.a < 0.0 and nv < s:           # never through its own set speed
      nv, self.a = min(self.v, s), 0.0
    elif self.a > 0.0 and nv > s:
      nv, self.a = max(self.v, s), 0.0
    self.v = max(0.0, nv)
    return self.v


def min_after_raise(car, set_now, T, tap_s=TAP_S):
  """The lowest speed (mph) the car `car` (a Cruise, not changed) reaches if our taps walk the dash from `set_now` up to T from now
  (one tap per tap_s, the first at once)."""
  c = car.clone()
  lo, t, horizon = c.v, 0.0, DELAY_S + 6.0 * TAU_S + max(0.0, T - set_now) * tap_s
  while t < horizon:
    v0 = c.v
    c.step(dash_after(set_now, T, t, tap_s))
    lo = min(lo, c.v)
    t += c.dt
    if c.v > v0 and t > DELAY_S:
      break                             # turned back up: the minimum is behind
  return lo


def raise_due(car, set_now, T, tap_s=TAP_S):
  """True when the dash, pushed below T (set_now < T), should be raised back to T now: the car would bottom out within
  RAISE_SLACK_MPH of T (plan A9-R2a: the moment comes from the model, not a fixed distance above T)."""
  return set_now < T and min_after_raise(car, set_now, T, tap_s) <= T + RAISE_SLACK_MPH


def manoeuvre(v0, T, S1=None, raise_at=None, arrived=ARRIVED_MPH, dt=0.05, variant=TIMING, tap_s=TAP_S, tmax=90.0, set0=None):
  """From the first tap, with the car settled at `v0` and the dash at `set0` (default `v0`; above v0: a car held below its set speed,
  e.g. by a lead, which needs set0 - T taps, not v0 - T, plan A8-R3): our taps walk the dash toward S1 (default: straight to T),
  switching the goal to T when `raise_due` says (or, given `raise_at`, once the speed is within it of T: the old fixed rule).
  -> (seconds, metres) until the speed is within `arrived` of T."""
  goal1 = T if S1 is None else S1
  s0 = v0 if set0 is None else set0
  car = Cruise(v0, s0, dt, variant)
  setn, tap_t, raised, t, x = int(round(s0)), 0.0, False, 0.0, 0.0
  while car.v > T + arrived and t < tmax:
    if not raised:
      raised = car.v <= T + raise_at if raise_at is not None else raise_due(car, setn, T, tap_s)
    goal = T if raised else goal1
    if t >= tap_t - 1e-9 and setn != goal:
      setn += 1 if goal > setn else -1
      tap_t = t + tap_s
    car.step(setn)
    x += car.v * MPH * dt
    t += dt
  return t, x


def speed_at(v_mph, d_m, set0, goal, lowering, dt=0.2, variant=TIMING, scale=1.0, tmax=120.0):
  """The speed (mph) after `d_m` metres: the cruise settled on its current gap (dash `set0`), our taps walking the dash to `goal`
  from now when `lowering` (else it stays at `set0`); `scale` as in Cruise."""
  car = Cruise(v_mph, set0, dt, variant, scale)
  x, t = 0.0, 0.0
  while x < d_m and t < tmax:
    s = dash_after(set0, goal, t) if lowering else set0
    car.step(s)
    x += car.v * MPH * dt
    t += dt
  return car.v
