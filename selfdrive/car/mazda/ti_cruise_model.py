"""The stock cruise's MEASURED response to set-speed taps: the ONE cruise model (plan amendment A1, 2026-11-24).

Pure (no clock, no params, no messaging). Used by the curve planner's approach simulation and its warning's arrival prediction
(ti_curve_plan.py), the replay and early-dwell tools (tools/curve_slowing/replay.py, early_dwell.py) and every test scenario's
simulated car, so the planner, the warning and the test car can never again disagree about how the car slows.

Measured on this car (137 drives; E:/openpilot-mazda-ti-data/cruise-gap-20261124, q1_table.json, q1_sim.json, q2_summary.json):
- Settled deceleration against GAP (actual speed minus the dash set speed), medians per gap band, m/s^2. Two variants:
    "timing"        grade-corrected medians (used for timing: planner approach, warning arrival, replay, test cars)
    "conservative"  uncorrected medians, made non-decreasing (for safety checks; flattens near 0.6 from 6 mph)
  The 14+ mph band (5 events, 1.21 m/s^2) is in the TIMING variant (owner, 2026-11-24: trust that braking keeps growing beyond a
  14 mph gap and plan around it; plan A10), flat beyond 16 mph; the conservative variant is unchanged (flat from the 10-14 band).
- Response to a gap change: DELAY_S of dead time on the gap, then a first-order lag of TAU_S on the deceleration.
- Dash walk under our taps: exactly 1 mph per tap, one tap per TAP_S (sometimes TAP_S_SLOW); 0 of 293 taps missed.
- Speeding back up toward a higher set speed: ACCEL_MPH_S (cruise-slowing-20261122: 5 mph in 7-9 s).
- A HELD button (plan A7; 349 driver holds, q2_trains.pkl): 1 mph at HOLD_FIRST_S, then the next multiple of 5 mph beyond that at
  HOLD_LAND_S (landed on a multiple of 5 in 346 of 349), then HOLD_STEP_MPH every HOLD_STEP_S. Our feature holds only where
  `hold_landing` allows it (5 mph or more to go, no landing past the goal, the press capped at ti_cruise_taps.HOLD_MAX_S) and
  finishes with taps. Whether the car reads OUR held frames as a hold is not yet shown: ti_cruise_taps.HOLD_ENABLED (off).
"""
# RUNTIME: runs in card at 100 Hz (gc disabled) and ti_curve_plannerd at 5 Hz (gc frozen) (see docs/RUNTIME_CONSTRAINTS.md).
# Per call: create no class or closure, keep no container without a bound; never a position in a log or param.
import math
from collections import deque

from openpilot.selfdrive.car.mazda import ti_cruise_taps as TT

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
RAISE_CHECK_S = 0.05             # manoeuvre: the raise is judged only when a tap could go out, at most this often (A8-R6 cost) ...
RAISE_CHECK_MPH = 5.0            # ... and only within this of the target (raises found at T + 1.1 .. 4.5 for 25-79 mph, 2-15 off)

# held button (plan A7, measured on 349 driver holds)
HOLD_FIRST_S = 0.05             # the first 1 mph
HOLD_LAND_S = 0.55              # ... then the next multiple of 5 beyond it
HOLD_STEP_S = 0.60              # ... then HOLD_STEP_MPH per this
HOLD_STEP_MPH = 5
HOLD_MIN_MPH = 5                # a hold only with at least this far to go
HOLD_READ_S = 0.15              # a landing is read back (and the press released) within this; the press must fit TT.HOLD_MAX_S
HOLD = TT.HOLD_ENABLED          # the walk the planner plans with: held where allowed, else taps

# floors of a push (plan A10; owner, 2026-11-24: "I'm okay with setting cruise lower than the 15-mile-per-hour limit. I'm less okay
# with actually making the car go 15 miles per hour less than the limit."): the 15 mph maximum drop applies to the TARGET (what the
# car is slowed to); a transient push may take the DASH below it by the push depth, never below these. The car's actual speed keeps
# the floor as a hard cap: the planner raises in time (raise_due), and the car side raises a dash left below the floor by itself
# (ti_curve "push net") whatever the planner does (test_mazda_ti_curve_push.py, both braking tables).
PUSH_BELOW_MAX_DROP = True
PUSH_ABS_FLOOR_MPH = 25         # = ti_curve.ABS_FLOOR_MPH (pinned by a test)
PUSH_CAR_MIN_MPH = 19           # the car's own lowest set speed

# gap nodes (mph): 0, 0.5, then the centres of the measured bands 1-2, 2-3, 3-4, 4-6, 6-8, 8-10, 10-14, 14+ (timing: flat beyond 16)
GAP_MPH = (0.0, 0.5, 1.5, 2.5, 3.5, 5.0, 7.0, 9.0, 12.0, 16.0)
DECEL_MS2 = {
  "timing": (0.0, 0.05, 0.21, 0.31, 0.41, 0.58, 0.64, 0.81, 0.90, 1.21),
  "conservative": (0.0, 0.0, 0.01, 0.01, 0.17, 0.47, 0.57, 0.65, 0.65, 0.65),
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


def push_floor(floor, gap):
  """The lowest dash set speed a transient push `gap` below its target may use, the target's own floor being `floor` (plan A10):
  floor - gap, never below PUSH_ABS_FLOOR_MPH / PUSH_CAR_MIN_MPH; the floor itself when PUSH_BELOW_MAX_DROP is off."""
  if not PUSH_BELOW_MAX_DROP:
    return floor
  return max(PUSH_ABS_FLOOR_MPH, PUSH_CAR_MIN_MPH, int(floor) - int(gap))


def hold_first_landing(set0, direction):
  """The multiple of 5 a hold from `set0` lands on first: the next one beyond set0 + direction (measured: 45 -> 44 -> 40,
  46 -> 45 -> 40, 47 -> 46 -> 45; up: 45 -> 46 -> 50, 49 -> 50 -> 55)."""
  s1 = int(set0) + direction
  return ((s1 - 1) // 5) * 5 if direction < 0 else (s1 // 5 + 1) * 5


def hold_landing(set0, goal):
  """Where a hold from `set0` toward `goal` is released: (stop, seconds from its first frame to reading `stop`), or None when no hold
  is allowed (under HOLD_MIN_MPH to go, or the first landing past the goal). `stop` is the last landing not past the goal whose press
  still fits TT.HOLD_MAX_S; the rest is walked with taps."""
  set0, goal = int(round(set0)), int(round(goal))
  if abs(goal - set0) < HOLD_MIN_MPH:
    return None
  d = 1 if goal > set0 else -1
  L1 = hold_first_landing(set0, d)
  if d * (L1 - goal) > 0:
    return None
  k = 1
  while d * (L1 + d * HOLD_STEP_MPH * k - goal) <= 0 and HOLD_LAND_S + HOLD_STEP_S * k + HOLD_READ_S <= TT.HOLD_MAX_S + 1e-9:
    k += 1
  return L1 + d * HOLD_STEP_MPH * (k - 1), HOLD_LAND_S + HOLD_STEP_S * (k - 1)


def _hold_dash(set0, d, t, stop):
  if t < HOLD_FIRST_S - 1e-9:
    return set0
  if t < HOLD_LAND_S - 1e-9:
    return set0 + d
  k = int(math.floor((t - HOLD_LAND_S) / HOLD_STEP_S + 1e-9)) + 1
  v = hold_first_landing(set0, d) + d * HOLD_STEP_MPH * (k - 1)
  return max(v, stop) if d < 0 else min(v, stop)


class Walk:
  """The dash set speed while our presses walk it from `set0` to `goal`, starting at `t0`: taps (the first at t0, one per tap_s) or,
  with `hold` and where `hold_landing` allows, a hold released at its stop, then taps (the first tap_s after the stop is read)."""

  def __init__(self, set0, goal, t0=0.0, tap_s=TAP_S, hold=False):
    self.s0, self.goal, self.t0, self.tap_s = int(round(set0)), int(round(goal)), float(t0), float(tap_s)
    self.h = hold_landing(self.s0, self.goal) if hold else None

  def at(self, t):
    dt = t - self.t0
    if dt < -1e-9 or self.s0 == self.goal:
      return self.s0
    d = 1 if self.goal > self.s0 else -1
    if self.h is None:
      n = int(math.floor(dt / self.tap_s + 1e-9)) + 1
      return self.s0 + d * min(n, abs(self.goal - self.s0))
    stop, t_stop = self.h
    if dt < t_stop - 1e-9:
      return _hold_dash(self.s0, d, dt, stop)
    n = int(math.floor((dt - t_stop) / self.tap_s + 1e-9))
    return stop + d * min(max(n, 0), abs(self.goal - stop))

  def next_t(self, t):
    """The earliest time at or after `t` a new press may start (never inside the hold; on the walk's own press pace)."""
    if self.s0 == self.goal:
      return t                          # nothing pressed
    base = self.t0 if self.h is None else self.t0 + self.h[1]
    k = max(1, int(math.ceil((t - base) / self.tap_s - 1e-9)))
    return base + k * self.tap_s


def dash_after(set0, goal, t, tap_s=TAP_S, hold=False):
  """The dash set speed `t` seconds after the first of our presses walking from `set0` toward `goal` (first press at t = 0): taps,
  or with `hold` the held button where `hold_landing` allows it, then taps."""
  if t < 0.0 or set0 == goal:
    return set0
  if hold:
    return Walk(set0, goal, 0.0, tap_s, True).at(t)
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


def min_after_raise(car, set_now, T, tap_s=TAP_S, hold=False, delay_s=0.0):
  """The lowest speed (mph) the car `car` (a Cruise, not changed) reaches if our presses walk the dash from `set_now` up to T from
  now (one tap per tap_s, the first at once, or `delay_s` from now; with `hold`, held where `hold_landing` allows)."""
  c = car.clone()
  lo, t, horizon = c.v, 0.0, DELAY_S + 6.0 * TAU_S + max(0.0, T - set_now) * tap_s + delay_s
  while t < horizon:
    v0 = c.v
    c.step(dash_after(set_now, T, t - delay_s, tap_s, hold))
    lo = min(lo, c.v)
    t += c.dt
    if c.v > v0 and t > DELAY_S + delay_s:
      break                             # turned back up: the minimum is behind
  return lo


def raise_due(car, set_now, T, tap_s=TAP_S, hold=False):
  """True when the dash, pushed below T (set_now < T), should be raised back to T now: the car would bottom out within
  RAISE_SLACK_MPH of T (plan A9-R2a: the moment comes from the model, not a fixed distance above T)."""
  return set_now < T and min_after_raise(car, set_now, T, tap_s, hold) <= T + RAISE_SLACK_MPH


def _held_run(car, set0, goal1, T, tap_s, done, raise_at=None, tmax=90.0):
  """The push-and-raise manoeuvre with the held button where allowed (plan A7 / A10): a Walk toward goal1, switched to a Walk up to
  T when `raise_due` (with the hold) says, judged when a new press could start, every RAISE_CHECK_S at most, within RAISE_CHECK_MPH of
  T (or, given raise_at, once within it of T). `done(car, x, t)` ends it. -> (t, x, car, lowest speed after the raise)."""
  dt = car.dt
  walk = Walk(set0, goal1, 0.0, tap_s, True)
  raised, t, x, chk, lo = goal1 == T, 0.0, 0.0, 0.0, None
  while not done(car, x, t) and t < tmax:
    dash = walk.at(t)
    if not raised and t >= walk.next_t(t) - 1e-9 and t >= chk - 1e-9:
      if raise_at is not None:
        raised = car.v <= T + raise_at
      elif car.v <= T + RAISE_CHECK_MPH:
        raised, chk = raise_due(car, dash, T, tap_s, True), t + RAISE_CHECK_S
      if raised:
        walk = Walk(dash, T, t, tap_s, True)
        dash = walk.at(t)
    car.step(dash)
    if raised:
      lo = car.v if lo is None else min(lo, car.v)
    x += car.v * MPH * dt
    t += dt
  return t, x, car, lo


def manoeuvre(v0, T, S1=None, raise_at=None, arrived=ARRIVED_MPH, dt=0.05, variant=TIMING, tap_s=TAP_S, tmax=90.0, set0=None,
              hold=False):
  """From the first tap, with the car settled at `v0` and the dash at `set0` (default `v0`; above v0: a car held below its set speed,
  e.g. by a lead, which needs set0 - T taps, not v0 - T, plan A8-R3): our taps walk the dash toward S1 (default: straight to T),
  switching the goal to T when `raise_due` says (judged when a tap could go out, every RAISE_CHECK_S at most, within
  RAISE_CHECK_MPH of T) or, given `raise_at`, once the speed is within it of T (the old fixed rule). With `hold`: the held button
  where `hold_landing` allows it, then taps (`_held_run`); without, exactly the taps-only walk.
  -> (seconds, metres) until the speed is within `arrived` of T."""
  goal1 = T if S1 is None else S1
  s0 = v0 if set0 is None else set0
  car = Cruise(v0, s0, dt, variant)
  if hold:
    t, x, _, _ = _held_run(car, s0, goal1, T, tap_s, lambda c, x_, t_: c.v <= T + arrived, raise_at, tmax)
    return t, x
  setn, tap_t, raised, t, x, chk = int(round(s0)), 0.0, False, 0.0, 0.0, 0.0
  while car.v > T + arrived and t < tmax:
    if not raised:
      if raise_at is not None:
        raised = car.v <= T + raise_at
      elif t >= tap_t - 1e-9 and t >= chk - 1e-9 and car.v <= T + RAISE_CHECK_MPH:
        raised, chk = raise_due(car, setn, T, tap_s), t + RAISE_CHECK_S
    goal = T if raised else goal1
    if t >= tap_t - 1e-9 and setn != goal:
      setn += 1 if goal > setn else -1
      tap_t = t + tap_s
    car.step(setn)
    x += car.v * MPH * dt
    t += dt
  return t, x


def speed_at(v_mph, d_m, set0, goal, lowering, dt=0.2, variant=TIMING, scale=1.0, tmax=120.0, T=None, tap_s=TAP_S, hold=False):
  """The speed (mph) after `d_m` metres: the cruise settled on its current gap (dash `set0`), our taps walking the dash to `goal`
  from now when `lowering` (else it stays at `set0`); `scale` as in Cruise. With T (> goal, lowering): the planner's own manoeuvre
  (plan A11): the push toward `goal` and the walk back up to T when `raise_due` says, judged as in `manoeuvre`. With `hold`: held
  where `hold_landing` allows (as `manoeuvre`)."""
  car = Cruise(v_mph, set0, dt, variant, scale)
  x, t = 0.0, 0.0
  if hold and lowering and T is not None and goal < T:
    _, _, car, _ = _held_run(car, set0, goal, T, tap_s, lambda c, x_, t_: x_ >= d_m, None, tmax)
    return car.v
  if T is None or not lowering or goal >= T:
    while x < d_m and t < tmax:
      s = dash_after(set0, goal, t, hold=hold) if lowering else set0
      car.step(s)
      x += car.v * MPH * dt
      t += dt
    return car.v
  setn, tap_t, raised, chk = int(round(set0)), 0.0, False, 0.0
  while x < d_m and t < tmax:
    if not raised and t >= tap_t - 1e-9 and t >= chk - 1e-9 and car.v <= T + RAISE_CHECK_MPH:
      raised, chk = raise_due(car, setn, T, tap_s), t + RAISE_CHECK_S
    g = T if raised else goal
    if t >= tap_t - 1e-9 and setn != g:
      setn += 1 if g > setn else -1
      tap_t = t + tap_s
    car.step(setn)
    x += car.v * MPH * dt
    t += dt
  return car.v
