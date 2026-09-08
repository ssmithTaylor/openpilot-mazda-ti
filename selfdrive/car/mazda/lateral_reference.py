"""Lane-supported release of retained corner references for the Mazda TI plant.

Geometry grants permission to withdraw commitment; it does not create a new path
or estimate the vehicle's physical steering limit. Positive lateral acceleration,
offset and heading here mean right, as in the torque controller's curvature frame.
"""
from dataclasses import dataclass, replace
import math

import numpy as np

MAX_CAMERA_AGE = 0.20
MIN_LANE_PROBABILITY = 0.8
EXCESS_ACCEL = 0.10
INWARD_SPEED = 0.10
INSIDE_OFFSET = 0.25
RELEASE_DWELL = 0.20
RELEASE_RATE = 1.0  # m/s^3, for removal and restoration
UNWIND_RATE = 2.0  # steering-wheel degrees/s
LANE_RELEASE_FLAG = 4096  # plantState: retained reference is being withdrawn


@dataclass(frozen=True)
class LaneContext:
  valid: bool = False
  offset: float = 0.0
  heading10: float = 0.0
  heading20: float = 0.0
  age: float = 0.0
  reason: str = "missing"
  curvature10: float = 0.0
  curvature20: float = 0.0


def fit_lane_context(model, age):
  def reject(reason):
    return LaneContext(age=age, reason=reason)

  if str(model.meta.laneChangeState) != "off":
    return reject("lane_change")
  if len(model.laneLines) < 3 or len(model.laneLineProbs) < 3:
    return reject("missing_lanes")
  if not all(math.isfinite(p) and MIN_LANE_PROBABILITY <= p <= 1.0
             for p in (model.laneLineProbs[1], model.laneLineProbs[2])):
    return reject("confidence")
  left, right = model.laneLines[1], model.laneLines[2]
  x = np.asarray(left.x, dtype=float)
  ly, ry = np.asarray(left.y, dtype=float), np.asarray(right.y, dtype=float)
  if len(x) < 3 or len(x) != len(ly) or len(x) != len(ry):
    return reject("samples")
  if not np.array_equal(x, np.asarray(right.x)):
    return reject("different_support")
  if not np.isfinite(np.r_[x, ly, ry]).all() or not np.all(np.diff(x) > 0):
    return reject("samples")
  if x[0] > 0 or x[-1] < 20:
    return reject("support")
  width = float(np.interp(0, x, ry - ly))
  if not 2.5 <= width <= 5.0:
    return reject("width")
  y = (ly + ry) / 2
  headings, curvatures = [], []
  for span in (10, 20):
    selected = (x >= 0) & (x <= span)
    if selected.sum() < 3:
      return reject("fit_support")
    try:
      coefficients = np.polynomial.polynomial.polyfit(x[selected], y[selected], 2)
    except np.linalg.LinAlgError:
      return reject("fit_failure")
    if not np.isfinite(coefficients).all():
      return reject("fit_nonfinite")
    headings.append(float(-math.atan(coefficients[1])))
    curvatures.append(float(2 * coefficients[2] / (1 + coefficients[1] ** 2) ** 1.5))
  if abs(headings[0] - headings[1]) > math.radians(0.5):
    return reject("fit_disagreement")
  return LaneContext(True, -float(np.interp(0, x, y)), *headings, age, "valid", *curvatures)


class LaneObserver:
  """Fit each consumed model once; recheck service validity and age every update."""
  def __init__(self):
    self.model_mono = None
    self.context = LaneContext()

  def update(self, model, model_mono, valid, now_ns, camera_now_ns=None):
    if model is None or not valid:
      return LaneContext(reason="invalid_event" if model is not None else "missing")
    # camerad EOF uses CLOCK_BOOTTIME; Python event identities use CLOCK_MONOTONIC.
    # Offline callers must supply aligned clocks or explicitly use the no-suspend assumption.
    camera_now_ns = now_ns if camera_now_ns is None else camera_now_ns
    age = (camera_now_ns - int(model.timestampEof)) / 1e9
    if not math.isfinite(age) or not 0 <= age <= MAX_CAMERA_AGE:
      return LaneContext(age=age, reason="camera_age")
    if model_mono <= 0 or model_mono > now_ns:
      return LaneContext(age=age, reason="model_time")
    if model_mono != self.model_mono:
      self.context = fit_lane_context(model, age)
      self.model_mono = model_mono
    return replace(self.context, age=age)


class LaneRelease:
  def __init__(self):
    self.reset()

  def reset(self):
    self.elapsed = [0.0, 0.0]
    self.removed = [0.0, 0.0]
    self.permitted = False
    self.motion_permitted = False
    self.position_permitted = False

  def update(self, current, delayed, ff, tracked, measurement, enabled, dt, context, speed, wheel_rate, latest):
    if not enabled:
      self.reset()
      return ff, tracked
    direction = math.copysign(1.0, current) if current * delayed > 0 else 0.0
    self.permitted = self.motion_permitted = self.position_permitted = False
    fresh_excess = direction * latest > 0 and direction * (measurement - latest) > EXCESS_ACCEL
    if context.valid and direction and fresh_excess:
      inward_v = min(direction * speed * math.sin(context.heading10), direction * speed * math.sin(context.heading20))
      self.motion_permitted = inward_v > INWARD_SPEED and direction * context.offset + 0.5 * inward_v >= 0
      road = [direction * curvature * speed ** 2 for curvature in (context.curvature10, context.curvature20)]
      self.position_permitted = (direction * context.offset >= INSIDE_OFFSET and inward_v >= -INWARD_SPEED and
                                 min(road) > 0 and direction * measurement - max(road) > EXCESS_ACCEL)
      self.permitted = self.motion_permitted or self.position_permitted

    out = []
    for i, (raw, committed) in enumerate(((current, ff), (delayed, tracked))):
      extra = max(0.0, direction * (committed - raw)) if raw * committed > 0 else 0.0
      excessive = direction and direction * (measurement - current) > EXCESS_ACCEL and direction * (measurement - raw) > EXCESS_ACCEL
      qualifies = bool(excessive and self.permitted)
      self.elapsed[i] = self.elapsed[i] + dt if qualifies else 0.0
      target = 0.0
      if qualifies and self.elapsed[i] >= RELEASE_DWELL - 1e-12:
        target = extra if direction * wheel_rate < UNWIND_RATE else min(extra, self.removed[i])
      step = RELEASE_RATE * dt
      self.removed[i] += max(-step, min(step, target - self.removed[i]))
      self.removed[i] = max(0.0, min(extra, self.removed[i]))
      out.append(committed - direction * self.removed[i])
    return tuple(out)
