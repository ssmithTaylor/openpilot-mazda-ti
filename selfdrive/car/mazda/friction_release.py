"""Withdraw existing friction compensation once per qualified easing episode.

Acceleration and compensation are right-positive; wheel rate is the vehicle's
left-positive signal, so a positive rate unwinds a right turn. Geometry permission
and current-request dwell come from LaneRelease. This helper does not choose a path.
"""
import math

from openpilot.selfdrive.car.mazda.lateral_reference import EXCESS_ACCEL, RELEASE_DWELL, UNWIND_RATE

WITHDRAWAL_RATE = 340.0  # counts/s: at most the existing 85-count compensation in 0.25s


class FrictionRelease:
  def __init__(self):
    self.reset()

  def reset(self):
    self.removed = 0.0
    self.direction = 0.0
    self.completed = False

  def update(self, compensation, current, latest, measurement, permitted, dwell, wheel_rate, dt):
    direction = math.copysign(1.0, current)
    if current * latest <= 0:
      self.reset()
      return compensation
    if direction != self.direction:
      self.reset()
      self.direction = direction
    if direction * (latest - measurement) > EXCESS_ACCEL:
      self.completed = False
    if direction * (measurement - latest) <= EXCESS_ACCEL or direction * compensation <= 0:
      self.removed = 0.0
      return compensation
    unwind = direction * wheel_rate >= UNWIND_RATE
    if self.removed > 0 and unwind:
      self.completed = True
    eligible = permitted and dwell >= RELEASE_DWELL - 1e-12 and not self.completed and not unwind
    target = abs(compensation) if eligible else 0.0
    step = WITHDRAWAL_RATE * dt
    self.removed += max(-step, min(step, target - self.removed))
    self.removed = min(abs(compensation), max(0.0, self.removed))
    return compensation - direction * self.removed
