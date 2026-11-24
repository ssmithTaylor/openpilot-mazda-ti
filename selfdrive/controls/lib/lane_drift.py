"""Lane-drift evidence for the steering alerts.

On this car the steering command sitting at its limit is normal on bends it holds fine, so "Take Control" and
"Running Wide" need evidence that the car is actually drifting toward or over a lane line, not the command alone.
Lane-line y values from the model are measured from the car's CENTRE (y positive to the right); the car's side is
CAR_HALF_WIDTH nearer the line than the centre is.
"""
import math

CAR_HALF_WIDTH = 0.92        # m
DRIFT_CLEARANCE = 0.30       # m: car's side within this of a visible line (or over it) counts as drifting
MIN_LINE_PROB = 0.5


def side_clearances(model):
  """(left, right) clearance in metres from the car's side to each visible lane line; None for a line not seen.
  Negative means the car's side is over the line."""
  try:
    lines, probs = model.laneLines, model.laneLineProbs
    left_y, right_y = lines[1].y[0], lines[2].y[0]
    left_p, right_p = probs[1], probs[2]
  except (AttributeError, IndexError, TypeError):
    return None, None
  left = (-left_y - CAR_HALF_WIDTH) if (math.isfinite(left_y) and left_p > MIN_LINE_PROB) else None
  right = (right_y - CAR_HALF_WIDTH) if (math.isfinite(right_y) and right_p > MIN_LINE_PROB) else None
  return left, right


def lane_drift_evidence(model):
  """True if a visible lane line is close to or under the car's side. Fails OPEN (True) when no line is
  visible, because then there is no evidence the car is holding its lane either."""
  left, right = side_clearances(model)
  seen = [c for c in (left, right) if c is not None]
  if not seen:
    return True
  return min(seen) < DRIFT_CLEARANCE


class HoldTimer:
  """Counts consecutive frames of a condition; fires once held for hold_s."""
  def __init__(self, hold_s, dt):
    self.hold_frames, self.dt, self.frames = hold_s / dt, dt, 0

  def step(self, held):
    self.frames = self.frames + 1 if held else 0
    return held and self.frames >= self.hold_frames - 1e-9
