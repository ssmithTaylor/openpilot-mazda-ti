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
# Review I2 (2026-11-24): in a turn the evidence is taken on the OUTSIDE of the turn (the side the car runs wide toward), from the
# outside lane line OR the outside road edge, and fails OPEN when neither is seen with confidence, whatever the inside line shows.
# Turn direction: the sign of openpilot's desired curvature, RIGHT positive in this fork's convention (as the lane-line y: the
# device frame, y to the right; ti_curve_plan.MODEL_K_SIGN): curvature > 0 is a right turn (outside = LEFT), < 0 a left turn
# (outside = RIGHT). Below STRAIGHT_CURVATURE it is a straight: either line, as before.
STRAIGHT_CURVATURE = 2e-4    # 1/m (a 5 km radius)
MAX_EDGE_STD = 0.5           # m: a road edge is "seen with confidence" when the model's own std for it is at most this


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


def edge_clearances(model):
  """(left, right) clearance in metres from the car's side to each road edge seen with confidence (std <= MAX_EDGE_STD); None
  for an edge not seen. Negative means the car's side is past the edge."""
  try:
    edges, stds = model.roadEdges, model.roadEdgeStds
    left_y, right_y = edges[0].y[0], edges[1].y[0]
    left_s, right_s = stds[0], stds[1]
  except (AttributeError, IndexError, TypeError):
    return None, None
  left = (-left_y - CAR_HALF_WIDTH) if (math.isfinite(left_y) and math.isfinite(left_s) and left_s <= MAX_EDGE_STD) else None
  right = (right_y - CAR_HALF_WIDTH) if (math.isfinite(right_y) and math.isfinite(right_s) and right_s <= MAX_EDGE_STD) else None
  return left, right


def lane_drift_evidence(model, curvature=None):
  """True if the car's side is close to or over a line it may be running wide toward. In a turn (|curvature| at least
  STRAIGHT_CURVATURE, openpilot's desired curvature, RIGHT positive): the OUTSIDE lane line or the OUTSIDE road edge within
  DRIFT_CLEARANCE; FAILS OPEN (True) when neither is seen with confidence, whatever the inside line shows (review I2). On a straight
  (or curvature unknown): any visible line, failing open when none is visible (as before)."""
  left, right = side_clearances(model)
  try:
    k = float(curvature) if curvature is not None else None
  except (TypeError, ValueError):
    k = None
  if k is not None and math.isfinite(k) and abs(k) >= STRAIGHT_CURVATURE:
    e_left, e_right = edge_clearances(model)
    out = (left, e_left) if k > 0 else (right, e_right)      # right turn: runs wide to the LEFT; left turn: to the RIGHT
    # The model re-indexes laneLines[1]/[2] by the car's CENTRE: once the centre has passed the outside line, that line is reported
    # as the INSIDE-adjacent one and the "outside" line is a lane away, so looking only at the outside slot goes quiet exactly when
    # the car has crossed the line. So also take the inside-indexed line: when its clearance from the car's side is <= 0 (the line is
    # within half a car width of the centre, or on the other side of it) the car is straddling or past a line, which can only be the
    # one it ran wide over, so the gate is open. An inside line further than that is ignored, as before.
    inside = right if k > 0 else left
    if inside is not None and inside <= 0.0:
      return True
    seen = [c for c in out if c is not None]
    if not seen:
      return True
    return min(seen) < DRIFT_CLEARANCE
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
