"""Lane-drift evidence for the steering alerts.

On this car the steering command sitting at its limit is normal on bends it holds fine, so "Take Control" and
"Running Wide" need evidence that the car is actually drifting toward or over a lane line, not the command alone.
Lane-line y values from the model are measured from the car's CENTRE (y positive to the right); the car's side is
CAR_HALF_WIDTH nearer the line than the centre is.
"""
# RUNTIME: runs in controlsd at 100 Hz (while steering) and frogpilot_process at 20 Hz (every model frame on-road); gc disabled (see docs/RUNTIME_CONSTRAINTS.md).
# Per call: create no class or closure, keep no container without a bound, nothing that grows with drive time.
import math

CAR_HALF_WIDTH = 0.92        # m
DRIFT_CLEARANCE = 0.30       # m: car's side within this of a visible line (or over it) counts as drifting
MIN_LINE_PROB = 0.5
KEEP_LINE_PROB = 0.3         # a line that was confident (> MIN_LINE_PROB) stays so down to this while its position is consistent (gate only)
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


DEFAULT_LANE_WIDTH = 3.2     # m: median of the 23,850 ticks of the 2026-11-24 night drive (route 00000346) with both lines seen (IQR 3.07-3.26)
MIN_LANE_WIDTH, MAX_LANE_WIDTH = 2.4, 4.6   # m: a both-lines-seen width outside this is a model glitch (or a merge), not remembered
TURN_CLEARANCE = DRIFT_CLEARANCE            # m: 0.6 m was replayed on the 2026-11-24 night drive: 0.6 s earlier on one crossing, 2 extra short alerts; not adopted
LATCH_S = 1.5                # s: gate stays open this long after its last open tick while the steering stays at its limit (0.7 s Take Control hold + re-index)
LATCH_RELEASE_S = 0.3        # s: ... and is released once the steering has been clear of its limit this long
REINDEX_JUMP_FRAC = 0.6      # a lane line moving more than this fraction of the lane width between ticks is a re-index, not a car movement

# Predictive opening (2026-11-24, lanegate round 3). Position alone (within 0.30 m) cannot beat a fast drift with the 0.7 s Take Control
# hold: on the night drive's two hard-bend crossings the alert came 0.4-0.5 s AFTER the car's side was over the line. So in a turn at the
# steering limit the gate also opens when the car's side will reach the outside line within PREDICT_HORIZON_S at its present closing rate.
# Closing rate = least-squares slope of the outside clearance over the last RATE_WINDOW_S of model frames (NOT a tick difference).
# Line-position noise measured on the night drive (lines seen, at the limit): sd 0.04-0.05 m per frame (frame-to-frame difference sd
# 0.069 m), so a frame difference at 20 Hz would be +-1.4 m/s; a 0.6 s (13 frame) slope has sd 0.05*sqrt(12/(13*168))/0.05 s = 0.07 m/s,
# and PREDICT_MIN_RATE of 0.30 m/s is over 4 sd of it: noise alone cannot open the gate. Steady clearance (rate 0) and slow drifts below
# 0.30 m/s never predict; they reach the positional rule as before. Horizon = the 0.7 s hold + 0.8 s allowance for the driver (the logged
# drivers braked 0.4-0.5 s after the alert came).
RATE_WINDOW_S = 0.6
RATE_MIN_FRAMES = 8
PREDICT_MIN_RATE = 0.30      # m/s
PREDICT_HORIZON_S = 2.5      # s: 0.7 hold + 0.5 s lead wanted + ~1.3 s for the slope estimate lagging the true rate (replayed: 1.5 s gave no lead, 2.0 s ~0, 2.5 s 0.55-0.65 s)


def lane_drift_evidence(model, curvature=None, lane_width=None):
  return _evidence(model, curvature, lane_width)[0]


def _evidence(model, curvature, lane_width):
  """(open, referenced): referenced is False when the gate is open only because nothing gave a position (fail open). True if the car's side is close to or over a line it may be running wide toward. In a turn (|curvature| at least
  STRAIGHT_CURVATURE, openpilot's desired curvature, RIGHT positive), first available of:
    1. the OUTSIDE lane line seen with confidence: within DRIFT_CLEARANCE of the car's side; or the INSIDE-indexed line straddled or
       passed (clearance <= 0: the model re-indexes lines by the car's centre, so a crossed outside line shows up as the inside one);
    2. the OUTSIDE road edge seen with confidence (std <= MAX_EDGE_STD): within DRIFT_CLEARANCE;
    3. the INSIDE lane line only (outside unseen: an unseen line is not evidence of running wide): the car is wide when its side is
       further from the inside line than lane_width - car width - DRIFT_CLEARANCE (lane_width: last confidently seen, else DEFAULT);
    4. nothing gives a position: fail OPEN (True).
  On a straight (or curvature unknown): any visible line, failing open when none is visible (as before)."""
  left, right = side_clearances(model)
  try:
    k = float(curvature) if curvature is not None else None
  except (TypeError, ValueError):
    k = None
  if k is not None and math.isfinite(k) and abs(k) >= STRAIGHT_CURVATURE:
    e_left, e_right = edge_clearances(model)
    out_line, out_edge = (left, e_left) if k > 0 else (right, e_right)   # right turn: runs wide to the LEFT; left turn: to the RIGHT
    inside = right if k > 0 else left
    if out_line is not None:
      # (the re-index case) inside <= 0 = the car straddles / has passed the line that is now indexed on the inside
      return out_line < TURN_CLEARANCE or (inside is not None and inside <= 0.0), True
    if out_edge is not None:
      return out_edge < TURN_CLEARANCE or (inside is not None and inside <= 0.0), True
    if inside is not None:
      width = lane_width if lane_width else DEFAULT_LANE_WIDTH
      # inside is the car side's distance to the inside line; <= 0: the car straddles / is over it (the crossed line, re-indexed
      # inside, with the far line unseen -- review I2)
      return inside > width - 2 * CAR_HALF_WIDTH - DRIFT_CLEARANCE or inside <= 0.0, True
    return True, False
  seen = [c for c in (left, right) if c is not None]
  if not seen:
    return True, False
  return min(seen) < DRIFT_CLEARANCE, True


def outside_clearance(model, curvature, lane_width=None):
  """The car's side clearance to the outside line in a turn (None on a straight or with no reference): the outside line when seen,
  else the outside edge, else what the inside line implies (lane width - car width - inside clearance)."""
  try:
    k = float(curvature)
  except (TypeError, ValueError):
    return None
  if not (math.isfinite(k) and abs(k) >= STRAIGHT_CURVATURE):
    return None
  left, right = side_clearances(model)
  e_left, e_right = edge_clearances(model)
  out_line, out_edge = (left, e_left) if k > 0 else (right, e_right)
  inside = right if k > 0 else left
  if out_line is not None:
    return out_line
  if out_edge is not None:
    return out_edge
  if inside is not None:
    return (lane_width or DEFAULT_LANE_WIDTH) - 2 * CAR_HALF_WIDTH - inside
  return None


class _ProbView:
  """A view of the model whose laneLineProbs are replaced; everything else is read through.
  Defined ONCE at module level: a class defined inside the function was a new class per call, and a class is a reference
  cycle that only the cycle collector frees. controlsd runs with gc disabled (config_realtime_process), so at 100 Hz that
  leaked about 1-2 GB per engaged hour and kept every modelV2 message alive (2026-11-25: low memory after ~65 min)."""
  __slots__ = ("_model", "laneLineProbs")

  def __init__(self, model, probs):
    self._model, self.laneLineProbs = model, probs

  def __getattr__(self, name):
    return getattr(self._model, name)


def _with_line_probs(model, probs12):
  """A view of the model whose laneLineProbs[1] / [2] are replaced (the hysteresis); everything else read through."""
  p = list(model.laneLineProbs)
  p[1], p[2] = probs12
  return _ProbView(model, p)


def lane_width_seen(model):
  """The lane width in metres when both adjacent lines are seen with confidence and the width is plausible, else None."""
  try:
    left_y, right_y = model.laneLines[1].y[0], model.laneLines[2].y[0]
    if model.laneLineProbs[1] > MIN_LINE_PROB and model.laneLineProbs[2] > MIN_LINE_PROB:
      w = right_y - left_y
      if math.isfinite(w) and MIN_LANE_WIDTH <= w <= MAX_LANE_WIDTH:
        return w
  except (AttributeError, IndexError, TypeError):
    pass
  return None


def lane_change_active(model, left_blinker=False, right_blinker=False):
  """A driver-initiated lane change (blinker on, or the model's lane-change state not 'off'): no drift alerts from the gate."""
  try:
    if int(model.meta.laneChangeState) != 0:
      return True
  except (AttributeError, TypeError, ValueError):
    pass
  return bool(left_blinker or right_blinker)


class LaneDriftGate:
  """lane_drift_evidence with state: the remembered lane width, no evidence during a lane change, and a latch. Once the gate has
  opened while the steering is at its limit it stays open LATCH_S past its last open tick (through the model's re-index of the lines
  when the car crosses the outside one, and so for the whole Take Control hold) until the steering has been clear of the limit for
  LATCH_RELEASE_S. A line jumping about a lane width between ticks is a re-index: the latch is refreshed, not released."""
  def __init__(self, dt):
    self.dt = dt
    self.width = None
    self.latch_left = 0.0
    self.clear_s = 0.0
    self.prev_ys = None
    self.t = 0.0
    self.samples = []            # (time, outside clearance) per model frame, newest last
    self.conf = [False, False]   # lane lines [1], [2] confident (with hysteresis)

  def reset(self):
    self.latch_left, self.clear_s, self.prev_ys = 0.0, 0.0, None
    self.samples = []
    self.conf = [False, False]

  def closing_rate(self, model, curvature, reindex):
    """Low-passed closing rate (m/s, positive = approaching the outside line) or None: a least-squares slope over RATE_WINDOW_S of
    model frames. A re-index (line jump) restarts the series."""
    if reindex:
      self.samples = []
    c = outside_clearance(model, curvature, self.width)
    if c is None:
      self.samples = []
      return None
    if not self.samples or c != self.samples[-1][1]:       # one sample per new model frame
      self.samples.append((self.t, c))
    self.samples = [(t, v) for t, v in self.samples if self.t - t <= RATE_WINDOW_S + 1e-9]
    if len(self.samples) < RATE_MIN_FRAMES:
      return None
    n = len(self.samples)
    mt = sum(t for t, _ in self.samples) / n
    mv = sum(v for _, v in self.samples) / n
    den = sum((t - mt) ** 2 for t, _ in self.samples)
    return -sum((t - mt) * (v - mv) for t, v in self.samples) / den if den > 0 else None

  def update(self, model, curvature, at_limit, lane_change=False):
    w = lane_width_seen(model)
    if w is not None:
      self.width = w
    if lane_change:
      self.reset()
      return False
    ys = None
    try:
      ys = (float(model.laneLines[1].y[0]), float(model.laneLines[2].y[0]))
    except (AttributeError, IndexError, TypeError, ValueError):
      pass
    reindex = False
    if ys is not None and self.prev_ys is not None:
      reindex = any(abs(a - b) > REINDEX_JUMP_FRAC * (self.width or DEFAULT_LANE_WIDTH) for a, b in zip(ys, self.prev_ys))
    # Probability hysteresis: a line seen above MIN_LINE_PROB stays confident down to KEEP_LINE_PROB while its position is consistent
    # with the previous frame (a jump of REINDEX_JUMP_FRAC of a lane width is a re-index, not the same line), so a line fading
    # mid-crossing stays evidence. A line never above MIN_LINE_PROB is not.
    try:
      probs = [float(model.laneLineProbs[1]), float(model.laneLineProbs[2])]
    except (AttributeError, IndexError, TypeError, ValueError):
      probs = None
    if probs is not None and ys is not None:
      jump = REINDEX_JUMP_FRAC * (self.width or DEFAULT_LANE_WIDTH)
      for i in (0, 1):
        same = self.prev_ys is not None and abs(ys[i] - self.prev_ys[i]) <= jump
        self.conf[i] = probs[i] > MIN_LINE_PROB or (self.conf[i] and same and probs[i] >= KEEP_LINE_PROB)
      model = _with_line_probs(model, [max(probs[i], MIN_LINE_PROB + 1e-3) if self.conf[i] else probs[i] for i in (0, 1)])
    self.prev_ys = ys
    self.t += self.dt
    now, referenced = _evidence(model, curvature, self.width)
    rate = self.closing_rate(model, curvature, reindex)
    if at_limit and not now and rate is not None and rate >= PREDICT_MIN_RATE:
      c = self.samples[-1][1]
      if c - rate * PREDICT_HORIZON_S <= 0.0:      # the car's side reaches the line within the horizon
        now = referenced = True
    if not at_limit:
      self.clear_s += self.dt
      if self.clear_s >= LATCH_RELEASE_S:
        self.latch_left = 0.0
      return now
    self.clear_s = 0.0
    if now and referenced or (reindex and self.latch_left > 0.0):
      self.latch_left = LATCH_S      # only positive evidence arms the latch, not a fail-open tick
      return True
    if now:
      return True
    if self.latch_left > 0.0:
      self.latch_left -= self.dt
      return True
    return False


class HoldTimer:
  """Counts consecutive frames of a condition; fires once held for hold_s."""
  def __init__(self, hold_s, dt):
    self.hold_frames, self.dt, self.frames = hold_s / dt, dt, 0

  def step(self, held):
    self.frames = self.frames + 1 if held else 0
    return held and self.frames >= self.hold_frames - 1e-9
