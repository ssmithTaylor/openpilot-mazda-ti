"""The reviewed cruise-button tap machinery, shared by the cruise set-speed check (ti_cruise_check.py) and curve slowing (ti_curve.py).

Moved here unchanged from ti_cruise_check.py (which re-exports every name, so its behaviour and tests are untouched): the hard cap on a
tap (PressGuard: at most MAX_PRESSED_FRAMES pressed frames, at most MAX_TAP_SPAN_S first to last, a new tap only MIN_TAP_GAP_S after
the previous one's last pressed frame, enforced on frames AND on real time) and one tap in flight (Tap: waits for a new car button
frame, then pressed frames FRAME_SPACING apart, each with the car's last counter + 1). The only addition: PressGuard(min_gap_s) may
ask for a LONGER gap than MIN_TAP_GAP_S (never a shorter one); the default is exactly the reviewed one.
"""

DT = 0.01                       # the car controller's frame
FRAME_SPACING = 5               # control frames between pressed frames: 50 ms (the module itself sends at 10 Hz)
MAX_PRESSED_FRAMES = 3          # hard cap per tap, enforced by PressGuard whatever the recipe says
MAX_TAP_SPAN_S = 0.15           # hard cap, first to last pressed frame of a tap (a hold is read at 0.52 s; 0.31 s never was one)
MIN_TAP_GAP_S = 0.25            # last pressed frame of one tap to the first of the next
PHASE_WAIT_S = 0.2              # a tap waits at most this long for a new car button frame to start after (else it starts anyway)


class PressGuard:
  """The hard cap on a tap, independent of the recipe: at most MAX_PRESSED_FRAMES pressed frames, at most MAX_TAP_SPAN_S from the
  first to the last, and a new tap only MIN_TAP_GAP_S after the last pressed frame of the previous one. `allow` is called for every
  pressed frame before it is sent; a refused frame is not sent."""

  def __init__(self, min_gap_s=None):
    self.min_gap_s = MIN_TAP_GAP_S if min_gap_s is None else max(MIN_TAP_GAP_S, float(min_gap_s))   # never below the reviewed gap
    self.tap_first = None
    self.tap_first_mono = None
    self.tap_count = 0
    self.last_frame = None
    self.last_mono = None

  def begin(self, frame, mono=None):
    if self.last_frame is not None and (frame - self.last_frame) * DT < self.min_gap_s - 1e-9:
      return False
    if mono is not None and self.last_mono is not None and mono - self.last_mono < self.min_gap_s - 1e-6:
      return False
    self.tap_first, self.tap_first_mono, self.tap_count = frame, mono, 0
    return True

  def allow(self, frame, mono=None):
    if self.tap_first is None:
      return False
    if self.tap_count >= MAX_PRESSED_FRAMES or (frame - self.tap_first) * DT > MAX_TAP_SPAN_S + 1e-9:
      return False
    if mono is not None and self.tap_first_mono is not None and mono - self.tap_first_mono > MAX_TAP_SPAN_S + 1e-6:
      return False                    # real time, too: a stalled or delayed control loop cannot stretch a tap into a hold
    self.tap_count += 1
    self.last_frame = frame
    self.last_mono = mono
    return True

  def end(self):
    self.tap_first = None


class Tap:
  """One tap being sent: waits for a new car button frame (counter change) for at most PHASE_WAIT_S, then sends `n` pressed frames
  FRAME_SPACING apart. `frames`: [(frame, car counter, counter sent)]."""

  def __init__(self, button, n, frame, counter, target=None):
    self.button, self.n = button, n
    self.target = target              # the set speed this tap is to give: no pressed frame once it reads that (or anything else wrong)
    self.requested = frame
    self.counter0 = counter
    self.first = None
    self.frames = []

  def due(self, frame, counter):
    """True when a pressed frame is to be sent on this frame."""
    if self.first is None:
      if counter != self.counter0 or (frame - self.requested) * DT >= PHASE_WAIT_S - 1e-9:
        return True
      return False
    return len(self.frames) < self.n and (frame - self.first) % FRAME_SPACING == 0

  @property
  def done(self):
    return len(self.frames) >= self.n
