"""The reviewed cruise-button tap machinery, shared by the cruise set-speed check (ti_cruise_check.py) and curve slowing (ti_curve.py).

Moved here unchanged from ti_cruise_check.py (which re-exports every name, so its behaviour and tests are untouched): the hard cap on a
tap (PressGuard: at most MAX_PRESSED_FRAMES pressed frames, at most MAX_TAP_SPAN_S first to last, a new tap only MIN_TAP_GAP_S after
the previous one's last pressed frame, enforced on frames AND on real time) and one tap in flight (Tap: waits for a new car button
frame, then pressed frames FRAME_SPACING apart, each with the car's last counter + 1). The only addition: PressGuard(min_gap_s) may
ask for a LONGER gap than MIN_TAP_GAP_S (never a shorter one); the default is exactly the reviewed one.
"""
# RUNTIME: runs in card at 100 Hz; gc disabled (see docs/RUNTIME_CONSTRAINTS.md).
# Per frame: create no class or closure, keep no container without a bound; params / JSON at most 10 Hz.

DT = 0.01                       # the car controller's frame
FRAME_SPACING = 5               # control frames between pressed frames: 50 ms (the module itself sends at 10 Hz)
MAX_PRESSED_FRAMES = 3          # hard cap per tap, enforced by PressGuard whatever the recipe says
MAX_TAP_SPAN_S = 0.15           # hard cap, first to last pressed frame of a tap (a hold is read at 0.52 s; 0.31 s never was one)
MIN_TAP_GAP_S = 0.25            # last pressed frame of one tap to the first of the next (the 0.3 s cycle's gap; the cruise check's)
PHASE_WAIT_S = 0.2              # a tap waits at most this long for a new car button frame to start after (else it starts anyway)

# ---- the tap cycle: the ONE source of truth (owner, 2026-11-24: one tap per 0.2 s approved, 0.1 s to be tested on the car) ----------
# A tap starts on a new car button frame (the steering-wheel switch sends 0x09D at 10 Hz, CAR_FRAME_S, counter + 1 each), so a cycle
# is a whole number of car frames: 0.3 s = every third, 0.2 s = every second, 0.1 s = every one. PressGuard(cycle_s=...) enforces it:
# a new tap's first pressed frame at least cycle - half a car frame after the previous tap's first (jitter margin, so the car frame
# that starts the cycle is never missed) and at least cycle - one car frame after its last. The recipe (frames_for): up to
# 2 pressed frames 50 ms apart (the second only while the step has not been read back) at 0.2 s and 0.3 s, exactly as recorded
# (292 taps, all stepped: 106 read back after the first frame, 0.040 s; 186 after the second, 0.068 s); 1 frame at 0.1 s, where two
# would make a nearly unbroken pressed stream (a hold, not taps). Every pressed frame of a tap falls inside the 0.1 s before the
# switch's own unpressed frame (about 0.097 s after ours), so each tap is pressed, then released by the switch, then idle.
# NOT YET WIRED to curve slowing (merge into curve-stages, 2026-11-24): the planner (ti_curve_plan.TAP_S), the cruise model's walk
# (ti_cruise_model.TAP_S) and the curve car side (PressGuard() default) all still assume the 0.30 s cycle; here only the on-car
# tap-rate test uses the faster cycles.
CAR_FRAME_S = 0.1
TAP_CYCLE_S = 0.20              # the approved faster cycle, 5 mph/s (curve slowing's once wired; see above)
TAP_CYCLE_SLOW_S = 0.30         # the reviewed cycle: the cruise check, and curve slowing's fallback after missed steps
TAP_CYCLE_MIN_S = 0.10          # the shortest any path may ask for (the on-car tap-rate test only)
TAP_GAP_FLOOR_S = 0.04          # never two taps' pressed frames closer than this (one car frame of 0.1 s always lies between)


def cycle_gaps(cycle_s):
  """(first-to-first, last-to-first) minimum seconds between two taps on this cycle. 0.3 s and slower: exactly the reviewed guard
  (MIN_TAP_GAP_S from the last pressed frame); faster: whole car frames with half a car frame of margin."""
  c = max(TAP_CYCLE_MIN_S, float(cycle_s))
  if c >= TAP_CYCLE_SLOW_S - 1e-9:
    return 0.0, max(MIN_TAP_GAP_S, c - CAR_FRAME_S / 2)
  return c - CAR_FRAME_S / 2, max(TAP_GAP_FLOOR_S, c - CAR_FRAME_S)


def frames_for(cycle_s):
  """Pressed frames per tap on this cycle (see above)."""
  return 1 if cycle_s < TAP_CYCLE_S - 1e-9 else 2

# ---- the held button (plan A7, owner approved 2026-11-24) ---------------------------------------------------------------------------
# OFF: the recorded drives cannot show that the car reads OUR continuous stream as a hold. The car's own button module keeps sending
# its unpressed frame (same counter as ours) 0.1 s after each of ours, and our recorded 2-frame taps step later than 1-frame taps
# (0.068 s vs 0.040 s median, 186 / 106 taps), so the cruise unit does act on a frame repeating the last counter: it may see press /
# release every 0.1 s rather than a hold. A short on-car check decides; until then curve slowing uses taps only (and plans with them).
HOLD_ENABLED = False
HOLD_MAX_S = 2.5                # hard cap, first to last pressed frame of a hold (frames AND real time): 0.55 s + 3 x 0.6 s + read
HOLD_FRAME_MIN_S = 0.08         # never two held frames closer than this (one per car button frame, 10 Hz)
HOLD_MAX_FRAMES = int(HOLD_MAX_S / 0.1) + 1   # 26: one per car frame over the cap


class HoldGuard:
  """The hard cap on a hold, independent of the logic that drives it (like PressGuard for taps): at most HOLD_MAX_FRAMES pressed
  frames, at most HOLD_MAX_S from the first to the last, never two closer than HOLD_FRAME_MIN_S, all on frames AND real time; a new
  hold only MIN_TAP_GAP_S after the last pressed frame of anything (`begin` is given the tap guard too)."""

  def __init__(self):
    self.first = None
    self.first_mono = None
    self.count = 0
    self.last_frame = None
    self.last_mono = None

  def begin(self, frame, mono=None, other=None):
    pairs = [(self.last_frame, self.last_mono)] + ([(other.last_frame, other.last_mono)] if other is not None else [])
    for lf, lm in pairs:
      if lf is not None and (frame - lf) * DT < MIN_TAP_GAP_S - 1e-9:
        return False
      if mono is not None and lm is not None and mono - lm < MIN_TAP_GAP_S - 1e-6:
        return False
    self.first, self.first_mono, self.count = frame, mono, 0
    return True

  def allow(self, frame, mono=None):
    if self.first is None or self.count >= HOLD_MAX_FRAMES:
      return False
    if (frame - self.first) * DT > HOLD_MAX_S + 1e-9:
      return False
    if mono is not None and self.first_mono is not None and mono - self.first_mono > HOLD_MAX_S + 1e-6:
      return False                    # real time too: a stalled control loop cannot stretch a hold
    if self.count and (frame - self.last_frame) * DT < HOLD_FRAME_MIN_S - 1e-9:
      return False
    if self.count and mono is not None and self.last_mono is not None and mono - self.last_mono < HOLD_FRAME_MIN_S - 1e-6:
      return False
    self.count += 1
    self.last_frame = frame
    self.last_mono = mono
    return True

  def end(self):
    self.first = None


class PressGuard:
  """The hard cap on a tap, independent of the recipe: at most MAX_PRESSED_FRAMES pressed frames, at most MAX_TAP_SPAN_S from the
  first to the last, and a new tap only MIN_TAP_GAP_S after the last pressed frame of the previous one. `allow` is called for every
  pressed frame before it is sent; a refused frame is not sent."""

  def __init__(self, min_gap_s=None, cycle_s=None):
    self.min_gap_s = MIN_TAP_GAP_S if min_gap_s is None else max(MIN_TAP_GAP_S, float(min_gap_s))   # never below the reviewed gap
    self.first_gap_s = 0.0
    self.cycle_s = None
    if cycle_s is not None:
      self.set_cycle(cycle_s)
    self.tap_first = None
    self.tap_first_mono = None
    self.tap_count = 0
    self.last_frame = None
    self.last_mono = None
    self.prev_first = None          # the previous tap's first pressed frame (frames and real time)
    self.prev_first_mono = None

  def set_cycle(self, cycle_s):
    """Tap on this cycle from now on (cycle_gaps; never faster than TAP_CYCLE_MIN_S)."""
    self.cycle_s = max(TAP_CYCLE_MIN_S, float(cycle_s))
    self.first_gap_s, self.min_gap_s = cycle_gaps(self.cycle_s)

  def begin(self, frame, mono=None):
    if self.last_frame is not None and (frame - self.last_frame) * DT < self.min_gap_s - 1e-9:
      return False
    if mono is not None and self.last_mono is not None and mono - self.last_mono < self.min_gap_s - 1e-6:
      return False
    if self.prev_first is not None and (frame - self.prev_first) * DT < self.first_gap_s - 1e-9:
      return False
    if mono is not None and self.prev_first_mono is not None and mono - self.prev_first_mono < self.first_gap_s - 1e-6:
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
    if self.tap_count == 0:
      self.prev_first, self.prev_first_mono = frame, mono
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
