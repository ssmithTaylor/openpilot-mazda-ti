"""Lag guard (ti_lag.py): the car controller stands the on-car tests down by itself when the control loop is short of time, instead of
pushing the car into a "controls lagging" soft-disable.

Evidence (drives of 2026-11-22, E:/openpilot-mazda-ti-data/drive-20261122/analysis): controlsd and card share one core with about
0.4-1.1 ms of headroom per 10 ms cycle; the warning needs the 1-second mean loop time above 11.1 ms; with the drive plan in its tests
mode the core ran at 99.7-99.9 % and one lag soft-disable timed out into a disengagement. The A/B alternation's own cost was not
measurably above the plan off (card CPU 32 % in both), so the alternation is left running; only the identification tests and the
dither are held (the car controller's existing test hold, reason "control loop busy", shown on the pages and logged).

`LagGuard.tick(t)` is called once per car-controller cycle with a monotonic time in seconds (injectable for tests). It keeps the mean
of the last WINDOW cycle intervals (a ring buffer and a running sum: constant cost). Held when the mean is above HOLD_MS; released
only after the mean has stayed below RELEASE_MS for RELEASE_S. A gap longer than GAP_S (the process paused, a restart of the
cycle) clears the window rather than counting as one long cycle.
"""

HOLD_MS = 10.3
RELEASE_MS = 10.1
RELEASE_S = 10.0
WINDOW = 100
GAP_S = 1.0
WHY = "control loop busy"


class LagGuard:
  def __init__(self, log=None):
    self.log = log
    self.ring = [0.0] * WINDOW
    self.i = 0
    self.count = 0
    self.total = 0.0
    self.last = None
    self.held = False
    self.under_since = None
    self.mean_ms = None

  def _event(self, kind, t):
    if self.log is not None:
      try:
        self.log({"kind": kind, "mean_ms": None if self.mean_ms is None else round(self.mean_ms, 2), "t": round(t, 2)})
      except Exception:
        pass

  def tick(self, t):
    """Held after this cycle?"""
    if self.last is not None:
      dt = t - self.last
      if dt <= 0.0 or dt > GAP_S:
        self.ring, self.i, self.count, self.total = [0.0] * WINDOW, 0, 0, 0.0
      else:
        self.total += dt - self.ring[self.i]
        self.ring[self.i] = dt
        self.i = (self.i + 1) % WINDOW
        self.count = min(self.count + 1, WINDOW)
    self.last = t
    if self.count < WINDOW:
      return self.held
    self.mean_ms = 1000.0 * self.total / WINDOW
    if not self.held:
      if self.mean_ms > HOLD_MS:
        self.held, self.under_since = True, None
        self._event("lag_hold", t)
    elif self.mean_ms < RELEASE_MS:
      if self.under_since is None:
        self.under_since = t
      elif t - self.under_since >= RELEASE_S:
        self.held, self.under_since = False, None
        self._event("lag_release", t)
    else:
      self.under_since = None
    return self.held
