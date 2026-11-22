"""Lag guard (ti_lag.py): the car controller stands the on-car tests down by itself when the control loop is short of time, instead of
pushing the car into a "controls lagging" soft-disable.

Evidence (drives of 2026-11-22, E:/openpilot-mazda-ti-data/drive-20261122/analysis): controlsd and card share one core with about
0.4-1.1 ms of headroom per 10 ms cycle; the soft-disable comes from CONTROLSD's Ratekeeper (common/realtime.py: the mean of its last 100
cycle periods above 11.1 ms); with the drive plan in its tests mode the core ran at 99.7-99.9 % and one lag soft-disable timed out into a
disengagement. Two signals, either of which holds the tests:
  controls   controlsd's own lag as it publishes it (controlsState.cumLagMs: how late its current cycle is against its fixed schedule).
             Each new controlsState counts as one controlsd cycle of 10 ms plus the lag it added; the mean over the last WINDOW is
             controlsd's mean period, the quantity its soft-disable is decided on. card hands the latest controlsState to the car
             controller every cycle (card.py); a sample is taken when its log time changes.
  card       the car controller's own cycle period (monotonic), the mean of the last WINDOW intervals (card is paced by draining CAN, so it
             can look healthy while controlsd lags: it is the second signal, not the only one).
Held when a signal's mean period is above HOLD_MS, each cycle counted at most CLIP_MS: one stall, however long, adds at most 0.1 ms to the
mean and never holds by itself (three such cycles, or a loop steadily 0.3 ms slow, do). Released only after both means have stayed below RELEASE_MS for the resume wait: RELEASE_S, doubled
after each re-trigger within BACKOFF_WINDOW_S of the last release (capped at RELEASE_MAX_S), back to RELEASE_S after a quiet minute, so
the guard cannot cycle pause / resume / pause. A gap longer than GAP_S (the process paused, a restart) clears that signal's window.

`LagGuard.tick(t, ctl=None)` once per car-controller cycle: `t` monotonic seconds; `ctl` the latest (log time in seconds, cumLagMs) of
controlsState, or None. Returns whether the tests are held. Constant cost per cycle.
"""
from collections import deque

HOLD_MS = 10.3
RELEASE_MS = 10.1
CLIP_MS = 20.0          # a cycle counts at most this long in the mean: one stall (31 ms or 3 s) never holds by itself
RELEASE_S = 10.0
RELEASE_MAX_S = 60.0
BACKOFF_WINDOW_S = 60.0
WINDOW = 100
GAP_S = 1.0
PERIOD_MS = 10.0
WHY = "control loop busy"


class _Window:
  """The last WINDOW per-cycle periods (ms, each at most CLIP_MS): their mean."""

  def __init__(self):
    self.q = deque()
    self.total = 0.0

  def clear(self):
    self.q.clear()
    self.total = 0.0

  def add(self, ms):
    ms = min(ms, CLIP_MS)
    self.q.append(ms)
    self.total += ms
    if len(self.q) > WINDOW:
      self.total -= self.q.popleft()

  def mean(self):
    return self.total / len(self.q) if len(self.q) >= WINDOW else None


class LagGuard:
  def __init__(self, log=None):
    self.log = log
    self.card = _Window()
    self.ctl = _Window()
    self.last = None
    self.ctl_last = None             # (log time s, cumLagMs) of the last controlsState sample taken
    self.held = False
    self.under_since = None
    self.wait_s = RELEASE_S
    self.released_at = None
    self.card_ms = None
    self.ctl_ms = None
    self.signal = ""

  def _event(self, kind, t):
    if self.log is not None:
      try:
        self.log({"kind": kind, "signal": self.signal, "card_ms": None if self.card_ms is None else round(self.card_ms, 2),
                  "ctl_ms": None if self.ctl_ms is None else round(self.ctl_ms, 2), "wait_s": self.wait_s, "t": round(t, 2)})
      except Exception:
        pass

  def _take_ctl(self, ctl):
    if ctl is None:
      return
    stamp, lag = float(ctl[0]), float(ctl[1])
    if self.ctl_last is not None:
      if stamp == self.ctl_last[0]:
        return                                               # no new controlsState since the last cycle
      gap = stamp - self.ctl_last[0]
      if gap <= 0.0 or gap > GAP_S:
        self.ctl.clear()
      else:
        # controlsd never sleeps while it is late (its lag is positive), so each cycle took 10 ms plus the change in its lag (negative
        # when it caught up): the window's mean is exactly its mean period, the quantity its soft-disable is decided on
        self.ctl.add(PERIOD_MS + (lag - self.ctl_last[1]))
    self.ctl_last = (stamp, lag)

  def tick(self, t, ctl=None):
    """Held after this cycle?"""
    if self.last is not None:
      dt = t - self.last
      if dt <= 0.0 or dt > GAP_S:
        self.card.clear()
        self.under_since = None
      else:
        self.card.add(1000.0 * dt)
    self.last = t
    self._take_ctl(ctl)
    self.card_ms, self.ctl_ms = self.card.mean(), self.ctl.mean()
    busy_card = self.card_ms is not None and self.card_ms > HOLD_MS
    busy_ctl = self.ctl_ms is not None and self.ctl_ms > HOLD_MS
    known = [m for m in (self.card_ms, self.ctl_ms) if m is not None]
    calm = bool(known) and all(m < RELEASE_MS for m in known)
    if not self.held and self.released_at is not None and t - self.released_at > BACKOFF_WINDOW_S:
      self.wait_s, self.released_at = RELEASE_S, None        # a quiet minute: the resume wait is back to its base
    if not self.held:
      if busy_card or busy_ctl:
        if self.released_at is not None:
          self.wait_s = min(RELEASE_MAX_S, 2.0 * self.wait_s)  # re-triggered within a minute of a release: wait longer next time
        self.held, self.under_since = True, None
        self.signal = "controls" if busy_ctl else "card"
        self._event("lag_hold", t)
    elif calm:
      if self.under_since is None:
        self.under_since = t
      elif t - self.under_since >= self.wait_s:
        self.held, self.under_since, self.released_at = False, None, t
        self._event("lag_release", t)
    else:
      self.under_since = None
    return self.held
