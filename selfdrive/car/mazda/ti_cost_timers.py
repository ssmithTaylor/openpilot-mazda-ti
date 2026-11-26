"""Per-cycle CPU cost of this fork's hooks in controlsd and card, as the drive sees it (core-4 budget, 2026-11-26).

Each hook's wall time per cycle (time.perf_counter_ns around the call; the control processes are SCHED_FIFO on core 4, so wall time is
their CPU time unless an interrupt lands inside) goes into preallocated counters: sum, count, max and a 20 us histogram (p99 to 20 us).
Every SUMMARY_FRAMES (10 s at 100 Hz) one cloudlog event "ti_cost" carries, per hook, mean / p99 / max in microseconds and the number
of cycles it ran, plus any gauges (the car side's maximum plan age seen); then the counters are zeroed in place.

Event fields (read by the drive review): proc ("controlsd" / "card"), frames (cycles in the window), win_s (window length, s),
h: {hook: [n, mean_us, p99_us, max_us]}, g: {gauge: max value}. The "step" hook is the whole cycle (upstream + ours): our share of
the core is the other hooks' mean over step's mean.
"""
# RUNTIME: runs in controlsd and card at 100 Hz; gc disabled (see docs/RUNTIME_CONSTRAINTS.md).
# Per call: create no class or closure, keep no container without a bound, nothing that grows with drive time.
import time

SUMMARY_FRAMES = 1000           # 10 s at 100 Hz
BIN_NS = 20_000                 # histogram bin: 20 us
N_BINS = 500                    # 0 .. 10 ms; the last bin holds everything above


class HookTimers:
  def __init__(self, proc, hooks, gauges=(), log=None, every=SUMMARY_FRAMES):
    self.proc = proc
    self.hooks = tuple(hooks)
    self.gauges = tuple(gauges)
    n = len(self.hooks)
    self.index = {h: i for i, h in enumerate(self.hooks)}
    self.sum_ns = [0] * n
    self.count = [0] * n
    self.max_ns = [0] * n
    self.hist = [[0] * N_BINS for _ in range(n)]
    self.gauge = [None] * len(self.gauges)
    self.log = log
    self.every = every
    self.frames = 0
    self.t0 = time.monotonic()

  def add(self, i, ns):
    """Hook i took ns nanoseconds this cycle."""
    self.sum_ns[i] += ns
    self.count[i] += 1
    if ns > self.max_ns[i]:
      self.max_ns[i] = ns
    b = ns // BIN_NS
    self.hist[i][b if b < N_BINS else N_BINS - 1] += 1

  def gauge_max(self, j, v):
    g = self.gauge[j]
    if v is not None and (g is None or v > g):
      self.gauge[j] = v

  def _p99_us(self, i):
    n = self.count[i]
    if n == 0:
      return 0.0
    top = n // 100 + 1               # samples at or above the 99th percentile; walked down from the maximum's bin (few bins)
    h = self.hist[i]
    acc = 0
    b = min(self.max_ns[i] // BIN_NS, N_BINS - 1)
    while b > 0:
      acc += h[b]
      if acc >= top:
        break
      b -= 1
    return float((b + 1) * BIN_NS // 1000)      # the bin's upper edge

  def summary(self):
    hooks = {}
    for i, h in enumerate(self.hooks):
      n = self.count[i]
      hooks[h] = [n, round(self.sum_ns[i] / n / 1000.0, 1) if n else 0.0, self._p99_us(i), round(self.max_ns[i] / 1000.0, 1)]
    now = time.monotonic()
    return {"proc": self.proc, "frames": self.frames, "win_s": round(now - self.t0, 2), "h": hooks,
            "g": {g: (None if v is None else round(v, 3)) for g, v in zip(self.gauges, self.gauge)}}

  def reset(self):
    for i in range(len(self.hooks)):
      self.sum_ns[i] = 0
      self.count[i] = 0
      self.max_ns[i] = 0
      self.hist[i] = [0] * N_BINS      # once per window (a C-level fill; the old list is freed at once)
    for j in range(len(self.gauge)):
      self.gauge[j] = None
    self.frames = 0
    self.t0 = time.monotonic()

  def tick(self):
    """Once per cycle, after the hooks: every `every` cycles the summary is logged and the counters zeroed. Never raises."""
    self.frames += 1
    if self.frames < self.every:
      return None
    try:
      s = self.summary()
      if self.log is not None:
        self.log(s)
    except Exception:
      s = None
    self.reset()
    return s


def cloudlog_cost(event):
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event("ti_cost", **event)
  except Exception:
    pass
