"""Plain-Python stand-ins for the scalar numpy calls of the lateral controller's hot path (np.clip, np.interp, np.sign on one
number), each returning bit for bit the value numpy 1.26 returns (the device's numpy, aarch64), at a fraction of the cost: a numpy
call on a Python float converts it to an array scalar and back, several microseconds a call on the device, dozens of calls a cycle.

Exactness (checked against numpy 1.26.4 on aarch64 and x86-64 over millions of random and edge-case inputs, see
selfdrive/controls/lib/tests/test_scalar_np.py and the evidence script it names):
  clip(x, lo, hi)   numpy's float clip: max(x, lo) then min(., hi), each keeping the bound unless x is NaN or strictly beyond it,
                    so a tie between signed zeros returns the bound's zero (numpy 1.26), NaN stays NaN.
  sign(x)           1.0 / -1.0 / 0.0 (+0.0 for either zero) / NaN for NaN.
  Interp(xp, fp)(x) numpy's interp for one x with strictly increasing breakpoints: NaN in, NaN out; outside the table the end
                    values; on a breakpoint its value; between two, slope * (x - x_j) + y_j with numpy's retry from the right end
                    when that is NaN.
Values only: clip and sign return a Python number (the callers wrap them in float() or int()); Interp returns a Python float.
"""
# RUNTIME: runs in controlsd at 100 Hz; gc disabled (see docs/RUNTIME_CONSTRAINTS.md).
# Per call: create no class or closure, keep no container without a bound, nothing that grows with drive time.


def clip(x, lo, hi):
  t = x if (x != x or x > lo) else lo
  return t if (t != t or t < hi) else hi


def sign(x):
  if x > 0:
    return 1.0
  if x < 0:
    return -1.0
  if x == 0:
    return 0.0
  return float(x)                     # NaN


class Interp:
  """np.interp(x, xp, fp) for a scalar x and a fixed table (xp strictly increasing, at least two points)."""

  def __init__(self, xp, fp):
    self.xp = tuple(float(v) for v in xp)
    self.fp = tuple(float(v) for v in fp)
    n = len(self.xp)
    assert n >= 2 and n == len(self.fp) and all(self.xp[i] < self.xp[i + 1] for i in range(n - 1))
    self.slopes = tuple((self.fp[i + 1] - self.fp[i]) / (self.xp[i + 1] - self.xp[i]) for i in range(n - 1))
    self.last = n - 1

  def __call__(self, x):
    x = float(x)
    xp, fp = self.xp, self.fp
    if x != x:
      return x
    if x > xp[-1]:
      return fp[-1]
    if x < xp[0]:
      return fp[0]
    j = 0
    while j < self.last and x >= xp[j + 1]:
      j += 1
    if j == self.last or xp[j] == x:
      return fp[j]
    slope = self.slopes[j]
    r = slope * (x - xp[j]) + fp[j]
    if r != r:
      r = slope * (x - xp[j + 1]) + fp[j + 1]
      if r != r and fp[j] == fp[j + 1]:
        r = fp[j]
    return r
