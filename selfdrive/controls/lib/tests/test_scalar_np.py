"""scalar_np's clip / sign / Interp: numpy 1.26's results bit for bit (the device's numpy). The full evidence (1.2 million random and
edge cases per function against numpy 1.26.4 on aarch64 and x86-64, and the old and new lateral controller side by side over recorded
drives and a million randomised cycles) comes from scalar_np_evidence.py and ltq_side_by_side.py in this directory; this test pins the
edge cases and compares with whatever numpy is installed (numpy 2 differs from 1.26 only on a tie between signed zeros in clip, which
is left out when numpy 2 is the reference)."""
import math
import random
import struct

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.scalar_np import Interp, clip, sign
from openpilot.selfdrive.controls.lib.latcontrol_torque import LOW_SPEED_X, LOW_SPEED_Y
from openpilot.selfdrive.car.mazda.lateral_plant import GS_BP, Q_BP, V_BP

NAN, INF = float("nan"), float("inf")
NUMPY_1 = int(np.__version__.split(".")[0]) < 2


def bits(v):
  return struct.pack("<d", float(v)).hex()


@pytest.mark.parametrize("x, lo, hi, want", [
  (-0.0, 0.0, 1.0, 0.0), (0.0, -0.0, 1.0, -0.0), (-0.0, -1.0, 0.0, 0.0), (0.5, 0.0, 1.0, 0.5), (2.0, 0.0, 1.0, 1.0),
  (-2.0, 0.0, 1.0, 0.0), (INF, -1.0, 1.0, 1.0), (-INF, -1.0, 1.0, -1.0), (1.0, 0.0, 1.0, 1.0), (7.5, 1, 100, 7.5), (0.2, 1, 100, 1),
])
def test_clip_matches_numpy_126_on_the_edges(x, lo, hi, want):
  assert bits(clip(x, lo, hi)) == bits(want)


def test_clip_keeps_nan_with_its_bits():
  odd_nan = struct.unpack("<d", bytes.fromhex("010000000000f8ff"))[0]
  assert bits(clip(odd_nan, -1.0, 1.0)) == bits(odd_nan) and math.isnan(clip(NAN, 0.0, 1.0))


@pytest.mark.parametrize("x, want", [(3.0, 1.0), (-1e-300, -1.0), (0.0, 0.0), (-0.0, 0.0), (5e-324, 1.0), (INF, 1.0), (-INF, -1.0), (2, 1.0)])
def test_sign_matches_numpy_on_the_edges(x, want):
  assert bits(sign(x)) == bits(want)
  assert bits(sign(x)) == bits(np.sign(x))


@pytest.mark.parametrize("xp, fp", [(V_BP, Q_BP), (V_BP, GS_BP), (LOW_SPEED_X, LOW_SPEED_Y)])
def test_interp_matches_numpy_on_breakpoints_ends_and_non_finite(xp, fp):
  f = Interp(xp, fp)
  xs = [NAN, INF, -INF, -0.0, 0.0, 1e-300, 1e300] + [v for b in xp for v in (b, math.nextafter(b, INF), math.nextafter(b, -INF))]
  for x in xs:
    assert bits(f(x)) == bits(np.interp(x, xp, fp)), x
  assert isinstance(f(12.0), float)


def test_interp_rejects_a_table_it_does_not_reproduce():
  with pytest.raises(AssertionError):
    Interp([0.0, 1.0, 1.0], [0.0, 1.0, 2.0])        # repeated breakpoint: numpy's search order would matter
  with pytest.raises(AssertionError):
    Interp([0.0], [1.0])


def test_random_cases_match_the_installed_numpy():
  r = random.Random(5)
  tables = [Interp(V_BP, Q_BP), Interp(V_BP, GS_BP), Interp(LOW_SPEED_X, LOW_SPEED_Y)]
  raw = [(V_BP, Q_BP), (V_BP, GS_BP), (LOW_SPEED_X, LOW_SPEED_Y)]
  for _ in range(30000):
    x = r.choice([r.uniform(-50, 50), r.uniform(-1, 1), struct.unpack("<d", struct.pack("<Q", r.getrandbits(64)))[0]])
    lo = r.uniform(-10, 0)
    hi = lo + r.uniform(0, 10)
    if NUMPY_1 or not (x == lo or x == hi):                 # numpy 2 breaks a signed-zero tie the other way
      assert bits(clip(x, lo, hi)) == bits(np.clip(x, lo, hi)), (x, lo, hi)
    assert bits(sign(x)) == bits(np.sign(x)), x
    i = r.randrange(3)
    assert bits(tables[i](x)) == bits(np.interp(x, *raw[i])), (x, i)
