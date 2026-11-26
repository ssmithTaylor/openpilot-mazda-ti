"""The TI plant branch's PID and saturation timer with plain-Python scalar gains and clips (latcontrol_torque.PlantPID,
LatControlTorque._check_saturation; core-4 CPU, 2026-11-26) give the upstream PIDController's / LatControl's values and types bit for
bit: random sequences incl. NaN, infinities, zero limits (no authority), frozen integrator, one-point and changing gain tables."""
import math
import random
import struct
from types import SimpleNamespace as NS

import numpy as np

from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque, PlantPID
from openpilot.selfdrive.controls.lib.pid import PIDController

NUMPY_1 = int(np.__version__.split(".")[0]) < 2


def key(v):
  """Exact bits and type; on numpy 2 (the PC) a clip tie between signed zeros differs from numpy 1.26 (the device), so the sign of a
  zero is not compared there (scalar_np's documented exception)."""
  f = float(v)
  if f == 0.0 and not NUMPY_1:
    f = 0.0
  return struct.pack("<d", f).hex() if f == f else "nan", type(v).__name__


TABLES = [[[0], [0.8]], [[0.0, 10.0, 30.0], [1.2, 0.9, 0.6]], [[5, 20], [0.1, 0.3]], [[0.0], [0.0]], [[3.0], [float("nan")]], [[2, 2], [1.0, 2.0]]]


def _value(r):
  c = r.random()
  if c < 0.03:
    return float("nan")
  if c < 0.05:
    return r.choice([float("inf"), -float("inf"), 0.0, -0.0])
  return r.gauss(0, 2.0)


def test_plant_pid_matches_upstream_pid_bit_for_bit():
  for seed in range(6):
    r = random.Random(seed)
    up = PIDController(0.8, 0.2, k_d=0.05, rate=100)
    ours = PlantPID(0.8, 0.2, k_d=0.05, rate=100)
    for k in range(4000):
      if k % 97 == 0:                                   # controlsd assigns a (new) kp table every cycle; the values change rarely
        t = r.choice(TABLES)
        up._k_p = [list(t[0]), list(t[1])]
        ours._k_p = [list(t[0]), list(t[1])]
        t2 = r.choice(TABLES)
        up._k_i, ours._k_i = [list(t2[0]), list(t2[1])], [list(t2[0]), list(t2[1])]
      else:
        ours._k_p = [list(ours._k_p[0]), list(ours._k_p[1])]
        up._k_p = [list(up._k_p[0]), list(up._k_p[1])]
      lim = r.choice([0.0, 0.5, 2.0, 3.5, float("inf")])
      up.set_limits(lim, -lim)
      ours.set_limits(lim, -lim)
      args = dict(error=_value(r), error_rate=_value(r), speed=r.choice([0.0, 3.0, 10.0, 25.0, 40.0, abs(r.gauss(15, 10)), float('nan'), float('inf'), -1.0]),
                  feedforward=_value(r), freeze_integrator=r.random() < 0.2)
      a, b = up.update(**args), ours.update(**args)
      assert key(a) == key(b), (seed, k, args, a, b)
      for f in ("p", "i", "d", "f", "control"):
        assert key(getattr(up, f)) == key(getattr(ours, f)), (seed, k, f)
      for g in ("k_p", "k_i", "k_d"):
        assert key(getattr(up, g)) == key(getattr(ours, g)), (seed, k, g)
      if not math.isfinite(float(up.i)):
        up.reset()
        ours.reset()


def test_saturation_timer_matches_upstream_bit_for_bit():
  class Base(LatControl):
    def update(self, *a, **k):
      return 0.0, 0.0, None
  CP = NS(steerLimitTimer=0.4, steerActuatorDelay=0.1)
  try:
    up, ours = Base(CP, None, 0.01), Base(CP, None, 0.01)
  except Exception:                                     # LatControl's constructor reads more of CP on some trees
    import pytest
    pytest.skip("LatControl needs a full CarParams here")
  ours._check_saturation = LatControlTorque._check_saturation.__get__(ours)
  r = random.Random(3)
  for k in range(20000):
    CS = NS(vEgo=r.choice([0.0, 5.0, 12.0, 30.0]), steeringPressed=r.random() < 0.1)
    args = (r.random() < 0.6, CS, r.random() < 0.1, r.random() < 0.1)
    a, b = up._check_saturation(*args), ours._check_saturation(*args)
    assert a == b and type(a) is type(b), k
    assert key(up.sat_time) == key(ours.sat_time), k
