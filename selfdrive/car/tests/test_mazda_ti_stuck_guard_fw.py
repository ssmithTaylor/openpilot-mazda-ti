"""The stuck-request guard against the interceptor firmware's own unchanged-request monitor.

`fw_check_stuck` is a line-for-line port of `v6_ctl_check_stuck` from the reconstructed application-version-6 image (flash
0x15be-0x1dac; reconstruction validated against the instruction stream offline). It is the v6 routine: the car's unit reports
version 16, whose routine is not available, so this proves the guard against the v6 mechanism only. What the port encodes: the
monitor runs once per FOREGROUND pass (not per CAN frame, not per timer tick), compares the RAW received request (before the
+-600 clip) with the previous one, restarts its count on any change, on neutral and on exactly raw 1448 / 2648 (-600 / +600),
and reports 0x11 when more than 9000 passes have seen the same value. Observed trips (1.84-1.99 s) put the foreground loop near
4.7 kHz, about 47 passes per 10 ms frame; the tests sweep 20-100 passes per frame. (The guard cannot protect a unit whose loop
makes 9000 passes in under its 0.8 s hold, i.e. above about 112 passes per frame: that would mean trips after 0.8 s, which the
logged 1.84-1.99 s trips exclude; `test_the_margin_is_the_loop_speed` pins that boundary so a change of STUCK_HOLD_S is seen.)
"""
import random

import pytest

from openpilot.selfdrive.car.mazda.ti_stuck_guard import StuckGuard

RAW_MIN, RAW_NEUTRAL, RAW_MAX = 1448, 2048, 2648
RUN = 3


class FwStuck:
  def __init__(self):
    self.prev, self.passes = RAW_NEUTRAL, 0

  def check(self, received_raw, mode=RUN):
    if mode != RUN:
      return 0
    if received_raw != self.prev:
      self.prev, self.passes = received_raw, 0
      return 0
    if received_raw in (RAW_NEUTRAL, RAW_MIN, RAW_MAX):
      self.passes = 0
      return 0
    self.passes += 1
    return 0x11 if self.passes > 9000 else 0


def run(cmds, passes_per_frame, guard=True, drop=()):
  """cmds: the controller's final command per 10 ms frame. drop: frame indices lost on the bus (the unit keeps the last value)."""
  g, fw, stored = StuckGuard(), FwStuck(), RAW_NEUTRAL
  for i, c in enumerate(cmds):
    sent = g.update(int(c), True) if guard else int(c)
    if i not in drop:
      stored = (sent + 2048) & 0x0fff
    for _ in range(passes_per_frame):
      if fw.check(stored):
        return i
  return None


@pytest.mark.parametrize("ppf", [20, 47, 60, 100])
@pytest.mark.parametrize("value", [-570, 570, -540, 1, -1, 300, 599, 601, -650, 650, 1200])
def test_a_held_command_never_trips_the_firmware_monitor(value, ppf):
  assert run([value] * 3000, ppf) is None                                  # 30 s held


@pytest.mark.parametrize("ppf", [47, 60, 100])
def test_without_the_guard_the_same_hold_trips_it(ppf):
  assert run([-570] * 3000, ppf, guard=False) is not None                  # the monitor port is not vacuous
  assert run([-600] * 3000, ppf, guard=False) is None                      # exactly -600 is exempt
  assert run([0] * 3000, ppf, guard=False) is None


@pytest.mark.parametrize("ppf", [47, 100])
def test_a_lost_nudge_frame_or_a_whole_lost_burst_still_does_not_trip(ppf):
  cmds = [-570] * 3000
  assert run(cmds, ppf, drop={80}) is None                                 # first nudge frame of the first burst lost
  assert run(cmds, ppf, drop={81}) is None
  if ppf <= 47:                                                            # a whole burst lost: 1.62 s of one value, under 9000 passes at <= 55 per frame
    assert run(cmds, ppf, drop={80, 81}) is None


@pytest.mark.parametrize("seed", range(8))
def test_random_drives_never_trip(seed):
  rng = random.Random(seed)
  cmds, c = [], 0
  while len(cmds) < 6000:
    kind = rng.random()
    if kind < 0.35:                                                        # a hold at an arbitrary value
      c = rng.choice([rng.randint(-650, 650), -570, 570, -540, 600, -600, 0])
      cmds += [c] * rng.randint(50, 700)
    elif kind < 0.7:                                                       # a ramp
      target, step = rng.randint(-600, 600), rng.randint(1, 15)
      while c != target and len(cmds) < 6000:
        c += max(-step, min(step, target - c)); cmds.append(c)
    else:                                                                  # 1-count noise
      for _ in range(rng.randint(20, 300)):
        c = max(-600, min(600, c + rng.choice([-1, 0, 0, 1]))); cmds.append(c)
  assert run(cmds, rng.choice([30, 47, 100])) is None


def test_the_margin_is_the_loop_speed():
  """The fastest foreground loop the guard still covers: 9000 passes must take longer than the hold (80 frames) plus the burst."""
  assert run([-570] * 3000, 110) is None
  assert run([-570] * 3000, 120) is not None      # 120 passes per frame = a trip after 0.75 s: outside anything observed
