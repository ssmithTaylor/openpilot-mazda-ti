"""The stuck-request guard (ti_stuck_guard.py): the interceptor reports 0x11 (LKAS_STUK) and drops to OFF for about 31 s when it
receives the same nonzero request, other than exactly +-600, for about 1.9 s. The guard moves such a held command STUCK_NUDGE counts
toward zero for STUCK_NUDGE_FRAMES frames once it has been sent unchanged for STUCK_HOLD_FRAMES frames."""
import random
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.mazda import ti_stuck_guard as sg
from openpilot.selfdrive.car.mazda.ti_stuck_guard import (STUCK_HOLD_FRAMES, STUCK_NUDGE, STUCK_NUDGE_FRAMES, TI_FW_CLIP,
                                                          StuckGuard)

FW_SHORTEST_TRIP_S = 1.84     # the shortest constant hold observed to trip 0x11 (another fork's CX-9, same interceptor)


def eligible(v):
  return v != 0 and abs(v) != TI_FW_CLIP


def longest_identical_run(out, active=None):
  """the longest run of identical consecutive eligible values sent while active (an inactive frame ends a run)"""
  best, run, prev = 0, 0, None
  for i, v in enumerate(out):
    if active is not None and not active[i]:
      run, prev = 0, None
      continue
    run = run + 1 if v == prev else 1
    prev = v
    if eligible(v):
      best = max(best, run)
  return best


def feed(seq, active=True):
  g = StuckGuard()
  return g, [g.update(v, active) for v in seq]


def test_constants_leave_margin_for_a_lost_nudge_burst():
  assert TI_FW_CLIP == 600
  assert STUCK_HOLD_FRAMES == int(round(sg.STUCK_HOLD_S / sg.DT))
  # a nudge burst lost entirely on the bus: the unit sees two hold periods plus the burst as one streak; still under the trip
  assert (2 * STUCK_HOLD_FRAMES + STUCK_NUDGE_FRAMES) * sg.DT < FW_SHORTEST_TRIP_S - 0.15
  assert STUCK_NUDGE_FRAMES >= 2                       # one dropped frame never loses the whole burst
  assert 1 <= STUCK_NUDGE <= 5                         # inside the unit's +-6-count dead band: not felt


@pytest.mark.parametrize("value", [1, -1, 3, 300, -570, 650, 599, -599, 601, -650, 1200])
def test_constant_command_is_nudged_at_the_hold_time_and_never_held_longer(value):
  g, out = feed([value] * 6000)                        # 60 s
  assert out[:STUCK_HOLD_FRAMES] == [value] * STUCK_HOLD_FRAMES
  assert out[STUCK_HOLD_FRAMES] != value               # first nudge exactly at the hold time
  assert longest_identical_run(out) <= STUCK_HOLD_FRAMES
  # every nudge is toward zero, by at most STUCK_NUDGE, never across zero
  for v in out:
    assert abs(v - value) <= STUCK_NUDGE and abs(v) <= abs(value) and v * value >= 0
  nudged = [v for v in out if v != value]
  period = STUCK_HOLD_FRAMES + STUCK_NUDGE_FRAMES
  assert len(nudged) == STUCK_NUDGE_FRAMES * (6000 // period) + max(0, 6000 % period - STUCK_HOLD_FRAMES)
  assert g.bursts == len(nudged) // STUCK_NUDGE_FRAMES + (len(nudged) % STUCK_NUDGE_FRAMES > 0)


@pytest.mark.parametrize("value", [1, -1, 2, -2])
def test_a_tiny_command_is_nudged_to_zero_not_across_it(value):
  _, out = feed([value] * 500)
  assert set(out) == {value, 0}


@pytest.mark.parametrize("value", [0, TI_FW_CLIP, -TI_FW_CLIP])
def test_zero_and_exactly_600_are_never_nudged(value):
  g, out = feed([value] * 6000)
  assert out == [value] * 6000 and g.bursts == 0


def test_inactive_is_a_pass_through_and_restarts_the_count():
  g, out = feed([300] * 6000, active=False)
  assert out == [300] * 6000 and g.bursts == 0
  g = StuckGuard()
  out = [g.update(300, True) for _ in range(STUCK_HOLD_FRAMES - 1)]
  out.append(g.update(300, False))                    # lateral control off for a frame: the count starts again
  out += [g.update(300, True) for _ in range(STUCK_HOLD_FRAMES)]
  assert set(out) == {300}
  assert g.update(300, True) != 300


@pytest.mark.parametrize("seq", [
  [i // 3 for i in range(3000)],                       # slow ramp
  [300 + (i % 2) for i in range(6000)],                # 1-count noise
  [-570 + (i // 50) % 2 for i in range(6000)],         # alternates every 0.5 s
  [int(400 * ((i % 400) / 400)) + 1 for i in range(6000)],
])
def test_a_changing_command_is_never_nudged(seq):
  g, out = feed(seq)
  assert out == seq and g.bursts == 0


def test_a_change_during_the_burst_passes_straight_through():
  g = StuckGuard()
  out = [g.update(300, True) for _ in range(STUCK_HOLD_FRAMES + 1)]
  assert out[-1] == 300 - STUCK_NUDGE
  assert g.update(310, True) == 310                  # the command moved: sent as it is, the burst is over
  assert g.update(310, True) == 310


@pytest.mark.parametrize("seed", range(6))
def test_property_random_sequences(seed):
  rng = random.Random(seed)
  seq, v = [], 0
  while len(seq) < 20000:
    kind = rng.random()
    n = rng.randint(1, 400)
    if kind < 0.3:
      v = rng.choice([0, 600, -600, 1, -1, 2, -2, 599, 650, -570, rng.randint(-1200, 1200)])
      seq += [v] * n                                   # holds, some longer than the hold time
    elif kind < 0.6:
      seq += [v + rng.choice([-1, 0, 1]) for _ in range(n)]
    else:
      step = rng.choice([-6, -3, 3, 6])
      for _ in range(n):
        v = max(-1200, min(1200, v + step))
        seq.append(v)
  act = [rng.random() > 0.02 for _ in seq]
  g = StuckGuard()
  out = [g.update(c, a) for c, a in zip(seq, act)]
  assert longest_identical_run(out, act) <= STUCK_HOLD_FRAMES
  for c, a, o in zip(seq, act, out):
    assert abs(o - c) <= STUCK_NUDGE
    assert abs(o) <= abs(c) and o * c >= 0            # never more, never across zero
    if not a or not eligible(c):
      assert o == c


# ---- through the real CarController.update (ti_cost_check's in-memory params, a packer that records what it is given) ----------------
def _encode_cam_lkas2(values):
  """the 8 bytes of CAM_LKAS2 (opendbc/mazda_2017.dbc: LKAS_REQUEST 3|12@0+, CHKSUM 19|12@0+, KEY 39|32@0+, offsets -2048)"""
  req, chk, key = values["LKAS_REQUEST"] + 2048, values["CHKSUM"] + 2048, values["KEY"]
  return bytes([(req >> 8) & 0x0F, req & 0xFF, (chk >> 8) & 0x0F, chk & 0xFF, (key >> 24) & 0xFF, (key >> 16) & 0xFF,
                (key >> 8) & 0xFF, key & 0xFF])


def _controller(ti_max=None):
  import importlib.util
  import pathlib
  spec = importlib.util.spec_from_file_location("ti_cost_check_for_stuck_guard",
                                                pathlib.Path(__file__).parents[1] / "mazda" / "ti_cost_check.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  cc, _ = mod.make_controller()
  sent = []

  class Packer:
    def make_can_msg(self, name, bus, values):
      if name == "CAM_LKAS2":
        sent.append(_encode_cam_lkas2(values))
      return (0, 0, b"\0" * 8, bus)
  cc.packer = Packer()
  toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
  if ti_max is not None:
    toggles.ti_steer_max = ti_max
  return mod, cc, sent, toggles


def _decode(payload):
  return (((payload[0] & 0x0F) << 8) | payload[1]) - 2048


@pytest.mark.parametrize("ti_max,steer", [(600, 0.5), (650, 1.0), (570, 1.0), (600, 1.0), (1200, -0.25), (100, 0.01)])
def test_controller_held_command_changes_on_the_bus_within_the_hold_time(ti_max, steer):
  mod, cc, sent, toggles = _controller(ti_max)
  CC, CS = mod.inputs(45.0, 0.0, steer=steer)
  outs = []
  for k in range(1500):
    a, _ = cc.update(CC, CS, k * 10_000_000, toggles)
    outs.append(a.steerOutputCan)
  bus = [_decode(p) for p in sent]
  assert len(bus) == len(outs)
  assert bus == outs                                   # steerOutputCan is exactly the payload
  target = int(round(steer * cc.ccp.TI_STEER_MAX))
  assert max(abs(v) for v in bus) <= abs(target) <= cc.ccp.TI_STEER_MAX
  assert cc.lateral_diagnostics.tiLimited == bus[-1]   # the diagnostics report the value sent
  if abs(target) == TI_FW_CLIP:
    assert bus[-1] == target and longest_identical_run(bus) <= 1 and cc.ti_stuck_guard.bursts == 0   # exactly +-600: left alone
  else:
    assert longest_identical_run(bus) <= STUCK_HOLD_FRAMES
    assert cc.ti_stuck_guard.bursts >= 1
    # the limiter takes the command back up from the nudged value it was told was sent: never above its up-rate
    for x, y in zip(bus, bus[1:]):
      if abs(y) > abs(x):
        assert abs(y - x) <= cc.ccp.TI_STEER_DELTA_UP


def test_controller_reports_the_unguarded_command_as_steer_and_logs_bursts_rate_limited(monkeypatch):
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  events = []
  monkeypatch.setattr(ccm, "ti_stuck_log", lambda ev: events.append(ev))
  mod, cc, sent, toggles = _controller(650)
  CC, CS = mod.inputs(45.0, 0.0, steer=1.0)
  steer, can = [], []
  for k in range(3000):
    a, _ = cc.update(CC, CS, k * 10_000_000, toggles)
    steer.append(a.steer)
    can.append(a.steerOutputCan)
  assert any(c != 650 for c in can[200:])
  # controlsd's `steer` stays the limiter's output (the nudge is not a limit it should react to); steerOutputCan is what was sent
  assert all(abs(s * 650 - 650) < 1e-6 for s in steer[200:])
  assert 1 <= len(events) <= 3000 // int(ccm.TI_STUCK_LOG_EVERY_FRAMES) + 1
  assert events[0]["kind"] == "nudge" and events[0]["value"] == 650 and events[-1]["bursts"] >= len(events)


def test_controller_lateral_off_sends_zero_and_the_guard_restarts():
  mod, cc, sent, toggles = _controller(650)
  CC, CS = mod.inputs(45.0, 0.0, steer=1.0)
  for k in range(150):
    cc.update(CC, CS, k * 10_000_000, toggles)
  CC.latActive = False
  for k in range(150, 160):
    a, _ = cc.update(CC, CS, k * 10_000_000, toggles)
    assert a.steerOutputCan == 0
  assert cc.ti_stuck_guard.run == 0 and cc.ti_stuck_guard.last_out is None and cc.ti_stuck_guard.nudge_left == 0


@pytest.mark.parametrize("ti_max,steer", [(650, 1.0), (570, -1.0), (300, 0.5)])
def test_controller_slowest_up_rate_the_return_is_inside_the_limiter(ti_max, steer):
  # TI_STEER_DELTA_UP and _HIGH at their lowest bound (1 count per frame): the step back from the nudge is the limiter's, never 2
  mod, cc, sent, toggles = _controller(ti_max)
  toggles.ti_steer_delta_up, toggles.ti_steer_delta_up_high = 1, 1
  CC, CS = mod.inputs(45.0, 0.0, steer=steer)
  for k in range(2500):
    cc.update(CC, CS, k * 10_000_000, toggles)
  bus = [_decode(p) for p in sent]
  assert cc.ccp.TI_STEER_DELTA_UP == 1 and cc.ccp.TI_STEER_DELTA_UP_HIGH == 1
  target = int(round(steer * ti_max))
  assert target in bus and cc.ti_stuck_guard.bursts >= 1
  assert longest_identical_run(bus) <= STUCK_HOLD_FRAMES
  for x, y in zip(bus, bus[1:]):
    assert abs(y) <= abs(x) + 1 if abs(y) > abs(x) else abs(x - y) <= max(STUCK_NUDGE, cc.ccp.TI_STEER_DELTA_DOWN)


def test_a_stalled_loop_is_judged_in_real_time():
  """Frames arriving late (a control-loop stall): the nudge comes when STUCK_HOLD_S of clock time has passed, not after 80 frames."""
  from openpilot.selfdrive.car.mazda.ti_stuck_guard import StuckGuard, STUCK_HOLD_S, STUCK_NUDGE
  g = StuckGuard()
  t, outs = 0.0, []
  for _ in range(40):                      # 40 frames, each 0.05 s apart: 2.0 s of clock time, far fewer than 80 frames
    outs.append((t, g.update(-570, True, now=t)))
    t += 0.05
  first = next(tt for tt, o in outs if o != -570)
  assert first <= STUCK_HOLD_S + 0.05, first
  assert all(abs(o - (-570)) <= STUCK_NUDGE and abs(o) <= 570 for _, o in outs)
  longest, run, prev = 0.0, 0.0, None      # no value sent unchanged for longer than the hold plus one late frame
  start = 0.0
  for tt, o in outs:
    if o != prev:
      start, prev = tt, o
    longest = max(longest, tt - start)
  assert longest <= STUCK_HOLD_S + 0.05, longest


def test_without_a_clock_the_frame_count_still_rules():
  from openpilot.selfdrive.car.mazda.ti_stuck_guard import StuckGuard, STUCK_HOLD_FRAMES
  g = StuckGuard()
  outs = [g.update(300, True) for _ in range(STUCK_HOLD_FRAMES + 3)]
  assert outs[:STUCK_HOLD_FRAMES] == [300] * STUCK_HOLD_FRAMES and outs[STUCK_HOLD_FRAMES] == 298
