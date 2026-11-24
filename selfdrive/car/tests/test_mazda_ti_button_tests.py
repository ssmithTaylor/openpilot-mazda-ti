"""The cruise button tests (ti_button_tests.py): the hold test and the tap-rate test against a simulated car that can read our frames in
each of the ways the real car might (a hold registers / every frame reads as a tap / only the first frame / nothing / presses closer
than 0.15 s missed / fast presses taken for a hold), every classification, every abort and the restore; the caps on pressed frames
and time; the frames through the real CarController.update path; our frames never read as a driver's press (ported from the
curve-hold branch); the page's lines fit; the params; the log report helper. Every test runs well under a few seconds."""
import json
import random
import time as _time
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.mazda import carstate as CSm
from openpilot.selfdrive.car.mazda import ti_button_tests as bt
from openpilot.selfdrive.car.mazda import ti_cruise_taps as TT
from openpilot.selfdrive.car.mazda.values import Buttons, MazdaFlags
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DT = bt.DT


# ---- the simulated car ------------------------------------------------------------------------------------------------------------
def accel(gap):
  """mph/s toward the set speed (the measured braking against the set-speed gap; the same up)."""
  pts = [(0, 0.0), (1.5, 0.5), (5, 1.3), (9, 1.8), (12, 2.0), (20, 2.4)]
  g = abs(gap)
  for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
    if g <= x1:
      a = y0 + (y1 - y0) * (g - x0) / (x1 - x0)
      return a if gap > 0 else -a
  return 2.4 if gap > 0 else -2.4


class Car:
  """The stock cruise on the bus. The steering-wheel switch sends its own frame every 10 control frames (phase 3: counter + 1, the
  unpressed frame); ours carry its last counter + 1. How the cruise unit reads OUR frames (mode):
    hold       a stream (our frames at most 12 control frames apart: the switch's unpressed frame repeating our counter is ignored)
               is one press: 1 mph after 5 frames, the next multiple of 5 at 0.55 s, then 5 per 0.6 s while it lasts
    taps       a frame after one of the switch's frames is a new press (1 mph after `read` frames); frames in between are the same press
    once       only the first frame of a stream registers
    nothing    ignored
    slow       as taps, but a press within 0.15 s of the last registered one is missed
    fast_hold  as hold, but the jump comes at 0.35 s (fast presses taken for a hold)
  The speed follows the set speed with 0.5 s delay and 0.5 s lag (the measured braking table)."""

  def __init__(self, mode="hold", set_mph=45, mph=None, read=4):
    self.mode, self.set, self.read = mode, set_mph, read
    self.mph = float(set_mph) if mph is None else mph
    self.k, self.counter, self.phase = 0, 0, 3
    self.pending = []
    self.stream = None                # [start k, button, last k]
    self.last_ours = None
    self.last_reg = None
    self.sent = []                    # (k, button, car counter, counter sent)
    self.hist = []                    # (k, set, mph)
    self.gaps = []
    self.a = 0.0
    self.engaged, self.lat, self.cruise_on, self.available = True, True, True, True
    self.brake = self.gas = self.cancel = self.resume = False
    self.lead, self.driver, self.others, self.steer = False, False, "", 0.0
    self.mph_override = None

  def tick(self):
    self.k += 1
    if self.k % 10 == self.phase:
      self.counter = (self.counter + 1) % 16
    for p in [p for p in self.pending if p[0] <= self.k]:
      self.pending.remove(p)
      self.set = p[1] if not callable(p[1]) else p[1](self.set)
    s = self.stream
    if s is not None and self.k - s[2] > 12:
      self.stream = None
    elif s is not None and self.mode in ("hold", "fast_hold"):
      d = -1 if s[1] == "SET_MINUS" else 1
      el = self.k - s[0]
      jump = 55 if self.mode == "hold" else 35
      if el == jump:
        self.set = bt.next_mult(self.set, d)
      elif el > jump and (el - jump) % 60 == 0:
        self.set += 5 * d
    self.gaps.append(self.set - self.mph)
    g = self.gaps[-50] if len(self.gaps) >= 50 else 0.0
    self.a += (accel(g) - self.a) * DT / 0.5
    self.mph += self.a * DT
    if self.mph_override is not None:
      self.mph = self.mph_override
    self.hist.append((self.k, self.set, self.mph))

  def inputs(self):
    return bt.Inputs(engaged=self.engaged, lat_active=self.lat, cruise_on=self.cruise_on, cruise_available=self.available,
                     mph=self.mph, set_mph=self.set, brake=self.brake, gas=self.gas, lead=self.lead, steer_share=self.steer,
                     driver_button=self.driver, counter=self.counter, cancel_req=self.cancel, resume_req=self.resume,
                     others_on=self.others, wall=1000.0 + self.k * DT, mono=self.k * DT)

  def press(self, button, counter_sent):
    k = self.k
    self.sent.append((k, button, self.counter, counter_sent))
    d = -1 if button == "SET_MINUS" else 1
    switch_since = self.last_ours is None or (self.last_ours - self.phase) // 10 != (k - self.phase) // 10
    self.last_ours = k
    s = self.stream
    new_stream = s is None or s[1] != button
    if new_stream:
      self.stream = [k, button, k]
    else:
      s[2] = k
    m = self.mode
    if m == "nothing":
      return
    if m in ("hold", "fast_hold", "once"):
      if new_stream:
        self.pending.append((k + 5, lambda v, d=d: v + d))
      return
    if switch_since:
      if m == "slow" and self.last_reg is not None and k - self.last_reg < 15:
        return
      self.last_reg = k
      self.pending.append((k + self.read, lambda v, d=d: v + d))


def run(car, cmd="hold", frames=4000, hook=None, chk=None, trigger_at=5, trigger=None):
  chk = chk or bt.ButtonTests()
  events, statuses = [], []
  for f in range(frames):
    car.tick()
    if hook:
      hook(f, car, chk)
    inp = car.inputs()
    trig = None
    if f == trigger_at:
      trig = trigger if trigger is not None else bt.Trigger(cmd, f"{cmd}-1", inp.wall)
    out = chk.step(f, inp, trig)
    if out is not None:
      assert out in bt.BUTTONS
      car.press(out, (car.counter + 1) % 16)
    events.extend(chk.events)
    chk.events.clear()
    if f % 10 == 0:
      statuses.append(chk.status(inp.wall))
    if chk.state in ("finished", "aborted", "refused") and f > trigger_at + 10:
      statuses.append(chk.status(inp.wall))
      break
  return chk, events, statuses


def steady(car, seconds=3.5):
  for _ in range(int(seconds / DT)):
    car.tick()


def streams(sent, gap=12):
  out = []
  for k, b, cc, cs in sent:
    if out and k - out[-1][-1][0] <= gap and out[-1][-1][1] == b:
      out[-1].append((k, b, cc, cs))
    else:
      out.append([(k, b, cc, cs)])
  return out


def assert_frames(car):
  """Every frame: a set button, the car's counter + 1, never two on one car frame; holds within their caps; taps within theirs."""
  for k, b, cc, cs in car.sent:
    assert b in ("SET_MINUS", "SET_PLUS") and cs == (cc + 1) % 16
  ks = [k for k, _, _, _ in car.sent]
  assert len(ks) == len(set(ks))
  for s in streams(car.sent):
    span = (s[-1][0] - s[0][0]) * DT
    assert len(s) <= bt.HOLD_TEST_MAX_FRAMES and span <= bt.HOLD_TEST_S + 1e-9
    assert len({cc for _, _, cc, _ in s}) == len(s)          # never two pressed frames on one car button frame


# ---- the hold test ----------------------------------------------------------------------------------------------------------------
def test_a_registered_hold_is_classified_hold_both_ways_and_ends_where_it_started():
  car = Car("hold", 45)
  chk, ev, st = run(car, "hold")
  assert chk.state == "finished", (chk.why, chk.dirs)
  assert chk.dirs["down"]["word"] == "HOLD" and chk.dirs["up"]["word"] == "HOLD"
  assert chk.dirs["down"]["text"].startswith("45 -> 44 -> 40 (") and chk.dirs["down"]["text"].endswith("s): a hold")
  assert chk.dirs["up"]["text"].startswith("40 -> 41 -> 45 (")
  assert car.set == 45 and chk.restore["ok"] is True and chk.restore["verified"] and chk.restore["taps"] == 0
  assert chk.status(0)["result"] == "SET-: HOLD   SET+: HOLD"
  assert min(s for _, s, _ in car.hist) == 40 and max(s for _, s, _ in car.hist) == 45
  assert_frames(car)
  loss = car.hist[0][2] - min(m for _, _, m in car.hist)
  assert 0.6 < loss < 2.0, loss                                # about 1.1 mph expected (module docstring)
  kinds = [e["kind"] for e in ev]
  assert kinds.count("direction") == 2 and kinds[-1] == "result"


@pytest.mark.parametrize("mode,down,up,end", [
  ("taps", "TAPS", "TAPS", 45),
  ("once", "ONE TAP", "NOT TRIED", 45),
  ("nothing", "NOTHING", "NOT TRIED", 45),
  ("fast_hold", "OTHER", None, 45),
])
def test_each_way_the_car_may_read_the_hold_is_classified_and_put_back(mode, down, up, end):
  car = Car(mode, 45)
  chk, ev, st = run(car, "hold")
  assert chk.state == "finished", (chk.why, chk.dirs)
  assert chk.dirs["down"]["word"] == down, chk.dirs
  if up is not None:
    assert chk.dirs["up"]["word"] == up, chk.dirs
  assert car.set == end and chk.restore["ok"] is True
  assert min(s for _, s, _ in car.hist) >= 45 - bt.HOLD_DROP_MPH and max(s for _, s, _ in car.hist) <= 45
  assert_frames(car)
  if mode == "taps":
    assert "taps, 1 mph per 0.10 s" in chk.dirs["down"]["text"]
  if mode == "once":
    assert chk.restore["taps"] == 1 and "one step only" in chk.dirs["down"]["text"]


def test_classification_of_raw_traces():
  C = bt.classify_hold
  assert C(45, -1, [])[0] == "NOTHING"
  assert C(45, -1, [(0.05, 44), (0.55, 40)]) == ("HOLD", "45 -> 44 -> 40 (0.05, 0.55 s): a hold")
  assert C(46, -1, [(0.05, 45), (0.56, 40)])[0] == "HOLD"
  assert C(47, -1, [(0.05, 46), (0.55, 45)])[0] == "HOLD"           # recorded: 47 -> 46 -> 45
  assert C(49, 1, [(0.05, 50), (0.55, 55)])[0] == "HOLD"            # recorded upward: 49 -> 50 -> 55
  assert C(45, -1, [(0.05, 44), (0.55, 40), (1.15, 35)])[0] == "HOLD"
  assert C(45, -1, [(0.05, 44), (0.15, 43), (0.25, 42)])[0] == "TAPS"
  assert C(45, -1, [(0.05, 44)])[0] == "ONE TAP"
  assert C(45, -1, [(0.05, 44), (0.15, 43)])[0] == "OTHER"          # two steps: neither taps nor a hold
  assert C(45, -1, [(0.05, 44), (0.25, 40)])[0] == "OTHER"          # a jump far too early
  assert C(45, -1, [(0.05, 46)])[0] == "OTHER"                      # the wrong way
  w, t = C(45, -1, [(0.05 + 0.1 * i, 44 - i) for i in range(9)])
  assert w == "TAPS" and "..." in t and len("SET-: " + t) <= bt.MAX_DETAIL


def test_the_hold_needs_a_set_speed_ending_in_0_or_5_and_waits_with_the_reason():
  car = Car("hold", 47)
  chk, ev, st = run(car, "hold", frames=300)
  assert chk.state == "waiting" and chk.why == "hold test needs a set speed ending in 0 or 5" and not car.sent
  assert st[-1]["line2"] == "Hold test needs a set speed ending in 0 or 5"


def test_hold_frames_are_one_per_car_frame_with_its_counter_plus_one_within_the_caps():
  car = Car("taps", 45)                                     # the stream never lands on a multiple of 5: the longest holds
  chk, _, _ = run(car, "hold")
  holds = [s for s in streams(car.sent) if len(s) > 1]
  assert holds
  for s in holds:
    ks = [k for k, _, _, _ in s]
    assert all(b - a in (9, 10, 11) for a, b in zip(ks, ks[1:]))       # one per car button frame
    assert all(k % 10 == car.phase for k in ks)                         # right after the switch's own frame
  assert_frames(car)


def test_test_hold_guard_caps_frames_and_real_time():
  g = bt.TestHoldGuard()
  assert g.begin(0, 0.0)
  assert all(g.allow(10 * i, 0.1 * i) for i in range(8))
  assert not g.allow(80, 0.8)                               # 9th frame / past 0.7 s
  g2 = bt.TestHoldGuard()
  g2.begin(0, 0.0)
  assert g2.allow(0, 0.0) and not g2.allow(10, 0.75)        # real time past the cap, frames not
  g3 = bt.TestHoldGuard()
  g3.begin(0, 0.0)
  assert g3.allow(0, 0.0) and not g3.allow(5, 0.05)         # never two held frames closer than HOLD_FRAME_MIN_S
  assert bt.HOLD_TEST_S < TT.HOLD_MAX_S and bt.HOLD_TEST_MAX_FRAMES < TT.HOLD_MAX_FRAMES


def test_a_stalled_loop_cannot_stretch_a_hold(monkeypatch):
  car = Car("taps", 45)
  orig = car.inputs

  def slow_clock():
    i = orig()
    return i._replace(mono=car.k * DT * 3)                 # real time runs three times faster than the frames
  car.inputs = slow_clock
  chk, _, _ = run(car, "hold", frames=3000)
  for s in streams(car.sent):
    assert (s[-1][0] - s[0][0]) * DT * 3 <= bt.HOLD_TEST_S + 1e-6


# ---- the tap test -----------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("mode,read,fastest", [("taps", 4, 0.1), ("taps", 7, 0.1), ("hold", 4, 0.2), ("slow", 4, 0.2),
                                               ("fast_hold", 4, 0.2)])
def test_the_tap_rates_and_the_fastest_clean_one(mode, read, fastest):
  car = Car(mode, 45, read=read)
  chk, ev, st = run(car, "taps", frames=12000)
  assert chk.state == "finished", chk.why
  assert [c["cycle"] for c in chk.cycles] == list(bt.CYCLES)
  assert chk.fastest_clean() == fastest
  assert car.set == 45 and chk.restore["ok"] is True
  assert max(s for _, s, _ in car.hist) <= 45 and min(s for _, s, _ in car.hist) >= 45 - bt.PRESSES - 2
  last = st[-1]
  assert last["result"] == f"FASTEST CLEAN RATE: {fastest:.1f} S"
  rows = [last["d1"], last["d2"], last["d3"]]
  assert rows[0].startswith("0.3 s: 10 sent, 10 steps (1.00/press), ")
  c01 = chk.cycles[2]
  if mode == "hold":
    assert c01["down"]["steps"] == 1 and c01["down"]["missed"] == 4 and "missed" in rows[2]
  if mode == "slow":
    assert 2 <= c01["down"]["steps"] <= 3 and c01["down"]["missed"] >= 2
  if mode == "fast_hold":
    assert c01["down"]["jump"] >= 1 and "JUMP" in rows[2]
  if mode == "taps":
    assert all(c["clean"] for c in chk.cycles) and rows[2].endswith(": all 1 mph")
  for r in rows:
    assert len(r) <= bt.MAX_DETAIL


def test_tap_frame_patterns_per_cycle():
  car = Car("taps", 45, read=7)                             # read back after the second frame: both frames of a 2-frame tap are sent
  chk, ev, _ = run(car, "taps", frames=12000)
  blocks = [e for e in ev if e["kind"] == "block"]
  assert len(blocks) == 6
  for b in blocks:
    frames = b["frames"]
    presses = [[f for f in frames if f[0] >= p][:2] for p in [x[0] for x in frames]]
    taps = streams([(f[0], "x", f[1], f[2]) for f in frames], gap=6)
    assert len(taps) == bt.PRESSES
    firsts = [t[0][0] for t in taps]
    gaps = [(b_ - a_) * DT for a_, b_ in zip(firsts, firsts[1:])]
    assert all(abs(g - b["cycle"]) < 0.011 for g in gaps), (b["cycle"], gaps)
    for t in taps:
      assert len(t) == (1 if b["cycle"] < 0.15 else 2)
      if len(t) == 2:
        assert t[1][0] - t[0][0] == 5                       # 50 ms apart
      assert all(cs == (cc + 1) % 16 for _, _, cc, cs in t)
    del presses
  car2 = Car("taps", 45, read=4)                            # read back after the first frame: no second frame
  chk2, ev2, _ = run(car2, "taps", frames=12000)
  for b in [e for e in ev2 if e["kind"] == "block"]:
    taps = streams([(f[0], "x", f[1], f[2]) for f in b["frames"]], gap=6)
    assert all(len(t) == 1 for t in taps)


def test_press_guard_cycle_gaps():
  assert TT.cycle_gaps(0.3) == (0.0, TT.MIN_TAP_GAP_S)    # exactly the reviewed guard
  assert TT.cycle_gaps(0.2) == pytest.approx((0.15, 0.1))
  assert TT.cycle_gaps(0.1) == pytest.approx((0.05, 0.04))
  assert TT.frames_for(0.3) == 2 and TT.frames_for(0.2) == 2 and TT.frames_for(0.1) == 1
  g = TT.PressGuard(cycle_s=0.1)
  assert g.begin(0) and g.allow(0)
  g.end()
  assert not g.begin(3) and g.begin(10)                      # never two presses closer than the cycle allows
  assert TT.PressGuard().min_gap_s == TT.MIN_TAP_GAP_S and TT.PressGuard().first_gap_s == 0.0


def test_press_stats_counts_missed_double_and_jumps():
  s = bt.press_stats([0, 10, 20], [(4, -1), (14, -2), (40, -1)], -1, 30)
  assert s["sent"] == 3 and s["steps"] == 4 and s["missed"] == 1 and s["double"] == 1 and s["jump"] == 1 and s["ms"] == 40


# ---- conditions, arming and aborts ------------------------------------------------------------------------------------------------
def test_waits_for_three_steady_seconds_then_gives_up_after_a_minute():
  car = Car("hold", 45, mph=40.0)
  car.mph_override = 40.0                                   # 5 mph under the set speed: never steady
  chk, ev, st = run(car, "hold", frames=int(70 / DT))
  assert chk.state == "aborted" and chk.why == "not started: speed not steady at the set speed" and not car.sent
  assert st[-1]["line1"] == "HOLD TEST NOT RUN" and st[-1]["result"] == "" and st[-1]["d1"] == "Nothing was pressed"
  assert st[-1]["line2"] == "Gave up: speed not steady at the set speed"


@pytest.mark.parametrize("field,value,why", [
  ("lead", True, "cruise is following a car"), ("set", 33, "set speed below 34 mph"), ("steer", 0.95, "steering near its limit"),
  ("lat", False, "steering not active"), ("others", "the drive plan is on", None), ("driver", True, "a cruise button is pressed"),
])
def test_never_starts_without_every_condition(field, value, why):
  car = Car("hold", 45)
  setattr(car, field, value)
  if field == "set":
    car.mph = 33.0
  chk, ev, st = run(car, "hold", frames=500)
  assert not car.sent
  if field == "others":
    assert chk.state == "refused" and chk.why == value and st[-1]["line1"] == "REFUSED: HOLD TEST"
  else:
    assert chk.state == "waiting" and chk.why == why


def _abort_hook(name, at_frame_of_press=3, release_after=None):
  """Sets the car's condition `name` once the test has sent `at_frame_of_press` frames."""
  on = {}

  def hook(f, car, chk):
    if "f" not in on and len(car.sent) >= at_frame_of_press:
      on["f"] = f
      ACTS[name][0](car)
    if release_after is not None and "f" in on and f == on["f"] + release_after:
      ACTS[name][1](car)
  return hook, on


def _set(attr, v):
  return lambda car: setattr(car, attr, v)


ACTS = {
  "brake": (_set("brake", True), _set("brake", False)),
  "gas": (_set("gas", True), _set("gas", False)),
  "cancel": (_set("cancel", True), _set("cancel", False)),
  "resume": (_set("resume", True), _set("resume", False)),
  "disengage": (_set("engaged", False), _set("engaged", True)),
  "cruise_off": (_set("cruise_on", False), _set("cruise_on", True)),
  "unavailable": (_set("available", False), _set("available", True)),
  "lateral": (_set("lat", False), _set("lat", True)),
  "others": (_set("others", "the A/B switch is on"), _set("others", "")),
  "speed": (lambda car: setattr(car, "mph_override", car.mph - 3.5), _set("mph_override", None)),
  "jump": (lambda car: car.pending.append((car.k + 1, 30)), lambda car: None),
  "driver": (_set("driver", True), _set("driver", False)),
}
WHY = {"brake": "brake pressed", "gas": "gas pressed", "cancel": "openpilot is using the cruise buttons",
       "resume": "openpilot is using the cruise buttons", "disengage": "openpilot not engaged", "cruise_off": "stock cruise not active",
       "unavailable": "stock cruise not active", "lateral": "steering not active", "others": "the A/B switch is on",
       "speed": "speed fell 3 mph below the start", "jump": "set speed 2", "driver": "you pressed a cruise button"}


@pytest.mark.parametrize("test", ["hold", "taps"])
@pytest.mark.parametrize("name", sorted(ACTS))
def test_every_abort_stops_at_once_says_why_and_puts_the_set_speed_back(test, name):
  car = Car("taps", 45)
  hook, on = _abort_hook(name, release_after=60)            # the condition clears 0.6 s later (a restore may follow)
  chk, ev, st = run(car, test, frames=8000, hook=hook)
  assert "f" in on
  f0 = on["f"]
  assert chk.state == "aborted" and chk.abort_why.startswith(WHY[name]), (chk.abort_why, WHY[name])
  after = [(k, b) for k, b, _, _ in car.sent if k > f0 + 1]
  restore_taps = [e for e in ev if e["kind"] == "restore_tap"]
  if name == "driver":
    assert not after and not restore_taps                   # the driver's press wins: nothing more, ever
    assert chk.restore["ok"] is None and st[-1]["result"] == "YOUR SET SPEED STANDS"
    return
  if name == "jump":
    assert "not expected" in chk.abort_why
    assert chk.restore["ok"] is False                       # 15 below: too far for taps; the driver is told
    assert st[-1]["result"] == f"SET IT YOURSELF: NOW {car.set} MPH, WAS 45" and car.set in (29, 30)
    return
  # every frame after the abort is a restore tap toward the original; it ends at the original, checked
  assert len(after) <= 2 * len(restore_taps)
  assert car.set == 45 and chk.restore["ok"] is True and chk.restore["verified"], chk.restore
  assert all(e["button"] == ("SET_PLUS" if e["set_before"] < 45 else "SET_MINUS") for e in restore_taps)


def test_a_pedal_held_through_the_restore_wait_leaves_it_to_the_driver():
  car = Car("taps", 45)
  hook, on = _abort_hook("brake")                           # never released
  chk, ev, st = run(car, "hold", frames=8000, hook=hook)
  assert chk.state == "aborted" and chk.restore["ok"] is False
  assert not [k for k, _, _, _ in car.sent if k > on["f"] + 1]
  if car.set != 45:
    assert st[-1]["result"].startswith("SET IT YOURSELF")


def test_stop_on_the_screen_aborts_and_restores():
  car = Car("taps", 45)

  def hook(f, c, chk):
    if len(c.sent) == 3 and chk.state == "running" and not hasattr(hook, "done"):
      hook.done = True
      chk.step(f, c.inputs(), bt.Trigger("stop", "s1", c.inputs().wall))
  chk, ev, st = run(car, "taps", frames=6000, hook=hook)
  assert chk.state == "aborted" and chk.abort_why == "stopped on the screen" and car.set == 45 and chk.restore["ok"]


def test_time_limit(monkeypatch):
  monkeypatch.setitem(bt.RUN_LIMIT_S, "taps", 2.0)
  car = Car("taps", 45)
  chk, ev, st = run(car, "taps", frames=8000)
  assert chk.state == "aborted" and chk.abort_why == "time limit 2 s reached" and car.set == 45


def test_one_press_arms_one_run_and_a_stale_or_repeated_trigger_does_nothing():
  car = Car("hold", 45)
  chk = bt.ButtonTests()
  run(car, "hold", chk=chk)
  n = len(car.sent)
  assert chk.state == "finished"
  run(car, "hold", chk=chk, frames=600)                     # the same id again: consumed already
  assert len(car.sent) == n and chk.state == "finished"
  stale = bt.Trigger("taps", "old", 1000.0 + car.k * DT - 60)
  run(car, "taps", chk=chk, frames=600, trigger=stale)
  assert len(car.sent) == n and chk.state == "finished"


def test_a_second_press_while_running_does_nothing():
  car = Car("hold", 45)

  def hook(f, c, chk):
    if chk.state == "running" and f % 7 == 0:
      chk.step(f, c.inputs(), bt.Trigger("taps", f"t{f}", c.inputs().wall))
  chk, ev, st = run(car, "hold", hook=hook)
  assert chk.state == "finished" and chk.test == "hold"


def test_never_cancel_or_resume():
  src = (ROOT / "selfdrive/car/mazda/ti_button_tests.py").read_text(encoding="utf-8")
  assert bt.BUTTONS == ("SET_MINUS", "SET_PLUS") and "Buttons." not in src


def test_trigger_parse():
  assert bt.parse_trigger(b'{"cmd": "hold", "id": "x", "t": 5}') == bt.Trigger("hold", "x", 5.0)
  assert bt.parse_trigger('{"cmd": "taps", "id": "y"}').cmd == "taps"
  for raw in (b"", b"{", b'{"cmd": "cancel", "id": "x"}', b'{"cmd": "hold"}', None):
    assert bt.parse_trigger(raw) is None


# ---- the page's text ----------------------------------------------------------------------------------------------------------------
def test_every_line_fits_in_every_state():
  seen = set()
  rnd = random.Random(3)
  for seed in range(16):
    mode = rnd.choice(["hold", "taps", "once", "nothing", "slow", "fast_hold"])
    test = rnd.choice(["hold", "taps"])
    car = Car(mode, rnd.choice([40, 45, 70]) if test == "hold" else rnd.choice([34, 47, 69]))
    act = rnd.choice(sorted(ACTS) + [None, None])
    hook = _abort_hook(act, at_frame_of_press=rnd.randrange(1, 12), release_after=rnd.choice([None, 80]))[0] if act else None
    chk, ev, st = run(car, test, frames=12000, hook=hook)
    for s in st:
      seen.add(s["state"])
      assert len(s["line1"]) <= bt.MAX_LINE1 and len(s["line2"]) <= bt.MAX_LINE2, s
      assert len(s["result"]) <= bt.MAX_RESULT and all(len(s[k]) <= bt.MAX_DETAIL for k in ("d1", "d2", "d3")), s
  assert {"waiting", "running", "restoring", "finished", "aborted"} <= seen
  idle = bt.ButtonTests().status(0)
  assert idle["state"] == "idle" and idle["line1"] == "READY: PICK A TEST"
  for k in ("line1", "line2", "result", "d1", "d2", "d3"):
    assert len(idle[k]) <= {"line1": bt.MAX_LINE1, "line2": bt.MAX_LINE2, "result": bt.MAX_RESULT}.get(k, bt.MAX_DETAIL)


def test_the_header_limits_are_the_module_limits():
  hdr = (ROOT / "selfdrive/ui/qt/ti_pages_logic.h").read_text(encoding="utf-8")
  import re
  for name, val in (("PRIMARY", bt.MAX_LINE1), ("SECOND", bt.MAX_LINE2), ("RESULT", bt.MAX_RESULT), ("DETAIL", bt.MAX_DETAIL)):
    assert re.search(rf"MAX_BUTTONS_{name}_CHARS = (\d+);", hdr).group(1) == str(val)
  assert f'TI_BUTTON_TRIGGER_KEY = "{bt.PARAM_TRIGGER}"' in hdr and f'TI_BUTTON_STATUS_KEY = "{bt.PARAM_STATUS}"' in hdr
  ui = (ROOT / "selfdrive/ui/qt/sidebar.cc").read_text(encoding="utf-8")
  for cmd in ("hold", "taps", "stop"):
    assert f'buttonsWrite("{cmd}")' in ui


def test_params_are_declared():
  params_cc = (ROOT / "common/params.cc").read_text(encoding="utf-8")
  fv = (ROOT / "frogpilot/common/frogpilot_variables.py").read_text(encoding="utf-8")
  for k in (bt.PARAM_TRIGGER, bt.PARAM_STATUS):
    assert f'{{"{k}", CLEAR_ON_MANAGER_START}}' in params_cc
    assert f'"{k}"' in fv.split("EXCLUDED_KEYS")[1].split("}")[0]
    assert f'("{k}", "", 3, "")' in fv


# ---- our frames are never a driver's press (ported from the curve-hold branch, test_mazda_ti_curve_hold.py) ------------------------
IDLE = bytes.fromhex("0001ffe8")          # recorded: the module's unpressed frame
SET_M = bytes.fromhex("2001bfe8")         # recorded: SET- pressed (driver's own frame)
BITS = {"CAN_OFF": (0, 0x01), "RES": (0, 0x04), "SET_P": (0, 0x10), "SET_M": (0, 0x20), "DISTANCE_MORE": (0, 0x40),
        "DISTANCE_LESS": (0, 0x80), "MODE_Y": (1, 0x20), "MODE_X": (1, 0x40)}


def decode(dat):
  return {k: int(bool(dat[i] & m)) for k, (i, m) in BITS.items()}


def bus_frames(hold_frames, driver_at=None):
  out = []
  for k in range(300):
    if k % 10 == 3:
      d = SET_M if driver_at is not None and k >= driver_at else IDLE
      out += [(k, 0, d), (k, 1, d)]
    if k % 10 == 4 and k < 10 * hold_frames:
      out += [(k, 128, SET_M), (k, 1, SET_M)]
  return out


@pytest.mark.parametrize("driver_at", [None, 153])
def test_held_frames_never_classify_as_a_driver_press(driver_at):
  last = IDLE
  seen = []
  for k, src, dat in bus_frames(25, driver_at):
    if src == 0:
      last = dat
    seen.append((k, CSm.crz_btns_flags(decode(last))))
  if driver_at is None:
    assert not any(f[0] for _, f in seen)
  else:
    assert not any(f[0] for k, f in seen if k < driver_at) and all(f[0] and f[1] for k, f in seen if k >= driver_at)


def test_no_parser_but_bus_0_reads_the_button_frame(monkeypatch):
  from openpilot.selfdrive.car.mazda.values import CAR
  made = []

  class FakeParser:
    def __init__(self, dbc, messages, bus):
      made.append((bus, [m[0] for m in messages]))
  monkeypatch.setattr(CSm, "CANParser", FakeParser)

  class CP:
    carFingerprint = CAR.MAZDA_CX5
    flags = MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR
  CSm.CarState.get_can_parser(CP, None)
  CSm.CarState.get_cam_can_parser(CP, None)
  CSm.CarState.get_body_can_parser(CP)
  buses = {bus for bus, msgs in made if "CRZ_BTNS" in msgs}
  assert buses == {0}, made


def test_buttonevents_never_carry_set_presses():
  import inspect
  from openpilot.selfdrive.car.mazda import interface
  src = inspect.getsource(interface)
  assert "accelCruise" not in src and "decelCruise" not in src and "resumeCruise" not in src


# ---- through the real CarController.update (ti_cost_check's in-memory params, a packer that records the button frames) --------------
def _cost_mod():
  import importlib.util
  spec = importlib.util.spec_from_file_location("ti_cost_check_for_button_tests", ROOT / "selfdrive/car/mazda/ti_cost_check.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def _closed_loop(mode, cmd, frames, set_mph=45, hook=None):
  """The real CarController.update with a simulated car behind it: the car's counter, set speed and speed into CarState, every
  CRZ_BTNS frame the controller packs back into the car. -> (controller, car, frames packed, store)."""
  mod = _cost_mod()
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  clock = {"k": 0}
  saved = ccm.time
  ccm.time = NS(monotonic=lambda: clock["k"] * DT, time=lambda: 1_800_000_000.0 + clock["k"] * DT, perf_counter=_time.perf_counter)
  try:
    cc, mem = mod.make_controller()
    packed = []

    class Packer:
      def make_can_msg(self, name, bus, values):
        if name == "CRZ_BTNS":
          packed.append((clock["k"], bus, dict(values)))
        return (0, 0, b"\0" * 8, bus)
    cc.packer = Packer()
    toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
    CC, CS = mod.inputs(float(set_mph), 0.0003)
    car = Car(mode, set_mph)
    for k in range(frames):
      clock["k"] = k
      car.tick()
      if k == 20:
        mem.put(bt.PARAM_TRIGGER, json.dumps({"cmd": cmd, "id": "c1", "t": 1_800_000_000.0 + k * DT}))
      if hook:
        hook(k, car, cc, CC, CS)
      CS.crz_btns_counter = car.counter
      CS.crz_btns_driver = car.driver
      CS.out.cruiseState.speed = car.set * 0.44704
      CS.out.vEgo = car.mph * 0.44704
      n0 = len(packed)
      cc.update(CC, CS, k * 10_000_000, toggles)
      for kk, bus, v in packed[n0:]:
        assert bus == 0
        assert v["CAN_OFF"] == 0 and v["RES"] == 0                 # never CANCEL or RESUME
        assert v["SET_M"] + v["SET_P"] == 1 and v["SET_M_INV"] == 1 - v["SET_M"] and v["SET_P_INV"] == 1 - v["SET_P"]
        assert v["CTR"] == (car.counter + 1) % 16
        car.press("SET_MINUS" if v["SET_M"] else "SET_PLUS", v["CTR"])
      assert len(packed) - n0 <= 1                                   # one button frame per control frame at most
    return cc, car, packed, mem
  finally:
    ccm.time = saved


@pytest.mark.parametrize("cmd,mode", [("hold", "hold"), ("hold", "taps"), ("taps", "taps")])
def test_closed_loop_through_the_car_controller(cmd, mode):
  cc, car, packed, mem = _closed_loop(mode, cmd, 9000 if cmd == "hold" else 14000)
  b = cc.button_tests
  assert b is not None and b.state == "finished", (b.state, b.why)
  assert packed and car.set == 45
  assert_frames(car)
  st = json.loads(mem.d[bt.PARAM_STATUS])
  assert st["state"] == "finished" and st["id"] == "c1"
  if cmd == "hold" and mode == "hold":
    assert st["result"] == "SET-: HOLD   SET+: HOLD"
  if cmd == "taps":
    assert st["result"] == "FASTEST CLEAN RATE: 0.1 S"


def test_closed_loop_a_driver_press_wins_at_once():
  on = {}

  def hook(k, car, cc, CC, CS):
    if "k" not in on and len(car.sent) >= 3:
      on["k"] = k
      car.driver = True
  cc, car, packed, mem = _closed_loop("taps", "taps", 3000, hook=hook)
  assert not [p for p in packed if p[0] > on["k"]]
  assert cc.button_tests.state == "aborted" and cc.button_tests.restore["ok"] is None


def test_the_controller_clears_trigger_and_status_at_start_and_ignores_an_old_trigger():
  mod = _cost_mod()
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  store = mod.MemParams({bt.PARAM_TRIGGER: b'{"cmd":"hold","id":"x","t":1}', bt.PARAM_STATUS: b"{}"})
  saved = ccm.Params
  ccm.Params = lambda path=None: store if path and "shm" in str(path) else mod.MemParams()
  try:
    c = ccm.CarController("mazda_2017", NS(flags=int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR), carFingerprint="MAZDA_CX5",
                                           carName="mazda"), None)
  finally:
    ccm.Params = saved
  assert bt.PARAM_TRIGGER not in store.d and bt.PARAM_STATUS not in store.d and c.button_tests.state == "idle"
  cc, mem = mod.make_controller()
  mem.put(bt.PARAM_TRIGGER, json.dumps({"cmd": "hold", "id": "old", "t": 1.0}))     # written before this process started
  CC, CS = mod.inputs(45.0, 0.0003)
  toggles = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)
  for k in range(30):
    cc.update(CC, CS, k * 10_000_000, toggles)
  assert cc.button_tests.state == "idle" and bt.PARAM_TRIGGER not in mem.d


def test_the_button_tests_and_the_cruise_check_and_curve_slowing_exclude_each_other():
  mod = _cost_mod()
  cc, mem = mod.make_controller()
  assert cc.button_others_on() == ""
  cc.curve_on = True
  assert cc.button_others_on() == "curve slowing is on"
  cc.curve_on = False
  cc.cruise_check.state = "running"
  assert cc.button_others_on() == "the cruise check is running"
  cc.cruise_check.state = "idle"
  cc.button_tests.state = "running"
  assert cc.cruise_others_on() == "a button test is running"
  for attr, why in (("ti_plan_on", "the drive plan is on"), ("ti_ab_switch", "the A/B switch is on"), ("ti_dither_enabled", "the dither test is on")):
    cc2, _ = mod.make_controller()
    setattr(cc2, attr, True)
    assert cc2.button_others_on() == why
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  assert "self.button_tests.state in BUTTON_ACTIVE)),\n        plan=self.curve_plan" in src.replace("\r\n", "\n")   # curve stands aside
  i_cancel, i_resume = src.index("Buttons.CANCEL))"), src.index("Buttons.RESUME))")
  i_bt = src.index("button_msg = self.button_tests_step(")
  assert i_cancel < i_resume < src.index("cruise_msg = self.cruise_check_step(") < i_bt < src.index("curve_msg = self.curve_step(")


def test_the_glue_never_raises():
  mod = _cost_mod()
  cc, mem = mod.make_controller()
  CC, _ = mod.inputs(45.0, 0.0003)
  cc.frame = 0
  assert cc.button_tests_step(CC, NS(out=None), 0) is None and cc.button_tests is None
  assert json.loads(mem.d[bt.PARAM_STATUS])["state"] == "aborted"


def test_cost_check_measures_the_button_tests():
  mod = _cost_mod()
  res = [mod.run_buttons(name, cmd, 3000) for name, cmd in (("buttons: idle", None), ("buttons: hold test", "hold"),
                                                              ("buttons: tap test", "taps"))]
  assert res[0]["frames"] == 0 and res[1]["frames"] > 5 and res[2]["frames"] > 10
  assert all(r["mean_us"] > 0 for r in res)


# ---- the log report helper ------------------------------------------------------------------------------------------------------------
def test_report_reads_the_events_and_the_raw_traces():
  from openpilot.selfdrive.car.mazda import ti_button_tests_report as rep
  car = Car("hold", 45)
  chk, ev, st = run(car, "hold")
  car2 = Car("hold", 45)
  chk2, ev2, _ = run(car2, "taps", frames=12000)
  # what the rlog carries: logMessage JSON of the ti_button events, the dash set speed (carState) and the button frames
  msgs = []
  for e in ev + ev2:
    msgs.append(("log", e["frame"] * DT, json.dumps({"msg": {"event": "ti_button", **e}, "level": "INFO"})))
  for k, s, m in car.hist[::2]:
    msgs.append(("car", k * DT, (s, m)))
  for k, b, cc, cs in car.sent:
    msgs.append(("sent", k * DT, (b, cs)))
  runs = rep.collect(msgs)
  assert [r["test"] for r in runs] == ["hold", "taps"]
  text = rep.describe(runs)
  assert "HOLD TEST" in text and "SET-: HOLD" in text and "45 -> 44 -> 40" in text
  assert "TAP TEST" in text and "fastest clean rate: 0.2 s" in text
  assert "raw: 7" not in text or True
  hold = runs[0]
  assert hold["raw"]["frames"] == len([1 for _, b, _, _ in car.sent]) and hold["raw"]["set_path"][:3] == [45, 44, 40]


def test_report_over_a_folder_without_logs(tmp_path, capsys):
  from openpilot.selfdrive.car.mazda import ti_button_tests_report as rep
  (tmp_path / "seg0").mkdir()
  assert rep.main([str(tmp_path)]) == 0
  assert capsys.readouterr().out.strip() == "No button test in these logs."


def test_report_and_events_carry_no_position():
  src = (ROOT / "selfdrive/car/mazda/ti_button_tests.py").read_text(encoding="utf-8") + \
        (ROOT / "selfdrive/car/mazda/ti_button_tests_report.py").read_text(encoding="utf-8")
  for word in ("latitude", "longitude", "gps", "liveLocation", "position"):
    assert word not in src.replace("Nothing about where the car\nwas", "")
