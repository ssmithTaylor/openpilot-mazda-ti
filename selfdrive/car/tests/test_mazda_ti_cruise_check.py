"""The cruise set-speed check (ti_cruise_check.py): the frames it sends (against the recorded real-press templates), the hard cap on a
press, the sequence against simulated cruises that accept, ignore or hold, every abort path, the restore, the exclusion with the plan,
the alternation and the tests, "off after restart", and the screen's state against the controller's own (the page logic compiled
when a C++ compiler is available). Every test here runs well under a second."""
import ast
import json
import random
import re
import shutil
import subprocess
import tempfile
import os
import glob
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.mazda import ti_cruise_check as cc
from openpilot.selfdrive.car.mazda import mazdacan
from openpilot.selfdrive.car.mazda.values import Buttons, MazdaFlags

ROOT = Path(__file__).resolve().parents[3]

# ---- the frames ----------------------------------------------------------------------------------------------------------------
# The real-press templates of 0x09D, from 1.2 million recorded car frames (E:/openpilot-mazda-ti-data/cruise-buttons-20261122/
# RESULTS.txt): [b0, b1, b2, b3 = 0xC0 | ctr << 2, 0, 0, 0, 0], no checksum. Every recorded comma frame reproduced exactly too.
TEMPLATES = {"none": (0x00, 0x01, 0xFF), "SET_P": (0x10, 0x01, 0xDF), "SET_M": (0x20, 0x01, 0xBF), "RES": (0x04, 0x01, 0xF7),
             "CAN_OFF": (0x01, 0x01, 0xFD)}


def template(name, ctr):
  a, b, c = TEMPLATES[name]
  return bytes([a, b, c, 0xC0 | ((ctr & 15) << 2), 0, 0, 0, 0])


def _dbc_signals(msg="CRZ_BTNS", dbc="mazda_2017.dbc"):
  text = (ROOT / "opendbc" / dbc).read_text(encoding="utf-8", errors="replace")
  block = text[text.index(f" {msg}:"):]
  block = block[:block.index("\n\n")]
  return {m.group(1): (int(m.group(2)), int(m.group(3))) for m in re.finditer(r"SG_ (\w+) : (\d+)\|(\d+)@0\+", block)}


def pack(values):
  """A minimal big-endian (Motorola, @0) packer from the DBC, independent of opendbc's compiled one."""
  sig = _dbc_signals()
  data = bytearray(8)
  assert set(values) == set(sig), set(values) ^ set(sig)
  for name, v in values.items():
    start, length = sig[name]
    bit = start
    for k in range(length - 1, -1, -1):          # MSB at the start bit, walking down the byte, then into the next byte's MSB
      if (int(v) >> k) & 1:
        data[bit // 8] |= 1 << (bit % 8)
      bit = bit - 1 if bit % 8 else bit + 15
  return bytes(data)


class CapturePacker:
  def make_can_msg(self, name, bus, values):
    return (name, bus, dict(values))


CP = NS(flags=int(MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR))


def frame_for(button, counter):
  name, bus, values = mazdacan.create_button_cmd(CapturePacker(), CP, counter, button)
  assert (name, bus) == ("CRZ_BTNS", 0)
  return pack(values)


@pytest.mark.parametrize("counter", [0, 5, 14, 15])
def test_every_button_frame_is_the_recorded_template_with_the_next_counter(counter):
  nxt = (counter + 1) % 16
  assert frame_for(Buttons.NONE, counter) == template("none", nxt)
  assert frame_for(Buttons.SET_MINUS, counter) == template("SET_M", nxt)
  assert frame_for(Buttons.SET_PLUS, counter) == template("SET_P", nxt)
  assert frame_for(Buttons.RESUME, counter) == template("RES", nxt)       # unchanged by this work
  assert frame_for(Buttons.CANCEL, counter) == template("CAN_OFF", nxt)   # unchanged by this work


def test_set_frames_differ_from_idle_only_in_their_bit_and_its_inverse():
  sig = _dbc_signals()
  idle = int.from_bytes(frame_for(Buttons.NONE, 3), "big")

  def bitmask(name):
    start, _ = sig[name]
    return 1 << ((7 - start // 8) * 8 + start % 8)

  for button, names in ((Buttons.SET_MINUS, ("SET_M", "SET_M_INV")), (Buttons.SET_PLUS, ("SET_P", "SET_P_INV"))):
    diff = idle ^ int.from_bytes(frame_for(button, 3), "big")
    assert diff == bitmask(names[0]) | bitmask(names[1])


def test_the_cancel_and_resume_values_are_what_the_installed_build_sent():
  _, _, cancel = mazdacan.create_button_cmd(CapturePacker(), CP, 7, Buttons.CANCEL)
  _, _, resume = mazdacan.create_button_cmd(CapturePacker(), CP, 7, Buttons.RESUME)
  for v in (cancel, resume):
    assert (v["SET_P"], v["SET_P_INV"], v["SET_M"], v["SET_M_INV"]) == (0, 1, 0, 1)
  assert (cancel["CAN_OFF"], cancel["CAN_OFF_INV"], cancel["RES"], cancel["RES_INV"]) == (1, 0, 0, 1)
  assert (resume["CAN_OFF"], resume["CAN_OFF_INV"], resume["RES"], resume["RES_INV"]) == (0, 1, 1, 0)


# ---- the press cap ---------------------------------------------------------------------------------------------------------------
def test_the_guard_caps_a_tap_at_three_frames_and_0_15_s_and_spaces_taps():
  g = cc.PressGuard()
  assert g.begin(100)
  assert [g.allow(f) for f in (100, 105, 110, 115)] == [True, True, True, False]     # a 4th frame is never sent
  g.end()
  assert not g.begin(120)                       # 0.10 s after the last pressed frame: too soon
  assert g.begin(110 + int(cc.MIN_TAP_GAP_S / cc.DT))
  g.end()
  g2 = cc.PressGuard()
  g2.begin(0)
  assert g2.allow(0) and not g2.allow(16)       # 0.16 s after the first: over the span cap even as the second frame
  assert cc.MAX_PRESSED_FRAMES * cc.FRAME_SPACING * cc.DT < 0.3 and cc.MAX_TAP_SPAN_S < 0.3
  assert max(cc.PRESSED_FRAMES.values()) <= cc.MAX_PRESSED_FRAMES
  assert (max(cc.PRESSED_FRAMES.values()) - 1) * cc.FRAME_SPACING * cc.DT <= cc.MAX_TAP_SPAN_S + 1e-9


# ---- a simulated car -------------------------------------------------------------------------------------------------------------
class Car:
  """The cruise as the study saw it: the button module sends at 10 Hz (counter +1); a press is read 0.04 s after its first frame;
  a press held 0.52 s is a hold (jump to the next multiple of 5). mode: accept (both buttons), ignore, hold (every press is read as a
  hold), minus_only (SET_P ignored), late (read 1.5 s after the press)."""

  def __init__(self, mode="accept", set_mph=45, mph=45.0, phase=3):
    self.mode, self.set, self.mph = mode, set_mph, mph
    self.counter, self.phase = 0, phase
    self.pending = []                 # (frame, new set)
    self.last_press = None            # frame of the last pressed frame
    self.press_start = None
    self.sent = []                    # (frame, button, counter sent)
    self.engaged, self.cruise_on, self.available = True, True, True
    self.brake = self.gas = False
    self.lead = False
    self.driver_button = False
    self.cancel = self.resume = False
    self.others = ""
    self.steer = 0.0

  def frame(self, f):
    if f % 10 == self.phase:
      self.counter = (self.counter + 1) % 16
    for p in [p for p in self.pending if p[0] <= f]:
      self.set = p[1]
      self.pending.remove(p)
    return cc.Inputs(engaged=self.engaged, cruise_on=self.cruise_on, cruise_available=self.available, mph=self.mph, set_mph=self.set,
                     brake=self.brake, gas=self.gas, lead=self.lead, steer_share=self.steer, driver_button=self.driver_button,
                     counter=self.counter, cancel_req=self.cancel, resume_req=self.resume, others_on=self.others, wall=1000.0 + f * cc.DT)

  def press(self, f, button):
    self.sent.append((f, button, (self.counter + 1) % 16))
    new_press = self.last_press is None or f - self.last_press > 15
    self.last_press = f
    if new_press:
      self.press_start = f
    step = -1 if button == "SET_MINUS" else 1
    if self.mode == "ignore" or (self.mode == "minus_only" and step > 0):
      return
    if self.mode == "hold":
      if new_press:
        target = (self.set // 5) * 5 if step < 0 else (self.set // 5 + 1) * 5
        self.pending.append((f + 4, target if target != self.set else self.set + 5 * step))
      return
    if new_press:
      self.pending.append((f + (150 if self.mode == "late" else 4), self.set + step))
    if f - self.press_start >= 52:    # a press this long is a hold (never reached: the cap)
      self.pending.append((f + 1, (self.set // 5 + (1 if step > 0 else 0)) * 5))


def run(car, frames=6000, trigger_at=10, events=None, hook=None, chk=None, trigger=None):
  chk = chk or cc.CruiseCheck()
  statuses = []
  for f in range(frames):
    if hook:
      hook(f, car, chk)
    inp = car.frame(f)
    trig = trigger if trigger is not None and f == trigger_at else (cc.Trigger("run", "r1", inp.wall) if f == trigger_at and trigger is None else None)
    out = chk.step(f, inp, trig)
    if out is not None:
      assert out in cc.BUTTONS
      car.press(f, out)
    if events is not None:
      events.extend(chk.events)
    chk.events.clear()
    statuses.append((chk.state, chk.sub, chk.status(inp.wall)))
    if f > trigger_at + 50 and chk.state in ("finished", "aborted", "refused") and f > trigger_at + 400:
      break
  return chk, statuses


def taps(car):
  """[[frames of one tap]] from what was sent (frames within 0.2 s of each other belong to one tap)."""
  out = []
  for f, b, c in car.sent:
    if out and f - out[-1][-1][0] < 20 and out[-1][-1][1] == b:
      out[-1].append((f, b, c))
    else:
      out.append([(f, b, c)])
  return out


def assert_cap(car):
  ts = taps(car)
  for t in ts:
    assert len(t) <= cc.MAX_PRESSED_FRAMES
    assert (t[-1][0] - t[0][0]) * cc.DT <= cc.MAX_TAP_SPAN_S + 1e-9
  for a, b in zip(ts, ts[1:]):
    assert (b[0][0] - a[-1][0]) * cc.DT >= cc.MIN_TAP_GAP_S - 1e-9
  for f, _, _ in car.sent:                  # never more than 3 pressed frames inside any 0.3 s
    assert sum(1 for g, _, _ in car.sent if f <= g < f + 30) <= cc.MAX_PRESSED_FRAMES


def test_accepting_cruise_gives_yes_3_of_3_with_two_frame_taps():
  car = Car("accept")
  ev = []
  chk, _ = run(car, events=ev)
  assert chk.state == "finished" and chk.result() == "YES"
  assert chk.n_ok("A") == 3 and chk.results["B"] == []
  ts = taps(car)
  assert [t[0][1] for t in ts] == ["SET_MINUS", "SET_PLUS"] * 3
  assert all(len(t) == 2 and t[1][0] - t[0][0] == cc.FRAME_SPACING for t in ts)
  assert all(r["minus_ms"] == 40 and r["plus_ms"] == 40 for r in chk.results["A"])
  assert car.set == 45
  assert_cap(car)
  tap_ev = [e for e in ev if e["kind"] == "tap"]
  assert len(tap_ev) == 6 and all(len(e["frames"]) == 2 for e in tap_ev)
  for e in tap_ev:                              # each frame logged with the car's counter and the counter sent (car's + 1)
    assert all(fr[2] == (fr[1] + 1) % 16 for fr in e["frames"])
    assert e["set_before"] is not None
  assert [e for e in ev if e["kind"] == "result"][0]["result"] == "YES"


def test_taps_start_just_after_a_car_button_frame():
  car = Car("accept", phase=7)
  run(car)
  for t in taps(car):
    assert t[0][0] % 10 == 7                   # the frame the counter changed on: in the interval after the car's frame


def test_ignoring_cruise_gives_no_after_both_recipes():
  car = Car("ignore")
  ev = []
  chk, _ = run(car, events=ev)
  assert chk.state == "finished" and chk.result() == "NO"
  assert chk.n_ok("A") == 0 and chk.n_ok("B") == 0 and len(chk.results["A"]) == 3 and len(chk.results["B"]) == 3
  ts = taps(car)
  assert [len(t) for t in ts] == [2, 2, 2, 3, 3, 3] and all(t[0][1] == "SET_MINUS" for t in ts)   # recipe B: 3 frames
  assert any(e["kind"] == "fallback" for e in ev)
  assert car.set == 45
  assert_cap(car)


def test_a_cruise_that_reads_a_hold_aborts_and_restores_at_most_three_taps():
  car = Car("hold", set_mph=43, mph=43.0)
  ev = []
  chk, _ = run(car, events=ev)
  assert chk.state == "aborted" and "set speed changed to 40" in chk.why
  rest = [t for t in taps(car)][1:]
  # the corrective tap is read as a hold too (40 -> 45): not the single step asked for, so the restore stops at once
  assert 1 <= len(rest) <= cc.RESTORE_TAPS and all(t[0][1] == "SET_PLUS" and len(t) <= 2 for t in rest)
  assert chk.restore["why"] == "not restored: set speed changed to 45 mph" and car.set == 45
  assert_cap(car)


def test_partly_when_some_rounds_fail():
  car = Car("accept")
  count = {"n": 0}
  orig_press = car.press

  def flaky(f, button):
    car.sent.append((f, button, 0))
    if button == "SET_MINUS":
      count["n"] += 1
    if button == "SET_MINUS" and count["n"] in (3, 4):        # the second round's minus tap (2 frames) is ignored
      return
    car.sent.pop()
    orig_press(f, button)
  car.press = flaky
  chk, _ = run(car)
  assert chk.state == "finished" and chk.result() == "PARTLY" and chk.n_ok("A") == 2 and chk.results["B"] == []


ABORTS = [
  ("brake", lambda c: setattr(c, "brake", True), "brake pressed"),
  ("gas", lambda c: setattr(c, "gas", True), "gas pressed"),
  ("disengaged", lambda c: setattr(c, "engaged", False), "openpilot not engaged"),
  ("cruise off", lambda c: setattr(c, "cruise_on", False), "stock cruise not active"),
  ("fault", lambda c: setattr(c, "available", False), "cruise unavailable or faulted"),
  ("cancel", lambda c: setattr(c, "cancel", True), "openpilot is using the cruise buttons"),
  ("resume", lambda c: setattr(c, "resume", True), "openpilot is using the cruise buttons"),
  ("driver button", lambda c: setattr(c, "driver_button", True), "a cruise button is pressed"),
  ("lead", lambda c: setattr(c, "lead", True), "cruise is following a car"),
  ("speed band", lambda c: setattr(c, "mph", 29.0), "speed outside 30-70 mph"),
  ("speed drift", lambda c: setattr(c, "mph", 49.5), "speed moved away from the set speed"),
  ("steering", lambda c: setattr(c, "steer", 0.95), "steering near its limit"),
  ("plan on", lambda c: setattr(c, "others", "the drive plan is on"), "the drive plan is on"),
  ("ab on", lambda c: setattr(c, "others", "the A/B switch is on"), "the A/B switch is on"),
  ("unasked change", lambda c: setattr(c, "set", 47), "set speed changed to 47"),
]


@pytest.mark.parametrize("name,act,why", ABORTS, ids=[a[0] for a in ABORTS])
@pytest.mark.parametrize("when", ["wait_minus", "pause"])
def test_every_abort_path_stops_at_once_and_says_why(name, act, why, when):
  car = Car("late" if when == "wait_minus" else "accept")
  state = {"done": False, "at": None}

  def hook(f, c, chk):
    if not state["done"] and chk.state == "running" and chk.sub == when and (when != "pause" or chk.round == 2):
      act(c)
      state["done"], state["at"], state["set"] = True, f, c.set
  chk, _ = run(car, hook=hook)
  assert state["done"]
  assert chk.state == "aborted" and chk.why.startswith(why), chk.why
  # nothing more of the rounds after the abort: any later frame is a restore tap toward the original only
  later = [s for s in car.sent if s[0] > state["at"]]
  if name in ("brake", "gas", "disengaged", "cruise off", "fault", "cancel", "resume", "driver button"):
    assert later == []
  for _, b, _ in later:                         # toward the original set speed (45) only
    assert b == ("SET_MINUS" if state["set"] > 45 else "SET_PLUS")
  assert_cap(car)


def test_a_late_acceptance_after_the_wait_is_an_unasked_change_and_is_restored():
  car = Car("late")
  chk, _ = run(car)
  assert chk.state == "aborted" and "set speed changed to 44" in chk.why
  assert chk.results["A"][0]["minus"] is False
  assert car.sent[-1][1] == "SET_PLUS"          # a corrective tap toward 45


def test_set_plus_not_accepted_restores_with_at_most_three_taps_then_stops():
  car = Car("minus_only")
  chk, _ = run(car)
  assert chk.state == "aborted" and chk.why.startswith("set-plus not accepted")
  ts = taps(car)
  assert [t[0][1] for t in ts] == ["SET_MINUS", "SET_PLUS"] + ["SET_PLUS"] * cc.RESTORE_TAPS
  assert chk.restore["why"] == f"not restored after {cc.RESTORE_TAPS} taps" and car.set == 44
  assert_cap(car)


def test_restore_succeeds_when_the_car_accepts():
  car = Car("accept")

  def hook(f, c, chk):
    if chk.state == "running" and chk.sub in ("tap_plus", "wait_plus") and c.set == 44 and c.mode == "accept":
      c.mode = "minus_only"                    # the plus of the first round is ignored ...
    if chk.state == "restoring":
      c.mode = "accept"                        # ... and the corrective one is accepted
  chk, _ = run(car, hook=hook)
  assert chk.state == "aborted" and chk.restore["why"] == "restored" and car.set == 45


def test_no_restore_against_the_driver():
  car = Car("accept")

  def hook(f, c, chk):
    if chk.state == "running" and chk.sub == "tap_plus" and c.set == 44:
      c.driver_button = True              # before the plus tap's first frame
  chk, _ = run(car, hook=hook)
  assert chk.state == "aborted" and chk.why == "a cruise button is pressed"
  assert chk.restore["why"].startswith("not restored: the driver")
  assert [b for _, b, _ in car.sent].count("SET_PLUS") <= 1


def test_time_limit():
  car = Car("accept")

  def hook(f, c, chk):
    if chk.state == "running" and chk.round == 2 and chk.sub == "pre":
      c.mph = 47.5                             # not steady: the round waits, inside the recipe's 20 s
  chk, _ = run(car, hook=hook)
  assert chk.state == "aborted" and chk.why.startswith("time limit")


def test_waiting_with_the_reason_then_given_up_after_a_minute():
  car = Car("accept")
  car.lead = True
  chk, st = run(car, frames=int(cc.START_WAIT_S / cc.DT) + 500)
  assert any(s[0] == "waiting" and s[2]["why"] == "cruise is following a car" for s in st)
  assert chk.state == "aborted" and chk.why == "not started: cruise is following a car" and car.sent == []


def test_waits_for_three_steady_seconds_before_the_first_tap():
  car = Car("accept")
  chk, _ = run(car, trigger_at=0)
  assert car.sent[0][0] >= int(cc.STEADY_S / cc.DT)


@pytest.mark.parametrize("why", ["the drive plan is on", "the A/B switch is on", "a test is on", "the dither test is on"])
def test_refused_while_another_mode_is_on_and_sends_nothing(why):
  car = Car("accept")
  car.others = why
  chk, st = run(car, frames=2000)
  assert chk.state == "refused" and chk.why == why and car.sent == []
  assert st[-1][2]["state"] == "refused"


def test_one_press_arms_one_run_and_never_repeats():
  car = Car("accept")
  chk, _ = run(car)
  n = len(car.sent)
  for f in range(6000, 9000):                  # long after: the same trigger again, then nothing
    out = chk.step(f, car.frame(f), cc.Trigger("run", "r1", 1000.0 + f * cc.DT) if f == 6000 else None)
    assert out is None
  assert len(car.sent) == n and chk.state == "finished"
  # a new press is a new run
  chk2, _ = run(Car("accept"), chk=chk, trigger=cc.Trigger("run", "r2", 1000.1))
  assert chk2.state == "finished" and chk2.run_id == "r2"


def test_a_second_press_while_running_does_nothing_and_stop_aborts():
  car = Car("accept")

  def hook(f, c, chk):
    if f == 600:
      chk.step(f, c.frame(f), cc.Trigger("run", "again", 1000.0 + f * cc.DT))
    if f == 700:
      chk.step(f, c.frame(f), cc.Trigger("stop", "s1", 1000.0 + f * cc.DT))
  chk, _ = run(car, hook=hook)
  assert chk.run_id == "r1" and chk.state == "aborted" and chk.why == "stopped on the screen"


def test_a_stale_trigger_is_ignored():
  car = Car("accept")
  chk, _ = run(car, frames=1000, trigger=cc.Trigger("run", "old", 1000.0 - cc.TRIGGER_MAX_AGE_S - 5))
  assert chk.state == "idle" and car.sent == []


def test_set_speed_floor_precondition():
  car = Car("accept", set_mph=24, mph=24.0)
  car.mph = 31.0
  chk, st = run(car, frames=1500)
  assert car.sent == [] and any(s[2]["why"] == "set speed below 25 mph" for s in st)


def test_never_cancel_or_resume():
  src = (ROOT / "selfdrive/car/mazda/ti_cruise_check.py").read_text(encoding="utf-8")
  assert cc.BUTTONS == ("SET_MINUS", "SET_PLUS") and "Buttons." not in src   # the module can only name the two set buttons


# ---- the car controller's glue -----------------------------------------------------------------------------------------------------
def _cc_source():
  return (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")


def _method(name):
  src = _cc_source()
  tree = ast.parse(src)
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CarController")
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
  return ast.get_source_segment(src, fn)


class Mem:
  def __init__(self):
    self.d = {}

  def get(self, k, encoding=None):
    return self.d.get(k)

  def put(self, k, v):
    self.d[k] = v

  def remove(self, k):
    self.d.pop(k, None)


def _glue_host(started):
  import textwrap
  import time as _t
  from openpilot.common.conversions import Conversions as CV
  sent = []
  env = {"cruise_check_mod": cc, "json": json, "CV": CV, "Buttons": Buttons, "ti_cruise_log": lambda e: sent.append(("log", e)),
         "mazdacan": NS(create_button_cmd=lambda packer, CP_, counter, button: ("btn", counter, button)), "time": _t}
  exec(textwrap.dedent(_method("cruise_check_step")), env)
  exec(textwrap.dedent(_method("cruise_others_on")), env)
  host = NS(cruise_check=cc.CruiseCheck(), cruise_started_wall=started, cruise_published=None, frame=0, params_memory=Mem(),
            ccp=NS(TI_STEER_MAX=600), packer=None, CP=None, ti_plan_on=False, ti_ab_switch=False, ti_ident_all=False, ti_ident_on=(),
            ti_dither_enabled=False)
  host.step = lambda CC, CS, s: env["cruise_check_step"](host, CC, CS, s)
  host.others = lambda: env["cruise_others_on"](host)
  host.cruise_others_on = host.others
  return host, sent


def _cs(mph=45, set_mph=45, counter=1, button=False):
  return NS(out=NS(vEgo=mph * 0.44704, brakePressed=False, gasPressed=False,
                   cruiseState=NS(enabled=True, available=True, speed=set_mph * 0.44704)), crz_btns_counter=counter,
            crz_btns_driver=button, crz_lead=False)


def test_the_controller_clears_trigger_and_status_when_it_starts():
  init = _method("__init__")
  assert "cruise_check_mod.PARAM_TRIGGER, cruise_check_mod.PARAM_STATUS" in init and "self.params_memory.remove(key)" in init
  assert "self.cruise_started_wall = time.time()" in init


def test_a_trigger_written_before_the_controller_started_is_never_acted_on():
  import time as _t
  host, _ = _glue_host(started=_t.time())
  host.params_memory.put(cc.PARAM_TRIGGER, json.dumps({"cmd": "run", "id": "x", "t": _t.time() - 5}))
  CC = NS(enabled=True, cruiseControl=NS(cancel=False, resume=False))
  host.step(CC, _cs(), 0)
  assert cc.PARAM_TRIGGER not in host.params_memory.d                # consumed
  assert host.cruise_check.state == "idle"
  assert json.loads(host.params_memory.d[cc.PARAM_STATUS])["state"] == "idle"


def test_the_glue_runs_a_check_and_sends_only_set_buttons():
  import time as _t
  host, log = _glue_host(started=_t.time() - 1)
  host.params_memory.put(cc.PARAM_TRIGGER, json.dumps({"cmd": "run", "id": "x", "t": _t.time()}))
  CC = NS(enabled=True, cruiseControl=NS(cancel=False, resume=False))
  out = []
  set_mph = 45
  for f in range(800):
    host.frame = f
    m = host.step(CC, _cs(set_mph=set_mph, counter=(f // 10) % 16), 0)
    if m is not None:
      out.append((f, m))
  assert out and all(m[0] == "btn" and m[2] == Buttons.SET_MINUS for _, m in out)   # the car here never changes: no acceptance
  assert any(e[1]["kind"] == "tap" for e in log)
  st = json.loads(host.params_memory.d[cc.PARAM_STATUS])
  assert st["state"] == "running" and st["id"] == "x"


def test_the_glue_sends_nothing_while_openpilot_cancels_or_the_plan_is_on():
  import time as _t
  host, _ = _glue_host(started=_t.time() - 1)
  host.ti_plan_on = True
  host.params_memory.put(cc.PARAM_TRIGGER, json.dumps({"cmd": "run", "id": "x", "t": _t.time()}))
  CC = NS(enabled=True, cruiseControl=NS(cancel=False, resume=False))
  for f in range(500):
    host.frame = f
    assert host.step(CC, _cs(counter=(f // 10) % 16), 0) is None
  assert host.cruise_check.state == "refused" and host.cruise_check.why == "the drive plan is on"


def test_the_glue_is_called_after_cancel_and_resume_and_never_raises():
  src = _cc_source()
  i_cancel = src.index("Buttons.CANCEL))")
  i_resume = src.index("Buttons.RESUME))")
  i_check = src.index("cruise_msg = self.cruise_check_step(")
  assert i_cancel < i_resume < i_check
  host, _ = _glue_host(started=0)
  bad = NS(out=None)
  CC = NS(enabled=True, cruiseControl=NS(cancel=False, resume=False))
  assert host.step(CC, bad, 0) is None and host.cruise_check is None    # an error: off for the process, said on the screen
  assert json.loads(host.params_memory.d[cc.PARAM_STATUS])["state"] == "aborted"


def test_params_are_declared():
  params_cc = (ROOT / "common/params.cc").read_text(encoding="utf-8")
  fv = (ROOT / "frogpilot/common/frogpilot_variables.py").read_text(encoding="utf-8")
  for k in (cc.PARAM_TRIGGER, cc.PARAM_STATUS):
    assert f'{{"{k}", CLEAR_ON_MANAGER_START}}' in params_cc
    assert f'"{k}"' in fv.split("EXCLUDED_KEYS")[1].split("}")[0]
    assert f'("{k}", "", 3, "")' in fv


# ---- the screen tells the truth ----------------------------------------------------------------------------------------------------
def _random_scenario(seed):
  rnd = random.Random(seed)
  car = Car(rnd.choice(["accept", "ignore", "hold", "minus_only", "late"]), set_mph=rnd.choice([30, 43, 45, 62]))
  car.mph = float(car.set)
  acts = [a[1] for a in ABORTS]
  at = rnd.randrange(50, 2500)
  act = rnd.choice(acts + [None, None])

  def hook(f, c, chk):
    if act is not None and f == at:
      act(c)
  return car, hook


def test_property_status_matches_the_controller_in_every_state():
  seen = set()
  for seed in range(12):
    car, hook = _random_scenario(seed)
    chk, st = run(car, hook=hook, frames=4000)
    for state, sub, s in st:
      assert s["state"] == state
      seen.add(state)
      if state == "running":
        assert sub in ("pre", "tap_minus", "wait_minus", "tap_plus", "wait_plus", "pause")
      if state in ("finished",):
        assert s["result"] in ("YES", "NO", "PARTLY")
      else:
        assert s["result"] == ""
      if state == "aborted":
        assert s["why"]
  assert {"waiting", "running", "finished", "aborted"} <= seen


def _compiler():
  for c in ("g++", "clang++", "c++"):
    p = shutil.which(c)
    if p:
      return [p, "-std=c++17", "-Wall", "-Wshadow", "-I", str(ROOT)], None
  for vc in glob.glob(r"C:\Program Files*\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvars64.bat"):
    return None, vc
  return None, None


def test_property_the_page_logic_shows_the_controllers_state():
  """Every status the controller publishes in a set of scenarios is given to the page's own logic (ti_pages_logic.h cruiseLook, via
  selfdrive/ui/tests/cruise_page_cli.cc): the screen's state word must be the controller's state, and the result line must say what
  the controller found."""
  gcc, vcvars = _compiler()
  if gcc is None and vcvars is None:
    pytest.skip("no C++ compiler here (run on a build machine)")
  lines, want = [], []
  for seed in range(6):
    car, hook = _random_scenario(seed)
    _, st = run(car, hook=hook, frames=4000)
    for state, _, s in st[::7]:
      lines.append(cc_line(s, fresh=True, others=""))
      want.append(state)
  # stale and missing reports, and another mode on
  lines.append(cc_line({"state": "running"}, fresh=False, others=""))
  want.append("noreport")
  lines.append(cc_line(None, fresh=False, others=""))
  want.append("noreport")
  lines.append(cc_line({"state": "idle"}, fresh=True, others="the drive plan is on"))
  want.append("unavailable")
  src = str(ROOT / "selfdrive/ui/tests/cruise_page_cli.cc")
  with tempfile.TemporaryDirectory() as d:
    exe = os.path.join(d, "c.exe" if os.name == "nt" else "c")
    inp = os.path.join(d, "in.txt")
    Path(inp).write_text("\n".join(lines) + "\n", encoding="utf-8")
    if gcc is not None:
      subprocess.run(gcc + [src, "-o", exe], check=True, capture_output=True, text=True)
      run_ = subprocess.run([exe, inp], capture_output=True, text=True)
    else:
      cmd = (f'"{vcvars}" >nul && cl /nologo /std:c++17 /EHsc /W3 /I"{ROOT}" "{src}" /Fe:"{exe}" /Fo:"{os.path.join(d, "c.obj")}" >nul && "{exe}" "{inp}"')
      run_ = subprocess.run('cmd /s /c "' + cmd + '"', shell=True, capture_output=True, text=True)
    got = run_.stdout.split("\n")
    rows = [g.split("\t") for g in got if g.strip()]
    assert len(rows) == len(want), run_.stdout[-2000:] + run_.stderr[-2000:]
    for (word, result, enabled), w, line in zip(rows, want, lines):
      assert word == w, (line, word, w)
      if w == "finished":
        assert result.startswith("SET SPEED CAN BE CHANGED: ")
      if w in ("running", "waiting", "restoring", "unavailable", "noreport"):
        assert enabled == "0"                   # RUN CHECK only when the car reports it is free


def cc_line(s, fresh, others):
  """One status as the page reads it (the fields sidebar.cc takes from TiCruiseCheckStatus), tab separated for the CLI."""
  if s is None:
    return "\t".join(["0", "0", "", "", "A", "0", "", "", "0", "0", others or "-", ""]).replace("\t\t", "\t-\t")
  rows_a = ";".join(",".join(str(x) for x in r) for r in s.get("a", []))
  rows_b = ";".join(",".join(str(x) for x in r) for r in s.get("b", []))
  fields = ["1", "1" if fresh else "0", s.get("state", ""), s.get("why", "") or "-", s.get("phase", "A"), str(s.get("round", 0)),
            rows_a or "-", rows_b or "-", str(s.get("orig") or 0), str(s.get("set") or 0), others or "-", s.get("result", "") or "-"]
  return "\t".join(f if f != "" else "-" for f in fields)
