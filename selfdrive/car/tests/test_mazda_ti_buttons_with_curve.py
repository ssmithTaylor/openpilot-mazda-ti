"""The cruise button tests (ti_button_tests.py) and curve slowing (ti_curve.py) in ONE car controller (the merge of 2026-11-24 of the
button-tests build into curve-stages): at most one cruise-button frame source per control frame; the button tests refuse while curve
slowing is on and curve slowing (and its safe exit) stands aside while a button test or the cruise check runs; the safe exit waits for
a button test and carries on after it; a button test whose restore failed writes the generic TiSetSpeedLeft record (who "button test")
that the on-road alert shows, until the driver presses a cruise button. Runs on the PC."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.mazda import ti_button_tests as bt
from openpilot.selfdrive.car.mazda import ti_curve as C

ROOT = Path(__file__).resolve().parents[3]
TOGGLES = NS(ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False, ti_dither=False)


def _load(name, rel):
  spec = importlib.util.spec_from_file_location(name, ROOT / rel)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


@pytest.fixture(scope="module")
def cost():
  return _load("ti_cost_check_for_merge_tests", "selfdrive/car/mazda/ti_cost_check.py")


@pytest.fixture(scope="module")
def btt():
  return _load("ti_button_tests_tests_for_merge", "selfdrive/car/tests/test_mazda_ti_button_tests.py")


def _rig(cost, monkeypatch, mph=45.0):
  """A real controller (gen1 + interceptor, in-memory params), its inputs, a simulated clock and a packer that records CRZ_BTNS."""
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  clock = {"k": 0}
  monkeypatch.setattr(ccm, "time", NS(monotonic=lambda: clock["k"] * 0.01, time=lambda: 1_800_000_000.0 + clock["k"] * 0.01,
                                      perf_counter=__import__("time").perf_counter))
  monkeypatch.setattr(ccm, "ti_curve_log", lambda ev: None)
  monkeypatch.setattr(ccm, "ti_button_log", lambda ev: None)
  cc, mem = cost.make_controller()
  packed = []

  class Packer:
    def make_can_msg(self, name, bus, values):
      if name == "CRZ_BTNS":
        packed.append((clock["k"], dict(values)))
      return (0, 0, b"\0" * 8, bus)
  cc.packer = Packer()
  CC, CS = cost.inputs(mph, 0.0003)
  CS.out.cruiseState.speed = mph * 0.44704
  return cc, mem, CC, CS, clock, packed


def _frames(cc, CC, CS, clock, packed, k0, n, each=None):
  out = []
  for k in range(k0, k0 + n):
    clock["k"] = k
    if k % 10 == 3:
      CS.crz_btns_counter = (CS.crz_btns_counter + 1) % 16
    if each:
      each(k)
    n0 = len(packed)
    _, sends = cc.update(CC, CS, k * 10_000_000, TOGGLES)
    assert len(packed) - n0 <= 1                       # at most one cruise-button frame per control frame, whoever sends it
    out.append((k, sends, packed[n0:]))
  return out


# ---- mutual exclusion ------------------------------------------------------------------------------------------------------------
def test_the_button_tests_refuse_while_curve_slowing_is_on(cost, monkeypatch):
  cc, mem, CC, CS, clock, packed = _rig(cost, monkeypatch)
  mem.put(C.PARAM_ON, b"1")
  _frames(cc, CC, CS, clock, packed, 0, 20)
  assert cc.curve_on
  mem.put(bt.PARAM_TRIGGER, json.dumps({"cmd": "hold", "id": "m1", "t": 1_800_000_000.0 + 0.2}))
  _frames(cc, CC, CS, clock, packed, 20, 600)
  st = json.loads(mem.d[bt.PARAM_STATUS])
  assert cc.button_tests.state == "refused" and "curve slowing is on" in json.dumps(st).lower(), st
  assert not packed                                   # nothing pressed by either


def test_curve_slowing_stands_aside_while_a_button_test_or_the_cruise_check_runs(cost, monkeypatch):
  cc, mem, CC, CS, clock, packed = _rig(cost, monkeypatch)
  assert not cc._curve_inputs(CC, CS, 0.0, None, None).check_active
  for st in bt.ACTIVE_STATES:
    cc.button_tests.state = st
    assert cc._curve_inputs(CC, CS, 0.0, None, None).check_active, st
  for st in ("idle", "refused", "finished", "aborted"):
    cc.button_tests.state = st
    assert not cc._curve_inputs(CC, CS, 0.0, None, None).check_active, st
  cc.button_tests.state = "idle"
  cc.cruise_check.state = "running"
  assert cc._curve_inputs(CC, CS, 0.0, None, None).check_active
  cc.cruise_check.state = "idle"
  cc.button_tests = None                              # the module failed: curve slowing is unaffected
  assert not cc._curve_inputs(CC, CS, 0.0, None, None).check_active


def _arm_exit(cc, ref=50):
  """The safe exit armed as after a controls restart that found the dash pushed below the driver's `ref`."""
  assert cc.curve_exit.arm(0, None, "controls restarted", ref=ref, floor=ref - 1, restart=True)   # line: ref - 1 (above 45)


def test_the_safe_exit_waits_for_a_button_test_and_carries_on_after_it(cost, monkeypatch):
  cc, mem, CC, CS, clock, packed = _rig(cost, monkeypatch)
  _arm_exit(cc)
  real = cc.button_tests
  calls = {"n": 0}

  def quiet_step(CC_, CS_, ti):                       # a button test in progress that happens not to press on these frames
    calls["n"] += 1
    return None
  monkeypatch.setattr(cc, "button_tests_step", quiet_step)
  real.state = "running"
  _frames(cc, CC, CS, clock, packed, 0, 500)
  assert calls["n"] == 500 and not packed and cc.curve_exit.active, (cc.curve_exit.result, cc.curve_exit.events, cc.curve.state)   # waited 5 s, never pressed, still armed
  assert "waiting" in cc.curve_exit.line2
  real.state = "finished"
  _frames(cc, CC, CS, clock, packed, 500, 200)
  assert packed and all(v["SET_P"] == 1 and v["SET_M"] == 0 for _, v in packed)   # the exit raises once the test is done


def test_a_button_frame_wins_and_the_exit_never_presses_on_the_same_frame(cost, monkeypatch):
  cc, mem, CC, CS, clock, packed = _rig(cost, monkeypatch)
  _arm_exit(cc)
  from openpilot.selfdrive.car.mazda import mazdacan
  from openpilot.selfdrive.car.mazda.values import Buttons
  # a misbehaving combination on purpose: the button tests say they are idle (so the exit is NOT told to wait) yet send a frame
  # on every 10 Hz frame; the controller must still send at most one button frame per control frame, the button test's
  cc.button_tests.state = "idle"
  sent_by_bt = []

  def step(CC_, CS_, ti):
    if cc.frame % 10 == 0:
      sent_by_bt.append(cc.frame)
      return mazdacan.create_button_cmd(cc.packer, cc.CP, CS_.crz_btns_counter, Buttons.SET_MINUS)
    return None
  monkeypatch.setattr(cc, "button_tests_step", step)
  rows = _frames(cc, CC, CS, clock, packed, 0, 400)
  assert sent_by_bt
  minus = [k for k, v in packed if v["SET_M"]]
  plus = [k for k, v in packed if v["SET_P"]]
  assert len(minus) == len(sent_by_bt)
  assert plus and not set(plus) & set(minus)          # the exit pressed too, only on other frames


def test_one_source_per_frame_in_the_send_block_source():
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8").replace("\r\n", "\n")
  assert "if button_msg is not None and cruise_msg is None:" in src
  assert "if curve_msg is not None and cruise_msg is None and button_msg is None:" in src
  assert "curve_msg = self.curve_step(CC, CS, ti_apply_steer)" in src   # stage 3's monitor gets the sent (guarded) command
  i_guard = src.index("ti_apply_steer = guard.update(")
  assert i_guard < src.index("cruise_msg = self.cruise_check_step(") < src.index("button_msg = self.button_tests_step(") < \
    src.index("curve_msg = self.curve_step(")


# ---- TiSetSpeedLeft from the button tests ------------------------------------------------------------------------------------------
def test_set_left_is_only_a_failed_restore_left_lower_and_unanswered():
  b = bt.ButtonTests()
  assert bt.set_left(b, 40) is None and bt.set_left(None, 40) is None
  b.state, b.orig, b.restore = "finished", 45, {"ok": False, "left_at": 40}
  assert bt.set_left(b, 40) == (40, 45)
  assert bt.set_left(b, 38) == (38, 45)
  assert bt.set_left(b, 45) is None and bt.set_left(b, 47) is None and bt.set_left(b, None) is None
  b.answered = True
  assert bt.set_left(b, 40) is None
  b.answered, b.restore = False, {"ok": True, "left_at": 45}
  assert bt.set_left(b, 40) is None
  assert bt.PARAM_LEFT == C.PARAM_LEFT


def test_a_failed_restore_through_the_controller_raises_the_on_road_alert_until_the_driver_presses(btt):
  from openpilot.selfdrive.controls.lib import events as ev
  seen = {}

  def hook(k, car, cc, CC, CS):
    b = cc.button_tests
    if "brake" not in seen and len(car.sent) >= 5:
      seen["brake"] = k
      CS.out.brakePressed = True                      # held: the restore can never finish
    if b is not None and b.not_back() and "end" not in seen:
      seen["end"] = k
    if "end" in seen:
      raw = cc._test_mem.d.get(bt.PARAM_LEFT)
      seen.setdefault("recs", []).append((k, json.loads(raw) if raw else None))
      if k == seen["end"] + 300:
        car.driver = True
  cc, car, packed, mem = btt._closed_loop("taps", "taps", 6500, hook=hook)
  assert "end" in seen, cc.button_tests.state
  recs = seen["recs"]
  before = [r for k, r in recs if seen["end"] + 20 <= k < seen["end"] + 300]
  assert before and all(r and r["who"] == "button test" and r["now"] == car.set and r["was"] == 45 for r in before)
  ms = sorted({r["m"] for r in before})
  assert len(ms) >= 5 and max(b - a for a, b in zip(ms, ms[1:])) <= 0.5 + 1e-6   # rewritten every 0.5 s (the reader's 1.5 s)
  last = recs[-1][1]
  assert last["now"] is None and last["was"] is None and cc.button_tests.answered   # cleared by the driver's press
  # what the on-road alert reads from such a record: its own code, its own second line
  rec = before[-1]
  val, _ = C.read_curve_code(NS(get=lambda key: json.dumps(rec) if key == C.PARAM_LEFT else None), rec["m"] + 0.1)
  assert val == C.alert_code_value(C.CODE_SET_LEFT_TEST, car.set, 45)
  sm = {"frogpilotPlan": NS(steerAdvisorySpeed=val)}
  al = ev.steer_authority_advisory_alert(NS(minSteerSpeed=0.0, minEnableSpeed=0.0), NS(), sm, False, 100, NS())
  assert al.alert_text_1 == f"CRUISE SET TO {car.set}, WAS 45" and al.alert_text_2 == "Button test stopped: set it yourself"


def test_curve_slowings_own_record_keeps_its_code_and_text():
  rec = {"m": 10.0, "who": "curve slowing", "now": 40, "was": 55}
  val, _ = C.read_curve_code(NS(get=lambda key: json.dumps(rec) if key == C.PARAM_LEFT else None), 10.1)
  assert val == C.alert_code_value(C.CODE_SET_LEFT, 40, 55) and C.CODE_SET_LEFT_TEST == 5


def test_the_button_test_alert_fits():
  from openpilot.selfdrive.controls.lib import events as ev
  at = _load("alert_text_for_merge", "selfdrive/car/tests/test_mazda_ti_alert_text.py")
  for a, b in ((0, 0), (25, 40), (999, 999)):
    al = at.call(ev.steer_authority_advisory_alert, False, advisory=C.alert_code_value(C.CODE_SET_LEFT_TEST, a, b))
    at.check(al, f"button test left {a} {b}")
    assert al.alert_text_1 == f"CRUISE SET TO {a}, WAS {b}"


def test_the_record_writers_never_clear_each_other(cost, monkeypatch):
  cc, mem, CC, CS, clock, packed = _rig(cost, monkeypatch)
  b = cc.button_tests
  b.state, b.orig, b.restore = "finished", 45, {"ok": False, "left_at": 40}
  cc.frame = 100
  cc._button_left(b, 40)
  assert json.loads(mem.d[bt.PARAM_LEFT])["who"] == "button test"
  cc.curve_left = (38, 50)                            # curve slowing's own record holds: it wins, the button one waits
  cc.frame = 150
  mem.d.pop(bt.PARAM_LEFT)
  cc._button_left(b, 40)
  assert bt.PARAM_LEFT not in mem.d
  cc._curve_left(None, None, None)                    # curve slowing's record ends: it does not clear the button test's
  assert bt.PARAM_LEFT not in mem.d
  cc.frame = 151
  cc._button_left(b, 40)                              # ... which is rewritten at once
  assert json.loads(mem.d[bt.PARAM_LEFT])["now"] == 40
