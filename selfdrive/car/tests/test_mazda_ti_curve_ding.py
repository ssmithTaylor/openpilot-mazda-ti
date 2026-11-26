"""The curve element's one ding (owner's approved design 2026-11-26: "one ding with red"): ti_curve_plannerd writes a red chip's onset
(TiCurveDing); frogpilot_planner raises the FrogPilot event tiCurveDing ONCE per onset while engaged; the event's alert has no banner
(size none), no text, and plays the ordinary prompt (the openpilot ding) once. Never a loop: the same onset never raises it twice, a
missed onset (not engaged) is not raised later, and nothing repeats while the onset stays in the param. Synthetic only."""
import json
import re

from cereal import car, custom, log
from openpilot.selfdrive.controls.lib.events import EVENTS, ET
from openpilot.selfdrive.car.mazda import ti_curve_view as V

ROOT = __import__("pathlib").Path(__file__).resolve().parents[3]
FEN = custom.FrogPilotCarEvent.EventName


class _Params:
  def __init__(self):
    self.d = {}

  def get(self, k):
    v = self.d.get(k)
    return None if v is None else v.encode()


def _run(onsets, engaged=lambda t: True, seconds=10.0):
  """frogpilot_planner's 10 Hz read over `seconds`; onsets: {time: need} written by the planner. -> times a ding was raised."""
  pm = _Params()
  last, out = None, []
  for i in range(int(seconds * 10)):
    t = 100.0 + i * 0.1
    for to, need in onsets.items():
      if abs(t - to) < 1e-6:
        pm.d[V.PARAM_DING] = json.dumps({"m": to, "n": 1, "need": need})
    last, due = V.curve_ding_due(pm, t, last, engaged(t))
    if due:
      out.append(round(t, 1))
  return out


def test_one_onset_one_ding_even_though_the_param_stays():
  assert _run({101.0: 20}) == [101.0]


def test_two_onsets_two_dings():
  assert _run({101.0: 20, 105.0: 24}) == [101.0, 105.0]


def test_not_engaged_at_the_onset_means_no_ding_later_either():
  assert _run({101.0: 20}, engaged=lambda t: t > 101.5) == []


def test_a_stale_onset_is_never_raised():
  pm = _Params()
  pm.d[V.PARAM_DING] = json.dumps({"m": 50.0, "n": 1, "need": 20})
  assert V.curve_ding_due(pm, 100.0, None, True) == (None, False)


def test_the_event_is_a_single_prompt_with_no_banner():
  a = EVENTS[FEN.tiCurveDing][ET.WARNING]
  assert a.alert_size == log.ControlsState.AlertSize.none
  assert a.alert_text_1 == "" and a.alert_text_2 == ""
  assert a.audible_alert == car.CarControl.HUDControl.AudibleAlert.prompt       # prompt: plays once (soundd), never promptRepeat
  assert a.visual_alert == car.CarControl.HUDControl.VisualAlert.none
  assert a.duration <= 1.0
  assert set(EVENTS[FEN.tiCurveDing]) == {ET.WARNING}                           # only while engaged; never a disable or no-entry


def test_frogpilot_raises_it_once_and_clears_the_flag():
  src = (ROOT / "frogpilot/controls/lib/frogpilot_events.py").read_text(encoding="utf-8")
  m = re.search(r"if getattr\(self\.frogpilot_planner, \"curve_ding\", False\):\n\s+self\.frogpilot_planner\.curve_ding = False\n"
                r"\s+self\.events\.add\(FrogPilotEventName\.tiCurveDing\)", src)
  assert m, "frogpilot_events must raise tiCurveDing once and clear the planner's flag"
  plan = (ROOT / "frogpilot/controls/frogpilot_planner.py").read_text(encoding="utf-8")
  assert "curve_ding_due(params_memory" in plan and 'sm["controlsState"].enabled' in plan


def test_the_enum_is_an_addition_at_the_end():
  src = (ROOT / "cereal/custom.capnp").read_text(encoding="utf-8")
  block = src[src.index("enum EventName @0xaedffd8f31e7b55d"):]
  block = block[:block.index("}")]
  nums = [int(n) for n in re.findall(r"@(\d+);", block)]
  assert nums == list(range(len(nums)))
  assert re.search(r"tiCurveDing @(\d+);", block).group(1) == str(max(nums))
