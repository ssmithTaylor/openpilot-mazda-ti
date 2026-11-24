"""On-road alert text must fit the alert box on the comma's screen, at the longest numbers it can show (runs on the PC).

selfdrive/controls/tests/test_alerts.py::test_alert_text_length checks the same on the comma, but it cannot run on a PC and
its callback alerts are built from a default SubMaster (steerAdvisorySpeed 0 -> "Slow to 5 mph"), so the curve warning's
longest values were never measured. This file builds them with the real callback at 3-digit speeds, mph and km/h.

Budget: the alert box is the onroad width (2160 px, minus the 300 px sidebar when it is open) less its 40 px margin each side.
Text is measured at the NOMINAL font of selfdrive/ui/qt/onroad/alerts.cc (small 74 semibold; mid 88 bold / 66), not at the
smaller sizes the renderer falls back to for long text with the sidebar open: those fallbacks depend on character counts and
leave no margin for the rounded corners and glyph overhang, so a text that only fits when shrunk is treated as too long.
"""
import os
from types import SimpleNamespace as NS

import pytest
from PIL import Image, ImageDraw, ImageFont

from cereal import log
from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.controls.lib.events import Alert, EVENTS
from openpilot.selfdrive.controls.lib import events as ev

AlertSize = log.ControlsState.AlertSize
FONTS = os.path.join(BASEDIR, "selfdrive/assets/fonts")
BUDGET = 2160 - 300 - 2 * 40
_draw = ImageDraw.Draw(Image.new("RGB", (0, 0)))
_font = {
  (AlertSize.small, 0): ImageFont.truetype(os.path.join(FONTS, "Inter-SemiBold.ttf"), 74),
  (AlertSize.mid, 0): ImageFont.truetype(os.path.join(FONTS, "Inter-Bold.ttf"), 88),
  (AlertSize.mid, 1): ImageFont.truetype(os.path.join(FONTS, "Inter-SemiBold.ttf"), 66),   # the renderer draws it lighter: this is wider
}


def width(alert, i):
  txt = (alert.alert_text_1, alert.alert_text_2)[i]
  left, _, right, _ = _draw.textbbox((0, 0), txt, _font[(alert.alert_size, i)])
  return right - left


def check(alert, where):
  if alert.alert_size not in (AlertSize.small, AlertSize.mid):
    return   # full-size alerts wrap
  for i in range(1 if alert.alert_size == AlertSize.small else 2):
    w = width(alert, i)
    assert w <= BUDGET, f"{where}: {(alert.alert_text_1, alert.alert_text_2)[i]!r} is {w} px, budget {BUDGET}"


def static_alerts():
  for name, by_type in EVENTS.items():
    for et, a in by_type.items():
      if isinstance(a, Alert):
        yield pytest.param(a, id=f"{name}-{et}")


@pytest.mark.parametrize("alert", static_alerts())
def test_static_alert_fits(alert):
  check(alert, alert.alert_type)


def call(fn, metric, **kw):
  sm = {"frogpilotPlan": NS(steerAdvisorySpeed=kw.get("advisory", 0.0))}
  CP = NS(minSteerSpeed=kw.get("min_steer", 0.0), minEnableSpeed=kw.get("min_enable", 0.0))
  return fn(CP, NS(), sm, metric, 100, NS())


# longest values: 3-digit mph (a 160 mph corner never happens, but the callback must not depend on that) and km/h
SPEEDS_MS = [0.1, 2.0, 14.9, 15.0, 26.8, 31.3, 44.7, 55.6, 69.6, 99.9]


@pytest.mark.parametrize("metric", [False, True])
@pytest.mark.parametrize("sign", [1.0, -1.0], ids=["model_advisory", "curve_warning"])
@pytest.mark.parametrize("v", SPEEDS_MS)
def test_corner_speed_alert_fits(v, sign, metric):
  check(call(ev.steer_authority_advisory_alert, metric, advisory=sign * v), f"corner alert {sign * v} m/s metric={metric}")


@pytest.mark.parametrize("metric", [False, True])
@pytest.mark.parametrize("v", SPEEDS_MS)
def test_speed_floor_alerts_fit(v, metric):
  check(call(ev.below_steer_speed_alert, metric, min_steer=v), f"below steer speed {v}")
  check(call(ev.below_engage_speed_alert, metric, min_enable=v), f"below engage speed {v}")


def test_the_longest_curve_text_is_actually_measured():
  """Guard against the test passing because it measured nothing: the 3-digit km/h curve warning is the widest string."""
  a = call(ev.steer_authority_advisory_alert, True, advisory=-69.6)   # 250 km/h
  assert "250" in a.alert_text_1 and width(a, 0) > 1000


# ---- review I1 / I4 (2026-11-24): curve slowing's fixed texts, carried as codes ----------------------------------------------------
def test_curve_code_constants_match_the_feature():
  from openpilot.selfdrive.car.mazda import ti_curve as C
  assert ev.CURVE_CODE_UNIT == C.ALERT_CODE_UNIT
  assert (C.CODE_ASSIST_OFF, C.CODE_ASSIST_STUCK, C.CODE_DROPOUT, C.CODE_SET_LEFT) == (1, 2, 3, 4)


@pytest.mark.parametrize("code", [1, 2, 3, 4])
@pytest.mark.parametrize("a,b", [(0, 0), (25, 40), (999, 999)])
def test_curve_code_alerts_fit_and_never_say_corner_needs(code, a, b):
  from openpilot.selfdrive.car.mazda import ti_curve as C
  al = call(ev.steer_authority_advisory_alert, False, advisory=C.alert_code_value(code, a, b))
  check(al, f"curve code {code} {a} {b}")
  assert "corner needs" not in al.alert_text_1
  if code == 4:
    assert al.alert_text_1 == f"CRUISE SET TO {a}, WAS {b}"
  else:
    assert al.alert_text_1 == "STEERING ASSIST OFF: slow down"
