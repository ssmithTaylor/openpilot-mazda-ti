"""The Tests page restyle (owner, 2026-11-26: "same content, touch targets unchanged"): every control rectangle of every on-car page
(ti_pages_logic.h layout(), which sidebar.cc hit-tests and draws with) is exactly what it was before the restyle (the copy in
selfdrive/ui/tests/ti_pages_rects_golden.txt, printed by ti_pages_rects_cli.cc at forks-table-20261126), and the restyled home page
takes every control's rectangle from ctlRect() (no rectangle of its own for a control)."""
import re
import subprocess
import tempfile
from pathlib import Path

import pytest

from openpilot.selfdrive.car.tests.test_mazda_ti_curve_view_logic import _build

pytestmark = pytest.mark.xdist_group("curve_view_cpp")

ROOT = Path(__file__).resolve().parents[3]


def test_every_touch_target_is_where_it_was():
  with tempfile.TemporaryDirectory() as d:
    exe = _build(ROOT / "selfdrive/ui/tests/ti_pages_rects_cli.cc", d)
    got = subprocess.run([exe], capture_output=True, text=True, check=True).stdout.split("\n")
  want = (ROOT / "selfdrive/ui/tests/ti_pages_rects_golden.txt").read_text(encoding="utf-8").split("\n")
  assert [g.strip() for g in got if g.strip()] == [w.strip() for w in want if w.strip()]


def test_the_home_page_draws_its_controls_in_their_own_rectangles():
  src = (ROOT / "selfdrive/ui/qt/sidebar.cc").read_text(encoding="utf-8")
  body = src[src.index("void TestsWindow::drawHome(QPainter &p) {"):]
  body = body[:body.index("\n}\n")]
  for ctl in ("CTL_START", "CTL_STOP", "CTL_BACK", "CTL_NAV_SETUP", "CTL_NAV_PROGRESS"):
    assert f"ctlRect({ctl})" in body, ctl
  assert not re.search(r"QRect\(\s*\d+\s*,\s*\d+\s*,", body), "a literal rectangle on the home page"
