"""The driving screen's curve element, UI side (selfdrive/ui/qt/onroad/ti_curve_view_logic.h): its own C++ tests (parsing, visibility,
chip timing, the fit rule, colours) compiled and run when a C++ compiler is here (g++, clang++ or MSVC; on this PC the Docker image
runs them, see the module docstring of the header), and its decisions checked against the planner's (ti_curve_view.py) on payloads the
planner really writes over a synthetic drive. Also: the drawing takes its numbers from the header and the widget reads the param that
plannerd writes."""
import glob
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from openpilot.selfdrive.car.mazda import ti_curve_view as V
from openpilot.selfdrive.car.tests.test_mazda_ti_curve_view import _drive

pytestmark = pytest.mark.xdist_group("curve_view_cpp")   # compiles C++: never in parallel with itself

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "selfdrive/ui/qt/onroad/ti_curve_view_logic.h"


def _compiler():
  for cc in ("g++", "clang++", "c++"):
    path = shutil.which(cc)
    if path:
      return [path, "-std=c++17", "-Wall", "-Wshadow", "-I", str(ROOT)], None
  for vc in glob.glob(r"C:\Program Files*\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvars64.bat"):
    return None, vc
  return None, None


def _build(src, d):
  gcc, vcvars = _compiler()
  if gcc is None and vcvars is None:
    pytest.skip("no C++ compiler here (the Docker image runs these: see test_ti_curve_view_logic.cc)")
  exe = os.path.join(d, "t.exe" if os.name == "nt" else "t")
  if gcc is not None:
    subprocess.run(gcc + [str(src), "-o", exe], check=True, capture_output=True, text=True)
  else:
    cmd = (f'"{vcvars}" >nul && cl /nologo /std:c++17 /EHsc /W3 /I"{ROOT}" "{src}" /Fe:"{exe}" /Fo:"{os.path.join(d, "t.obj")}" >nul')
    r = subprocess.run('cmd /s /c "' + cmd + '"', shell=True, capture_output=True, text=True)
    assert os.path.exists(exe), r.stdout + r.stderr
  return exe


def test_the_header_compiles_and_its_own_tests_pass():
  with tempfile.TemporaryDirectory() as d:
    exe = _build(ROOT / "selfdrive/ui/tests/test_ti_curve_view_logic.cc", d)
    run = subprocess.run([exe], capture_output=True, text=True)
    assert run.stdout.strip().endswith("ok"), run.stdout + run.stderr


def test_the_ui_decides_the_chip_as_the_planner_does_on_real_payloads():
  out, _ = _drive(ticks=260)
  lines, want = [], []
  for _, _, p, _, log in out:
    lines.append(f"{p['m']:.2f} 1 {p['mph'] / 10:.1f} 1 {V.view_json(p)}")
    want.append(log[1])
  with tempfile.TemporaryDirectory() as d:
    exe = _build(ROOT / "selfdrive/ui/tests/curve_view_cli.cc", d)
    run = subprocess.run([exe], input="\n".join(lines) + "\n", capture_output=True, text=True)
  got = [list(map(int, ln.split())) for ln in run.stdout.strip().splitlines()]
  assert len(got) == len(out)
  for (vis, cls, mph, secs, dr, bend, npts), w, (_, _, p, _, _) in zip(got, want, out):
    assert vis == 0                                   # shown: on, engaged, fresh, fast enough
    assert cls == w                                   # the same chip as the planner's rule at age 0 (and so the same ding)
    if cls:
      b = p["b"][bend]
      assert mph == (b[4] if cls == V.CLS_RED else b[5]) and dr == b[3]
    assert npts == len(p["r"])
  assert {g[1] for g in got} >= {0, 2}


def test_drawing_takes_its_numbers_from_the_header_and_reads_the_planners_param():
  cc = (ROOT / "selfdrive/ui/qt/onroad/ti_curve_view.cc").read_text(encoding="utf-8")
  assert '#include "selfdrive/ui/qt/onroad/ti_curve_view_logic.h"' in (ROOT / "selfdrive/ui/qt/onroad/ti_curve_view.h").read_text(
    encoding="utf-8")
  for name in ("visibility(", "chipFor(", "ribbonRaw(", "styleFor(", "resumeRings(", "hatchMarks(", "carAngle(", "chipBox("):
    assert name in cc, name
  assert 'PARAM_KEY = "' + V.PARAM_VIEW + '"' in HEADER.read_text(encoding="utf-8")
  cam = (ROOT / "selfdrive/ui/qt/onroad/annotated_camera.cc").read_text(encoding="utf-8")
  assert "curve_view" in cam
  assert "qt/onroad/ti_curve_view.cc" in (ROOT / "selfdrive/ui/SConscript").read_text(encoding="utf-8")


def test_header_constants_match_the_planner():
  h = HEADER.read_text(encoding="utf-8")
  assert f"constexpr int VERSION = {V.VIEW_VERSION};" in h
  assert "constexpr double SHOW_MPH = 19.0;" in h and V.MIN_MPH == 19.0
  assert "constexpr int MAX_BENDS = 8;" in h and V.MAX_BENDS == 8
  assert "constexpr int MAX_PTS = 128;" in h and V.VIEW_PTS <= 128
