"""The on-car pages' Qt-free logic (selfdrive/ui/qt/ti_pages_logic.h): checked against the car side's names and constants, then
compiled and run on its own when a C++ compiler is available (g++, clang++ or MSVC). The Qt code (sidebar.cc) cannot be compiled
off the device; this file also checks, by reading its source, that it takes its rectangles and its writes from the header."""
import ast
import glob
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
MAZDA = ROOT / "selfdrive/car/mazda"


def _load_plan():
  # ti_plan.py needs no package: loaded by path, so this file does not need the whole car stack
  import importlib.util
  spec = importlib.util.spec_from_file_location("ti_plan_for_pages_test", MAZDA / "ti_plan.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


tp = _load_plan()
AB_SRC = (MAZDA / "ti_ab.py").read_text(encoding="utf-8")


class ab:  # the alternation's names, read from its source
  ALLOWED = tuple(re.findall(r'^  "(\w+)": Spec\("', AB_SRC, re.M))
  PARAM_ON = re.search(r'PARAM_ON, PARAM_PIN, PARAM_CONFIG, PARAM_STATUS = "(\w+)", "(\w+)"', AB_SRC).group(1)
  PARAM_PIN = re.search(r'PARAM_ON, PARAM_PIN, PARAM_CONFIG, PARAM_STATUS = "(\w+)", "(\w+)"', AB_SRC).group(2)
  MESSAGE = ast.literal_eval(re.search(r"^MESSAGE = (\{.*?\})\s*$", AB_SRC, re.M | re.S).group(1))


HEADER = (ROOT / "selfdrive/ui/qt/ti_pages_logic.h").read_text(encoding="utf-8")
SIDEBAR = (ROOT / "selfdrive/ui/qt/sidebar.cc").read_text(encoding="utf-8")


def _compiler():
  for cc in ("g++", "clang++", "c++"):
    path = shutil.which(cc)
    if path:
      return [path, "-std=c++17", "-Wall", "-Wshadow", "-I", str(ROOT)], None
  for vc in glob.glob(r"C:\Program Files*\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvars64.bat"):
    return None, vc
  return None, None


def test_the_header_compiles_and_its_own_tests_pass():
  gcc, vcvars = _compiler()
  if gcc is None and vcvars is None:
    pytest.skip("no C++ compiler here (run on a build machine)")
  src = str(ROOT / "selfdrive/ui/tests/test_ti_pages_logic.cc")
  with tempfile.TemporaryDirectory() as d:
    exe = os.path.join(d, "t.exe" if os.name == "nt" else "t")
    if gcc is not None:
      subprocess.run(gcc + [src, "-o", exe], check=True, capture_output=True, text=True)
      run = subprocess.run([exe], capture_output=True, text=True)
    else:
      cmd = (f'"{vcvars}" >nul && cl /nologo /std:c++17 /EHsc /W3 /I"{ROOT}" "{src}" /Fe:"{exe}" '
             f'/Fo:"{os.path.join(d, "t.obj")}" >nul && "{exe}"')
      run = subprocess.run('cmd /s /c "' + cmd + '"', shell=True, capture_output=True, text=True)
    assert run.stdout.strip().endswith("ok"), run.stdout + run.stderr


def test_the_plain_names_cover_exactly_the_alternation_settings():
  block = HEADER[HEADER.index("inline const char *plainName"):]
  block = block[:block.index("return setting;")]
  names = re.findall(r'\{"(\w+)", "[^"]+"\}', block)
  assert names == list(ab.ALLOWED)


def test_the_short_hold_texts_cover_exactly_the_hold_reasons():
  block = HEADER[HEADER.index("constexpr HoldShort HOLD_SHORT[]"):]
  block = block[:block.index("};")]
  whys = re.findall(r'\{"([^"]+)", "[^"]+", Tone::\w+\}', block)
  assert set(whys) == set(tp.HOLD_WHYS) and len(whys) == len(tp.HOLD_WHYS)
  assert int(re.search(r"constexpr int N_HOLD_SHORT = (\d+);", HEADER).group(1)) == len(tp.HOLD_WHYS)


def test_the_header_mirrors_the_speed_numbers_it_words():
  assert f"{ab.MESSAGE['speed']}" and "36-69" in ab.MESSAGE["speed"]
  assert "28-31" in ab.MESSAGE["eps"]
  assert int(re.search(r"SPEED_GATE_LOW_MPH = (\d+)", HEADER).group(1)) == 36
  assert int(re.search(r"SPEED_GATE_HIGH_MPH = (\d+)", HEADER).group(1)) == 69
  assert int(re.search(r"EPS_LOW_MPH = (\d+)", HEADER).group(1)) == 28
  assert int(re.search(r"EPS_HIGH_MPH = (\d+)", HEADER).group(1)) == 31
  assert tp.PLAN_ALT_MIN_MPH == 35 and tp.PLAN_ALT_MAX_MPH == 56


def test_the_param_keys_of_the_writes_are_the_car_side_keys():
  assert ab.PARAM_ON == re.search(r'KEY_AB_ON = "(\w+)"', HEADER).group(1)
  assert ab.PARAM_PIN == re.search(r'KEY_AB_PIN = "(\w+)"', HEADER).group(1)
  assert tp.PARAM_PLAN_ON == re.search(r'KEY_PLAN_ON = "(\w+)"', HEADER).group(1)


def test_the_qt_page_takes_its_rectangles_and_writes_from_the_header():
  # every control rectangle is the header's; no hand-placed control is left
  assert "ti_pages::rectOf(" in SIDEBAR and "ti_pages::controlAt(" in SIDEBAR
  assert not re.search(r"QRect TestsWindow::(abBox|abRect|runAllRect)", SIDEBAR)
  # START and STOP, RUN ALL and ALL TESTS OFF go through the tested sequences
  assert "tests.runOps(startOps(ab_configured)" in SIDEBAR and "tests.runOps(stopOps())" in SIDEBAR
  assert "runOps(ti_pages::runAllOps())" in SIDEBAR and "runOps(ti_pages::allTestsOffOps())" in SIDEBAR
  # the controller's params are written through the executor only (no direct write of the plan switch is left on a page)
  assert SIDEBAR.count('TI_PLAN_ON_KEY, true') == 0 and SIDEBAR.count('"TiPlanOn", true') == 0
