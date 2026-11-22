"""The A/B page's Qt-free logic (selfdrive/ui/qt/ti_ab_logic.h): checked against ti_ab.py, then compiled and run on its own
when a C++ compiler is available (g++, clang++ or MSVC)."""
import glob
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from openpilot.selfdrive.car.mazda import ti_ab as ab

ROOT = Path(__file__).resolve().parents[3]
HEADER = (ROOT / "selfdrive/ui/qt/ti_ab_logic.h").read_text(encoding="utf-8")


def test_the_header_mirrors_the_allowlist_ranges_and_steps():
  rows = re.findall(r'\{"(\w+)", "(\w+)", (true|false), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+)\}', HEADER)
  assert [r[0] for r in rows] == list(ab.ALLOWED)
  for name, toggle, is_bool, lo, hi, max_step, step in rows:
    s = ab.SETTINGS[name]
    assert toggle == s.attr
    assert (is_bool == "true") == (s.kind == "bool")
    assert (float(lo), float(hi), float(max_step), float(step)) == (s.lo, s.hi, s.max_step, s.step)
  assert int(re.search(r"constexpr int N_SETTINGS = (\d+);", HEADER).group(1)) == len(ab.SETTINGS)


def test_the_header_mirrors_the_block_bounds_and_the_wait_texts():
  for name, value in (("MIN_BLOCK_MIN", ab.MIN_BLOCK_MIN), ("MAX_BLOCK_MIN", ab.MAX_BLOCK_MIN),
                      ("DEFAULT_BLOCK_MIN", ab.DEFAULT_BLOCK_MIN)):
    assert float(re.search(rf"constexpr double {name} = ([\d.]+);", HEADER).group(1)) == value
  pairs = re.findall(r'\{"(\w+)", "([^"]+)"\}', HEADER[HEADER.index("REASONS[N_REASONS]"):])
  assert dict(pairs) == {k: ab.MESSAGE[k] for k in ab.PRIORITY}
  assert [p[0] for p in pairs] == list(ab.PRIORITY)


def _compiler():
  for cc in ("g++", "clang++", "c++"):
    path = shutil.which(cc)
    if path:
      return [path, "-std=c++17", "-Wall", "-I", str(ROOT)], None
  for vc in glob.glob(r"C:\Program Files*\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvars64.bat"):
    return None, vc
  return None, None


def test_the_header_compiles_and_its_own_tests_pass():
  gcc, vcvars = _compiler()
  if gcc is None and vcvars is None:
    pytest.skip("no C++ compiler here (run on the device or a build machine)")
  src = str(ROOT / "selfdrive/ui/tests/test_ti_ab_logic.cc")
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


def test_the_header_mirrors_the_drive_plan_constants():
  from openpilot.selfdrive.car.mazda import ti_plan as tp
  for name, value in (("PLAN_TARGET_MIN_DEFAULT", tp.TARGET_MIN_PER_SIDE), ("PLAN_ALT_MIN_MPH", tp.PLAN_ALT_MIN_MPH),
                      ("PLAN_ALT_MAX_MPH", tp.PLAN_ALT_MAX_MPH)):
    assert float(re.search(rf"constexpr double {name} = ([\d.]+);", HEADER).group(1)) == value
  assert "35-64 mph" in HEADER


def test_the_header_words_every_hold_reason_and_none_says_another_test_is_on():
  from openpilot.selfdrive.car.mazda import ti_plan as tp
  block = HEADER[HEADER.index("inline const char *holdText"):]
  pairs = dict(re.findall(r'\{"([^"]+)", "([^"]+)"\}', block[:block.index("return \"\";")]))
  assert set(pairs) == set(tp.HOLD_WHYS)
  assert all(text and "another test" not in text for text in pairs.values())
