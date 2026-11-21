"""The Tests screen's Qt-free logic (selfdrive/ui/qt/ti_tests_logic.h): compiled and run on its own when a C++
compiler is available (g++, clang++ or MSVC), and checked against ti_ident.py's windows and target choice."""
import glob
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from openpilot.selfdrive.car.mazda import ti_ident as ti

ROOT = Path(__file__).resolve().parents[3]
HEADER = (ROOT / "selfdrive/ui/qt/ti_tests_logic.h").read_text(encoding="utf-8")
MPH = 0.44704


def test_the_header_mirrors_the_generators_windows_and_limits():
  rows = re.findall(r'\{([\d.]+), ([\d.]+), "([\d-]+)"\}', HEADER)
  assert [(float(a), float(b), c) for a, b, c in rows] == [(w[0], w[1], w[2]) for w in ti.WINDOWS]
  for name, value in (("MIN_TEST_MPH", ti.MIN_TEST_MPH), ("EPS_LOW_MPH", ti.EPS_LOW_MPH),
                      ("EPS_HIGH_MPH", ti.EPS_HIGH_MPH), ("MAX_TEST_MPH", ti.MAX_TEST_MPH)):
    assert float(re.search(rf"constexpr double {name} = ([\d.]+);", HEADER).group(1)) == value
  assert int(re.search(r"constexpr int N_WINDOWS = (\d+);", HEADER).group(1)) == len(ti.WINDOWS)


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
  src = str(ROOT / "selfdrive/ui/tests/test_ti_tests_logic.cc")
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


def test_the_python_target_choice_agrees_with_the_header_on_a_grid():
  """Same answers as chooseTarget for a grid of speeds and progress patterns (the header's rule, restated)."""
  import itertools
  needs = [{"s": s, "c": c} for s, c in ((100.0, 5), (100.0, 5), (100.0, 5), (100.0, 5))]
  for mph in range(19, 81):
    for pattern in itertools.product((0, 1), repeat=4):
      secs = [100.0 + 10 * i for i in range(4)]
      cells = [5 if p else 0 for p in pattern]

      def remaining(key, window, step, secs=secs, cells=cells):
        i = [w[2] for w in ti.WINDOWS].index(window)
        if key != ti.SWEEP or step != 1:
          return 0.0
        return secs[i] if cells[i] else 0.0
      want = None
      best = None
      for i, w in enumerate(ti.WINDOWS):
        if not cells[i]:
          continue
        d = max(w[0] - mph, mph - w[1], 0.0)
        rank = (int(d + 0.5), -secs[i])
        if best is None or rank < best:
          best, want = rank, w[2]
      got = ti.target_window(float(mph), remaining)
      assert (got[0] if got else None) == want, (mph, pattern)
