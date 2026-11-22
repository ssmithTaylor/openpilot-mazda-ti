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
  assert tp.PLAN_ALT_MIN_MPH == 35 and tp.PLAN_ALT_MAX_MPH == 64


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


def test_nothing_is_chosen_on_the_screen_the_plan_file_names_the_experiment_and_a_bad_plan_runs_nothing():
  body = SIDEBAR[SIDEBAR.index("void TestsWindow::setupAct"):SIDEBAR.index("void TestsWindow::mouseReleaseEvent")]
  assert "CTL_SETUP_RESET" in body and "ConfirmationDialog::confirm" in body and "abSave" not in body and "setupStep" not in body
  abopen = SIDEBAR[SIDEBAR.index("void TestsWindow::abOpen() {"):SIDEBAR.index("// B for the shown setting")]
  assert "loadPlan()" in abopen and "plan.setting" in abopen and "TI_AB_CONFIG_KEY" not in abopen        # the stored config is not where the experiment comes from
  assert "ti_drive_plan.json" in SIDEBAR and "ti_pages::planProblem(plan)" in SIDEBAR
  assert "!plan_ok ||" in SIDEBAR                                                                          # START does nothing from an invalid plan
  for control in ("CTL_SETUP_USE", "CTL_SETUP_PREV", "CTL_SETUP_NEXT", "CTL_SETUP_B_MINUS", "CTL_SETUP_BLOCK_PLUS"):
    assert f"add({control}," not in HEADER                                                                # no editing controls are laid out any more


def _load_drive_plan():
  import importlib.util
  spec = importlib.util.spec_from_file_location("ti_drive_plan_for_test", MAZDA / "ti_drive_plan.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def test_the_shipped_plan_is_valid_names_the_agreed_experiment_and_matches_the_code_limits():
  import json as _json
  dp = _load_drive_plan()
  plan, why = dp.load()
  assert why == "" and plan is not None
  alt = plan["alternation"]
  assert alt["setting"] == "LatNoFrictionRelay" and alt["a"] == "stored" and alt["b"] == "other" and alt["block_min"] == 4
  assert (alt["min_mph"], alt["max_mph"], alt["target_min_per_side"]) == (tp.PLAN_ALT_MIN_MPH, tp.PLAN_ALT_MAX_MPH, tp.TARGET_MIN_PER_SIDE)
  assert alt["setting"] in ab.ALLOWED and alt["block_min"] >= 4
  defs = _json.loads((MAZDA / "ti_tests.json").read_text(encoding="utf-8"))
  assert [t["id"] for t in defs["tests"] if t["id"] in dp.STEP_TESTS] == list(plan["tests"]["ids"]) or set(plan["tests"]["ids"]) <= {t["id"] for t in defs["tests"]}


def test_an_invalid_plan_is_refused_with_a_reason_and_the_header_agrees():
  import copy
  dp = _load_drive_plan()
  plan, _ = dp.load()
  mutations = {
    "setting is not allowed": lambda p: p["alternation"].update(setting="TiSteerMax"),
    "A must be your stored value": lambda p: p["alternation"].update(a=0.8),
    "B must be the other value": lambda p: p["alternation"].update(b=1),
    "block length outside 4-10 min": lambda p: p["alternation"].update(block_min=3),
    "target must be 10 min per side": lambda p: p["alternation"].update(target_min_per_side=30),
    "an amount has no reason": lambda p: p["alternation"].update(reason=""),
    "speed range must be 35-64 mph": lambda p: p["alternation"].update(max_mph=56),
    "tests must cover all four windows": lambda p: p["tests"].update(windows=p["tests"]["windows"][:1]),
    "tests must be sweep, pulses, ladder": lambda p: p["tests"].update(ids=["ti_dither"]),
    "no plan name": lambda p: p.update(name=""),
    "optional must be true or false": lambda p: p["tests"]["windows"][1].update(optional="yes"),
  }
  for why, mutate in mutations.items():
    bad = copy.deepcopy(plan)
    mutate(bad)
    assert dp.problem(bad) == why, why
  assert dp.problem("nonsense") == "no plan name" and dp.load("missing.json")[0] is None
  for why in mutations:                                    # every reason the file loader gives is one the screen can word within its line
    assert len(why) <= 40 and why in HEADER
