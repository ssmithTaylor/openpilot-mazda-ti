"""The plan START runs: a data file shipped with the build (ti_drive_plan.json). The screen never picks an experiment and the driver never
picks one: this file does, and the code that runs it (ti_plan.py, ti_ab.py, ti_tests.json) has its own fixed limits. So a plan is
accepted only when it names exactly what those limits run; anything else is refused with a reason ("PLAN INVALID") and nothing runs.
The same rules are in selfdrive/ui/qt/ti_pages_logic.h (planProblem), compared by test_mazda_ti_pages_logic.py."""
import json
from pathlib import Path

PLAN_PATH = Path(__file__).with_name("ti_drive_plan.json")
STEP_TESTS = ("ti_sweep", "ti_pulses", "ti_ladder")
WINDOWS = ("21-27", "35-41", "49-56", "64-80")
BOOL_SETTINGS = ("LatNoFrictionRelay", "LatFrictionComp", "LatDamping", "LatOutputFilter", "LatCommitSetpoint")
NUMBER_SETTINGS = ("SteerKP",)
BLOCK_MIN_RANGE = (4, 10)
TARGET_MIN = 20          # = ti_plan.TARGET_MIN_PER_SIDE
RANGE_MPH = (35, 56)     # = ti_plan.PLAN_ALT_MIN_MPH, PLAN_ALT_MAX_MPH


def problem(plan):
  """"" when the plan is acceptable, else the first reason (short, plain)."""
  try:
    if not isinstance(plan, dict) or not isinstance(plan.get("name"), str) or not plan["name"].strip():
      return "no plan name"
    tests = plan.get("tests")
    if not isinstance(tests, dict) or tuple(tests.get("ids", ())) != STEP_TESTS:
      return "tests must be sweep, pulses, ladder"
    if tuple(tests.get("windows", ())) != WINDOWS:
      return "tests must cover all four windows"
    alt = plan.get("alternation")
    if alt is None:
      return ""
    if not isinstance(alt, dict) or alt.get("setting") not in BOOL_SETTINGS + NUMBER_SETTINGS:
      return "setting is not allowed"
    if alt.get("a") != "stored":
      return "A must be your stored value"
    b = alt.get("b")
    if alt["setting"] in BOOL_SETTINGS:
      if b != "other":
        return "B must be the other value"
    elif isinstance(b, bool) or not isinstance(b, (int, float)) or not 0.70 <= b <= 1.00:
      return "B is outside 0.70-1.00"
    block = alt.get("block_min")
    if isinstance(block, bool) or not isinstance(block, (int, float)) or not BLOCK_MIN_RANGE[0] <= block <= BLOCK_MIN_RANGE[1]:
      return "block length outside 4-10 min"
    if alt.get("target_min_per_side") != TARGET_MIN:
      return "target must be 20 min per side"
    if (alt.get("min_mph"), alt.get("max_mph")) != RANGE_MPH:
      return "speed range must be 35-56 mph"
    return ""
  except Exception:
    return "plan unreadable"


def load(path=PLAN_PATH):
  """(plan or None, reason): the shipped plan, or why it cannot be used."""
  try:
    plan = json.loads(Path(path).read_text(encoding="utf-8"))
  except Exception:
    return None, "plan file missing or unreadable"
  why = problem(plan)
  return (plan, "") if not why else (None, why)
