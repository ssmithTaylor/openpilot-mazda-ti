"""The alternation's band with the drive plan, 35-64 mph since 2026-11-22: a block runs and scores at 57-63 mph (the former gap), starts
there up to 61 mph (MARGIN_MPH inside the ceiling), and saturation at those speeds returns B to A as anywhere else."""
import importlib.util
from pathlib import Path

from openpilot.selfdrive.car.mazda import ti_ab as ab
from openpilot.selfdrive.car.mazda import ti_plan as tp

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("ti_ab_tests_for_band", HERE / "test_mazda_ti_ab.py")
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)


def planned_b():
  r = T.r_pending()
  r.ab.set_plan_limits(tp.PLAN_ALT_MAX_MPH, 600.0)
  T.quiet_until_block_b(r)
  r.secs(6.0)
  assert r.outs[-1].s == 1.0 and r.outs[-1].arm == "B"
  return r


def test_a_block_runs_and_scores_at_57_to_63_mph_and_saturation_there_returns_to_a():
  r = planned_b()
  total = lambda: sum(t for d in r.ab.progress.values() for t in d.values())
  before = total()
  r.secs(20.0, v_ego=T.v_mph(60.0))
  assert r.ab.mode == "running"
  assert total() - before >= 6.0                                    # time at 60 mph is scored (10 s test blocks: wash-ins between)
  r = planned_b()                                                   # a fresh B block, then 60-63 mph inside it
  r.secs(1.0, v_ego=T.v_mph(60.0))
  r.secs(2.0, v_ego=T.v_mph(63.4))
  assert r.ab.block is not None and r.outs[-1].s == 1.0            # still inside 35-64 mph (hysteresis to the ceiling)
  r.secs(0.6, v_ego=T.v_mph(60.0), steer=1.0)                      # the request at its limit for half a second at 60 mph
  sw = r.kinds("switch")[-1]
  assert sw["why"] == "release:saturation" and sw["s_to"] == 0.0
  r.secs(4.0, v_ego=T.v_mph(60.0), steer=1.0)
  assert r.outs[-1].s == 0.0


def test_blocks_start_up_to_61_mph_and_not_closer_to_the_ceiling():
  r = T.r_pending()
  r.ab.set_plan_limits(tp.PLAN_ALT_MAX_MPH, 600.0)
  r.secs(40.0, v_ego=T.v_mph(62.5))
  assert not r.kinds("block_start") and "margin" in str(r.ab.last_reason)
  r2 = T.r_pending()
  r2.ab.set_plan_limits(tp.PLAN_ALT_MAX_MPH, 600.0)
  r2.secs(40.0, v_ego=T.v_mph(60.5))
  assert r2.kinds("block_start")
  assert tp.PLAN_ALT_MAX_MPH - ab.MARGIN_MPH == 61.0


def test_the_pages_state_where_a_block_starts():
  import re
  hdr = (HERE.parents[1] / "ui/qt/ti_ab_logic.h").read_text(encoding="utf-8")
  assert float(re.search(r"constexpr double START_MARGIN_MPH = ([\d.]+);", hdr).group(1)) == ab.MARGIN_MPH
