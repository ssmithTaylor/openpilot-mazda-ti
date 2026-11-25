"""Planner restart mid-bend (2026-11-24): the car side remembers the lowest ceiling it acted on in the slowdown and sends no SET+ above
it because the planner's published ceiling jumped up or vanished, in a bend driven under the 1.3 m/s2 bend rule too (1.1 here); it
lets go on the straight road past the corner's end, TRUST_HOLD_M past the end, CAP_STALE_S with no confirming plan, or an explicit
end of the intervention."""
import importlib.util
import pathlib

import pytest

from openpilot.selfdrive.car.mazda import ti_curve as C

_spec = importlib.util.spec_from_file_location("tc_cap_car", pathlib.Path(__file__).parent / "test_mazda_ti_curve.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)

SOFT = dict(la_now=1.1, desired_la=1.1)        # a bend the 1.3 m/s2 bend rule does not see
STRAIGHT = dict(la_now=0.2, desired_la=0.2)


def _slowed(dist=600.0):
  car, cv = T.Car(55.0, 55), C.Curve()
  T.run(cv, car, 3000, lambda f, c: T.plan(40, dist=dist), **STRAIGHT)
  assert car.set == 40 and cv.intervening() and cv.cap_lo == 40
  return car, cv


@pytest.mark.parametrize("later", ["higher_corner", "vanished"])
def test_restarted_planner_cannot_raise_above_the_remembered_ceiling_in_a_soft_bend(later):
  car, cv = _slowed()
  after = (lambda f, c: T.plan(50, dist=0.0)) if later == "higher_corner" else (lambda f, c: T.plan(None))
  nxt = lambda f, c: None if f < 3000 + 300 else after(f, c)                  # the planner restarts: no plan for 3 s, then its word
  T.run(cv, car, 1500, nxt, start=3000, **SOFT)                # 15 s (270 m) at 1.1 m/s2, inside the corner
  assert car.set == 40, car.set
  assert any(e["kind"] == "cap_hold" for e in cv.events)


@pytest.mark.parametrize("later", ["higher_corner", "vanished"])
def test_straight_road_past_the_end_lets_go(later):
  car, cv = _slowed(150.0)
  after = (lambda f, c: T.plan(50, dist=0.0)) if later == "higher_corner" else (lambda f, c: T.plan(None))
  nxt = lambda f, c: None if f < 3000 + 300 else after(f, c)
  T.run(cv, car, 1500, nxt, start=3000, **STRAIGHT)       # straight and 270 m on: past the corner's end (150 m ahead at the restart)
  assert car.set > 40


def test_cap_cannot_stick_forever_in_a_long_soft_bend():
  car, cv = _slowed()
  T.run(cv, car, int((C.CAP_STALE_S + 5) / C.DT), lambda f, c: T.plan(None), start=3000, **SOFT)
  assert cv.cap_lo is None
  T.run(cv, car, 1500, lambda f, c: T.plan(None), start=6000, **SOFT)
  assert car.set > 40                                            # nothing else holds it: the restore goes ahead


def test_a_lower_ceiling_lowers_the_cap_and_the_planned_raise_to_it_is_allowed():
  car, cv = T.Car(55.0, 55), C.Curve()
  T.run(cv, car, 800, lambda f, c: T.plan(42, corner=48, dist=300.0), **STRAIGHT)       # the push below the corner speed
  assert car.set == 42 and cv.cap_lo == 48
  T.run(cv, car, 1500, lambda f, c: T.plan(48, corner=48, dist=0.0), start=800, **SOFT)
  assert car.set == 48


@pytest.mark.parametrize("revised", [lambda f, c: T.plan(48, dist=500.0), lambda f, c: T.plan(None)])
def test_continuous_stream_that_revises_or_drops_a_corner_on_a_straight_is_obeyed_at_once(revised):
  car, cv = _slowed(600.0)
  T.run(cv, car, 100, revised, start=3000, **STRAIGHT)           # 1 s: raised as before the cap existed
  assert car.set > 40 and not any(e["kind"] == "cap_hold" for e in cv.events)


def test_cap_without_a_published_end_is_not_released_by_the_corner_start():
  car, cv = T.Car(55.0, 55), C.Curve()
  T.run(cv, car, 3000, lambda f, c: T.plan(40, dist=20.0), **STRAIGHT)       # "dist" only: the corner's START, no end
  assert cv.cap_lo == 40 and cv.cap_end is None
