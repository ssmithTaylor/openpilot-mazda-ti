"""Car-side fixes for a winding road (2026-11-25): a restore without a model preview when trusted memory says the road ahead is clear;
SET+ never inside a corner's extent (the plan's corner start/end), whatever the lateral acceleration."""
import importlib.util
import pathlib

from openpilot.selfdrive.car.mazda import ti_curve as C

_spec = importlib.util.spec_from_file_location("tc_cs_car", pathlib.Path(__file__).parent / "test_mazda_ti_curve.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
STRAIGHT = dict(la_now=0.2, desired_la=0.2)


def _slowed():
  car, cv = T.Car(55.0, 55), C.Curve()
  T.run(cv, car, 3000, lambda f, c: T.plan(40, dist=60.0), **STRAIGHT)
  assert car.set == 40 and cv.intervening()
  return car, cv


def test_restore_without_a_model_preview_on_trusted_clear_road():
  car, cv = _slowed()
  T.run(cv, car, 3000, lambda f, c: T.plan(None, model_k=None), start=3000, **STRAIGHT)
  assert car.set == 55, car.set


def test_restore_without_a_preview_goes_only_to_the_trusted_hold_ceiling():
  car, cv = _slowed()
  T.run(cv, car, 3000, lambda f, c: T.plan(46, kind="hold", model_k=None), start=3000, **STRAIGHT)
  assert car.set == 46, car.set


def test_no_preview_and_untrusted_memory_stays_blocked():
  car, cv = _slowed()
  T.run(cv, car, 3000, lambda f, c: T.plan(None, trusted=False, model_k=None), start=3000, **STRAIGHT)
  assert car.set == 40, car.set


def test_no_preview_never_raises_in_a_bend():
  car, cv = _slowed()
  T.run(cv, car, 1500, lambda f, c: T.plan(None, model_k=None), start=3000, la_now=2.2, desired_la=2.2)
  assert car.set == 40, car.set
