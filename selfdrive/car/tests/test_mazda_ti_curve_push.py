"""The push below the 15 mph line (owner, 2026-11-24): "I'm okay with setting cruise lower than the 15-mile-per-hour limit. I'm less
okay with actually making the car go 15 miles per hour less than the limit."

So the DASH set speed may go below the floor (the driver's set speed - 15, never below 25) by the push depth, while the car's ACTUAL
speed keeps the floor as a hard cap. Closed loop: the car side (ti_curve.Curve) on the phase-aware test car, the car braking as the ONE
cruise model says (both braking tables), with a planner stand-in that pushes as deep as the rules allow and raises when the model
says (as ti_curve_plan does), and then: on time, one tap cycle late, never, or the planner dying mid-push."""
import pytest

from openpilot.selfdrive.car.mazda import ti_cruise_model as CM
from openpilot.selfdrive.car.mazda import ti_curve as C
from openpilot.selfdrive.car.mazda import ti_curve_plan as P
from openpilot.selfdrive.car.tests.test_mazda_ti_curve import Car, plan

DT = C.DT
PLAN_EVERY = 20                      # the planner publishes at 5 Hz


def floor_of(ref):
  return max(C.ABS_FLOOR_MPH, ref - C.MAX_SHED_MPH)


def run_push(ref, variant, raise_mode="on_time", corner=None, seconds=40.0):
  """Cruise at `ref` (the driver's set speed, the car at it), a corner needing far less than the floor allows, the deepest push.
  raise_mode: on_time (the planner raises when CM.raise_due says), late (one tap cycle after that), never (the planner keeps asking
  for the push, alive), dead (the planner stops publishing once the dash has reached the push). -> (car, curve, min speed)."""
  car = Car(float(ref), ref)
  car.cruise = CM.Cruise(float(ref), ref, DT, variant)
  curve = C.Curve()
  fl = floor_of(ref)
  need = corner if corner is not None else C.ABS_FLOOR_MPH     # the corner wants as low as possible: the floor binds
  T = max(need, fl)
  shadow = CM.Cruise(float(ref), ref, 0.2)                     # the planner's model car (timing table), synced to the speed
  st = {"raise_at": None, "raised": False, "dead_f": None, "last": None}

  def plan_fn(f, c):
    """-> (plan, plan_age)."""
    if st["dead_f"] is not None:
      return st["last"], (f - st["dead_f"]) * DT + 0.1            # the last output, ageing: late after 1 s, dead after 3 s
    if f % PLAN_EVERY == 0:
      t = f * DT
      shadow.v = c.mph
      shadow.step(c.set)
      if not st["raised"] and st["raise_at"] is None and c.set < T and CM.raise_due(shadow, c.set, T):
        st["raise_at"] = t + (CM.TAP_S if raise_mode == "late" else 0.0)
      if st["raise_at"] is not None and t >= st["raise_at"] - 1e-9 and raise_mode != "never":
        st["raised"] = True
      if raise_mode == "dead" and curve.target is not None and curve.target < fl and c.set <= curve.target:
        st["dead_f"] = f                                         # dies the moment the push is reached
      st["last"] = plan(T if st["raised"] else C.ABS_FLOOR_MPH, corner=T, dist=300.0)   # as the planner: corner = the target
    return st["last"], 0.1

  lo = float(ref)
  for f in range(int(seconds / DT)):
    car.tick()
    p, age = plan_fn(f, car)
    inp = C.Inputs(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, brake=False,
                   gas=False, lead=False, driver_button=False, counter=car.counter, plan=p, plan_age=age, mono=f * DT)
    b = curve.step(f, inp)
    if b is not None:
      car.press(b)
    car.history.append((f, car.set, curve.state, curve.ref))
    lo = min(lo, car.mph)
  return car, curve, lo

REFS = (40, 45, 50, 55, 60, 65, 70)


@pytest.mark.parametrize("variant", (CM.TIMING, CM.CONSERVATIVE))
@pytest.mark.parametrize("raise_mode", ("on_time", "late", "never", "dead"))
@pytest.mark.parametrize("ref", REFS)
def test_actual_speed_keeps_the_15_mph_cap(ref, raise_mode, variant):
  car, curve, lo = run_push(ref, variant, raise_mode)
  fl = floor_of(ref)
  dash_lo = min(s for _, s, _, _ in car.history)
  # the dash may go below the floor by the push (never below 25 nor the car's minimum) ...
  assert dash_lo >= max(C.ABS_FLOOR_MPH, P.CAR_MIN_MPH, CM.push_floor(fl, P.DRIVE_GAP_MPH))
  if fl - P.DRIVE_GAP_MPH >= C.ABS_FLOOR_MPH:
    assert dash_lo == fl - P.DRIVE_GAP_MPH, dash_lo             # the deepest push was really used
  # ... the car's speed never: not below the driver's set speed - 15, not below the absolute floor
  assert lo >= fl - 1e-6, (lo, fl)
  assert lo >= C.ABS_FLOOR_MPH - 1e-6
  assert curve.state != "stopped", curve.why
  if raise_mode == "on_time":
    assert not any(e["kind"] == "push_net" for e in curve.events)   # a planner raising on time is never overruled
  else:
    assert fl - P.DRIVE_GAP_MPH < C.ABS_FLOOR_MPH or raise_mode == "late" or any(e["kind"] == "push_net" for e in curve.events)


def test_the_cap_is_measured_against_the_drivers_set_speed():
  # the driver's set speed (60) is the reference, not a lower car speed reached on the way: floor 45, push to 39, car >= 45
  car, curve, lo = run_push(60, CM.TIMING, "never")
  assert curve.ref == 60 and curve.floor == 45 and lo >= 45 - 1e-6
  assert any(e["kind"] == "push_net" for e in curve.events)        # this side raised it back by itself, and logged it


def test_push_net_does_not_fire_when_the_target_is_above_the_floor():
  # a corner needing 50 from 60 (floor 45): the target 50 is above the floor, the push to 44 is the planner's ordinary one and is
  # raised by the planner on time; the safety net never acts
  car, curve, lo = run_push(60, CM.TIMING, "on_time", corner=50)
  assert not any(e["kind"] == "push_net" for e in curve.events)
  assert lo >= 50 - 0.5
