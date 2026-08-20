"""Tests for the steering-authority advisory.

The properties worth pinning are the ones that make it usable rather than merely correct: it must
stay silent on corners the car actually completed, it must never advise a speed increase, and its
search must not be able to collapse the way it did on its first acceptance run -- an inverted
bisection drove the answer to the floor, where the maximum-slowdown guard then suppressed it
entirely, so the advisory silently never fired and every test that only checked "no crash" passed.
"""
import unittest

from openpilot.frogpilot.controls.lib import steer_authority as SA


class TestCeiling(unittest.TestCase):
  def test_matches_the_observed_latch(self):
    # The fit is calibrated on SAT slope, not on this. At 73 km/h the corpus shows the rack latching
    # at 21-24 deg delivering 2.6-2.7 m/s^2, so the model reproducing it is a real check.
    self.assertAlmostEqual(SA.la_at_ceiling(73.0), 2.65, delta=0.15)

  def test_available_grip_rises_with_speed(self):
    # Counter-intuitive but measured: a given steering angle buys more lateral acceleration the
    # faster you go, and SAT per degree rises only as v^1.28. Slowing costs authority as well as
    # demand, which is exactly why a naive v^2 estimate under-advises.
    self.assertLess(SA.la_at_ceiling(55.0), SA.la_at_ceiling(73.0))
    self.assertLess(SA.la_at_ceiling(73.0), SA.la_at_ceiling(90.0))

  def test_torque_rises_with_angle_and_speed(self):
    self.assertLess(SA.sat_torque(15.0, 73.0), SA.sat_torque(25.0, 73.0))
    self.assertLess(SA.sat_torque(20.0, 60.0), SA.sat_torque(20.0, 85.0))


class TestAdvisory(unittest.TestCase):
  def test_silent_on_corners_the_car_completed(self):
    # Five recorded hands-off passes, demand against the speed they were taken at. Every one of
    # these finished without the driver touching the wheel, so advising a slowdown would be noise.
    for demand, kph in ((3.08, 73), (2.94, 73), (2.95, 73), (3.09, 73), (2.81, 73), (3.15, 73)):
      self.assertEqual(SA.advisory_speed_margin(demand, kph), 0.0,
                       f"advised a slowdown on a corner the car completed: {demand} at {kph}")

  def test_fires_on_corners_that_needed_the_driver(self):
    # The hard direction, every recorded pass. All of these ran out of steering.
    for demand, kph in ((3.54, 71), (3.55, 72), (3.59, 72), (3.49, 73), (3.58, 72), (3.51, 72)):
      advised = SA.advisory_speed_margin(demand, kph)
      self.assertGreater(advised, 0.0, f"stayed silent on a corner that needed rescue: {demand}")
      self.assertLess(advised, kph, "advised a speed at or above the current one")

  def test_never_advises_speeding_up(self):
    for demand in (1.0, 2.0, 3.0, 4.0, 5.0):
      for kph in (40, 55, 70, 85, 100):
        advised = SA.advisory_speed_margin(demand, kph)
        self.assertTrue(advised == 0.0 or advised < kph)

  def test_the_advised_speed_actually_fits(self):
    # The regression guard. A search that collapses to the floor still returns "a number"; what it
    # does not do is return a speed at which the corner fits.
    for demand, kph in ((3.54, 71), (3.58, 72), (4.0, 80)):
      advised = SA.advisory_speed_margin(demand, kph)
      if advised <= 0.0:
        continue
      scaled = demand * (advised / kph) ** 2
      self.assertLessEqual(SA.predicted_drift_m(scaled, advised), SA.ALLOWED_DRIFT_M + 0.05,
                           "the advised speed does not bring the corner inside the drift budget")

  def test_harder_corners_advise_slower(self):
    a = SA.advisory_speed_margin(3.5, 73)
    b = SA.advisory_speed_margin(4.2, 73)
    self.assertGreater(a, 0.0)
    self.assertGreater(b, 0.0)
    self.assertLess(b, a)

  def test_refuses_to_guess_outside_its_range(self):
    # An absurd corner is not an excuse to invent a number; saying nothing is the honest output.
    self.assertEqual(SA.advisory_speed_margin(12.0, 73), 0.0)
    self.assertEqual(SA.advisory_speed_margin(3.5, 20), 0.0)


class TestDrift(unittest.TestCase):
  def test_zero_when_inside_authority(self):
    self.assertEqual(SA.predicted_drift_m(2.0, 73.0), 0.0)

  def test_grows_with_demand(self):
    self.assertLess(SA.predicted_drift_m(3.0, 73.0), SA.predicted_drift_m(3.6, 73.0))

  def test_separates_the_two_populations(self):
    # The whole design rests on this gap: the completed passes sit under the budget, the rescued
    # ones over it. If these ever converge the criterion has stopped discriminating.
    completed = max(SA.predicted_drift_m(d, 73) for d in (3.08, 2.94, 3.15))
    rescued = min(SA.predicted_drift_m(d, 72) for d in (3.49, 3.55, 3.59))
    self.assertLess(completed, SA.ALLOWED_DRIFT_M)
    self.assertGreater(rescued, SA.ALLOWED_DRIFT_M)


if __name__ == "__main__":
  unittest.main()
