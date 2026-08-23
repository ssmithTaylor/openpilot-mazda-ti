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
    self.assertAlmostEqual(SA.la_at_ceiling(73.0), 2.55, delta=0.15)

  def test_ceiling_is_flat_with_speed(self):
    # An earlier version had this rising (2.21 at 52 km/h to 3.29 at 90) because it converted a
    # ceiling ANGLE to acceleration with a pure v^2 map while the slope it divided by already
    # carried the understeer denominator -- a double count. The torque ceiling is a front-axle
    # FORCE ceiling, so the acceleration ceiling is flat. Measured slope over 61 latch events is
    # -0.003 m/s^2 per km/h; the old model assumed +0.028.
    for v in (60.0, 73.0, 90.0, 113.0):
      self.assertAlmostEqual(SA.la_at_ceiling(v), SA.CEILING_LA, delta=1e-6)

  def test_catches_a_fast_hard_corner(self):
    # The regression guard for that bug. Under the rising-ceiling model this scored as fitting
    # comfortably and produced no advisory at all -- a false negative at speed, the dangerous
    # direction. The 113 km/h latch in the corpus delivers 2.62, not the 3.9 the old model implied.
    self.assertGreater(SA.advisory_speed_margin(3.2, 90.0), 0.0)

  def test_never_advises_below_the_lkas_gate(self):
    # Below ~45-52 km/h the EPS drops the stock path and authority falls 3-4x. That is the one
    # regime where slowing costs more than it buys, so the advisory must not steer into it.
    for demand in (3.0, 4.0, 5.0, 6.0):
      for kph in (58.0, 65.0, 75.0):
        advised = SA.advisory_speed_margin(demand, kph)
        self.assertTrue(advised == 0.0 or advised >= SA.MIN_ADVISORY_FLOOR_KPH)

  def test_does_not_nag_with_trivial_advice(self):
    # A 1 km/h advisory is below the resolution of what the driver can set and reads as noise.
    for demand, kph in ((3.09, 73.0), (2.94, 73.0)):
      self.assertEqual(SA.advisory_speed_margin(demand, kph), 0.0)

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
    # The rescued passes sit clearly over the budget. The completed ones sit just above it too --
    # 1.06 to 1.20 m at the corrected 2.55 ceiling -- so the drift criterion ALONE does not separate
    # them, and the display floor is carrying part of the discrimination. That is not a papered-over
    # weakness but a property of the corpus: several passes at the same demand went both ways, so no
    # threshold on demand can cleanly split them. What must hold is the ordering and a real gap.
    completed = max(SA.predicted_drift_m(d, 73) for d in (3.08, 2.94, 3.15))
    rescued = min(SA.predicted_drift_m(d, 72) for d in (3.49, 3.55, 3.59))
    self.assertGreater(rescued, completed + 0.3, "the two populations have stopped separating")
    self.assertGreater(rescued, SA.ALLOWED_DRIFT_M)


class TestAdvisoryTrigger(unittest.TestCase):
  """The display logic that made the advisory audible: occupancy firing, the hold, and the
  monotone-down shown value. Calibrated on route 0000027a (see steer_authority.py)."""

  def _run(self, trig, frames):
    out = []
    for active, fla, v in frames:
      out.append(trig.update(active, fla, v))
    return out

  def test_fires_on_blippy_crossings(self):
    # The forecast crosses the threshold in sub-0.2 s blips on approach; a plain consecutive
    # sustain erased every early lead in replay. Interleave crossings with sub-threshold frames:
    # 4 crossings inside the 30-frame window must fire.
    trig = SA.AdvisoryTrigger()
    frames = []
    for _ in range(4):
      frames.append((True, 2.9, 74.0))     # over threshold (flat drift 0.7)
      frames.append((True, 2.0, 74.0))     # back under
      frames.append((True, 2.0, 74.0))
    out = self._run(trig, frames)
    self.assertGreater(out[-1], 0.0, "four blips in the window did not fire")

  def test_needs_occupancy_not_one_frame(self):
    trig = SA.AdvisoryTrigger()
    out = self._run(trig, [(True, 2.9, 74.0)] + [(True, 2.0, 74.0)] * 29)
    self.assertEqual(out[-1], 0.0, "a single crossing fired the advisory")

  def test_silent_below_the_floor_speed(self):
    # The phantom-drift guard: both recorded false fires were forecast blips at 49-55 km/h read
    # against the uncalibrated low-speed ceiling. Below the floor the trigger must not arm, no
    # matter how large the forecast.
    trig = SA.AdvisoryTrigger()
    out = self._run(trig, [(True, 5.0, 50.0)] * 60)
    self.assertTrue(all(x == 0.0 for x in out))

  def test_silent_when_toggled_off(self):
    trig = SA.AdvisoryTrigger()
    out = self._run(trig, [(False, 3.5, 74.0)] * 60)
    self.assertTrue(all(x == 0.0 for x in out))

  def test_silent_under_threshold(self):
    # flat-ceiling drift at 2.6 m/s^2 is 0.1 m -- under ADVISORY_FIRE_DRIFT_M.
    trig = SA.AdvisoryTrigger()
    out = self._run(trig, [(True, 2.6, 74.0)] * 60)
    self.assertTrue(all(x == 0.0 for x in out))

  def test_holds_after_the_trigger_clears(self):
    # 0.4 s of display was unreadable on the road. Once fired, the value must survive the trigger
    # dropping, for ADVISORY_HOLD_FRAMES, then clear.
    trig = SA.AdvisoryTrigger()
    self._run(trig, [(True, 2.9, 74.0)] * 10)          # fire
    after = self._run(trig, [(True, 0.0, 74.0)] * (SA.ADVISORY_HOLD_FRAMES - 5))
    self.assertGreater(after[-1], 0.0, "the hold did not carry the display")
    cleared = self._run(trig, [(True, 0.0, 74.0)] * 20)
    self.assertEqual(cleared[-1], 0.0, "the hold never released")

  def test_shown_speed_never_rises_while_showing(self):
    # The raw value dithered 42/41/40 at 10 Hz on the road, re-chiming each change. While showing,
    # the number may only fall.
    trig = SA.AdvisoryTrigger()
    shown = self._run(trig, [(True, 2.9, 74.0)] * 10 + [(True, 3.4, 74.0)] * 10
                            + [(True, 2.9, 74.0)] * 10)
    showing = [x for x in shown if x > 0.0]
    self.assertGreater(len(showing), 5)
    for a, b in zip(showing, showing[1:]):
      self.assertLessEqual(b, a + 1e-9, "the shown advisory speed rose while displayed")

  def test_advised_speed_is_meaningfully_slower(self):
    trig = SA.AdvisoryTrigger()
    out = self._run(trig, [(True, 3.4, 74.0)] * 10)
    self.assertGreater(out[-1], 0.0)
    self.assertLess(out[-1], 74.0 - SA.MIN_MEANINGFUL_SLOWDOWN_KPH + 1e-6)


if __name__ == "__main__":
  unittest.main()
