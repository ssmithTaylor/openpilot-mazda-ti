import math
from types import SimpleNamespace
import pytest
import numpy as np
from openpilot.selfdrive.car.mazda.lateral_reference import LaneContext, LaneObserver, LaneRelease


def model():
  x = np.linspace(0, 30, 31)
  mid = -0.5 - math.tan(math.radians(1)) * x + 0.002 * x**2
  return SimpleNamespace(
    timestampEof=1000000000,
    meta=SimpleNamespace(laneChangeState='off'),
    laneLineProbs=[0.0, 0.99, 0.99, 0.0],
    laneLines=[SimpleNamespace(), SimpleNamespace(x=x, y=mid - 1.8), SimpleNamespace(x=x, y=mid + 1.8)],
  )


class TestLaneObserver:
  def test_camera_and_event_clocks_can_have_different_suspend_offsets(self):
    message = model()
    message.timestampEof += 10_000_000_000
    observer = LaneObserver()
    result = observer.update(message, 1_050_000_000, True, 1_100_000_000, 11_100_000_000)
    assert result.valid and result.age == .1
    assert not observer.update(message, 1_050_000_000, True, 1_200_000_001, 11_200_000_001).valid
    assert observer.update(message, 1_200_000_000, True, 1_100_000_000, 11_100_000_000).reason == 'model_time'

  def test_failed_numeric_fit_rejects_context(self, monkeypatch):
    def fail(*args, **kwargs):
      raise np.linalg.LinAlgError('SVD did not converge')
    monkeypatch.setattr(np.polynomial.polynomial, 'polyfit', fail)
    result = LaneObserver().update(model(), 1_050_000_000, True, 1_100_000_000)
    assert not result.valid and result.reason == 'fit_failure'
  def test_same_frame_is_cached_but_health_and_age_are_not(self, monkeypatch):
    observer = LaneObserver()
    message = model()
    original_fit = np.polynomial.polynomial.polyfit
    calls = []

    def fit(*args, **kwargs):
      calls.append(None)
      return original_fit(*args, **kwargs)

    monkeypatch.setattr(np.polynomial.polynomial, 'polyfit', fit)
    first = observer.update(message, 1050000000, True, 1100000000)
    assert first.valid
    assert first.offset == pytest.approx(0.5)
    assert first.heading10 == pytest.approx(math.radians(1))
    second = observer.update(message, 1050000000, True, 1150000000)
    assert second.age == 0.15
    assert len(calls) == 2
    assert not observer.update(message, 1050000000, False, 1160000000).valid
    assert not observer.update(message, 1050000000, True, 1200000001).valid
    assert len(calls) == 2
    observer.update(message, 1060000000, True, 1170000000)
    assert len(calls) == 4

  def test_future_missing_and_invalid_model_times(self):
    observer = LaneObserver()
    assert not observer.update(None, 0, False, 1100000000).valid
    assert not observer.update(model(), 1150000000, True, 1100000000).valid
    assert not observer.update(model(), 0, True, 1100000000).valid
    assert not observer.update(model(), 950000000, True, 999999999).valid

  def test_geometry_rejections(self):
    for field in ('confidence', 'lane_change', 'nan', 'width', 'support'):
      message = model()
      if field == 'confidence':
        message.laneLineProbs[1] = float('nan')
      elif field == 'lane_change':
        message.meta.laneChangeState = 'laneChangeStarting'
      elif field == 'nan':
        message.laneLines[1].y[3] = float('nan')
      elif field == 'width':
        message.laneLines[2].y += 4
      elif field == 'support':
        message.laneLines[1].x = message.laneLines[1].x[::-1]
      assert not LaneObserver().update(message, 1050000000, True, 1100000000).valid


class TestLaneRelease:
  def setup_method(self):
    self.context = LaneContext(True, 0.5, 0.0, 0.0, 0.1, 'valid', 2.0 / 625, 2.0 / 625)

  def step(self, release, context=None, latest=2.4, rate=0.0, enabled=True):
    return release.update(2.4, 2.4, 2.9, 2.9, 3.0, enabled, 0.01, self.context if context is None else context, 25.0, rate, latest)

  def test_inside_opening_and_outward_recovery_differ(self):
    release = LaneRelease()
    for _ in range(19):
      assert self.step(release) == (2.9, 2.9)
    assert self.step(release)[0] < 2.9
    outward = LaneContext(True, 0.5, -0.02, -0.02, 0.1, 'valid', 2.0 / 625, 2.0 / 625)
    previous = release.removed[:]
    self.step(release, outward)
    assert all((a <= b for a, b in zip(release.removed, previous, strict=False)))
    recovering = LaneRelease()
    for _ in range(60):
      assert self.step(recovering, outward) == (2.9, 2.9)

  def test_context_loss_and_fresh_tightening_restore_without_latching(self):
    for lost_context, latest in ((LaneContext(), 2.4), (self.context, 3.1)):
      release = LaneRelease()
      for _ in range(40):
        self.step(release)
      assert release.removed[0] > 0.1
      for _ in range(60):
        previous = release.removed[:]
        self.step(release, lost_context, latest)
        assert all((a <= b for a, b in zip(release.removed, previous, strict=False)))
      assert release.removed == [0.0, 0.0]
      assert self.step(release) == (2.9, 2.9)

  def test_unwind_stops_growth_and_reset_clears_dwell(self):
    release = LaneRelease()
    for _ in range(40):
      self.step(release)
    previous = release.removed[:]
    for _ in range(20):
      self.step(release, rate=3.0)
      assert all((a <= b for a, b in zip(release.removed, previous, strict=False)))
      previous = release.removed[:]
    self.step(release, enabled=False)
    assert release.removed == [0.0, 0.0]
    assert self.step(release) == (2.9, 2.9)
