import math

import pytest

from tools.mazda_ti.legacy_input_constraints import curvature_compatible, float32, product_compatible, rounding_interval


def test_actual_serialization_rejects_nearby_but_different_field():
  actual = .0039123456789
  args = dict(computed_actual=actual, recorded_actual=float32(actual), speed=float32(22.3),
              recorded_actual_accel=float32(actual*float32(22.3)**2),
              recorded_desired=float32(.004123456789),
              recorded_desired_accel=float32(.004123456789*float32(22.3)**2))
  assert curvature_compatible(**args)
  args['recorded_actual'] = float32(actual+1e-9)
  assert not curvature_compatible(**args)


def test_product_allows_internal_precision_lost_in_first_serialization():
  internal = 1.0 + 2.0**-25
  scale = 1.0 + 2.0**-24
  assert float32(float32(internal)*scale) != float32(internal*scale)
  assert product_compatible(float32(internal), scale, float32(internal*scale))
  assert not product_compatible(float32(internal), scale, float32(internal*scale+.01))


def test_signed_intervals_zero_and_extreme_finite():
  lo, hi = rounding_interval(1.)
  assert rounding_interval(-1.) == (-hi, -lo)
  assert rounding_interval(0.) == rounding_interval(-0.) == (-2.0**-150, 2.0**-150)
  assert all(math.isfinite(v) for v in rounding_interval(float32(3.4028234663852886e38)))
  assert product_compatible(-1., 3., -3.)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf'), 1e100])
def test_nonfinite_or_overflow_rejected(value):
  with pytest.raises(ValueError):
    float32(value)


def test_invalid_serialized_observation_and_scale_rejected():
  with pytest.raises(ValueError, match='exactly represented'):
    rounding_interval(.1)
  for scale in [0., -1., float('nan'), float('inf')]:
    with pytest.raises(ValueError, match='Scale'):
      product_compatible(1., scale, 1.)
