"""Serialization constraints for legacy curvature observations, not input identities.

These reject incompatible candidates without consulting controller outputs.
Overlapping Float32 rounding intervals are only a necessary condition: ties,
unlogged internal precision, and equal-valued publications can remain ambiguous.
"""
import math
import struct


def float32(value: float) -> float:
  """Round a finite value as a Cap'n Proto Float32 field would be stored."""
  if not math.isfinite(value):
    raise ValueError("Nonfinite observation")
  try:
    result = struct.unpack("<f", struct.pack("<f", value))[0]
  except OverflowError as exc:
    raise ValueError("Float32 overflow") from exc
  if not math.isfinite(result):
    raise ValueError("Float32 overflow")
  return result


def rounding_interval(value: float) -> tuple[float, float]:
  """Conservative closed interval that can serialize to this finite Float32.

  Midpoint ties are deliberately included at both ends. Both signed zeros use
  the union of their numerical intervals; this check does not infer sign bits.
  """
  if float32(value) != value:
    raise ValueError("Observation is not an exactly represented Float32")
  bits = struct.unpack("<I", struct.pack("<f", abs(value)))[0]
  if bits == 0:
    return (-2.0**-150, 2.0**-150)
  lower = struct.unpack("<f", struct.pack("<I", bits-1))[0]
  upper = struct.unpack("<f", struct.pack("<I", bits+1))[0]
  if not math.isfinite(upper):
    upper = abs(value) + (abs(value)-lower)
  lo, hi = (lower+abs(value))/2, (abs(value)+upper)/2
  return (lo, hi) if value > 0 else (-hi, -lo)


def product_compatible(recorded_factor: float, scale: float, recorded_product: float) -> bool:
  """Can one internal factor fit both serialized factor and product fields?"""
  if not math.isfinite(scale) or scale <= 0:
    raise ValueError("Scale must be finite and positive")
  lo, hi = rounding_interval(recorded_factor)
  plo, phi = rounding_interval(recorded_product)
  return lo*scale <= phi and hi*scale >= plo


def curvature_compatible(*, computed_actual: float, recorded_actual: float,
                         speed: float, recorded_actual_accel: float,
                         recorded_desired: float, recorded_desired_accel: float) -> bool:
  """Check the shared legacy angle-model equations and Float32 serialization.

  Requires source verification that these exact equations produce the fields.
  Never use the outcome as independent physical truth or a consumed-input ID.
  """
  return (float32(computed_actual) == recorded_actual
          and float32(computed_actual*speed**2) == recorded_actual_accel
          and product_compatible(recorded_desired, speed**2, recorded_desired_accel))
