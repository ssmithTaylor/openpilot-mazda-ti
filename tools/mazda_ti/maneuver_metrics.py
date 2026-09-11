"""Pure observed-drive maneuver metrics on explicit rows; no physical acceptance decision.

Rows come from observed_drive_rows.py. Each function declares the health groups it
needs and uses only rows healthy for those groups. Time is integer nanoseconds at
the interface and seconds in durations; values are left-constant between samples;
nothing bridges a gap longer than Parameters.max_gap_s. Lane geometry is one
camera-model family, not surveyed truth; TI counts are command evidence only.
"""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass, fields
import math

NS = 1_000_000_000
GROUPS = ('lane', 'speed', 'steering', 'yaw', 'accel', 'roll', 'controls', 'command')
STATUSES = ('measured_estimate', 'right_censored', 'unresolved', 'unscorable')


@dataclass(frozen=True)
class Parameters:
  params_id: str
  max_gap_s: float = 0.2
  smooth_span_s: float = 0.25
  velocity_span_s: float = 0.5
  plateau_deadband_mps: float = 0.02
  entry_accel_mps2: float = 1.0
  exit_accel_mps2: float = 0.5
  anchor_persistence_s: float = 0.5
  recovery_horizon_s: float = 8.0
  e_settle_m: float | None = None
  v_settle_mps: float | None = None
  t_dwell_s: float = 1.0
  t_settle_max_s: float | None = None
  a_min_m: float | None = None
  t_min_s: float = 0.4
  episode_quiet_s: float = 2.0
  corridor_half_width_m: float | None = None
  jerk_smooth_span_s: float = 0.25
  j_limit_mps3: float | None = None
  min_eligible_cases: int = 2
  min_recovery_support_s: float = 4.0

  def __post_init__(self):
    for f in fields(self):
      value = getattr(self, f.name)
      if f.name == 'params_id':
        if not isinstance(value, str) or not value.strip():
          raise ValueError('params_id must be a nonempty string')
      elif value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                  or not math.isfinite(value) or value < 0):
        raise ValueError(f'{f.name} must be a finite nonnegative number or null')
    if self.max_gap_s <= 0 or self.smooth_span_s <= 0 or self.velocity_span_s <= 0:
      raise ValueError('Spans and gap limit must be positive')

  def as_dict(self):
    return asdict(self)

  @property
  def max_gap_ns(self):
    return int(round(self.max_gap_s * NS))

  @property
  def absence_edge_s(self):
    """Observation lost at each end of an interval purely to smoothing and derivative support."""
    return self.smooth_span_s + self.velocity_span_s


def absence_supported(cov, params):
  """True only when the whole relevant interval was observed apart from filter-edge loss.

  A symptom can be asserted absent only if no internal gap exceeds max_gap_s and the
  leading/trailing unobserved spans are no longer than the filter edge; otherwise
  missing data could contain the entire event.
  """
  return (cov['maximum_gap_s'] <= params.max_gap_s and cov['leading_unobserved_s'] <= params.absence_edge_s
          and cov['trailing_unobserved_s'] <= params.absence_edge_s)


DEFAULT_PARAMETERS = Parameters(params_id='observed-drive-v1-draft')


def _t(row):
  return int(row['mono_ns'])


def healthy_rows(rows, groups):
  """Rows whose declared health groups are all healthy, in time order."""
  unknown = set(groups) - set(GROUPS)
  if unknown:
    raise ValueError(f'Unknown health groups: {sorted(unknown)}')
  out = sorted((r for r in rows if all(r['health'].get(g) is True for g in groups)), key=_t)
  for a, b in zip(out, out[1:]):
    if _t(b) <= _t(a):
      raise ValueError('Duplicate or unordered row timestamps')
  return out


def segments(rows, max_gap_ns):
  """Split ordered rows into observation segments at gaps over max_gap_ns."""
  parts, current = [], []
  for row in rows:
    if current and _t(row) - _t(current[-1]) > max_gap_ns:
      parts.append(current)
      current = []
    current.append(row)
  if current:
    parts.append(current)
  return parts


def _bracketed(times, lo, hi, max_gap_ns):
  """Samples exist at or before lo and at or after hi, with no internal gap over max_gap_ns."""
  i, j = bisect_right(times, lo) - 1, bisect_left(times, hi)
  if i < 0 or j >= len(times):
    return False
  return all(times[k + 1] - times[k] <= max_gap_ns for k in range(i, j))


def _integral_left_constant(times, values, lo, hi):
  total = 0.0
  for i in range(len(times) - 1):
    left, right = max(times[i], lo), min(times[i + 1], hi)
    if right > left:
      total += values[i] * (right - left) / NS
  return total


def smoothed(rows, key, params, span_s=None):
  """Centred boxcar over [t-h, t+h] with bracketing support; null where unsupported."""
  times = [_t(r) for r in rows]
  values = [float(r[key]) for r in rows]
  half = int(round((params.smooth_span_s if span_s is None else span_s) * NS / 2))
  out = []
  for t in times:
    if _bracketed(times, t - half, t + half, params.max_gap_ns):
      out.append(_integral_left_constant(times, values, t - half, t + half) / (2 * half / NS))
    else:
      out.append(None)
  return out


def _nearest(times, target, max_offset_ns):
  """Index of the sample nearest target, or None if none lies within max_offset_ns."""
  j = bisect_left(times, target)
  candidates = [i for i in (j - 1, j) if 0 <= i < len(times)]
  if not candidates:
    return None
  i = min(candidates, key=lambda k: abs(times[k] - target))
  return i if abs(times[i] - target) <= max_offset_ns else None


def derivative(rows, key, params, span_s=None):
  """Central difference of the smoothed series between the samples nearest t +/- velocity_span_s/2.

  The difference is divided by the actual separation of the two samples used, so a
  timestamp perturbation moves the estimate smoothly instead of switching a held
  endpoint under a nominal span. Null where either endpoint is missing, unsupported,
  further than max_gap_s from its target, or separated from the other by a gap.
  """
  times = [_t(r) for r in rows]
  s = smoothed(rows, key, params, span_s)
  d = int(round(params.velocity_span_s * NS / 2))
  out = []
  for t in times:
    a, b = _nearest(times, t - d, params.max_gap_ns), _nearest(times, t + d, params.max_gap_ns)
    if a is None or b is None or a == b or s[a] is None or s[b] is None or not _bracketed(times, times[a], times[b], params.max_gap_ns):
      out.append(None)
    else:
      out.append((s[b] - s[a]) / ((times[b] - times[a]) / NS))
  return out


def elapsed_weights(rows, start_ns, end_ns, max_gap_ns):
  """Seconds each row's value is held within [start_ns, end_ns).

  A value is held only until the next sample, and only when that sample lies within
  max_gap_ns. The last sample holds nothing: observation is never fabricated beyond
  the final supported interval. Pass rows including margin rows beyond the window so
  the trailing bracket exists; the weights clip to the window.
  """
  times = [_t(r) for r in rows]
  weights = []
  for i, t in enumerate(times):
    if i + 1 >= len(times) or times[i + 1] - t > max_gap_ns:
      weights.append(0.0)
      continue
    left, right = max(t, start_ns), min(times[i + 1], end_ns)
    weights.append(max(0.0, (right - left) / NS))
  return weights


def coverage(rows, start_ns, end_ns, max_gap_ns):
  """Supported observation inside the window, with leading/trailing unobserved spans."""
  weights = elapsed_weights(rows, start_ns, end_ns, max_gap_ns)
  times = [_t(r) for r in rows]
  supported = [(max(times[i], start_ns), min(times[i + 1], end_ns)) for i in range(len(times) - 1)
               if weights[i] > 0]
  first = supported[0][0] if supported else end_ns
  last = supported[-1][1] if supported else start_ns
  internal = [(times[i + 1] - times[i]) / NS for i in range(len(times) - 1)
              if times[i + 1] > start_ns and times[i] < end_ns and times[i + 1] - times[i] > max_gap_ns]
  return {'valid_duration_s': sum(weights), 'required_duration_s': (end_ns - start_ns) / NS,
          'maximum_gap_s': max(internal, default=0.0), 'leading_unobserved_s': (first - start_ns) / NS,
          'trailing_unobserved_s': (end_ns - last) / NS if supported else (end_ns - start_ns) / NS}


def weighted_rms(values, weights):
  total = sum(weights)
  if total <= 0:
    return None
  return math.sqrt(sum(w * v * v for v, w in zip(values, weights, strict=True)) / total)


def weighted_percentile(values, weights, q):
  """Smallest value whose cumulative weight fraction reaches q."""
  pairs = sorted((v, w) for v, w in zip(values, weights, strict=True) if w > 0)
  total = sum(w for _, w in pairs)
  if total <= 0:
    return None
  running = 0.0
  for v, w in pairs:
    running += w
    if running / total >= q:
      return v
  return pairs[-1][0]
