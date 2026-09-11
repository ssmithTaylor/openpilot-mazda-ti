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


def _sign(v, deadband):
  if v is None:
    return None
  return 1 if v > deadband else -1 if v < -deadband else 0


def extrema(rows, key, params):
  """Alternating extrema of the smoothed series per observation segment.

  Sign changes of the derivative mark extrema. A flat run (|d| <= plateau deadband)
  counts once, at its midpoint, only when the derivative signs before and after it
  are opposite; a run between same-sign derivatives is a shoulder. Prominence is the
  absolute change from the previous opposite extremum (first: from the first valid
  smoothed value).
  """
  out = []
  for segment_index, part in enumerate(segments(rows, params.max_gap_ns)):
    s = smoothed(part, key, params)
    d = derivative(part, key, params)
    times = [_t(r) for r in part]
    signs = [_sign(v, params.plateau_deadband_mps) for v in d]
    first_valid = next((v for v in s if v is not None), None)
    previous = None
    last_nonzero, last_nonzero_i = None, None
    flat_start = None
    for i, sg in enumerate(signs):
      if sg is None or s[i] is None:
        flat_start = None
        continue
      if sg == 0:
        if flat_start is None:
          flat_start = i
        continue
      if last_nonzero is not None and sg != last_nonzero:
        if flat_start is not None:
          j = (flat_start + i - 1) // 2
          plateau, plateau_s = True, (times[i - 1] - times[flat_start]) / NS
        else:
          j = i - 1 if s[i - 1] is not None else i
          plateau, plateau_s = False, 0.0
        kind = 'max' if last_nonzero > 0 else 'min'
        ref = previous['value'] if previous else first_valid
        item = {'mono_ns': times[j], 'value': s[j], 'kind': kind, 'segment': segment_index,
                'prominence_m': abs(s[j] - ref) if ref is not None else None, 'plateau': plateau,
                'plateau_duration_s': plateau_s}
        out.append(item)
        previous = item
      flat_start = None
      last_nonzero, last_nonzero_i = sg, i
  return out


def cycle_episodes(extrema_list, params):
  """Group qualifying alternating extrema into episodes and count cycles.

  Qualifying: prominence >= a_min_m and spacing from the previous qualifying extremum
  >= t_min_s. A chain terminates at an observation-segment change or at a quiet
  interval, detected as a plateau (|de/dt| within the deadband) lasting longer than
  episode_quiet_s; that plateau extremum is a terminator and joins no chain. Slow but
  continuous oscillations therefore still count. A maximal alternating chain with >= 3
  members is an episode with floor((n - 1) / 2) cycles; a chain of exactly 2 is a
  truncated cycle. cycle_count sums episodes. Requires a calibrated a_min_m.
  """
  if params.a_min_m is None:
    raise ValueError('a_min_m is not calibrated')
  chains, chain = [], []
  for e in extrema_list:
    quiet = e['plateau'] and e['plateau_duration_s'] > params.episode_quiet_s
    if quiet:
      if chain:
        chains.append(chain)
      chain = []
      continue
    ok = (e['prominence_m'] is not None and e['prominence_m'] >= params.a_min_m
          and (not chain or e['segment'] == chain[-1]['segment'])
          and (not chain or (e['mono_ns'] - chain[-1]['mono_ns']) / NS >= params.t_min_s)
          and (not chain or e['kind'] != chain[-1]['kind']))
    if ok:
      chain.append(e)
    else:
      if chain:
        chains.append(chain)
      chain = [e] if (e['prominence_m'] is not None and e['prominence_m'] >= params.a_min_m) else []
  if chain:
    chains.append(chain)
  episodes, truncated = [], 0
  for c in chains:
    if len(c) == 2:
      truncated += 1
    if len(c) < 3:
      continue
    swings = [abs(b['value'] - a['value']) for a, b in zip(c, c[1:])]
    ratios = [b / a for a, b in zip(swings, swings[1:]) if a > 0]
    episodes.append({'start_ns': c[0]['mono_ns'], 'end_ns': c[-1]['mono_ns'], 'cycles': (len(c) - 1) // 2,
                     'peak_to_peak_m': max(swings), 'amplitude_ratio': ratios, 'extrema': c})
  return {'cycle_count': sum(e['cycles'] for e in episodes), 'episode_count': len(episodes),
          'longest_episode_cycles': max((e['cycles'] for e in episodes), default=0),
          'truncated_cycles': truncated, 'episodes': episodes}


SIGNED_FIELDS = ('lane_offset_m', 'lane_heading10_rad', 'lane_heading20_rad', 'road_curvature10_per_m',
                 'road_curvature20_per_m', 'steering_angle_deg', 'steering_rate_dps', 'yaw_rate_rps',
                 'yaw_lateral_accel_mps2', 'device_lateral_accel_mps2', 'roll_rad', 'request_mps2',
                 'actual_mps2', 'ti_command_counts')
MIN_BEND_CURVATURE = 1e-4


def _in_window(rows, window):
  return [r for r in rows if window[0] <= _t(r) < window[1]]


def orient(rows, window):
  """Direction from the elapsed-time integral of road curvature over lane-healthy rows; never from commands.

  A bend need not fill the window: the sign of the curvature integral identifies the
  sustained bend, and its magnitude must exceed MIN_BEND_CURVATURE times the observed
  duration (a mean curvature of at least 1e-4 per metre).
  """
  lane = healthy_rows(rows, ('lane',))
  w = elapsed_weights(lane, window[0], window[1], int(0.2 * NS))
  observed = sum(w)
  if observed <= 0:
    return {'direction': None, 'mean_curvature_per_m': None, 'status': 'unscorable', 'reason': 'no_lane_rows'}
  integral = sum(wi * float(r['road_curvature10_per_m']) for r, wi in zip(lane, w, strict=True))
  mean = integral / observed
  if abs(mean) < MIN_BEND_CURVATURE:
    return {'direction': None, 'mean_curvature_per_m': mean, 'status': 'unscorable', 'reason': 'no_sustained_bend'}
  return {'direction': 1 if mean > 0 else -1, 'mean_curvature_per_m': mean, 'status': 'measured_estimate', 'reason': 'valid'}


def oriented(rows, direction):
  """Copy rows with every signed field multiplied by direction (positive = inside of the bend)."""
  out = []
  for r in rows:
    c = dict(r)
    for key in SIGNED_FIELDS:
      if c.get(key) is not None:
        c[key] = direction * c[key]
    out.append(c)
  return out


def _first_sustained(times, series, start_i, predicate, persist_ns, max_gap_ns):
  """First index i >= start_i where predicate holds on valid values, without a gap over max_gap_ns, for persist_ns."""
  i = start_i
  while i < len(times):
    if series[i] is None or not predicate(series[i]):
      i += 1
      continue
    j = i
    while (j + 1 < len(times) and series[j + 1] is not None and predicate(series[j + 1])
           and times[j + 1] - times[j] <= max_gap_ns):
      j += 1
    if times[j] - times[i] >= persist_ns:
      return i
    i = j + 1
  return None


def phase_anchors(rows, window, params):
  """Entry from curvature*speed^2, then speed frozen at entry so slowing cannot create an unwind."""
  lane = _in_window(healthy_rows(rows, ('lane', 'speed')), window)
  base = {'anchor_source': 'model_road_geometry', 'v_entry_mps': None, 'entry_ns': None, 'peak_ns': None,
          'unwind_ns': None, 'recovery': None, 'absent_phases': [], 'critical_gap': False, 'coverage': {},
          'instantaneous_exit_crossing_ns': None, 'status': 'unscorable', 'reason': 'no_lane_rows',
          'limits': ['Model-derived road geometry, not surveyed truth; rider/geographic windows only bound the search']}
  if not lane:
    return base
  times = [_t(r) for r in lane]
  persist = int(round(params.anchor_persistence_s * NS))
  a_live = smoothed([{**r, '_a': r['road_curvature10_per_m'] * r['speed_mps'] ** 2} for r in lane], '_a', params)
  entry_i = _first_sustained(times, a_live, 0, lambda v: v >= params.entry_accel_mps2, persist, params.max_gap_ns)
  if entry_i is None:
    return {**base, 'absent_phases': ['entry', 'sustained', 'unwind', 'recovery'], 'reason': 'no_entry'}
  v_entry = float(lane[entry_i]['speed_mps'])
  a_road = smoothed([{**r, '_a': r['road_curvature10_per_m'] * v_entry ** 2} for r in lane], '_a', params)
  peak_i = max((i for i in range(entry_i, len(times)) if a_road[i] is not None), key=lambda i: a_road[i])
  unwind_i = _first_sustained(times, a_road, peak_i, lambda v: v <= params.exit_accel_mps2, persist, params.max_gap_ns)
  live_exit = _first_sustained(times, a_live, peak_i, lambda v: v <= params.exit_accel_mps2, persist, params.max_gap_ns)
  gaps = [(times[k], times[k + 1]) for k in range(len(times) - 1) if times[k + 1] - times[k] > params.max_gap_ns]
  def touches(t):
    return t is not None and any(lo <= t + persist and t - persist <= hi for lo, hi in gaps)
  result = {**base, 'v_entry_mps': v_entry, 'entry_ns': times[entry_i], 'peak_ns': times[peak_i],
            'instantaneous_exit_crossing_ns': times[live_exit] if live_exit is not None else None,
            'status': 'measured_estimate', 'reason': 'valid'}
  if unwind_i is None:
    result['absent_phases'] = ['unwind', 'recovery']
    result['critical_gap'] = touches(times[entry_i])
    result['coverage'] = {'entry_to_end_s': (times[-1] - times[entry_i]) / NS}
    return result
  unwind = times[unwind_i]
  result['unwind_ns'] = unwind
  result['recovery'] = {'start_ns': unwind, 'end_ns': min(unwind + int(round(params.recovery_horizon_s * NS)), window[1])}
  result['critical_gap'] = touches(times[entry_i]) or touches(unwind)
  result['coverage'] = {'entry_to_unwind_s': (unwind - times[entry_i]) / NS,
                        'recovery_observed_s': (min(times[-1], result['recovery']['end_ns']) - unwind) / NS}
  return result
