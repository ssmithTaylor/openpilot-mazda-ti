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
  required = (end_ns - start_ns) / NS
  return {'valid_duration_s': min(sum(weights), required), 'required_duration_s': required,
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


def record(metric, *, unit, status='unscorable', reason='not_computed', value=None, lower_bound_s=None,
           valid_duration_s=None, required_duration_s=None, maximum_gap_s=None, critical_gap=None,
           phase_coverage=None, supporting_event_ids=None, diagnostics=None):
  if status not in STATUSES:
    raise ValueError(f'Unknown status {status}')
  return {'metric': metric, 'version': 1, 'value': value, 'unit': unit, 'status': status, 'reason': reason,
          'lower_bound_s': lower_bound_s, 'valid_duration_s': valid_duration_s,
          'required_duration_s': required_duration_s, 'maximum_gap_s': maximum_gap_s,
          'critical_gap': critical_gap, 'phase_coverage': phase_coverage or {},
          'uncertainty_method': 'not_established_v1',
          'supporting_event_ids': supporting_event_ids or [], 'diagnostics': diagnostics or {}}


def _coverage(rows, start_ns, end_ns, max_gap_ns):
  """Record fields from coverage(); rows must include margin rows so trailing support is real."""
  c = coverage(rows, start_ns, end_ns, max_gap_ns)
  return {'valid_duration_s': c['valid_duration_s'], 'required_duration_s': c['required_duration_s'],
          'maximum_gap_s': c['maximum_gap_s']}


def _full_coverage(rows, start_ns, end_ns, params):
  return coverage(rows, start_ns, end_ns, params.max_gap_ns)


def _calibrated(params, *names):
  missing = [n for n in names if getattr(params, n) is None]
  return missing


def settling(rows, window, anchors, params, interventions=()):
  """Elapsed time from the unwind anchor to the start of the first confirmed dwell."""
  missing = _calibrated(params, 'e_settle_m', 'v_settle_mps')
  if missing:
    return record('settling_time', unit='s', reason=f'uncalibrated:{",".join(missing)}')
  if anchors.get('unwind_ns') is None:
    return record('settling_time', unit='s', reason='no_unwind_anchor')
  if anchors.get('critical_gap'):
    return record('settling_time', unit='s', reason='critical_gap', critical_gap=True)
  unwind, rec_end = anchors['unwind_ns'], anchors['recovery']['end_ns']
  for iv in interventions:
    # Overlap test: an intervention that starts before unwind and has not ended (unknown end
    # is treated as continuing) still assists the recovery.
    if iv['kind'] == 'intervention' and iv['start_ns'] < rec_end and (iv.get('end_ns') is None or iv['end_ns'] > unwind):
      return record('settling_time', unit='s', reason='assisted_recovery',
                    diagnostics={'intervention_start_s': (iv['start_ns'] - unwind) / NS,
                                 'intervention_end_known': iv.get('end_ns') is not None})
  # Filters use every lane-healthy row, including margin rows beyond recovery, so support is
  # not discarded at the recovery end; only dwell scoring is restricted to [unwind, rec_end).
  lane = healthy_rows(rows, ('lane',))
  times = [_t(r) for r in lane]
  e = smoothed(lane, 'lane_offset_m', params)
  v = derivative(lane, 'lane_offset_m', params)
  cov = _coverage(lane, unwind, rec_end, params.max_gap_ns)
  dwell_ns = int(round(params.t_dwell_s * NS))
  start_i, end_i = bisect_left(times, unwind), bisect_left(times, rec_end)
  last_violation = unwind
  first_gap_i = next((k for k in range(start_i, end_i - 1) if times[k + 1] - times[k] > params.max_gap_ns), None)
  gap_end_i = first_gap_i + 1 if first_gap_i is not None else end_i

  def scan(lo, hi):
    """Return (dwell_start_i or None, last_violation_ns, candidate_start_i or None) over [lo, hi)."""
    last_v, cand = None, None
    for i in range(lo, hi):
      if i > lo and times[i] - times[i - 1] > params.max_gap_ns:
        # A real observation gap breaks continuity of observation: it is not itself a
        # violation (last_v is untouched), but it invalidates any candidate dwell in
        # progress, which cannot be confirmed across missing data.
        cand = None
      if e[i] is None or v[i] is None:
        # Missing filter support with no timestamp gap (edge of data) neither starts nor
        # invalidates a candidate dwell already in progress; it just can't be confirmed here.
        continue
      ok = abs(e[i]) <= params.e_settle_m and abs(v[i]) <= params.v_settle_mps
      if not ok:
        last_v, cand = times[i], None
        continue
      if cand is None:
        cand = i
      if times[i] - times[cand] >= dwell_ns:
        return cand, last_v, cand
    return None, last_v, cand

  dwell_i, last_v, cand = scan(start_i, gap_end_i)
  last_violation = last_v if last_v is not None else unwind
  diag = {'residual_offset_m': None, 'last_overshoot_m': None, 'candidate_dwell_start_s': None, 'later_dwell_observation': None}
  base = dict(valid_duration_s=cov['valid_duration_s'], required_duration_s=cov['required_duration_s'],
              maximum_gap_s=cov['maximum_gap_s'], critical_gap=False,
              phase_coverage={'recovery_observed_s': anchors['coverage'].get('recovery_observed_s')})
  if dwell_i is not None:
    tail = [abs(e[i]) for i in range(dwell_i, end_i) if e[i] is not None]
    peak = max((abs(e[i]) for i in range(start_i, dwell_i) if e[i] is not None), default=None)
    diag.update(residual_offset_m=tail[-1] if tail else None, last_overshoot_m=peak)
    return record('settling_time', unit='s', status='measured_estimate', reason='first_confirmed_dwell',
                  value=(times[dwell_i] - unwind) / NS, diagnostics=diag, **base)
  bound = (last_violation - unwind) / NS
  if cand is not None:
    diag['candidate_dwell_start_s'] = (times[cand] - unwind) / NS
  if first_gap_i is not None:
    later_i, _, _ = scan(gap_end_i, end_i)
    if later_i is not None:
      # Absolute row time, not elapsed-since-unwind: this observation sits after a gap the
      # first scan could not cross, so it is reported on the clip's own timeline.
      diag['later_dwell_observation'] = {'start_s': times[later_i] / NS,
                                         'residual_offset_m': abs(e[later_i])}
    return record('settling_time', unit='s', status='unresolved', reason='gap_before_first_dwell',
                  lower_bound_s=bound, diagnostics=diag, **base)
  return record('settling_time', unit='s', status='right_censored', reason='recovery_ended_before_dwell',
                lower_bound_s=bound, diagnostics=diag, **base)


def cycles(rows, window, params):
  """Alternating lane-motion cycles over the window; command reversals are a separate diagnostic."""
  if params.a_min_m is None:
    return record('lane_motion_cycles', unit='cycles', reason='uncalibrated:a_min_m')
  lane_all = healthy_rows(rows, ('lane',))
  lane = _in_window(lane_all, window)
  if not lane:
    return record('lane_motion_cycles', unit='cycles', reason='no_lane_rows')
  ex = [e for e in extrema(lane_all, 'lane_offset_m', params) if window[0] <= e['mono_ns'] < window[1]]
  ep = cycle_episodes(ex, params)
  full = _full_coverage(lane_all, window[0], window[1], params)
  cov = {k: full[k] for k in ('valid_duration_s', 'required_duration_s', 'maximum_gap_s')}
  cmd = _in_window(healthy_rows(rows, ('command',)), window)
  signs = [1 if r['ti_command_counts'] > 0 else -1 if r['ti_command_counts'] < 0 else 0 for r in cmd]
  reversals = sum(1 for a, b in zip(signs, signs[1:]) if a and b and a != b) if cmd else None
  covered = absence_supported(full, params)
  diagnostics = {**ep, 'observed_cycle_count': ep['cycle_count'], 'command_sign_reversals': reversals,
                 'peak_to_peak_max_m': max((e['peak_to_peak_m'] for e in ep['episodes']), default=None),
                 'leading_unobserved_s': full['leading_unobserved_s'], 'trailing_unobserved_s': full['trailing_unobserved_s'],
                 'absence_supported': covered}
  if ep['cycle_count'] > 0 or covered:
    # Observed cycles are evidence regardless of coverage; absence needs the whole interval observed.
    return record('lane_motion_cycles', unit='cycles', status='measured_estimate', reason='counted' if covered else 'counted_with_partial_coverage',
                  value=ep['cycle_count'], critical_gap=not covered, **cov, diagnostics=diagnostics)
  return record('lane_motion_cycles', unit='cycles', status='unresolved', reason='interval_not_fully_observed',
                critical_gap=True, **cov, diagnostics=diagnostics)


def hold_late_wide(rows, window, anchors, params, direction):
  """Inward peak/dwell, outward peak, time to recovery and footprint-free boundary proxies."""
  if params.corridor_half_width_m is None:
    return record('late_wide_excursion', unit='m', reason='uncalibrated:corridor_half_width_m')
  if anchors.get('entry_ns') is None:
    return record('late_wide_excursion', unit='m', reason='no_entry_anchor')
  end = anchors['recovery']['end_ns'] if anchors.get('recovery') else window[1]
  lane = healthy_rows(rows, ('lane',))
  if not [r for r in lane if anchors['entry_ns'] <= _t(r) < end]:
    return record('late_wide_excursion', unit='m', reason='no_lane_rows')
  times = [_t(r) for r in lane]
  w = elapsed_weights(lane, anchors['entry_ns'], end, params.max_gap_ns)
  e = [x if wi > 0 or (anchors['entry_ns'] <= t < end) else None
       for x, wi, t in zip(smoothed(lane, 'lane_offset_m', params), w, times, strict=True)]
  valid = [(i, x) for i, x in enumerate(e) if x is not None]
  if not valid:
    return record('late_wide_excursion', unit='m', reason='no_supported_smoothed_values')
  inward_i, inward = max(valid, key=lambda p: p[1])
  after = [(i, x) for i, x in valid if i > inward_i]
  outward_i, outward = min(after, key=lambda p: p[1]) if after else (None, None)
  dwell = sum(w[i] for i, x in valid if x > params.corridor_half_width_m)
  ttr, unwind = None, anchors.get('unwind_ns')
  if unwind is not None:
    dwell_ns = int(round(params.t_dwell_s * NS))
    run, prev_i = None, None
    for i, x in enumerate(e):
      if times[i] < unwind:
        continue
      unsupported = x is None or (prev_i is not None and times[i] - times[prev_i] > params.max_gap_ns)
      prev_i = i
      if unsupported or abs(x) > params.corridor_half_width_m:
        run = None
        continue
      run = i if run is None else run
      if times[i] - times[run] >= dwell_ns:
        ttr = (times[run] - unwind) / NS
        break
  e_right = [direction * x if x is not None else None for x in e]
  left = [r['lane_width_m'] / 2 + x for r, x in zip(lane, e_right, strict=True) if x is not None]
  right = [r['lane_width_m'] / 2 - x for r, x in zip(lane, e_right, strict=True) if x is not None]
  inside = 'right' if direction > 0 else 'left'
  proxies = ({'min_left_m': min(left), 'min_right_m': min(right), 'inside_side': inside,
              'min_inside_m': min(right) if inside == 'right' else min(left),
              'min_outside_m': min(left) if inside == 'right' else min(right),
              'basis': 'reference point to model lane line; no vehicle footprint'} if left else
             {'min_left_m': None, 'min_right_m': None, 'inside_side': inside, 'min_inside_m': None, 'min_outside_m': None,
              'basis': 'no supported smoothed values'})
  full = _full_coverage(lane, anchors['entry_ns'], end, params)
  cov = {k: full[k] for k in ('valid_duration_s', 'required_duration_s', 'maximum_gap_s')}
  covered = absence_supported(full, params)
  diagnostics = {'inward_peak_m': inward, 'inward_peak_ns': times[inward_i], 'inward_dwell_s': dwell,
                 'outward_peak_m': outward, 'outward_peak_ns': times[outward_i] if outward_i is not None else None,
                 'time_to_recovery_s': ttr, 'boundary_distance_proxy_m': proxies, 'coverage_adequate': covered,
                 'leading_unobserved_s': full['leading_unobserved_s'], 'trailing_unobserved_s': full['trailing_unobserved_s']}
  value = abs(min(outward, 0.0)) if outward is not None else None
  if not covered:
    # Observed excursion/dwell remain diagnostics; absence of the symptom cannot be asserted.
    return record('late_wide_excursion', unit='m', status='unresolved', reason='interval_not_fully_observed',
                  value=value, critical_gap=True, **cov, diagnostics=diagnostics)
  status = 'measured_estimate' if ttr is not None or unwind is None else 'right_censored'
  return record('late_wide_excursion', unit='m', status=status, reason='outward_peak_after_inward_peak' if outward is not None else 'no_outward_motion',
                value=value, critical_gap=anchors.get('critical_gap'), **cov, diagnostics=diagnostics)


def path_quality(rows, window, anchors, params):
  """Elapsed-time RMS and p95 of |e| and time outside the corridor; no re-centering."""
  lane = healthy_rows(rows, ('lane',))
  if not _in_window(lane, window):
    return record('path_rms', unit='m', reason='no_lane_rows')
  w = elapsed_weights(lane, window[0], window[1], params.max_gap_ns)
  e = [abs(r['lane_offset_m']) for r in lane]
  cov = _coverage(lane, window[0], window[1], params.max_gap_ns)
  half = params.corridor_half_width_m
  outside = sum(wi for wi, x in zip(w, e, strict=True) if half is not None and x > half)
  rec = None
  if anchors.get('recovery'):
    r0, r1 = anchors['recovery']['start_ns'], anchors['recovery']['end_ns']
    ws = elapsed_weights(lane, r0, r1, params.max_gap_ns)
    if sum(ws) > 0:
      rec = {'rms_m': weighted_rms(e, ws), 'p95_m': weighted_percentile(e, ws, 0.95)}
  return record('path_rms', unit='m', status='measured_estimate', reason='elapsed_time_weighted',
                value=weighted_rms(e, w), critical_gap=False, **cov,
                diagnostics={'p95_m': weighted_percentile(e, w, 0.95), 'time_outside_corridor_s': outside if half is not None else None,
                             'recovery': rec, 'target': 'lane centre, no re-centering'})


def comfort_proxy(rows, window, params):
  """Lateral jerk proxy from d/dt(speed * yaw rate); a kinematic proxy, not occupant acceleration."""
  yaw = healthy_rows(rows, ('yaw', 'speed'))
  if len(_in_window(yaw, window)) < 3:
    return record('lateral_jerk_p95', unit='m/s^3', reason='no_yaw_rows')
  j = derivative(yaw, 'yaw_lateral_accel_mps2', params, span_s=params.jerk_smooth_span_s)
  w = elapsed_weights(yaw, window[0], window[1], params.max_gap_ns)
  # margin rows supply filter support only; peak and p95 use supported in-window values (positive weight)
  pairs = [(abs(x), wi) for x, wi in zip(j, w, strict=True) if x is not None and wi > 0]
  if not pairs:
    return record('lateral_jerk_p95', unit='m/s^3', reason='no_supported_derivative')
  vals, ws = [p[0] for p in pairs], [p[1] for p in pairs]
  above = sum(wi for x, wi in pairs if params.j_limit_mps3 is not None and x > params.j_limit_mps3)
  cov = _coverage(yaw, window[0], window[1], params.max_gap_ns)
  return record('lateral_jerk_p95', unit='m/s^3', status='measured_estimate', reason='kinematic_proxy',
                value=weighted_percentile(vals, ws, 0.95), critical_gap=False, **cov,
                diagnostics={'peak_mps3': max(vals), 'duration_above_limit_s': above if params.j_limit_mps3 is not None else None,
                             'basis': 'kinematic_proxy', 'signal': 'liveLocationKalman calibrated yaw rate times carState speed',
                             'smoothing_s': params.jerk_smooth_span_s, 'derivative_span_s': params.velocity_span_s,
                             'sample_basis': 'model-rate latest-publication join'})


def annotations(case_annotations, window, labels=None):
  """Carry rider-supplied crossings and interventions; never inferred from telemetry.

  A count of zero is a measurement only when the case's labels explicitly declare the
  event absent over a stated scope; an empty annotation list alone is unresolved.
  """
  labels = labels or {}
  out = {}
  for metric, kind, label in (('boundary_crossings', 'crossing', 'crossing'), ('driver_catches', 'intervention', 'intervention')):
    items = [a for a in case_annotations if a['kind'] == kind and window[0] <= a['start_ns'] < window[1]]
    ids = [f"{kind}:{a['start_ns']}" for a in items]
    if items:
      out[metric] = record(metric, unit='events', status='measured_estimate', reason='rider_annotation', value=len(items),
                           supporting_event_ids=ids, diagnostics={'provenance': 'rider_annotation', 'items': items})
    elif labels.get(label) == 'absent' and labels.get('scope'):
      out[metric] = record(metric, unit='events', status='measured_estimate', reason='rider_declared_absent',
                           value=0, supporting_event_ids=[f'{kind}:absent:{labels["scope"]}'],
                           diagnostics={'provenance': 'rider_annotation', 'scope': labels['scope'], 'items': []})
    else:
      out[metric] = record(metric, unit='events', status='unresolved', reason='no_annotation_or_negative_label',
                           diagnostics={'provenance': 'rider_annotation', 'items': []})
  return out


PHYSICAL_MAP = {'settling_time': 'settling_time', 'lane_motion_cycles': 'scallop_peak_to_peak',
                'late_wide_excursion': 'late_wide_excursion', 'lateral_jerk_p95': 'lateral_jerk_p95',
                'driver_catches': 'driver_catches', 'path_rms': 'corridor_violation_duration'}


def to_physical_records(records_by_metric, identity, contact_status):
  """Shape measurement records for physical_evidence.classify_metrics; clearance is never supplied."""
  out = []
  for name, target in PHYSICAL_MAP.items():
    rec = records_by_metric.get(name)
    if rec is None or rec['status'] not in ('measured_estimate',):
      continue
    value = rec['value']
    if name == 'lane_motion_cycles':
      value = rec['diagnostics'].get('peak_to_peak_max_m')
      if value is None:
        continue
    if name == 'path_rms':
      value = rec['diagnostics'].get('time_outside_corridor_s')
      if value is None:
        continue
    item = {'metric': target, 'value': int(value) if target == 'driver_catches' else float(value),
            'unit': {'settling_time': 's', 'scallop_peak_to_peak': 'm', 'late_wide_excursion': 'm',
                     'lateral_jerk_p95': 'm/s^3', 'driver_catches': 'events', 'corridor_violation_duration': 's'}[target],
            'valid_duration_s': rec['valid_duration_s'], 'required_duration_s': rec['required_duration_s'],
            'maximum_gap_s': rec['maximum_gap_s'], 'critical_gap': bool(rec['critical_gap']), 'ambiguous': False,
            'uncertainty_method': rec['uncertainty_method'], 'source_identity': identity,
            'supporting_event_ids': rec['supporting_event_ids'] or [f'{name}:{identity["window_ns"][0]}'],
            'contact_status': contact_status, 'complete_recovery': rec['status'] == 'measured_estimate'}
    if target == 'driver_catches':
      item.update(valid_duration_s=(identity['window_ns'][1] - identity['window_ns'][0]) / NS,
                  required_duration_s=(identity['window_ns'][1] - identity['window_ns'][0]) / NS, maximum_gap_s=0.0)
    out.append(item)
  return out


def physical_verdicts(records_by_metric, identity, contact_status):
  from tools.mazda_ti.physical_evidence import classify_metrics
  return classify_metrics(to_physical_records(records_by_metric, identity, contact_status),
                          evidence_type='recorded_baseline', source_arm='reference', verified_identity=identity)
