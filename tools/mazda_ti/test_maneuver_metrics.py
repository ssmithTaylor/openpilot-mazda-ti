"""Synthetic-row tests for the pure maneuver metric library."""
import math

import pytest

from tools.mazda_ti import maneuver_metrics as mm

NS = 1_000_000_000
ALL = ('lane', 'speed', 'steering', 'yaw', 'accel', 'roll', 'controls', 'command')


def make_rows(times_s, offset, *, curvature=0.004, speed=24.0, width=3.6, yaw=None, command=500, unhealthy=None):
  """Right-positive rows; offset/curvature/speed/yaw/command may be constants or callables of time."""
  rows = []
  for t in times_s:
    e = offset(t) if callable(offset) else offset
    k = curvature(t) if callable(curvature) else curvature
    v = speed(t) if callable(speed) else speed
    y = (yaw(t) if callable(yaw) else (k * v * v if yaw is None else yaw))
    c = command(t) if callable(command) else command
    health = dict.fromkeys(ALL, True)
    for group in (unhealthy or {}).get(round(t, 6), ()):
      health[group] = False
    rows.append({
      'mono_ns': int(round(t * NS)), 'camera_eof_ns': int(round(t * NS)) - 38_000_000, 'camera_age_s': 0.038,
      'frame_id': int(t * 20), 'health': health,
      'health_reason': {g: 'valid' if ok else 'missing' for g, ok in health.items()},
      'lane_change_state': 'off', 'lane_fit_reason': 'valid', 'lane_probability_min': 0.95,
      'lane_offset_m': e if health['lane'] else None,
      'lane_heading10_rad': 0.0 if health['lane'] else None, 'lane_heading20_rad': 0.0 if health['lane'] else None,
      'road_curvature10_per_m': k if health['lane'] else None, 'road_curvature20_per_m': k if health['lane'] else None,
      'lane_width_m': width if health['lane'] else None,
      'speed_mps': v if health['speed'] else None,
      'steering_angle_deg': 17.0 if health['steering'] else None, 'steering_rate_dps': 0.0 if health['steering'] else None,
      'yaw_rate_rps': (y / v) if health['yaw'] else None, 'yaw_std_rps': 0.001 if health['yaw'] else None,
      'yaw_lateral_accel_mps2': y if (health['yaw'] and health['speed']) else None,
      'device_lateral_accel_mps2': y if health['accel'] else None,
      'roll_rad': 0.03 if health['roll'] else None, 'controls_active': True if health['controls'] else None,
      'request_mps2': 2.2 if health['controls'] else None, 'actual_mps2': 2.1 if health['controls'] else None,
      'diag_version': 0 if health['controls'] else None,
      'ti_command_counts': c if health['command'] else None,
      'ages_ns': {'carState': 500_000, 'liveLocationKalman': 40_000_000, 'liveParameters': 40_000_000,
                  'controlsState': 5_000_000, 'ti_send': 1_000_000},
    })
  return rows


def grid(start, stop, hz=20.0):
  n = int(round((stop - start) * hz))
  return [start + i / hz for i in range(n + 1)]


def bend(t, *, entry=2.0, exit_=8.0, k=0.004):
  """Road curvature for a bend: zero before entry, k during, zero after exit."""
  return k if entry <= t <= exit_ else 0.0


def strip(rows, groups):
  """Return rows with the given groups marked unhealthy and their fields nulled."""
  out = []
  for r in rows:
    r = {**r, 'health': dict(r['health']), 'health_reason': dict(r['health_reason'])}
    for g in groups:
      r['health'][g] = False
      r['health_reason'][g] = 'missing'
    if 'command' in groups:
      r['ti_command_counts'] = None
    if 'controls' in groups:
      r['controls_active'] = r['request_mps2'] = r['actual_mps2'] = r['diag_version'] = None
    if 'roll' in groups:
      r['roll_rad'] = None
    if 'accel' in groups:
      r['device_lateral_accel_mps2'] = None
    out.append(r)
  return out


def test_parameters_are_frozen_and_versioned():
  p = mm.DEFAULT_PARAMETERS
  assert p.params_id == 'observed-drive-v1-draft'
  assert (p.max_gap_s, p.smooth_span_s, p.velocity_span_s) == (0.2, 0.25, 0.5)
  assert p.episode_quiet_s == 2.0 and abs(p.absence_edge_s - 0.75) < 1e-12
  with pytest.raises(Exception):
    p.max_gap_s = 1.0
  assert mm.Parameters(**{**p.as_dict(), 'params_id': 'x', 'e_settle_m': 0.3}).e_settle_m == 0.3
  with pytest.raises(ValueError):
    mm.Parameters(params_id='', max_gap_s=0.2)
  with pytest.raises(ValueError):
    mm.Parameters(params_id='x', max_gap_s=float('nan'))


def test_healthy_rows_filters_only_declared_groups():
  rows = make_rows(grid(0, 1), 0.0, unhealthy={0.5: ('command',), 0.7: ('lane',)})
  lane_rows = mm.healthy_rows(rows, ('lane',))
  assert len(lane_rows) == len(rows) - 1
  assert all(r['mono_ns'] != int(0.7 * NS) for r in lane_rows)
  assert len(mm.healthy_rows(rows, ('lane', 'command'))) == len(rows) - 2
  with pytest.raises(ValueError):
    mm.healthy_rows(rows, ('bogus',))


def test_segments_split_on_gap_over_limit():
  times = [t for t in grid(0, 2) if not 0.9 < t < 1.3]
  parts = mm.segments(make_rows(times, 0.0), int(0.2 * NS))
  assert [len(p) for p in parts] == [19, 15]
  assert len(mm.segments(make_rows(grid(0, 1), 0.0), int(0.2 * NS))) == 1


def test_smoothing_exists_at_every_interior_row_on_gap_free_20hz():
  rows = make_rows(grid(0, 2), lambda t: 0.5 * t)
  s = mm.smoothed(rows, 'lane_offset_m', mm.DEFAULT_PARAMETERS)
  assert s[0] is None and s[-1] is None
  assert all(v is not None for v in s[3:-3])
  for row, v in zip(rows[3:-3], s[3:-3], strict=True):
    assert abs(v - 0.5 * row['mono_ns'] / NS) < 0.03


def test_smoothing_is_null_when_support_contains_a_gap():
  times = [t for t in grid(0, 2) if not 0.95 < t < 1.25]
  rows = make_rows(times, 0.0)
  s = mm.smoothed(rows, 'lane_offset_m', mm.DEFAULT_PARAMETERS)
  by_t = {round(r['mono_ns'] / NS, 3): v for r, v in zip(rows, s, strict=True)}
  assert by_t[0.95] is None and by_t[1.25] is None
  assert by_t[0.5] is not None and by_t[1.6] is not None


def test_irregular_timestamps_match_regular_within_tolerance():
  f = lambda t: math.sin(2 * math.pi * t / 3)
  regular = make_rows(grid(0, 3), f)
  irregular = make_rows([t + (0.004 if i % 3 else -0.004) for i, t in enumerate(grid(0, 3))], f)
  a = mm.derivative(regular, 'lane_offset_m', mm.DEFAULT_PARAMETERS)
  b = mm.derivative(irregular, 'lane_offset_m', mm.DEFAULT_PARAMETERS)
  pairs = [(x, y) for x, y in zip(a, b, strict=True) if x is not None and y is not None]
  assert len(pairs) > 40
  assert max(abs(x - y) for x, y in pairs) < 0.1


def test_elapsed_weights_cover_window_and_exclude_gaps_and_never_extend_past_last_sample():
  times = [t for t in grid(0, 2) if not 0.9 < t < 1.3]
  w = mm.elapsed_weights(make_rows(times, 0.0), 0, 2 * NS, int(0.2 * NS))
  assert abs(sum(w) - (2.0 - 0.4)) < 1e-6
  short = mm.elapsed_weights(make_rows(grid(0, 1), 0.0), 0, 10 * NS, int(0.2 * NS))
  assert abs(sum(short) - 1.0) < 1e-6 and short[-1] == 0.0
  cov = mm.coverage(make_rows(grid(0, 1), 0.0), 0, 10 * NS, int(0.2 * NS))
  assert abs(cov['valid_duration_s'] - 1.0) < 1e-6 and abs(cov['trailing_unobserved_s'] - 9.0) < 1e-6
  assert cov['leading_unobserved_s'] == 0.0
  assert abs(mm.weighted_rms([1.0, 3.0], [1.0, 1.0]) - math.sqrt(5.0)) < 1e-12
  assert mm.weighted_percentile([1.0, 2.0, 3.0, 4.0], [1.0, 1.0, 1.0, 1.0], 0.95) == 4.0


def _sine_rows(cycles=2, period=3.0, amp=0.4, start=0.0):
  return make_rows(grid(start, start + cycles * period), lambda t: amp * math.sin(2 * math.pi * (t - start) / period))


def test_extrema_alternate_with_prominence():
  ex = mm.extrema(_sine_rows(), 'lane_offset_m', mm.DEFAULT_PARAMETERS)
  kinds = [e['kind'] for e in ex]
  assert kinds[:4] == ['max', 'min', 'max', 'min']
  assert all(a != b for a, b in zip(kinds, kinds[1:]))
  assert all(e['prominence_m'] > 0.7 for e in ex[1:])


def test_plateau_between_opposite_signs_is_one_extremum_and_same_sign_is_none():
  def hill(t):  # rise, flat top from 2 to 3 s, fall
    return min(t, 2.0) if t < 3 else max(0.0, 5.0 - t)
  rows = make_rows(grid(0, 5), hill)
  ex = [e for e in mm.extrema(rows, 'lane_offset_m', mm.DEFAULT_PARAMETERS) if e['kind'] == 'max']
  assert len(ex) == 1 and ex[0]['plateau'] is True
  assert abs(ex[0]['mono_ns'] / NS - 2.5) < 0.2

  def shoulder(t):  # rise, flat from 2 to 3 s, rise again: no extremum
    return t if t < 2 else 2.0 if t < 3 else t - 1
  ex = mm.extrema(make_rows(grid(0, 5), shoulder), 'lane_offset_m', mm.DEFAULT_PARAMETERS)
  assert [e for e in ex if e['plateau']] == []


def test_peak_trough_peak_is_one_cycle_and_two_extrema_are_truncated():
  params = mm.Parameters(**{**mm.DEFAULT_PARAMETERS.as_dict(), 'a_min_m': 0.1})
  ex = mm.extrema(make_rows(grid(0, 3.0), lambda t: 0.4 * math.sin(2 * math.pi * t / 3.0)), 'lane_offset_m', params)
  # 3 s of a 3 s period: max at 0.75, min at 2.25 -> two extrema only
  out = mm.cycle_episodes(ex, params)
  assert out['cycle_count'] == 0 and out['truncated_cycles'] == 1
  full = mm.cycle_episodes(mm.extrema(_sine_rows(cycles=2), 'lane_offset_m', params), params)
  assert full['cycle_count'] >= 1 and full['episode_count'] == 1


def test_two_separate_episodes_sum_and_longest_is_reported():
  params = mm.Parameters(**{**mm.DEFAULT_PARAMETERS.as_dict(), 'a_min_m': 0.1})
  def two(t):
    if 0 <= t <= 4.5:
      return 0.4 * math.sin(2 * math.pi * t / 3.0)
    if 10 <= t <= 14.5:
      return 0.4 * math.sin(2 * math.pi * (t - 10) / 3.0)
    return 0.0
  ex = mm.extrema(make_rows(grid(0, 16), two), 'lane_offset_m', params)
  out = mm.cycle_episodes(ex, params)
  # the 4.5-10 s flat stretch is a plateau longer than episode_quiet_s: a terminator, not a connecting trough
  assert out['episode_count'] == 2 and out['cycle_count'] == 2 and out['longest_episode_cycles'] == 1


def test_slow_continuous_oscillation_still_counts():
  params = mm.Parameters(**{**mm.DEFAULT_PARAMETERS.as_dict(), 'a_min_m': 0.1})
  ex = mm.extrema(make_rows(grid(0, 20), lambda t: 0.4 * math.sin(2 * math.pi * t / 5.0)), 'lane_offset_m', params)
  out = mm.cycle_episodes(ex, params)
  assert out['episode_count'] == 1 and out['cycle_count'] >= 3


def test_cycles_never_chain_across_an_observation_gap():
  params = mm.Parameters(**{**mm.DEFAULT_PARAMETERS.as_dict(), 'a_min_m': 0.1})
  # a peak, then a 2.5 s gap, then a trough and a peak: two fragments that combined would look like one cycle.
  # Each fragment must carry the filters' support (smooth 0.125 s + velocity 0.25 s past the extremum).
  f = lambda t: 0.4 * math.sin(2 * math.pi * t / 3.0)
  times = [t for t in grid(0, 7.5) if not 2.0 < t < 4.5]
  ex = mm.extrema(make_rows(times, f), 'lane_offset_m', params)
  assert {e['segment'] for e in ex} == {0, 1}
  assert mm.cycle_episodes(ex, params)['cycle_count'] == 0


def test_small_wiggles_below_prominence_do_not_count():
  params = mm.Parameters(**{**mm.DEFAULT_PARAMETERS.as_dict(), 'a_min_m': 0.1})
  ex = mm.extrema(make_rows(grid(0, 6), lambda t: 0.02 * math.sin(2 * math.pi * t)), 'lane_offset_m', params)
  assert mm.cycle_episodes(ex, params)['cycle_count'] == 0
