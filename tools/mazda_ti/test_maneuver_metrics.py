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


def test_orient_uses_road_curvature_sign_only():
  right = make_rows(grid(0, 20), 0.0, curvature=lambda t: bend(t, k=0.004), command=lambda t: -600)
  left = make_rows(grid(0, 20), 0.0, curvature=lambda t: bend(t, k=-0.004), command=lambda t: 600)
  assert mm.orient(right, (0, 20 * NS))['direction'] == 1   # bend covers 6 of 20 s; still oriented
  assert mm.orient(left, (0, 20 * NS))['direction'] == -1
  assert mm.orient(strip(right, ('command', 'controls')), (0, 20 * NS)) == mm.orient(right, (0, 20 * NS))
  flat = mm.orient(make_rows(grid(0, 10), 0.0, curvature=0.0), (0, 10 * NS))
  assert flat['direction'] is None and flat['reason'] == 'no_sustained_bend'


def test_oriented_flips_every_signed_field_consistently():
  rows = make_rows(grid(0, 1), 0.3, curvature=-0.004, command=-500)
  o = mm.oriented(rows, -1)
  r = o[5]
  assert r['lane_offset_m'] == -0.3 and r['road_curvature10_per_m'] == 0.004 and r['ti_command_counts'] == 500
  assert r['steering_angle_deg'] == -17.0 and r['yaw_lateral_accel_mps2'] > 0
  assert r['lane_width_m'] == 3.6 and r['speed_mps'] == 24.0


def test_anchors_follow_road_curvature_inside_window():
  rows = mm.oriented(make_rows(grid(0, 20), 0.0, curvature=lambda t: bend(t, entry=3, exit_=9)), 1)
  a = mm.phase_anchors(rows, (0, 20 * NS), mm.DEFAULT_PARAMETERS)
  assert a['status'] == 'measured_estimate' and a['anchor_source'] == 'model_road_geometry'
  assert abs(a['entry_ns'] / NS - 3.2) < 0.4      # first sustained instant, shifted by smoothing support
  assert abs(a['unwind_ns'] / NS - 9.2) < 0.4
  assert a['recovery'] == {'start_ns': a['unwind_ns'], 'end_ns': a['unwind_ns'] + 8 * NS}
  assert a['absent_phases'] == [] and a['critical_gap'] is False


def test_anchors_ignore_command_and_controller_streams():
  base = mm.oriented(make_rows(grid(0, 20), 0.0, curvature=lambda t: bend(t, entry=3, exit_=9), command=lambda t: 600 if t < 6 else 0), 1)
  a = mm.phase_anchors(base, (0, 20 * NS), mm.DEFAULT_PARAMETERS)
  b = mm.phase_anchors(strip(base, ('command', 'controls', 'roll', 'accel')), (0, 20 * NS), mm.DEFAULT_PARAMETERS)
  assert a == b


def test_slowing_on_constant_curvature_does_not_create_unwind():
  # curvature constant 0.004 from 3 s onward; speed drops from 24 to 8 m/s after 9 s
  rows = mm.oriented(make_rows(grid(0, 20), 0.0, curvature=lambda t: 0.004 if t >= 3 else 0.0,
                               speed=lambda t: 24.0 if t < 9 else 8.0), 1)
  a = mm.phase_anchors(rows, (0, 20 * NS), mm.DEFAULT_PARAMETERS)
  assert a['unwind_ns'] is None and 'unwind' in a['absent_phases']
  assert a['instantaneous_exit_crossing_ns'] is not None and abs(a['instantaneous_exit_crossing_ns'] / NS - 9.0) < 0.5


def test_gap_touching_anchor_is_critical():
  times = [t for t in grid(0, 20) if not 9.2 < t < 9.8]
  rows = mm.oriented(make_rows(times, 0.0, curvature=lambda t: bend(t, entry=3, exit_=9)), 1)
  a = mm.phase_anchors(rows, (0, 20 * NS), mm.DEFAULT_PARAMETERS)
  assert a['critical_gap'] is True


CAL = mm.Parameters(**{**mm.DEFAULT_PARAMETERS.as_dict(), 'params_id': 'test-cal', 'e_settle_m': 0.15,
                       'v_settle_mps': 0.1, 'a_min_m': 0.1, 't_settle_max_s': 3.0, 'corridor_half_width_m': 0.15,
                       'recovery_horizon_s': 12.0})


def _recovery_case(offset, times=None, curvature=None):
  rows = mm.oriented(make_rows(times or grid(0, 20), offset, curvature=curvature or (lambda t: bend(t, entry=3, exit_=9))), 1)
  return rows, mm.phase_anchors(rows, (0, 20 * NS), CAL)


def test_settling_measures_first_confirmed_dwell():
  # inside the bend the car sits 0.5 m inside; after unwind it decays to zero by ~11.5 s
  rows, anchors = _recovery_case(lambda t: 0.5 if t < 9.5 else 0.5 * math.exp(-(t - 9.5) / 0.6))
  s = mm.settling(rows, (0, 20 * NS), anchors, CAL)
  assert s['status'] == 'measured_estimate' and s['unit'] == 's'
  assert 0.5 < s['value'] < 2.5
  assert s['diagnostics']['residual_offset_m'] < 0.15 and s['critical_gap'] is False


def test_settling_right_censored_bounds_at_last_confirmed_violation():
  # after unwind at ~9.5 s the offset stays 0.5 m until 18.6 s, then meets conditions until the end (20 s)
  rows, anchors = _recovery_case(lambda t: 0.5 if t < 18.6 else 0.0, times=grid(0, 20))
  s = mm.settling(rows, (0, 20 * NS), anchors, CAL)
  assert s['status'] == 'right_censored' and s['value'] is None
  # recovery runs to the window end (20 s); last confirmed violation is near 18.6 s, unwind ~9.2 s
  assert 9.0 < s['lower_bound_s'] < 9.9
  assert s['diagnostics']['candidate_dwell_start_s'] is not None


def test_settling_confirms_a_dwell_that_ends_at_the_recovery_boundary_using_margin_support():
  # recovery ends at 20 s (window end); the dwell 18.85-19.85 s needs filter support from margin rows past 20 s
  rows = mm.oriented(make_rows(grid(-1, 22), lambda t: 0.5 if t < 18.5 else 0.0, curvature=lambda t: bend(t, entry=3, exit_=9)), 1)
  anchors = mm.phase_anchors(rows, (0, 20 * NS), CAL)
  s = mm.settling(rows, (0, 20 * NS), anchors, CAL)
  assert s['status'] == 'measured_estimate' and 9.2 < s['value'] < 10.2


def test_settling_unresolved_when_gap_precedes_first_dwell_and_later_dwell_is_retained():
  times = [t for t in grid(0, 20) if not 10.0 < t < 10.6]
  rows, anchors = _recovery_case(lambda t: 0.5 if t < 9.5 else 0.0, times=times)
  s = mm.settling(rows, (0, 20 * NS), anchors, CAL)
  assert s['status'] == 'unresolved'
  assert s['lower_bound_s'] is not None and s['lower_bound_s'] < 1.0
  assert s['diagnostics']['later_dwell_observation']['start_s'] > 10.6


def test_settling_with_intervention_overlapping_recovery_is_assisted():
  rows, anchors = _recovery_case(lambda t: 0.5 if t < 9.5 else 0.0)
  inside = [{'kind': 'intervention', 'start_ns': 10 * NS, 'end_ns': None, 'provenance': 'rider', 'confidence': 'medium'}]
  assert mm.settling(rows, (0, 20 * NS), anchors, CAL, interventions=inside)['reason'] == 'assisted_recovery'
  before_unknown_end = [{'kind': 'intervention', 'start_ns': 5 * NS, 'end_ns': None, 'provenance': 'rider', 'confidence': 'medium'}]
  assert mm.settling(rows, (0, 20 * NS), anchors, CAL, interventions=before_unknown_end)['reason'] == 'assisted_recovery'
  ended_before = [{'kind': 'intervention', 'start_ns': 5 * NS, 'end_ns': 6 * NS, 'provenance': 'rider', 'confidence': 'medium'}]
  assert mm.settling(rows, (0, 20 * NS), anchors, CAL, interventions=ended_before)['status'] == 'measured_estimate'


def test_settling_without_unwind_anchor_is_unscorable():
  rows = mm.oriented(make_rows(grid(0, 20), 0.5, curvature=lambda t: 0.004 if t >= 3 else 0.0), 1)
  a = mm.phase_anchors(rows, (0, 20 * NS), CAL)
  assert mm.settling(rows, (0, 20 * NS), a, CAL)['reason'] == 'no_unwind_anchor'


def test_cycles_record_and_command_reversals_are_separate():
  rows = make_rows(grid(0, 12), lambda t: 0.4 * math.sin(2 * math.pi * t / 3.0) if 2 <= t <= 8 else 0.0,
                   command=lambda t: 500 if int(t * 4) % 2 else -500)
  c = mm.cycles(rows, (0, 12 * NS), CAL)
  assert c['value'] >= 1 and c['unit'] == 'cycles'
  assert c['diagnostics']['command_sign_reversals'] > 20
  quiet = mm.cycles(make_rows(grid(0, 12), 0.05, command=lambda t: 500 if int(t * 4) % 2 else -500), (0, 12 * NS), CAL)
  assert quiet['value'] == 0
  assert mm.cycles(strip(rows, ('command',)), (0, 12 * NS), CAL)['value'] == c['value']


def test_hold_late_wide_and_boundary_proxies():
  rows = make_rows(grid(0, 20), lambda t: 0.5 if 3 <= t < 9.5 else (-0.3 if 9.5 <= t < 11 else 0.0), width=4.0,
                   curvature=lambda t: bend(t, entry=3, exit_=9))
  o = mm.oriented(rows, 1)
  a = mm.phase_anchors(o, (0, 20 * NS), CAL)
  h = mm.hold_late_wide(o, (0, 20 * NS), a, CAL, 1)
  d = h['diagnostics']
  assert abs(d['inward_peak_m'] - 0.5) < 0.05 and d['inward_dwell_s'] > 5
  assert abs(d['outward_peak_m'] + 0.3) < 0.05
  assert h['status'] == 'measured_estimate' and h['unit'] == 'm'
  # right bend, 4 m lane: +0.5 gives left 2.5 / right 1.5, the -0.3 excursion gives left 1.7 / right 2.3
  proxies = d['boundary_distance_proxy_m']
  assert abs(proxies['min_left_m'] - 1.7) < 0.06 and abs(proxies['min_right_m'] - 1.5) < 0.06
  assert proxies['inside_side'] == 'right' and abs(proxies['min_inside_m'] - 1.5) < 0.06
  const = mm.oriented(make_rows(grid(0, 20), 0.5, width=4.0, curvature=lambda t: bend(t, entry=3, exit_=9)), 1)
  cp = mm.hold_late_wide(const, (0, 20 * NS), mm.phase_anchors(const, (0, 20 * NS), CAL), CAL, 1)['diagnostics']['boundary_distance_proxy_m']
  assert abs(cp['min_left_m'] - 2.5) < 0.06 and abs(cp['min_right_m'] - 1.5) < 0.06
  left_bend = mm.hold_late_wide(mm.oriented(make_rows(grid(0, 20), lambda t: -0.5 if 3 <= t < 9.5 else 0.0, width=4.0,
                                                      curvature=lambda t: bend(t, entry=3, exit_=9, k=-0.004)), -1),
                                (0, 20 * NS), mm.phase_anchors(mm.oriented(make_rows(grid(0, 20), -0.5, width=4.0,
                                                      curvature=lambda t: bend(t, entry=3, exit_=9, k=-0.004)), -1), (0, 20 * NS), CAL), CAL, -1)
  assert left_bend['diagnostics']['boundary_distance_proxy_m']['inside_side'] == 'left'


def test_path_quality_low_rms_with_declared_crossing_keeps_crossing():
  rows = mm.oriented(make_rows(grid(0, 20), 0.02, curvature=lambda t: bend(t, entry=3, exit_=9)), 1)
  a = mm.phase_anchors(rows, (0, 20 * NS), CAL)
  q = mm.path_quality(rows, (0, 20 * NS), a, CAL)
  assert q['value'] < 0.05 and q['unit'] == 'm' and q['diagnostics']['time_outside_corridor_s'] == 0.0
  ann = mm.annotations([{'kind': 'crossing', 'start_ns': 5 * NS, 'end_ns': 7 * NS, 'provenance': 'rider', 'confidence': 'high'}], (0, 20 * NS))
  assert ann['boundary_crossings']['value'] == 1 and ann['boundary_crossings']['diagnostics']['provenance'] == 'rider_annotation'
  assert ann['driver_catches']['status'] == 'unresolved' and ann['driver_catches']['value'] is None
  declared = mm.annotations([], (0, 20 * NS), labels={'intervention': 'absent', 'crossing': 'absent', 'scope': 'clip 0-20 s'})
  assert declared['driver_catches']['value'] == 0 and declared['driver_catches']['status'] == 'measured_estimate'
  assert mm.annotations([], (0, 20 * NS), labels={'intervention': 'absent'})['driver_catches']['status'] == 'unresolved'


def test_comfort_proxy_reports_kinematic_basis_and_needs_yaw_and_speed_only():
  rows = make_rows(grid(0, 10), 0.0, yaw=lambda t: 2.0 * math.sin(2 * math.pi * t / 4.0))
  c = mm.comfort_proxy(rows, (0, 10 * NS), CAL)
  assert c['unit'] == 'm/s^3' and c['value'] > 0 and c['diagnostics']['basis'] == 'kinematic_proxy'
  assert mm.comfort_proxy(strip(rows, ('lane', 'command', 'controls')), (0, 10 * NS), CAL)['value'] == c['value']
  assert mm.comfort_proxy(strip(rows, ('yaw',)), (0, 10 * NS), CAL)['status'] == 'unscorable'
  spike = make_rows(grid(0, 12), 0.0, yaw=lambda t: 0.0 if t < 11.0 else 2.0)  # step outside the 0-10 s window
  sp = mm.comfort_proxy(spike, (0, 10 * NS), CAL)
  assert sp['diagnostics']['peak_mps3'] < 1e-9 and sp['value'] < 1e-9


def test_sparse_coverage_cannot_assert_absence_but_observed_cycles_count():
  # one second of flat data inside a ten-second window
  rows = make_rows(grid(0, 1), 0.0)
  c = mm.cycles(rows, (0, 10 * NS), CAL)
  assert c['status'] == 'unresolved' and c['reason'] == 'interval_not_fully_observed' and c['diagnostics']['observed_cycle_count'] == 0
  # a single 1.8 s internal gap in an otherwise complete 20 s window could hide the whole symptom
  holed = make_rows([t for t in grid(-1, 21) if not 8.0 < t < 9.8], 0.0)
  g = mm.cycles(holed, (0, 20 * NS), CAL)
  assert g['status'] == 'unresolved' and g['critical_gap'] is True and g['diagnostics']['observed_cycle_count'] == 0
  complete = mm.cycles(make_rows(grid(-1, 21), 0.0), (0, 20 * NS), CAL)
  assert complete['status'] == 'measured_estimate' and complete['value'] == 0 and complete['critical_gap'] is False
  wiggle = make_rows(grid(0, 4), lambda t: 0.4 * math.sin(2 * math.pi * t / 1.5))
  w = mm.cycles(wiggle, (0, 10 * NS), CAL)
  assert w['status'] == 'measured_estimate' and w['value'] >= 1 and w['reason'] == 'counted_with_partial_coverage'
  o = mm.oriented(make_rows(grid(0, 6), 0.5, curvature=lambda t: bend(t, entry=3, exit_=9)), 1)
  a = mm.phase_anchors(o, (0, 20 * NS), CAL)
  h = mm.hold_late_wide(o, (0, 20 * NS), a, CAL, 1)
  assert h['status'] == 'unresolved' and h['diagnostics']['coverage_adequate'] is False


def test_missing_lane_geometry_is_unscorable_not_zero():
  rows = strip(make_rows(grid(0, 20), 0.5), ('lane',))
  a = mm.phase_anchors(rows, (0, 20 * NS), CAL)
  assert a['status'] == 'unscorable'
  assert mm.cycles(rows, (0, 20 * NS), CAL)['status'] == 'unscorable'
  assert mm.path_quality(rows, (0, 20 * NS), a, CAL)['value'] is None


def test_resampling_preserves_cycles_and_rms():
  f = lambda t: 0.4 * math.sin(2 * math.pi * t / 3.0) if 2 <= t <= 11 else 0.0
  results = []
  for hz in (10.0, 20.0, 50.0):
    rows = make_rows(grid(0, 14, hz), f)
    a = mm.phase_anchors(rows, (0, 14 * NS), CAL)
    results.append((mm.cycles(rows, (0, 14 * NS), CAL)['value'], mm.path_quality(rows, (0, 14 * NS), a, CAL)['value']))
  assert len({c for c, _ in results}) == 1
  assert max(r for _, r in results) - min(r for _, r in results) < 0.02
