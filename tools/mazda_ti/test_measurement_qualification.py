"""Runner tests with an injected extractor; no rlogs."""
import json
import math
from pathlib import Path

import pytest

from tools.mazda_ti import maneuver_metrics as mm
from tools.mazda_ti import measurement_qualification as mq
from tools.mazda_ti.test_maneuver_metrics import bend, grid, make_rows

NS = 1_000_000_000


def case(cid, role='development', labels=None, annotations=None, contact='unknown', window=(0, 20 * NS)):
  c = {'id': cid, 'role': role, 'route': 'r' + cid, 'data_root': 'raw', 'rlogs': {f'r{cid}--1/rlog': 'a' * 64},
       'window_ns': list(window), 'window_source': {'description': 'test', 'sha256': None}, 'contact_status': contact}
  if labels is not None:
    c['labels'] = labels
  if annotations is not None:
    c['annotations'] = annotations
  return c


SMOOTH = {'scalloping': 'absent', 'settling': 'smooth', 'provenance': 'rider', 'confidence': 'high', 'scope': 'window'}


def request(cases):
  return {'format_version': 1, 'data_roots': {'raw': 'C:/nowhere'}, 'cases': cases}


def fake_extractor(profiles):
  """Map case route -> offset callable (and optional times); returns observed_drive_rows-shaped output."""
  def run(data_root, rlogs, expected, start_ns, end_ns, margin_ns):
    route = next(iter(rlogs)).split('--')[0]
    offset, times = profiles[route]
    lo, hi = (start_ns - margin_ns) / NS, (end_ns + margin_ns) / NS
    rows = make_rows(times or grid(lo, hi), offset, curvature=lambda t: bend(t, entry=3, exit_=9))
    return {'format_version': 1, 'window_ns': [start_ns, end_ns], 'rlogs': list(rlogs), 'raw_sha256': dict(expected),
            'source_sha256': {'x': 'b' * 64}, 'runtime': {}, 'rows': rows, 'initdata': [], 'unhealthy_count_by_group_and_reason': {},
            'camera_age_s': {'min': 0.03, 'max': 0.04, 'median': 0.035}, 'lane_fit_policy': {}, 'limits': []}
  return run


def test_request_validation_rejects_outcome_fields_on_holdout_and_bad_windows():
  mq.validate_request(request([case('a', labels=SMOOTH), case('h', role='holdout')]))
  with pytest.raises(ValueError, match='holdout'):
    mq.validate_request(request([case('h', role='holdout', labels=SMOOTH)]))
  with pytest.raises(ValueError, match='holdout'):
    mq.validate_request(request([case('h', role='holdout', annotations=[{'kind': 'crossing', 'start_ns': 1, 'end_ns': 2, 'provenance': 'r', 'confidence': 'high'}])]))
  with pytest.raises(ValueError, match='holdout'):
    mq.validate_request(request([case('h', role='holdout', contact='confirmed_no_intervention')]))
  bad = case('a', labels=SMOOTH); bad['window_ns'] = [5.0, 6.0]
  with pytest.raises(ValueError, match='window'):
    mq.validate_request(request([bad]))
  bad = case('a', labels=SMOOTH); bad['rlogs'] = {'x/rlog': 'nothex'}
  with pytest.raises(ValueError, match='sha256'):
    mq.validate_request(request([bad]))
  bad = case('a', labels=SMOOTH); bad['input_status'] = {'ok': False, 'problems': [{'rlog': 'other--1/rlog', 'reason': 'absent'}]}
  with pytest.raises(ValueError, match='input_status'):
    mq.validate_request(request([bad]))
  ok = case('a', labels=SMOOTH); ok['input_status'] = {'ok': False, 'problems': [{'rlog': 'ra--1/rlog', 'reason': 'current bytes differ from retained hash'}]}
  mq.validate_request(request([ok]))
  with pytest.raises(ValueError, match='duplicate'):
    mq.validate_request(request([case('a', labels=SMOOTH), case('a', labels=SMOOTH)]))
  bad = request([case('a', labels=SMOOTH)]); bad['data_roots'] = {'raw': 'relative/path'}
  with pytest.raises(ValueError, match='data_roots'):
    mq.validate_request(bad)
  posix = request([case('a', labels=SMOOTH)]); posix['data_roots'] = {'raw': '/abs/posix'}
  mq.validate_request(posix)
  windows = request([case('a', labels=SMOOTH)]); windows['data_roots'] = {'raw': 'C:/x'}
  mq.validate_request(windows)


def _smooth_profile(residual=0.05, decay=0.6):
  return (lambda t: 0.5 if t < 9.5 else residual + (0.5 - residual) * math.exp(-(t - 9.5) / decay), None)


def test_calibrate_derives_parameters_and_excludes_short_recovery_support():
  profiles = {'ra': _smooth_profile(0.05), 'rb': _smooth_profile(0.08, 0.9),
              'rc': (lambda t: 0.5 if t < 9.5 else 0.0, [t for t in grid(0, 20) if t < 11.0])}  # recovery support < 4 s
  req = request([case('a', labels=SMOOTH), case('b', labels=SMOOTH), case('c', labels=SMOOTH)])
  out = mq.calibrate(req, mm.DEFAULT_PARAMETERS, fake_extractor(profiles))
  p = out['params']
  assert p.params_id != mm.DEFAULT_PARAMETERS.params_id and p.e_settle_m is not None and p.v_settle_mps is not None
  assert p.a_min_m is not None and p.t_settle_max_s is not None and p.corridor_half_width_m == p.e_settle_m
  elig = out['derivation']['e_settle_m']['cases']
  assert elig['a']['eligible'] and elig['b']['eligible'] and not elig['c']['eligible']
  assert 'recovery_support' in elig['c']['reason']
  assert out['derivation']['e_settle_m']['rule'].startswith('max over eligible cases of elapsed-time p95')


def test_calibrate_fails_parameter_when_too_few_eligible_cases():
  profiles = {'ra': _smooth_profile(), 'rc': (lambda t: 0.5 if t < 9.5 else 0.0, [t for t in grid(0, 20) if t < 11.0])}
  req = request([case('a', labels=SMOOTH), case('c', labels=SMOOTH)])
  with pytest.raises(ValueError, match='e_settle_m') as excinfo:
    mq.calibrate(req, mm.DEFAULT_PARAMETERS, fake_extractor(profiles))
  assert set(excinfo.value.derivation) == {'e_settle_m', 'v_settle_mps', 'a_min_m', 't_settle_max_s'}


def test_calibrate_skips_extraction_for_input_unavailable_cases():
  calls = []
  base_extractor = fake_extractor({'ra': _smooth_profile(), 'rb': _smooth_profile()})

  def recording_extractor(data_root, rlogs, expected, start_ns, end_ns, margin_ns):
    calls.append(list(rlogs))
    return base_extractor(data_root, rlogs, expected, start_ns, end_ns, margin_ns)

  bad = case('z', labels=SMOOTH)
  bad['input_status'] = {'ok': False, 'problems': [{'rlog': 'rz--1/rlog', 'reason': 'missing'}]}
  req = request([case('a', labels=SMOOTH), case('b', labels=SMOOTH), bad])
  out = mq.calibrate(req, mm.DEFAULT_PARAMETERS, recording_extractor)
  assert not any(r[0].startswith('rz') for r in calls)
  assert set(out['cases']) == {'a', 'b', 'z'}
  z = out['cases']['z']
  assert z['anchors']['status'] == 'unscorable' and z['anchors']['reason'] == 'input_unavailable'
  for name in ('e_settle_m', 'v_settle_mps', 'a_min_m', 't_settle_max_s'):
    assert out['derivation'][name]['cases']['z']['eligible'] is False
    assert out['derivation'][name]['cases']['z']['reason'] == 'input_unavailable'


def test_calibrate_uses_only_smooth_labelled_development_cases():
  profiles = {'ra': _smooth_profile(), 'rb': _smooth_profile(), 'rx': (lambda t: 0.9, None), 'rh': (lambda t: 0.9, None)}
  req = request([case('a', labels=SMOOTH), case('b', labels=SMOOTH),
                 case('x', labels={**SMOOTH, 'settling': 'problem'}), case('h', role='holdout')])
  out = mq.calibrate(req, mm.DEFAULT_PARAMETERS, fake_extractor(profiles))
  assert set(out['cases']) == {'a', 'b'}


def test_write_calibration_refuses_existing_files(tmp_path):
  profiles = {'ra': _smooth_profile(), 'rb': _smooth_profile()}
  out = mq.calibrate(request([case('a', labels=SMOOTH), case('b', labels=SMOOTH)]), mm.DEFAULT_PARAMETERS, fake_extractor(profiles))
  mq.write_calibration(out, tmp_path / 'params.json', tmp_path / 'calibration.json')
  loaded = json.loads((tmp_path / 'params.json').read_text())
  assert loaded['params_id'] == out['params'].params_id and mm.Parameters(**loaded) == out['params']
  cal = json.loads((tmp_path / 'calibration.json').read_text())
  assert 'tools/mazda_ti/maneuver_metrics.py' in cal['source_sha256'] and len(cal['request_sha256']) == 64
  assert cal['cases']['a']['extraction']['raw_sha256'] == {'ra--1/rlog': 'a' * 64}
  with pytest.raises(FileExistsError):
    mq.write_calibration(out, tmp_path / 'params.json', tmp_path / 'calibration2.json')


def _cal_params():
  return mm.Parameters(**{**mm.DEFAULT_PARAMETERS.as_dict(), 'params_id': 'cal-test', 'e_settle_m': 0.15, 'v_settle_mps': 0.1,
                          'a_min_m': 0.1, 't_settle_max_s': 3.0, 'corridor_half_width_m': 0.15})


def test_detection_outcomes_use_three_values_and_never_alarm_on_short_censoring():
  p = _cal_params()
  smooth_rec = {'metrics': {'settling_time': mm.record('settling_time', unit='s', status='right_censored', reason='x', lower_bound_s=1.0),
                            'lane_motion_cycles': mm.record('lane_motion_cycles', unit='cycles', status='measured_estimate', reason='counted', value=0),
                            'late_wide_excursion': mm.record('late_wide_excursion', unit='m', status='measured_estimate', reason='x', value=0.0,
                                                             diagnostics={'inward_dwell_s': 6.0, 'coverage_adequate': True})}}
  # held inside for 6 s with no outward excursion is a modest-offset pass, not the complaint sequence
  assert mq.detection_outcomes(smooth_rec, p) == {'settling': 'unresolved', 'cycles': 'not_detected', 'held_inward': 'detected',
                                                  'over_release': 'not_detected', 'hold_late_wide_sequence': 'not_detected'}
  seq = {'metrics': {**smooth_rec['metrics'], 'late_wide_excursion': mm.record('late_wide_excursion', unit='m', status='measured_estimate', reason='x', value=0.4,
                                                                               diagnostics={'inward_dwell_s': 6.0, 'coverage_adequate': True})}}
  sparse = {'metrics': {**smooth_rec['metrics'], 'late_wide_excursion': mm.record('late_wide_excursion', unit='m', status='unresolved', reason='interval_not_fully_observed', value=0.0,
                                                                                  diagnostics={'inward_dwell_s': 0.0, 'coverage_adequate': False})}}
  assert mq.detection_outcomes(sparse, p)['held_inward'] == 'unresolved'
  assert mq.detection_outcomes(seq, p)['hold_late_wide_sequence'] == 'detected' and mq.detection_outcomes(seq, p)['over_release'] == 'detected'
  long_rec = {'metrics': {**smooth_rec['metrics'], 'settling_time': mm.record('settling_time', unit='s', status='right_censored', reason='x', lower_bound_s=3.5)}}
  assert mq.detection_outcomes(long_rec, p)['settling'] == 'detected'
  unscorable = {'metrics': {**smooth_rec['metrics'], 'lane_motion_cycles': mm.record('lane_motion_cycles', unit='cycles', reason='no_lane_rows')}}
  assert mq.detection_outcomes(unscorable, p)['cycles'] == 'unresolved'


def test_evaluate_seals_holdout_and_reports_three_outcome_tables(tmp_path):
  p = _cal_params()
  profiles = {'ra': _smooth_profile(), 'rs': (lambda t: 0.5 if t < 9.5 else (0.5 if t < 18.6 else 0.0), None),
              'rc': (lambda t: 0.4 * math.sin(2 * math.pi * t / 3.0) if 3 <= t <= 9 else 0.0, None), 'rh': _smooth_profile()}
  req = request([case('a', labels=SMOOTH), case('s', labels={**SMOOTH, 'settling': 'problem'}, contact='confirmed_no_intervention'),
                 case('c', labels={**SMOOTH, 'scalloping': 'present'}), case('h', role='holdout')])
  out = mq.evaluate(req, p, fake_extractor(profiles))
  assert set(out['development']) == {'a', 's', 'c'} and set(out['holdout']) == {'h'}
  fa = out['report']['false_alarms']['a']
  assert fa == {'settling': 'not_detected', 'cycles': 'not_detected', 'hold_late_wide_sequence': 'not_detected'}
  assert out['report']['symptom_detection']['c']['scalloping'] == 'detected'
  assert out['report']['symptom_detection']['s']['settling'] in ('detected', 'unresolved')
  assert out['report']['validation_status']['status'] == 'unresolved'
  assert out['development']['a']['physical_verdicts'][0]['metric'] == 'minimum_left_clearance'
  mq.write_evaluation(out, tmp_path / 'eval', Path('request.json'), Path('params.json'), request_sha256='1' * 64, params_sha256='2' * 64)
  result = json.loads((tmp_path / 'eval' / 'result.json').read_text())
  sealed = json.loads((tmp_path / 'eval' / 'holdout-sealed.json').read_text())
  text = (tmp_path / 'eval' / 'report.md').read_text()
  assert 'h' in sealed['cases'] and 'metrics' in sealed['cases']['h']
  assert sealed['params']['sha256'] == '2' * 64 and sealed['params']['path'] == 'params.json' and sealed['params_id'] == result['params_id']
  assert sealed['request']['sha256'] == '1' * 64 and sealed['request']['path'] == 'request.json'
  assert 'runtime' in sealed and 'python_version' in sealed['runtime'] and sealed['runtime'] == result['runtime']
  assert result['holdout'] == {'case_ids': ['h'], 'count': 1, 'sealed_sha256': mq.sha256(tmp_path / 'eval' / 'holdout-sealed.json')}
  assert 'settling_time' not in json.dumps(result['holdout']) and '"h"' not in json.dumps(result['development'])
  assert 'runtime' not in result['holdout'] and 'params' not in result['holdout']
  assert 'settling_time' not in text.split('## Holdout (sealed)')[1]
  assert 'holdout' in text and 'unresolved' in text and 'not_detected' in text
  assert 'tools/mazda_ti/measurement_qualification.py' in result['source_sha256'] and sealed['source_sha256'] == result['source_sha256']
  assert result['report']['coverage_totals'] == {'development_cases': 3, 'holdout_cases': 1, 'development_input_unavailable': 0,
                                                 'holdout_input_unavailable': 0, 'development_measured': 3}


def test_unavailable_inputs_stay_in_the_denominator_as_unscorable(tmp_path):
  p = _cal_params()
  gone = case('g', labels=SMOOTH)
  gone['input_status'] = {'ok': False, 'problems': [{'rlog': 'rg--1/rlog', 'reason': 'file absent'}]}
  req = request([case('a', labels=SMOOTH), gone, case('h', role='holdout')])
  calls = []
  def extractor(*args):
    calls.append(args[1][0])
    return fake_extractor({'ra': _smooth_profile(), 'rh': _smooth_profile()})(*args)
  out = mq.evaluate(req, p, extractor)
  assert 'rg--1/rlog' not in calls
  assert out['development']['g']['metrics']['settling_time']['reason'] == 'input_unavailable'
  assert out['report']['false_alarms']['g'] == {'settling': 'unresolved', 'cycles': 'unresolved', 'hold_late_wide_sequence': 'unresolved'}
  assert {u['case'] for u in out['report']['unscorable'] if u['reason'] == 'input_unavailable'} == {'g'}
  assert out['report']['coverage_totals']['development_input_unavailable'] == 1 and out['report']['coverage_totals']['development_cases'] == 2
  with pytest.raises(ValueError, match='e_settle_m'):
    mq.calibrate(request([case('a', labels=SMOOTH), gone]), mm.DEFAULT_PARAMETERS, extractor)


def test_evaluate_never_reads_labels_of_holdout(monkeypatch):
  p = _cal_params()
  req = request([case('a', labels=SMOOTH), case('h', role='holdout')])
  seen = []
  original = mq.detection_outcomes
  monkeypatch.setattr(mq, 'detection_outcomes', lambda rec, params: seen.append(rec['id']) or original(rec, params))
  mq.evaluate(req, p, fake_extractor({'ra': _smooth_profile(), 'rh': _smooth_profile()}))
  assert seen == ['a']


def test_calibrate_e_settle_m_uses_smoothed_offset_not_raw_p95():
  # Finding A: p95_abs_offset_m must come from the smoothed series, mirroring the velocity
  # branch, so 20 Hz noise on an otherwise-settled offset is mostly cancelled by the boxcar.
  amplitude, baseline, noise_hz, cutover = 0.06, 0.02, 13.0, 9.1

  def offset(t):
    return 0.5 if t < cutover else baseline + amplitude * math.sin(2 * math.pi * noise_hz * (t - cutover))

  times = grid(0, 20, 100.0)
  profiles = {'ra': (offset, times), 'rb': (offset, times)}
  req = request([case('a', labels=SMOOTH), case('b', labels=SMOOTH)])
  out = mq.calibrate(req, mm.DEFAULT_PARAMETERS, fake_extractor(profiles))
  smoothed_p95 = out['cases']['a']['recovery_stats']['p95_abs_offset_m']
  assert smoothed_p95 is not None

  # Reconstruct the same continuous-recovery rows calibrate used and compute the raw
  # (pre-fix) p95 directly from the unsmoothed offset, for comparison.
  rows = make_rows(times, offset, curvature=lambda t: bend(t, entry=3, exit_=9))
  oriented_rows = mm.oriented(rows, 1)
  anchors = mm.phase_anchors(oriented_rows, (0, 20 * NS), mm.DEFAULT_PARAMETERS)
  rec = mq._continuous_recovery_rows(oriented_rows, anchors, mm.DEFAULT_PARAMETERS)
  raw_e = [abs(r['lane_offset_m']) for r in rec]
  raw_w = mm.elapsed_weights(rec, int(rec[0]['mono_ns']), int(rec[-1]['mono_ns']) + 1, mm.DEFAULT_PARAMETERS.max_gap_ns)
  raw_p95 = mm.weighted_percentile(raw_e, raw_w, 0.95)

  assert smoothed_p95 < raw_p95
  assert raw_p95 / smoothed_p95 > 2.0
  assert out['params'].e_settle_m == mq._round_up(smoothed_p95, 0.01)
