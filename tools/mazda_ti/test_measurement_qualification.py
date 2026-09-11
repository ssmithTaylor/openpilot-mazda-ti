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
