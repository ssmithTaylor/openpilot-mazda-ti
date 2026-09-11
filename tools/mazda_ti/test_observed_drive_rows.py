"""Generated-rlog tests for the observed-drive row extractor."""
import json
from pathlib import Path

import pytest
from cereal import log

from tools.mazda_ti import observed_drive_rows as odr

NS = 1_000_000_000


def model(mono, eof, probs=(0.0, 0.95, 0.95, 0.0), y=(-6, -1.8, 1.8, 6), lane_change='off', frame=1):
  ev = log.Event.new_message(logMonoTime=mono, valid=True)
  m = ev.init('modelV2')
  m.timestampEof, m.frameId = eof, frame
  m.meta.laneChangeState = lane_change
  m.laneLineProbs = list(probs)
  lanes = m.init('laneLines', 4)
  for lane, yy in zip(lanes, y, strict=True):
    lane.x, lane.y = [0, 5, 10, 15, 20, 25], [yy] * 6
  return ev


def car(mono, speed=24.0, angle=-17.0, rate=1.0, valid=True, can=True):
  ev = log.Event.new_message(logMonoTime=mono, valid=valid)
  c = ev.init('carState')
  c.vEgo, c.steeringAngleDeg, c.steeringRateDeg, c.canValid = speed, angle, rate, can
  return ev


def llk(mono, yaw=0.09, ay=2.1, ok=True):
  ev = log.Event.new_message(logMonoTime=mono, valid=True)
  k = ev.init('liveLocationKalman')
  k.angularVelocityCalibrated.value, k.angularVelocityCalibrated.std, k.angularVelocityCalibrated.valid = [0, 0, yaw], [0, 0, 0.001], True
  k.accelerationDevice.value, k.accelerationDevice.valid = [0, ay, 0], True
  k.sensorsOK = k.inputsOK = k.posenetOK = ok
  return ev


def params(mono, roll=0.03):
  ev = log.Event.new_message(logMonoTime=mono, valid=True)
  ev.init('liveParameters').roll = roll
  return ev


def controls(mono, active=True):
  ev = log.Event.new_message(logMonoTime=mono, valid=True)
  t = ev.init('controlsState').lateralControlState.init('torqueState')
  t.active, t.desiredLateralAccel, t.actualLateralAccel = active, 2.2, 2.1
  t.mazdaDiagnostics.version = 0
  return ev


def ti(mono, counts=500):
  ev = log.Event.new_message(logMonoTime=mono, valid=True)
  q = -counts + 2048
  frames = ev.init('sendcan', 1)
  frames[0].address, frames[0].src, frames[0].dat = 0x249, 1, bytes([(q >> 8) & 15, q & 255, 0, 0, 0, 0, 0, 0])
  return ev


def initdata():
  ev = log.Event.new_message(logMonoTime=1, valid=True)
  d = ev.init('initData')
  d.gitCommit, d.dirty = 'abc', False
  entries = d.params.init('entries', 2)
  entries[0].key, entries[0].value = 'TiSteerMax', b'600.000000'
  entries[1].key, entries[1].value = 'TorqueInterceptorEnabled', b'1'
  return ev


def stream(n=40, *, drop_ti=False, low_prob_at=None, stale_camera_at=None):
  events = [initdata()]
  for i in range(n):
    t = 10 * NS + i * 50_000_000
    probs = (0.0, 0.75, 0.95, 0.0) if i == low_prob_at else (0.0, 0.95, 0.95, 0.0)
    eof = t - 400_000_000 if i == stale_camera_at else t - 38_000_000
    events += [car(t - 500_000), llk(t - 40_000_000), params(t - 40_000_000), controls(t - 5_000_000)]
    if not drop_ti:
      events.append(ti(t - 1_000_000))
    events.append(model(t, eof, probs=probs, frame=i))
  return events


def write_rlog(path, events):
  path.parent.mkdir(parents=True)
  chunks = []
  for ev in events:
    ev.clear_write_flag()
    chunks.append(ev.to_bytes())
  path.write_bytes(b''.join(chunks))


def test_join_produces_healthy_rows_with_width_and_decoded_command():
  fit, _ = odr.load_lane_fit()
  out = odr.extract(odr.serialized(stream()), 10 * NS, 12 * NS, NS, fit)
  rows = out['rows']
  assert len(rows) == 40 and all(all(r['health'].values()) for r in rows)
  r = rows[5]
  assert abs(r['lane_width_m'] - 3.6) < 1e-6 and r['ti_command_counts'] == 500
  assert r['steering_angle_deg'] == 17.0 and r['steering_rate_dps'] == -1.0
  assert abs(r['yaw_lateral_accel_mps2'] - 0.09 * 24.0) < 1e-9 and r['lane_offset_m'] == pytest.approx(0.0)
  assert out['lane_fit_policy']['min_lane_probability'] == 0.8
  assert out['initdata'][0]['settings']['TiSteerMax'] == '600.000000'


def test_lane_probability_below_production_policy_is_lane_unhealthy_only():
  fit, _ = odr.load_lane_fit()
  rows = odr.extract(odr.serialized(stream(low_prob_at=7)), 10 * NS, 12 * NS, NS, fit)['rows']
  bad = rows[7]
  assert bad['health']['lane'] is False and bad['health_reason']['lane'] == 'confidence'
  assert bad['lane_offset_m'] is None and bad['health']['speed'] is True and bad['ti_command_counts'] == 500


def test_stale_camera_is_lane_unhealthy_and_missing_ti_is_command_unhealthy_only():
  fit, _ = odr.load_lane_fit()
  rows = odr.extract(odr.serialized(stream(stale_camera_at=3)), 10 * NS, 12 * NS, NS, fit)['rows']
  assert rows[3]['health']['lane'] is False and rows[3]['health_reason']['lane'] == 'camera_age'
  rows = odr.extract(odr.serialized(stream(drop_ti=True)), 10 * NS, 12 * NS, NS, fit)['rows']
  assert all(r['health']['command'] is False and r['ti_command_counts'] is None for r in rows)
  assert all(r['health']['lane'] and r['health']['yaw'] for r in rows)


def test_camera_age_limit_reads_loaded_module_with_fallback_and_reports_source():
  # Finding E: the extractor must read the camera-age limit off the loaded lateral_reference
  # module (not its own copied constant), falling back only when the attribute is absent,
  # and publish whichever was used plus its source.
  fit, _ = odr.load_lane_fit()
  out = odr.extract(odr.serialized(stream()), 10 * NS, 12 * NS, NS, fit)
  assert out['lane_fit_policy']['max_camera_age_s'] == fit.MAX_CAMERA_AGE
  assert out['lane_fit_policy']['max_camera_age_source'] == 'selfdrive/car/mazda/lateral_reference.py::MAX_CAMERA_AGE'
  assert all(r['health']['lane'] for r in out['rows'])  # 38 ms camera age, under the real 0.2 s limit

  class TighterAge:
    MAX_CAMERA_AGE = 0.03

    def __getattr__(self, name):
      return getattr(fit, name)

  tightened = odr.extract(odr.serialized(stream()), 10 * NS, 12 * NS, NS, TighterAge())
  assert tightened['lane_fit_policy']['max_camera_age_s'] == 0.03
  assert tightened['lane_fit_policy']['max_camera_age_source'] == 'selfdrive/car/mazda/lateral_reference.py::MAX_CAMERA_AGE'
  # the same 38 ms camera age is unhealthy under the proxy's tighter limit: proves the value is actually used
  assert all(r['health']['lane'] is False and r['health_reason']['lane'] == 'camera_age' for r in tightened['rows'])

  class NoAge:
    def __getattr__(self, name):
      if name == 'MAX_CAMERA_AGE':
        raise AttributeError(name)
      return getattr(fit, name)

  fallback = odr.extract(odr.serialized(stream()), 10 * NS, 12 * NS, NS, NoAge())
  assert fallback['lane_fit_policy']['max_camera_age_s'] == odr.MAX_CAMERA_AGE_S
  assert fallback['lane_fit_policy']['max_camera_age_source'] == 'fallback_default'
  assert all(r['health']['lane'] for r in fallback['rows'])  # fallback equals the same 0.2 s default


def test_run_binds_hashes_and_refuses_changed_raw_or_existing_output(tmp_path):
  root = tmp_path / 'raw'
  rlog = root / 'route--1' / 'rlog'
  write_rlog(rlog, stream())
  good = odr.sha256(rlog)
  out = odr.run(root, ['route--1/rlog'], {'route--1/rlog': good}, 10 * NS, 12 * NS, NS)
  assert out['raw_sha256'] == {'route--1/rlog': good} and len(out['rows']) == 40
  assert {'selfdrive/car/mazda/lateral_reference.py', 'tools/mazda_ti/maneuver_metrics.py', 'tools/mazda_ti/physical_evidence.py'} <= set(out['source_sha256'])
  with pytest.raises(ValueError):
    odr.run(root, ['route--1/rlog'], {'route--1/rlog': 'f' * 64}, 10 * NS, 12 * NS, NS)
  target = tmp_path / 'rows.json'
  odr.main(['--data-root', str(root), '--rlog', 'route--1/rlog', '--sha256', good,
            '--start-ns', str(10 * NS), '--end-ns', str(12 * NS), '--output', str(target)])
  assert json.loads(target.read_text())['window_ns'] == [10 * NS, 12 * NS]
  with pytest.raises(FileExistsError):
    odr.main(['--data-root', str(root), '--rlog', 'route--1/rlog', '--sha256', good,
              '--start-ns', str(10 * NS), '--end-ns', str(12 * NS), '--output', str(target)])
