"""Extract a hash-bound, health-annotated observed-drive row stream from explicit rlogs.

One row per modelV2 publication, joined to the latest preceding carState,
liveLocationKalman, liveParameters, torque controlsState and published 0x249 TI
send. Health is recorded per field group; nothing is dropped and no missing value
becomes zero. This is a latest-publication join, not exact consumed identity.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
import importlib.util
from pathlib import Path
import statistics
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.mazda_ti.audit_diagnostics import read_events  # noqa: E402
from tools.mazda_ti.provenance import environment, sha256, under, write_json  # noqa: E402

NS = 1_000_000_000
MAX_JOIN_AGE_NS = 200_000_000
MAX_CAMERA_AGE_S = 0.2
GROUPS = ('lane', 'speed', 'steering', 'yaw', 'accel', 'roll', 'controls', 'command')
SETTINGS = ('TiSteerMax', 'TiSteerDeltaUp', 'TiSteerDeltaDown', 'TiSteerDeltaUpKnee', 'TiSteerDeltaUpHigh',
            'LatDamping', 'LatCommitSetpoint', 'LatFrictionComp', 'LatOutputFilter', 'LatNoFrictionRelay',
            'TorqueInterceptorEnabled', 'SteerKP', 'NNFF', 'Model')
LANE_FIT_PATH = ROOT / 'selfdrive/car/mazda/lateral_reference.py'


SOURCE_FILES = ('tools/mazda_ti/observed_drive_rows.py', 'tools/mazda_ti/maneuver_metrics.py',
                'tools/mazda_ti/measurement_qualification.py', 'tools/mazda_ti/physical_evidence.py',
                'tools/mazda_ti/audit_diagnostics.py', 'tools/mazda_ti/provenance.py',
                'selfdrive/car/mazda/lateral_reference.py')


def source_hashes():
  """Hashes of every measurement dependency, taken before execution and rechecked after."""
  paths = [ROOT / name for name in SOURCE_FILES] + sorted((ROOT / 'cereal').glob('*.capnp'))
  return {p.relative_to(ROOT).as_posix(): sha256(p) for p in paths if p.is_file()}


def load_lane_fit():
  spec = importlib.util.spec_from_file_location('observed_drive_lane_fit', LANE_FIT_PATH)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module, LANE_FIT_PATH


def serialized(events):
  from cereal import log
  chunks = []
  for ev in events:
    ev.clear_write_flag()
    chunks.append(ev.to_bytes())
  return log.Event.read_multiple_bytes(b''.join(chunks))


def _latest(times, items, t):
  i = bisect_right(times, t) - 1
  return (None, None) if i < 0 else (items[i], t - times[i])


def _group(ok, reason):
  return bool(ok), reason


def extract(events, start_ns, end_ns, margin_ns, fit):
  camera_age_limit_source = 'selfdrive/car/mazda/lateral_reference.py::MAX_CAMERA_AGE'
  if hasattr(fit, 'MAX_CAMERA_AGE'):
    camera_age_limit = float(fit.MAX_CAMERA_AGE)
  else:
    camera_age_limit, camera_age_limit_source = MAX_CAMERA_AGE_S, 'fallback_default'
  models, cars, llks, prms, ctls, tis, init = [], [], [], [], [], [], []
  lo, hi = start_ns - margin_ns, end_ns + margin_ns
  for ev in events:
    which, t = ev.which(), int(ev.logMonoTime)
    if which == 'initData':
      d = ev.initData
      settings = {k: None for k in SETTINGS}
      settings.update({str(e.key): bytes(e.value).decode(errors='replace') for e in d.params.entries if str(e.key) in SETTINGS})
      init.append({'git_commit': str(d.gitCommit), 'dirty': bool(d.dirty), 'settings': settings})
      continue
    if not lo <= t <= hi:
      continue
    if which == 'modelV2':
      m = ev.modelV2
      ctx = fit.fit_lane_context(m, 0.0)
      width = None
      if ctx.valid:
        left, right = m.laneLines[1], m.laneLines[2]
        width = float(np.interp(0, np.asarray(left.x, dtype=float), np.asarray(right.y, dtype=float) - np.asarray(left.y, dtype=float)))
      probs = list(m.laneLineProbs)
      models.append({'t': t, 'valid': bool(ev.valid), 'eof': int(m.timestampEof), 'frame': int(m.frameId),
                     'lane_change': str(m.meta.laneChangeState), 'fit': ctx, 'width': width,
                     'prob_min': min(float(probs[1]), float(probs[2])) if len(probs) >= 3 else None})
    elif which == 'carState':
      c = ev.carState
      cars.append({'t': t, 'valid': bool(ev.valid), 'can': bool(c.canValid), 'speed': float(c.vEgo),
                   'angle': -float(c.steeringAngleDeg), 'rate': -float(c.steeringRateDeg)})
    elif which == 'liveLocationKalman':
      k = ev.liveLocationKalman
      llks.append({'t': t, 'valid': bool(ev.valid), 'yaw_valid': bool(k.angularVelocityCalibrated.valid),
                   'accel_valid': bool(k.accelerationDevice.valid),
                   'health': bool(k.sensorsOK and k.inputsOK and k.posenetOK),
                   'yaw': float(k.angularVelocityCalibrated.value[2]), 'yaw_std': float(k.angularVelocityCalibrated.std[2]),
                   'ay': float(k.accelerationDevice.value[1])})
    elif which == 'liveParameters':
      prms.append({'t': t, 'valid': bool(ev.valid), 'roll': float(ev.liveParameters.roll)})
    elif which == 'controlsState' and ev.controlsState.lateralControlState.which() == 'torqueState':
      x = ev.controlsState.lateralControlState.torqueState
      ctls.append({'t': t, 'valid': bool(ev.valid), 'active': bool(x.active), 'request': float(x.desiredLateralAccel),
                   'actual': float(x.actualLateralAccel), 'diag_version': int(x.mazdaDiagnostics.version)})
    elif which == 'sendcan':
      for f in ev.sendcan:
        if f.src == 1 and f.address == 0x249 and len(f.dat) == 8:
          q = (((f.dat[0] & 15) << 8) | f.dat[1]) - 2048
          tis.append({'t': t, 'command': -q})
  for lst in (models, cars, llks, prms, ctls, tis):
    lst.sort(key=lambda x: x['t'])
  ct, lt, pt, kt, tt = ([x['t'] for x in lst] for lst in (cars, llks, prms, ctls, tis))
  rows, unhealthy, ages = [], {g: {} for g in GROUPS}, []
  for m in models:
    t = m['t']
    if not start_ns - margin_ns <= t <= end_ns + margin_ns:
      continue
    car, car_age = _latest(ct, cars, t)
    k, k_age = _latest(lt, llks, t)
    p, p_age = _latest(pt, prms, t)
    c, c_age = _latest(kt, ctls, t)
    s, s_age = _latest(tt, tis, t)
    camera_age = (t - m['eof']) / NS
    ages.append(camera_age)
    health, reason = {}, {}
    if not m['valid']:
      health['lane'], reason['lane'] = _group(False, 'invalid_event')
    elif not m['fit'].valid:
      health['lane'], reason['lane'] = _group(False, m['fit'].reason)
    elif not 0 <= camera_age <= camera_age_limit:
      health['lane'], reason['lane'] = _group(False, 'camera_age')
    else:
      health['lane'], reason['lane'] = _group(True, 'valid')
    def source(item, age, ok, name):
      if item is None:
        return _group(False, 'missing')
      if age > MAX_JOIN_AGE_NS:
        return _group(False, 'stale')
      return _group(ok(item), 'valid' if ok(item) else name)
    health['speed'], reason['speed'] = source(car, car_age, lambda x: x['valid'] and x['can'], 'invalid')
    health['steering'], reason['steering'] = health['speed'], reason['speed']
    health['yaw'], reason['yaw'] = source(k, k_age, lambda x: x['valid'] and x['yaw_valid'] and x['health'], 'invalid')
    health['accel'], reason['accel'] = source(k, k_age, lambda x: x['valid'] and x['accel_valid'] and x['health'], 'invalid')
    health['roll'], reason['roll'] = source(p, p_age, lambda x: x['valid'], 'invalid')
    health['controls'], reason['controls'] = source(c, c_age, lambda x: x['valid'], 'invalid')
    health['command'], reason['command'] = source(s, s_age, lambda x: True, 'invalid')
    for g in GROUPS:
      if not health[g]:
        unhealthy[g][reason[g]] = unhealthy[g].get(reason[g], 0) + 1
    fit_ok = health['lane']
    rows.append({
      'mono_ns': t, 'camera_eof_ns': m['eof'], 'camera_age_s': camera_age, 'frame_id': m['frame'],
      'health': health, 'health_reason': reason, 'lane_change_state': m['lane_change'],
      'lane_fit_reason': m['fit'].reason, 'lane_probability_min': m['prob_min'],
      'lane_offset_m': float(m['fit'].offset) if fit_ok else None,
      'lane_heading10_rad': float(m['fit'].heading10) if fit_ok else None,
      'lane_heading20_rad': float(m['fit'].heading20) if fit_ok else None,
      'road_curvature10_per_m': float(m['fit'].curvature10) if fit_ok else None,
      'road_curvature20_per_m': float(m['fit'].curvature20) if fit_ok else None,
      'lane_width_m': m['width'] if fit_ok else None,
      'speed_mps': car['speed'] if health['speed'] else None,
      'steering_angle_deg': car['angle'] if health['steering'] else None,
      'steering_rate_dps': car['rate'] if health['steering'] else None,
      'yaw_rate_rps': k['yaw'] if health['yaw'] else None, 'yaw_std_rps': k['yaw_std'] if health['yaw'] else None,
      'yaw_lateral_accel_mps2': k['yaw'] * car['speed'] if (health['yaw'] and health['speed']) else None,
      'device_lateral_accel_mps2': k['ay'] if health['accel'] else None,
      'roll_rad': p['roll'] if health['roll'] else None,
      'controls_active': c['active'] if health['controls'] else None,
      'request_mps2': c['request'] if health['controls'] else None,
      'actual_mps2': c['actual'] if health['controls'] else None,
      'diag_version': c['diag_version'] if health['controls'] else None,
      'ti_command_counts': s['command'] if health['command'] else None,
      'ages_ns': {'carState': car_age, 'liveLocationKalman': k_age, 'liveParameters': p_age,
                  'controlsState': c_age, 'ti_send': s_age},
    })
  return {'rows': rows, 'initdata': init, 'unhealthy_count_by_group_and_reason': unhealthy,
          'camera_age_s': ({'min': min(ages), 'max': max(ages), 'median': statistics.median(ages)} if ages else None),
          'lane_fit_policy': {'source': 'selfdrive/car/mazda/lateral_reference.py::fit_lane_context',
                              'min_lane_probability': float(fit.MIN_LANE_PROBABILITY), 'max_camera_age_s': camera_age_limit,
                              'max_camera_age_source': camera_age_limit_source,
                              'clock_assumption': 'camerad boot-time EOF and monotonic publication assumed unsuspended; ages outside [0, max] are unhealthy'}}


def run(data_root, rlogs, expected_sha256, start_ns, end_ns, margin_ns):
  if type(start_ns) is not int or type(end_ns) is not int or not 0 <= start_ns < end_ns or margin_ns < 0:
    raise ValueError('Require integer nanosecond window with start < end and nonnegative margin')
  if len(set(rlogs)) != len(rlogs) or set(rlogs) != set(expected_sha256):
    raise ValueError('Each rlog needs exactly one expected hash')
  paths = [under(Path(data_root), name) for name in rlogs]
  before = {name: sha256(p) for name, p in zip(rlogs, paths, strict=True)}
  if before != expected_sha256:
    raise ValueError('Raw rlog bytes differ from the declared hashes')
  sources_before = source_hashes()
  fit, fit_path = load_lane_fit()
  runtime = environment()
  result = extract(read_events(paths), start_ns, end_ns, margin_ns, fit)
  if {name: sha256(p) for name, p in zip(rlogs, paths, strict=True)} != before or environment() != runtime:
    raise ValueError('Inputs or runtime changed during extraction')
  if source_hashes() != sources_before:
    raise ValueError('Source files changed during extraction')
  return {'format_version': 1, 'scope': __doc__, 'window_ns': [start_ns, end_ns], 'margin_ns': margin_ns,
          'rlogs': rlogs, 'raw_sha256': before, 'source_sha256': sources_before, 'runtime': runtime,
          **result,
          'limits': ['Latest-publication health-gated join; not exact consumed-input identity',
                     'Lane geometry is model-derived, not surveyed truth',
                     'Published TI send is command evidence, not delivered torque',
                     'No missing value is converted to zero']}


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--rlog', action='append', required=True)
  parser.add_argument('--sha256', action='append', required=True)
  parser.add_argument('--start-ns', type=int, required=True)
  parser.add_argument('--end-ns', type=int, required=True)
  parser.add_argument('--margin-ns', type=int, default=NS)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args(argv)
  if len(args.rlog) != len(args.sha256):
    parser.error('Supply one --sha256 per --rlog')
  if args.output.exists():
    raise FileExistsError(args.output)
  result = run(args.data_root, args.rlog, dict(zip(args.rlog, args.sha256, strict=True)), args.start_ns, args.end_ns, args.margin_ns)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  write_json(args.output, result)


if __name__ == '__main__':
  main()
