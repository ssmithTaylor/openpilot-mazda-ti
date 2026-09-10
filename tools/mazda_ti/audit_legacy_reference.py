"""Describe hidden feedback references in pinned legacy plant-controller logs.

No candidate policy, actuator replay, receipt inference or physical pass/fail.
"""
import argparse
import ast
from bisect import bisect_right
from dataclasses import asdict
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys

from .audit_diagnostics import read_events
from .legacy_reference import reference_from_observations
from .provenance import ROOT, environment, read_json, sha256, under, write_json

REVISION = '2c50a68456d6f8eb18c5688a72a2c055a2572182'


def validate_recorded_configuration(snapshots, gains):
  if not snapshots or any(x != (REVISION, False, '0', '0') for x in snapshots) or len(gains) != 1:
    raise ValueError('Require pinned clean source, NNFF/NNFFLite disabled and one recorded gain')
  return next(iter(gains))


def constants(source):
  found = {}
  for node in ast.parse(source).body:
    if isinstance(node, ast.Assign):
      for target in node.targets:
        if isinstance(target, ast.Name):
          try:
            found[target.id] = ast.literal_eval(node.value)
          except (ValueError, TypeError):
            pass
  return found


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--request', type=Path, required=True)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if args.output.exists():
    parser.error('Preserve existing output')
  request_hash = sha256(args.request)
  request = read_json(args.request)
  windows = request['windows']
  if not windows or any(not 0 < w['start_ns'] < w['end_ns'] for w in windows.values()):
    raise ValueError('Require nonempty positive half-open windows')
  raw = {name: under(args.data_root, name) for name in request['rlogs']}
  raw_hashes = {name: sha256(path) for name, path in raw.items()}
  if len(raw) != len(request['rlogs']) or raw_hashes != request['raw_sha256']:
    raise ValueError('Duplicate or changed raw inputs')
  source_paths = [Path(__file__).resolve(), ROOT/'tools/mazda_ti/legacy_reference.py',
                  ROOT/'tools/mazda_ti/legacy_input_constraints.py',
                  ROOT/'tools/mazda_ti/audit_diagnostics.py', ROOT/'tools/mazda_ti/provenance.py',
                  ROOT/'selfdrive/car/mazda/lateral_reference.py', ROOT/'cereal/__init__.py',
                  *sorted((ROOT/'cereal').glob('*.capnp'))]
  sources = {p.relative_to(ROOT).as_posix(): sha256(p) for p in source_paths}
  runtime = environment()
  blobs = {p: subprocess.check_output(['git', 'show', f'{REVISION}:{p}'], cwd=ROOT)
           for p in ('selfdrive/controls/controlsd.py', 'selfdrive/controls/lib/latcontrol_torque.py',
                     'selfdrive/controls/lib/drive_helpers.py')}
  table = constants(blobs['selfdrive/controls/lib/latcontrol_torque.py'])
  minimum = constants(blobs['selfdrive/controls/lib/drive_helpers.py'])['MIN_SPEED']
  spec = importlib.util.spec_from_file_location('reference_audit_geometry', ROOT/'selfdrive/car/mazda/lateral_reference.py')
  geometry = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = geometry
  spec.loader.exec_module(geometry)
  start = min(w['start_ns'] for w in windows.values())
  end = max(w['end_ns'] for w in windows.values())
  controllers, models, params, snapshots, gains = {}, {}, {}, [], set()
  for event in read_events(list(raw.values())):
    service, stamp = event.which(), int(event.logMonoTime)
    if service == 'initData':
      settings = {v.key: bytes(v.value).decode() for v in event.initData.params.entries
                  if v.key in ('NNFF', 'NNFFLite')}
      snapshots.append((event.initData.gitCommit, bool(event.initData.dirty),
                        settings.get('NNFF'), settings.get('NNFFLite')))
    elif service == 'carParams':
      cp = event.carParams
      if cp.carFingerprint != 'MAZDA_CX5' or cp.lateralTuning.which() != 'torque' or not cp.lateralTuning.torque.useSteeringAngle:
        raise ValueError('Unsupported CarParams')
      gains.add(float(cp.lateralTuning.torque.kp))
    elif start-1_000_000_000 <= stamp < end:
      if service == 'controlsState':
        if stamp in controllers:
          raise ValueError('Duplicate controller identity')
        controllers[stamp] = (bool(event.valid), event.controlsState.to_dict())
      elif service == 'modelV2':
        if stamp in models:
          raise ValueError('Duplicate model identity')
        g = asdict(geometry.fit_lane_context(event.modelV2, 0.0))
        g.pop('age')  # Placeholder used only for pure fitting; not a freshness claim.
        models[stamp] = {'valid': bool(event.valid), 'geometry': g,
                         'camera_eof_ns': int(event.modelV2.timestampEof)}
      elif service == 'liveParameters':
        if stamp in params:
          raise ValueError('Duplicate parameter identity')
        params[stamp] = bool(event.valid)
  kp = validate_recorded_configuration(snapshots, gains)
  parameter_stamps = sorted(params)
  rows = []
  for stamp, (valid, c) in sorted(controllers.items()):
    groups = [name for name, w in windows.items() if w['start_ns'] <= stamp < w['end_ns']]
    if not groups:
      continue
    torque = c['lateralControlState'].get('torqueState')
    row = {'mono_ns': stamp, 'windows': groups, 'controller_valid': valid, 'native': torque}
    model_id = c['lateralPlanMonoTime']
    row['model_mono_ns'] = model_id
    row['model'] = models.get(model_id)
    row['model_age_ns'] = stamp-model_id
    i = bisect_right(parameter_stamps, stamp)-1
    p = parameter_stamps[i] if i >= 0 else None
    row['latest_parameter_publication'] = None if p is None else {'mono_ns': p, 'valid': params[p], 'age_ns': stamp-p}
    row['reference'] = None
    if torque is None or not torque['active'] or torque['version'] != 2:
      row['unavailable_reason'] = 'unsupported_or_inactive_controller'
    else:
      try:
        row['reference'] = reference_from_observations(curvature=c['curvature'],
            measured_accel=torque['actualLateralAccel'], desired_curvature=c['desiredCurvature'],
            delayed_setpoint=torque['desiredLateralAccel'], logged_error=torque['error'],
            torque_params_kp=kp, low_speed_x=table['LOW_SPEED_X'], low_speed_y=table['LOW_SPEED_Y'],
            minimum_speed=minimum)
      except ValueError as exc:
        row['unavailable_reason'] = str(exc)
    rows.append(row)
  result = {'scope': __doc__, 'request': request, 'request_sha256': request_hash,
            'source_revision': REVISION, 'raw_sha256': raw_hashes, 'source_sha256': sources,
            'git_blob_sha256': {p: hashlib.sha256(b).hexdigest() for p, b in blobs.items()},
            'runtime': runtime, 'torque_params_kp': kp, 'rows': rows,
            'limits': ['Algebraic description; not independent controller reproduction or vehicle simulation.',
                       'Reference bounds cover Float32 serialization only, not measurement uncertainty.',
                       'Current request includes upstream curvature limiting; history includes filtering and delay.',
                       'Exact model identifier is logged; parameter publication is as-of, not consumed identity.',
                       'Lane fit is model-origin geometry, not surveyed lane position or vehicle clearance.',
                       'Native frictionTorque includes relay/breaker but excludes proactive friction compensation.']}
  if not rows or any(not any(name in r['windows'] for r in rows) for name in windows):
    raise ValueError('Missing window coverage')
  if (request_hash != sha256(args.request) or raw_hashes != {n: sha256(p) for n, p in raw.items()}
      or sources != {p.relative_to(ROOT).as_posix(): sha256(p) for p in source_paths} or runtime != environment()):
    raise ValueError('Source, inputs or runtime changed during analysis')
  write_json(args.output, result)
  print({'rows': len(rows), 'unresolved': sum(r['reference'] is None for r in rows), 'sha256': sha256(args.output)})


if __name__ == '__main__':
  main()
