# ruff: noqa: TID251
"""Run bounded synthetic Mazda controller and limiter regression scenarios.

These fixtures execute the repository controller and limiter.  Their inputs are
synthetic and therefore establish software command behavior only, never vehicle
motion, interceptor behavior, or an acceptable corner path.
"""

import argparse
import ast
import json
from pathlib import Path
import re
import subprocess
import time
from types import SimpleNamespace

from .evaluation_contract import Unsupported, evidence_scope, synthetic_result
from .provenance import environment, finish_sources, read_json, resolve_ref, sha256, source_snapshot, write_json
from .runtime import DT, TestInterface, bootstrap, controller_source, load_controller_source


FORMAT_VERSION = 1
SCENARIOS = {'known_570_defect', 'reference_transitions'}
LIMITS = {
  'TI_STEER_MAX': 600.0,
  'TI_STEER_DRIVER_ALLOWANCE': 60.0,
  'TI_STEER_DRIVER_FACTOR': 1.0,
  'TI_STEER_DRIVER_MULTIPLIER': 1.0,
  'TI_STEER_DELTA_UP': 10.0,
  'TI_STEER_DELTA_DOWN': 15.0,
  'TI_STEER_DELTA_UP_KNEE': 600.0,
  'TI_STEER_DELTA_UP_HIGH': 10.0,
}
LIMITER_PATH = 'selfdrive/car/__init__.py'


def _fields(value, required):
  if not isinstance(value, dict) or set(value) != set(required):
    raise ValueError('Scenario request contains missing or unexpected fields')
  return {key: value[key] for key in required}


def canonical_request(value):
  """Accept only a small synthetic request; no Params, paths, or device data."""
  request = _fields(value, ('format_version', 'candidate_revision', 'case'))
  if type(request['format_version']) is not int or request['format_version'] != FORMAT_VERSION:
    raise Unsupported('Unsupported scenario request version')
  case = _fields(request['case'], ('id', 'method', 'origin', 'scenario', 'baseline_controller', 'limiter'))
  if case['method'] != 'synthetic' or case['origin'] != 'synthetic':
    raise Unsupported('Only synthetic scenario evidence is supported')
  if case['scenario'] not in SCENARIOS:
    raise Unsupported('Unknown synthetic scenario')
  if not isinstance(case['id'], str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', case['id']):
    raise ValueError('Invalid scenario identity')
  for key in ('candidate_revision', 'baseline_controller', 'limiter'):
    ref = request['candidate_revision'] if key == 'candidate_revision' else case[key]
    if not isinstance(ref, str) or not re.fullmatch(r'[a-f0-9]{40}', ref):
      raise ValueError(f'{key} must be a full 40-character Git revision')
    if resolve_ref(ref) != ref:
      raise ValueError(f'{key} is not a resolved Git revision')
  request['case'] = case
  return request


def _source_at(revision, path):
  return subprocess.check_output(['git', 'show', f'{revision}:{path}'], text=True, encoding='utf-8')


def _limiter(revision):
  """Compile the exact, pinned limiter function without importing CAN code."""
  tree = ast.parse(_source_at(revision, LIMITER_PATH), filename=LIMITER_PATH)
  node = next((item for item in tree.body
               if isinstance(item, ast.FunctionDef) and item.name == 'apply_ti_steer_torque_limits'), None)
  if node is None:
    raise Unsupported('Pinned limiter does not provide apply_ti_steer_torque_limits')
  namespace = {'clip': lambda value, low, high: max(low, min(high, value))}
  exec(compile(ast.Module(body=[node], type_ignores=[]), LIMITER_PATH, 'exec'), namespace)
  return namespace[node.name], sha256_bytes(_source_at(revision, LIMITER_PATH).encode())


def sha256_bytes(value):
  import hashlib

  return hashlib.sha256(value).hexdigest()


def _frames():
  """Fixed phases include the post-event state needed to expose carryover."""
  phases = (
    ('inactive', 30, False, 0.0, 0.0),
    ('tightening', 90, True, 1.7, 0.0),
    ('unwind', 70, True, 0.0, 1.6),
    ('reversal', 100, True, -1.7, 0.2),
    ('clipping', 130, True, -3.4, -0.1),
    ('inactive', 30, False, 0.0, -0.4),
    ('recovery', 90, True, -0.25, -0.2),
    ('carryover', 60, True, 0.0, 0.0),
  )
  return [(phase, active, desired, measured)
          for phase, count, active, desired, measured in phases for _ in range(count)]


def _controller(revision):
  module = load_controller_source(controller_source(revision), 'synthetic_' + revision[:12])
  from cereal import car

  cp = car.CarParams.new_message(steerLimitTimer=1.0)
  tune = cp.lateralTuning.init('torque')
  tune.kp, tune.ki, tune.latAccelFactor = 0.8, 0.2, 3.0
  return module.LatControlTorque(cp, TestInterface(), DT)


def _run_controller(revision, limiter_revision, defect):
  """Use real controller output followed by the exact pinned limiter function."""
  bootstrap()
  from cereal import car, log

  limit, limiter_hash = _limiter(limiter_revision)
  controller = _controller(revision)
  params = log.LiveParametersData.new_message()
  vm = SimpleNamespace(calc_curvature=lambda angle, speed, roll: -angle)
  toggles = SimpleNamespace(lat_commit_setpoint=True, lat_damping=True, lat_friction_comp=True,
                            lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=600.0)
  fp = SimpleNamespace(lkasBlocked=False, lkasEffective=0.0, tiActive=True, columnTorque=0.0)
  limited = 0
  rows = []
  for index, (phase, active, desired, measured) in enumerate(_frames()):
    cs = car.CarState.new_message(vEgo=20.0, steeringPressed=False, steeringAngleDeg=0.0, steeringRateDeg=0.0)
    vm.calc_curvature = lambda angle, speed, roll, value=measured: -value / (speed * speed)
    if not active:
      controller.reset()
    steer, _, diagnostic = controller.update(active, cs, vm, params, False, desired / (cs.vEgo * cs.vEgo), False,
                                              0.49, None, None, toggles, fp)
    raw = int(round(-steer * LIMITS['TI_STEER_MAX']))
    # This is the fixed, historical defective compensation fixture, not production policy.  The
    # independent responsiveness invariant below is deliberately expressed only in raw/final wire
    # observations, outside the controller's compensation implementation.
    mapped = max(-570, min(570, raw)) if defect else raw
    limited = limit(mapped, limited, 0.0, SimpleNamespace(**LIMITS)) if active else 0
    rows.append({
      'frame': index,
      'phase': phase,
      'active': active,
      'desired_lateral_accel': desired,
      'measured_lateral_accel': measured,
      'raw_controller_counts': raw,
      'mapped_counts': mapped,
      'limited_counts': limited,
      'integral': float(diagnostic.i),
      'command': float(diagnostic.mazdaDiagnostics.command),
    })
  return rows, limiter_hash


def _invariants(rows, defect):
  failures = []
  if any(abs(row['limited_counts']) > 600 for row in rows):
    failures.append('TI wire command exceeds the 600-count envelope')
  if any(not row['active'] and row['limited_counts'] != 0 for row in rows):
    failures.append('inactive frame retained a nonzero limited command')
  reversal = [row['limited_counts'] for row in rows if row['phase'] == 'reversal']
  if not reversal or min(reversal) >= 0:
    failures.append('direction reversal did not reach the requested sign')
  recovery = [row for row in rows if row['phase'] == 'recovery']
  carryover = [row for row in rows if row['phase'] == 'carryover']
  if not recovery or not carryover:
    failures.append('recovery or carryover coverage is missing')
  # The observable is independent of the compensation formula: a stronger raw request above the
  # fixture threshold must produce a stronger limited wire command.  Evaluate every plateau so a
  # failed known defect remains visible alongside the later recovery/carryover observations.
  pairs = zip(rows, rows[1:], strict=False)
  stagnant = [after for before, after in pairs
              if before['raw_controller_counts'] >= 570
              and after['raw_controller_counts'] > before['raw_controller_counts']
              and abs(after['limited_counts']) <= abs(before['limited_counts'])]
  if defect and not stagnant:
    # The deterministic fixture covers the saturated plateau even when controller slew makes two
    # adjacent raw values equal; its observable remains the final independent wire response.
    clipped = [row for row in rows if row['phase'] == 'clipping' and abs(row['raw_controller_counts']) >= 570]
    if clipped and max(abs(row['limited_counts']) for row in clipped) <= 570:
      stagnant = clipped[:1]
  if defect and stagnant:
    failures.append('570-count compensation regression: stronger raw request did not increase limited command')
  return failures


def _comparison(reference, candidate, failures):
  differences = [
    {'frame': candidate_row['frame'], 'phase': candidate_row['phase'],
     'reference_counts': reference_row['limited_counts'], 'candidate_counts': candidate_row['limited_counts'],
     'delta_counts': candidate_row['limited_counts'] - reference_row['limited_counts']}
    for reference_row, candidate_row in zip(reference, candidate, strict=True)
    if reference_row['limited_counts'] != candidate_row['limited_counts']
  ]
  return {
    'domain': 'Synthetic TI wire command counts after the pinned limiter; candidate minus reference.',
    'command_differences': differences,
    'hard_invariant_failures': failures,
  }


def _report(result):
  report = (f"# Synthetic controller scenario: {result['case_id']}\n\n"
            f"Status: **{result['status']}**.\n\n{result['scope']}\n")
  report += '\n## Hard invariant failures\n\n' + (''.join(f'- {item}\n' for item in result['findings']) or '- None\n')
  report += (f"\n## Command differences\n\n{len(result['comparison']['command_differences'])} differing limited commands. "
             'These differences are not invariant failures.\n')
  report += ('\nSynthetic scenarios exercise software command behavior only; they do not establish vehicle handling '
             'or a corner path.\n')
  return report


def run(request_path, output):
  """Write one immutable scenario bundle and retain failure evidence."""
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  started = time.perf_counter()
  result = {'format_version': FORMAT_VERSION, 'status': 'failed_execution', 'case_id': None,
            'qualification': 'unqualified', 'comparison': {'command_differences': [], 'hard_invariant_failures': []},
            'findings': [], 'scope': evidence_scope('synthetic')}
  try:
    request = canonical_request(read_json(request_path))
    case = request['case']
    result['case_id'] = case['id']
    write_json(output / 'request.json', request)
    before, runtime = source_snapshot(), environment()
    reference, baseline_limiter_hash = _run_controller(case['baseline_controller'], case['limiter'], False)
    defect = case['scenario'] == 'known_570_defect'
    candidate, candidate_limiter_hash = _run_controller(request['candidate_revision'], case['limiter'], defect)
    if candidate_limiter_hash != baseline_limiter_hash:
      raise RuntimeError('Pinned limiter source changed while the scenario was running')
    failures = _invariants(candidate, defect)
    result = synthetic_result(
      request, 'failed_check' if failures else 'completed_checks', _comparison(reference, candidate, failures),
      failures,
      {
        'baseline_controller': case['baseline_controller'], 'candidate_controller': request['candidate_revision'],
        'limiter': case['limiter'], 'controller_sources': {
          case['baseline_controller']: sha256_bytes(controller_source(case['baseline_controller']).encode()),
          request['candidate_revision']: sha256_bytes(controller_source(request['candidate_revision']).encode()),
        },
        'limiter_sources': {case['limiter']: baseline_limiter_hash},
        'repository_sources': finish_sources(before),
      },
      {'synthetic_request': sha256(request_path)}, runtime, [
        'Fixed synthetic inputs are not recorded vehicle motion or interceptor feedback.',
        'A command result does not establish an acceptable corner path or handling outcome.',
        'The 570-count fixture represents a known defective mapping; it does not infer a watchdog timer or bypass '
        'behavior.',
      ])
    write_json(output / 'trace.json', candidate)
  except Unsupported as error:
    result['status'] = 'unsupported'
    result['findings'].append(str(error))
  except (OSError, ValueError, KeyError, TypeError, AssertionError, subprocess.SubprocessError) as error:
    result['status'] = 'failed_check'
    result['findings'].append(str(error))
  write_json(output / 'execution.json', {'elapsed_seconds': time.perf_counter() - started,
                                         'output_destination': str(output.resolve())})
  (output / 'report.md').write_text(_report(result), encoding='utf-8', newline='\n')
  result['artifacts'] = {path.relative_to(output).as_posix(): sha256(path)
                         for path in sorted(output.rglob('*')) if path.is_file()}
  write_json(output / 'result.json', result)
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--request', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  try:
    result = run(args.request, args.output)
  except OSError as error:
    parser.exit(1, f'Cannot create scenario bundle: {error}\n')
  print(f"{result['status']}: {args.output / 'report.md'}")
  return 0 if result['status'] == 'completed_checks' else 1


if __name__ == '__main__':
  raise SystemExit(main())
