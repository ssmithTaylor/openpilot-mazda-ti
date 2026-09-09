# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Prepare audited inputs and replay a specified Mazda TI experiment locally."""

import argparse
import contextlib
import hashlib
import io
from pathlib import Path
import sys
from types import SimpleNamespace

from .provenance import check_artifacts, check_sources, environment, finish_sources, read_json, resolve_ref, sha256, source_snapshot, under, write_json

PREPARED_FILES = ('spec.json', 'audit.json', 'mapping.json', 'sampling.json')
REPLAY_FILES = ('trace.json', 'trace.jsonl', 'trace-sends.json')

LIMIT_KEYS = {
  'TI_STEER_MAX': 'TiSteerMax',
  'TI_STEER_DELTA_UP': 'TiSteerDeltaUp',
  'TI_STEER_DELTA_DOWN': 'TiSteerDeltaDown',
  'TI_STEER_DRIVER_ALLOWANCE': 'TiSteerDriverAllowance',
  'TI_STEER_DRIVER_MULTIPLIER': 'TiSteerDriverMultiplier',
  'TI_STEER_DELTA_UP_KNEE': 'TiSteerDeltaUpKnee',
  'TI_STEER_DELTA_UP_HIGH': 'TiSteerDeltaUpHigh',
}
TOGGLE_KEYS = {'LatDamping', 'LatCommitSetpoint', 'LatFrictionComp', 'LatOutputFilter', 'LatNoFrictionRelay', 'TorqueInterceptorEnabled', 'SteerKP'}


def spec_read(path):
  spec = read_json(path)
  if spec['format_version'] != 1:
    raise ValueError('Unsupported experiment format')
  required = set(LIMIT_KEYS.values()) | TOGGLE_KEYS
  if not required.issubset(spec['settings']):
    raise ValueError(f'Missing expected settings: {sorted(required - set(spec["settings"]))}')
  if spec['settings']['TiSteerMax'] != 600 or spec['settings']['LatOutputFilter'] or not spec['settings']['LatNoFrictionRelay']:
    raise ValueError('This qualified replay requires TI600, output filter off and friction relay off')
  if spec['settings']['TorqueInterceptorEnabled'] != 1:
    raise ValueError('This replay requires the TorqueInterceptorEnabled vehicle configuration')
  if len(spec['rlogs']) != len(set(spec['rlogs'])) or not spec['rlogs']:
    raise ValueError('Provide a non-empty list of distinct rlogs')
  window = spec['window']
  if not window['anchor_i_at'] < window['activation'] <= window['start'] < window['end']:
    raise ValueError('Require anchor < activation <= scoring start < scoring end')
  for key in ('baseline_controller', 'warmup_controller', 'limiter'):
    spec[key] = resolve_ref(spec[key])
    if spec[key] == 'worktree':
      raise ValueError(f'{key} must be pinned to a Git revision')
  spec['candidate_controller'] = resolve_ref(spec['candidate_controller'])
  if spec['fpcs_sample_at'] not in ('carState', 'controlsState'):
    raise ValueError('Unknown FPCS sampling assumption')
  return spec


def settings_check(paths, expected):
  from cereal import log

  result = []
  for path in paths:
    found = False
    for event in log.Event.read_multiple_bytes(path.read_bytes()):
      if event.which() != 'initData':
        continue
      entries = {str(e.key): bytes(e.value).decode('utf-8') for e in event.initData.params.entries if str(e.key) in expected}
      for key, value in expected.items():
        if key not in entries or float(entries[key]) != float(value):
          raise ValueError(f'Initial setting {key} differs or is missing in {path.parent.name}')
      result.append({'segment': path.parent.name, 'initial_settings_match': True})
      found = True
      break
    if not found:
      raise ValueError(f'Missing initial settings in {path.parent.name}')
  return result


def prepare(args):
  before = source_snapshot()
  runtime_before = environment()
  spec = spec_read(args.spec)
  paths = [under(args.data_root, name) for name in spec['rlogs']]
  input_hashes = {name: sha256(path) for name, path in zip(spec['rlogs'], paths, strict=False)}
  args.output.mkdir(parents=True, exist_ok=False)
  from .runtime import bootstrap

  bootstrap()
  from .input_audit import audit
  from .request_sampling import infer, load

  checked = settings_check(paths, spec['settings'])
  window = spec['window']
  audited = audit(paths, window['anchor_i_at'] - 1.0, window['end'] + 0.05)
  mapped = [
    r
    for r in audited['frames']
    if 'liveParameters_logMonoTime' in r
    and (r.get('liveParameters_candidates_within_2e9') == 1 or r.get('liveParameters_candidates_control_equivalent', False))
  ]
  limits = SimpleNamespace(**{key: float(spec['settings'][value]) for key, value in LIMIT_KEYS.items()}, TI_STEER_DRIVER_FACTOR=1)
  sampling = infer(load(paths), round(window['activation'] * 1e9), round(window['end'] * 1e9), limits, spec['limiter'])
  write_json(args.output / 'spec.json', spec)
  write_json(args.output / 'audit.json', audited)
  write_json(args.output / 'mapping.json', {'frames': mapped, 'unmapped': len(audited['frames']) - len(mapped)})
  write_json(args.output / 'sampling.json', sampling)
  files = {name: sha256(args.output / name) for name in PREPARED_FILES}
  if any(sha256(path) != input_hashes[name] for name, path in zip(spec['rlogs'], paths, strict=False)) or environment() != runtime_before:
    raise ValueError('Inputs or runtime changed during preparation')
  result = {
    'format_version': 2,
    'stage': 'prepared',
    'input_sha256': input_hashes,
    'prepared_sha256': files,
    'repository_sources': finish_sources(before),
    'environment': runtime_before,
    'settings_checks': checked,
    'input_audit_summary': audited['summary'],
    'sampling_summary': sampling['summary'],
    'limitations': [
      'Initial settings are checked; continuous runtime toggle identity is not inferred from those snapshots.',
      'Compatible request identities are inferred from original sends, not directly logged receipt times.',
    ],
  }
  write_json(args.output / 'preparation.json', result)
  print(f'Prepared {len(mapped)} mapped control frames and {sampling["summary"]["sends"]} sends')


def validate_baseline(path, prep_hash, history, runtime, *, replay_files=REPLAY_FILES):
  baseline = read_json(path)
  if (
    baseline.get('format_version') != 2
    or baseline.get('stage') != 'replayed'
    or baseline.get('variant') != 'baseline'
    or baseline.get('qualification') != 'exact_recorded_commands'
    or not baseline.get('command_bounds_pass')
    or baseline.get('frames', 0) <= 0
    or baseline.get('sends', 0) <= 0
    or baseline.get('preparation_sha256') != prep_hash
    or baseline.get('history') != history
  ):
    raise ValueError('Baseline is unqualified or belongs to another preparation/history')
  if baseline.get('environment') != runtime:
    raise ValueError('Baseline runtime differs from current runtime')
  check_sources(baseline['repository_sources'])
  check_artifacts(path.parent, baseline['output_sha256'], replay_files)
  return baseline


def replay(args):
  before = source_snapshot()
  runtime_before = environment()
  preparation = read_json(args.prepared / 'preparation.json')
  if preparation.get('format_version') != 2 or preparation.get('stage') != 'prepared':
    raise ValueError('Unsupported preparation record')
  check_sources(preparation['repository_sources'])
  if preparation['environment'] != runtime_before:
    raise ValueError('Runtime environment differs from preparation')
  check_artifacts(args.prepared, preparation['prepared_sha256'], PREPARED_FILES)
  spec = read_json(args.prepared / 'spec.json')
  paths = [under(args.data_root, name) for name in spec['rlogs']]
  if any(sha256(path) != preparation['input_sha256'][name] for name, path in zip(spec['rlogs'], paths, strict=False)):
    raise ValueError('Raw-log bytes differ from preparation')
  prep_hash = sha256(args.prepared / 'preparation.json')
  if args.variant == 'candidate':
    if args.baseline_result is None:
      raise ValueError('Candidate replay requires a qualified baseline result')
    validate_baseline(args.baseline_result, prep_hash, args.history, runtime_before)
  args.output.mkdir(parents=True, exist_ok=False)
  from . import controller_replay
  from .runtime import controller_source

  window = spec['window']
  controller_ref = spec['baseline_controller' if args.variant == 'baseline' else 'candidate_controller']
  context = args.variant == 'candidate'
  report_arguments = {'spec': spec, 'variant': args.variant, 'history': args.history}
  options = SimpleNamespace(
    rlogs=[str(p) for p in paths],
    settings=spec['settings'],
    limiter_ref=spec['limiter'],
    start=window['start'],
    end=window['end'],
    anchor_i_at=window['anchor_i_at'],
    variant_start=window['activation'],
    software_feedback_from=window['activation'],
    input_audit=str(args.prepared / 'mapping.json'),
    sample_offset_ms=0.0,
    force_offset=spec['force_offset'],
    seed_i=False,
    kp=spec['settings']['SteerKP'],
    no_commit=not spec['settings']['LatCommitSetpoint'],
    no_damping=not spec['settings']['LatDamping'],
    no_friction_comp=not spec['settings']['LatFrictionComp'],
    variant=args.variant,
    controller_ref=controller_ref,
    warmup_controller_ref=spec['warmup_controller'],
    output_prefix=str(args.output / 'trace'),
    request_identity_map=str(args.prepared / 'sampling.json'),
    request_identity_mode=args.history,
    integrated_model_context=context,
    fpcs_sample_at=spec['fpcs_sample_at'],
    report_arguments=report_arguments,
  )
  with contextlib.redirect_stdout(io.StringIO()):
    summary, rows = controller_replay.run(options)
  sends = read_json(args.output / 'trace-sends.json')['sends']
  exact = (
    summary['rows_scored'] > 0
    and len(sends) > 0
    and summary['output_residual_counts']['max'] < 1
    and summary['plant_state_mismatch_rows'] == 0
    and all(r['counts'] == r['recorded_counts'] for r in sends)
  )
  bounds = all(abs(r['counts']) <= 600 for r in sends) and all(abs(b['counts'] - a['counts']) <= 15 for a, b in zip(sends, sends[1:], strict=False))
  if environment() != runtime_before or any(sha256(path) != preparation['input_sha256'][name] for name, path in zip(spec['rlogs'], paths, strict=False)):
    raise ValueError('Runtime or raw input changed during replay')
  sources = finish_sources(before)
  result = {
    'format_version': 2,
    'stage': 'replayed',
    'variant': args.variant,
    'history': args.history,
    'qualification': ('exact_recorded_commands' if exact else 'failed_baseline') if args.variant == 'baseline' else 'candidate_commands_only',
    'command_bounds_pass': bounds,
    'preparation_sha256': prep_hash,
    'repository_sources': sources,
    'environment': runtime_before,
    'production_sha256': {
      name: sources[name]
      for name in ('selfdrive/controls/lib/latcontrol_torque.py', 'selfdrive/car/mazda/lateral_reference.py', 'selfdrive/controls/controlsd.py')
    },
    'controller_sources': {ref: hashlib.sha256(controller_source(ref).encode()).hexdigest() for ref in (controller_ref, spec['warmup_controller'])},
    'baseline_result_sha256': sha256(args.baseline_result) if args.variant == 'candidate' else None,
    'input_sha256': preparation['input_sha256'],
    'output_sha256': {name: sha256(args.output / name) for name in REPLAY_FILES},
    'frames': summary['rows_scored'],
    'sends': len(sends),
    'controller_residual_counts': summary['output_residual_counts'],
    'limitations': summary['limits'] + ['This command replay does not predict a candidate vehicle path.'],
  }
  write_json(args.output / 'result.json', result)
  if not bounds or (args.variant == 'baseline' and not exact):
    raise ValueError('Replay failed qualification; failed evidence has been retained')
  print(f'{args.variant}: {result["qualification"]}; {result["frames"]} frames, {len(sends)} sends')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  commands = parser.add_subparsers(dest='command', required=True)
  first = commands.add_parser('prepare')
  first.add_argument('--spec', type=Path, required=True)
  second = commands.add_parser('replay')
  second.add_argument('--prepared', type=Path, required=True)
  second.add_argument('--variant', choices=('baseline', 'candidate'), required=True)
  second.add_argument('--history', choices=('earliest', 'latest'), required=True)
  second.add_argument('--baseline-result', type=Path)
  for command in (first, second):
    command.add_argument('--data-root', type=Path, required=True)
    command.add_argument('--output', type=Path, required=True, help='New directory; existing results are preserved')
  args = parser.parse_args()
  try:
    (prepare if args.command == 'prepare' else replay)(args)
  except (ValueError, OSError, AssertionError, KeyError) as error:
    print(f'Replay workflow failed: {error}', file=sys.stderr)
    return 1
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
