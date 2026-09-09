# ruff: noqa: TID251
"""Bounded, resumable scheduling for completed common-adapter evaluations."""

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import time

from .evaluate import evaluate, verify_bundle
from .evaluation_contract import canonical_request
from .provenance import read_json, resolve_ref, sha256, source_snapshot, write_json


FORMAT_VERSION = 1


def _inside(root, name):
  path = (root / name).resolve()
  if not isinstance(name, str) or Path(name).is_absolute() or not path.is_relative_to(root.resolve()):
    raise ValueError('Batch path escapes the manifest directory')
  return path


def _manifest(path):
  value = read_json(path)
  if not isinstance(value, dict) or set(value) != {'format_version', 'resources', 'cases'} or value['format_version'] != FORMAT_VERSION:
    raise ValueError('Unsupported batch manifest')
  resources = value['resources']
  if not isinstance(resources, dict) or set(resources) != {'max_workers', 'host_memory_mb', 'worker_memory_mb'}:
    raise ValueError('Invalid batch resource declaration')
  if any(type(resources[key]) is not int or resources[key] <= 0 for key in resources):
    raise ValueError('Batch resources must be positive integers')
  cases = value['cases']
  if not isinstance(cases, list) or not cases:
    raise ValueError('Batch requires cases')
  cleaned = []
  for case in cases:
    required = {'id', 'request', 'isolation'}
    allowed = required | {'process_group'}
    if not isinstance(case, dict) or not required <= set(case) or set(case) - allowed:
      raise ValueError('Invalid batch case')
    if not isinstance(case['id'], str) or not case['id'] or '/' in case['id'] or '\\' in case['id']:
      raise ValueError('Invalid batch case identity')
    if case['isolation'] not in ('demonstrated', 'unproven'):
      raise ValueError('Unknown process isolation state')
    if not isinstance(case['request'], str):
      raise ValueError('Batch request path is invalid')
    cleaned.append(dict(case))
  if len({case['id'] for case in cleaned}) != len(cleaned):
    raise ValueError('Batch case identities must be distinct')
  return resources, cleaned


def _clean_worktree():
  for args in (['git', 'diff', '--quiet'], ['git', 'diff', '--cached', '--quiet']):
    if subprocess.run(args, check=False).returncode:
      raise ValueError('Worktree candidate is mutable; commit or discard source changes before freezing')


def _freeze(request_path):
  value = canonical_request(read_json(request_path))
  if value['candidate_revision'] == 'worktree':
    _clean_worktree()
    value['candidate_revision'] = resolve_ref('HEAD')
  else:
    value['candidate_revision'] = resolve_ref(value['candidate_revision'])
  spec = value['case']['experiment']
  for key in ('baseline_controller', 'warmup_controller', 'limiter'):
    spec[key] = resolve_ref(spec[key])
  return canonical_request(value)


def _key(request):
  return hashlib.sha256(json.dumps(request, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _attempt(root):
  root.mkdir(parents=True, exist_ok=True)
  number = 1
  while (root / f'attempt-{number:03d}').exists():
    number += 1
  return root / f'attempt-{number:03d}'


def _valid_attempt(root, data_root):
  if not root.exists():
    return None, None
  errors = []
  for path in sorted(root.glob('attempt-*'), reverse=True):
    try:
      result = verify_bundle(path, data_root)
      return path, result
    except (OSError, ValueError, KeyError, TypeError) as error:
      if (path / 'result.json').exists():
        errors.append(str(error))
  return None, errors[-1] if errors else None


def _copy_complete(source, destination):
  shutil.copytree(source, destination)
  return read_json(destination / 'result.json')


def _case_worker(task):
  """Run one immutable case in a separate process when scheduler isolation permits."""
  case, frozen, manifest_root, data_root, output, cache_root, frozen_sources = task
  if source_snapshot() != frozen_sources:
    raise RuntimeError('Source drift after batch freeze')
  case_root = Path(output) / 'cases' / case['id']
  existing, existing_result = _valid_attempt(case_root, Path(data_root))
  if existing is not None:
    return {'id': case['id'], 'result': existing_result, 'reused': True, 'cache_invalidated': None, 'frozen_request': frozen}
  cache_case_root = Path(cache_root) / _key(frozen)
  cached, cache_result = _valid_attempt(cache_case_root, Path(data_root))
  destination = _attempt(case_root)
  if cached is not None:
    result = _copy_complete(cached, destination)
    return {'id': case['id'], 'result': result, 'reused': True, 'cache_invalidated': None, 'frozen_request': frozen}
  case_root.mkdir(parents=True, exist_ok=True)
  request_path = case_root / f'input-{destination.name}.json'
  write_json(request_path, frozen)
  result = evaluate(request_path, Path(data_root), destination)
  # Keep every failed/partial attempt. Completed adapter bundles are copied into a separate,
  # immutable cache attempt only after the verifier accepts all dependency identities.
  bundle = destination
  if result['status'] == 'completed_checks':
    verify_bundle(bundle, Path(data_root))
    _copy_complete(bundle, _attempt(cache_case_root))
  if source_snapshot() != frozen_sources:
    raise RuntimeError('Source drift during batch execution')
  return {'id': case['id'], 'result': result, 'reused': False, 'cache_invalidated': cache_result or existing_result,
          'frozen_request': frozen}


def _workers(resources, cases, requested, interrupted):
  configured = resources['max_workers'] if requested is None else requested
  if type(configured) is not int or configured <= 0:
    raise ValueError('Worker count must be positive')
  bound = min(configured, resources['max_workers'], max(1, (os_cpu_count() or 1)), resources['host_memory_mb'] // resources['worker_memory_mb'])
  if bound < 1:
    raise ValueError('Declared worker memory exceeds declared host memory')
  if interrupted is not None or any(case['isolation'] != 'demonstrated' for case in cases):
    return 1
  return bound


def os_cpu_count():
  import os

  return os.cpu_count()


def _host():
  return {'system': platform.system(), 'release': platform.release(), 'machine': platform.machine(),
          'python': platform.python_version()}


def evaluate_batch(manifest_path, data_root, output, *, cache_root=None, workers=None, stop_after_cases=None):
  """Evaluate independent common-adapter cases without changing canonical evidence."""
  manifest_path, data_root, output = Path(manifest_path), Path(data_root), Path(output)
  resources, cases = _manifest(manifest_path)
  if stop_after_cases is not None and (type(stop_after_cases) is not int or stop_after_cases < 0):
    raise ValueError('stop_after_cases must be a nonnegative integer')
  manifest_root = manifest_path.parent.resolve()
  frozen = []
  for case in cases:
    frozen.append((case, _freeze(_inside(manifest_root, case['request']))))
  output.mkdir(parents=True, exist_ok=True)
  cache_root = Path(cache_root) if cache_root is not None else output / 'cache'
  source_lock = source_snapshot()
  active_workers = _workers(resources, cases, workers, stop_after_cases)
  started, records = time.perf_counter(), []
  tasks = [(case, request, str(manifest_root), str(data_root), str(output), str(cache_root), source_lock) for case, request in frozen]
  if active_workers == 1:
    for task in tasks:
      if stop_after_cases is not None and len(records) >= stop_after_cases:
        break
      records.append(_case_worker(task))
  else:
    with ProcessPoolExecutor(max_workers=active_workers) as pool:
      submitted = [pool.submit(_case_worker, task) for task in tasks]
      for future in as_completed(submitted):
        records.append(future.result())
  records.sort(key=lambda row: row['id'])
  interrupted = stop_after_cases is not None and len(records) < len(tasks)
  canonical = {'format_version': FORMAT_VERSION, 'cases': [
    {'id': row['id'], 'status': row['result']['status'], 'result': row['result'], 'frozen_request': row['frozen_request']}
    for row in records
  ]}
  statuses = [row['result']['status'] for row in records]
  status = 'interrupted' if interrupted else ('completed_checks' if statuses and all(item == 'completed_checks' for item in statuses) else 'failed_check')
  elapsed = time.perf_counter() - started
  execution = {'host': _host(), 'workers': active_workers, 'elapsed_seconds': elapsed,
               'timing_kind': 'repeated' if any(row['reused'] for row in records) else 'cold',
               'throughput_cases_per_second': len(records) / elapsed if elapsed else None,
               'cases': [{'id': row['id'], 'reused': row['reused']} for row in records],
               'cache_invalidations': [{'id': row['id'], 'reason': row['cache_invalidated']} for row in records if row['cache_invalidated']]}
  sequence = len(list(output.glob('execution-*.json'))) + 1
  write_json(output / f'canonical-{sequence:03d}.json', canonical)
  write_json(output / f'execution-{sequence:03d}.json', execution)
  return {'format_version': FORMAT_VERSION, 'status': status, 'canonical': canonical, 'execution': execution}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--manifest', type=Path, required=True)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--cache-root', type=Path)
  parser.add_argument('--workers', type=int)
  args = parser.parse_args()
  result = evaluate_batch(args.manifest, args.data_root, args.output, cache_root=args.cache_root, workers=args.workers)
  print(f"{result['status']}: {args.output}")
  return 0 if result['status'] == 'completed_checks' else 1


if __name__ == '__main__':
  raise SystemExit(main())
