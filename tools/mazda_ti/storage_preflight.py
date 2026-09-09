"""Read-only storage preflight for an output/cache workspace pair."""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys


def _target(value):
  if not isinstance(value, (str, os.PathLike)):
    raise ValueError('Path must be a string or path-like value')
  path = Path(value)
  if not path.is_absolute():
    raise ValueError('Paths must be absolute')
  if os.name == 'nt':
    reserved = {'CON','PRN','AUX','NUL',*(f'COM{i}' for i in range(1,10)),*(f'LPT{i}' for i in range(1,10))}
    for part in path.parts[1:]:
      if any(char in part for char in '<>:"|?*') or part.endswith((' ', '.')) or part.split('.')[0].upper() in reserved:
        raise ValueError(f'Invalid Windows path component: {part}')
  try:
    path = path.resolve(strict=False)
  except (OSError, RuntimeError, ValueError) as error:
    raise ValueError(f'Invalid path: {value}') from error
  if path.exists() and not path.is_dir():
    raise ValueError('Storage target must be a directory or nonexistent descendant')
  return path


def _ancestor(path):
  current = path
  while not current.exists():
    parent = current.parent
    if parent == current:
      raise ValueError(f'No existing ancestor for path: {path}')
    current = parent
  if not current.is_dir():
    raise ValueError(f'Nearest existing ancestor is not a directory: {current}')
  return current


def _probe(ancestor):
  volume = f'dev:{ancestor.stat().st_dev}'
  return volume, shutil.disk_usage(ancestor).free


def assess(output, cache, required_workspace_bytes, headroom_bytes, *, probe=_probe):
  if type(required_workspace_bytes) is not int or required_workspace_bytes < 0:
    raise ValueError('Required workspace bytes must be a nonnegative integer')
  if type(headroom_bytes) is not int or headroom_bytes < 0:
    raise ValueError('Headroom bytes must be a nonnegative integer')
  targets = [('output', _target(output)), ('cache', _target(cache))]
  if targets[0][1] == targets[1][1] or targets[0][1].is_relative_to(targets[1][1]) or targets[1][1].is_relative_to(targets[0][1]):
    raise ValueError('Output and cache paths must be distinct and non-overlapping')

  grouped = {}
  order = []
  bound_targets = []
  for role, path in targets:
    ancestor = _ancestor(path)
    volume, free = probe(ancestor)
    if not isinstance(volume, str) or not volume or type(free) is not int or free < 0:
      raise ValueError('Storage probe returned an invalid volume or free-byte count')
    if volume not in grouped:
      grouped[volume] = {'free':free, 'copies':0}
      order.append(volume)
    else:
      grouped[volume]['free'] = min(grouped[volume]['free'], free)
    grouped[volume]['copies'] += 1
    bound_targets.append({'role':role, 'path':str(path), 'nearest_existing_ancestor':str(ancestor),
                          'volume':f'volume-{order.index(volume)+1}'})

  volumes = []
  for number, volume in enumerate(order, 1):
    item = grouped[volume]
    required = item['copies'] * required_workspace_bytes + headroom_bytes
    volumes.append({'id':f'volume-{number}', 'free_bytes':item['free'], 'required_bytes':required,
                    'workspace_copies':item['copies'], 'sufficient':item['free'] >= required})
  ready = all(item['sufficient'] for item in volumes)
  return {'format_version':1, 'status':'ready' if ready else 'insufficient_space',
          'inputs':{'required_workspace_bytes':required_workspace_bytes, 'headroom_bytes':headroom_bytes,
                    'targets':bound_targets}, 'volumes':volumes,
          'scope':'Point-in-time free-space check from caller estimates; no reservation or precise size prediction.'}


def _write_report(path, value):
  with path.open('x', encoding='utf-8', newline='\n') as stream:
    json.dump(value, stream, sort_keys=True, indent=2)
    stream.write('\n')


def main(argv=None):
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', required=True)
  parser.add_argument('--cache', required=True)
  parser.add_argument('--required-workspace-bytes', required=True, type=int)
  parser.add_argument('--headroom-bytes', required=True, type=int)
  parser.add_argument('--report', required=True)
  try:
    args = parser.parse_args(argv)
    report_path = _target(args.report)
    if report_path.exists() or not report_path.parent.is_dir():
      raise ValueError('Report path must be a new file under an existing directory')
    result = assess(args.output, args.cache, args.required_workspace_bytes, args.headroom_bytes)
    if any(report_path.is_relative_to(Path(target['path'])) for target in result['inputs']['targets']):
      raise ValueError('Report path must be outside output and cache targets')
    _write_report(report_path, result)
  except (OSError, ValueError) as error:
    print(f'storage preflight failed: {error}', file=sys.stderr)
    return 2
  if result['status'] != 'ready':
    print('storage preflight: insufficient_space', file=sys.stderr)
    return 1
  print('storage preflight: ready')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
