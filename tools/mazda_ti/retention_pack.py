"""Create summary-only retention packs and verified cleanup manifests."""

import argparse
import base64
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import sys


KEEP_NAMES = {'request.json', 'resolved-request.json', 'experiment.json', 'result.json',
              'report.md', 'execution.json', 'failure.json', 'stderr.txt'}
RAW_NAMES = {'rlog', 'qlog', 'fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc', 'qcamera.ts', 'bootlog'}
REPARSE_POINT = 0x400


def sha256(path):
  value = hashlib.sha256()
  with path.open('rb') as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b''):
      value.update(block)
  return value.hexdigest()


def overlaps(first, second):
  return first == second or first.is_relative_to(second) or second.is_relative_to(first)


def is_reparse(path):
  stat = path.lstat()
  return path.is_symlink() or bool(getattr(stat, 'st_file_attributes', 0) & REPARSE_POINT)


def walk_without_links(root):
  yield root
  for parent, directories, files in os.walk(root, followlinks=False):
    parent = Path(parent)
    for name in [*directories, *files]:
      yield parent / name


def validate(study_value, run_value, protected_values):
  supplied = [Path(study_value), Path(run_value), *(Path(value) for value in protected_values)]
  if any(not path.is_absolute() for path in supplied):
    raise ValueError('Study, run and protected roots must be absolute')
  lexical_study, lexical_run = supplied[0].absolute(), supplied[1].absolute()
  if '..' in supplied[1].parts or not lexical_run.is_relative_to(lexical_study):
    raise ValueError('Run root must be an absolute child of the study')

  cursor = lexical_study
  for part in lexical_run.relative_to(lexical_study).parts:
    cursor = cursor / part
    if (cursor.exists() or cursor.is_symlink()) and is_reparse(cursor):
      raise ValueError(f'Reparse point rejected: {cursor}')

  study, run = lexical_study.resolve(), lexical_run.resolve()
  if run == study or not run.is_relative_to(study) or not run.is_dir():
    raise ValueError('Resolved run root must be a child directory of the study')
  protected = [path.resolve() for path in supplied[2:]]
  if any(overlaps(run, path) for path in protected):
    raise ValueError('Run root overlaps a protected keep/raw root')
  recognized = (run / '.mazda-generated-run.json').is_file() or any(run.glob('cases/*/attempt-*/result.json'))
  if not recognized:
    raise ValueError('Run root is not marked or recognizable generated evaluation output')

  for path in walk_without_links(run):
    if is_reparse(path):
      raise ValueError(f'Reparse point rejected: {path}')
    name = path.name.lower()
    if path.is_file() and any(name == raw or name.startswith(raw + '.') for raw in RAW_NAMES):
      raise ValueError(f'Raw source filename rejected: {path.name}')
  return study, run


def inventory(run):
  files = sorted((path for path in run.rglob('*') if path.is_file()),
                 key=lambda path: path.relative_to(run).as_posix())
  return [{'path': path.relative_to(run).as_posix(), 'size_bytes': path.stat().st_size, 'sha256': sha256(path)}
          for path in files]


def retention_priority(item):
  path = Path(item['path'])
  name = path.name
  if name in {'request.json', 'resolved-request.json', 'experiment.json'} or name.startswith('input-attempt-'):
    return 0
  if name in {'report.md', 'failure.json', 'stderr.txt', 'execution.json'}:
    return 1
  if name == 'result.json' and not any(part in {'baseline', 'candidate'} for part in path.parts):
    return 2
  if name == 'result.json':
    return 3
  return None


def payload(study, run, protected, max_file_bytes, max_total_bytes):
  if any(type(value) is not int or value < 0 for value in (max_file_bytes, max_total_bytes)):
    raise ValueError('Retention budgets must be nonnegative integers')
  study, run = validate(study, run, protected)
  files = inventory(run)
  retained, omitted, used = {}, {}, 0
  selected = sorted((item for item in files if retention_priority(item) is not None),
                    key=lambda item: (retention_priority(item), item['path']))
  for item in selected:
    path = run / item['path']
    if item['size_bytes'] > max_file_bytes:
      omitted[item['path']] = 'exceeds_per_file_budget'
    elif used + item['size_bytes'] > max_total_bytes:
      omitted[item['path']] = 'exceeds_total_budget'
    else:
      retained[item['path']] = base64.b64encode(path.read_bytes()).decode('ascii')
      used += item['size_bytes']
  return {
    'format_version': 1, 'run_root': run.relative_to(study).as_posix(), 'inventory': files,
    'retained_files': retained, 'omitted_retained_metadata': omitted,
    'retention_budget': {'max_file_bytes': max_file_bytes, 'max_total_bytes': max_total_bytes,
                         'retained_source_bytes': used},
    'qualification': 'summary_only_requires_full_bundle_rebuild',
    'limitations': ['Full traces, prepared mappings and caches are inventory-only.',
                    'This pack cannot verify or qualify a release until the full bundle is rebuilt.',
                    'Freeing storage is a separate explicit action.'],
  }


def packed_bytes(value):
  raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
  output = io.BytesIO()
  with gzip.GzipFile(filename='', mode='wb', fileobj=output, mtime=0) as stream:
    stream.write(raw)
  return output.getvalue()


def absolute_destination(value, label):
  path = Path(value)
  if not path.is_absolute():
    raise ValueError(f'{label} must be absolute')
  return path.resolve()


def write_new(path, data):
  if not path.parent.is_dir():
    raise ValueError('Destination parent must already exist')
  with path.open('xb') as stream:
    stream.write(data)


def pack(study, run, destination, protected, max_file_bytes=262144, max_total_bytes=2097152):
  destination = absolute_destination(destination, 'Pack path')
  value = payload(study, run, protected, max_file_bytes, max_total_bytes)
  if destination.is_relative_to(Path(run).resolve()):
    raise ValueError('Pack must be outside the generated run root')
  write_new(destination, packed_bytes(value))
  return value


def verify_cleanup(study, run, pack_value, manifest_value, protected):
  pack_path = absolute_destination(pack_value, 'Pack path')
  manifest_path = absolute_destination(manifest_value, 'Manifest path')
  study, run = validate(study, run, protected)
  if pack_path.is_relative_to(run) or manifest_path.is_relative_to(run):
    raise ValueError('Pack and manifest must be outside the run root')
  with gzip.open(pack_path, 'rb') as stream:
    saved = json.loads(stream.read())
  if saved['run_root'] != run.relative_to(study).as_posix() or saved['inventory'] != inventory(run):
    raise ValueError('Run files changed after the retention pack was created')
  budget = saved['retention_budget']
  current = payload(study, run, protected, budget['max_file_bytes'], budget['max_total_bytes'])
  if pack_path.read_bytes() != packed_bytes(current):
    raise ValueError('Retention pack content differs from the deterministic current pack')
  plan = {'format_version': 1, 'status': 'verified_for_explicit_external_deletion', 'run_root': str(run),
          'pack_path': str(pack_path), 'pack_sha256': sha256(pack_path), 'inventory': saved['inventory'],
          'deletion_scope': 'Delete only the resolved run_root with a separate reviewed native operation.'}
  write_new(manifest_path, (json.dumps(plan, sort_keys=True, indent=2) + '\n').encode())
  return plan


def main(argv=None):
  parser = argparse.ArgumentParser()
  actions = parser.add_subparsers(dest='action', required=True)
  for name in ('pack', 'verify-cleanup'):
    command = actions.add_parser(name)
    command.add_argument('--study-root', required=True)
    command.add_argument('--run-root', required=True)
    command.add_argument('--pack', required=True)
    command.add_argument('--protected-root', action='append', default=[])
    if name == 'pack':
      command.add_argument('--max-retained-file-bytes', required=True, type=int)
      command.add_argument('--max-retained-total-bytes', required=True, type=int)
    else:
      command.add_argument('--manifest', required=True)
  args = parser.parse_args(argv)
  try:
    if args.action == 'pack':
      pack(args.study_root, args.run_root, args.pack, args.protected_root,
           args.max_retained_file_bytes, args.max_retained_total_bytes)
    else:
      verify_cleanup(args.study_root, args.run_root, args.pack, args.manifest, args.protected_root)
  except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
    print(f'retention {args.action} failed: {error}', file=sys.stderr)
    return 2
  print(f'retention {args.action}: complete')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
