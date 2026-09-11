"""Explicit, offline-testable collection of raw Mazda evidence into an inventory.

This module only reads a supplied source tree and writes a supplied destination.  It
does not import Params, contact a vehicle, or participate in evaluation/replay.
"""

import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import stat
import time

from .provenance import read_json, sha256, under, write_json


FORMAT_VERSION = 1
DEFAULT_TRANSFER_LIMIT_BYTES_PER_SECOND = 10_000_000
MAX_TRANSFER_LIMIT_BYTES_PER_SECOND = 10_000_000
TRANSFER_CHUNK_BYTES = 256 * 1024
SEGMENT_NAME = re.compile(r'(?P<route>[A-Za-z0-9_-]+)--(?P<segment>\d+)$')
FULL_GIT_REVISION = re.compile(r'[0-9a-f]{40}\Z')
WHITELISTED_SETTINGS = frozenset({
  'TiSteerMax', 'TiSteerDeltaUp', 'TiSteerDeltaDown', 'TiSteerDriverAllowance', 'TiSteerDriverMultiplier',
  'TiSteerDeltaUpKnee', 'TiSteerDeltaUpHigh', 'LatDamping', 'LatCommitSetpoint', 'LatFrictionComp',
  'LatOutputFilter', 'LatNoFrictionRelay', 'TorqueInterceptorEnabled', 'SteerKP',
})
BOOLEAN_SETTINGS = frozenset({'LatOutputFilter', 'LatNoFrictionRelay', 'TorqueInterceptorEnabled'})


class TransferInterrupted(RuntimeError):
  """A controlled interruption leaves the verified prefix in ``rlog.part``."""


class Pacer:
  """Bound copied bytes without changing the source or relying on source metadata."""

  def __init__(self, bytes_per_second, monotonic=time.monotonic, sleep=time.sleep):
    if bytes_per_second <= 0:
      raise ValueError('transfer limit must be positive')
    if bytes_per_second > MAX_TRANSFER_LIMIT_BYTES_PER_SECOND:
      raise ValueError('transfer limit must not exceed 10 MB/s')
    self.bytes_per_second = bytes_per_second
    self.monotonic = monotonic
    self.sleep = sleep
    self.started_at = monotonic()
    self.bytes_copied = 0

  def copied(self, count):
    self.bytes_copied += count
    delay = self.started_at + self.bytes_copied / self.bytes_per_second - self.monotonic()
    if delay > 0:
      self.sleep(delay)


def metadata_for(source_root, route):
  path = source_root / f'{route}.metadata.json'
  reject_link_or_reparse(path, 'metadata')
  if not path.exists():
    return {}
  metadata = read_json(path)
  if not isinstance(metadata, dict):
    raise ValueError(f'Metadata for {route} must be an object')
  return metadata


def whitelisted_settings(metadata):
  raw = metadata.get('settings')
  if not isinstance(raw, dict):
    return {}, sorted(WHITELISTED_SETTINGS), ['settings']
  settings = {}
  invalid = []
  for name in WHITELISTED_SETTINGS:
    value = raw.get(name)
    numeric = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    valid = (isinstance(value, bool) or numeric) if name in BOOLEAN_SETTINGS else numeric
    if valid:
      settings[name] = value
    elif name in raw:
      invalid.append(name)
  missing = sorted(WHITELISTED_SETTINGS - set(settings))
  return settings, missing, invalid


def discover(source_root):
  """Create a standalone inventory from a fixture or immutable source snapshot."""
  source_root = Path(source_root).resolve()
  if not source_root.is_dir():
    raise ValueError(f'Source root does not exist: {source_root}')
  grouped = {}
  for directory in sorted(source_root.iterdir()):
    match = SEGMENT_NAME.fullmatch(directory.name)
    rlog = directory / 'rlog'
    if match:
      reject_link_or_reparse(directory, 'segment directory')
      reject_link_or_reparse(rlog, 'rlog')
    if match and directory.is_dir() and rlog.is_file():
      grouped.setdefault(match['route'], []).append((int(match['segment']), rlog))
  routes = []
  for route, segments in grouped.items():
    metadata = metadata_for(source_root, route)
    settings, missing_settings, invalid_settings = whitelisted_settings(metadata)
    expected = metadata.get('expected_segments')
    valid_expected_segments = isinstance(expected, list) and all(
      isinstance(value, int) and not isinstance(value, bool) and value >= 0 for value in expected
    )
    expected_segments = sorted(set(expected)) if valid_expected_segments else None
    present = {segment for segment, _ in segments}
    missing_segments = sorted(set(expected_segments) - present) if expected_segments is not None else []
    unknown_evidence = []
    if expected_segments is None:
      unknown_evidence.append('expected_segments')
    revision = metadata.get('source_revision')
    if revision is None:
      unknown_evidence.append('source_revision')
    elif not isinstance(revision, str) or not FULL_GIT_REVISION.fullmatch(revision):
      unknown_evidence.append('source_revision_invalid')
    unknown_evidence.extend(f'settings.{name}' for name in invalid_settings)
    routes.append({
      'route_id': route,
      'source_revision': revision if isinstance(revision, str) and FULL_GIT_REVISION.fullmatch(revision) else None,
      'settings': settings,
      'missing_settings': missing_settings,
      'invalid_settings': sorted(invalid_settings),
      'unknown_evidence': sorted(unknown_evidence),
      'missing_segments': missing_segments,
      'completeness': 'incomplete' if missing_segments else ('unknown' if expected_segments is None else 'complete'),
      'segments': [{
        'segment': segment,
        'source_path': f'{route}--{segment}/rlog',
        'bytes': path.stat().st_size,
        'sha256': sha256(path),
        'transfer_status': 'not_requested',
        'integrity': 'unknown',
      } for segment, path in sorted(segments)],
    })
  return {'format_version': FORMAT_VERSION, 'kind': 'mazda_ti_ingestion_inventory', 'routes': routes}


def include_adjacent(segments, adjacent):
  return {segment + offset for segment in segments for offset in range(-adjacent, adjacent + 1) if segment + offset >= 0}


def selected_segments(route, selected, adjacent):
  known = {entry['segment'] for entry in route['segments']}
  requested = include_adjacent(selected.get(route['route_id'], []), adjacent)
  return known & requested, sorted(requested - known)


def is_reparse_point(path):
  """Recognize Windows junctions and other reparse points, even when not symlinks."""
  try:
    attributes = getattr(os.lstat(path), 'st_file_attributes', 0)
  except FileNotFoundError:
    return False
  return bool(attributes & getattr(stat, 'FILE_ATTRIBUTE_REPARSE_POINT', 0))


def reject_link_or_reparse(path, kind):
  if path.is_symlink():
    raise ValueError(f'Rejecting symlinked {kind}: {path}')
  if is_reparse_point(path):
    raise ValueError(f'Rejecting Windows reparse-point {kind}: {path}')


def safe_child(root, name):
  """Reject a child path that escapes through traversal, a symlink, or a junction."""
  root = Path(root).resolve()
  under(root, name)
  child = root
  for part in Path(name).parts:
    child /= part
    reject_link_or_reparse(child, 'transfer path')
  return child


def copy_with_resume(source, destination, limit, stop_after_bytes=None):
  """Copy one immutable file, resuming a matching prefix and verifying its hash."""
  expected = sha256(source)
  destination.parent.mkdir(parents=True, exist_ok=True)
  partial = destination.with_name(destination.name + '.part')
  for path in (source, destination, partial):
    reject_link_or_reparse(path, 'transfer path')
  resumed_from = partial.stat().st_size if partial.exists() else 0
  if resumed_from > source.stat().st_size:
    return 'corrupt_partial', resumed_from
  if resumed_from and partial.read_bytes() != source.read_bytes()[:resumed_from]:
    return 'corrupt_partial', resumed_from
  pacer = Pacer(limit)
  copied_this_run = 0
  source_size = source.stat().st_size
  with source.open('rb') as input_stream, partial.open('ab') as output_stream:
    input_stream.seek(resumed_from)
    while block := input_stream.read(TRANSFER_CHUNK_BYTES):
      read_bytes = len(block)
      if stop_after_bytes is not None:
        remaining = stop_after_bytes - copied_this_run
        if remaining <= 0:
          raise TransferInterrupted('controlled transfer interruption')
        block = block[:remaining]
      output_stream.write(block)
      copied_this_run += len(block)
      pacer.copied(len(block))
      if stop_after_bytes is not None and copied_this_run >= stop_after_bytes and (len(block) < read_bytes or input_stream.tell() < source_size):
        raise TransferInterrupted('controlled transfer interruption')
  if sha256(partial) != expected:
    return 'corrupt_transfer', resumed_from
  partial.replace(destination)
  return 'complete', resumed_from


def collect_inventory(
  inventory, source_root, destination_root, selected=None, adjacent=0, transfer_limit=DEFAULT_TRANSFER_LIMIT_BYTES_PER_SECOND, stop_after_bytes=None,
):
  """Explicitly collect selected inventory files; evaluation never calls this function."""
  if inventory.get('format_version') != FORMAT_VERSION or inventory.get('kind') != 'mazda_ti_ingestion_inventory':
    raise ValueError('Unsupported ingestion inventory')
  if not selected:
    raise ValueError('Collection requires at least one explicit segment selection')
  if adjacent < 0:
    raise ValueError('adjacent must not be negative')
  Pacer(transfer_limit)
  result = copy.deepcopy(inventory)
  source_root = Path(source_root).resolve()
  destination_root = Path(destination_root).resolve()
  known_routes = {route['route_id'] for route in result['routes']}
  missing_routes = [
    {'route_id': route_id, 'segments': sorted(include_adjacent(segments, adjacent))}
    for route_id, segments in sorted(selected.items()) if route_id not in known_routes
  ]
  if missing_routes:
    result['requested_missing_routes'] = missing_routes
  remaining_stop = stop_after_bytes
  for route in result['routes']:
    want, missing = selected_segments(route, selected, adjacent)
    if missing:
      route['requested_missing_segments'] = missing
    for entry in route['segments']:
      if entry['segment'] not in want:
        continue
      source = safe_child(source_root, entry['source_path'])
      destination = safe_child(destination_root, entry['source_path'])
      entry['transfer_limit_bytes_per_second'] = transfer_limit
      if not source.is_file() or sha256(source) != entry['sha256']:
        entry['transfer_status'] = 'corrupt_source'
        entry['integrity'] = 'corrupt'
        continue
      if destination.exists():
        entry['transfer_status'] = 'duplicate' if sha256(destination) == entry['sha256'] else 'corrupt_destination'
        entry['integrity'] = 'verified' if entry['transfer_status'] == 'duplicate' else 'corrupt'
        continue
      before = (destination.with_name(destination.name + '.part').stat().st_size if destination.with_name(destination.name + '.part').exists() else 0)
      try:
        status, resumed = copy_with_resume(source, destination, transfer_limit, remaining_stop)
      except TransferInterrupted as error:
        entry['transfer_status'] = 'partial'
        entry['integrity'] = 'incomplete'
        set_collection_status(result)
        error.inventory = result
        raise
      entry['transfer_status'] = status
      entry['integrity'] = 'verified' if status == 'complete' else 'corrupt'
      entry['resumed_from_bytes'] = resumed
      if remaining_stop is not None:
        remaining_stop -= max(0, source.stat().st_size - before)
  set_collection_status(result)
  return result


def set_collection_status(inventory):
  for route in inventory['routes']:
    statuses = {entry['transfer_status'] for entry in route['segments']}
    route['collection_status'] = 'corrupt' if any(status.startswith('corrupt') for status in statuses) else (
      'complete'
      if route['completeness'] == 'complete' and statuses <= {'complete', 'duplicate'} and not route.get('requested_missing_segments')
      else 'incomplete'
    )


def parse_selection(values):
  selected = {}
  for value in values:
    route, separator, segment = value.rpartition(':')
    if not separator or not segment.isdigit():
      raise ValueError('segments must use ROUTE:SEGMENT')
    selected.setdefault(route, []).append(int(segment))
  return selected or None


def parser():
  result = argparse.ArgumentParser(description='Create or explicitly collect a Mazda raw-evidence inventory; never evaluates or changes a vehicle.')
  commands = result.add_subparsers(dest='command', required=True)
  discover_parser = commands.add_parser('discover', help='read an offline source snapshot and write a versioned inventory')
  discover_parser.add_argument('--source-root', type=Path, required=True)
  discover_parser.add_argument('--output', type=Path, required=True)
  collect_parser = commands.add_parser('collect', help='explicitly transfer selected inventory files from a supplied source root')
  collect_parser.add_argument('--inventory', type=Path, required=True)
  collect_parser.add_argument('--source-root', type=Path, required=True)
  collect_parser.add_argument('--destination-root', type=Path, required=True)
  collect_parser.add_argument('--output', type=Path, required=True)
  collect_parser.add_argument('--segment', action='append', required=True, help='ROUTE:SEGMENT; collection never defaults to bulk transfer')
  collect_parser.add_argument('--adjacent', type=int, default=0, help='include this many neighboring segments for warmup/identity coverage')
  collect_parser.add_argument('--transfer-limit-bytes-per-second', type=int, default=DEFAULT_TRANSFER_LIMIT_BYTES_PER_SECOND)
  collect_parser.add_argument('--stop-after-bytes', type=int, help=argparse.SUPPRESS)
  return result


def main(argv=None):
  args = parser().parse_args(argv)
  if args.command == 'discover':
    output = discover(args.source_root)
  else:
    try:
      output = collect_inventory(read_json(args.inventory), args.source_root, args.destination_root, parse_selection(args.segment), args.adjacent,
                                 args.transfer_limit_bytes_per_second, args.stop_after_bytes)
    except TransferInterrupted as error:
      write_json(args.output, error.inventory)
      raise SystemExit('Collection interrupted; the partial inventory is available for resume.') from error
  write_json(args.output, output)


if __name__ == '__main__':
  main()
