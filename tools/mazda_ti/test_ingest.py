"""Offline contract tests for the Mazda evidence ingestion inventory."""

import json
import os
from pathlib import Path
import subprocess

import pytest

from .ingest import Pacer, TransferInterrupted, collect_inventory, discover, main


REVISION = 'a' * 40
ALLOWED = {'TiSteerMax': 600, 'TorqueInterceptorEnabled': True}


def test_pacer_sleeps_to_enforce_the_declared_transfer_rate():
  readings = iter([0.0, 0.0])
  delays = []

  pacer = Pacer(100, monotonic=lambda: next(readings), sleep=delays.append)
  pacer.copied(10)

  assert delays == [0.1]

  with pytest.raises(ValueError, match='10 MB/s'):
    Pacer(10_000_001)


def write_segment(source, route, segment, payload):
  path = source / f'{route}--{segment}' / 'rlog'
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_bytes(payload)
  return path


def write_metadata(source, route, **values):
  (source / f'{route}.metadata.json').write_text(json.dumps(values), encoding='utf-8')


def test_discover_command_writes_the_standalone_inventory_contract(tmp_path):
  source = tmp_path / 'source'
  write_segment(source, 'route-a', 0, b'zero')
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0], settings=ALLOWED)
  output = tmp_path / 'inventory.json'

  main(['discover', '--source-root', str(source), '--output', str(output)])

  assert json.loads(output.read_text(encoding='utf-8'))['kind'] == 'mazda_ti_ingestion_inventory'


def test_discovery_is_versioned_redacts_unapproved_settings_and_preserves_missing_evidence(tmp_path):
  source = tmp_path / 'source'
  write_segment(source, 'route-a', 0, b'zero')
  write_segment(source, 'route-a', 2, b'two')
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0, 1, 2], settings={**ALLOWED, 'ApiToken': 'private'})

  inventory = discover(source)

  assert inventory['format_version'] == 1
  assert inventory['kind'] == 'mazda_ti_ingestion_inventory'
  route = inventory['routes'][0]
  assert route['source_revision'] == REVISION
  assert route['completeness'] == 'incomplete'
  assert route['missing_segments'] == [1]
  assert route['settings'] == ALLOWED
  assert 'ApiToken' not in json.dumps(inventory)
  assert 'LatDamping' in route['missing_settings']


def test_discovery_marks_short_revisions_and_boolean_numeric_settings_invalid(tmp_path):
  source = tmp_path / 'source'
  write_segment(source, 'route-a', 0, b'zero')
  write_metadata(source, 'route-a', source_revision='too-short', expected_segments=[0], settings={'TiSteerMax': True, 'TorqueInterceptorEnabled': True})

  route = discover(source)['routes'][0]

  assert route['source_revision'] is None
  assert 'source_revision_invalid' in route['unknown_evidence']
  assert route['invalid_settings'] == ['TiSteerMax']
  assert 'TiSteerMax' in route['missing_settings']


def test_collection_includes_adjacent_segments_records_10mb_limit_and_skips_verified_duplicates(tmp_path):
  source = tmp_path / 'source'
  for segment in range(3):
    write_segment(source, 'route-a', segment, bytes([segment]) * 8)
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0, 1, 2], settings=ALLOWED)
  destination = tmp_path / 'raw'

  first = collect_inventory(discover(source), source, destination, selected={'route-a': [1]}, adjacent=1)
  entries = first['routes'][0]['segments']
  assert [entry['segment'] for entry in entries] == [0, 1, 2]
  assert {entry['transfer_status'] for entry in entries} == {'complete'}
  assert {entry['transfer_limit_bytes_per_second'] for entry in entries} == {10_000_000}

  second = collect_inventory(first, source, destination, selected={'route-a': [1]}, adjacent=1)
  assert {entry['transfer_status'] for entry in second['routes'][0]['segments']} == {'duplicate'}


def test_requested_adjacent_segment_that_is_unavailable_stays_visible_as_missing(tmp_path):
  source = tmp_path / 'source'
  write_segment(source, 'route-a', 1, b'one')
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0, 1, 2], settings=ALLOWED)

  result = collect_inventory(discover(source), source, tmp_path / 'raw', selected={'route-a': [1]}, adjacent=1)

  route = result['routes'][0]
  assert route['requested_missing_segments'] == [0, 2]
  assert route['collection_status'] == 'incomplete'


def test_collection_requires_selected_segments_and_preserves_unknown_route_coverage(tmp_path):
  source = tmp_path / 'source'
  write_segment(source, 'route-a', 0, b'zero')
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0], settings=ALLOWED)
  inventory = discover(source)

  with pytest.raises(ValueError, match='explicit segment'):
    collect_inventory(inventory, source, tmp_path / 'raw')

  with pytest.raises(SystemExit):
    main(['collect', '--inventory', 'missing.json', '--source-root', str(source), '--destination-root', str(tmp_path / 'raw'), '--output', 'out.json'])

  result = collect_inventory(inventory, source, tmp_path / 'raw', selected={'route-missing': [7]}, adjacent=1)
  assert result['requested_missing_routes'] == [{'route_id': 'route-missing', 'segments': [6, 7, 8]}]


def test_interrupted_transfer_leaves_a_partial_file_that_a_second_collection_resumes(tmp_path):
  source = tmp_path / 'source'
  payload = bytes(range(256)) * 32
  write_segment(source, 'route-a', 0, payload)
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0], settings=ALLOWED)
  destination = tmp_path / 'raw'

  with pytest.raises(TransferInterrupted):
    collect_inventory(discover(source), source, destination, selected={'route-a': [0]}, stop_after_bytes=1000)
  partial = destination / 'route-a--0' / 'rlog.part'
  assert partial.exists()
  assert partial.stat().st_size == 1000

  resumed = collect_inventory(discover(source), source, destination, selected={'route-a': [0]})
  entry = resumed['routes'][0]['segments'][0]
  assert entry['transfer_status'] == 'complete'
  assert entry['resumed_from_bytes'] == 1000
  assert (destination / 'route-a--0' / 'rlog').read_bytes() == payload


def test_changed_source_after_discovery_is_reported_as_corrupt_without_copying_it(tmp_path):
  source = tmp_path / 'source'
  original = write_segment(source, 'route-a', 0, b'known-good')
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0], settings=ALLOWED)
  discovered = discover(source)
  original.write_bytes(b'changed')

  result = collect_inventory(discovered, source, tmp_path / 'raw', selected={'route-a': [0]})

  entry = result['routes'][0]['segments'][0]
  assert entry['transfer_status'] == 'corrupt_source'
  assert entry['integrity'] == 'corrupt'
  assert not (tmp_path / 'raw' / 'route-a--0' / 'rlog').exists()


def test_discovery_rejects_symlinked_rlog_and_metadata(tmp_path):
  source = tmp_path / 'source'
  target = tmp_path / 'target'
  target.write_bytes(b'zero')
  rlog = source / 'route-a--0' / 'rlog'
  rlog.parent.mkdir(parents=True)
  try:
    rlog.symlink_to(target)
  except OSError as error:
    pytest.skip(f'Symlinks unavailable in this environment: {error}')

  with pytest.raises(ValueError, match='symlink'):
    discover(source)

  rlog.unlink()
  rlog.write_bytes(b'zero')
  metadata_target = tmp_path / 'metadata.json'
  metadata_target.write_text('{}', encoding='utf-8')
  (source / 'route-a.metadata.json').symlink_to(metadata_target)
  with pytest.raises(ValueError, match='symlink'):
    discover(source)


def test_collection_rejects_partial_symlink_that_escapes_destination(tmp_path):
  source = tmp_path / 'source'
  write_segment(source, 'route-a', 0, b'zero')
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0], settings=ALLOWED)
  destination = tmp_path / 'raw'
  partial = destination / 'route-a--0' / 'rlog.part'
  partial.parent.mkdir(parents=True)
  try:
    partial.symlink_to(tmp_path / 'outside')
  except OSError as error:
    pytest.skip(f'Symlinks unavailable in this environment: {error}')

  with pytest.raises(ValueError, match='escapes|symlink'):
    collect_inventory(discover(source), source, destination, selected={'route-a': [0]})


def test_collection_rejects_destination_symlink_that_escapes_destination(tmp_path):
  source = tmp_path / 'source'
  write_segment(source, 'route-a', 0, b'zero')
  write_metadata(source, 'route-a', source_revision=REVISION, expected_segments=[0], settings=ALLOWED)
  destination = tmp_path / 'raw'
  final = destination / 'route-a--0' / 'rlog'
  final.parent.mkdir(parents=True)
  try:
    final.symlink_to(tmp_path / 'outside')
  except OSError as error:
    pytest.skip(f'Symlinks unavailable in this environment: {error}')

  with pytest.raises(ValueError, match='escapes|symlink'):
    collect_inventory(discover(source), source, destination, selected={'route-a': [0]})


def make_junction(link, target):
  if os.name != 'nt':
    pytest.skip('Windows junction fixture')
  result = subprocess.run(['cmd', '/c', 'mklink', '/J', str(link), str(target)], check=False, capture_output=True, text=True)
  if result.returncode:
    pytest.skip(f'Junctions unavailable in this environment: {result.stderr}')


def test_discovery_rejects_windows_junction_segment_and_metadata(tmp_path):
  target = tmp_path / 'target'
  write_segment(target, 'route-a', 0, b'zero')
  source = tmp_path / 'source'
  source.mkdir()
  make_junction(source / 'route-a--0', target / 'route-a--0')

  with pytest.raises(ValueError, match='reparse'):
    discover(source)

  (source / 'route-a--0').rmdir()
  write_segment(source, 'route-a', 0, b'zero')
  make_junction(source / 'route-a.metadata.json', target)
  with pytest.raises(ValueError, match='reparse'):
    discover(source)
