# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Fail closed on incomplete or changed preparation/baseline evidence."""

import copy
import json
from pathlib import Path
import tempfile

import pytest

from . import provenance
from .run import REPLAY_FILES, validate_baseline


@pytest.fixture
def evidence(request, monkeypatch):
  temporary = tempfile.TemporaryDirectory()
  request.addfinalizer(temporary.cleanup)
  root = Path(temporary.name)
  monkeypatch.setattr(provenance, 'ROOT', root)
  sources = {}
  for name in provenance.REPLAY_SOURCE_FILES:
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'fixture\n')
    sources[name] = provenance.sha256(path)
  result_path = root / 'baseline/result.json'
  result_path.parent.mkdir()
  outputs = {}
  for name in REPLAY_FILES:
    path = result_path.parent / name
    path.write_bytes(b'fixture artifact\n')
    outputs[name] = provenance.sha256(path)
  runtime = {'fixture_runtime': 1}
  result = dict(
    format_version=2,
    stage='replayed',
    variant='baseline',
    qualification='exact_recorded_commands',
    command_bounds_pass=True,
    frames=2,
    sends=2,
    preparation_sha256='prepared',
    history='earliest',
    environment=runtime,
    repository_sources=sources,
    output_sha256=outputs,
  )
  result_path.write_text(json.dumps(result))
  return root, result_path, result, runtime


def test_verified_baseline_and_relocated_root_are_accepted(evidence):
  _, path, result, runtime = evidence
  assert validate_baseline(path, 'prepared', 'earliest', runtime) == result


@pytest.mark.parametrize('name', ['selfdrive/controls/lib/pid.py', 'selfdrive/car/mazda/lateral_plant.py', 'selfdrive/car/__init__.py', 'cereal/log.capnp'])
def test_dependency_change_invalidates_baseline(evidence, name):
  root, path, _, runtime = evidence
  (root / name).write_bytes(b'changed after replay\n')
  with pytest.raises(ValueError, match='Source identity changed'):
    validate_baseline(path, 'prepared', 'earliest', runtime)


@pytest.mark.parametrize(
  'field,value',
  [
    ('qualification', 'failed_baseline'),
    ('variant', 'candidate'),
    ('command_bounds_pass', False),
    ('frames', 0),
    ('sends', 0),
    ('history', 'latest'),
    ('preparation_sha256', 'other'),
    ('environment', {}),
  ],
)
def test_wrong_or_unqualified_baseline_is_rejected(evidence, field, value):
  _, path, result, runtime = evidence
  changed = copy.deepcopy(result)
  changed[field] = value
  path.write_text(json.dumps(changed))
  with pytest.raises(ValueError):
    validate_baseline(path, 'prepared', 'earliest', runtime)


def test_changed_baseline_trace_is_rejected(evidence):
  _, path, _, runtime = evidence
  (path.parent / 'trace-sends.json').write_bytes(b'changed send\n')
  with pytest.raises(ValueError, match='Artifact changed'):
    validate_baseline(path, 'prepared', 'earliest', runtime)


@pytest.mark.parametrize('field', ['repository_sources', 'output_sha256'])
def test_incomplete_manifest_is_rejected(evidence, field):
  _, path, result, runtime = evidence
  result[field].pop(next(iter(result[field])))
  path.write_text(json.dumps(result))
  with pytest.raises(ValueError):
    validate_baseline(path, 'prepared', 'earliest', runtime)


@pytest.mark.parametrize('payload', ['{"value": NaN}', '{"value": 1e999}'])
def test_nonfinite_json_is_rejected(evidence, payload):
  _, path, _, _ = evidence
  path.write_text(payload)
  with pytest.raises(ValueError, match='Non-finite'):
    provenance.read_json(path)


def test_data_path_cannot_escape_root(evidence):
  root, _, _, _ = evidence
  with pytest.raises(ValueError, match='escapes'):
    provenance.under(root, '../outside/rlog')
