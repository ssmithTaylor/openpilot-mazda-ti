# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Identity auditor failure cases through serialized production schemas."""

import json
from pathlib import Path
import subprocess
import sys

import pytest

from cereal import log
from .audit_diagnostics import INPUTS, audit, read_events


def event(service, mono):
  result = log.Event.new_message(logMonoTime=mono, valid=True)
  result.init(service)
  return result


def recording(active=True):
  events = [event(service, 100 + index) for index, service in enumerate(INPUTS)]
  controller = event('controlsState', 200)
  state = controller.controlsState.lateralControlState.init('torqueState')
  state.active = active
  d = state.mazdaDiagnostics
  d.version = 1
  for index, snapshot in enumerate(d.init('inputs', len(INPUTS))):
    snapshot.service = INPUTS[index]
    snapshot.logMonoTime = 100 + index
    snapshot.valid = snapshot.alive = snapshot.frequencyOk = snapshot.checksPassed = snapshot.seen = True
  command = event('carControl', 210)
  command.carControl.controlsStateMonoTime = 200
  output = event('carOutput', 230)
  output.carOutput.applySequence = 5
  output.carOutput.appliedCarControlMonoTime = 210
  output.carOutput.appliedAtMonoTime = 220
  output.carOutput.appliedCarControlChecksPassed = True
  output.carOutput.mazdaDiagnostics.version = 1
  output.carOutput.mazdaDiagnostics.tiLimited = 317
  return events + [controller, command, output]


def serialized(events):
  data = []
  for e in events:
    e.clear_write_flag()
    data.append(e.to_bytes())
  return log.Event.read_multiple_bytes(b''.join(data))


def test_exact_chain_crosses_segment_boundary_and_is_order_independent(tmp_path):
  events = recording()
  paths = [tmp_path / 'before', tmp_path / 'scored']
  paths[0].write_bytes(b''.join(e.to_bytes() for e in events[:-1]))
  paths[1].write_bytes(events[-1].to_bytes())
  result = audit(read_events(paths), 225, 240)
  assert result['active_identity_coverage_complete']
  assert result['rows'][0]['inputs']['carOutput'] == 106
  assert result == audit(read_events(paths[::-1]), 225, 240)
  missing = audit(read_events(paths[1:]), 225, 240)
  assert missing['summary']['issue_counts'] == {'missing_carControl': 1}
  assert not missing['identity_coverage_complete']


def test_repeated_apply_retains_old_identity_despite_new_command():
  events = recording()
  newer_command = event('carControl', 235)
  newer_command.carControl.controlsStateMonoTime = 200
  repeated = event('carOutput', 240)
  repeated.carOutput = events[-1].carOutput
  repeated.valid = False
  result = audit(serialized(events + [newer_command, repeated]), 225, 250)
  assert result['active_identity_coverage_complete']
  assert [r['carControl'] for r in result['rows']] == [210, 210]
  assert result['summary']['unique_applies'] == 1
  assert result['summary']['repeated_apply_publications'] == 1
  assert result['summary']['invalid_output_publications'] == 1
  repeated.carOutput.mazdaDiagnostics.tiLimited = 318
  failed = audit(serialized(events + [repeated]), 225, 250)
  assert failed['summary']['issue_counts'] == {'repeated_apply_changed_payload': 1}
  boundary = audit(serialized(events + [repeated]), 240, 250)
  assert boundary['summary']['issue_counts'] == {'repeated_apply_changed_payload': 1}


@pytest.mark.parametrize(('changed', 'reason'), [
  ('missing_input', 'missing_input_event'), ('future_input', 'future_input_identity'),
  ('missing_controller', 'missing_controlsState'), ('old_controller', 'absent_or_unsupported_controller_diagnostics'),
  ('old_output', 'absent_or_unsupported_actuator_diagnostics'), ('validity', 'input_validity_mismatch'),
  ('service_set', 'invalid_input_service_set'), ('unseen', 'unseen_input'),
  ('apply_order', 'invalid_apply_identity_or_order'),
])
def test_incomplete_evidence_is_not_qualified(changed, reason):
  events = recording()
  d = events[-3].controlsState.lateralControlState.torqueState.mazdaDiagnostics
  if changed == 'missing_input':
    events.pop(0)
  elif changed == 'future_input':
    d.inputs[0].logMonoTime = 201
  elif changed == 'missing_controller':
    events.pop(-3)
  elif changed == 'old_controller':
    d.version = 0
  elif changed == 'old_output':
    events[-1].carOutput.mazdaDiagnostics.version = 0
  elif changed == 'validity':
    d.inputs[0].valid = False
  elif changed == 'service_set':
    d.inputs[0].service = 'modelV2'
  elif changed == 'unseen':
    d.inputs[0].seen = False
  elif changed == 'apply_order':
    events[-1].carOutput.appliedAtMonoTime = 209
  result = audit(serialized(events), 225, 240)
  assert not result['identity_coverage_complete']
  assert reason in result['summary']['issue_counts']


@pytest.mark.parametrize(('sequence', 'reason'), [(3, 'apply_sequence_reset'), (7, 'missing_apply_publications')])
def test_sequence_discontinuities_are_explicit(sequence, reason):
  events = recording()
  output = event('carOutput', 250)
  output.carOutput = events[-1].carOutput
  output.carOutput.applySequence = sequence
  output.carOutput.appliedAtMonoTime = 245
  result = audit(serialized(events + [output]), 225, 260)
  assert not result['identity_coverage_complete']
  assert reason in result['summary']['issue_counts']


def test_health_is_separate_from_identity_and_inactive_is_not_active_evidence():
  events = recording(active=False)
  d = events[-3].controlsState.lateralControlState.torqueState.mazdaDiagnostics
  d.inputs[0].checksPassed = d.inputs[0].alive = False
  result = audit(serialized(events), 225, 240)
  assert result['identity_coverage_complete']
  assert not result['active_identity_coverage_complete']
  assert result['summary']['input_snapshots_failed_checks'] == 1
  assert not audit(serialized(events), 300, 400)['identity_coverage_complete']


def test_old_default_output_and_duplicate_identity_fail_closed():
  old = audit(serialized([event('carOutput', 230)]), 225, 240)
  assert not old['identity_coverage_complete']
  assert old['summary']['issue_counts'] == {'absent_or_unsupported_actuator_diagnostics': 1}
  assert old['summary']['unique_applies'] == 0
  assert 'applies_with_failed_checks_publications' not in old['summary']
  events = recording()
  with pytest.raises(ValueError, match='duplicate event identity'):
    audit(serialized(events + [events[0]]), 225, 240)


def test_cli_relocation_is_byte_identical_and_existing_output_is_preserved(tmp_path):
  data = b''.join(e.to_bytes() for e in recording())
  root = Path(__file__).resolve().parents[2]
  reports = []
  for name in ('first', 'relocated'):
    folder = tmp_path / name
    folder.mkdir()
    (folder / 'rlog').write_bytes(data)
    report = folder / 'report.json'
    command = [sys.executable, '-m', 'tools.mazda_ti.audit_diagnostics', '--data-root', str(folder),
               '--rlogs', 'rlog', '--start-ns', '225', '--end-ns', '240', '--output', str(report)]
    result = subprocess.run(command, cwd=root, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    reports.append(report.read_bytes())
    refused = subprocess.run(command, cwd=root, capture_output=True, text=True)
    assert refused.returncode != 0
    assert report.read_bytes() == reports[-1]
  assert reports[0] == reports[1]
  document = json.loads(reports[0])
  assert document['active_identity_coverage_complete']
  assert len(document['input_sha256']['rlog']) == 64
  assert 'tools/mazda_ti/audit_diagnostics.py' in document['repository_sources']
