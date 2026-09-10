# ruff: noqa: TID251
"""Instrumented evaluation uses serialized nested identities, never publication proximity."""

import json
import os
from pathlib import Path

import pytest

from .audit_diagnostics import INPUTS
from .example_fixture import create_example
from .evaluate import evaluate, verify_bundle
from .provenance import read_json, sha256
from .instrumented import component_snapshot
from cereal import log

REVISION = 'ad14961a29f59635d5f657a43000ca2a1ade62fb'


def instrumented_fixture(root):
  """Zero-command analytic fixture: both paths, bypass, re-entry and inactive reset."""
  request_path = create_example(root)
  request = read_json(request_path)
  case, events, latest = request['case'], [], {}
  case.update(method='instrumented', history='exact')
  spec = case['experiment']
  spec.update(baseline_controller=REVISION, warmup_controller=REVISION, limiter=REVISION, fpcs_sample_at='diagnostic')
  request['candidate_revision'] = REVISION
  raw = root / spec['rlogs'][0]
  for reader in log.Event.read_multiple_bytes(raw.read_bytes()):
    e = reader.as_builder()
    kind, mono = e.which(), int(e.logMonoTime)
    index = (mono - 1_000_000_000) // 10_000_000
    active, ti = index not in (17, 18), index not in (13, 14, 15)
    if kind == 'initData':
      e.initData.gitCommit = REVISION
    if kind == 'modelV2':
      fp = log.Event.new_message(logMonoTime=mono - 2, valid=True, frogpilotCarState={'tiActive': ti})
      location = log.Event.new_message(logMonoTime=mono - 1, valid=True, liveLocationKalman={})
      events.extend([fp, location])
      latest.update(frogpilotCarState=mono - 2, liveLocationKalman=mono - 1)
    if kind == 'controlsState':
      state = e.controlsState.lateralControlState.torqueState
      state.active = active
      state.plantState = 1 if not ti else 7 if 17 <= index <= 22 else 3
      d = state.mazdaDiagnostics
      d.version, d.settings = 1, 16
      d.kp, d.ki, d.kd = .800000011920929, .20000000298023224, .06
      d.modelContextNow = d.cameraContextNow = mono
      for s, name in zip(d.init('inputs', len(INPUTS)), INPUTS, strict=True):
        s.service, s.logMonoTime = name, latest[name]
        s.valid = s.alive = s.frequencyOk = s.checksPassed = s.seen = True
    if kind == 'carControl':
      e.carControl.controlsStateMonoTime = latest.get('controlsState', 790_000_000)
      e.carControl.latActive = active
    if kind == 'carOutput':
      co = e.carOutput
      co.applySequence = index + 2 if index >= 0 else 1
      co.appliedCarControlMonoTime, co.appliedAtMonoTime = latest['carControl'], mono - 1
      co.appliedCarControlChecksPassed = True
      d = co.mazdaDiagnostics
      d.version, d.latActive, d.tiAllowed = 1, active, ti
      d.tiMax, d.tiDeltaUp, d.tiDeltaDown = 600, 10, 15
      d.tiDriverAllowance, d.tiDriverMultiplier, d.tiDeltaUpKnee, d.tiDeltaUpHigh = 15, 40, 630, 9
    latest[kind] = mono
    events.append(e)
  raw.write_bytes(b''.join(e.to_bytes() for e in events))
  case['input_sha256'] = {spec['rlogs'][0]: sha256(raw)}
  request_path.write_text(json.dumps(request))
  return request_path


def test_complete_instrumented_fixture_qualifies_both_paths_and_reports_transitions(tmp_path):
  request = instrumented_fixture(tmp_path / 'raw')
  result = evaluate(request, request.parent, tmp_path / 'bundle')
  assert result['status'] == 'completed_checks', result['findings']
  assert result['comparison']['changed_sends'] == 0
  assert result['comparison']['stock']['changed_sends'] == 0
  assert result['comparison']['availability'] == {'ti_loss': 1, 'ti_reentry': 1, 'stock_fallback_applies': 3, 'inactive_applies': 2}
  assert verify_bundle(tmp_path / 'bundle', request.parent) == result
  rows = [json.loads(line) for line in (tmp_path / 'bundle/candidate/trace.jsonl').read_text().splitlines()]
  active, inactive = [row for row in rows if row['active']], [row for row in rows if not row['active']]
  assert active and inactive
  prior = [row['previous_applied_feedback'] for row in rows if row['previous_applied_feedback'] is not None]
  assert prior and rows[0]['previous_applied_feedback'] is None
  assert all(p['source_controller_mono'] <= p['source_request_mono'] <= p['applied_mono'] <=
             p['publication_mono'] < row['mono'] for row in rows
             if (p := row['previous_applied_feedback']) is not None)
  assert any(p['stock_fallback'] and p['selected_counts'] == p['stock_counts'] for p in prior)
  assert any(p['ti_selected'] and p['selected_counts'] == p['ti_counts'] for p in prior)
  assert any(not p['active'] and p['selected_counts'] == 0 for p in prior)
  assert all(row['replay_components']['command'] == 0 and row['compensation_counts'] == 0 for row in active)
  assert all(row['replay_components'] is None and row['recorded_components'] is None and
             'compensation_counts' not in row and 'consumed_feedback_steer' not in row for row in inactive)
  # Stock is required evidence even while the recorded TI path supplies feedback.
  (tmp_path / 'bundle/candidate/trace-stock-sends.json').write_text('{}')
  with pytest.raises(ValueError):
    verify_bundle(tmp_path / 'bundle', request.parent)


def test_component_snapshot_preserves_native_signed_values_and_missing_state():
  event = log.Event.new_message()
  diagnostics = event.init('controlsState').lateralControlState.init('torqueState').mazdaDiagnostics
  diagnostics.version = 1
  diagnostics.inverseCommand, diagnostics.frictionCompensation = -501.25, -68.75
  diagnostics.integralBefore, diagnostics.integralAfter = .0919, .0920
  diagnostics.freezeReasons = 3
  # Exercise the actual serialized schema, including explicit zeros/defaults.
  with log.Event.from_bytes(event.to_bytes()) as reader:
    d = reader.controlsState.lateralControlState.torqueState.mazdaDiagnostics
    values = component_snapshot(d, True)
    assert values['inverseCommand'] == -501.25 and values['frictionCompensation'] == -68.75
    assert values['integralBefore'] == .0919 and values['integralAfter'] == .0920
    assert values['freezeReasons'] == 3
    assert component_snapshot(d, False) is None
  diagnostics.version = 0
  assert component_snapshot(diagnostics, True) is None
  diagnostics.version = 2
  assert component_snapshot(diagnostics, True) is None


@pytest.mark.parametrize('corruption', ['missing_input', 'version', 'validity', 'sequence', 'stock_previous', 'ti_previous',
                                       'controller_output', 'command_pair', 'active_pair', 'serialized_output', 'inactive_output'])
def test_corrupted_serialized_evidence_cannot_score_a_candidate(tmp_path, corruption):
  request_path = instrumented_fixture(tmp_path / 'raw')
  request = read_json(request_path)
  name = request['case']['experiment']['rlogs'][0]
  raw = request_path.parent / name
  events = [e.as_builder() for e in log.Event.read_multiple_bytes(raw.read_bytes())]
  controller_mono = 1_173_000_000 if corruption == 'inactive_output' else 1_143_000_000
  control = next(e.controlsState.lateralControlState.torqueState for e in events
                 if e.which() == 'controlsState' and e.logMonoTime == controller_mono)
  output = next(e.carOutput for e in events if e.which() == 'carOutput' and e.logMonoTime == 1_146_000_000)
  command = next(e.carControl for e in events if e.which() == 'carControl' and e.logMonoTime == controller_mono + 1_000_000)
  if corruption == 'missing_input':
    control.mazdaDiagnostics.inputs[0].logMonoTime += 1
  elif corruption == 'version':
    control.mazdaDiagnostics.version = 0
  elif corruption == 'validity':
    control.mazdaDiagnostics.inputs[0].valid = False
  elif corruption == 'sequence':
    output.applySequence += 1
  elif corruption == 'stock_previous':
    output.mazdaDiagnostics.stockPrevious = 1
  elif corruption == 'ti_previous':
    output.mazdaDiagnostics.tiPrevious = 1
  elif corruption == 'controller_output':
    control.mazdaDiagnostics.command = 3
  elif corruption == 'command_pair':
    command.actuators.steer = .0001  # Same rounded integer, but not the controller's serialized request.
  elif corruption == 'active_pair':
    command.latActive = output.mazdaDiagnostics.latActive = False
  elif corruption in ('serialized_output', 'inactive_output'):
    # Keep the controller/apply pair consistent and integer histories unchanged.
    control.output = command.actuators.steer = .25 if corruption == 'inactive_output' else .0001
  raw.write_bytes(b''.join(e.to_bytes() for e in events))
  request['case']['input_sha256'][name] = sha256(raw)
  request_path.write_text(json.dumps(request))
  result = evaluate(request_path, request_path.parent, tmp_path / 'bundle')
  assert result['status'] == 'failed_check', result
  assert result['comparison'] is None
  assert not (tmp_path / 'bundle/candidate').exists()


def test_exact_inputs_cross_adjacent_segments_without_nearest_fallback(tmp_path):
  request_path = instrumented_fixture(tmp_path / 'raw')
  request = read_json(request_path)
  first_name = request['case']['experiment']['rlogs'][0]
  first = request_path.parent / first_name
  events = list(log.Event.read_multiple_bytes(first.read_bytes()))
  initial = next(e for e in events if e.which() == 'initData')
  second_name = 'synthetic--1/rlog'
  second = request_path.parent / second_name
  second.parent.mkdir()
  first.write_bytes(b''.join(e.as_builder().to_bytes() for e in events if e.logMonoTime < 1_150_000_000))
  second.write_bytes(initial.as_builder().to_bytes() + b''.join(e.as_builder().to_bytes() for e in events if e.logMonoTime >= 1_150_000_000))
  case = request['case']
  case['experiment']['rlogs'] = [first_name, second_name]
  case['input_sha256'] = {first_name: sha256(first), second_name: sha256(second)}
  request_path.write_text(json.dumps(request))
  assert evaluate(request_path, request_path.parent, tmp_path / 'complete')['status'] == 'completed_checks'
  case['experiment']['rlogs'] = [second_name]
  case['input_sha256'] = {second_name: sha256(second)}
  request_path.write_text(json.dumps(request))
  missing = evaluate(request_path, request_path.parent, tmp_path / 'missing')
  assert missing['status'] == 'failed_check'
  assert missing['comparison'] is None


@pytest.mark.skipif(not os.environ.get('MAZDA_EVAL_DATA_ROOT'), reason='Private recorded transition not supplied')
def test_recorded_ti_loss_stock_fallback_and_recovery(tmp_path):
  request = Path(__file__).with_name('examples') / 'cutout-evaluation.json'
  data = Path(os.environ['MAZDA_EVAL_DATA_ROOT'])
  result = evaluate(request, data, tmp_path / 'bundle')
  assert result['status'] == 'completed_checks', result['findings']
  assert result['comparison']['sends'] == 6999
  assert result['comparison']['changed_sends'] == result['comparison']['stock']['changed_sends'] == 0
  assert result['comparison']['availability'] == {'ti_loss': 1, 'ti_reentry': 1, 'stock_fallback_applies': 185, 'inactive_applies': 3968}
  assert verify_bundle(tmp_path / 'bundle', data) == result
