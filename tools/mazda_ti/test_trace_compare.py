"""Portable phase comparison keeps missing and inactive trace evidence visible."""

import json

import pytest

from .provenance import read_json, write_json
from .trace_compare import compare_trace_bundles
from .example_fixture import create_example
from .evaluate import evaluate


def test_native_integral_effect_is_not_reported_as_torque_counts(tmp_path):
  def native(integral, compensation):
    return [{'mono': index, 'active': True, 'integral_after_mps2': integral,
             'compensation_counts': compensation} for index in range(4)]
  reference = bundle(tmp_path, 'reference', native(.090, 68))
  control = bundle(tmp_path, 'control', native(.090, 68))
  candidate = bundle(tmp_path, 'candidate', native(.092, 30))
  result = compare_trace_bundles(reference, candidate, control, tmp_path / 'report')
  integral = result['metrics']['integral_after_mps2']
  assert integral['units'] == 'm/s^2'
  assert integral['worst_absolute_delta'] == pytest.approx(.002)
  assert result['metrics']['compensation_counts']['minimum_delta'] == -38
  assert result['metrics']['integral_counts']['availability'] == 'unavailable'
  assert result['metrics']['ti_command_counts']['availability'] == 'unavailable'


def bundle(root, name, rows, *, status='completed_checks', phases=None, provenance=None):
  path = root / name
  path.mkdir()
  write_json(path / 'result.json', {'format_version': 1, 'status': status, 'case_id': name,
                                    'scope': 'Recorded command evidence; not physical handling proof.'})
  write_json(path / 'trace.json', {'format_version': 1, 'capabilities': {'version': 1},
                                   'phase_windows': phases or [], 'provenance': provenance or {'source_sha256': 'a' * 64, 'input_sha256': 'b' * 64},
                                   'physical_observations': [{'text': 'rider observation remains separate', 'provenance': 'rider-note'}]})
  with (path / 'trace.jsonl').open('x', encoding='utf-8') as stream:
    for row in rows:
      stream.write(json.dumps(row) + '\n')
  return path


def rows(request, integral, compensation, command, *, active=True, stock=None):
  return [{'mono': float(i), 'active': active, 'request_counts': request + i, 'retained_reference_counts': request,
           'integral_counts': integral + i, 'compensation_counts': compensation, 'clipped': abs(command) >= 600,
           'constant_command': i in (1, 2), 'ti_command_counts': command + i,
           **({'stock_command_counts': stock + i} if stock is not None else {})} for i in range(4)]


def test_phase_report_keeps_favorable_unfavorable_missing_and_inactive_effects(tmp_path):
  phases = [{'name': 'entry', 'start': 0, 'end': 1, 'source': 'annotation', 'uncertainty': 'declared'},
            {'name': 'sustained', 'start': 1, 'end': 2, 'source': 'annotation', 'uncertainty': 'declared'},
            {'name': 'unwind', 'start': 2, 'end': 3, 'source': 'annotation', 'uncertainty': 'declared'},
            {'name': 'recovery', 'start': 3, 'end': 4, 'source': 'annotation', 'uncertainty': 'declared'}]
  reference = bundle(tmp_path, 'reference', rows(10, 1, 2, 20, stock=3), phases=phases)
  candidate = bundle(tmp_path, 'candidate', rows(8, -2, 1, 15, stock=2), phases=phases)
  control = bundle(tmp_path, 'control', rows(9, 0, 2, 18, stock=3), phases=phases)
  result = compare_trace_bundles(reference, candidate, control, tmp_path / 'report')

  assert result['format_version'] == 1
  assert result['status'] == 'completed_checks'
  assert result['phases'][0]['source'] == 'annotation'
  assert result['phase_effects']['entry']['candidate_minus_reference']['ti_command_counts']['worst_absolute_delta'] == 5
  assert result['command_effects']['candidate_minus_reference']['ti_command_counts']['worst_absolute_delta'] == 5
  assert result['command_effects']['candidate_minus_current_control']['ti_command_counts']['worst_absolute_delta'] == 3
  assert result['metrics']['stock_command_counts']['availability'] == 'supported'
  assert result['findings']['invariants'] == []
  assert result['findings']['physical_observations'][0]['provenance'] == 'rider-note'
  assert 'score' not in result
  assert 'Unfavorable' in (tmp_path / 'report/report.md').read_text()


def test_inferred_phases_and_optional_capabilities_never_fabricate_zero(tmp_path):
  reference = bundle(tmp_path, 'reference', rows(10, 1, 2, 20))
  candidate = bundle(tmp_path, 'candidate', [{'mono': 0.0, 'active': False}, {'mono': 1.0, 'active': False}])
  control = bundle(tmp_path, 'control', rows(9, 0, 2, 18), status='failed_check')
  result = compare_trace_bundles(reference, candidate, control, tmp_path / 'report')

  assert {row['name'] for row in result['phases']} == {'entry', 'sustained', 'unwind', 'recovery'}
  assert all(row['source'] == 'documented_chronological_rule' for row in result['phases'])
  assert result['metrics']['request_counts']['availability'] == 'inactive'
  assert result['metrics']['stock_command_counts']['availability'] == 'inactive'
  assert result['findings']['invariants'][0]['arm'] == 'current_control'
  assert result['status'] == 'failed_check'
  assert read_json(tmp_path / 'report/result.json') == result


def test_shifted_or_missing_sample_identity_and_tampered_input_cannot_compare(tmp_path):
  candidate = bundle(tmp_path, 'candidate', rows(10, 1, 2, 20))
  shifted = rows(10, 1, 2, 20)
  shifted[0]['mono'] = .5
  reference = bundle(tmp_path, 'reference', shifted)
  control = bundle(tmp_path, 'control', rows(10, 1, 2, 20), provenance={'source_sha256': 'c' * 64, 'input_sha256': 'd' * 64})
  result = compare_trace_bundles(reference, candidate, control, tmp_path / 'report')

  assert result['metrics']['ti_command_counts']['availability'] == 'unavailable'
  assert any(item['arm'] == 'current_control' for item in result['findings']['invariants'])
  assert result['provenance']['candidate']['input_sha256'] == 'b' * 64


def test_invalid_declared_phase_writes_failed_result(tmp_path):
  bad = [{'name': 'entry', 'start': 1, 'end': 0, 'source': 'annotation', 'uncertainty': 'declared'}]
  reference = bundle(tmp_path, 'reference', rows(10, 1, 2, 20), phases=bad)
  candidate = bundle(tmp_path, 'candidate', rows(10, 1, 2, 20), phases=bad)
  control = bundle(tmp_path, 'control', rows(10, 1, 2, 20), phases=bad)
  result = compare_trace_bundles(reference, candidate, control, tmp_path / 'report')
  assert result['status'] == 'failed_check'
  assert 'Invalid phase' in result['findings']['invariants'][0]['finding']


def test_content_hashes_and_unambiguous_references_keep_forged_metadata_unqualified(tmp_path):
  left = tmp_path / 'left'
  right = tmp_path / 'right'
  left.mkdir(); right.mkdir()
  reference = bundle(left, 'same-name', rows(10, 1, 2, 20))
  candidate = bundle(right, 'same-name', rows(10, 1, 2, 20))
  control = bundle(right, 'control', rows(10, 1, 2, 20))
  result = compare_trace_bundles(reference, candidate, control, tmp_path / 'report', evidence_root=tmp_path)

  assert result['status'] == 'completed_checks'
  assert result['identity_validation'] == 'content_bound_unqualified'
  assert result['provenance']['reference']['bundle'] != result['provenance']['candidate']['bundle']
  assert result['provenance']['candidate']['artifact_sha256']['trace.jsonl']


def test_verified_full_bundle_requires_an_explicit_retained_arm(tmp_path):
  request = create_example(tmp_path / 'raw')
  full = tmp_path / 'full'
  assert evaluate(request, request.parent, full)['status'] == 'completed_checks'
  result = compare_trace_bundles(full / 'candidate', full / 'candidate', full / 'candidate', tmp_path / 'report',
                                 data_root=request.parent, evidence_root=tmp_path)
  assert result['identity_validation'] == 'verified_full_bundle'
  assert result['status'] == 'completed_checks'
  assert result['provenance']['candidate']['source_identities'] == read_json(full / 'result.json')['source_identities']
  assert result['provenance']['candidate']['input_sha256'] == read_json(full / 'result.json')['input_sha256']
  forged = full / 'forged'
  forged.mkdir()
  (forged / 'trace.json').write_bytes((full / 'candidate/trace.json').read_bytes())
  (forged / 'trace.jsonl').write_bytes((full / 'candidate/trace.jsonl').read_bytes())
  with pytest.raises(ValueError, match='baseline or candidate'):
    compare_trace_bundles(forged, full / 'candidate', full / 'candidate', tmp_path / 'forged-report',
                          data_root=request.parent, evidence_root=tmp_path)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
def test_nonfinite_trace_values_are_rejected(tmp_path, value):
  reference = bundle(tmp_path, 'reference', rows(10, 1, 2, 20))
  candidate_rows = rows(10, 1, 2, 20)
  candidate_rows[0]['integral_counts'] = value
  candidate = bundle(tmp_path, 'candidate', candidate_rows)
  control = bundle(tmp_path, 'control', rows(10, 1, 2, 20))
  with pytest.raises(ValueError, match='finite'):
    compare_trace_bundles(reference, candidate, control, tmp_path / 'report')
