"""Geographic selection preserves direction, source coverage and annotation scope."""

import numpy as np
import pytest
import hashlib
import math
import json

from .geographic_match import _reference_coverage, _request, find_candidates, run
from .provenance import write_json


def poses():
    t = np.arange(-5., 26., .05)
    return np.column_stack([100 + t, 35 + 10*t/111320, np.full(len(t), -84.), np.full(len(t), 10.),
                            np.zeros(len(t)), np.ones(len(t)), np.full(len(t), 3.)])


@pytest.mark.parametrize('case,accepted,reason', [
    ('same', True, None), ('shifted', True, None), ('reverse', False, 'opposite_or_mismatched_direction'),
    ('reverse_heading_lie', False, 'anchor_order'), ('tail_missing', False, 'missing_geographic_endpoint'),
    ('truncated', False, 'temporal_boundary'), ('gap', False, 'pose_gap'),
    ('unhealthy', False, 'localization_invalid'), ('parallel_far', False, None), ('joined', True, None)])
def test_geographic_candidates(case, accepted, reason):
    reference = poses()
    query = reference.copy()
    if case == 'shifted':
        query[:, 0] += 100
    elif case in ('reverse', 'reverse_heading_lie'):
        query[:, 1] = query[::-1, 1]
        query[:, 3] = -10 if case == 'reverse' else 10
    elif case == 'tail_missing':
        query = query[query[:, 0] < 112]
    elif case == 'truncated':
        query = query[(query[:, 0] >= 100) & (query[:, 0] <= 120)]
    elif case == 'gap':
        query = query[(query[:, 0] < 108) | (query[:, 0] > 109)]
    elif case == 'unhealthy':
        query[(query[:, 0] > 108) & (query[:, 0] < 109), 5] = 0
    elif case == 'parallel_far':
        query[:, 2] += .01
    elif case == 'joined':
        query = np.vstack([query[:300], query[300:]])
    matches = find_candidates(query, reference, [100, 105, 110, 115], 100, 120)
    assert any(m['accepted_geographic_candidate'] for m in matches) == accepted
    if reason:
        assert any(reason in m['reasons'] for m in matches)


def test_nearest_endpoints_and_anchors_cannot_stitch_two_passes():
    reference = poses()
    query = reference.copy()
    t = query[:, 0]
    first, returning = t < 105, (t >= 105) & (t < 106)
    distance = np.where(first, (t-95)*25-50, np.where(returning, 200-(t-105)*250, (t-106)*25-50))
    east = np.where(returning, 100, np.where(first, np.where((distance > 20) & (distance < 180), 10, 0),
                                           np.where((distance > 20) & (distance < 180), 0, 20)))
    query[:, 1] = 35 + distance/111320
    query[:, 2] = -84 + east/(111320*math.cos(math.radians(35)))
    query[:, 3] = np.where(returning, -250, 25)
    matches = find_candidates(query, reference, [100, 105, 110, 115], 100, 120)
    assert matches and not any(m['accepted_geographic_candidate'] for m in matches)
    assert any('anchor_endpoint_order' in m['reasons'] for m in matches)


def test_large_finite_position_uncertainty_is_visible_not_silently_certified():
    reference = poses()
    reference[:, 6] = 10000
    match = find_candidates(reference, reference, [100, 105, 110, 115], 100, 120)[0]
    assert match['maximum_position_std_norm_m'] == 10000


def request():
    return {'format_version': 1, 'case_id': 'same-road',
            'reference': {'rlogs': ['reference--0/rlog'], 'start': 100, 'end': 120, 'anchors': [100, 105, 110, 115]},
            'candidate': {'rlogs': ['candidate--0/rlog']}}


def write_poses(path, rows):
    from cereal import log

    path.parent.mkdir(parents=True, exist_ok=True)
    raw = []
    for t, lat, lon, north, east, valid, std in rows:
        event = log.Event.new_message(logMonoTime=round(t*1e9), valid=True)
        q = event.init('liveLocationKalman')
        q.positionGeodetic.value, q.positionGeodetic.valid = [float(lat), float(lon), 0.0], bool(valid)
        q.velocityNED.value = [float(north), float(east), 0.0]
        q.velocityNED.valid = bool(valid)
        q.positionECEF.std = [float(std), 0.0, 0.0]
        q.inputsOK = q.posenetOK = q.sensorsOK = True
        raw.append(event.to_bytes())
    path.write_bytes(b''.join(raw))


def test_raw_pipeline_repeats_after_relocation_and_does_not_inherit_annotations(tmp_path):
    value = request()
    input_path = tmp_path/'request.json'
    write_json(input_path, value)
    reference = poses()
    candidate = reference.copy()
    candidate[:, 0] += 100
    for root in (tmp_path/'first-data', tmp_path/'relocated-data'):
        write_poses(root/'reference--0/rlog', reference)
        write_poses(root/'candidate--0/rlog', candidate)
    first = run(input_path, tmp_path/'first-data', tmp_path/'first')
    second = run(input_path, tmp_path/'relocated-data', tmp_path/'second')
    assert first == second
    assert first['status'] == 'completed_search' and first['qualification'] == 'geographic_candidates'
    assert first['accepted_candidates'] == 1
    assert 199.9 < first['matches'][0]['bounds'][0] < 200.1
    assert 'Rider/contact status remains unreviewed' in (tmp_path/'first/report.md').read_text()
    with pytest.raises(FileExistsError):
        run(input_path, tmp_path/'first-data', tmp_path/'first')
    value['candidate']['rider_confirmed'] = True
    with pytest.raises(ValueError):
        _request(value)


def test_request_hash_binds_parsed_snapshot_even_if_path_changes(tmp_path, monkeypatch):
    from . import geographic_match as module

    path = tmp_path/'request.json'
    value = request()
    write_json(path, value)
    expected = hashlib.sha256(path.read_bytes()).hexdigest()
    for role in ('reference', 'candidate'):
        write_poses(tmp_path/(role+'--0/rlog'), poses())
    reader = module._poses

    def changing_reader(paths):
        changed = request()
        changed['case_id'] = 'changed-after-snapshot'
        path.write_text(json.dumps(changed), encoding='utf-8')
        return reader(paths)

    monkeypatch.setattr(module, '_poses', changing_reader)
    result = run(path, tmp_path, tmp_path/'result')
    assert result['request']['case_id'] == value['case_id']
    assert result['request_sha256'] == expected != hashlib.sha256(path.read_bytes()).hexdigest()


def test_reference_coverage_cannot_interpolate_across_missing_or_invalid_data():
    rows = poses()
    for candidate in (rows[rows[:, 0] > 101], rows[(rows[:, 0] < 108) | (rows[:, 0] > 109)]):
        with pytest.raises(ValueError, match='Reference'):
            _reference_coverage(candidate, 100, 120)
    rows[100, 5] = 0
    with pytest.raises(ValueError, match='Reference'):
        _reference_coverage(rows, 100, 120)


def test_explicit_finite_ordered_request_and_single_route_required():
    for change in ({'anchors': [100, 110, 105]}, {'start': float('nan')}, {'anchors': [100, 105, 121]},
                   {'rlogs': ['one--0/rlog', 'other--0/rlog']}):
        value = request()
        value['reference'].update(change)
        with pytest.raises(ValueError):
            _request(value)
