"""Identify geographic candidates for an explicitly pinned road window.

Promoted from the local post-drive matcher. Geography selects the comparison;
controller requests, lane position and outcomes never select the alignment.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

from .audit_diagnostics import read_events
from .provenance import environment, finish_sources, sha256, source_snapshot, under, write_json


def _request(value):
    if set(value) != {'format_version', 'case_id', 'reference', 'candidate'} or type(value['format_version']) is not int or value['format_version'] != 1:
        raise ValueError('Unsupported geographic request')
    if not isinstance(value['case_id'], str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', value['case_id']):
        raise ValueError('Invalid case identity')
    if set(value['reference']) != {'rlogs', 'start', 'end', 'anchors'} or set(value['candidate']) != {'rlogs'}:
        raise ValueError('Explicit reference window, anchors and two rlog lists are required')
    for role in ('reference', 'candidate'):
        paths = value[role]['rlogs']
        if not isinstance(paths, list) or not paths or any(not isinstance(path, str) for path in paths) or len(set(paths)) != len(paths):
            raise ValueError('Require nonempty distinct rlog paths per role')
        routes = {Path(path).parent.name.rsplit('--', 1)[0] for path in paths}
        if len(routes) != 1:
            raise ValueError('Each role must contain one route')
    ref = value['reference']
    if (not isinstance(ref['anchors'], list) or len(ref['anchors']) < 3 or
        not all(type(v) in (int, float) and math.isfinite(v) for v in [ref['start'], ref['end'], *ref['anchors']]) or
        not 0 < ref['start'] <= ref['anchors'][0] < ref['anchors'][-1] <= ref['end'] or
        any(b <= a for a, b in zip(ref['anchors'], ref['anchors'][1:]))):
        raise ValueError('Require ordered finite anchors within a positive reference window')
    return value


def _poses(paths):
    rows = {}
    for event in read_events(paths):
        if event.which() != 'liveLocationKalman':
            continue
        q = event.liveLocationKalman
        position, velocity = q.positionGeodetic.value, q.velocityNED.value
        if len(position) < 2 or len(velocity) < 2:
            continue
        row = [event.logMonoTime / 1e9, position[0], position[1], velocity[0], velocity[1],
               float(event.valid and q.positionGeodetic.valid and q.velocityNED.valid and q.inputsOK and q.posenetOK and q.sensorsOK),
               float(np.linalg.norm(q.positionECEF.std))]
        key = int(event.logMonoTime)
        if key in rows and not np.array_equal(rows[key], row, equal_nan=True):
            raise ValueError('Conflicting duplicate localization identity')
        rows[key] = row
    return np.asarray([row for _, row in sorted(rows.items())], dtype=float).reshape(-1, 7)


def _reference_coverage(pose, start, end):
    if len(pose) < 2 or pose[0, 0] >= start or pose[-1, 0] <= end:
        raise ValueError('Reference lacks surrounding localization coverage')
    first, last = np.searchsorted(pose[:, 0], [start, end])
    window = pose[max(0, first - 1):last + 1]
    if (not np.isfinite(window).all() or np.any(window[:, 5] != 1) or
        np.any(np.abs(window[:, 1]) > 90) or np.any(np.abs(window[:, 2]) > 180) or np.max(np.diff(window[:, 0])) > .2):
        raise ValueError('Reference localization is invalid or contains a gap')
    return window


def xy(ll, origin):
    return (np.asarray(ll) - origin) * [111320.0, 111320.0 * math.cos(math.radians(origin[0]))]

def find_candidates(pose, reference_pose, anchors, start, end):
    if len(pose) < 2:
        return []
    origin = np.array([np.interp(anchors[1], reference_pose[:, 0], reference_pose[:, j]) for j in [1, 2]])
    target = np.array([[np.interp(t, reference_pose[:, 0], reference_pose[:, j]) for j in [1, 2]] for t in anchors])
    endpoints = np.array([[np.interp(t, reference_pose[:, 0], reference_pose[:, j]) for j in [1, 2]] for t in [start, end]])
    good = np.isfinite(pose).all(axis=1) & (np.abs(pose[:, 1]) <= 90) & (np.abs(pose[:, 2]) <= 180)
    pose = pose[good]
    if len(pose) < 2:
        return []
    points = xy(pose[:, 1:3], origin)
    distance = np.linalg.norm(points, axis=1)
    near = np.flatnonzero(distance < 25.0)
    if not len(near):
        return []
    groups = np.split(near, np.flatnonzero(np.diff(pose[near, 0]) > 3.0) + 1)
    out = []
    ref_velocity = np.array([np.interp(anchors[1], reference_pose[:, 0], reference_pose[:, j]) for j in [3, 4]])
    ref_heading = math.atan2(ref_velocity[1], ref_velocity[0])
    for group in groups:
        center = int(group[np.argmin(distance[group])])
        t = pose[center, 0]
        window = pose[(pose[:, 0] >= t - 30) & (pose[:, 0] <= t + 30)]
        wxy = xy(window[:, 1:3], origin)
        distances = np.linalg.norm(wxy[:, None, :] - xy(target, origin)[None, :, :], axis=2)
        nearest = np.argmin(distances, axis=0)
        # Anchor 1 defines this proximity group; do not borrow an identical location
        # from another nearby pass in the surrounding search window.
        nearest[1] = np.flatnonzero(window[:, 0] == t)[0]
        times = window[nearest, 0]
        errors = distances[nearest, np.arange(len(anchors))]
        endpoint_dist = np.linalg.norm(wxy[:, None, :] - xy(endpoints, origin)[None, :, :], axis=2)
        endpoint_indices = np.argmin(endpoint_dist, axis=0)
        bounds = window[endpoint_indices, 0]
        heading = math.atan2(pose[center, 4], pose[center, 3])
        angle = abs((math.degrees(heading - ref_heading) + 180) % 360 - 180)
        reasons = []
        if angle >= 45:
            reasons.append('opposite_or_mismatched_direction')
        if not np.all(np.diff(times) > 0):
            reasons.append('anchor_order')
        if max(errors) > 30:
            reasons.append('anchor_distance')
        if bounds[1] <= bounds[0]:
            reasons.append('endpoint_order')
        chronology = list(zip([start, *anchors, end], [bounds[0], *times, bounds[1]], strict=True))
        if any((r2 > r1 and t2 <= t1) or (r2 == r1 and t2 != t1)
               for (r1, t1), (r2, t2) in zip(chronology, chronology[1:])):
            reasons.append('anchor_endpoint_order')
        if max(endpoint_dist[endpoint_indices, [0, 1]]) > 30:
            reasons.append('missing_geographic_endpoint')
        if min(bounds) <= window[0, 0] + 0.1 or max(bounds) >= window[-1, 0] - 0.1:
            reasons.append('temporal_boundary')
        inside = window[(window[:, 0] >= min(bounds)) & (window[:, 0] <= max(bounds))]
        if len(inside) < 2 or max(np.diff(inside[:, 0]), default=math.inf) > 0.2:
            reasons.append('pose_gap')
        if len(inside) and (not np.all(inside[:, 5] == 1)):
            reasons.append('localization_invalid')
        out.append(dict(accepted_geographic_candidate=not reasons, reasons=reasons, center_mono=float(t), central_distance_m=float(distance[center]), heading_difference_deg=float(angle), anchor_times=times.tolist(), anchor_distances_m=errors.tolist(), bounds=bounds.tolist(), endpoint_distances_m=endpoint_dist[endpoint_indices, [0, 1]].tolist(),
                        maximum_position_std_norm_m=float(np.max(inside[:, 6])) if len(inside) else None))
    return out


def run(request_path, data_root, output):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    payload = Path(request_path).read_bytes()
    request = _request(json.loads(payload))
    before, runtime = source_snapshot(), environment()
    poses, hashes = {}, {}
    for role in ('reference', 'candidate'):
        names = request[role]['rlogs']
        paths = [under(Path(data_root), name) for name in names]
        hashes[role] = {name: sha256(path) for name, path in zip(names, paths, strict=True)}
        poses[role] = _poses(paths)
        if any(sha256(path) != hashes[role][name] for name, path in zip(names, paths, strict=True)):
            raise ValueError('Raw localization input changed during matching')
    ref = request['reference']
    reference_window = _reference_coverage(poses['reference'], ref['start'], ref['end'])
    matches = find_candidates(poses['candidate'], poses['reference'], ref['anchors'], ref['start'], ref['end'])
    result = {'format_version': 1, 'status': 'completed_search', 'case_id': request['case_id'],
              'qualification': 'geographic_candidates', 'request': request, 'matches': matches,
              'accepted_candidates': sum(match['accepted_geographic_candidate'] for match in matches),
              'input_sha256': hashes, 'request_sha256': hashlib.sha256(payload).hexdigest(),
              'reference_maximum_position_std_norm_m': float(np.max(reference_window[:, 6])),
              'repository_sources': finish_sources(before), 'environment': runtime,
              'scope': 'Geographic candidates only; controller or lane outcomes never select alignment.',
              'limitations': ['Geographic proximity and direction require road/video confirmation; nearby parallel roads can still match.',
                              'Candidate traversals inherit no rider outcome, contact status or lane identity from the reference.',
                              'Latitude/longitude are localization observations, not calibrated lane or vehicle-body position.',
                              'Position standard-deviation norms are reported, not screened against a calibrated acceptance threshold.',
                              'No matches in supplied files does not establish absence from the full drive.']}
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'result.json', result)
    lines = ['# Geographic window search', '', result['scope'], '',
             f"Accepted geographic candidates: {result['accepted_candidates']}. Rider/contact status remains unreviewed.", '']
    for match in matches:
        lines.append(f"- {match['bounds']}: {'candidate' if match['accepted_geographic_candidate'] else 'rejected'}; {match['reasons']}; maximum position std norm {match['maximum_position_std_norm_m']} m")
    lines += ['', 'Position uncertainty is retained but not thresholded. Nearby parallel roads still require review.']
    (output / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8', newline='\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.request, args.data_root, args.output)
    print(f"{result['status']}: {result['accepted_candidates']} geographic candidates; {args.output / 'report.md'}")


if __name__ == '__main__':
    main()
