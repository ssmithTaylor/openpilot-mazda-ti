"""Qualify observed-drive symptom measurements on declared cases; never a handling verdict.

calibrate: smooth-labelled development cases only, per-parameter eligibility.
evaluate: every case with a frozen parameter file; holdout measurements are sealed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.mazda_ti import maneuver_metrics as mm  # noqa: E402
from tools.mazda_ti.provenance import environment, read_json, sha256, write_json  # noqa: E402

NS = mm.NS
ROLES = ('development', 'holdout')
CONTACT = ('confirmed_no_intervention', 'confirmed_intervention', 'unknown')
OUTCOME_KEYS = ('scalloping', 'settling', 'held_inward', 'over_release', 'crossing', 'intervention')
LABEL_VALUES = {'scalloping': ('absent', 'present', 'uncertain'), 'settling': ('smooth', 'problem', 'uncertain'),
                'held_inward': ('absent', 'present', 'uncertain'), 'over_release': ('absent', 'present', 'uncertain'),
                'crossing': ('absent', 'present', 'uncertain'), 'intervention': ('absent', 'present', 'uncertain')}
HEX = re.compile('[a-f0-9]{64}')


def validate_request(request):
  if request.get('format_version') != 1 or not isinstance(request.get('data_roots'), dict) or not request['data_roots']:
    raise ValueError('Request needs format_version 1 and named data_roots')
  for name, root in request['data_roots'].items():
    if not isinstance(root, str) or not (PureWindowsPath(root).is_absolute() or PurePosixPath(root).is_absolute()):
      raise ValueError(f'data_roots[{name!r}] must be an absolute path string')
  cases = request.get('cases')
  if not isinstance(cases, list) or not cases:
    raise ValueError('Request needs a nonempty cases list')
  seen = set()
  for c in cases:
    cid = c.get('id')
    if not isinstance(cid, str) or not cid or cid in seen:
      raise ValueError(f'Missing or duplicate case id: {cid!r}')
    seen.add(cid)
    if c.get('role') not in ROLES:
      raise ValueError(f'{cid}: role must be development or holdout')
    if c.get('data_root') not in request['data_roots'] or not isinstance(c.get('route'), str):
      raise ValueError(f'{cid}: unknown data_root or missing route')
    rlogs = c.get('rlogs')
    if not isinstance(rlogs, dict) or not rlogs or any(not isinstance(k, str) or not isinstance(v, str) or not HEX.fullmatch(v) for k, v in rlogs.items()):
      raise ValueError(f'{cid}: rlogs must map relative paths to sha256 hex')
    w = c.get('window_ns')
    if not isinstance(w, list) or len(w) != 2 or any(type(x) is not int for x in w) or not 0 <= w[0] < w[1]:
      raise ValueError(f'{cid}: window_ns must be two increasing integers')
    if not isinstance(c.get('window_source'), dict) or 'description' not in c['window_source']:
      raise ValueError(f'{cid}: window_source needs a description')
    if c.get('contact_status') not in CONTACT:
      raise ValueError(f'{cid}: contact_status must be one of {CONTACT}')
    status = c.get('input_status', {'ok': True, 'problems': []})
    if not isinstance(status, dict) or not isinstance(status.get('ok'), bool) or not isinstance(status.get('problems'), list) \
       or (status['ok'] and status['problems']) or (not status['ok'] and not status['problems']) \
       or any(p.get('rlog') not in rlogs or not isinstance(p.get('reason'), str) for p in status['problems']):
      raise ValueError(f'{cid}: input_status needs ok and per-rlog problems for this case\'s own rlogs')
    labels, annotations = c.get('labels'), c.get('annotations')
    if c['role'] == 'holdout':
      if labels is not None or annotations is not None or c['contact_status'] != 'unknown':
        raise ValueError(f'{cid}: holdout cases must carry no labels, annotations or known contact status')
      continue
    if labels is not None:
      if not isinstance(labels, dict) or not all(k in labels for k in ('provenance', 'confidence', 'scope')):
        raise ValueError(f'{cid}: labels need provenance, confidence and scope')
      for key in OUTCOME_KEYS:
        if key in labels and labels[key] not in LABEL_VALUES[key]:
          raise ValueError(f'{cid}: label {key} must be one of {LABEL_VALUES[key]}')
    for a in annotations or []:
      if a.get('kind') not in ('crossing', 'intervention') or type(a.get('start_ns')) is not int \
         or not (a.get('end_ns') is None or (type(a['end_ns']) is int and a['end_ns'] > a['start_ns'])) \
         or not isinstance(a.get('provenance'), str) or not isinstance(a.get('confidence'), str):
        raise ValueError(f'{cid}: annotation needs kind, integer start_ns, end_ns or null, provenance, confidence')
  return request


def case_rows(request, case, extractor):
  root = Path(request['data_roots'][case['data_root']])
  margin = int(round((mm.DEFAULT_PARAMETERS.smooth_span_s + mm.DEFAULT_PARAMETERS.velocity_span_s) * NS)) + NS
  return extractor(root, list(case['rlogs']), dict(case['rlogs']), case['window_ns'][0], case['window_ns'][1], margin)


def is_smooth_control(case):
  labels = case.get('labels') or {}
  return case['role'] == 'development' and labels.get('scalloping') == 'absent' and labels.get('settling') == 'smooth'


def _continuous_recovery_rows(rows, anchors, params):
  """Lane-healthy rows from the unwind anchor through the first gap or recovery end."""
  if anchors.get('unwind_ns') is None:
    return []
  lane = [r for r in mm.healthy_rows(rows, ('lane',)) if anchors['unwind_ns'] <= int(r['mono_ns']) < anchors['recovery']['end_ns']]
  parts = mm.segments(lane, params.max_gap_ns)
  return parts[0] if parts else []


def _round_up(value, step):
  return math.ceil(value / step - 1e-12) * step


def calibrate(request, params, extractor):
  from tools.mazda_ti.observed_drive_rows import source_hashes
  validate_request(request)
  sources_before = source_hashes()
  smooth = [c for c in request['cases'] if is_smooth_control(c)]
  cases, per_case = {}, {}
  for c in smooth:
    if not c.get('input_status', {'ok': True})['ok']:
      cases[c['id']] = {'orientation': None, 'anchors': {'unwind_ns': None, 'critical_gap': None, 'status': 'unscorable', 'reason': 'input_unavailable'},
                        'recovery_support_s': 0.0, 'longest_lane_support_s': 0.0, 'recovery_stats': {}, 'offset_residual_mad_m': None,
                        'settling_probe': None, 'window_ns': list(c['window_ns']), 'extraction': None,
                        'input_problems': c['input_status']['problems']}
      continue
    extracted = case_rows(request, c, extractor)
    window = tuple(c['window_ns'])
    orient = mm.orient(extracted['rows'], window)
    rows = mm.oriented(extracted['rows'], orient['direction']) if orient['direction'] else extracted['rows']
    anchors = mm.phase_anchors(rows, window, params) if orient['direction'] else {'unwind_ns': None, 'critical_gap': None, 'status': 'unscorable', 'reason': orient['reason']}
    rec = _continuous_recovery_rows(rows, anchors, params)
    support = (int(rec[-1]['mono_ns']) - int(rec[0]['mono_ns'])) / NS if len(rec) > 1 else 0.0
    lane_all = mm.healthy_rows(rows, ('lane',))
    segs = mm.segments(lane_all, params.max_gap_ns)
    durations = [(p, (int(p[-1]['mono_ns']) - int(p[0]['mono_ns'])) / NS) for p in segs]
    longest_part, longest = max(durations, key=lambda pd: pd[1]) if durations else (None, 0.0)
    stats = {}
    if rec:
      w = mm.elapsed_weights(rec, int(rec[0]['mono_ns']), int(rec[-1]['mono_ns']) + 1, params.max_gap_ns)
      s = mm.smoothed(rec, 'lane_offset_m', params)
      offset_pairs = [(abs(x), wi) for x, wi in zip(s, w, strict=True) if x is not None]
      v = mm.derivative(rec, 'lane_offset_m', params)
      pairs = [(abs(x), wi) for x, wi in zip(v, w, strict=True) if x is not None]
      stats = {'p95_abs_offset_m': mm.weighted_percentile([p[0] for p in offset_pairs], [p[1] for p in offset_pairs], 0.95) if offset_pairs else None,
               'p95_abs_velocity_mps': mm.weighted_percentile([p[0] for p in pairs], [p[1] for p in pairs], 0.95) if pairs else None}
    residual = None
    if longest >= params.min_recovery_support_s:
      part = longest_part
      s = mm.smoothed(part, 'lane_offset_m', params)
      slow = mm.smoothed(part, 'lane_offset_m', params, span_s=2.0)
      res = [abs(a - b) for a, b in zip(s, slow, strict=True) if a is not None and b is not None]
      residual = statistics.median(res) if res else None
    cases[c['id']] = {'orientation': orient, 'anchors': anchors, 'recovery_support_s': support,
                      'longest_lane_support_s': longest, 'recovery_stats': stats, 'offset_residual_mad_m': residual,
                      'settling_probe': None, 'window_ns': list(window),
                      'extraction': {k: extracted.get(k) for k in ('raw_sha256', 'source_sha256', 'runtime', 'unhealthy_count_by_group_and_reason', 'camera_age_s', 'lane_fit_policy', 'initdata')}}
    per_case[c['id']] = (rows, anchors, rec)
  derivation = {}

  def eligible_recovery(cid):
    info = cases[cid]
    if info.get('input_problems'):
      return False, 'input_unavailable'
    if info['anchors'].get('unwind_ns') is None:
      return False, 'no_unwind_anchor'
    if info['anchors'].get('critical_gap'):
      return False, 'critical_gap_at_anchor'
    if info['recovery_support_s'] < params.min_recovery_support_s:
      return False, f'recovery_support {info["recovery_support_s"]:.2f}s < {params.min_recovery_support_s}s'
    return True, 'eligible'

  shortfalls = []

  def derive(name, rule, eligibility, values, step):
    table = {cid: {'eligible': ok, 'reason': why, 'value': values.get(cid)} for cid, (ok, why) in eligibility.items()}
    usable = [values[cid] for cid, row in table.items() if row['eligible'] and values.get(cid) is not None]
    if len(usable) < params.min_eligible_cases:
      derivation[name] = {'rule': rule, 'cases': table, 'value': None, 'eligible_count': len(usable),
                          'reason': f'shortfall: {len(usable)} eligible cases, need {params.min_eligible_cases}'}
      shortfalls.append(f'{name} ({len(usable)} eligible, need {params.min_eligible_cases})')
      return None
    derivation[name] = {'rule': rule, 'cases': table, 'value': _round_up(max(usable), step), 'eligible_count': len(usable)}
    return derivation[name]['value']

  def eligible_lane_support(cid):
    info = cases[cid]
    if info.get('input_problems'):
      return False, 'input_unavailable'
    if info['longest_lane_support_s'] >= params.min_recovery_support_s:
      return True, 'eligible'
    return False, 'lane_support too short'

  rec_elig = {cid: eligible_recovery(cid) for cid in cases}
  e_settle = derive('e_settle_m', 'max over eligible cases of elapsed-time p95 |e| in continuous recovery observation, rounded up to 0.01',
                    rec_elig, {cid: cases[cid]['recovery_stats'].get('p95_abs_offset_m') for cid in cases}, 0.01)
  v_settle = derive('v_settle_mps', 'max over eligible cases of elapsed-time p95 |de/dt| in continuous recovery observation, rounded up to 0.01',
                    rec_elig, {cid: cases[cid]['recovery_stats'].get('p95_abs_velocity_mps') for cid in cases}, 0.01)
  a_elig = {cid: eligible_lane_support(cid) for cid in cases}
  a_min = derive('a_min_m', 'three times the max over eligible cases of the median |smoothed offset - 2 s moving mean|, rounded up to 0.01',
                 a_elig, {cid: (3 * cases[cid]['offset_residual_mad_m']) if cases[cid]['offset_residual_mad_m'] is not None else None for cid in cases}, 0.01)
  probe = mm.Parameters(**{**params.as_dict(), 'e_settle_m': e_settle, 'v_settle_mps': v_settle})
  settle_values, settle_elig = {}, {}
  for cid in cases:
    ok, why = rec_elig[cid]
    if cid not in per_case:
      settle_elig[cid] = (False, why)
      continue
    rows, anchors, rec = per_case[cid]
    if ok:
      s = mm.settling(rows, tuple(next(c for c in smooth if c['id'] == cid)['window_ns']), anchors, probe)
      cases[cid]['settling_probe'] = s
      if s['status'] == 'measured_estimate':
        settle_values[cid] = s['value'] * 1.5
      else:
        ok, why = False, f'no confirmed dwell ({s["status"]}: {s["reason"]})'
    settle_elig[cid] = (ok, why)
  t_max = derive('t_settle_max_s', 'max over eligible cases of confirmed settling_time_s times 1.5, rounded up to 0.1', settle_elig, settle_values, 0.1)
  if shortfalls:
    err = ValueError('Insufficient eligible cases for: ' + '; '.join(shortfalls))
    err.derivation = derivation
    raise err
  digest = hashlib.sha256(json.dumps({'base': params.params_id, 'e': e_settle, 'v': v_settle, 'a': a_min, 't': t_max}, sort_keys=True).encode()).hexdigest()[:12]
  new = mm.Parameters(**{**params.as_dict(), 'params_id': f'{params.params_id}-cal-{digest}', 'e_settle_m': e_settle,
                         'v_settle_mps': v_settle, 'a_min_m': a_min, 't_settle_max_s': t_max, 'corridor_half_width_m': e_settle})
  if source_hashes() != sources_before:
    raise ValueError('Source files changed during calibration')
  return {'params': new, 'derivation': derivation, 'cases': cases, 'base_params_id': params.params_id,
          'source_sha256': sources_before, 'request_sha256': request_digest(request)}


def request_digest(request):
  return hashlib.sha256(json.dumps(request, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def write_calibration(result, params_out, output):
  params_out, output = Path(params_out), Path(output)
  if params_out.exists() or output.exists():
    raise FileExistsError('Preserve existing calibration outputs')
  params_out.parent.mkdir(parents=True, exist_ok=True)
  output.parent.mkdir(parents=True, exist_ok=True)
  write_json(params_out, result['params'].as_dict())
  write_json(output, {'format_version': 1, 'scope': 'Draft parameter derivation from smooth controls; calibration, not validation',
                      'base_params_id': result['base_params_id'], 'params_id': result['params'].params_id,
                      'params_sha256': sha256(params_out), 'request_sha256': result['request_sha256'],
                      'source_sha256': result['source_sha256'], 'derivation': result['derivation'], 'cases': result['cases'],
                      'runtime': environment()})


def _cli_calibrate(args):
  from tools.mazda_ti.observed_drive_rows import run as extractor
  request = read_json(args.request)
  base = mm.Parameters(**read_json(args.params)) if args.params else mm.DEFAULT_PARAMETERS
  result = calibrate(request, base, extractor)
  write_calibration(result, args.params_out, args.output)


FAMILIES = {'cycles': 'lane_motion_cycles', 'held_inward': 'late_wide_excursion', 'over_release': 'late_wide_excursion',
            'hold_late_wide_sequence': 'late_wide_excursion', 'settling': 'settling_time'}
LABEL_TO_FAMILY = {'scalloping': 'cycles', 'held_inward': 'held_inward', 'over_release': 'over_release', 'settling': 'settling'}
SYMPTOM_VALUES = {'scalloping': 'present', 'held_inward': 'present', 'over_release': 'present', 'settling': 'problem'}


def _annotation_list(case):
  return [dict(a) for a in (case.get('annotations') or [])]


def measure_case(request, case, params, extractor):
  extracted = case_rows(request, case, extractor)
  window = tuple(case['window_ns'])
  orient = mm.orient(extracted['rows'], window)
  direction = orient['direction']
  rows = mm.oriented(extracted['rows'], direction) if direction else extracted['rows']
  anchors = (mm.phase_anchors(rows, window, params) if direction else
             {'anchor_source': 'model_road_geometry', 'entry_ns': None, 'unwind_ns': None, 'recovery': None,
              'critical_gap': None, 'coverage': {}, 'absent_phases': ['entry', 'sustained', 'unwind', 'recovery'],
              'status': 'unscorable', 'reason': orient['reason']})
  ann = _annotation_list(case)
  metrics = {'settling_time': mm.settling(rows, window, anchors, params, interventions=ann),
             'lane_motion_cycles': mm.cycles(rows, window, params),
             'late_wide_excursion': (mm.hold_late_wide(rows, window, anchors, params, direction) if direction else
                                     mm.record('late_wide_excursion', unit='m', reason=orient['reason'])),
             'path_rms': mm.path_quality(rows, window, anchors, params),
             'lateral_jerk_p95': mm.comfort_proxy(rows, window, params),
             **mm.annotations(ann, window, labels=case.get('labels'))}
  identity = {'route': case['route'], 'source_sha256': hashlib.sha256(json.dumps(extracted['source_sha256'], sort_keys=True).encode()).hexdigest(),
              'raw_sha256': dict(extracted['raw_sha256']), 'window_ns': list(window)}
  return {'id': case['id'], 'role': case['role'], 'route': case['route'], 'window_ns': list(window),
          'window_source': case['window_source'], 'contact_status': case['contact_status'],
          'extraction': {k: extracted.get(k) for k in ('raw_sha256', 'source_sha256', 'unhealthy_count_by_group_and_reason', 'camera_age_s', 'lane_fit_policy', 'initdata')},
          'row_count': len(extracted['rows']), 'orientation': orient, 'anchors': anchors, 'metrics': metrics,
          'physical_verdicts': mm.physical_verdicts(metrics, identity, case['contact_status'])}


def unavailable_case(case):
  """A case whose exact inputs are missing or changed: kept in every denominator, never measured."""
  reason = 'input_unavailable'
  metrics = {name: mm.record(name, unit=unit, reason=reason, diagnostics={'input_problems': case['input_status']['problems']})
             for name, unit in (('settling_time', 's'), ('lane_motion_cycles', 'cycles'), ('late_wide_excursion', 'm'),
                                ('path_rms', 'm'), ('lateral_jerk_p95', 'm/s^3'), ('boundary_crossings', 'events'), ('driver_catches', 'events'))}
  return {'id': case['id'], 'role': case['role'], 'route': case['route'], 'window_ns': list(case['window_ns']),
          'window_source': case['window_source'], 'contact_status': case['contact_status'], 'extraction': None, 'row_count': 0,
          'orientation': {'direction': None, 'status': 'unscorable', 'reason': reason}, 'input_problems': case['input_status']['problems'],
          'anchors': {'anchor_source': 'model_road_geometry', 'entry_ns': None, 'unwind_ns': None, 'recovery': None, 'critical_gap': None,
                      'coverage': {}, 'absent_phases': ['entry', 'sustained', 'unwind', 'recovery'], 'status': 'unscorable', 'reason': reason},
          'metrics': metrics, 'physical_verdicts': []}


def detection_outcomes(case_record, params):
  m = case_record['metrics']
  out = {}
  s = m['settling_time']
  if s['status'] == 'measured_estimate':
    out['settling'] = 'detected' if s['value'] > params.t_settle_max_s else 'not_detected'
  elif s['status'] in ('right_censored', 'unresolved') and s['lower_bound_s'] is not None and s['lower_bound_s'] > params.t_settle_max_s:
    out['settling'] = 'detected'
  else:
    out['settling'] = 'unresolved'
  c = m['lane_motion_cycles']
  out['cycles'] = ('detected' if c['value'] >= 1 else 'not_detected') if c['status'] == 'measured_estimate' else 'unresolved'
  h = m['late_wide_excursion']
  if h['status'] in ('measured_estimate', 'right_censored') and h['diagnostics'].get('coverage_adequate') is True:
    held = h['diagnostics']['inward_dwell_s'] > params.t_dwell_s
    wide = h['value'] is not None and h['value'] > params.corridor_half_width_m
    out['held_inward'] = 'detected' if held else 'not_detected'
    out['over_release'] = 'detected' if wide else 'not_detected'
    # The complaint sequence needs both; a modest inside offset alone is not the complaint.
    out['hold_late_wide_sequence'] = 'detected' if (held and wide) else 'not_detected'
  else:
    out['held_inward'] = out['over_release'] = out['hold_late_wide_sequence'] = 'unresolved'
  return out


def evaluate(request, params, extractor):
  from tools.mazda_ti.observed_drive_rows import source_hashes
  validate_request(request)
  sources_before = source_hashes()
  for name in ('e_settle_m', 'v_settle_mps', 'a_min_m', 't_settle_max_s', 'corridor_half_width_m'):
    if getattr(params, name) is None:
      raise ValueError(f'evaluate requires calibrated {name}')
  development, holdout = {}, {}
  for c in request['cases']:
    rec = unavailable_case(c) if not c.get('input_status', {'ok': True})['ok'] else measure_case(request, c, params, extractor)
    (holdout if c['role'] == 'holdout' else development)[c['id']] = rec
  false_alarms, detection, disagreements, unscorable = {}, {}, [], []
  for c in request['cases']:
    if c['role'] != 'development':
      continue
    rec = development[c['id']]
    outcomes = detection_outcomes(rec, params)
    rec['detection_outcomes'] = outcomes
    for family, metric in FAMILIES.items():
      if rec['metrics'][metric]['status'] == 'unscorable':
        unscorable.append({'case': c['id'], 'family': family, 'reason': rec['metrics'][metric]['reason']})
    labels = c.get('labels') or {}
    if is_smooth_control(c):
      # Individual detectors on a smooth pass are diagnostics; the sequence detector is the alarm.
      false_alarms[c['id']] = {k: outcomes[k] for k in ('settling', 'cycles', 'hold_late_wide_sequence')}
      for family, outcome in outcomes.items():
        if outcome == 'detected':
          disagreements.append({'case': c['id'], 'family': family, 'label': 'smooth', 'outcome': outcome})
    symptoms = {k: v for k, v in labels.items() if k in LABEL_TO_FAMILY and v == SYMPTOM_VALUES[k]}
    if symptoms:
      detection[c['id']] = {k: outcomes[LABEL_TO_FAMILY[k]] for k in symptoms}
      for k, outcome in detection[c['id']].items():
        if outcome == 'not_detected':
          disagreements.append({'case': c['id'], 'family': LABEL_TO_FAMILY[k], 'label': k, 'outcome': outcome})
  totals = {'development_cases': len(development), 'holdout_cases': len(holdout),
            'development_input_unavailable': sum(1 for r in development.values() if r.get('input_problems')),
            'holdout_input_unavailable': sum(1 for r in holdout.values() if r.get('input_problems')),
            'development_measured': sum(1 for r in development.values() if not r.get('input_problems'))}
  report = {'coverage_totals': totals, 'false_alarms': false_alarms, 'symptom_detection': detection, 'disagreements': disagreements,
            'unscorable': unscorable,
            'validation_status': {'status': 'unresolved', 'reason': 'development cases were previously inspected; holdout results are sealed without independent labels',
                                  'holdout_count': len(holdout)},
            'limits': ['Lane geometry is model-derived, not surveyed truth', 'Latest-publication join, not exact consumed identity',
                       'TI counts are command evidence, not delivered torque', 'Boundary distances are reference-point proxies without vehicle footprint',
                       'Thresholds are draft calibration values, not accepted physical limits',
                       'Development cases are calibration evidence, not validation']}
  if source_hashes() != sources_before:
    raise ValueError('Source files changed during evaluation')
  return {'params_id': params.params_id, 'params': params.as_dict(), 'development': development, 'holdout': holdout,
          'report': report, 'source_sha256': sources_before, 'request_sha256': request_digest(request)}


def _md_table(headers, rows):
  lines = ['| ' + ' | '.join(headers) + ' |', '| ' + ' | '.join('---' for _ in headers) + ' |']
  lines += ['| ' + ' | '.join(str(x) for x in row) + ' |' for row in rows]
  return lines


def render_report(result, holdout_sha256):
  r = result['report']
  lines = ['# Observed-drive measurement qualification', '',
           f"Parameters `{result['params_id']}`. Measurement qualification only: no handling verdict, no physical acceptance.", '',
           '## False alarms per labelled smooth maneuver', '']
  lines += _md_table(['case', 'settling', 'cycles', 'hold_late_wide_sequence'], [(cid, o['settling'], o['cycles'], o['hold_late_wide_sequence']) for cid, o in r['false_alarms'].items()])
  lines += ['', '## Symptom detection within label scope', '']
  lines += _md_table(['case', 'label', 'outcome'], [(cid, k, v) for cid, d in r['symptom_detection'].items() for k, v in d.items()])
  lines += ['', '## Disagreements (calibration gaps)', '']
  lines += _md_table(['case', 'family', 'label', 'outcome'], [(d['case'], d['family'], d['label'], d['outcome']) for d in r['disagreements']]) if r['disagreements'] else ['none']
  t = r['coverage_totals']
  lines += ['', '## Coverage totals', '', f"Development cases {t['development_cases']} (measured {t['development_measured']}, inputs unavailable {t['development_input_unavailable']}); holdout cases {t['holdout_cases']} (inputs unavailable {t['holdout_input_unavailable']}).", '', '## Unscorable', '']
  lines += _md_table(['case', 'family', 'reason'], [(u['case'], u['family'], u['reason']) for u in r['unscorable']]) if r['unscorable'] else ['none']
  lines += ['', '## Per-case records', '']
  for cid, rec in result['development'].items():
    a = rec['anchors']
    lines += [f'### {cid}', '', f"Role development; contact `{rec['contact_status']}`; rows {rec['row_count']}; orientation {rec['orientation']['direction']}; anchors status `{a['status']}` (entry {a.get('entry_ns')}, unwind {a.get('unwind_ns')}, critical gap {a.get('critical_gap')}).", '']
    lines += _md_table(['metric', 'status', 'value', 'unit', 'lower bound s', 'reason'],
                       [(k, m['status'], m['value'], m['unit'], m['lower_bound_s'], m['reason']) for k, m in rec['metrics'].items()])
    lines += ['', 'Physical evidence boundary: ' + ', '.join(f"{v['metric']}={v['status']}" for v in rec['physical_verdicts']), '']
  lines += ['## Holdout (sealed)', '', f"{r['validation_status']['holdout_count']} cases sealed in `holdout-sealed.json` (SHA-256 `{holdout_sha256}`). Validation status: {r['validation_status']['status']}.", '',
            '## Limits', ''] + [f'- {x}' for x in r['limits']]
  return '\n'.join(lines) + '\n'


def write_evaluation(result, output_dir, request_path, params_path, *, request_sha256, params_sha256):
  out = Path(output_dir)
  if out.exists():
    raise FileExistsError(out)
  out.mkdir(parents=True)
  runtime = environment()
  sealed = out / 'holdout-sealed.json'
  write_json(sealed, {'format_version': 1, 'request': {'path': str(request_path), 'sha256': request_sha256},
                      'params': {'path': str(params_path), 'sha256': params_sha256}, 'params_id': result['params_id'],
                      'source_sha256': result['source_sha256'], 'request_content_sha256': result['request_sha256'],
                      'runtime': runtime, 'cases': result['holdout'],
                      'note': 'Outcome-blind measurements; no labels were supplied or read for these cases'})
  sealed_hash = sha256(sealed)
  public = {'format_version': 1, 'scope': 'Observed-drive measurement qualification; no handling verdict',
            'request': {'path': str(request_path), 'sha256': request_sha256}, 'params': {'path': str(params_path), 'sha256': params_sha256, 'values': result['params']},
            'params_id': result['params_id'], 'runtime': runtime, 'source_sha256': result['source_sha256'],
            'request_content_sha256': result['request_sha256'], 'development': result['development'],
            'holdout': {'case_ids': sorted(result['holdout']), 'count': len(result['holdout']), 'sealed_sha256': sealed_hash},
            'report': result['report']}
  write_json(out / 'result.json', public)
  (out / 'report.md').write_text(render_report(result, sealed_hash), encoding='utf-8', newline='\n')


def _cli_evaluate(args):
  from tools.mazda_ti.observed_drive_rows import run as extractor
  request = read_json(args.request)
  params = mm.Parameters(**read_json(args.params))
  result = evaluate(request, params, extractor)
  write_evaluation(result, args.output, args.request, args.params, request_sha256=sha256(args.request), params_sha256=sha256(args.params))


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  sub = parser.add_subparsers(dest='stage', required=True)
  c = sub.add_parser('calibrate')
  c.add_argument('--request', type=Path, required=True)
  c.add_argument('--params', type=Path, default=None, help='Base parameter file; defaults to the built-in draft')
  c.add_argument('--params-out', type=Path, required=True)
  c.add_argument('--output', type=Path, required=True)
  c.set_defaults(func=_cli_calibrate)
  e = sub.add_parser('evaluate')
  e.add_argument('--request', type=Path, required=True)
  e.add_argument('--params', type=Path, required=True)
  e.add_argument('--output', type=Path, required=True)
  e.set_defaults(func=_cli_evaluate)
  args = parser.parse_args(argv)
  args.func(args)


if __name__ == '__main__':
  main()
