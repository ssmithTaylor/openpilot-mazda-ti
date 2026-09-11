"""Keep observed motion, fixed-motion replay, and predictions distinct.

This is an evidence classifier, not a lane detector or a roadworthiness gate.
Callers of observed-drive classification must independently verify the supplied
source identity against original drive artifacts. Hashes alone do not validate
geometry, driver contact, measurement error, or a physical acceptance threshold.
"""

import math
import re


FORMAT_VERSION = 1
METRIC_UNITS = {
  'minimum_left_clearance': 'm',
  'minimum_right_clearance': 'm',
  'corridor_violation_duration': 's',
  'driver_catches': 'events',
  'late_wide_excursion': 'm',
  'settling_time': 's',
  'scallop_peak_to_peak': 'm',
  'lateral_jerk_p95': 'm/s^3',
}
OBSERVED_TYPES = ('recorded_baseline', 'observed_candidate_drive')


def _finite(value):
  return type(value) in (int, float) and math.isfinite(value)


def _identity_valid(identity):
  if not isinstance(identity, dict):
    return False
  hashes = identity.get('raw_sha256')
  window = identity.get('window_ns')
  return (isinstance(identity.get('route'), str) and bool(identity['route']) and
          isinstance(identity.get('source_sha256'), str) and re.fullmatch('[a-f0-9]{64}', identity['source_sha256']) is not None and
          isinstance(hashes, dict) and bool(hashes) and
          all(isinstance(path, str) and bool(path) and isinstance(digest, str) and re.fullmatch('[a-f0-9]{64}', digest)
              for path, digest in hashes.items()) and
          isinstance(window, list) and len(window) == 2 and all(type(value) is int for value in window) and
          0 <= window[0] < window[1])


def classify_metrics(records=None, *, evidence_type, source_arm, verified_identity=None):
  """Return explicit missing/unqualified records; never infer a physical pass.

  evidence_type is a caller-owned context, never taken from a metric record.
  verified_identity is the exact route/source/raw/window identity independently
  checked by the caller; omitted identities cannot qualify an observed value.
  Version 1 intentionally provides no calibrated physical acceptance operation.
  """
  if evidence_type not in (*OBSERVED_TYPES, 'fixed_motion_replay', 'predicted_motion'):
    raise ValueError('Unknown physical evidence type')
  if source_arm not in ('reference', 'candidate', 'current_control'):
    raise ValueError('Unknown source arm')
  if evidence_type == 'observed_candidate_drive' and source_arm != 'candidate':
    raise ValueError('Observed candidate drive requires the candidate source arm')
  if records is None:
    records = []
  if not isinstance(records, list) or any(not isinstance(record, dict) for record in records):
    raise ValueError('Physical metrics must be a list of records')
  by_name = {}
  for record in records:
    name = record.get('metric')
    if name not in METRIC_UNITS or name in by_name:
      raise ValueError('Physical metric names must be supported and unique')
    by_name[name] = record
  result = []
  for name, unit in METRIC_UNITS.items():
    record = by_name.get(name)
    item = {'metric': name, 'version': FORMAT_VERSION, 'evidence_type': evidence_type, 'source_arm': source_arm,
            'value': None, 'unit': unit, 'valid_duration_s': None, 'required_duration_s': None,
            'maximum_gap_s': None, 'uncertainty_method': None, 'source_identity': None,
            'supporting_event_ids': [], 'reference_value': None, 'paired_effect': None,
            'threshold': None, 'gate_category': 'physical_observation', 'status': 'not_measured'}
    if evidence_type == 'fixed_motion_replay':
      reason = 'Candidate commands do not change recorded vehicle motion; replay geometry cannot measure this outcome.'
    elif evidence_type == 'predicted_motion':
      item['status'] = 'unsupported'
      reason = 'No validated candidate-responsive physical prediction model is implemented by this classifier.'
    elif record is None:
      reason = 'No observed physical metric supplied.'
    elif not _identity_valid(verified_identity) or record.get('source_identity') != verified_identity:
      reason = 'Missing, invalid, or mismatched independently verified drive/source/window identity.'
    elif record.get('unit') != unit or not _finite(record.get('value')):
      reason = 'Missing or invalid observed value/unit.'
    elif ((name == 'driver_catches' and (type(record['value']) is not int or record['value'] < 0)) or
          (name not in ('minimum_left_clearance', 'minimum_right_clearance') and record['value'] < 0)):
      reason = 'Metric value is outside its physical domain.'
    elif (not all(_finite(record.get(key)) for key in ('valid_duration_s', 'required_duration_s', 'maximum_gap_s')) or
          not 0 < record['valid_duration_s'] <= record['required_duration_s'] or record['maximum_gap_s'] < 0):
      reason = 'Missing or invalid physical measurement coverage.'
    elif record['required_duration_s'] > (verified_identity['window_ns'][1] - verified_identity['window_ns'][0]) / 1e9:
      reason = 'Required measurement duration exceeds the verified drive window.'
    elif (record.get('critical_gap') is not False or record.get('ambiguous') is not False or
          not isinstance(record.get('uncertainty_method'), str) or not record['uncertainty_method'].strip() or
          not isinstance(record.get('supporting_event_ids'), list) or not record['supporting_event_ids'] or
          any(not isinstance(value, str) or not value for value in record['supporting_event_ids'])):
      reason = 'Ambiguous measurement, critical gap, or missing uncertainty/event evidence.'
    elif name in ('driver_catches', 'settling_time') and record.get('contact_status') not in ('confirmed_no_intervention', 'confirmed_intervention'):
      reason = 'Driver intervention status is unknown; do not infer contact from steeringPressed or TI.'
    elif name == 'settling_time' and record.get('contact_status') == 'confirmed_intervention':
      reason = 'Driver-assisted recovery cannot measure autonomous settling.'
    elif name == 'settling_time' and record.get('complete_recovery') is not True:
      reason = 'Recovery observation is incomplete or right-censored.'
    else:
      item.update({key: record[key] for key in ('value', 'valid_duration_s', 'required_duration_s', 'maximum_gap_s',
                                               'uncertainty_method', 'source_identity', 'supporting_event_ids')})
      item['status'] = 'observed_unqualified'
      reason = 'Source-bound observation retained; measurement calibration and frozen physical acceptance are not established here.'
    item['reason'] = reason
    result.append(item)
  return result


def replay_physical_summary(reference_metadata, candidate_metadata):
  """A command comparison can retain baseline context, never candidate motion.

  Legacy annotation text remains explicitly unqualified and tied to the
  reference trace artifact by the enclosing report's provenance hashes.
  Candidate-supplied annotation text is not rendered as a physical observation.
  """
  observations = reference_metadata.get('physical_observations', [])
  if not isinstance(observations, list):
    observations = []
  baseline = [{**item, 'source_arm': 'reference', 'evidence_type': 'recorded_baseline', 'status': 'unqualified',
               'qualification_reason': 'Legacy context only; enclosing artifact hash does not validate rider annotation or physical geometry.'}
              for item in observations if isinstance(item, dict)]
  return {'format_version': FORMAT_VERSION, 'status': 'not_measured', 'road_performance_decision': 'not_assessed',
          'baseline_observations': baseline,
          'candidate_metrics': classify_metrics(evidence_type='fixed_motion_replay', source_arm='candidate'),
          'candidate_metadata_claims_ignored': any(candidate_metadata.get(key) for key in ('physical_observations', 'physical_metrics')),
          'reason': 'Fixed-motion replay measures command changes. Candidate handling requires a separately observed candidate drive.'}
