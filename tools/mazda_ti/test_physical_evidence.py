"""Physical observations cannot become candidate outcomes through replay."""

from copy import deepcopy

import pytest

from .physical_evidence import classify_metrics, replay_physical_summary


IDENTITY = {'route': 'synthetic-route', 'source_sha256': 'a' * 64, 'raw_sha256': {'route--0/rlog': 'b' * 64},
            'window_ns': [1_000_000_000, 4_000_000_000]}


def observation(**changes):
  return {'metric': 'minimum_left_clearance', 'value': .3, 'unit': 'm', 'source_identity': deepcopy(IDENTITY),
          'valid_duration_s': 3., 'required_duration_s': 3., 'maximum_gap_s': .02,
          'uncertainty_method': 'synthetic independent calibration', 'supporting_event_ids': ['clip:event-1'],
          'critical_gap': False, 'ambiguous': False, **changes}


def classify(record, evidence_type='observed_candidate_drive', identity=IDENTITY):
  return classify_metrics([record], evidence_type=evidence_type, source_arm='candidate', verified_identity=identity)


def test_copied_replay_geometry_is_never_candidate_motion_even_with_identity():
  result = classify(observation(), 'fixed_motion_replay')
  assert all(item['status'] == 'not_measured' and item['value'] is None for item in result)
  assert all(item['paired_effect'] is None and item['threshold'] is None for item in result)


def test_prediction_cannot_masquerade_as_observed_or_validated_motion():
  assert all(item['status'] == 'unsupported' and item['value'] is None for item in classify(observation(), 'predicted_motion'))


@pytest.mark.parametrize('changes', [{'ambiguous': True}, {'critical_gap': True}, {'uncertainty_method': ''},
                                    {'valid_duration_s': 0}, {'required_duration_s': 2}, {'maximum_gap_s': -1},
                                    {'supporting_event_ids': []}, {'unit': 'counts'}, {'value': float('nan')}])
def test_missing_or_ambiguous_physical_measurements_never_fill_zero(changes):
  record = classify(observation(**changes))[0]
  assert record['status'] == 'not_measured'
  assert record['value'] is None


@pytest.mark.parametrize('identity', [None, {**IDENTITY, 'source_sha256': 'c' * 64},
                                      {**IDENTITY, 'window_ns': [1., 4.]}, {**IDENTITY, 'raw_sha256': {}}])
def test_missing_changed_or_invalid_verified_drive_identity_cannot_support_outcome(identity):
  assert classify(observation(), identity=identity)[0]['value'] is None


def test_source_bound_observation_is_retained_but_never_a_roadworthy_pass():
  result = classify(observation())
  assert result[0]['status'] == 'observed_unqualified'
  assert result[0]['value'] == .3
  assert result[0]['source_identity'] == IDENTITY
  assert all(item['paired_effect'] is None and item['threshold'] is None for item in result)
  assert all(item['status'] not in ('passed', 'safe', 'roadworthy') for item in result)


@pytest.mark.parametrize('name,unit,value', [('driver_catches', 'events', -1), ('driver_catches', 'events', .5),
                                           ('scallop_peak_to_peak', 'm', -1), ('lateral_jerk_p95', 'm/s^3', -1),
                                           ('corridor_violation_duration', 's', -1)])
def test_impossible_metric_domains_are_not_observations(name, unit, value):
  result = classify(observation(metric=name, unit=unit, value=value, contact_status='confirmed_no_intervention'))
  assert next(item for item in result if item['metric'] == name)['value'] is None


def test_negative_clearance_preserves_observed_crossing_and_window_bounds_are_enforced():
  assert classify(observation(value=-.1))[0]['value'] == -.1
  assert classify(observation(valid_duration_s=4., required_duration_s=4.))[0]['value'] is None


def test_observed_candidate_cannot_use_a_reference_role():
  with pytest.raises(ValueError, match='candidate source arm'):
    classify_metrics([observation()], evidence_type='observed_candidate_drive', source_arm='reference', verified_identity=IDENTITY)


@pytest.mark.parametrize('extra', [{'contact_status': 'uncertain'}, {'contact_status': 'confirmed_intervention'},
                                  {'contact_status': 'confirmed_no_intervention', 'complete_recovery': False}])
def test_assisted_unknown_or_incomplete_recovery_cannot_be_autonomous_settling(extra):
  record = observation(metric='settling_time', unit='s', **{'complete_recovery': True, **extra})
  result = next(item for item in classify(record) if item['metric'] == 'settling_time')
  assert result['value'] is None


def test_reference_observations_preserved_and_candidate_metadata_never_copied():
  reference = {'physical_observations': [{'text': 'rider reports smooth', 'provenance': 'source note'}]}
  candidate = {'physical_observations': [{'text': 'candidate is safer'}], 'physical_metrics': [observation()]}
  original = deepcopy(reference)
  result = replay_physical_summary(reference, candidate)
  assert reference == original
  assert result['baseline_observations'][0]['text'] == 'rider reports smooth'
  assert result['baseline_observations'][0]['source_arm'] == 'reference'
  assert result['baseline_observations'][0]['status'] == 'unqualified'
  assert result['candidate_metadata_claims_ignored'] is True
  assert result['road_performance_decision'] == 'not_assessed'
  assert all(item['value'] is None for item in result['candidate_metrics'])
