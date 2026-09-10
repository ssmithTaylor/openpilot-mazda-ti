# ruff: noqa: TID251
"""Failure-focused checks for the candidate-applied feedback boundary.

These tests intentionally stay at the typed feedback seam.  They do not alter
the controller, limiter, recorded actuator schedule, or vehicle observations.
"""

import pytest

from .applied_feedback import AppliedLateralFeedback, PreviousAppliedFeedback
from .recorded_feedback import serialized_steer
from .test_stock_feedback import events as selection_events
from .test_stock_feedback import make as make_selection


def snapshot(**overrides):
  values = dict(
    publication_mono=1190,
    applied_mono=1180,
    source_request_mono=1170,
    source_controller_mono=1160,
    apply_sequence=1,
    ti_counts=20,
    stock_counts=40,
    selected_counts=20,
    normalized_steer=serialized_steer(20 / 600),
    active=True,
    ti_allowed=True,
    ti_available=True,
    ti_selected=True,
    stock_fallback=False,
    limiter_frozen=False,
  )
  values.update(overrides)
  return AppliedLateralFeedback(**values)


@pytest.mark.parametrize(('field', 'value'), [
  ('publication_mono', 1200),       # publication is equal to the consuming update
  ('publication_mono', 1201),       # publication is in the consuming update's future
  ('applied_mono', 1191),            # apply follows publication
  ('source_request_mono', 1181),     # request follows apply
  ('source_controller_mono', 1171),  # controller update follows request
  ('source_controller_mono', 0),     # missing producer identity
])
def test_feedback_identity_has_strict_no_future_causal_order(field, value):
  candidate = snapshot(**{field: value})
  with pytest.raises(ValueError, match='future|causal order|identity'):
    PreviousAppliedFeedback(1200, candidate.normalized_steer, candidate)


@pytest.mark.parametrize('counts', [-600, -1, 0, 1, 599, 600])
def test_normalized_feedback_is_signed_float32_wire_scale(counts):
  candidate = snapshot(
    ti_counts=counts,
    selected_counts=counts,
    normalized_steer=serialized_steer(counts / 600),
  )
  previous = PreviousAppliedFeedback(1200, candidate.normalized_steer, candidate)
  assert previous.candidate.normalized_steer == serialized_steer(counts / 600)
  assert previous.observed_steer == previous.candidate.normalized_steer


def test_normalized_feedback_rejects_unserialized_count_scale():
  candidate = snapshot(normalized_steer=20 / 600)
  with pytest.raises(ValueError, match='Normalized feedback'):
    PreviousAppliedFeedback(1200, serialized_steer(20 / 600), candidate)


@pytest.mark.parametrize('overrides', [
  dict(selected_counts=40),  # TI selected, but stock counts were injected
  dict(ti_selected=False, stock_fallback=True, selected_counts=20),  # wrong fallback path
  dict(active=False, ti_selected=False, stock_fallback=True, selected_counts=40),
])
def test_selected_feedback_must_match_the_declared_ti_or_stock_path(overrides):
  candidate = snapshot(normalized_steer=serialized_steer(overrides.get('selected_counts', 20) / 600), **overrides)
  with pytest.raises(ValueError, match='selection|actuator path|fallback'):
    PreviousAppliedFeedback(1200, candidate.normalized_steer, candidate)


def test_initial_or_recorded_warmup_feedback_has_no_candidate_owner():
  replay = make_selection(selection_events())
  # This output was applied before activation.  It is valid observed feedback,
  # but there is no candidate-owned producer identity to attach to it.
  previous = replay.previous_applied_for_update(1100, 1020)
  assert previous.observed_steer == 0
  assert previous.candidate is None
  assert not previous.candidate_available


def test_future_consumed_output_is_rejected_before_candidate_is_exposed():
  replay = make_selection(selection_events())
  for i in range(1, 8):
    replay.publish(990 + i * 100, .5)

  with pytest.raises(ValueError, match='future|causal order'):
    replay.previous_applied_for_update(1420, 1420)
  with pytest.raises(ValueError, match='future|causal order'):
    replay.previous_applied_for_update(1410, 1420)


def test_limiter_freeze_is_the_callers_pre_update_state():
  frozen = make_selection(selection_events())
  unfrozen = make_selection(selection_events())
  for replay in (frozen, unfrozen):
    for i in range(1, 8):
      replay.publish(990 + i * 100, .5)

  frozen_prior = frozen.previous_applied_for_update(1450, 1420, limiter_frozen=True)
  unfrozen_prior = unfrozen.previous_applied_for_update(1450, 1420, limiter_frozen=False)
  assert frozen_prior.candidate.limiter_frozen is True
  assert unfrozen_prior.candidate.limiter_frozen is False
  assert frozen_prior.candidate.selected_counts == unfrozen_prior.candidate.selected_counts == 40


def test_dropout_inactive_and_reengage_keep_selected_scale_and_owner_state_distinct():
  replay = make_selection(selection_events())
  for i in range(1, 8):
    replay.publish(990 + i * 100, .5)

  dropout = replay.previous_applied_for_update(1450, 1420)
  assert dropout.candidate.ti_available is False
  assert dropout.candidate.ti_selected is False
  assert dropout.candidate.stock_fallback is True
  assert dropout.candidate.selected_counts == dropout.candidate.stock_counts == 40
  assert dropout.observed_steer == serialized_steer(40 / 600)

  inactive = replay.previous_applied_for_update(1650, 1620)
  assert inactive.candidate.active is False
  assert inactive.candidate.stock_fallback is False
  assert inactive.candidate.selected_counts == 0
  assert inactive.observed_steer == 0

  reengaged = replay.previous_applied_for_update(1750, 1720)
  assert reengaged.candidate.active is True
  assert reengaged.candidate.stock_fallback is True
  assert reengaged.candidate.selected_counts == reengaged.candidate.stock_counts == 10
  assert reengaged.observed_steer == serialized_steer(10 / 600)


def test_ti_active_selection_does_not_substitute_the_parallel_stock_history():
  replay = make_selection(selection_events())
  for i in range(1, 8):
    replay.publish(990 + i * 100, .5 if i in (1, 7) else -.5)

  # On this apply both histories have advanced, but TI is the selected path
  # and the two limited commands intentionally differ at the zero crossing.
  previous = replay.previous_applied_for_update(1250, 1220)
  assert previous.candidate.ti_selected is True
  assert previous.candidate.ti_counts == -5
  assert previous.candidate.stock_counts == -10
  assert previous.candidate.selected_counts == previous.candidate.ti_counts
  assert previous.observed_steer == serialized_steer(-5 / 600)


def test_equal_injected_controller_requests_reproduce_recorded_feedback_exactly():
  rows = selection_events()
  replay = make_selection(rows)
  commands = {int(event.logMonoTime): event.carControl for event in rows if event.which() == 'carControl'}
  outputs = [(int(event.logMonoTime), event.carOutput) for event in rows if event.which() == 'carOutput']
  for command_mono, command in commands.items():
    if command_mono >= 1080:
      replay.publish(command.controlsStateMonoTime, command.actuators.steer)

  for output_mono, output in outputs:
    assert replay.output_for(output_mono) == serialized_steer(output.actuatorsOutput.steer)
  assert all(row['recorded'] == row['replay'] for row in replay.finish())
