"""Replay feedback exposure must not rewrite production plant history."""

import struct
from types import SimpleNamespace

import pytest

from .applied_feedback import AppliedLateralFeedback, PreviousAppliedFeedback
from .runtime import ReplayAppliedFeedback, expose_applied_feedback


def applied(update_mono=1_000_000_000, *, active=True, ti_counts=-300, stock_counts=-250):
  selected = ti_counts
  candidate = AppliedLateralFeedback(
    publication_mono=update_mono - 1_000_000,
    applied_mono=update_mono - 2_000_000,
    source_request_mono=update_mono - 3_000_000,
    source_controller_mono=update_mono - 4_000_000,
    apply_sequence=7,
    ti_counts=ti_counts,
    stock_counts=stock_counts,
    selected_counts=selected,
    normalized_steer=struct.unpack('f', struct.pack('f', selected / 600))[0],
    active=active,
    ti_allowed=active,
    ti_available=True,
    ti_selected=active,
    stock_fallback=False,
    limiter_frozen=False,
  )
  return PreviousAppliedFeedback(update_mono, candidate.normalized_steer, candidate)


def controller(u_prev=411.0):
  return SimpleNamespace(plant=SimpleNamespace(u_prev=u_prev))


def test_exposure_shares_typed_context_without_rewriting_raw_request_history():
  ctrl = controller()
  previous = applied()
  context = expose_applied_feedback(ctrl, previous.update_mono, previous)

  assert ctrl._replay_applied_feedback is context
  assert ctrl.plant._replay_applied_feedback is context
  assert context.candidate.ti_counts == -300
  assert context.stock_request_counts() == -250
  assert ctrl.plant.u_prev == 411.0


def test_absent_feedback_is_an_exact_noop_on_plant_history():
  ctrl = controller()
  context = expose_applied_feedback(ctrl, 1_000_000_000, None)

  assert context.candidate is None
  assert ctrl.plant.u_prev == 411.0
  with pytest.raises(ValueError, match='unavailable'):
    context.stock_request_counts()


def test_stock_request_requires_separate_exact_history():
  previous = applied(stock_counts=None)
  context = ReplayAppliedFeedback(previous.update_mono, previous)

  assert context.candidate.ti_counts == -300
  with pytest.raises(ValueError, match='stock-command history'):
    context.stock_request_counts()


def test_inactive_feedback_cannot_restore_nonzero_history():
  previous = applied(active=False, ti_counts=-1, stock_counts=0)

  with pytest.raises(ValueError, match='Inactive'):
    ReplayAppliedFeedback(previous.update_mono, previous)


def test_inactive_zero_feedback_is_visible_but_not_stock_request_history():
  previous = applied(active=False, ti_counts=0, stock_counts=0)
  ctrl = controller()
  context = expose_applied_feedback(ctrl, previous.update_mono, previous)

  assert context.candidate is previous.candidate
  assert ctrl.plant.u_prev == 411.0
  with pytest.raises(ValueError, match='Inactive'):
    context.stock_request_counts()


def test_context_rejects_wrong_or_nonmonotonic_update_identity():
  previous = applied()
  with pytest.raises(ValueError, match='different controller update'):
    ReplayAppliedFeedback(previous.update_mono + 1, previous)

  ctrl = controller()
  expose_applied_feedback(ctrl, previous.update_mono, previous)
  with pytest.raises(ValueError, match='strictly increasing'):
    expose_applied_feedback(ctrl, previous.update_mono, previous)
