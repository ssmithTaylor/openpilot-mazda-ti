import pytest

from tools.mazda_ti.legacy_receipt_constraints import sequence_compatible


def call(**changes):
  values = dict(car_state_ns=100, parameters_ns=80, previous_car_state_ns=90,
                previous_parameters_ns=80, cycle_start_ns=91, publication_ns=110,
                receive_timeout_ns=20)
  values.update(changes)
  return sequence_compatible(**values)


def test_new_publication_and_retained_parameters_allowed():
  assert call()
  assert call(parameters_ns=105)


def test_same_message_requires_time_for_receive_timeout():
  assert not call(car_state_ns=90)
  assert call(car_state_ns=90, cycle_start_ns=90)
  assert call(car_state_ns=90, cycle_start_ns=89)


def test_no_backtracking_even_when_timeout_possible():
  assert not call(car_state_ns=89, cycle_start_ns=1)
  assert not call(parameters_ns=79)


def test_missing_prior_identity_does_not_invent_one():
  assert call(previous_car_state_ns=None, previous_parameters_ns=None)


def test_future_publications_rejected():
  assert not call(car_state_ns=111)
  assert not call(parameters_ns=111)


@pytest.mark.parametrize('change', [dict(cycle_start_ns=111), dict(cycle_start_ns=0),
                                  dict(car_state_ns=100.), dict(receive_timeout_ns=-1)])
def test_bad_clock_contract_is_an_error(change):
  with pytest.raises(ValueError):
    call(**change)
