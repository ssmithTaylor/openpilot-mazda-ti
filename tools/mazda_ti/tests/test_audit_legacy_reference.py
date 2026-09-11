import pytest

from tools.mazda_ti.audit_legacy_reference import REVISION, constants, validate_recorded_configuration


def test_constants_does_not_execute_source():
  assert constants('X = [1, 2]\nY = 1.0\nZ = forbidden()') == {'X': [1, 2], 'Y': 1.0}


def test_recorded_configuration_accepts_only_declared_controller():
  assert validate_recorded_configuration([(REVISION, False, '0', '0')]*2, {.8}) == .8


@pytest.mark.parametrize('snapshots,gains', [([], {.8}), ([(REVISION, True, '0', '0')], {.8}),
    ([('other', False, '0', '0')], {.8}), ([(REVISION, False, '1', '0')], {.8}),
    ([(REVISION, False, '0', '1')], {.8}), ([(REVISION, False, None, '0')], {.8}),
    ([(REVISION, False, '0', '0')], {.8, 1.})])
def test_reject_unqualified_source_or_mode(snapshots, gains):
  with pytest.raises(ValueError):
    validate_recorded_configuration(snapshots, gains)
