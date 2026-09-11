import pytest

from tools.mazda_ti.audit_lane_motion_balance import validate_request


def test_declared_window_can_be_validated_without_raw_recordings():
  validate_request({'cases':{'a':{'bounds_ns':[1,2],'data_root':'old','raw_sha256':{'route--0/rlog':'hash'}}}})


@pytest.mark.parametrize('change', [{'bounds_ns':[2,1]}, {'bounds_ns':[1.,2.]},
    {'bounds_ns':[True,2]}, {'data_root':'elsewhere'}, {'raw_sha256':{}},
    {'raw_sha256':{'route--0/rlog':'hash','other--1/rlog':'hash'}},
    {'raw_sha256':{'route--0/qlog':'hash'}}])
def test_invalid_case_is_rejected_before_extraction(change):
  c = {'bounds_ns':[1,2],'data_root':'old','raw_sha256':{'route--0/rlog':'hash'}}
  c.update(change)
  with pytest.raises(ValueError):
    validate_request({'cases':{'a':c}})
