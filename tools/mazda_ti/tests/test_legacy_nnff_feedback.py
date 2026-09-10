import numpy as np
import pytest

from tools.mazda_ti.legacy_nnff_feedback import feedback_error,isolate_rates


def model(v):
  return v[1]+2*v[2]


def test_rates_are_isolated_without_changing_feature_arrays():
  sp=[20.,2.,-.2,0.]+[2.]*14
  ms=[20.,1.9,.1,0.]+[1.9]*14
  old_sp,old_ms=list(sp),list(ms)
  r=isolate_rates(model,sp,ms,2.)
  assert r['original']==pytest.approx(-.5)
  assert r['zero_both_rates']==pytest.approx(.1)
  assert r['zero_desired_rate']==pytest.approx(-.1)
  assert r['zero_measured_rate']==pytest.approx(-.3)
  assert (sp,ms)==(old_sp,old_ms)


def test_high_demand_blend_preserves_sign_and_float32_stores():
  def neural(v):
    return 3*v[1] if len(v)==4 else v[1]
  sp=[20.,2.,0.,0.]+[0.]*14
  ms=[20.,1.,0.,0.]+[0.]*14
  assert feedback_error(neural,sp,ms,1.)==1.
  assert feedback_error(neural,sp,ms,1.5)==2.
  assert feedback_error(neural,sp,ms,2.)==3.
  assert feedback_error(lambda v: -3*v[1] if len(v)==4 else v[1],sp,ms,2.)==1.


@pytest.mark.parametrize('values',[[0.]*17,[float('nan')]+[0.]*17])
def test_invalid_feature_vectors_fail_closed(values):
  with pytest.raises(ValueError): feedback_error(model,values,[0.]*18,2.)
