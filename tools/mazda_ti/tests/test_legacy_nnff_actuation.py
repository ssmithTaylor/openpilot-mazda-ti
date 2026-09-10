from types import SimpleNamespace as NS

import pytest

from tools.mazda_ti.feedback import Request, pure_limiter, pure_stock_limiter
from tools.mazda_ti.legacy_nnff_actuation import EXPECTED, REVISION, replay_dual, echo_request_map, copied_key


def fixture(request, previous):
  ti,_ = pure_limiter(REVISION)
  return NS(prior_requests={1:Request(1,request,True)},requests=[],initial_send=(2,previous),
            initial_state=(2,0),initial_sampled_request=Request(1,request,True),request_identity={3:1},
            limits=NS(**EXPECTED,TI_STEER_DRIVER_FACTOR=1),limiter=ti,
            events=[(3,'state',0),(4,'output',None),(5,'send',None),(6,'output',None)])


def test_historical_scaling_and_previous_apply_feedback_order():
  stock,params,_ = pure_stock_limiter(REVISION)
  f=fixture(.5,325)
  r=replay_dual(f,stock,params,300)
  assert r['sends'][0]['ti_requested']==325
  assert r['sends'][0]['stock_requested']==300
  assert r['sends'][0]['ti']==325 and r['sends'][0]['stock']==300
  assert [x['counts'] for x in r['outputs']]==[325,325]
  assert [x['steer'] for x in r['outputs']]==[.5,.5]


def test_two_limit_histories_cross_zero_at_different_rates():
  stock,params,_ = pure_stock_limiter(REVISION)
  r=replay_dual(fixture(-.5,10),stock,params,10)
  assert [x['counts'] for x in r['outputs']]==[10,-5]
  assert r['sends'][0]['ti']==-5
  assert r['sends'][0]['stock']==-10


@pytest.mark.parametrize('replacement', [{99:0.}, {1:float('nan')}, {1:1.1}])
def test_bad_replacements_do_not_enter_limiter_history(replacement):
  stock,params,_ = pure_stock_limiter(REVISION)
  with pytest.raises(ValueError):
    replay_dual(fixture(.5,0),stock,params,0,replacement)


def test_copy_fields_exclude_both_steering_outputs():
  d = dict(gas=0,brake=0,steeringAngleDeg=0,accel=.1,longControlState='pid',speed=20,curvature=.003,steer=.5,steerOutputCan=300)
  assert copied_key(d)==copied_key(dict(d,steer=-.9,steerOutputCan=-600))


def test_echo_excludes_newer_request_without_using_its_steer():
  frames = [{'state_ns':40,'upper_bound_ns':35,'echo':('old',)},
            {'state_ns':50,'upper_bound_ns':45,'echo':('new',)}]
  commands = {10:{'echo':('old',),'steer':.1},20:{'echo':('old',),'steer':.9},30:{'echo':('new',),'steer':-.5}}
  assert echo_request_map(frames,commands,'latest')[0]=={40:20,50:30}
  assert echo_request_map(frames,commands,'earliest')[0]=={40:10,50:30}
  commands[20]['steer']=-.4
  assert echo_request_map(frames,commands,'latest')[0]=={40:20,50:30}


def test_echo_history_cannot_go_backwards():
  with pytest.raises(ValueError):
    echo_request_map([{'state_ns':40,'upper_bound_ns':35,'echo':('new',)},
                      {'state_ns':50,'upper_bound_ns':45,'echo':('old',)}],
                     {10:{'echo':('old',)},30:{'echo':('new',)}},'latest')


def test_copied_field_match_cannot_select_future_publication():
  assert echo_request_map([{'state_ns':40,'upper_bound_ns':35,'echo':('same',)}],
      {10:{'echo':('same',)},45:{'echo':('same',)}},'latest')[0]=={40:10}
