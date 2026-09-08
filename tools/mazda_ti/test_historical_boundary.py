# ruff: noqa: TID251
"""Historical requests can be published after activation using older consumed input."""
from types import SimpleNamespace as NS

import pytest

from .controller_replay import publish_paired_request
from .feedback import TiFeedbackReplay, Request, Output, pure_limiter
from cereal import log


def fixture(controller_mono=101, command_mono=102):
  old = Request(90, .25, True)
  limiter, _ = pure_limiter('HEAD')
  limits = NS(TI_STEER_MAX=600, TI_STEER_DELTA_UP=10, TI_STEER_DELTA_DOWN=15,
              TI_STEER_DRIVER_ALLOWANCE=15, TI_STEER_DRIVER_MULTIPLIER=40, TI_STEER_DRIVER_FACTOR=1,
              TI_STEER_DELTA_UP_KNEE=630, TI_STEER_DELTA_UP_HIGH=9)
  replay = TiFeedbackReplay(NS(start_ns=100, end_ns=120, initial_send=(90,0), initial_output=Output(95,0,0),
                              initial_request=old, initial_sampled_request=old, initial_state=(90,0),
                              prior_requests={90:old}, events=[(104,'state',0),(106,'send',None),(109,'output',None)],
                              request_identity={104:command_mono}, limits=limits, limiter=limiter))
  event = log.Event.new_message(logMonoTime=command_mono)
  event.init('carControl')
  event.carControl.actuators.steer = .25
  event.carControl.actuators.curvature = 0
  decoded = log.Event.from_bytes(event.to_bytes())
  reader = decoded.__enter__()
  streams = NS(times={'carControl':[command_mono]}, rows={'carControl':[(command_mono,reader)]})
  return replay, streams, decoded


@pytest.mark.parametrize('controller_mono,expected', [(101,-10),(99,10)])
def test_published_request_is_available_even_when_consumed_input_precedes_activation(controller_mono,expected):
  replay,streams,decoded = fixture(controller_mono)
  try:
    publish_paired_request(replay,streams,controller_mono,0,.25,-.5,True)
    replay.output_at(109)
    assert replay.sends == [(106,expected)]
  finally:
    decoded.__exit__(None,None,None)


def test_changed_recorded_pair_is_rejected():
  replay,streams,decoded = fixture()
  try:
    with pytest.raises(RuntimeError,match='pairing'):
      publish_paired_request(replay,streams,101,0,.5,-.5,True)
  finally:
    decoded.__exit__(None,None,None)
