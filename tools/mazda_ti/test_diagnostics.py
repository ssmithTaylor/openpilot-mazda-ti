# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Schema and actual call-boundary checks without device sockets or raw drives."""

import ast
import math
import struct
from types import SimpleNamespace as NS

import pytest
import numpy as np

from .runtime import ROOT, TestInterface as PlantInterface, bootstrap

controller_module = bootstrap()
from cereal import car, custom, log
from openpilot.selfdrive.car.mazda.lateral_diagnostics import INPUT_SERVICES, record_inputs
from openpilot.selfdrive.car.mazda.lateral_reference import LaneContext


def synthetic_controller():
  # A controller/schema fixture, not recorded vehicle parameters or a motion model.
  cp = car.CarParams.new_message(steerLimitTimer=1.0)
  tune = cp.lateralTuning.init('torque')
  tune.kp, tune.ki, tune.latAccelFactor = 0.8, 0.1, 3.0
  return controller_module.LatControlTorque(cp, PlantInterface(), 0.01)


def f32(value):
  return struct.unpack('f', struct.pack('f', value))[0]


@pytest.mark.parametrize('direction', [-1, 1])
def test_controller_diagnostics_reconstruct_commands_and_reference_release(direction):
  ctrl = synthetic_controller()
  params = log.LiveParametersData.new_message()
  cs = car.CarState.new_message(vEgo=25.0, steeringPressed=False)
  vm = NS(calc_curvature=lambda *args: -direction * 3.0 / 625)
  toggles = NS(lat_commit_setpoint=True, lat_damping=True, lat_friction_comp=True, lat_output_filter=True, lat_no_friction_relay=False, ti_steer_max=600.0)
  fp = NS(lkasBlocked=False, lkasEffective=-direction * 308.0, tiActive=True, columnTorque=160.0)
  ctrl._release_context = LaneContext(True, direction * 0.5, 0.0, 0.0, 0.1, 'valid', direction * 0.0032, direction * 0.0032)
  saw_release = saw_compensation = saw_freeze = False
  for frame in range(450):
    cs.steeringPressed = 410 <= frame < 420
    limited = 420 <= frame < 430
    steer, _, msg = ctrl.update(True, cs, vm, params, limited, direction * (3.0 if frame < 300 else 2.4) / 625, False, 0.48, None, None, toggles, fp)
    with log.ControlsState.LateralTorqueState.from_bytes(msg.to_bytes()) as reader:
      d = reader.mazdaDiagnostics
      assert d.version == 1
      assert d.command / d.tiMax == -steer
      assert reader.output == f32(steer)
      assert d.effectiveFeedforward == d.committedFeedforward - direction * d.removedFeedforward
      assert d.effectiveSetpoint == d.committedSetpoint - direction * d.removedSetpoint
      assert d.freezeReasons == int(limited) | (int(cs.steeringPressed) << 1)
      if d.freezeReasons:
        assert d.integralAfter == d.integralBefore
        saw_freeze = True

      def clip(value, limit=d.tiMax):
        return max(-limit, min(limit, value))

      reconstructed = clip(clip(clip(d.inverseCommand + d.frictionCompensation) + d.frictionRelay) + d.breakerBoost)
      assert d.commandBeforeSmoothing == reconstructed
      assert d.command == clip(reconstructed + d.outputFilterBlend * (d.outputFilter - reconstructed))
      assert reader.frictionTorque == f32((d.frictionRelay + d.breakerBoost) / d.tiMax)
      assert d.settings == 15
      assert math.isfinite(d.pidOutput)
      saw_release |= max(d.removedFeedforward, d.removedSetpoint) > 0.01
      saw_compensation |= abs(d.frictionCompensation) > 1
  assert saw_release and saw_compensation and saw_freeze
  _, _, inactive = ctrl.update(False, cs, vm, params, False, 0.0, False, 0.48, None, None, toggles, fp)
  assert not inactive.active
  assert inactive.mazdaDiagnostics.version == 1
  assert inactive.mazdaDiagnostics.command == 0
  assert inactive.mazdaDiagnostics.removedFeedforward == 0


def test_controlsd_records_snapshot_at_actual_call_site_only_for_plant():
  tree = ast.parse((ROOT / 'selfdrive/controls/controlsd.py').read_text(encoding='utf-8'))
  candidates = [
    node
    for node in ast.walk(tree)
    if isinstance(node, ast.If) and any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'record_inputs' for n in ast.walk(node))
  ]
  boundary = min(candidates, key=lambda node: len(list(ast.walk(node))))
  code = compile(ast.Module(body=[boundary], type_ignores=[]), 'controlsd-diagnostics-boundary', 'exec')
  host = NS(LaC=synthetic_controller(), sm=InputFixture())
  for enabled in (True, False):
    if not enabled:
      host.LaC.plant = None
    msg = log.ControlsState.LateralTorqueState.new_message()
    exec(code, dict(self=host, lac_log=msg, record_inputs=record_inputs, LatControlTorque=controller_module.LatControlTorque))
    assert len(msg.mazdaDiagnostics.inputs) == (len(INPUT_SERVICES) if enabled else 0)


def test_controlsd_passes_both_clock_domains_to_lane_observer():
  tree = ast.parse((ROOT / 'selfdrive/controls/controlsd.py').read_text(encoding='utf-8'))
  candidates = [node for node in ast.walk(tree) if isinstance(node, ast.If) and
                any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'update_model_context'
                    for n in ast.walk(node))]
  boundary = min(candidates, key=lambda node: len(list(ast.walk(node))))
  calls = []
  class Controller:
    def update_model_context(self, *args):
      calls.append(args)
  clock_calls = []
  def boot_clock(clock):
    clock_calls.append(clock)
    return 11_100_000_000
  sm = InputFixture()
  host = NS(LaC=Controller(), sm=sm)
  message = object()
  exec(compile(ast.Module(body=[boundary], type_ignores=[]), 'actual-clock-boundary', 'exec'),
       dict(self=host, model_v2=message, LatControlTorque=Controller,
            time=NS(monotonic_ns=lambda: 1_100_000_000, clock_gettime_ns=boot_clock, CLOCK_BOOTTIME=7)))
  assert calls == [(message, sm.logMonoTime['modelV2'], True, 1_100_000_000, 11_100_000_000)]
  assert clock_calls == [7]


def test_published_car_control_links_matching_controls_state_without_changing_request():
  tree = ast.parse((ROOT / 'selfdrive/controls/controlsd.py').read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'publish_logs')
  start = next(i for i, n in enumerate(method.body) if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'cc_send' for t in n.targets))
  block = method.body[start : start + 5]
  assert isinstance(block[-1], ast.Expr)
  sent = []

  def new_message(kind):
    message = log.Event.new_message()
    message.init(kind)
    return message

  cc = car.CarControl.new_message(latActive=True)
  cc.actuators.steer = 0.123
  env = dict(
    CC=cc,
    CS=NS(canValid=True),
    dat=NS(logMonoTime=91234567890123456),
    messaging=NS(new_message=new_message),
    self=NS(pm=NS(send=lambda name, msg: sent.append(msg.to_bytes()))),
  )
  exec(compile(ast.Module(body=block, type_ignores=[]), 'actual-carControl-publish', 'exec'), env)
  with log.Event.from_bytes(sent[0]) as reader:
    assert reader.carControl.controlsStateMonoTime == env['dat'].logMonoTime
    assert reader.carControl.actuators.steer == cc.actuators.steer
    assert reader.carControl.latActive
  assert cc.controlsStateMonoTime == 0


class InputFixture:
  def __init__(self, services=INPUT_SERVICES):
    self.logMonoTime = {s: 123456789123456789 + i for i, s in enumerate(services)}
    self.valid = {s: True for s in services}
    self.alive = {s: True for s in services}
    self.freq_ok = {s: True for s in services}
    self.updated = {s: False for s in services}
    self.seen = {s: True for s in services}
    self.frame = 1

  def all_checks(self, services):
    return all(self.valid[s] and self.alive[s] and self.freq_ok[s] for s in services)

  def all_alive(self, services):
    return all(self.alive[s] for s in services)


def test_input_snapshot_preserves_identity_health_and_serializes():
  sm = InputFixture()
  sm.updated['carState'] = True
  sm.alive['frogpilotCarState'] = False
  sm.valid['modelV2'] = False
  sm.freq_ok['liveParameters'] = False
  msg = log.ControlsState.LateralTorqueState.new_message()
  diag = msg.init('mazdaDiagnostics')
  diag.version = 1
  record_inputs(diag, sm)
  sm.logMonoTime['carState'] += 100  # snapshot must not alias later SubMaster state
  with log.ControlsState.LateralTorqueState.from_bytes(msg.to_bytes()) as reader:
    rows = {str(row.service): row for row in reader.mazdaDiagnostics.inputs}
    assert tuple(rows) == INPUT_SERVICES
    assert rows['carState'].logMonoTime == 123456789123456789
    assert rows['carState'].updated
    assert not rows['frogpilotCarState'].alive
    assert not rows['frogpilotCarState'].checksPassed
    assert not rows['modelV2'].valid
    assert not rows['liveParameters'].frequencyOk
    assert rows['carOutput'].checksPassed


def test_absent_diagnostics_are_not_mistaken_for_zero_control_state():
  msg = log.ControlsState.LateralTorqueState.new_message()
  assert msg.mazdaDiagnostics.version == 0
  assert len(msg.mazdaDiagnostics.inputs) == 0
  assert car.CarOutput.new_message().applySequence == 0


def card_fixture():
  """Compile the real two card methods; replace only hardware and transport edges."""
  tree = ast.parse((ROOT / 'selfdrive/car/card.py').read_text(encoding='utf-8'))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Car')
  selected = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in ('controls_update', 'state_publish')]
  assert len(selected) == 2
  sent = []

  def new_message(kind):
    msg = log.Event.new_message()
    msg.init(kind)
    return msg

  clock = NS(now=10.0)
  env = dict(
    car=car,
    custom=custom,
    DT_CTRL=0.01,
    REPLAY=False,
    time=NS(monotonic=lambda: clock.now),
    messaging=NS(new_message=new_message),
    can_list_to_can_capnp=lambda *args, **kwargs: b'sendcan',
  )
  exec(compile(ast.Module(body=selected, type_ignores=[]), 'actual-card-methods', 'exec'), env)
  host = NS(
    sm=InputFixture(('carControl',)),
    initialized_prev=True,
    last_actuators_output=car.CarControl.Actuators.new_message(),
    last_applied_car_control_mono=0,
    last_apply_mono=0,
    apply_sequence=0,
    last_applied_car_control_checks=False,
    last_mazda_diagnostics=None,
    pm=NS(send=lambda name, msg: sent.append((name, msg.to_bytes() if name != 'sendcan' else msg))),
    can_rcv_cum_timeout_counter=0,
    rk=NS(remaining=0.01),
    frogpilot_toggles=NS(),
  )
  applied = []

  def apply(cc, now, toggles):
    applied.append((cc.actuators.steer, now))
    return car.CarControl.Actuators.new_message(steer=cc.actuators.steer, steerOutputCan=123), []

  host.CI = NS(apply=apply, CC=NS(lateral_diagnostics=car.CarOutput.MazdaActuatorDiagnostics.new_message(version=1, tiLimited=123)))
  cs = car.CarState.new_message(canValid=True)
  fpcs = custom.FrogPilotCarState.new_message()
  cc = car.CarControl.new_message()
  cc.actuators.steer = 0.25

  def publish():
    env['state_publish'](host, cs, fpcs)
    data = next(data for name, data in reversed(sent) if name == 'carOutput')
    with log.Event.from_bytes(data) as reader:
      return reader.to_dict()

  return host, cc, clock, applied, publish, lambda: env['controls_update'](host, cs, cc)


def test_car_output_identity_tracks_previous_apply_and_survives_skipped_apply():
  host, cc, clock, applied, publish, update = card_fixture()
  first = publish()['carOutput']
  assert first['applySequence'] == 0
  identity = host.sm.logMonoTime['carControl']
  update()
  host.sm.logMonoTime['carControl'] = identity + 1
  second = publish()['carOutput']
  assert second['appliedCarControlMonoTime'] == identity  # not the newer sampled packet
  assert second['applySequence'] == 1
  assert second['appliedAtMonoTime'] == 10_000_000_000
  assert second['actuatorsOutput']['steerOutputCan'] == 123
  assert second['mazdaDiagnostics']['tiLimited'] == 123
  assert second['appliedCarControlChecksPassed']
  host.sm.alive['carControl'] = False
  clock.now = 11.0
  update()
  skipped = publish()
  assert not skipped['valid']
  assert skipped['carOutput'] == second  # valid belongs to publication; checks belong to the prior apply
  assert len(applied) == 1
  host.sm.alive['carControl'] = True
  host.sm.valid['carControl'] = False  # card's existing apply gate uses alive, not all_checks
  update()
  third = publish()['carOutput']
  assert third['applySequence'] == 2
  assert third['appliedCarControlMonoTime'] == identity + 1
  assert third['appliedAtMonoTime'] == 11_000_000_000
  assert not third['appliedCarControlChecksPassed']


def test_failed_apply_does_not_claim_a_new_command():
  host, _, _, _, publish, update = card_fixture()

  def fail(*args):
    raise RuntimeError('apply failed')

  host.CI.apply = fail
  with pytest.raises(RuntimeError, match='apply failed'):
    update()
  assert publish()['carOutput']['applySequence'] == 0


def test_mazda_actuator_log_uses_limited_counts_previous_state_and_live_settings():
  # Execute the actual steering calculation and diagnostic blocks. CAN packing,
  # longitudinal controls and device Params are outside this test's boundary.
  tree = ast.parse((ROOT / 'selfdrive/car/mazda/carcontroller.py').read_text(encoding='utf-8'))
  update = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'update')
  begin = next(i for i, n in enumerate(update.body) if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'apply_steer' for t in n.targets))
  end = next(
    i
    for i, n in enumerate(update.body)
    if isinstance(n, ast.Assign) and any(isinstance(t, ast.Attribute) and t.attr == 'ti_apply_steer_last' for t in n.targets)
  )
  diagnostic = next(
    n for n in update.body if isinstance(n, ast.If) and any(isinstance(t, ast.Attribute) and t.attr == 'lateral_diagnostics' for t in ast.walk(n))
  )
  code = compile(ast.Module(body=update.body[begin : end + 1] + [diagnostic], type_ignores=[]), 'actual-mazda-steering-blocks', 'exec')
  functions = ast.parse((ROOT / 'selfdrive/car/__init__.py').read_text(encoding='utf-8'))
  limiters = [n for n in functions.body if isinstance(n, ast.FunctionDef) and n.name in ('apply_driver_steer_torque_limits', 'apply_ti_steer_torque_limits')]
  env = dict(clip=np.clip, car=car, MazdaFlags=NS(TORQUE_INTERCEPTOR=1))
  exec(compile(ast.Module(body=limiters, type_ignores=[]), 'actual-steering-limiters', 'exec'), env)
  limits = NS(
    STEER_MAX=600,
    STEER_DELTA_UP=10,
    STEER_DELTA_DOWN=25,
    STEER_DRIVER_ALLOWANCE=15,
    STEER_DRIVER_MULTIPLIER=40,
    STEER_DRIVER_FACTOR=1,
    TI_STEER_MAX=600,
    TI_STEER_DELTA_UP=10,
    TI_STEER_DELTA_DOWN=15,
    TI_STEER_DRIVER_ALLOWANCE=15,
    TI_STEER_DRIVER_MULTIPLIER=40,
    TI_STEER_DRIVER_FACTOR=1,
    TI_STEER_DELTA_UP_KNEE=630,
    TI_STEER_DELTA_UP_HIGH=9,
  )
  host = NS(CP=NS(flags=1), ccp=limits, apply_steer_last=500, ti_apply_steer_last=500, record_ti_stats=lambda *args: None)
  cc = car.CarControl.new_message(latActive=True)
  cc.actuators.steer = 1.0
  cs = NS(out=NS(steeringTorque=0.0), ti_lkas_allowed=True)
  env.update(self=host, CC=cc, CS=cs)
  exec(code, env)
  first = host.lateral_diagnostics
  assert first.tiRequested == 600 and first.tiLimited == 510
  assert first.stockRequested == 600 and first.stockLimited == 510
  assert first.tiPrevious == 500
  assert first.tiDeltaUp == 10
  limits.TI_STEER_DELTA_UP = 6
  exec(code, env)
  assert host.lateral_diagnostics.tiLimited == 516
  assert host.lateral_diagnostics.tiPrevious == 510
  assert host.lateral_diagnostics.tiDeltaUp == 6
  assert first.tiLimited == 510  # prior snapshot survives next apply
  cs.ti_lkas_allowed = False
  exec(code, env)
  assert host.lateral_diagnostics.tiRequested == 0
  assert host.lateral_diagnostics.tiLimited == 0
  assert host.lateral_diagnostics.stockLimited > 0
  cc.latActive = False
  exec(code, env)
  assert host.lateral_diagnostics.stockRequested == 0
  assert host.lateral_diagnostics.stockLimited == 0
