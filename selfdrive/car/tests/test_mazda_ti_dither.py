"""TI identification dither: signal generator, gating, and the car controller's steering block with it.

The car controller cannot be constructed without the compiled CAN packer and params store, so its steering block, its
dither reset and its output report are executed from the source (the method tools/mazda_ti/test_diagnostics.py uses),
with the real limiters from selfdrive/car/__init__.py."""
import ast
import math
import random
import struct
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from cereal import car

from openpilot.selfdrive.car.mazda import ti_dither as td
from openpilot.selfdrive.car.mazda.ti_dither import TiDither

ROOT = Path(__file__).resolve().parents[3]
TI_MAX = 600

QUIET = dict(v_ego=25.0, steering_pressed=False, driver_torque=-1.0, blinker=False, base_request=50, ti_max=TI_MAX,
             lat_demand=0.3)


def frames(seconds):
  return int(round(seconds / td.DT))


# ---- signal generator ------------------------------------------------------------------------------------------

@pytest.mark.parametrize('preset', sorted(td.PRESETS))
def test_waveform_is_zero_mean_bounded_and_slew_limited(preset):
  w = np.array([td.waveform(k, preset) for k in range(frames(600))])
  assert abs(w.mean()) < 0.05
  assert np.max(np.abs(w)) <= td.peak_counts(preset) + 1e-9
  assert np.max(np.abs(np.diff(w))) <= td.max_slew_per_frame(preset) + 1e-9
  # Every tone is present at its own amplitude (synchronous detection over whole seconds).
  t = np.arange(len(w)) * td.DT
  for f, a in td.PRESETS[preset]:
    amp = abs(2 * np.mean(w * np.exp(-2j * np.pi * f * t)))
    assert amp == pytest.approx(a, rel=1e-3)


def test_active_preset_stays_well_inside_the_ti_rate_limit():
  # The dither alone may use at most half of the default TI_STEER_DELTA_UP (6 counts per frame), so the controller's
  # own ramp keeps at least half of it; and it peaks below the rack's friction floor (~2.5 sensor counts).
  assert td.max_slew_per_frame() <= 3.0
  assert td.peak_counts() * 0.0544 < 2.5
  # Avoid the slow lane weave (~0.2 Hz) and the natural steering mode near 1.5-1.7 Hz.
  for f, _ in td.PRESETS[td.ACTIVE_PRESET]:
    assert f >= 1.0 and not 1.4 <= f <= 1.9


def test_waveform_is_deterministic_in_the_frame_counter():
  a, b = TiDither(), TiDither()
  out_a = [a.update(True, k, **QUIET) for k in range(frames(5))]
  out_b = [b.update(True, k, **QUIET) for k in range(frames(5))]
  assert out_a == out_b
  # At full envelope the integer dither is the rounded waveform of the frame.
  assert all(a.update(True, k, **QUIET) == int(round(td.waveform(k))) for k in range(1000, 1200))


# ---- gating and envelope ---------------------------------------------------------------------------------------

def test_disabled_is_always_zero():
  d = TiDither()
  assert all(d.update(False, k, **QUIET) == 0 for k in range(frames(30)))


def test_fades_in_over_a_second_and_out_within_a_quarter_second():
  d = TiDither()
  peak_first_half = max(abs(d.update(True, k, **QUIET)) for k in range(frames(0.5)))
  for k in range(frames(0.5), frames(1.0)):
    d.update(True, k, **QUIET)
  assert d.envelope == pytest.approx(1.0)
  assert peak_first_half < td.peak_counts() * 0.55
  # Switch off: zero within FADE_OUT_S and zero from then on.
  k0 = frames(10)
  out = [d.update(False, k0 + k, **QUIET) for k in range(frames(2))]
  assert all(v == 0 for v in out[frames(td.FADE_OUT_S):])
  # The envelope never steps: successive gains differ by at most what one frame of fade allows.
  d = TiDither()
  gains = []
  for k in range(frames(3)):
    d.update(k < frames(1.5), k, **QUIET)
    gains.append(d.gain())
  assert np.max(np.abs(np.diff(gains))) <= math.pi / 2 * td.DT / td.FADE_OUT_S + 1e-12


@pytest.mark.parametrize('change', [
  dict(v_ego=14.9), dict(steering_pressed=True), dict(driver_torque=4.0), dict(driver_torque=-4.5),
  dict(blinker=True), dict(base_request=301), dict(base_request=-301), dict(lat_demand=1.01), dict(lat_demand=-1.2),
])
def test_each_gate_condition_alone_turns_it_off(change):
  assert td.gate_open(**QUIET)
  assert not td.gate_open(**{**QUIET, **change})
  d = TiDither()
  for k in range(frames(2)):
    d.update(True, k, **QUIET)
  out = [d.update(True, frames(2) + k, **{**QUIET, **change}) for k in range(frames(1))]
  assert all(v == 0 for v in out[frames(td.FADE_OUT_S):])


@pytest.mark.parametrize('touch', [dict(steering_pressed=True), dict(driver_torque=6.0), dict(blinker=True)])
def test_holdoff_after_driver_touch_or_blinker(touch):
  d = TiDither()
  for k in range(frames(2)):
    d.update(True, k, **QUIET)
  d.update(True, frames(2), **{**QUIET, **touch})
  out = [d.update(True, frames(2) + 1 + k, **QUIET) for k in range(frames(td.HOLDOFF_S) - 1)]
  assert all(v == 0 for v in out[frames(td.FADE_OUT_S):])
  later = [d.update(True, frames(10) + k, **QUIET) for k in range(frames(2))]
  assert max(abs(v) for v in later) > 0


def test_reset_is_immediate():
  d = TiDither()
  for k in range(frames(2)):
    d.update(True, k, **QUIET)
  d.reset()
  assert d.envelope == 0.0 and d.last == 0
  assert d.update(True, frames(2), **QUIET) == int(round(d.gain() * td.waveform(frames(2))))
  assert d.gain() < 1e-3


# ---- the car controller's steering block ----------------------------------------------------------------------

def _update_nodes():
  tree = ast.parse((ROOT / 'selfdrive/car/mazda/carcontroller.py').read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CarController')
  update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
  body = update.body
  begin = next(i for i, n in enumerate(body)
               if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'apply_steer' for t in n.targets))
  reset = next(i for i, n in enumerate(body)
               if isinstance(n, ast.If) and any(isinstance(c, ast.Attribute) and c.attr == 'reset' for c in ast.walk(n)))
  builder = next(n for n in body if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'new_actuators' for t in n.targets))
  report = next(n for n in body if isinstance(n, ast.If) and any(
    isinstance(t, ast.Attribute) and t.attr == 'steerOutputCan' for t in ast.walk(n)))
  diagnostic = next(n for n in body if isinstance(n, ast.If) and any(
    isinstance(t, ast.Attribute) and t.attr == 'lateral_diagnostics' for t in ast.walk(n)))
  tail = [builder, report, diagnostic]
  full = compile(ast.Module(body=body[begin:reset + 1] + tail, type_ignores=[]), 'mazda-steering-block', 'exec')
  # Without the dither's reset: the steering block as it runs on a host that has no dither at all (the getattr path),
  # i.e. the undithered code.
  bare = compile(ast.Module(body=body[begin:reset] + tail, type_ignores=[]), 'mazda-steering-block-bare', 'exec')
  return full, bare


def _limiters():
  tree = ast.parse((ROOT / 'selfdrive/car/__init__.py').read_text(encoding='utf-8'))
  nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef)
           and n.name in ('apply_driver_steer_torque_limits', 'apply_ti_steer_torque_limits')]
  env = dict(clip=np.clip)
  exec(compile(ast.Module(body=nodes, type_ignores=[]), 'limiters', 'exec'), env)
  return env['apply_driver_steer_torque_limits'], env['apply_ti_steer_torque_limits']


BLOCK, BARE_BLOCK = _update_nodes()
DRIVER_LIMITS, TI_LIMITS = _limiters()


def ccp():
  # CarControllerParams defaults for the TI car (values.py).
  return NS(STEER_MAX=600, STEER_DELTA_UP=10, STEER_DELTA_DOWN=25, STEER_DRIVER_ALLOWANCE=15,
            STEER_DRIVER_MULTIPLIER=1, STEER_DRIVER_FACTOR=1,
            TI_STEER_MAX=TI_MAX, TI_STEER_DELTA_UP=6, TI_STEER_DELTA_DOWN=15, TI_STEER_DRIVER_ALLOWANCE=15,
            TI_STEER_DRIVER_MULTIPLIER=40, TI_STEER_DRIVER_FACTOR=1, TI_STEER_DELTA_UP_KNEE=600,
            TI_STEER_DELTA_UP_HIGH=6)


def host(dither, enabled):
  h = NS(CP=NS(flags=1), ccp=ccp(), apply_steer_last=0, ti_apply_steer_last=0, record_ti_stats=lambda *a: None,
         ti_dither_enabled=enabled, frame=0, ti_dither_seen=False, ti_dither_frames=0,
         ti_reported_last=0)
  if dither:
    h.ti_dither = TiDither()
  return h


def step(h, steer, *, lat_active=True, ti_allowed=True, v=25.0, driver=-1.0, pressed=False, blinker=False,
         curvature=0.0003):
  cc = car.CarControl.new_message(latActive=lat_active, leftBlinker=blinker)
  cc.actuators.steer = steer
  cc.actuators.curvature = curvature
  cc = cc.as_reader()
  cs = NS(out=NS(vEgo=v, steeringPressed=pressed, steeringTorque=driver, leftBlinker=False, rightBlinker=False),
          ti_lkas_allowed=ti_allowed)
  env = dict(self=h, CC=cc, CS=cs, car=car, MazdaFlags=NS(TORQUE_INTERCEPTOR=1),
             apply_driver_steer_torque_limits=DRIVER_LIMITS, apply_ti_steer_torque_limits=TI_LIMITS,
             rate_room=td.rate_room)
  exec(BLOCK if hasattr(h, 'ti_dither') else BARE_BLOCK, env)
  h.frame += 1
  a = env['new_actuators']
  return dict(requested=env['ti_new_steer'], sent=env['ti_apply_steer'], dither=env['ti_dither'], steer=a.steer,
              can=a.steerOutputCan, diag_req=h.lateral_diagnostics.tiRequested,
              diag_lim=h.lateral_diagnostics.tiLimited, prev=env['ti_previous'], cc_steer=cc.actuators.steer)


def drive(seed, n=frames(60)):
  """A random but plausible request sequence with the gate's inputs moving in and out of range."""
  rng = random.Random(seed)
  steer, rows = 0.0, []
  for k in range(n):
    steer = max(-1.0, min(1.0, steer + rng.gauss(0, 0.004) - 0.002 * steer))
    seg = (k // 700) % 7
    rows.append(dict(steer=steer if seg != 5 else 0.9 * math.copysign(1, steer or 1),
                     lat_active=seg != 3, ti_allowed=seg != 4, v=25.0 if seg != 2 else 12.0,
                     driver=-1.0 if seg != 1 else 9.0, pressed=seg == 1, blinker=seg == 6 and k % 700 < 200))
  return rows


@pytest.mark.parametrize('seed', range(4))
def test_toggle_off_is_bit_identical_to_the_undithered_path(seed):
  off, ref = host(True, False), host(False, False)
  for row in drive(seed):
    a, b = step(off, **row), step(ref, **row)
    assert a == b
    assert a['dither'] == 0
    if row['lat_active'] and row['ti_allowed']:
      # the expression carcontroller reported before the dither existed, as stored (float32)
      assert a['steer'] == struct.unpack('f', struct.pack('f', a['sent'] / TI_MAX))[0]


@pytest.mark.parametrize('seed', range(4))
def test_dither_on_never_exceeds_the_existing_limits_and_is_recoverable(seed):
  on = host(True, True)
  saw = 0
  for row in drive(seed):
    prev, shadow = on.ti_apply_steer_last, on.ti_reported_last
    r = step(on, **row)
    if not (row['lat_active'] and row['ti_allowed']):
      assert r['dither'] == 0 and r['sent'] == 0
      continue
    base = int(round(r['cc_steer'] * TI_MAX))
    assert r['requested'] == base + r['dither']
    assert abs(r['dither']) <= td.peak_counts()
    # The total went through the unchanged limiter: magnitude, driver torque and rate bounds all hold.
    assert r['sent'] == TI_LIMITS(r['requested'], prev, row['driver'], on.ccp)
    assert abs(r['sent']) <= TI_MAX
    up = on.ccp.TI_STEER_DELTA_UP
    assert -max(up, on.ccp.TI_STEER_DELTA_DOWN) <= r['sent'] - prev <= max(up, on.ccp.TI_STEER_DELTA_DOWN)
    # Reconstruction from logged fields only.
    assert r['can'] == r['sent'] == r['diag_lim']
    # Reported: the limiter's output for the undithered request on the undithered history while the dither is
    # running, else the command sent. Applied dither = sent - reported, bounded by the dither's peak.
    reported = int(round(r['steer'] * TI_MAX))
    if on.ti_dither.envelope > 0.0:
      assert reported == TI_LIMITS(base, shadow, row['driver'], on.ccp)
    else:
      assert reported == r['sent']
    applied = r['can'] - reported
    assert abs(applied) <= td.peak_counts()
    if r['sent'] == r['requested'] and reported == base:
      assert applied == r['dither']
    assert r['diag_req'] - int(round(r['cc_steer'] * TI_MAX)) == r['dither']
    saw += r['dither'] != 0
  assert saw > 1000


def test_dither_on_reports_steer_without_the_dither_so_the_limited_flag_is_unchanged():
  on = host(True, True)
  for k in range(frames(3)):
    step(on, 0.05)
  assert on.ti_dither.envelope == 1.0
  rows = [step(on, 0.05) for _ in range(frames(5))]
  assert max(abs(r['dither']) for r in rows) >= 15
  # Unlimited frames: the reported output equals the request to well within controlsd's 1e-2 test, while the command
  # actually sent carries the dither.
  for r in rows:
    assert abs(r['steer'] - r['cc_steer']) <= 0.5 / TI_MAX + 1e-6
    assert r['can'] == int(round(r['cc_steer'] * TI_MAX)) + r['dither']


def test_lateral_off_or_ti_not_running_zeroes_the_dither_at_once():
  on = host(True, True)
  for k in range(frames(3)):
    step(on, 0.05)
  assert on.ti_dither.envelope == 1.0
  r = step(on, 0.05, ti_allowed=False)
  assert r['dither'] == 0 and on.ti_dither.envelope == 0.0
  for k in range(frames(3)):
    step(on, 0.05)
  r = step(on, 0.05, lat_active=False)
  assert r['dither'] == 0 and r['sent'] == 0 and on.ti_dither.envelope == 0.0


def test_driver_touch_zeroes_the_dither_at_once():
  for touch in (dict(pressed=True, driver=9.0), dict(driver=4.5), dict(driver=-6.0)):
    on = host(True, True)
    for k in range(frames(3)):
      step(on, 0.05)
    assert on.ti_dither.envelope == 1.0
    r = step(on, 0.05, **touch)
    assert r['dither'] == 0 and on.ti_dither.envelope == 0.0
  # A blinker still fades (it is not a touch).
  d = TiDither()
  for k in range(frames(2)):
    d.update(True, k, **QUIET)
  d.update(True, frames(2), **{**QUIET, 'blinker': True})
  assert 0.0 < d.envelope < 1.0


@pytest.mark.parametrize('preset', sorted(td.PRESETS))
def test_defaults_leave_room_and_a_tight_rate_limit_switches_the_dither_off(preset):
  limits = ccp()
  assert td.rate_room(limits) == 6 and td.rate_room_ok(td.rate_room(limits), preset)
  for up in (1, 2, 3, 5):
    tight = NS(**{**vars(limits), 'TI_STEER_DELTA_UP': up})
    assert td.rate_room(tight) == up
    assert td.rate_room_ok(up, preset) == (up >= td.RATE_ROOM_FACTOR * td.max_slew_per_frame(preset))
  assert not td.rate_room_ok(5, preset)
  # A knee inside the dither's working range brings in the high-range rate.
  kneed = NS(**{**vars(limits), 'TI_STEER_DELTA_UP_KNEE': 200, 'TI_STEER_DELTA_UP_HIGH': 2})
  assert td.rate_room(kneed) == 2
  d = TiDither(preset)
  out = [d.update(True, k, **QUIET, room=1) for k in range(frames(5))]
  assert not any(out) and d.envelope == 0.0


def test_car_controller_with_delta_up_1_sends_no_dither():
  on = host(True, True)
  on.ccp.TI_STEER_DELTA_UP = 1
  on.ccp.TI_STEER_DELTA_UP_HIGH = 1
  rows = [step(on, 0.01) for _ in range(frames(5))]
  assert all(r['dither'] == 0 for r in rows)


def test_clipped_frames_report_the_undithered_limiter_output():
  """Where the limiter clips the dithered total (a fast ramp), the reported command is what the limiter sends for the
  undithered request from the same state, so controlsd's limited flag reflects the controller's own request."""
  on, ref = host(True, True), host(True, False)
  for k in range(frames(3)):
    step(on, 0.02)
    step(ref, 0.02)
  ref_rows = []
  for k in range(frames(1)):
    m = k % 30 if (k // 30) % 2 == 0 else 30 - k % 30
    ref_rows.append(step(ref, (20 + 7 * m) / TI_MAX))
  saw_clip = 0
  for k in range(frames(1)):
    prev, shadow = on.ti_apply_steer_last, on.ti_reported_last
    m = k % 30 if (k // 30) % 2 == 0 else 30 - k % 30
    r = step(on, (20 + 7 * m) / TI_MAX)    # a 7 count/frame zig-zag below 300: the rising half is rate limited
    base = int(round(r['cc_steer'] * TI_MAX))
    assert r['sent'] == TI_LIMITS(base + r['dither'], prev, -1.0, on.ccp)
    assert int(round(r['steer'] * TI_MAX)) == TI_LIMITS(base, shadow, -1.0, on.ccp)
    # The undithered history is exactly the run without the dither.
    assert int(round(r['steer'] * TI_MAX)) == ref_rows[k]['sent']
    saw_clip += r['dither'] != 0 and r['sent'] != base + r['dither']
  assert saw_clip > 10


def _ti_config():
  tree = ast.parse((ROOT / 'selfdrive/car/mazda/carcontroller.py').read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CarController')
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'ti_config')
  env = dict(TI_DITHER_PRESET=td.ACTIVE_PRESET)
  exec(compile(ast.Module(body=[fn], type_ignores=[]), 'ti_config', 'exec'), env)
  return env['ti_config']


def test_stats_stamp_names_dithered_runs_only():
  cfg = _ti_config()
  off = host(True, False)
  off.ti_steer_threshold = 6
  for k in range(frames(3)):
    step(off, 0.05)
  assert 'TiDither' not in cfg(off)
  on = host(True, True)
  on.ti_steer_threshold = 6
  for k in range(frames(3)):
    step(on, 0.05)
  stamp = cfg(on)['TiDither']
  assert stamp['preset'] == td.ACTIVE_PRESET and stamp['frames'] == on.ti_dither_frames > 100
  assert {k: v for k, v in cfg(on).items() if k != 'TiDither'} == cfg(off)
