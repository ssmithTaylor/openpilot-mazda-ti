# ruff: noqa: TID251
"""Synthetic controller/geometry/rlog workload; no sockets or actuator calls."""
import argparse
from pathlib import Path
import time
from types import SimpleNamespace as NS

import numpy as np

from .provenance import environment, finish_sources, source_snapshot, write_json


def run(frames):
  # The fixture reads controlsd's real subscription list, which excludes carState.
  # A separate event exercises the dedicated carState snapshot boundary.
  from .test_diagnostics import actual_controlsd_inputs, synthetic_controller, record_inputs, car, log

  ctrl, sm = synthetic_controller(), actual_controlsd_inputs()
  cs = car.CarState.new_message(vEgo=25.0, canValid=True)
  params = log.LiveParametersData.new_message()
  fp = NS(lkasBlocked=False, lkasEffective=-308.0, tiActive=True, columnTorque=160.0)
  vm = NS(calc_curvature=lambda *_: -3.0 / 625.0)
  toggles = NS(lat_commit_setpoint=True, lat_damping=True, lat_friction_comp=True,
               lat_output_filter=False, lat_no_friction_relay=True, ti_steer_max=600.0)
  x = np.linspace(0, 30, 31)
  mid = -.5 + .0016*x*x
  model = NS(timestampEof=0, meta=NS(laneChangeState='off'), laneLineProbs=[0.0, .99, .99, 0.0],
             laneLines=[NS(), NS(x=x, y=mid-1.8), NS(x=x, y=mid+1.8)])
  times, model_times, sizes = [], [], []
  coverage = dict(withdrawal=0, completed=0, inactive=0)
  for frame in range(frames):
    phase = frame % 800
    active = not 480 <= phase < 500
    request = 2.4 if 300 <= phase < 410 or 440 <= phase < 480 else (3.4 if 410 <= phase < 440 else 3.0)
    cs.steeringRateDeg = 3.0 if 390 <= phase < 410 else 0.0
    now = time.monotonic_ns()
    boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME) if hasattr(time, 'CLOCK_BOOTTIME') else now
    if frame % 5 == 0:
      model.timestampEof, model_mono = boot-100_000_000, now-20_000_000
    cs_event = log.Event.new_message(logMonoTime=now, valid=True)
    cs_event.carState = cs
    started = time.perf_counter_ns()
    if not active:
      ctrl.reset()
    ctrl.update_model_context(model, model_mono, True, now, boot)
    _, _, msg = ctrl.update(active, cs, vm, params, False, request/625.0, False, .48, None, model, toggles, fp)
    record_inputs(msg.mazdaDiagnostics, sm, cs_event, True)
    event = log.Event.new_message(logMonoTime=now)
    event.init('controlsState').lateralControlState.torqueState = msg
    serialized = event.to_bytes()
    elapsed = (time.perf_counter_ns()-started)/1e6
    times.append(elapsed)
    sizes.append(len(serialized))
    if frame % 5 == 0:
      model_times.append(elapsed)
    with log.Event.from_bytes(serialized) as decoded:
      d = decoded.controlsState.lateralControlState.torqueState.mazdaDiagnostics
      assert d.version == 1 and d.frictionReleaseVersion == 1
      assert d.inputs[0].logMonoTime == now and d.inputs[0].checksPassed
      assert d.frictionWithdrawal == ctrl.friction_release.removed
      assert d.frictionReleaseCompleted == ctrl.friction_release.completed
      coverage['withdrawal'] += d.frictionWithdrawal > 0
      coverage['completed'] += d.frictionReleaseCompleted
      coverage['inactive'] += not active
      if not active:
        assert d.frictionWithdrawal == 0 and not d.frictionReleaseCompleted
  if not all(coverage.values()):
    raise RuntimeError('Benchmark did not exercise every required state')
  return dict(frames=frames, median_ms=float(np.median(times)), p99_ms=float(np.percentile(times, 99)),
              max_ms=max(times), new_model_p99_ms=float(np.percentile(model_times, 99)),
              mean_event_bytes=float(np.mean(sizes)), coverage=coverage,
              scope='Synthetic controller, lane fit, diagnostics and serialization; excludes decode checks, transport and concurrent onroad load.')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--frames', type=int, default=2400)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if args.frames < 800 or args.output.exists():
    parser.error('Use at least 800 frames and a new output path')
  before, runtime = source_snapshot(), environment()
  result = run(args.frames)
  if environment() != runtime:
    raise RuntimeError('Runtime identity changed during benchmark')
  result.update(repository_sources=finish_sources(before), environment=runtime)
  result['p99_within_10ms'] = result['p99_ms'] < 10.0
  write_json(args.output, result)
  print({key: value for key, value in result.items() if key not in ('repository_sources', 'environment')})
  return 0 if result['p99_within_10ms'] else 1


if __name__ == '__main__':
  raise SystemExit(main())
