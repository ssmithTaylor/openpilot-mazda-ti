"""Reconstruct pinned legacy NNFF/PID output offline; no applied-CAN or physical qualification."""
import argparse
import ast
from collections import Counter
from bisect import bisect_right
import hashlib
import json
import math
from numbers import Number
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
import numpy as np

from tools.mazda_ti import legacy_nnff_runtime as h

HELPER = Path(h.__file__).resolve()
from tools.mazda_ti.legacy_input_constraints import curvature_compatible
from tools.mazda_ti.legacy_receipt_constraints import sequence_compatible


def previous_limit(previous_output, reported_output):
    """controlsd computes this after its update; consume it on the NEXT update."""
    return abs(previous_output - reported_output) > 1e-2


def select_serialized(c, mono, streams, times, vm, choice, previous_ids=None, receive_timeout_ns=None):
    """Same bounded search as the prior probe, stricter observation constraints."""
    t = c.lateralControlState.torqueState
    cs_end = bisect_right(times['carState'], mono)
    lp_end = bisect_right(times['liveParameters'], mono)
    candidates = []
    for ce in streams['carState'][max(0, cs_end-4):cs_end]:
        cs = ce.carState
        for pe in streams['liveParameters'][max(0, lp_end-3):lp_end]:
            if receive_timeout_ns is not None and not sequence_compatible(
                    car_state_ns=int(ce.logMonoTime), parameters_ns=int(pe.logMonoTime),
                    previous_car_state_ns=None if previous_ids is None else previous_ids['carState'],
                    previous_parameters_ns=None if previous_ids is None else previous_ids['liveParameters'],
                    cycle_start_ns=int(c.startMonoTime), publication_ns=mono,
                    receive_timeout_ns=receive_timeout_ns):
                continue
            lp = pe.liveParameters
            vm.update_params(max(lp.stiffnessFactor, .1), max(lp.steerRatio, .1))
            actual = -vm.calc_curvature(math.radians(cs.steeringAngleDeg-lp.angleOffsetDeg), cs.vEgo, lp.roll)
            if curvature_compatible(computed_actual=actual, recorded_actual=c.curvature, speed=cs.vEgo,
                                    recorded_actual_accel=t.actualLateralAccel,
                                    recorded_desired=c.desiredCurvature, recorded_desired_accel=t.desiredLateralAccel):
                candidates.append((ce, pe, abs(c.desiredCurvature*cs.vEgo**2-t.desiredLateralAccel)))
    if not candidates:
        return None
    ce, pe, error = (max if choice == 'latest' else min)(candidates,
        key=lambda pair: (int(pair[0].logMonoTime), int(pair[1].logMonoTime)))
    return ce, pe, len(candidates), error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--choice', choices=['earliest', 'latest'], required=True)
    parser.add_argument('--output-bound', choices=['carState', 'controlsState'], required=True)
    parser.add_argument('--constraints', choices=['heuristic', 'serialized'], required=True)
    parser.add_argument('--receipt-sequence', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Preserve existing output')
    req = json.loads(args.request.read_text())
    anchor_start = req['integral_anchor_start_ns']
    if not req['warmup_start_ns'] < anchor_start < req['score_start_ns'] < req['end_ns']:
        raise ValueError('Require warmup < integral anchor < score start < end')
    request_hash = h.sha256(args.request)
    files = [h.under(args.data_root, p) for p in req['rlogs']]
    raw_hashes = {p: h.sha256(f) for p, f in zip(req['rlogs'], files)}
    if raw_hashes != req['raw_sha256']:
        raise ValueError('Raw source mismatch')
    paths = h.SOURCES + ['frogpilot/common/frogpilot_variables.py', 'selfdrive/controls/lib/latcontrol.py',
                        'cereal/messaging/__init__.py', 'msgq_repo/msgq/__init__.py',
                        'msgq_repo/msgq/impl_msgq.cc', 'msgq_repo/msgq/impl_zmq.cc',
                        'msgq_repo/msgq/msgq.cc', 'msgq_repo/msgq/ipc_pyx.pyx', 'msgq_repo/msgq/ipc.cc']
    blobs = {p: subprocess.check_output(['git', 'show', h.REF+':'+p], cwd=h.ROOT) for p in paths}
    receive_timeout_ns = None
    if args.receipt_sequence:
        if args.constraints != 'serialized':
            raise ValueError('Receipt sequence requires serialized observation constraints')
        calls = [n for n in ast.walk(ast.parse(blobs['selfdrive/controls/controlsd.py']))
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'sub_sock'
                 and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == 'carState']
        if len(calls) != 1 or {k.arg: ast.literal_eval(k.value) for k in calls[0].keywords} != {'timeout': 20}:
            raise ValueError('Original carState socket contract changed')
        receive_timeout_ns = 20_000_000
    ns, model = h.original_runtime(blobs)
    ns['Number'] = Number
    h.definitions(blobs['selfdrive/controls/lib/pid.py'], ns, 'original_pid')
    sources = [Path(__file__).resolve(), HELPER, h.ROOT/'tools/mazda_ti/audit_diagnostics.py',
               h.ROOT/'tools/mazda_ti/provenance.py', h.ROOT/'cereal/__init__.py',
               h.ROOT/'tools/mazda_ti/legacy_input_constraints.py',
               h.ROOT/'tools/mazda_ti/legacy_receipt_constraints.py',
               *sorted((h.ROOT/'cereal').glob('*.capnp'))]
    source_hashes = {p.relative_to(h.ROOT).as_posix(): h.sha256(p) for p in sources}
    runtime = h.environment()
    streams = {s: [] for s in ['initData', 'carParams', 'carState', 'controlsState', 'modelV2',
                               'liveParameters', 'liveLocationKalman', 'liveDelay', 'carOutput']}
    for e in h.read_events(files):
        if e.which() in streams:
            streams[e.which()].append(e)
    for stream in streams.values():
        stream.sort(key=lambda e: int(e.logMonoTime))
    times = {s: [int(e.logMonoTime) for e in stream] for s, stream in streams.items()}
    for s, ts in times.items():
        if s not in ['initData', 'carParams'] and ts != sorted(set(ts)):
            raise ValueError('Duplicate source timestamp: '+s)
    cp = streams['carParams'][0].carParams.as_builder()
    if cp.carFingerprint != 'MAZDA_CX5' or not cp.lateralTuning.torque.useSteeringAngle:
        raise ValueError('Unsupported vehicle configuration')
    settings = []
    for e in streams['initData']:
        values = {v.key: bytes(v.value).decode() for v in e.initData.params.entries
                  if v.key in ['SteerKP', 'NNFF', 'NNFFLite']}
        if values != {'SteerKP': '1.000000', 'NNFF': '1', 'NNFFLite': '1'}:
            raise ValueError('Unexpected startup settings')
        settings.append(values)
    if not settings or cp.lateralTuning.torque.kp != 1.0:
        raise ValueError('Cannot establish kp: requires recorded custom and stock both 1')
    def forbidden(*args):
        raise ValueError('Unexpected fallback torque path')
    ctrl = ns['LatControlNNFF'](cp, NS(torque_from_lateral_accel=lambda: forbidden), .01)
    vm = ns['VehicleModel'](cp)
    models = {int(e.logMonoTime): e for e in streams['modelV2']}
    rows = []
    counts = Counter()
    last_delay = None
    limited = False
    previous_mono = None
    anchor = None
    previous_original_i = None
    previous_ids = None
    for e in streams['controlsState']:
        mono = int(e.logMonoTime)
        if not req['warmup_start_ns'] <= mono < req['end_ns']:
            continue
        c = e.controlsState
        if c.lateralControlState.which() != 'torqueState' or not c.lateralControlState.torqueState.active:
            raise ValueError(f'Inactive/unexpected controller at {mono}; this bounded probe requires continuous activity')
        if previous_mono is not None and mono-previous_mono > 30_000_000:
            raise ValueError('Controller publication gap exceeds declared 30ms')
        previous_mono = mono
        t = c.lateralControlState.torqueState
        if args.constraints == 'serialized':
            selected = select_serialized(c, mono, streams, times, vm, args.choice, previous_ids, receive_timeout_ns)
        else:
            selected = h.select_inputs(c, mono, streams, times, vm, args.choice)
        if selected is None:
            raise ValueError(f'Unresolved active history at {mono}')
        ce, pe, n, _ = selected
        previous_ids = {'carState': int(ce.logMonoTime), 'liveParameters': int(pe.logMonoTime)}
        cs, lp = ce.carState, pe.liveParameters
        le = h.asof(streams['liveLocationKalman'], times['liveLocationKalman'], mono)
        de = h.asof(streams['liveDelay'], times['liveDelay'], mono)
        me = models.get(int(c.lateralPlanMonoTime))
        bound = int(ce.logMonoTime) if args.output_bound == 'carState' else mono
        oe = h.asof(streams['carOutput'], times['carOutput'], bound)
        if any(x is None for x in [le, de, me, oe]) or not all(x.valid for x in [e, ce, pe, le, de, me, oe]):
            raise ValueError('Missing/unhealthy input')
        if int(me.logMonoTime) > mono or len(me.modelV2.orientation.x) < ns['CONTROL_N']:
            raise ValueError('Future model or fallback model')
        if int(de.logMonoTime) != last_delay:
            ctrl.update_live_delay(de.liveDelay.lateralDelay)
            last_delay = int(de.logMonoTime)
        vm.update_params(max(lp.stiffnessFactor, .1), max(lp.steerRatio, .1))
        i_before = float(ctrl.pid.i)
        output, _, logged = ctrl.update(True, cs, vm, lp, limited, c.desiredCurvature, False,
                                       de.liveDelay.lateralDelay+ns['LAT_SMOOTH_SECONDS'],
                                       le.liveLocationKalman, me.modelV2, NS(nnff=True, nnff_lite=True))
        counts['updates'] += 1
        counts['ambiguous_inputs'] += n > 1
        if anchor is None and mono >= anchor_start:
            # One post-update recorded state anchor, never repeated or fitted.
            ctrl.pid.i = float(t.i)
            anchor = {'mono_ns': mono, 'post_update_i': float(t.i),
                      'initial_output_for_next_limit': float(t.output)}
            output = float(t.output)
        elif anchor is not None:
            row = {'mono_ns': mono, 'input_candidates': n, 'i_before': i_before,
                   'cycle_start_ns': int(c.startMonoTime),
                   'scored': mono >= req['score_start_ns'], 'original_i_before': previous_original_i,
                   'steer_limited_previous_cycle': bool(limited), 'steering_pressed_input': bool(cs.steeringPressed),
                   'freeze_integrator': bool(limited or cs.steeringPressed or cs.vEgo < 5),
                   'input_mono_ns': {x.which(): int(x.logMonoTime) for x in [ce, pe, le, de, me, oe]},
                   'car_output_age_ns': mono-int(oe.logMonoTime),
                   'reported_output': float(oe.carOutput.actuatorsOutput.steer)}
            for name in ['error', 'p', 'i', 'd', 'f', 'output']:
                row['original_'+name] = float(getattr(t, name))
                row['reproduced_'+name] = float(getattr(logged, name))
                row[name+'_residual'] = float(getattr(logged, name))-float(getattr(t, name))
            rows.append(row)
        previous_original_i = float(t.i)
        # Update AFTER the controller as in original publish_logs. Baseline uses
        # its own reconstructed output after the single declared state anchor.
        limited = previous_limit(float(output), float(oe.carOutput.actuatorsOutput.steer))
    scored = [r for r in rows if r['scored']]
    if not scored:
        raise ValueError('No scored updates')
    summary = {'counts': dict(counts), 'scored_frames': len(scored), 'absolute_residuals': {},
               'exact_float32_frames': {}, 'freeze_frames': sum(r['freeze_integrator'] for r in scored)}
    for name in ['error', 'p', 'i', 'd', 'f', 'output']:
        summary['absolute_residuals'][name] = dict(zip(['median', 'p95', 'p99', 'max'],
            map(float, np.quantile([abs(r[name+'_residual']) for r in scored], [.5, .95, .99, 1]))))
        summary['exact_float32_frames'][name] = sum(r[name+'_residual'] == 0 for r in scored)
    result = {'scope': __doc__, 'qualification': 'exploratory_not_qualified_command_baseline',
              'request': req, 'request_sha256': request_hash, 'raw_sha256': raw_hashes,
              'source_sha256': source_hashes, 'runtime': runtime, 'git_revision': h.REF,
              'git_blob_sha256': {p: hashlib.sha256(b).hexdigest() for p, b in blobs.items()},
              'settings': settings, 'pid_gains': cp.lateralTuning.torque.to_dict(),
              'input_choice': args.choice, 'output_publication_bound': args.output_bound,
              'constraint_method': args.constraints,
              'receipt_sequence': args.receipt_sequence, 'receive_timeout_ns': receive_timeout_ns,
              'anchor': anchor, 'summary': summary, 'rows': rows,
              'limits': ['CS/LP observation compatibility and LLK/delay latest publication are not consumed identities.',
                         'Receipt sequence assumes normal ordered non-conflating socket operation, without allocation failure, fake backend or publisher restart.',
                         'carOutput bound is a declared receipt hypothesis; its output is recorded, not candidate-driven.',
                         'Original PID and neural code execute; saturation alert facade remains unqualified.',
                         'Logged Float32 desired curvature and one Float32 integral anchor replace unlogged internal precision.',
                         'No applied CAN reconstruction or physical simulation; no tuning or device changes.']}
    if h.sha256(args.request) != request_hash or raw_hashes != {p: h.sha256(f) for p, f in zip(req['rlogs'], files)}:
        raise ValueError('Input mutation')
    if source_hashes != {p.relative_to(h.ROOT).as_posix(): h.sha256(p) for p in sources} or runtime != h.environment():
        raise ValueError('Source/runtime mutation')
    h.write_json(args.output, result)
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
