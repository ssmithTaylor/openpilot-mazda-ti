# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Reproduce logged lane context from exact consumed model identities, without actuation.

This audits the lane observer, not controller commands, lane selection or vehicle motion.
Run the separate applied-output identity auditor before interpreting road performance.
"""

import argparse
from collections import Counter
import math
from pathlib import Path
from types import SimpleNamespace as NS

from .audit_diagnostics import read_events
from .provenance import environment, finish_sources, sha256, source_snapshot, under, write_json
from .runtime import bootstrap

FIELDS = {
  'age': 'cameraAge', 'offset': 'laneOffset', 'heading10': 'laneHeading10',
  'heading20': 'laneHeading20', 'curvature10': 'laneCurvature10', 'curvature20': 'laneCurvature20',
}
ABS_TOLERANCE = 1e-10  # Geometry/age comparison only; never a command or physical-outcome tolerance.


def compact_model(model):
  return NS(timestampEof=int(model.timestampEof), meta=NS(laneChangeState=str(model.meta.laneChangeState)),
            laneLineProbs=list(model.laneLineProbs),
            laneLines=[NS(x=list(line.x), y=list(line.y)) for line in model.laneLines])


def audit(events, start_ns, end_ns):
  if not 0 < start_ns < end_ns:
    raise ValueError('Require 0 < start_ns < end_ns')
  bootstrap()
  from openpilot.selfdrive.car.mazda.lateral_reference import LaneObserver

  models, controllers = {}, {}
  for event in events:
    service, mono = event.which(), int(event.logMonoTime)
    if service == 'modelV2':
      if mono <= 0 or mono in models:
        raise ValueError(f'Zero or duplicate model identity: {mono}')
      models[mono] = (bool(event.valid), compact_model(event.modelV2))
    elif service == 'controlsState' and start_ns <= mono < end_ns:
      if mono in controllers:
        raise ValueError(f'Duplicate controller identity: {mono}')
      state = event.controlsState.lateralControlState
      if state.which() == 'torqueState':
        controllers[mono] = (bool(state.torqueState.active), state.torqueState.mazdaDiagnostics.to_dict())
      else:
        controllers[mono] = (False, {'version': 0})

  observer = LaneObserver()
  rows, issues, reasons, max_errors = [], [], Counter(), dict.fromkeys(FIELDS, 0.0)
  for mono, (active, d) in sorted(controllers.items()):
    row = {'controlsState': mono, 'active': active, 'complete': False}
    rows.append(row)

    def issue(reason, mono=mono, **details):
      issues.append({'controlsState': mono, 'reason': reason, **details})

    if d.get('version', 0) != 1:
      issue('absent_or_unsupported_diagnostics')
      continue
    inputs = [s for s in d['inputs'] if s['service'] == 'modelV2']
    if len(inputs) != 1:
      issue('invalid_model_snapshot_count')
      continue
    snapshot = inputs[0]
    identity = snapshot['logMonoTime']
    row.update(modelV2=identity, checksPassed=snapshot['checksPassed'])
    if not snapshot['seen'] or not 0 < identity <= mono:
      issue('unseen_or_invalid_model_identity')
      continue
    if identity not in models:
      issue('missing_consumed_model', identity=identity)
      continue
    valid, model = models[identity]
    if snapshot['valid'] != valid:
      issue('model_validity_mismatch')
      continue
    if not 0 < d['modelContextNow'] <= mono or d['cameraContextNow'] <= 0:
      issue('invalid_context_clocks')
      continue
    context = observer.update(model, identity, snapshot['checksPassed'], d['modelContextNow'], d['cameraContextNow'])
    row.update(lane_probabilities=model.laneLineProbs, lane_change_state=model.meta.laneChangeState,
               recorded_reason=d['laneReason'], reproduced_reason=context.reason,
               reproduced_valid=context.valid, camera_age=context.age,
               raw_request=d['rawRequest'], delayed_request=d['delayedRequest'],
               effective_setpoint=d['effectiveSetpoint'], effective_feedforward=d['effectiveFeedforward'],
               removed_setpoint=d['removedSetpoint'], removed_feedforward=d['removedFeedforward'],
               lane_offset=context.offset if context.valid else None)
    count_before = len(issues)
    if context.valid != d['laneValid'] or context.reason != d['laneReason']:
      issue('lane_decision_mismatch', recorded=d['laneReason'], reproduced=context.reason)
    for field, logged in FIELDS.items():
      actual, expected = getattr(context, field), d[logged]
      error = abs(actual - expected)
      if not math.isfinite(actual) or not math.isfinite(expected) or error > ABS_TOLERANCE:
        issue('lane_value_mismatch', field=field)
      if math.isfinite(error):
        max_errors[field] = max(max_errors[field], error)
    row['complete'] = len(issues) == count_before
    reasons[context.reason] += 1

  complete = bool(rows) and not issues
  return {
    'format_version': 1, 'window_ns': [start_ns, end_ns], 'context_reproduction_complete': complete,
    'active_context_reproduction_complete': complete and any(row['active'] for row in rows),
    'absolute_geometry_tolerance': ABS_TOLERANCE, 'maximum_absolute_errors': max_errors,
    'summary': {'controller_publications': len(rows), 'active_publications': sum(r['active'] for r in rows),
                'reproduced_reasons': dict(sorted(reasons.items())),
                'issue_counts': dict(sorted(Counter(i['reason'] for i in issues).items()))},
    'rows': rows, 'issues': issues,
    'limitations': [
      'Reproduces the current source lane observer only; does not reproduce controller commands or establish vehicle handling.',
      'Scope is supplied controller publications; unrecorded time and physical lane boundaries are not certified.',
      'Model lane confidence is distinct from message health and from correctness of lane identity.',
      'Recorded CLOCK_MONOTONIC and CLOCK_BOOTTIME samples are used separately; no clock-alignment assumption.',
      'Use the applied-output identity auditor separately; this check does not join commands to CAN applies.',
      'One original route only; process-replay nested timestamp rewriting is unqualified.',
    ],
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--rlogs', nargs='+', required=True)
  parser.add_argument('--start-ns', type=int, required=True)
  parser.add_argument('--end-ns', type=int, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if args.output.exists():
    parser.error('Output already exists; select a new evidence path')
  if len(set(args.rlogs)) != len(args.rlogs):
    parser.error('Provide distinct rlogs from one route')
  paths = [under(args.data_root, name) for name in args.rlogs]
  source_before, runtime_before = source_snapshot(), environment()
  hashes = {name: sha256(path) for name, path in zip(args.rlogs, paths, strict=True)}
  result = audit(read_events(paths), args.start_ns, args.end_ns)
  if any(sha256(path) != hashes[name] for name, path in zip(args.rlogs, paths, strict=True)) or environment() != runtime_before:
    raise ValueError('Inputs or runtime changed during audit')
  result.update(input_sha256=hashes, repository_sources=finish_sources(source_before), environment=runtime_before)
  write_json(args.output, result)
  print(f'Lane context reproduced: {result["context_reproduction_complete"]}; {result["summary"]}')
  return 0 if result['active_context_reproduction_complete'] else 1


if __name__ == '__main__':
  raise SystemExit(main())
