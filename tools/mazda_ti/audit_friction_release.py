# ruff: noqa: TID251
"""Check versioned friction state and adjacent recorded updates; no handling score."""
import argparse
from collections import Counter
import math
from pathlib import Path

from .audit_diagnostics import read_events
from .provenance import environment, finish_sources, sha256, source_snapshot, under, write_json
from .runtime import bootstrap, DT

TOLERANCE = 1e-9  # State/contribution counts only; never an integer-command tolerance.


def audit(events, start_ns, end_ns):
  if not 0 < start_ns < end_ns:
    raise ValueError('Require 0 < start_ns < end_ns')
  module = bootstrap()
  states, controllers = {}, {}
  for event in events:
    service, mono = event.which(), int(event.logMonoTime)
    if service == 'carState':
      if mono <= 0 or mono in states:
        raise ValueError('Zero or duplicate carState identity')
      states[mono] = (bool(event.valid), float(event.carState.steeringRateDeg))
    elif service == 'controlsState' and start_ns <= mono < end_ns:
      if mono in controllers:
        raise ValueError('Duplicate controller identity')
      t = event.controlsState.lateralControlState
      controllers[mono] = (bool(t.torqueState.active), t.torqueState.mazdaDiagnostics.to_dict()) if t.which() == 'torqueState' else (False, {})
  previous = None
  rows, issues = [], []
  counts = Counter()
  for mono, (active, d) in sorted(controllers.items()):
    row = {'controlsState': mono, 'active': active, 'transition_checked': False, 'complete': False}
    rows.append(row)
    before = len(issues)

    def issue(reason, mono=mono, **detail):
      issues.append(dict(controlsState=mono, reason=reason, **detail))

    if d.get('version') != 1 or d.get('frictionReleaseVersion', 0) != 1:
      issue('absent_or_unsupported_state_extension')
      previous = None
      continue
    names = ['frictionWithdrawal', 'frictionReleaseDirection']
    if active:
      names += ['frictionCompensation', 'frictionGate', 'inverseCommand', 'tiMax', 'plantLimit',
                'filteredRequest', 'rawRequest', 'measurement', 'dwellFeedforward']
    if any(not math.isfinite(d[k]) for k in names):
      issue('nonfinite_state_or_input')
      previous = None
      continue
    removed, direction, completed = d['frictionWithdrawal'], d['frictionReleaseDirection'], d['frictionReleaseCompleted']
    if direction not in (-1, 0, 1) or not 0 <= removed <= module.FRIC_COMP_MAX or (removed and direction == 0):
      issue('invalid_release_state')
      previous = None
      continue
    row.update(withdrawal=removed, direction=direction, completed=completed)
    enabled = active and bool(d['settings'] & 4) and d['plantLimit'] > module.FRIC_COMP_MIN_AUTHORITY
    compensation = 0.0
    if enabled:
      magnitude = min(module.FRIC_COMP_BASE + module.FRIC_COMP_LOAD*abs(d['inverseCommand']), module.FRIC_COMP_MAX)
      magnitude = min(magnitude, max(0.0, module.FRIC_COMP_KEEPOUT*d['tiMax']-abs(d['inverseCommand'])))
      over = max(abs(d['frictionGate'])-module.FRIC_COMP_LA_DEAD, 0.0)
      compensation = magnitude*math.tanh(over/module.FRIC_COMP_LA_SOFT)*(1 if d['frictionGate'] >= 0 else -1)
    if active:
      row['compensation_before_withdrawal'] = compensation
      if abs(d['frictionCompensation']-(compensation-direction*removed)) > TOLERANCE:
        issue('compensation_decomposition_mismatch')
      if removed > abs(compensation)+TOLERANCE:
        issue('withdrawal_exceeds_available_compensation')
    if not enabled and (removed != 0 or direction != 0 or completed):
      issue('reset_state_mismatch')
    rate = None
    if enabled:
      snapshots = [s for s in d.get('inputs', []) if s['service'] == 'carState']
      if len(snapshots) != 1 or not snapshots[0]['seen'] or not 0 < snapshots[0]['logMonoTime'] <= mono:
        issue('invalid_car_state_snapshot')
      elif snapshots[0]['logMonoTime'] not in states:
        issue('missing_consumed_car_state')
      else:
        snapshot = snapshots[0]
        valid, rate = states[snapshot['logMonoTime']]
        row.update(carState=snapshot['logMonoTime'], wheel_rate=rate, input_checks_passed=snapshot['checksPassed'])
        if valid != snapshot['valid'] or not math.isfinite(rate):
          issue('car_state_validity_or_rate_mismatch')
    if previous and mono-previous[0] > 30_000_000:
      issue('controller_publication_gap', previous=previous[0])
      previous = None
    if previous and len(issues) == before:
      helper = module.FrictionRelease()
      helper.removed, helper.direction, helper.completed = previous[1:]
      if enabled:
        helper.update(compensation, d['filteredRequest'], d['rawRequest'], d['measurement'],
                      d['motionPermitted'] or d['positionPermitted'], d['dwellFeedforward'], rate, DT)
      else:
        helper.reset()
      row['transition_checked'] = True
      if abs(helper.removed-removed) > TOLERANCE or helper.direction != direction or helper.completed != completed:
        issue('release_transition_mismatch', expected_removed=helper.removed,
              expected_direction=helper.direction, expected_completed=helper.completed)
    row['complete'] = len(issues) == before
    counts['active'] += active
    counts['withdrawal_positive'] += removed > 0
    counts['completed'] += completed
    counts['transitions_checked'] += row['transition_checked']
    previous = (mono, removed, direction, completed) if row['complete'] else None
  return dict(format_version=1, window_ns=[start_ns, end_ns], rows=rows, issues=issues,
              state_consistent=bool(rows) and not issues and counts['active'] > 0 and counts['transitions_checked'] > 0,
              summary=dict(counts, controller_publications=len(rows), issue_counts=dict(Counter(i['reason'] for i in issues))),
              state_count_tolerance=TOLERANCE, assumed_controller_dt=DT,
              limitations=[
                'First complete state is an observed anchor, not independently reconstructed history.',
                'Adjacent logged publications are treated as consecutive updates. No controller sequence counter proves missing-update absence.',
                'Gaps above30ms fail; smaller missing intervals can be undetectable when state is unchanged.',
                'Uses recorded lane permissions/dwell and plant output. Audit lane context and applied identities separately.',
                'Reproduces current helper state/contribution only, not full commands, motor delivery or lane performance.',
              ])


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--data-root', type=Path, required=True)
  parser.add_argument('--rlogs', nargs='+', required=True)
  parser.add_argument('--start-ns', type=int, required=True)
  parser.add_argument('--end-ns', type=int, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if args.output.exists() or len(set(args.rlogs)) != len(args.rlogs):
    parser.error('Use distinct inputs and a new output path')
  paths = [under(args.data_root, name) for name in args.rlogs]
  sources, runtime = source_snapshot(), environment()
  hashes = {name: sha256(path) for name, path in zip(args.rlogs, paths, strict=True)}
  result = audit(read_events(paths), args.start_ns, args.end_ns)
  if environment() != runtime or hashes != {name: sha256(path) for name, path in zip(args.rlogs, paths, strict=True)}:
    raise ValueError('Inputs or runtime changed during audit')
  result.update(input_sha256=hashes, repository_sources=finish_sources(sources), environment=runtime)
  write_json(args.output, result)
  print(f'State consistent: {result["state_consistent"]}; {result["summary"]}')
  return 0 if result['state_consistent'] else 1


if __name__ == '__main__':
  raise SystemExit(main())
