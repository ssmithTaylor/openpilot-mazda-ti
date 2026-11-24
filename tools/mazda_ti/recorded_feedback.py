"""Replay TI feedback using original recorded apply and consumed-output identities.

The caller supplies the production limiter and publishes its controller's normalized outputs.
Physical sensors, actuator availability and the apply schedule remain recorded. No I/O or
vehicle simulation occurs here. The separate diagnostic auditor qualifies the full input chain.
"""

from dataclasses import dataclass
import math
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.car.mazda.ti_stuck_guard import StuckGuard, STUCK_HOLD_FRAMES, STUCK_NUDGE_FRAMES, TI_FW_CLIP, toward_zero

from .applied_feedback import AppliedLateralFeedback, PreviousAppliedFeedback

STUCK_GUARD_MODES = (False, True, 'auto')


def _guard_key(g):
  """Everything that decides a StuckGuard's future outputs (run only matters up to STUCK_HOLD_FRAMES)."""
  return (g.last_out, min(g.run, STUCK_HOLD_FRAMES), g.since, g.held if g.nudge_left else None, g.nudge_left)


def _initial_guards(previous, first_pre):
  """Every guard state consistent with a window that opens on a frame whose previous SENT value was `previous`.

  The recording does not carry the guard's state, only the value sent on the previous frame. Unknown: how many frames it
  had been sent (1..STUCK_HOLD_FRAMES; more is the same as STUCK_HOLD_FRAMES), whether lateral was inactive (reset: only
  when 0 was sent), and whether that frame was a nudge with nudge frames still to send for `first_pre`. `since` is
  unknown too and left None: an earlier real-time trigger is the same behaviour as a longer run, which is enumerated.
  Order = replay preference: the longest plausible hold first (the replayed guard nudges no later than the car could)."""
  out = []
  for run in range(STUCK_HOLD_FRAMES, 0, -1):
    g = StuckGuard()
    g.last_out, g.run = previous, run
    out.append(g)
  if previous == 0:
    out.append(StuckGuard())
  if first_pre not in (0, TI_FW_CLIP, -TI_FW_CLIP) and first_pre != previous and toward_zero(first_pre) == previous:
    for left in range(1, STUCK_NUDGE_FRAMES):
      g = StuckGuard()
      g.last_out, g.run, g.held, g.nudge_left = previous, STUCK_NUDGE_FRAMES - left, first_pre, left
      out.append(g)
  return out


def _copy_guard(g):
  c = StuckGuard()
  c.__dict__.update(g.__dict__)
  return c


def serialized_steer(value):
  value = float(value)
  if not math.isfinite(value) or abs(value) > 1:
    raise ValueError('Controller steering output must be finite and within [-1, 1]')
  return float(np.float32(value))


def limits_from_diagnostics(d):
  fields = {
    'TI_STEER_MAX': 'tiMax', 'TI_STEER_DELTA_UP': 'tiDeltaUp', 'TI_STEER_DELTA_DOWN': 'tiDeltaDown',
    'TI_STEER_DRIVER_ALLOWANCE': 'tiDriverAllowance', 'TI_STEER_DRIVER_MULTIPLIER': 'tiDriverMultiplier',
    'TI_STEER_DELTA_UP_KNEE': 'tiDeltaUpKnee', 'TI_STEER_DELTA_UP_HIGH': 'tiDeltaUpHigh',
  }
  values = {name: float(d[field]) for name, field in fields.items()}
  if values['TI_STEER_MAX'] != 600 or any(not math.isfinite(v) or v < 0 for v in values.values()):
    raise ValueError('Only finite, nonnegative TI600 limiter settings are qualified')
  return SimpleNamespace(**values, TI_STEER_DRIVER_FACTOR=1)


@dataclass(frozen=True)
class Apply:
  sequence: int
  applied_at: int
  controller: int
  request_mono: int
  request: float
  allowed: bool
  previous: int
  recorded: int
  driver_torque: float
  limits: SimpleNamespace
  active: bool = False
  ti_selected: bool = True
  stock_previous: int = 0
  stock_recorded: int = 0
  reported: float = 0.0       # recorded carOutput.actuatorsOutput.steer


class RecordedTiFeedback:
  """One activation boundary, one route, recorded serialized carOutput/carControl events.

  `outputs` contains (event identity, Event) pairs; `commands` maps identities to Events.
  Construction verifies the recorded TI request, sequential limiter and serialized feedback
  for every apply at/after activation published before end_ns. Stock fallback requires
  an explicit pinned stock limiter and GEN1 settings. Both limiter histories then run
  on every apply, including while the other path supplies the published feedback.

  Stuck-request guard (selfdrive/car/mazda/ti_stuck_guard.py, commits 36ef39e/f87000a). The car
  controller passes the TI limiter's output (pre) through StuckGuard.update(pre, latActive and
  tiAllowed, now=appliedAtMonoTime * 1e-9); the value it returns is sent, recorded as tiLimited
  and becomes the next tiPrevious, while actuatorsOutput.steer stays pre / 600. The recording
  carries NO marker of the guard (mazdaDiagnostics.version is still 1; the `ti_stuck_guard`
  cloudlog event is rate limited to one per 10 s and is not an input here), so the caller says:
    stuck_guard=False (default): exactly the checks and replay of a pre-guard build.
    stuck_guard=True: sent = guard(pre) is reproduced exactly on every apply, tiLimited == sent,
      steer == pre / 600, sent is the history; replay runs the same guard on the candidate's
      commands. A hold the guard would have nudged but the recording did not is rejected.
    stuck_guard='auto': the whole window must reproduce exactly under one model: the False model
      is tried first (so un-nudged drives behave exactly as before), then the True model.
      `stuck_guard_present` reports the one that qualified. Trade-off: a window with no nudgeable
      hold of STUCK_HOLD_S fits both and is replayed WITHOUT the guard, so a replayed controller
      that holds a value would not be nudged; models are never mixed per frame (mixing would
      accept a recording that skips a nudge the guard must send).
  The clock is exact: card.py records appliedAtMonoTime = the now_nanos given to the controller.
  The guard's state at the window start is NOT recorded (only the previous sent value): every
  consistent start state is tried (held 1..STUCK_HOLD_FRAMES frames, reset, mid-burst) and only
  exact reproductions survive; once the sent value changes they coincide. If several survive
  the whole window, replay starts from the longest plausible hold. The stock path is not guarded.
  """

  def __init__(self, outputs, commands, activation_ns, end_ns, limiter, *, stock_limiter=None, stock_limits=None,
               stuck_guard=False):
    if not 0 < activation_ns < end_ns:
      raise ValueError('Require 0 < activation_ns < end_ns')
    if not any(stuck_guard is m for m in (False, True)) and stuck_guard != 'auto':
      raise ValueError("stuck_guard must be False, True or 'auto'")
    self.activation_ns, self.end_ns, self.limiter = activation_ns, end_ns, limiter
    if (stock_limiter is None) != (stock_limits is None):
      raise ValueError('Provide both stock limiter and settings')
    self.stock_limiter = stock_limiter
    self.stock_limits = SimpleNamespace(**vars(stock_limits)) if stock_limits is not None else None
    if self.stock_limits is not None:
      expected = {'STEER_MAX': 600, 'STEER_DELTA_UP': 10, 'STEER_DELTA_DOWN': 25,
                  'STEER_DRIVER_ALLOWANCE': 15, 'STEER_DRIVER_MULTIPLIER': 40, 'STEER_DRIVER_FACTOR': 1}
      if vars(self.stock_limits) != expected:
        raise ValueError('Only explicit pinned GEN1 stock600 settings are qualified')
    self.outputs, unique = {}, {}
    for mono, event in outputs:
      if mono >= end_ns:
        continue
      if mono <= 0 or mono in self.outputs or int(event.logMonoTime) != mono:
        raise ValueError('Duplicate or inconsistent output identity')
      co = event.carOutput
      self.outputs[mono] = (int(co.applySequence), int(co.appliedAtMonoTime), serialized_steer(co.actuatorsOutput.steer))
      if co.appliedAtMonoTime < activation_ns:
        continue
      d = co.mazdaDiagnostics.to_dict()
      if d.get('version', 0) != 1 or not 0 < co.appliedCarControlMonoTime <= co.appliedAtMonoTime <= mono or co.applySequence <= 0:
        raise ValueError('Missing or invalid apply diagnostics/identity')
      payload = co.to_dict()
      sequence = int(co.applySequence)
      if sequence in unique:
        if unique[sequence][1] != payload:
          raise ValueError('Repeated apply changed payload')
        continue
      command_event = commands.get(int(co.appliedCarControlMonoTime))
      if command_event is None or int(command_event.logMonoTime) != co.appliedCarControlMonoTime:
        raise ValueError('Missing or inconsistent applied carControl identity')
      cc = command_event.carControl
      if not 0 < cc.controlsStateMonoTime <= co.appliedCarControlMonoTime or bool(cc.latActive) != d['latActive']:
        raise ValueError('Invalid controller identity or lateral-active mismatch')
      if cc.latActive and not d['tiAllowed'] and self.stock_limiter is None:
        raise ValueError('Active stock fallback while TI unavailable is not qualified')
      request = serialized_steer(cc.actuators.steer)
      allowed = bool(cc.latActive and d['tiAllowed'])
      if (int(round(request * 600)) if allowed else 0) != d['tiRequested']:
        raise ValueError('Recorded TI request does not match applied carControl')
      if not math.isfinite(d['driverTorque']):
        raise ValueError('Non-finite driver torque')
      apply = Apply(sequence, int(co.appliedAtMonoTime), int(cc.controlsStateMonoTime), int(co.appliedCarControlMonoTime), request, allowed,
                    int(d['tiPrevious']), int(d['tiLimited']), d['driverTorque'], limits_from_diagnostics(d),
                    bool(cc.latActive), bool(d['tiAllowed']), int(d['stockPrevious']), int(d['stockLimited']),
                    float(co.actuatorsOutput.steer))
      if not -600 <= apply.previous <= 600 or not -600 <= apply.recorded <= 600:
        raise ValueError('Recorded TI command outside qualified envelope')
      if self.stock_limiter is not None:
        if (int(round(request * 600)) if cc.latActive else 0) != d['stockRequested']:
          raise ValueError('Recorded stock request does not match applied carControl')
        if not -600 <= apply.stock_previous <= 600 or not -600 <= apply.stock_recorded <= 600:
          raise ValueError('Recorded stock command outside qualified envelope')
      ti_feedback = self.stock_limiter is None or apply.ti_selected
      feedback = apply.recorded if ti_feedback else apply.stock_recorded
      # TI feedback under a guard model is checked against the PRE-nudge command in _verify (unknown here).
      if (stuck_guard is False or not ti_feedback) and float(co.actuatorsOutput.steer) != serialized_steer(feedback / 600):
        raise ValueError('Recorded feedback does not represent the TI limited command')
      unique[sequence] = (apply, payload)
    self.applies = [pair[0] for _, pair in sorted(unique.items())]
    if not self.applies:
      raise ValueError('No recorded applies after activation')
    if stuck_guard == 'auto':
      try:
        self.guard_start = self._verify(False)
      except ValueError as exact_error:
        try:
          self.guard_start = self._verify(True)
        except ValueError as guard_error:
          raise ValueError(f'{exact_error}; under the stuck-guard model: {guard_error}') from None
    else:
      self.guard_start = self._verify(stuck_guard)
    self.stuck_guard_present = self.guard_start is not None
    self.guard = _copy_guard(self.guard_start) if self.stuck_guard_present else None
    self.cursor, self.previous = 0, self.applies[0].previous
    self.requests, self.counts, self.reported = {}, {}, {}
    self.stock_previous, self.stock_counts = self.applies[0].stock_previous, {}

  def _verify(self, guarded):
    """Sequential exact verification; returns the guard start state for replay (None: no guard model)."""
    previous = self.applies[0].previous
    stock_previous = self.applies[0].stock_previous
    guards = None
    if guarded:
      first = self.applies[0]
      guards = [(g, _copy_guard(g)) for g in _initial_guards(previous, self._limit(first, first.request, previous))]
    for i, apply in enumerate(self.applies):
      if i and (apply.sequence != self.applies[i-1].sequence + 1 or apply.applied_at <= self.applies[i-1].applied_at):
        raise ValueError('Missing apply sequence or nonincreasing apply time')
      if apply.previous != previous:
        raise ValueError('Recorded previous TI command is inconsistent')
      pre = self._limit(apply, apply.request, previous)
      if guards is None:
        previous = pre
        if previous != apply.recorded:
          raise ValueError('Production limiter does not reproduce recorded TI command')
      else:
        survivors, seen = [], set()
        for start, g in guards:
          if g.update(pre, apply.allowed, now=apply.applied_at * 1e-9) == apply.recorded and _guard_key(g) not in seen:
            seen.add(_guard_key(g))
            survivors.append((start, g))
        if not survivors:
          raise ValueError(f'Production limiter with stuck guard does not reproduce recorded TI command '
                           f'(apply {apply.sequence}: limiter {pre}, recorded {apply.recorded})')
        guards, previous = survivors, apply.recorded
      if (self.stock_limiter is None or apply.ti_selected) and apply.reported != serialized_steer(pre / 600):
        raise ValueError('Recorded feedback does not represent the TI limited command')
      if self.stock_limiter is not None:
        if apply.stock_previous != stock_previous:
          raise ValueError('Recorded previous stock command is inconsistent')
        stock_previous = self._stock_limit(apply, apply.request, stock_previous)
        if stock_previous != apply.stock_recorded:
          raise ValueError('Production limiter does not reproduce recorded stock command')
    return guards[0][0] if guards is not None else None

  def _limit(self, apply, steer, previous):
    return self.limiter(int(round(steer * 600)), previous, apply.driver_torque, apply.limits) if apply.allowed else 0

  def _stock_limit(self, apply, steer, previous):
    return self.stock_limiter(int(round(steer * 600)), previous, apply.driver_torque, self.stock_limits) if apply.active else 0

  def publish(self, controller_mono, steer):
    if controller_mono <= 0 or controller_mono in self.requests:
      raise ValueError('Duplicate or invalid controller output identity')
    self.requests[controller_mono] = serialized_steer(steer)

  def history_request(self, controller_mono, replayed, recorded):
    """Preserve requests AND feedback before activation; never mix warmup histories."""
    return serialized_steer(replayed if controller_mono >= self.activation_ns else recorded)

  def _advance(self, sequence):
    while self.cursor < len(self.applies) and self.applies[self.cursor].sequence <= sequence:
      apply = self.applies[self.cursor]
      if apply.controller < self.activation_ns:
        steer = apply.request
      else:
        if apply.controller not in self.requests:
          raise ValueError(f'Missing replay controller output: {apply.controller}')
        steer = self.requests[apply.controller]
      pre = self._limit(apply, steer, self.previous)
      # The car's stuck guard on the replayed command; what it sends is the next history, `steer` stays pre.
      self.previous = self.guard.update(pre, apply.allowed, now=apply.applied_at * 1e-9) if self.guard is not None else pre
      self.counts[apply.sequence] = self.previous
      self.reported[apply.sequence] = pre
      if self.stock_limiter is not None:
        self.stock_previous = self._stock_limit(apply, steer, self.stock_previous)
        self.stock_counts[apply.sequence] = self.stock_previous
      self.cursor += 1
    if sequence not in self.counts:
      raise ValueError('Requested apply is absent from the replay schedule')

  def output_for(self, output_mono):
    if output_mono not in self.outputs:
      raise ValueError('Missing consumed carOutput identity')
    sequence, applied_at, recorded = self.outputs[output_mono]
    if applied_at < self.activation_ns:
      return recorded
    self._advance(sequence)
    if self.stock_limiter is not None:
      apply = self.applies[sequence - self.applies[0].sequence]
      if not apply.ti_selected:
        return serialized_steer(self.stock_counts[sequence] / 600)
    return serialized_steer(self.reported[sequence] / 600)

  def previous_applied_for_update(self, update_mono, output_mono, limiter_frozen=False):
    """Return only causally prior, candidate-owned apply state for ``update_mono``."""
    observed = self.output_for(output_mono)
    sequence, applied_at, _ = self.outputs[output_mono]
    if applied_at < self.activation_ns:
      return PreviousAppliedFeedback(int(update_mono), observed, None)
    apply = next((a for a in self.applies if a.sequence == sequence), None)
    if apply is None:
      raise ValueError('Missing apply identity for consumed output')
    if apply.controller < self.activation_ns:
      return PreviousAppliedFeedback(int(update_mono), observed, None)
    stock = self.stock_counts.get(sequence) if self.stock_limiter is not None else None
    fallback = bool(apply.active and not apply.ti_selected)
    # TI counts as reported to controlsd: the pre-nudge command (equal to the sent one whenever the guard did not nudge).
    selected = stock if fallback else self.reported[sequence]
    snapshot = AppliedLateralFeedback(
      int(output_mono), apply.applied_at, apply.request_mono, apply.controller, sequence,
      self.reported[sequence], stock, selected, observed, apply.active, apply.allowed,
      apply.ti_selected, bool(apply.active and apply.ti_selected), fallback, bool(limiter_frozen),
    )
    return PreviousAppliedFeedback(int(update_mono), observed, snapshot)

  def finish(self):
    self._advance(self.applies[-1].sequence)
    return [{'sequence': a.sequence, 'applied_at': a.applied_at, 'controlsState': a.controller,
             'recorded': a.recorded, 'replay': self.counts[a.sequence]} for a in self.applies]
