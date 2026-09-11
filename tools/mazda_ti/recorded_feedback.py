"""Replay TI feedback using original recorded apply and consumed-output identities.

The caller supplies the production limiter and publishes its controller's normalized outputs.
Physical sensors, actuator availability and the apply schedule remain recorded. No I/O or
vehicle simulation occurs here. The separate diagnostic auditor qualifies the full input chain.
"""

from dataclasses import dataclass
import math
from types import SimpleNamespace

import numpy as np

from .applied_feedback import AppliedLateralFeedback, PreviousAppliedFeedback


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


class RecordedTiFeedback:
  """One activation boundary, one route, recorded serialized carOutput/carControl events.

  `outputs` contains (event identity, Event) pairs; `commands` maps identities to Events.
  Construction verifies the recorded TI request, sequential limiter and serialized feedback
  for every apply at/after activation published before end_ns. Stock fallback requires
  an explicit pinned stock limiter and GEN1 settings. Both limiter histories then run
  on every apply, including while the other path supplies the published feedback.
  """

  def __init__(self, outputs, commands, activation_ns, end_ns, limiter, *, stock_limiter=None, stock_limits=None):
    if not 0 < activation_ns < end_ns:
      raise ValueError('Require 0 < activation_ns < end_ns')
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
                    bool(cc.latActive), bool(d['tiAllowed']), int(d['stockPrevious']), int(d['stockLimited']))
      if not -600 <= apply.previous <= 600 or not -600 <= apply.recorded <= 600:
        raise ValueError('Recorded TI command outside qualified envelope')
      if self.stock_limiter is not None:
        if (int(round(request * 600)) if cc.latActive else 0) != d['stockRequested']:
          raise ValueError('Recorded stock request does not match applied carControl')
        if not -600 <= apply.stock_previous <= 600 or not -600 <= apply.stock_recorded <= 600:
          raise ValueError('Recorded stock command outside qualified envelope')
      feedback = apply.recorded if self.stock_limiter is None or apply.ti_selected else apply.stock_recorded
      if float(co.actuatorsOutput.steer) != serialized_steer(feedback / 600):
        raise ValueError('Recorded feedback does not represent the TI limited command')
      unique[sequence] = (apply, payload)
    self.applies = [pair[0] for _, pair in sorted(unique.items())]
    if not self.applies:
      raise ValueError('No recorded applies after activation')
    previous = self.applies[0].previous
    stock_previous = self.applies[0].stock_previous
    for i, apply in enumerate(self.applies):
      if i and (apply.sequence != self.applies[i-1].sequence + 1 or apply.applied_at <= self.applies[i-1].applied_at):
        raise ValueError('Missing apply sequence or nonincreasing apply time')
      if apply.previous != previous:
        raise ValueError('Recorded previous TI command is inconsistent')
      previous = self._limit(apply, apply.request, previous)
      if previous != apply.recorded:
        raise ValueError('Production limiter does not reproduce recorded TI command')
      if self.stock_limiter is not None:
        if apply.stock_previous != stock_previous:
          raise ValueError('Recorded previous stock command is inconsistent')
        stock_previous = self._stock_limit(apply, apply.request, stock_previous)
        if stock_previous != apply.stock_recorded:
          raise ValueError('Production limiter does not reproduce recorded stock command')
    self.cursor, self.previous = 0, self.applies[0].previous
    self.requests, self.counts = {}, {}
    self.stock_previous, self.stock_counts = self.applies[0].stock_previous, {}

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
      self.previous = self._limit(apply, steer, self.previous)
      self.counts[apply.sequence] = self.previous
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
    return serialized_steer(self.counts[sequence] / 600)

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
    selected = stock if fallback else self.counts[sequence]
    snapshot = AppliedLateralFeedback(
      int(output_mono), apply.applied_at, apply.request_mono, apply.controller, sequence,
      self.counts[sequence], stock, selected, observed, apply.active, apply.allowed,
      apply.ti_selected, bool(apply.active and apply.ti_selected), fallback, bool(limiter_frozen),
    )
    return PreviousAppliedFeedback(int(update_mono), observed, snapshot)

  def finish(self):
    self._advance(self.applies[-1].sequence)
    return [{'sequence': a.sequence, 'applied_at': a.applied_at, 'controlsState': a.controller,
             'recorded': a.recorded, 'replay': self.counts[a.sequence]} for a in self.applies]
