# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""TI outgoing-command software replay; no vehicle/EPS/motion prediction.

Time arguments are integer logMonoTime nanoseconds. The original schedule is
fixed. Explicit inferred request maps describe output-compatible sampling
histories, not proof of card's unlogged socket receive time. Baseline verification
is mandatory for each fixture before candidate use. Physical motion remains fixed.
"""

import ast
from bisect import bisect_right
from dataclasses import dataclass
import hashlib
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
from .applied_feedback import AppliedLateralFeedback, PreviousAppliedFeedback
from .runtime import ROOT

sys.path.insert(0, str(ROOT))
from cereal import log


@dataclass(frozen=True)
class Request:
  mono: int
  steer: float
  active: bool
  source_controller_mono: int | None = None


@dataclass(frozen=True)
class Output:
  mono: int
  counts: int
  steer: float


def pure_limiter(revision):
  raw = subprocess.check_output(['git', 'show', revision + ':selfdrive/car/__init__.py'], cwd=ROOT)
  tree = ast.parse(raw.decode())
  node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'apply_ti_steer_torque_limits')
  scope = {'clip': np.clip}
  exec(compile(ast.Module(body=[node], type_ignores=[]), 'recorded_TÎ™_limiter', 'exec'), scope)
  return scope[node.name], hashlib.sha256(raw).hexdigest()


def pure_stock_limiter(revision):
  """Pinned GEN1 stock request limiter/settings; counts are not EPS delivery."""
  raw = subprocess.check_output(['git', 'show', revision + ':selfdrive/car/__init__.py'], cwd=ROOT)
  values = subprocess.check_output(['git', 'show', revision + ':selfdrive/car/mazda/values.py'], cwd=ROOT)
  node = next(n for n in ast.parse(raw.decode()).body
              if isinstance(n, ast.FunctionDef) and n.name == 'apply_driver_steer_torque_limits')
  scope = {'clip': np.clip}
  exec(compile(ast.Module(body=[node], type_ignores=[]), 'recorded_stock_limiter', 'exec'), scope)
  cls = next(n for n in ast.parse(values.decode()).body if isinstance(n, ast.ClassDef) and n.name == 'CarControllerParams')
  init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
  gen1 = next(n for n in init.body if isinstance(n, ast.If) and ast.unparse(n.test) == 'CP.flags & MazdaFlags.GEN1')
  required = {'STEER_MAX', 'STEER_DELTA_UP', 'STEER_DELTA_DOWN', 'STEER_DRIVER_ALLOWANCE',
              'STEER_DRIVER_MULTIPLIER', 'STEER_DRIVER_FACTOR'}
  constants = {n.targets[0].attr: ast.literal_eval(n.value) for n in gen1.body
               if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Attribute) and n.targets[0].attr in required}
  if set(constants) != required:
    raise ValueError('Missing pinned GEN1 stock limiter constants')
  return scope[node.name], SimpleNamespace(**constants), {
    'selfdrive/car/__init__.py': hashlib.sha256(raw).hexdigest(),
    'selfdrive/car/mazda/values.py': hashlib.sha256(values).hexdigest(),
  }


class Fixture:
  """Immutable-in-use observations and schedule; no candidate state lives here."""

  def __init__(self, paths, start_ns, end_ns, limits, limiter_ref, request_identity=None):
    self.start_ns, self.end_ns = int(start_ns), int(end_ns)
    if self.end_ns <= self.start_ns:
      raise ValueError('Empty or reversed interval')
    self.limits = SimpleNamespace(**vars(limits))
    self.limiter, self.source_sha256 = pure_limiter(limiter_ref)
    self.paths = [str(Path(p).resolve()) for p in paths]
    rr = {k: [] for k in ['request', 'state', 'send', 'output', 'health']}
    self.input_sha256 = {}
    for path in self.paths:
      raw = Path(path).read_bytes()
      self.input_sha256[path] = hashlib.sha256(raw).hexdigest()
      for m in log.Event.read_multiple_bytes(raw):
        t, kind = int(m.logMonoTime), m.which()
        if t > self.end_ns:
          continue
        if kind == 'carControl':
          rr['request'].append(Request(t, float(m.carControl.actuators.steer), bool(m.carControl.latActive)))
        elif kind == 'carState':
          rr['state'].append((t, float(m.carState.steeringTorque)))
        elif kind == 'carOutput':
          a = m.carOutput.actuatorsOutput
          rr['output'].append(Output(t, int(a.steerOutputCan), float(a.steer)))
        elif kind == 'sendcan':
          for c in m.sendcan:
            if c.address == 0x249 and c.src == 1:
              d = c.dat
              rr['send'].append((t, (((d[0] & 15) << 8) | d[1]) - 2048))
        elif kind == 'can':
          for c in m.can:
            if c.address == 0x24A and c.src == 1:
              rr['health'].append((t, int(c.dat[3]), int(c.dat[6])))
    for k, values in rr.items():
      values.sort(key=lambda r: r.mono if isinstance(r, (Output, Request)) else r[0])
      times = [r.mono if isinstance(r, (Output, Request)) else r[0] for r in values]
      # CAN packets may contain several feedback frames sharing their
      # container timestamp. Every mode/ramp value is checked below.
      if k != 'health' and any(a >= b for a, b in zip(times, times[1:], strict=False)):
        raise ValueError('Duplicate or unordered ' + k + ' timestamps')

    def latest(kind, t):
      values = rr[kind]
      times = [r.mono if isinstance(r, (Output, Request)) else r[0] for r in values]
      idx = bisect_right(times, t) - 1
      if idx < 0:
        raise ValueError('Need prior ' + kind + ' observation for initialization')
      return values[idx]

    self.initial_send = latest('send', self.start_ns)
    self.initial_output = latest('output', self.start_ns)
    self.initial_request = latest('request', self.start_ns)
    self.initial_state = latest('state', self.start_ns)
    self.initial_sampled_request = latest('request', self.initial_state[0])
    self.request_identity = {int(k): int(v) for k, v in (request_identity or {}).items() if self.start_ns < int(k) <= self.end_ns}
    request_lookup = {r.mono: r for r in rr['request']}
    state_times = {t for t, _ in rr['state']}
    for state_time, request_time in self.request_identity.items():
      if state_time not in state_times or request_time not in request_lookup:
        raise ValueError('Sampling identity references a missing observation')
      if request_time > latest('output', state_time).mono:
        raise ValueError('Request was created after the source-supported sampling upper bound')
    ordered = [v for _, v in sorted(self.request_identity.items())]
    if any(a > b for a, b in zip(ordered, ordered[1:], strict=False)):
      raise ValueError('Sampled request identities must be monotone')
    self.prior_requests = {r.mono: r for r in rr['request'] if r.mono <= self.start_ns}
    health = [latest('health', self.start_ns)] + [r for r in rr['health'] if r[0] > self.start_ns]
    if any(mode != 3 or ramp for _, mode, ramp in health):
      raise ValueError('Only continuously RUN/non-ramping TI fixtures are supported')
    # An inactive request could be modeled by zeroing, but changes stock
    # fallback / engagement state assumptions. Keep this narrow seam explicit.
    requests = [self.initial_request] + [r for r in rr['request'] if r.mono > self.start_ns]
    if not all(r.active for r in requests):
      raise ValueError('Only continuously latActive fixtures are supported')
    self.requests = [r for r in rr['request'] if r.mono > self.start_ns]
    self.actual_sends = dict(r for r in rr['send'] if r[0] > self.start_ns)
    self.actual_outputs = {r.mono: r for r in rr['output'] if r.mono > self.start_ns}
    self.events = sorted(
      [(t, 'state', sensor) for t, sensor in rr['state'] if t > self.start_ns]
      + [(t, 'send', None) for t in self.actual_sends]
      + [(t, 'output', None) for t in self.actual_outputs]
    )
    event_times = [r[0] for r in self.events]
    if len(set(event_times)) != len(event_times):
      raise ValueError('Exact timestamp ties require explicit ordering evidence')
    self.baseline_verified = False

  def new(self):
    if not self.baseline_verified:
      raise RuntimeError('verify_baseline() must pass before candidate use')
    return TiFeedbackReplay(self, require_source_identity=True)

  def verify_baseline(self):
    replay = TiFeedbackReplay(self)
    for r in self.requests:
      replay.publish_request(r.mono, r.steer, r.active)
    replay.advance_until(self.end_ns)
    send_errors = [{'mono': t, 'actual': self.actual_sends[t], 'predicted': u} for t, u in replay.sends if u != self.actual_sends[t]]
    output_errors = [
      {'mono': r.mono, 'actual': vars(self.actual_outputs[r.mono]), 'predicted': vars(r)} for r in replay.outputs if r != self.actual_outputs[r.mono]
    ]
    if len(replay.sends) != len(self.actual_sends) or len(replay.outputs) != len(self.actual_outputs):
      raise AssertionError('Missing replay publications')
    self.baseline_verified = not send_errors and not output_errors
    result = dict(sends=len(replay.sends), outputs=len(replay.outputs), send_errors=send_errors, output_errors=output_errors, exact=self.baseline_verified)
    if not self.baseline_verified:
      raise AssertionError(result)
    return result


class TiFeedbackReplay:
  """Candidate requests + frozen schedule produce mutually consistent feedback.

  Construct through Fixture.new(). Call advance_until at the controller's
  audited input cutoff, then publish_request at its paired original carControl
  publication time. publish_request advances pending events first. There is no
  interpolation or wall-clock waiting and no per-frame state anchoring.
  """

  def __init__(self, fixture, require_source_identity=False):
    self.fixture = fixture
    self.require_source_identity = bool(require_source_identity)
    self.now = fixture.start_ns
    self.index = 0
    self.last_send = fixture.initial_send[1]
    self.latest_output = fixture.initial_output
    self.latest_request = fixture.initial_request
    self.sampled_request = fixture.initial_sampled_request
    self.sensor = fixture.initial_state[1]
    self.sends = []
    self.outputs = []
    self.output_history = [fixture.initial_output]
    self.output_times = [fixture.initial_output.mono]
    self.request_log = []
    self.requests_by_time = dict(fixture.prior_requests)
    self.last_send_identity = (fixture.initial_send[0], fixture.initial_sampled_request)
    self.output_apply_identity = {}

  def advance_until(self, mono):
    mono = int(mono)
    if not self.now <= mono <= self.fixture.end_ns:
      raise ValueError(f'Time must advance monotonically inside the fixture: now={self.now}, requested={mono}, end={self.fixture.end_ns}')
    while self.index < len(self.fixture.events) and self.fixture.events[self.index][0] <= mono:
      t, kind, value = self.fixture.events[self.index]
      if kind == 'state':
        self.sensor = value
        request_time = self.fixture.request_identity.get(t)
        if request_time is None:
          self.sampled_request = self.latest_request
        else:
          if request_time not in self.requests_by_time:
            raise ValueError('Candidate did not publish the required sampled request')
          self.sampled_request = self.requests_by_time[request_time]
      elif kind == 'send':
        wanted = round(self.sampled_request.steer * self.fixture.limits.TI_STEER_MAX)
        self.last_send = self.fixture.limiter(wanted, self.last_send, self.sensor, self.fixture.limits)
        self.sends.append((t, self.last_send))
        self.last_send_identity = (t, self.sampled_request)
      elif kind == 'output':
        # cereal CarControl.Actuators.steer is Float32 on the wire.
        steer = float(np.float32(self.last_send / self.fixture.limits.TI_STEER_MAX))
        self.latest_output = Output(t, self.last_send, steer)
        self.outputs.append(self.latest_output)
        self.output_history.append(self.latest_output)
        self.output_times.append(t)
        self.output_apply_identity[t] = self.last_send_identity
      self.index += 1
    self.now = mono
    return self.latest_output

  def output_at(self, mono):
    """Read simulated publication history without rewinding actuator state.

    A controller can consume an older message after its previous request was
    published. That asynchronous receipt does not reverse physical time.
    """
    mono = int(mono)
    if mono > self.now:
      self.advance_until(mono)
    index = bisect_right(self.output_times, mono) - 1
    if index < 0:
      raise ValueError('Requested feedback precedes initialized publication history')
    return self.output_history[index]

  def previous_applied_for_update(self, update_mono, output_mono, limiter_frozen=False):
    """Expose a prior candidate apply only when its producing update is known."""
    output = self.output_at(output_mono)
    identity = self.output_apply_identity.get(output.mono)
    if identity is None:
      return PreviousAppliedFeedback(int(update_mono), output.steer, None)
    applied_mono, request = identity
    if request.source_controller_mono is None or request.source_controller_mono < self.fixture.start_ns:
      return PreviousAppliedFeedback(int(update_mono), output.steer, None)
    snapshot = AppliedLateralFeedback(
      output.mono, applied_mono, request.mono, request.source_controller_mono,
      None, output.counts, None, output.counts,
      output.steer, request.active, request.active, True, request.active, False,
      bool(limiter_frozen),
    )
    return PreviousAppliedFeedback(int(update_mono), output.steer, snapshot)

  def publish_request(self, mono, steer, active=True, source_controller_mono=None):
    if not active:
      raise ValueError('Engagement transitions are outside this adapter scope')
    if not np.isfinite(steer) or abs(steer) > 1:
      raise ValueError('Candidate normalized steer must be finite and within existing bounds')
    if self.require_source_identity and source_controller_mono is None:
      raise ValueError('Candidate request requires its producing controller identity')
    if source_controller_mono is not None and not 0 < int(source_controller_mono) <= int(mono):
      raise ValueError('Invalid candidate request/controller identity')
    self.advance_until(mono)
    self.latest_request = Request(int(mono), float(np.float32(steer)), True,
                                  int(source_controller_mono) if source_controller_mono is not None else None)
    self.requests_by_time[int(mono)] = self.latest_request
    self.request_log.append(self.latest_request)
