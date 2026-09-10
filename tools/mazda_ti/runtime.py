"""Load the real Python controller on Windows without hardware interfaces.

PID, reference filters, vehicle model, plant inverse, breaker and final controller
clipping are real repository code. Only hardware-importing modules needed for
literal constants are replaced by constant-only modules sourced from their AST.
No CAN packer, device Params, sockets, or vehicle-motion simulator is provided.
"""

import ast
from dataclasses import dataclass
import importlib
from pathlib import Path
import sys
import types

from .applied_feedback import AppliedLateralFeedback, PreviousAppliedFeedback

ROOT = Path(__file__).resolve().parents[2]


def bootstrap():
  sys.path.insert(0, str(ROOT))
  for name, directory in [('openpilot', ROOT), ('openpilot.selfdrive.car', ROOT / 'selfdrive/car')]:
    module = types.ModuleType(name)
    module.__path__ = [str(directory)]
    sys.modules[name] = module
  for name, relative, constants in [
    ('openpilot.common.realtime', 'common/realtime.py', ['DT_CTRL', 'DT_MDL']),
    ('openpilot.selfdrive.car.interfaces', 'selfdrive/car/interfaces.py', ['FRICTION_THRESHOLD']),
  ]:
    tree = ast.parse((ROOT / relative).read_text(encoding='utf-8'))
    module = types.ModuleType(name)
    module.__file__ = str(ROOT / relative)
    module.__doc__ = 'Constant-only test facade; values read from ' + relative
    for node in tree.body:
      if isinstance(node, ast.Assign):
        for target in node.targets:
          if isinstance(target, ast.Name) and target.id in constants:
            setattr(module, target.id, ast.literal_eval(node.value))
    if any(not hasattr(module, c) for c in constants):
      raise RuntimeError('Required repository constant missing: ' + relative)
    sys.modules[name] = module
  return importlib.import_module('openpilot.selfdrive.controls.lib.latcontrol_torque')


def load_controller_source(source, name):
  """Load a pinned or experimental controller in memory; never replace repo code."""
  module = types.ModuleType(name)
  module.__file__ = name
  exec(compile(source, name, 'exec'), module.__dict__)
  return module


DT = 0.01


@dataclass(frozen=True)
class ReplayAppliedFeedback:
  """Candidate-owned actuator feedback visible to one replay update.

  The context is diagnostic input.  In particular, installing it does not
  replace ``TiLateralPlant.u_prev``: that field preserves the production
  controller's raw stock-request history.  A candidate that deliberately
  models the limited stock command must request it through
  :meth:`stock_request_counts` and thereby prove that exact stock history is
  available.
  """

  update_mono: int
  previous: PreviousAppliedFeedback | None

  def __post_init__(self):
    if self.update_mono <= 0:
      raise ValueError('Replay update identity must be positive')
    if self.previous is not None and self.previous.update_mono != self.update_mono:
      raise ValueError('Applied feedback belongs to a different controller update')
    candidate = self.candidate
    if candidate is not None and not candidate.active:
      if candidate.ti_counts != 0 or candidate.stock_counts not in (None, 0):
        raise ValueError('Inactive applied feedback cannot restore actuator history')

  @property
  def candidate(self) -> AppliedLateralFeedback | None:
    return self.previous.candidate if self.previous is not None else None

  def stock_request_counts(self) -> int:
    """Return the prior limited stock request without changing plant state."""
    candidate = self.candidate
    if candidate is None:
      raise ValueError('Candidate-owned applied feedback is unavailable')
    if candidate.stock_counts is None:
      raise ValueError('Exact candidate stock-command history is unavailable')
    return int(candidate.stock_counts)


def expose_applied_feedback(controller, update_mono, previous):
  """Install one immutable pre-update context on a replay controller and plant.

  This is the production-neutral seam: replay-only candidate code can inspect
  ``_replay_applied_feedback`` on either object, while an unmodified controller
  and ``TiLateralPlant`` execute exactly their existing path.
  """
  context = ReplayAppliedFeedback(int(update_mono), previous)
  last = getattr(controller, '_replay_applied_feedback', None)
  if last is not None and context.update_mono <= last.update_mono:
    raise ValueError('Replay feedback updates must be strictly increasing')
  plant = getattr(controller, 'plant', None)
  if plant is None:
    raise ValueError('Applied feedback requires a replay controller with a lateral plant')
  controller._replay_applied_feedback = context
  plant._replay_applied_feedback = context
  return context


class TestInterface:
  """Actual Mazda plant; conversion hooks initialize legacy PID limits only."""

  def __init__(self):
    from openpilot.selfdrive.car.mazda.lateral_plant import TiLateralPlant

    self.lateral_plant = TiLateralPlant(600.0, DT)

  def torque_from_lateral_accel(self):
    return lambda value, tune: value / tune.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda value, tune: value * tune.latAccelFactor


def controller_source(ref):
  """Explicit Git revision or current worktree; no session-specific default."""
  import subprocess

  if ref == 'worktree':
    return (ROOT / 'selfdrive/controls/lib/latcontrol_torque.py').read_text(encoding='utf-8')
  return subprocess.check_output(['git', 'show', ref + ':selfdrive/controls/lib/latcontrol_torque.py'], cwd=ROOT, text=True, encoding='utf-8')
