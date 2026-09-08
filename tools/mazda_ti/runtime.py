"""Load the real Python controller on Windows without hardware interfaces.

PID, reference filters, vehicle model, plant inverse, breaker and final controller
clipping are real repository code. Only hardware-importing modules needed for
literal constants are replaced by constant-only modules sourced from their AST.
No CAN packer, device Params, sockets, or vehicle-motion simulator is provided.
"""

import ast
import importlib
from pathlib import Path
import sys
import types

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
