"""Capture replay inputs and runtime dependencies without machine-specific paths."""

import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
REPLAY_SOURCE_FILES = {
  'selfdrive/controls/controlsd.py',
  'selfdrive/controls/lib/latcontrol_torque.py',
  'selfdrive/controls/lib/latcontrol.py',
  'selfdrive/controls/lib/pid.py',
  'selfdrive/controls/lib/vehicle_model.py',
  'selfdrive/controls/lib/drive_helpers.py',
  'selfdrive/car/mazda/lateral_reference.py',
  'selfdrive/car/mazda/lateral_diagnostics.py',
  'selfdrive/car/mazda/lateral_plant.py',
  'selfdrive/car/mazda/carcontroller.py',
  'selfdrive/car/card.py',
  'selfdrive/car/__init__.py',
  'selfdrive/car/interfaces.py',
  'common/realtime.py',
  'common/filter_simple.py',
  'cereal/log.capnp',
  'cereal/car.capnp',
}


def sha256(path):
  value = hashlib.sha256()
  with path.open('rb') as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b''):
      value.update(block)
  return value.hexdigest()


def write_json(path, value):
  with path.open('x', encoding='utf-8', newline='\n') as stream:
    json.dump(value, stream, sort_keys=True, indent=2, allow_nan=False)
    stream.write('\n')


def read_json(path):
  def reject(value):
    raise ValueError(f'Non-finite JSON: {value}')

  def finite(value):
    number = float(value)
    if not math.isfinite(number):
      reject(value)
    return number

  return json.loads(path.read_text(encoding='utf-8'), parse_constant=reject, parse_float=finite)


def resolve_ref(ref):
  if ref == 'worktree':
    return ref
  return subprocess.check_output(['git', 'rev-parse', '--verify', ref + '^{commit}'], cwd=ROOT, text=True).strip()


def source_snapshot():
  """Snapshot before imports; select actual loaded files when finishing the run."""
  names = subprocess.check_output(['git', 'ls-files', '-z', '--', '*.py', '*.capnp'], cwd=ROOT).decode().split('\0')
  paths = {ROOT / name for name in names if name and (ROOT / name).is_file()}
  paths.update((ROOT / 'tools/mazda_ti').glob('*.py'))
  paths.add(ROOT / 'selfdrive/car/mazda/lateral_reference.py')
  paths.add(ROOT / 'selfdrive/car/mazda/lateral_diagnostics.py')
  return {p.relative_to(ROOT).as_posix(): sha256(p) for p in sorted(paths)}


def environment():
  packages = {}
  for name in ('numpy', 'pycapnp'):
    distribution = importlib.metadata.distribution(name)
    files = {}
    for item in distribution.files or []:
      if Path(item).suffix.lower() in ('.pyd', '.so', '.dll', '.dylib', '.py', '.pyi'):
        path = Path(distribution.locate_file(item))
        if path.is_file():
          files[str(item).replace('\\', '/')] = sha256(path)
    packages[name] = {'version': distribution.version, 'runtime_files': files}
  return {
    'python_version': platform.python_version(),
    'implementation': platform.python_implementation(),
    'system': platform.system(),
    'machine': platform.machine(),
    'byteorder': sys.byteorder,
    'python_executable_sha256': sha256(Path(sys.executable)),
    'packages': packages,
  }


def finish_sources(before):
  names = {name for name in before if name.startswith('tools/mazda_ti/') or name.endswith('.capnp')}
  names.update(REPLAY_SOURCE_FILES)
  for module in tuple(sys.modules.values()):
    filename = getattr(module, '__file__', None)
    if filename:
      path = Path(filename).resolve()
      if path.is_relative_to(ROOT) and path.suffix == '.py':
        names.add(path.relative_to(ROOT).as_posix())
  result = {}
  for name in sorted(names):
    if name not in before:
      raise ValueError(f'Loaded source was absent from the pre-run snapshot: {name}')
    current = sha256(ROOT / name)
    if before[name] != current:
      raise ValueError(f'Source changed during the run: {name}')
    result[name] = current
  return result


def check_sources(sources):
  if not REPLAY_SOURCE_FILES.issubset(sources):
    raise ValueError('Replay source lock is incomplete; regenerate preparation and replay')
  for name, expected in sources.items():
    path = under(ROOT, name)
    if not path.is_file() or sha256(path) != expected:
      raise ValueError(f'Source identity changed: {name}; regenerate preparation and replay')


def check_artifacts(root, hashes, required):
  if set(hashes) != set(required):
    raise ValueError('Artifact manifest is incomplete or unexpected')
  for name, expected in hashes.items():
    if sha256(under(root, name)) != expected:
      raise ValueError(f'Artifact changed: {name}')


def under(root, name):
  path = (root / name).resolve()
  if Path(name).is_absolute() or not path.is_relative_to(root.resolve()):
    raise ValueError(f'Data path escapes the supplied root: {name}')
  return path
