"""Measure the pinned controller's isolated pre-withdrawal compensation map.

This is an expression-level check, not controller, limiter or vehicle replay.
Unsupported source shapes fail closed rather than silently testing a copied law.
"""

import argparse
import ast
import hashlib
import math
from pathlib import Path

from .provenance import environment, finish_sources, resolve_ref, sha256, source_snapshot, write_json
from .runtime import controller_source


GATES = (-3.0, -0.2, 0.0, 0.2, 3.0)
SCOPE = ('Isolated production compensation expressions before withdrawal and final clipping; '
         'no PID history, software limiter, firmware receipt or vehicle-motion prediction.')


def sha256_bytes(value):
  return hashlib.sha256(value).hexdigest()


def extract(source):
  """Compile only the four recognized production expressions and their constants."""
  tree = ast.parse(source)
  constants = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      if node.targets[0].id.startswith('FRIC_COMP_'):
        constants[node.targets[0].id] = ast.literal_eval(node.value)
  cls = next((n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LatControlTorque'), None)
  method = next((n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_update_with_plant'), None) if cls else None
  blocks = [n for n in method.body if isinstance(n, ast.If) and 'lat_friction_comp' in ast.unparse(n.test)] if method else []
  if len(blocks) != 1:
    raise ValueError('Unsupported compensation source: expected one enabled compensation block')
  nodes = blocks[0].body[:4]
  targets = []
  for node in nodes:
    target = node.targets[0] if isinstance(node, ast.Assign) and len(node.targets) == 1 else (
      node.target if isinstance(node, ast.AugAssign) else None)
    targets.append(target.id if isinstance(target, ast.Name) else None)
  if targets != ['comp_mag', 'comp_mag', 'over', 'fric_comp']:
    raise ValueError('Unsupported compensation source: expression boundary changed')
  boundary = blocks[0].body[4] if len(blocks[0].body) > 4 else None
  if (not isinstance(boundary, ast.Assign) or len(boundary.targets) != 1 or
      ast.unparse(boundary.targets[0]) != 'fric_comp' or not isinstance(boundary.value, ast.Call) or
      ast.unparse(boundary.value.func) != 'self.friction_release.update' or not boundary.value.args or
      ast.unparse(boundary.value.args[0]) != 'fric_comp'):
    raise ValueError('Unsupported compensation source: pre-withdrawal boundary changed')
  names = {n.id for node in nodes for n in ast.walk(node) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
  if names - (set(constants) | {'command', 'gate_la', 'u_max', 'comp_mag', 'over', 'min', 'max', 'abs', 'math'}):
    raise ValueError('Unsupported compensation source: additional expression dependencies')
  module = ast.Module(body=nodes, type_ignores=[])
  code = compile(module, '<pinned-compensation-expressions>', 'exec')

  def compensation(command, gate):
    namespace = dict(constants, command=command, gate_la=gate, u_max=600.0, math=math)
    exec(code, namespace)
    return float(namespace['fric_comp'])

  return compensation, {'constants': constants, 'expression_ast_sha256': sha256_bytes(ast.dump(module).encode()),
                        'expression_source': ast.unparse(module)}


def measure(compensation):
  """Sweep both signs, including opposing gate/command signs and the dead zone."""
  summaries, failures = [], []
  for gate in GATES:
    values = []
    for step in range(-6000, 6001):
      command = step / 10.0
      comp = compensation(command, gate)
      value = command + comp
      if not math.isfinite(value) or abs(value) > 600.0 + 1e-9:
        failures.append(f'gate={gate}: nonfinite or out-of-envelope compensated command at {command}')
        break
      values.append(value)
    if len(values) != 12001:
      continue
    differences = [b - a for a, b in zip(values, values[1:])]
    flat = [i for i, delta in enumerate(differences) if abs(delta) <= 1e-9]
    reversals = sum(delta < -1e-9 for delta in differences)
    if flat or reversals:
      failures.append(f'gate={gate}: {len(flat)} flat and {reversals} reversed full-precision response intervals')
    summaries.append({'gate_mps2': gate, 'samples': len(values), 'minimum_increment_counts': min(differences),
                      'maximum_increment_counts': max(differences), 'flat_intervals': len(flat),
                      'reversed_intervals': reversals,
                      'first_flat_inverse_counts': None if not flat else (flat[0] - 6000) / 10.0,
                      'rounded_output_values': len({round(value) for value in values})})
  return summaries, failures


def run(revision, output):
  before = source_snapshot()
  if len(revision) != 40 or resolve_ref(revision) != revision:
    raise ValueError('Provide a full immutable Git revision')
  source = controller_source(revision)
  compensation, expressions = extract(source)
  summaries, failures = measure(compensation)
  result = {'format_version': 1, 'status': 'failed_check' if failures else 'completed_checks',
            'qualification': 'isolated_compensation_expression', 'scope': SCOPE,
            'controller_revision': revision, 'controller_source_sha256': sha256_bytes(source.encode()),
            'analyzer_sha256': sha256(Path(__file__)), 'repository_sources': finish_sources(before),
            'runtime': environment(), 'expressions': expressions,
            'grid': {'inverse_counts': [-600.0, 600.0], 'step_counts': 0.1, 'gates_mps2': list(GATES)},
            'gates': summaries, 'findings': failures,
            'limitations': ['Strict response concerns full-precision mapping, not every quantized 0.1-count step.',
                            'The enabled, available-authority branch is isolated; gating, withdrawal and state evolution are excluded.',
                            'A varying request does not establish TI receipt or avoidance of its stuck-request fault.']}
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  write_json(output / 'result.json', result)
  (output / 'report.md').write_text('# Production compensation map\n\n' + SCOPE + '\n\nStatus: **' + result['status'] +
                                   '**\n\n' + '\n'.join('- ' + item for item in failures or ['No mapping failures on the declared grid.']) + '\n',
                                   encoding='utf-8', newline='\n')
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--revision', required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result = run(args.revision, args.output)
  print(f'{result["status"]}: {args.output / "report.md"}')
  return 0 if result['status'] == 'completed_checks' else 1


if __name__ == '__main__':
  raise SystemExit(main())
