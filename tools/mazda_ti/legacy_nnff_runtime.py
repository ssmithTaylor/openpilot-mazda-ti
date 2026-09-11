"""Pinned original NNFF runtime and legacy input constraints for offline probes."""
import ast
from bisect import bisect_right
from collections import deque
import io
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from cereal import log, car
from tools.mazda_ti.audit_diagnostics import read_events
from tools.mazda_ti.provenance import environment, sha256, under, write_json

REF = '2a098cdbb2ae1a1c231220f37faca45b9d8b8c97'
NN = 'frogpilot/controls/lib/neural_network_feedforward.py'
ASSET = 'frogpilot/assets/nnff_models/MAZDA_CX9_2021.json'
SOURCES = [NN, ASSET, 'common/filter_simple.py', 'common/numpy_fast.py',
           'selfdrive/modeld/constants.py', 'selfdrive/controls/lib/vehicle_model.py',
           'selfdrive/controls/controlsd.py', 'selfdrive/controls/lib/drive_helpers.py',
           'selfdrive/controls/lib/pid.py', 'frogpilot/tinygrad_modeld/tinygrad_modeld.py']


class FeedforwardCapture:
    """No PID feedback or output qualification: capture its feedforward argument only."""
    def __init__(self, *args, **kwargs):
        self.p = self.i = self.d = self.f = 0.0

    def update(self, error, *, feedforward, **kwargs):
        self.f = feedforward
        return 0.0


class NoActuationBase:
    def __init__(self, *args):
        self.steer_max = 1.0

    def _check_saturation(self, *args):
        return False


def definitions(source, namespace, label):
    tree = ast.parse(source.decode('utf-8'))
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    exec(compile(tree, label, 'exec'), namespace)


def constant(source, name):
    values = [ast.literal_eval(n.value) for n in ast.parse(source.decode('utf-8')).body
              if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in n.targets)]
    if len(values) != 1:
        raise ValueError('Missing or ambiguous original constant: '+name)
    return values[0]


def original_runtime(blobs):
    ns = {'np': np, 'solve': np.linalg.solve, 'car': car, 'log': log, 'math': math,
          'json': json, 'deque': deque, 'LatControl': NoActuationBase, 'PIDController': FeedforwardCapture}
    for path in ['common/filter_simple.py', 'common/numpy_fast.py', 'selfdrive/modeld/constants.py',
                 'selfdrive/controls/lib/vehicle_model.py']:
        definitions(blobs[path], ns, path)
    ns['CONTROL_N'] = constant(blobs['selfdrive/controls/lib/drive_helpers.py'], 'CONTROL_N')
    ns['LAT_SMOOTH_SECONDS'] = constant(blobs['frogpilot/tinygrad_modeld/tinygrad_modeld.py'], 'LAT_SMOOTH_SECONDS')
    pid_tree=ast.parse(blobs['selfdrive/controls/lib/pid.py'].decode())
    pid=next(n for n in pid_tree.body if isinstance(n,ast.ClassDef) and n.name=='PIDController')
    init=next(n for n in pid.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
    defaults=dict(zip([a.arg for a in init.args.args][-len(init.args.defaults):],init.args.defaults))
    if ast.literal_eval(defaults['k_f']) != 1.0:
        raise ValueError('Feedforward gain differs from capture assumption')
    def asset_open(path, mode):
        if path != ASSET or mode != 'r':
            raise ValueError('Unexpected model file access')
        return io.StringIO(blobs[ASSET].decode('utf-8'))
    ns['open'] = asset_open
    definitions(blobs[NN], ns, NN)
    OriginalModel = ns['FluxModel']
    class CapturedModel(OriginalModel):
        def evaluate(self, values):
            self.last_input = list(values)
            return super().evaluate(values)
    model = CapturedModel(ASSET)
    if model.friction_override:
        raise ValueError('This probe requires the verified no-override model')
    ns['get_nn_model'] = lambda *args: model
    return ns, model


def asof(rows, times, stamp):
    i = bisect_right(times, stamp)-1
    return None if i < 0 else rows[i]


def select_inputs(c, mono, streams, times, vm, choice):
    """Constrain identities with logged desired/actual curvature, never f or output."""
    t = c.lateralControlState.torqueState
    cs_end = bisect_right(times['carState'], mono)
    lp_end = bisect_right(times['liveParameters'], mono)
    candidates = []
    for ce in streams['carState'][max(0,cs_end-4):cs_end]:
        cs = ce.carState
        desired_error = abs(c.desiredCurvature*cs.vEgo**2-t.desiredLateralAccel)
        if desired_error > 5e-7:
            continue
        for pe in streams['liveParameters'][max(0,lp_end-3):lp_end]:
            lp = pe.liveParameters
            vm.update_params(max(lp.stiffnessFactor,.1),max(lp.steerRatio,.1))
            curvature = -vm.calc_curvature(math.radians(cs.steeringAngleDeg-lp.angleOffsetDeg),cs.vEgo,lp.roll)
            if abs(curvature-c.curvature) <= 2e-9 and abs(curvature*cs.vEgo**2-t.actualLateralAccel) <= 5e-7:
                candidates.append((ce,pe,desired_error))
    if not candidates:
        return None
    fn = max if choice == 'latest' else min
    ce,pe,error = fn(candidates,key=lambda pair:(int(pair[0].logMonoTime),int(pair[1].logMonoTime)))
    return ce, pe, len(candidates), error
