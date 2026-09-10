"""Audit historical NNFF request scaling, dual limiters and TI output feedback.

Pinned recorded650 configuration only; no device writes or changed vehicle path.
"""
import argparse
import ast
from bisect import bisect_right
import hashlib
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np

from .audit_diagnostics import read_events
from .feedback import Fixture, pure_stock_limiter
from .provenance import ROOT, environment, sha256, under, write_json

REVISION = '2a098cdbb2ae1a1c231220f37faca45b9d8b8c97'
KEYS = {'TI_STEER_MAX':'TiSteerMax','TI_STEER_DELTA_UP':'TiSteerDeltaUp',
        'TI_STEER_DELTA_DOWN':'TiSteerDeltaDown','TI_STEER_DRIVER_ALLOWANCE':'TiSteerDriverAllowance',
        'TI_STEER_DRIVER_MULTIPLIER':'TiSteerDriverMultiplier','TI_STEER_DELTA_UP_KNEE':'TiSteerDeltaUpKnee',
        'TI_STEER_DELTA_UP_HIGH':'TiSteerDeltaUpHigh'}
EXPECTED = dict(zip(KEYS, (650,10,15,15,40,620,8)))
ECHO_FIELDS = ('gas','brake','steeringAngleDeg','accel','longControlState','speed','curvature')


def copied_key(actuators):
  """Fields retained by original new_actuators=CC.actuators.as_builder()."""
  return tuple(actuators[k] for k in ECHO_FIELDS)


def echo_request_map(frames, commands, choice):
  """Monotone histories using copied fields only, never steer or limited output."""
  if choice not in ('earliest','latest'): raise ValueError('Unknown echo choice')
  groups = {}
  for t,c in sorted(commands.items()): groups.setdefault(tuple(c['echo']),[]).append(t)
  lower,rows = 0,[]
  for frame in frames:
    ts = groups.get(tuple(frame['echo']),[])
    eligible = [t for t in ts if lower<=t<=frame['upper_bound_ns']]
    if not eligible: raise ValueError({'missing_copied_request':frame['state_ns']})
    rows.append(dict(frame,candidates=eligible))
    lower = eligible[0]
  upper = max(commands)
  for row in reversed(rows):
    row['candidates'] = [t for t in row['candidates'] if t<=upper]
    if not row['candidates']: raise ValueError('No monotone copied-field history')
    upper = row['candidates'][-1]
  mapping = {r['state_ns']:r['candidates'][0 if choice=='earliest' else -1] for r in rows}
  return mapping,[{k:v for k,v in r.items() if k!='echo'} for r in rows]


def replay_dual(fixture, stock_limiter, stock_limits, initial_stock, replacements=None):
  """Original fixed schedule and source-bounded request map; own limiter histories."""
  replacements = replacements or {}
  requests = {**fixture.prior_requests, **{r.mono:r for r in fixture.requests}}
  if set(replacements)-set(requests):
    raise ValueError('Replacement references unknown request')
  steer = {t:float(np.float32(replacements.get(t,r.steer))) for t,r in requests.items()}
  if any(not np.isfinite(v) or abs(v)>1 for v in steer.values()):
    raise ValueError('Invalid normalized request')
  ti,stock = fixture.initial_send[1],initial_stock
  sensor = fixture.initial_state[1]
  sampled = fixture.initial_sampled_request.mono
  sends,outputs = [],[]
  for t,kind,value in fixture.events:
    if kind=='state':
      sensor = value
      sampled = fixture.request_identity[t]
      if sampled>t or sampled not in requests:
        raise ValueError('Unknown/future sampled request')
    elif kind=='send':
      u = steer[sampled]
      ti_wanted,stock_wanted = round(u*fixture.limits.TI_STEER_MAX),round(u*stock_limits.STEER_MAX)
      ti = fixture.limiter(ti_wanted,ti,sensor,fixture.limits)
      stock = stock_limiter(stock_wanted,stock,sensor,stock_limits)
      sends.append({'mono_ns':t,'request_ns':sampled,'steer':u,'driver_sensor':sensor,
                    'ti_requested':ti_wanted,'stock_requested':stock_wanted,'ti':ti,'stock':stock})
    elif kind=='output':
      outputs.append({'mono_ns':t,'counts':ti,'steer':float(np.float32(ti/fixture.limits.TI_STEER_MAX))})
  return {'sends':sends,'outputs':outputs}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--request',type=Path,required=True)
  parser.add_argument('--data-root',type=Path,required=True)
  parser.add_argument('--controller-trace',type=Path,action='append',default=[])
  parser.add_argument('--receipt-mode',choices=('latest-bound','echo-earliest','echo-latest'),default='latest-bound')
  parser.add_argument('--output',type=Path,required=True)
  args = parser.parse_args()
  if args.output.exists(): parser.error('Preserve previous output')
  request_hash = sha256(args.request)
  req = json.loads(args.request.read_text())
  start,end = req['integral_anchor_start_ns'],req['end_ns']
  if not req['warmup_start_ns']<start<req['score_start_ns']<end:
    raise ValueError('Invalid controller request bounds')
  paths = {n:under(args.data_root,n) for n in req['rlogs']}
  if {n:sha256(p) for n,p in paths.items()} != req['raw_sha256']:
    raise ValueError('Changed raw input')
  source_paths = [Path(__file__).resolve(),*[ROOT/'tools/mazda_ti'/n for n in
    ('feedback.py','runtime.py','audit_diagnostics.py','provenance.py')],
    ROOT/'cereal/__init__.py',*sorted((ROOT/'cereal').glob('*.capnp'))]
  sources = {p.relative_to(ROOT).as_posix():sha256(p) for p in source_paths}
  runtime = environment()
  blob_names = ['selfdrive/car/__init__.py','selfdrive/car/mazda/values.py',
                'selfdrive/car/mazda/carcontroller.py','selfdrive/car/mazda/carstate.py',
                'selfdrive/car/mazda/mazdacan.py','selfdrive/car/card.py',
                'selfdrive/car/interfaces.py','selfdrive/car/mazda/interface.py',
                'selfdrive/controls/controlsd.py','opendbc/mazda_2017.dbc']
  blobs = {p:subprocess.check_output(['git','show',f'{REVISION}:{p}'],cwd=ROOT) for p in blob_names}
  cls = next(n for n in ast.parse(blobs['selfdrive/car/mazda/values.py']).body if isinstance(n,ast.ClassDef) and n.name=='CarControllerParams')
  init = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
  gen1 = next(n for n in init.body if isinstance(n,ast.If) and ast.unparse(n.test)=='CP.flags & MazdaFlags.GEN1')
  factor = next(ast.literal_eval(n.value) for n in gen1.body if isinstance(n,ast.Assign)
                and isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='TI_STEER_DRIVER_FACTOR')
  times = {s:[] for s in ('carState','carControl','carOutput','controlsState')}
  commands,controllers,stock_sends,original_outputs,echoes = {},{},{},{},{}
  settings = []
  invalid = []
  for e in read_events(list(paths.values())):
    s,t = e.which(),int(e.logMonoTime)
    if s=='initData':
      if e.initData.gitCommit!=REVISION or e.initData.dirty: raise ValueError('Unsupported recorded source')
      v = {p.key:bytes(p.value).decode() for p in e.initData.params.entries
           if p.key in [*KEYS.values(),'NNFF','NNFFLite','TorqueInterceptorEnabled']}
      if any(v.get(k)!='1' for k in ('NNFF','NNFFLite','TorqueInterceptorEnabled')):
        raise ValueError('Unsupported recorded controller/actuator mode')
      limits = {k:float(v[n]) for k,n in KEYS.items()}
      if limits!=EXPECTED: raise ValueError('Unsupported historical limiter settings')
      settings.append(v)
    if s=='carParams' and e.carParams.carFingerprint!='MAZDA_CX5':
      raise ValueError('Unsupported vehicle')
    if s in times:
      times[s].append(t)
      if start<t<=end and not e.valid: invalid.append({'service':s,'mono_ns':t})
    if s=='carControl':
      commands[t] = {'steer':float(e.carControl.actuators.steer),'curvature':float(e.carControl.actuators.curvature),
                     'echo':copied_key(e.carControl.actuators.to_dict())}
    elif s=='controlsState':
      c=e.controlsState
      if c.lateralControlState.which()=='torqueState':
        controllers[t] = {'output':float(c.lateralControlState.torqueState.output),'curvature':float(c.desiredCurvature)}
    elif s=='carOutput':
      original_outputs[t] = float(e.carOutput.actuatorsOutput.steer)
      echoes[t] = copied_key(e.carOutput.actuatorsOutput.to_dict())
    elif s=='sendcan':
      for frame in e.sendcan:
        if frame.address==0x243 and frame.src==0:
          if t in stock_sends: raise ValueError('Duplicate stock send')
          stock_sends[t] = (((frame.dat[0]&15)<<8)|frame.dat[1])-2048
      if start<t<=end and not e.valid: invalid.append({'service':s,'mono_ns':t})
  if invalid or not settings: raise ValueError({'invalid_publications':invalid,'settings_count':len(settings)})
  for s,ts in times.items():
    ts.sort()
    if len(set(ts))!=len(ts): raise ValueError('Duplicate '+s)
  mapping = {}
  echo_frames = []
  stock_times = sorted(stock_sends)
  for t in times['carState']:
    if not start<t<=end: continue
    output = times['carOutput'][bisect_right(times['carOutput'],t)-1]
    mapping[t] = times['carControl'][bisect_right(times['carControl'],output)-1]
    if args.receipt_mode!='latest-bound':
      send = stock_times[bisect_right(stock_times,t)]
      echo = times['carOutput'][bisect_right(times['carOutput'],send)]
      next_state = times['carState'][bisect_right(times['carState'],t)]
      if not output<t<send<echo<next_state:
        raise ValueError('Missing/ambiguous card cycle order for copied-field constraint')
      echo_frames.append({'state_ns':t,'send_ns':send,'upper_bound_ns':output,'echo_ns':echo,'echo':echoes[echo]})
  echo_choices = []
  if echo_frames:
    mapping,echo_choices = echo_request_map(echo_frames,commands,args.receipt_mode.split('-')[1])
  fixture = Fixture(list(paths.values()),start,end,SimpleNamespace(**EXPECTED,TI_STEER_DRIVER_FACTOR=factor),REVISION,request_identity=mapping)
  if {t for t in stock_sends if start<t<=end} != set(fixture.actual_sends):
    raise ValueError('TI/stock send schedules differ; dual-path coverage is incomplete')
  stock_limit,stock_params,_ = pure_stock_limiter(REVISION)
  initial_stock = stock_sends[max(t for t in stock_sends if t<=start)]
  baseline = replay_dual(fixture,stock_limit,stock_params,initial_stock)
  ti_errors = [r for r in baseline['sends'] if r['ti']!=fixture.actual_sends[r['mono_ns']]]
  stock_errors = [r for r in baseline['sends'] if r['stock']!=stock_sends[r['mono_ns']]]
  output_errors = [r for r in baseline['outputs'] if (r['counts'],r['steer'])!=(fixture.actual_outputs[r['mono_ns']].counts,fixture.actual_outputs[r['mono_ns']].steer)]
  exact = not (ti_errors or stock_errors or output_errors)
  summary = {'sends':len(baseline['sends']),'outputs':len(baseline['outputs']),
             'ti_errors':len(ti_errors),'stock_errors':len(stock_errors),'output_errors':len(output_errors),'exact':exact}
  pairings = {}
  for t in sorted(commands):
    if not start<t<=end: continue
    ct = times['controlsState'][bisect_right(times['controlsState'],t)-1]
    c = controllers.get(ct)
    if ct>t or c is None or c['output']!=commands[t]['steer'] or c['curvature']!=commands[t]['curvature']:
      raise ValueError({'unpaired_carControl':t,'preceding_controlsState':ct})
    pairings[t] = ct
  arms = []
  trace_hashes = {}
  for path in args.controller_trace if exact else []:
    digest = sha256(path)
    trace = json.loads(path.read_text())
    if (trace['request']!=req or trace['git_revision']!=REVISION or trace['raw_sha256']!=req['raw_sha256']
        or trace['runtime']!=runtime or any(sha256(ROOT/p)!=h for p,h in trace['source_sha256'].items())):
      raise ValueError('Controller trace/source/runtime incompatible')
    trace_hashes[str(path.resolve())] = digest
    by_time = {r['mono_ns']:r for r in trace['rows']}
    replacements = {}
    for cc,ct in pairings.items():
      if ct<=trace['anchor']['mono_ns']: continue
      if ct not in by_time or by_time[ct]['original_output']!=commands[cc]['steer']:
        raise ValueError('Controller trace lacks paired output')
      replacements[cc] = by_time[ct]['reproduced_output']
    arm = replay_dual(fixture,stock_limit,stock_params,initial_stock,replacements)
    sent = [dict(r,ti_actual=fixture.actual_sends[r['mono_ns']],stock_actual=stock_sends[r['mono_ns']]) for r in arm['sends']]
    arm_outputs = {r['mono_ns']:r['steer'] for r in arm['outputs']}
    feedback_flags = []
    tr = trace['rows']
    for i,r in enumerate(tr[:-1]):
      co = r['input_mono_ns']['carOutput']
      candidate_co = original_outputs[co] if co<=start else arm_outputs[co]
      expected = tr[i+1]['steer_limited_previous_cycle']
      from_recorded = abs(r['reproduced_output']-r['reported_output'])>1e-2
      from_replay = abs(r['reproduced_output']-candidate_co)>1e-2
      if from_recorded!=expected or from_replay!=expected:
        feedback_flags.append({'controller_ns':r['mono_ns'],'expected_next':expected,'recorded_next':from_recorded,'replayed_next':from_replay})
    arms.append({'trace_sha256':digest,'input_choice':trace['input_choice'],'output_bound':trace['output_publication_bound'],
                 'summary':{'sends':len(sent),'ti_errors':sum(r['ti']!=r['ti_actual'] for r in sent),
                   'stock_errors':sum(r['stock']!=r['stock_actual'] for r in sent),
                   'max_ti_error':max(abs(r['ti']-r['ti_actual']) for r in sent),
                   'max_stock_error':max(abs(r['stock']-r['stock_actual']) for r in sent),
                   'feedback_flag_differences':len(feedback_flags)},
                 'feedback_flag_differences':feedback_flags,'sends':sent,'outputs':arm['outputs']})
  result = {'scope':__doc__,'qualification':'component_audit_not_release_qualification',
            'all_supplied_controller_arms_exact':bool(arms) and exact and all(
                a['summary']['ti_errors']==a['summary']['stock_errors']==a['summary']['feedback_flag_differences']==0 for a in arms),
            'request':req,'request_sha256':request_hash,'source_revision':REVISION,
            'raw_sha256':req['raw_sha256'],'source_sha256':sources,'runtime':runtime,
            'git_blob_sha256':{p:hashlib.sha256(b).hexdigest() for p,b in blobs.items()},
            'settings':settings,'receipt_mode':args.receipt_mode,'echo_choices':echo_choices,
            'sampling':'Request creation bounded by preceding carOutput. Optional next-output copied fields constrain identities; no steering or limited-count values select history.',
            'summary':summary,'baseline':baseline,'baseline_errors':{'ti':ti_errors,'stock':stock_errors,'output':output_errors},
            'request_to_controller':pairings,'request_map':mapping,'controller_arms':arms,
            'controller_arms_not_run_reason':None if exact else 'Recorded actuator baseline failed; preserve and resolve first.',
            'limits':['Fixed recorded schedule and physical observations; continuous active TI RUN only.',
                      'Legacy publication bounds are hypotheses, not explicit receipt identities.',
                      'Checks decoded steering counts and TI feedback, not every CAN payload/checksum byte or EPS delivery.',
                      'Recorded650 scale does not authorize raising the present600-count vehicle limit.',
                      'A failing exact count check is retained; no one-count tolerance or fitted timing exception.',
                      'NNFF trace uses original recorded observations; feedback equivalence needs its own reported checks.']}
  if (request_hash!=sha256(args.request) or req['raw_sha256']!={n:sha256(p) for n,p in paths.items()}
      or sources!={p.relative_to(ROOT).as_posix():sha256(p) for p in source_paths}
      or runtime!=environment() or any(sha256(Path(p))!=h for p,h in trace_hashes.items())):
    raise ValueError('Source/input/runtime mutation')
  write_json(args.output,result)
  print(json.dumps({'baseline':summary,'controller_arms':[a['summary'] for a in arms]}))


if __name__=='__main__': main()
