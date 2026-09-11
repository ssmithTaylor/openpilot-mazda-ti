"""Audit planar lane-motion consistency at original model publication times."""
import argparse
from bisect import bisect_right
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from tools.mazda_ti.audit_diagnostics import read_events
from tools.mazda_ti.lane_motion_balance import integrate_observations
from tools.mazda_ti.provenance import environment, sha256, under, write_json


def validate_request(request):
  if not request.get('cases'):
    raise ValueError('At least one case required')
  for case in request['cases'].values():
    bounds = case['bounds_ns']
    if (len(bounds)!=2 or any(type(t) is not int for t in bounds) or not 0<bounds[0]<bounds[1]
        or case['data_root'] not in ('old','reference') or not case['raw_sha256']):
      raise ValueError('Require valid integer bounds, data-root selector and nonempty raw manifest')
    routes = {Path(name).parent.name.rsplit('--',1)[0] for name in case['raw_sha256']}
    if len(routes)!=1 or any(Path(name).name!='rlog' for name in case['raw_sha256']):
      raise ValueError('Each case must contain rlogs from one route')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--request',type=Path,required=True)
  parser.add_argument('--old-data-root',type=Path,required=True)
  parser.add_argument('--reference-data-root',type=Path,required=True)
  parser.add_argument('--output',type=Path,required=True)
  args = parser.parse_args()
  if args.output.exists():
    parser.error('Preserve existing output')
  request_hash = sha256(args.request)
  request = json.loads(args.request.read_text())
  validate_request(request)
  gpath = ROOT/'selfdrive/car/mazda/lateral_reference.py'
  spec = importlib.util.spec_from_file_location('balance_geometry',gpath)
  geom = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = geom
  spec.loader.exec_module(geom)
  sources = {p.relative_to(ROOT).as_posix():sha256(p) for p in [Path(__file__).resolve(),gpath,
      ROOT/'tools/mazda_ti/lane_motion_balance.py',ROOT/'tools/mazda_ti/provenance.py',
      ROOT/'tools/mazda_ti/audit_diagnostics.py',ROOT/'cereal/__init__.py',*sorted((ROOT/'cereal').glob('*.capnp'))]}
  runtime = environment()
  result = {'scope':__doc__,'request':request,'request_sha256':request_hash,'source_sha256':sources,
            'runtime':runtime,'cases':{},'limits':[
      'Fixed declared bounds; selection provenance belongs to the request, not the measured outcome.',
      'Nearest model publication to each bound, at most40ms; endpoint selection never consults outcomes.',
      'CarState/localizer are latest publications before model publication, not consumed identities.',
      'Camera EOF is recorded separately; event-time kinematic balance is not camera-aligned ground truth.',
      'Only lane geometry, speed and yaw health affect availability; liveParameters is not an input here.',
      'Any unavailable row prevents whole-window integration; no interpolation across bad geometry.',
      'Recorded-state balance cannot predict a changed vehicle path.',
      'Planar no-sideslip approximation; model geometry is not surveyed Frenet coordinates.',
      'Yaw sensitivity covers a coherent reported-std perturbation only; no confidence-interval interpretation.',
      'Both fit spans share model samples; neither is a calibrated uncertainty bound.']}
  for name in sorted(request['cases']):
    hashes = request['cases'][name]['raw_sha256']
    root = args.reference_data_root if request['cases'][name]['data_root']=='reference' else args.old_data_root
    files = {n:under(root,n) for n in hashes}
    if hashes != {n:sha256(p) for n,p in files.items()}:
      raise ValueError('Raw differs from original geometry extraction')
    bounds = request['cases'][name]['bounds_ns']
    cs, llk, models = {}, {}, {}
    for event in read_events(list(files.values())):
      s,t = event.which(),int(event.logMonoTime)
      if not bounds[0]-1_000_000_000<=t<=bounds[1]+100_000_000:
        continue
      if s=='carState':
        if t in cs: raise ValueError('Duplicate carState')
        cs[t] = {'valid':bool(event.valid),'can_valid':bool(event.carState.canValid),'speed':float(event.carState.vEgo)}
      elif s=='liveLocationKalman':
        if t in llk: raise ValueError('Duplicate localizer')
        k = event.liveLocationKalman
        llk[t] = {'valid':bool(event.valid),'yaw_valid':bool(k.angularVelocityCalibrated.valid),
                  'sensors_ok':bool(k.sensorsOK),'inputs_ok':bool(k.inputsOK),'posenet_ok':bool(k.posenetOK),
                  'yaw_rate':float(k.angularVelocityCalibrated.value[2]),'yaw_std_rps':float(k.angularVelocityCalibrated.std[2])}
      elif s=='modelV2':
        if t in models: raise ValueError('Duplicate model')
        g = asdict(geom.fit_lane_context(event.modelV2,0.))
        g.pop('age')
        models[t] = {'valid':bool(event.valid),'camera_eof_ns':int(event.modelV2.timestampEof),'geometry':g}
    mt,ct,lt = sorted(models),sorted(cs),sorted(llk)
    endpoints = [min(mt,key=lambda t:(abs(t-b),t)) for b in bounds]
    if any(abs(t-b)>40_000_000 for t,b in zip(endpoints,bounds)):
      raise ValueError('Missing model endpoint')
    rows = []
    for t in mt:
      if not endpoints[0]<=t<=endpoints[1]: continue
      ci,li = bisect_right(ct,t)-1,bisect_right(lt,t)-1
      if min(ci,li)<0: raise ValueError('Missing preceding speed/yaw publication')
      c,l,m = cs[ct[ci]],llk[lt[li]],models[t]
      good = (m['valid'] and m['geometry']['valid'] and c['valid'] and c['can_valid']
              and all(l[k] for k in ('valid','yaw_valid','sensors_ok','inputs_ok','posenet_ok'))
              and t-ct[ci]<=200_000_000 and t-lt[li]<=200_000_000)
      rows.append({'mono_ns':t,'available':bool(good),'model':m,'carState':dict(c,mono_ns=ct[ci],age_ns=t-ct[ci]),
                   'localizer':dict(l,mono_ns=lt[li],age_ns=t-lt[li])})
    case = {'raw_sha256':hashes,'requested_bounds_ns':bounds,'selected_bounds_ns':endpoints,
            'rows':rows,'unavailable_rows':sum(not r['available'] for r in rows),'fits':{}}
    for span in (10,20):
      obs = [dict(available=r['available'],mono_ns=r['mono_ns'],speed=r['carState']['speed'],
           offset=r['model']['geometry']['offset'],heading=r['model']['geometry'][f'heading{span}'],
           road_curvature=r['model']['geometry'][f'curvature{span}'],
           yaw_rate=r['localizer']['yaw_rate'],yaw_std_rps=r['localizer']['yaw_std_rps']) for r in rows]
      try: case['fits'][str(span)] = integrate_observations(obs,maximum_gap_s=.1)
      except ValueError as exc: case['fits'][str(span)] = {'unavailable':str(exc)}
    result['cases'][name] = case
    if hashes != {n:sha256(p) for n,p in files.items()}: raise ValueError('Raw changed')
  if (request_hash!=sha256(args.request) or sources!={p:sha256(ROOT/p) for p in sources}
      or runtime!=environment()): raise ValueError('Source/input/runtime changed')
  write_json(args.output,result)
  print(json.dumps({n:{'rows':len(c['rows']),'unavailable_rows':c['unavailable_rows'],
            'fits':{s:{k:v for k,v in f.items() if k!='intervals'} for s,f in c['fits'].items()}}
                   for n,c in result['cases'].items()},indent=2))


if __name__=='__main__': main()
