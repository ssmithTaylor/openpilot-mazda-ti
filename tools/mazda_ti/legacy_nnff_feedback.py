"""Isolate rate inputs in original NNFF feedback without changing its replay."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import sys

import numpy as np

from . import legacy_nnff_probe as probe
from . import legacy_nnff_runtime as h


def feedback_error(evaluate, setpoint, measurement, desired_acceleration):
  """Original full-neural error and high-demand blend, including Float32 stores."""
  if len(setpoint)!=18 or len(measurement)!=18:
    raise ValueError('Requires original 18-input network')
  if not np.all(np.isfinite([*setpoint,*measurement,desired_acceleration])):
    raise ValueError('Nonfinite input')
  error = float(np.float32(evaluate(list(setpoint))-evaluate(list(measurement))))
  blend = float(np.interp(abs(desired_acceleration),[1.,2.],[0.,1.]))
  if blend>0:
    stronger = evaluate([setpoint[0],setpoint[1]-measurement[1],setpoint[2]-measurement[2],0.])
    if np.sign(error)==np.sign(stronger) and abs(error)<abs(stronger):
      error = float(np.float32(error*(1-blend)+stronger*blend))
  return error


def isolate_rates(evaluate, setpoint, measurement, desired_acceleration):
  result = {}
  for mode in ('original','zero_desired_rate','zero_measured_rate','zero_both_rates'):
    sp,ms = list(setpoint),list(measurement)
    if mode in ('zero_desired_rate','zero_both_rates'): sp[2]=0.
    if mode in ('zero_measured_rate','zero_both_rates'): ms[2]=0.
    result[mode] = feedback_error(evaluate,sp,ms,desired_acceleration)
  return result


def capture(baseline, data_root, baseline_path):
  """Fresh original replay; side evaluations never enter PID or model history."""
  runtime_factory,writer,argv = h.original_runtime,h.write_json,sys.argv
  captured,completed = [],[]

  def instrument(blobs):
    ns,model = runtime_factory(blobs)
    update = ns['LatControlNNFF'].update
    original_evaluate = model.evaluate
    pure_evaluate = lambda v: ns['FluxModel'].evaluate(model,list(v))
    calls = []

    def observe(values):
      values_copy = list(values)
      out = original_evaluate(values)
      calls.append({'input':values_copy,'output':out})
      return out

    model.evaluate = observe

    def observed_update(self,*args,**kwargs):
      calls.clear()
      result = update(self,*args,**kwargs)
      log = result[2]
      if not log.active or len(calls) not in (3,4):
        raise ValueError('Unsupported NNFF evaluation path')
      sp,ms = calls[0]['input'],calls[1]['input']
      desired = float(args[5])*float(args[1].vEgo)**2
      effects = isolate_rates(pure_evaluate,sp,ms,desired)
      if effects['original']!=float(log.error):
        raise ValueError('Feedback expression does not exactly reproduce original error')
      captured.append({'desired_mps2':float(log.desiredLateralAccel),'actual_mps2':float(log.actualLateralAccel),
                       'setpoint_input':sp,'measurement_input':ms,'neural_calls':list(calls),
                       'error_variants':effects,'p':float(log.p),'i':float(log.i),'f':float(log.f),
                       'output':float(log.output)})
      return result

    ns['LatControlNNFF'].update = observed_update
    return ns,model

  try:
    h.original_runtime = instrument
    h.write_json = lambda path,result: completed.append(result)
    # Request bytes remain the existing shared case; never regenerate an approximate request.
    request = h.ROOT/'tools/mazda_ti/cases/legacy_nnff_261_vw.json'
    if json.loads(request.read_text())!=baseline['request'] or h.sha256(request)!=baseline['request_sha256']:
      raise ValueError('This comparison requires the fixed VW request')
    sys.argv = ['legacy_nnff_probe','--request',str(request),'--data-root',str(data_root),
      '--choice',baseline['input_choice'],'--output-bound',baseline['output_publication_bound'],
      '--constraints','serialized','--receipt-sequence','--output',str(baseline_path)+'.unused-capture-output']
    with contextlib.redirect_stdout(io.StringIO()): probe.main()
  finally:
    h.original_runtime,h.write_json,sys.argv = runtime_factory,writer,argv
  if len(completed)!=1 or completed[0]!=baseline:
    raise ValueError('Instrumented replay differs from the complete original baseline')
  if len(captured)!=baseline['summary']['counts']['updates']:
    raise ValueError('Capture coverage differs from replay updates')
  rows = captured[-len(baseline['rows']):]
  for row,old in zip(rows,baseline['rows']):
    if row['error_variants']['original']!=old['reproduced_error'] or row['output']!=old['reproduced_output']:
      raise ValueError('Capture/controller row alignment failed')
    row.update(mono_ns=old['mono_ns'],scored=old['scored'],input_mono_ns=old['input_mono_ns'])
  return rows


def main():
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--baseline',type=Path,required=True)
  parser.add_argument('--data-root',type=Path,required=True)
  parser.add_argument('--output',type=Path,required=True)
  args=parser.parse_args()
  if args.output.exists(): parser.error('Preserve earlier output')
  digest=h.sha256(args.baseline)
  baseline=json.loads(args.baseline.read_text())
  if baseline['git_revision']!=h.REF or not baseline['receipt_sequence']:
    raise ValueError('Unsupported baseline provenance')
  source_hash=h.sha256(Path(__file__))
  rows=capture(baseline,args.data_root,args.baseline)
  if digest!=h.sha256(args.baseline) or source_hash!=h.sha256(Path(__file__)):
    raise ValueError('Evidence/source mutation')
  args.output.parent.mkdir(parents=True,exist_ok=True)
  h.write_json(args.output,{'scope':__doc__,'qualification':'instantaneous_expression_isolation_only',
    'baseline_sha256':digest,'baseline_request':baseline['request'],'source_sha256':source_hash,
    'baseline_source_sha256':baseline['source_sha256'],'git_blob_sha256':baseline['git_blob_sha256'],
    'runtime':baseline['runtime'],'input_choice':baseline['input_choice'],
    'output_bound':baseline['output_publication_bound'],'full_baseline_identical':True,'rows':rows,
    'limits':['Rate interventions change only neural feedback input index2; all other features fixed.',
      'No candidate PID/integrator/feedforward/actuator or physical-path evolution.',
      'Effect includes nonlinear error blend; individual rate effects need not add.',
      'Original input index2 is scaled desired/measured jerk, not raw steering rate.',
      'Original normalized reconstruction residuals and receipt assumptions remain.']})
  print(json.dumps({'captured_rows':len(rows),'full_baseline_identical':True}))


if __name__=='__main__': main()
