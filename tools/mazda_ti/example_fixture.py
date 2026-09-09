"""Generate public straight-line synthetic rlogs; expected commands are independently specified as zero."""

import argparse
from pathlib import Path

from cereal import log
from .provenance import read_json, sha256, write_json


def create_example(output):
  """Serialize fixed inputs, not outputs obtained by running the controller being tested."""
  output.mkdir(parents=True, exist_ok=False)
  request = read_json(Path(__file__).with_name('examples') / 'vw-evaluation.json')
  case, events = request['case'], []
  case.update(id='synthetic-straight-v1', origin='synthetic', history='latest')
  spec = case['experiment']
  spec.update(name='synthetic-straight-v1', rlogs=['synthetic--0/rlog'],
              window={'anchor_i_at': 1.05, 'activation': 1.1, 'start': 1.1, 'end': 1.29})
  spec['settings'].update(LatDamping=0, LatCommitSetpoint=0, LatFrictionComp=0)

  def event(kind, mono, values):
    message = log.Event.new_message(logMonoTime=mono, valid=True, **{kind: values})
    events.append(message.to_bytes())

  event('initData', 1, {'params': {'entries': [{'key': k, 'value': str(v).encode()} for k, v in sorted(spec['settings'].items())]}})
  event('carParams', 2, {'mass': 1700, 'rotationalInertia': 3000, 'wheelbase': 2.7, 'centerToFront': 1.1,
                       'steerRatio': 15, 'tireStiffnessFront': 80000, 'tireStiffnessRear': 80000, 'steerLimitTimer': 1,
                       'lateralTuning': {'torque': {'kp': .8, 'ki': .2, 'latAccelFactor': 3}}})
  event('liveParameters', 3, {'stiffnessFactor': 1, 'steerRatio': 15})
  event('liveTorqueParameters', 4, {'latAccelFactorFiltered': 3})
  event('liveDelay', 5, {'lateralDelay': .39})
  event('frogpilotCarState', 6, {'tiActive': True})
  event('can', 7, [{'address': 0x24A, 'src': 1, 'dat': bytes([0, 0, 0, 3, 0, 0, 0, 0])}])
  event('carControl', 800_000_000, {'latActive': True})
  event('carState', 810_000_000, {'vEgo': 20})
  event('sendcan', 820_000_000, [{'address': 0x249, 'src': 1, 'dat': bytes([8, 0])}])
  event('carOutput', 830_000_000, {})
  for index in range(30):
    mono = 1_000_000_000 + index * 10_000_000
    event('modelV2', mono + 1_000_000, {})
    event('carState', mono + 2_000_000, {'vEgo': 20})
    event('controlsState', mono + 3_000_000, {'lateralPlanMonoTime': mono + 1_000_000,
                                          'lateralControlState': {'torqueState': {'active': True, 'plantState': 3}}})
    event('carControl', mono + 4_000_000, {'latActive': True})
    event('sendcan', mono + 5_000_000, [{'address': 0x249, 'src': 1, 'dat': bytes([8, 0])}])
    event('carOutput', mono + 6_000_000, {})
  raw = output / 'synthetic--0/rlog'
  raw.parent.mkdir()
  raw.write_bytes(b''.join(events))
  case['input_sha256'] = {spec['rlogs'][0]: sha256(raw)}
  write_json(output / 'request.json', request)
  return output / 'request.json'


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  print(create_example(parser.parse_args().output))
