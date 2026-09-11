"""Versioned request allowlist and evidence-derived result fields."""

import math
from pathlib import PurePosixPath
import re

from .run import LIMIT_KEYS, TOGGLE_KEYS

SCOPE = ('Steering commands under recorded physical motion; no prediction of a candidate vehicle path or successful cornering. '
         'Software TI feedback follows each candidate; physical observations and actuator availability stay recorded.')
LIMITATIONS = [
  'One compatible historical sampling history is evaluated; it does not bound all possible histories.',
  'A candidate revision selects controller source; other dependencies are the captured current repository sources.',
]


class Unsupported(ValueError):
  """The request asks for coverage this adapter does not provide."""


def evidence_scope(origin):
  if origin == 'recorded':
    return SCOPE
  if origin == 'synthetic':
    return ('Synthetic fixed-input command fixture; no recorded physical handling evidence. '
            'Software TI feedback follows each candidate; no prediction of a vehicle path or successful cornering.')
  return 'Evidence origin unavailable; no recorded physical handling evidence or successful cornering established.'


def canonical_request(value, resolved=False):
  """Reject unknown fields and arbitrary values before persisting any request content."""
  def fields(obj, required, optional=()):
    if not isinstance(obj, dict) or not set(required) <= obj.keys() or obj.keys() - set(required) - set(optional):
      raise ValueError('Request contains missing or unexpected fields')
    return {key: obj[key] for key in required}

  if not isinstance(value, dict):
    raise ValueError('Evaluation request must be an object')
  if type(value.get('format_version')) is not int or value['format_version'] != 1:
    raise Unsupported('Unsupported evaluation request version')
  if isinstance(value.get('case'), dict) and value['case'].get('method') not in ('historical', 'instrumented'):
    raise Unsupported('Unsupported replay adapter')
  request = fields(value, ('format_version', 'candidate_revision', 'case'))
  case = fields(value['case'], ('id', 'method', 'history', 'experiment', 'input_sha256'), ('origin',))
  case['origin'] = value['case'].get('origin', 'recorded')
  histories = ('exact',) if case['method'] == 'instrumented' else ('earliest', 'latest')
  if case['history'] not in histories or case['origin'] not in ('recorded', 'synthetic'):
    raise Unsupported('Unsupported history or evidence origin')
  if resolved and request['candidate_revision'] == 'worktree':
    raise Unsupported('This slice requires a committed candidate controller revision')
  keys = ('format_version', 'name', 'rlogs', 'window', 'baseline_controller', 'warmup_controller', 'limiter',
          'settings', 'fpcs_sample_at', 'force_offset') + (('candidate_controller',) if resolved else ())
  spec = fields(case['experiment'], keys)
  if type(spec['format_version']) is not int or spec['format_version'] != 1:
    raise Unsupported('Unsupported experiment version')
  spec['window'] = fields(spec['window'], ('anchor_i_at', 'activation', 'start', 'end'))
  spec['settings'] = fields(spec['settings'], sorted(set(LIMIT_KEYS.values()) | TOGGLE_KEYS))
  if any(type(v) not in (int, float) or not math.isfinite(v) for v in [*spec['window'].values(), *spec['settings'].values()]):
    raise ValueError('Settings and window values must be finite numbers')
  samples = ('diagnostic',) if case['method'] == 'instrumented' else ('carState', 'controlsState')
  if type(spec['force_offset']) is not bool or spec['fpcs_sample_at'] not in samples:
    raise ValueError('Invalid sampling configuration for the replay method')
  refs = [request['candidate_revision'], spec['baseline_controller'], spec['warmup_controller'], spec['limiter']]
  if resolved:
    refs.append(spec['candidate_controller'])
    if request['candidate_revision'] != spec['candidate_controller']:
      raise ValueError('Resolved candidate identities differ')
  if any(not isinstance(ref, str) or not re.fullmatch(r'[A-Za-z0-9_./-]{1,128}', ref) for ref in refs):
    raise ValueError('Invalid source revision')
  if any(not isinstance(v, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_. -]{0,79}', v) for v in (case['id'], spec['name'])):
    raise ValueError('Invalid case identity or name')
  names = spec['rlogs']
  if not isinstance(names, list) or not names or any(not isinstance(n, str) or not re.fullmatch(r'[A-Za-z0-9_./-]+', n)
      or PurePosixPath(n).is_absolute() or '..' in PurePosixPath(n).parts for n in names):
    raise ValueError('Recordings require nonempty relative paths')
  if len(names) != len(set(names)) or not isinstance(case['input_sha256'], dict) or set(case['input_sha256']) != set(names):
    raise ValueError('Input manifest must cover exactly the declared rlogs')
  if any(not isinstance(v, str) or not re.fullmatch('[a-f0-9]{64}', v) for v in case['input_sha256'].values()):
    raise ValueError('Invalid recording hash')
  request['case'] = dict(case, experiment=spec, input_sha256={n: case['input_sha256'][n] for n in names})
  return request


def completed_result(request, baseline, candidate, comparison):
  """Every trusted top-level field derives from validated request/replay records."""
  case, sources = request['case'], baseline['controller_sources'] | candidate['controller_sources']
  spec = case['experiment']
  return {'format_version': 1, 'status': 'completed_checks', 'case_id': case['id'], 'history': case['history'],
          'window': spec['window'], 'qualification': baseline['qualification'], 'comparison': comparison, 'findings': [],
          'scope': evidence_scope(case['origin']),
          'source_identities': {**{k: spec[k] for k in ('baseline_controller', 'candidate_controller', 'warmup_controller', 'limiter')},
                                'repository_sources': candidate['repository_sources'], 'controller_sources': sources},
          'input_sha256': candidate['input_sha256'], 'runtime': candidate['environment'],
          'limitations': candidate['limitations'] + (LIMITATIONS if case['method'] == 'historical' else LIMITATIONS[1:])}


def synthetic_result(request, status, comparison, findings, source_identities, input_sha256, runtime, limitations):
  """Use the same top-level outcome contract for bounded synthetic evidence."""
  case = request['case']
  return {
    'format_version': 1,
    'status': status,
    'case_id': case['id'],
    'qualification': 'synthetic_controller_limiter',
    'comparison': comparison,
    'findings': findings,
    'scope': evidence_scope(case['origin']),
    'source_identities': source_identities,
    'input_sha256': input_sha256,
    'runtime': runtime,
    'limitations': limitations,
  }
