"""Versioned, serial mixed-corpus evaluation with explicit coverage and exposure."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile

from .evaluate import evaluate, verify_bundle
from .evaluation_contract import Unsupported, canonical_request
from .provenance import read_json, resolve_ref, sha256, under, write_json

CATEGORIES = {'difficult_corner', 'successful_corner', 'failed_corner', 'straight_gentle', 'actuator_transition', 'engagement_boundary'}
SCOPE = ('Per-case software and command checks on fixed observations; no prediction of physical cornering success. '
         'Rider annotations remain observations of the original drive. TI steering signals do not establish driver contact.')


def fields(value, keys):
  if not isinstance(value, dict) or set(value) != set(keys.split()):
    raise ValueError('Missing or unexpected corpus fields')


def label(value):
  if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', value):
    raise ValueError('Invalid corpus identity')
  return value


def words(value):
  if not isinstance(value, str) or not 0 < len(value) <= 1000 or any(ord(c) < 32 for c in value):
    raise ValueError('Invalid corpus description')


def manifest_read(path, candidate):
  m = read_json(path)
  fields(m, 'format_version id version cases sources unavailable profiles')
  if type(m['format_version']) is not int or m['format_version'] != 1:
    raise Unsupported('Unsupported corpus version')
  if not isinstance(m['cases'], list) or not isinstance(m['sources'], list) or not isinstance(m['profiles'], dict):
    raise ValueError('Invalid corpus collection types')
  label(m['id'])
  label(m['version'])
  sources = {}
  for source in m['sources']:
    fields(source, 'id path sha256 kind')
    label(source['id'])
    if source['id'] in sources or source['kind'] not in ('rider', 'analysis', 'baseline_failure'):
      raise ValueError('Duplicate or unsupported evidence source')
    if not isinstance(source['sha256'], str) or not re.fullmatch('[a-f0-9]{64}', source['sha256']):
      raise ValueError('Invalid evidence identity')
    under(Path('.'), source['path'])
    sources[source['id']] = source
  ids = set()
  for entry in m['cases']:
    fields(entry, 'case control_revision categories group exposures annotations expected exclusion')
    entry['case'] = canonical_request({'format_version': 1, 'candidate_revision': candidate, 'case': entry['case']})['case']
    case_id = label(entry['case']['id'])
    if case_id in ids:
      raise ValueError('Duplicate case identity')
    ids.add(case_id)
    spec = entry['case']['experiment']
    if any(not re.fullmatch('[a-f0-9]{40}', spec[k]) for k in ('baseline_controller', 'warmup_controller', 'limiter')):
      raise ValueError('Corpus baseline dependencies require full commit identities')
    if entry['control_revision'] != spec['baseline_controller']:
      raise Unsupported('Corpus v1 uses the original baseline as the control arm')
    if not isinstance(entry['categories'], list) or not set(entry['categories']) <= CATEGORIES:
      raise ValueError('Unsupported corpus category')
    if entry['group'] not in ('development', 'withheld') or not isinstance(entry['exposures'], list):
      raise ValueError('Invalid exposure declaration')
    for exposure in entry['exposures']:
      words(exposure)
    if entry['group'] == 'withheld' and entry['exposures']:
      raise ValueError('Previously examined cases must be development cases')
    fields(entry['expected'], 'minimum_sends stock_commands')
    if type(entry['expected']['minimum_sends']) is not int or entry['expected']['minimum_sends'] <= 0:
      raise ValueError('Expected coverage must be positive')
    if type(entry['expected']['stock_commands']) is not bool:
      raise ValueError('Invalid stock expectation')
    if not isinstance(entry['annotations'], list):
      raise ValueError('Annotations must be a list')
    for annotation in entry['annotations']:
      fields(annotation, 'kind value confidence source window')
      if annotation['kind'] not in ('intended_lane', 'crossing_direction', 'catch', 'contact', 'speed', 'entry', 'outcome'):
        raise ValueError('Unsupported annotation kind')
      words(annotation['value'])
      if annotation['source'] not in sources or annotation['confidence'] not in ('confirmed', 'approximate', 'uncertain'):
        raise ValueError('Annotation provenance is incomplete')
      if annotation['kind'] in ('catch', 'contact', 'outcome') and annotation['confidence'] == 'confirmed':
        if sources[annotation['source']]['kind'] != 'rider':
          raise ValueError('Confirmed contact/outcome requires rider evidence, not TI signals')
      window = annotation['window']
      if window is not None and (not isinstance(window, list) or len(window) != 2 or
                                 any(type(v) not in (int, float) for v in window) or not 0 <= window[0] <= window[1]):
        raise ValueError('Invalid annotation interval')
    for outcome in ('successful_corner', 'failed_corner'):
      if outcome in entry['categories'] and not any(a['kind'] == 'outcome' and a['value'] == outcome and
                                                   a['confidence'] == 'confirmed' for a in entry['annotations']):
        raise ValueError('Physical outcome category requires a confirmed rider annotation')
      if outcome in entry['categories'] and entry['case']['origin'] != 'recorded':
        raise ValueError('Synthetic fixtures cannot supply recorded physical outcomes')
    if entry['exclusion'] is not None:
      fields(entry['exclusion'], 'reason source')
      words(entry['exclusion']['reason'])
      if entry['exclusion']['source'] not in sources:
        raise ValueError('Exclusion requires retained evidence')
  if not ids:
    raise ValueError('Corpus must contain cases')
  if not isinstance(m['unavailable'], dict) or not set(m['unavailable']) <= CATEGORIES | {'withheld', 'matched_conditions'}:
    raise ValueError('Unknown unavailable category')
  for reason in m['unavailable'].values():
    words(reason)
  for name, profile in m['profiles'].items():
    label(name)
    fields(profile, 'required_cases required_categories require_unexposed_withheld')
    if not isinstance(profile['required_cases'], list) or not isinstance(profile['required_categories'], list):
      raise ValueError('Profile requirements must be lists')
    if not set(profile['required_cases']) <= ids or not set(profile['required_categories']) <= CATEGORIES:
      raise ValueError('Profile references unknown coverage')
    if not profile['required_cases'] or type(profile['require_unexposed_withheld']) is not bool:
      raise ValueError('Profile requires cases and an explicit withheld policy')
  return m


def evaluate_corpus(manifest_path, candidate, profile_name, data_root, evidence_root, ledger_path, output):
  output.mkdir(parents=True, exist_ok=False)
  result = {'format_version': 1, 'status': 'failed_execution', 'profile_pass': False, 'cases': [], 'findings': [], 'scope': SCOPE}
  try:
    if ledger_path.resolve().is_relative_to(output.resolve()):
      raise ValueError('Exposure ledger must persist outside the new output bundle')
    candidate = resolve_ref(candidate)
    m = manifest_read(manifest_path, candidate)
    if profile_name not in m['profiles']:
      raise Unsupported('Unknown corpus profile')
    ledger = read_json(ledger_path) if ledger_path.exists() else {'format_version': 1, 'evaluated_inputs': []}
    fields(ledger, 'format_version evaluated_inputs')
    if type(ledger['format_version']) is not int or ledger['format_version'] != 1 or not isinstance(ledger['evaluated_inputs'], list) or any(
        not isinstance(h, str) or not re.fullmatch('[a-f0-9]{64}', h) for h in ledger['evaluated_inputs']):
      raise ValueError('Invalid exposure ledger')
    prior = set(ledger['evaluated_inputs'])
    write_json(output / 'manifest.json', m)
    write_json(output / 'exposure-before.json', ledger)
    result.update(corpus_id=m['id'], corpus_version=m['version'], candidate_revision=candidate, profile=profile_name,
                  manifest_sha256=sha256(manifest_path), manifest_artifact_sha256=sha256(output / 'manifest.json'),
                  sources=m['sources'], unavailable=m['unavailable'])
    sources = {s['id']: s for s in m['sources']}
    (output / 'cases').mkdir()
    (output / 'requests').mkdir()
    covered, fresh_withheld = set(), False
    for entry in m['cases']:
      case = entry['case']
      inputs = set(case['input_sha256'].values())
      already_exposed = bool(inputs & prior) or bool(entry['exposures']) or entry['group'] == 'development'
      row = {'id': case['id'], 'method': case['method'], 'origin': case['origin'], 'categories': entry['categories'], 'group': entry['group'],
             'previously_exposed': already_exposed, 'exposed_by_run': True, 'annotations': entry['annotations'],
             'status': 'failed_check', 'comparison': None, 'findings': [], 'exclusion': entry['exclusion']}
      result['cases'].append(row)
      ledger['evaluated_inputs'] = sorted(set(ledger['evaluated_inputs']) | inputs)
      ledger_path.parent.mkdir(parents=True, exist_ok=True)
      with tempfile.NamedTemporaryFile(mode='w', dir=ledger_path.parent, delete=False, encoding='utf-8') as stream:
        stream.write(json.dumps(ledger, sort_keys=True, indent=2) + '\n')
      Path(stream.name).replace(ledger_path)  # Persist exposure even if this run subsequently fails.
      prior.update(inputs)
      used_sources = {a['source'] for a in entry['annotations']}
      if entry['exclusion']:
        used_sources.add(entry['exclusion']['source'])
      try:
        for name in sorted(used_sources):
          source = sources[name]
          if sha256(under(evidence_root, source['path'])) != source['sha256']:
            raise ValueError('Annotation or exclusion evidence identity changed')
        if entry['exclusion']:
          row.update(status='unsupported', findings=['Retained exclusion: ' + entry['exclusion']['reason']])
          continue
        request = output / 'requests' / (case['id'] + '.json')
        write_json(request, {'format_version': 1, 'candidate_revision': candidate, 'case': case})
        bundle = output / 'cases' / case['id']
        outcome = evaluate(request, data_root, bundle)
        row.update(status=outcome['status'], findings=outcome['findings'], comparison=outcome['comparison'],
                   result_sha256=sha256(bundle / 'result.json'), bundle='cases/' + case['id'])
        if outcome['status'] == 'completed_checks':
          verify_bundle(bundle, data_root)
          if (outcome['comparison']['sends'] < entry['expected']['minimum_sends'] or
              entry['expected']['stock_commands'] and 'stock' not in outcome['comparison']):
            raise ValueError('Required per-case evidence coverage is incomplete')
          covered.update(entry['categories'])
          fresh_withheld |= entry['group'] == 'withheld' and not already_exposed
      except (ValueError, OSError) as error:
        row.update(status='failed_check', comparison=None, findings=[str(error)])
    profile = m['profiles'][profile_name]
    by_id = {c['id']: c for c in result['cases']}
    missing = [name for name in profile['required_cases'] if by_id[name]['status'] != 'completed_checks']
    missing += sorted((set(profile['required_categories']) - covered) | (set(profile['required_categories']) & m['unavailable'].keys()))
    if profile['require_unexposed_withheld'] and (not fresh_withheld or 'withheld' in m['unavailable']):
      missing.append('unexposed withheld case')
    result.update(profile_pass=not missing, status='failed_check' if missing else 'completed_checks',
                  covered_categories=sorted(covered), findings=['Missing required coverage: ' + ', '.join(missing)] if missing else [])
    write_json(output / 'exposure-after.json', ledger)
    result['exposure_sha256'] = {name: sha256(output / name) for name in ('exposure-before.json', 'exposure-after.json')}
  except Unsupported as error:
    result.update(status='unsupported', findings=[str(error)])
  except (ValueError, KeyError, TypeError) as error:
    result.update(status='failed_check', findings=[str(error)])
  except (OSError, subprocess.SubprocessError) as error:
    result['findings'] = [str(error)]
  write_json(output / 'result.json', result)
  report = '# Corpus evaluation\n\n' + f"Status: **{result['status']}**. Required profile passed: {result['profile_pass']}.\n\n{SCOPE}\n"
  report += '\n| Case | Method / origin | Status | Prior exposure |\n| --- | --- | --- | --- |\n'
  for row in result['cases']:
    report += f"| {row['id']} | {row['method']} / {row['origin']} | {row['status']} | {row['previously_exposed']} |\n"
  for row in result['cases']:
    report += f"\n## {row['id']}\n\n" + '; '.join(row['findings']) + '\n'
    if 'bundle' in row:
      report += f"\n[Case evidence]({row['bundle']}/report.md)\n"
    for a in row['annotations']:
      report += f"\n- {a['kind']}: {a['value']} ({a['confidence']}; source {a['source']}; interval {a['window']}).\n"
  report += '\nUnavailable: ' + json.dumps(result.get('unavailable', {}), sort_keys=True) + '\n'
  report += '\n' + '; '.join(result['findings']) + '\n'
  (output / 'report.md').write_text(report, encoding='utf-8')
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  for name in ('manifest', 'data-root', 'evidence-root', 'exposure-ledger', 'output'):
    parser.add_argument('--' + name, type=Path, required=True)
  parser.add_argument('--candidate-revision', required=True)
  parser.add_argument('--profile', required=True)
  args = parser.parse_args()
  result = evaluate_corpus(args.manifest, args.candidate_revision, args.profile, args.data_root, args.evidence_root, args.exposure_ledger, args.output)
  print(result['status'] + ': ' + str(args.output / 'report.md'))
  return 0 if result['profile_pass'] else 1


if __name__ == '__main__':
  raise SystemExit(main())
