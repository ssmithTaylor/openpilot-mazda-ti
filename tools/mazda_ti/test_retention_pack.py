import gzip
import json
from pathlib import Path

import pytest
from . import retention_pack as subject


def generated(tmp_path):
  study=tmp_path/'study'; run=study/'run'; attempt=run/'cases'/'case-a'/'attempt-001'
  attempt.mkdir(parents=True); (run/'.mazda-generated-run.json').write_text('{}')
  (attempt/'request.json').write_text('{"a":1}')
  (attempt/'result.json').write_text('{"status":"completed_checks"}')
  (attempt/'trace.jsonl').write_text('bulk\n')
  return study,run


def test_pack_is_deterministic_and_inventory_covers_bulk(tmp_path):
  study,run=generated(tmp_path); a=tmp_path/'a.json.gz'; b=tmp_path/'b.json.gz'
  subject.pack(study,run,a,[]); subject.pack(study,run,b,[])
  assert a.read_bytes()==b.read_bytes()
  body=json.loads(gzip.decompress(a.read_bytes()))
  assert [x['path'] for x in body['inventory']]==['.mazda-generated-run.json','cases/case-a/attempt-001/request.json','cases/case-a/attempt-001/result.json','cases/case-a/attempt-001/trace.jsonl']
  assert 'cases/case-a/attempt-001/trace.jsonl' not in body['retained_files']
  assert body['qualification']=='summary_only_requires_full_bundle_rebuild'


def test_rejects_raw_overlap_outside_and_reparse(tmp_path):
  study,run=generated(tmp_path)
  with pytest.raises(ValueError): subject.pack(study,run,tmp_path/'x.gz',[run/'cases'])
  (run/'rlog.bz2').write_bytes(b'raw')
  with pytest.raises(ValueError): subject.pack(study,run,tmp_path/'x.gz',[])


def test_external_protected_root_is_allowed(tmp_path):
  study,run=generated(tmp_path); external=tmp_path/'raw'; external.mkdir()
  subject.pack(study,run,tmp_path/'pack.gz',[external])


def test_unmarked_unrecognized_root_is_rejected(tmp_path):
  study=tmp_path/'study'; run=study/'run'; run.mkdir(parents=True)
  with pytest.raises(ValueError): subject.pack(study,run,tmp_path/'x.gz',[])


def test_verify_cleanup_rejects_changed_source_and_pack(tmp_path):
  study,run=generated(tmp_path); pack=tmp_path/'pack.gz'; plan=tmp_path/'plan.json'
  subject.pack(study,run,pack,[]); subject.verify_cleanup(study,run,pack,plan,[])
  assert json.loads(plan.read_text())['status']=='verified_for_explicit_external_deletion'
  (run/'cases/case-a/attempt-001/trace.jsonl').write_text('changed')
  with pytest.raises(ValueError): subject.verify_cleanup(study,run,pack,tmp_path/'bad.json',[])


def test_pack_and_manifest_must_be_outside_run_and_tampering_is_rejected(tmp_path):
  study,run=generated(tmp_path)
  with pytest.raises(ValueError): subject.pack(study,run,run/'pack.gz',[])
  pack=tmp_path/'pack.gz'; subject.pack(study,run,pack,[])
  body=json.loads(gzip.decompress(pack.read_bytes())); body['limitations']=[]
  with gzip.GzipFile(filename='',mode='wb',fileobj=pack.open('wb'),mtime=0) as stream:
    stream.write(json.dumps(body,sort_keys=True,separators=(',',':')).encode())
  with pytest.raises(ValueError): subject.verify_cleanup(study,run,pack,tmp_path/'plan.json',[])


def test_large_retained_metadata_is_inventory_only_with_reason(tmp_path):
  study,run=generated(tmp_path); report=run/'cases/case-a/attempt-001/report.md'; report.write_bytes(b'x'*20)
  pack=tmp_path/'pack.gz'; subject.pack(study,run,pack,[],max_file_bytes=10,max_total_bytes=100)
  body=json.loads(gzip.decompress(pack.read_bytes()))
  assert body['omitted_retained_metadata'][report.relative_to(run).as_posix()]=='exceeds_per_file_budget'
  assert next(x for x in body['inventory'] if x['path']==report.relative_to(run).as_posix())['size_bytes']==20


def test_run_root_and_nested_ancestor_links_are_rejected(tmp_path):
  study,real=generated(tmp_path); link=study/'linked-run'
  try: link.symlink_to(real,target_is_directory=True)
  except OSError: pytest.skip('symlink unavailable')
  with pytest.raises(ValueError): subject.pack(study,link,tmp_path/'a.gz',[])
  outer=study/'outer'; outer.mkdir(); nested=outer/'linked'; nested.symlink_to(real,target_is_directory=True)
  with pytest.raises(ValueError): subject.pack(study,nested,tmp_path/'b.gz',[])


def test_verify_cleanup_rejects_added_and_deleted_files(tmp_path):
  study,run=generated(tmp_path); pack=tmp_path/'pack.gz'; subject.pack(study,run,pack,[])
  added=run/'new.txt'; added.write_text('new')
  with pytest.raises(ValueError): subject.verify_cleanup(study,run,pack,tmp_path/'add.json',[])
  added.unlink(); (run/'cases/case-a/attempt-001/trace.jsonl').unlink()
  with pytest.raises(ValueError): subject.verify_cleanup(study,run,pack,tmp_path/'delete.json',[])


def test_dot_dot_run_escape_is_rejected(tmp_path):
  study, _ = generated(tmp_path)
  outside = tmp_path / 'outside'
  (outside / 'cases' / 'case-a' / 'attempt-001').mkdir(parents=True)
  (outside / '.mazda-generated-run.json').write_text('{}')
  escaped = study / 'child' / '..' / '..' / 'outside'
  with pytest.raises(ValueError):
    subject.pack(study, escaped, tmp_path / 'escape.gz', [])


def test_relative_pack_and_manifest_paths_are_rejected(tmp_path):
  study, run = generated(tmp_path)
  with pytest.raises(ValueError):
    subject.pack(study, run, Path('relative.gz'), [])
  pack = tmp_path / 'pack.gz'
  subject.pack(study, run, pack, [])
  with pytest.raises(ValueError):
    subject.verify_cleanup(study, run, Path('relative.gz'), tmp_path / 'plan.json', [])
  with pytest.raises(ValueError):
    subject.verify_cleanup(study, run, pack, Path('relative.json'), [])


def test_requests_are_globally_prioritized_over_nested_results(tmp_path):
  study, run = generated(tmp_path)
  early = run / 'cases' / 'case-a' / 'attempt-001'
  nested = early / 'candidate'
  nested.mkdir()
  (nested / 'result.json').write_bytes(b'r' * 20)
  later = run / 'cases' / 'case-z' / 'attempt-001'
  later.mkdir(parents=True)
  (later / 'request.json').write_bytes(b'q' * 20)
  pack = tmp_path / 'priority.gz'
  subject.pack(study, run, pack, [], max_file_bytes=100, max_total_bytes=27)
  body = json.loads(gzip.decompress(pack.read_bytes()))
  assert 'cases/case-a/attempt-001/request.json' in body['retained_files']
  assert 'cases/case-z/attempt-001/request.json' in body['retained_files']
  assert body['omitted_retained_metadata']['cases/case-a/attempt-001/candidate/result.json'] == 'exceeds_total_budget'
