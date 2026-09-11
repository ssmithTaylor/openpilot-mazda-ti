import hashlib, json
from pathlib import Path
import pytest
from tools.mazda_ti.rider_review import build_html, request_hash, validate_annotation

def make(tmp_path):
  clip=tmp_path/'v.mp4'; clip.write_bytes(b'video'); video_hash=hashlib.sha256(b'video').hexdigest()
  manifest=tmp_path/'manifest.json'; manifest.write_text('{"source":"test"}\n')
  mp=tmp_path/'m.json'; mapping={"identity":{"route":"r","segment":"s"},"inputs":{"video":{"sha256":video_hash}},"events":{}}
  for name,mono,lo,hi in [('s',10,1,1.1),('e',11,2,2.1)]: mapping['events'][name]={"target_mono_s":mono,"before":{"generated_clip_pts_s":lo},"after":{"generated_clip_pts_s":hi}}
  mp.write_text(json.dumps(mapping))
  return {"format_version":1,"route":"r","segment":"s","manifest":{"path":str(manifest.resolve()),"sha256":hashlib.sha256(manifest.read_bytes()).hexdigest()},"video":{"path":str(clip.resolve()),"sha256":video_hash},"mapping":{"path":str(mp.resolve()),"sha256":hashlib.sha256(mp.read_bytes()).hexdigest()},"intervals":[{"id":"i","start_mono_s":10,"end_mono_s":11,"start_event":"s","end_event":"e"}]}
def ann(req, labels=None, **kw): return {"format_version":1,"request_sha256":request_hash(req),"manifest_sha256":req['manifest']['sha256'],"annotations":[{"id":"i","labels":['scallop'] if labels is None else labels,"confidence":"high","wheel_contact":"uncertain","rider_provenance":"rider",**kw}]}
def test_valid_multi_label_and_canonical(tmp_path): assert validate_annotation(make(tmp_path),ann(make(tmp_path),['scallop','held_inward']))['annotations'][0]['labels']==['scallop','held_inward']
@pytest.mark.parametrize('field', ['route','segment'])
def test_missing_identity(tmp_path,field):
  req=make(tmp_path); req.pop(field)
  with pytest.raises(ValueError): validate_annotation(req,ann(req))
def test_video_tamper(tmp_path):
  req=make(tmp_path); Path(req['video']['path']).write_bytes(b'bad')
  with pytest.raises(ValueError,match='video'): validate_annotation(req,ann(req))
def test_mapping_tamper_and_refreshed_target(tmp_path):
  req=make(tmp_path); p=Path(req['mapping']['path']); d=json.loads(p.read_text()); d['events']['e']['target_mono_s']=12; p.write_text(json.dumps(d)); req['mapping']['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
  with pytest.raises(ValueError,match='timing'): validate_annotation(req,ann(req))
def test_missing_or_reversed_pts(tmp_path):
  req=make(tmp_path); p=Path(req['mapping']['path']); d=json.loads(p.read_text()); d['events']['s']['before']['generated_clip_pts_s']=-1; p.write_text(json.dumps(d)); req['mapping']['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
  with pytest.raises(ValueError): validate_annotation(req,ann(req))
  req=make(tmp_path); p=Path(req['mapping']['path']); d=json.loads(p.read_text())
  d['events']['s']['after']['generated_clip_pts_s']=2.05
  p.write_text(json.dumps(d)); req['mapping']['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
  with pytest.raises(ValueError,match='chronology'): validate_annotation(req,ann(req))
def test_overlap_duplicate_and_unknown(tmp_path):
  req=make(tmp_path); req['intervals'].append(dict(req['intervals'][0]))
  with pytest.raises(ValueError,match='unique'): validate_annotation(req,ann(req))
  req=make(tmp_path); req['intervals'].append(dict(req['intervals'][0],id='x'))
  with pytest.raises(ValueError,match='overlap'): validate_annotation(req,ann(req))
  bad=ann(make(tmp_path),['scallop','scallop'])
  with pytest.raises(ValueError): validate_annotation(make(tmp_path),bad)
def test_exclusive_confidence_contact_provenance(tmp_path):
  req=make(tmp_path)
  for labels,kw in [([],{}),(['smooth','scallop'],{}),(['uncertain','scallop'],{}),(['scallop'],{'confidence':''}),(['scallop'],{'wheel_contact':''}),(['scallop'],{'rider_provenance':''})]:
    with pytest.raises(ValueError): validate_annotation(req,ann(req,labels,**kw))
  bad=ann(req); bad['annotations'][0]['note']={'not':'text'}
  with pytest.raises(ValueError,match='note'): validate_annotation(req,bad)
def test_intervention_pts(tmp_path):
  req=make(tmp_path)
  with pytest.raises(ValueError): validate_annotation(req,ann(req,['intervention']))
  assert validate_annotation(req,ann(req,['intervention'],intervention_clip_pts_s=1.5))
def test_request_manifest_and_ids(tmp_path):
  req=make(tmp_path); bad=ann(req); bad['request_sha256']='0'*64
  with pytest.raises(ValueError): validate_annotation(req,bad)
  bad=ann(req); bad['manifest_sha256']='b'*64
  with pytest.raises(ValueError): validate_annotation(req,bad)
  bad=ann(req); bad['format_version']=2
  with pytest.raises(ValueError,match='format_version'): validate_annotation(req,bad)
  bad=ann(req); bad['annotations'][0]['rider_provenance']=['not','text']
  with pytest.raises(ValueError,match='provenance'): validate_annotation(req,bad)
def test_two_outputs_and_functional_html(tmp_path):
  req=make(tmp_path); a=tmp_path/'a.html'; b=tmp_path/'b.html'; build_html(req,a); build_html(req,b)
  assert a.read_bytes()==b.read_bytes(); text=a.read_text(); assert all(x in text for x in ['data-id="i"','checkbox','data-confidence','data-contact','data-provenance','data-capture','file:///','start PTS','end PTS','annotations:annotations','const stable=','stable(value)+"\\n"','annotation-json','await navigator.clipboard.writeText','Clipboard unavailable'])
  assert 'JSON.stringify(value,Object.keys(value)' not in text
  with pytest.raises(FileExistsError): build_html(req,a)
