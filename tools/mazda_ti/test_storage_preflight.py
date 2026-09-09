import errno
import json
from pathlib import Path

import pytest

from . import storage_preflight as subject


def test_same_volume_counts_two_workspaces_and_one_headroom(tmp_path):
  report = subject.assess(tmp_path/'out'/'new', tmp_path/'cache'/'new', 100, 25,
                          probe=lambda ancestor: ('disk-a', 250))
  assert report['status'] == 'ready'
  assert report['volumes'] == [{'id':'volume-1','free_bytes':250,'required_bytes':225,
                                'workspace_copies':2,'sufficient':True}]


def test_same_volume_uses_conservative_free_reading(tmp_path):
  readings = iter([260, 240])
  report = subject.assess(tmp_path/'out', tmp_path/'cache', 100, 25,
                          probe=lambda ancestor: ('disk-a', next(readings)))
  assert report['volumes'][0]['free_bytes'] == 240
  assert report['volumes'][0]['sufficient'] is True


def test_different_volumes_each_require_workspace_and_headroom(tmp_path):
  def probe(ancestor):
    return ('output-disk', 130) if 'out' in str(ancestor) else ('cache-disk', 124)
  (tmp_path/'out').mkdir(); (tmp_path/'cache').mkdir()
  report = subject.assess(tmp_path/'out'/'new', tmp_path/'cache'/'new', 100, 25, probe=probe)
  assert report['status'] == 'insufficient_space'
  assert [v['required_bytes'] for v in report['volumes']] == [125,125]
  assert [v['sufficient'] for v in report['volumes']] == [True,False]


def test_nonexistent_descendants_probe_nearest_existing_ancestor(tmp_path):
  seen = []
  subject.assess(tmp_path/'a'/'b'/'out', tmp_path/'c'/'cache', 1, 0,
                 probe=lambda ancestor: (seen.append(ancestor) or ('disk',10)))
  assert seen == [tmp_path, tmp_path]
  assert not (tmp_path/'a').exists() and not (tmp_path/'c').exists()


@pytest.mark.parametrize('workspace,headroom', [(-1,0),(1,-1),(True,0),(1,1.5)])
def test_invalid_sizes_are_rejected(tmp_path, workspace, headroom):
  with pytest.raises(ValueError):
    subject.assess(tmp_path/'out', tmp_path/'cache', workspace, headroom)


def test_relative_identical_file_and_missing_volume_paths_are_rejected(tmp_path, monkeypatch):
  monkeypatch.chdir(tmp_path)
  with pytest.raises(ValueError): subject.assess(Path('out'), tmp_path/'cache', 1, 0)
  with pytest.raises(ValueError): subject.assess(tmp_path/'same', tmp_path/'same', 1, 0)
  with pytest.raises(ValueError): subject.assess(tmp_path/'out', tmp_path/'out'/'cache', 1, 0)
  file = tmp_path/'file'; file.write_text('x')
  with pytest.raises(ValueError): subject.assess(file, tmp_path/'cache', 1, 0)
  with pytest.raises(ValueError): subject.assess(tmp_path/'out', Path('Z:/missing/root'), 1, 0)


@pytest.mark.skipif(subject.os.name != 'nt', reason='Windows path grammar')
@pytest.mark.parametrize('name', ['bad:name','bad*name','CON','aux.txt','trailing.'])
def test_invalid_windows_components_are_rejected(tmp_path, name):
  with pytest.raises(ValueError): subject.assess(tmp_path/name, tmp_path/'cache', 1, 0)


def test_cli_reports_low_space_without_creating_targets(tmp_path, capsys):
  out, cache, report = tmp_path/'out', tmp_path/'cache', tmp_path/'report.json'
  code = subject.main(['--output',str(out),'--cache',str(cache),'--required-workspace-bytes','100',
                       '--headroom-bytes','999999999999999999','--report',str(report)])
  assert code == 1 and json.loads(report.read_text())['status'] == 'insufficient_space'
  assert not out.exists() and not cache.exists()
  assert 'insufficient_space' in capsys.readouterr().err


def test_report_cannot_be_created_inside_target(tmp_path, capsys):
  out = tmp_path/'out'; out.mkdir()
  code = subject.main(['--output',str(out),'--cache',str(tmp_path/'cache'),
                       '--required-workspace-bytes','1','--headroom-bytes','0','--report',str(out/'report.json')])
  assert code == 2 and not (out/'report.json').exists()
  assert 'outside output and cache' in capsys.readouterr().err


def test_reporting_enospc_uses_stderr_and_nonzero(tmp_path, monkeypatch, capsys):
  monkeypatch.setattr(subject, '_write_report', lambda *args: (_ for _ in ()).throw(OSError(errno.ENOSPC,'full')))
  code = subject.main(['--output',str(tmp_path/'out'),'--cache',str(tmp_path/'cache'),
                       '--required-workspace-bytes','1','--headroom-bytes','0','--report',str(tmp_path/'r')])
  assert code == 2
  assert '[Errno 28]' in capsys.readouterr().err
