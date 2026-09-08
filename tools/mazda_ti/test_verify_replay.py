# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import pytest
from .verify_replay import EvidenceError, FIELDS, REQUIRED_SOURCES, verify
from .provenance import REPLAY_SOURCE_FILES


class TestEvidenceVerification:
  @pytest.fixture(autouse=True)
  def setup(self, request):
    temporary = tempfile.TemporaryDirectory()
    request.addfinalizer(temporary.cleanup)
    self.root = Path(temporary.name)
    sources = {}
    for name in REQUIRED_SOURCES:
      path = self.root / name
      path.parent.mkdir(parents=True, exist_ok=True)
      path.write_bytes(b'# fixture source\n')
      sources[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    self.lock = self.root / 'run-result.json'
    self.lock.write_text(json.dumps({'production_sha256': sources}))
    self.baseline, self.candidate = (self.root / 'baseline', self.root / 'candidate')
    row = {name: 0.0 for name in FIELDS}
    row.update(mono=10.0, active=True, lane_removed=[0.1, 0.1], lane_valid=True, lane_reason='valid', replay_plantState=3075)
    self.rows = [row, dict(row, mono=10.01)]
    self.sends = [{'mono': 10000000000, 'counts': -100, 'recorded_counts': -100}, {'mono': 10010000000, 'counts': -110, 'recorded_counts': -110}]
    self.write(self.baseline, self.rows, self.sends)
    changed = [dict(r, replay_plantState=r['replay_plantState'] + 4096) for r in self.rows]
    self.write(self.candidate, changed, self.sends)

  def write(self, prefix, rows, sends):
    Path(str(prefix) + '.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    Path(str(prefix) + '-sends.json').write_text(json.dumps({'sends': sends}))

  def verify(self):
    return verify(self.root, self.baseline, self.candidate, self.lock, 10.0, 10.02, 4096)

  def test_deterministic_report_and_intentional_flag(self):
    first = self.verify()
    second = self.verify()
    assert json.dumps(first, sort_keys=True) == json.dumps(second, sort_keys=True)
    assert first['control_frames'] == 2
    assert first['sends'] == 2
    assert str(self.root) not in json.dumps(first)

  def version2_lock(self):
    sources = {}
    for name in REPLAY_SOURCE_FILES:
      path = self.root / name
      path.parent.mkdir(parents=True, exist_ok=True)
      if not path.exists():
        path.write_bytes(b'# full dependency fixture\n')
      sources[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    Path(str(self.candidate) + '.json').write_bytes(b'{}')
    outputs = {'trace' + suffix: hashlib.sha256(Path(str(self.candidate) + suffix).read_bytes()).hexdigest() for suffix in ('.json', '.jsonl', '-sends.json')}
    self.lock.write_text(json.dumps(dict(format_version=2, repository_sources=sources, output_sha256=outputs)))

  def test_version2_checks_transitive_sources(self):
    self.version2_lock()
    assert self.verify()['control_frames'] == 2
    (self.root / 'selfdrive/controls/lib/pid.py').write_bytes(b'changed')
    with pytest.raises(EvidenceError, match='Source changed'):
      self.verify()

  def test_version2_rejects_changed_artifacts_even_outside_comparison_window(self):
    self.version2_lock()
    Path(str(self.candidate) + '.jsonl').write_text(Path(str(self.candidate) + '.jsonl').read_text() + '\n')
    with pytest.raises(EvidenceError, match='Candidate artifact changed'):
      self.verify()

  def test_source_change_rejects_old_evidence(self):
    (self.root / 'selfdrive/car/mazda/lateral_reference.py').write_text('# different source\n')
    with pytest.raises(EvidenceError, match='Source changed'):
      self.verify()

  def test_changed_command_rejected(self):
    sends = copy.deepcopy(self.sends)
    sends[0]['counts'] += 1
    Path(str(self.candidate) + '-sends.json').write_text(json.dumps({'sends': sends}))
    with pytest.raises(EvidenceError, match='Integer sends'):
      self.verify()

  def test_changed_controller_output_rejected_even_if_clipped_send_matches(self):
    rows = [dict(r, replay_plantState=r['replay_plantState'] + 4096) for r in self.rows]
    rows[0]['replay_output'] = 0.001
    self.write(self.candidate, rows, self.sends)
    with pytest.raises(EvidenceError, match='replay_output'):
      self.verify()

  def test_missing_and_duplicate_frames_rejected(self):
    changed = [dict(r, replay_plantState=r['replay_plantState'] + 4096) for r in self.rows]
    for rows in (changed[:1], [changed[0], changed[0]]):
      self.write(self.candidate, rows, self.sends)
      with pytest.raises(EvidenceError):
        self.verify()

  def test_unearned_flag_rejected(self):
    rows = [dict(r, lane_removed=[0.0, 0.0]) for r in self.rows]
    self.write(self.baseline, rows, self.sends)
    self.write(self.candidate, [dict(r, replay_plantState=r['replay_plantState'] + 4096) for r in rows], self.sends)
    with pytest.raises(EvidenceError, match='Plant-state'):
      self.verify()

  def test_nonfinite_trace_rejected(self):
    rows = [dict(r, mono=float('nan')) for r in self.rows]
    self.write(self.candidate, rows, self.sends)
    with pytest.raises(EvidenceError, match='Non-finite'):
      self.verify()
