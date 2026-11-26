"""integ3 review T1-I2: FrogPilot's boot jobs (backups, stats) never start while the car is on-road.

The boot thread waits for a valid clock; with the GPS clock fix that can happen mid-drive. wait_until_offroad holds the
jobs until IsOnroad is false and logs when it defers and when it runs."""
import ast
from pathlib import Path

from openpilot.frogpilot.common.boot_jobs import OFFROAD_POLL_S, wait_until_offroad

FUNCS = Path(__file__).resolve().parents[2] / "common" / "frogpilot_functions.py"


class FakeParams:
  def __init__(self, onroad_reads):
    self.seq = list(onroad_reads)
    self.reads = 0

  def get_bool(self, key):
    assert key == "IsOnroad"
    self.reads += 1
    return self.seq.pop(0) if self.seq else False


def _run(onroad_reads):
  clock = {"t": 100.0}
  sleeps, lines = [], []

  def sleep(s):
    sleeps.append(s)
    clock["t"] += s
  waited = wait_until_offroad(FakeParams(onroad_reads), sleep=sleep, monotonic=lambda: clock["t"], log=lines.append)
  return waited, sleeps, lines


def test_off_road_runs_at_once_without_sleeping():
  waited, sleeps, lines = _run([False])
  assert waited == 0.0 and sleeps == []
  assert lines == ["boot jobs: running (deferred 0.0 s)"]


def test_on_road_defers_until_off_road_polling_slowly_and_logs_once():
  waited, sleeps, lines = _run([True] * 30 + [False])     # 30 polls on the road (5 min), then parked
  assert sleeps == [OFFROAD_POLL_S] * 30 and OFFROAD_POLL_S >= 5.0
  assert waited == 30 * OFFROAD_POLL_S
  assert len(lines) == 2 and lines[0].startswith("boot jobs: deferred") and lines[1] == f"boot jobs: running (deferred {waited} s)"


def test_default_log_goes_to_the_drive_log(monkeypatch, capsys):
  """re-review R2-M6: the decision lines reach cloudlog (the drive log), not only manager's stdout."""
  import inspect
  from openpilot.frogpilot.common import boot_jobs
  events = []
  import openpilot.common.swaglog as swaglog
  monkeypatch.setattr(swaglog.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  assert inspect.signature(wait_until_offroad).parameters["log"].default is boot_jobs.log_line
  boot_jobs.log_line("boot jobs: running (deferred 0.0 s)")
  assert events == [("boot_jobs", {"msg": "boot jobs: running (deferred 0.0 s)"})]
  assert capsys.readouterr().out.strip() == "boot jobs: running (deferred 0.0 s)"


def _calls_in_order(fn):
  names = []
  for node in ast.walk(fn):
    if isinstance(node, ast.Call):
      f = node.func
      names.append((node.lineno, node.col_offset, f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", None)))
  return [n for _, _, n in sorted(names)]


def test_boot_thread_waits_for_off_road_before_backups_and_stats():
  tree = ast.parse(FUNCS.read_text(encoding="utf-8"))
  boot = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "boot_thread")
  calls = _calls_in_order(boot)
  assert "wait_until_offroad" in calls
  w = calls.index("wait_until_offroad")
  assert calls.index("system_time_valid") < w
  for job in ("backup_frogpilot", "backup_toggles", "send_stats"):
    assert w < calls.index(job), job
