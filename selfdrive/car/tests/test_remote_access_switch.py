"""The owner's remote-access switch: a flag file keeps manager from starting athenad (owner, 2026-11-25)."""
import ast
import os

from openpilot.system.manager.remote_access import NO_REMOTE_ACCESS_FLAG, remote_access_ignores

HERE = os.path.dirname(os.path.abspath(__file__))
MANAGER = os.path.normpath(os.path.join(HERE, "..", "..", "..", "system", "manager", "manager.py"))
PROCESS_CONFIG = os.path.normpath(os.path.join(HERE, "..", "..", "..", "system", "manager", "process_config.py"))


def test_flag_present_blocks_the_daemon(tmp_path):
  flag = tmp_path / "no_remote_access"
  flag.write_text("")
  assert remote_access_ignores(str(flag)) == ["manage_athenad"]


def test_flag_absent_changes_nothing(tmp_path):
  assert remote_access_ignores(str(tmp_path / "no_remote_access")) == []


def test_a_directory_is_not_the_flag(tmp_path):
  d = tmp_path / "no_remote_access"
  d.mkdir()
  assert remote_access_ignores(str(d)) == []


def test_flag_lives_on_the_data_partition():
  assert NO_REMOTE_ACCESS_FLAG == "/data/no_remote_access"


def test_the_name_matches_the_daemon_in_process_config():
  with open(PROCESS_CONFIG, encoding="utf-8") as f:
    src = f.read()
  assert 'DaemonProcess("manage_athenad", "system.athena.manage_athenad", "AthenadPid")' in src


def test_manager_adds_the_ignores_before_processes_start():
  with open(MANAGER, encoding="utf-8") as f:
    src = f.read()
  tree = ast.parse(src)
  thread = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "manager_thread")
  body = ast.get_source_segment(src, thread)
  add = body.index("ignore += remote_access_ignores()")
  first_use = body.index("not_run=ignore")
  assert add < first_use
