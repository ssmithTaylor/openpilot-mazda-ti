"""The install tools (review I3, 2026-11-24): tools/dev/comma_checks.sh is TRACKED (.gitignore's comma*.sh rule hid it) and is what
install_comma.sh runs at stage 6; install_comma.sh runs its pre-flight (every script it will run later is in the commit) BEFORE it
pushes anything. Read-only git queries; nothing is pushed or installed."""
import os
import pathlib
import re
import shutil
import subprocess

import pytest

HERE = pathlib.Path(__file__).resolve().parent


def _git(*a):
  return subprocess.run(["git", *a], cwd=HERE, capture_output=True, text=True, check=False)


def _root():
  r = _git("rev-parse", "--show-toplevel")
  if r.returncode != 0:
    pytest.skip("not inside a git checkout")
  return pathlib.Path(r.stdout.strip())


def _bash():
  for c in (r"C:\Program Files\Git\bin\bash.exe", os.path.expanduser(r"~\scoop\apps\git\current\bin\bash.exe")):
    if os.path.exists(c):
      return c
  b = shutil.which("bash")
  if b is None or b.lower().endswith(r"system32\bash.exe"):
    pytest.skip("no Git Bash")
  return b


def test_comma_checks_is_tracked_and_run_by_the_install():
  root = _root()
  assert _git("-C", str(root), "ls-files", "--error-unmatch", "tools/dev/comma_checks.sh").returncode == 0
  install = (root / "tools/dev/install_comma.sh").read_text(encoding="utf-8")
  assert "bash tools/dev/comma_checks.sh" in install


def test_comma_checks_has_no_addresses_keys_or_credentials():
  txt = (_root() / "tools/dev/comma_checks.sh").read_text(encoding="utf-8")
  assert not re.search(r"\b\d{1,3}(\.\d{1,3}){3}\b", txt)           # no IP address
  assert not re.search(r"(?i)(ssh|\.pem|id_rsa|_key\b|password|token|secret|@[a-z0-9.-]+)", txt)


def test_preflight_runs_before_any_push():
  install = (_root() / "tools/dev/install_comma.sh").read_text(encoding="utf-8")
  pre = install.index("bash tools/dev/install_preflight.sh")
  assert pre < install.index("git push") and pre < install.index('remote "cd $REMOTE_DIR')


def test_preflight_lists_every_script_the_install_runs():
  root = _root()
  pre = (root / "tools/dev/install_preflight.sh").read_text(encoding="utf-8")
  checks = (root / "tools/dev/comma_checks.sh").read_text(encoding="utf-8")
  for f in re.findall(r"python3 ([\w/.-]+\.py)", checks) + ["tools/dev/pc_test.py", "tools/dev/comma_checks.sh"]:
    assert f in pre, f


def test_preflight_passes_on_head_and_fails_without_comma_checks():
  root = _root()
  bash = _bash()
  ok = subprocess.run([bash, "tools/dev/install_preflight.sh", "HEAD"], cwd=root, capture_output=True, text=True)
  assert ok.returncode == 0, ok.stdout + ok.stderr
  # the branch head the review found (comma_checks.sh hidden by .gitignore): refused before anything is pushed
  if _git("cat-file", "-e", "bea958a3^{commit}").returncode == 0:
    bad = subprocess.run([bash, "tools/dev/install_preflight.sh", "bea958a3"], cwd=root, capture_output=True, text=True)
    assert bad.returncode == 1 and "comma_checks.sh" in bad.stdout
