"""FrogPilot's own background activity stays quiet while driving (owner, 2026-11-26 item 5: "Quietening FrogPilot's own background
activity while driving. Its updater writes as much log text as our planner and keeps connections to GitHub open."). Every
function still runs parked. The three processes cannot be imported on a PC (psutil / jwt / the compiled MPC), so the functions
under test are taken from the source with ast and run against fakes: the code tested is the code shipped."""
import ast
import datetime
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

REPO = Path(__file__).resolve().parents[2]


def _load(rel, names, env):
  """The named top-level functions of REPO/rel, compiled with globals `env` (fakes for everything they use)."""
  tree = ast.parse((REPO / rel).read_text(encoding="utf-8"))
  body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
  assert {n.name for n in body} == set(names), (rel, names)
  exec(compile(ast.Module(body=body, type_ignores=[]), str(REPO / rel), "exec"), env)
  return env


# ---- system/updated/updated.py ----------------------------------------------------------------------------------------------------
class _Stop(BaseException):
  """Ends the updater's endless loop (a BaseException: the loop's own `except Exception` must not swallow it)."""


class _Params:
  def __init__(self, onroad):
    self.d = {"IsOnroad": onroad, "IsOffroad": not onroad}

  def get_bool(self, k):
    return bool(self.d.get(k, False))

  def get(self, k, encoding=None):
    return self.d.get(k)

  def put(self, k, v):
    self.d[k] = v

  def put_bool(self, k, v):
    self.d[k] = v

  def remove(self, k):
    self.d.pop(k, None)


def _run_updated(tmp_path, onroad, manual=False, loops=4, time_valid=True):
  calls = []
  params = _Params(onroad)
  mem = _Params(onroad)
  mem.d["ManualUpdateInitiated"] = manual

  class Updater:
    def set_params(self, *a):
      calls.append("set_params")

    def check_for_update(self):
      calls.append("check")

    def fetch_update(self):
      calls.append("fetch")

  class Wait:
    def __init__(self):
      self.n, self.user_request = 0, 0
      self.ready_event = NS(clear=lambda: None)

    def sleep(self, t):
      calls.append(("sleep", t))
      self.n += 1
      if self.n >= loops:
        raise _Stop

  env = {
    "Params": lambda: params, "LOCK_FILE": str(tmp_path / "lock"), "fcntl": NS(flock=lambda *a: None, LOCK_EX=2, LOCK_NB=4),
    "psutil": NS(Process=lambda: NS(), LINUX=False), "Path": Path, "os": __import__("os"), "STAGING_ROOT": str(tmp_path / "st"),
    "cloudlog": NS(event=lambda *a, **k: calls.append(("event", a)), info=lambda *a, **k: calls.append(("info", a[0])),
                   exception=lambda *a, **k: calls.append(("exception", a))),
    "Updater": Updater, "WaitTimeHelper": Wait, "set_consistent_flag": lambda v: None,
    "get_frogpilot_toggles": lambda: NS(automatic_updates=True), "params_memory": mem,
    "init_overlay": lambda: calls.append("init_overlay"), "system_time_valid": lambda: time_valid, "datetime": datetime,
    "ZoneInfo": lambda z: datetime.timezone.utc, "read_time_from_param": lambda p, k: None, "write_time_to_param": lambda p, k: None,
    "UserRequest": NS(NONE=0, CHECK=1, FETCH=2), "subprocess": __import__("subprocess"), "OVERLAY_INIT": tmp_path / ".overlay_init",
  }
  _load("system/updated/updated.py", ["main"], env)
  with pytest.raises(_Stop):
    env["main"]()
  return calls


def test_updater_onroad_does_no_overlay_rebuild_no_git_and_no_params(tmp_path):
  """On-road (2026-11-25 drive: once a minute, as the clock is not valid on the road) the updater rebuilt its overlay (umount,
  rm -rf, mount), ran git diff, wrote it to a param and logged it whole: 400 kB of log text a minute. On-road it now only waits."""
  for time_valid in (True, False):
    calls = _run_updated(tmp_path, onroad=True, time_valid=time_valid)
    assert "init_overlay" not in calls and "set_params" not in calls and "check" not in calls and "fetch" not in calls, calls
    assert all(c == ("sleep", 60) for c in calls if isinstance(c, tuple) and c[0] == "sleep")
    assert not [c for c in calls if isinstance(c, tuple) and c[0] == "info"], calls     # and logs nothing


def test_updater_offroad_still_checks_and_fetches(tmp_path):
  calls = _run_updated(tmp_path, onroad=False)
  assert calls.count("init_overlay") >= 2 and "set_params" in calls and "check" in calls and "fetch" in calls, calls


def test_updater_onroad_manual_request_still_runs(tmp_path):
  """A request from the settings screen ("check for update") is honoured whatever the state, as before."""
  calls = _run_updated(tmp_path, onroad=True, manual=True)
  assert "init_overlay" in calls
