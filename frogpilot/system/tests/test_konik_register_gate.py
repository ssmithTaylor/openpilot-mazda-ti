"""Re-review R2-I1: Konik (and comma) registration with the owner's /data/no_remote_access flag, and its bound without it.

Before: boot_thread called register(register_konik=True) as soon as the clock was valid (with the GPS clock fix: mid-drive),
before the off-road wait, ungated, and register() never gave up without a spinner (max_wait 60 fails its own "< 60" test), so
with no network it logged a traceback every 1-15 s for ever and the backups behind it never ran.
Now: flag present -> no registration request from the boot path, the boot thread or manager; the backups run. Flag absent ->
one bounded attempt after the off-road wait, one failure logged, then the backups. The modules import the compiled stack, so
the functions are taken from their source with ast and run against fakes (the code tested is the code shipped); nothing
leaves this PC."""
import ast
import pathlib
from types import SimpleNamespace as NS

import pytest

from openpilot.system.manager import remote_access as RA

ROOT = pathlib.Path(__file__).resolve().parents[3]
UNREG = "UnregisteredDevice"


@pytest.fixture
def flag_path(tmp_path, monkeypatch):
  path = tmp_path / "no_remote_access"
  monkeypatch.setattr(RA, "NO_REMOTE_ACCESS_FLAG", str(path))
  monkeypatch.setattr(RA, "_logged", set())
  return path


def _load(rel, names, env):
  tree = ast.parse((ROOT / rel).read_text(encoding="utf-8"))
  keep = [n for n in tree.body if (isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names) or
          (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in n.targets))]
  assert {getattr(n, "name", None) or n.targets[0].id for n in keep} == set(names), names
  exec(compile(ast.Module(body=keep, type_ignores=[]), str(ROOT / rel), "exec"), env)
  return env


class Params:
  def __init__(self, d):
    self.d = dict(d)

  def get(self, k, encoding=None):
    return self.d.get(k)

  def put(self, k, v):
    self.d[k] = v

  def remove(self, k):
    self.d.pop(k, None)

  def get_bool(self, k):
    return bool(self.d.get(k, False))


def _boot(register, params, calls):
  """frogpilot_boot_functions with fakes; the boot thread runs synchronously to its end."""
  env = {
    "params": params, "register": register, "UNREGISTERED_DONGLE_ID": UNREG, "remote_access_blocked": RA.remote_access_blocked,
    "use_konik_server": lambda: True, "system_time_valid": lambda: True, "time": NS(sleep=lambda s: None),
    "FrogPilotVariables": lambda: NS(update=lambda **k: None), "ModelManager": lambda boot_run: None,
    "ThemeManager": lambda boot_run: NS(update_active_theme=lambda **k: None), "get_frogpilot_toggles": lambda: None,
    "VIDEO_CACHE_PATH": NS(exists=lambda: False), "delete_file": lambda p: None,
    "wait_until_offroad": lambda p, **k: calls.append("wait_until_offroad"),
    "backup_frogpilot": lambda b: calls.append("backup_frogpilot"), "backup_toggles": lambda c: calls.append("backup_toggles"),
    "send_stats": lambda: calls.append("send_stats"),
    "threading": NS(Thread=lambda target, daemon=True: NS(start=target)), "print": lambda *a: calls.append(("print",) + a),
  }
  _load("frogpilot/common/frogpilot_functions.py",
        {"KONIK_LATE_REGISTER_MAX_WAIT_S", "late_konik_registration", "frogpilot_boot_functions"}, env)
  env["frogpilot_boot_functions"](NS(), NS(clear_all=lambda: None))
  return env


def test_flag_present_no_registration_at_all_and_backups_run(flag_path):
  flag_path.write_text("")
  regs, calls = [], []
  params = Params({"StockDongleId": "stock0123456789a"})       # Konik switch on, no Konik id stored (the reviewer's case)
  _boot(lambda **k: regs.append(k) or UNREG, params, calls)
  assert regs == []
  assert [c for c in calls if isinstance(c, str)] == ["wait_until_offroad", "backup_frogpilot", "backup_toggles", "send_stats"]
  assert params.d.get("DongleId") == "stock0123456789a"         # identity unchanged: the stock id, as on an offline boot


def test_flag_absent_register_failing_is_bounded_and_backups_run(flag_path):
  regs, calls = [], []

  def register(**k):
    regs.append(k)
    calls.append("register")
    return UNREG                                                 # what a bounded register() returns when it gives up
  params = Params({"StockDongleId": "stock0123456789a"})
  env = _boot(register, params, calls)
  assert len(regs) == 2                                          # the boot-path quick try and one late attempt, no more
  late = regs[1]
  assert late["register_konik"] and not late["show_spinner"] and late["max_wait"] < 60 and late["log_each_failure"] is False
  assert late["max_wait"] == env["KONIK_LATE_REGISTER_MAX_WAIT_S"]
  names = [c for c in calls if isinstance(c, str)]
  assert names == ["register", "wait_until_offroad", "register", "backup_frogpilot", "backup_toggles", "send_stats"]
  assert sum(1 for c in calls if isinstance(c, tuple) and "gave up" in c[1]) == 1
  assert "KonikDongleId" not in params.d and params.d["DongleId"] == "stock0123456789a"


def test_flag_absent_register_succeeding_stores_the_ids(flag_path):
  regs, calls = [], []
  answers = iter([UNREG, "konik0123456789a"])                    # the quick try fails offline, the late one succeeds
  params = Params({"StockDongleId": "stock0123456789a"})
  _boot(lambda **k: regs.append(k) or next(answers), params, calls)
  assert params.d["KonikDongleId"] == "konik0123456789a" and params.d["DongleId"] == "konik0123456789a"


def test_flag_absent_quick_try_succeeding_needs_no_late_attempt(flag_path):
  regs, calls = [], []
  params = Params({})
  _boot(lambda **k: regs.append(k) or "konik0123456789a", params, calls)
  assert len(regs) == 1 and params.d["KonikDongleId"] == "konik0123456789a" and params.d["DongleId"] == "konik0123456789a"


# ---- system/athena/registration.py: the bound itself, and allow_network=False -----------------------------------------------------
class Clock:
  def __init__(self):
    self.t = 0.0

  def monotonic(self):
    return self.t

  def sleep(self, s):
    self.t += s


def _register_env(params, clock, posts, logs, tmp_path, fail=True):
  persist = tmp_path / "persist"
  (persist / "comma").mkdir(parents=True)
  (persist / "comma" / "id_rsa.pub").write_text("pub")
  (persist / "comma" / "id_rsa").write_text("priv")

  def api_get(*a, **k):
    posts.append(k.get("serial"))
    clock.t += 0.01
    if fail:
      raise ConnectionError("no network")
    return NS(status_code=200, text='{"dongle_id": "konik0123456789a"}')
  env = {
    "Params": lambda: params, "Path": pathlib.Path, "Paths": NS(persist_root=lambda: str(persist)), "UNREGISTERED_DONGLE_ID": UNREG,
    "HARDWARE": NS(get_serial=lambda: "serial0", get_imei=lambda i: f"imei{i}"), "time": clock, "api_get": api_get,
    "jwt": NS(encode=lambda *a, **k: "tok"), "json": __import__("json"), "random": __import__("random"),
    "string": __import__("string"), "datetime": __import__("datetime").datetime, "timedelta": __import__("datetime").timedelta,
    "Spinner": lambda: NS(update=lambda s: None, close=lambda: None),
    "cloudlog": NS(exception=lambda m: logs.append(("exception", m)), info=lambda m: None, warning=lambda m: logs.append(("w", m)),
                   event=lambda *a, **k: logs.append(("event", a, k))),
  }
  return _load("system/athena/registration.py", {"register"}, env)["register"]


def test_register_konik_without_spinner_gives_up_within_the_bound_and_logs_one_failure(tmp_path):
  params, clock, posts, logs = Params({"DongleId": "stock0123456789a", "IMEI": "i", "HardwareSerial": "s"}), Clock(), [], []
  register = _register_env(params, clock, posts, logs, tmp_path)
  assert register(show_spinner=False, register_konik=True, max_wait=59, log_each_failure=False) == UNREG
  assert clock.t < 59 + 16 and 1 < len(posts) < 20
  assert [entry for entry in logs if entry[0] == "exception"] == [("exception", "failed to authenticate")]
  assert params.d["DongleId"] == "stock0123456789a"            # a Konik attempt never rewrites DongleId inside register()


def test_register_with_network_not_allowed_sends_nothing(tmp_path):
  for start in ({"IMEI": "i", "HardwareSerial": "s"}, {"DongleId": UNREG, "IMEI": "i", "HardwareSerial": "s"}):
    params, clock, posts, logs = Params(start), Clock(), [], []
    register = _register_env(params, clock, posts, logs, tmp_path / str(len(start)), fail=False)
    assert register(show_spinner=True, allow_network=False) == UNREG and posts == []
    assert params.d["DongleId"] == UNREG                       # as an offline registration leaves it: manager then skips athenad
    assert register(show_spinner=False, register_konik=True, allow_network=False) == UNREG and posts == []


def test_register_registered_device_is_unchanged_by_the_flag(tmp_path):
  params, clock, posts, logs = Params({"DongleId": "stock0123456789a", "IMEI": "i", "HardwareSerial": "s"}), Clock(), [], []
  register = _register_env(params, clock, posts, logs, tmp_path, fail=False)
  assert register(show_spinner=True, allow_network=False) == "stock0123456789a" and posts == []
  assert register(show_spinner=True) == "stock0123456789a" and posts == []


def test_manager_passes_the_flag_to_register():
  tree = ast.parse((ROOT / "system/manager/manager.py").read_text(encoding="utf-8"))
  calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", None) == "register"]
  assert len(calls) == 1
  kw = {k.arg: ast.unparse(k.value) for k in calls[0].keywords}
  assert kw.get("allow_network") == "not remote_access_off()"
