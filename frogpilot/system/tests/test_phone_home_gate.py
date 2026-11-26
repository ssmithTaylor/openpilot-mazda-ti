"""The owner's remote-access flag (/data/no_remote_access) also stops FrogPilot's phone-home (integ3 review T1-I2).

With the flag present: no geocoder request with the last GPS position, no stats post, no device registration (not even
its reachability ping), no error report, no weather lookup (position + device id) and no theme submission to
FrogPilot's API. With the flag absent: the same calls go out as before. The modules import the compiled stack, so the
functions are taken from their source with ast and run against fakes (the code tested is the code shipped); no request
leaves this PC. One helper (system/manager/remote_access.remote_access_blocked) holds the flag; no call site repeats
its path."""
import ast
import datetime
import json
import pathlib
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest

from openpilot.system.manager import remote_access as RA

ROOT = pathlib.Path(__file__).resolve().parents[3]
API = "https://frogpilot.example/api"


@pytest.fixture(params=[True, False], ids=["flag_present", "flag_absent"])
def flag(request, tmp_path, monkeypatch):
  path = tmp_path / "no_remote_access"
  if request.param:
    path.write_text("")
  monkeypatch.setattr(RA, "NO_REMOTE_ACCESS_FLAG", str(path))
  monkeypatch.setattr(RA, "_logged", set())
  return request.param


class Net:
  """records every request; answers enough for the code paths to run"""
  def __init__(self):
    self.calls = []

  def _resp(self, url):
    if "nominatim" in url and url.endswith("/reverse"):
      body = {"address": {"city": "Testville", "state": "TS", "country": "Testland", "country_code": "tl"}}
    elif "nominatim" in url:
      body = []
    elif url.endswith("/register"):
      body = {"api_token": "t", "frogpilot_dongle_id": "fp0123456789"}
    else:
      body = {"sha": "abc"}
    return NS(status_code=200, text="", json=lambda: body, raise_for_status=lambda: None)

  def get(self, url, params=None, **k):
    self.calls.append(("GET", url, params))
    return self._resp(url)

  def post(self, url, json=None, **k):
    self.calls.append(("POST", url, json))
    return self._resp(url)

  def head(self, url, **k):
    self.calls.append(("HEAD", url, None))
    return self._resp(url)

  def Session(self):
    net = self

    class S:
      headers: dict = {}

      def __enter__(self):
        return self

      def __exit__(self, *a):
        return False

      def get(self, url, params=None, **k):
        return net.get(url, params, **k)

      def post(self, url, json=None, **k):
        return net.post(url, json, **k)
    return S()


def _load(rel, names, env):
  """functions / classes / constants `names` from a module's source, imports stripped, run with the fakes in env"""
  src = (ROOT / rel).read_text(encoding="utf-8")
  tree = ast.parse(src)
  keep = [n for n in tree.body if (isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names) or
          (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in n.targets))]
  env.setdefault("remote_access_blocked", RA.remote_access_blocked)
  exec(compile(ast.Module(body=keep, type_ignores=[]), str(ROOT / rel), "exec"), env)
  return env


class Params:
  def __init__(self, d):
    self.d = d

  def get(self, k, encoding=None):
    return self.d.get(k)

  def get_float(self, k):
    return 0.0

  def get_bool(self, k):
    return False

  def put(self, k, v):
    self.d[k] = v


def test_stats_and_geocoder(flag):
  net = Net()
  pos = {"latitude": 12.5, "longitude": -45.25}
  toggles = NS(car_make="mazda")
  env = _load("frogpilot/system/frogpilot_stats.py",
              {"BASE_URL", "GITHUB_API_URL", "MINIMUM_POPULATION", "TRACKED_BRANCHES", "get_branch_commits", "get_city_center",
               "send_stats"},
              {"requests": NS(Session=net.Session, post=net.post, exceptions=NS(RequestException=Exception)), "json": json,
               "FROGPILOT_API": API, "clean_model_name": lambda n: n, "get_frogpilot_toggles": lambda: toggles,
               "get_frogpilot_api_info": lambda: ("tok", {}, "tici", "fp0123456789"),
               "params": Params({"LastGPSPosition": json.dumps(pos), "Model": "x_default"}), "car": None, "custom": None})
  env["send_stats"]()
  hosts = [(m, u) for m, u, _ in net.calls]
  if flag:
    assert net.calls == []
    assert env["get_city_center"](12.5, -45.25) == (0.0, 0.0, "N/A", "N/A", "N/A") and net.calls == []
  else:
    geo = [c for c in net.calls if "nominatim" in c[1] and c[1].endswith("/reverse")]
    assert geo and geo[0][2]["lat"] == 12.5 and geo[0][2]["lon"] == -45.25         # unchanged: the position goes to the geocoder
    assert ("POST", f"{API}/stats") in hosts


def test_register(flag):
  net = Net()
  pings = []
  params = Params({"DongleId": "d0", "FrogPilotDongleId": None})
  env = _load("frogpilot/common/frogpilot_functions.py", {"register_device"},
              {"requests": NS(post=net.post), "FROGPILOT_API": API, "params": params, "time": NS(sleep=lambda s: None),
               "is_url_pingable": lambda u: pings.append(u) or True, "dataclasses": NS(asdict=lambda b: {}),
               "HARDWARE": NS(get_device_type=lambda: "tici"),
               "threading": NS(Thread=lambda target, daemon=True: NS(start=target))})
  env["register_device"](object())
  if flag:
    assert net.calls == [] and pings == []
  else:
    assert pings and [c[1] for c in net.calls] == [f"{API}/register"] and net.calls[0][2]["dongle_id"] == "d0"


def test_error_report(flag, tmp_path):
  net = Net()
  env = _load("frogpilot/common/frogpilot_utilities.py", {"capture_report"},
              {"requests": NS(post=net.post, exceptions=NS(RequestException=Exception)), "FROGPILOT_API": API,
               "is_url_pingable": lambda u: True, "get_frogpilot_api_info": lambda: ("tok", {}, "tici", "fp0123456789"),
               "ERROR_LOGS_PATH": tmp_path})
  env["capture_report"]("user", "it broke", {"t": 1})
  assert [c[1] for c in net.calls] == ([] if flag else [f"{API}/discord/report"])


def test_weather_lookup(flag):
  net = Net()

  class SyncExecutor:
    def submit(self, fn):
      f = Future()
      f.set_result(fn())
      return f
  env = _load("frogpilot/controls/lib/weather_checker.py", {"WeatherChecker", "CACHE_DISTANCE", "MAX_RETRIES", "RETRY_DELAY"},
              {"FROGPILOT_API": API, "time": NS(sleep=lambda s: None)})
  wc = env["WeatherChecker"].__new__(env["WeatherChecker"])
  wc.__dict__.update(dict(is_daytime=False, api_25_calls=0, api_3_calls=0, sunrise=0, sunset=0, weather_id=0, hourly_forecast=None,
                          last_gps_position=None, last_updated=None, requesting=False, user_api_key=None, check_interval=900,
                          api_token="tok", build_metadata={}, device_type="tici", dongle_id="fp0123456789",
                          session=NS(post=net.post), executor=SyncExecutor()))
  wc.update_offsets = lambda toggles: None
  wc.update_weather({"latitude": 12.5, "longitude": -45.25}, datetime.datetime(2026, 11, 26, 12), NS())
  if flag:
    assert net.calls == []
  else:
    assert [c[1] for c in net.calls] == [f"{API}/weather"] and net.calls[0][2]["lat"] == 12.5


def test_theme_submission_is_gated_before_any_request():
  """submit_theme is a Flask route inside the_pond's app (needs the device): its first statement is the gate and returns."""
  tree = ast.parse((ROOT / "frogpilot/system/the_pond/the_pond.py").read_text(encoding="utf-8"))
  fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "submit_theme")
  first = fn.body[0]
  assert isinstance(first, ast.If) and isinstance(first.test, ast.Call), ast.dump(first)[:200]
  assert getattr(first.test.func, "id", None) == "remote_access_blocked" and first.test.args[0].value == "theme_submission"
  assert isinstance(first.body[-1], ast.Return)


def test_the_flag_path_is_written_in_one_place_only():
  hits = []
  for base in ("frogpilot", "system", "selfdrive"):
    for p in ROOT.joinpath(base).rglob("*.py"):
      if "/tests/" in p.as_posix():
        continue
      if "/data/no_remote_access" in p.read_text(encoding="utf-8", errors="ignore"):
        hits.append(p.relative_to(ROOT).as_posix())
  assert hits == ["system/manager/remote_access.py"]


def test_a_skip_is_logged_once_per_call_name(tmp_path, monkeypatch, capsys):
  f = tmp_path / "no_remote_access"
  f.write_text("")
  monkeypatch.setattr(RA, "NO_REMOTE_ACCESS_FLAG", str(f))
  monkeypatch.setattr(RA, "_logged", set())
  assert all(RA.remote_access_blocked("weather") for _ in range(5)) and RA.remote_access_blocked("stats")
  out = capsys.readouterr().out.splitlines()
  assert out == ["remote_access: weather skipped (no_remote_access flag present)",
                 "remote_access: stats skipped (no_remote_access flag present)"]
