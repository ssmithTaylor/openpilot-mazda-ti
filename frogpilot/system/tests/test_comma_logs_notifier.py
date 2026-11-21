"""The off-road log-collector notifier: off by default, nothing sent off the home subnet, one bounded notification."""
import ast
import http.client
import io
import ipaddress
import json
import re
import threading
import urllib.error
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.frogpilot.system import comma_logs_notifier as cln

ROOT = Path(__file__).resolve().parents[3]
HOME = ipaddress.ip_network("10.20.30.0/24")  # a test subnet; the real one is provisioned on the device only
AT_HOME = [ipaddress.ip_interface("10.20.30.100/24")]
AWAY = [ipaddress.ip_interface("172.16.5.9/24")]


class Clock:
  def __init__(self):
    self.t = 0.0

  def __call__(self):
    return self.t

  def sleep(self, s):
    self.t += s


class FakeHttp:
  def __init__(self, results=()):
    self.results, self.calls = list(results), []

  def notify(self, body):
    self.calls.append(body)
    r = self.results.pop(0)
    if isinstance(r, BaseException):
      raise r
    return r


def make(http, wifi=lambda: True, addrs=lambda: AT_HOME):
  c = Clock()
  logs = []
  n = cln.Notifier(http, wifi, HOME, addrs, summary=lambda: (12, "0000033b--abcdef0123"), sleep=c.sleep, clock=c,
                   log=logs.append)
  return n, c, logs


def test_notifies_once_and_returns_without_waiting():
  http = FakeHttp([202])
  n, c, logs = make(http)
  assert n.run_once() == "notified"
  assert http.calls == [{"device": "comma", "segments": 12, "newest": "0000033b--abcdef0123"}]
  assert c.t == cln.START_DELAY_S  # no polling afterwards
  assert not hasattr(n, "wait_until_done")


def test_other_wifi_sends_nothing_at_all():
  http = FakeHttp()
  n, c, logs = make(http, addrs=lambda: AWAY)
  assert n.run_once() == "not_home" and http.calls == []
  assert c.t <= cln.START_DELAY_S + cln.HOME_WAIT_S + 10


def test_no_wifi_or_no_address_sends_nothing():
  for wifi, addrs in ((lambda: False, lambda: AT_HOME), (lambda: True, lambda: [])):
    http = FakeHttp()
    n, c, logs = make(http, wifi=wifi, addrs=addrs)
    assert n.run_once() == "not_home" and http.calls == []


def test_leaving_home_between_retries_stops_sending():
  where = iter([AT_HOME, AT_HOME, AWAY])  # wait_for_home, first attempt, then away before the retry
  http = FakeHttp([OSError("timed out")] * 5)
  n, c, logs = make(http, addrs=lambda: next(where))
  assert n.run_once() == "not_notified" and len(http.calls) == 1


def test_collector_absent_gives_up_after_bounded_attempts():
  http = FakeHttp([OSError("timed out")] * cln.NOTIFY_ATTEMPTS)
  n, c, logs = make(http)
  assert n.run_once() == "not_notified" and len(http.calls) == cln.NOTIFY_ATTEMPTS
  assert c.t <= cln.START_DELAY_S + sum(cln.NOTIFY_BACKOFF_S)


@pytest.mark.parametrize("err", [http.client.BadStatusLine("x"), http.client.IncompleteRead(b""), ValueError("bad"),
                                 urllib.error.URLError("dns")])
def test_malformed_replies_and_network_errors_end_quietly(err):
  http = FakeHttp([err] * cln.NOTIFY_ATTEMPTS)
  n, c, logs = make(http)
  assert n.run_once() == "not_notified"


@pytest.mark.parametrize("code", [401, 403, 302])
def test_bad_token_or_redirect_is_not_retried(code):
  err = urllib.error.HTTPError("u", code, "x", {}, io.BytesIO(b""))
  http = FakeHttp([err])
  n, c, logs = make(http)
  assert n.run_once() == "not_notified" and len(http.calls) == 1


def test_config_is_off_unless_https_url_token_and_subnet():
  ok = ("https://collector.example", "tok", "10.20.30.0/24")
  assert cln.load_config(*ok)[0]["home"] == HOME
  for url, tok, net in [("", *ok[1:]), (None, *ok[1:]), ("http://collector.example", *ok[1:]),
                        (ok[0], "", ok[2]), (ok[0], ok[1], ""), (ok[0], ok[1], "not-a-subnet")]:
    cfg, why = cln.load_config(url, tok, net)
    assert cfg is None and why
  with pytest.raises(ValueError):
    cln.Http("http://collector.example", "tok")


def test_wlan_addresses_parsed_locally():
  out = "3: wlan0    inet 10.20.30.100/24 brd 10.20.30.255 scope global dynamic wlan0\\       valid_lft 86000sec\n"
  addrs = cln.wlan_addresses(run=lambda *a, **k: NS(stdout=out))
  assert addrs == AT_HOME and cln.on_home_subnet(HOME, addrs)
  assert not cln.on_home_subnet(HOME, AWAY)

  def boom(*a, **k):
    raise OSError("no ip")
  assert cln.wlan_addresses(run=boom) == []


def test_settings_and_summary(tmp_path):
  assert cln.read_setting("token", str(tmp_path)) == ""
  (tmp_path / "token").write_text("abc\n")
  assert cln.read_setting("token", str(tmp_path)) == "abc"
  for d in ("0000033a--9b9fc41e44--0", "0000033a--9b9fc41e44--1", "0000033b--0123456789--0", "boot", "crash"):
    (tmp_path / d).mkdir()
  assert cln.segment_summary(str(tmp_path)) == (3, "0000033b--0123456789")
  assert cln.segment_summary(str(tmp_path / "missing")) == (0, "")


def _server(handler):
  srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
  threading.Thread(target=srv.serve_forever, daemon=True).start()
  return srv, f"http://127.0.0.1:{srv.server_address[1]}"


def test_http_client_sends_token_and_body():
  seen = {}

  class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
      pass

    def do_POST(self):
      seen["auth"] = self.headers.get("Authorization")
      seen["body"] = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
      self.send_response(202)
      self.send_header("Content-Length", "2")
      self.end_headers()
      self.wfile.write(b"{}")

  srv, base = _server(H)
  try:
    assert cln.Http(base, "tok", require_https=False).notify({"device": "comma", "segments": 1, "newest": ""}) == 202
    assert seen == {"auth": "Bearer tok", "body": {"device": "comma", "segments": 1, "newest": ""}}
  finally:
    srv.shutdown()


def test_redirect_is_refused_and_token_not_forwarded():
  hits = {"elsewhere": 0}

  class Elsewhere(BaseHTTPRequestHandler):
    def log_message(self, *a):
      pass

    def do_POST(self):
      hits["elsewhere"] += 1
      self.send_response(200)
      self.end_headers()

  other, other_base = _server(Elsewhere)

  class Redirector(BaseHTTPRequestHandler):
    def log_message(self, *a):
      pass

    def do_POST(self):
      self.send_response(307)
      self.send_header("Location", other_base + "/api/notify")
      self.send_header("Content-Length", "0")
      self.end_headers()

  srv, base = _server(Redirector)
  try:
    with pytest.raises(urllib.error.HTTPError) as e:
      cln.Http(base, "tok", require_https=False).notify({})
    assert e.value.code == 307 and hits["elsewhere"] == 0
  finally:
    srv.shutdown()
    other.shutdown()


def _gate():
  src = (ROOT / "system" / "manager" / "process_config.py").read_text(encoding="utf8")
  tree = ast.parse(src)
  fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run_comma_logs_notifier")
  fn.args.args[2].annotation = None  # CP: car.CarParams
  ns = {}
  exec(compile(ast.Module(body=[fn], type_ignores=[]), "process_config", "exec"), ns)
  return ns["run_comma_logs_notifier"], src


class P:
  def __init__(self, url):
    self.url = url

  def get(self, key, encoding=None):
    assert key == "CommaLogsUrl"
    return self.url


@pytest.mark.parametrize("started,url,expected", [
  (False, None, False), (False, "", False), (False, "  ", False),
  (False, "https://collector.example", True), (True, "https://collector.example", False),
])
def test_process_runs_only_offroad_with_a_url(started, url, expected):
  gate, src = _gate()
  assert gate(started, P(url), NS(), False, False, NS()) is expected
  assert re.search(r'PythonProcess\("comma_logs_notifier", "frogpilot\.system\.comma_logs_notifier", run_comma_logs_notifier\)', src)


def test_param_registered_and_kept_across_toggle_reset():
  import importlib.util
  spec = importlib.util.spec_from_file_location("cps", ROOT / "frogpilot" / "tools" / "check_params_sync.py")
  cps = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(cps)
  assert "CommaLogsUrl" in cps.python_keys() and "CommaLogsUrl" in cps.source_keys()
  src = (ROOT / "frogpilot" / "common" / "frogpilot_variables.py").read_text(encoding="utf8")
  assert '("CommaLogsUrl", "", 3, "")' in src  # empty by default = off
  excluded = ast.literal_eval(re.search(r"^EXCLUDED_KEYS = (\{.*?\})", src, re.M | re.S).group(1))
  assert "CommaLogsUrl" in excluded  # toggle reset keeps it; toggle backups leave it out


def test_no_control_path_or_hardcoded_paths():
  src = (ROOT / "frogpilot" / "system" / "comma_logs_notifier.py").read_text(encoding="utf8")
  for forbidden in ("selfdrive.controls", "selfdrive.car", "panda", "ForcePowerDown", "DisablePowerDown", "put(",
                    "realdata", "192.168."):
    assert forbidden not in src
  assert "Paths.log_root()" in src
