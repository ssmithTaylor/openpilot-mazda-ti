"""The off-road log-collector notifier: bounded, off by default, nothing sent without Wi-Fi, a URL and a token."""
import ast
import io
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


class Clock:
  def __init__(self):
    self.t = 0.0

  def __call__(self):
    return self.t

  def sleep(self, s):
    self.t += s


class FakeHttp:
  def __init__(self, notify=(), status=()):
    self.notify_results, self.status_results = list(notify), list(status)
    self.notifies, self.statuses = [], 0

  def notify(self, body):
    self.notifies.append(body)
    r = self.notify_results.pop(0)
    if isinstance(r, BaseException):
      raise r
    return r, {}

  def status(self):
    self.statuses += 1
    r = self.status_results.pop(0)
    if isinstance(r, BaseException):
      raise r
    return 200, r


def make(http, wifi=lambda: True):
  c = Clock()
  logs = []
  n = cln.Notifier(http, wifi, summary=lambda: (12, "0000033b--abcdef0123"), sleep=c.sleep, clock=c, log=logs.append)
  return n, c, logs


def test_happy_path_notifies_once_then_stops_when_done():
  http = FakeHttp(notify=[202], status=[{"done": False, "state": "pulling"}, {"done": True, "state": "idle"}])
  n, c, logs = make(http)
  assert n.run_once() == "done"
  assert http.notifies == [{"device": "comma", "segments": 12, "newest": "0000033b--abcdef0123"}]
  assert http.statuses == 2


def test_collector_absent_gives_up_after_bounded_attempts():
  http = FakeHttp(notify=[OSError("timed out")] * cln.NOTIFY_ATTEMPTS)
  n, c, logs = make(http)
  assert n.run_once() == "not_notified"
  assert len(http.notifies) == cln.NOTIFY_ATTEMPTS
  assert c.t <= cln.START_DELAY_S + sum(cln.NOTIFY_BACKOFF_S) + 60 * cln.NOTIFY_ATTEMPTS


def test_bad_token_is_not_retried():
  err = urllib.error.HTTPError("u", 401, "unauthorized", {}, io.BytesIO(b"{}"))
  http = FakeHttp(notify=[err])
  n, c, logs = make(http)
  assert n.run_once() == "not_notified" and len(http.notifies) == 1


def test_no_wifi_means_nothing_is_sent():
  http = FakeHttp()
  n, c, logs = make(http, wifi=lambda: False)
  assert n.run_once() == "no_wifi" and http.notifies == []
  assert c.t <= cln.START_DELAY_S + cln.WIFI_WAIT_S + 10


def test_status_polling_is_bounded():
  http = FakeHttp(notify=[202], status=[{"done": False, "state": "pulling"}] * 1000)
  n, c, logs = make(http)
  assert n.run_once() == "window_ended"
  assert http.statuses <= cln.POLL_WINDOW_S // cln.POLL_EVERY_S + 1
  http = FakeHttp(notify=[202], status=[OSError("gone")] * 10)
  n, c, logs = make(http)
  assert n.run_once() == "unreachable" and http.statuses == cln.POLL_MAX_FAILURES


def test_stops_when_collector_cannot_pull():
  http = FakeHttp(notify=[202], status=[{"done": False, "state": "low_space"}])
  n, c, logs = make(http)
  assert n.run_once() == "low_space"


def test_segment_summary_reads_names_only(tmp_path):
  for d in ("0000033a--9b9fc41e44--0", "0000033a--9b9fc41e44--1", "0000033b--0123456789--0", "boot", "crash"):
    (tmp_path / d).mkdir()
  assert cln.segment_summary(str(tmp_path)) == (3, "0000033b--0123456789")
  assert cln.segment_summary(str(tmp_path / "missing")) == (0, "")


def test_token_file(tmp_path):
  assert cln.read_token(str(tmp_path / "none")) == ""
  p = tmp_path / "token"
  p.write_text("abc\n")
  assert cln.read_token(str(p)) == "abc"


def test_http_client_against_a_local_server():
  seen = {}

  class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
      pass

    def _reply(self, obj):
      body = json.dumps(obj).encode()
      self.send_response(202 if self.command == "POST" else 200)
      self.send_header("Content-Length", str(len(body)))
      self.end_headers()
      self.wfile.write(body)

    def do_POST(self):
      seen["auth"] = self.headers.get("Authorization")
      seen["body"] = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
      self._reply({"queued": "x"})

    def do_GET(self):
      self._reply({"done": True})

  srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
  threading.Thread(target=srv.serve_forever, daemon=True).start()
  try:
    http = cln.Http(f"http://127.0.0.1:{srv.server_address[1]}/", "tok")
    assert http.notify({"device": "comma", "segments": 1, "newest": ""})[0] == 202
    assert seen == {"auth": "Bearer tok", "body": {"device": "comma", "segments": 1, "newest": ""}}
    assert http.status() == (200, {"done": True})
  finally:
    srv.shutdown()


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


def test_param_registered_everywhere_the_fork_requires():
  import importlib.util
  spec = importlib.util.spec_from_file_location("cps", ROOT / "frogpilot" / "tools" / "check_params_sync.py")
  cps = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(cps)
  assert "CommaLogsUrl" in cps.python_keys() and "CommaLogsUrl" in cps.source_keys()
  defaults = (ROOT / "frogpilot" / "common" / "frogpilot_variables.py").read_text(encoding="utf8")
  assert '("CommaLogsUrl", "", 3, "")' in defaults  # empty by default = off


def test_no_control_path_imports():
  src = (ROOT / "frogpilot" / "system" / "comma_logs_notifier.py").read_text(encoding="utf8")
  for forbidden in ("selfdrive.controls", "selfdrive.car", "panda", "ForcePowerDown", "DisablePowerDown", "put("):
    assert forbidden not in src
