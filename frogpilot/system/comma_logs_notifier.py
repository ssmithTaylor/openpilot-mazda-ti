#!/usr/bin/env python3
"""Tell the home log collector that the device is parked and on Wi-Fi, then wait briefly for it to finish.

Off by default. It runs only while off-road (the manager starts it when the car is off and stops it when a drive
starts) and only when the CommaLogsUrl param holds the collector's URL, e.g. https://comma-logs.<home domain>.
An empty param means off. The shared token is read from TOKEN_PATH, which the owner provisions by hand; it is not
a param, so it is not shown in the settings or included in param backups.

Once per off-road period: wait for Wi-Fi (deviceState.networkType), POST /api/notify with the segment count and
the newest route name, then poll /api/status until the collector says `done` or the poll window ends. Every step
is bounded (attempts, timeouts, total time); when the collector is absent it gives up quietly and does nothing
else until the next off-road period. The collector does the pulling itself, over SSH; this process sends
nothing but the notification. Nothing here touches steering, control or the power-down logic.
"""
import json
import os
import ssl
import time
import urllib.error
import urllib.request

URL_PARAM = "CommaLogsUrl"
TOKEN_PATH = "/data/comma_logs/token"
LOG_ROOT = "/data/media/0/realdata_konik"

START_DELAY_S = 20           # let the manager and network settle after the drive ends
WIFI_WAIT_S = 30 * 60        # give up on this off-road period if no Wi-Fi within this time
NOTIFY_ATTEMPTS = 6          # bounded retries for the notification itself
NOTIFY_BACKOFF_S = (15, 30, 60, 120, 240)
HTTP_TIMEOUT_S = 8
POLL_EVERY_S = 60
POLL_WINDOW_S = 3 * 3600     # stop asking after this long; the collector keeps pulling regardless
POLL_MAX_FAILURES = 5        # consecutive status failures before giving up


def segment_summary(root=LOG_ROOT):
  """(segment count, newest route name) from directory names only; no file is opened."""
  try:
    names = os.listdir(root)
  except OSError:
    return 0, ""
  segs = [n for n in names if "--" in n and n.rsplit("--", 1)[-1].isdigit()]
  routes = sorted({n.rsplit("--", 1)[0] for n in segs})
  return len(segs), (routes[-1] if routes else "")


def read_token(path=TOKEN_PATH):
  try:
    with open(path) as f:
      return f.read().strip()
  except OSError:
    return ""


class Http:
  def __init__(self, base, token, timeout=HTTP_TIMEOUT_S):
    self.base, self.token, self.timeout = base.rstrip("/"), token, timeout
    self.ctx = ssl.create_default_context()

  def _req(self, method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(self.base + path, data=data, method=method,
                                 headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=self.timeout, context=self.ctx) as r:
      return r.status, json.loads(r.read(65536) or b"{}")

  def notify(self, body):
    return self._req("POST", "/api/notify", body)

  def status(self):
    return self._req("GET", "/api/status")


class Notifier:
  """One off-road period. All I/O is injected so it can be tested without a device or network."""

  def __init__(self, http, wifi, summary=segment_summary, sleep=time.sleep, clock=time.monotonic, log=print):
    self.http, self.wifi, self.summary = http, wifi, summary
    self.sleep, self.clock, self.log = sleep, clock, log

  def wait_for_wifi(self):
    t0 = self.clock()
    while not self.wifi():
      if self.clock() - t0 > WIFI_WAIT_S:
        return False
      self.sleep(10)
    return True

  def notify(self):
    count, newest = self.summary()
    body = {"device": "comma", "segments": count, "newest": newest}
    for i in range(NOTIFY_ATTEMPTS):
      if not self.wifi():
        return False
      try:
        code, _ = self.http.notify(body)
        if 200 <= code < 300:
          self.log(f"comma_logs_notifier: collector notified ({count} segments on the device)")
          return True
        if code in (401, 403):
          self.log("comma_logs_notifier: collector refused the token; not retrying")
          return False
      except urllib.error.HTTPError as e:
        if e.code in (401, 403):
          self.log("comma_logs_notifier: collector refused the token; not retrying")
          return False
      except (OSError, ValueError):
        pass  # collector absent, DNS failure, timeout, TLS failure: retry a bounded number of times
      if i < NOTIFY_ATTEMPTS - 1:
        self.sleep(NOTIFY_BACKOFF_S[min(i, len(NOTIFY_BACKOFF_S) - 1)])
    self.log("comma_logs_notifier: collector not reachable; giving up until the next drive")
    return False

  def wait_until_done(self):
    t0, failures = self.clock(), 0
    while self.clock() - t0 < POLL_WINDOW_S:
      self.sleep(POLL_EVERY_S)
      try:
        _, st = self.http.status()
        failures = 0
        if st.get("done"):
          self.log("comma_logs_notifier: collector reports done")
          return "done"
        if st.get("state") in ("low_space", "waiting_for_ssh_key"):
          self.log(f"comma_logs_notifier: collector cannot pull ({st.get('state')}); stopping")
          return st.get("state")
      except (OSError, ValueError):
        failures += 1
        if failures >= POLL_MAX_FAILURES:
          return "unreachable"
    return "window_ended"

  def run_once(self):
    self.sleep(START_DELAY_S)
    if not self.wait_for_wifi():
      return "no_wifi"
    if not self.notify():
      return "not_notified"
    return self.wait_until_done()


def main():
  from cereal import log, messaging
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog

  url = (Params().get(URL_PARAM, encoding="utf-8") or "").strip()
  token = read_token()
  if not url.startswith(("https://", "http://")) or not token:
    cloudlog.info("comma_logs_notifier: not configured (CommaLogsUrl or the token file is missing); idle")
    while True:
      time.sleep(3600)  # the manager stops this process when a drive starts

  sm = messaging.SubMaster(["deviceState"])
  wifi_type = log.DeviceState.NetworkType.wifi

  def wifi():
    sm.update(0)
    return bool(sm.seen["deviceState"] and sm["deviceState"].networkType == wifi_type)

  result = Notifier(Http(url, token), wifi, log=cloudlog.info).run_once()
  cloudlog.info(f"comma_logs_notifier: {result}")
  while True:
    time.sleep(3600)  # once per off-road period; the manager restarts the process after the next drive


if __name__ == "__main__":
  main()
