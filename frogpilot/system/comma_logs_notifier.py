#!/usr/bin/env python3
"""Tell the home log collector that the device is parked on the home network. One request, then stop.

Off by default. It runs only while off-road (the manager starts it when the car is off and stops it when a drive
starts) and only when the CommaLogsUrl param holds the collector's https:// URL. An empty param means off.

Two more settings live in CONFIG_DIR, outside the params (so they are not shown in settings, not in toggle
backups, and not in this repository); the owner provisions them by hand:
  token        the shared token for the collector
  home_subnet  the home network in CIDR form, e.g. 10.0.0.0/24

Nothing is sent anywhere else: before any DNS lookup or connection the device must be on Wi-Fi AND have a wlan0
address inside home_subnet (read locally with `ip`, no traffic). Off the home subnet it does nothing at all.
On the home network it POSTs /api/notify once (segment count and newest route name), retrying a bounded number of
times, then exits its work: the collector pulls over SSH by itself, so there is nothing to wait for. https only,
redirects refused (so the token can only go to the configured host, inside a verified TLS session). Any failure
ends quietly. Nothing here touches steering, control or the power-down logic.
"""
import http.client
import ipaddress
import json
import os
import ssl
import subprocess
import time
import urllib.error
import urllib.request

URL_PARAM = "CommaLogsUrl"
CONFIG_DIR = "/data/comma_logs"

START_DELAY_S = 20           # let the manager and network settle after the drive ends
HOME_WAIT_S = 30 * 60        # give up on this off-road period if not on the home network within this time
NOTIFY_ATTEMPTS = 6          # bounded retries for the notification
NOTIFY_BACKOFF_S = (15, 30, 60, 120, 240)
HTTP_TIMEOUT_S = 8
NET_ERRORS = (OSError, ValueError, http.client.HTTPException)  # URLError and HTTPError are OSErrors


def segment_summary(root):
  """(segment count, newest route name) from directory names only; no file is opened."""
  try:
    names = os.listdir(root)
  except OSError:
    return 0, ""
  segs = [n for n in names if "--" in n and n.rsplit("--", 1)[-1].isdigit()]
  routes = sorted({n.rsplit("--", 1)[0] for n in segs})
  return len(segs), (routes[-1] if routes else "")


def read_setting(name, config_dir=CONFIG_DIR):
  try:
    with open(os.path.join(config_dir, name)) as f:
      return f.read().strip()
  except OSError:
    return ""


def load_config(url, token, subnet):
  """(config, None) or (None, why it is off). Validates without touching the network."""
  url = (url or "").strip()
  if not url:
    return None, "CommaLogsUrl is empty"
  if not url.startswith("https://"):
    return None, "CommaLogsUrl must start with https://"
  if not token:
    return None, f"no token in {CONFIG_DIR}/token"
  try:
    net = ipaddress.ip_network(subnet, strict=False)
  except ValueError:
    return None, f"no valid home subnet in {CONFIG_DIR}/home_subnet"
  return {"url": url.rstrip("/"), "token": token, "home": net}, None


def wlan_addresses(run=subprocess.run):
  """IPv4 interface addresses on wlan0, read locally (`ip -4 -o addr show dev wlan0`)."""
  try:
    out = run(["ip", "-4", "-o", "addr", "show", "dev", "wlan0"], capture_output=True, text=True, timeout=5).stdout
  except (OSError, subprocess.SubprocessError):
    return []
  addrs = []
  for line in out.splitlines():
    parts = line.split()
    if "inet" in parts:
      try:
        addrs.append(ipaddress.ip_interface(parts[parts.index("inet") + 1]))
      except (ValueError, IndexError):
        pass
  return addrs


def on_home_subnet(home, addrs):
  return any(a.ip in home for a in addrs)


class NoRedirect(urllib.request.HTTPRedirectHandler):
  def redirect_request(self, req, fp, code, msg, headers, newurl):
    fp.close()  # the redirect reply's connection is not used again
    raise urllib.error.HTTPError(req.full_url, code, "redirect refused", headers, fp)


class Http:
  def __init__(self, base, token, timeout=HTTP_TIMEOUT_S, require_https=True):
    if require_https and not base.startswith("https://"):
      raise ValueError("https only")
    self.base, self.token, self.timeout = base.rstrip("/"), token, timeout
    self.opener = urllib.request.build_opener(NoRedirect, urllib.request.HTTPSHandler(context=ssl.create_default_context()))

  def notify(self, body):
    req = urllib.request.Request(self.base + "/api/notify", data=json.dumps(body).encode(), method="POST",
                                 headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"})
    try:
      with self.opener.open(req, timeout=self.timeout) as r:
        r.read(65536)  # drain; the reply's content is not needed
        return r.status
    except urllib.error.HTTPError as e:
      e.close()  # a non-2xx reply carries the open connection; close it on every error path
      raise


class Notifier:
  """One off-road period. All I/O is injected so it can be tested without a device or network."""

  def __init__(self, http, wifi, home, addrs, summary, sleep=time.sleep, clock=time.monotonic, log=print):
    self.http, self.wifi, self.home, self.addrs, self.summary = http, wifi, home, addrs, summary
    self.sleep, self.clock, self.log = sleep, clock, log

  def at_home(self):
    """Checked before every attempt: Wi-Fi and a wlan0 address on the home subnet. No network traffic."""
    return bool(self.wifi()) and on_home_subnet(self.home, self.addrs())

  def wait_for_home(self):
    t0 = self.clock()
    while not self.at_home():
      if self.clock() - t0 > HOME_WAIT_S:
        return False
      self.sleep(10)
    return True

  def notify(self):
    count, newest = self.summary()
    body = {"device": "comma", "segments": count, "newest": newest}
    for i in range(NOTIFY_ATTEMPTS):
      if not self.at_home():
        return False  # left the home network: send nothing
      try:
        code = self.http.notify(body)
        if 200 <= code < 300:
          self.log(f"comma_logs_notifier: collector notified ({count} segments on the device)")
          return True
      except urllib.error.HTTPError as e:
        e.close()  # idempotent; Http.notify closes it too, but whichever client raised it, nothing stays open
        if e.code in (401, 403):
          self.log("comma_logs_notifier: collector refused the token; not retrying")
          return False
        if 300 <= e.code < 400:
          self.log("comma_logs_notifier: collector answered with a redirect; refused, not retrying")
          return False
      except NET_ERRORS:
        pass  # collector absent, DNS failure, timeout, TLS failure, malformed reply: bounded retries
      if i < NOTIFY_ATTEMPTS - 1:
        self.sleep(NOTIFY_BACKOFF_S[min(i, len(NOTIFY_BACKOFF_S) - 1)])
    self.log("comma_logs_notifier: collector not reachable; giving up until the next drive")
    return False

  def run_once(self):
    self.sleep(START_DELAY_S)
    if not self.wait_for_home():
      return "not_home"
    return "notified" if self.notify() else "not_notified"


def idle_forever():
  while True:
    time.sleep(3600)  # once per off-road period; the manager stops this process when a drive starts


def off_control_core():
  """Cores 0-3 only, before any thread starts (the threads inherit it): never core 4, where controlsd and card run real-time
  (core-4 budget, 2026-11-26: seen there in the drive logs). Never raises; a host without sched_setaffinity keeps its cores."""
  try:
    cores = [c for c in os.sched_getaffinity(0) if c in (0, 1, 2, 3)]
    if cores:
      os.sched_setaffinity(0, cores)
  except Exception:
    pass


def main():
  off_control_core()
  from cereal import log, messaging
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  from openpilot.system.hardware.hw import Paths

  cfg, why = load_config(Params().get(URL_PARAM, encoding="utf-8"), read_setting("token"), read_setting("home_subnet"))
  if cfg is None:
    cloudlog.info(f"comma_logs_notifier: off ({why})")
    idle_forever()

  sm = messaging.SubMaster(["deviceState"])
  wifi_type = log.DeviceState.NetworkType.wifi

  def wifi():
    sm.update(0)
    return bool(sm.seen["deviceState"] and sm["deviceState"].networkType == wifi_type)

  root = Paths.log_root()
  try:
    result = Notifier(Http(cfg["url"], cfg["token"]), wifi, cfg["home"], wlan_addresses,
                      lambda: segment_summary(root), log=cloudlog.info).run_once()
  except Exception:  # never let a surprise here reach the manager as a crash loop
    cloudlog.exception("comma_logs_notifier: unexpected error")
    result = "error"
  cloudlog.info(f"comma_logs_notifier: {result}")
  idle_forever()


if __name__ == "__main__":
  main()
