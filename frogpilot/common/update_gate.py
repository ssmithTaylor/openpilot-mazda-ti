"""When frogpilot_process may run its model / theme / map update checks, and a connectivity probe that leaves no garbage
(re-review R2-I2).

RUNTIME: frogpilot_process (20 Hz, garbage collector OFF: a reference cycle left here is never freed). Everything below runs
in the update_checks worker thread, never in the 20 Hz loop: one IsOnroad param read every ONROAD_POLL_S while it waits, and
at most one probe (a TCP connect to each host in turn) per back-off step.

Before: update_checks pinged github.com / gitlab.com with requests every 60 s until one answered. On this car (no network
while driving, almost never otherwise) that never ended: it carried on into the next drive, and every failed requests ping
left about 64 collector-tracked objects (reviewer's probe) that a gc-off process never frees.
Now: no probe and no update check while on-road (the thread ends at once; the next off-road transition starts it again);
parked without a network the probe backs off 60 s, 120 s, 240 s ... up to RETRY_CEILING_S; the probe is a raw socket
connect whose failure leaves no reference cycle (test_update_checks_offline.py checks that with gc disabled).

Replayability: the decisions are logged (print + cloudlog event "update_checks"): no network (once per wait), the car went
on-road (the wait ended), network found (with the number of failed probes).
"""
import socket
import time

PROBE_HOSTS = (("github.com", 443), ("gitlab.com", 443))
PROBE_TIMEOUT_S = 5.0
RETRY_FIRST_S = 60.0
RETRY_CEILING_S = 6 * 3600.0
ONROAD_POLL_S = 10.0


def network_reachable(hosts=PROBE_HOSTS, timeout=PROBE_TIMEOUT_S, getaddrinfo=socket.getaddrinfo, make_socket=socket.socket) -> bool:
  """True if a TCP connection to any of the hosts opens. No HTTP, no exception kept, no reference cycle on failure."""
  for host, port in hosts:
    try:
      infos = getaddrinfo(host, port, 0, socket.SOCK_STREAM)
    except OSError:
      continue
    for family, socktype, proto, _canon, addr in infos:
      s = None
      try:
        s = make_socket(family, socktype, proto)
        s.settimeout(timeout)
        ok = s.connect_ex(addr) == 0
      except OSError:
        ok = False
      finally:
        if s is not None:
          s.close()
      if ok:
        return True
  return False


def _log(line, **fields):
  print(f"update_checks: {line}")
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.event("update_checks", msg=line, **fields)
  except Exception:
    pass


def wait_for_network_parked(is_onroad, reachable=network_reachable, sleep=time.sleep, log=_log,
                            first_s=RETRY_FIRST_S, ceiling_s=RETRY_CEILING_S, poll_s=ONROAD_POLL_S) -> bool:
  """Block while parked until the network answers (True). Returns False, without probing, as soon as the car is on-road."""
  delay = first_s
  failures = 0
  while True:
    if is_onroad():
      if failures:
        log("car on-road, stopped waiting for a network", failures=failures)
      return False
    if reachable():
      if failures:
        log("network found", failures=failures)
      return True
    failures += 1
    if failures == 1:
      log(f"no network; retrying with back-off from {first_s:.0f} s up to {ceiling_s:.0f} s", failures=failures)
    waited = 0.0
    while waited < delay:
      if is_onroad():
        log("car on-road, stopped waiting for a network", failures=failures)
        return False
      sleep(poll_s)
      waited += poll_s
    delay = min(delay * 2, ceiling_s)
