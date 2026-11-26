"""Re-review R2-I2: frogpilot_process's update checks on a car that is almost never online.

Before: update_checks pinged github.com / gitlab.com (requests HEAD) every 60 s until one answered: parked without a network
that never ended, carried on into the next drive, and each failed ping left ~64 collector-tracked objects in a process that
runs with the garbage collector OFF. Now: no probe and no check on-road; parked, the probe backs off (doubling to a ceiling of
hours); the probe is a raw TCP connect that leaves no garbage. Nothing here leaves this PC: the probes go to a closed port on
127.0.0.1 or to a resolver stub that fails."""
import ast
import gc
import pathlib
import socket
from types import SimpleNamespace as NS

import pytest

from openpilot.frogpilot.common import update_gate as UG

ROOT = pathlib.Path(__file__).resolve().parents[3]


class _Stop(BaseException):
  pass


def _update_checks(onroad_seq, reachable_seq, max_sleeps=5000):
  """update_checks from frogpilot_process.py's source, with fakes; returns (sleeps, probes, work done)."""
  sleeps, probes, work = [], [], []
  onroad = list(onroad_seq)

  def sleep(s):
    sleeps.append(s)
    if len(sleeps) > max_sleeps:
      raise _Stop

  def reachable(*a, **k):
    probes.append(1)
    return reachable_seq.pop(0) if reachable_seq else False

  def wait_for_network_parked(is_onroad):
    return UG.wait_for_network_parked(is_onroad, reachable=reachable, sleep=sleep, log=lambda *a, **k: None)

  env = {
    "params": NS(get_bool=lambda k: (onroad.pop(0) if len(onroad) > 1 else onroad[0]) if k == "IsOnroad" else False),
    "wait_for_network_parked": wait_for_network_parked, "time": NS(sleep=lambda s: None),
    # the old code's names, so that it runs too (and fails the assertions instead of a NameError)
    "is_url_pingable": lambda u: probes.append(u) or (reachable_seq.pop(0) if reachable_seq else False),
    "run_thread_with_lock": lambda name, fn, args=(): work.append(name), "update_maps": None,
  }
  tree = ast.parse((ROOT / "frogpilot/frogpilot_process.py").read_text(encoding="utf-8"))
  fn = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "update_checks"]
  exec(compile(ast.Module(body=fn, type_ignores=[]), "frogpilot_process.py", "exec"), env)
  env["time"] = NS(sleep=sleep)
  mm = NS(update_models=lambda b: work.append("models"))
  tm = NS(update_themes=lambda t, b: work.append("themes"))
  try:
    env["update_checks"](mm, None, tm, None)
  except _Stop:
    work.append("STILL RUNNING")
  return sleeps, probes, work


def test_on_road_no_probe_and_no_check():
  sleeps, probes, work = _update_checks([True], [True])
  assert probes == [] and work == [] and sleeps == []


def test_parked_without_network_backs_off_then_stops_when_the_car_starts():
  # parked for 2000 polls (about 5.5 hours of 10 s polls), no network, then the car starts
  sleeps, probes, work = _update_checks([False] * 2000 + [True], [])
  assert "STILL RUNNING" not in work and "models" not in work
  assert all(s == UG.ONROAD_POLL_S for s in sleeps)
  # probes at 0, 60, 180, 420, 900, 1860, 3780, 7620, 15300 s ...: logarithmic, not one a minute
  assert 6 <= len(probes) <= 10, len(probes)
  assert len(probes) < len(sleeps) * UG.ONROAD_POLL_S / 60 / 10


def test_back_off_doubles_to_the_ceiling():
  t = {"s": 0.0}
  probe_times = []

  def reachable():
    probe_times.append(t["s"])
    return False

  def sleep(s):
    t["s"] += s
    if t["s"] > 40 * 3600:
      raise _Stop
  with pytest.raises(_Stop):
    UG.wait_for_network_parked(lambda: False, reachable=reachable, sleep=sleep, log=lambda *a, **k: None)
  gaps = [b - a for a, b in zip(probe_times, probe_times[1:], strict=False)]
  assert gaps[0] == UG.RETRY_FIRST_S and gaps[1] == 2 * UG.RETRY_FIRST_S
  assert max(gaps) == UG.RETRY_CEILING_S and UG.RETRY_CEILING_S >= 3600


def test_parked_with_network_runs_the_checks_as_before():
  sleeps, probes, work = _update_checks([False], [True])
  assert work == ["models", "themes", "update_maps"] and len(probes) == 1


def test_network_found_after_some_failures_runs_the_checks():
  sleeps, probes, work = _update_checks([False], [False, False, True])
  assert work == ["models", "themes", "update_maps"] and len(probes) == 3


def test_probe_real_socket_refused_is_false_and_success_is_true():
  def local(host, port, *a):
    if host == "unresolvable":
      raise socket.gaierror(-2, "Name or service not known")
    return socket.getaddrinfo("127.0.0.1", port, 0, socket.SOCK_STREAM)
  srv = socket.socket()
  srv.bind(("127.0.0.1", 0))
  srv.listen(1)
  port = srv.getsockname()[1]
  try:
    assert UG.network_reachable((("unresolvable", 443), ("x", port)), 1.0, local)
  finally:
    srv.close()
  assert not UG.network_reachable((("unresolvable", 443), ("x", port)), 1.0, local)   # now refused


def test_failed_probes_leave_no_tracked_objects_with_the_collector_off():
  """frogpilot_process runs with gc disabled: N failed probes (resolver failure and refused connection) must not grow the
  collector-tracked object count (the requests ping they replace left ~64 per failure)."""
  def local(host, port, *a):
    if host == "unresolvable":
      raise socket.gaierror(-2, "Name or service not known")
    return socket.getaddrinfo("127.0.0.1", 9, 0, socket.SOCK_STREAM)
  no_dns = (("unresolvable", 443), ("unresolvable", 443))      # the car offline: name resolution fails
  refused = (("refused", 9),)                                    # a network but nothing answers (slow to fail on Windows)
  waits = {"n": 0}

  def onroad():
    waits["n"] += 1
    return waits["n"] > 40

  for _ in range(3):
    UG.network_reachable(no_dns, 0.2, local)
    UG.network_reachable(refused, 0.2, local)
  was_enabled = gc.isenabled()
  gc.collect()
  gc.disable()
  try:
    n0 = len(gc.get_objects())
    for _ in range(200):
      assert not UG.network_reachable(no_dns, 0.2, local)
    for _ in range(10):
      assert not UG.network_reachable(refused, 0.2, local)
    # and the wait loop itself, probing with the real probe, parked then on-road
    UG.wait_for_network_parked(onroad, reachable=lambda: UG.network_reachable(no_dns, 0.2, local), sleep=lambda s: None,
                               log=lambda *a, **k: None, first_s=10.0, poll_s=10.0)
    n1 = len(gc.get_objects())
    cyclic = gc.collect()
  finally:
    if was_enabled:
      gc.enable()
  assert n1 - n0 <= 2, f"tracked objects grew by {n1 - n0} over 210 failed probes"
  assert cyclic == 0, f"{cyclic} objects were cyclic garbage"


def test_frogpilot_process_uses_the_gate_and_no_longer_pings_with_requests():
  src = (ROOT / "frogpilot/frogpilot_process.py").read_text(encoding="utf-8")
  tree = ast.parse(src)
  fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "update_checks")
  names = {getattr(n.func, "id", None) for n in ast.walk(fn) if isinstance(n, ast.Call)}
  assert "wait_for_network_parked" in names and "is_url_pingable" not in names
  imported = {a.name for n in tree.body if isinstance(n, ast.ImportFrom) for a in n.names}
  assert "is_url_pingable" not in imported                      # nothing else in this process uses the requests ping
