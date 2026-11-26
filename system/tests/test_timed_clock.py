"""timed corrects the system clock from GPS time when it is off by 10 s or more in EITHER direction.

Upstream's check (`diff = now - gps; if diff < 10 s: return`) only corrected a clock that was AHEAD of GPS time; on this
car the clock sits ~1100 days behind while driving without internet and was never corrected. timed.py imports
timezonefinder and the compiled messaging stack, so `set_time` is taken from the source with ast and run against fakes
(the code tested is the code shipped). `date -s` is never run: subprocess is a fake that records the call."""
import ast
import datetime as real_datetime
import subprocess as real_subprocess
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

REPO = Path(__file__).resolve().parents[2]
GPS = real_datetime.datetime(2026, 11, 26, 12, 0, 0)


def _load_set_time(now, date_s_fails=False):
  """set_time from system/timed.py with a fake clock reading `now`, a fake subprocess and a recording cloudlog."""
  state = {"now": now}
  calls, events = [], []

  class FakeDatetime(real_datetime.datetime):
    @classmethod
    def now(cls, tz=None):
      return state["now"]

  def run(cmd, shell=False, check=False):
    calls.append(cmd)
    if date_s_fails:
      raise real_subprocess.CalledProcessError(1, cmd)
    state["now"] = GPS  # the fake `date -s` sets the clock

  log = NS(debug=lambda *a, **k: None, exception=lambda *a, **k: None,
           event=lambda name, **kw: events.append((name, kw)))
  env = {
    "datetime": NS(datetime=FakeDatetime, timedelta=real_datetime.timedelta),
    "subprocess": NS(run=run, CalledProcessError=real_subprocess.CalledProcessError),
    "cloudlog": log,
  }
  tree = ast.parse((REPO / "system/timed.py").read_text(encoding="utf-8"))
  body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "set_time"]
  assert len(body) == 1
  exec(compile(ast.Module(body=body, type_ignores=[]), str(REPO / "system/timed.py"), "exec"), env)
  return env["set_time"], calls, events


@pytest.mark.parametrize("offset", [
  real_datetime.timedelta(days=-1100),   # the car's case: clock far BEHIND GPS (failed on the old code)
  real_datetime.timedelta(seconds=-10),  # behind by exactly the threshold
  real_datetime.timedelta(seconds=10),   # ahead by exactly the threshold
  real_datetime.timedelta(days=3),       # ahead
])
def test_clock_off_by_10s_or_more_is_corrected(offset):
  set_time, calls, events = _load_set_time(GPS + offset)
  set_time(GPS)
  assert len(calls) == 1 and "date -s" in calls[0] and str(GPS) in calls[0]
  assert events == [("timed.set_time", {"ok": True, "offset_before_s": round(offset.total_seconds(), 3),
                                         "offset_after_s": 0.0})]


@pytest.mark.parametrize("offset_s", [0.0, 9.999, -9.999, 5.0, -5.0])
def test_clock_within_10s_is_left_alone(offset_s):
  set_time, calls, events = _load_set_time(GPS + real_datetime.timedelta(seconds=offset_s))
  set_time(GPS)
  assert calls == [] and events == []


def test_failed_set_is_logged_with_offset():
  set_time, calls, events = _load_set_time(GPS - real_datetime.timedelta(days=1100), date_s_fails=True)
  set_time(GPS)
  assert len(calls) == 1
  assert events == [("timed.set_time", {"ok": False, "offset_before_s": -1100 * 86400.0})]


def test_log_line_carries_no_position():
  src = (REPO / "system/timed.py").read_text(encoding="utf-8")
  fn = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == "set_time")
  names = {n.id for n in ast.walk(fn) if isinstance(n, ast.Name)}
  assert not names & {"pos", "llk", "lat", "lng", "positionGeodetic"}
