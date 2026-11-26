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
UTC = real_datetime.timezone.utc
GPS = real_datetime.datetime(2026, 11, 26, 12, 0, 0, tzinfo=UTC)


def _load_set_time(now, date_s_fails=False):
  """set_time from system/timed.py with a fake clock reading `now`, a fake subprocess and a recording cloudlog."""
  state = {"now": now}
  calls, events = [], []

  class FakeDatetime(real_datetime.datetime):
    @classmethod
    def now(cls, tz=None):
      return state["now"] if tz is None else state["now"].astimezone(tz)

  def run(cmd, shell=False, check=False):
    calls.append(cmd)
    if date_s_fails:
      raise real_subprocess.CalledProcessError(1, cmd)
    state["now"] = GPS  # the fake `date -s` sets the clock

  log = NS(debug=lambda *a, **k: None, exception=lambda *a, **k: None,
           event=lambda name, **kw: events.append((name, kw)))
  env = {
    "datetime": NS(datetime=FakeDatetime, timedelta=real_datetime.timedelta, timezone=real_datetime.timezone),
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
  assert len(calls) == 1 and calls[0] == "TZ=UTC date -s '2026-11-26 12:00:00.000000'"
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


@pytest.mark.parametrize("gps", [
  real_datetime.datetime(1980, 1, 6, tzinfo=UTC),        # the earliest value the receiver arithmetic can give
  real_datetime.datetime(2024, 6, 1, tzinfo=UTC),        # before this build
  real_datetime.datetime(2025, 12, 31, 23, 59, tzinfo=UTC),
  real_datetime.datetime(2046, 2, 1, tzinfo=UTC),        # more than 20 years after it
  real_datetime.datetime(2099, 1, 1, tzinfo=UTC),
])
def test_implausible_gps_time_is_refused_and_logged_once(gps):
  set_time, calls, events = _load_set_time(GPS - real_datetime.timedelta(days=1100))
  for _ in range(3):
    set_time(gps)
  assert calls == []
  assert events == [("timed.gps_time_refused", {"gps_year": gps.year})]


def test_plausible_gps_time_at_the_bounds_is_accepted():
  for gps in (real_datetime.datetime(2026, 1, 1, tzinfo=UTC), real_datetime.datetime(2045, 12, 1, tzinfo=UTC)):
    set_time, calls, events = _load_set_time(gps - real_datetime.timedelta(days=1100))
    set_time(gps)
    assert len(calls) == 1, gps


def test_log_line_carries_no_position():
  src = (REPO / "system/timed.py").read_text(encoding="utf-8")
  fn = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == "set_time")
  names = {n.id for n in ast.walk(fn) if isinstance(n, ast.Name)}
  assert not names & {"pos", "llk", "lat", "lng", "positionGeodetic"}


# ---- integ3 review T1-I1: UTC end to end, whatever the device's zone ---------------------------------------------------------------
from zoneinfo import ZoneInfo  # noqa: E402


def _load_zoned(zone, epoch):
  """set_time and gps_datetime from system/timed.py on a fake device whose system clock holds `epoch` (true UTC seconds) and whose
  local zone is `zone`: naive now()/fromtimestamp() give LOCAL wall time in that zone, as on the device; the fake `date -s` honours
  the TZ=UTC prefix (reads the string as UTC, as GNU date does) and sets the system clock from it."""
  zi = ZoneInfo(zone)
  sysclock = {"epoch": epoch}
  calls = []

  class ZonedDatetime(real_datetime.datetime):
    @classmethod
    def now(cls, tz=None):
      t = real_datetime.datetime.fromtimestamp(sysclock["epoch"], tz or zi)
      return t.replace(tzinfo=None) if tz is None else t

    @classmethod
    def fromtimestamp(cls, ts, tz=None):
      t = real_datetime.datetime.fromtimestamp(ts, tz or zi)
      return t.replace(tzinfo=None) if tz is None else t

  def run(cmd, shell=False, check=False):
    calls.append(cmd)
    assert cmd.startswith("TZ=UTC date -s '") and cmd.endswith("'"), cmd
    t = real_datetime.datetime.fromisoformat(cmd[len("TZ=UTC date -s '"):-1])
    sysclock["epoch"] = (t if t.tzinfo else t.replace(tzinfo=UTC)).timestamp()

  log = NS(debug=lambda *a, **k: None, exception=lambda *a, **k: None, event=lambda name, **kw: None)
  env = {
    "datetime": NS(datetime=ZonedDatetime, timedelta=real_datetime.timedelta, timezone=real_datetime.timezone),
    "subprocess": NS(run=run, CalledProcessError=real_subprocess.CalledProcessError),
    "cloudlog": log,
  }
  tree = ast.parse((REPO / "system/timed.py").read_text(encoding="utf-8"))
  body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ("set_time", "gps_datetime")]
  exec(compile(ast.Module(body=body, type_ignores=[]), str(REPO / "system/timed.py"), "exec"), env)
  assert "gps_datetime" in env, "timed.py builds GPS time with gps_datetime (aware UTC)"
  return env["set_time"], env["gps_datetime"], sysclock, calls


ZONE_CASES = [
  ("UTC", "2026-11-26T17:00:00"),
  ("America/Chicago", "2026-11-26T17:00:00"),       # UTC-6 (the device is in a US zone)
  ("America/Los_Angeles", "2026-11-26T17:00:00"),   # UTC-8
  ("Asia/Kolkata", "2026-11-26T17:00:00"),          # UTC+5:30
  ("Pacific/Auckland", "2026-11-26T17:00:00"),      # UTC+13 (southern summer time)
  ("America/New_York", "2026-11-01T05:59:55"),      # second pass crosses the fall-back boundary (06:00 UTC, 02:00 EDT -> 01:00 EST)
  ("America/New_York", "2026-03-08T06:59:55"),      # second pass crosses the spring-forward boundary (07:00 UTC)
]


@pytest.mark.parametrize("zone,gps_iso", ZONE_CASES)
@pytest.mark.parametrize("behind_s", [1100 * 86400.0, -3 * 86400.0])
def test_clock_set_to_gps_in_any_zone_and_not_set_again(zone, gps_iso, behind_s):
  gps = real_datetime.datetime.fromisoformat(gps_iso).replace(tzinfo=UTC).timestamp()
  set_time, gps_datetime, sysclock, calls = _load_zoned(zone, gps - behind_s)
  set_time(gps_datetime(gps * 1000.0))
  stamp = real_datetime.datetime.fromtimestamp(gps, UTC).strftime("%Y-%m-%d %H:%M:%S.%f")
  assert calls == [f"TZ=UTC date -s '{stamp}'"]                 # the command run: UTC wall time, TZ=UTC
  assert abs(sysclock["epoch"] - gps) < 1e-3, sysclock["epoch"] - gps
  # the next loop of timed, 10 s later (a GPS time 0.15 s old, as llk can be): no further set
  sysclock["epoch"] += 10.0
  set_time(gps_datetime((gps + 10.0 - 0.15) * 1000.0))
  assert len(calls) == 1, calls


def test_main_builds_gps_time_with_the_utc_helper():
  src = (REPO / "system/timed.py").read_text(encoding="utf-8")
  fn = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == "main")
  called = {(c.func.attr if isinstance(c.func, ast.Attribute) else getattr(c.func, "id", None))
            for c in ast.walk(fn) if isinstance(c, ast.Call)}
  assert "gps_datetime" in called and "fromtimestamp" not in called and "utcfromtimestamp" not in called
