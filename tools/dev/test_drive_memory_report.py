"""tools/dev/drive_memory_report.py: the slope / verdict / projection arithmetic on synthetic series (no real logs needed)."""
import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location("drive_memory_report_for_test", Path(__file__).with_name("drive_memory_report.py"))
R = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(R)


def test_slope_is_least_squares_and_needs_points():
  xs = [0, 60, 120, 180, 240, 300]
  assert R.slope(xs, [10 + 0.5 * x for x in xs]) == pytest.approx(0.5)
  assert R.slope([0, 1, 2], [0, 1, 2]) is None              # under MIN_POINTS
  assert R.slope([5] * 6, list(range(6))) is None           # no spread in x


def test_verdict_flags_only_over_50_mb_per_hour():
  assert "FLAG" in R.verdict("controlsd", 1200.0)
  assert "FLAG" not in R.verdict("controlsd", 49.9) and "ok" in R.verdict("controlsd", -3.0)
  assert "not enough" in R.verdict("x", None)
  assert "too short" in R.verdict("controlsd", 1200.0, span_min=1.0)            # one segment: never flagged


def test_projection_to_90_percent():
  t = [k * 60.0 for k in range(11)]                         # 10 minutes, 1 % per minute from 50 %
  per_h, mins = R.minutes_to(t, [50.0 + k for k in range(11)])
  assert per_h == pytest.approx(60.0) and mins == pytest.approx(30.0)
  assert R.minutes_to(t, [60.0] * 11) == (0.0, None)        # flat: no projection


def test_summary_of_a_leaking_controlsd_and_a_flat_card_with_engaged_minutes():
  t = [k * 10.0 for k in range(361)]                        # one hour, a sample every 10 s
  leak = [(x, 300e6 + x * (1500e6 / 3600.0)) for x in t]    # 1.5 GB/h
  flat = [(x, 200e6 + (5e5 if (x // 10) % 2 else 0.0)) for x in t]
  ctl = [(x, x < 1800.0, 0.0) for x in t]                   # engaged the first half hour only
  dev = [(x, 40.0 + x / 3600.0 * 30.0, None) for x in t]    # 30 %/h
  s = R.summarise({"controlsd": leak, "card": flat}, dev, ctl, [(0.0, 3e9), (3600.0, 1e9)])
  c = s["processes"]["controlsd"]
  assert c["MB_per_h"] == pytest.approx(1500.0, rel=1e-3) and c["start_MB"] == pytest.approx(300.0)
  assert s["processes"]["card"]["MB_per_h"] == pytest.approx(0.0, abs=1.0)
  assert s["flagged"] == ["controlsd"] and s["engaged_min"] == pytest.approx(30.0, abs=0.2)
  assert s["mem_pct_per_h"] == pytest.approx(30.0) and s["minutes_to_90"] == pytest.approx(40.0, abs=0.5)
  text = R.report(s)
  assert "VERDICT: FLAG controlsd" in text and "projected 90 % in" in text


def test_engaged_clock_counts_only_engaged_steps():
  assert R.engaged_clock([0, 1, 2, 3, 4], [True, True, False, False, True]) == [0, 1, 2, 2, 2]


def test_reader_never_touches_other_event_types(tmp_path, monkeypatch):
  """Position-free by construction: an event of any other type is skipped on its type alone, before any field is read."""
  class Ev:
    def __init__(self, w):
      self.w = w

    def which(self):
      return self.w

    def __getattr__(self, name):
      raise AssertionError(f"read {name} of a {self.w} event")
  seg = tmp_path / "r--0"
  seg.mkdir()
  (seg / "rlog").write_bytes(b"x")
  import types
  fake_log = types.SimpleNamespace(Event=types.SimpleNamespace(read_multiple_bytes=lambda b: [Ev("gpsLocationExternal"),
                                                                                              Ev("liveLocationKalman"), Ev("carParams")]))
  import sys
  monkeypatch.setitem(sys.modules, "cereal", types.SimpleNamespace(log=fake_log))
  series, dev, ctl, avail = R.read_drive(tmp_path)
  assert series == {} and dev == [] and ctl == []
