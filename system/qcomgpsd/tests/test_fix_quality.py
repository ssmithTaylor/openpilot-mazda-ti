"""The receiver's horizontal accuracy figures go to a bounded once-a-minute log line; gpsLocation.horizontalAccuracy is not
published, because locationd thresholds on it (locationd.cc handle_gps, SANE_GPS_UNCERTAINTY)."""
import ast
from pathlib import Path

from openpilot.system.qcomgpsd.fix_quality import FIELDS, COUNTS, FixQuality, WINDOW

REPO = Path(__file__).resolve().parents[3]


def _report(i):
  r = {f: float(i) for f in FIELDS.values()}
  r.update({f: 7 for f in COUNTS.values()})
  r.update({"t_DblFinalPosLatLon[0]": 0.7, "t_DblFinalPosLatLon[1]": -1.4, "q_FltFinalPosAlt": 100.0})
  return r


def test_one_summary_per_window_and_bounded():
  fq = FixQuality()
  outs = [fq.add(_report(i), has_fix=(i % 10 != 0)) for i in range(3 * WINDOW)]
  sums = [o for o in outs if o is not None]
  assert len(sums) == 3
  s = sums[0]
  assert s["reports"] == WINDOW and s["fixes"] == WINDOW - WINDOW // 10
  assert s["semimajor_m"][0] == 1.0 and s["semimajor_m"][2] == WINDOW - 1
  assert s["gps_used"] == [7, 7, 7]
  assert all(len(v) == 0 for v in fq.vals.values())


def test_summary_carries_no_position():
  fq = FixQuality(window=2)
  fq.add(_report(1), True)
  s = fq.add(_report(2), True)
  text = repr(s).lower()
  assert "lat" not in text and "lon" not in text and "alt" not in text.replace("vert", "")


def test_no_fix_window_gives_empty_stats():
  fq = FixQuality(window=3)
  for _ in range(2):
    assert fq.add(_report(1), False) is None
  s = fq.add(_report(1), False)
  assert s["fixes"] == 0 and s["hdop"] is None


def test_horizontal_accuracy_is_not_published():
  """If this ever changes, every consumer must be re-checked first: locationd rejects a fix when
  norm(horizontalAccuracy, verticalAccuracy) >= SANE_GPS_UNCERTAINTY."""
  tree = ast.parse((REPO / "system/qcomgpsd/qcomgpsd.py").read_text(encoding="utf-8"))
  assigned = {t.attr for n in ast.walk(tree) if isinstance(n, ast.Assign) for t in n.targets if isinstance(t, ast.Attribute)}
  assert "horizontalAccuracy" not in assigned
  assert "getHorizontalAccuracy" in (REPO / "selfdrive/locationd/locationd.cc").read_text(encoding="utf-8")
