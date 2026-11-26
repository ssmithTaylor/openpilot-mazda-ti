"""Replayability guard for the steering alerts outside curve slowing (owner, 2026-11-25: "Let's make sure we're properly logging
information about our features so that we can enhance them after the fact"; 2026-11-26 item 6). Sibling of
test_mazda_ti_curve_replayable.py: fails when the lane-drift gate gains a piece of state that is neither in its lane_gate record nor
in the commented exemption list (lane_drift.LANE_GATE_LOGGED / LANE_GATE_EXEMPT). Adding one: log it, or exempt it with the reason a
replay does not need it. Synthetic inputs only."""
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib import lane_drift as LD


def _model(i):
  ly = -1.8 + 0.7 * ((i // 30) % 2)
  return NS(laneLines=[NS(y=[ly - 3.6]), NS(y=[ly]), NS(y=[ly + 3.6]), NS(y=[ly + 7.2])],
            laneLineProbs=[0.1, 0.9 if i % 7 else 0.35, 0.8, 0.1], roadEdges=[NS(y=[ly - 4.0]), NS(y=[ly + 7.6])],
            roadEdgeStds=[0.3, 0.3], meta=NS(laneChangeState=0), action=NS(desiredCurvature=0.004 if i % 100 < 60 else 0.0))


def test_every_lane_gate_attribute_is_logged_or_exempt():
  g = LD.LaneDriftGate(0.05)
  for i in range(400):
    m = _model(i)
    g.update(m, m.action.desiredCurvature, i % 50 < 35, i % 200 == 199)
  names = set(vars(g))
  missing = sorted(n for n in names if n not in LD.LANE_GATE_LOGGED and n not in LD.LANE_GATE_EXEMPT)
  assert not missing, f"LaneDriftGate state not in the lane_gate record nor exempt: {missing}"
  stale = sorted(n for n in list(LD.LANE_GATE_LOGGED) + list(LD.LANE_GATE_EXEMPT) if n not in names)
  assert not stale, f"listed but no longer an attribute: {stale}"
  assert not set(LD.LANE_GATE_LOGGED) & set(LD.LANE_GATE_EXEMPT)


def test_lane_gate_record_carries_every_logged_key():
  seen = []
  g, log = LD.LaneDriftGate(0.05), LD.OnChange(20, 5)
  saved = LD.emit_event
  LD.emit_event = lambda name, rec: seen.append(rec)
  try:
    for i in range(200):
      m = _model(i)
      out = g.update(m, m.action.desiredCurvature, True, False)
      LD.log_lane_gate(g, log, "c", out, True, False)
  finally:
    LD.emit_event = saved
  assert seen
  keys = set(seen[-1])
  for where in LD.LANE_GATE_LOGGED.values():
    key = where.split(" ")[0]
    assert key == "bit" or key == "bits" or key in keys, (where, keys)
  assert len(LD.LANE_GATE_BITS) == 8 and max(r["s"] for r in seen) < 256
