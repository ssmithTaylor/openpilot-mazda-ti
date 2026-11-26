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


# ---- Running Wide / Mid-Corner / Steering Assist Dropped (ti_authority) -------------------------------------------------------------
# FrogPilot's own FrogPilotEvents state (upstream, not this fork's alerts): not part of the guard.
UPSTREAM_EVENTS_STATE = {"frogpilot_planner", "theme_manager", "error_log", "events", "always_on_lateral_enabled_previously",
                         "previous_traffic_mode", "random_event_playing", "startup_seen", "stopped_for_light", "max_acceleration",
                         "random_event_timer", "tracked_lead_distance", "played_events"}


def test_every_steering_alert_attribute_of_frogpilot_events_is_logged_or_exempt():
  from openpilot.selfdrive.car.mazda import ti_mem_check as MC
  from openpilot.frogpilot.controls.lib import steer_authority as SA
  step, teardown = MC.setup_frogpilot_events()
  try:
    for i in range(1300):
      step(i)
    fe = step.__closure__ and next(c.cell_contents for c in step.__closure__ if type(c.cell_contents).__name__ == "FrogPilotEvents")
    names = set(vars(fe)) - UPSTREAM_EVENTS_STATE
  finally:
    teardown()
  assert step.seen.get("rec:ti_authority"), step.seen
  missing = sorted(n for n in names if n not in SA.AUTHORITY_EVENTS_STATE and n not in SA.AUTHORITY_EVENTS_EXEMPT)
  assert not missing, f"FrogPilotEvents alert state not in the ti_authority record nor exempt: {missing}"
  stale = sorted(n for n in list(SA.AUTHORITY_EVENTS_STATE) + list(SA.AUTHORITY_EVENTS_EXEMPT) if n not in names)
  assert not stale, f"listed but no longer an attribute: {stale}"


def test_every_steering_alert_attribute_of_the_planner_is_logged_or_exempt():
  """FrogPilotPlanner's TI attributes (named steer_* / latch_* / curve_* / advisory_*): read from the source, as the planner needs
  the whole FrogPilot stack to construct."""
  import re
  from pathlib import Path
  from openpilot.frogpilot.controls.lib import steer_authority as SA
  src = (Path(SA.__file__).resolve().parents[1] / "frogpilot_planner.py").read_text(encoding="utf-8")
  names = set(re.findall(r"self\.((?:steer|latch|curve|advisory)_\w+)\s*[=,)]", src))
  names |= set(re.findall(r'getattr\(self, "((?:steer|latch|curve|advisory)_\w+)"', src))
  assert {"latch_frames", "latch_bits", "steer_latched"} <= names, names
  missing = sorted(n for n in names if n not in SA.AUTHORITY_PLANNER_STATE and n not in SA.AUTHORITY_PLANNER_EXEMPT)
  assert not missing, f"FrogPilotPlanner alert state not in the ti_authority record nor exempt: {missing}"
  assert not set(SA.AUTHORITY_PLANNER_STATE) & set(SA.AUTHORITY_PLANNER_EXEMPT)


# ---- stuck-request guard ------------------------------------------------------------------------------------------------------------
def test_every_stuck_guard_attribute_is_logged_or_exempt():
  from openpilot.selfdrive.car.mazda import ti_stuck_guard as SG
  g = SG.StuckGuard()
  for k in range(300):
    g.update(300 if k < 250 else 0, k % 120 != 119, now=k * 0.01)
  names = set(vars(g))
  missing = sorted(n for n in names if n not in SG.STUCK_LOGGED and n not in SG.STUCK_EXEMPT)
  assert not missing, f"StuckGuard state not in the ti_stuck_guard event nor exempt: {missing}"
  stale = sorted(n for n in list(SG.STUCK_LOGGED) + list(SG.STUCK_EXEMPT) if n not in names)
  assert not stale, f"listed but no longer an attribute: {stale}"
  assert not set(SG.STUCK_LOGGED) & set(SG.STUCK_EXEMPT)
