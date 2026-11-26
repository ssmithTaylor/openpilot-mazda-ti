"""Replayability of the steering alerts' state (owner, 2026-11-26, item 6: "Logging the last unlogged pieces: the small counters behind
the 'Running Wide' and 'Mid-Corner' alerts and the lane-alert gate's state"). Records are written on a CHANGE of discrete state only
(never of an analogue value that moves every frame), bounded in size and rate, position free, and logging can never raise into the
loop that calls it. Synthetic inputs only."""
import json
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.controls.lib import lane_drift as LD

RIGHT_TURN = 0.004


def model(left_y, right_y, p=0.9, k=0.0, lcs=0):
  return NS(laneLines=[NS(y=[left_y - 3.6]), NS(y=[left_y]), NS(y=[right_y]), NS(y=[right_y + 3.6])],
            laneLineProbs=[0.1, p, p, 0.1], roadEdges=[NS(y=[left_y - 4.0]), NS(y=[right_y + 4.0])], roadEdgeStds=[9.0, 9.0],
            meta=NS(laneChangeState=lcs), action=NS(desiredCurvature=k))


@pytest.fixture
def records(monkeypatch):
  out = []
  monkeypatch.setattr(LD, "emit_event", lambda name, rec: out.append((name, rec)))
  return out


def _run(gate, log, models, at_limit=True, proc="c"):
  for m in models:
    out = gate.update(m, m.action.desiredCurvature, at_limit, False)
    LD.log_lane_gate(gate, log, proc, out, at_limit, False)


def test_lane_gate_varying_but_not_crossing_gives_one_record_and_a_crossing_one_more(records):
  g, log = LD.LaneDriftGate(0.05), LD.OnChange(20, 5)
  # in a right bend at the limit, the outside (left) line wanders 0.5-0.6 m from the car's side every frame: never within 0.30 m
  _run(g, log, [model(-1.8 - 0.05 * math.sin(i), 1.8, k=RIGHT_TURN) for i in range(200)])
  assert len(records) == 1, records
  name, rec = records[0]
  assert name == "lane_gate" and rec["p"] == "c" and LD.lane_gate_bits(rec["s"])["out"] is False
  # the car's side comes within 0.2 m of the outside line: the gate opens and latches: one more record
  _run(g, log, [model(-1.12, 2.48, k=RIGHT_TURN)] * 50)
  assert len(records) == 2, records
  bits = LD.lane_gate_bits(records[1][1]["s"])
  assert bits["out"] and bits["latched"] and bits["referenced"]
  assert abs(records[1][1]["c"] - (1.12 - LD.CAR_HALF_WIDTH)) < 0.01


def test_lane_gate_record_is_bounded_and_position_free(records):
  g, log = LD.LaneDriftGate(0.05), LD.OnChange(20, 5)
  _run(g, log, [model(-1.12, 2.48, k=RIGHT_TURN)] * 20, proc="f")
  for _, rec in records:
    text = json.dumps(rec)
    assert len(text) <= LD.LANE_GATE_MAX_B, (len(text), text)
    for bad in ("lat\"", "lon", "latitude", "longitude", "vin", "name"):
      assert bad not in text.lower()


def test_lane_gate_flapping_is_rate_capped_and_the_cap_is_counted(records):
  g, log = LD.LaneDriftGate(0.05), LD.OnChange(20, 5)
  # a line flickering across the confidence threshold every frame, 3 s
  _run(g, log, [model(-1.8, 1.8, p=0.9 if i % 2 else 0.2) for i in range(60)], at_limit=False)
  assert len(records) <= 3 * 5 + 1
  assert sum(r["sup"] for _, r in records) > 0 or log.sup > 0


def test_lane_gate_logging_cannot_raise(monkeypatch):
  def boom(name, rec):
    raise RuntimeError("log down")
  monkeypatch.setattr(LD, "emit_event", boom)
  g, log = LD.LaneDriftGate(0.05), LD.OnChange(20, 5)
  _run(g, log, [model(-1.12, 2.48, k=RIGHT_TURN)] * 5)         # no exception reaches the caller
  assert log.dead                                              # and logging stays off for the rest of the process


def test_lane_gate_logging_does_not_change_the_gate():
  """The gate's answers with and without the record step are identical (logging only reads)."""
  ms = [model(-1.8 + 0.01 * i, 1.8 + 0.01 * i, p=0.4 + 0.5 * (i % 3 == 0), k=RIGHT_TURN * (1 if i % 50 < 30 else -1))
        for i in range(300)]
  a, b, log = LD.LaneDriftGate(0.05), LD.LaneDriftGate(0.05), LD.OnChange(20, 5)
  for i, m in enumerate(ms):
    lim = i % 40 < 25
    ra = a.update(m, m.action.desiredCurvature, lim, False)
    rb = b.update(m, m.action.desiredCurvature, lim, False)
    LD.log_lane_gate(b, log, "c", rb, lim, False)
    assert ra == rb and a.latch_left == b.latch_left


# ---- ti_authority: the counters behind "Running Wide - Slow Down" and "Steering Has Stopped Mid-Corner" -----------------------------
from openpilot.frogpilot.controls.lib import steer_authority as SA  # noqa: E402


def _fe():
  """The parts of FrogPilotEvents / FrogPilotPlanner log_authority reads."""
  fe = NS(runwide_frames=0, runwide_display_hold=0, latch_display_hold=0, ti_dropout_hold=0, lane_drift_gate=LD.LaneDriftGate(0.05))
  return fe, NS(latch_frames=0, latch_bits=0)


def _key(**kw):
  a = dict(lat_active=True, ti_now=True, base=False, drift=False, rw_shown=False, latch_cond=False, latched=False, lt_shown=False,
           dd_shown=False, adv_on=True)
  a.update(kw)
  return SA.authority_key(**a)


def test_authority_varying_but_not_crossing_gives_one_record_and_a_crossing_one_more(records):
  fe, pl = _fe()
  log = LD.OnChange(20, 5)
  # steering near but under its ceiling, speed, asks and column load moving every frame: no discrete change
  for i in range(200):
    SA.log_authority(log, _key(), fe, pl, 25.0 + 0.1 * math.sin(i), 2.0 + 0.2 * math.sin(i), 1.9, 0.9 + 0.03 * math.sin(i),
                     3.0 * math.sin(i), 150.0 + 10 * math.sin(i))
  assert len(records) == 1, records
  assert records[0][0] == "ti_authority"
  # the latch condition is met (at the clip, wheel stopped, column loaded): one more
  pl.latch_bits, pl.latch_frames = 7, 1
  for i in range(3):
    SA.log_authority(log, _key(latch_cond=True), fe, pl, 25.0, 2.3, 2.25, 1.0, 0.5, 200.0)
    pl.latch_frames += 1
  assert len(records) == 2, records
  rec = records[1][1]
  assert SA.authority_bits(rec["s"])["latch_cond"] and rec["lb"] == 7 and rec["ltf"] == 1


def test_authority_record_says_why_mid_corner_fired(records):
  """A Mid-Corner record carries what is needed afterwards to tell a real stall from a false one: the command, wheel rate and
  column load the latch used, the asked and delivered lateral acceleration (delivering what was asked = not losing the bend) and
  the car side's clearance to the outside line."""
  fe, pl = _fe()
  log = LD.OnChange(20, 5)
  m = NS(laneLines=[NS(y=[-5.0]), NS(y=[-1.3]), NS(y=[2.3]), NS(y=[6.0])], laneLineProbs=[0.1, 0.9, 0.9, 0.1],
         roadEdges=[NS(y=[-6.0]), NS(y=[7.0])], roadEdgeStds=[9.0, 9.0], meta=NS(laneChangeState=0), action=NS(desiredCurvature=0.004))
  fe.lane_drift_gate.update(m, 0.004, True, False)
  pl.latch_bits, pl.latch_frames = 7, SA.LATCH_DEBOUNCE_FRAMES
  SA.log_authority(log, _key(latch_cond=True, latched=True, lt_shown=True), fe, pl, 24.6, 2.31, 2.28, 1.0, 0.4, 203.0)
  rec = records[-1][1]
  for k in ("st", "sr", "col", "des", "act", "oc", "v", "lb", "ltf", "ltd", "rwf", "rwd", "dd"):
    assert k in rec, k
  assert abs(rec["oc"] - (1.3 - LD.CAR_HALF_WIDTH)) < 0.01 and rec["des"] == 2.31 and rec["act"] == 2.28
  text = json.dumps(rec)
  assert len(text) <= SA.AUTHORITY_MAX_B, (len(text), text)
  for bad in ("lat\"", "lon", "latitude", "longitude", "vin"):
    assert bad not in text.lower()


def test_authority_logging_cannot_raise(monkeypatch):
  def boom(name, rec):
    raise RuntimeError("log down")
  monkeypatch.setattr(LD, "emit_event", boom)
  fe, pl = _fe()
  log = LD.OnChange(20, 5)
  SA.log_authority(log, _key(), fe, pl, 25.0, 2.0, 1.9, 0.9, 1.0, 100.0)
  SA.log_authority(log, _key(), NS(), NS(), 25.0, 2.0, 1.9, 0.9, 1.0, 100.0)     # even a broken host
  assert log.dead


# ---- stuck-request guard: the record says how each logged nudge was triggered ----------------------------------------------------
def test_stuck_guard_burst_records_the_run_and_the_clock_hold():
  """The ti_stuck_guard event stays at most one per 10 s (the cumulative `bursts` already counts the unlogged ones and every sent
  value is in sendcan); each record now also says how long the value had been held when the burst fired, in frames ("run") and by
  the clock ("held_s"): run under STUCK_HOLD_FRAMES with held_s at the hold = the control loop was late (the real-time arm fired)."""
  from openpilot.selfdrive.car.mazda.ti_stuck_guard import StuckGuard, STUCK_HOLD_FRAMES, STUCK_HOLD_S
  g = StuckGuard()
  for k in range(STUCK_HOLD_FRAMES + 2):
    g.update(300, True, now=k * 0.01)
    if g.burst_started:
      break
  assert g.burst_started and g.fired_run == STUCK_HOLD_FRAMES and abs(g.fired_s - STUCK_HOLD_FRAMES * 0.01) < 1e-6
  g = StuckGuard()                                   # a late loop: 0.05 s per frame
  for k in range(40):
    g.update(-570, True, now=k * 0.05)
    if g.burst_started:
      break
  assert g.burst_started and g.fired_run < STUCK_HOLD_FRAMES and g.fired_s >= STUCK_HOLD_S
  g = StuckGuard()                                   # no clock: the frame count, no clock figure
  for _ in range(STUCK_HOLD_FRAMES + 1):
    g.update(300, True)
  assert g.burst_started and g.fired_run == STUCK_HOLD_FRAMES and g.fired_s is None


def test_controller_stuck_event_carries_run_and_held_s(monkeypatch):
  from openpilot.selfdrive.car.mazda import carcontroller as ccm
  from openpilot.selfdrive.car.tests import test_mazda_ti_stuck_guard as T
  events = []
  monkeypatch.setattr(ccm, "ti_stuck_log", lambda ev: events.append(ev))
  mod, cc, sent, toggles = T._controller(650)
  CC, CS = mod.inputs(45.0, 0.0, steer=1.0)
  for k in range(1200):
    cc.update(CC, CS, k * 10_000_000, toggles)
  assert events and all("run" in e and "held_s" in e for e in events)
  assert len(json.dumps(events[0])) <= 160
