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
