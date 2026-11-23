"""The TI diagnostic server's snapshot loop: refreshed at most twice a second (it used to turn over on every carState message), with
the park check and the live tools reading the same values as before."""
from types import SimpleNamespace as NS

from openpilot.frogpilot.system.ti_mcp import ti_mcp_server as m


class FakeSM:
  """A SubMaster whose update takes `cost` simulated seconds (a 100 Hz service answers at once; offroad it waits for its timeout)."""

  def __init__(self, clock, cost=0.0, gear="park", v=0.0, standstill=True):
    self.clock, self.cost, self.updates = clock, cost, 0
    self.d = {
      "carState": NS(vEgo=v, gearShifter=gear, standstill=standstill, steeringAngleDeg=1.5, steeringTorque=-2.0,
                     steerFaultTemporary=False),
      "carControl": NS(latActive=v > 0, actuators=NS(steer=0.1)),
      "controlsState": NS(curvature=0.001, desiredCurvature=0.0012),
      "liveTorqueParameters": NS(latAccelFactorFiltered=2.5, frictionCoefficientFiltered=0.1, liveValid=True, useParams=True,
                                 totalBucketPoints=1000.0, decay=50.0),
      "liveParameters": NS(steerRatio=15.0),
      "liveDelay": NS(lateralDelay=0.2, lateralDelayEstimate=0.21, status="estimated", validBlocks=5),
    }

  def update(self, timeout):
    self.updates += 1
    self.clock[0] += self.cost

  def __getitem__(self, k):
    return self.d[k]


def _run(cost, cycles):
  clock = [100.0]
  snap = m.Snapshot()
  sm = FakeSM(clock, cost)

  def sleep(dt):
    assert dt > 0
    clock[0] += dt
  snap.run(sm=sm, clock=lambda: clock[0], sleep=sleep, cycles=cycles)
  return snap, sm, clock[0] - 100.0


def test_the_snapshot_is_refreshed_at_most_twice_a_second_when_messages_arrive_at_100_hz():
  snap, sm, elapsed = _run(cost=0.01, cycles=40)
  assert sm.updates == 40
  assert sm.updates / elapsed <= 2.0 + 1e-9                 # was 100 per second: one turn per carState message
  assert elapsed >= 40 * m.SNAPSHOT_PERIOD_S - 1e-9
  assert snap.get()["available"] is True


def test_a_slow_update_is_not_followed_by_a_further_wait():
  _, sm, elapsed = _run(cost=1.0, cycles=5)                  # offroad: each update waits out its 1 s timeout
  assert sm.updates == 5 and abs(elapsed - 5.0) < 1e-9


def test_the_snapshot_period_stays_well_inside_the_park_checks_freshness_limit():
  assert m.SNAPSHOT_PERIOD_S <= m.SNAPSHOT_MAX_AGE_S / 4


def _with_snapshot(monkeypatch, gear, v, standstill, onroad):
  snap = m.Snapshot()
  snap._refresh(FakeSM([0.0], gear=gear, v=v, standstill=standstill))
  monkeypatch.setattr(m, "snapshot", snap)
  monkeypatch.setattr(m, "params", NS(get_bool=lambda k: onroad if k == "IsOnroad" else False, get=lambda k: None))
  monkeypatch.setattr(m, "params_memory", NS(get=lambda k: None))
  return snap


def test_parked_the_heavy_tools_still_run_and_in_motion_they_are_still_refused(monkeypatch):
  _with_snapshot(monkeypatch, "park", 0.0, True, onroad=True)
  assert m._is_parked() and m._refuse_if_driving("analyze_segment") is None
  _with_snapshot(monkeypatch, "drive", 12.0, False, onroad=True)
  assert not m._is_parked() and "refused" in m._refuse_if_driving("analyze_segment")["error"]
  _with_snapshot(monkeypatch, "drive", 12.0, False, onroad=False)
  assert m._refuse_if_driving("analyze_segment") is None     # offroad: never refused


def test_the_live_tool_reads_the_same_fields_from_the_snapshot(monkeypatch):
  _with_snapshot(monkeypatch, "drive", 12.0, False, onroad=True)
  out = m.tool_ti_status({})
  assert out["live_available"] is True and out["speed_ms"] == 12.0 and out["engaged"] is True and out["driver_torque"] == -2.0
  live = m.snapshot.get()
  assert live["gear_shifter"] == "drive" and live["learned_lateral_delay"] == 0.2 and live["curvature_error"] == 0.0002
