"""Forks and "beside the record" in road memory: regression drives that pin the INSTALLED build's behaviour (1c8a450d), and two known
faults kept visible as strict xfails. SYNTHETIC roads and coordinates only.

The fork-handling feature (same-road pick at links, turn-signal rule, fork switch, drop on a switch; commits 447837fb / b85d708f) was
taken out of integ-20261126 (integ2-20261126): two independent reviews found that on the approach to a real bend on the road actually
driven it could cancel a slowdown in progress and its warning (first beside the record, then through the fork switch). The real
problem it addressed (3 cases in 132 drives) goes back to design; the two tests marked xfail below describe it.

Every PINNED drive here was run on a 1c8a450d checkout and recorded tick by tick as (metres along, published ceiling, alert, slowdowns
in progress, latched warnings); stored as the tick count plus the ticks where any of those changes (the tick grid is fixed, so this is
the whole sequence). The planner must reproduce it exactly.
"""
import numpy as np
import pytest

from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.selfdrive.car.mazda import ti_curve_plan as P

MPH = P.MPH
DT = 0.2


def cat(*trs):
  return {k: np.concatenate([t[k] for t in trs]) for k in trs[0]}


def drive(mem, path, v_mph, seconds, lat_fn=None):
  """Constant speed along `path` (a synthetic_trace dict), the fix moved sideways by lat_fn(s) -> (metres left, heading offset rad).
  -> (rows (metres along, ceiling, alert, slowdowns in progress, latched warnings) per tick, [(metres along, event)])."""
  pl = P.Planner(mem)
  s_road = np.arange(len(path["x"])) * M.SPACING_M
  rows, evs, s = [], [], 0.0
  v = v_mph * MPH
  for k in range(int(seconds / DT)):
    if s > s_road[-1]:
      break
    j = min(int(s // M.SPACING_M), len(path["x"]) - 1)
    h = float(path["hdg"][j])
    d, dh = lat_fn(s) if lat_fn is not None else (0.0, 0.0)
    x = float(np.interp(s, s_road, path["x"])) - np.sin(h) * d
    y = float(np.interp(s, s_road, path["y"])) + np.cos(h) * d
    p = pl.step(k * DT, P.Fix(True, x, y, h + dh), v, set_mph=v_mph, yaw_k=float(path["kap"][j]), ref_mph=v_mph)
    rows.append((round(s, 1), p.ceil, (p.diag or {}).get("alert"), len(pl.active), len(pl.latch)))
    evs += [(round(s, 1), e) for e in pl.events]
    pl.events.clear()
    s += v * DT
  return rows, evs


def changes(rows):
  out, prev = [], None
  for r in rows:
    if r[1:] != prev:
      out.append(r)
      prev = r[1:]
  return out


def starts_on(evs, mem, tr):
  lo, hi = int(mem.trace_start[tr]), int(mem.trace_end(tr))
  return [s for s, e in evs if e["kind"] == "start" and isinstance(e.get("point"), (int, np.integer)) and lo <= e["point"] < hi]


# ---- beside the record (independent review of integ-20261126, finding 1) ------------------------------------------------------
# A lane change, a GPS offset or a matcher miss sets road matching's "beside the record" flag; the slowdown in progress, its ceiling
# and its warning run on (dead reckoned). Synthetic straight road into a 37 mph bend (1/120 m) at 1500 m, approached at 60 mph.
PROBE_BEND_M = 1500.0


def probe_road():
  a = M.synthetic_trace([(1500, 0.0), (200, 1 / 120.0), (600, 0.0)])
  return a, M.synthetic([a], nexts=[-1])


def lane_change(s0, width=3.7, length=110.0):
  def f(s):
    if s < s0:
      return 0.0, 0.0
    if s > s0 + length:
      return width, 0.0
    u = (s - s0) / length
    return width * (3 * u * u - 2 * u ** 3), float(np.arctan(width * (6 * u - 6 * u * u) / length))
  return f


def two_lane_changes(s1, s2):
  l1, l2 = lane_change(s1), lane_change(s2)
  return lambda s: (l1(s)[0] + l2(s)[0], l1(s)[1] + l2(s)[1])


BESIDE = {
  "lane_changes_ending_10m_before_the_bend": two_lane_changes(1260, 1380),
  "gps_jump_6m_for_40m": lambda s: ((6.0, 0.0) if 1300 <= s <= 1340 else (0.0, 0.0)),
  "lane_changes_before_the_slowdown_starts": two_lane_changes(1000, 1150),
}
# 1c8a450d: (ticks, change points)
BESIDE_1C8A450D = {
  'lane_changes_ending_10m_before_the_bend': (428, [
    (0.0, None, None, 0, 0),
    (976.3, None, None, 0, 1),
    (1244.6, 50, 37, 1, 1),
    (1341.1, 39, 37, 1, 1),
    (1727.4, None, None, 0, 0),
  ]),
  'gps_jump_6m_for_40m': (428, [
    (0.0, None, None, 0, 0),
    (976.3, None, None, 0, 1),
    (1244.6, 50, 37, 1, 1),
    (1341.1, 39, 37, 1, 1),
    (1727.4, None, None, 0, 0),
  ]),
  'lane_changes_before_the_slowdown_starts': (428, [
    (0.0, None, None, 0, 0),
    (976.3, None, None, 0, 1),
    (1244.6, None, 37, 0, 1),
    (1448.4, 39, 37, 1, 1),
    (1727.4, None, None, 0, 0),
  ]),
}


@pytest.mark.parametrize("name", list(BESIDE))
def test_beside_the_record_keeps_the_installed_plan(name):
  a, mem = probe_road()
  rows, evs = drive(mem, a, 60, 120, BESIDE[name])
  assert any(e["kind"] == "rm_off" for _, e in evs), "the probe must actually put the car beside the record"
  assert (len(rows), changes(rows)) == BESIDE_1C8A450D[name]
  first_alert = next(s for s, c, al, _, _ in rows if al is not None)
  assert PROBE_BEND_M - first_alert > 250.0
  assert all(c is not None for s, c, al, _, _ in rows if PROBE_BEND_M <= s <= PROBE_BEND_M + 150.0)


# ---- a slip road recorded once beside the road driven (second review of integ-20261126) ---------------------------------------
# A ends; B (its link, the road driven) has a 37 mph bend 120 m on; C (a slip road, recorded once, not linked) starts 3.7 m right of
# B's record and diverges at 0.03 rad. The car stays on B, `off` metres right of B's record (its lane plus a GPS bias), at 60 mph.
# The installed build keeps the ceiling and the warning on every one of the 38 bend ticks.
SLIP_OFFSETS_M = (3.7, 4.5, 5.5)


def slip_road(off):
  a = M.synthetic_trace([(1000, 0.0)])
  b = M.synthetic_trace([(120, 0.0), (200, 1 / 120.0), (600, 0.0)], x0=a["x"][-1] + 5.0, y0=0.0, h0=0.0)
  c = M.synthetic_trace([(600, 0.0)], x0=a["x"][-1] + 5.0, y0=-3.7, h0=-0.03)
  mem = M.synthetic([a, b, c], nexts=[1, -1, -1])
  return mem, cat(a, b), (lambda s: (-off, 0.0)), (1005.0 + 120.0, 1005.0 + 320.0)


# 1c8a450d: (ticks, change points)
SLIP_1C8A450D = {
  3.7: (357, [
    (0.0, None, None, 0, 0),
    (600.8, None, None, 0, 1),
    (863.7, 50, None, 1, 1),
    (869.0, 50, 37, 1, 1),
    (960.2, 39, 37, 1, 1),
    (1346.5, None, None, 0, 0),
  ]),
  4.5: (357, [
    (0.0, None, None, 0, 0),
    (600.8, None, None, 0, 1),
    (863.7, 50, None, 1, 1),
    (869.0, 50, 37, 1, 1),
    (960.2, 39, 37, 1, 1),
    (1346.5, None, None, 0, 0),
  ]),
  5.5: (357, [
    (0.0, None, None, 0, 0),
    (600.8, None, None, 0, 1),
    (863.7, 50, None, 1, 1),
    (869.0, 50, 37, 1, 1),
    (960.2, 39, 37, 1, 1),
    (1346.5, None, None, 0, 0),
  ]),
}


@pytest.mark.parametrize("off", SLIP_OFFSETS_M)
def test_slip_road_beside_the_road_driven_keeps_the_installed_plan(off):
  mem, path, lat_fn, (lo, hi) = slip_road(off)
  rows, evs = drive(mem, path, 60, 90, lat_fn)
  inb = [r for r in rows if lo <= r[0] <= hi]
  assert len(inb) == 38
  assert sum(r[1] is None for r in inb) == 0, "bend ticks without a ceiling"
  assert sum(r[2] is None for r in inb) == 0, "bend ticks without the warning"
  assert (len(rows), changes(rows)) == SLIP_1C8A450D[off]


# ---- no other road: beside the branch, the slowdown runs on (dead reckoned) -----------------------------------------------------
RAMP_K = -1 / 120.0


def case_b(with_main=True):
  """A single-pass turn-off r is h1's link; with_main: the multi-pass main road goes on straight, starting 45 m past h1's end."""
  h1 = M.synthetic_trace([(2000, 0.0)])
  r = M.synthetic_trace([(100, -0.0015), (100, 0.0), (150, RAMP_K), (300, 0.0)], x0=h1["x"][-1] + 4.0, y0=0.0, h0=0.0)
  trs = [h1, r]
  if with_main:
    trs.append(M.synthetic_trace([(2500, 0.0)], x0=h1["x"][-1] + 45.0, y0=0.0, h0=0.0))
  mem = M.synthetic(trs, nexts=[1] + [-1] * (len(trs) - 1))
  lo, hi = int(mem.trace_start[1]), int(mem.trace_end(1))
  mem.npass[lo:hi] = 1                                  # the turn-off: taken once
  return h1, r, mem


# 1c8a450d: (ticks, change points)
NO_OTHER_ROAD_1C8A450D = (600, [
  (0.0, None, None, 0, 0),
  (1679.1, None, None, 0, 1),
  (1947.3, 50, 37, 1, 1),
  (2043.9, 39, 37, 1, 1),
  (2376.5, None, None, 0, 0),
])


def test_beside_the_branch_with_no_other_road_keeps_the_installed_plan():
  """The lock follows the link, the car goes straight, road matching says it is beside the record: that flag alone is not evidence
  of another road (a lane change or a GPS offset looks the same), so the slowdown runs on, dead reckoned."""
  h1, r, mem = case_b(with_main=False)
  rows, evs = drive(mem, M.synthetic_trace([(3400, 0.0)]), 60, 120)
  assert starts_on(evs, mem, 1)
  s_off = next(s for s, e in evs if e["kind"] == "rm_off")
  assert any(e["kind"] == "dead_reckon" and s == s_off for s, e in evs)
  assert (len(rows), changes(rows)) == NO_OTHER_ROAD_1C8A450D


# ---- known faults (the problem the removed fork feature addressed; back to design) ----------------------------------------------
BEND_K = 1 / 90.0


def case_a():
  """The link b leads onto the road usually turned onto (a bend 160 m in); the straight road c starts 40 m past a's end, no link."""
  a = M.synthetic_trace([(800, 0.0)])
  b = M.synthetic_trace([(60, 0.005), (100, 0.0), (150, BEND_K), (300, 0.0)], x0=a["x"][-1] + 5.0, y0=0.0, h0=0.0)
  c = M.synthetic_trace([(1500, 0.0)], x0=a["x"][-1] + 40.0, y0=0.0, h0=0.0)
  return M.synthetic([a, b, c], nexts=[1, -1, -1])


@pytest.mark.xfail(strict=True, reason="KNOWN FAULT (fork feature removed, back to design): memory follows a trace's continuation "
                                       "link onto the road usually turned onto while the car goes straight on another remembered road")
def test_known_fault_straight_on_is_slowed_for_the_linked_branch():
  mem = case_a()
  rows, evs = drive(mem, M.synthetic_trace([(2300, 0.0)]), 50, 90)
  assert not starts_on(evs, mem, 1), "slowed for a bend on the branch the car did not take"
  assert all(c is None for _, c, _, _, _ in rows)


@pytest.mark.xfail(strict=True, reason="KNOWN FAULT (fork feature removed, back to design): a turn-off taken once (the link) slows "
                                       "the car on the main road that goes on straight")
def test_known_fault_a_turn_off_taken_once_slows_the_car_on_the_main_road():
  h1, r, mem = case_b()
  rows, evs = drive(mem, M.synthetic_trace([(4400, 0.0)]), 60, 120)
  assert not starts_on(evs, mem, 1), "slowed for the turn-off's bend while staying on the main road"
  assert all(c is None for _, c, _, _, _ in rows)
