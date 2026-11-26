"""Forks in road memory (owner, 2026-11-25: "I wonder if we should just assume that the driver is going to continue on the same road
until they don't"). SYNTHETIC roads only, built to reproduce the two measured cases of the 132-drive replay:
  (a) the look-ahead followed a trace's continuation link onto the road usually turned onto, while the car went straight on another
      remembered road that starts just past the fork (not linked);
  (b) a turn-off taken once (a single-pass branch) is the link; the multi-pass road goes on straight; road matching flagged the car
      beside the branch but the slowdown stayed (dead-reckoned to the bend's end).
The planner plans along the SAME road at a fork (the straightest continuation; the turn signal toward a branch makes it the plan), and
a slowdown planned on the road not taken is dropped once the car is positively matched on the other road past the fork (fork_switch).
Being merely beside the record (a lane change, a GPS offset, a matcher miss) never drops anything (review of integ-20261126, finding 1).
"""
import numpy as np
import pytest

from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.selfdrive.car.mazda import ti_curve_plan as P

MPH = P.MPH
DT = 0.2


def run(mem, path, v_mph, seconds, blink_side_fn=None, planner=None):
  """Constant speed along `path` (a synthetic_trace dict). -> list of (t, along m, Plan, events of the tick, planner)."""
  pl = planner or P.Planner(mem)
  s_road = np.arange(len(path["x"])) * M.SPACING_M
  out, s = [], 0.0
  v = v_mph * MPH
  for k in range(int(seconds / DT)):
    t = k * DT
    if s > s_road[-1]:
      break
    j = min(int(s // M.SPACING_M), len(path["x"]) - 1)
    fix = P.Fix(True, float(np.interp(s, s_road, path["x"])), float(np.interp(s, s_road, path["y"])), float(path["hdg"][j]))
    bs = blink_side_fn(t, s) if blink_side_fn is not None else 0
    p = pl.step(t, fix, v, set_mph=v_mph, yaw_k=float(path["kap"][j]), ref_mph=v_mph, blinker=bool(bs), blink_side=bs)
    out.append((t, s, p, list(pl.events), pl))
    pl.events.clear()
    s += v * DT
  return out


def starts_on(res, mem, tr):
  lo, hi = int(mem.trace_start[tr]), int(mem.trace_end(tr))
  return [(t, s) for t, s, p, ev, _ in res for e in ev if e["kind"] == "start" and isinstance(e.get("point"), (int, np.integer))
          and lo <= e["point"] < hi]


def cat(*trs):
  return {k: np.concatenate([t[k] for t in trs]) for k in trs[0]}


# ---- case (a): the link leads onto the road usually turned onto; the car goes straight onto another remembered road ------------
BEND_K = 1 / 90.0


def case_a():
  a = M.synthetic_trace([(800, 0.0)])
  b = M.synthetic_trace([(60, 0.005), (100, 0.0), (150, BEND_K), (300, 0.0)], x0=a["x"][-1] + 5.0, y0=0.0, h0=0.0)
  c = M.synthetic_trace([(1500, 0.0)], x0=a["x"][-1] + 40.0, y0=0.0, h0=0.0)   # the straight road, starting past the fork, no link
  mem = M.synthetic([a, b, c], nexts=[1, -1, -1])
  return a, b, c, mem


def test_a_straight_on_is_never_slowed_for_the_linked_branch():
  a, b, c, mem = case_a()
  res = run(mem, M.synthetic_trace([(2300, 0.0)]), 50, 90)
  assert not starts_on(res, mem, 1), "slowed for a bend on the branch the car did not take"
  assert all(p.ceil is None for _, _, p, _, _ in res)
  # the lock followed the same road through the fork: trusted on the straight road past it, never lost there
  past = [(p, pl) for t, s, p, ev, pl in res if 900 < s < 2000]
  assert past and all(p.trusted for p, _ in past)
  assert past[-1][1].trace == 2
  # the decision is in the tick record (index, why), not as a position
  fks = [p.diag.get("fk") for t, s, p, ev, _ in res if p.diag and p.diag.get("fk")]
  assert fks and fks[0][0] == P.FORK_STRAIGHT and fks[0][1] == 2 and fks[0][2] == 1


def test_a_without_the_rule_the_branch_is_slowed_for(monkeypatch):
  """The failure the rule fixes, reproduced: with FORK_ON off the planner slows for the branch's bend (today's behaviour)."""
  monkeypatch.setattr(P, "FORK_ON", False)
  a, b, c, mem = case_a()
  res = run(mem, M.synthetic_trace([(2300, 0.0)]), 50, 90)
  assert starts_on(res, mem, 1)


def first_ceiling_s(res):
  return next((s for t, s, p, ev, _ in res if p.ceil is not None and p.kind == "corner"), None)


def test_a_taking_the_branch_unsignalled_is_still_slowed_for_its_bend():
  """The cost of the rule: a driver who takes the remembered branch without signalling. The plan moves onto the branch as soon as
  the car is matched on it, and the slowdown for its bend still starts before the bend."""
  a, b, c, mem = case_a()
  path = cat(a, b)
  res = run(mem, path, 50, 90)
  s_first = first_ceiling_s(res)
  bend_entry = 800 + 60 + 100
  assert s_first is not None and s_first < bend_entry, s_first
  assert any(e["kind"] == "fork_switch" for _, _, _, ev, _ in res for e in ev)


def test_a_signal_toward_the_branch_makes_it_the_plan():
  a, b, c, mem = case_a()
  path = cat(a, b)
  base = first_ceiling_s(run(mem, path, 50, 90))
  sig = run(mem, path, 50, 90, blink_side_fn=lambda t, s: 1 if 500 < s < 800 else 0)    # left: the branch turns left
  s_sig = first_ceiling_s(sig)
  assert s_sig is not None and s_sig < base
  fks = [p.diag.get("fk") for t, s, p, ev, _ in sig if p.diag and p.diag.get("fk")]
  assert any(f[0] == P.FORK_SIGNAL and f[2] == 0 for f in fks)
  # a signal the OTHER way is not toward the branch: the plan stays on the straight road
  other = run(mem, M.synthetic_trace([(2300, 0.0)]), 50, 90, blink_side_fn=lambda t, s: -1 if 500 < s < 800 else 0)
  assert not starts_on(other, mem, 1)


# ---- case (b): a single-pass turn-off is the link; the multi-pass road continues straight ---------------------------------------
RAMP_K = -1 / 120.0


def case_b(with_main=True):
  h1 = M.synthetic_trace([(2000, 0.0)])
  r = M.synthetic_trace([(100, -0.0015), (100, 0.0), (150, RAMP_K), (300, 0.0)], x0=h1["x"][-1] + 4.0, y0=0.0, h0=0.0)
  trs = [h1, r]
  if with_main:
    trs.append(M.synthetic_trace([(2500, 0.0)], x0=h1["x"][-1] + 45.0, y0=0.0, h0=0.0))
  mem = M.synthetic(trs, nexts=[1] + [-1] * (len(trs) - 1))
  lo, hi = int(mem.trace_start[1]), int(mem.trace_end(1))
  mem.npass[lo:hi] = 1                                  # the turn-off: taken once
  return h1, r, mem


def test_b_staying_on_the_main_road_is_never_slowed_for_the_single_pass_branch():
  h1, r, mem = case_b()
  res = run(mem, M.synthetic_trace([(4400, 0.0)]), 60, 120)
  assert not starts_on(res, mem, 1)
  assert all(p.ceil is None for _, _, p, _, _ in res)


def test_b_without_the_rule_the_branch_is_slowed_for(monkeypatch):
  monkeypatch.setattr(P, "FORK_ON", False)
  h1, r, mem = case_b()
  res = run(mem, M.synthetic_trace([(4400, 0.0)]), 60, 120)
  assert starts_on(res, mem, 1)


def test_b_beside_the_record_with_no_other_road_runs_on_dead_reckoned():
  """No other remembered road to follow (the same-road rule cannot help, and no fork switch can happen): the lock follows the branch,
  the car goes straight, road matching says it is beside the record. That flag alone is not evidence of another road (a lane change
  or a GPS offset looks the same), so the slowdown runs on, dead reckoned, as before the fork work (it used to be dropped here)."""
  h1, r, mem = case_b(with_main=False)
  res = run(mem, M.synthetic_trace([(3400, 0.0)]), 60, 120)
  assert starts_on(res, mem, 1), "the branch's slowdown starts before the fork (the look-ahead had nothing else)"
  k_off = next(k for k, (t, s, p, ev, _) in enumerate(res) if any(e["kind"] == "rm_off" for e in ev))
  assert not any(e["kind"] == "rm_drop" for _, _, _, ev, _ in res for e in ev)
  assert any(e["kind"] == "dead_reckon" for e in res[k_off][3])
  assert any(p.ceil is not None for t, s, p, ev, _ in res[k_off + 1:])


def test_b_signal_toward_the_turn_off_keeps_it_planned():
  h1, r, mem = case_b()
  path = cat(h1, r)
  res = run(mem, path, 60, 120, blink_side_fn=lambda t, s: -1 if 1500 < s < 2000 else 0)   # right: the ramp leaves right
  assert starts_on(res, mem, 1)


def test_a_car_in_a_bend_keeps_its_slowdown_on_a_road_switch():
  """The one exception to the fork switch's drop: the car is cornering itself (its yaw-sensor in-bend state): a slowdown is never
  dropped inside a bend the car is actually in (review C1). The slowdown is for a bend on the branch; the switch is onto the main
  road (trace 0 here: the only other one)."""
  h1, r, mem = case_b(with_main=False)
  pl = P.Planner(mem)
  key = int(mem.trace_start[1]) + 20
  pl.active[key] = {"T": 40, "need": 40, "Tc": 40, "at": 1e9, "entry": 0.0, "mode": "push", "since": 0.0, "start_t": 0.0, "phase": 1}
  pl.bend_on = True
  pl._drop_for_switch(0, "matched on the other road at a fork")
  assert key in pl.active
  assert any(e["kind"] == "dead_reckon" for e in pl.events)
  pl.bend_on = False
  pl._drop_for_switch(0, "matched on the other road at a fork")
  assert key not in pl.active
  assert any(e["kind"] == "rm_drop" and e["n"] == 1 for e in pl.events)


def test_no_alternative_no_change():
  """A link with no other remembered road near its end: the walk is exactly the memory's own (no behaviour change)."""
  a = M.synthetic_trace([(1000, 0.0)])
  b = M.synthetic_trace([(600, 0.0), (150, 1 / 150.0), (500, 0.0)], x0=a["x"][-1] + 5.0, y0=a["y"][-1], h0=a["hdg"][-1])
  mem = M.synthetic([a, b], nexts=[1, -1])
  pl = P.Planner(mem)
  for i0 in (0, 50, 150, 199):
    idx, dist = mem.ahead(0, i0, 900.0)
    idx2, dist2 = pl._ahead(0, i0, 900.0)
    assert np.array_equal(idx, idx2) and np.allclose(dist, dist2)


@pytest.mark.parametrize("case", ["a", "b"])
def test_fork_log_is_small_and_position_free(case):
  mem = case_a()[3] if case == "a" else case_b()[2]
  pl = P.Planner(mem)
  c = pl._fork_cands(0)
  assert len(c) == 2
  fk = None
  for tr, p, along, turn in c:
    assert isinstance(tr, int) and isinstance(p, int)
  idx, dist = pl._ahead(0, int(mem.trace_end(0)) - 40, 600.0)
  fk = pl.fk
  assert isinstance(fk, list) and len(fk) == 4 and all(isinstance(x, int) for x in fk)


# ---- beside the record is NOT a fork (independent review of integ-20261126, finding 1) ------------------------------------------
# A lane change, a GPS offset or a matcher miss sets road matching's "beside the record" flag. That never drops a slowdown, its
# ceiling, guard, latched warning or hold, and never delays a warning ("a GPS gap or a matcher miss never drops (or delays) a
# warning", fourth review MUST 1): the plan in those drives must be EXACTLY the one before the fork work. Only a positive match on the
# other road past a fork (fork_switch) drops what was planned on the road not taken.
# Synthetic straight road into a 37 mph bend (1/120 m) at 1500 m, approached at 60 mph.
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


PROBES = {
  "lane_changes_ending_10m_before_the_bend": two_lane_changes(1260, 1380),
  "gps_jump_6m_for_40m": lambda s: ((6.0, 0.0) if 1300 <= s <= 1340 else (0.0, 0.0)),
  "lane_changes_before_the_slowdown_starts": two_lane_changes(1000, 1150),
}
# Expected: the same synthetic drives run on commit 1c8a450d (the build before the fork work: 447837fb), recorded tick by tick as
# (metres along, published ceiling, alert, slowdowns in progress, latched warnings), here as the ticks where any of them changes.
# All three are identical to that build with RM_DROP off at 3147bb1d (the reviewer's probe).
PRE_FORK = {
  "lane_changes_ending_10m_before_the_bend":
    [(0.0, None, None, 0, 0), (976.3, None, None, 0, 1), (1244.6, 50, 37, 1, 1), (1341.1, 39, 37, 1, 1), (1727.4, None, None, 0, 0)],
  "gps_jump_6m_for_40m":
    [(0.0, None, None, 0, 0), (976.3, None, None, 0, 1), (1244.6, 50, 37, 1, 1), (1341.1, 39, 37, 1, 1), (1727.4, None, None, 0, 0)],
  "lane_changes_before_the_slowdown_starts":
    [(0.0, None, None, 0, 0), (976.3, None, None, 0, 1), (1244.6, None, 37, 0, 1), (1448.4, 39, 37, 1, 1), (1727.4, None, None, 0, 0)],
}


def run_offset(mem, path, v_mph, seconds, lat_fn):
  """Constant speed along `path` with the fix moved sideways by lat_fn(s) -> (metres left, heading offset rad)."""
  pl = P.Planner(mem)
  s_road = np.arange(len(path["x"])) * M.SPACING_M
  rows, evs, s = [], [], 0.0
  v = v_mph * MPH
  for k in range(int(seconds / DT)):
    if s > s_road[-1]:
      break
    j = min(int(s // M.SPACING_M), len(path["x"]) - 1)
    h = float(path["hdg"][j])
    d, dh = lat_fn(s)
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


@pytest.mark.parametrize("name", list(PROBES))
def test_beside_the_record_keeps_the_plan_exactly_as_before_the_fork_work(name):
  a, mem = probe_road()
  rows, evs = run_offset(mem, a, 60, 120, PROBES[name])
  assert any(e["kind"] == "rm_off" for _, e in evs), "the probe must actually put the car beside the record"
  assert changes(rows) == PRE_FORK[name]
  assert not any(e["kind"] == "rm_drop" for _, e in evs)
  # the warning shows from the same distance before the bend, and a ceiling is published through the bend
  first_alert = next(s for s, c, al, n, nl in rows if al is not None)
  assert PROBE_BEND_M - first_alert > 250.0
  assert all(c is not None for s, c, al, n, nl in rows if PROBE_BEND_M <= s <= PROBE_BEND_M + 150.0)


def fork_bend_on_the_planned_road():
  """The link b leaves at 14 deg, straight; the straight-on road c (the plan by the same-road rule) has a bend 150 m in."""
  a = M.synthetic_trace([(800, 0.0)])
  b = M.synthetic_trace([(500, 0.0)], x0=a["x"][-1] + 5.0, y0=0.0, h0=0.25)   # straight (not a bend the car corners in)
  c = M.synthetic_trace([(150, 0.0), (200, 1 / 90.0), (800, 0.0)], x0=a["x"][-1] + 40.0, y0=0.0, h0=0.0)
  return a, b, c, M.synthetic([a, b, c], nexts=[1, -1, -1])


def test_a_road_switch_that_clears_the_old_roads_warning_logs_it():
  """The car takes the branch the plan did not follow: the warning latched for the planned road's bend is cleared on the switch,
  and that is logged (rm_drop with the latched-warning count), also when no slowdown is in progress."""
  a, b, c, mem = fork_bend_on_the_planned_road()
  res = run(mem, cat(a, b), 50, 60)
  k_sw = next(k for k, r in enumerate(res) if any(e["kind"] == "fork_switch" for e in r[3]))
  pl_before = res[k_sw - 1][4]
  sw = [e for e in res[k_sw][3] if e["kind"] == "rm_drop"]
  assert sw and sw[0]["nl"] >= 1, res[k_sw][3]
  assert all(p.diag.get("alert") is None for _, _, p, _, _ in res[k_sw:] if p.diag)
  del pl_before


def test_a_cleared_latch_is_logged_with_no_slowdown_in_progress():
  a, b, c, mem = fork_bend_on_the_planned_road()
  pl = P.Planner(mem)
  key = int(mem.trace_start[2]) + 20            # a bend point on the road not taken
  pl.latch[key] = {"key": key, "need": 30, "entry": 100.0, "low": 120.0, "end": 140.0, "rec": False}
  pl.hold_latch = {key: 40}
  assert not pl.active
  pl._drop_for_switch(1, "matched on the other road at a fork")
  assert key not in pl.latch and key not in pl.hold_latch
  ev = [e for e in pl.events if e["kind"] == "rm_drop"]
  assert ev and ev[0]["n"] == 0 and ev[0]["nl"] == 1 and ev[0]["nh"] == 1


def test_a_switch_keeps_warnings_for_bends_on_the_road_now_followed():
  a, b, c, mem = fork_bend_on_the_planned_road()
  pl = P.Planner(mem)
  key = int(mem.trace_start[1]) + 20            # a point on the road the car is now matched on
  pl.latch[key] = {"key": key, "need": 30, "entry": 100.0, "low": 120.0, "end": 140.0, "rec": False}
  pl._drop_for_switch(1, "matched on the other road at a fork")
  assert key in pl.latch
  assert not any(e["kind"] == "rm_drop" for e in pl.events)
