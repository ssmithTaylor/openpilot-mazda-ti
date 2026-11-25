"""Winding-road closed-loop sweep (mazda_ti_winding_gen): runs every family's deterministic scenarios and writes one JSON per chunk to
$WINDING_OUT (default E:/openpilot-mazda-ti-data/winding-tests/out). The sweep itself is not an assertion suite: the findings are read
by tools/mazda_sim/winding_report.py. The full sweep (test_winding_chunk) and the probe are marked slow: run on demand with
pc_test.py --slow -n 2 -- <abs path to this file> -k chunk. The normal `pc_test.py --curve` run executes only REPRO: the fixed repro
cases of the failure classes found by the sweep (a few seconds each), with each class's assertion."""
import importlib.util
import json
import os
import pathlib
import time

import pytest

_spec = importlib.util.spec_from_file_location("mazda_ti_winding_gen", pathlib.Path(__file__).parent / "mazda_ti_winding_gen.py")
G = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(G)

OUT = pathlib.Path(os.environ.get("WINDING_OUT", "E:/openpilot-mazda-ti-data/winding-tests/out"))
N = int(os.environ.get("WINDING_N", "24"))          # scenarios per family
CHUNK = int(os.environ.get("WINDING_CHUNK", "4"))
FAMILIES = ("F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8")
CASES = [(fam, c) for fam in FAMILIES for c in range(0, N, CHUNK)]


@pytest.mark.slow
@pytest.mark.parametrize("fam,c0", CASES)
def test_winding_chunk(fam, c0):
  OUT.mkdir(parents=True, exist_ok=True)
  res = []
  for seed in range(c0, min(N, c0 + CHUNK)):
    t0 = time.time()
    sc = G.gen(fam, seed)
    r = G.run(sc)
    r["wall_s"] = round(time.time() - t0, 1)
    r["scenario"] = sc
    res.append(r)
  (OUT / f"{fam}_{c0:03d}.json").write_text(json.dumps(res, default=str))


@pytest.mark.slow
def test_one_probe():
  """a single quick scenario, timed (smoke)"""
  t0 = time.time()
  r = G.run(G.gen(os.environ.get("WINDING_PROBE_FAM", "F2"), int(os.environ.get("WINDING_PROBE_SEED", "0"))))
  print("PROBE", round(time.time() - t0, 1), json.dumps({k: v for k, v in r.items() if k != "bends"}, default=str)[:1500])
  for b in r.get("bends", []):
    print(b)
  assert r.get("exc") is None, r["exc"]


def test_dump():
  """WINDING_DUMP=F2:0 -> out/dump_F2_0.json with the 10 Hz rows, set-speed changes, bends and events"""
  spec = os.environ.get("WINDING_DUMP")
  if not spec:
    pytest.skip("WINDING_DUMP not set")
  OUT.mkdir(parents=True, exist_ok=True)
  for one in spec.split(","):
    fam, seed = one.split(":")
    r = G.run(G.gen(fam, int(seed)), keep_rows=True)
    (OUT / f"dump_{fam}_{seed}.json").write_text(json.dumps(r, default=str))


# ---- the fixed repro subset (run by pc_test.py --curve) ---------------------------------------------------------------------------
REPRO = (("F1", 61), ("F1", 18), ("F2", 0), ("F1", 3), ("F5", 16))
_RES = {}


def _run(fam, seed):
  if (fam, seed) not in _RES:
    _RES[(fam, seed)] = G.run(G.gen(fam, seed), keep_rows=True)
  return _RES[(fam, seed)]


@pytest.mark.parametrize("fam,seed", REPRO)
def test_repro_runs(fam, seed):
  """every repro case runs to the road end without a planner / car-side exception"""
  r = _run(fam, seed)
  assert r.get("exc") is None, r["exc"]
  assert r["finished"]


def _bend_min_mph(r, b):
  a, e = r["bl"][b["i"]][0], r["bl"][b["i"]][1]
  return min(row[3] for row in r["rows"] if a <= row[1] <= e)


def _from_cruise(r, b, before_m=250.0):
  """the car was at the driver's set speed somewhere in the before_m before the bend (it arrives from cruise, not from a bend)"""
  a = r["bl"][b["i"]][0]
  return any(row[3] >= r["set"] - 0.5 for row in r["rows"] if a - before_m <= row[1] < a)


@pytest.mark.parametrize("fam,seed", (("F1", 61), ("F1", 18)))
def test_class1_record_bound_kept_for_small_drops(fam, seed):
  """CLASS 1: a remembered bend whose RECORD allows a speed only a few mph (<= LATE_SHED_MPH) under the set speed keeps that bound
  whichever stage would handle it (F1/61: set 35, record bends allowed 32 were all driven at 35; F1/18: set 45, the set speed
  walked up to 43 inside a record bend allowed 40). Every such bend: no SET+ inside it above its allowed speed; one reached from
  cruise: arrival at or under allowed + 1 mph and graded (the car is not taken more than 1.5 mph under the allowed speed)."""
  r = _run(fam, seed)
  small = [b for b in r["bends"] if not b["drv"] and b["needed"] and r["set"] - b["allowed"] <= 5 and b["allowed"] >= b["floor"]]
  assert small, "the case must contain small-drop record bends"
  for b in small:
    assert b["bad_up"] == 0, b
    if _from_cruise(r, b):
      assert b["peak"] <= b["allowed"] + 1.0, b
      nxt = [(a2, al2) for (a2, *_x), al2 in zip(r["bl"], r["allowed"]) if 0 < a2 - r["bl"][b["i"]][1] <= 150.0]
      if not any(al2 < b["allowed"] for _a, al2 in nxt):       # graded, unless a lower bend follows at once
        assert _bend_min_mph(r, b) >= b["allowed"] - 1.5, (b, _bend_min_mph(r, b))
