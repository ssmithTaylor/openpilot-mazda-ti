#!/usr/bin/env python3
"""Aggregate the winding-road sweep (selfdrive/car/tests/test_mazda_ti_winding.py -> out/F*_*.json) into failure classes by family.
Plain Python (no openpilot imports). Usage: python winding_report.py <out dir> [--examples]"""
import collections
import glob
import json
import os
import sys

ALERT_LEAD_S = 4.0


def classify(r):
  """-> {class: [detail, ...]} for one scenario"""
  out = collections.defaultdict(list)
  if r.get("exc"):
    out["EXC planner/car-side exception"].append(r["exc"].strip().splitlines()[-1][:160])
  if not r.get("finished"):
    out["NOT FINISHED (car never reached the road end)"].append("")
  if r.get("stuck"):
    out["STUCK at road end (set != driver's or state not idle)"].append(f"set {r['end_set']} owner {r['end_owner']} {r['end_state']}")
  for b in r.get("bends", []):
    if b["drv"]:
      continue
    over = b["peak"] - b["allowed"]
    if over > 1.0:
      lead = b["warn_lead_s"]
      if b["unwarned_over_rows"] > 0 and over > 2.5:
        out["OVER allowed+2.5 with no warning in the bend"].append((b["i"], b["kind"], b["allowed"], b["peak"], b["min_set"], b["floor"]))
      elif over > 2.5 and (lead is None or lead < ALERT_LEAD_S):
        out["OVER, warning late (< 4 s)"].append((b["i"], b["allowed"], b["peak"], lead))
      elif over <= 2.5:
        out["OVER by 1-2.5 mph (inside the warning margin)"].append((b["i"], b["allowed"], b["peak"]))
      else:
        out["(ok) OVER but warned in time"].append((b["i"], b["allowed"], b["peak"], b["floor"]))
      if b["needed"] and b["min_set"] >= max(b["allowed"], b["floor"]) + 3 and b["min_set"] >= b["peak"] - 1:
        out["NO SLOWDOWN for a bend that needed one"].append((b["i"], b["allowed"], b["peak"], b["min_set"], b["floor"]))
    if b["bad_up"] > 0:
      out["SET+ in a bend above its allowed / floor"].append((b["i"], b["allowed"], b["floor"], b["bad_up_to"]))
  if r.get("gap15_unjustified_s", 0) > 0:
    out["ACTUAL > 15 mph under driver's set (not floor-justified)"].append(r["gap15_unjustified_s"])
  if r.get("dwell_s", 0) > 3.0:
    out["DWELL > 3 s more than 3 mph under the needed speed"].append((r["dwell_s"], r["dwell_max"]))
  if r.get("yoyo", 0) > 0:
    out["YO-YO (SET+ then SET- within 5 s)"].append(r["yoyo"])
  if r.get("restore_undone", 0) > 0:
    out["RESTORE UNDONE (SET+ followed by >= 2 SET- within 8 s)"].append(r["restore_undone"])
  if r.get("taps_per_min", 0) > 20:
    out["TAPS > 20 / min"].append(r["taps_per_min"])
  for a in r.get("alerts", []):
    if not a["justified"]:
      out["ALERT not justified (no bend after it needed it)"].append((a["s"], a["need"], a["len_s"]))
  pe = r.get("pl_events", {})
  if pe.get("lock_lost", 0) >= 3:
    out["LOCK/LOSE cycling (>= 3 lock losses)"].append((pe.get("lock_lost"), pe.get("lock")))
  for e in r.get("stopped", []):
    if e.get("kind") in ("stop", "planner_dead", "exit_armed"):
      out["STOP / set-speed-left / planner dead"].append(e.get("kind") + ":" + str(e.get("why", ""))[:60])
  return out


def main():
  d = sys.argv[1] if len(sys.argv) > 1 else "E:/openpilot-mazda-ti-data/winding-tests/out"
  rs = []
  for p in sorted(glob.glob(os.path.join(d, "F*_*.json"))):
    rs += json.load(open(p))
  fams = sorted({r["family"] for r in rs})
  table = collections.defaultdict(lambda: collections.Counter())
  ex = collections.defaultdict(list)
  nsc = collections.Counter(r["family"] for r in rs)
  nb = collections.Counter()
  for r in rs:
    nb[r["family"]] += len(r.get("bends", []))
    for c, v in classify(r).items():
      table[c][r["family"]] += 1
      ex[(c, r["family"])].append((r["seed"], r["set"], r["mem"], r["opts"], v[:2]))
  print("scenarios", dict(nsc), "bends", dict(nb))
  print(f"{'class':62s} " + " ".join(f"{f:>5s}" for f in fams) + "   all")
  for c in sorted(table):
    print(f"{c[:62]:62s} " + " ".join(f"{table[c][f]:5d}" for f in fams) + f" {sum(table[c].values()):5d}")
  if "--examples" in sys.argv:
    for (c, f), v in sorted(ex.items()):
      print(f"\n== {c} / {f}: {len(v)}")
      for e in v[:4]:
        print("  ", e)
  # per-family figures
  print()
  for f in fams:
    sub = [r for r in rs if r["family"] == f]
    tpm = sorted(r.get("taps_per_min", 0) for r in sub)
    print(f"{f}: taps/min median {tpm[len(tpm) // 2]} max {tpm[-1]}; yo-yo total {sum(r.get('yoyo', 0) for r in sub)}; "
          f"max gap under set {max(r.get('max_gap', 0) for r in sub)}; dwell>3s in {sum(r.get('dwell_s', 0) > 3 for r in sub)}; "
          f"exceptions {sum(bool(r.get('exc')) for r in sub)}; wall {sum(r.get('wall_s', 0) for r in sub):.0f} s")
  grade_table(rs)
  kinds = collections.defaultdict(lambda: [0, 0, 0, 0])
  for r in rs:
    if r["family"] != "F3":
      continue
    for b in r.get("bends", []):
      k = kinds[b["kind"].split("@")[0]]
      k[0] += 1
      k[1] += (b["peak"] - b["allowed"] > 2.5 and b["unwarned_over_rows"] > 0)
      k[2] += (b["peak"] - b["allowed"] > 1)
      k[3] += (b["warn_lead_s"] is not None and b["warn_lead_s"] >= ALERT_LEAD_S)
  print("F3 by piece: n, over+2.5 unwarned, over+1, warned >= 4 s before")
  for k in sorted(kinds):
    print("  ", k, kinds[k])


def grade_table(rs):
  """F4: arrival error (peak - allowed, mph) at bends the car could be slowed for (allowed >= floor), by grade and memory grade"""
  g = collections.defaultdict(list)
  for r in rs:
    if r["family"] != "F4":
      continue
    for b in r.get("bends", []):
      if b["needed"] and b["allowed"] >= b["floor"] and r["mem"] != "none" and r["set"] - b["allowed"] > 5:
        g[(r["opts"]["grade"], bool(r["opts"]["mem_grade"]))].append(b["peak"] - b["allowed"])
  print("F4 grade, mem_grade: n, mean err, max err, share over +1")
  for k in sorted(g):
    v = g[k]
    print(f"  {k[0]:+.2f} {str(k[1]):5s} n={len(v):3d} mean={sum(v) / len(v):+.2f} max={max(v):+.1f} over1={sum(x > 1 for x in v) / len(v):.2f}")


if __name__ == "__main__":
  main()
