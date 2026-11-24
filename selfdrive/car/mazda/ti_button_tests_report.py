#!/usr/bin/env python3
"""The cruise button tests' outcomes from drive logs, in plain words (run on the PC over the home collector's copies, not on the comma):

    python selfdrive/car/mazda/ti_button_tests_report.py <folder with rlogs> [--json]

Reads every rlog / rlog.bz2 / rlog.zst under the folder (in name order), and from each:
  - the `ti_button` log events the car controller wrote (armed, start, held and tapped frames, each direction's / each cycle's result,
    aborted, restore, result), and
  - the raw traces: the dash set speed (carState.cruiseState.speed, 50 Hz in the log) and our CRZ_BTNS frames (sendcan 0x09D, with
    their counters), so each run's own words can be checked against what the car actually showed.
Prints per run: the test, how it ended and why, each direction's verdict (hold test) or each cycle's row (tap test), the set speed at
the start and at the end, the lowest speed, and the raw cross-check (pressed frames sent, the dash's path). Nothing about where the car
was: set speeds, times and counts only.
"""
import argparse
import bz2
import json
import os
import sys

MPH = 0.44704


def _parse_log_line(text):
  start = text.find("{")
  if start < 0:
    return None
  try:
    return json.loads(text[start:])
  except Exception:
    return None


def read_rlog(path):
  """-> (kind, seconds, payload) in log order: ("log", t, text) for every logMessage, ("car", t, (set mph, speed mph)) for every
  carState, ("sent", t, (button, counter)) for every SET frame of ours in sendcan."""
  raw = open(path, "rb").read()
  if path.endswith(".bz2"):
    raw = bz2.decompress(raw)
  elif path.endswith(".zst"):
    import zstandard
    raw = zstandard.ZstdDecompressor().decompressobj().decompress(raw)
  from cereal import log as capnp_log
  for e in capnp_log.Event.read_multiple_bytes(raw):
    try:
      w = e.which()
      t = e.logMonoTime * 1e-9        # monotonic for the whole drive: the segments' traces line up
      if w == "logMessage":
        yield ("log", t, e.logMessage)
      elif w == "carState":
        c = e.carState
        yield ("car", t, (int(round(c.cruiseState.speed / MPH)) if c.cruiseState.speed > 0.5 else None, c.vEgo / MPH))
      elif w == "sendcan":
        for f in e.sendcan:
          if f.address == 0x09D and len(f.dat) >= 4 and f.dat[0] & 0x30:
            yield ("sent", t, ("SET_PLUS" if f.dat[0] & 0x10 else "SET_MINUS", (f.dat[3] >> 2) & 15))
    except Exception:
      continue


def collect(msgs):
  """The runs in a stream of (kind, t, payload): one dict per armed run, with its events and the raw traces between its start and
  its result."""
  runs, cur = [], None
  raw_car, raw_sent = [], []
  for kind, t, p in msgs:
    if kind == "car":
      raw_car.append((t, p[0], p[1]))
      continue
    if kind == "sent":
      raw_sent.append((t, p[0], p[1]))
      continue
    rec = _parse_log_line(p) if isinstance(p, str) else None
    msg = rec.get("msg") if isinstance(rec, dict) else None
    if not isinstance(msg, dict) or msg.get("event") != "ti_button":
      continue
    k = msg.get("kind")
    if k in ("armed", "refused") or cur is None or msg.get("id") != cur["id"]:
      cur = {"id": msg.get("id"), "test": msg.get("test"), "events": [], "t0": t, "t_start": None, "t_end": None, "result": None}
      runs.append(cur)
    cur["events"].append(msg)
    if k == "start":
      cur["t_start"] = t
    if k == "result":
      cur["result"], cur["t_end"] = msg, t
  for r in runs:
    a = r["t_start"] if r["t_start"] is not None else r["t0"]
    b = r["t_end"] if r["t_end"] is not None else float("inf")
    path = []
    for t, s, _ in raw_car:
      if a - 0.5 <= t <= b and s is not None and (not path or path[-1] != s):
        path.append(s)
    speeds = [m for t, _, m in raw_car if a <= t <= b]
    r["raw"] = {"frames": sum(1 for t, _, _ in raw_sent if a - 0.5 <= t <= b + 0.1), "set_path": path,
                "lowest_mph": round(min(speeds), 1) if speeds else None}
  return runs


def _row(c):
  parts = [x for x in (c.get("down"), c.get("up")) if x]
  sent = sum(x.get("sent", 0) for x in parts)
  steps = sum(x.get("steps", 0) for x in parts)
  ms = sorted(x["ms"] for x in parts if x.get("ms") is not None)
  bits = [f"{c.get('cycle')} s: {sent} presses, {steps} steps" + (f" ({steps / sent:.2f} per press)" if sent else ""),
          f"median {ms[len(ms) // 2]} ms to the step" if ms else "no step"]
  for key, word in (("missed", "missed"), ("double", "double"), ("jump", "jump of 2+ mph (read as a hold)")):
    n = sum(x.get(key, 0) for x in parts)
    if n:
      bits.append(f"{n} {word}")
  if c.get("cut"):
    bits.append("cut short")
  bits.append("every press stepped 1 mph" if c.get("clean") else "not clean")
  return ", ".join(bits)


def describe(runs):
  out = []
  for i, r in enumerate(runs, 1):
    res = r["result"] or {}
    name = {"hold": "HOLD TEST", "taps": "TAP TEST"}.get(r["test"], str(r["test"]))
    state = res.get("state", "no result in these logs")
    out.append(f"Run {i}: {name} ({r['id']}): {state}" + (f": {res['why']}" if res.get("why") else ""))
    if r["test"] == "hold":
      for key, label in (("down", "SET-"), ("up", "SET+")):
        d = (res.get("hold") or {}).get(key)
        if d is None:
          d = next((e for e in r["events"] if e.get("kind") == "direction" and e.get("dir") == key), None)
        if d is not None:
          out.append(f"  {label}: {d.get('word')}: {d.get('text')}")
    elif r["test"] == "taps":
      for c in res.get("taps") or []:
        out.append("  " + _row(c))
      f = res.get("fastest_clean")
      out.append(f"  fastest clean rate: {f} s" if f is not None else "  no rate stepped 1 mph on every press")
    rest = res.get("restore") or {}
    if res:
      out.append(f"  set speed {res.get('orig')} mph at the start, {res.get('set_mph')} at the end; {rest.get('note', '')}")
      if res.get("start_mph") is not None:
        out.append(f"  speed {res.get('start_mph')} mph at the start, lowest {res.get('lowest_mph')} mph")
    raw = r.get("raw") or {}
    if raw:
      path = raw.get("set_path") or []
      shown = " -> ".join(str(v) for v in (path if len(path) <= 16 else path[:12] + ["..."] + path[-3:]))
      out.append(f"  raw: {raw.get('frames')} pressed frames sent; dash {shown or 'not in these logs'}")
  return "\n".join(out) if out else "No button test in these logs."


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
  ap.add_argument("folder")
  ap.add_argument("--json", action="store_true")
  a = ap.parse_args(argv)
  files = []
  for root, _, names in os.walk(a.folder):
    for n in names:
      if n in ("rlog", "rlog.bz2", "rlog.zst") or n.endswith((".rlog", ".rlog.bz2", ".rlog.zst")):
        files.append(os.path.join(root, n))
  files.sort()

  def stream():
    for p in files:
      try:
        yield from read_rlog(p)
      except Exception as e:
        print(f"skipped {p}: {type(e).__name__}: {e}", file=sys.stderr)
  runs = collect(stream())
  if a.json:
    print(json.dumps(runs, default=str))
  else:
    print(describe(runs))
  return 0


if __name__ == "__main__":
  sys.exit(main())
