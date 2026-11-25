#!/usr/bin/env python3
"""Post-drive memory report: per-process RSS growth and the device's memory trend over one drive.

  python tools/dev/drive_memory_report.py <drive dir> [--json]      # <drive dir>/<route>--<n>/rlog[.zst|.bz2], read only

Reads ONLY three message types and nothing else from any event (position-free by construction: every other event is skipped on
its type alone): procLog (per-process RSS, available memory), deviceState (memory used percent) and controlsState (engaged,
cumulative lag). Prints per process: RSS at start and end, the least-squares slope in MB per engaged minute and in MB per hour of
drive, and a verdict flagging any process growing faster than 50 MB/h; then the device's memory-percent slope and the projected
minutes to 90 % (the system's low-memory alert), and controlsd's cumulative lag at the start and end.
Read the logs from the home collector's store, never from the device.
"""
import argparse
import bz2
import json
import re
import sys
from pathlib import Path

WANTED = ("procLog", "deviceState", "controlsState")
FLAG_MB_PER_H = 50.0
LOW_MEMORY_PCT = 90.0
MIN_POINTS = 5
MIN_SPAN_MIN = 10.0     # a slope over a shorter span (one segment) is start-up and sampling noise: reported, never flagged
# Our processes and the upstream ones most worth watching; any other process is reported too when it grows past the flag.
WATCH = ("controlsd", "card", "ti_curve_plannerd", "plannerd", "radard", "modeld", "ui", "loggerd", "locationd", "paramsd",
         "mapd", "frogpilot_process", "ti_mcp")


# ---- the arithmetic (unit-tested on synthetic series) --------------------------------------------------------------------------------
def slope(xs, ys):
  """Least-squares slope dy/dx, or None with fewer than MIN_POINTS points or no spread in x."""
  n = len(xs)
  if n < MIN_POINTS or n != len(ys):
    return None
  mx, my = sum(xs) / n, sum(ys) / n
  den = sum((x - mx) ** 2 for x in xs)
  if den <= 0:
    return None
  return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den


def verdict(name, mb_per_h, limit=FLAG_MB_PER_H, span_min=None):
  if mb_per_h is None:
    return f"{name}: not enough samples"
  if span_min is not None and span_min < MIN_SPAN_MIN:
    return f"{name}: {mb_per_h:+.0f} MB/h over only {span_min:.0f} min: too short to judge (needs {MIN_SPAN_MIN:.0f} min)"
  if mb_per_h > limit:
    return f"{name}: FLAG growing {mb_per_h:.0f} MB/h (over {limit:.0f} MB/h): a leak candidate"
  return f"{name}: ok ({mb_per_h:+.1f} MB/h)"


def minutes_to(t_s, pct, target=LOW_MEMORY_PCT):
  """(slope in % per hour, minutes from the last sample until the fitted line reaches `target`, or None when not rising)."""
  s = slope(t_s, pct)
  if s is None:
    return None, None
  per_h = s * 3600.0
  if s <= 0:
    return per_h, None
  n = len(t_s)
  mx, my = sum(t_s) / n, sum(pct) / n
  now = my + s * (t_s[-1] - mx)
  return per_h, max(0.0, (target - now) / s / 60.0)


def engaged_clock(times, engaged):
  """Cumulative engaged seconds at each sample (a step counts toward the engaged clock when it starts engaged)."""
  out, acc = [], 0.0
  for k, t in enumerate(times):
    if k and engaged[k - 1]:
      acc += t - times[k - 1]
    out.append(acc)
  return out


def summarise(series, dev, ctl, avail=()):
  """series: {process: [(t s, rss bytes)]}; dev: [(t, mem pct, None)]; ctl: [(t, enabled, cumLagMs)]; avail: [(t, available bytes)]."""
  ct = [c[0] for c in ctl]
  ce = [c[1] for c in ctl]
  eng = engaged_clock(ct, ce) if ctl else []

  def eng_at(t):
    if not ct:
      return None
    import bisect
    k = bisect.bisect_right(ct, t) - 1
    return eng[k] if k >= 0 else 0.0
  procs = {}
  for name, pts in sorted(series.items()):
    pts = sorted(pts)
    ts = [p[0] for p in pts]
    mb = [p[1] / 1e6 for p in pts]
    s_h = slope(ts, mb)
    e = [eng_at(t) for t in ts] if ct else []
    s_eng = slope(e, mb) if e and e[-1] - e[0] > 60 else None
    procs[name] = {"n": len(pts), "start_MB": round(mb[0], 1) if mb else None, "end_MB": round(mb[-1], 1) if mb else None,
                   "MB_per_h": None if s_h is None else round(s_h * 3600.0, 1),
                   "MB_per_engaged_min": None if s_eng is None else round(s_eng * 60.0, 2),
                   "engaged_min": round((e[-1] - e[0]) / 60.0, 1) if e else None}
    span = (ts[-1] - ts[0]) / 60.0 if ts else 0.0
    procs[name]["span_min"] = round(span, 1)
    procs[name]["verdict"] = verdict(name, procs[name]["MB_per_h"], span_min=span)
  dt = [d[0] for d in dev]
  dp = [d[1] for d in dev]
  pct_h, mins = minutes_to(dt, dp)
  out = {"processes": procs, "drive_min": round((max(dt) - min(dt)) / 60.0, 1) if dt else None,
         "mem_pct_start": dp[0] if dp else None, "mem_pct_end": dp[-1] if dp else None,
         "mem_pct_per_h": None if pct_h is None else round(pct_h, 2), "minutes_to_90": None if mins is None else round(mins, 1),
         "engaged_min": round(eng[-1] / 60.0, 1) if eng else None,
         "avail_MB_start": round(avail[0][1] / 1e6) if avail else None, "avail_MB_end": round(avail[-1][1] / 1e6) if avail else None,
         "cum_lag_ms_start": ctl[0][2] if ctl else None, "cum_lag_ms_end": ctl[-1][2] if ctl else None}
  out["flagged"] = [n for n, p in procs.items() if p["MB_per_h"] is not None and p["MB_per_h"] > FLAG_MB_PER_H
                    and p["span_min"] >= MIN_SPAN_MIN]
  return out


# ---- reading (only the three wanted types) ----------------------------------------------------------------------------------------------
def _seg_key(p):
  m = re.search(r"--(\d+)$", p.name)
  return int(m.group(1)) if m else -1


def segment_files(drive_dir):
  segs = sorted((p for p in Path(drive_dir).iterdir() if p.is_dir() and re.search(r"--\d+$", p.name)), key=_seg_key)
  out = []
  for s in segs:
    for n in ("rlog", "rlog.zst", "rlog.bz2"):
      if (s / n).exists():
        out.append(s / n)
        break
  return out


def _read_bytes(path):
  data = Path(path).read_bytes()
  if str(path).endswith(".bz2"):
    return bz2.decompress(data)
  if str(path).endswith(".zst"):
    import zstandard
    return zstandard.ZstdDecompressor().decompressobj().decompress(data)
  return data


def proc_name(p):
  cmd = list(p.cmdline)
  if cmd:
    for c in cmd:                     # python processes: the module path, e.g. selfdrive.controls.controlsd
      if "." in c and "/" not in c and not c.startswith("-"):
        return c.split(".")[-1]
    return cmd[0].split("/")[-1]
  return p.name


def read_drive(drive_dir, all_procs=True):
  from cereal import log
  series, dev, ctl, dev_avail = {}, [], [], []
  for f in segment_files(drive_dir):
    try:
      events = log.Event.read_multiple_bytes(_read_bytes(f))
    except Exception as e:
      print(f"skip {f.parent.name}: {type(e).__name__}", file=sys.stderr)
      continue
    try:
      for ev in events:
        w = ev.which()
        if w not in WANTED:
          continue                  # nothing else is ever read
        t = ev.logMonoTime * 1e-9
        if w == "procLog":
          pl = ev.procLog
          for p in pl.procs:
            nm = proc_name(p)
            if all_procs or nm in WATCH:
              series.setdefault(nm, []).append((t, p.memRss))
          try:
            avail = int(pl.mem.available)
          except Exception:
            avail = 0
          if avail:
            dev_avail.append((t, avail))
        elif w == "deviceState":
          dev.append((t, float(ev.deviceState.memoryUsagePercent), None))
        else:
          cs = ev.controlsState
          ctl.append((t, bool(cs.enabled), float(cs.cumLagMs)))
    except Exception as e:          # a truncated last segment: keep what was read
      print(f"partial {f.parent.name}: {type(e).__name__}", file=sys.stderr)
  # a process seen in only a few samples (a short-lived helper) is noise in the table
  series = {k: v for k, v in series.items() if len(v) >= MIN_POINTS}
  return series, sorted(dev), sorted(ctl), sorted(dev_avail)


def report(s, top=12):
  lines = []
  procs = s["processes"]
  shown = [n for n in procs if n in WATCH or n in s["flagged"]]
  rest = sorted((n for n in procs if n not in shown), key=lambda n: -(procs[n]["MB_per_h"] or 0))[:max(0, top - len(shown))]
  lines.append(f"drive {s['drive_min']} min, engaged {s['engaged_min']} min")
  lines.append(f"{'process':20s} {'start MB':>9s} {'end MB':>8s} {'MB/eng min':>11s} {'MB/h':>8s}")
  for n in sorted(shown) + rest:
    p = procs[n]
    lines.append(f"{n[:20]:20s} {p['start_MB']!s:>9s} {p['end_MB']!s:>8s} {p['MB_per_engaged_min']!s:>11s} {p['MB_per_h']!s:>8s}")
  for n in sorted(shown) + rest:
    lines.append("  " + procs[n]["verdict"])
  lines.append(f"device memory used {s['mem_pct_start']} % -> {s['mem_pct_end']} %, slope {s['mem_pct_per_h']} %/h, "
               + ("projected 90 % in " + f"{s['minutes_to_90']} min" if s["minutes_to_90"] is not None else "not rising toward 90 %"))
  lines.append(f"available memory {s['avail_MB_start']} MB -> {s['avail_MB_end']} MB")
  lines.append(f"controls cumulative lag {s['cum_lag_ms_start']} -> {s['cum_lag_ms_end']} ms")
  lines.append("VERDICT: " + (f"FLAG {', '.join(s['flagged'])} growing over {FLAG_MB_PER_H:.0f} MB/h" if s["flagged"] else
                              f"no process over {FLAG_MB_PER_H:.0f} MB/h"))
  return "\n".join(lines)


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("drive_dir")
  ap.add_argument("--json", action="store_true")
  a = ap.parse_args(argv)
  s = summarise(*read_drive(a.drive_dir))
  print(json.dumps(s, indent=1) if a.json else report(s))
  return 2 if s["flagged"] else 0


if __name__ == "__main__":
  sys.exit(main())
