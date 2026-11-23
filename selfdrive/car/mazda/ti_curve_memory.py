"""Road memory for curve slowing (ti_curve_plan.py): the file format, the loader and the nearest-point search.

The memory file is built OFF the car (tools/curve_slowing/build_memory.py) from the owner's recorded drives and copied to the comma by
hand. It holds where they have driven, so it is private: it is never written into the repository, logs, reports, test fixtures or
anything sent anywhere. Tests use synthetic roads (`synthetic`). The planner logs no position from it, only trace / point indices,
distances and curvatures.

Format (numpy .npz, version 1). Every point is SPACING_M along its trace from the previous one, in the direction it was driven:
  meta         json text: {"version": 1, "spacing_m": 5.0, "lat0": .., "lon0": .., "kx": .., "ky": .., ...}
               local metres: x = (lon - lon0) * kx (east), y = (lat - lat0) * ky (north)
  x, y         float32 metres
  hdg          float32 radians, direction of travel, east = 0, counter-clockwise positive (atan2(dy, dx))
  kap          float32 1/m, left positive, from yaw rate / speed (the car's yaw sensor), each pass smoothed over ~10 m: CONSERVATIVE,
               the sharpest pass where there are 1-2 passes, the 90th percentile of |kap| (signed as the median) with 3 or more
               (a stored pass that understated the corner was the largest avoidable miss in the remembered-roads study)
  v            float16 m/s, the median speed driven
  flags        uint8: FLAG_LIMIT (the steering reached its limit here on some pass), FLAG_TAKEOVER (a firm takeover here on some pass),
               FLAG_NOKAP (every pass's curvature here was excluded as unrepresentative; kap is 0, not measured),
               FLAG_LANE (with lateral control active the car touched or crossed a lane line here on some pass, no blinker)
  v_flag       float16 m/s, the lowest speed of a pass that set a flag here (nan where no flag)
  npass        uint8, passes behind the point
  bank         float16 m/s^2 (nan where unknown); informational only
  grade        OPTIONAL float16 radians, road grade, positive = uphill in the direction of travel (median over passes of the
               liveLocationKalman calibrated pitch, low-passed ~3 s); nan where unknown; read as zeros when the file has none
  v_proven     OPTIONAL float16 m/s: the highest speed of a pass through the point with lateral control active and no steering-limit
               event or firm takeover from 80 m before to 80 m after it; nan where none, and read as nan when the file has none
  trace_start  int32 [n_traces + 1]: trace t is points trace_start[t] .. trace_start[t+1]-1
  trace_next   int32 [n_traces]: the trace that continues this one past its last point (-1: none or ambiguous)
"""
import json
import math

import numpy as np

VERSION = 1
SPACING_M = 5.0
FLAG_LIMIT = 1
FLAG_TAKEOVER = 2
FLAG_NOKAP = 4                   # no representative pass: kap is 0 here, not a measured straight
FLAG_LANE = 8                    # with lateral control active the car touched or crossed a lane line here on some pass
CELL_M = 50.0                    # grid cell of the nearest-point index
KEYS = ("x", "y", "hdg", "kap", "v", "flags", "v_flag", "npass", "bank", "trace_start", "trace_next")
OPTIONAL_KEYS = ("bank", "grade", "v_proven")


def wrap(a):
  return (a + math.pi) % (2.0 * math.pi) - math.pi


class Memory:
  """The loaded memory: flat point arrays, per-point trace id, and a grid index for the nearest-point search."""

  def __init__(self, arrays, meta):
    self.meta = meta
    self.x = np.asarray(arrays["x"], np.float64)
    self.y = np.asarray(arrays["y"], np.float64)
    self.hdg = np.asarray(arrays["hdg"], np.float64)
    self.kap = np.asarray(arrays["kap"], np.float64)
    self.v = np.asarray(arrays["v"], np.float64)
    self.flags = np.asarray(arrays["flags"], np.uint8)
    self.v_flag = np.asarray(arrays["v_flag"], np.float64)
    self.npass = np.asarray(arrays["npass"], np.int32)
    self.bank = np.asarray(arrays.get("bank", np.full(len(self.x), np.nan)), np.float64)
    self.grade = np.asarray(arrays.get("grade", np.zeros(len(self.x))), np.float64)
    self.v_proven = np.asarray(arrays.get("v_proven", np.full(len(self.x), np.nan)), np.float64)
    self.trace_start = np.asarray(arrays["trace_start"], np.int64)
    self.trace_next = np.asarray(arrays["trace_next"], np.int64)
    self.n = len(self.x)
    self.n_traces = len(self.trace_start) - 1
    self.spacing = float(meta.get("spacing_m", SPACING_M))
    self.lat0 = float(meta.get("lat0", 0.0))
    self.lon0 = float(meta.get("lon0", 0.0))
    self.kx = float(meta.get("kx", 111320.0))
    self.ky = float(meta.get("ky", 110574.0))
    self._check()
    self.trace_of = np.repeat(np.arange(self.n_traces), np.diff(self.trace_start))
    # grid index: points sorted by cell key; a cell's points are found with searchsorted
    cx = np.floor(self.x / CELL_M).astype(np.int64)
    cy = np.floor(self.y / CELL_M).astype(np.int64)
    key = cx * 2000003 + cy
    self._order = np.argsort(key, kind="stable")
    self._keys = key[self._order]

  def _check(self):
    n = self.n
    for name in ("y", "hdg", "kap", "v", "flags", "v_flag", "npass", "bank", "grade", "v_proven"):
      if len(getattr(self, name)) != n:
        raise ValueError(f"memory: {name} has {len(getattr(self, name))} points, x has {n}")
    if len(self.trace_start) < 1 or self.trace_start[0] != 0 or self.trace_start[-1] != n or np.any(np.diff(self.trace_start) < 1):
      raise ValueError("memory: trace_start is not a partition of the points")
    if len(self.trace_next) != len(self.trace_start) - 1:
      raise ValueError("memory: trace_next has the wrong length")
    if np.any((self.trace_next < -1) | (self.trace_next >= len(self.trace_start) - 1)):
      raise ValueError("memory: trace_next out of range")
    if not (np.all(np.isfinite(self.x)) and np.all(np.isfinite(self.y)) and np.all(np.isfinite(self.hdg)) and np.all(np.isfinite(self.kap))):
      raise ValueError("memory: non-finite position, heading or curvature")

  # ---- geometry ------------------------------------------------------------------------------------------------------------------
  def to_local(self, lat, lon):
    return (lon - self.lon0) * self.kx, (lat - self.lat0) * self.ky

  def trace_end(self, t):
    return int(self.trace_start[t + 1])          # one past the last point

  def near(self, x, y, radius):
    """Indices of points within `radius` metres of (x, y)."""
    r = int(math.ceil(radius / CELL_M))
    cx, cy = int(math.floor(x / CELL_M)), int(math.floor(y / CELL_M))
    out = []
    for i in range(cx - r, cx + r + 1):
      base = i * 2000003
      lo = np.searchsorted(self._keys, base + cy - r, side="left")
      hi = np.searchsorted(self._keys, base + cy + r, side="right")
      if hi > lo:
        out.append(self._order[lo:hi])
    if not out:
      return np.zeros(0, np.int64)
    idx = np.concatenate(out)
    d2 = (self.x[idx] - x) ** 2 + (self.y[idx] - y) ** 2
    return idx[d2 <= radius * radius]

  def project(self, i, x, y):
    """(along-trace offset in metres from point i, signed lateral metres, left positive) of (x, y) against point i's heading."""
    dx, dy = x - self.x[i], y - self.y[i]
    c, s = math.cos(self.hdg[i]), math.sin(self.hdg[i])
    return dx * c + dy * s, -dx * s + dy * c

  def ahead(self, t, i, dist_m, max_traces=8):
    """Points from point i of trace t onward for up to dist_m metres, following trace_next links (each only once).
    -> (point indices, distance ahead of point i in metres)."""
    idx, dist = [], []
    d0 = 0.0
    seen = set()
    cur_t, cur_i = t, i
    while d0 <= dist_m and cur_t >= 0 and cur_t not in seen and len(seen) < max_traces:
      seen.add(cur_t)
      end = self.trace_end(cur_t)
      k = min(end - cur_i, int(math.floor((dist_m - d0) / self.spacing)) + 1)
      if k <= 0:
        break
      idx.append(np.arange(cur_i, cur_i + k))
      dist.append(d0 + self.spacing * np.arange(k))
      d0 += self.spacing * (end - cur_i)             # the next trace starts about one spacing after this one's last point
      nxt = int(self.trace_next[cur_t])
      if nxt < 0:
        break
      cur_t, cur_i = nxt, int(self.trace_start[nxt])
    if not idx:
      return np.zeros(0, np.int64), np.zeros(0)
    return np.concatenate(idx), np.concatenate(dist)


def load(path):
  """The memory at `path`, checked; raises on anything malformed."""
  with np.load(path, allow_pickle=False) as z:
    meta = json.loads(str(z["meta"]))
    if int(meta.get("version", -1)) != VERSION:
      raise ValueError(f"memory: version {meta.get('version')} (expected {VERSION})")
    arrays = {k: z[k] for k in KEYS + ("grade", "v_proven") if k in z.files}
  missing = [k for k in KEYS if k not in arrays and k not in OPTIONAL_KEYS]
  if missing:
    raise ValueError(f"memory: missing {missing}")
  return Memory(arrays, meta)


def save(path, arrays, meta):
  """Write a memory file (the builder and the tests). `arrays` as described in the module docstring."""
  meta = dict(meta)
  meta.setdefault("version", VERSION)
  meta.setdefault("spacing_m", SPACING_M)
  out = {
    "meta": np.array(json.dumps(meta)),
    "x": np.asarray(arrays["x"], np.float32), "y": np.asarray(arrays["y"], np.float32),
    "hdg": np.asarray(arrays["hdg"], np.float32), "kap": np.asarray(arrays["kap"], np.float32),
    "v": np.asarray(arrays["v"], np.float16), "flags": np.asarray(arrays["flags"], np.uint8),
    "v_flag": np.asarray(arrays["v_flag"], np.float16), "npass": np.asarray(np.clip(arrays["npass"], 0, 255), np.uint8),
    "bank": np.asarray(arrays.get("bank", np.full(len(arrays["x"]), np.nan)), np.float16),
    "trace_start": np.asarray(arrays["trace_start"], np.int32), "trace_next": np.asarray(arrays["trace_next"], np.int32),
  }
  if "grade" in arrays:
    out["grade"] = np.asarray(arrays["grade"], np.float16)
  if "v_proven" in arrays:
    out["v_proven"] = np.asarray(arrays["v_proven"], np.float16)
  with open(path, "wb") as f:
    np.savez_compressed(f, **out)


# ---- synthetic roads (tests and the dry run only; never the owner's roads) -----------------------------------------------------------
def synthetic_trace(segments, x0=0.0, y0=0.0, h0=0.0, spacing=SPACING_M):
  """Points of a road made of (length m, curvature 1/m) pieces, starting at (x0, y0) heading h0. -> dict of x, y, hdg, kap arrays."""
  xs, ys, hs, ks = [], [], [], []
  x, y, h = x0, y0, h0
  for length, k in segments:
    n = int(round(length / spacing))
    for _ in range(n):
      xs.append(x)
      ys.append(y)
      hs.append(h)
      ks.append(k)
      h2 = h + k * spacing
      if abs(k) > 1e-9:
        x += (math.sin(h2) - math.sin(h)) / k
        y += (math.cos(h) - math.cos(h2)) / k
      else:
        x += spacing * math.cos(h)
        y += spacing * math.sin(h)
      h = h2
  return {"x": np.array(xs), "y": np.array(ys), "hdg": np.array([wrap(a) for a in hs]), "kap": np.array(ks)}


def synthetic(traces, v=25.0, nexts=None, flags=None, v_flag=None):
  """A Memory from synthetic traces (each a dict from synthetic_trace). flags / v_flag: optional per-trace arrays."""
  cat = lambda k: np.concatenate([t[k] for t in traces])
  n = sum(len(t["x"]) for t in traces)
  starts = np.r_[0, np.cumsum([len(t["x"]) for t in traces])]
  fl = np.concatenate(flags) if flags is not None else np.zeros(n, np.uint8)
  vf = np.concatenate(v_flag) if v_flag is not None else np.full(n, np.nan)
  arrays = {"x": cat("x"), "y": cat("y"), "hdg": cat("hdg"), "kap": cat("kap"), "v": np.full(n, v), "flags": fl, "v_flag": vf,
            "npass": np.full(n, 3), "bank": np.full(n, np.nan), "trace_start": starts,
            "trace_next": np.array(nexts if nexts is not None else [-1] * len(traces))}
  return Memory(arrays, {"version": VERSION, "spacing_m": SPACING_M, "lat0": 0.0, "lon0": 0.0, "kx": 111320.0, "ky": 110574.0})
