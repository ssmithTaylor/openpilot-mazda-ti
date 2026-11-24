"""The driving model's recorded preview for every replay segment the seg20 extraction does not have (position-free).

For each segment of the replay caches (DATA_DIR/cache/drive_*.npz) without a seg20 copy, reads its rlog once and writes
DATA_DIR/cache/model/<tag>.npz: pred_t (33 preview times, 0..10 s) and pred_la (n, 33) = orientationRate.z * velocity.x on the same
20 Hz grid as the segment's rl20 cache (t = 0 at the first carState), exactly as the seg20 extraction does. Used by unnecessary.py.

  python tools/curve_slowing/model_cache.py [--workers 12]
"""
import argparse
import glob
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_memory as B   # noqa: E402

DT = 0.05


def out_path(tag):
  return os.path.join(B.DATA_DIR, 'cache', 'model', tag + '.npz')


def read_one(args):
  tag, path = args
  if os.path.exists(out_path(tag)):
    return tag, 'cached'
  try:
    from cereal import log as clog
    with open(path, 'rb') as fh:
      evs = list(clog.Event.read_multiple_bytes(fh.read()))
    cs, mv, T = [], [], None
    for e in evs:
      w = e.which()
      if w == 'carState':
        cs.append(e.logMonoTime)
      elif w == 'modelV2':
        m = e.modelV2
        z = np.array(m.orientationRate.z, np.float32)
        vx = np.array(m.velocity.x, np.float32)
        if len(z) == 33 and len(vx) == 33:
          if T is None:
            T = np.array(m.orientationRate.t, np.float32)
          mv.append((e.logMonoTime, z * vx))
    if len(cs) < 500 or not mv:
      return tag, 'short'
    t0 = cs[0]
    g = np.arange(0, (cs[-1] - t0) * 1e-9, DT)
    mt = np.array([(x[0] - t0) * 1e-9 for x in mv])
    k = np.clip(np.searchsorted(mt, g, side='right') - 1, 0, len(mv) - 1)
    la = np.stack([x[1] for x in mv])[k].astype(np.float32)
    la[g < mt[0]] = np.nan
    os.makedirs(os.path.dirname(out_path(tag)), exist_ok=True)
    with open(out_path(tag), 'wb') as fh:
      np.savez_compressed(fh, pred_la=la, pred_t=T)
    return tag, 'read'
  except Exception as ex:
    return tag, 'error ' + type(ex).__name__


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--workers', type=int, default=12)
  a = ap.parse_args()
  want = set()
  for f in glob.glob(os.path.join(B.DATA_DIR, 'cache', 'drive_*.npz')):
    with np.load(f, allow_pickle=False) as R:
      want |= {str(t) for t in R['seg_tags'] if not os.path.exists(os.path.join(B.SEG20, str(t) + '.npz'))}
  tags, _ = B.recorded_tags()
  todo = [(t, tags[t]) for t in sorted(want) if t in tags and not os.path.exists(out_path(t))]
  print(f'{len(want)} segments without seg20; {len(todo)} to read; {len(want - set(tags))} without an rlog on this PC', flush=True)
  t0 = time.time()
  from collections import Counter
  from multiprocessing import Pool
  c = Counter()
  with Pool(a.workers) as pool:
    for i, (_, st) in enumerate(pool.imap_unordered(read_one, todo, chunksize=2)):
      c[st.split()[0]] += 1
      if (i + 1) % 50 == 0:
        print(f'  {i + 1}/{len(todo)} {dict(c)} {time.time() - t0:.0f} s', flush=True)
  print(f'done {dict(c)} in {time.time() - t0:.0f} s', flush=True)


if __name__ == '__main__':
  main()
