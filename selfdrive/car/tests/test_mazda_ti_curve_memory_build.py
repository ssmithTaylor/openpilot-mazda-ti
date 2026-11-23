"""tools/curve_slowing/build_memory.py on SYNTHETIC roads only (no recorded drive, no real position)."""
import math

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve_memory as M
from openpilot.tools.curve_slowing import build_memory as B

V = 10.0                    # m/s; at 20 Hz one sample every 0.5 m


def make_pass(drive, segments, x0=0.0, y0=0.0, h0=0.0, kap_bias=0.0, sats=(), tks=()):
  """a pass along a synthetic road: 20 Hz samples every 0.5 m; sats / tks are event distances in metres"""
  tr = M.synthetic_trace(segments, x0, y0, h0, spacing=V * B.DT)
  n = len(tr['x'])
  s = np.arange(n) * V * B.DT
  ev = lambda ds: np.array([int(round(d / (V * B.DT))) for d in ds], int)
  return dict(drive=drive, s=s, X=tr['x'], Y=tr['y'], v=np.full(n, V), kap=tr['kap'] + kap_bias, bank=np.full(n, np.nan),
              sat_i=ev(sats), tk_i=ev(tks))


CURVE = [(200, 0.0), (150, 0.01), (250, 0.0)]


def test_conservative_kap_and_npass():
  passes = [make_pass(f'd{i}', CURVE, y0=dy, kap_bias=b) for i, (dy, b) in enumerate([(0.0, 0.002), (2.0, 0.0), (-2.0, -0.001)])]
  arrays, st = B.build(passes)
  assert st['traces'] == 1
  mid = len(arrays['x']) // 2
  assert np.all(arrays['npass'][5:-5] == 3)
  # 3 passes: 90th percentile of |kap| (0.009, 0.010, 0.012 on the arc -> 0.0116), signed by the median
  i_arc = int(round(275 / M.SPACING_M))
  assert abs(arrays['kap'][i_arc] - 0.0116) < 3e-4
  # 2 passes: the sharpest one
  arrays2, _ = B.build(passes[1:])
  assert abs(arrays2['kap'][i_arc] - 0.010) < 3e-4
  assert abs(arrays2['kap'][5] - (-0.001)) < 3e-4
  assert mid > 0 and abs(len(arrays['x']) * M.SPACING_M - 600) < 20


def test_flag_spreading_and_v_flag():
  passes = [make_pass('d0', CURVE), make_pass('d1', CURVE, sats=[300.0], tks=[500.0])]
  arrays, _ = B.build(passes)
  g = np.arange(len(arrays['x'])) * M.SPACING_M
  lim = (arrays['flags'] & M.FLAG_LIMIT) > 0
  tk = (arrays['flags'] & M.FLAG_TAKEOVER) > 0
  assert lim[np.abs(g - 300) <= 70].all() and not lim[np.abs(g - 300) >= 90].any()
  assert tk[np.abs(g - 500) <= 50].all() and not tk[g <= 430].any()
  assert np.allclose(arrays['v_flag'][lim | tk], V, atol=0.01)
  assert np.isnan(arrays['v_flag'][~(lim | tk)]).all()


def test_rejoin_cuts_and_links(tmp_path):
  # road A: 1000 m east. Later drive: north up a side road, right turn (radius 30 m) onto A at x = 500, then along A.
  a = make_pass('d0', [(1000, 0.0)])
  side = make_pass('d1', [(270, 0.0), (math.pi / 2 * 30, -1 / 30), (400, 0.0)], x0=470, y0=-300, h0=math.pi / 2)
  arrays, st = B.build([a, side])
  ts, nx = arrays['trace_start'], arrays['trace_next']
  assert st['traces'] == 3                       # A before the join, the side road, A after the join
  firsts = ts[:-1]
  # the side trace ends at the join and links to the A piece starting there; A's first piece links to it too
  a2 = [t for t in range(3) if abs(arrays['y'][firsts[t]]) < 1 and arrays['x'][firsts[t]] > 400][0]
  side_t = [t for t in range(3) if arrays['y'][firsts[t]] < -100][0]
  a1 = ({0, 1, 2} - {a2, side_t}).pop()
  assert nx[side_t] == a2 and nx[a1] == a2 and nx[a2] == -1
  sl = slice(ts[a2], ts[a2 + 1])
  both = (arrays['x'][sl] > 520) & (arrays['x'][sl] < 880)
  assert np.all(arrays['npass'][sl][both] == 2) and np.all(arrays['npass'][sl][arrays['x'][sl] > 920] == 1)
  p = tmp_path / 'm.npz'
  M.save(p, arrays, {'lat0': 0.0, 'lon0': 0.0, 'kx': 1.0, 'ky': 1.0})
  m = M.load(p)
  idx, dist = m.ahead(side_t, int(ts[side_t + 1]) - 3, 100.0)
  assert m.trace_of[idx[-1]] == a2


def test_link_ambiguous_and_backwards():
  t0 = M.synthetic_trace([(100, 0.0)])                       # ends at x = 95
  t1 = M.synthetic_trace([(100, 0.0)], x0=100.0)             # starts 5 m ahead
  t2 = M.synthetic_trace([(100, 0.0)], x0=100.0, y0=8.0)     # another start 5 m ahead: a fork -> ambiguous
  t3 = M.synthetic_trace([(100, 0.0)], x0=200.0, h0=math.pi) # starts at t1's end but heading back: not a continuation
  cat = lambda tr, k: np.concatenate([t[k] for t in tr])
  starts = np.r_[0, np.cumsum([len(t['x']) for t in (t0, t1, t2, t3)])]
  nx = B.link_traces(cat((t0, t1, t2, t3), 'x'), cat((t0, t1, t2, t3), 'y'), cat((t0, t1, t2, t3), 'hdg'), starts)
  assert nx[0] == -1 and nx[1] == -1
  starts2 = np.r_[0, np.cumsum([len(t['x']) for t in (t0, t1)])]
  nx2 = B.link_traces(cat((t0, t1), 'x'), cat((t0, t1), 'y'), cat((t0, t1), 'hdg'), starts2)
  assert list(nx2) == [1, -1]


def test_short_roads_dropped_and_split_points():
  arrays, st = B.build([make_pass('d0', [(60, 0.0)]), make_pass('d1', [(300, 0.0)], y0=500.0)])
  assert st['traces'] == 1 and len(arrays['x']) == 60
  assert B.split_points(100, {0, 3, 30, 35, 95}) == [30]


def test_group_median_ignores_nan():
  out = B.group_median(np.array([0, 0, 0, 2, 2]), np.array([1.0, 5.0, np.nan, 2.0, 4.0]), 3)
  assert out[0] == 3.0 and np.isnan(out[1]) and out[2] == 3.0


def test_conservative_kap_rule():
  key = np.array([0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
  val = np.array([0.01, -0.02] + [0.001 * i for i in range(1, 11)])
  out = B.conservative_kap(key, val, 2)
  assert out[0] == -0.02
  assert abs(out[1] - np.percentile(np.abs(val[2:]), 90)) < 1e-12


def test_firm_takeover_events():
  n = 200
  lat = np.ones(n); en = np.ones(n); ti = np.zeros(n); pressed = np.zeros(n); drv = np.zeros(n)
  drv[20:40] = 8.0; lat[45:] = 0.0          # firm hold 1 s, lateral control drops 0.25 s later -> takeover
  drv[100:104] = 9.0                        # brief bump: not sustained
  drv[150:170] = 6.0                        # sustained but control stays on: not a takeover (lat already off -> skipped too)
  sats, tks = B.run_events(ti, lat, pressed, en, drv)
  assert list(tks) == [20] and len(sats) == 0
  lat2 = np.ones(n); drv2 = np.zeros(n); drv2[50:70] = -7.0
  assert len(B.run_events(ti, lat2, pressed, en, drv2)[1]) == 0
  en2 = np.ones(n); en2[80:] = 0.0          # brake disengage within 2 s
  assert list(B.run_events(ti, lat2, pressed, en2, drv2)[1]) == [50]


def test_flag_carried_across_link():
  # road A 300 m; a later drive turns off A... simpler: event near the end of road A whose window crosses into the continuing trace
  a = make_pass('d0', [(1000, 0.0)])
  side = make_pass('d1', [(270, 0.0), (math.pi / 2 * 30, -1 / 30), (400, 0.0)], x0=470, y0=-300, h0=math.pi / 2, sats=[300.0])
  arrays, _ = B.build([a, side])
  ts, nx = arrays['trace_start'], arrays['trace_next']
  side_t = [t for t in range(len(ts) - 1) if arrays['y'][ts[t]] < -100][0]
  u = nx[side_t]
  assert u >= 0
  lim = (arrays['flags'][ts[u]:ts[u + 1]] & M.FLAG_LIMIT) > 0
  assert lim[:3].all()
