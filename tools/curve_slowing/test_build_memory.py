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


def test_second_sharpest_kap_and_npass():
  passes = [make_pass(f'd{i}', CURVE, y0=dy, kap_bias=b) for i, (dy, b) in enumerate([(0.0, 0.002), (2.0, 0.0), (-2.0, -0.001)])]
  arrays, st = B.build(passes)
  assert st['traces'] == 1
  mid = len(arrays['x']) // 2
  assert np.all(arrays['npass'][5:-5] == 3)
  # 3 passes: the second sharpest |kap| (0.009, 0.010, 0.012 on the arc -> 0.010), signed by the median
  i_arc = int(round(275 / M.SPACING_M))
  assert abs(arrays['kap'][i_arc] - 0.010) < 3e-4
  # 2 passes: the second sharpest is the gentler one
  arrays2, _ = B.build(passes[1:])
  assert abs(arrays2['kap'][i_arc] - 0.009) < 3e-4
  # 1 pass: its own value
  arrays1, _ = B.build(passes[:1])
  assert abs(arrays1['kap'][i_arc] - 0.012) < 3e-4
  assert mid > 0 and abs(len(arrays['x']) * M.SPACING_M - 600) < 20


def test_flag_spreading_and_v_flag():
  # the arc is 200-350 m; a takeover on the straight at 500 m flags nothing, one at 260 m only the arc's points
  passes = [make_pass('d0', CURVE), make_pass('d1', CURVE, sats=[300.0], tks=[500.0]), make_pass('d2', CURVE, tks=[260.0])]
  arrays, _ = B.build(passes)
  g = np.arange(len(arrays['x'])) * M.SPACING_M
  lim = (arrays['flags'] & M.FLAG_LIMIT) > 0
  tk = (arrays['flags'] & M.FLAG_TAKEOVER) > 0
  assert lim[np.abs(g - 300) <= 70].all() and not lim[np.abs(g - 300) >= 90].any()
  assert not tk[g >= 400].any()
  assert tk[(g >= 215) & (g <= 315)].all() and not tk[g <= 190].any()
  assert np.all(np.abs(arrays['kap'][tk]) >= B.BEND_KAP)
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


def test_second_sharpest_rule():
  key = np.array([0, 0, 1, 1, 1, 1, 2])
  val = np.array([0.01, -0.02, -0.004, -0.005, 0.03, -0.006, 0.007])
  out = B.robust_kap(key, val, 4)
  assert out[0] == -0.01                    # second sharpest of two (0.01), signed by the median (-0.005)
  # one sharp outlier (0.03) cannot set the corner; the passes disagree by more than 2x, so the lowest |kap|, signed by the median
  assert out[1] == -0.004
  assert out[2] == 0.007 and np.isnan(out[3])
  # agreeing passes (within 2x): the second sharpest
  out2 = B.robust_kap(np.zeros(3, int), np.array([0.010, 0.012, 0.015]), 1)
  assert out2[0] == 0.012


def test_stop_mask_and_lane_events():
  n = 600
  v = np.full(n, 15.0)
  v[200:300] = np.linspace(15.0, 0.0, 100)        # -3 m/s^2 to a stop
  v[300:340] = 0.0
  v[340:440] = np.linspace(0.0, 15.0, 100)        # pull away
  s = np.r_[0, np.cumsum(v[:-1] * B.DT)]
  m = B.stop_mask(v, s)
  assert m[205:300].all() and m[300:340].all() and m[340:395].all()   # 19 mph = 8.5 m/s reached at ~397
  assert not m[:195].any() and not m[460:].any()
  v2 = np.full(n, 15.0); v2[200:260] = np.linspace(15.0, 12.0, 60)    # a brake that does not end in a stop
  assert not B.stop_mask(v2, np.r_[0, np.cumsum(v2[:-1] * B.DT)]).any()
  d = np.full(n, 1.6); lat = np.ones(n); blink = np.zeros(n)
  d[100:110] = 0.7                                 # 0.5 s on the line
  d[200:203] = 0.5                                 # 0.15 s: too short
  d[400:420] = 0.6; blink[480] = 1.0              # blinker within 5 s: a lane change
  assert list(B.lane_events(d, lat, blink)) == [100]
  lat[:] = 0.0
  assert len(B.lane_events(d, lat, blink)) == 0   # openpilot not steering


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


def test_grade_median_and_optional_key(tmp_path):
  passes = []
  for i, gr in enumerate((0.02, 0.03, 0.05)):
    pp = make_pass(f'd{i}', CURVE)
    pp['grade'] = np.full(len(pp['s']), gr)
    passes.append(pp)
  arrays, _ = B.build(passes)
  assert np.allclose(arrays['grade'][5:-5], 0.03)
  p = tmp_path / 'g.npz'
  M.save(p, arrays, {'lat0': 0.0, 'lon0': 0.0, 'kx': 1.0, 'ky': 1.0})
  assert np.allclose(M.load(p).grade[5:-5], 0.03, atol=1e-4)
  del arrays['grade']
  M.save(p, arrays, {'lat0': 0.0, 'lon0': 0.0, 'kx': 1.0, 'ky': 1.0})
  m = M.load(p)
  assert m.grade.dtype == np.float64 and np.all(m.grade == 0.0)
  assert np.isnan(B.build(passes[:1] + [make_pass('d9', CURVE)])[0]['grade']).sum() == 0     # one pass with grade is enough



def _straight_with_swerve(drive, **kw):
  """1000 m straight road east; the pass has a 1/50 m curvature blip at 400-440 m (a lane change or swerve)"""
  pp = make_pass(drive, [(1000, 0.0)], **kw)
  blip = (pp['s'] >= 400) & (pp['s'] <= 440)
  pp['kap'] = np.where(blip, 0.02, pp['kap'])
  return pp, blip


def test_exclusions_blinker_hand_slow():
  g400 = int(round(420 / M.SPACING_M))
  for key in ('blink', 'pressed', 'drv', 'slow'):
    pp, blip = _straight_with_swerve('d0')
    if key == 'slow':
      pp['v'] = np.where(blip, 5.0, pp['v'])            # below 15 mph through the blip
    else:
      sig = np.zeros(len(pp['s']))
      sig[np.where(blip)[0][0]] = 6.0 if key == 'drv' else 1.0   # one sample: padded to +-3 s / +-2 s (30 m / 20 m at 10 m/s)
      pp[key] = sig
    info = {}
    arrays, _ = B.build([pp], info)
    assert abs(arrays['kap'][g400]) < 1e-9 and arrays['flags'][g400] & M.FLAG_NOKAP, key
    assert arrays['npass'][g400] == 1 and not arrays['flags'][60] & M.FLAG_NOKAP      # 300 m: clear of the start and the blip
  # a second, clean pass supplies the curvature where the first is excluded
  a, blip = _straight_with_swerve('d0')
  a['blink'] = blip.astype(float)
  b = make_pass('d1', [(1000, 0.0)], y0=1.0)
  arrays, _ = B.build([a, b])
  assert abs(arrays['kap'][g400]) < 1e-4 and not arrays['flags'][g400] & M.FLAG_NOKAP and arrays['npass'][g400] == 2


def test_one_swerve_does_not_make_a_corner():
  a, _ = _straight_with_swerve('d0')
  b = make_pass('d1', [(1000, 0.0)], y0=1.0)
  arrays, _ = B.build([a, b])
  assert abs(arrays['kap'][int(round(420 / M.SPACING_M))]) < 1e-4      # second sharpest of (0.02, 0.0)


def test_last_150_m_before_leaving_excluded():
  # road A 1000 m east (drive 0). Drive 1 drives A to x = 500 with a bend just before, then turns off south (a new road).
  a = make_pass('d0', [(1000, 0.0)])
  off = make_pass('d1', [(420, 0.0), (40, 0.004), (40, 0.0), (math.pi / 2 * 30, -1 / 30), (300, 0.0)])
  info = {}
  arrays, _ = B.build([a, off], info)
  assert info['excluded_by_reason']['last 150 m before leaving'] > 0
  x, kap = arrays['x'], arrays['kap']
  near = (np.abs(arrays['y']) < 1) & (x > 420) & (x < 460)
  assert np.all(np.abs(kap[near]) < 1e-4)          # the bend of the pass that left A is not A's curvature
  far = (np.abs(arrays['y']) < 1) & (x > 100) & (x < 300)
  assert np.all(arrays['npass'][far] == 2)


def test_v_proven():
  clean = make_pass('d0', CURVE)
  fast = make_pass('d1', CURVE, sats=[300.0])
  fast['v'] = np.full(len(fast['s']), 20.0)
  fast['lat_act'] = np.ones(len(fast['s']))
  off = make_pass('d2', CURVE)
  off['v'] = np.full(len(off['s']), 30.0)
  off['lat_act'] = np.zeros(len(off['s']))          # lateral control off: proves nothing
  arrays, _ = B.build([clean, fast, off])
  g = np.arange(len(arrays['x'])) * M.SPACING_M
  assert np.allclose(arrays['v_proven'][np.abs(g - 300) <= 75], V, atol=0.01)       # the 20 m/s pass had a limit event within 80 m
  assert np.allclose(arrays['v_proven'][g >= 400], 20.0, atol=0.01)
  p = '/nonexistent'
  del p
  arrays2, _ = B.build([off])
  assert np.isnan(arrays2['v_proven']).all()


def test_v_proven_optional_in_loader(tmp_path):
  arrays, _ = B.build([make_pass('d0', CURVE)])
  f = tmp_path / 'v.npz'
  M.save(f, arrays, {'lat0': 0.0, 'lon0': 0.0, 'kx': 1.0, 'ky': 1.0})
  assert np.allclose(M.load(f).v_proven, V, atol=0.01)
  del arrays['v_proven']
  M.save(f, arrays, {'lat0': 0.0, 'lon0': 0.0, 'kx': 1.0, 'ky': 1.0})
  m = M.load(f)
  assert m.v_proven.dtype == np.float64 and np.isnan(m.v_proven).all()


def test_lane_flag_and_v_proven():
  a = make_pass('d0', CURVE)
  a['lat_act'] = np.ones(len(a['s']))
  b = make_pass('d1', CURVE)
  b['v'] = np.full(len(b['s']), 20.0)
  b['lat_act'] = np.ones(len(b['s']))
  b['lane_i'] = np.array([int(round(300 / (V * B.DT)))])      # a lane-line touch at 300 m
  arrays, _ = B.build([a, b])
  g = np.arange(len(arrays['x'])) * M.SPACING_M
  lane = (arrays['flags'] & M.FLAG_LANE) > 0
  assert lane[np.abs(g - 300) <= 40].all() and not lane[np.abs(g - 300) >= 70].any() and not lane[g >= 365].any()
  assert np.allclose(arrays['v_flag'][lane], 20.0, atol=0.01)
  assert np.allclose(arrays['v_proven'][np.abs(g - 300) <= 75], V, atol=0.01)     # the faster pass touched the line: not proven
  assert np.allclose(arrays['v_proven'][(g >= 400) & (g <= 550)], 20.0, atol=0.01)


def test_takeover_not_a_brake_to_a_stop():
  n = 400
  lat = np.ones(n); en = np.ones(n); ti = np.zeros(n); pressed = np.zeros(n); drv = np.zeros(n); brk = np.zeros(n)
  drv[100:120] = 8.0; lat[125:] = 0.0
  v = np.full(n, 15.0)
  s = np.r_[0, np.cumsum(v[:-1] * B.DT)]
  assert list(B.run_events(ti, lat, pressed, en, drv, brk=brk, v=v, s=s)[1]) == [100]
  brk2 = brk.copy(); brk2[125] = 1.0                            # disengaged by the brake
  assert len(B.run_events(ti, lat, pressed, en, drv, brk=brk2, v=v, s=s)[1]) == 0
  v2 = v.copy(); v2[150:] = 2.0                                  # stops within 150 m (onset at 75 m, stop from ~111 m)
  s2 = np.r_[0, np.cumsum(v2[:-1] * B.DT)]
  assert len(B.run_events(ti, lat, pressed, en, drv, brk=brk, v=v2, s=s2)[1]) == 0


def test_kap_med_and_loader_fallback(tmp_path):
  passes = [make_pass(f'd{i}', CURVE, y0=dy, kap_bias=b) for i, (dy, b) in enumerate([(0.0, 0.002), (2.0, 0.0), (-2.0, -0.001)])]
  arrays, _ = B.build(passes)
  i_arc = int(round(275 / M.SPACING_M))
  assert abs(arrays['kap_med'][i_arc] - 0.010) < 3e-4
  f = tmp_path / 'k.npz'
  M.save(f, arrays, {'lat0': 0.0, 'lon0': 0.0, 'kx': 1.0, 'ky': 1.0})
  assert abs(M.load(f).kap_med[i_arc] - 0.010) < 3e-4
  del arrays['kap_med']
  M.save(f, arrays, {'lat0': 0.0, 'lon0': 0.0, 'kx': 1.0, 'ky': 1.0})
  m = M.load(f)
  assert m.kap_med.dtype == np.float64 and np.array_equal(m.kap_med, m.kap)


def test_trouble_points_keep_their_curvature():
  # two passes of the arc; the sharper one hit the steering limit at 300 m with the blinker on and the driver's hand on the wheel
  calm = make_pass('d0', CURVE)
  hot = make_pass('d1', CURVE, kap_bias=0.004, sats=[300.0])
  hot['blink'] = np.ones(len(hot['s']))                       # every sample of this pass would be excluded
  hot['pressed'] = np.ones(len(hot['s']))
  arrays, _ = B.build([calm, hot])
  i = int(round(300 / M.SPACING_M))
  assert abs(arrays['kap'][i] - 0.014) < 3e-4                  # the event pass's own (sharper) curvature, exclusions ignored
  assert arrays['flags'][i] & M.FLAG_LIMIT and not arrays['flags'][i] & M.FLAG_NOKAP
  assert abs(arrays['kap_med'][i] - 0.010) < 3e-4             # the median of the usable passes is unchanged
  # hand steering near the event is not excluded
  p = make_pass('d2', CURVE, sats=[300.0])
  p['pressed'] = np.ones(len(p['s']))
  xb = B.excl_bits(p)
  e = p['sat_i'][0]
  assert not (xb[e - 50:e + 50] & B.EX_HAND).any() and (xb[:e - 70] & B.EX_HAND).all()
