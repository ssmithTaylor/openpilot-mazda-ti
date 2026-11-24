"""tools/curve_slowing/ground_truth.py and the replay's must-catch check on SYNTHETIC runs only (no recorded drive, no position)."""
import json
import os

import numpy as np
import pytest

from openpilot.tools.curve_slowing import ground_truth as G
from openpilot.tools.curve_slowing import unnecessary as U

DT = G.DT


def _run(n=1200, bend=(400, 600), la=2.4, v=20.0, sat=None, lane=None, tk=(), lat=True):
  kap = np.zeros(n)
  kap[bend[0]:bend[1]] = la / v ** 2
  ti = np.zeros(n)
  if sat is not None:
    ti[sat[0]:sat[1]] = 600.0
  lane_d = np.full(n, 1.8)
  if lane is not None:
    lane_d[lane[0]:lane[1]] = 0.8                   # the car's side 0.12 m over the line
  return dict(v=np.full(n, v), kap=kap, lat_act=np.full(n, 1.0 if lat else 0.0), pressed=np.zeros(n), blink=np.zeros(n),
              lane_d=lane_d, ti=ti, tk_i=np.asarray(tk, int), off=np.array([0, 600]), tags=['d-0', 'd-1'])


def test_trouble_needs_two_seconds_at_the_limit_and_a_takeover_or_excursion():
  p, = G.passes_of_run(_run(sat=(420, 470), lane=(450, 460)))           # 2.5 s at the limit, 0.5 s over the line
  assert p['car'] and p['trouble'] and p['excursion'] and not p['takeover']
  p, = G.passes_of_run(_run(sat=(420, 450), lane=(450, 460)))           # 1.5 s at the limit: not trouble
  assert not p['trouble']
  p, = G.passes_of_run(_run(sat=(420, 470)))                            # at the limit, held: not trouble
  assert not p['trouble']
  p, = G.passes_of_run(_run(sat=(420, 470), tk=(590,)))                 # firm takeover
  assert p['trouble'] and p['takeover']
  p, = G.passes_of_run(_run(sat=(420, 470), lane=(450, 454)))           # 0.2 s over the line: not an excursion
  assert not p['excursion'] and not p['trouble']
  p, = G.passes_of_run(_run(sat=(420, 470), tk=(590,), lat=False))      # not car-steered: never trouble
  assert not p['car'] and not p['trouble']


def test_bend_pass_definition_and_limit_onset():
  assert G.passes_of_run(_run(la=1.2)) == []                           # below 1.3 m/s^2
  assert G.passes_of_run(_run(bend=(400, 415))) == []                  # under 1 s
  r = _run(sat=(430, 480))
  r['ti'][410:412] = 600.0                                              # a 0.1 s touch before the sustained limit
  p, = G.passes_of_run(r)
  assert p['limit_on'] == 430 and p['tag'] == 'd-0' and p['t_seg'] == pytest.approx(p['i'] * DT)


def test_must_catch_reads_the_private_file(tmp_path):
  f = tmp_path / 'must_catch.json'
  f.write_text(json.dumps([{"route": "d", "segment": 0, "t0": 20.0, "t1": 30.0}]))
  p = dict(tag='d-0', t_seg=25.0, t_a=20.0, a=400, b=600, trouble=True, limit_on=430, car=True, std=True)
  res = dict(drive='d', passes=[p], s3=[dict(i=440, p=0)], alerts=[], slow=[])
  out, = U.must_catch([res], str(f))
  assert out['covered'] and out['alert_lag_s'] == pytest.approx(0.5)
  res['s3'] = [dict(i=460, p=0)]
  assert not U.must_catch([res], str(f))[0]['covered']                   # 1.5 s after the onset


MUST_CATCH = os.path.join(U.OUT_DIR, 'must_catch.json')


@pytest.mark.skipif(not (os.path.exists(MUST_CATCH) and os.path.exists(G.RUNS_DIR)), reason='private must-catch file / recorded runs absent')
def test_must_catch_0342_s15():
  # plan T9: the one must-catch bend (private file: route, segment, seconds only); stage 3 must alert within 1 s of the limit's onset
  entries = json.load(open(MUST_CATCH))
  routes = sorted({e['route'] for e in entries})
  R = [U.replay_drive((d, U.MEM_DIR + '/curve_memory_all_v6.npz', dict(_prov={}))) for d in routes]
  out = U.must_catch(R, MUST_CATCH)
  assert out and all(o['covered'] for o in out), out
