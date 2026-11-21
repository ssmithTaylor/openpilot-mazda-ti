"""Steering identification tests (ti_ident.py): signal tables, generator, gating, band bookkeeping with the tests
accountant, reconstruction from the logged events, and the car controller's steering block with them."""
import importlib.util
import json
import math
import random
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from cereal import car

from openpilot.selfdrive.car.mazda import ti_dither as td
from openpilot.selfdrive.car.mazda import ti_ident as ti
from openpilot.selfdrive.car.mazda import ti_tests as tt
from openpilot.selfdrive.car.mazda.ti_ident import TiIdent

ROOT = Path(__file__).resolve().parents[3]
MPH = 0.44704
TI_MAX = 600
MAX_HELD_FRAMES = 40     # longest run of one non-zero integer value at full amplitude (the stuck fault needs seconds)


def v_for(band):
  lo, hi = next((b[0], b[1]) for b in ti.BANDS if b[2] == band)
  return (lo + hi) / 2 * MPH


def blocked_for(band):
  return next(b[3] for b in ti.BANDS if b[2] == band)


def quiet(band="31-45", **over):
  q = dict(v_ego=v_for(band), steering_pressed=False, driver_torque=-1.0, blinker=False, base_request=50,
           ti_max=TI_MAX, lat_demand=0.2, curvature=0.0003, lkas_blocked=blocked_for(band), room=6)
  q.update(over)
  return q


class Progress:
  """The accountant's `remaining`, in memory."""
  def __init__(self):
    self.have = {}

  def remaining(self, key, band, step):
    return max(0.0, ti.STEP_SECONDS[key] - self.have.get((key, band, step), 0.0))

  def add(self, credit):
    if credit is not None:
      k = credit[:3]
      self.have[k] = self.have.get(k, 0.0) + credit[3]


def settle(g, key, band, frame0=0, n=None, prog=None):
  """Run quiet frames (regime settles in 3 s); returns outputs."""
  prog = prog or Progress()
  out = []
  for k in range(n if n is not None else 400):
    out.append(g.update(key, frame0 + k, **quiet(band), remaining=prog.remaining))
    prog.add(g.credit)
  return out


# ---- tables ----------------------------------------------------------------------------------------------------

@pytest.mark.parametrize('key', ti.KEYS)
def test_every_chunk_is_exactly_zero_mean_bounded_and_slew_limited(key):
  for s in range(ti.n_steps(key)):
    f, a, n, lead = ti.step_params(key, s)
    w = [ti.value(key, s, ph, 1.0) for ph in range(n)]
    assert sum(w) == 0, (key, s)                         # exact, integer counts, every chunk
    assert max(abs(x) for x in w) <= a
    full = [ti.unit(key, s, ph) for ph in range(-lead, n)]
    gains = [ti.gain((ph + lead) / lead) if ph < 0 and lead else 1.0 for ph in range(-lead, n)]
    x = np.array(full) * np.array(gains)
    assert np.max(np.abs(np.diff(x))) <= ti.MAX_SLEW + 1e-9, (key, s)
    if f is not None:
      assert abs(f * n * ti.DT - round(f * n * ti.DT)) < 1e-9   # whole cycles: seamless chunk to chunk
      assert ti.unit(key, s, 0) == 0.0 and abs(ti.unit(key, s, n - 1) + ti.unit(key, s, 1)) < 1e-9
    else:
      assert len(ti.doublet(a)) < n                       # a quiet settling gap after every doublet


@pytest.mark.parametrize('key', ti.KEYS)
def test_never_a_held_non_zero_value_for_long(key):
  for s in range(ti.n_steps(key)):
    n = ti.step_params(key, s)[2]
    w = [ti.value(key, s, ph, 1.0) for ph in range(n)] * 2
    run, longest = 1, 0
    for a, b in zip(w, w[1:]):
      run = run + 1 if a == b else 1
      if b != 0:
        longest = max(longest, run)
    assert longest <= MAX_HELD_FRAMES, (key, s, longest)


def test_amplitudes_and_their_classes():
  # Sweep and pulses' small step in the dither's class (preset 1 peaks at 22 counts); the ladder and the large pulse
  # step above it, justified in the drive card / report. Nothing near the 600 clip even on top of a 300-count request.
  assert max(a for _, a in ti.SWEEP_TONES) <= td.peak_counts()
  assert ti.PULSE_AMPS[0] <= td.peak_counts()
  assert max(ti.LADDER_AMPS) == 92 and max(ti.LADDER_AMPS) + td.MAX_BASE_FRACTION * TI_MAX < TI_MAX
  freqs = [f for f, _ in ti.SWEEP_TONES]
  assert freqs[0] == pytest.approx(0.3) and freqs[-1] == pytest.approx(5.0) and freqs == sorted(freqs)
  assert list(ti.LADDER_AMPS) == sorted(ti.LADDER_AMPS)
  assert min(ti.LADDER_AMPS) * 0.0544 < 0.5 < 4.0 < max(ti.LADDER_AMPS) * 0.0544   # below the response, above friction
  # Per-band ladder caps fall with speed
  caps = [ti.LADDER_CAPS[b[2]] for b in ti.BANDS]
  assert caps == sorted(caps, reverse=True)


def test_capture_time_per_band_is_short():
  per_band = {k: {b[2]: ti.steps_in_band(k, b[2]) * ti.STEP_SECONDS[k] for b in ti.BANDS} for k in ti.KEYS}
  assert per_band[ti.SWEEP] == {"19-28": 140, "31-45": 140, "45-60": 140, "60-80": 140}
  assert per_band[ti.LADDER] == {"19-28": 70, "31-45": 40, "45-60": 40, "60-80": 40}
  assert per_band[ti.PULSES] == {"19-28": 80, "31-45": 80, "45-60": 80, "60-80": 80}
  for k in ti.KEYS:      # a step is a whole number of chunks
    for s in range(ti.n_steps(k)):
      n = ti.step_params(k, s)[2]
      assert (ti.STEP_SECONDS[k] / (n * ti.DT)) == int(ti.STEP_SECONDS[k] / (n * ti.DT))


def test_shipped_definitions_match_the_generator():
  tests, errors = tt.load_definitions()
  assert errors == []
  by_key = {t.settings[0][0]: t for t in tests}
  for k in ti.KEYS:
    t = by_key[k]
    assert t.metric == "step_seconds" and t.at_ignition_off and t.target_seconds == ti.STEP_SECONDS[k]
    assert [b[2] for b in t.bins] == [b[2] for b in ti.BANDS]
    assert t.steps == tuple(ti.steps_in_band(k, b[2]) for b in ti.BANDS)
  # A definition that disagrees with the code is ignored as a whole
  doc = json.loads(tt.DEFINITIONS_PATH.read_text(encoding='utf-8'))
  bad = doc["tests"][2]
  bad["progress"]["steps"] = [7, 7, 7, 7]
  t, why = tt.validate_test(bad)
  assert t is None and "steps must be" in why
  bad["progress"]["steps"] = [7, 4, 4]
  assert tt.validate_test(bad)[1] == "steps must list one count per bin"
  good = doc["tests"][1]
  for change, text in ((dict(target_seconds=30), "target_seconds must be"), (dict(metric="engaged_seconds"),
                       "steps go with step_seconds")):
    t = json.loads(json.dumps(good))
    t["progress"].update(change)
    assert text in tt.validate_test(t)[1]
  t = json.loads(json.dumps(good))
  del t["progress"]["steps"]
  assert tt.validate_test(t)[1] == "step_seconds needs bins and steps"
  t = json.loads(json.dumps(good))
  t["settings"] = [{"key": "TiDither", "on": True}]
  assert "does not report steps" in tt.validate_test(t)[1]


# ---- gate and envelope -----------------------------------------------------------------------------------------

def test_off_or_no_band_or_unsettled_regime_is_zero():
  g = TiIdent()
  assert all(g.update(None, k, **quiet()) == 0 for k in range(1000))
  g = TiIdent()
  out = [g.update(ti.SWEEP, k, **quiet()) for k in range(299)]
  assert not any(out) and g.reason == "regime"           # 3 s for the regime to settle
  out = [g.update(ti.SWEEP, 299 + k, **quiet()) for k in range(200)]
  assert any(out) and g.reason == "ok"
  for v_mph, reason in ((10, "band"), (18.9, "band"), (29.5, "band"), (85, "band")):
    g = TiIdent()
    out = [g.update(ti.SWEEP, k, **quiet(v_ego=v_mph * MPH)) for k in range(800)]
    assert not any(out) and g.reason == reason


@pytest.mark.parametrize('band', [b[2] for b in ti.BANDS])
def test_the_regime_must_be_the_bands(band):
  g = TiIdent()
  out = [g.update(ti.SWEEP, k, **quiet(band, lkas_blocked=not blocked_for(band))) for k in range(800)]
  assert not any(out) and g.reason == "regime"
  g = TiIdent()
  out = [g.update(ti.SWEEP, k, **quiet(band, lkas_blocked=None)) for k in range(800)]
  assert not any(out)


def _running(key=ti.SWEEP, band="31-45", prog=None):
  g = TiIdent()
  settle(g, key, band, n=500, prog=prog)
  assert g.running and g.env == 1.0 and g.reason == "ok"
  return g, 500


@pytest.mark.parametrize('touch', [dict(steering_pressed=True), dict(driver_torque=6.0), dict(driver_torque=-7.0)])
def test_a_touch_zeroes_at_once_and_holds_off(touch):
  g, k = _running()
  assert g.update(ti.SWEEP, k, **quiet(**touch)) == 0 and not g.running
  assert g.holdoff == int(td.HOLDOFF_S / ti.DT)
  out = [g.update(ti.SWEEP, k + 1 + i, **quiet()) for i in range(g.holdoff)]
  assert not any(out)
  out = [g.update(ti.SWEEP, k + 400 + i, **quiet()) for i in range(200)]
  assert any(out)


def test_a_small_reading_must_persist_like_the_dithers():
  g, k = _running()
  for i in range(td.TOUCH_CONFIRM_FRAMES - 1):
    g.update(ti.SWEEP, k + i, **quiet(driver_torque=4.0))
    assert g.running and not g.touch
  g.update(ti.SWEEP, k + 2, **quiet())
  g, k = _running()
  out = [g.update(ti.SWEEP, k + i, **quiet(driver_torque=-4.0)) for i in range(td.TOUCH_CONFIRM_FRAMES)]
  assert out[-1] == 0 and not g.running and g.touch


@pytest.mark.parametrize('change,reason', [
  (dict(blinker=True), "blinker"), (dict(curvature=0.0045), "curve"), (dict(lat_demand=1.2), "curve"),
  (dict(base_request=301), "command"), (dict(v_ego=29 * MPH), "band"),
])
def test_other_gate_conditions_fade_out_within_a_quarter_second(change, reason):
  g, k = _running()
  out = [g.update(ti.SWEEP, k + i, **quiet(**change)) for i in range(100)]
  assert g.reason == reason
  assert not any(out[int(td.FADE_OUT_S / ti.DT) + 1:]) and not g.running


def test_too_little_rate_room_zeroes_at_once():
  g, k = _running()
  assert g.update(ti.SWEEP, k, **quiet(room=5)) == 0 and not g.running and g.reason == "rate"
  limits = NS(TI_STEER_MAX=600, TI_STEER_DELTA_UP=6, TI_STEER_DELTA_DOWN=15, TI_STEER_DELTA_UP_KNEE=600,
              TI_STEER_DELTA_UP_HIGH=6)
  assert all(ti.rate_room(limits, key) == 6 and ti.rate_room_ok(6) for key in ti.KEYS)
  kneed = NS(**{**vars(limits), 'TI_STEER_DELTA_UP_KNEE': 350, 'TI_STEER_DELTA_UP_HIGH': 2})
  assert ti.rate_room(kneed, ti.LADDER) == 2 and ti.rate_room(kneed, ti.SWEEP) == 6


# ---- steps, chunks, bands --------------------------------------------------------------------------------------

def test_chunks_follow_each_other_without_a_gap_and_steps_advance():
  prog = Progress()
  g = TiIdent()
  out = settle(g, ti.SWEEP, "31-45", n=300 + 50 + ti.SWEEP_CHUNK * 14 + 40, prog=prog)
  # after the 3 s settle and a 0.5 s lead-in, 14 chunks back to back: all 7 tones x 2 chunks, then band complete
  credits = [k for k, v in prog.have.items()]
  assert sorted(credits) == [(ti.SWEEP, "31-45", s) for s in range(7)]
  assert all(v == ti.STEP_SECONDS[ti.SWEEP] for v in prog.have.values())
  run = out[300:300 + 50 + ti.SWEEP_CHUNK * 14]
  zeros = [i for i, v in enumerate(run) if v == 0]
  gaps = max((b - a for a, b in zip(zeros, zeros[1:])), default=0)
  assert gaps < 400                         # it keeps moving (zeros only at crossings)
  assert g.reason == "band_done"
  # after the last chunk only the 0.25 s fade-out of the waveform, then nothing
  assert not any(out[-5:])


def test_a_paused_chunk_is_repeated_and_nothing_else():
  prog = Progress()
  g = TiIdent()
  settle(g, ti.SWEEP, "31-45", n=300 + 50 + ti.SWEEP_CHUNK + 500, prog=prog)    # one chunk done, half of the next
  assert prog.have == {(ti.SWEEP, "31-45", 0): 10.0}
  assert g.step == 0 and g.ph == 500
  for i in range(30):                                                      # blinker: pause
    g.update(ti.SWEEP, 5000 + i, **quiet(blinker=True), remaining=prog.remaining)
  assert not g.running
  k = 6000
  out = []
  while not prog.have.get((ti.SWEEP, "31-45", 1)) and len(out) < 5000:
    out.append(g.update(ti.SWEEP, k, **quiet(), remaining=prog.remaining))
    prog.add(g.credit)
    k += 1
  # resumed after the hold-off, at once, on the same step: the step's second chunk, then the next tone
  assert prog.have[(ti.SWEEP, "31-45", 0)] == 20.0 and g.step == 1
  first = next(i for i, v in enumerate(out) if v != 0)
  assert first <= int(td.HOLDOFF_S / ti.DT) + 20       # + the first lead-in frames that round to zero


def test_a_step_counts_in_whatever_band_the_car_is_in():
  prog = Progress()
  g = TiIdent()
  settle(g, ti.PULSES, "45-60", n=300 + ti.PULSE_PERIOD * 3 + 1, prog=prog)
  assert prog.have == {(ti.PULSES, "45-60", 0): 6.0}
  # speed up into the next band mid-pulse: that pulse is lost, the next ones count there, with the regime settled
  out = [g.update(ti.PULSES, 2000 + i, **quiet("60-80"), remaining=prog.remaining) for i in range(1)]
  for i in range(ti.PULSE_PERIOD * 2 + 5):
    g.update(ti.PULSES, 2001 + i, **quiet("60-80"), remaining=prog.remaining)
    prog.add(g.credit)
  assert prog.have.get((ti.PULSES, "60-80", 0), 0) >= 2.0
  assert prog.have[(ti.PULSES, "45-60", 0)] == 6.0


def test_a_band_change_mid_chunk_is_not_credited():
  prog = Progress()
  g = TiIdent()
  settle(g, ti.SWEEP, "45-60", n=300 + 50 + 400, prog=prog)
  for i in range(ti.SWEEP_CHUNK):
    g.update(ti.SWEEP, 1000 + i, **quiet("60-80"), remaining=prog.remaining)
    prog.add(g.credit)
  assert (ti.SWEEP, "45-60", 0) not in prog.have and (ti.SWEEP, "60-80", 0) not in prog.have


def test_the_ladder_runs_only_the_steps_under_the_bands_cap():
  for band in ("19-28", "60-80"):
    prog = Progress()
    g = TiIdent()
    peak = 0
    for k in range(300 + 100 + 30000):
      peak = max(peak, abs(g.update(ti.LADDER, k, **quiet(band), remaining=prog.remaining)))
      prog.add(g.credit)
    assert peak <= ti.LADDER_CAPS[band]
    assert {s for (_, b, s) in prog.have} == set(range(ti.steps_in_band(ti.LADDER, band)))
    assert g.reason == "band_done"


def test_pulse_signs_alternate_and_each_doublet_balances():
  g = TiIdent()
  out = settle(g, ti.PULSES, "31-45", n=300 + ti.PULSE_PERIOD * 4)
  run = out[300:]
  firsts = [next(v for v in run[i * ti.PULSE_PERIOD:(i + 1) * ti.PULSE_PERIOD] if v != 0) for i in range(4)]
  assert [math.copysign(1, f) for f in firsts] == [1, -1, 1, -1]
  for i in range(4):
    assert sum(run[i * ti.PULSE_PERIOD:(i + 1) * ti.PULSE_PERIOD]) == 0


# ---- the accountant: band bookkeeping, completion, persistence, reset -------------------------------------------

class FakeParams:
  def __init__(self):
    self.store = {}

  def get(self, key, encoding=None):
    return self.store.get(key)

  def put(self, key, value):
    self.store[key] = value

  def put_nonblocking(self, key, value):
    self.store[key] = value


def _accountant(params=None, mem=None):
  tests, errors = tt.load_definitions()
  assert errors == []
  return tt.TestAccountant(tests, params or FakeParams(), mem or FakeParams())


TOGGLES = NS(ti_dither=False, ti_sweep=False, ti_ladder=True, ti_pulses=False)


def _drive(acc, g, band, frames, frame0=0, key=ti.LADDER, toggles=TOGGLES):
  for k in range(frames):
    g.update(key, frame0 + k, **quiet(band), remaining=acc.remaining)
    acc.update(toggles, v_for(band), True, (), credit=g.credit)
    if (frame0 + k) % 500 == 0:
      acc.publish((frame0 + k) * ti.DT, {key: g.reason}, wall=1e9)


def test_progress_per_band_and_step_completion_persistence_and_reset():
  params, mem = FakeParams(), FakeParams()
  acc = _accountant(params, mem)
  g = TiIdent()
  _drive(acc, g, "31-45", 300 + 100 + 4 * 1000 + 200)
  acc.publish(1e6, {ti.LADDER: g.reason}, wall=1e9)
  doc = json.loads(mem.store[tt.PROGRESS_KEY])
  assert doc["ti_ladder"]["31-45"] == {str(s): 10.0 for s in range(4)}
  assert doc["_live"]["status"]["ti_ladder"] == tt.REASON_TEXT["band_done"]
  assert all(acc.remaining(ti.LADDER, "31-45", s) == 0 for s in range(4))
  assert acc.remaining(ti.LADDER, "19-28", 0) == 10.0
  # completion by the definition (what the UI computes): every required bin has steps x target
  t = next(t for t in acc.tests if t.id == "ti_ladder")
  def done(doc):
    e = doc.get("ti_ladder", {})
    return all(sum(min(v, t.target_seconds) for s, v in e.get(b[2], {}).items() if int(s) < n) >= n * t.target_seconds
               for b, n in zip(t.bins, t.steps))
  assert not done(doc)
  # a reboot: the tmpfs copy is gone, the flash copy carries it; a new process continues, nothing is redone
  acc.publish(1e6 + 100, {}, wall=1e9)
  mem2 = FakeParams()
  acc2 = tt.TestAccountant(acc.tests, params, mem2)
  assert acc2.remaining(ti.LADDER, "31-45", 3) == 0 and acc2.remaining(ti.LADDER, "45-60", 0) == 10.0
  g2 = TiIdent()
  _drive(acc2, g2, "31-45", 1000, frame0=10**6)
  assert g2.reason == "band_done" and not any(k[0] == "ti_ladder" for k in [])
  for band in ("19-28", "45-60", "60-80"):
    _drive(acc2, g2, band, 300 + 100 + 7 * 1000 + 200, frame0=2 * 10**6 + hash(band) % 1000 * 10**4)
  acc2.publish(2e6, {}, wall=1e9)
  final = json.loads(mem2.store[tt.PROGRESS_KEY])
  assert done(final)
  # reset (what the UI writes): the entry goes, the marker is honoured, everything is to capture again
  mem2.store[tt.PROGRESS_KEY] = json.dumps({tt.RESET_KEY: {"ti_ladder": 7}})
  acc2.publish(2e6 + 1, {}, wall=1e9)
  assert acc2.remaining(ti.LADDER, "31-45", 0) == 10.0


def test_credits_count_only_while_the_test_is_switched_on():
  acc = _accountant()
  acc.update(NS(ti_dither=False, ti_sweep=False, ti_ladder=False, ti_pulses=False), 20.0, True, (),
             credit=(ti.LADDER, "31-45", 0, 5.0))
  assert acc.pending_steps == {}
  acc.update(TOGGLES, 20.0, True, (), credit=(ti.SWEEP, "31-45", 0, 10.0))     # not the switched-on test
  assert acc.pending_steps == {}


# ---- reconstruction from the logged events ---------------------------------------------------------------------

def _random_inputs(seed, n):
  rng = random.Random(seed)
  rows, v, blocked = [], v_for("31-45"), False
  for k in range(n):
    seg = (k // 900) % 9
    v = {0: v_for("31-45"), 1: v_for("19-28"), 2: v_for("45-60"), 3: 29.5 * MPH, 4: v_for("60-80")}.get(seg % 5)
    blocked = v < 28 * MPH or (seg == 7 and k % 900 < 300)
    rows.append(dict(v_ego=v + rng.gauss(0, 0.05), steering_pressed=seg == 6 and k % 900 < 3,
                     driver_torque=rng.choice([-1.0, 0.0, 1.0, 4.0 if rng.random() < 0.02 else 2.0]),
                     blinker=seg == 8 and k % 900 < 100, base_request=int(rng.gauss(30, 60)), ti_max=TI_MAX,
                     lat_demand=rng.gauss(0, 0.3), curvature=rng.gauss(0, 0.0015), lkas_blocked=blocked,
                     room=6 if rng.random() > 0.001 else 3))
  return rows


@pytest.mark.parametrize('seed', range(3))
@pytest.mark.parametrize('key', ti.KEYS)
def test_the_logged_events_rebuild_the_signal_exactly(key, seed):
  logged = []
  g = TiIdent(log=logged.append)
  prog = Progress()
  rows = _random_inputs(seed, 40000)
  out = []
  for k, row in enumerate(rows):
    armed = key if (k // 5000) % 4 != 3 else None          # switched off and on again
    out.append(g.update(armed, k, **row, remaining=prog.remaining))
    prog.add(g.credit)
  assert sum(1 for v in out if v) > 2000
  assert ti.reconstruct(logged, len(rows)) == out
  assert all(e["test"] == key for e in logged if "test" in e)
  json.dumps(logged)                                       # the events are plain JSON (cloudlog)
  # bounded, and no held non-zero plateau even with the random pauses
  assert max(abs(v) for v in out) <= ti.peak_counts(key)
  run, longest = 1, 0
  for a, b in zip(out, out[1:]):
    run = run + 1 if a == b else 1
    if b:
      longest = max(longest, run)
  assert longest <= MAX_HELD_FRAMES


# ---- the car controller's steering block -----------------------------------------------------------------------

def _dither_tests():
  spec = importlib.util.spec_from_file_location('td_tests_for_ident', ROOT / 'selfdrive/car/tests/test_mazda_ti_dither.py')
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


M = _dither_tests()


class Acc:
  def __init__(self):
    self.prog = Progress()

  def remaining(self, key, band, step):
    return self.prog.remaining(key, band, step)


def host(on=(), dither=False, ident=True):
  h = M.host(True, dither)
  if ident:
    h.ti_ident = TiIdent()
    h.ti_ident_on = tuple(on)
    h.ti_ident_armed = None
    h.ti_ident_frames = {}
    h.ti_ident_seen = set()
    h.ti_tests = Acc()
  return h


def step(h, steer, *, lat_active=True, ti_allowed=True, v=v_for("31-45"), driver=-1.0, pressed=False, blinker=False,
         curvature=0.0003, blocked=False):
  cc = car.CarControl.new_message(latActive=lat_active, leftBlinker=blinker)
  cc.actuators.steer = steer
  cc.actuators.curvature = curvature
  cc = cc.as_reader()
  cs = NS(out=NS(vEgo=v, steeringPressed=pressed, steeringTorque=driver, leftBlinker=False, rightBlinker=False),
          ti_lkas_allowed=ti_allowed, lkas_blocked=blocked)
  env = dict(self=h, CC=cc, CS=cs, car=car, MazdaFlags=NS(TORQUE_INTERCEPTOR=1),
             apply_driver_steer_torque_limits=M.DRIVER_LIMITS, apply_ti_steer_torque_limits=M.TI_LIMITS,
             rate_room=td.rate_room, ti_ident_mod=ti)
  exec(M.BLOCK, env)
  h.frame += 1
  ident = getattr(h, "ti_ident", None)
  if ident is not None:
    h.ti_tests.prog.add(ident.credit)
  a = env['new_actuators']
  return dict(requested=env['ti_new_steer'], sent=env['ti_apply_steer'], dither=env['ti_dither'],
              ident=env.get('ti_ident', 0), steer=a.steer, can=a.steerOutputCan,
              diag_req=h.lateral_diagnostics.tiRequested, diag_lim=h.lateral_diagnostics.tiLimited,
              prev=env['ti_previous'], cc_steer=cc.actuators.steer)


def _rows(seed, n=M.frames(60)):
  rows = M.drive(seed, n)
  for r in rows:
    r['v'] = v_for("31-45") if r['v'] > 20 else v_for("19-28")
    r['blocked'] = r['v'] < 28 * MPH
  return rows


@pytest.mark.parametrize('seed', range(3))
def test_no_test_armed_is_bit_identical_to_the_undithered_path(seed):
  for on in ((), (ti.SWEEP, ti.LADDER)):                 # nothing on, or two on at once (then none runs)
    a_host, ref = host(on), M.host(False, False)
    for row in _rows(seed):
      a = step(a_host, **row)
      kw = {k: v for k, v in row.items() if k != 'blocked'}
      b = M.step(ref, **kw)
      assert a['ident'] == 0
      assert {k: a[k] for k in b} == b


def test_the_dither_and_a_test_never_run_together():
  h = host((ti.PULSES,), dither=True)
  rows = [step(h, 0.05) for _ in range(M.frames(30))]
  assert all(r['ident'] == 0 for r in rows) and any(r['dither'] for r in rows)
  assert h.ti_ident_armed is None


@pytest.mark.parametrize('key', ti.KEYS)
@pytest.mark.parametrize('seed', range(2))
def test_armed_test_stays_inside_the_limits_and_is_recoverable_from_logged_fields(key, seed):
  h = host((key,))
  saw = 0
  for row in _rows(seed, M.frames(120)):
    prev, shadow = h.ti_apply_steer_last, h.ti_reported_last
    r = step(h, **row)
    if not (row['lat_active'] and row['ti_allowed']):
      assert r['ident'] == 0 and r['sent'] == 0
      continue
    base = int(round(r['cc_steer'] * TI_MAX))
    assert r['dither'] == 0 and r['requested'] == base + r['ident']
    assert abs(r['ident']) <= ti.peak_counts(key)
    assert r['sent'] == M.TI_LIMITS(r['requested'], prev, row['driver'], h.ccp) and abs(r['sent']) <= TI_MAX
    assert r['diag_req'] - base == r['ident']           # requested added counts, exact from logged fields
    reported = int(round(r['steer'] * TI_MAX))
    if h.ti_ident.emitted:
      assert reported == M.TI_LIMITS(base, shadow, row['driver'], h.ccp)
    saw += r['ident'] != 0
  assert saw > (300 if key == ti.PULSES else 1000)


def test_steering_block_without_the_ident_attribute_still_runs():
  h = host(ident=False)
  rows = [step(h, 0.05) for _ in range(100)]
  assert all(r['ident'] == 0 for r in rows)
