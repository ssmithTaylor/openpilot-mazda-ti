"""Steering identification tests (ti_ident.py): signal tables, generator, gating, credited units, extra collection, the
run-all scheduler, band bookkeeping with the tests accountant, reconstruction from the logged events, and the car
controller's steering block with them."""
import ast
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
SETTLE = int(ti.REGIME_SETTLE_S / ti.DT)
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
  """The accountant's `remaining`, in memory (negative: extra)."""
  def __init__(self):
    self.have = {}

  def remaining(self, key, band, step):
    return ti.required_seconds(key, band, step) - self.have.get((key, band, step), 0.0)

  def add(self, credit):
    if credit is not None:
      k = credit[:3]
      self.have[k] = self.have.get(k, 0.0) + credit[3]

  def band_done(self, key, band):
    return all(self.remaining(key, band, s) <= 1e-6 for s in range(ti.steps_in_band(key, band)))


def complete(prog, key, band, steps=None):
  for s_ in (steps if steps is not None else range(ti.steps_in_band(key, band))):
    prog.have[(key, band, s_)] = ti.required_seconds(key, band, s_)


def first_required(key, band):
  return next(s_ for s_ in range(ti.steps_in_band(key, band)) if ti.required_seconds(key, band, s_) > 0)


def drive(g, key, band, n, frame0=0, prog=None, **over):
  prog = prog if prog is not None else Progress()
  out = []
  for k in range(n):
    out.append(g.update(key, frame0 + k, **quiet(band, **over), remaining=prog.remaining))
    prog.add(g.credit)
  return out, prog


def band_frames(key, band):
  """Frames to complete a band from rest: settle, every step's lead-in, units, and the fades between steps."""
  total = SETTLE + 2
  for s in range(ti.steps_in_band(key, band)):
    _, _, n, lead = ti.step_params(key, s)
    need = ti.required_seconds(key, band, s)
    if need > 0:
      total += lead + n * math.ceil(need / (n * ti.DT) - 1e-9) + ti.fade_frames(key, s) + 2
  return total


# ---- tables ----------------------------------------------------------------------------------------------------

@pytest.mark.parametrize('key', ti.KEYS)
def test_every_unit_is_exactly_zero_mean_bounded_slew_limited_and_heading_neutral(key):
  for s in range(ti.n_steps(key)):
    p, a, n, lead = ti.step_params(key, s)
    w = [ti.value(key, s, ph, 1.0) for ph in range(n)]
    assert sum(w) == 0, (key, s)                         # exact, integer counts, every unit
    assert max(abs(x) for x in w) <= a
    x = np.array([ti.unit(key, s, ph) * (ti.gain((ph + lead) / lead) if ph < 0 and lead else 1.0)
                  for ph in range(-lead, 2 * n)])
    assert np.max(np.abs(np.diff(x))) <= ti.MAX_SLEW + 1e-9, (key, s)   # lead-in, and unit into unit
    assert ti.STEP_SLEW[(key, s)] <= ti.MAX_SLEW + 1e-9
    if p is not None:
      assert p % 2 == 0 and n % p == 0
      assert n // p >= ti.UNIT_MIN_PERIODS and n * ti.DT >= ti.UNIT_MIN_S - 1e-9   # the smallest valid unit
      assert n * ti.DT <= max(ti.UNIT_MIN_S + p * ti.DT, ti.UNIT_MIN_PERIODS * p * ti.DT) + 1e-9
      heading = np.cumsum(x)
      assert abs(heading[lead:lead + n].mean()) < 1e-6 * a * n
      assert abs(heading[lead + n:lead + 2 * n].mean()) < 1e-6 * a * n    # the next unit too
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
  assert max(a for _, a in ti.SWEEP_TONES) <= td.peak_counts() == ti.SMALL_AMP
  assert ti.PULSE_AMPS[0] <= td.peak_counts()
  assert max(ti.LADDER_AMPS) == 92 and max(ti.LADDER_AMPS) + ti.MAX_BASE_FRACTION_SMALL * TI_MAX < TI_MAX
  assert ti.SMALL_AMP + ti.MAX_BASE_FRACTION_SMALL * TI_MAX < TI_MAX - 100
  freqs = [ti.freq(ti.SWEEP, s) for s in range(ti.n_steps(ti.SWEEP))]
  assert freqs[0] == pytest.approx(0.3, abs=0.005) and freqs[-1] == pytest.approx(5.0) and freqs == sorted(freqs)
  assert list(ti.LADDER_AMPS) == sorted(ti.LADDER_AMPS)
  caps = [ti.LADDER_CAPS[b[2]] for b in ti.BANDS]
  assert caps == sorted(caps, reverse=True)
  assert ti.LADDER_CAPS["19-28"] == ti.LADDER_LOW_BAND_CAP == 55.0 and ti.LADDER_LOW_BAND_FULL_CAP == 92.0
  # the strict limits are the dither's; the small ones only for steps of the dither's size
  assert (ti.MAX_LAT_DEMAND, ti.MAX_BASE_FRACTION_STRICT, ti.TOUCH_HOLDOFF_S) == (1.0, 0.5, 3.0)
  assert ti.BLINKER_HOLDOFF_S == 3.0 and ti.MAX_CURVATURE == 0.004
  assert [ti.small(ti.LADDER, s) for s in range(5)] == [True, True, True, False, False]
  assert [ti.small(ti.PULSES, s) for s in range(2)] == [True, False]


def test_capture_time_per_band_follows_what_each_step_needs():
  per_band = {k: {b[2]: round(ti.band_required(k, b[2]), 2) for b in ti.BANDS} for k in ti.KEYS}
  assert per_band[ti.SWEEP] == {"19-28": 190.5, "31-45": 67.26, "45-60": 56.68, "60-80": 40.92}
  assert per_band[ti.LADDER] == {"19-28": 60.0, "31-45": 100.0, "45-60": 85.0, "60-80": 75.0}
  assert per_band[ti.PULSES] == {"19-28": 54.0, "31-45": 80.0, "45-60": 72.0, "60-80": 74.0}
  for b in ti.BANDS:
    assert sum(ti.band_required(k, b[2]) for k in ti.KEYS) <= 6 * 60        # no band needs an inordinate time
    for k in ti.KEYS:
      assert len(ti.REQUIRED_S[b[2]][k]) == ti.steps_in_band(k, b[2])
      for s in range(ti.steps_in_band(k, b[2])):
        need = ti.required_seconds(k, b[2], s)
        n = ti.step_params(k, s)[2] * ti.DT
        assert 0 <= need <= ti.STEP_CAP_S
        assert abs(need / n - round(need / n)) < 1e-6                    # whole units
  # the optional (extra only) steps: the 0.3 Hz tone, the ladder's two smallest steps, and below 28 mph the 22- and
  # 37-count ladder steps and the small pulse
  for b in ti.BANDS:
    assert ti.required_seconds(ti.SWEEP, b[2], 0) == 0
    assert ti.required_seconds(ti.LADDER, b[2], 0) == ti.required_seconds(ti.LADDER, b[2], 1) == 0
  assert ti.required_seconds(ti.PULSES, "19-28", 0) == 0 and ti.required_seconds(ti.LADDER, "19-28", 3) == 0


def test_shipped_definitions_match_the_generator():
  tests, errors = tt.load_definitions()
  assert errors == []
  by_key = {t.settings[0][0]: t for t in tests}
  for k in ti.KEYS:
    t = by_key[k]
    assert t.metric == "step_seconds" and t.at_ignition_off and t.target_seconds == ti.STEP_CAP_S
    assert t.needs == tuple(ti.REQUIRED_S[b[2]][k] for b in ti.BANDS)
    assert [b[2] for b in t.bins] == [b[2] for b in ti.BANDS]
    assert t.steps == tuple(ti.steps_in_band(k, b[2]) for b in ti.BANDS)
  doc = json.loads(tt.DEFINITIONS_PATH.read_text(encoding='utf-8'))
  bad = doc["tests"][2]
  bad["progress"]["steps"] = [7, 7, 7, 7]
  t, why = tt.validate_test(bad)
  assert t is None and ("steps must be" in why or "needs must" in why)
  bad["progress"]["steps"] = [5, 4, 4]
  assert tt.validate_test(bad)[1] == "steps must list one count per bin"
  bad["progress"]["steps"] = [5, 4, 4, 4]
  bad["progress"]["needs"][1][2] = 30.0
  assert "needs must be" in tt.validate_test(bad)[1]                     # the needs must be the code's
  good = doc["tests"][1]
  for change, text in ((dict(target_seconds=30), "target_seconds must be"), (dict(metric="engaged_seconds"),
                       "steps go with step_seconds")):
    t = json.loads(json.dumps(good))
    t["progress"].update(change)
    assert text in tt.validate_test(t)[1]
  t = json.loads(json.dumps(good))
  t["settings"] = [{"key": "TiIdentAll", "on": True}]
  assert "not allowed" in tt.validate_test(t)[1]          # the run-all switch is not a test of its own


def _drift(key, step, v, units=2):
  """Open-loop lateral offset (m) from one lead-in plus `units` units, with the HELD upper bound of the wheel's
  response (6.4 deg per 100 counts, steering ratio 15.5, wheelbase 2.7 m, no understeer, no controller)."""
  _, _, n, lead = ti.step_params(key, step)
  tau = np.array([ti.unit(key, step, ph) * (ti.gain((ph + lead) / lead) if ph < 0 and lead else 1.0)
                  for ph in range(-lead, n * units)])
  kappa = np.radians(tau * 0.064 / 15.5) / 2.7
  heading = np.cumsum(v * kappa * ti.DT)
  return np.cumsum(v * np.sin(heading) * ti.DT), n, lead


@pytest.mark.parametrize('key,step,mph', [(ti.LADDER, 4, 28), (ti.LADDER, 3, 45), (ti.LADDER, 3, 80),
                                          (ti.SWEEP, 0, 80), (ti.SWEEP, 1, 80)])
def test_no_sideways_drift_from_a_unit(key, step, mph):
  y, n, lead = _drift(key, step, mph * MPH, units=4)
  ends = y[lead + n - 1::n]
  assert np.ptp(ends) < 0.02                              # unit ends do not walk away
  assert np.max(np.abs(y)) < 0.5                          # held upper bound, open loop


def test_the_pulse_pairs_cancel_their_heading():
  tau = np.array([ti.unit(ti.PULSES, 1, ph, sign) for sign in (1, -1) for ph in range(ti.PULSE_PERIOD)])
  assert abs(np.cumsum(tau)[-1]) < 1e-9


@pytest.mark.parametrize('key', ti.KEYS)
def test_every_fade_stays_under_the_slew_limit(key):
  for s in range(ti.n_steps(key)):
    p, a, n, lead = ti.step_params(key, s)
    for start in range(0, n, max(1, n // 37)):
      env, ph, x = 1.0, start, []
      while True:
        x.append(ti.gain(env) * ti.unit(key, s, ph % n if p is not None else ph))
        ph += 1
        if p is None:
          if ph >= len(ti.doublet(a)):
            break
        else:
          env -= 1.0 / ti.fade_frames(key, s)
          if env <= 0:
            break
      x.append(0.0)
      assert np.max(np.abs(np.diff(x))) <= ti.MAX_SLEW + 1e-9, (key, s, start)


# ---- gate --------------------------------------------------------------------------------------------------------

def test_off_or_no_band_or_unsettled_regime_is_zero():
  g = TiIdent()
  assert all(g.update(None, k, **quiet()) == 0 for k in range(500))
  g = TiIdent()
  out = [g.update(ti.SWEEP, k, **quiet()) for k in range(SETTLE)]
  assert not any(out) and g.reason == "regime"
  out = [g.update(ti.SWEEP, SETTLE + k, **quiet()) for k in range(200)]
  assert any(out) and g.reason == "ok"
  for v_mph in (10, 18.9, 85):
    g = TiIdent()
    out = [g.update(ti.SWEEP, k, **quiet(v_ego=v_mph * MPH)) for k in range(400)]
    assert not any(out) and g.reason == "band"


def test_the_28_31_mph_gap_counts_by_the_logged_regime():
  assert ti.band_for(29.5 * MPH, True) == ("19-28", True)
  assert ti.band_for(29.5 * MPH, False) == ("31-45", False)
  assert ti.band_for(29.5 * MPH, None) == (None, None)
  for blocked, band in ((True, "19-28"), (False, "31-45")):
    _, prog = drive(TiIdent(), ti.PULSES, "31-45", 2000, v_ego=29.5 * MPH, lkas_blocked=blocked)
    assert prog.have and all(b == band for (_, b, _) in prog.have)


@pytest.mark.parametrize('band', [b[2] for b in ti.BANDS])
def test_the_regime_must_be_the_bands(band):
  g = TiIdent()
  out = [g.update(ti.SWEEP, k, **quiet(band, lkas_blocked=not blocked_for(band))) for k in range(500)]
  assert not any(out) and g.reason == "regime"


def _running(key=ti.SWEEP, band="31-45", prog=None, step_to=None):
  g = TiIdent()
  prog = prog if prog is not None else Progress()
  if step_to is not None:                                  # earlier steps already complete
    complete(prog, key, band, range(step_to))
  drive(g, key, band, SETTLE + 150, prog=prog)
  assert g.running and g.env == 1.0 and g.reason == "ok"
  return g, SETTLE + 150, prog


@pytest.mark.parametrize('touch', [dict(steering_pressed=True), dict(driver_torque=6.0), dict(driver_torque=-7.0)])
@pytest.mark.parametrize('key,step,hold', [(ti.SWEEP, 0, ti.TOUCH_HOLDOFF_SMALL_S), (ti.LADDER, 3, ti.TOUCH_HOLDOFF_S)])
def test_a_touch_zeroes_at_once_and_holds_off_by_step_size(touch, key, step, hold):
  g, k, prog = _running(key, step_to=step)
  assert g.update(key, k, **quiet(**touch), remaining=prog.remaining) == 0 and not g.running
  out = [g.update(key, k + 1 + i, **quiet(), remaining=prog.remaining) for i in range(int(hold / ti.DT) - 2)]
  assert not any(out) and g.reason == "holdoff"
  out = [g.update(key, k + 400 + i, **quiet(), remaining=prog.remaining) for i in range(200)]
  assert any(out)


def test_a_blinker_holds_off_the_full_time_for_every_step():
  g, k, prog = _running()
  for i in range(20):
    g.update(ti.SWEEP, k + i, **quiet(blinker=True), remaining=prog.remaining)
  out = [g.update(ti.SWEEP, k + 20 + i, **quiet(), remaining=prog.remaining) for i in range(290)]
  assert not any(out[ti.fade_frames(ti.SWEEP, 0):]) and g.reason == "holdoff"


def test_a_small_reading_must_persist_like_the_dithers():
  g, k, prog = _running()
  for i in range(td.TOUCH_CONFIRM_FRAMES - 1):
    g.update(ti.SWEEP, k + i, **quiet(driver_torque=4.0), remaining=prog.remaining)
    assert g.running and not g.touch
  g, k, prog = _running()
  out = [g.update(ti.SWEEP, k + i, **quiet(driver_torque=-4.0), remaining=prog.remaining)
         for i in range(td.TOUCH_CONFIRM_FRAMES)]
  assert out[-1] == 0 and not g.running and g.touch


@pytest.mark.parametrize('key,step,change,reason', [
  (ti.SWEEP, 0, dict(blinker=True), "blinker"),
  (ti.SWEEP, 0, dict(curvature=0.011), "curve"), (ti.SWEEP, 0, dict(lat_demand=1.6), "lat"),
  (ti.SWEEP, 0, dict(base_request=451), "command"), (ti.SWEEP, 0, dict(v_ego=85 * MPH), "band"),
  (ti.LADDER, 3, dict(curvature=0.0045), "curve"), (ti.LADDER, 3, dict(lat_demand=1.1), "lat"),
  (ti.LADDER, 3, dict(base_request=301), "command"), (ti.PULSES, 1, dict(base_request=-301), "command"),
])
def test_other_gate_conditions_fade_out(key, step, change, reason):
  g, k, prog = _running(key, step_to=step)
  out = [g.update(key, k + i, **quiet(**change), remaining=prog.remaining) for i in range(200)]
  assert g.reason == reason and not g.running
  stop = max(ti.fade_frames(key, step), len(ti.doublet(ti.step_params(key, step)[1])) if key == ti.PULSES else 0)
  assert not any(out[stop + 1:])


@pytest.mark.parametrize('key,step,change', [
  (ti.SWEEP, 0, dict(curvature=0.0045)), (ti.SWEEP, 0, dict(lat_demand=1.2)), (ti.SWEEP, 0, dict(base_request=420)),
  (ti.LADDER, 2, dict(curvature=0.008)), (ti.PULSES, 0, dict(base_request=-400)),
])
def test_small_steps_keep_running_where_only_the_strict_limits_close(key, step, change):
  g, k, prog = _running(key, step_to=step)
  out = [g.update(key, k + i, **quiet(**change), remaining=prog.remaining) for i in range(300)]
  assert g.reason == "ok" and any(out)


def test_rate_room_is_judged_on_the_steps_own_slew():
  g, k, prog = _running()                                 # 0.6 Hz, 22 counts: slew ~1.6 counts/frame
  room = math.ceil(ti.RATE_ROOM_FACTOR * ti.STEP_SLEW[(ti.SWEEP, g.step)])
  assert room < 5
  assert g.update(ti.SWEEP, k, **quiet(room=room), remaining=prog.remaining) != 0 or g.running
  assert g.reason == "ok"
  g, k, prog = _running(step_to=6)                        # 5 Hz, 9 counts: slew ~2.9
  assert g.update(ti.SWEEP, k, **quiet(room=5), remaining=prog.remaining) == 0 and not g.running
  assert g.reason == "rate"
  limits = NS(TI_STEER_MAX=600, TI_STEER_DELTA_UP=6, TI_STEER_DELTA_DOWN=15, TI_STEER_DELTA_UP_KNEE=600,
              TI_STEER_DELTA_UP_HIGH=6)
  assert all(ti.rate_room(limits, key) == 6 for key in ti.KEYS)
  assert all(ti.rate_room_ok(6, k, s) for k in ti.KEYS for s in range(ti.n_steps(k)))


# ---- credited units, resume, bands, extra -------------------------------------------------------------------------

def test_a_band_completes_with_no_dead_time_and_keeps_collecting_extra():
  g = TiIdent()
  out, prog = drive(g, ti.SWEEP, "31-45", band_frames(ti.SWEEP, "31-45") + 2000)
  assert prog.band_done(ti.SWEEP, "31-45")
  quiet_run, longest = 0, 0
  for v in out[SETTLE + 5:]:
    quiet_run = quiet_run + 1 if v == 0 else 0
    longest = max(longest, quiet_run)
  assert longest < 40                                     # it keeps moving: no dead time beyond the fades
  assert g.reason == "extra" and g.running and any(out[-500:])
  extra = {s: prog.have.get((ti.SWEEP, "31-45", s), 0.0) - ti.required_seconds(ti.SWEEP, "31-45", s)
           for s in range(7)}
  assert sum(extra.values()) > 0                           # extra is counted


def test_extra_goes_to_the_least_collected_step_first():
  prog = Progress()
  for s in range(4):
    prog.have[(ti.LADDER, "31-45", s)] = ti.required_seconds(ti.LADDER, "31-45", s) + (5.0 if s != 2 else 0.0)
  g = TiIdent()
  drive(g, ti.LADDER, "31-45", SETTLE + 5, prog=prog)
  assert g.running and g.step == 2 and g.extra


def test_a_pause_loses_only_the_unit_in_progress_and_the_step_resumes():
  g = TiIdent()
  s0 = first_required(ti.SWEEP, "31-45")
  n = ti.step_params(ti.SWEEP, s0)[2]
  lead = ti.SWEEP_LEAD_IN
  _, prog = drive(g, ti.SWEEP, "31-45", SETTLE + lead + n + n // 2)   # one unit done, half of the next
  assert prog.have == {(ti.SWEEP, "31-45", s0): ti.unit_credit(ti.SWEEP, s0)}
  for i in range(30):
    g.update(ti.SWEEP, 5000 + i, **quiet(blinker=True), remaining=prog.remaining)
  assert not g.running
  out, _ = drive(g, ti.SWEEP, "31-45", int(ti.BLINKER_HOLDOFF_S / ti.DT) + lead + n + 10, frame0=6000, prog=prog)
  assert prog.have[(ti.SWEEP, "31-45", s0)] == pytest.approx(2 * ti.unit_credit(ti.SWEEP, s0))   # kept, one more
  first = next(i for i, v in enumerate(out) if v != 0)
  assert first <= int(ti.BLINKER_HOLDOFF_S / ti.DT) + 20
  assert g.step == s0                                      # the same step resumed


def test_a_step_counts_in_whatever_band_the_car_is_in():
  g = TiIdent()
  _, prog = drive(g, ti.PULSES, "45-60", SETTLE + ti.PULSE_PERIOD * 3 + 2)
  assert prog.have == {(ti.PULSES, "45-60", 0): 6.0}
  drive(g, ti.PULSES, "60-80", SETTLE + ti.PULSE_PERIOD * 2 + 5, frame0=2000, prog=prog)
  assert prog.have.get((ti.PULSES, "60-80", 0), 0) >= 2.0 and prog.have[(ti.PULSES, "45-60", 0)] == 6.0


def test_a_band_change_mid_unit_is_not_credited():
  g = TiIdent()
  _, prog = drive(g, ti.SWEEP, "45-60", SETTLE + 50 + 300)
  s0 = first_required(ti.SWEEP, "45-60")
  drive(g, ti.SWEEP, "60-80", ti.step_params(ti.SWEEP, s0)[2] - 300, frame0=1000, prog=prog)
  assert (ti.SWEEP, "45-60", s0) not in prog.have and (ti.SWEEP, "60-80", s0) not in prog.have


def test_the_ladder_runs_only_the_steps_under_the_bands_cap():
  for band in ("19-28", "60-80"):
    g = TiIdent()
    out, prog = drive(g, ti.LADDER, band, band_frames(ti.LADDER, band) + 5000)
    assert max(abs(v) for v in out) <= ti.LADDER_CAPS[band]
    assert {s for (_, _, s) in prog.have} == set(range(ti.steps_in_band(ti.LADDER, band)))
    assert prog.band_done(ti.LADDER, band) and g.reason == "extra"


def test_pulse_signs_alternate_and_each_doublet_balances():
  out, _ = drive(TiIdent(), ti.PULSES, "31-45", SETTLE + ti.PULSE_PERIOD * 4 + 1)
  run = out[SETTLE:]
  firsts = [next(v for v in run[i * ti.PULSE_PERIOD:(i + 1) * ti.PULSE_PERIOD] if v != 0) for i in range(4)]
  assert [math.copysign(1, f) for f in firsts] == [1, -1, 1, -1]
  for i in range(4):
    assert sum(run[i * ti.PULSE_PERIOD:(i + 1) * ti.PULSE_PERIOD]) == 0


def test_unknown_progress_runs_nothing():
  g = TiIdent()
  out = [g.update(ti.SWEEP, k, **quiet(), remaining=lambda k_, b, s: None) for k in range(600)]
  assert not any(out) and g.reason == "off"


# ---- run all: the scheduler -----------------------------------------------------------------------------------------

def test_schedule_order_stickiness_band_change_and_extra():
  prog = Progress()
  r = prog.remaining
  assert ti.RUN_ALL_ORDER == (ti.SWEEP, ti.PULSES, ti.LADDER)
  assert ti.schedule(None, None, "31-45", r) == ti.SWEEP
  prog.have[(ti.PULSES, "31-45", 0)] = 5.0
  assert ti.schedule(ti.PULSES, "31-45", "31-45", r) == ti.PULSES      # stays on an incomplete test
  assert ti.schedule(ti.PULSES, "19-28", "31-45", r) == ti.SWEEP       # band change: the first incomplete one
  assert ti.schedule(ti.SWEEP, "31-45", None, r) == ti.SWEEP           # no band: keep
  complete(prog, ti.SWEEP, "31-45")
  assert ti.schedule(ti.SWEEP, "31-45", "31-45", r) == ti.PULSES       # complete: the next in order
  complete(prog, ti.PULSES, "31-45")
  complete(prog, ti.LADDER, "31-45")
  # all complete: extra, least collected first, with hysteresis
  assert ti.schedule(ti.LADDER, "31-45", "31-45", r) == ti.LADDER
  prog.have[(ti.LADDER, "31-45", 2)] += 0.3 * ti.band_required(ti.LADDER, "31-45")
  assert ti.schedule(ti.LADDER, "31-45", "31-45", r) == ti.SWEEP
  assert ti.schedule(None, None, "31-45", lambda k, b, s: None) is None


def _run_all(n, seed=None, frame0=0, prog=None, rows=None):
  """The controller's run-all loop around one generator: schedule, then update."""
  prog = prog if prog is not None else Progress()
  g = TiIdent(log=None)
  logged = []
  g.log = logged.append
  armed, band_armed, out, keys = None, None, [], []
  rng = random.Random(seed)
  for k in range(n):
    row = rows[k] if rows is not None else quiet("31-45")
    band, _ = ti.band_for(row["v_ego"], row["lkas_blocked"])
    armed = ti.schedule(armed, band_armed, band, prog.remaining)
    if band is not None:
      band_armed = band
    out.append(g.update(armed, frame0 + k, **row, remaining=prog.remaining))
    keys.append(g.emitted)
    prog.add(g.credit)
  return g, out, keys, logged, prog


def test_run_all_runs_the_tests_in_order_and_only_after_a_full_fade_and_quiet():
  total = sum(band_frames(k, "31-45") for k in ti.KEYS) + 5000
  g, out, keys, logged, prog = _run_all(total)
  assert all(prog.band_done(k, "31-45") for k in ti.KEYS)
  starts = [e for e in logged if e["kind"] == "run"]
  order = []
  for e in starts:
    if not order or order[-1] != e["test"]:
      order.append(e["test"])
  assert order[:3] == [ti.SWEEP, ti.PULSES, ti.LADDER]
  # between two tests: the old one fully stopped, then SWITCH_QUIET_S of zero output
  quiet_frames = int(ti.SWITCH_QUIET_S / ti.DT)
  for i in range(1, len(keys)):
    if keys[i] is not None and keys[i - 1] is not None:
      assert keys[i] == keys[i - 1]                         # never two tests in consecutive frames
  last = None
  for i, k_ in enumerate(keys):
    if k_ is not None:
      if last is not None and k_ != last[0]:
        assert i - last[1] > quiet_frames                  # at least SWITCH_QUIET_S of zero in between
      last = (k_, i)
  assert ti.reconstruct(logged, len(out)) == out
  assert g.reason == "extra"


def _random_rows(seed, n):
  rng = random.Random(seed)
  rows = []
  for k in range(n):
    seg = (k // 900) % 9
    v = {0: v_for("31-45"), 1: v_for("19-28"), 2: v_for("45-60"), 3: 29.5 * MPH, 4: v_for("60-80")}.get(seg % 5)
    blocked = v < 28 * MPH or (seg == 3 and k % 900 < 450) or (seg == 7 and k % 900 < 300)
    rows.append(dict(v_ego=v + rng.gauss(0, 0.05), steering_pressed=seg == 6 and k % 900 < 3,
                     driver_torque=rng.choice([-1.0, 0.0, 1.0, 4.0 if rng.random() < 0.02 else 2.0]),
                     blinker=seg == 8 and k % 900 < 100, base_request=int(rng.gauss(30, 120)), ti_max=TI_MAX,
                     lat_demand=rng.gauss(0, 0.6), curvature=rng.gauss(0, 0.003), lkas_blocked=blocked,
                     room=6 if rng.random() > 0.001 else 3))
  return rows


@pytest.mark.parametrize('seed', range(3))
def test_run_all_over_a_random_drive_is_one_signal_at_a_time_and_exactly_reconstructed(seed):
  rows = _random_rows(seed, 60000)
  prog = Progress()                                        # some tests already complete in some bands
  complete(prog, ti.SWEEP, "31-45")
  complete(prog, ti.SWEEP, "45-60")
  complete(prog, ti.PULSES, "45-60")
  g, out, keys, logged, prog = _run_all(len(rows), rows=rows, prog=prog)
  assert len({k_ for k_ in keys if k_}) >= 2 and sum(1 for v in out if v) > 5000
  assert ti.reconstruct(logged, len(out)) == out
  for i in range(1, len(keys)):
    if keys[i] and keys[i - 1]:
      assert keys[i] == keys[i - 1]
  cuts = {e["frame"] for e in logged if e["kind"] == "stop" and e["mode"] == "zero"}
  assert all(abs(b - a) <= ti.MAX_SLEW + 1 for k, (a, b) in enumerate(zip(out, out[1:])) if k + 1 not in cuts)


@pytest.mark.parametrize('seed', range(2))
@pytest.mark.parametrize('key', ti.KEYS)
def test_the_logged_events_rebuild_a_single_test_exactly(key, seed):
  logged = []
  g = TiIdent(log=logged.append)
  prog = Progress()
  rows = _random_rows(seed, 40000)
  out = []
  for k, row in enumerate(rows):
    armed = key if (k // 5000) % 4 != 3 else None
    out.append(g.update(armed, k, **row, remaining=prog.remaining))
    prog.add(g.credit)
  assert sum(1 for v in out if v) > 2000
  assert ti.reconstruct(logged, len(rows)) == out
  json.dumps(logged)
  assert max(abs(v) for v in out) <= ti.peak_counts(key)
  run, longest = 1, 0
  for a, b in zip(out, out[1:]):
    run = run + 1 if a == b else 1
    if b:
      longest = max(longest, run)
  assert longest <= MAX_HELD_FRAMES


def test_pause_counters_per_test_band_and_reason():
  g = TiIdent()
  drive(g, ti.SWEEP, "31-45", 300)
  drive(g, ti.SWEEP, "31-45", 100, frame0=300, blinker=True)
  drive(g, ti.SWEEP, "31-45", 100, frame0=400, v_ego=85 * MPH)
  fb = g.frames_by(ti.SWEEP)
  assert fb["31-45"]["regime"] == SETTLE and fb["31-45"]["blinker"] == 100 and sum(fb["none"].values()) == 100
  assert 0 < g.share(ti.SWEEP) < 1 and g.share(ti.LADDER) is None


# ---- the accountant: band bookkeeping, completion, persistence, reset, run all, live block -----------------------

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


TOGGLES = NS(ti_dither=False, ti_sweep=False, ti_ladder=True, ti_pulses=False, ti_ident_all=False)
ALL = NS(ti_dither=False, ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=True)


def _drive_acc(acc, g, band, frames, frame0=0, key=ti.LADDER, toggles=TOGGLES):
  for k in range(frames):
    g.update(key, frame0 + k, **quiet(band), remaining=acc.remaining)
    active = (g.emitted,) if g.emitted and g.reason in ("ok", "extra") else ()
    acc.update(toggles, v_for(band), True, active, credit=g.credit)
    if (frame0 + k) % 500 == 0:
      acc.publish((frame0 + k) * ti.DT, {key: g.reason}, wall=1e9)


def test_progress_per_band_and_step_completion_extra_persistence_and_reset():
  params, mem = FakeParams(), FakeParams()
  acc = _accountant(params, mem)
  g = TiIdent()
  _drive_acc(acc, g, "31-45", band_frames(ti.LADDER, "31-45") + 1200)
  acc.publish(1e6, {ti.LADDER: g.reason}, wall=1e9)
  doc = json.loads(mem.store[tt.PROGRESS_KEY])
  entry = doc["ti_ladder"]["31-45"]
  assert entry["2"] >= 60.0 and entry["3"] >= 40.0 and sum(entry.values()) > 100.0      # extra counted
  assert doc["_live"]["status"]["ti_ladder"] == tt.REASON_TEXT["extra"]
  assert all(acc.remaining(ti.LADDER, "31-45", s) <= 0 for s in range(4))
  assert acc.remaining(ti.LADDER, "19-28", 4) == 60.0 and acc.remaining(ti.LADDER, "19-28", 0) == 0.0
  # a reboot: the flash copy carries it
  acc.publish(1e6 + 100, {}, wall=1e9)
  acc2 = tt.TestAccountant(acc.tests, params, FakeParams())
  assert acc2.remaining(ti.LADDER, "31-45", 3) <= 0 and acc2.remaining(ti.LADDER, "45-60", 2) == 60.0
  # reset (what the UI writes)
  acc2.params_memory.store[tt.PROGRESS_KEY] = json.dumps({tt.RESET_KEY: {"ti_ladder": 7}})
  acc2.update(TOGGLES, v_for("31-45"), True, ())          # the test is on, so the next publish runs
  acc2.publish(2e6, {}, wall=1e9)
  assert acc2.remaining(ti.LADDER, "31-45", 2) == 60.0
  # a broken accountant: unknown progress, so nothing runs
  acc2.failed = True
  assert acc2.remaining(ti.LADDER, "31-45", 0) is None


def test_a_running_test_reads_active_between_credits():
  # The field report: "keeps switching to ON · WAIT". Units are credited every 2-7 s, so the 5 s live block must
  # say "active" while the test runs clean, not only in the intervals in which a unit was credited.
  acc, mem = _accountant(), None
  g = TiIdent()
  seen = []
  for k in range(SETTLE + 3000):
    g.update(ti.SWEEP, k, **quiet(), remaining=acc.remaining)
    active = (g.emitted,) if g.emitted and g.reason in ("ok", "extra") else ()
    acc.update(NS(**{**vars(TOGGLES), 'ti_ladder': False, 'ti_sweep': True}), v_for("31-45"), True, active,
               credit=g.credit)
    if k % 200 == 199 and k > SETTLE + 200:
      acc.publish(k * ti.DT, {ti.SWEEP: g.reason}, wall=1e9)
      seen.append("ti_sweep" in json.loads(acc.params_memory.store[tt.PROGRESS_KEY])["_live"]["active"])
  assert seen and all(seen)


def test_run_all_counts_every_step_test_and_publishes_the_all_block():
  acc = _accountant()
  g = TiIdent()
  for k in range(SETTLE + 1500):
    g.update(ti.SWEEP, k, **quiet(), remaining=acc.remaining)
    active = (g.emitted,) if g.emitted and g.reason in ("ok", "extra") else ()
    acc.update(ALL, v_for("31-45"), True, active, credit=g.credit)
  acc.publish(100.0, {ti.SWEEP: g.reason, ti.PULSES: "other", ti.LADDER: "other"},
              notes={ti.SWEEP: " · 80% captured"}, run_all=(ti.SWEEP, g.reason), wall=1e9)
  live = json.loads(acc.params_memory.store[tt.PROGRESS_KEY])["_live"]
  assert set(live["on"]) == {"ti_sweep", "ti_ladder", "ti_pulses"} and "ti_sweep" in live["active"]
  assert live["all"] == {"on": True, "test": "ti_sweep",
                         "status": "Steering sweep 0.3-5 Hz: Running · 80% captured"}
  assert all(len(v) <= 80 for v in live["status"].values())


# ---- the car controller's steering block -------------------------------------------------------------------------

def _dither_tests():
  spec = importlib.util.spec_from_file_location('td_tests_for_ident', ROOT / 'selfdrive/car/tests/test_mazda_ti_dither.py')
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


M = _dither_tests()


def _controller_method(name):
  tree = ast.parse((ROOT / 'selfdrive/car/mazda/carcontroller.py').read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CarController')
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
  env = {}
  exec(compile(ast.Module(body=[fn], type_ignores=[]), name, 'exec'), env)
  return env[name]


TI_IDENT_FAILED = _controller_method('ti_ident_failed')


class Acc:
  def __init__(self):
    self.prog = Progress()

  def remaining(self, key, band, step):
    return self.prog.remaining(key, band, step)


def host(on=(), dither=False, ident=True, run_all=False):
  h = M.host(True, dither)
  h.ti_ident_failed = lambda: TI_IDENT_FAILED(h)
  if ident:
    h.ti_ident = TiIdent()
    h.ti_ident_on = tuple(on)
    h.ti_ident_all = run_all
    h.ti_ident_all_band = None
    h.ti_ident_all_seen = False
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
              prev=env['ti_previous'], cc_steer=cc.actuators.steer, emitted=getattr(ident, "emitted", None))


def _rows(seed, n=M.frames(60)):
  rows = M.drive(seed, n)
  for r in rows:
    r['v'] = v_for("31-45") if r['v'] > 20 else v_for("19-28")
    r['blocked'] = r['v'] < 28 * MPH
  return rows


@pytest.mark.parametrize('seed', range(3))
def test_nothing_armed_is_bit_identical_to_the_undithered_path(seed):
  # nothing on; two tests on at once (then none runs); run all with a single test also on (then none runs)
  for kw in (dict(on=()), dict(on=(ti.SWEEP, ti.LADDER)), dict(on=(ti.SWEEP,), run_all=True)):
    a_host, ref = host(**kw), M.host(False, False)
    for row in _rows(seed):
      a = step(a_host, **row)
      kw_ref = {k: v for k, v in row.items() if k != 'blocked'}
      b = M.step(ref, **kw_ref)
      assert a['ident'] == 0
      assert {k: a[k] for k in b} == b


def test_the_dither_and_a_test_never_run_together():
  for kw in (dict(on=(ti.PULSES,)), dict(run_all=True)):
    h = host(dither=True, **kw)
    rows = [step(h, 0.05) for _ in range(M.frames(30))]
    assert all(r['ident'] == 0 for r in rows) and any(r['dither'] for r in rows)
    assert h.ti_ident_armed is None


@pytest.mark.parametrize('kw', [dict(on=(ti.SWEEP,)), dict(on=(ti.LADDER,)), dict(on=(ti.PULSES,)),
                                dict(run_all=True)])
@pytest.mark.parametrize('seed', range(2))
def test_armed_tests_stay_inside_the_limits_and_are_recoverable_from_logged_fields(kw, seed):
  h = host(**kw)
  if kw.get('run_all'):                                    # sweep already complete at 31-45 mph: the scheduler moves on
    complete(h.ti_tests.prog, ti.SWEEP, "31-45")
  saw, keys = 0, set()
  for row in _rows(seed, M.frames(240)):
    prev, shadow = h.ti_apply_steer_last, h.ti_reported_last
    r = step(h, **row)
    if not (row['lat_active'] and row['ti_allowed']):
      assert r['ident'] == 0 and r['sent'] == 0
      continue
    base = int(round(r['cc_steer'] * TI_MAX))
    assert r['dither'] == 0 and r['requested'] == base + r['ident']
    assert r['sent'] == M.TI_LIMITS(r['requested'], prev, row['driver'], h.ccp) and abs(r['sent']) <= TI_MAX
    assert r['diag_req'] - base == r['ident']
    if r['emitted']:
      assert abs(r['ident']) <= ti.peak_counts(r['emitted'])
      assert int(round(r['steer'] * TI_MAX)) == M.TI_LIMITS(base, shadow, row['driver'], h.ccp)
      keys.add(r['emitted'])
    saw += r['ident'] != 0
  assert saw > 300
  if kw.get('run_all'):
    assert len(keys) >= 2                                    # the scheduler moved on by itself


def test_steering_block_without_the_ident_attribute_still_runs():
  h = host(ident=False)
  assert all(step(h, 0.05)['ident'] == 0 for _ in range(100))


class Boom(TiIdent):
  def __init__(self, at):
    super().__init__()
    self.at, self.calls = at, 0

  def update(self, *a, **kw):
    self.calls += 1
    if self.calls >= self.at:
      raise RuntimeError("forced")
    return super().update(*a, **kw)


@pytest.mark.parametrize('at', [1, 50, M.frames(8)])
def test_a_raising_generator_is_switched_off_and_the_controller_runs_on(at, monkeypatch):
  logged = []
  import openpilot.common.swaglog as swaglog
  monkeypatch.setattr(swaglog.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
  h, ref = host((ti.SWEEP,)), M.host(True, False)
  h.ti_ident = Boom(at)
  rows = [dict(steer=0.05)] * M.frames(20)
  before = [step(h, **r) for r in rows[:at - 1]]
  after = [step(h, **r) for r in rows[at - 1:]]
  assert h.ti_ident is None and h.ti_ident_armed is None and logged == ["ti_ident disabled"]
  assert all(r['ident'] == 0 for r in after)
  refs = [M.step(ref, **r) for r in rows]
  if not any(r['ident'] for r in before):
    assert [{k: r[k] for k in refs[0]} for r in before + after] == refs
  else:
    for r, q in zip(after[5:], refs[at - 1 + 5:]):
      assert r['requested'] == q['requested']


def test_a_raising_reset_is_switched_off_too(monkeypatch):
  import openpilot.common.swaglog as swaglog
  monkeypatch.setattr(swaglog.cloudlog, "exception", lambda *a, **k: None)
  h = host((ti.SWEEP,))

  def bad(*a, **k):
    raise RuntimeError("forced")
  h.ti_ident.reset = bad
  r = step(h, 0.05, lat_active=False)
  assert h.ti_ident is None and r['sent'] == 0
  assert step(h, 0.05)['ident'] == 0


def test_every_pause_reason_has_its_own_screen_text_and_stop_events_name_it():
  codes = {"off", "touch", "rate", "blinker", "holdoff", "band", "regime", "command", "curve", "lat", "extra", "switch",
           "ok"}
  assert codes - {"off"} <= set(tt.REASON_TEXT) and len({tt.REASON_TEXT[c] for c in codes - {"off"}}) == len(codes) - 1
  logged = []
  g = TiIdent(log=logged.append)
  prog = Progress()
  for change in (dict(blinker=True), dict(curvature=0.02), dict(lat_demand=2.0), dict(base_request=500),
                 dict(v_ego=85 * MPH), dict(driver_torque=7.0)):
    drive(g, ti.SWEEP, "31-45", 600, frame0=len(logged) * 1000 + 10 ** 5, prog=prog)
    drive(g, ti.SWEEP, "31-45", 5, frame0=len(logged) * 1000 + 10 ** 5 + 600, prog=prog, **change)
  whys = [e.get("why") for e in logged if e["kind"] == "stop"]
  assert {"blinker", "curve", "lat", "command", "band", "touch"} <= set(whys)
  assert all("step" in e for e in logged if e["kind"] == "stop")
