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


def quiet(band="35-41", **over):
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
  assert ti.LADDER_CAPS["21-27"] == ti.LADDER_LOW_BAND_CAP == 55.0 and ti.LADDER_LOW_BAND_FULL_CAP == 92.0
  # the strict limits are the dither's; the small ones only for steps of the dither's size
  assert (ti.MAX_LAT_DEMAND, ti.MAX_BASE_FRACTION_STRICT, ti.TOUCH_HOLDOFF_S) == (1.0, 0.5, 3.0)
  assert ti.BLINKER_HOLDOFF_S == 3.0 and ti.MAX_CURVATURE == 0.004
  assert [ti.small(ti.LADDER, s) for s in range(5)] == [True, True, True, False, False]
  assert [ti.small(ti.PULSES, s) for s in range(2)] == [True, False]


def test_capture_time_per_band_follows_what_each_step_needs():
  per_band = {k: {b[2]: round(ti.band_required(k, b[2]), 2) for b in ti.BANDS} for k in ti.KEYS}
  assert per_band[ti.SWEEP] == {"21-27": 80.04, "35-41": 67.26, "49-56": 56.68, "64-80": 40.92}   # 21-27: cut to about 80 s by the plan (ti_drive_plan.json)
  assert per_band[ti.LADDER] == {"21-27": 60.0, "35-41": 100.0, "49-56": 85.0, "64-80": 75.0}
  assert per_band[ti.PULSES] == {"21-27": 54.0, "35-41": 80.0, "49-56": 72.0, "64-80": 74.0}
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
  assert ti.required_seconds(ti.PULSES, "21-27", 0) == 0 and ti.required_seconds(ti.LADDER, "21-27", 3) == 0


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


def test_the_windows_have_gaps_and_none_straddles_the_eps_region():
  assert [(w[0], w[1], w[2]) for w in ti.WINDOWS] == [(21, 27, "21-27"), (35, 41, "35-41"), (49, 56, "49-56"),
                                                      (64, 80, "64-80")]
  for lo, hi, _, alone in ti.WINDOWS:
    assert hi <= ti.EPS_LOW_MPH or lo >= ti.EPS_HIGH_MPH             # nothing straddles 28-31 mph
    assert alone == (hi <= ti.EPS_LOW_MPH)
  assert all(a[1] < b[0] for a, b in zip(ti.WINDOWS, ti.WINDOWS[1:]))      # gaps between every two
  assert ti.MIN_TEST_MPH == 19.0 and ti.EPS_LOW_MPH == 28.0 and ti.EPS_HIGH_MPH == 31.0


@pytest.mark.parametrize('mph,window,profile,state', [
  (18.9, None, None, "below"), (19.0, None, "21-27", "between"), (20.9, None, "21-27", "between"),
  (21.0, "21-27", "21-27", "inside"), (26.99, "21-27", "21-27", "inside"), (27.0, None, "21-27", "between"),
  (27.99, None, "21-27", "between"), (28.0, None, None, "transition"), (30.99, None, None, "transition"),
  (31.0, None, "35-41", "between"), (34.9, None, "35-41", "between"), (35.0, "35-41", "35-41", "inside"),
  (41.0, None, "35-41", "between"), (44.9, None, "35-41", "between"), (45.1, None, "49-56", "between"),
  (49.0, "49-56", "49-56", "inside"), (56.0, None, "49-56", "between"), (59.9, None, "49-56", "between"),
  (60.1, None, "64-80", "between"), (64.0, "64-80", "64-80", "inside"), (79.9, "64-80", "64-80", "inside"),
  (80.0, None, None, "above")])
def test_zone_for_windows_gaps_and_the_no_test_regions(mph, window, profile, state):
  z = ti.zone_for(mph * MPH)
  assert (z.window, z.profile, z.state) == (window, profile, state)


@pytest.mark.parametrize('mph,reason', [(18.0, "band"), (29.5, "transition"), (85.0, "band")])
def test_no_test_runs_below_19_or_in_28_31_or_above_80(mph, reason):
  for blocked in (True, False):
    g = TiIdent()
    out = [g.update(ti.SWEEP, k, **quiet(v_ego=mph * MPH, lkas_blocked=blocked)) for k in range(600)]
    assert not any(out) and g.reason == reason


def test_a_running_test_fades_out_on_entering_the_28_31_region_not_a_cut():
  g, k, prog = _running(ti.SWEEP, "21-27", step_to=1)
  out = [g.update(ti.SWEEP, k + i, **quiet("21-27", v_ego=29.5 * MPH), remaining=prog.remaining) for i in range(200)]
  assert g.reason == "transition" and out[-1] == 0
  assert max(abs(b - a) for a, b in zip(out, out[1:])) <= ti.MAX_SLEW + 1


@pytest.mark.parametrize('mph,blocked,profile', [(20.0, True, "21-27"), (33.0, False, "35-41"),
                                                 (43.0, False, "35-41"), (47.0, False, "49-56"),
                                                 (58.0, False, "49-56"), (62.0, False, "64-80")])
def test_between_windows_the_test_runs_logged_and_tagged_but_ticks_no_box(mph, blocked, profile):
  logged = []
  g = TiIdent(log=logged.append)
  prog = Progress()
  out = []
  for k in range(6000):
    out.append(g.update(ti.SWEEP, k, **quiet(v_ego=mph * MPH, lkas_blocked=blocked), remaining=prog.remaining))
    prog.add(g.credit)
    assert g.credit is None
  assert any(out) and g.reason == "gap" and not prog.have
  runs = [e for e in logged if e["kind"] in ("run", "chunk")]
  assert runs and all(e["band"] == "gap" and e["profile"] == profile and abs(e["mph"] - mph) < 0.1 for e in runs)
  assert ti.reconstruct(logged, len(out)) == out


def test_the_regime_must_match_the_speed_between_windows_too():
  for mph, wrong in ((20.0, False), (33.0, True)):
    g = TiIdent()
    out = [g.update(ti.SWEEP, k, **quiet(v_ego=mph * MPH, lkas_blocked=wrong)) for k in range(500)]
    assert not any(out) and g.reason == "regime"


def test_a_unit_that_wanders_out_of_its_window_is_not_credited_and_one_inside_is():
  prog = Progress()
  g = TiIdent()
  k = 0
  while not g.running or g.env < 1.0:                       # a unit under way inside the window
    g.update(ti.SWEEP, k, **quiet("35-41"), remaining=prog.remaining)
    k += 1
  drive_out = 0
  while g.step == 1 and drive_out < 3000 and prog.have == {}:
    g.update(ti.SWEEP, k, **quiet("35-41", v_ego=43.0 * MPH), remaining=prog.remaining)   # the gap above 41 mph
    prog.add(g.credit)
    k += 1
    drive_out += 1
  assert prog.have == {}                                    # the unit that crossed the edge counted nothing
  for _ in range(3000):
    g.update(ti.SWEEP, k, **quiet("35-41"), remaining=prog.remaining)
    prog.add(g.credit)
    k += 1
  assert prog.have and all(b == "35-41" for (_, b, _) in prog.have)


def test_progress_ticks_per_window_only_inside_it():
  # one drive crossing every window and gap: credit lands only under the four window labels
  prog = Progress()
  g = TiIdent()
  k = 0
  for mph, blocked in ((20, True), (24, True), (33, False), (38, False), (45, False), (52, False), (62, False),
                       (70, False)):
    for _ in range(4000):
      g.update(ti.SWEEP, k, **quiet(v_ego=mph * MPH, lkas_blocked=blocked), remaining=prog.remaining)
      prog.add(g.credit)
      k += 1
  assert {b for (_, b, _) in prog.have} <= {"21-27", "35-41", "49-56", "64-80"} and len({b for (_, b, _) in prog.have}) == 4
  assert {"21-27", "35-41", "49-56", "64-80"} == {b for (_, b, _) in prog.have}


@pytest.mark.parametrize('band', [b[2] for b in ti.BANDS])
def test_the_regime_must_be_the_bands(band):
  g = TiIdent()
  out = [g.update(ti.SWEEP, k, **quiet(band, lkas_blocked=not blocked_for(band))) for k in range(500)]
  assert not any(out) and g.reason == "regime"


def _running(key=ti.SWEEP, band="35-41", prog=None, step_to=None):
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
  stop = max(ti.fade_frames(key, step), len(ti.doublet(ti.step_params(key, step)[1])) if key == ti.PULSES else 0)
  if ti.small(key, step) or reason in ("blinker", "band"):
    assert g.reason == reason and not g.running
    assert not any(out[stop + 1:])
  else:
    # revision 3: a strict step closed only by a limit fades out as before; the same test then goes on with a SMALL
    # step (which that limit does not close) instead of idling. Nothing above the small amplitude is ever sent again.
    assert max(abs(v) for v in out[stop + 1:]) <= ti.SMALL_AMP
    assert g.gate is None or g.gate["reason"] == reason


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
  out, prog = drive(g, ti.SWEEP, "35-41", band_frames(ti.SWEEP, "35-41") + 2000)
  assert prog.band_done(ti.SWEEP, "35-41")
  quiet_run, longest = 0, 0
  for v in out[SETTLE + 5:]:
    quiet_run = quiet_run + 1 if v == 0 else 0
    longest = max(longest, quiet_run)
  assert longest < 40                                     # it keeps moving: no dead time beyond the fades
  assert g.reason == "extra" and g.running and any(out[-500:])
  extra = {s: prog.have.get((ti.SWEEP, "35-41", s), 0.0) - ti.required_seconds(ti.SWEEP, "35-41", s)
           for s in range(7)}
  assert sum(extra.values()) > 0                           # extra is counted


def test_extra_goes_to_the_least_collected_step_first():
  prog = Progress()
  for s in range(4):
    prog.have[(ti.LADDER, "35-41", s)] = ti.required_seconds(ti.LADDER, "35-41", s) + (5.0 if s != 2 else 0.0)
  g = TiIdent()
  drive(g, ti.LADDER, "35-41", SETTLE + 5, prog=prog)
  assert g.running and g.step == 2 and g.extra


def test_a_pause_loses_only_the_unit_in_progress_and_the_step_resumes():
  g = TiIdent()
  s0 = first_required(ti.SWEEP, "35-41")
  n = ti.step_params(ti.SWEEP, s0)[2]
  lead = ti.SWEEP_LEAD_IN
  _, prog = drive(g, ti.SWEEP, "35-41", SETTLE + lead + n + n // 2)   # one unit done, half of the next
  assert prog.have == {(ti.SWEEP, "35-41", s0): ti.unit_credit(ti.SWEEP, s0)}
  for i in range(30):
    g.update(ti.SWEEP, 5000 + i, **quiet(blinker=True), remaining=prog.remaining)
  assert not g.running
  out, _ = drive(g, ti.SWEEP, "35-41", int(ti.BLINKER_HOLDOFF_S / ti.DT) + lead + n + 10, frame0=6000, prog=prog)
  assert prog.have[(ti.SWEEP, "35-41", s0)] == pytest.approx(2 * ti.unit_credit(ti.SWEEP, s0))   # kept, one more
  first = next(i for i, v in enumerate(out) if v != 0)
  assert first <= int(ti.BLINKER_HOLDOFF_S / ti.DT) + 20
  assert g.step == s0                                      # the same step resumed


def test_a_step_counts_in_whatever_band_the_car_is_in():
  g = TiIdent()
  _, prog = drive(g, ti.PULSES, "49-56", SETTLE + ti.PULSE_PERIOD * 3 + 2)
  assert prog.have == {(ti.PULSES, "49-56", 0): 6.0}
  drive(g, ti.PULSES, "64-80", SETTLE + ti.PULSE_PERIOD * 2 + 5, frame0=2000, prog=prog)
  assert prog.have.get((ti.PULSES, "64-80", 0), 0) >= 2.0 and prog.have[(ti.PULSES, "49-56", 0)] == 6.0


def test_a_band_change_mid_unit_is_not_credited():
  g = TiIdent()
  _, prog = drive(g, ti.SWEEP, "49-56", SETTLE + 50 + 300)
  s0 = first_required(ti.SWEEP, "49-56")
  drive(g, ti.SWEEP, "64-80", ti.step_params(ti.SWEEP, s0)[2] - 300, frame0=1000, prog=prog)
  assert (ti.SWEEP, "49-56", s0) not in prog.have and (ti.SWEEP, "64-80", s0) not in prog.have


def test_the_ladder_runs_only_the_steps_under_the_bands_cap():
  for band in ("21-27", "64-80"):
    g = TiIdent()
    out, prog = drive(g, ti.LADDER, band, band_frames(ti.LADDER, band) + 20000)
    assert max(abs(v) for v in out) <= ti.LADDER_CAPS[band]
    assert {s for (_, _, s) in prog.have} == set(range(ti.steps_in_band(ti.LADDER, band)))
    assert prog.band_done(ti.LADDER, band) and g.reason == "extra"


def test_pulse_signs_alternate_and_each_doublet_balances():
  out, _ = drive(TiIdent(), ti.PULSES, "35-41", SETTLE + ti.PULSE_PERIOD * 4 + 1)
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

def _pick(r, prog, window="35-41", frame=0, can_run=None):
  return r.pick(frame, window, prog.remaining, can_run)


def test_run_all_order_stickiness_window_change_and_unknown_progress():
  prog = Progress()
  assert ti.RUN_ALL_ORDER == (ti.SWEEP, ti.PULSES, ti.LADDER)
  r = ti.RunAll(seed=1)
  assert _pick(r, prog) == ti.SWEEP
  prog.have[(ti.PULSES, "35-41", 0)] = 5.0
  r.current = ti.PULSES
  assert _pick(r, prog) == ti.PULSES                                    # stays on an incomplete test
  assert _pick(r, prog, "21-27") == ti.SWEEP                            # window change: the first incomplete one
  assert r.pick(1, None, prog.remaining) == ti.SWEEP                    # nowhere: keep
  complete(prog, ti.SWEEP, "35-41")
  assert _pick(r, prog) == ti.PULSES                                    # complete: the next in order
  assert ti.RunAll(seed=1).pick(0, "35-41", lambda k, b, s: None) is None


def _all_complete(prog, window="35-41", level=200.0):
  """Every test complete in the window, and level at `level` total seconds each (extra on step 0)."""
  for k in ti.KEYS:
    complete(prog, k, window)
    have = sum(prog.have[(k, window, s_)] for s_ in range(ti.steps_in_band(k, window)))
    prog.have[(k, window, 0)] += level - have


def test_random_continuation_keeps_running_among_the_least_collected_and_is_seeded():
  prog = Progress()
  _all_complete(prog)
  seen = set()
  for seed in range(40):
    ev = []
    r = ti.RunAll(seed=seed, emit=lambda f, kind, **kw: ev.append(dict(kind=kind, frame=f, **kw)))
    t = _pick(r, prog)
    assert t in ti.KEYS and r.mode == "random" and r.draw == 1
    assert ev[0]["tied"] == list(ti.RUN_ALL_ORDER)                      # all complete: all within TIE_S
    seen.add(t)
    again = ti.RunAll(seed=seed)
    assert _pick(again, prog) == t                                      # same seed, same choice
  assert seen == set(ti.KEYS)                                           # random: every test gets drawn


def test_random_continuation_prefers_the_least_total_and_never_starves():
  prog = Progress()
  _all_complete(prog)
  prog.have[(ti.SWEEP, "35-41", 0)] += ti.TIE_S * 3                     # sweep far ahead of the others
  prog.have[(ti.LADDER, "35-41", 0)] += ti.TIE_S * 1.5
  r = ti.RunAll(seed=3)
  assert _pick(r, prog) == ti.PULSES                                    # the only one within TIE_S of the least
  # simulate the continuation: each pick adds time to the picked test; totals stay level within TIE_S + one block
  tot = {k: ti.window_total(k, "35-41", prog.remaining) for k in ti.KEYS}
  r = ti.RunAll(seed=5)
  counts = {k: 0 for k in ti.KEYS}
  for i in range(300):
    t = _pick(r, prog, frame=i)
    counts[t] += 1
    prog.have[(t, "35-41", 0)] += 4.0                                   # one block of a test: 4 s
    r.current = t
  totals = [ti.window_total(k, "35-41", prog.remaining) for k in ti.KEYS]
  assert max(totals) - min(totals) < ti.TIE_S + 4.0 + 1e-6
  assert all(c > 20 for c in counts.values())


def test_random_choice_stays_while_the_current_test_is_within_tie_and_leaves_when_it_leads():
  prog = Progress()
  _all_complete(prog)
  r = ti.RunAll(seed=2)
  first = _pick(r, prog)
  draws = r.draw
  prog.have[(first, "35-41", 0)] += ti.TIE_S - 1.0
  assert _pick(r, prog, frame=1) == first and r.draw == draws          # stays: no new draw
  prog.have[(first, "35-41", 0)] += 2.0
  assert _pick(r, prog, frame=2) != first and r.draw == draws + 1


def test_picks_are_logged_and_every_random_one_is_recomputed_from_the_log():
  prog = Progress()
  _all_complete(prog)
  ev = []
  r = ti.RunAll(seed=99, emit=lambda f, kind, **kw: ev.append(dict(kind=kind, frame=f, **kw)))
  for i in range(200):
    t = _pick(r, prog, frame=i * 60)                      # a second apart or so: inside the rate limit
    prog.have[(t, "35-41", 0)] += 5.0
    r.current = t
  picks = [e for e in ev if e["kind"] == "pick"]
  assert len(picks) > 10 and all(e["seed"] == 99 and e["window"] == "35-41" for e in picks)
  assert all(e["mode"] == "random" and set(e) >= {"test", "draw", "totals", "tied", "cands", "tie_s"} for e in picks)
  assert ti.verify_picks(ev) == []
  tampered = [dict(e) for e in ev]
  tampered[3]["test"] = next(k for k in ti.KEYS if k != tampered[3]["test"])
  assert ti.verify_picks(tampered)


def test_required_phase_picks_are_logged_too_and_a_blocked_test_is_passed_over_after_patience():
  prog = Progress()
  ev = []
  r = ti.RunAll(seed=1, emit=lambda f, kind, **kw: ev.append(dict(kind=kind, frame=f, **kw)))
  blocked = {ti.SWEEP}
  can = lambda k, w: k not in blocked
  assert _pick(r, prog, can_run=can) == ti.PULSES                        # blocked from the start: the next that can run
  r = ti.RunAll(seed=1, emit=lambda f, kind, **kw: ev.append(dict(kind=kind, frame=f, **kw)))
  assert _pick(r, prog) == ti.SWEEP
  n = int(ti.BLOCK_PATIENCE_S / ti.DT)
  got = [_pick(r, prog, frame=i, can_run=can) for i in range(1, n + 5)]
  assert got[0] == ti.SWEEP and got[n - 3] == ti.SWEEP and got[-1] == ti.PULSES   # patience, then passed over
  assert [e["mode"] for e in ev if e["kind"] == "pick"][-1] == "blocked"
  assert ev[-1]["passed_over"] == ti.SWEEP


def _run_all(n, seed=None, frame0=0, prog=None, rows=None):
  """The controller's run-all loop around one generator: pick, then update."""
  prog = prog if prog is not None else Progress()
  g = TiIdent(log=None)
  logged = []
  g.log = logged.append
  runner = ti.RunAll(seed=seed if seed is not None else 11, emit=g.event_for_pick)
  out, keys = [], []
  for k in range(n):
    row = rows[k] if rows is not None else quiet("35-41")
    profile = ti.zone_for(row["v_ego"]).profile
    armed = runner.pick(frame0 + k, profile, prog.remaining, lambda t, w: g.can_run(t, w, prog.remaining))
    out.append(g.update(armed, frame0 + k, **row, remaining=prog.remaining))
    keys.append(g.emitted)
    prog.add(g.credit)
  return g, out, keys, logged, prog


def test_run_all_runs_the_tests_in_order_and_only_after_a_full_fade_and_quiet():
  total = sum(band_frames(k, "35-41") for k in ti.KEYS) + 5000
  g, out, keys, logged, prog = _run_all(total)
  assert all(prog.band_done(k, "35-41") for k in ti.KEYS)
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
  assert g.reason in ("extra", "ok")
  assert [e for e in logged if e["kind"] == "pick" and e["mode"] == "random"]      # went on, drawing at random
  assert ti.verify_picks(logged) == []


def _random_rows(seed, n):
  rng = random.Random(seed)
  rows = []
  for k in range(n):
    seg = (k // 900) % 9
    v = {0: v_for("35-41"), 1: v_for("21-27"), 2: v_for("49-56"), 3: 29.5 * MPH, 4: v_for("64-80")}.get(seg % 5)
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
  complete(prog, ti.SWEEP, "35-41")
  complete(prog, ti.SWEEP, "49-56")
  complete(prog, ti.PULSES, "49-56")
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
  drive(g, ti.SWEEP, "35-41", 300)
  drive(g, ti.SWEEP, "35-41", 100, frame0=300, blinker=True)
  drive(g, ti.SWEEP, "35-41", 100, frame0=400, v_ego=85 * MPH)
  fb = g.frames_by(ti.SWEEP)
  assert fb["35-41"]["regime"] == SETTLE and fb["35-41"]["blinker"] == 100 and sum(fb["none"].values()) == 100
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
  _drive_acc(acc, g, "35-41", band_frames(ti.LADDER, "35-41") + 1200)
  acc.publish(1e6, {ti.LADDER: g.reason}, wall=1e9)
  doc = json.loads(mem.store[tt.PROGRESS_KEY])
  entry = doc["ti_ladder"]["35-41"]
  assert entry["2"] >= 60.0 and entry["3"] >= 40.0 and sum(entry.values()) > 100.0      # extra counted
  assert doc["_live"]["status"]["ti_ladder"] == tt.REASON_TEXT["extra"]
  assert all(acc.remaining(ti.LADDER, "35-41", s) <= 0 for s in range(4))
  assert acc.remaining(ti.LADDER, "21-27", 4) == 60.0 and acc.remaining(ti.LADDER, "21-27", 0) == 0.0
  # a reboot: the flash copy carries it
  acc.publish(1e6 + 100, {}, wall=1e9)
  acc2 = tt.TestAccountant(acc.tests, params, FakeParams())
  assert acc2.remaining(ti.LADDER, "35-41", 3) <= 0 and acc2.remaining(ti.LADDER, "49-56", 2) == 60.0
  # reset (what the UI writes)
  acc2.params_memory.store[tt.PROGRESS_KEY] = json.dumps({tt.RESET_KEY: {"ti_ladder": 7}})
  acc2.update(TOGGLES, v_for("35-41"), True, ())          # the test is on, so the next publish runs
  acc2.publish(2e6, {}, wall=1e9)
  assert acc2.remaining(ti.LADDER, "35-41", 2) == 60.0
  # a broken accountant: unknown progress, so nothing runs
  acc2.failed = True
  assert acc2.remaining(ti.LADDER, "35-41", 0) is None


def test_a_running_test_reads_active_between_credits():
  # The field report: "keeps switching to ON · WAIT". Units are credited every 2-7 s, so the 5 s live block must
  # say "active" while the test runs clean, not only in the intervals in which a unit was credited.
  acc, mem = _accountant(), None
  g = TiIdent()
  seen = []
  for k in range(SETTLE + 3000):
    g.update(ti.SWEEP, k, **quiet(), remaining=acc.remaining)
    active = (g.emitted,) if g.emitted and g.reason in ("ok", "extra") else ()
    acc.update(NS(**{**vars(TOGGLES), 'ti_ladder': False, 'ti_sweep': True}), v_for("35-41"), True, active,
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
    acc.update(ALL, v_for("35-41"), True, active, credit=g.credit)
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
    h.ti_run_all = ti.RunAll(seed=7, emit=h.ti_ident.event_for_pick)
    h.ti_ident_all_seen = False
    h.ti_ident_armed = None
    h.ti_ident_frames = {}
    h.ti_ident_seen = set()
    h.ti_tests = Acc()
  return h


def step(h, steer, *, lat_active=True, ti_allowed=True, v=v_for("35-41"), driver=-1.0, pressed=False, blinker=False,
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
    r['v'] = v_for("35-41") if r['v'] > 20 else v_for("21-27")
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
  if kw.get('run_all'):                                    # sweep already complete at 35-41 mph: the scheduler moves on
    complete(h.ti_tests.prog, ti.SWEEP, "35-41")
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
    drive(g, ti.SWEEP, "35-41", 600, frame0=len(logged) * 1000 + 10 ** 5, prog=prog)
    drive(g, ti.SWEEP, "35-41", 5, frame0=len(logged) * 1000 + 10 ** 5 + 600, prog=prog, **change)
  whys = [e.get("why") for e in logged if e["kind"] == "stop"]
  assert {"blinker", "curve", "lat", "command", "band", "touch"} <= set(whys)
  assert all("step" in e for e in logged if e["kind"] == "stop")


# ---- review round 2 ---------------------------------------------------------------------------------------------

def test_run_all_completion_is_per_step_not_per_band_total():
  # The fd05555 progress pattern: 20 s on every sweep step, the now-optional 0.3 Hz step included, is 140 s against
  # 67.3 s required at 35-41 mph, yet the 0.6 Hz step (26.56 s needed) is short.
  prog = Progress()
  for s_ in range(7):
    prog.have[(ti.SWEEP, "35-41", s_)] = 20.0
  assert ti.band_share(ti.SWEEP, "35-41", prog.remaining, clip=False) > 1.0
  assert ti.band_share(ti.SWEEP, "35-41", prog.remaining) < 1.0
  r = ti.RunAll(seed=1)
  assert _pick(r, prog) == ti.SWEEP
  assert _pick(r, prog) == ti.SWEEP
  # and the generator goes to the short step
  g = TiIdent()
  drive(g, ti.SWEEP, "35-41", SETTLE + 5, prog=prog)
  assert g.step == 1 and not g.extra


def test_progress_from_another_signal_table_is_kept_aside_and_does_not_count():
  mem, flash = FakeParams(), FakeParams()
  old = {"ti_sweep": {"35-41": {str(k): 20.0 for k in range(7)}, "49-56": {str(k): 20.0 for k in range(7)}},
         "ti_dither": {"43-50": 120.0}}
  flash.store[tt.PROGRESS_KEY] = json.dumps(old)              # what fd05555 left (no table recorded)
  acc = _accountant(flash, mem)
  assert acc.remaining(ti.SWEEP, "35-41", 1) == ti.required_seconds(ti.SWEEP, "35-41", 1)   # starts from zero
  acc.update(NS(**{**vars(TOGGLES), 'ti_ladder': False, 'ti_sweep': True}), v_for("35-41"), True, (),
             credit=(ti.SWEEP, "35-41", 1, 3.32))
  acc.publish(1.0, {}, wall=1e9)
  doc = json.loads(mem.store[tt.PROGRESS_KEY])
  assert doc["ti_sweep~old"] == old["ti_sweep"]                # kept, under its own key
  assert doc["ti_sweep"]["_table"] == ti.SIGNAL_HASH[ti.SWEEP] and doc["ti_sweep"]["35-41"] == {"1": 3.32}
  assert doc["ti_dither"] == {"43-50": 120.0}                  # the dither's record is untouched
  # a later table change moves this one aside too, under its hash
  doc["ti_sweep"]["_table"] = "0123456789ab"
  mem.store[tt.PROGRESS_KEY] = json.dumps(doc)
  acc2 = tt.TestAccountant(acc.tests, flash, mem)
  assert acc2.remaining(ti.SWEEP, "35-41", 1) == ti.required_seconds(ti.SWEEP, "35-41", 1)
  acc2.update(NS(**{**vars(TOGGLES), 'ti_ladder': False, 'ti_sweep': True}), v_for("35-41"), True, ())
  acc2.publish(2.0, {}, wall=1e9)
  doc2 = json.loads(mem.store[tt.PROGRESS_KEY])
  assert doc2["ti_sweep~0123456789ab"]["35-41"] == {"1": 3.32} and doc2["ti_sweep~old"] == old["ti_sweep"]
  # the same table survives a reboot as it is
  flash.store[tt.PROGRESS_KEY] = json.dumps({k: v for k, v in doc.items() if not k.startswith("_")})
  flash_doc = json.loads(flash.store[tt.PROGRESS_KEY])
  flash_doc["ti_sweep"]["_table"] = ti.SIGNAL_HASH[ti.SWEEP]
  flash.store[tt.PROGRESS_KEY] = json.dumps(flash_doc)
  acc3 = tt.TestAccountant(acc.tests, flash, FakeParams())
  assert acc3.remaining(ti.SWEEP, "35-41", 1) == pytest.approx(ti.required_seconds(ti.SWEEP, "35-41", 1) - 3.32)


def _transition_sums(key, band):
  """Counts and heading (running sum) of the signal over pairs of step transitions in a long extra run."""
  prog = Progress()
  for k_ in ti.KEYS:
    complete(prog, k_, band)
  g = TiIdent(log=None)
  logged = []
  g.log = logged.append
  out, _ = drive(g, key, band, 120000, prog=prog)
  return np.array(out, dtype=float), logged


@pytest.mark.parametrize('band', ["35-41", "21-27"])
@pytest.mark.parametrize('key', [ti.SWEEP, ti.LADDER])
def test_extra_mode_long_run_mean_and_heading_stay_near_zero(key, band):
  out, logged = _transition_sums(key, band)
  runs = [e["frame"] for e in logged if e["kind"] == "run"]
  assert len(runs) > 10                                        # many transitions in extra mode
  run_out = out[SETTLE:]
  assert abs(run_out.mean()) < 0.02                            # no constant bias (review: -0.36 / -1.14 before)
  heading = np.cumsum(run_out)
  # bounded by one unit's own heading swing plus one transition, not growing with the length of the run (a bias of
  # 0.36 counts would reach 43000 count-frames here)
  assert np.max(np.abs(heading)) < 4000
  assert abs(heading[-1]) < 4000


def test_extra_dwells_several_units_per_step():
  _, logged = _transition_sums(ti.SWEEP, "49-56")
  per_run = []
  units = 0
  for e in logged:
    if e["kind"] == "run":
      if units:
        per_run.append(units)
      units = 1
    elif e["kind"] == "chunk":
      units += 1
  assert per_run and min(per_run) >= ti.EXTRA_DWELL_UNITS


def test_the_dither_waits_for_a_running_test_to_end():
  h = host((ti.SWEEP,))
  for _ in range(M.frames(5)):
    step(h, 0.05)
  assert h.ti_ident.running
  h.ti_dither_enabled = True                                   # the dither switched on mid-test
  rows = [step(h, 0.05) for _ in range(M.frames(5))]
  both = [r for r in rows if r['dither'] != 0 and (r['ident'] != 0 or r['emitted'])]
  assert not both                                              # never both in one frame
  first_dither = next(i for i, r in enumerate(rows) if r['dither'] != 0)
  last_test = max(i for i, r in enumerate(rows) if r['emitted'])
  assert first_dither > last_test                              # the dither starts only after the test has ended


# ---- revision 3: window limits, re-arm dwell, gate readout, targets, credit events -------------------------------------

def test_no_amplitude_cap_or_limit_was_raised_in_revision_3():
  assert ti.LADDER_CAPS == {"21-27": 55.0, "35-41": 37.0, "49-56": 37.0, "64-80": 37.0}
  assert ti.PULSE_AMPS == (22.0, 44.0) and max(a for _, a in ti.SWEEP_TONES) == 22.0
  assert ti.LADDER_LOW_BAND_CAP == 55.0 and ti.MAX_SLEW == 3.0
  assert ti.COMMAND_LIMIT_FRACTION == {w[2]: (0.5, 0.75) for w in ti.WINDOWS}      # the previous revision's 300 / 450
  assert (ti.MAX_CURVATURE, ti.MAX_CURVATURE_SMALL, ti.MAX_LAT_DEMAND, ti.MAX_LAT_DEMAND_SMALL) == (0.004, 0.01, 1.0, 1.5)
  assert (ti.TOUCH_HOLDOFF_S, ti.TOUCH_HOLDOFF_SMALL_S, ti.BLINKER_HOLDOFF_S) == (3.0, 1.5, 3.0)
  assert ti.REGIME_SETTLE_S == 1.0 and ti.RATE_ROOM_FACTOR == 2


@pytest.mark.parametrize('window', [w[2] for w in ti.WINDOWS])
def test_the_command_limit_is_per_window_and_per_step_size(window, monkeypatch):
  step_small, step_big = 1, 3                               # ladder 11 counts and 37 counts
  g = TiIdent()
  assert g.limit_bounds(ti.LADDER, step_big, window, 600)["command"] == 300
  assert g.limit_bounds(ti.LADDER, step_small, window, 600)["command"] == 450
  monkeypatch.setitem(ti.COMMAND_LIMIT_FRACTION, window, (0.4, 0.6))     # the one place to change it for one window
  assert g.limit_bounds(ti.LADDER, step_big, window, 600)["command"] == 240
  other = next(w[2] for w in ti.WINDOWS if w[2] != window)
  assert g.limit_bounds(ti.LADDER, step_big, other, 600)["command"] == 300


def test_every_limit_still_closes_the_gate_and_reports_its_value_against_the_limit():
  for key, step, over, reason, value, limit in (
      (ti.LADDER, 3, dict(base_request=301), "command", 301, 300),
      (ti.SWEEP, 1, dict(curvature=0.011), "curve", 0.011, 0.01),
      (ti.LADDER, 3, dict(curvature=0.005), "curve", 0.005, 0.004),
      (ti.SWEEP, 1, dict(lat_demand=1.6), "lat", 1.6, 1.5),
      (ti.LADDER, 3, dict(lat_demand=1.1), "lat", 1.1, 1.0)):
    g, k, prog = _running(key, "35-41", step_to=step)
    g.update(key, k, **quiet("35-41", **over), remaining=prog.remaining)
    assert g.reason == reason and g.fading, (reason, g.reason)
    assert g.gate["value"] == pytest.approx(value, rel=1e-3) and g.gate["limit"] == pytest.approx(limit, rel=1e-3)
    assert g.gate["window"] == "35-41" and g.gate["what"] and g.gate["unit"]
    out = [g.update(key, k + 1 + i, **quiet("35-41", **over), remaining=prog.remaining) for i in range(400)]
    if ti.small(key, step):
      assert out[-1] == 0 and not g.running             # fades out, then nothing
    else:
      assert max(abs(v) for v in out[ti.fade_frames(key, step) + 1:]) <= ti.SMALL_AMP   # only small steps go on
    assert max(abs(b - a) for a, b in zip(out, out[1:])) <= ti.MAX_SLEW + 1


def _chatter(over_counts, period, n, window="21-27"):
  """The ladder 55 step at base commands hovering at the 300 limit: `period` frames alternately above and below."""
  logged = []
  g = TiIdent(log=logged.append)
  prog = Progress()
  complete(prog, ti.LADDER, window, range(4))
  out = []
  for k in range(n):
    base = 280 + (over_counts if (k // period) % 2 else 0)
    out.append(g.update(ti.LADDER, k, **quiet(window, base_request=base), remaining=prog.remaining))
    prog.add(g.credit)
  return g, out, logged


def test_a_command_hovering_at_the_limit_does_not_start_and_cut_units_over_and_over():
  # 9 frames (0.09 s) above 300, 9 below: with the dwell no unit ever starts (before, one started every time it dipped)
  g, out, logged = _chatter(30, 9, 3000)
  assert [e for e in logged if e["kind"] in ("run", "chunk") and e["amp"] > ti.SMALL_AMP] == []   # no 55-count unit
  assert [e for e in logged if e["kind"] == "stop" and e["why"] == "command"] == []              # and none cut by it
  assert max(abs(v) for v in out) <= ti.SMALL_AMP
  assert g.gate_last is not None and g.gate_last[1]["reason"] == "command"      # the screen still says why


def test_the_gate_rearms_after_the_dwell_and_cuts_stay_immediate():
  logged = []
  g = TiIdent(log=logged.append)
  prog = Progress()
  complete(prog, ti.LADDER, "21-27", range(4))
  base = 350
  for k in range(1500):
    if k == 400:
      base = 250                                           # the command comes down for good
    if k == 1000:
      base = 360                                           # and goes over again
    g.update(ti.LADDER, k, **quiet("21-27", base_request=base), remaining=prog.remaining)
  first = next(e["frame"] for e in logged if e["kind"] in ("run", "chunk") and e["amp"] == 55.0)
  dwell = int(round(ti.REARM_S / ti.DT))
  unit = ti.step_params(ti.LADDER, 0)[2]                    # a small extra unit under way finishes first (no dead time)
  assert 400 + dwell <= first <= 400 + dwell + unit + ti.fade_frames(ti.LADDER, 0) + ti.LADDER_LEAD_IN
  stop = next(e for e in logged if e["kind"] == "stop" and e["frame"] >= 1000)
  assert stop["frame"] == 1000 and stop["why"] == "command"                       # the cut is at once


def test_a_unit_cut_in_its_lead_in_fades_only_what_had_come_in():
  g = TiIdent()
  prog = Progress()
  complete(prog, ti.LADDER, "21-27", range(4))
  k = 0
  while not g.running:
    g.update(ti.LADDER, k, **quiet("21-27"), remaining=prog.remaining)
    k += 1
  for _ in range(20):                                       # 20 frames into a 100-frame lead-in
    g.update(ti.LADDER, k, **quiet("21-27"), remaining=prog.remaining)
    k += 1
  env = g.env
  assert 0 < env < 0.3
  g.update(ti.LADDER, k, **quiet("21-27", base_request=400), remaining=prog.remaining)
  n = 0
  while g.running and n < 1000:
    n += 1
    k += 1
    g.update(ti.LADDER, k, **quiet("21-27", base_request=400), remaining=prog.remaining)
  assert n <= int(env * ti.fade_frames(ti.LADDER, 4)) + 3                      # only the remaining envelope fades


def test_run_all_passes_over_a_test_blocked_only_by_a_limit_and_keeps_a_signal_running():
  prog = Progress()
  complete(prog, ti.SWEEP, "21-27")
  complete(prog, ti.PULSES, "21-27")                        # only the ladder is left: its 55-count step is closed
  g = TiIdent()
  r = ti.RunAll(seed=4, emit=g.event_for_pick)
  outs = []
  for k in range(4000):                                     # the base command sits above 300 the whole time
    armed = r.pick(k, "21-27", prog.remaining, lambda t, w: g.can_run(t, w, prog.remaining))
    outs.append(g.update(armed, k, **quiet("21-27", base_request=330), remaining=prog.remaining))
    prog.add(g.credit)
  assert sum(1 for v in outs if v) > 2000                   # the small steps of the same test keep it busy
  assert max(abs(v) for v in outs) <= 22
  assert all(e["amp"] <= 22 for e in g.events if e["kind"] in ("run", "chunk"))


def test_a_blocked_big_step_does_not_stall_the_test_while_a_small_step_may_run():
  prog = Progress()
  g = TiIdent()
  out = [g.update(ti.LADDER, k, **quiet("21-27", base_request=330), remaining=prog.remaining) for k in range(3000)]
  assert sum(1 for v in out if v) > 1500 and max(abs(v) for v in out) <= 22


def test_credit_events_carry_a_progress_snapshot_for_the_analysis():
  logged = []
  g = TiIdent(log=logged.append)
  prog = Progress()
  for k in range(6000):
    g.update(ti.SWEEP, k, **quiet("35-41"), remaining=prog.remaining)
    prog.add(g.credit)
  credits = [e for e in logged if e["kind"] == "credit"]
  assert credits
  c = credits[0]
  assert set(c) >= {"test", "band", "step", "s", "left", "frame"} and c["band"] == "35-41"
  assert c["left"] == pytest.approx(ti.required_seconds(ti.SWEEP, "35-41", c["step"]) - c["s"], abs=0.01)
  assert ti.reconstruct(logged, 6000)


def test_target_window_least_speed_change_then_most_outstanding_and_how_much_is_left():
  prog = Progress()
  r = prog.remaining
  w, secs, cells, dist = ti.target_window(52.0, r)
  assert (w, dist) == ("49-56", 0.0) and cells > 0 and secs > 0                 # inside: the window itself
  assert ti.target_window(30.0, r)[0] == "21-27" and ti.target_window(33.0, r)[0] == "35-41"
  assert ti.window_target_mph("21-27") == 24 and ti.window_target_mph("35-41") == 38
  assert ti.window_target_mph("49-56") == 52 and ti.window_target_mph("64-80") == 72
  # 60 mph is 4 mph above 49-56 and 4 below 64-80: the one with more outstanding wins the tie
  prog.have[(ti.SWEEP, "64-80", 1)] = 13.0
  assert ti.target_window(60.0, r)[0] == "49-56"
  for k in ti.KEYS:
    complete(prog, k, "49-56")
  assert ti.target_window(52.0, r)[0] != "49-56"                                  # complete windows are not targets
  for w_ in ("21-27", "35-41", "64-80"):
    for k in ti.KEYS:
      complete(prog, k, w_)
  assert ti.target_window(52.0, r) is None
  assert ti.target_window(52.0, lambda k, b, s: None) is None
  one = ti.target_window(24.0, Progress().remaining, keys=(ti.PULSES,))
  assert one[0] == "21-27" and one[2] == 1                                        # pulses: one required cell at 21-27


def test_revision_2_progress_is_moved_aside_and_the_new_windows_start_empty():
  # What the installed build (526c88c, generator 5) keeps: bands 19-28 ... 60-80, signal hash without the windows
  rev2_hash = "r2sweep00000"
  old = {"ti_sweep": {"_table": rev2_hash, "19-28": {"1": 30.0}, "31-45": {"1": 26.56}, "60-80": {"6": 4.0}},
         "ti_sweep~old": {"19-28": {"0": 20.0}}, "ti_dither": {"43-50": 120.0}}
  mem, flash = FakeParams(), FakeParams()
  flash.store[tt.PROGRESS_KEY] = json.dumps(old)
  acc = _accountant(flash, mem)
  for w in ("21-27", "35-41", "49-56", "64-80"):
    assert acc.remaining(ti.SWEEP, w, 1) == ti.required_seconds(ti.SWEEP, w, 1)       # nothing carried over
  acc.update(NS(**{**vars(TOGGLES), 'ti_ladder': False, 'ti_sweep': True}), v_for("35-41"), True, (),
             credit=(ti.SWEEP, "35-41", 1, 3.32))
  acc.publish(1.0, {}, wall=1e9)
  doc = json.loads(mem.store[tt.PROGRESS_KEY])
  assert doc[f"ti_sweep~{rev2_hash}"] == {k: v for k, v in old["ti_sweep"].items() if k != "_table"} | {"_table": rev2_hash}
  assert doc["ti_sweep~old"] == old["ti_sweep~old"]                    # the earlier aside is untouched
  assert doc["ti_dither"] == old["ti_dither"]
  assert doc["ti_sweep"]["35-41"] == {"1": 3.32} and set(doc["ti_sweep"]) == {"_table", "35-41"}
  # the signal hash changed with the windows, so any record collected under the old windows is set aside
  import hashlib
  old_signal = ["zero-heading phase, sign alternating per run", ti._table(ti.SWEEP)]
  assert hashlib.sha1(json.dumps(old_signal, sort_keys=True).encode()).hexdigest()[:12] != ti.SIGNAL_HASH[ti.SWEEP]


def test_the_gate_block_is_published_once_a_second_for_the_screen_and_cleared_after():
  publish = _controller_method('publish_ti_gate')
  mem = FakeParams()
  puts = []
  mem.put = lambda k, v: puts.append((k, json.loads(v)))
  g = TiIdent()
  prog = Progress()
  complete(prog, ti.LADDER, "21-27", range(4))
  h = NS(ti_ident=g, ti_tests=object(), ti_ident_armed=ti.LADDER, params_memory=mem, ti_gate_published=None)
  env = dict(self=h, json=json, time=__import__("time"))
  fn = publish
  fn.__globals__.update(json=json, time=__import__("time"))
  for k in range(150):
    g.update(ti.LADDER, k, **quiet("21-27", base_request=330), remaining=lambda a, b, c: 0.0 if (a, b, c) != (ti.LADDER, "21-27", 4) else 60.0)
  fn(h)
  assert puts and puts[-1][0] == "TiTestGate"
  blk = puts[-1][1]
  assert blk["reason"] == "command" and blk["value"] == 330 and blk["limit"] == 300 and blk["window"] == "21-27"
  assert blk["test"] == ti.LADDER and blk["t"] > 0
  # the limit no longer closes anything: one clearing block, then nothing more
  for k in range(150, 400):
    g.update(ti.LADDER, k, **quiet("21-27", base_request=100), remaining=lambda a, b, c: 0.0)
  fn(h)
  assert set(puts[-1][1]) == {"t"}
  n = len(puts)
  fn(h)
  assert len(puts) == n


# ---- review fixes: speed hysteresis, pick log rate, non-finite inputs -----------------------------------------------

def _mph_run(g, prog, mph_seq, key=ti.SWEEP, start=0, blocked_for_mph=None):
  out, reasons = [], []
  for k, mph in enumerate(mph_seq):
    blocked = (mph < 28) if blocked_for_mph is None else blocked_for_mph(mph)
    out.append(g.update(key, start + k, **quiet("35-41", v_ego=mph * MPH, lkas_blocked=blocked), remaining=prog.remaining))
    prog.add(g.credit)
    reasons.append(g.reason)
  return out, reasons


@pytest.mark.parametrize('edge,going_off,start,end', [(19.0, "down", 24.0, 18.5), (28.0, "up", 24.0, 28.2),
                                                      (31.0, "down", 35.0, 30.5), (80.0, "up", 72.0, 80.5)])
def test_entering_an_off_region_starts_the_fade_at_once(edge, going_off, start, end):
  g = TiIdent()
  prog = Progress()
  _mph_run(g, prog, [start] * (SETTLE + 300))
  assert g.running and g.env == 1.0
  _, reasons = _mph_run(g, prog, [end], start=SETTLE + 300)
  assert g.fading and reasons[-1] in ("band", "transition")         # the very first frame past the edge


@pytest.mark.parametrize('edge,low,high', [(19.0, 18.5, 19.9), (80.0, 79.0, 80.5)])
def test_hovering_at_a_hard_edge_never_delays_off_and_delays_on_only_until_clear(edge, low, high):
  g = TiIdent()
  prog = Progress()
  start = 24.0 if edge == 19.0 else 72.0
  _mph_run(g, prog, [start] * (SETTLE + 300))
  seq = ([low] * 3 + [high] * 3) * 200                       # flips every 3 frames across the edge
  out, reasons = _mph_run(g, prog, seq, start=SETTLE + 300)
  starts = [e for e in g.events if e["kind"] in ("run", "chunk") and e["frame"] >= SETTLE + 300]
  assert starts == []                                        # never restarted while hovering
  assert all(r in ("band", "transition", "switch") or r for r in reasons)
  assert not any(out[200:])                                  # and quiet: no signal left
  # all the way clear, it comes back
  _, reasons = _mph_run(g, prog, [24.0 if edge == 19.0 else 72.0] * (SETTLE + 400), start=SETTLE + 300 + len(seq))
  assert g.running


def test_the_28_31_edges_hold_off_until_one_mph_clear_and_cut_in_at_once():
  g = TiIdent()
  prog = Progress()
  _mph_run(g, prog, [24.0] * (SETTLE + 300))
  _, r = _mph_run(g, prog, [28.0], start=5000)
  assert r[-1] == "transition" and g.fading
  for mph in (27.9, 27.5, 27.1, 28.4, 30.9, 31.0, 31.5, 31.9):      # hovering round both edges: stays off
    _, r = _mph_run(g, prog, [mph] * 400, start=6000, blocked_for_mph=lambda m: m < 28)
    assert r[-1] in ("transition", "band", "regime"), (mph, r[-1])
    assert not g.running
  _, r = _mph_run(g, prog, [33.0] * (SETTLE + 400), start=9000)
  assert g.running                                                  # clear of 32: on


def _hover_events(mid, hysteresis, monkeypatch):
  monkeypatch.setattr(ti, "ZONE_HYSTERESIS_MPH", hysteresis)
  ev = []
  g = TiIdent(log=ev.append)
  r = ti.RunAll(seed=3, emit=g.event_for_pick)
  prog = Progress()
  _all_complete(prog, "35-41" if mid == 45.0 else "49-56")       # the lower window done, the upper one not
  for k in range(7000):
    mph = mid + (0.6 if (k // 7) % 2 else -0.6) if k > 400 else mid - 3.0     # flips every 7 frames, +-0.6 mph
    zone = g.advance_zone(mph * MPH)
    armed = r.pick(k, zone.profile, prog.remaining, lambda t, w: g.can_run(t, w, prog.remaining))
    g.update(armed, k, **quiet("35-41", v_ego=mph * MPH, lkas_blocked=False), remaining=prog.remaining)
    prog.add(g.credit)
  return ev, g


@pytest.mark.parametrize('mid', [45.0, 60.0])
def test_a_car_hovering_at_the_45_and_60_mph_profile_midpoints_makes_no_churn_and_no_event_flood(mid, monkeypatch):
  with_h, _ = _hover_events(mid, 1.0, monkeypatch)
  without, _ = _hover_events(mid, 0.0, monkeypatch)
  assert len(with_h) < 30 and len(without) > 3 * len(with_h)         # the same drive without the margin floods the log
  # clearly past the midpoint it does switch
  ev, g = _hover_events(mid, 1.0, monkeypatch)
  for k in range(600):
    g.update(ti.SWEEP, 8000 + k, **quiet("35-41", v_ego=(mid + 3.0) * MPH, lkas_blocked=False))
  assert g.zone.profile == ("49-56" if mid == 45.0 else "64-80")


def test_the_held_profile_never_crosses_the_steering_regime():
  g = TiIdent()
  for mph in (24.0, 27.6, 20.5, 33.0, 38.0, 43.9, 45.4, 47.0, 58.0, 61.5, 70.0):
    z = g.advance_zone(mph * MPH)
    g.n += 1
    if z.profile is not None:
      assert (z.profile == "21-27") == (mph < 28.0)                  # the 55-count cap profile only below 28 mph
      assert z.ti_alone == (mph < 28.0)


def test_pick_events_are_rate_limited_and_say_how_many_were_held_back():
  ev = []
  r = ti.RunAll(seed=5, emit=lambda f, kind, **kw: ev.append(dict(kind=kind, frame=f, **kw)))
  prog = Progress()
  _all_complete(prog)
  for i in range(1000):                                              # a window flipping every frame: a pick per frame
    window = "35-41" if i % 2 else "49-56"
    if window == "49-56":
      _all_complete(prog, "49-56")
    r.pick(i, window, prog.remaining)
  assert 0 < len(ev) <= 1000 * ti.DT / ti.PICK_LOG_MIN_S + 1          # at most one per PICK_LOG_MIN_S
  assert sum(e.get("skipped", 0) for e in ev) > 100
  assert ti.verify_picks(ev) == []


@pytest.mark.parametrize('field', ['v_ego', 'curvature', 'lat_demand', 'base_request', 'driver_torque'])
@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
@pytest.mark.parametrize('key,step', [(ti.SWEEP, 1), (ti.LADDER, 3)])
def test_a_non_finite_input_closes_the_gate_and_zeroes_at_once(field, bad, key, step):
  g, k, prog = _running(key, "35-41", step_to=step)
  over = {field: bad}
  assert g.update(key, k, **quiet("35-41", **over), remaining=prog.remaining) == 0 and not g.running
  assert g.reason == "input"
  out = [g.update(key, k + 1 + i, **quiet("35-41", **over), remaining=prog.remaining) for i in range(600)]
  assert not any(out) and g.reason == "input"
  assert [e for e in g.events if e["kind"] == "run" and e["frame"] > k] == []
  # and it starts again from nothing once the input is finite
  out = [g.update(key, k + 700 + i, **quiet("35-41"), remaining=prog.remaining) for i in range(SETTLE + 400)]
  assert any(out)


def test_a_non_finite_input_is_also_a_closed_limit_for_scheduling():
  g = TiIdent()
  prog = Progress()
  g.update(ti.LADDER, 0, **quiet("35-41", curvature=float('nan')), remaining=prog.remaining)
  assert g.can_run(ti.LADDER, "35-41", prog.remaining) is False
  assert g.limit_state(ti.LADDER, 3, "35-41", 50, float('nan'), 0.1, 600)[0] == "curve"
