"""The NNFF gain correction's clamp, in the TI-only regime.

Below Mazda's ~45 km/h LKAS gate the stock EPS path is dead (measured: 100 % of engaged clean
frames below 45 km/h are TI-only with lkas_block set) and the car has roughly half its highway
authority. The network was trained on stock-Mazda data, which cannot contain a single frame from
that regime -- stock LKAS does not operate there -- so it asks as though the stock path were
helping. The state-aware required correction (plant inverse / network steady ask) is 2.0-2.8 in
TI-only against 1.03-1.27 with both actuators; the old flat cap of 1.5 vetoed the measurement and
left the feedforward at ~0.71 of need, which drove the measured 0.74 delivery (vs the plant
branch's 0.98) below 45 km/h -- the "no steering pulling away from a stoplight" complaint.

These tests pin: the required ratio really is >2 in TI-only (physics, from the real network and
plant), the correction now follows it there up to GAIN_MAX_TI_ONLY, and nothing changes in the
both-actuators regime where 1.5 was never binding.
"""
import os
import sys
import types

import pytest

# neural_network_feedforward imports frogpilot_variables, which circularly imports sentry ->
# frogpilot_variables; on the device the import ORDER primes the cycle, but any direct import
# (including pytest collection) hits it. The module only needs three names from it, none of which
# these tests exercise (the controller is built via __new__), so inject a minimal stand-in.
_ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "..", "..", "..", "frogpilot", "assets", "nnff_models")
try:
  import openpilot.frogpilot.common.frogpilot_variables  # noqa: F401
except ImportError:
  fv = types.ModuleType("openpilot.frogpilot.common.frogpilot_variables")
  fv.NNFF_MODELS_PATH = os.path.normpath(_ASSETS)
  fv.get_nnff_model_files = lambda: [f[:-5] for f in os.listdir(fv.NNFF_MODELS_PATH) if f.endswith(".json")]
  fv.get_nnff_substitutes = lambda: {}
  sys.modules["openpilot.frogpilot.common.frogpilot_variables"] = fv

from cereal import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.frogpilot.controls.lib.neural_network_feedforward import (
  FluxModel, LatControlNNFF, GAIN_MIN, GAIN_MAX, GAIN_MAX_TI_ONLY, GAIN_TAU,
)
from openpilot.selfdrive.car.mazda.lateral_plant import TiLateralPlant

DT = 0.01
MODEL_PATH = os.path.join(os.path.normpath(_ASSETS), "MAZDA_CX9_2021.json")


def controller(stock_active):
  """A LatControlNNFF with just what _gain_ratio touches, in a chosen actuator state."""
  c = LatControlNNFF.__new__(LatControlNNFF)
  c.lat_torque_nn_model = FluxModel(MODEL_PATH)
  c.nnff_loaded = True
  c.plant = TiLateralPlant(600.0)
  c.plant.ti_active = True
  c.plant.stock_active = stock_active
  c.plant.ramp_in = False
  c.gain_ratio = FirstOrderFilter(1.0, GAIN_TAU, DT)
  return c


def settled_gain(c, v, la, corr_on=True, n=1500):
  CS = car.CarState.new_message()
  CS.vEgo = v
  toggles = types.SimpleNamespace(nnff_gain_correction=corr_on)
  g = 1.0
  for _ in range(n):
    g = c._gain_ratio(CS, 0.0, la, toggles)
  return g


class TestGainClamp:
  def test_required_ratio_exceeds_old_cap_in_ti_only(self):
    """The physics premise: at 30-40 km/h TI-only, the plant needs >=2x what the network asks."""
    c = controller(stock_active=False)
    for v in (8.3, 11.1):
      need = c.plant.inverse(1.0, v) / 600.0
      ask = c._model_steady_ff(v, 1.0, 0.0)
      assert need / ask > 2.0

  def test_ti_only_correction_follows_the_measurement(self):
    """With the stock path dead the correction may exceed the old 1.5 cap, up to the TI-only cap."""
    c = controller(stock_active=False)
    g = settled_gain(c, 11.1, 1.0)
    assert g > GAIN_MAX + 0.2
    assert g <= GAIN_MAX_TI_ONLY + 1e-6

  def test_both_actuators_unchanged(self):
    """At highway speed with both actuators the computed ratio never met the old cap; the new cap
    must not change it."""
    c = controller(stock_active=True)
    g = settled_gain(c, 20.3, 1.5)
    assert 0.9 < g <= GAIN_MAX + 1e-6

  def test_cap_still_a_cap(self):
    """Even in TI-only the correction is bounded and floored."""
    c = controller(stock_active=False)
    g = settled_gain(c, 8.3, 0.5)      # required ~2.8, near the cap
    assert GAIN_MIN <= g <= GAIN_MAX_TI_ONLY + 1e-6

  def test_off_means_one(self):
    c = controller(stock_active=False)
    assert settled_gain(c, 11.1, 1.0, corr_on=False, n=1500) == pytest.approx(1.0, abs=1e-3)

  def test_no_plant_means_one(self):
    c = controller(stock_active=False)
    c.plant = None
    assert settled_gain(c, 11.1, 1.0) == pytest.approx(1.0, abs=1e-3)
