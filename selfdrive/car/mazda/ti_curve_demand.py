"""What a bend asks of the steering against what the steering can hold (pure functions; curve slowing, plan section 1 / 3).

Curvature k is LEFT POSITIVE (1/m) everywhere here, as the planner's yaw-based curvature and the road memory are. Demand is the lateral
acceleration the bend needs at a speed: v^2 |k| plus the road's bank term (sign(k) * g_bank; 0 until the memory carries a corrected
bank, `Memory.bank_v >= 2`). Capacity is what the steering held at its limit, measured per speed band and per DIRECTION (the car holds
less to the right from 35 mph). Two tables:
  CAP_BANKED  measured with the bank term removed (governor round two; the existing figure under 35 mph)
  CAP_RAW     measured without a bank term (round one): what the forecast, the map and a memory without corrected bank are judged on
A bend's speed (`need_mph`) is the highest whole mph at which its demand stays within the capacity of its direction, checked over
+-SPEED_WINDOW_MPH (band edges, speed error). Graded (plan A6, owner 2026-11-24): the drop follows the excess, nothing forces a minimum.

The driving model's preview under-reads the bend the further ahead it looks: FC_RATIO (raw / actual, measured per lead) calibrates it.
"""
import math

import numpy as np

MPH = 0.44704
CAR_MIN_MPH = 19
MAX_MPH = 90
SPEED_WINDOW_MPH = 0.75
LEFT, RIGHT, UNKNOWN = 1, -1, 0

# (lowest mph of the band, left m/s^2, right m/s^2); the figure is the band CENTRE value (capacity() interpolates between centres, R7)
CAP_BANKED = ((0.0, 1.41, 1.41), (32.0, 2.20, 2.20), (35.0, 2.48, 2.01), (45.0, 2.47, 1.91), (55.0, 2.51, 1.83))
CAP_RAW = ((0.0, 1.41, 1.41), (32.0, 2.23, 2.30), (35.0, 2.49, 2.42), (45.0, 2.56, 2.19), (55.0, 2.48, 2.07))   # under 32: the banked figure
# the model preview's lateral acceleration, raw over actual, per lead (s); flat beyond the last
FC_LEAD_S = (0.0, 2.0, 4.0, 5.0, 6.0, 8.0, 10.0)
FC_RATIO = (1.0, 0.951, 0.888, 0.839, 0.783, 0.700, 0.538)      # north-dakota (every recorded drive and the memory ran it)

# Per driving model (the `Model` param; the planner process reads it once at start). The calibration is model-specific: the same
# recorded frames previewed by each model (2026-11-25) give, as median forecast / realised curvature on bends:
#   E:/openpilot-mazda-ti-data/model-switch-20261125/results-20261124.txt (16 segments, 14,303 rows; 2,393 / 2,258 / 2,171 / 2,098
#     bend rows at 2 / 4 / 5 / 6 s): north-dakota 0.997 / 0.971 / 0.932 / 0.893, dark-souls 1.023 / 0.931 / 0.912 / 0.840
#     (dark-souls / north-dakota: 1.026 / 0.959 / 0.979 / 0.941);
#   E:/openpilot-mazda-ti-data/model-compare-20261125/results.json (20 replays of 3 sets, 68 bend passes; ~12,800-13,250 bend rows
#     per lead, pooled as the median over files of each file's median): dark-souls / north-dakota 0.965 / 0.958 / 0.912 / 0.884
#     (19-20 files).
# That measurement's own north-dakota figures are not FC_RATIO's (its realised curvature and bend rows are defined differently:
# 0.893 at 6 s against FC_RATIO's 0.783), so dark-souls' entry is north-dakota's FC_RATIO times the measured dark-souls /
# north-dakota ratio on the SAME frames (the model difference, not a second calibration). The two sources disagree at 2 and 5 s: the
# larger bend sample (model-compare, ~12,800 rows a lead against ~2,100-2,400) is used at every lead; it is also the cautious one
# (reads a bend higher). 0 s: 1.0 as north-dakota's; 8 and 10 s (not measured): the 6 s ratio carried on.
DS_OVER_ND = (1.0, 0.965, 0.958, 0.912, 0.884, 0.884, 0.884)
FC_RATIO_BY_MODEL = {
  "north-dakota": FC_RATIO,
  "dark-souls": tuple(round(a * b, 3) for a, b in zip(FC_RATIO, DS_OVER_ND)),   # 1.0, .918, .851, .765, .692, .619, .476
}
DEFAULT_FC_MODEL = "north-dakota"     # an unknown or unreadable name: north-dakota's values (the planner logs it once)


def fc_ratio_for(model):
  """-> (FC_RATIO table for the driving model `model`, known). Unknown / empty: DEFAULT_FC_MODEL's table and False."""
  r = FC_RATIO_BY_MODEL.get(model or "")
  return (r, True) if r is not None else (FC_RATIO_BY_MODEL[DEFAULT_FC_MODEL], False)


# Ruling R7 (plan A12): capacity is CONTINUOUS in speed from 32 mph: linear between the band centres (32-34: 33, 35-44: 39.5, 45-54:
# 49.5, 55+: 60 mph), flat between 32 and 33 and beyond 60; the one real step stays at STEP_MPH (below it the stock lane-keep path is
# blocked: 1.41). Band steps turned a bend 10 % over capacity at 45 mph into a 4 mph drop to the left and 1 to the right.
STEP_MPH = 32.0
BAND_CENTRES_MPH = (33.0, 39.5, 49.5, 60.0)


def _centre_table(table):
  return (np.array([r[1] for r in table[1:]], np.float64), np.array([r[2] for r in table[1:]], np.float64))


_CENTRE = {False: _centre_table(CAP_RAW), True: _centre_table(CAP_BANKED)}


# Ruling R14 (plan A15): no slowing INTO the low-capacity zone. The step at STEP_MPH is real, but a bend that fits at or above
# STEP_MPH on the capacity just above the step is never planned below STEP_MPH because the capacity steps down there: need_mph(...,
# hold_step=True) judges every speed below STEP_MPH on the capacity AT STEP_MPH, and need_hi() keeps that figure only where it is at
# least STEP_MPH (a bend too sharp even for that capacity must be taken below the step anyway, where the low-speed capacity is what
# counts: its stepped figure). The planner uses need_hi() while the car is at or above STEP_MPH (or a bend it started from there is
# still in progress); a car already below STEP_MPH plans on the stepped capacity (need_mph as always).
def capacity(mph, side, banked=False, hold_step=False):
  """m/s^2 the steering holds at `mph` turning to `side` (+1 left, -1 right, 0 unknown: the LOWER of the two). Continuous from
  STEP_MPH (R7): linear between the band centres; below STEP_MPH the lowest band's figure (hold_step: the figure at STEP_MPH, R14)."""
  table = CAP_BANKED if banked else CAP_RAW
  if hold_step and mph < STEP_MPH:
    mph = STEP_MPH
  if mph < STEP_MPH:
    left, right = table[0][1], table[0][2]
  else:
    cl, cr = _CENTRE[bool(banked)]
    left = float(np.interp(mph, BAND_CENTRES_MPH, cl))
    right = float(np.interp(mph, BAND_CENTRES_MPH, cr))
  if side > 0:
    return left
  if side < 0:
    return right
  return min(left, right)


def demand(mph, k, g_bank=0.0):
  """m/s^2 the bend of signed curvature k (left +) asks at `mph`: v^2 |k| + sign(k) g_bank."""
  v = mph * MPH
  return v * v * abs(k) + math.copysign(1.0, k) * g_bank if k != 0 else 0.0


_RUNGS = np.arange(MAX_MPH, CAR_MIN_MPH - 1, -1)          # highest first


def _window(r):
  return (max(float(CAR_MIN_MPH), r - SPEED_WINDOW_MPH), float(r), r + SPEED_WINDOW_MPH)


def _kmax_table(side, banked, hold_step=False):
  """For every rung: the largest |k| (no bank term) whose demand fits the capacity at every speed of the rung's window."""
  return np.array([min(capacity(s, side, banked, hold_step) / (s * MPH) ** 2 for s in _window(int(r))) for r in _RUNGS])


_KMAX = {(s, b): _kmax_table(s, b) for s in (LEFT, RIGHT, UNKNOWN) for b in (False, True)}
_KMAX_HOLD = {(s, b): _kmax_table(s, b, True) for s in (LEFT, RIGHT, UNKNOWN) for b in (False, True)}


def need_hi(k, g_bank=None):
  """R14: the need of a car at or above STEP_MPH: on the capacity held at its STEP_MPH figure below the step where that need is at
  least STEP_MPH, else (the bend asks to go below the step whatever the step) the stepped need. -> int array."""
  hold = need_mph(k, g_bank, hold_step=True)
  return np.where(hold >= STEP_MPH, hold, need_mph(k, g_bank))


def need_mph(k, g_bank=None, hold_step=False):
  """Highest whole mph (CAR_MIN_MPH .. MAX_MPH) at which each signed curvature k fits its direction's capacity over the speed window;
  CAR_MIN_MPH when none does. g_bank None: no bank term, CAP_RAW; else the bank term (m/s^2, per point or scalar) and CAP_BANKED.
  hold_step: below STEP_MPH judged on the capacity at STEP_MPH (R14; use need_hi). Vectorised. -> int array."""
  kmax = _KMAX_HOLD if hold_step else _KMAX
  k = np.atleast_1d(np.asarray(k, np.float64))
  ka = np.abs(k)
  out = np.full(len(k), CAR_MIN_MPH, np.int64)
  if g_bank is None:
    for side in (LEFT, RIGHT):
      sel = (k > 0) if side == LEFT else (k <= 0)
      if not sel.any():
        continue
      ok = ka[sel, None] <= kmax[(side, False)][None, :] + 1e-12
      first = np.where(ok.any(axis=1), ok.argmax(axis=1), len(_RUNGS) - 1)
      out[sel] = _RUNGS[first]
    return out
  g = np.broadcast_to(np.asarray(g_bank, np.float64), k.shape)
  sg = np.where(k > 0, 1.0, np.where(k < 0, -1.0, 0.0))
  side = np.where(k > 0, LEFT, RIGHT)
  done = np.zeros(len(k), bool)
  for r in _RUNGS:
    ok = ~done
    for s in _window(int(r)):
      cap = np.where(side == LEFT, capacity(s, LEFT, True, hold_step), capacity(s, RIGHT, True, hold_step))
      ok &= (s * MPH) ** 2 * ka + sg * g <= cap + 1e-9
    out[ok] = r
    done |= ok
  return out


def fits(k_abs, mph):
  """True when |k| fits at `mph` over the speed window on the LOWER side's raw capacity (direction unknown)."""
  return all(abs(k_abs) * (s * MPH) ** 2 <= capacity(s, UNKNOWN) + 1e-12 for s in _window(mph))


def calibrate(ts, la_raw, ratio=None):
  """The model preview's lateral acceleration at leads `ts` (s) divided by FC_RATIO at that lead: what the bend will actually ask.
  ratio: the driving model's table (fc_ratio_for), default FC_RATIO (north-dakota)."""
  return np.asarray(la_raw, np.float64) / np.interp(np.asarray(ts, np.float64), FC_LEAD_S, FC_RATIO if ratio is None else ratio)


def forecast_need(ts, k, v, lo_s, hi_s, hi=False, ratio=None):
  """The forecast's sharpest bend between leads lo_s and hi_s (s): ts, k (signed curvature, LEFT POSITIVE) of the preview, v (m/s)
  the speed now; hi: the need as need_hi (R14: a car at or above STEP_MPH). -> (need mph, lead s, calibrated signed demand m/s^2 at
  the speed now, side +1 / -1), or (None, None, None, 0)."""
  ts = np.asarray(ts, np.float64)
  k = np.asarray(k, np.float64)
  sel = (ts >= lo_s - 1e-9) & (ts <= hi_s + 1e-9) & np.isfinite(k)
  if not sel.any():
    return None, None, None, 0
  v = max(float(v), 1.0)
  la = calibrate(ts[sel], v * v * np.abs(k[sel]), ratio)
  j = int(np.argmax(la))
  kj = float(k[sel][j])
  side = LEFT if kj > 0 else RIGHT if kj < 0 else UNKNOWN
  if side == UNKNOWN:
    return MAX_MPH, float(ts[sel][j]), 0.0, UNKNOWN
  keff = math.copysign(float(la[j]) / (v * v), kj)
  return int((need_hi if hi else need_mph)(keff)[0]), float(ts[sel][j]), math.copysign(float(la[j]), kj), side
