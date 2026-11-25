"""On-car A/B alternation (ti_ab.py): while the owner drives normally, ONE controller tuning setting alternates between two
values, A and B, in blocks of a few minutes, and every sample can afterwards be assigned to A or B exactly. OFF unless
the A/B toggle (param TiAbOn, tmpfs, set from the A/B page next to the Tests pages) is on and a valid configuration
(param TiAbConfig) exists. A reboot, or a controls restart, always starts with it off.

What it is NOT. It never writes the owner's persistent setting: value A is, by construction, his own stored value as the
controller reads it this cycle (the same number the car would use with the mode absent). B, and every blend between A
and B, exists only in memory, in a view of the toggles handed to the lateral controller (AbToggles). It changes no
torque authority, limit or panda/CAN item: the allowlist (SETTINGS) holds only gains, filters and compensation terms
that the lateral controller reads live every cycle and that enter its command before the unchanged clips, rate limiter
and interceptor bounds. Everything else of the 15 tuning settings is excluded, with the reason, in EXCLUDED.

When it acts (`TiAb.update`, 100 Hz, from controlsd). Each cycle it takes the road and driver state and is in one of:
  off      the toggle is off, the configuration is refused, tests are on, the controller is not the plant branch, or an
           error disabled it for the rest of the drive. The value in force is A (the owner's).
  idle     on, but not engaged / a fault / the speed is outside 19-80 mph. Value A.
  running  engaged: alternating in blocks, or pinned.
Hard releases (return to A) are: toggle off, tests on (dither, sweep, ladder, pulses, run all), configuration refused or
changed, not engaged (immediate: the controller is reset then), a fault or TI not in RUN, bad inputs, speed below 19 or
from 80 mph (hysteresis 1 mph), the lateral controller is not the plant branch. Except when not engaged, or after an
error (immediate), the return is a ramp (RAMP, see below) so the steering request has no step.

Blocks. Block k has arm A or B: pair p = k // 2 is (A, B) or (B, A) by the seeded draw `first_of_pair(seed, p)` (a hash of
"seed:p", recomputable from the log without our random module), so every pair holds one block of each and time-of-drive
trends cancel. A block is block_s long (default 4 min, 2-10 min) counted in ENGAGED running time. A block that ends early
for any reason except its own end and scored under MIN_SCORED_KEEP_S (scored seconds: wash-in and ramps excluded) is voided: the same k
is run again; at or above it the scored seconds are kept (partial blocks count by their time in force). A change of value
at a block start waits until it is safe (gates below, held for SETTLE_S); after MAX_WAIT_S (60 s) the switch is skipped,
the value stays, and the next block is tried. Pinning A or B from the screen stops alternation until released; a pin
ramps in at once (no gate except the hard ones above).

Switch gates (all must hold, continuously for SETTLE_S, closing immediately and reopening with hysteresis): finite
inputs; no driver touch (pressed flag, >= 6 counts, or >= 4 counts for 3 frames) and 3 s since the last touch, blinker or
lane change; 20-79 mph and not within 1 mph of the 28-31 mph EPS region; straight (|curvature| <= 0.002 1/m, lateral demand
v^2 |curvature| <= 0.4 m/s^2, closing at 0.0025 and 0.5); the steering request |steer| <= 0.25 and not saturated.

Continuity. The effective value is A + s (B - A) with s in [0, 1] moved linearly over RAMP frames (per setting; zero
where the controller itself blends the change: LatCommitSetpoint, LatOutputFilter). A float setting (SteerKP) takes the
blended number; a boolean setting takes weight w = (1 - s) A + s B, honoured by latcontrol_torque.py (damping, friction
compensation and the friction relay scale their term by w), so the term fades in or out rather than appearing.

Log (cloudlog event "ti_ab", JSON, format in TI_AB.md): config, block_start, block_end, wait_start, wait_end, skip,
switch (every change of value: from, to, ramp frames, wash-in frames), pin, unpin, release, hb (10 s), refuse, error.
`reconstruct(events, n)` rebuilds, frame by frame, the blend s, the arm in force and the wash-in flag from the events
alone (exactly; test_mazda_ti_ab.py checks it against the live module), and `verify_schedule` recomputes every block's arm
from the seed.
"""
# RUNTIME: runs in controlsd at 100 Hz; gc disabled (see docs/RUNTIME_CONSTRAINTS.md).
# Per call: create no class or closure, keep no container without a bound, nothing that grows with drive time.
import hashlib
import json
import math
import os
import time
from typing import NamedTuple, Optional

from openpilot.selfdrive.car.mazda import ti_ident as ti
from openpilot.selfdrive.car.mazda import ti_plan
from openpilot.selfdrive.car.mazda.ti_dither import (DRIVER_FIRM_COUNTS, DRIVER_QUIET_COUNTS, HOLDOFF_S,
                                                     TOUCH_CONFIRM_FRAMES)

VERSION = 1
DT = 0.01
MS_TO_MPH = 1.0 / 0.44704
EVENT = "ti_ab"

PARAM_ON, PARAM_PIN, PARAM_CONFIG, PARAM_STATUS = "TiAbOn", "TiAbPin", "TiAbConfig", "TiAbStatus"
TEST_TOGGLES = ("ti_dither", "ti_sweep", "ti_ladder", "ti_pulses", "ti_ident_all")   # frogpilot_toggles names


class Spec(NamedTuple):
  name: str          # the param (one of the 15 tuning settings)
  attr: str          # the frogpilot_toggles attribute the controller reads
  kind: str          # "float" or "bool"
  lo: float          # hard range (a bool: 0, 1)
  hi: float
  max_step: float    # largest |A - B| (a bool: 1)
  step: float        # the screen's increment for B
  ramp_s: float      # blend time of a change (0: the controller blends it itself)
  wash_in_s: float   # after a change the samples are marked wash-in for this long (ramp + settling)
  weighted: bool     # the controller honours a blend weight (ab_weights) for this setting
  why: str


# First-trial limits (named so that they can be widened later on evidence, never by the screen)
KP_LO, KP_HI, KP_MAX_STEP = 0.70, 1.00, 0.10
# After the ramp ends the integrator (time constant about kp/ki, around 4 s) must re-converge for a setting that changes the
# steady error (gain, friction compensation, relay): wash-in is the ramp plus this; the others settle in a few seconds
WASH_AFTER_INTEGRATOR_S = 10.0
# Alternation is active only between these speeds (below about 28 mph the interceptor steers alone with little authority)
ALT_MIN_MPH, ALT_MAX_MPH = 35.0, 70.0
# A setting that can act only above this lateral demand (m/s^2): minutes are scored only there
ACTS_ABOVE_LAT = {"LatCommitSetpoint": 1.0}

# ---- the allowlist -----------------------------------------------------------------------------------------------
# Each is read LIVE every cycle by the lateral controller (latcontrol_torque.py, plant branch; controlsd for SteerKP),
# shapes only the command before the unchanged clip / rate limiter / interceptor bounds, and needs no recalibration.
SETTINGS = {
  "SteerKP": Spec("SteerKP", "steerKp", "float", KP_LO, KP_HI, KP_MAX_STEP, 0.05, 2.0, 2.0 + WASH_AFTER_INTEGRATOR_S, False,
                  "proportional gain, written to the PID every cycle; a step in it is a step in the output only through "
                  "the current error, so it is ramped over 2 s"),
  "LatCommitSetpoint": Spec("LatCommitSetpoint", "lat_commit_setpoint", "bool", 0, 1, 1, 1, 0.0, 6.0, False,
                            "reference filter, blended in by the controller over 0.5 s with warm state"),
  "LatDamping": Spec("LatDamping", "lat_damping", "bool", 0, 1, 1, 1, 1.0, 6.0, True,
                     "rate-feedback term, scaled by the blend weight so it fades in or out over 1 s"),
  "LatFrictionComp": Spec("LatFrictionComp", "lat_friction_comp", "bool", 0, 1, 1, 1, 1.0, 1.0 + WASH_AFTER_INTEGRATOR_S, True,
                          "friction compensation, scaled by the blend weight over 1 s (zero on straights anyway)"),
  "LatOutputFilter": Spec("LatOutputFilter", "lat_output_filter", "bool", 0, 1, 1, 1, 0.0, 4.5, False,
                          "output smoothing, blended by the controller over 0.25 s with the filter tracking the command"),
  "LatNoFrictionRelay": Spec("LatNoFrictionRelay", "lat_no_friction_relay", "bool", 0, 1, 1, 1, 1.0, 1.0 + WASH_AFTER_INTEGRATOR_S, True,
                             "breakaway relay (value True = relay off), scaled by the blend weight over 1 s"),
}
ALLOWED = tuple(SETTINGS)

EXCLUDED = {
  "TiSteerMax": "torque authority: the interceptor's request ceiling (and the hardware clamp is near 600)",
  "TiSteerDeltaUp": "rate limit of the interceptor request: a limit, and the stuck-request fault depends on it",
  "TiSteerDeltaDown": "rate limit of the interceptor request: a limit",
  "TiSteerDeltaUpKnee": "rate limit knee: a limit",
  "TiSteerDeltaUpHigh": "rate limit above the knee: a limit",
  "TiSteerDriverAllowance": "how far the driver's torque may fight the request: a safety limit",
  "TiSteerDriverMultiplier": "how fast the request backs off under driver torque: a safety limit",
  "TiSteerThreshold": "decides when the driver counts as overriding (carstate steeringPressed): a safety input, and this "
                      "mode's own gates read it",
  "ForceAutoTune": "chooses where the torque parameters come from: needs the learned parameters to settle again",
}
assert set(SETTINGS) | set(EXCLUDED) == {
  "SteerKP", "LatCommitSetpoint", "LatDamping", "LatFrictionComp", "LatOutputFilter", "LatNoFrictionRelay", "TiSteerMax",
  "TiSteerDeltaUp", "TiSteerDeltaDown", "TiSteerDriverAllowance", "TiSteerDriverMultiplier", "TiSteerDeltaUpKnee",
  "TiSteerDeltaUpHigh", "TiSteerThreshold", "ForceAutoTune"}

# ---- blocks, waiting, gates ---------------------------------------------------------------------------------------
DEFAULT_BLOCK_MIN, MIN_BLOCK_MIN, MAX_BLOCK_MIN = 4.0, 4.0, 10.0
MAX_WAIT_S = 60.0           # a pending switch waits this long, then is skipped
SETTLE_S = 1.0              # every gate open this long before a switch
BALANCE_MARGIN_S = 30.0     # the next block is the side that is behind when the two sides' scored time differs by more than this
MIN_SCORED_KEEP_S = 30.0    # a block cut short with less SCORED time than this is voided and run again; with this much or more its scored time is kept
MARGIN_MPH = 3.0            # a block starts only when the speed is this far inside either edge of the range ...
MARGIN_HOLD_S = 3.0         # ... and has been for this long (a block started at the edge ends within seconds, for speed)
# Saturation: while B is in force (or being blended in) and the steering request sits at its limit (|steer| >= SAT_STEER of the interceptor
# maximum, about 580 of 600 counts) for SAT_RELEASE_S in a row, the value returns to A by the normal ramp and the block ends (voided unless it
# scored MIN_SCORED_KEEP_S). Only ever a return to the owner's value; nothing here lets anything run that did not.
SAT_STEER = 0.97
SAT_RELEASE_S = 0.5
HB_S = 10.0
H = ti.ZONE_HYSTERESIS_MPH
SPEED_OFF_LOW, SPEED_ON_LOW = ALT_MIN_MPH, ALT_MIN_MPH + H                   # released below 35, ready from 36 mph
SPEED_OFF_HIGH, SPEED_ON_HIGH = ALT_MAX_MPH, ALT_MAX_MPH - H                 # released from 70, ready below 69 mph
EPS_KEEPOUT = (ti.EPS_LOW_MPH - H, ti.EPS_HIGH_MPH + H)                      # no switch from 27 up to 32 mph
# (open, close) of each Schmitt gate
GATES = {"curve": (0.002, 0.0025),       # 1/m, desired curvature
         "lat": (0.4, 0.5),              # m/s^2, v^2 |curvature|
         "request": (0.25, 0.30)}        # |actuators.steer| (share of the TI maximum), the last cycle's
WINDOW_LABELS = tuple(w[2] for w in ti.WINDOWS) + ("gap",)
PRIORITY = ("input", "touch", "lane_change", "holdoff", "speed", "eps", "margin", "curve", "lat", "request", "saturated", "settle")
MESSAGE = {"input": "bad inputs", "touch": "hands on the wheel", "lane_change": "lane change", "holdoff": "after a touch or blinker",
           "speed": "speed outside 36-69 mph", "eps": "near 28-31 mph", "margin": "speed not yet well inside the range", "curve": "road bends", "lat": "lateral demand",
           "request": "steering request large", "saturated": "steering saturated", "settle": "settling"}


def _table_hash():
  spec = [[tuple(s) for s in SETTINGS.values()], DEFAULT_BLOCK_MIN, MIN_BLOCK_MIN, MAX_BLOCK_MIN, MAX_WAIT_S, SETTLE_S,
          MIN_SCORED_KEEP_S, BALANCE_MARGIN_S, MARGIN_MPH, MARGIN_HOLD_S, SAT_STEER, SAT_RELEASE_S, GATES, EPS_KEEPOUT, ACTS_ABOVE_LAT, WASH_AFTER_INTEGRATOR_S, [SPEED_OFF_LOW, SPEED_ON_LOW, SPEED_OFF_HIGH, SPEED_ON_HIGH], VERSION]
  return hashlib.sha1(json.dumps(spec, sort_keys=True).encode()).hexdigest()[:12]


TABLE_HASH = _table_hash()

# ---- progress that survives an ignition cycle ---------------------------------------------------------------------------
# The scored seconds per side (and the engaged seconds per window) of the experiment now set up, kept in a persistent param so a
# fuel stop does not lose them. Keyed by the experiment (the generator/table hash, the setting, the owner's value A, value B): a
# different experiment starts from zero and the old record is cleared; the A/B set-up page's RESET PROGRESS removes the param.
# Reading it back changes no gate and cannot start anything: it only fills the counters of an activation that is already allowed.
PARAM_PROGRESS = "TiAbProgress"
MAX_STORED_S = 3600.0 * 100


def progress_key(cfg):
  return [TABLE_HASH, cfg.setting, round(float(cfg.a), 4), round(float(cfg.b), 4)]


def _stored_seconds(x):
  if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x < 0.0 or x > MAX_STORED_S:
    raise ValueError("bad stored seconds")
  return float(x)


def parse_progress(raw):
  """The stored record as {"key", "committed", "progress"}, or None when absent or in any way unreadable (then zero is used)."""
  try:
    if isinstance(raw, (bytes, bytearray)):
      raw = raw.decode("utf-8")
    if not raw:
      return None
    d = json.loads(raw)
    if d.get("v") != 1 or not isinstance(d.get("key"), list):
      return None
    committed = {a: _stored_seconds(d["committed"][a]) for a in "AB"}
    progress = {a: {w: _stored_seconds(d["progress"][a].get(w, 0.0)) for w in WINDOW_LABELS} for a in "AB"}
    return {"key": d["key"], "committed": committed, "progress": progress}
  except Exception:
    return None


# ---- schedule -------------------------------------------------------------------------------------------------------
def _uniform(seed, draw):
  """The same hash draw as ti_ident._uniform: a reader recomputes it without our random module."""
  h = hashlib.sha256(f"{seed}:{draw}".encode()).digest()
  return int.from_bytes(h[:6], "big") / float(1 << 48)


def first_of_pair(seed, pair):
  return "A" if _uniform(seed, pair) < 0.5 else "B"


def arm_for(seed, idx):
  """The arm of block `idx`: pair idx // 2 is (A, B) or (B, A)."""
  first = first_of_pair(seed, idx // 2)
  other = "B" if first == "A" else "A"
  return first if idx % 2 == 0 else other


# ---- configuration --------------------------------------------------------------------------------------------------
class Config(NamedTuple):
  setting: str
  a: object          # the owner's stored value when validated
  b: object
  block_s: float
  seed: Optional[int]   # None: drawn at activation


def _is_num(x):
  return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _same(x, y):
  if isinstance(x, bool) or isinstance(y, bool):
    return x is y or (isinstance(x, bool) and isinstance(y, bool) and x == y)
  return abs(float(x) - float(y)) <= 1e-9


def validate(raw, stored):
  """(Config, "") or (None, message). `raw` is the TiAbConfig param (JSON text, bytes or a dict); `stored` maps a setting's
  name to the owner's stored value (a float; a bool; None: unreadable). The message is meant for the screen."""
  if raw is None or raw == "" or raw == b"":
    return None, "A/B is not configured: choose a setting and a value B"
  try:
    obj = json.loads(raw.decode("utf-8") if isinstance(raw, (bytes, bytearray)) else raw) if not isinstance(raw, dict) else raw
  except Exception:
    return None, "A/B configuration unreadable: set it again"
  if not isinstance(obj, dict):
    return None, "A/B configuration unreadable: set it again"
  name = obj.get("setting")
  spec = SETTINGS.get(name) if isinstance(name, str) else None
  if spec is None:
    why = EXCLUDED.get(name) if isinstance(name, str) else None
    return None, (f"{name} is not allowed: {why}" if why else f"{name} is not an A/B setting (allowed: {', '.join(ALLOWED)})")
  a = stored.get(name) if isinstance(stored, dict) else None
  ok_a = isinstance(a, bool) if spec.kind == "bool" else _is_num(a)
  if not ok_a:
    return None, f"your stored {name} is unreadable"
  if spec.kind == "float" and not spec.lo <= a <= spec.hi:
    return None, f"your stored {name} {a:g} is outside the A/B range {spec.lo:g}-{spec.hi:g}"
  b = obj.get("b")
  if spec.kind == "bool":
    if not isinstance(b, bool):
      return None, f"value B for {name} must be on or off"
    if b is a:
      return None, f"B is the same as your stored {name} ({'on' if a else 'off'})"
  else:
    if not _is_num(b):
      return None, f"value B for {name} must be a number"
    if not spec.lo <= b <= spec.hi:
      return None, f"B {b:g} is outside the {name} range {spec.lo:g}-{spec.hi:g}"
    if abs(b - a) > spec.max_step + 1e-9:
      return None, f"A to B step {abs(b - a):.3g} exceeds the {name} limit {spec.max_step:g}"
    if abs(b - a) < spec.step / 2:
      return None, f"B is the same as your stored {name} ({a:g})"
  if "a" in obj and obj["a"] is not None and not _same(obj["a"], a):
    return None, f"A is always your stored value ({a if spec.kind == 'bool' else format(a, 'g')}): clear A or set it to that"
  minutes = obj.get("block_min", DEFAULT_BLOCK_MIN)
  if not _is_num(minutes) or not MIN_BLOCK_MIN <= minutes <= MAX_BLOCK_MIN:
    return None, f"block length must be {MIN_BLOCK_MIN:g}-{MAX_BLOCK_MIN:g} minutes"
  seed = obj.get("seed")
  if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 1 << 32):
    return None, "seed must be a whole number below 4294967296"
  return Config(name, a, b, float(minutes) * 60.0, seed), ""


def stored_value(spec, toggles):
  """The owner's value of a setting as the controller reads it now (None when unreadable)."""
  try:
    if spec.kind == "bool":
      v = getattr(toggles, spec.attr, False)
      return v if isinstance(v, bool) else None
    kp = toggles.steerKp
    if len(kp) == 2 and len(kp[0]) == 1 and len(kp[1]) == 1 and _is_num(kp[1][0]):
      return float(kp[1][0])
  except Exception:
    pass
  return None


# ---- the blend ------------------------------------------------------------------------------------------------------
class Ramp:
  """s in [0, 1] moved linearly. Frame order (shared with `reconstruct`): `advance` first, then a decision of the same
  frame calls `start`, which leaves s where it is (frames > 0) or sets it (0 frames)."""

  def __init__(self):
    self.s = 0.0
    self.to = 0.0
    self.frm = 0.0
    self.k = 0
    self.n = 0

  def advance(self):
    if self.k < self.n:
      self.k += 1
      self.s = self.to if self.k >= self.n else self.frm + (self.to - self.frm) * self.k / self.n
    return self.s

  def start(self, to, frames):
    self.to = float(to)
    if frames <= 0:
      self.s, self.frm, self.k, self.n = self.to, self.to, 0, 0
    else:
      self.frm, self.k, self.n = self.s, 0, int(frames)

  @property
  def moving(self):
    return self.k < self.n


class AbToggles:
  """The toggles as the lateral controller sees them while a blend is in force: every attribute of the owner's toggles,
  with the A/B setting's attribute replaced and, for a weighted boolean, `ab_weights`. Never written back."""
  __slots__ = ("_base", "_over")

  def __init__(self, base, over):
    object.__setattr__(self, "_base", base)
    object.__setattr__(self, "_over", over)

  def __getattr__(self, name):
    over = object.__getattribute__(self, "_over")
    if name in over:
      return over[name]
    return getattr(object.__getattribute__(self, "_base"), name)

  def __setattr__(self, name, value):
    raise AttributeError("the A/B view of the toggles is read-only")


class Out(NamedTuple):
  active: bool            # a value other than A is in force or ramping: the controller must be given `overrides`
  s: float
  setting: Optional[str]
  value: object           # the effective value (float, or bool)
  weight: Optional[float]  # a weighted boolean's blend weight toward "on"
  arm: Optional[str]      # the arm in force, "A" or "B" (None: the mode is off)
  wash_in: bool


class TiAb:
  def __init__(self, log=None, block_s=None):
    self.log = log
    self.block_s_override = None if block_s is None else float(block_s)   # (unit tests only: shorter blocks)
    self.block_len = 0.0
    self.speed_off_high, self.speed_on_high = SPEED_OFF_HIGH, SPEED_ON_HIGH
    self.target_s = None         # drive plan: scored seconds per side that complete the alternation (None: never completes)
    self.stored_raw = None       # the persistent progress record as last read (TiAbRunner reads it); None: absent
    self.save_fn = None          # writes (str) or clears (None) the persistent record; None in unit tests
    self._restored = None        # (key, stored_raw) the counters were last filled from
    self.cfg_valid = False
    self.last_release = None
    self._refused = None
    self.events = []
    self.n = 0
    self.ramp = Ramp()
    self.dead = False
    self.reason_dead = ""
    self.seen_off = False        # the toggle has been seen off since this process started (no start from a stale on)
    self.cfg_key = None          # (cfg tuple minus a) of the live activation; None: not activated
    self.cfg = None
    self.spec = None
    self.seed = 0
    self.message = ""
    self.restart_msg = "the A/B switch was on when controls started: switch it off and on again"
    self.mode = "off"            # off / idle / running / pinned
    self._cache = None
    self._reset_activation()
    # driver / road gate state
    self.loud = 0
    self.touch_age = 10 ** 6
    self.blink_age = 10 ** 6
    self.gate_closed = {k: False for k in GATES}
    self.ok_frames = 0
    self.margin_frames = 0       # consecutive cycles with the speed well inside the range (MARGIN_MPH from either edge)
    self.sat_frames = 0          # consecutive cycles with the steering request at its limit
    self.speed_ok = False
    self.last_reason = None
    self.mph = 0.0
    self.window = "gap"
    self.mono_ns = None

  # -- bookkeeping ---------------------------------------------------------------------------------------------------
  def _reset_activation(self):
    self.idx = 0                 # the block to run next / running
    self.block = None            # (idx, arm) while a block runs
    self.block_frames = 0
    self.block_switched = False
    self.pending_since = None    # frame the current pending block started waiting
    self.wait_hist = {}
    self.wait_logged = False
    self.pin = None
    self.pin_wait = None
    self.pin_wait_logged = None
    self.reversed_hold = False
    self.pin_error = False
    self.wash_left = 0
    self.progress = {arm: {w: 0.0 for w in WINDOW_LABELS} for arm in "AB"}
    self.engaged_frames = 0
    self.hb_at = 0
    self.skipped = 0
    self.switches = 0
    self.voided_idx = None
    self.committed = {"A": 0.0, "B": 0.0}   # scored seconds of non-voided blocks, per arm
    self.block_scored = 0                   # scored frames of the running block
    self.complete = False

  def _event(self, kind, **kw):
    e = {"frame": self.n, "kind": kind, **kw}
    if self.mono_ns is not None:
      e["mono_ns"] = int(self.mono_ns)
    self.events.append(e)
    if len(self.events) > 4000:
      del self.events[:2000]
    if self.log is not None:
      try:
        self.log(e)
      except Exception:
        pass

  @property
  def arm_in_force(self):
    return "A" if self.ramp.to == 0.0 else "B"

  def effective(self):
    """(value, weight) at the present blend."""
    s = self.ramp.s
    if self.spec is None or self.cfg is None:
      return None, None
    a, b = self.cfg.a, self.cfg.b
    if self.spec.kind == "bool":
      w = (1.0 - s) * float(a) + s * float(b)
      return (a if s == 0.0 else (b if s == 1.0 else (w >= 0.5))), w
    return (a if s == 0.0 else (b if s == 1.0 else a + s * (b - a))), None

  def _switch(self, to, why, immediate=False):
    """Change the value in force: the blend moves to `to` (0: A, 1: B)."""
    if float(to) == self.ramp.to and not (immediate and (self.ramp.moving or self.ramp.s != float(to))):
      return
    spec = self.spec
    frames = 0 if immediate else int(round(spec.ramp_s / DT * abs(float(to) - self.ramp.s)))
    s_from = self.ramp.s
    self.ramp.start(to, frames)
    self.wash_left = 0 if (immediate and float(to) == 0.0) else int(round(spec.wash_in_s / DT))
    self.switches += 1
    self._event("switch", setting=spec.name, why=why, s_from=s_from, s_to=float(to), ramp_frames=frames,
                wash_frames=self.wash_left, a=self.cfg.a, b=self.cfg.b, arm="A" if to == 0 else "B", immediate=bool(immediate))

  def _restore(self, cfg, force=False):
    """Fill the counters from the stored record when it is for this experiment, else zero them (and clear a record of another
    experiment). Completion is judged on what is restored. Never raises."""
    try:
      key = progress_key(cfg)
      if not force and self._restored == (key, self.stored_raw):
        return
      doc = parse_progress(self.stored_raw)
      self.committed = {"A": 0.0, "B": 0.0}
      self.progress = {arm: {w: 0.0 for w in WINDOW_LABELS} for arm in "AB"}
      self.complete = False
      if doc is not None and doc["key"] == key:
        self.committed, self.progress = doc["committed"], doc["progress"]
      elif doc is not None and self.save_fn is not None:
        self.save_fn(None)                         # the experiment changed: the old record goes
        self.stored_raw = None
      self._restored = (key, self.stored_raw)
      if (self.target_s is not None and self.committed["A"] >= self.target_s and self.committed["B"] >= self.target_s):
        self.complete = True
        self._event("complete", committed={a: round(v, 1) for a, v in self.committed.items()}, target_s=self.target_s, restored=True)
    except Exception:
      self.committed = {"A": 0.0, "B": 0.0}
      self.progress = {arm: {w: 0.0 for w in WINDOW_LABELS} for arm in "AB"}
      self.complete = False

  def _save(self):
    if self.save_fn is None or self.cfg is None:
      return
    try:
      raw = json.dumps({"v": 1, "key": progress_key(self.cfg), "setting": self.cfg.setting, "a": round(float(self.cfg.a), 4),
                        "b": round(float(self.cfg.b), 4), "committed": {a: round(v, 2) for a, v in self.committed.items()},
                        "progress": {a: {w: round(t, 2) for w, t in d.items()} for a, d in self.progress.items()}})
      self.save_fn(raw)
      self.stored_raw = raw
      self._restored = (progress_key(self.cfg), raw)
    except Exception:
      pass

  def _end_block(self, reason):
    if self.block is None:
      return
    idx, arm = self.block
    engaged_s = round(self.block_frames * DT, 2)
    scored_s = round(self.block_scored * DT, 2)
    voided = reason != "done" and scored_s < MIN_SCORED_KEEP_S
    self._event("block_end", idx=idx, arm=arm, engaged_s=engaged_s, scored_s=scored_s, reason=reason, voided=voided)
    if not voided:
      self.idx = idx + 1
      self.committed[arm] += self.block_scored * DT
    else:
      self.voided_idx = idx
    self.block, self.block_frames, self.block_scored = None, 0, 0
    if (self.target_s is not None and not self.complete and self.committed["A"] >= self.target_s
        and self.committed["B"] >= self.target_s):
      self.complete = True
      self._event("complete", committed={a: round(v, 1) for a, v in self.committed.items()}, target_s=self.target_s)
    if not voided:
      self._save()
    self.pending_since, self.wait_hist, self.wait_logged = None, {}, False

  def _end_wait(self, outcome):
    if self.pending_since is not None and self.wait_logged:
      self._event("wait_end", idx=self.idx, outcome=outcome, waited_s=round((self.n - self.pending_since) * DT, 2),
                  reasons={k: round(v * DT, 2) for k, v in self.wait_hist.items()})
    self.pending_since, self.wait_hist, self.wait_logged = None, {}, False

  def _release(self, why, immediate):
    """Back to the owner's value A (ramped unless `immediate`). Idempotent."""
    if self.block is not None:
      self._end_block(why)
    if self.pending_since is not None:
      self._end_wait(f"release:{why}")
    if self.ramp.to != 0.0 or self.ramp.s != 0.0:
      self._switch(0.0, f"release:{why}", immediate=immediate)
    elif self.last_release != why:
      self._event("release", why=why)
    self.last_release = why

  def _fail(self, exc):
    """Any exception: the value returns to A at once and the mode is off for the rest of this process."""
    self.dead = True
    self.reason_dead = f"{type(exc).__name__}: {exc}"[:120]
    self.ramp.start(0.0, 0)
    self.wash_left = 0
    self.mode = "off"
    self.block = None
    self.pin = None
    self._event("error", why=self.reason_dead)

  # -- configuration -------------------------------------------------------------------------------------------------
  def _config(self, raw, stored_all):
    """(Config, message) with a cache on (raw, the stored value of the configured setting)."""
    name = None
    try:
      name = (raw if isinstance(raw, dict) else json.loads(raw.decode("utf-8") if isinstance(raw, (bytes, bytearray)) else raw)).get("setting")
    except Exception:
      pass
    key = (raw if not isinstance(raw, dict) else json.dumps(raw, sort_keys=True), None if name not in stored_all else stored_all[name])
    if self._cache is not None and self._cache[0] == key:
      return self._cache[1]
    res = validate(raw, stored_all)
    self._cache = (key, res)
    return res

  # -- the cycle -----------------------------------------------------------------------------------------------------
  def update(self, **kw):
    """One control cycle; see `_update` for the inputs. Never raises."""
    try:
      return self._update(**kw)
    except Exception as exc:
      try:
        self._fail(exc)
      except Exception:
        self.dead = True
        self.ramp.start(0.0, 0)
      return Out(False, 0.0, None, None, None, None, False)

  def _update(self, *, on, pin, config, stored, engaged, v_ego, curvature, steer, saturated, steering_pressed, driver_torque,
              lane_change, blinker, ti_ok, fault, controller_ok, tests_active, mono_ns=None):
    """`stored`: {setting name: the owner's value now}. `pin`: None, "A", "B" or anything else (an unreadable pin: read as A)."""
    self.n += 1
    self.mono_ns = mono_ns
    if self.dead:
      return Out(False, 0.0, None, None, None, None, False)
    self.ramp.advance()
    if self.wash_left > 0:
      self.wash_left -= 1
    if not on:
      self.seen_off = True

    finite = (isinstance(v_ego, (int, float)) and math.isfinite(v_ego) and isinstance(curvature, (int, float)) and math.isfinite(curvature)
              and isinstance(steer, (int, float)) and math.isfinite(steer) and isinstance(driver_torque, (int, float))
              and math.isfinite(driver_torque))
    self.sat_frames = self.sat_frames + 1 if (finite and abs(steer) >= SAT_STEER) else 0
    mag = abs(driver_torque) if finite else 0.0
    self.loud = self.loud + 1 if mag >= DRIVER_QUIET_COUNTS else 0
    touch = bool(steering_pressed) or mag >= DRIVER_FIRM_COUNTS or self.loud >= TOUCH_CONFIRM_FRAMES
    self.touch_age = 0 if touch else self.touch_age + 1
    self.blink_age = 0 if (blinker or lane_change) else self.blink_age + 1
    mph = v_ego * MS_TO_MPH if finite else 0.0
    self.mph = mph
    if finite:
      if mph < SPEED_OFF_LOW or mph >= self.speed_off_high:
        self.speed_ok = False
      elif SPEED_ON_LOW <= mph < self.speed_on_high:
        self.speed_ok = True
    else:
      self.speed_ok = False
    self.window = (ti.zone_for(v_ego).window or "gap") if finite else "gap"

    # ---- is the mode wanted, and valid
    cfg, msg = (None, "")
    if on and self.seen_off:
      cfg, msg = self._config(config, stored)
    self.cfg_valid = cfg is not None
    if cfg is not None and self.cfg_key is None:
      self._restore(cfg)            # a stopped or restarted experiment picks up its stored progress (no effect on whether anything runs)
    hard, immediate = None, False
    if not on:
      hard = "off"
      self.message = ""
    elif not self.seen_off:
      hard = "restart"
      self.message = self.restart_msg
    elif cfg is None:
      hard = "config"
      self.message = msg
      if msg != self._refused:
        self._refused = msg
        self._event("refuse", why=msg)
    else:
      self._refused = None
      key = (cfg.setting, cfg.b, cfg.block_s, cfg.seed)
      if self.cfg_key is not None and key != self.cfg_key:
        hard = "config_changed"
        self.seen_off = False          # a changed configuration needs the switch off and on again
        self.restart_msg = "the A/B configuration changed: switch A/B off and on again"
        self.message = self.restart_msg
      elif self.complete:
        hard = "complete"
        self.message = "A/B complete: your own value A stays for the rest of the drive"
      elif tests_active:
        hard = "tests"
        self.message = "tests are on: A/B stays off while any test is on"
      elif not controller_ok:
        hard = "controller"
        self.message = "A/B needs the standard lateral controller (not NNFF)"
      elif not engaged:
        hard, immediate = "disengaged", True
        self.message = "waiting for engagement"
      elif fault or not ti_ok:
        hard = "fault"
        self.message = "steering fault or interceptor not running"
      elif not finite:
        hard = "input"
        self.message = "bad inputs"
      elif not self.speed_ok:
        hard = "speed"
        self.message = f"speed outside 35-{self.speed_off_high:g} mph"
      elif self.sat_frames >= SAT_RELEASE_S / DT and (self.ramp.to != 0.0 or self.ramp.s != 0.0):
        hard = "saturation"
        self.message = "steering at its limit: back to your own value"
      else:
        self.message = ""

    if hard is not None:
      self.margin_frames = 0            # the speed must be well inside the range again, for MARGIN_HOLD_S, before a block starts
      if self.spec is not None:
        self._release(hard, immediate=immediate)
      if hard in ("off", "config", "config_changed", "restart"):
        self.mode = "off"
        if self.spec is not None and not self.ramp.moving and self.ramp.s == 0.0:
          self.cfg_key, self.cfg, self.spec = None, None, None       # back at A and finished: forget the activation
      else:
        self.mode = "complete" if hard == "complete" else "idle"
      return self._out()
    self.last_release = None

    # ---- valid, engaged, allowed: activate on first use
    if self.cfg_key is None:
      self.spec = SETTINGS[cfg.setting]
      self.seed = cfg.seed if cfg.seed is not None else int.from_bytes(os.urandom(4), "big")
      self.cfg = cfg
      self.cfg_key = (cfg.setting, cfg.b, cfg.block_s, cfg.seed)
      self.block_len = self.cfg.block_s if self.block_s_override is None else self.block_s_override
      self._reset_activation()
      self._restore(cfg, force=True)
      self.ramp = Ramp()
      self._event("config", version=VERSION, table=TABLE_HASH, setting=cfg.setting, a=cfg.a, b=cfg.b,
                  block_s=self.block_len, seed=self.seed, seed_given=cfg.seed is not None,
                  ramp_s=self.spec.ramp_s, wash_in_s=self.spec.wash_in_s, weighted=self.spec.weighted)
    else:
      self.cfg = cfg                # A is the owner's stored value of THIS cycle
    self.engaged_frames += 1

    # ---- switch gates
    reason = self._gate(finite, touch, lane_change or blinker, steer, saturated, curvature, v_ego, mph)
    self.last_reason = reason

    # ---- a touch, blinker or lane change while the value is moving toward B: back to A (a ramp toward A goes on)
    if (touch or lane_change or blinker) and self.ramp.moving and self.ramp.to == 1.0:
      self._end_block("reversed")
      self._switch(0.0, "reverse:touch" if touch else "reverse:blinker")
      self.reversed_hold = True

    # ---- pin: anything but a clear "A" or "B" that is not empty is an unreadable pin, read as A
    pin_error = pin not in (None, "", b"", "A", "B")
    want_pin = pin if pin in ("A", "B") else ("A" if pin_error else None)
    if pin_error and not self.pin_error:
      self._event("refuse", why="the pin is unreadable: held at A")
    self.pin_error = pin_error
    if pin_error:
      self.message = "the pin is unreadable: held at A"
    if want_pin != self.pin:
      if want_pin is not None:
        self._end_block("pinned")
        if self.pending_since is not None:
          self._end_wait("pinned")
        self._event("pin", arm=want_pin)
      else:
        self._event("unpin", arm=self.pin)
      self.pin = want_pin
    self.pin_wait = None
    if self.pin is not None:
      self.mode = "pinned"
      if self.pin == "A":
        self._switch(0.0, "pin")                    # A: at once, by ramp
      elif self.ramp.to == 1.0:
        pass
      elif reason is None:                          # B: only when the automatic switch gates are open
        self._switch(1.0, "pin")
      else:
        self.pin_wait = reason
        if self.pin_wait_logged != reason:
          self.pin_wait_logged = reason
          self._event("pin_wait", arm="B", why=reason)
    else:
      self.mode = "running"
      self._blocks(reason, self.block_len)

    # ---- scored time
    acts = ACTS_ABOVE_LAT.get(self.spec.name)
    if self.wash_left == 0 and not self.ramp.moving and (acts is None or v_ego * v_ego * abs(curvature) >= acts):
      self.progress[self.arm_in_force][self.window] += DT
      if self.block is not None and self.pin is None:
        self.block_scored += 1
    if self.n - self.hb_at >= HB_S / DT:
      self.hb_at = self.n
      self._event("hb", s=self.ramp.s, arm=self.arm_in_force, wash_left=self.wash_left, idx=self.idx,
                  block=None if self.block is None else self.block[0], pin=self.pin, window=self.window,
                  progress={a: {w: round(t, 2) for w, t in d.items() if t} for a, d in self.progress.items()})
    return self._out()

  def _gate(self, finite, touch, changing, steer, saturated, curvature, v_ego, mph):
    reasons = []
    if not finite:
      reasons.append("input")
    if touch:
      reasons.append("touch")
    if changing:
      reasons.append("lane_change")
    elif self.touch_age < HOLDOFF_S / DT or self.blink_age < HOLDOFF_S / DT:
      reasons.append("holdoff")
    comfy = finite and self.speed_ok and SPEED_OFF_LOW + MARGIN_MPH - 1e-6 <= mph < self.speed_off_high - MARGIN_MPH
    self.margin_frames = self.margin_frames + 1 if comfy else 0
    if not self.speed_ok:
      reasons.append("speed")
    elif EPS_KEEPOUT[0] <= mph < EPS_KEEPOUT[1]:
      reasons.append("eps")
    elif self.margin_frames < MARGIN_HOLD_S / DT:
      reasons.append("margin")
    if finite:
      vals = {"curve": abs(curvature), "lat": v_ego * v_ego * abs(curvature), "request": abs(steer)}
      for g, v in vals.items():
        lo, hi = GATES[g]
        if v > hi:
          self.gate_closed[g] = True
        elif v <= lo:
          self.gate_closed[g] = False
        if self.gate_closed[g]:
          reasons.append(g)
    if saturated:
      reasons.append("saturated")
    first = next((r for r in PRIORITY if r in reasons), None)
    self.ok_frames = self.ok_frames + 1 if first is None else 0
    if first is None and self.ok_frames < SETTLE_S / DT:
      return "settle"
    return first

  def _next_arm(self, idx):
    """(arm, why) of block idx: the seeded pair order ("seed"), except that when one side is behind the other by more than
    BALANCE_MARGIN_S of scored time (partial blocks cut at hand-overs make the sides uneven) the next block is the side that is behind
    ("balance"). The analysis pairs by time in force, so the label of a block does not matter to it; the reason is logged."""
    a, b = self.committed["A"], self.committed["B"]
    if abs(a - b) > BALANCE_MARGIN_S:
      return ("A" if a < b else "B"), "balance"
    return arm_for(self.seed, idx), "seed"

  def _blocks(self, reason, block_s):
    if self.block is None:
      idx = self.idx
      arm, arm_why = self._next_arm(idx)
      target = 0.0 if arm == "A" else 1.0
      if self.pending_since is None:
        self.pending_since = self.n
        self.wait_hist, self.wait_logged = {}, False
      need = target != self.ramp.to
      if need and reason is not None:
        self.wait_hist[reason] = self.wait_hist.get(reason, 0) + 1
        if not self.wait_logged:
          self.wait_logged = True
          self._event("wait_start", idx=idx, arm=arm, why=reason)
        if (self.n - self.pending_since) * DT >= MAX_WAIT_S:
          self._event("skip", idx=idx, arm=arm, waited_s=round((self.n - self.pending_since) * DT, 2),
                      reasons={k: round(v * DT, 2) for k, v in self.wait_hist.items()}, kept=self.arm_in_force)
          self.skipped += 1
          self.idx = idx + 1
          self.pending_since, self.wait_hist, self.wait_logged = None, {}, False
        return
      waited = round((self.n - self.pending_since) * DT, 2)
      if need:
        self._end_wait("switched")
        self._switch(target, "block")
      elif self.wait_logged:
        self._end_wait("not_needed")
      self.block, self.block_frames = (idx, arm), 0
      self.pending_since = None
      self._event("block_start", idx=idx, arm=arm, arm_why=arm_why, switched=bool(need), waited_s=waited, block_s=block_s,
                  first=first_of_pair(self.seed, idx // 2), repeat=self.voided_idx == idx)
      return
    self.block_frames += 1
    if self.block_frames * DT >= block_s:
      self._end_block("done")

  # -- outputs -------------------------------------------------------------------------------------------------------
  def _out(self):
    active = self.spec is not None and (self.ramp.s != 0.0 or self.ramp.moving or self.ramp.to != 0.0)
    value, weight = self.effective()
    arm = self.arm_in_force if self.spec is not None else None
    return Out(bool(active), self.ramp.s, None if self.spec is None else self.spec.name, value, weight, arm,
               self.wash_left > 0)

  def overrides(self, out, toggles):
    """{toggles attribute: value} for the view when `out.active`."""
    spec = SETTINGS[out.setting]
    if spec.kind == "float":
      kp = toggles.steerKp
      return {spec.attr: [list(kp[0]), [out.value]]}
    over = {spec.attr: bool(out.value)}
    if spec.weighted:
      over["ab_weights"] = {spec.attr: out.weight}
    return over

  def set_plan_limits(self, ceiling_mph, target_s):
    """The drive plan's first-trial limits: the alternation's upper speed and the scored seconds per side that complete it."""
    self.speed_off_high, self.speed_on_high = float(ceiling_mph), float(ceiling_mph) - H
    self.target_s = None if target_s is None else float(target_s)

  def clear_plan_limits(self):
    self.speed_off_high, self.speed_on_high, self.target_s = SPEED_OFF_HIGH, SPEED_ON_HIGH, None

  @property
  def at_a_settled(self):
    """The blend is exactly A, not moving, and has been for the wash-in time."""
    return self.ramp.s == 0.0 and not self.ramp.moving and self.ramp.to == 0.0 and self.wash_left == 0

  def status_key(self):
    """(state, msg, arm, pin, block idx, wait reason, complete) exactly as status() holds them, without building it."""
    block_idx = self.block[0] if (self.block is not None and self.cfg is not None) else None
    if self.pin_wait is not None:
      wait_reason = self.pin_wait
    elif self.mode == "running" and self.block is None and self.pending_since is not None:
      wait_reason = self.last_reason or "settle"
    else:
      wait_reason = None
    return ("error" if self.dead else self.mode,
            (f"A/B stopped after an error: {self.reason_dead}" if self.dead else self.message),
            self.arm_in_force if self.spec is not None else None, self.pin, block_idx, wait_reason, self.complete)

  def status(self, now=None):
    """The block the A/B page reads (param TiAbStatus)."""
    block = None
    if self.block is not None and self.cfg is not None:
      length = self.block_len
      block = {"idx": self.block[0], "arm": self.block[1], "len_s": length, "left_s": round(max(length - self.block_frames * DT, 0.0), 1),
               "scored_s": round(self.block_scored * DT, 1)}   # provisional: committed to its side when the block ends (pages show it)
    wait = None
    if self.pin_wait is not None:
      wait = {"reason": self.pin_wait, "why": "pin waiting: " + MESSAGE.get(self.pin_wait, ""), "s": 0.0, "max_s": 0.0, "arm": "B", "pin": True}
    elif self.mode == "running" and self.block is None and self.pending_since is not None:
      wait = {"reason": self.last_reason or "settle", "why": MESSAGE.get(self.last_reason or "settle", ""),
              "s": round((self.n - self.pending_since) * DT, 1), "max_s": MAX_WAIT_S,
              "arm": self._next_arm(self.idx)[0]}
    spec = self.spec
    value, weight = self.effective()
    return {"v": VERSION, "t": round(time.time() if now is None else now, 1), "state": "error" if self.dead else self.mode,
            "msg": (f"A/B stopped after an error: {self.reason_dead}" if self.dead else self.message),
            "setting": None if spec is None else spec.name, "kind": None if spec is None else spec.kind,
            "a": None if self.cfg is None else self.cfg.a, "b": None if self.cfg is None else self.cfg.b,
            "value": value, "arm": self.arm_in_force if spec is not None else None, "blend": round(self.ramp.s, 3),
            "pin": self.pin, "block": block, "wait": wait, "wash_in": self.wash_left > 0, "seed": self.seed if spec else None,
            "idx": self.idx, "skipped": self.skipped, "mph": round(self.mph, 1),
            "scored": {a: round(v, 1) for a, v in self.committed.items()}, "target_s": self.target_s, "complete": self.complete,
            "progress": {a: {w: round(t, 1) for w, t in d.items()} for a, d in self.progress.items()}}


# ---- reconstruction from the log ---------------------------------------------------------------------------------------
def reconstruct(events, n_frames):
  """Per frame 1..n_frames (the module's frame counter): (s, arm in force, wash-in) from the `switch` events alone, with the
  module's own frame order: advance, then the frame's events."""
  by_frame = {}
  for e in events:
    if e.get("kind") == "switch":
      by_frame.setdefault(e["frame"], []).append(e)
  ramp, wash, out = Ramp(), 0, []
  for f in range(1, n_frames + 1):
    ramp.advance()
    if wash > 0:
      wash -= 1
    for e in by_frame.get(f, ()):
      ramp.start(e["s_to"], e["ramp_frames"])
      wash = e["wash_frames"]
    out.append((ramp.s, "A" if ramp.to == 0.0 else "B", wash > 0))
  return out


def verify_schedule(events):
  """Every block_start whose arm differs from the seed's schedule (the list of mismatching events)."""
  seed = None
  bad = []
  for e in events:
    if e.get("kind") == "config":
      seed = e["seed"]
    elif e.get("kind") == "block_start" and seed is not None:
      if e.get("arm_why") == "balance":
        continue                       # chosen to even out the sides' scored time, not by the seed
      if arm_for(seed, e["idx"]) != e["arm"] or first_of_pair(seed, e["idx"] // 2) != e["first"]:
        bad.append(e)
  return bad


# ---- the glue controlsd calls --------------------------------------------------------------------------------------
class TiAbRunner:
  """Reads the params at a low rate, calls the module, publishes the status, and returns the toggles the lateral controller
  is to use: the owner's own object whenever A is in force (so the controller runs exactly as without this mode), else an
  AbToggles view. Never raises; after an error it returns the owner's toggles for the rest of the process."""
  ON_EVERY, CONFIG_EVERY, STATUS_EVERY = 10, 100, 50     # cycles

  def __init__(self, params, params_memory, log=None, block_s=None, plan_target_min=None):
    self.params, self.memory = params, params_memory
    self.ab = TiAb(log=log, block_s=block_s)
    self.ab.save_fn = self._save_progress
    self._written = None
    self._clear_switches()
    self.tests_defs, self.tests_doc, self.tests_behind = None, None, False
    try:
      from openpilot.selfdrive.car.mazda import ti_tests
      self.ti_tests = ti_tests
      self.tests_defs = ti_tests.load_definitions()[0]
    except Exception:
      self.ti_tests = None            # no test definitions here: the tests never get a visit inside the range (the plan as before)
    self.plan = ti_plan.Plan(log=lambda kind, **kw: self.ab._event(kind, **kw))
    self.plan_on, self.plan_ack, self.plan_dead = False, None, False
    self.plan_out = None
    self.plan_target_s = 60.0 * (ti_plan.TARGET_MIN_PER_SIDE if plan_target_min is None else plan_target_min)
    self.plan_published = None
    self.k = 0
    self.on, self.pin, self.config = False, None, None
    self.read_error = ""
    self.published = None
    self.published_k = -10 ** 9

  def _clear_switches(self):
    """Controls (re)started: the plan switch, the alternation switch and its pin are OFF, whatever tmpfs still holds (the comma stays
    powered across a fuel stop, so the tmpfs state of the last drive survives a restart of this process). The screens show the
    switches, so a switch that said ON while nothing could run was the defect; now what they show after a restart is OFF, and
    START works first time. Never raises."""
    for key in (PARAM_ON, ti_plan.PARAM_PLAN_ON):
      try:
        put = getattr(self.memory, "put_bool", None)
        if put is not None:
          put(key, False)
        else:
          self.memory.put(key, False)
      except Exception:
        pass
    try:
      self.memory.remove(PARAM_PIN)
    except Exception:
      pass

  def _read_tests_progress(self):
    """The tests' progress as the accountant publishes it (tmpfs copy first, else the flash copy); None when unreadable."""
    self.tests_doc = None
    if self.ti_tests is None:
      return
    try:
      for store in (self.memory, self.params):
        raw = store.get(self.ti_tests.PROGRESS_KEY)
        if isinstance(raw, (bytes, bytearray)):
          raw = raw.decode("utf-8", "replace")
        doc = self.ti_tests.parse_progress(raw) if raw else None
        if doc is not None:
          self.tests_doc = doc
          return
    except Exception:
      self.tests_doc = None

  VISIT_PATIENCE_S, VISIT_COOLDOWN_S = 45.0, 60.0    # a tests visit that credits nothing for this long is ended; no new one for the cooldown

  def _tests_behind(self, v_ego):
    """Interleaving inside the alternation's range (ti_plan.tests_behind): the need with the lowest share of its requirement done is
    served, so that every need in the range finishes together. The alternation's share counts the block in progress (blocks are
    4 minutes, so only counting them at their end made the alternation look behind for most of the time). A visit that credits nothing
    (the gates are closed, the road is not straight) is ended after VISIT_PATIENCE_S so the alternation is not starved by it.
    False on any doubt."""
    if not ti_plan.INTERLEAVE_TESTS_IN_RANGE:
      self.tests_behind = False        # ti_plan.tests_behind is False whatever the shares: the shares are not computed (cost)
      return False
    try:
      window = ti.zone_for(v_ego).window
      now = self.k * DT
      if self.ti_tests is None or self.tests_defs is None or window not in ti_plan.INTERLEAVE_WINDOWS:
        self.tests_behind = False
        return False
      have, need = self.ti_tests.window_share(self.tests_doc or {}, self.tests_defs, window)
      tests_share = have / need if need > 0 else 1.0
      target = self.ab.target_s
      if not target:
        alt_share = None
      else:
        in_block = self.ab.block_scored * DT if self.ab.block is not None else 0.0
        alt_share = min(1.0, (self.ab.committed["A"] + self.ab.committed["B"] + in_block) / (2.0 * target))
      was = self.tests_behind
      behind = ti_plan.tests_behind(window, tests_share, alt_share, was)
      if behind and not was:
        self._visit = (now, tests_share)
      if behind and was:
        t0, share0 = getattr(self, "_visit", (now, tests_share))
        if now - t0 >= self.VISIT_PATIENCE_S and tests_share <= share0 + 1e-9:
          behind = False
          self._cooldown_until = now + self.VISIT_COOLDOWN_S
        elif tests_share > share0 + 1e-9:
          self._visit = (now, tests_share)             # it is crediting: the patience starts again from here
      if behind and now < getattr(self, "_cooldown_until", -1.0):
        behind = False
      self.tests_behind = behind
      return behind
    except Exception:
      self.tests_behind = False
      return False

  def _save_progress(self, raw):
    """Write the experiment's progress (persistent), or clear it (None)."""
    self._written = (raw, time.time())
    if raw is None:
      self.params.remove(PARAM_PROGRESS)
    else:
      put = getattr(self.params, "put_nonblocking", None) or self.params.put
      put(PARAM_PROGRESS, raw)

  def _read(self):
    try:
      self.on = bool(self.memory.get_bool(PARAM_ON))
      raw_pin = self.memory.get(PARAM_PIN)
      if isinstance(raw_pin, (bytes, bytearray)):
        raw_pin = raw_pin.decode("utf-8", "replace")
      self.pin = None if raw_pin in (None, "") else raw_pin
      self.plan_on = bool(self.memory.get_bool(ti_plan.PARAM_PLAN_ON))
      raw_ack = self.memory.get(ti_plan.PARAM_PLAN_ACK)
      try:
        self.plan_ack = json.loads(raw_ack) if raw_ack else None
      except Exception:
        self.plan_ack = None
      self.read_error = ""
    except Exception as exc:
      self.on, self.pin, self.plan_on = False, None, False
      self.read_error = f"A/B switch unreadable: {type(exc).__name__}"
    if self.k % self.CONFIG_EVERY == 0 or self.config is None:
      self._read_tests_progress()
      try:
        self.config = self.params.get(PARAM_CONFIG)
      except Exception:
        self.config = None
      try:
        raw = self.params.get(PARAM_PROGRESS)
        if isinstance(raw, (bytes, bytearray)):
          raw = raw.decode("utf-8", "replace")
        if self._written is None or raw == self._written[0] or time.time() - self._written[1] > 5.0:
          self.ab.stored_raw = raw         # (a write just made may not have landed yet: until it has, keep what was written)
      except Exception:
        self.ab.stored_raw = None

  def step(self, toggles, **inputs):
    """`inputs`: engaged, v_ego, curvature, steer, saturated, steering_pressed, driver_torque, lane_change, blinker, ti_ok,
    fault, controller_ok, mono_ns."""
    try:
      if self.ab.dead:
        self._publish_plan(dead=self.plan_dead)
        self._publish()
        return toggles
      if self.k % self.ON_EVERY == 0:
        self._read()
      self.k += 1
      if toggles is not getattr(self, "_stored_for", None):
        # the owner's stored values and the test toggles, read again only when controlsd hands over a new toggles object (it
        # replaces it on every reload and never changes it in place; cost: six reads and a scan every cycle)
        specs = [SETTINGS[n] for n in ALLOWED]
        self._stored = {s.name: stored_value(s, toggles) for s in specs}
        self._tests_on = any(getattr(toggles, t, False) is True for t in TEST_TOGGLES)
        self._stored_for = toggles
      stored, tests = self._stored, self._tests_on
      on, paused = self.on, tests
      try:
        v = inputs["v_ego"]
        mph = v * MS_TO_MPH if isinstance(v, (int, float)) and math.isfinite(v) else 0.0
        if self.plan_on:
          self.ab.set_plan_limits(ti_plan.PLAN_ALT_MAX_MPH, self.plan_target_s)
        else:
          self.ab.clear_plan_limits()
        po = self.plan.update(on=self.plan_on, cfg_valid=self.ab.cfg_valid, complete=self.ab.complete, mph=mph,
                              ab_at_a=self.ab.at_a_settled, ack=self.plan_ack, now=time.time(), tests_toggles_on=tests,
                              ceiling=ti_plan.PLAN_ALT_MAX_MPH,
                              tests_behind_here=self.plan_on and self._tests_behind(v))
        self.plan_out = po
        if self.plan_on:
          on, paused = True, tests or not po.ab_allowed
      except Exception as exc:
        # the plan is off and both modes are off for the rest of the process: the controller stays on the owner's settings
        self.plan_dead = True
        self.ab._fail(exc)
        self._publish_plan(dead=True)
        self._publish()
        return toggles
      out = self.ab.update(on=on, pin=self.pin, config=self.config, stored=stored, tests_active=paused, **inputs)
      if self.plan_on and paused and not tests and self.ab.message.startswith("tests are on"):
        self.ab.message = "A/B paused: the drive plan is running " + ("the tests" if po.mode == "tests" else "a hand-over")
      if self.read_error and not self.ab.message:
        self.ab.message = self.read_error
      self._publish_plan()
      self._publish()
      if not out.active:
        return toggles
      return AbToggles(toggles, self.ab.overrides(out, toggles))
    except Exception:
      try:
        self.ab._fail(RuntimeError("runner"))
      except Exception:
        pass
      return toggles

  def _publish_plan(self, dead=False):
    """The plan for the car controller (TiPlan): on every change and every 0.5 s, so a silent controls process is noticed."""
    po = self.plan_out or ti_plan.PlanOut("off", False, "", "", 0)
    at_a = self.ab.at_a_settled
    key = ("dead", at_a) if dead else (po.mode, po.seq, at_a)
    if key != self.plan_published or self.k % 50 == 0:
      try:
        self.memory.put(ti_plan.PARAM_PLAN, ti_plan.encode_plan(po, time.time(), dead, self.plan.nonce, at_a))
        self.plan_published = key
      except Exception:
        pass

  def _publish(self):
    po = self.plan_out
    plan = None if (po is None or not self.plan_on) and not self.plan_dead else {
      "on": self.plan_on, "mode": "dead" if self.plan_dead else po.mode, "why": "" if po is None else po.why,
      "next": "" if po is None else po.next, "target_min": self.plan_target_s / 60.0}
    # the key from the same fields status() would hold (TiAb.status_key), so the full status is built only on the cycles it is
    # published (cost: it was built and thrown away on most cycles)
    state, msg, arm, pin, block_idx, wait_reason, complete = self.ab.status_key()
    key = (state, msg, arm, pin, block_idx, wait_reason, plan and plan["mode"], complete)
    if key != self.published or self.k - self.published_k >= (self.STATUS_EVERY if state != "off" else 4 * self.STATUS_EVERY):
      st = self.ab.status()
      st["plan"] = plan
      try:
        self.memory.put(PARAM_STATUS, json.dumps(st))
        self.published, self.published_k = key, self.k
      except Exception:
        pass
