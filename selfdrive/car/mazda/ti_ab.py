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
for any reason except its own end and lasted under MIN_BLOCK_KEEP_S is voided: the same k is run again. A change of value
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
import hashlib
import json
import math
import os
import time
from typing import NamedTuple, Optional

from openpilot.selfdrive.car.mazda import ti_ident as ti
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


# ---- the allowlist -----------------------------------------------------------------------------------------------
# Each is read LIVE every cycle by the lateral controller (latcontrol_torque.py, plant branch; controlsd for SteerKP),
# shapes only the command before the unchanged clip / rate limiter / interceptor bounds, and needs no recalibration.
SETTINGS = {
  "SteerKP": Spec("SteerKP", "steerKp", "float", 0.50, 1.10, 0.25, 0.05, 2.0, 5.0, False,
                  "proportional gain, written to the PID every cycle; a step in it is a step in the output only through "
                  "the current error, so it is ramped over 2 s"),
  "LatCommitSetpoint": Spec("LatCommitSetpoint", "lat_commit_setpoint", "bool", 0, 1, 1, 1, 0.0, 4.0, False,
                            "reference filter, blended in by the controller over 0.5 s with warm state"),
  "LatDamping": Spec("LatDamping", "lat_damping", "bool", 0, 1, 1, 1, 1.0, 4.0, True,
                     "rate-feedback term, scaled by the blend weight so it fades in or out over 1 s"),
  "LatFrictionComp": Spec("LatFrictionComp", "lat_friction_comp", "bool", 0, 1, 1, 1, 1.0, 4.0, True,
                          "friction compensation, scaled by the blend weight over 1 s (zero on straights anyway)"),
  "LatOutputFilter": Spec("LatOutputFilter", "lat_output_filter", "bool", 0, 1, 1, 1, 0.0, 3.5, False,
                          "output smoothing, blended by the controller over 0.25 s with the filter tracking the command"),
  "LatNoFrictionRelay": Spec("LatNoFrictionRelay", "lat_no_friction_relay", "bool", 0, 1, 1, 1, 1.0, 4.0, True,
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
DEFAULT_BLOCK_MIN, MIN_BLOCK_MIN, MAX_BLOCK_MIN = 4.0, 2.0, 10.0
MAX_WAIT_S = 60.0           # a pending switch waits this long, then is skipped
SETTLE_S = 1.0              # every gate open this long before a switch
MIN_BLOCK_KEEP_S = 60.0     # a block cut short under this is voided and run again
HB_S = 10.0
H = ti.ZONE_HYSTERESIS_MPH
SPEED_OFF_LOW, SPEED_ON_LOW = ti.MIN_TEST_MPH, ti.MIN_TEST_MPH + H           # released below 19, ready from 20 mph
SPEED_OFF_HIGH, SPEED_ON_HIGH = ti.MAX_TEST_MPH, ti.MAX_TEST_MPH - H         # released from 80, ready below 79 mph
EPS_KEEPOUT = (ti.EPS_LOW_MPH - H, ti.EPS_HIGH_MPH + H)                      # no switch from 27 up to 32 mph
# (open, close) of each Schmitt gate
GATES = {"curve": (0.002, 0.0025),       # 1/m, desired curvature
         "lat": (0.4, 0.5),              # m/s^2, v^2 |curvature|
         "request": (0.25, 0.30)}        # |actuators.steer| (share of the TI maximum), the last cycle's
WINDOW_LABELS = tuple(w[2] for w in ti.WINDOWS) + ("gap",)
PRIORITY = ("input", "touch", "lane_change", "holdoff", "speed", "eps", "curve", "lat", "request", "saturated", "settle")
MESSAGE = {"input": "bad inputs", "touch": "hands on the wheel", "lane_change": "lane change", "holdoff": "after a touch or blinker",
           "speed": "speed outside 20-79 mph", "eps": "near 28-31 mph", "curve": "road bends", "lat": "lateral demand",
           "request": "steering request large", "saturated": "steering saturated", "settle": "settling"}


def _table_hash():
  spec = [[tuple(s) for s in SETTINGS.values()], DEFAULT_BLOCK_MIN, MIN_BLOCK_MIN, MAX_BLOCK_MIN, MAX_WAIT_S, SETTLE_S,
          MIN_BLOCK_KEEP_S, GATES, EPS_KEEPOUT, [SPEED_OFF_LOW, SPEED_ON_LOW, SPEED_OFF_HIGH, SPEED_ON_HIGH], VERSION]
  return hashlib.sha1(json.dumps(spec, sort_keys=True).encode()).hexdigest()[:12]


TABLE_HASH = _table_hash()


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
    self.pin_error = False
    self.wash_left = 0
    self.progress = {arm: {w: 0.0 for w in WINDOW_LABELS} for arm in "AB"}
    self.engaged_frames = 0
    self.hb_at = 0
    self.skipped = 0
    self.switches = 0
    self.voided_idx = None

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

  def _end_block(self, reason):
    if self.block is None:
      return
    idx, arm = self.block
    engaged_s = round(self.block_frames * DT, 2)
    voided = reason != "done" and engaged_s < MIN_BLOCK_KEEP_S
    self._event("block_end", idx=idx, arm=arm, engaged_s=engaged_s, reason=reason, voided=voided)
    if not voided:
      self.idx = idx + 1
    else:
      self.voided_idx = idx
    self.block, self.block_frames = None, 0
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

    finite = all(isinstance(x, (int, float)) and math.isfinite(x) for x in (v_ego, curvature, steer, driver_torque))
    mag = abs(driver_torque) if finite else 0.0
    self.loud = self.loud + 1 if mag >= DRIVER_QUIET_COUNTS else 0
    touch = bool(steering_pressed) or mag >= DRIVER_FIRM_COUNTS or self.loud >= TOUCH_CONFIRM_FRAMES
    self.touch_age = 0 if touch else self.touch_age + 1
    self.blink_age = 0 if (blinker or lane_change) else self.blink_age + 1
    mph = v_ego * MS_TO_MPH if finite else 0.0
    self.mph = mph
    if finite:
      if mph < SPEED_OFF_LOW or mph >= SPEED_OFF_HIGH:
        self.speed_ok = False
      elif SPEED_ON_LOW <= mph < SPEED_ON_HIGH:
        self.speed_ok = True
    else:
      self.speed_ok = False
    self.window = (ti.zone_for(v_ego).window or "gap") if finite else "gap"

    # ---- is the mode wanted, and valid
    cfg, msg = (None, "")
    if on and self.seen_off:
      cfg, msg = self._config(config, stored)
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
        self.message = "speed outside 19-80 mph"
      else:
        self.message = ""

    if hard is not None:
      if self.spec is not None:
        self._release(hard, immediate=immediate)
      if hard in ("off", "config", "config_changed", "restart"):
        self.mode = "off"
        if self.spec is not None and not self.ramp.moving and self.ramp.s == 0.0:
          self.cfg_key, self.cfg, self.spec = None, None, None       # back at A and finished: forget the activation
      else:
        self.mode = "idle"
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
    if self.pin is not None:
      self.mode = "pinned"
      self._switch(0.0 if self.pin == "A" else 1.0, "pin")
    else:
      self.mode = "running"
      self._blocks(reason, self.block_len)

    # ---- scored time
    if self.wash_left == 0 and not self.ramp.moving:
      self.progress[self.arm_in_force][self.window] += DT
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
    if not self.speed_ok:
      reasons.append("speed")
    elif EPS_KEEPOUT[0] <= mph < EPS_KEEPOUT[1]:
      reasons.append("eps")
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

  def _blocks(self, reason, block_s):
    if self.block is None:
      idx = self.idx
      arm = arm_for(self.seed, idx)
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
      self._event("block_start", idx=idx, arm=arm, switched=bool(need), waited_s=waited, block_s=block_s,
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

  def status(self, now=None):
    """The block the A/B page reads (param TiAbStatus)."""
    block = None
    if self.block is not None and self.cfg is not None:
      length = self.block_len
      block = {"idx": self.block[0], "arm": self.block[1], "len_s": length, "left_s": round(max(length - self.block_frames * DT, 0.0), 1)}
    wait = None
    if self.mode == "running" and self.block is None and self.pending_since is not None:
      wait = {"reason": self.last_reason or "settle", "why": MESSAGE.get(self.last_reason or "settle", ""),
              "s": round((self.n - self.pending_since) * DT, 1), "max_s": MAX_WAIT_S,
              "arm": arm_for(self.seed, self.idx)}
    spec = self.spec
    value, weight = self.effective()
    return {"v": VERSION, "t": round(time.time() if now is None else now, 1), "state": "error" if self.dead else self.mode,
            "msg": (f"A/B stopped after an error: {self.reason_dead}" if self.dead else self.message),
            "setting": None if spec is None else spec.name, "kind": None if spec is None else spec.kind,
            "a": None if self.cfg is None else self.cfg.a, "b": None if self.cfg is None else self.cfg.b,
            "value": value, "arm": self.arm_in_force if spec is not None else None, "blend": round(self.ramp.s, 3),
            "pin": self.pin, "block": block, "wait": wait, "wash_in": self.wash_left > 0, "seed": self.seed if spec else None,
            "idx": self.idx, "skipped": self.skipped, "mph": round(self.mph, 1),
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
      if arm_for(seed, e["idx"]) != e["arm"] or first_of_pair(seed, e["idx"] // 2) != e["first"]:
        bad.append(e)
  return bad


# ---- the glue controlsd calls --------------------------------------------------------------------------------------
class TiAbRunner:
  """Reads the params at a low rate, calls the module, publishes the status, and returns the toggles the lateral controller
  is to use: the owner's own object whenever A is in force (so the controller runs exactly as without this mode), else an
  AbToggles view. Never raises; after an error it returns the owner's toggles for the rest of the process."""
  ON_EVERY, CONFIG_EVERY, STATUS_EVERY = 10, 100, 50     # cycles

  def __init__(self, params, params_memory, log=None, block_s=None):
    self.params, self.memory = params, params_memory
    self.ab = TiAb(log=log, block_s=block_s)
    self.k = 0
    self.on, self.pin, self.config = False, None, None
    self.read_error = ""
    self.published = None
    self.published_k = -10 ** 9

  def _read(self):
    try:
      self.on = bool(self.memory.get_bool(PARAM_ON))
      raw_pin = self.memory.get(PARAM_PIN)
      if isinstance(raw_pin, (bytes, bytearray)):
        raw_pin = raw_pin.decode("utf-8", "replace")
      self.pin = None if raw_pin in (None, "") else raw_pin
      self.read_error = ""
    except Exception as exc:
      self.on, self.pin = False, None
      self.read_error = f"A/B switch unreadable: {type(exc).__name__}"
    if self.k % self.CONFIG_EVERY == 0 or self.config is None:
      try:
        self.config = self.params.get(PARAM_CONFIG)
      except Exception:
        self.config = None

  def step(self, toggles, **inputs):
    """`inputs`: engaged, v_ego, curvature, steer, saturated, steering_pressed, driver_torque, lane_change, blinker, ti_ok,
    fault, controller_ok, mono_ns."""
    try:
      if self.ab.dead:
        self._publish()
        return toggles
      if self.k % self.ON_EVERY == 0:
        self._read()
      self.k += 1
      specs = [SETTINGS[n] for n in ALLOWED]
      stored = {s.name: stored_value(s, toggles) for s in specs}
      tests = any(getattr(toggles, t, False) is True for t in TEST_TOGGLES)
      out = self.ab.update(on=self.on, pin=self.pin, config=self.config, stored=stored, tests_active=tests, **inputs)
      if self.read_error and not self.ab.message:
        self.ab.message = self.read_error
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

  def _publish(self):
    st = self.ab.status()
    key = (st["state"], st["msg"], st["arm"], st["pin"], st["block"] and st["block"]["idx"], st["wait"] and st["wait"]["reason"])
    if key != self.published or self.k - self.published_k >= (self.STATUS_EVERY if st["state"] != "off" else 4 * self.STATUS_EVERY):
      try:
        self.memory.put(PARAM_STATUS, json.dumps(st))
        self.published, self.published_k = key, self.k
      except Exception:
        pass
