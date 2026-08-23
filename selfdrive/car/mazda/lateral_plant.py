"""Lateral plant model for a Mazda GEN1 with a Torque Interceptor.

This car steers through two actuators at once, and openpilot sends the same request to both:

  stock path   CAM_LKAS (0x243) -> EPS. The EPS applies it and reports back what it applied as
               STEER_RATE.LKAS_EFFECTIVE (0x241): about 0.90 of the request, hard-clipped at +-308
               counts, and gated off entirely below ~45 km/h (LKAS_BLOCK) with hysteresis back on
               around 52. Linear while it lasts, worth 0.3-0.6 m/s^2 at its clip.
  interceptor  CAM_LKAS2 (0x249) -> TI, which biases the EPS torque sensor. Its lateral response is
               quadratic in the command: nothing much below ~150-200 counts, then steeply rising to
               ~1.0 m/s^2 at 600 counts below 52 km/h and ~1.55 above 65.

A single "torque -> lateral acceleration" gain (openpilot's latAccelFactor, or a network trained on
another car) is a chord through that convex, clipped, state-dependent curve: too weak at small
commands, too strong at large ones, and it moves with whatever roads were driven last. This module
is the measured shape instead, so the feedforward can be inverted from it.

These are STEADY-STATE gains. The car does not answer a command and stop: identification over
119 min of engaged driving (39 routes; TI-analysis/tools/rlog_analysis/dyn_fit.py) puts a 1.0 s
first-order lag behind 0.2 s of pure delay, r^2 0.79 against 0.73 for the "answered within 0.3 s"
assumption the first cut of these tables was fitted on. That first cut therefore held a 0.3 s map,
and a feedforward inverting it over-commanded whenever a corner lasted -- measured 1.2-1.3x too much
torque with both actuators live. Refitted 2026-08-18: q from interceptor-alone frames with an
instrumented estimator (the command is generated inside the loop, so plain least squares is biased;
modelV2's desired lateral accel is the instrument), g_s from the same dynamics with q held fixed.
Scored against every recorded frame (model_check.py), the tables now sit within 5 % of measured for
both actuators from 250 counts up, and run deliberately strong with the interceptor alone (0.75-0.86
of measured), which under-commands rather than over-steers where the data is thinnest.

The shape and tables are properties of the EPS firmware and the interceptor's DAC scaling and are
fixed; live learning is limited to latAccelOffset today and, later, a bounded gain multiplier per
actuator state.

Sign convention here: `u` (counts) and `la` (m/s^2) are both positive = left, matching the wire.
The controller works in its own frame and handles signs at the call site.
"""
import numpy as np

# --- fitted tables (see module docstring) -----------------------------------------------------
V_BP = [7.0, 12.5, 15.0, 17.5, 20.0, 25.0, 30.0, 35.0]              # m/s (25, 45, 54, 63, 72, 90, 108, 126 km/h)
Q_BP = [2.79e-6, 3.14e-6, 3.49e-6, 4.00e-6, 4.51e-6, 4.96e-6, 4.96e-6, 4.96e-6]  # m/s^2 per count^2, interceptor
GS_BP = [1.21e-3, 1.21e-3, 1.31e-3, 1.88e-3, 2.14e-3, 2.34e-3, 2.41e-3, 2.41e-3]  # m/s^2 per LKAS_EFFECTIVE count, stock

K_EFF = 0.90            # LKAS_EFFECTIVE the EPS applies per count of stock request, settled (measured p50 0.91)
EFF_MAX = 308.0         # EPS clip on the stock path (never exceeded in 342 segments)
U_EFF_CLIP = EFF_MAX / K_EFF   # 342 counts of request: above this the stock path adds nothing more
U_KNEE = 150.0          # below this the interceptor term is linearised so the inverse stays finite at 0
EFF_RAMP_UP = 1.5       # counts/frame the EPS ramps LKAS_EFFECTIVE in after LKAS_BLOCK clears (measured 1.4-1.6)

# ramp-in bookkeeping
RAMP_SETTLE_TOL = 16.0  # counts: |e_meas - K_EFF*u| below this means the EPS has caught up
RAMP_SETTLE_FRAMES = 5
RAMP_SETTLE_EFF = 300.0 # at the clip there is nothing left to ramp
RAMP_TIMEOUT = 3.0      # s, hard end of ramp-in

# stock path declared dead (camera fault, LKAS switched off) when it reports nothing while asked
DEAD_STOCK_TIME = 1.0   # s of LKAS_EFFECTIVE == 0 while asking for more than DEAD_STOCK_REQ
DEAD_STOCK_REQ = 100.0  # counts of applied stock request
DEAD_STOCK_CLEAR = 20.0 # counts: any real feedback revives it

ALPHA_MIN, ALPHA_MAX = 0.75, 1.25   # bounds on the learned per-state gain multipliers (phase 2)

# plantState log codes. Bits 0-1 are the actuator state; everything above is a flag added to it so
# a drive can be split on it afterwards. The full allocation is listed here because the flags are
# set in three different files, and a collision would be silent -- it would just mislabel a
# population, which is the failure mode this whole field exists to prevent.
STATE_NONE = 0
STATE_STOCK_ONLY = 1
STATE_TI_ONLY = 2
STATE_TI_STOCK = 3
STATE_RAMP_FLAG = 4      # added to the above while the stock path is ramping in
# 8   OUT_FILTER_FLAG -- latcontrol_torque.py, output filter active
# 16  RETIRED (was STALL_MOD_FLAG: stall modulation -- never once triggered in 5 h of logs)
# 32  NNFF_FLAG       -- neural_network_feedforward.py, NNFF produced this frame
# 64  NNFF_GAIN_FLAG  -- neural_network_feedforward.py, gain correction moved the command
# 128 RETIRED (was DEMAND_CAP_FLAG: demand cap -- tripped the TI 0x11 watchdog on the road)
# 256 RETIRED (was FF_LOOKAHEAD_FLAG: lookahead FF -- wandered on the road in both forms)
# 512 FRIC_COMP_FLAG    -- latcontrol_torque.py, friction compensation actually contributing
# Retired bits are frozen forever: recorded rlogs carry them with the old meanings, and the
# analysis decoders keep reading them. Allocate new behaviours from "next free" only.
# next free: 1024


class TiLateralPlant:
  def __init__(self, ti_steer_max=600.0, dt=0.01):
    self.dt = float(dt)
    self.ti_steer_max = float(ti_steer_max)
    self.alpha_ti = 1.0
    self.alpha_stock = 1.0

    # state
    self.stock_active = True      # assume the higher-authority actuator set until told otherwise:
    self.ti_active = True         # under-commanding is the failure mode we accept, over-steer is not
    self.ramp_in = False
    self.ramp_frames = 0
    self.ramp_settled_frames = 0
    self.dead_stock = False
    self.dead_frames = 0
    self.e_used = 0.0             # stock torque assumed for this frame, counts (0 unless ramping in)
    self.u_prev = 0.0
    self._u_grid = np.arange(0.0, 601.0, 10.0)

  def reset(self):
    """Back to first principles, for a fresh engagement: re-arm the ramp-in and give the stock path
    another chance (a camera fault or an LKAS switch may have been fixed since we gave up on it)."""
    self.ramp_in = True
    self.ramp_frames = 0
    self.ramp_settled_frames = 0
    self.dead_stock = False
    self.dead_frames = 0
    self.e_used = 0.0
    self.u_prev = 0.0

  # -- helpers -----------------------------------------------------------------------------------
  def set_multipliers(self, alpha_ti=1.0, alpha_stock=1.0):
    """Learned gain multipliers per actuator state (phase 2). Bounded: a bad estimate can only move
    the feedforward by a quarter, and the PID covers the rest."""
    self.alpha_ti = float(np.clip(alpha_ti, ALPHA_MIN, ALPHA_MAX))
    self.alpha_stock = float(np.clip(alpha_stock, ALPHA_MIN, ALPHA_MAX))

  def set_ti_steer_max(self, ti_steer_max):
    self.ti_steer_max = float(np.clip(ti_steer_max, 100.0, 1200.0))
    self._u_grid = np.arange(0.0, self.ti_steer_max + 1.0, max(self.ti_steer_max / 60.0, 1.0))

  def _q(self, v):
    return float(np.interp(v, V_BP, Q_BP)) * self.alpha_ti

  def _gs(self, v):
    return float(np.interp(v, V_BP, GS_BP)) * self.alpha_stock

  # -- state machine -----------------------------------------------------------------------------
  def update_state(self, v, lkas_blocked, eff_meas, ti_active, active=True):
    """Called once per frame before forward/inverse. `eff_meas` is STEER_RATE.LKAS_EFFECTIVE (signed
    counts) and `lkas_blocked`/`ti_active` come from the EPS and the interceptor. Missing inputs
    (frogpilotCarState stale) should be passed as lkas_blocked=False, ti_active=True: the lower-torque
    assumption, i.e. we under-deliver rather than over-steer."""
    u_prev = abs(self.u_prev)
    e_abs = abs(float(eff_meas))
    same_sign = (float(eff_meas) == 0.0) or (np.sign(eff_meas) == np.sign(self.u_prev)) or self.u_prev == 0.0

    if not active:
      # not steering: nothing to infer, re-arm so the next engagement uses the measurement
      self.ramp_in = True
      self.ramp_frames = 0
      self.ramp_settled_frames = 0
      self.dead_frames = 0
      self.e_used = 0.0
      self.ti_active = bool(ti_active)
      self.stock_active = not bool(lkas_blocked) and not self.dead_stock
      return self.plant_state_code()

    self.ti_active = bool(ti_active)

    # the stock path is dead if it reports nothing while we are asking it for real torque
    if self.dead_stock:
      if e_abs > DEAD_STOCK_CLEAR:
        self.dead_stock = False
        self.dead_frames = 0
        self.ramp_in = True
        self.ramp_frames = 0
        self.ramp_settled_frames = 0
    elif not lkas_blocked and not self.ramp_in and K_EFF * u_prev > DEAD_STOCK_REQ and e_abs == 0.0:
      self.dead_frames += 1
      if self.dead_frames * self.dt >= DEAD_STOCK_TIME:
        self.dead_stock = True
    else:
      self.dead_frames = 0

    self.stock_active = (not bool(lkas_blocked)) and (not self.dead_stock)

    if not self.stock_active:
      # blocked or dead: no stock contribution, and re-arm ramp-in for whenever it comes back
      self.e_used = 0.0
      self.ramp_in = True
      self.ramp_frames = 0
      self.ramp_settled_frames = 0
      return self.plant_state_code()

    if self.ramp_in:
      # The EPS winds LKAS_EFFECTIVE in at ~1.5 counts/frame after LKAS_BLOCK clears, so for those
      # ~2 s the measurement is the only truthful source of what the stock path is contributing.
      # (It is never used once settled: e_meas lags the request by 70-90 ms, and feeding that back
      # into the inverse would close a delayed loop with gain ~1 and ring on turn-in.)
      self.e_used = min(e_abs, EFF_MAX) if same_sign else 0.0
      self.ramp_frames += 1
      caught_up = abs(e_abs - K_EFF * u_prev) <= RAMP_SETTLE_TOL
      self.ramp_settled_frames = self.ramp_settled_frames + 1 if caught_up else 0
      if (self.ramp_settled_frames >= RAMP_SETTLE_FRAMES or self.e_used >= RAMP_SETTLE_EFF
          or self.ramp_frames * self.dt >= RAMP_TIMEOUT):
        self.ramp_in = False
        self.ramp_settled_frames = 0
    else:
      self.e_used = 0.0   # settled: the stock term is algebraic in u, solved inside forward/inverse

    return self.plant_state_code()

  def plant_state_code(self):
    code = STATE_NONE
    if self.ti_active and self.stock_active:
      code = STATE_TI_STOCK
    elif self.ti_active:
      code = STATE_TI_ONLY
    elif self.stock_active:
      code = STATE_STOCK_ONLY
    if self.stock_active and self.ramp_in:
      code += STATE_RAMP_FLAG
    return code

  # -- model -------------------------------------------------------------------------------------
  def _stock_la(self, u_abs, v):
    """Lateral accel from the stock path for an interceptor command of u_abs counts."""
    if not self.stock_active:
      return 0.0 * u_abs
    if self.ramp_in:
      return self._gs(v) * min(self.e_used, EFF_MAX)          # measured, while the EPS winds in
    return self._gs(v) * np.minimum(K_EFF * u_abs, EFF_MAX)    # settled: what the EPS will apply

  def _ti_la(self, u_abs, v):
    """Interceptor contribution: quadratic above the knee, linear through the origin below it."""
    if not self.ti_active:
      return 0.0 * u_abs
    return self._q(v) * u_abs * np.maximum(u_abs, U_KNEE)

  def forward(self, u, v):
    """Steady-state lateral acceleration (m/s^2, +left) for a command of u counts (+left)."""
    u = float(u)
    u_abs = min(abs(u), self.ti_steer_max)
    return float(np.sign(u) * (self._stock_la(u_abs, v) + self._ti_la(u_abs, v)))

  def la_max(self, v):
    """What this car can actually do right now, at the ceiling and in the current actuator state.
    Used for the PID limits so the integrator winds against real authority, and so the saturation
    warning means something."""
    return float(self._stock_la(self.ti_steer_max, v) + self._ti_la(self.ti_steer_max, v))

  def inverse(self, la, v):
    """Command (counts, +left) that the model says produces `la`, i.e. the SMALLEST command that
    does: the forward model is non-decreasing but not strictly increasing (the stock path clips, and
    with the interceptor off it is flat above 342 counts), and asking for more torque than a request
    can use would put a step on the wire the moment the other actuator comes back.

    The exception is a request at or beyond what the car can currently do: there we go to the
    ceiling, so that the controller's saturation check (which looks for a command at the ceiling)
    reports "Turn Exceeds Steering Limit" honestly -- e.g. with the interceptor tripped off, where
    all the authority there is amounts to 0.3-0.6 m/s^2. The extra counts do nothing in that state.

    Returns 0 when neither actuator is in play."""
    la = float(la)
    a = abs(la)
    grid = self._u_grid
    la_grid = np.atleast_1d(self._stock_la(grid, v) + self._ti_la(grid, v)).astype(float)
    if la_grid.size != grid.size or la_grid[-1] <= 1e-9:
      return 0.0                                   # no authority at all: command nothing
    if a >= la_grid[-1]:
      u = float(grid[-1])                          # beyond what we can do; say so by saturating
    else:
      hi = int(np.searchsorted(la_grid, a, side="left"))
      if hi <= 0:
        u = float(grid[0])
      else:
        lo = hi - 1
        span = la_grid[hi] - la_grid[lo]
        frac = 0.0 if span <= 0.0 else (a - la_grid[lo]) / span
        u = float(grid[lo] + frac * (grid[hi] - grid[lo]))
    return float(np.clip(u, 0.0, self.ti_steer_max) * (1.0 if la >= 0.0 else -1.0))

  def inverse_closed(self, la, v):
    """Closed-form inverse of the settled model, for tests and for reading the algebra:

      settled, both paths, u <= U_EFF_CLIP:   q*u^2 + K_EFF*g_s*u = A      (u >= U_KNEE)
                           u  > U_EFF_CLIP:   q*u^2 + g_s*EFF_MAX = A
      below the knee the TI term is linear:   (K_EFF*g_s + q*U_KNEE)*u = A
      TI only:                                q*u^2 = A  (or q*U_KNEE*u = A below the knee)
      stock only:                             K_EFF*g_s*u = A
    """
    la = float(la)
    a = abs(la)
    sign = 1.0 if la >= 0.0 else -1.0
    q = self._q(v) if self.ti_active else 0.0
    gs = self._gs(v) if self.stock_active else 0.0
    if self.stock_active and self.ramp_in:
      a = max(a - gs * min(self.e_used, EFF_MAX), 0.0)
      gs = 0.0
    if q == 0.0 and gs == 0.0:
      return 0.0
    if q == 0.0:                                   # stock only: flat once the EPS clips
      u = min(a / (K_EFF * gs), U_EFF_CLIP) if a < gs * EFF_MAX else self.ti_steer_max
      return sign * float(np.clip(u, 0.0, self.ti_steer_max))
    # try the linear-below-the-knee branch first
    u = a / (K_EFF * gs + q * U_KNEE)
    if u > U_KNEE:
      if gs > 0.0:
        u = (-K_EFF * gs + np.sqrt((K_EFF * gs) ** 2 + 4.0 * q * a)) / (2.0 * q)
        if u > U_EFF_CLIP:                          # stock path clipped: the rest is all interceptor
          u = np.sqrt(max(a - gs * EFF_MAX, 0.0) / q)
      else:
        u = np.sqrt(a / q)
    return sign * float(np.clip(u, 0.0, self.ti_steer_max))
