# A/B alternation: log format and use (ti_ab.py)

One controller tuning setting alternates between the owner's own value (A) and a value B in blocks of a few minutes while he
drives. A is his stored value as the lateral controller reads it (never written, never copied); B and the blend between exist
only in memory. Off at every boot, off by default, off while any identification test is on.

## Params

| key | where | meaning |
|---|---|---|
| `TiAbConfig` | persistent | JSON `{"setting": "SteerKP", "b": 0.95, "block_min": 4, "seed": 123}`. `a` may be given but must equal the stored value. `block_min` 4-10 (default 4). `seed` optional (drawn and logged when absent). Refused with a message on the A/B page outside the ranges below. |
| `TiAbOn` | tmpfs, cleared at manager start | the switch (the A/B page's box). A configuration change while on stops the run: switch off and on again. |
| `TiAbPin` | tmpfs | `A` or `B`: alternation stops, the value ramps to that arm; empty: alternate. |
| `TiAbStatus` | tmpfs, 2 Hz while on | what the page shows (state, arm, value, block, wait reason, progress) |

First-trial limits (named constants in `ti_ab.py`, changed later only on evidence): alternation is active only between 35 and 70 mph (released
below 35 and from 70, back with 1 mph hysteresis; below about 28 mph the interceptor steers alone with little authority); blocks 4-10 minutes
(default 4); a pin to A is immediate by ramp, a pin to B waits for the same straight-road gates as an automatic switch ("pin waiting: <reason>", log
`pin_wait`) and needs a second tap on the page within 3 s; a driver touch, blinker or lane change while the value is moving toward B turns the ramp
back to A (`switch` why `reverse:*`, the block ends `reversed`, voided and repeated); a ramp toward A is never interrupted.

Allowed settings, range, largest |A-B|, blend (ramp) and wash-in. Wash-in is the ramp plus the time the loop needs to settle; where a
setting changes the steady error (gain, friction compensation, relay) the integrator (time constant about kp/ki, around 4 s) must re-converge, so
the wash-in is the ramp plus 10 s; the others only need the filters and blends to settle.
- SteerKP: 0.70-1.00, 0.10, ramp 2 s, wash-in 12 s.
- LatNoFrictionRelay (on/off): ramp 1 s, wash-in 11 s.
- LatFrictionComp (on/off): ramp 1 s, wash-in 11 s.
- LatDamping (on/off): ramp 1 s, wash-in 6 s.
- LatCommitSetpoint (on/off): no module ramp (the controller blends over 0.5 s), wash-in 6 s. It has no effect on a straight, so its blocks can
  be scored only in bends: the page says so and the per-side minutes count it only while the lateral demand is at least 1.0 m/s^2.
- LatOutputFilter (on/off): no module ramp (the controller blends over 0.25 s), wash-in 4.5 s.

Excluded: TiSteerMax, TiSteerDeltaUp, TiSteerDeltaDown, TiSteerDeltaUpKnee, TiSteerDeltaUpHigh (authority and limits), TiSteerDriverAllowance,
TiSteerDriverMultiplier, TiSteerThreshold (driver override safety), ForceAutoTune (changes where the torque parameters come from).

## Events

cloudlog event `ti_ab` (rlog `logMessage`, JSON), the same style as `ti_ident`. Every event has `kind`, `frame` (the module's cycle counter,
100 Hz from process start, the same clock for every event of one process) and `mono_ns` (CLOCK_BOOTTIME of the cycle, the clock of
`logMonoTime`).

- `config`: `setting a b block_s seed seed_given ramp_s wash_in_s weighted version table`. Starts one activation (blocks count from 0).
- `block_start`: `idx arm first switched waited_s block_s repeat`. `arm` = `arm_for(seed, idx)`: pair `idx // 2` is (A, B) if `first` is "A" else (B, A);
  `first` = "A" when `sha256(f"{seed}:{pair}")[:6]/2**48 < 0.5`. `repeat`: a voided block run again.
- `block_end`: `idx arm engaged_s reason voided`. `reason` done, pinned, or a release reason. A block cut short under 60 s is voided and its idx runs again.
- `wait_start` / `wait_end`: a switch is pending and unsafe. `wait_end`: `outcome` switched, not_needed, pinned or release:*, `waited_s`, `reasons` {gate: seconds}.
- `skip`: the wait hit 60 s; the value stays, `kept` is the arm in force, the next idx is tried. `reasons` as above.
- `switch`: EVERY change of the value in force: `why` (block, pin, release:*), `s_from s_to ramp_frames wash_frames a b arm immediate`.
- `pin` / `unpin` (`arm`), `pin_wait` (a pin to B is waiting for a gate: `why`), `release` (a hard release when no value had to change: `why`), `refuse` (`why`: configuration or pin refused), `error` (`why`: the mode is
  off for the rest of the drive), `hb` every 10 s (`s arm wash_left idx block pin window progress`).

## Assigning every sample to A or B, and wash-in

The blend `s` runs from 0 (A) to 1 (B). `reconstruct(events, n)` in `ti_ab.py` returns, for every module frame, `(s, arm, wash_in)` from the `switch`
events alone: at a `switch` the blend moves linearly from where it is to `s_to` over `ramp_frames` frames (0 = at once) and `wash_frames` frames are
marked wash-in. The value in force is `A + s (B - A)` (a boolean: weight `(1-s)A + sB` of "on"). Score a sample as A when `s == 0` and as B when `s == 1`
and the sample is not wash-in; everything else (ramping, `wash_in`) is excluded. Also exclude samples where lateral control was not active
(the module is at A then). SteerKP reaches the PID one control cycle after `s`. `verify_schedule(events)` recomputes every block's arm from the seed.
Samples marked `gap` in `hb.progress` are driven between the four speed windows (21-27, 35-41, 49-56, 64-80 mph; only 35-70 mph is driven by this mode): used for scoring but not counted for coverage.
