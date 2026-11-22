# A/B alternation: log format and use (ti_ab.py)

One controller tuning setting alternates between the owner's own value (A) and a value B in blocks of a few minutes while he
drives. A is his stored value as the lateral controller reads it (never written, never copied); B and the blend between exist
only in memory. Off at every boot, off by default, off while any identification test is on.

## Params

| key | where | meaning |
|---|---|---|
| `TiAbConfig` | persistent (written by SET UP A/B; absent = no alternation set up, START then runs the tests alone) | JSON `{"setting": "SteerKP", "b": 0.95, "block_min": 4, "seed": 123}`. `a` may be given but must equal the stored value. `block_min` 4-10 (default 4). `seed` optional (drawn and logged when absent). Refused with a message on the A/B page outside the ranges below. |
| `TiAbOn` | tmpfs, cleared at manager start AND whenever controls (re)starts (`TiAbRunner.__init__`), as `TiPlanOn` and `TiAbPin` are; the UI clears them at the end of a drive | the switch (the A/B page's box). A configuration change while on stops the run: switch off and on again. |
| `TiAbPin` | tmpfs | `A` or `B`: alternation stops, the value ramps to that arm; empty: alternate. |
| `TiAbProgress` | persistent | JSON `{v, key, setting, a, b, committed {A, B} seconds, progress {A|B: {window: seconds}}}`, written at each block end. `key` = [table hash, setting, A, B]: restored when the same experiment is set up (completion is judged on the restored total), cleared when the experiment changes, removed by RESET PROGRESS on the set-up page; an unreadable record reads as zero. Restoring only fills counters of an activation that is already allowed: it can never start the mode. |
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
- `block_start`: `idx arm arm_why first switched waited_s block_s repeat`. `arm_why` is `seed` (the seeded pair order) or `balance`: when the two sides' scored time differs by more than `BALANCE_MARGIN_S` (30 s; partial blocks cut at hand-overs make it uneven) the next block is the side that is behind, so completion (both sides at target) is not held up by one lagging side. `verify_schedule` skips `balance` blocks. The analysis pairs by time in force from the `switch` events, so the label of a block, and therefore this change, does not affect it; a reader that recomputes arms from the seed alone must skip `arm_why == "balance"` blocks (the simulator-side reader should be told). `arm` = `arm_for(seed, idx)`: pair `idx // 2` is (A, B) if `first` is "A" else (B, A);
  `first` = "A" when `sha256(f"{seed}:{pair}")[:6]/2**48 < 0.5`. `repeat`: a voided block run again.
- `block_end`: `idx arm engaged_s scored_s reason voided`. `reason` done, pinned, or a release reason. `scored_s` is the block's scored time (wash-in, ramps, pinned time excluded). A block that ends early (speed, hand-over, disengagement, switch off) with `scored_s` of at least `MIN_SCORED_KEEP_S` (30 s) is KEPT: its scored seconds are committed and the next idx follows; below that it is voided and its idx runs again (before this revision the limit was 60 s of engaged time). Analysis pairs by time in force from the `switch` events (see below), never by block label, so partial blocks are usable as they are.
- A block starts only when the speed is `MARGIN_MPH` (3 mph) inside either edge of the range and has been for `MARGIN_HOLD_S` (3 s): gate reason `margin` (in `PRIORITY` after `eps`). A block already running still ends at the old edges (35 mph and the ceiling).
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

## Drive plan (ti_plan.py)

`TiPlanOn` (tmpfs, off at every boot, set by START on the Tests first page when an alternation is set up; STOP, or starting a single test, switches it off) sequences the alternation and the tests; with it off nothing here acts.
The alternation runs where it is configured, incomplete and the speed is 35-64 mph (`PLAN_ALT_MAX_MPH`, 64 since 2026-11-22: the 57-63 mph gap went to the alternation; earlier 56 because replay evidence covered 35-56 mph only;
a mile of hysteresis, and `MIN_DWELL_S` = 10 s between speed-driven changes); everywhere else, and everywhere once it is complete, the tests run in run-all
mode under all their own gates. Complete = `TARGET_MIN_PER_SIDE` (10; the plan file gives the reason) scored minutes on each side, counted at the end of each non-voided block from
scored (non-wash-in, not ramping, not pinned; for LatCommitSetpoint only in bends) time. On completion: `complete` event, the value returns to A by its
ramp and stays A for the rest of the drive (`state` "complete", notice on the page), the tests take over.

`plan` events (`mode` off/hold/ab/tests, `why`, `seq`): alternation time is `mode == "ab"` after the `ab_allowed` start (the first `block_start`), test
time is `mode == "tests"`; `hold` is a hand-over in which neither runs. tmpfs: `TiPlan` {seq, mode, t} from controls (0.5 s heartbeat; stale after 3 s
allows no test), `TiPlanAck` {seq, quiet, t} from the car controller. tests -> A/B: A/B may start only after an acknowledgement of the current seq has been
quiet for 1 s (no test signal, fade included) and then only through the usual straight-road gate; A/B -> tests: the plan holds, the blend returns to
A and has been A for its wash-in, then it publishes "tests". An error in the plan ("dead") holds both for the process and the tests stay held.

Plan hardening: `TiPlan` and `TiPlanAck` carry a per-process random `nonce` besides `seq`; an acknowledgement counts only if both match the current plan.
`TiPlan` also carries `at_a` (the blend is exactly A and settled). With the plan on OR off, the car controller holds every test unless a published plan is fresh
(3 s) and `at_a` is true; a dead, stale or unreadable plan holds. With no plan ever published and the plan off, the tests are as without any of this. Every change of
that decision is logged by the car controller as `ti_ab` `tests_hold` {held, why, plan_on, nonce, seq}; `why`: plan dead, plan missing, plan unreadable,
plan stale, alternation not at A, plan says <mode>, A/B switch on.
