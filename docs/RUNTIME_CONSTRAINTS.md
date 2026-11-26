# Runtime constraints of the processes our code runs in

Verified in this tree (2026-11-25, branch memguard-20261125); every fact carries its file:line. Per-module classification:
`tools/dev/per_frame_inventory.json` (human view `tools/dev/PER_FRAME_INVENTORY.md`). A newly found constraint goes here AND into a
test or lint rule in the same change (`docs/BRIEFING_CHECKLIST.md`).

## controlsd (`selfdrive/controls/controlsd.py`)
- Rate: 100 Hz, `controlsd.py:214` `Ratekeeper(100)`.
- Collector: OFF. `controlsd.py:1181` `config_realtime_process(4, Priority.CTRL_HIGH)` -> `common/realtime.py:41` `gc.disable()`.
  Nothing ever frees a reference cycle in this process.
- Real time: core 4, priority CTRL_HIGH (`controlsd.py:1181`).
- Ours here: lane_drift gate (`:850`, `:864`), TiAbRunner.step (`:269`), LatControlTorque with the TI plant (`:804-814`),
  record_inputs (`:817`).
- Budget: the install cost check (`selfdrive/car/mazda/ti_cost_check.py:428`, `--budget-us 300`) times TiAbRunner.step and the
  lateral controller; its docstring (`:20-21`) gives the frame: ~1.5 ms per 10 ms cycle free on the control core, a lag soft-disable
  when the 1-second mean is above 11.1 ms.
- Consequences for code:
  - Create no class, closure or bound method per call (2026-11-25: a class per call leaked 1-2 GB per engaged hour).
  - Keep no list / dict / set without a bound; trim event buffers (`ti_ab.py` `_event`: 4000 -> 2000).
  - Allocate nothing proportional to drive time; keep no reference to a message past its frame.
  - No blocking I/O; params reads / writes and JSON at a fraction of the rate (the existing code: 10 Hz reads, 1 Hz status).
  - Catch exceptions without storing them (a stored traceback holds frames: a cycle).

## card (`selfdrive/car/card.py`, the Mazda CarController / CarState / CarInterface)
- Rate: 100 Hz, `card.py:107` `Ratekeeper(100)`.
- Collector: OFF. `card.py:260` `config_realtime_process(4, Priority.CTRL_HIGH)` -> `common/realtime.py:41`.
- Real time: core 4 (shared with controlsd), CTRL_HIGH.
- Ours here: CarController.update via `card.py:221` `CI.apply`; the lag guard's input `card.py:217-220`; CarState TI fields
  (`selfdrive/car/mazda/carstate.py:64`).
- Budget: `ti_cost_check.py:428` 300 us per cycle added by any tests mode (comma_checks.sh stage 1).
- Params: tmpfs (`/dev/shm/params`) for anything written while driving; flash writes put_nonblocking and at most once a minute
  (`carcontroller.py` publish_ti_stats, `frame % 6000`). A new params key must also be declared in `common/params.cc`
  (`carcontroller.py:624`; checked by `frogpilot/tools/check_params_sync.py`, comma_checks.sh stage 2), else the device does not boot.
- Consequences for code: the controlsd list above, plus: never send a button frame or CAN message outside the existing tap
  machinery; anything optional is imported in try/except so that without it the controller runs unchanged (`carcontroller.py:14-53`).

## ti_curve_plannerd (`selfdrive/car/mazda/ti_curve_plannerd.py`)
- Rate: 5 Hz, `ti_curve_plannerd.py:27` `RATE_HZ = 5.0`, `:471` Ratekeeper.
- Collector: ON, everything loaded at start frozen: `:469` `gc.freeze()` (a full collection over the memory took 54 ms, `:468`).
- Real time: none; cores 0-3, below normal priority, never core 4 (`:423` keep_off_control_core).
- Started by the manager onroad: `system/manager/process_config.py:128`.
- Budget: tick p99 under 20 ms of the 200 ms tick (`selfdrive/car/mazda/ti_curve_dryrun.py:38`, comma_checks.sh stage 4).
- Never a position in a log or param: `ti_curve_plannerd.py:16`; plan JSON checked by `test_mazda_ti_curve_plan.py:452`.
- Consequences for code: caches keyed by finite domains only (lint PF4 waivers name the domain); nothing created per tick that the
  frozen generation keeps; positions stay inside the process (indices, distances, speeds, curvatures out).

## frogpilot_process (`frogpilot/frogpilot_process.py`)
- Rate: 20 Hz, `frogpilot_process.py:55` `Ratekeeper(1 / DT_MDL)`; polls modelV2 (`:76`).
- Collector: OFF. `frogpilot_process.py:57` `config_realtime_process(5, Priority.CTRL_LOW)` -> `common/realtime.py:41`.
- On-road only (`:112-116`, `deviceState.started`), engaged or not: FrogPilotPlanner.update / publish, FrogPilotTracking.update.
  Ours here: FrogPilotEvents.update's TI block, which runs LaneDriftGate.update every model frame (`frogpilot_events.py:262`), and
  the planner's TI block (`frogpilot_planner.py:122-162`: steer_authority, ti_curve alert readers). Parked it runs none of this.
- 2026-11-25 second leak: lane_drift's per-call class (fixed in cc877397) also ran here, 13.6 KB per cycle on the comma (the class
  plus the modelV2 message it kept), ~1 GB/h on-road; the drive measured 1.0-1.4 GB/h. Scenario `frogpilot_events` in ti_mem_check.
- Consequences for code: the controlsd list above; keep no reference to a SubMaster message past the cycle (it holds the whole
  message buffer, about 10 KB for modelV2).

## ti_mcp (`frogpilot/system/ti_mcp/ti_mcp_server.py`)
- On request over HTTP, not per frame; only with TiMcpEnabled (`system/manager/process_config.py:78`). Read-only by design
  (module docstring). Collector on (no config_realtime_process call).

## UI (C++ Qt: `selfdrive/ui/qt/ti_pages_logic.h`, `selfdrive/ui/qt/sidebar.cc`, `frogpilot/ui/qt/offroad/lateral_settings.cc`)
- Reads our params (TiCurveStatus, TiTuningStats). Rate and memory behaviour NOT verified here; the Python guards do not cover it.

## Python version
- Device: `.python-version` = 3.11.4 (used by `tools/install_python_dependencies.sh:44`), `pyproject.toml:47` mypy 3.11,
  `Dockerfile.openpilot_base:79` pyenv 3.11.4. On the comma (checked 2026-11-25 over ssh): Python 3.11.4, pycapnp 1.3.0,
  numpy 1.26.4.
- 3.11 free lists: a freed tuple (up to 2000 per size), float, list or dict is kept by the interpreter and keeps its tracemalloc
  trace until a full collection; bounded, but it reads as per-frame growth in a short window (2026-11-25 install: card_tests and
  planner failed on it, flat at 176,000 B after 2000 ticks for the planner's 6-tuples; the PC's 3.14 does not show it).
  `ti_mem_check.measure` reads each window after a DEBUG_SAVEALL full collection (free lists emptied, cycles kept as growth);
  guarded by `test_no_per_frame_leaks.py` test_guard_does_not_count_memory_a_full_collection_returns_without_finding_garbage.
- PC tests: the venv runs Python 3.14.7. Code must run on 3.11 (no 3.12+ syntax or stdlib); gc and tracemalloc numbers differ between
  versions, so the install check re-runs the leak guard on the device (comma_checks.sh stage 6).

## Never written to logs, params or reports
- GPS positions, place names, the VIN. Map lists are matched inside ti_curve_plannerd only.

## Generated files (rebuild when the sources change; a stale file fails silently on the car)
- `selfdrive/car/mazda/ti_curve_approach.npz`: built by `tools/dev/build_approach_table.py`; keyed by `approach_src_hash`
  (`ti_curve_plan.py:237-245`: ti_cruise_model.py, ti_cruise_taps.py, ARRIVED_MPH, TAP_S, HOLD, APPROACH_DT). Stale -> not loaded
  (`ti_curve_plan.py` load_approach_table returns 0) and every approach is simulated on the device. Guarded by
  `test_mazda_ti_curve_approach_table.py:8`.
  Any edit to ti_cruise_model.py or ti_cruise_taps.py, a comment included, changes the hash: rebuild the table in the same change
  (both files are exempt from the RUNTIME header for this reason: `per_frame_inventory.json` header_exempt).
- `ti_ident.py:251,264` TABLE_HASH / SIGNAL_HASH: computed at import from the tables (identify a test version in logs; changing a
  table changes the hash).

## Checks that must pass for a change to per-frame code
- `selfdrive/car/tests/test_no_per_frame_leaks.py` (cyclic garbage 0, retained growth, cost growth) and
  `selfdrive/car/tests/test_lint_per_frame.py`, both in `tools/dev/pc_test.py --curve` and `--all`.
- On the comma at install: `tools/dev/comma_checks.sh` (cost, params sync, planner dry run, memory guard `ti_mem_check.py --quick`).

## Replayability (owner, 2026-11-25: "Let's make sure we're properly logging information about our features so that we can enhance them after the fact")
- Every feature logs the inputs, the state and the decision needed to replay it from a drive log: what is in an rlog message
  already (carState, carControl, carOutput, controlsState, modelV2, frogpilotPlan, can) need not be logged again; anything else is.
- Curve planner: tick record (`ti_curve_plan` kind "tick", 5 Hz) incl. "cs" (car side's ref / floor / acting as given to the planner),
  "mpr" + `map_list` records (the map list exactly as used), "ask"; replay helper `ti_curve_plannerd.logged_inputs`. Schema:
  `PLANNER_STEP_INPUTS`, `PLANNER_STATE`.
- Car side: `ti_curve` kind "cs" (`Curve.diag`, 5 Hz while on or active), `drv_btn` (every driver cruise-button press, on or off),
  `deliver_phase` (stage 3 phase, timers, stock-assist flags, on change at 10 Hz), `left` (TiSetSpeedLeft on change). Schema:
  `ti_curve.DIAG_STATE`, `DIAG_EXEMPT`, `DIAG_INPUTS`, `INPUTS_IN_RLOG`.
- Guarded by `selfdrive/car/tests/test_mazda_ti_curve_replayable.py`: a new Planner / Curve attribute or input that is neither
  logged nor exempt with a reason fails it.
- No positions, place names or VIN in any log (distances along a path, speeds, curvatures, flags only); every record bounded
  (tick <= TICK_MAX_B + TICK_REPLAY_MAX_B, map_list <= ~7 KB once per list, diag ~0.5 KB at 5 Hz).
