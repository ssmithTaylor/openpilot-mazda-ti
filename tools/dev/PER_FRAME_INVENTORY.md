# Per-frame inventory (code this fork added that runs every frame / tick)

Machine-readable source: `tools/dev/per_frame_inventory.json` (read by `tools/dev/lint_per_frame.py`, which fails when a listed
module lacks its `# RUNTIME:` header or a new `ti_*.py` / `selfdrive/controls/lib/*.py` imported by a per-frame entry point is not
classified there). Code facts only. Constraints per process: `docs/RUNTIME_CONSTRAINTS.md`.

## Processes

| process | rate | collector | where |
|---|---|---|---|
| controlsd | 100 Hz (`controlsd.py:214` Ratekeeper(100)) | **disabled** | `controlsd.py:1181` config_realtime_process -> `common/realtime.py:41` gc.disable() |
| card | 100 Hz (`card.py:107` Ratekeeper(100)) | **disabled** | `card.py:260` config_realtime_process -> `common/realtime.py:41` |
| ti_curve_plannerd | 5 Hz (`ti_curve_plannerd.py:27` RATE_HZ) | on, **frozen** after load | `ti_curve_plannerd.py:469` gc.freeze(); cores 0-3 (`:423` keep_off_control_core) |
| frogpilot_process | 20 Hz (`frogpilot_process.py:55` Ratekeeper(1 / DT_MDL)); on-road branch every modelV2 while started, engaged or not (`:112-116`) | **disabled** | `frogpilot_process.py:57` config_realtime_process -> `common/realtime.py:41` |
| ti_mcp (frogpilot/system/ti_mcp) | on request (HTTP), not per frame | on | `system/manager/process_config.py:78`, only with TiMcpEnabled |
| ui (C++ Qt: `selfdrive/ui/qt/ti_pages_logic.h`, `sidebar.cc`) | UI loop (upstream; rate not verified here) | n/a (C++) | not covered by the Python guards |

## Entry points and their leak tests (`selfdrive/car/tests/test_no_per_frame_leaks.py`, scenarios in `selfdrive/car/mazda/ti_mem_check.py`)

| process | call | at | scenario |
|---|---|---|---|
| controlsd | LaneDriftGate.update / lane_change_active / reset | `controlsd.py:850,864` | lane_drift |
| controlsd | TiAbRunner.step (Controls.ab_step) | `controlsd.py:269` | ab_runner |
| controlsd | LatControlTorque.update_model_context / update with the TI plant | `controlsd.py:804-814` | lateral |
| controlsd | lateral_diagnostics.record_inputs | `controlsd.py:817` | lateral |
| card | CarController.update: ti_tests, ti_plan, ti_ident, ti_dither, ti_stuck_guard, ti_lag, stats, Flag This Moment, Start A New Measurement | `card.py:221` CI.apply | card_tests |
| card | curve_step / _curve_monitor: ti_curve, ti_curve_monitor, ti_curve_demand, ti_cruise_model, ti_cruise_taps | `carcontroller.py:922,961` | card_curve |
| card | button_tests_step / cruise_check_step: ti_button_tests, ti_cruise_check | `carcontroller.py:783,862` | card_buttons |
| controlsd, card | HookTimers.add / gauge_max / tick (ti_cost_timers) | `controlsd.py` state_control, controlsd_thread; `card.py` controls_update, card_thread; `carcontroller.py` update, curve_step | card_tests, card_curve, card_buttons |
| card | CC.ti_lag_ctl set from controlsState | `card.py:217-220` | card_tests |
| card | CarState.update_gen1 TI fields (needs the compiled CAN parser) | `carstate.py:64` | none (gap) |
| controlsd | LatControlNNFF.update (NNFF toggle on only; changed by this fork) | `controlsd.py:303-314` | none (gap) |
| frogpilot_process | FrogPilotEvents.update: LaneDriftGate.update / lane_change_active, TI dropout, running wide, latch | `frogpilot_events.py:262` | frogpilot_events |
| frogpilot_process | FrogPilotPlanner.update TI block: steer_authority predicted_drift_m / AdvisoryTrigger.update, ti_curve curve_alert_mph / read_curve_code | `frogpilot_planner.py:122-162` | frogpilot_events |
| ti_curve_plannerd | tick: can_reads, car_side, map_input_ref / map_profile_ref, map_list_log, fresh_model, fresh_limit, Planner.step, plan_json, log_payload, tick_log | `ti_curve_plannerd.py:480-` | planner |

## Modules

controlsd: lane_drift.py, latcontrol_torque.py, scalar_np.py, lateral_plant.py, lateral_reference.py, lateral_diagnostics.py,
friction_release.py, ti_ab.py. controlsd and card: ti_plan.py, ti_tests.py, ti_cost_timers.py (per-hook cost, summary event `ti_cost` every 10 s). card: carcontroller.py, carstate.py, interface.py,
ti_dither.py, ti_stuck_guard.py, ti_ident.py, ti_lag.py, ti_cruise_check.py, ti_cruise_taps.py, ti_button_tests.py,
ti_curve_monitor.py. card and ti_curve_plannerd: ti_curve.py, ti_curve_demand.py, ti_cruise_model.py. ti_curve_plannerd:
ti_curve_plan.py, ti_curve_plannerd.py, ti_curve_memory.py. controlsd (NNFF toggle): neural_network_feedforward.py.
frogpilot_process: frogpilot_planner.py, frogpilot_events.py, steer_authority.py, and lane_drift.py / ti_curve.py (shared above).

Not per frame (classified): ti_cost_check.py, ti_mem_check.py, ti_curve_dryrun.py (install checks), ti_button_tests_report.py,
ti_drive_plan.py (offline), the upstream `selfdrive/controls/lib` modules, and the upstream FrogPilot `frogpilot/controls` modules
(frogpilot_card.py, conditional_experimental_mode.py, frogpilot_acceleration.py, frogpilot_following.py, frogpilot_vcruise.py,
weather_checker.py: unchanged from the snapshot).

## Bounded state found by reading (lint waivers / harness drains)

- `ti_ab.TiAb.events` trimmed 4000 -> 2000 (`ti_ab.py` _event); heartbeat every 10 s: full after about 11 h, about 1-3 MB.
- `ti_ident.TiIdent.events` trimmed 2000 -> 1000 (`ti_ident.py` _event).
- Memo caches keyed by finite domains: ti_ident `_STEPS_IN_BAND`, `_STEP_PARAMS`, `_SMALL`, `_SMALL_FLAGS`, `_PEAK`, `_REP`;
  ti_dither `_PEAK_ALL`; ti_curve_plan `_APPROACH` (whole-mph keys, ~23k preloaded, flat at 22722 in the 2 h soak) and `_RAISE_THR`
  (set speed from the CAN signal x whole-mph target; flat at 5 in the soak).
- carcontroller TiFlaggedMoments / TiTuningStatsHistory lists: written only on a tap, trimmed to FLAG_HISTORY / RUN_HISTORY.
