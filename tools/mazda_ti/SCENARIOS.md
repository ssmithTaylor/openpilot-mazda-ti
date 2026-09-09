# Synthetic controller scenarios

Run a versioned, bounded scenario request with a new output directory:

```bash
python -m tools.mazda_ti.scenarios --request scenario.json --output evidence/scenario
```

`candidate_revision`, `baseline_controller`, and `limiter` must each be resolved,
full 40-character Git commits. The adapter reads the controller source and the
`apply_ti_steer_torque_limits` implementation from those commits, runs the real
controller with a fixed Mazda plant interface, and applies the real limiter. It
does not load Params, start a process, publish CAN, or touch a vehicle.

The result uses the evaluation result's top-level status, identity, source hash,
runtime, scope, limitation, and artifact fields. `comparison.command_differences`
is informational. `comparison.hard_invariant_failures` controls `failed_check`.
The report retains all frames, including recovery and carryover, after a failure.
Elapsed time and the absolute output destination live in `execution.json`; that
machine-specific file is excluded from the canonical artifact hash map.

Choose the fixture that exercises the proposed change:

- `reference_transitions` covers tightening, unwind, sign reversal, sustained
  clipping, inactive-to-active reset, recovery, and retained state carryover.
- `known_570_defect` injects a post-controller clip at 570 counts. It must fail
  as a negative control; it does not execute the production compensation law.
- `feedback_transitions` includes smooth fixed ramps through the compensation
  band and couples the previous simulated apply to the controller. Each update
  consumes the prior limiter eligibility, then computes next-cycle eligibility
  against three earlier returned requests before appending the current request.
  This is a declared synchronous schedule, not a recovered message-receipt history.
- `reference_release` settles each direction, steps request and prescribed
  measurement to zero, and retains reversal/recovery. Compensation and damping
  are disabled; integration is frozen at zero. A third arm runs the identical
  candidate with commitment disabled. Its native feedforward/setpoint traces
  isolate reference persistence from compensation and integral carryover.

`reference_transitions` and `known_570_defect` force limiter eligibility false;
they cannot qualify a change to feedback-dependent integration. All fixtures
keep vehicle observations fixed independently of software commands. They do not
simulate steering motion or transport latency.

Each bundle saves both reference and candidate traces. Trace fields include the
actual returned normalized `steer`, consumed prior feedback, limiter eligibility,
native integral and feedforward in m/s², and inverse/compensation commands in TI
counts. Inactive native control terms are unavailable. `reference_release` also
saves `commitment-disabled-trace.json` and per-zero-step persistence metrics:
first zero crossing, first old-sign contribution at/below 0.01 m/s² or 1 count,
peak old-sign inverse counts and integrated count-seconds. A null crossing time
means it was not reached in that phase. These measures characterize software
state; retained old-sign torque can be appropriate with real delayed motion.

The 570 fixture is a deterministic regression model only. It does not infer any
TI watchdog, actuator availability, or physical steering response. No scenario
proves vehicle handling or an acceptable corner path.

## Inspect the actual compensation formula

Run the separate expression probe when changing compensation headroom:

```text
python -m tools.mazda_ti.compensation_map --revision FULL_40_CHARACTER_COMMIT --output evidence/compensation-map
```

The probe extracts the four production compensation expressions immediately
before the recognized withdrawal call, from the pinned controller source. It
rejects changed expression boundaries or additional dependencies. It evaluates
commands from -600 to +600 in 0.1-count steps at five positive, zero and negative
gate values, including opposing gate/command signs. Its responsiveness check
fails flat or reversed full-precision response intervals and nonfinite or
out-of-envelope results. Integer quantization may still repeat adjacent values;
strict response is not required at every 0.1-count wire step.

`result.json` preserves source revision, source/AST/helper hashes, runtime,
declared grid, metrics and failures. Output directories are immutable. The
qualification is `isolated_compensation_expression`, separate from full
controller/limiter scenarios and release qualification. The available-authority,
enabled branch is isolated before withdrawal; this omits PID evolution, limiting,
message receipt and the firmware's unknown stuck-request timing/tolerance.
Follow it with feedback scenarios and qualified recorded-drive comparisons.
