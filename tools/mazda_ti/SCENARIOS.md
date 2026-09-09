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

Two scenarios are intentionally the complete public fixture set:

- `reference_transitions` covers tightening, unwind, sign reversal, sustained
  clipping, inactive-to-active reset, recovery, and retained state carryover.
- `known_570_defect` passes the same controller and limiter sequence through the
  fixed historical 570-count compensation mapping. Its independent observable is
  a stronger raw request with no stronger final limited command. It must fail.

The 570 fixture is a deterministic regression model only. It does not infer any
TI watchdog, actuator availability, or physical steering response. Neither
scenario proves vehicle handling or an acceptable corner path.
