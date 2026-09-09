# Mazda isolated startup evaluation

The required profile cold-starts the real `controlsd` `ManagerProcess` through
`selfdrive.test.process_replay.replay_process`. It publishes a bounded recorded
Mazda input stream through the configured messaging services and observes
`controlsState`, `carControl`, and `onroadEvents`. It does not start `card` or
`pandad`, construct a CAN publisher, connect to a vehicle, or use live Params.

Run it from a built Linux openpilot checkout. Repeat `--rlog` for any earlier
file needed to supply the route's recorded `carParams`; the last file is the
bounded startup stream. This example replays twenty recorded `carState` cycles:

```text
PARAMS_ROOT=/tmp/mazda-startup-params \
python -m tools.mazda_ti.startup_eval \
  --profile full_process \
  --rlog PATH/TO/ROUTE-START/rlog \
  --rlog PATH/TO/STARTUP-SEGMENT/rlog \
  --max-carstate-messages 20 \
  --output NEW-EVIDENCE
```

The Linux checkout must import `msgq.ipc_pyx`, `common.params_pyx`, both
`opendbc.can` extensions, `common.transformations.transformations`,
`selfdrive.pandad.pandad_api_impl`, `msgq.visionipc.visionipc_pyx`, and this
fork's tinygrad `commonmodel_pyx`. A normal openpilot build supplies these.
When a Windows checkout is copied into Linux, restore Git mode-120000 entries
as symlinks before building; the checkout's text placeholders are not usable
Linux packages. The result reports missing imports as unsupported rather than
claiming completion.

The full profile uses process replay's explicit Mazda fingerprint mode, the
exact recorded `CarParams`, an isolated schema-default `FrogPilotCarParams`,
repository-default FrogPilot toggles, and an isolated enabled-TI setting so the
Mazda plant diagnostics are constructed. These values exist only below the
temporary replay Params prefix. No safety process consumes them.

A completed result requires all emitted `carControl` messages to remain
inactive with zero steering, and rejects any observed `can` or `sendcan`
service. The report hashes each input and all relevant process/schema sources.
It captures process stdout/stderr and exceptions. Timing is local replay wall
time; it is not full-system or device scheduling evidence.

Timestamp qualification comes from the generated
`controlsState.lateralControlState.torqueState.mazdaDiagnostics.inputs` rows.
Every seen nested service/time identity must exactly match a recorded input.
Every rewritten outer `controlsState` time must match an actual replay trigger
plus the configured four-millisecond processing offset. Invented timestamp rows
cannot satisfy this profile.

The narrower schema check remains available:

```text
python -m tools.mazda_ti.startup_eval \
  --profile schema_boundary \
  --output NEW-EVIDENCE
```

It sends one zero-speed `carState` message through the dedicated subscriber in
a unique fake messaging prefix. The source fixture rejects the known failure
where `carState` is incorrectly added to controlsd's `SubMaster`. This profile
does not satisfy the full-process requirement.

On Windows, or in an unbuilt Linux checkout, the command writes an explicit
`unsupported` result. A failed process writes `failed_execution`, the complete
traceback, and captured child output. Neither status satisfies the required
profile. Completed process replay establishes isolated software startup and
inactive command behavior only; it says nothing about CAN delivery, steering
hardware, scheduling on a comma device, or physical vehicle handling.
