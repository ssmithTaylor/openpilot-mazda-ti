# On-comma checks: only what the PC cannot answer

Run by `tools/dev/install_comma.sh` after the build (or by hand: `bash tools/dev/comma_checks.sh --base <old commit>`).
Target: under 3 minutes after the build. Every pure-Python logic test runs on the PC (`python tools/dev/pc_test.py --curve`
or `--changed`) on the same commit before the push; on 2026-11-23 those took 2.6 minutes on the comma.

| Check | Kept / dropped | Why |
|---|---|---|
| Build (`scons -j$(nproc) --cache-populate --minimal`, login shell) | kept (install step) | only the device toolchain builds the device binaries; warnings are errors (`-Wshadow`) |
| `selfdrive/car/mazda/ti_cost_check.py` | kept | per-cycle cost is device-specific (PC understates ~10x); budget 300 us/cycle |
| `frogpilot/tools/check_params_sync.py` | kept | needs the compiled params extension, stubbed on the PC |
| `selfdrive/controls/lib/tests/scalar_np_evidence.py` | kept only when `selfdrive/controls/lib/` or `selfdrive/car/mazda/ti_*` changed | it measures device numerics of the controller paths; nothing else changes its answer |
| `ti_curve_dryrun.py` (with `--rlog`) or `tools/dev/profile_curve_plan.py` | kept | planner tick time on the device's CPU with the installed memory; PC timings understate it |
| Smoke import of changed non-test modules | kept | catches a module that imports a device-only package; seconds |
| Curve unit / plan / e2e / scenario tests | dropped | pure Python, deterministic: same answer on the PC in ~16 s (`pc_test.py --curve`) |
| State-machine and other controls tests | dropped | pure Python; PC-runnable ones run in `pc_test.py --all`; `test_state_machine.py` / `test_alerts.py` import controlsd (POSIX-only `os.register_at_fork`, tinygrad) and are listed in `PC_EXCLUDED`: run them on the comma by hand only when `selfdrive/controls/` changed |
| Long suites (`slow_on_device`) | dropped | PC runs the full suites once on the final candidate |

Tests the PC still cannot run (in `pc_test.py` `PC_EXCLUDED`): `selfdrive/controls/tests/test_alerts.py`,
`test_state_machine.py` (controlsd import chain needs POSIX `os.register_at_fork` and a real tinygrad), and upstream
`selfdrive/controls/lib/tests/test_latcontrol.py`, `test_vehicle_model.py` (they call `CarInterface.get_params` without
FrogPilot's toggles; they fail the same way anywhere, not a PC setup fault).
