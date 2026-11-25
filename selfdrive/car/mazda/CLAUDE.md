# selfdrive/car/mazda: agent notes (runtime constraints)

`carcontroller.py`, `carstate.py`, `interface.py` and the `ti_*.py` modules they call run in card at 100 Hz with the cycle
collector OFF (`selfdrive/car/card.py:260`); `ti_ab.py` / `ti_plan.py` / `ti_tests.py` also in controlsd (gc off);
`ti_curve_plannerd.py` / `ti_curve_plan.py` / `ti_curve_memory.py` at 5 Hz with gc frozen after load. Each module's `# RUNTIME:`
header says which. Read `docs/RUNTIME_CONSTRAINTS.md` first.
- No class / closure per call; no container without a bound; event buffers trimmed (`ti_ab.py`, `ti_ident.py` _event).
- Params while driving: tmpfs, at most 10 Hz; flash at most once a minute; a new key also in `common/params.cc`.
- Never a position, place name or the VIN in a log, param or report.
- `ti_curve_approach.npz` is generated: rebuild with `tools/dev/build_approach_table.py` when ti_cruise_model.py, ti_cruise_taps.py
  or the approach settings change.
- Install-time checks on the comma: `tools/dev/comma_checks.sh` (quick, device-only; no replays).

Before a change here is done (run from a working directory OUTSIDE the repo; plain pytest fails):
- `<venv python> tools/dev/pc_test.py --curve -n 2` (includes `selfdrive/car/tests/test_no_per_frame_leaks.py` and
  `selfdrive/car/tests/test_lint_per_frame.py`); `--all -n 2` before merging. 0 failed.
- `python tools/dev/lint_per_frame.py`: 0 unwaived hits. A waiver `# per-frame-ok: <the bound, and where it is>` only after you
  have verified the bound.
- A new per-frame module: classify it in `tools/dev/per_frame_inventory.json`, give it the `# RUNTIME:` header, add a scenario to
  `selfdrive/car/mazda/ti_mem_check.py`.
