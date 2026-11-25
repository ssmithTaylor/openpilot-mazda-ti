# selfdrive/controls: agent notes (runtime constraints)

`controlsd.py` runs at 100 Hz with the cycle collector OFF (`controlsd.py:1181` config_realtime_process -> `common/realtime.py:41`
gc.disable()). Read `docs/RUNTIME_CONSTRAINTS.md` first. In anything controlsd calls every cycle:
- create no class, closure or bound method per call (2026-11-25: a class per call leaked 1-2 GB per engaged hour);
- keep no list / dict / set without a bound, and no reference to a message past its frame;
- no blocking I/O; params and JSON at a fraction of the rate, never every cycle;
- our per-cycle code here: the lane-drift gate, TiAbRunner.step, the TI lateral controller, record_inputs.

Before a change here is done (run from a working directory OUTSIDE the repo; plain pytest fails):
- `<venv python> tools/dev/pc_test.py --curve -n 2` (includes `selfdrive/car/tests/test_no_per_frame_leaks.py` and
  `selfdrive/car/tests/test_lint_per_frame.py`); `--all -n 2` before merging. 0 failed.
- `python tools/dev/lint_per_frame.py`: 0 unwaived hits. A waiver `# per-frame-ok: <the bound, and where it is>` only after you
  have verified the bound.
- A new per-frame module: classify it in `tools/dev/per_frame_inventory.json`, give it the `# RUNTIME:` header, add a scenario to
  `selfdrive/car/mazda/ti_mem_check.py`.
