# selfdrive/controls/lib: agent notes (runtime constraints)

Most modules here run inside controlsd at 100 Hz with the cycle collector OFF (see `docs/RUNTIME_CONSTRAINTS.md`). Ours:
`lane_drift.py`, `latcontrol_torque.py` (TI plant branch), `scalar_np.py`; each carries a `# RUNTIME:` header.
- No class or closure defined inside a function that runs per frame (`lane_drift._with_line_probs` did, 2026-11-25).
- Every container has a visible bound (deque maxlen, trim, or a finite key domain named in a waiver).
- Nothing grows with drive time; nothing blocks.
- A new module here that controlsd imports must be classified in `tools/dev/per_frame_inventory.json` (the lint fails otherwise).

Before a change here is done (run from a working directory OUTSIDE the repo; plain pytest fails):
- `<venv python> tools/dev/pc_test.py --curve -n 2` (includes `selfdrive/car/tests/test_no_per_frame_leaks.py` and
  `selfdrive/car/tests/test_lint_per_frame.py`); `--all -n 2` before merging. 0 failed.
- `python tools/dev/lint_per_frame.py`: 0 unwaived hits. A waiver `# per-frame-ok: <the bound, and where it is>` only after you
  have verified the bound.
- A new per-frame module: classify it in `tools/dev/per_frame_inventory.json`, give it the `# RUNTIME:` header, add a scenario to
  `selfdrive/car/mazda/ti_mem_check.py`.
