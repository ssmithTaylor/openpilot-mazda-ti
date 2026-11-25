# Briefing and review checklist for on-car code (coordinators)

Every builder brief for code that runs on the car states:
1. The process and rate the code runs in, and its collector state (from `tools/dev/per_frame_inventory.json`: controlsd and card
   100 Hz gc OFF; ti_curve_plannerd 5 Hz gc frozen). New per-frame code: which process, and the inventory entry to add.
2. "Read `docs/RUNTIME_CONSTRAINTS.md` and the `CLAUDE.md` of the directory first."
3. The time budget there and where it is checked (cost check 300 us/cycle; planner tick p99 20 ms).
4. The checks that must pass: `tools/dev/pc_test.py --curve -n 2` (includes the leak tests and the per-frame lint) and `--all -n 2`
   before merge, run from a working directory outside the repo; `tools/dev/lint_per_frame.py` with 0 unwaived hits.
5. What is a device-only check (comma_checks.sh: quick, no replays, never alongside the collector) and what runs on the PC.
6. Never a GPS position, place name or the VIN in code, logs, params, fixtures or the report.
7. A new per-frame entry point gets a scenario in `selfdrive/car/mazda/ti_mem_check.py` whose input sequence starts and ends every
   state it keeps.

Every review brief asks, explicitly, for each function that runs per frame / tick:
- What does this leave behind per call? (a class, closure, bound method, stored exception, a reference to the frame's message)
- What grows with drive time? (lists, dicts, sets, caches, strings appended to, event buffers: where is the bound?)
- What does it cost per call, and does the cost grow as the drive goes on?
- Is anything read or written to params / disk / JSON every frame, or does anything block?
- Does a waiver (`# per-frame-ok:`) name a bound the reviewer has checked?
- Was it tested with the collector off (`gc.disable()`), as card and controlsd run?

Rule for new constraints: a constraint discovered during a task (from a drive, a review, a failure) is written into
`docs/RUNTIME_CONSTRAINTS.md` with its file:line AND encoded as a test or a lint rule in the same change. A constraint that exists
only in a brief, a memory file or a coordinator's head will be broken by the next builder.
