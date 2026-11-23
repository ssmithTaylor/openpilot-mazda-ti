# Review procedure for on-car features (one page)

Why: on 2026-11-23 four fix rounds on curve slowing cost ~45 minutes each, ~10 of it fixing. The rest was repeated fixed
cost: full cold-start reviews, acceptance runs, full suites, manual installs. This procedure keeps a fix round in minutes.

## Rules

1. **Diff-scoped, time-boxed.** A fix-round review reads only `git diff <last reviewed commit>..HEAD` and stops at 15 minutes.
2. **One full review per install candidate.** The full (cold-start) review, `accept.py` full mode, `pc_test.py --all` and the
   memory rebuild run once, on the final candidate.
3. **A finding comes with a runnable failing case.** It is added to `selfdrive/car/tests/test_mazda_ti_curve_scenarios.py`
   (or the feature's own scenario file) in the same message. No case, no finding (say "unverified concern" instead).
4. **A fix is verified by running that case**: `python tools/dev/pc_test.py -- <test file>::<case>`, then
   `pc_test.py --changed`. Not by another review.
5. **Scope freezes when the review starts.** New findings and owner requests go on the next-install list unless they are
   safety faults in what is about to be installed.
6. **One fault per fix round.** Failing test first, fix, rerun.

## Loop

`edit -> pc_test.py --changed (~20 s) -> commit -> diff review (<=15 min) -> [final candidate only: full review, accept, --all]
-> owner's yes -> install_comma.sh --owner-approved "<words>"` (comma checks < 3 min after the build).

## Reviewer brief (paste; fill the <>)

```
Review ONLY this diff: git -C <worktree> diff <base>..<head>   (feature: <name>; on the car now: <commit>)
Time box: 15 minutes. Report what you found at the end of it.
Read first: tools/mazda_sim/RULES.md (standing rules). No positions, place names or VIN in any file.
Do not push, do not touch the car, GitHub or the cluster. Never weaken or delete a test.
Run the standing check first: python tools/dev/pc_test.py --changed --base <base>
For each finding:
  - severity: SAFETY (blocks install) / MUST / LATER
  - a runnable failing case added to selfdrive/car/tests/test_mazda_ti_curve_scenarios.py
    (use its e2e() harness; synthetic roads only), committed on branch <review branch>
  - the exact command that shows it failing
Out of scope: anything not in the diff; style; ideas for later (list them in one line each).
Scope is frozen: do not ask for new features.
Reply in at most 25 lines: findings with test names, commands, and what you did not get to.
```

## Builder brief (paste; fill the <>)

```
Fix ONE fault: <fault, one sentence>. Failing case: <test node id> (or write it first if none exists).
Worktree/branch: <path> / <branch>. Time box: 15 minutes; report what is committed at the end of it.
Read first: tools/mazda_sim/RULES.md. No positions/place names/VIN. No pushes; never touch the car, GitHub, the cluster.
Never weaken or delete a test.
Steps: 1) run the case, see it fail  2) fix  3) python tools/dev/pc_test.py -- <test file>::<case>
4) python tools/dev/pc_test.py --changed  5) commit (one commit, message names the fault and the case).
Do not fix anything else; list other problems you see in one line each.
```
