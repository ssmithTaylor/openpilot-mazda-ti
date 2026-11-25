#!/usr/bin/env bash
# Runs ON THE COMMA after the build, before the reboot. Only what the PC cannot answer (see tools/dev/COMMA-CHECKS.md).
#   bash tools/dev/comma_checks.sh [--base <commit before the pull>] [--rlog <rlog> ...]
# Exit 0 only when every check passes. Stops at the first failure.
set -euo pipefail
cd "$(dirname "$0")/../.."
BASE=""; RLOGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --base) BASE="$2"; shift 2;;
    --rlog) shift; while [ $# -gt 0 ] && [[ "$1" != --* ]]; do RLOGS+=("$1"); shift; done;;
    *) echo "unknown argument $1"; exit 2;;
  esac
done
export PYTHONPATH="${PYTHONPATH:-/data/pythonpath}"
T0=$(date +%s)
stage() { echo; echo "=== [comma_checks $(( $(date +%s) - T0 )) s] $*"; }
# readiness line at the end of every install, pass or fail: free disk on /data, memory used (WARNING under 20 % free disk)
trap 'python3 selfdrive/car/mazda/ti_mem_check.py --readiness || true' EXIT
changed() { [ -n "$BASE" ] && git diff --name-only "$BASE" HEAD | grep -qE "$1"; }

stage "1/6 cost check (budget 300 us/cycle; device timing)"
python3 selfdrive/car/mazda/ti_cost_check.py

stage "2/6 params sync (compiled params extension)"
python3 frogpilot/tools/check_params_sync.py

stage "3/6 scalar/numpy evidence (only when its inputs changed)"
if [ -z "$BASE" ] || changed '^(selfdrive/controls/lib/|selfdrive/car/mazda/ti_|selfdrive/controls/lib/tests/scalar_np_evidence.py)'; then
  python3 selfdrive/controls/lib/tests/scalar_np_evidence.py
else
  echo "skipped: no change under selfdrive/controls/lib or selfdrive/car/mazda/ti_* since $BASE"
fi

stage "4/6 curve planner dry run (tick budget on the device)"
if [ ${#RLOGS[@]} -gt 0 ]; then
  python3 selfdrive/car/mazda/ti_curve_dryrun.py --rlog "${RLOGS[@]}"
else
  python3 tools/dev/profile_curve_plan.py --memory /data/ti_curve/curve_memory_v1.npz --traces 3
fi

stage "5/6 smoke import of changed processes"
CHANGED_PY=$(git diff --name-only "${BASE:-HEAD~1}" HEAD -- '*.py' || true)
MODS=$(echo "$CHANGED_PY" | grep -vE '(^|/)test_|/tests/|^tools/' | sed 's/\.py$//; s#/#.#g' || true)
# the real process entry points too, whenever anything under them changed (a module can import fine alone and still break them)
if echo "$CHANGED_PY" | grep -qE '^(selfdrive/car/|selfdrive/controls/|frogpilot/)'; then
  MODS="$MODS selfdrive.car.card selfdrive.controls.controlsd frogpilot.frogpilot_process"
fi
# each import as the process manager does it: openpilot.system.sentry first (system/manager/process.py imports it before any
# process module), else frogpilot_variables / card report a circular import that never happens on the car
for m in $(echo $MODS | tr ' ' '\n' | awk '!seen[$0]++'); do
  [ -f "${m//.//}.py" ] || continue
  python3 -c "import importlib; import openpilot.system.sentry; importlib.import_module('openpilot.$m')" && echo "ok   $m" || { echo "FAIL $m"; exit 1; }
done

stage "6/6 per-frame memory guard (gc off, ~20-30 s: cyclic garbage, retained growth; thresholds as the PC tests)"
python3 selfdrive/car/mazda/ti_mem_check.py --quick

stage "all comma checks passed"
