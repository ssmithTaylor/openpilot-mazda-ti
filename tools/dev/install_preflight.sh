#!/usr/bin/env bash
# Pre-flight for tools/dev/install_comma.sh (review I3, 2026-11-24): every script the install will run LATER (on the PC and on the
# comma) must be tracked in the commit being installed, or nothing is pushed. Read-only: it only asks git about <rev>.
#   bash tools/dev/install_preflight.sh <rev>
# Exit 0 when all are present; 1 (naming each missing one) otherwise.
set -euo pipefail
REV="${1:?usage: install_preflight.sh <rev>}"
cd "$(dirname "$0")/../.."
# install_comma.sh runs: pc_test.py (stage 1), comma_checks.sh (stage 6); comma_checks.sh runs the rest
NEEDED=(
  tools/dev/pc_test.py
  tools/dev/comma_checks.sh
  selfdrive/car/mazda/ti_cost_check.py
  frogpilot/tools/check_params_sync.py
  selfdrive/controls/lib/tests/scalar_np_evidence.py
  selfdrive/car/mazda/ti_curve_dryrun.py
  tools/dev/profile_curve_plan.py
)
MISSING=0
for f in "${NEEDED[@]}"; do
  if ! git cat-file -e "$REV:$f" 2>/dev/null; then
    echo "pre-flight: $f is not in $REV (untracked or ignored?): the install would fail after pushing"
    MISSING=1
  fi
done
[ $MISSING = 0 ] && echo "pre-flight: all ${#NEEDED[@]} scripts the install runs are in $REV"
exit $MISSING
