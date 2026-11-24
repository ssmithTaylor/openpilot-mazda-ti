#!/usr/bin/env bash
# The whole comma install as one command, run ON THE PC (Git Bash). THIS PUSHES <branch> TO origin/main AND REBOOTS THE COMMA.
# The assistant running it needs the owner's yes EACH TIME, quoted in --owner-approved.
#
#   bash tools/dev/install_comma.sh --owner-approved "<the owner's words>" [--branch <branch>] [--memory <file.npz>] [--rlog <path on comma>] [--dry-run]
#
# Stages (stop on first failure; the collector is ALWAYS resumed by the exit trap):
#   1 preconditions: clean tree, branch is a fast-forward of origin/main, PC tests green (pc_test.py --changed)
#   2 pause the collector      3 push <branch>:main      4 comma: git pull --ff-only      5 comma: build (login shell)
#   6 comma: comma_checks.sh   7 optional road memory copy (previous kept as .prev)       8 reboot, wait, verify commit + processes
set -euo pipefail
DEVICE="comma@192.168.15.100"; KEY="$HOME/.ssh/comma_key"; REMOTE_DIR="/data/openpilot"
KCTL=(kubectl --context default -n utilities)
BRANCH=""; MEMORY=""; RLOG=""; APPROVED=""; DRY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --owner-approved) APPROVED="$2"; shift 2;;
    --branch) BRANCH="$2"; shift 2;;
    --memory) MEMORY="$2"; shift 2;;
    --rlog) RLOG="$2"; shift 2;;
    --dry-run) DRY=1; shift;;
    *) echo "unknown argument: $1"; exit 2;;
  esac
done
cd "$(dirname "$0")/../.."
BRANCH="${BRANCH:-$(git rev-parse --abbrev-ref HEAD)}"
if [ -z "$APPROVED" ]; then
  echo "REFUSED: --owner-approved \"<the owner's words>\" is required. This pushes $BRANCH to origin/main and reboots the comma."; exit 2
fi
echo "This pushes $BRANCH to origin/main and REBOOTS the comma. Owner's words: \"$APPROVED\"  (dry run: $DRY)"
SSH=(ssh -i "$KEY" -o BatchMode=yes -o ConnectTimeout=10 "$DEVICE")
SCP=(scp -i "$KEY" -o BatchMode=yes)
T0=$(date +%s)
stage() { echo; echo "=== [install $(( $(date +%s) - T0 )) s] $*"; }
run() { if [ $DRY = 1 ]; then echo "DRY: $*"; else "$@"; fi; }
remote() { if [ $DRY = 1 ]; then echo "DRY: ssh $DEVICE $*"; else "${SSH[@]}" "$@"; fi; }

PAUSED=0
resume() {
  if [ $PAUSED = 1 ]; then echo; echo "=== resume the collector (always)"; run "${KCTL[@]}" scale deploy/comma-logs-collector --replicas=1 || echo "WARNING: collector NOT resumed: run it by hand"; fi
}
trap resume EXIT

stage "1 preconditions"
[ -z "$(git status --porcelain)" ] || { echo "working tree not clean"; exit 1; }
bash tools/dev/install_preflight.sh "$BRANCH" || { echo "pre-flight failed: nothing pushed"; exit 1; }   # before anything is pushed
run git fetch origin main
git merge-base --is-ancestor origin/main "$BRANCH" || { echo "$BRANCH is not a fast-forward of origin/main"; exit 1; }
NEW=$(git rev-parse "$BRANCH"); echo "installing $NEW"
PY="${PC_PYTHON:-python}"
run "$PY" tools/dev/pc_test.py --changed --base origin/main
[ -z "$MEMORY" ] || [ -f "$MEMORY" ] || { echo "memory file not found"; exit 1; }

stage "2 pause the collector"
run "${KCTL[@]}" scale deploy/comma-logs-collector --replicas=0; PAUSED=1

stage "3 push $BRANCH:main"
run git push origin "$BRANCH:main"

stage "4 comma: pull"
if [ $DRY = 1 ]; then OLD="<commit on the comma before the pull>"; else OLD=$(remote "cd $REMOTE_DIR && git rev-parse HEAD" || echo ""); fi
remote "cd $REMOTE_DIR && git pull --ff-only"

stage "5 comma: build (login shell)"
remote "cd $REMOTE_DIR && bash -lc 'scons -j\$(nproc) --cache-populate --minimal'"

stage "6 comma: checks"
remote "cd $REMOTE_DIR && bash tools/dev/comma_checks.sh ${OLD:+--base $OLD} ${RLOG:+--rlog $RLOG}"

if [ -n "$MEMORY" ]; then
  stage "7 road memory"
  remote "cp -f /data/ti_curve/curve_memory_v1.npz /data/ti_curve/curve_memory_v1.prev.npz 2>/dev/null || true"
  run "${SCP[@]}" "$MEMORY" "$DEVICE:/data/ti_curve/curve_memory_v1.npz.new"
  remote "mv -f /data/ti_curve/curve_memory_v1.npz.new /data/ti_curve/curve_memory_v1.npz"
fi

stage "8 reboot and verify"
remote "sudo reboot" || true
if [ $DRY = 0 ]; then
  sleep 30
  for i in $(seq 1 36); do "${SSH[@]}" true 2>/dev/null && break; sleep 5; done
fi
GOT=$(remote "cd $REMOTE_DIR && git rev-parse HEAD" || echo "?")
[ $DRY = 1 ] || [ "$GOT" = "$NEW" ] || { echo "comma is on $GOT, expected $NEW"; exit 1; }
remote "pgrep -f selfdrive.ui.ui >/dev/null || pgrep -x ui >/dev/null && pgrep -f pandad >/dev/null && echo 'ui and pandad running'"
stage "done: $NEW installed (check the switches are off on the screen)"
