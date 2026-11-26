#!/usr/bin/env python3
"""Run the on-car Python tests on the Windows (or Linux) PC with everything set up, in one command.

  python tools/dev/pc_test.py --curve            # curve-slowing modules, slow tests excluded   (target < 60 s)
  python tools/dev/pc_test.py --changed [--base origin/main]   # tests affected by files changed vs base (target < 30 s)
  python tools/dev/pc_test.py --all              # every Mazda TI / curve / controls suite the PC can run
  python tools/dev/pc_test.py --curve --slow     # include tests marked slow
  python tools/dev/pc_test.py -- <pytest args>   # anything after -- goes to pytest (e.g. a single node id)

What it sets up (so that no test fails for setup reasons):
  * a disposable shim directory OUTSIDE the repo (%LOCALAPPDATA%/openpilot-pcshim/<hash>, or ~/.cache on Linux) with
    junctions/symlinks `openpilot/<common,selfdrive,frogpilot,system,third_party,tools>`, `cereal`, `opendbc` -> this
    checkout (the checkout's own `openpilot` symlink is dead text on Windows);
  * stubs for compiled / Linux-only modules: opendbc parser/packer, params_pyx, msgq ipc, transformations, msgq /
    rednose / teleoprtc / tinygrad packages, fcntl, smbus2 (sources: tools/dev/pcshim/);
  * pytest from a cwd outside the repo with --import-mode=importlib, -p no:cacheprovider, no bytecode, the repo's own
    markers (strict), and xdist (-n auto) when more than a few test files are selected.
It prints the wall time. Exit code is pytest's.
"""
import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SHIM_SRC = Path(__file__).resolve().parent / "pcshim"
LINK_DIRS = ("common", "selfdrive", "frogpilot", "system", "third_party", "tools")
ROOT_LINKS = ("cereal", "opendbc")

# The curve-slowing feature's own tests (car side, planner, plannerd, e2e, screen, scenarios, memory builder).
CURVE_TESTS = (
  "selfdrive/car/tests/test_mazda_ti_curve.py",
  "selfdrive/car/tests/test_mazda_ti_curve_plan.py",
  "selfdrive/car/tests/test_mazda_ti_curve_plannerd.py",
  "selfdrive/car/tests/test_mazda_ti_curve_e2e.py",
  "selfdrive/car/tests/test_mazda_ti_curve_screen.py",
  "selfdrive/car/tests/test_mazda_ti_curve_scenarios.py",
  "selfdrive/car/tests/test_mazda_ti_cruise_model.py",
  "selfdrive/car/tests/test_mazda_ti_curve_hold.py",
  "selfdrive/car/tests/test_mazda_ti_curve_push.py",
  "selfdrive/car/tests/test_mazda_ti_curve_monitor.py",
  "selfdrive/car/tests/test_mazda_ti_curve_exit.py",
  "selfdrive/car/tests/test_mazda_ti_curve_cap.py",
  "selfdrive/car/tests/test_mazda_ti_curve_alert_loop.py",
  "selfdrive/car/tests/test_mazda_ti_curve_aim.py",
  "selfdrive/car/tests/test_mazda_ti_curve_approach_table.py",
  "selfdrive/car/tests/test_mazda_ti_curve_release.py",
  "selfdrive/car/tests/test_mazda_ti_curve_replayable.py",
  "selfdrive/car/tests/test_mazda_ti_curve_carside.py",
  "selfdrive/car/tests/test_mazda_ti_curve_record.py",
  "selfdrive/car/tests/test_mazda_ti_curve_forks.py",
  "selfdrive/car/tests/test_mazda_ti_curve_perf_table.py",
  "selfdrive/car/tests/test_mazda_ti_curve_dryrun_inputs.py",
  "selfdrive/car/tests/test_mazda_ti_winding.py",
  "selfdrive/car/tests/test_no_per_frame_leaks.py",       # per-frame leaks / cost growth of every per-frame entry point (gc off)
  "selfdrive/car/tests/test_lint_per_frame.py",
  "tools/curve_slowing/test_early_dwell.py",
  "tools/curve_slowing/test_build_memory.py",
)
# --all: every suite the car-side work touches that runs on the PC.
ALL_GLOBS = (
  "selfdrive/car/tests/test_mazda_ti*.py",
  "selfdrive/car/tests/test_lane_drift_alerts.py",
  "selfdrive/car/tests/test_no_per_frame_leaks.py",
  "selfdrive/car/tests/test_lint_per_frame.py",
  "tools/dev/test_drive_memory_report.py",
  "tools/curve_slowing/test_*.py",
  "selfdrive/controls/lib/tests/test_*.py",
  "selfdrive/controls/tests/test_alerts.py",
  "selfdrive/controls/tests/test_state_machine.py",
)
# Tests that cannot run on a PC at all (hardware, compiled extensions, real processes). Never selected by --changed/--all.
PC_EXCLUDED = {
  # controlsd import chain reaches os.register_at_fork (POSIX-only) and tinygrad.tensor (dead symlink here).
  # These run on the comma (tools/dev/comma_checks.sh, controls group).
  "selfdrive/controls/tests/test_alerts.py": "imports controlsd -> os.register_at_fork / tinygrad (POSIX, compiled)",
  "selfdrive/controls/tests/test_state_machine.py": "imports controlsd -> tinygrad.tensor (dead symlink on Windows)",
  # Upstream openpilot tests that build CarParams through get_params() without FrogPilot's toggles: they fail the same
  # way on any machine (not a PC setup fault); see tools/dev/COMMA-CHECKS.md.
  "selfdrive/controls/lib/tests/test_latcontrol.py": "upstream: CarInterface.get_params called without frogpilot_toggles",
  "selfdrive/controls/lib/tests/test_vehicle_model.py": "upstream: CarInterface.get_params called without frogpilot_toggles",
}
TEST_ROOTS = ("selfdrive", "frogpilot", "tools", "common", "system")


def shim_dir():
  base = Path(os.environ.get("LOCALAPPDATA", "")) if os.name == "nt" else Path.home() / ".cache"
  if not str(base).strip() or not base.exists():
    base = Path(tempfile.gettempdir())
  return base / "openpilot-pcshim" / hashlib.sha1(str(REPO).lower().encode()).hexdigest()[:10]


def _link(link: Path, target: Path):
  if link.exists() or link.is_symlink():
    try:
      if link.resolve() == target.resolve():
        return
    except OSError:
      pass
    if link.is_dir() and not link.is_symlink() and os.name != "nt":
      shutil.rmtree(link)
    else:
      os.rmdir(link) if link.is_dir() else link.unlink()
  if os.name == "nt":
    subprocess.run(["cmd", "/c", "mklink", "/J", str(link), str(target)], check=True, stdout=subprocess.DEVNULL)
  else:
    link.symlink_to(target, target_is_directory=True)


def build_shim() -> Path:
  d = shim_dir()
  (d / "openpilot").mkdir(parents=True, exist_ok=True)
  for n in LINK_DIRS:
    _link(d / "openpilot" / n, REPO / n)
  for n in ROOT_LINKS:
    _link(d / n, REPO / n)
  wanted = {"openpilot/__init__.py": "openpilot_init.py", "pc_plugin.py": "pc_plugin.py"}
  if os.name == "nt":
    wanted.update({"fcntl.py": "fcntl.py", "smbus2.py": "smbus2.py"})
  for dst, src in wanted.items():
    data = (SHIM_SRC / src).read_bytes()
    p = d / dst
    if not p.exists() or p.read_bytes() != data:
      p.write_bytes(data)
  return d


def git(*a):
  return subprocess.run(["git", "-C", str(REPO), *a], capture_output=True, text=True, check=True).stdout


def all_test_files():
  out = []
  for root in TEST_ROOTS:
    out += [p for p in (REPO / root).rglob("test_*.py") if "third_party" not in p.parts]
  return out


def changed_tests(base: str):
  """Static import map: a changed test file selects itself; a changed module selects every test file that imports it,
  and the tests of every module that imports it (one level), within the --all set. Non-Python changes select nothing."""
  merge = git("merge-base", base, "HEAD").strip()
  names = set(git("diff", "--name-only", merge).split()) | set(git("ls-files", "--others", "--exclude-standard").split())
  py = [n for n in names if n.endswith(".py") and (REPO / n).exists()]
  allowed = {str(p.relative_to(REPO)).replace("\\", "/") for g in ALL_GLOBS for p in REPO.glob(g)}
  tests = {n for n in py if n in allowed}
  stems = {Path(n).stem for n in py if not Path(n).name.startswith("test_")}
  if not stems:
    return sorted(tests), names
  src_files = [p for root in TEST_ROOTS for p in (REPO / root).rglob("*.py") if not p.name.startswith("test_")]

  def importers(stem_set, files):
    pat = re.compile(r"^\s*(?:from\s+\S*\b(?:%s)\b|import\s+.*\b(?:%s)\b|from\s+\S+\s+import\s+.*\b(?:%s)\b)" %
                     (("|".join(stem_set),) * 3), re.M)
    hits = set()
    for p in files:
      try:
        if pat.search(p.read_text(encoding="utf-8", errors="ignore")):
          hits.add(p)
      except OSError:
        pass
    return hits

  level1 = {p.stem for p in importers(stems, src_files)} - stems
  sel = importers(stems | level1, [REPO / a for a in allowed])
  tests |= {str(p.relative_to(REPO)).replace("\\", "/") for p in sel}
  return sorted(tests), names


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  g = ap.add_mutually_exclusive_group()
  g.add_argument("--curve", action="store_true")
  g.add_argument("--changed", action="store_true")
  g.add_argument("--all", action="store_true")
  ap.add_argument("--base", default="origin/main", help="base ref for --changed (default origin/main)")
  ap.add_argument("--slow", action="store_true", help="include tests marked slow")
  ap.add_argument("-n", default=None, help="xdist workers (default: auto when >2 files, else 0)")
  ap.add_argument("--list", action="store_true", help="print the selected test files and exit")
  ap.add_argument("rest", nargs="*", help="extra pytest args / node ids (after --)")
  a = ap.parse_args()
  t0 = time.monotonic()

  if a.changed:
    files, names = changed_tests(a.base)
    print(f"[pc_test] {len(names)} changed file(s) vs {a.base}; {len(files)} test file(s) selected")
  elif a.all:
    files = sorted({str(p.relative_to(REPO)).replace("\\", "/") for g_ in ALL_GLOBS for p in REPO.glob(g_)})
  elif a.curve or not a.rest:
    files = [f for f in CURVE_TESTS if (REPO / f).exists()]
  else:
    files = []
  files = [f for f in files if f not in PC_EXCLUDED]
  if a.list:
    print("\n".join(files))
    return 0
  if not files and not a.rest:
    print("[pc_test] nothing to run"); print(f"[pc_test] wall {time.monotonic() - t0:.1f} s")
    return 0

  shim = build_shim()
  env = dict(os.environ)
  env.update(PYTHONPATH=os.pathsep.join([str(shim), str(REPO)]), PYTHONDONTWRITEBYTECODE="1", PYTHONUTF8="1",
             CI="1", OPENPILOT_PC_TEST="1")
  # 8 workers: -n auto (32 here) runs test_build_memory out of memory and costs more in start-up than it saves.
  n = a.n if a.n is not None else (str(min(8, os.cpu_count() or 1)) if len(files) > 2 else "0")
  cmd = [sys.executable, "-B", "-m", "pytest", "-c", str(REPO / "pyproject.toml"), "--rootdir", str(REPO),
         "--confcutdir", str(REPO / "selfdrive" / "car"), "-o", "addopts=", "--import-mode=importlib",
         "-p", "no:cacheprovider", "-p", "pc_plugin", "--strict-markers", "-q", "-W", "ignore::DeprecationWarning", "--durations=5"]
  if n != "0":
    cmd += ["-n", str(n), "--dist", "loadgroup"]  # per test; the screen test (C++ compile) is one xdist_group
  if not a.slow:
    cmd += ["-m", "not slow and not tici"]
  rest = [str(REPO / r) if not r.startswith("-") and (REPO / r.split("::")[0]).exists() else r for r in a.rest]
  cmd += [str(REPO / f) for f in files] + rest
  print(f"[pc_test] {len(files)} file(s), workers={n}, shim={shim}")
  r = subprocess.run(cmd, cwd=str(shim.parent), env=env)
  print(f"[pc_test] wall {time.monotonic() - t0:.1f} s, exit {r.returncode}")
  return r.returncode


if __name__ == "__main__":
  sys.exit(main())
