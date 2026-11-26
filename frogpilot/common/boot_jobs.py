"""FrogPilot's boot jobs (backups, stats) wait until the car is off-road (integ3 review T1-I2).

RUNTIME: called once per boot from the manager process's boot thread (frogpilot_functions.frogpilot_boot_functions),
after the system clock has become valid. Not in controlsd / card / the planner. While on-road it reads the IsOnroad
param once every OFFROAD_POLL_S; nothing else.

Why: the boot thread waits for a valid clock. With the GPS clock fix the clock can become valid in the middle of a
drive, and the jobs behind it (rsync + tar + zstd of the install tree, a toggles backup, the stats post) would then
load the disk and CPU while driving. They are deferred until the car is parked (ignition off; manager clears IsOnroad).

Replayability: the decision is logged as two lines, "deferred" (once, when the car was on-road at the moment the
jobs were due) and "running" with how long they were deferred, so a drive log shows whether and when they ran.
"""
import time

OFFROAD_POLL_S = 10.0  # one param read every 10 s while driving; the jobs are not urgent


def wait_until_offroad(params, sleep=time.sleep, monotonic=time.monotonic, log=print, poll_s: float = OFFROAD_POLL_S) -> float:
  """Block while params says the car is on-road. Returns the seconds deferred (0.0 when it was off-road already)."""
  t0 = monotonic()
  deferred = False
  while params.get_bool("IsOnroad"):
    if not deferred:
      log("boot jobs: deferred, the car is on-road (backups and stats wait until it is off-road)")
      deferred = True
    sleep(poll_s)
  waited = round(monotonic() - t0, 1) if deferred else 0.0
  log(f"boot jobs: running (deferred {waited} s)")
  return waited
