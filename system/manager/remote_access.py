"""Owner's switch (2026-11-25): keep the remote-access service (athenad) from starting.

RUNTIME: read once by manager at start (not per frame). No other process imports this.

athenad has no setting of its own. It connects to the fleet server whatever the upload toggles say, forwards the
device's text logs to it and serves the server's file-upload requests. On 2026-11-25 it re-sent 2,499 text log
files in a loop (none acknowledged), about 7 GB in an hour.

Switch: the flag file below. Present = manager never starts manage_athenad. Remove the file and reboot to restore
the service. A daemon left over from an earlier manager start keeps running until the next reboot.

The same flag also stops FrogPilot's phone-home (integ3 review T1-I2): the stats post and its geocoder lookup of the
last GPS position, device registration, error reports, the weather lookup and theme submissions to FrogPilot's API.
Those call sites ask remote_access_blocked(<name>) and skip the request when it returns True. Downloads of models,
themes and map data and the update check are not gated (they send no identity beyond the request itself).
RUNTIME of remote_access_blocked: one stat() per call, at boot or on a user action, or once per weather check
(minutes); the skip is logged once per call name per process (bounded).
"""
import os

NO_REMOTE_ACCESS_FLAG = "/data/no_remote_access"


_logged: set[str] = set()


def remote_access_off(flag: str | None = None) -> bool:
  """True while the owner's flag file exists (a regular file; a directory of that name is not the flag)."""
  return os.path.isfile(flag or NO_REMOTE_ACCESS_FLAG)


def remote_access_ignores(flag: str | None = None) -> list[str]:
  """Process names manager must not start: ["manage_athenad"] while the flag file exists, else []."""
  return ["manage_athenad"] if remote_access_off(flag) else []


def remote_access_blocked(name: str, flag: str | None = None) -> bool:
  """For an outbound call that sends the device's identity or position off the device: True = skip it (flag present).
  The first skip of each name in a process is logged (name only, nothing else)."""
  if not remote_access_off(flag):
    return False
  if name not in _logged:
    _logged.add(name)
    print(f"remote_access: {name} skipped (no_remote_access flag present)")
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event("remote_access.blocked", call=name)
    except Exception:
      pass
  return True
