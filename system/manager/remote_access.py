"""Owner's switch (2026-11-25): keep the remote-access service (athenad) from starting.

RUNTIME: read once by manager at start (not per frame). No other process imports this.

athenad has no setting of its own. It connects to the fleet server whatever the upload toggles say, forwards the
device's text logs to it and serves the server's file-upload requests. On 2026-11-25 it re-sent 2,499 text log
files in a loop (none acknowledged), about 7 GB in an hour.

Switch: the flag file below. Present = manager never starts manage_athenad. Remove the file and reboot to restore
the service. A daemon left over from an earlier manager start keeps running until the next reboot.
"""
import os

NO_REMOTE_ACCESS_FLAG = "/data/no_remote_access"


def remote_access_ignores(flag: str = NO_REMOTE_ACCESS_FLAG) -> list[str]:
  """Process names manager must not start: ["manage_athenad"] while the flag file exists, else []."""
  return ["manage_athenad"] if os.path.isfile(flag) else []
