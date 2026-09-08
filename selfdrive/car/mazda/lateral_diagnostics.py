"""Rlog input identities for the Mazda plant controller; no control decisions."""

INPUT_SERVICES = ('carState', 'frogpilotCarState', 'modelV2', 'liveParameters', 'liveTorqueParameters', 'liveDelay', 'carOutput', 'liveLocationKalman')


def record_inputs(diagnostics, sm):
  """Call beside LaC.update, before another SubMaster update can replace inputs."""
  snapshots = diagnostics.init('inputs', len(INPUT_SERVICES))
  for snapshot, service in zip(snapshots, INPUT_SERVICES, strict=True):
    snapshot.service = service
    snapshot.logMonoTime = sm.logMonoTime[service]
    snapshot.valid = sm.valid[service]
    snapshot.alive = sm.alive[service]
    snapshot.frequencyOk = sm.freq_ok[service]
    snapshot.checksPassed = sm.all_checks([service])
    snapshot.updated = sm.updated[service]
    snapshot.seen = sm.seen[service]
