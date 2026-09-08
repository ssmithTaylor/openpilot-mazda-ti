"""Rlog input identities for the Mazda plant controller; no control decisions."""

INPUT_SERVICES = ('carState', 'frogpilotCarState', 'modelV2', 'liveParameters', 'liveTorqueParameters', 'liveDelay', 'carOutput', 'liveLocationKalman')


def record_inputs(diagnostics, sm, car_state_event, car_state_updated):
  """Call beside LaC.update, before another SubMaster update can replace inputs."""
  snapshots = diagnostics.init('inputs', len(INPUT_SERVICES))
  for snapshot, service in zip(snapshots, INPUT_SERVICES, strict=True):
    snapshot.service = service
    if service == 'carState':
      # controlsd reads this event through its dedicated blocking socket.
      # Preserve the held event's identity on timeout, just like CS_prev.
      snapshot.seen = car_state_event is not None
      snapshot.updated = car_state_updated
      snapshot.alive = car_state_updated
      snapshot.frequencyOk = False  # no SubMaster frequency estimate for this socket
      if car_state_event is not None:
        snapshot.logMonoTime = car_state_event.logMonoTime
        snapshot.valid = car_state_event.valid
        snapshot.checksPassed = car_state_updated and car_state_event.valid and car_state_event.carState.canValid
      continue
    snapshot.logMonoTime = sm.logMonoTime[service]
    snapshot.valid = sm.valid[service]
    snapshot.alive = sm.alive[service]
    snapshot.frequencyOk = sm.freq_ok[service]
    snapshot.checksPassed = sm.all_checks([service])
    snapshot.updated = sm.updated[service]
    snapshot.seen = sm.seen[service]
