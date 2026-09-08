# Standalone CLI imports must work before the openpilot namespace facade exists.
# ruff: noqa: TID251
"""Infer compatible monotone card request histories; publication is not receipt."""

from functools import cache
import numpy as np
from cereal import log
from .feedback import pure_limiter


def load(paths):
  rows = {k: [] for k in ['carControl', 'carState', 'carOutput', 'sendcan']}
  for path in paths:
    raw = path.read_bytes()
    for e in log.Event.read_multiple_bytes(raw):
      t = int(e.logMonoTime)
      kind = e.which()
      if kind == 'carControl':
        rows[kind].append((t, float(e.carControl.actuators.steer), bool(e.carControl.latActive)))
      elif kind == 'carState':
        rows[kind].append((t, float(e.carState.steeringTorque)))
      elif kind == 'carOutput':
        rows[kind].append((t, int(e.carOutput.actuatorsOutput.steerOutputCan)))
      elif kind == 'sendcan':
        for c in e.sendcan:
          if c.src == 1 and c.address == 0x249:
            rows[kind].append((t, (((c.dat[0] & 15) << 8) | c.dat[1]) - 2048))
    del raw
  for k in rows:
    rows[k].sort()
  return rows


def infer(rows, start, end, limits, limiter_ref):
  """Retain all monotone request sequences compatible with original sends.

  Message creation before carOutput is necessary, not proof of receipt.
  No assumed socket-delay bound or fitted timestamp offset is used.
  """
  times = {k: np.array([r[0] for r in v], dtype=np.int64) for k, v in rows.items()}
  request = rows['carControl']
  wanted = np.rint(np.array([r[1] for r in request]) * 600).astype(int)
  limit, _ = pure_limiter(limiter_ref)

  @cache
  def limited(value, previous, sensor):
    return int(limit(value, previous, sensor, limits))

  previous = rows['sendcan'][np.searchsorted(times['sendcan'], start, side='right') - 1][1]
  lower = 0
  frames = []
  for t, actual in rows['sendcan']:
    if not start < t <= end:
      continue
    state = rows['carState'][np.searchsorted(times['carState'], t, side='right') - 1]
    output = rows['carOutput'][np.searchsorted(times['carOutput'], t, side='right') - 1]
    upper = np.searchsorted(times['carControl'], output[0], side='right') - 1
    indices = np.arange(lower, upper + 1)
    compatible_values = {int(value) for value in np.unique(wanted[indices]) if limited(int(value), previous, state[1]) == actual}
    feasible = [int(j) for j in indices if int(wanted[j]) in compatible_values and request[j][2]]
    if not feasible:
      raise ValueError(dict(no_feasible_request=t, actual=actual, previous=previous, lower=lower, upper=int(upper)))
    frames.append(dict(send=t, state=state[0], output=output[0], actual=actual, previous=previous, latest_before_output=int(upper), feasible=feasible))
    lower = min(feasible)
    previous = actual
  # A later uniquely constrained request also limits earlier possible samples.
  upper = len(request) - 1
  for frame in reversed(frames):
    frame['feasible'] = [j for j in frame['feasible'] if j <= upper]
    if not frame['feasible']:
      raise ValueError('No complete monotone history')
    upper = max(frame['feasible'])
  result = []
  maps = {mode: {} for mode in ['earliest', 'latest']}
  for f in frames:
    choices = [request[j][0] for j in f.pop('feasible')]
    f['compatible_count'] = len(choices)
    f['earliest_request'] = choices[0]
    f['latest_request'] = choices[-1]
    latest_bound = request[f.pop('latest_before_output')][0]
    f['latest_bound_request'] = latest_bound
    f['upper_bound_sample_rejected'] = latest_bound not in choices
    result.append(f)
    for mode in maps:
      maps[mode][str(f['state'])] = f[mode + '_request']
  for mapping in maps.values():
    values = list(mapping.values())
    assert all(a <= b for a, b in zip(values, values[1:], strict=False))
  return dict(
    frames=result,
    mappings=maps,
    summary=dict(
      sends=len(result),
      unique=sum(r['compatible_count'] == 1 for r in result),
      ambiguous=sum(r['compatible_count'] > 1 for r in result),
      upper_bound_rejections=[r for r in result if r['upper_bound_sample_rejected']],
      max_compatible_requests=max(r['compatible_count'] for r in result),
    ),
  )
