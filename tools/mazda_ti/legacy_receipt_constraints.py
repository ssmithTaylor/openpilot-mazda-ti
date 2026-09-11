"""Necessary sequencing checks for source-verified legacy controlsd reception.

Assumes one ordered non-conflating carState reader, normal receive semantics,
no publisher restart, and timeout reuse only after the configured wait. These
checks are not substitutes for explicitly logged consumed publication IDs.
"""


def sequence_compatible(*, car_state_ns: int, parameters_ns: int,
                        previous_car_state_ns: int | None,
                        previous_parameters_ns: int | None,
                        cycle_start_ns: int, publication_ns: int,
                        receive_timeout_ns: int) -> bool:
  """Reject impossible repeats/backtracking under the declared socket contract.

  A normal blocking receive that returns no message can reuse CS_prev only
  after its timeout. The whole step's elapsed time is a conservative upper
  bound on time spent receiving. At/above timeout, reuse remains possible.
  Parameter publications are retained or advance; they cannot move backward.
  """
  values = [car_state_ns, parameters_ns, cycle_start_ns, publication_ns, receive_timeout_ns]
  values += [v for v in [previous_car_state_ns, previous_parameters_ns] if v is not None]
  if any(type(v) is not int or v <= 0 for v in values):
    raise ValueError('Require positive integer nanosecond identities and clocks')
  if cycle_start_ns > publication_ns:
    raise ValueError('Cycle starts after its publication')
  if car_state_ns > publication_ns or parameters_ns > publication_ns:
    return False
  if previous_parameters_ns is not None and parameters_ns < previous_parameters_ns:
    return False
  if previous_car_state_ns is None:
    return True
  if car_state_ns < previous_car_state_ns:
    return False
  return car_state_ns > previous_car_state_ns or publication_ns-cycle_start_ns >= receive_timeout_ns
