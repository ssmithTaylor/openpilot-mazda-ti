"""Typed causal boundary for controller-visible simulated actuator feedback."""

from dataclasses import dataclass
import math
import struct


@dataclass(frozen=True)
class AppliedLateralFeedback:
  """One completed apply represented by a published output.

  All timestamps are logMonoTime nanoseconds.  ``source_controller_mono`` is
  the controller update that produced ``source_request_mono``; it may be older
  than the immediately preceding update when the recorded transport sampled an
  older request.
  """

  publication_mono: int
  applied_mono: int
  source_request_mono: int
  source_controller_mono: int
  apply_sequence: int | None
  ti_counts: int
  stock_counts: int | None
  selected_counts: int
  normalized_steer: float
  active: bool
  ti_allowed: bool
  ti_available: bool
  ti_selected: bool
  stock_fallback: bool
  limiter_frozen: bool

  def validate_before(self, update_mono: int) -> None:
    if not 0 < self.source_controller_mono <= self.source_request_mono <= self.applied_mono <= self.publication_mono < update_mono:
      raise ValueError('Applied feedback identity is missing, future, or out of causal order')
    if self.apply_sequence is not None and self.apply_sequence <= 0:
      raise ValueError('Applied feedback requires a positive apply sequence')
    if not all(-600 <= value <= 600 for value in (self.ti_counts, self.selected_counts)):
      raise ValueError('Applied feedback is outside the qualified TI600 envelope')
    if self.stock_counts is not None and not -600 <= self.stock_counts <= 600:
      raise ValueError('Stock feedback is outside the qualified stock600 envelope')
    wire_steer = struct.unpack('f', struct.pack('f', self.selected_counts / 600))[0]
    if not math.isfinite(self.normalized_steer) or self.normalized_steer != wire_steer:
      raise ValueError('Normalized feedback does not represent the selected command')
    if self.stock_fallback != (self.active and not self.ti_selected):
      raise ValueError('Stock fallback selection is inconsistent')
    expected = self.stock_counts if self.stock_fallback else self.ti_counts
    if expected is None or self.selected_counts != expected:
      raise ValueError('Selected feedback does not match its actuator path')


@dataclass(frozen=True)
class PreviousAppliedFeedback:
  """Feedback visible before one controller update.

  ``observed_steer`` always preserves the consumed recorded/simulated output.
  ``candidate`` is intentionally unavailable until that output represents an
  apply sourced by a candidate-owned controller update.
  """

  update_mono: int
  observed_steer: float
  candidate: AppliedLateralFeedback | None

  def __post_init__(self) -> None:
    if self.update_mono <= 0 or not math.isfinite(self.observed_steer) or abs(self.observed_steer) > 1:
      raise ValueError('Invalid controller update or observed feedback')
    if self.candidate is not None:
      self.candidate.validate_before(self.update_mono)
      if float(self.observed_steer) != float(self.candidate.normalized_steer):
        raise ValueError('Candidate feedback differs from the consumed output')

  @property
  def candidate_available(self) -> bool:
    return self.candidate is not None
