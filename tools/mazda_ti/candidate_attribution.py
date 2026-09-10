"""Guard candidate attribution against historical-controller confounding."""

from __future__ import annotations

from typing import Any


class AttributionError(ValueError):
  """Evidence arms cannot support a candidate-only comparison."""


REQUIRED_CONTEXT = ("controller_parent", "dependencies", "preparation", "warmup", "settings", "history", "window")


def _same(left: dict[str, Any], right: dict[str, Any]) -> None:
  for key in REQUIRED_CONTEXT:
    if left.get(key) != right.get(key):
      raise AttributionError(f"equivalent-control context mismatch: {key}")


def validate_candidate_attribution(recorded: dict[str, Any], equivalent: dict[str, Any] | None,
                                   candidate: dict[str, Any]) -> dict[str, Any]:
  """Validate that candidate deltas are attributable to candidate source.

  ``recorded`` is the historical baseline, ``equivalent`` is a simulated
  current-controller control arm, and ``candidate`` is the changed arm. When
  candidate and recorded controller sources differ, the equivalent arm is
  mandatory and must share all replay context with the candidate.
  """
  recorded_source = recorded.get("controller_source")
  candidate_source = candidate.get("controller_source")
  if not recorded_source or not candidate_source:
    raise AttributionError("controller_source is required")
  if candidate_source != recorded_source:
    if equivalent is None:
      raise AttributionError("equivalent-control arm required when candidate source differs from recorded source")
    if equivalent.get("controller_source") != candidate_source:
      raise AttributionError("equivalent-control controller source differs from candidate")
    _same(candidate, equivalent)
  elif equivalent is not None:
    _same(candidate, equivalent)
  return {"status": "attributable", "candidate_only": candidate_source != recorded_source,
          "historical_to_current": candidate_source != recorded_source,
          "equivalent_control_required": candidate_source != recorded_source}
