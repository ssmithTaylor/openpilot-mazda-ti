"""Guard candidate attribution against historical-controller confounding."""

from __future__ import annotations

from typing import Any


class AttributionError(ValueError):
  """Evidence arms cannot support a candidate-only comparison."""


REQUIRED_CONTEXT = ("dependencies", "preparation", "warmup", "settings", "history", "window")


def _same(left: dict[str, Any], right: dict[str, Any]) -> None:
  for key in REQUIRED_CONTEXT:
    if key not in left or key not in right:
      raise AttributionError(f"equivalent-control context missing: {key}")
    if left.get(key) != right.get(key):
      raise AttributionError(f"equivalent-control context mismatch: {key}")


def validate_candidate_attribution(recorded: dict[str, Any], equivalent: dict[str, Any] | None,
                                   candidate: dict[str, Any]) -> dict[str, Any]:
  """Validate that candidate deltas are attributable to candidate source.

  ``recorded`` is the historical baseline, ``equivalent`` is a simulated
  current-controller control arm, and ``candidate`` is the changed arm. When
  candidate and recorded controller sources differ, the equivalent arm is
  mandatory. Its controller source must be the candidate's declared parent,
  while all non-lineage replay context must match the candidate. Requiring the
  equivalent arm to use the candidate source would compare the candidate with
  itself and erase the source delta this guard is meant to isolate.
  """
  recorded_source = recorded.get("controller_source")
  candidate_source = candidate.get("controller_source")
  if not recorded_source or not candidate_source:
    raise AttributionError("controller_source is required")
  if candidate_source != recorded_source:
    if equivalent is None:
      raise AttributionError("equivalent-control arm required when candidate source differs from recorded source")
    equivalent_source = equivalent.get("controller_source")
    candidate_parent = candidate.get("controller_parent")
    if not equivalent_source or not candidate_parent:
      raise AttributionError("equivalent-control source and candidate controller_parent are required")
    if equivalent_source != candidate_parent:
      raise AttributionError("candidate controller_parent differs from equivalent-control source")
    _same(candidate, equivalent)
  elif equivalent is not None:
    if equivalent.get("controller_source") != candidate_source:
      raise AttributionError("same-source equivalent-control source differs from candidate")
    _same(candidate, equivalent)
  equivalent_source = equivalent.get("controller_source") if equivalent is not None else candidate_source
  return {"status": "attributable", "candidate_only": candidate_source != equivalent_source,
          "historical_to_current": recorded_source != equivalent_source,
          "equivalent_control_required": candidate_source != recorded_source}
