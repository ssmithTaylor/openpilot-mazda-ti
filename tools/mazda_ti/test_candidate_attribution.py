import pytest

from tools.mazda_ti.candidate_attribution import AttributionError, validate_candidate_attribution


def arm(source="current", parent="parent"):
  return {"controller_source": source, "controller_parent": parent, "dependencies": {"pid": "a"},
          "preparation": "prep", "warmup": "warm", "settings": {"ti": 600},
          "history": "earliest", "window": [10, 20]}


def test_requires_and_accepts_equivalent_current_control():
  with pytest.raises(AttributionError, match="required"):
    validate_candidate_attribution(arm("recorded"), None, arm("candidate", "current"))
  eq = arm("current", "current-parent")
  result = validate_candidate_attribution(arm("recorded"), eq, arm("candidate", "current"))
  assert result == {"status": "attributable", "candidate_only": True,
                    "historical_to_current": True, "equivalent_control_required": True}


def test_rejects_context_or_dependency_mismatch():
  eq = arm("current"); eq["history"] = "latest"
  with pytest.raises(AttributionError, match="history"):
    validate_candidate_attribution(arm("recorded"), eq, arm("candidate", "current"))
  eq = arm("other")
  with pytest.raises(AttributionError, match="controller_parent"):
    validate_candidate_attribution(arm("recorded"), eq, arm("candidate", "current"))


def test_rejects_context_missing_from_both_arms():
  eq = arm("current")
  candidate = arm("candidate", "current")
  del eq["preparation"]
  del candidate["preparation"]
  with pytest.raises(AttributionError, match="context missing: preparation"):
    validate_candidate_attribution(arm("recorded"), eq, candidate)


def test_rejects_missing_lineage_and_candidate_as_its_own_control():
  with pytest.raises(AttributionError, match="required"):
    validate_candidate_attribution(arm("recorded"), arm("current"), arm("candidate", ""))
  with pytest.raises(AttributionError, match="controller_parent"):
    validate_candidate_attribution(arm("recorded"), arm("candidate"), arm("candidate", "current"))


def test_candidate_directly_parented_by_recorded_source_is_isolated():
  result = validate_candidate_attribution(arm("recorded"), arm("recorded"), arm("candidate", "recorded"))
  assert result["candidate_only"] is True
  assert result["historical_to_current"] is False


def test_equivalent_current_arm_can_prove_no_candidate_source_delta():
  result = validate_candidate_attribution(arm("recorded"), arm("current"), arm("current", "current"))
  assert result == {"status": "attributable", "candidate_only": False,
                    "historical_to_current": True, "equivalent_control_required": True}


def test_same_source_does_not_require_extra_arm():
  assert validate_candidate_attribution(arm("same"), None, arm("same"))["candidate_only"] is False


def test_same_source_optional_control_must_really_be_same_source():
  with pytest.raises(AttributionError, match="same-source"):
    validate_candidate_attribution(arm("same"), arm("other"), arm("same"))
