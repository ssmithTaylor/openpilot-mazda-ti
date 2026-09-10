import pytest

from tools.mazda_ti.candidate_attribution import AttributionError, validate_candidate_attribution


def arm(source="current"):
  return {"controller_source": source, "controller_parent": "parent", "dependencies": {"pid": "a"},
          "preparation": "prep", "warmup": "warm", "settings": {"ti": 600},
          "history": "earliest", "window": [10, 20]}


def test_requires_and_accepts_equivalent_current_control():
  with pytest.raises(AttributionError, match="required"):
    validate_candidate_attribution(arm("recorded"), None, arm("candidate"))
  eq = arm("candidate")
  assert validate_candidate_attribution(arm("recorded"), eq, arm("candidate"))["candidate_only"]


def test_rejects_context_or_dependency_mismatch():
  eq = arm("candidate"); eq["history"] = "latest"
  with pytest.raises(AttributionError, match="history"):
    validate_candidate_attribution(arm("recorded"), eq, arm("candidate"))
  eq = arm("other")
  with pytest.raises(AttributionError, match="controller source"):
    validate_candidate_attribution(arm("recorded"), eq, arm("candidate"))


def test_same_source_does_not_require_extra_arm():
  assert validate_candidate_attribution(arm("same"), None, arm("same"))["candidate_only"] is False
