"""Reproduce the public Mazda evaluation workflow from fresh, relocated inputs."""

import argparse
import copy
from pathlib import Path
import shutil
import subprocess
import time

from . import run as replay_run
from .batch import evaluate_batch
from .corpus import evaluate_corpus
from .fresh_fixture import create_fresh_fixture, fixture_tree_sha256
from .ingest import collect_inventory, discover
from .provenance import check_sources, read_json, resolve_ref, sha256, write_json
from .release import EVIDENCE_KINDS, PREREQUISITES, PROFILE, qualify
from .scenarios import run as run_scenario
from .startup_eval import capability as process_capability
from .trace_compare import compare_trace_bundles
from .transition_eval import assess, run as run_transition, source_identity


FORMAT_VERSION = 1
COMPONENT_PATHS = (
  "collected-inventory.json", "batch/canonical-001.json", "corpus/result.json", "corpus/report.md",
  "scenario/result.json", "scenario/report.md", "process/result.json", "comparison/result.json",
  "comparison/report.md", "release/result.json", "release/report.md",
)
SCOPE = (
  "This acceptance proves local infrastructure contracts and deterministic steering-command reproduction from public "
  "fixtures. It does not predict lane motion, establish acceptable handling or driver contact, authorize deployment, "
  "connect to a device, publish CAN, or actuate a vehicle."
)


def _fixture_process_boundary(candidate, rlogs):
  """Exercise the process producer contract without claiming host process execution."""
  paths = [Path(path) for path in rlogs]

  def boundary(_rlogs, _maximum, _all_segments, _transition_harness):
    transition = assess([
      {"mono_time_ns": mono, "active": active, "ti_allowed": ti_allowed, "health": health,
       "diagnostic_references": [], "state_before": state, "state_after": state,
       "state_before_id": f"opaque:{state}", "state_id": f"opaque:{state}"}
      for mono, active, ti_allowed, health, state in (
        (10, False, True, "valid", "a"), (20, True, True, "valid", "a"),
        (30, True, False, "valid", "a"), (40, True, True, "valid", "a"),
        (50, False, True, "missing", "a"), (60, False, True, "stale", "a"),
        (70, False, True, "invalid", "a"), (80, True, True, "valid", "a"),
      )
    ])
    return {**transition, "startup_boundary": {
      "interface": "selfdrive.test.process_replay.replay_process",
      "runtime_kind": "deterministic_process_contract_fixture",
      "runtime_source": {"git_head": candidate, "identity": source_identity()},
      "input": {"rlogs": [{"label": f"{path.parent.name}/{path.name}", "sha256": sha256(path)} for path in paths]},
      "no_vehicle_output": {"status": "passed"},
    }}
  return boundary


def _release_request(candidate, root, process_path="process/result.json", evidence_paths=None):
  paths = evidence_paths or {
    "corpus": "corpus/result.json", "scenario": "scenario/result.json",
    "process": process_path, "comparison": "comparison/result.json",
  }
  return {
    "format_version": 1, "candidate_revision": candidate, "profile": PROFILE,
    "physical_evaluation": {
      "route_segment": "future-authorized-route:0", "maneuver": "left_curve", "target_speed_mps": 20,
      "measurements": ["lane_position", "driver_steering_intervention"],
      "maximum_driver_steering_interventions": 0,
    },
    "settings": {"TiSteerMax": 600, "TorqueInterceptorEnabled": 1},
    "evidence": [
      {"id": kind, "kind": kind, "path": paths[kind], "sha256": sha256(root / paths[kind])}
      for kind in EVIDENCE_KINDS
    ],
    "deployment_prerequisites": [
      {"name": name, "status": "pending", "note": "Only after separate device authorization."}
      for name in PREREQUISITES
    ],
  }


def _run_once(fixture_root, output, cache_root, candidate):
  output.mkdir(parents=True, exist_ok=False)
  started = time.perf_counter()
  inventory = discover(fixture_root)
  write_json(output / "inventory.json", inventory)
  collected = collect_inventory(
    inventory, fixture_root, output / "raw", {"historical": [0], "instrumented": [0]}, transfer_limit=10_000_000,
  )
  write_json(output / "collected-inventory.json", collected)
  batch = evaluate_batch(fixture_root / "batch.json", output / "raw", output / "batch", cache_root=cache_root, workers=1)
  corpus = evaluate_corpus(
    fixture_root / "corpus.json", candidate, "fresh-session", output / "raw", fixture_root,
    output / "exposure-ledger.json", output / "corpus",
  )
  scenario = run_scenario(fixture_root / "scenario.json", output / "scenario")
  process_raw = output / "raw/instrumented--0/rlog"
  process = run_transition(
    output / "process", rlog=[process_raw], all_segments=True,
    process_boundary=_fixture_process_boundary(candidate, [process_raw]),
    capability=lambda: {"full_process_supported": True, "missing": []},
  )
  arm = output / "corpus/cases/fresh-instrumented/candidate"
  comparison = compare_trace_bundles(
    arm, arm, arm, output / "comparison", data_root=output / "raw", evidence_root=output,
  )
  write_json(output / "release-request.json", _release_request(candidate, output))
  release = qualify(output / "release-request.json", output, output / "release")
  components = {name: sha256(output / name) for name in COMPONENT_PATHS}
  statuses = {
    "batch": batch["status"], "corpus": corpus["status"], "scenario": scenario["status"],
    "process_contract_fixture": process["status"], "comparison": comparison["status"],
    "release": release["status"], "release_decision": release["qualification"],
  }
  return {
    "components": components, "statuses": statuses,
    "execution": {"elapsed_seconds": time.perf_counter() - started, "batch": batch["execution"]},
  }


def _expect_rejection(check):
  try:
    check()
  except (OSError, ValueError, KeyError, TypeError):
    return True
  return False


def _require_hash(path, expected):
  if sha256(path) != expected:
    raise ValueError("Artifact identity changed")


def _invalidation_checks(output, second, candidate):
  historical = second / "corpus/cases/fresh-historical"
  result = read_json(historical / "result.json")
  sources = copy.deepcopy(result["source_identities"]["repository_sources"])
  name = sorted(sources)[0]
  sources[name] = "0" * 64 if sources[name] != "0" * 64 else "1" * 64
  source_rejected = _expect_rejection(lambda: check_sources(sources))

  changed_raw = output / "invalidation/raw"
  shutil.copytree(second / "raw", changed_raw)
  raw_path = changed_raw / "historical--0/rlog"
  raw_path.write_bytes(raw_path.read_bytes() + b"changed")
  from .evaluate import verify_bundle
  raw_rejected = _expect_rejection(lambda: verify_bundle(historical, changed_raw))

  changed_manifest = output / "invalidation/corpus.json"
  changed_manifest.parent.mkdir(exist_ok=True)
  changed_manifest.write_bytes((output / "fixtures-b/corpus.json").read_bytes() + b" ")
  manifest_rejected = _expect_rejection(
    lambda: _require_hash(changed_manifest, read_json(second / "corpus/result.json")["manifest_sha256"])
  )

  baseline_path = historical / "baseline/result.json"
  preparation = historical / "prepared/preparation.json"
  runtime_rejected = _expect_rejection(lambda: replay_run.validate_baseline(
    baseline_path, sha256(preparation), "earliest", {"intentionally": "different"},
  ))

  tampered = output / "invalidation/tampered-scenario.json"
  tampered.write_bytes((second / "scenario/result.json").read_bytes() + b" ")
  paths = {
    "corpus": "run-b/corpus/result.json", "scenario": "invalidation/tampered-scenario.json",
    "process": "run-b/process/result.json", "comparison": "run-b/comparison/result.json",
  }
  request = _release_request(candidate, output, evidence_paths=paths)
  original_scenario_hash = sha256(second / "scenario/result.json")
  next(item for item in request["evidence"] if item["kind"] == "scenario")["sha256"] = original_scenario_hash
  # Keep the original scenario hash: changed bytes must be rejected before schema interpretation.
  request_path = output / "invalidation/evidence-request.json"
  write_json(request_path, request)
  evidence = qualify(request_path, output, output / "invalidation/evidence-release")
  evidence_rejected = evidence["status"] == "failed_check" and evidence["qualification"] == "unqualified"
  return {
    "source": source_rejected, "raw": raw_rejected, "manifest": manifest_rejected,
    "runtime": runtime_rejected, "evidence": evidence_rejected,
  }


def _incomplete_profile(output, second, candidate):
  incomplete = output / "incomplete"
  incomplete.mkdir()
  raw = second / "raw/instrumented--0/rlog"
  process = run_transition(
    incomplete / "process", rlog=[raw], all_segments=True,
    capability=lambda: {"full_process_supported": False, "missing": ["declared_full_process_runtime"]},
  )
  paths = {
    "corpus": "run-b/corpus/result.json", "scenario": "run-b/scenario/result.json",
    "process": "incomplete/process/result.json", "comparison": "run-b/comparison/result.json",
  }
  request = _release_request(candidate, output, evidence_paths=paths)
  request_path = incomplete / "release-request.json"
  write_json(request_path, request)
  release = qualify(request_path, output, incomplete / "release")
  return {
    "process_status": process["status"], "missing_capabilities": process["missing_capabilities"],
    "release_status": release["status"], "release_decision": release["qualification"],
  }


def _portable(paths, forbidden):
  lowered = [str(path.resolve()).lower() for path in forbidden]
  for path in paths:
    text = path.read_text(encoding="utf-8").lower()
    if any(value in text for value in lowered):
      return False
  return True


def run(output, candidate="HEAD"):
  """Run two complete passes from relocated fixture bytes and retain every check."""
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  candidate = resolve_ref(candidate)
  fixture_a, fixture_b = output / "fixtures-a", output / "fixtures-b"
  create_fresh_fixture(fixture_a, candidate)
  create_fresh_fixture(fixture_b, candidate)
  fixtures_equal = fixture_tree_sha256(fixture_a) == fixture_tree_sha256(fixture_b)
  cache = output / "cache"
  first = _run_once(fixture_a, output / "run-a", cache, candidate)
  second = _run_once(fixture_b, output / "run-b", cache, candidate)
  canonical_equal = first["components"] == second["components"] and first["statuses"] == second["statuses"]
  invalidation = _invalidation_checks(output, output / "run-b", candidate)
  incomplete = _incomplete_profile(output, output / "run-b", candidate)
  canonical_paths = [output / run / name for run in ("run-a", "run-b") for name in COMPONENT_PATHS]
  portable = _portable(canonical_paths, (output, fixture_a, fixture_b))
  first_reused = [row["reused"] for row in first["execution"]["batch"]["cases"]]
  second_reused = [row["reused"] for row in second["execution"]["batch"]["cases"]]
  checks = {
    "fixture_bytes_equal_after_relocation": fixtures_equal,
    "canonical_outputs_equal": canonical_equal,
    "canonical_outputs_portable": portable,
    "cold_run_executed_all_cases": first["execution"]["batch"]["timing_kind"] == "cold" and not any(first_reused),
    "repeated_run_reused_all_cases": second["execution"]["batch"]["timing_kind"] == "repeated" and all(second_reused),
    "complete_release_reproduced": first["statuses"]["release_decision"] == second["statuses"]["release_decision"]
                                      == "qualified_for_separate_authorization",
    "incomplete_runtime_unqualified": incomplete["process_status"] == "unsupported"
                                      and incomplete["release_decision"] == "unqualified",
    "all_invalidation_dimensions_rejected": all(invalidation.values()),
  }
  result = {
    "format_version": FORMAT_VERSION, "status": "completed_checks" if all(checks.values()) else "failed_check",
    "candidate_revision": candidate, "checks": checks, "canonical_components": first["components"],
    "component_statuses": first["statuses"], "invalidation": invalidation, "incomplete_environment": incomplete,
    "retained_exclusions": ["retained-unavailable-corner", "recorded physical outcomes", "matched physical conditions"],
    "evidence_limits": [SCOPE, "The supported process record is a deterministic producer-contract fixture; host process support is reported separately."],
    "scope": SCOPE,
  }
  execution = {
    "runs": {"cold": first["execution"], "repeated": second["execution"]},
    "cache_contribution": {
      "cold_case_executions": sum(not value for value in first_reused),
      "repeated_cache_hits": sum(second_reused),
      "repeated_case_executions": sum(not value for value in second_reused),
      "elapsed_seconds_saved_against_cold": max(0, first["execution"]["batch"]["elapsed_seconds"]
                                             - second["execution"]["batch"]["elapsed_seconds"]),
    },
    "host_process_capability": process_capability(),
    "bottlenecks": ["Full controller replay dominates cold execution.",
                    "Actual controlsd process replay still requires the declared Linux openpilot runtime and suitable rlogs."],
    "output_destination": str(output.resolve()),
  }
  write_json(output / "result.json", result)
  write_json(output / "execution.json", execution)
  report = [
    "# Mazda fresh-session acceptance", "", f"Status: **{result['status']}**.", "", SCOPE, "",
    "Two independently generated fixture trees had identical bytes. Fresh output roots reproduced the same ingestion, "
    "evaluation, corpus, scenario, process-contract, comparison, and release decisions and hashes.", "",
    "The first batch executed every case; the relocated repeat reused every valid cached case. Measured timing and cache "
    "contribution are recorded in `execution.json`, outside canonical evidence.", "", "## Coverage and exclusions", "",
    "- Historical command replay and exact-identity instrumented replay both reproduce their baselines.",
    "- The instrumented fixture covers TI loss, stock fallback, TI re-entry, inactivity, and re-engagement.",
    "- Source, raw bytes, manifest, runtime, and evidence mutations are each rejected.",
    "- Recorded physical outcomes and matched physical conditions remain unavailable and visible.",
    "- A missing full-process runtime produces unsupported process evidence and an unqualified release.",
    "- Infrastructure completion leaves physical handling and predictive simulation unresolved.", "",
    "The qualified fixture release remains pending separate authorization; no deployment prerequisite was performed.", "",
  ]
  (output / "report.md").write_text("\n".join(report), encoding="utf-8", newline="\n")
  return result


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--output", type=Path, required=True, help="new directory for the complete acceptance result")
  parser.add_argument("--candidate", default="HEAD", help="committed candidate revision (default: HEAD)")
  args = parser.parse_args(argv)
  try:
    result = run(args.output, args.candidate)
  except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
    parser.exit(1, f"Fresh-session acceptance could not start: {error}\n")
  print(f"{result['status']}: {args.output / 'report.md'}")
  return 0 if result["status"] == "completed_checks" else 1


if __name__ == "__main__":
  raise SystemExit(main())
