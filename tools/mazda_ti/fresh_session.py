"""Reproduce the public Mazda evaluation workflow from fresh, relocated inputs."""

import argparse
import copy
import os
from pathlib import Path
from pathlib import PurePosixPath, PureWindowsPath
import re
import shutil
import subprocess
import sys
import time
import uuid

from . import run as replay_run
from .batch import evaluate_batch
from .corpus import evaluate_corpus
from .fresh_fixture import create_fresh_fixture, fixture_tree_sha256
from .ingest import collect_inventory, discover
from .provenance import ROOT, check_sources, read_json, resolve_ref, sha256, write_json
from .release import EVIDENCE_KINDS, PREREQUISITES, PROFILE, qualify
from .scenarios import run as run_scenario
from .startup_eval import capability as process_capability
from .trace_compare import compare_trace_bundles
from .transition_eval import assess, run as run_transition, source_identity


FORMAT_VERSION = 1
QUALIFIED_DECISION = "qualified_for_separate_authorization"
COMPONENT_PATHS = (
  "collected-inventory.json", "batch/canonical-001.json", "corpus/result.json", "corpus/report.md",
  "scenario/result.json", "scenario/report.md", "process-contract/result.json", "process/result.json", "comparison/result.json",
  "comparison/report.md", "release/result.json", "release/report.md",
)
SCOPE = (
  "This acceptance proves local infrastructure contracts and deterministic steering-command reproduction from public "
  "fixtures. It does not predict lane motion, establish acceptable handling or driver contact, authorize deployment, "
  "connect to a device, publish CAN, or actuate a vehicle."
)
FIXTURE_PROCESS_REJECTION = "process: Record does not match the process evidence schema"


def _fixture_process_boundary(rlogs):
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
      "runtime_source": {"git_head": None, "kind": "current_worktree_fixture", "identity": source_identity()},
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
  process_contract = run_transition(
    output / "process-contract", rlog=[process_raw], all_segments=True,
    process_boundary=_fixture_process_boundary([process_raw]),
    capability=lambda: {"full_process_supported": True, "missing": []},
  )
  process = run_transition(output / "process", rlog=[process_raw], all_segments=True)
  arm = output / "corpus/cases/fresh-instrumented/candidate"
  comparison = compare_trace_bundles(
    arm, arm, arm, output / "comparison", data_root=output / "raw", evidence_root=output,
  )
  write_json(output / "release-request.json", _release_request(candidate, output))
  release = qualify(output / "release-request.json", output, output / "release")
  components = {name: sha256(output / name) for name in COMPONENT_PATHS}
  statuses = {
    "batch": batch["status"], "corpus": corpus["status"], "scenario": scenario["status"],
    "process_contract_fixture": process_contract["status"], "process_declared_runtime": process["status"],
    "comparison": comparison["status"],
    "release": release["status"], "release_decision": release["qualification"],
  }
  return {
    "components": components, "statuses": statuses,
    "release_findings": release["findings"],
    "execution": {"elapsed_seconds": time.perf_counter() - started, "batch": batch["execution"],
                  "worker_pid": os.getpid(), "session_identity": uuid.uuid4().hex},
  }


def _run_in_fresh_process(fixture_root, output, cache_root, candidate):
  """Run one pass in a new interpreter and read its retained summary."""
  subprocess.run([
    sys.executable, "-m", "tools.mazda_ti.fresh_session", "--worker",
    "--fixture-root", str(fixture_root), "--output", str(output),
    "--cache-root", str(cache_root), "--candidate", candidate,
  ], cwd=ROOT, check=True)
  return read_json(output / "session-summary.json")


def _expect_rejection(check):
  try:
    check()
  except (OSError, ValueError, KeyError, TypeError):
    return True
  return False


def _evidence_mutation_rejected(release, kind):
  expected = f"{kind}: Evidence identity changed: {kind}"
  return (release["status"] == "failed_check" and release["qualification"] == "unqualified"
          and expected in release["findings"])


def _require_hash(path, expected):
  if sha256(path) != expected:
    raise ValueError("Artifact identity changed")


def _invalidation_checks(output, second, candidate, cache_root):
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
  evidence_rejected = _evidence_mutation_rejected(evidence, "scenario")

  cache_case = sorted(path for path in cache_root.iterdir() if path.is_dir())[0]
  corrupt_attempt = cache_case / "attempt-001"
  prepared = corrupt_attempt / "prepared/spec.json"
  prepared.write_bytes(prepared.read_bytes() + b" ")
  recomputed = evaluate_batch(
    output / "fixtures-b/batch.json", second / "raw", output / "invalidation/cache-recompute",
    cache_root=cache_root, workers=1,
  )
  reused = [row["reused"] for row in recomputed["execution"]["cases"]]
  cache_recomputed = (
    recomputed["status"] == "completed_checks"
    and recomputed["execution"]["timing_kind"] == "mixed"
    and sorted(reused) == [False, True]
    and len(recomputed["execution"]["cache_invalidations"]) == 1
    and prepared.exists() and (cache_case / "attempt-002").is_dir()
  )
  return {
    "source": source_rejected, "raw": raw_rejected, "manifest": manifest_rejected,
    "runtime": runtime_rejected, "evidence": evidence_rejected,
    "prepared_artifact_cache": cache_recomputed,
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


WINDOWS_PATH = re.compile(r"(?<![A-Za-z0-9_])(?:[A-Za-z]:[\\/]|\\\\[^\\\s]+\\[^\\\s]+)")
POSIX_PATH = re.compile(r"(?<![A-Za-z0-9_+:/])/(?!/)(?:[^/\s]+/)*[^/\s]+")


def _absolute_path(value):
  return isinstance(value, str) and (
    PureWindowsPath(value).is_absolute() or PurePosixPath(value).is_absolute()
    or WINDOWS_PATH.search(value) is not None or POSIX_PATH.search(value) is not None
  )


def _json_has_absolute_path(value):
  if isinstance(value, dict):
    return any(_json_has_absolute_path(key) or _json_has_absolute_path(item) for key, item in value.items())
  if isinstance(value, list):
    return any(_json_has_absolute_path(item) for item in value)
  return _absolute_path(value)


def _portable(paths):
  for path in paths:
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".json" and _json_has_absolute_path(read_json(path)):
      return False
    if path.suffix != ".json" and (WINDOWS_PATH.search(text) or POSIX_PATH.search(text)):
      return False
  return True


def _acceptance_checks(first, second, fixtures_equal, portable, invalidation, incomplete):
  required = ("batch", "corpus", "scenario", "process_contract_fixture", "comparison")
  first_reused = [row["reused"] for row in first["execution"]["batch"]["cases"]]
  second_reused = [row["reused"] for row in second["execution"]["batch"]["cases"]]
  process_status = first["statuses"]["process_declared_runtime"]
  release_matches_process = (
    first["statuses"]["process_declared_runtime"] == second["statuses"]["process_declared_runtime"]
    and ((process_status == "completed_checks"
          and first["statuses"]["release"] == second["statuses"]["release"] == "completed_checks"
          and first["statuses"]["release_decision"] == second["statuses"]["release_decision"] == QUALIFIED_DECISION)
         or (process_status in ("unsupported", "failed_check")
             and first["statuses"]["release"] == second["statuses"]["release"] == "failed_check"
             and first["statuses"]["release_decision"] == second["statuses"]["release_decision"] == "unqualified"))
  )
  return {
    "fixture_bytes_equal_after_relocation": fixtures_equal,
    "canonical_outputs_equal": first["components"] == second["components"] and first["statuses"] == second["statuses"],
    "canonical_outputs_portable": portable,
    "separate_interpreter_sessions": first["execution"]["session_identity"] != second["execution"]["session_identity"],
    "required_component_stages_completed": all(first["statuses"][name] == second["statuses"][name] == "completed_checks"
                                               for name in required),
    "declared_runtime_result_explicit": process_status in ("completed_checks", "unsupported", "failed_check"),
    "cold_run_executed_all_cases": first["execution"]["batch"]["timing_kind"] == "cold" and not any(first_reused),
    "repeated_run_reused_all_cases": second["execution"]["batch"]["timing_kind"] == "repeated" and all(second_reused),
    "release_decision_reproduced": first["statuses"]["release_decision"] == second["statuses"]["release_decision"],
    "release_matches_declared_process_result": release_matches_process,
    "incomplete_runtime_unqualified": incomplete["process_status"] == "unsupported"
                                      and incomplete["release_status"] == "failed_check"
                                      and incomplete["release_decision"] == "unqualified",
    "all_invalidation_dimensions_rejected": all(invalidation.values()),
  }


def _report(result):
  lines = ["# Mazda fresh-session acceptance", "", f"Status: **{result['status']}**.", "", result["scope"], "",
           "## Acceptance checks", "", "| Check | Passed |", "| --- | --- |"]
  lines.extend(f"| {name} | {str(passed).lower()} |" for name, passed in sorted(result["checks"].items()))
  lines += ["", "## Component statuses", "", "| Component | Status |", "| --- | --- |"]
  lines.extend(f"| {name} | {status} |" for name, status in sorted(result["component_statuses"].items()))
  lines += ["", "## Canonical artifacts", "", "| Artifact | SHA-256 |", "| --- | --- |"]
  lines.extend(f"| {name} | `{digest}` |" for name, digest in sorted(result["canonical_components"].items()))
  lines += ["", "## Coverage and exclusions", "",
            "- Historical command replay and exact-identity instrumented replay are required component stages.",
            "- The instrumented fixture covers TI loss, stock fallback, TI re-entry, inactivity, and re-engagement.",
            "- Source, raw bytes, manifest, runtime, and evidence mutation checks are reported above.",
            "- Recorded physical outcomes and matched physical conditions remain unavailable and visible.",
            "- Infrastructure completion leaves physical handling and predictive simulation unresolved.", ""]
  if result["status"] == "completed_checks":
    lines.append("Every required acceptance check passed.")
  else:
    lines.append("One or more required acceptance checks failed; inspect the false rows and retained evidence.")
  lines += ["", f"Release decision: **{result['component_statuses']['release_decision']}**."]
  if result["component_statuses"]["process_declared_runtime"] != "completed_checks":
    lines.append("Actual declared-runtime process evidence is still required.")
  lines += ["", "Measured timing, cache contribution, host capability, and machine paths are recorded only in `execution.json`.", ""]
  return "\n".join(lines)


def run(output, candidate="HEAD"):
  """Run two complete passes from relocated fixture bytes and retain every check."""
  output = Path(output)
  candidate = resolve_ref(candidate)
  if re.fullmatch(r"[a-f0-9]{40}", candidate) is None:
    raise ValueError("Fresh-session acceptance requires a committed candidate revision")
  output.mkdir(parents=True, exist_ok=False)
  fixture_a, fixture_b = output / "fixtures-a", output / "fixtures-b"
  create_fresh_fixture(fixture_a, candidate)
  create_fresh_fixture(fixture_b, candidate)
  fixtures_equal = fixture_tree_sha256(fixture_a) == fixture_tree_sha256(fixture_b)
  cache = output / "cache"
  first = _run_in_fresh_process(fixture_a, output / "run-a", cache, candidate)
  second = _run_in_fresh_process(fixture_b, output / "run-b", cache, candidate)
  invalidation = _invalidation_checks(output, output / "run-b", candidate, cache)
  incomplete = _incomplete_profile(output, output / "run-b", candidate)
  canonical_paths = [output / run / name for run in ("run-a", "run-b") for name in COMPONENT_PATHS]
  portable = _portable(canonical_paths)
  first_reused = [row["reused"] for row in first["execution"]["batch"]["cases"]]
  second_reused = [row["reused"] for row in second["execution"]["batch"]["cases"]]
  checks = _acceptance_checks(first, second, fixtures_equal, portable, invalidation, incomplete)
  result = {
    "format_version": FORMAT_VERSION, "status": "completed_checks" if all(checks.values()) else "failed_check",
    "candidate_revision": candidate, "checks": checks, "canonical_components": first["components"],
    "component_statuses": first["statuses"], "invalidation": invalidation, "incomplete_environment": incomplete,
    "release_rejection_findings": first["release_findings"],
    "retained_exclusions": ["retained-unavailable-corner", "recorded physical outcomes", "matched physical conditions"],
    "evidence_limits": [SCOPE, "The deterministic process-contract fixture cannot qualify a release; release evidence comes from the declared process boundary."],
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
  (output / "report.md").write_text(_report(result), encoding="utf-8", newline="\n")
  return result


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--output", type=Path, required=True, help="new directory for the complete acceptance result")
  parser.add_argument("--candidate", default="HEAD", help="committed candidate revision (default: HEAD)")
  parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
  parser.add_argument("--fixture-root", type=Path, help=argparse.SUPPRESS)
  parser.add_argument("--cache-root", type=Path, help=argparse.SUPPRESS)
  args = parser.parse_args(argv)
  try:
    if args.worker:
      if args.fixture_root is None or args.cache_root is None:
        raise ValueError("Worker requires fixture and cache roots")
      summary = _run_once(args.fixture_root, args.output, args.cache_root, args.candidate)
      write_json(args.output / "session-summary.json", summary)
      return 0
    result = run(args.output, args.candidate)
  except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
    parser.exit(1, f"Fresh-session acceptance could not start: {error}\n")
  print(f"{result['status']}: {args.output / 'report.md'}")
  return 0 if result["status"] == "completed_checks" else 1


if __name__ == "__main__":
  raise SystemExit(main())
