"""Deterministic public inputs for the fresh-session acceptance workflow."""

import copy
from pathlib import Path
import shutil
import tempfile

from cereal import log

from .audit_diagnostics import INPUTS
from .example_fixture import create_example
from .provenance import read_json, sha256, write_json


def _historical_fixture(root, candidate):
  with tempfile.TemporaryDirectory(prefix="mazda-fresh-historical-") as temporary:
    generated = Path(temporary) / "fixture"
    request = read_json(create_example(generated))
    source = generated / request["case"]["experiment"]["rlogs"][0]
    destination = root / "historical--0/rlog"
    destination.parent.mkdir(parents=True)
    shutil.copyfile(source, destination)
  request["candidate_revision"] = candidate
  request["case"].update(id="fresh-historical", origin="synthetic")
  request["case"]["experiment"].update(name="fresh-historical", rlogs=["historical--0/rlog"])
  request["case"]["input_sha256"] = {"historical--0/rlog": sha256(destination)}
  return request


def _instrumented_fixture(root, candidate):
  with tempfile.TemporaryDirectory(prefix="mazda-fresh-instrumented-") as temporary:
    generated = Path(temporary) / "fixture"
    request = read_json(create_example(generated))
    raw = generated / request["case"]["experiment"]["rlogs"][0]
    events, latest = [], {}
    for reader in log.Event.read_multiple_bytes(raw.read_bytes()):
      event = reader.as_builder()
      kind, mono = event.which(), int(event.logMonoTime)
      index = (mono - 1_000_000_000) // 10_000_000
      active, ti_allowed = index not in (17, 18), index not in (13, 14, 15)
      if kind == "initData":
        event.initData.gitCommit = candidate
      if kind == "modelV2":
        fp = log.Event.new_message(logMonoTime=mono - 2, valid=True, frogpilotCarState={"tiActive": ti_allowed})
        location = log.Event.new_message(logMonoTime=mono - 1, valid=True, liveLocationKalman={})
        events.extend([fp, location])
        latest.update(frogpilotCarState=mono - 2, liveLocationKalman=mono - 1)
      if kind == "controlsState":
        state = event.controlsState.lateralControlState.torqueState
        state.active = active
        state.plantState = 1 if not ti_allowed else 7 if 17 <= index <= 22 else 3
        diagnostics = state.mazdaDiagnostics
        diagnostics.version, diagnostics.settings = 1, 16
        diagnostics.kp, diagnostics.ki, diagnostics.kd = .800000011920929, .20000000298023224, .06
        diagnostics.modelContextNow = diagnostics.cameraContextNow = mono
        for snapshot, name in zip(diagnostics.init("inputs", len(INPUTS)), INPUTS, strict=True):
          snapshot.service, snapshot.logMonoTime = name, latest[name]
          snapshot.valid = snapshot.alive = snapshot.frequencyOk = snapshot.checksPassed = snapshot.seen = True
      if kind == "carControl":
        event.carControl.controlsStateMonoTime = latest.get("controlsState", 790_000_000)
        event.carControl.latActive = active
      if kind == "carOutput":
        output = event.carOutput
        output.applySequence = index + 2 if index >= 0 else 1
        output.appliedCarControlMonoTime, output.appliedAtMonoTime = latest["carControl"], mono - 1
        output.appliedCarControlChecksPassed = True
        diagnostics = output.mazdaDiagnostics
        diagnostics.version, diagnostics.latActive, diagnostics.tiAllowed = 1, active, ti_allowed
        diagnostics.tiMax, diagnostics.tiDeltaUp, diagnostics.tiDeltaDown = 600, 10, 15
        diagnostics.tiDriverAllowance, diagnostics.tiDriverMultiplier = 15, 40
        diagnostics.tiDeltaUpKnee, diagnostics.tiDeltaUpHigh = 630, 9
      latest[kind] = mono
      events.append(event)

  destination = root / "instrumented--0/rlog"
  destination.parent.mkdir(parents=True)
  destination.write_bytes(b"".join(event.to_bytes() for event in events))
  request["candidate_revision"] = candidate
  request["case"].update(id="fresh-instrumented", method="instrumented", history="exact", origin="synthetic")
  request["case"]["experiment"].update(
    name="fresh-instrumented", rlogs=["instrumented--0/rlog"], baseline_controller=candidate,
    warmup_controller=candidate, limiter=candidate, fpcs_sample_at="diagnostic",
  )
  request["case"]["input_sha256"] = {"instrumented--0/rlog": sha256(destination)}
  return request


def create_fresh_fixture(output, candidate):
  """Create all portable inputs without copying generated controller outputs."""
  output = Path(output)
  output.mkdir(parents=True, exist_ok=False)
  requests = output / "requests"
  requests.mkdir()
  historical = _historical_fixture(output, candidate)
  instrumented = _instrumented_fixture(output, candidate)
  excluded = copy.deepcopy(historical)
  excluded["case"]["id"] = "retained-unavailable-corner"
  excluded["case"]["experiment"]["name"] = "retained-unavailable-corner"
  for name, request in (("historical", historical), ("instrumented", instrumented), ("excluded", excluded)):
    write_json(requests / f"{name}.json", request)

  settings = historical["case"]["experiment"]["settings"]
  for route, request in (("historical", historical), ("instrumented", instrumented)):
    write_json(output / f"{route}.metadata.json", {
      "expected_segments": [0], "source_revision": request["case"]["experiment"]["baseline_controller"],
      "settings": settings,
    })

  exclusion = output / "retained-exclusion.txt"
  exclusion.write_text(
    "No distributable recorded difficult-corner lane-path evidence is supplied. "
    "The retained case is excluded and cannot establish physical handling.\n",
    encoding="utf-8", newline="\n",
  )
  cases = [
    {
      "case": historical["case"], "control_revision": historical["case"]["experiment"]["baseline_controller"],
      "categories": ["straight_gentle"], "group": "development",
      "exposures": ["Deterministic public command fixture"], "annotations": [],
      "expected": {"minimum_sends": 19, "stock_commands": False}, "exclusion": None,
    },
    {
      "case": instrumented["case"], "control_revision": candidate,
      "categories": ["actuator_transition"], "group": "development",
      "exposures": ["Deterministic public transition fixture"], "annotations": [],
      "expected": {"minimum_sends": 19, "stock_commands": True}, "exclusion": None,
    },
    {
      "case": excluded["case"], "control_revision": excluded["case"]["experiment"]["baseline_controller"],
      "categories": ["difficult_corner"], "group": "withheld", "exposures": [], "annotations": [],
      "expected": {"minimum_sends": 19, "stock_commands": False},
      "exclusion": {"reason": "Recorded lane-path evidence is intentionally absent from this public fixture.",
                    "source": "retained-exclusion"},
    },
  ]
  write_json(output / "corpus.json", {
    "format_version": 1, "id": "fresh-session-public", "version": "1", "cases": cases,
    "sources": [{"id": "retained-exclusion", "path": exclusion.name, "sha256": sha256(exclusion), "kind": "analysis"}],
    "unavailable": {
      "successful_corner": "No recorded physical outcome is included in the public fixture.",
      "failed_corner": "No recorded physical outcome is included in the public fixture.",
      "matched_conditions": "Synthetic inputs do not provide matched physical entry conditions.",
    },
    "profiles": {"fresh-session": {
      "required_cases": ["fresh-historical", "fresh-instrumented"],
      "required_categories": ["straight_gentle", "actuator_transition"], "require_unexposed_withheld": False,
    }},
  })
  write_json(output / "batch.json", {
    "format_version": 1, "resources": {"max_workers": 1, "host_memory_mb": 4096, "worker_memory_mb": 512},
    "cases": [
      {"id": "fresh-historical", "request": "requests/historical.json", "isolation": "demonstrated"},
      {"id": "fresh-instrumented", "request": "requests/instrumented.json", "isolation": "demonstrated"},
    ],
  })
  write_json(output / "scenario.json", {
    "format_version": 1, "candidate_revision": candidate,
    "case": {"id": "fresh-reference-transitions", "method": "synthetic", "origin": "synthetic",
             "scenario": "reference_transitions", "baseline_controller": candidate, "limiter": candidate},
  })
  return {"historical": historical, "instrumented": instrumented}


def fixture_tree_sha256(root):
  """Return relative content identities for relocation checks."""
  root = Path(root)
  return {path.relative_to(root).as_posix(): sha256(path) for path in sorted(root.rglob("*")) if path.is_file()}
