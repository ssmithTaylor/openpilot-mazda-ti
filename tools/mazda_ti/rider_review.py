"""Create and validate deterministic, source-locked rider review bundles."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
import re
from pathlib import Path

LABELS = ("smooth", "scallop", "held_inward", "over_release", "intervention", "uncertain")
CONFIDENCE = ("low", "medium", "high")
CONTACT = ("yes", "no", "uncertain")
PHYSICAL = set(LABELS) - {"uncertain"}
IDENTIFIER = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")


def canonical(value):
  return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def hash_hex(value, name):
  if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdefABCDEF" for c in value):
    raise ValueError(f"{name} must be a 64-character SHA-256 hex digest")
  return value.lower()


def request_hash(request):
  return hashlib.sha256(canonical(request)).hexdigest()


def finite_number(value):
  return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def load_mapping(request):
  spec = request.get("mapping", {})
  path = Path(spec.get("path", ""))
  if not path.is_absolute():
    raise ValueError("mapping path must be absolute")
  expected = hash_hex(spec.get("sha256"), "mapping sha256")
  if not path.is_file() or sha256(path) != expected:
    raise ValueError("mapping source hash mismatch or missing")
  mapping = json.loads(path.read_text(encoding="utf-8"))
  if not isinstance(mapping.get("events"), dict):
    raise ValueError("mapping events are required")
  return mapping


def check_request(request):
  segment = request.get("segment")
  segment_valid = isinstance(segment, str) and bool(segment)
  if request.get("format_version") != 1 or not isinstance(request.get("route"), str) or not request["route"] or not segment_valid:
    raise ValueError("format_version, route, and segment are required")
  manifest = request.get("manifest", {})
  manifest_path = Path(manifest.get("path", ""))
  if not manifest_path.is_absolute():
    raise ValueError("manifest path must be absolute")
  manifest_sha256 = hash_hex(manifest.get("sha256"), "manifest sha256")
  if not manifest_path.is_file() or sha256(manifest_path) != manifest_sha256:
    raise ValueError("manifest source hash mismatch or missing")
  video = request.get("video", {})
  video_path = Path(video.get("path", ""))
  if not video_path.is_absolute():
    raise ValueError("video path must be absolute")
  if not video_path.is_file() or sha256(video_path) != hash_hex(video.get("sha256"), "video sha256"):
    raise ValueError("video source hash mismatch or missing")
  mapping = load_mapping(request)
  if mapping.get("identity") != {"route": request["route"], "segment": request["segment"]}:
    raise ValueError("mapping route/segment identity mismatch")
  if mapping.get("inputs", {}).get("video", {}).get("sha256") != video["sha256"]:
    raise ValueError("mapping video identity mismatch")
  events = mapping["events"]
  intervals = request.get("intervals")
  if not isinstance(intervals, list) or not intervals:
    raise ValueError("intervals are required")
  result = []
  seen = set()
  for item in intervals:
    ident = item.get("id")
    if not isinstance(ident, str) or IDENTIFIER.fullmatch(ident) is None or ident in seen:
      raise ValueError("interval IDs must be unique stable ASCII identifiers")
    seen.add(ident)
    start_event, end_event = item.get("start_event"), item.get("end_event")
    if start_event not in events or end_event not in events or start_event == end_event:
      raise ValueError("distinct start/end mapping events are required")
    start, end = events[start_event], events[end_event]
    if start.get("target_mono_s") != item.get("start_mono_s") or end.get("target_mono_s") != item.get("end_mono_s"):
      raise ValueError("interval timing does not match mapping targets")
    if not finite_number(item["start_mono_s"]) or not finite_number(item["end_mono_s"]) or not item["start_mono_s"] < item["end_mono_s"]:
      raise ValueError("interval monotonic bounds are invalid")
    brackets = {}
    for name, event in (("start", start), ("end", end)):
      brackets[name] = {}
      for side in ("before", "after"):
        row = event.get(side)
        if not isinstance(row, dict) or not finite_number(row.get("generated_clip_pts_s")) or row["generated_clip_pts_s"] < 0:
          raise ValueError("missing or invalid generated clip PTS bracket")
        brackets[name][side] = row
      if brackets[name]["before"]["generated_clip_pts_s"] > brackets[name]["after"]["generated_clip_pts_s"]:
        raise ValueError("reversed generated clip PTS bracket")
    if brackets["start"]["after"]["generated_clip_pts_s"] > brackets["end"]["before"]["generated_clip_pts_s"]:
      raise ValueError("interval camera PTS chronology overlaps or reverses")
    result.append({"id": ident, "start_mono_s": item["start_mono_s"], "end_mono_s": item["end_mono_s"], "brackets": brackets})
  result.sort(key=lambda row: (row["start_mono_s"], row["end_mono_s"], row["id"]))
  if any(a["end_mono_s"] > b["start_mono_s"] for a, b in zip(result, result[1:])):
    raise ValueError("intervals overlap")
  return result


def validate_annotation(request, annotation, *, manifest_sha256=None):
  intervals = check_request(request)
  if annotation.get("format_version") != 1:
    raise ValueError("annotation format_version must be 1")
  interval_by_id = {row["id"]: row for row in intervals}
  expected_request = request_hash(request)
  if annotation.get("request_sha256") != expected_request:
    raise ValueError("request hash mismatch")
  expected_manifest = request["manifest"]["sha256"]
  if annotation.get("manifest_sha256") != expected_manifest:
    raise ValueError("manifest hash mismatch")
  if manifest_sha256 is not None and annotation["manifest_sha256"] != hash_hex(manifest_sha256, "manifest override"):
    raise ValueError("manifest override mismatch")
  rows = annotation.get("annotations")
  ids = set(interval_by_id)
  if not isinstance(rows, list) or len(rows) != len(ids) or {row.get("id") for row in rows} != ids:
    raise ValueError("annotation IDs do not match request")
  output = []
  for row in sorted(rows, key=lambda item: item["id"]):
    labels = row.get("labels")
    if not isinstance(labels, list) or not labels or len(set(labels)) != len(labels) or any(label not in LABELS for label in labels):
      raise ValueError("unknown or duplicate labels")
    if list(labels) != [label for label in LABELS if label in labels]:
      raise ValueError("labels must use fixed LABELS order")
    if "smooth" in labels and len(labels) != 1:
      raise ValueError("smooth must be alone")
    if "uncertain" in labels and len(labels) != 1:
      raise ValueError("uncertain must be alone")
    if row.get("confidence") not in CONFIDENCE:
      raise ValueError("outcome confidence is required")
    if row.get("wheel_contact") not in CONTACT:
      raise ValueError("wheel_contact must be yes, no, or uncertain")
    provenance = row.get("rider_provenance")
    if provenance is not None and not isinstance(provenance, str):
      raise ValueError("rider provenance must be text")
    if PHYSICAL.intersection(labels) and (not isinstance(provenance, str) or not provenance.strip()):
      raise ValueError("physical labels require rider provenance")
    note = row.get("note")
    if note is not None and not isinstance(note, str):
      raise ValueError("annotation note must be text")
    intervention_pts = row.get("intervention_clip_pts_s")
    if "intervention" in labels:
      interval = interval_by_id[row["id"]]
      lo = interval["brackets"]["start"]["before"]["generated_clip_pts_s"]
      hi = interval["brackets"]["end"]["after"]["generated_clip_pts_s"]
      if not finite_number(intervention_pts) or not lo <= intervention_pts <= hi:
        raise ValueError("intervention clip PTS must be within interval")
    elif intervention_pts is not None:
      raise ValueError("intervention PTS requires intervention label")
    output.append({"id": row["id"], "labels": labels, "confidence": row["confidence"], "wheel_contact": row["wheel_contact"], "rider_provenance": provenance.strip() if isinstance(provenance, str) else None, "note": note.strip() if isinstance(note, str) and note.strip() else None, "intervention_clip_pts_s": intervention_pts})
  return {"format_version": 1, "request_sha256": expected_request, "manifest_sha256": expected_manifest, "annotations": output}


def build_html(request, output):
  intervals = check_request(request)
  video_uri = Path(request["video"]["path"]).resolve().as_uri()
  cards = []
  for row in intervals:
    start = row["brackets"]["start"]; end = row["brackets"]["end"]
    checks = "".join(f'<label><input type="checkbox" value="{label}">{label}</label>' for label in LABELS)
    cards.append(f'<section data-id="{html.escape(row["id"])}"><h2>{html.escape(row["id"])}</h2><p>MONO {row["start_mono_s"]}–{row["end_mono_s"]}; start PTS {start["before"]["generated_clip_pts_s"]}–{start["after"]["generated_clip_pts_s"]}; end PTS {end["before"]["generated_clip_pts_s"]}–{end["after"]["generated_clip_pts_s"]}</p><button data-seek="{start["before"]["generated_clip_pts_s"]},{end["after"]["generated_clip_pts_s"]}">Play interval</button><fieldset>{checks}</fieldset><select data-confidence><option value="">confidence</option>{"".join(f"<option>{value}</option>" for value in CONFIDENCE)}</select><select data-contact><option value="">wheel contact</option>{"".join(f"<option>{value}</option>" for value in CONTACT)}</select><input data-provenance placeholder="rider provenance"><input data-note placeholder="note"><input data-intervention-pts type="number" step="any" placeholder="intervention clip PTS"><button data-capture>Capture current video time</button></section>')
  seed = json.dumps({"format_version": 1, "request_sha256": request_hash(request), "manifest_sha256": request["manifest"]["sha256"]}, sort_keys=True).replace("<", "\\u003c")
  script = '<script>const seed=SEED;const LABELS=LABELS_JSON;const video=document.querySelector("video"),status=document.getElementById("status"),fallback=document.getElementById("annotation-json");const stable=v=>Array.isArray(v)?"["+v.map(stable).join(",")+"]":v!==null&&typeof v==="object"?"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+stable(v[k])).join(",")+"}":JSON.stringify(v);document.querySelectorAll("[data-seek]").forEach(b=>b.onclick=()=>{const [a,z]=b.dataset.seek.split(",").map(Number);video.currentTime=a;video.play();video.ontimeupdate=()=>{if(video.currentTime>=z){video.pause();video.ontimeupdate=null;}}});document.querySelectorAll("[data-capture]").forEach(b=>b.onclick=()=>b.parentElement.querySelector("[data-intervention-pts]").value=video.currentTime.toString());document.getElementById("copy").onclick=async()=>{try{const annotations=[...document.querySelectorAll("section")].map(s=>{const labels=LABELS.filter(x=>[...s.querySelectorAll("input[type=checkbox]:checked")].some(c=>c.value===x));const confidence=s.querySelector("[data-confidence]").value,contact=s.querySelector("[data-contact]").value,provenance=s.querySelector("[data-provenance]").value.trim(),pts=s.querySelector("[data-intervention-pts]").value; if(!labels.length||!confidence||!contact||((labels.includes("smooth")||labels.includes("uncertain"))&&labels.length>1)||(!provenance&&labels.some(x=>x!=="uncertain"))||(labels.includes("intervention")!==Boolean(pts)))throw Error("complete each annotation");const numericPts=pts?Number(pts):null,[lo,hi]=s.querySelector("[data-seek]").dataset.seek.split(",").map(Number);if(pts&&(!Number.isFinite(numericPts)||numericPts<lo||numericPts>hi))throw Error("intervention PTS is outside interval");return {id:s.dataset.id,labels,confidence,wheel_contact:contact,rider_provenance:provenance||null,note:s.querySelector("[data-note]").value.trim()||null,intervention_clip_pts_s:numericPts};});const value={...seed,annotations:annotations.sort((a,b)=>a.id<b.id?-1:a.id>b.id?1:0)},text=stable(value)+"\\n";fallback.value=text;try{if(!navigator.clipboard)throw Error("clipboard unavailable");await navigator.clipboard.writeText(text);status.textContent="Copied canonical JSON";}catch(error){fallback.focus();fallback.select();status.textContent="Clipboard unavailable; canonical JSON is selected below";}}catch(error){status.textContent=error.message;}};</script>'.replace("SEED", seed).replace("LABELS_JSON", json.dumps(LABELS))
  text = f'<!doctype html><meta charset="utf-8"><video controls src="{html.escape(video_uri)}"></video><div id="status"></div>{"".join(cards)}<button id="copy">Copy canonical JSON</button><textarea id="annotation-json" readonly></textarea>{script}'
  if output.exists():
    raise FileExistsError(output)
  output.write_text(text, encoding="utf-8", newline="\n")


def main():
  parser = argparse.ArgumentParser(); sub = parser.add_subparsers(dest="command", required=True)
  build = sub.add_parser("build"); build.add_argument("--request", type=Path, required=True); build.add_argument("--output", type=Path, required=True)
  validate = sub.add_parser("validate"); validate.add_argument("--request", type=Path, required=True); validate.add_argument("--annotation", type=Path, required=True); validate.add_argument("--manifest-sha256")
  args = parser.parse_args(); request = json.loads(args.request.read_text(encoding="utf-8"))
  if args.command == "build": build_html(request, args.output)
  else: print(canonical(validate_annotation(request, json.loads(args.annotation.read_text(encoding="utf-8")), manifest_sha256=args.manifest_sha256)).decode(), end="")


if __name__ == "__main__": main()
