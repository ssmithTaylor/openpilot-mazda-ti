"""Build a compact, deterministic coarse geographic inventory from NPZ extracts.

The inventory is a discovery aid. Only ``data`` columns named ``lat`` and
``lon`` plus ``mono`` affect geometry; controller, lane, and outcome columns
are ignored for matching. NPZ loading and whole-file hashing still cover the
complete archive. A hit remains ``coarse_candidate_only`` until the raw,
float64 geographic matcher has been run and the road has been independently
confirmed.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from pathlib import PurePosixPath
import re
import stat

import numpy as np

from .provenance import sha256


_FORMAT_VERSION = 1
_REQUIRED_REQUEST = {"format_version", "case_id", "reference"}
_REQUIRED_REFERENCE = {"extract", "rlogs", "start_mono", "end_mono", "tolerance_m"}
_LAT_ALIASES = {"latitude", "gps_latitude"}
_LON_ALIASES = {"longitude", "gps_longitude"}
_HEX_SHA256 = re.compile(r"[0-9a-f]{64}\Z")


def _finite_number(value, label):
  if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
    raise ValueError(f"{label} must be a finite number")
  return float(value)


def _request(value):
  """Validate and normalize an explicit pinned reference request."""
  if not isinstance(value, dict) or set(value) != _REQUIRED_REQUEST:
    raise ValueError("Request must contain exactly format_version, case_id and reference")
  if type(value["format_version"]) is not int or value["format_version"] != _FORMAT_VERSION:
    raise ValueError("Unsupported geographic inventory request")
  case_id = value["case_id"]
  if not isinstance(case_id, str) or not case_id or len(case_id) > 80 or any(
      not (char.isalnum() or char in "_.-") for char in case_id):
    raise ValueError("Invalid case identity")
  reference = value["reference"]
  if not isinstance(reference, dict) or set(reference) != _REQUIRED_REFERENCE:
    raise ValueError("Reference must contain exactly extract, rlogs, start_mono, end_mono and tolerance_m")
  extract = reference["extract"]
  if not isinstance(extract, dict) or set(extract) != {"path", "sha256"}:
    raise ValueError("Reference extract requires path and sha256")
  extract_path = extract["path"]
  extract_pure = PurePosixPath(extract_path) if isinstance(extract_path, str) else None
  if (extract_pure is None or not extract_path or "\\" in extract_path or extract_pure.is_absolute() or
      any(part in ("", ".", "..") for part in extract_pure.parts) or extract_pure.as_posix() != extract_path or
      not extract_path.lower().endswith(".npz") or not isinstance(extract["sha256"], str) or
      _HEX_SHA256.fullmatch(extract["sha256"]) is None):
    raise ValueError("Reference extract must be a normalized relative NPZ path with lowercase SHA-256")
  rlogs = reference["rlogs"]
  if not isinstance(rlogs, list) or not rlogs:
    raise ValueError("Reference requires source-locked raw rlogs")
  normalized_rlogs = []
  seen_paths = set()
  for source in rlogs:
    if not isinstance(source, dict) or set(source) != {"path", "sha256"}:
      raise ValueError("Each reference rlog requires path and sha256")
    path = source["path"]
    pure = PurePosixPath(path) if isinstance(path, str) else None
    if (pure is None or not path or "\\" in path or pure.is_absolute() or
        any(part in ("", ".", "..") for part in pure.parts) or pure.as_posix() != path or
        path in seen_paths or not isinstance(source["sha256"], str) or
        _HEX_SHA256.fullmatch(source["sha256"]) is None):
      raise ValueError("Reference rlog paths must be distinct normalized relative POSIX paths with lowercase SHA-256")
    seen_paths.add(path)
    normalized_rlogs.append({"path": path, "sha256": source["sha256"]})
  start = _finite_number(reference["start_mono"], "reference.start_mono")
  end = _finite_number(reference["end_mono"], "reference.end_mono")
  tolerance = _finite_number(reference["tolerance_m"], "reference.tolerance_m")
  if not start < end or tolerance <= 0:
    raise ValueError("Reference window must be increasing and tolerance must be positive")

  return {
    "format_version": _FORMAT_VERSION,
    "case_id": case_id,
    "reference": {
      "extract": {"path": extract_path, "sha256": extract["sha256"]},
      "rlogs": sorted(normalized_rlogs, key=lambda item: item["path"]),
      "start_mono": start,
      "end_mono": end,
      "tolerance_m": tolerance,
    },
  }


def _relative_npz_files(root):
  root_value = _path_text(root)
  _validate_ancestors(root_value)
  root = Path(root_value)
  root_link_type, root_is_reparse = _link_status(root)
  root = root.resolve()
  _validate_ancestors(root)
  if not root.is_dir():
    raise ValueError("Extract root must be an existing directory")
  files = []
  for directory, names, filenames in os.walk(root, followlinks=False):
    directory_path = Path(directory)
    if directory_path != root:
      _reject_link(directory_path)
    for name in names:
      _reject_link(directory_path / name)
    for name in filenames:
      path = directory_path / name
      _reject_link(path)
      if path.suffix.lower() == ".npz":
        if not path.is_file():
          raise ValueError(f"NPZ input must be a regular file: {path}")
        files.append(path)
  return root, sorted(files, key=lambda path: path.relative_to(root).as_posix()), {
    "caller_spelled_root": root_value,
    "root_link_type": root_link_type,
    "root_is_reparse": root_is_reparse,
    "resolved_root": str(root),
  }


def _path_text(path):
  raw = os.fspath(path)
  if isinstance(raw, bytes):
    return os.fsdecode(raw)
  if not isinstance(raw, str):
    raise TypeError("Extract root must be a path string or PathLike")
  return raw


def _validate_ancestors(path):
  """Reject links/reparse points in the caller's lexical path before resolve."""
  raw = _path_text(path)
  if any(part in (".", "..") for part in re.split(r"[\\/]", raw)):
    raise ValueError("Extract root must not contain lexical '.' or '..' components")
  lexical = Path(os.path.abspath(raw))
  current = lexical.parent
  while True:
    _reject_link(current)
    parent = current.parent
    if parent == current:
      break
    current = parent


def _is_reparse(path):
  try:
    attributes = getattr(os.lstat(path), "st_file_attributes", 0)
  except FileNotFoundError:
    return False
  return bool(attributes & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0))


def _link_status(path):
  is_symlink = path.is_symlink()
  is_reparse = _is_reparse(path)
  if is_symlink:
    link_type = "symlink"
  elif is_reparse:
    link_type = "reparse_point"
  else:
    link_type = "none"
  return link_type, is_reparse


def _reject_link(path):
  if path.is_symlink() or _is_reparse(path):
    raise ValueError(f"Extract search refuses symlinks and reparse points: {path}")


def _snapshot(root):
  root, paths, root_info = _relative_npz_files(root)
  rows = []
  for path in paths:
    relative = path.relative_to(root).as_posix()
    size = path.stat().st_size
    rows.append({"path": relative, "size_bytes": size, "sha256": sha256(path)})
  return root, rows, root_info


def _manifest_digest(rows):
  # JSON lines make the bound path set unambiguous while remaining compact.
  encoded = b"".join(
    (json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
    for row in rows
  )
  return hashlib.sha256(encoded).hexdigest()


def _column_index(columns, required, aliases):
  names = []
  for value in columns.tolist():
    if isinstance(value, bytes):
      try:
        value = value.decode("utf-8")
      except UnicodeDecodeError as error:
        raise ValueError("NPZ columns contain invalid UTF-8 bytes") from error
    if not isinstance(value, str):
      raise ValueError("NPZ columns must contain strings")
    names.append(value)
  if names.count(required) != 1:
    raise ValueError(f"NPZ schema requires exactly one {required!r} column")
  ambiguous = sorted(set(names) & aliases)
  if ambiguous:
    raise ValueError(f"Ambiguous NPZ schema: {required!r} and {ambiguous!r}")
  return names.index(required)


def _load_location(path):
  """Load and validate only the location arrays from one NPZ file."""
  try:
    with np.load(path, allow_pickle=False) as archive:
      keys = set(archive.files)
      if not {"data", "columns", "mono"}.issubset(keys):
        raise ValueError("NPZ schema requires data, columns and mono arrays")
      data, columns, mono = archive["data"], archive["columns"], archive["mono"]
  except (OSError, ValueError, KeyError, TypeError) as error:
    if isinstance(error, ValueError) and str(error).startswith(("NPZ schema", "Ambiguous")):
      raise
    raise ValueError(f"Unable to read NPZ location schema: {path}") from error
  if data.dtype != np.dtype(np.float32):
    raise ValueError(f"NPZ data must be float32: {path}")
  if mono.dtype != np.dtype(np.float64):
    raise ValueError(f"NPZ mono must be float64: {path}")
  if data.ndim != 2 or columns.ndim != 1 or mono.ndim != 1 or len(data) != len(mono) or len(columns) != data.shape[1]:
    raise ValueError(f"NPZ data, columns and mono must have compatible dimensions: {path}")
  if columns.dtype.kind not in "SU":
    raise ValueError(f"NPZ columns must be a string array: {path}")
  lat_index = _column_index(columns, "lat", _LAT_ALIASES)
  lon_index = _column_index(columns, "lon", _LON_ALIASES)
  lat, lon = data[:, lat_index], data[:, lon_index]
  if not np.isfinite(mono).all():
    raise ValueError(f"NPZ mono must be finite: {path}")
  finite = np.isfinite(lat) & np.isfinite(lon)
  if np.any(np.abs(lat[finite]) > 90) or np.any(np.abs(lon[finite]) > 180):
    raise ValueError(f"NPZ latitude or longitude is out of range: {path}")
  if len(mono) > 1 and np.any(np.diff(mono) <= 0):
    raise ValueError(f"NPZ mono must be strictly increasing: {path}")
  return lat, lon, mono, finite


def _point_distances(lat, lon, polyline):
  """Return point-to-polyline distances in metres using float32 positions."""
  origin_lat = np.float64(polyline[0, 0])
  lat_scale = np.float64(111320.0)
  lon_scale = lat_scale * np.cos(np.deg2rad(origin_lat))
  points = np.column_stack((lat.astype(np.float64), lon.astype(np.float64)))
  points = (points - polyline[0]) * np.array([lat_scale, lon_scale])
  vertices = (polyline - polyline[0]) * np.array([lat_scale, lon_scale])
  best = np.full(len(points), np.inf, dtype=np.float64)
  for first, second in zip(vertices, vertices[1:]):
    direction = second - first
    denominator = float(np.dot(direction, direction))
    projection = np.clip(np.sum((points - first) * direction, axis=1) / denominator, 0.0, 1.0)
    delta = points - (first + projection[:, None] * direction)
    best = np.minimum(best, np.sqrt(np.sum(delta * delta, axis=1)))
  return best


def _reference_polyline(root, reference, identities):
  """Derive the pinned reference geometry from a hash-verified extract."""
  descriptor = reference["extract"]
  relative = descriptor["path"]
  identity = next((row for row in identities if row["path"] == relative), None)
  if identity is None:
    raise ValueError("Reference extract is absent from the searched extract root")
  if identity["sha256"] != descriptor["sha256"]:
    raise ValueError("Reference extract SHA-256 does not match the pinned identity")
  path = root / PurePosixPath(relative)
  lat, lon, mono, valid = _load_location(path)
  in_window = (mono >= reference["start_mono"]) & (mono <= reference["end_mono"])
  selected = in_window & valid
  dropped = int(np.count_nonzero(in_window & ~valid))
  valid_mono = mono[valid]
  before = valid_mono[valid_mono <= reference["start_mono"]]
  after = valid_mono[valid_mono >= reference["end_mono"]]
  if not len(before) or not len(after):
    raise ValueError("Reference extract lacks valid localization coverage at a pinned window boundary")
  boundary_span = valid_mono[(valid_mono >= before[-1]) & (valid_mono <= after[0])]
  maximum_gap = float(np.max(np.diff(boundary_span), initial=0.0))
  if maximum_gap > 0.2:
    raise ValueError("Reference extract has a localization gap over 0.2 seconds in the pinned window")
  if np.count_nonzero(selected) < 2:
    raise ValueError("Reference extract has fewer than two samples in the pinned window")
  polyline = np.column_stack((lat[selected], lon[selected])).astype(np.float32)
  keep = np.ones(len(polyline), dtype=bool)
  keep[1:] = np.any(polyline[1:] != polyline[:-1], axis=1)
  polyline = polyline[keep]
  if len(polyline) < 2:
    raise ValueError("Reference extract has fewer than two unique points in the pinned window")
  return polyline, dropped, int(np.count_nonzero(in_window)), {
    "sample_count": int(np.count_nonzero(selected)),
    "boundary_mono": [float(before[-1]), float(after[0])],
    "maximum_mono_gap_s": maximum_gap,
  }


def _hit(path, identity, polyline, tolerance):
  lat, lon, mono, valid = _load_location(path)
  usable = int(np.count_nonzero(valid))
  dropped = len(valid) - usable
  if not usable:
    return None, dropped, len(valid)
  distances = _point_distances(lat[valid], lon[valid], polyline)
  near = distances <= tolerance
  if not np.any(near):
    return None, dropped, len(valid)
  valid_indices = np.flatnonzero(valid)
  indices = valid_indices[near]
  row_distances = np.full(len(valid), np.inf, dtype=np.float64)
  row_distances[valid_indices] = distances
  groups = np.split(indices, np.flatnonzero(np.diff(mono[indices]) > 3.0) + 1)
  result = {
    "path": identity["path"],
    "size_bytes": identity["size_bytes"],
    "sha256": identity["sha256"],
    "fragment": True,
    "traversals": [
      {
        "sample_count": int(len(group)),
        "minimum_distance_m": float(np.min(row_distances[group])),
        "mono": [float(mono[group[0]]), float(mono[group[-1]])],
      }
      for group in groups
    ],
  }
  if dropped:
    result["dropped_coordinate_rows"] = dropped
  return result, dropped, len(valid)


def build_inventory(request, extract_root):
  """Scan NPZ extracts and return a deterministic compact inventory."""
  request = _request(request)
  root, before, root_info = _snapshot(extract_root)
  polyline, reference_dropped, reference_rows, reference_window = _reference_polyline(root, request["reference"], before)
  reference_path = request["reference"]["extract"]["path"]
  hits = []
  unscannable = []
  partial_files = int(0 < reference_dropped < reference_rows)
  dropped_rows = reference_dropped
  for row in before:
    if row["path"] == reference_path:
      continue
    path = root / Path(row["path"])
    hit, dropped, total = _hit(path, row, polyline, request["reference"]["tolerance_m"])
    dropped_rows += dropped
    if 0 < dropped < total:
      partial_files += 1
    if dropped == total:
      unscannable.append({"path": row["path"], "size_bytes": row["size_bytes"],
                          "sha256": row["sha256"], "reason": "no_finite_coordinate_rows",
                          "dropped_coordinate_rows": dropped})
    if hit is not None:
      hits.append(hit)
  after_root, after, after_root_info = _snapshot(extract_root)
  if before != after or root != after_root or root_info != after_root_info:
    raise ValueError("NPZ inputs changed during geographic inventory; path set, size or bytes differ")
  return {
    "format_version": _FORMAT_VERSION,
    "kind": "mazda_ti_geographic_inventory",
    "status": "completed_search",
    "qualification": "coarse_candidate_only",
    "case_id": request["case_id"],
    "reference": request["reference"],
    "caller_spelled_root": root_info["caller_spelled_root"],
    "root_link_type": root_info["root_link_type"],
    "root_is_reparse": root_info["root_is_reparse"],
    "resolved_root": root_info["resolved_root"],
    "reference_window": reference_window,
    "searched_file_count": len(before),
    "hit_file_count": len(hits),
    "partial_file_count": partial_files,
    "dropped_coordinate_rows": dropped_rows,
    "unscannable_file_count": len(unscannable),
    "unscannable_files": unscannable,
    "candidate_raw_rlog_provenance": {
      "status": "unverified_not_supplied",
      "required_before_raw_verification_or_use": True,
      "note": "The NPZ path is not inferred as an rlog path.",
    },
    "manifest_encoding": "canonical-json-lines-v1:path,size_bytes,sha256",
    "manifest_sha256": _manifest_digest(before),
    "hits": sorted(hits, key=lambda item: item["path"]),
    "raw_float64_verification": {
      "required": True,
      "status": "required_before_road_naming_or_use",
      "tool": "tools.mazda_ti.geographic_match",
      "coordinate_precision": "float64",
      "rlogs": "declared_unverified_by_coarse_tool",
    },
  }


def run(request_path, extract_root, output):
  """Write one new inventory JSON file under a new output directory."""
  output = Path(output)
  if output.exists():
    raise FileExistsError(output)
  if not output.parent.is_dir():
    raise ValueError("Output parent must already exist")
  request_path = Path(request_path)
  payload = request_path.read_bytes()
  request = json.loads(payload, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"Non-finite JSON: {value}")))
  result = build_inventory(request, extract_root)
  if hashlib.sha256(request_path.read_bytes()).hexdigest() != hashlib.sha256(payload).hexdigest():
    raise ValueError("Request changed during geographic inventory")
  result["request_sha256"] = hashlib.sha256(payload).hexdigest()
  output.mkdir()
  with (output / "inventory.json").open("x", encoding="utf-8", newline="\n") as stream:
    json.dump(result, stream, sort_keys=True, indent=2, allow_nan=False)
    stream.write("\n")
  return result


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--request", type=Path, required=True)
  # Preserve the caller's spelling so lexical '.'/'..' components cannot be
  # normalized away before _relative_npz_files validates the boundary.
  parser.add_argument("--extract-root", type=str, required=True)
  parser.add_argument("--output", type=Path, required=True)
  args = parser.parse_args(argv)
  result = run(args.request, args.extract_root, args.output)
  print(f"{result['status']}: {result['hit_file_count']} coarse candidates; {args.output / 'inventory.json'}")


if __name__ == "__main__":
  main()
