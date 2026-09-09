# Mazda raw-evidence ingestion inventory

`python -m tools.mazda_ti.ingest` is an explicit collection tool. It is not called by replay or evaluation, and it never sends a message to a vehicle, changes software, or reads unrestricted Params.

The tool reads a supplied, immutable source tree. A production source may be a read-only mounted copy of the device log area; the known device location is `/data/media/0/realdata_konik`, where files are named `<route>--<segment>/rlog`. The existing device collection convention uses one throttled stream rather than unbounded per-segment transfers. Keep the source read-only and use the default 10,000,000 bytes/second limit when collecting. Unthrottled device pulls have caused reboots and are not a supported operation.

Raw rlogs, transfer directories, and generated inventories belong outside Git. Only this tool, its tests, and this contract are source-controlled.

## Source snapshot

Each discovered segment is a regular file at:

```text
SOURCE/<route>--<segment>/rlog
```

Optional context is an offline sidecar, never a Params dump:

```json
{
  "source_revision": "40-character lowercase Git revision",
  "expected_segments": [0, 1, 2],
  "settings": {
    "TiSteerMax": 600,
    "TorqueInterceptorEnabled": true
  }
}
```

Save it as `SOURCE/<route>.metadata.json`. Only the fourteen documented replay settings are copied into the inventory. Numeric settings reject booleans; the three documented toggle settings may use booleans or finite numeric values. Invalid setting values are listed in `invalid_settings`, while omitted settings are in `missing_settings`. A source revision must be a full 40-character lowercase commit hash; an absent or invalid value remains visible in `unknown_evidence`. Arbitrary keys, including tokens and unrestricted Params, are discarded. A route without `expected_segments` has `completeness: "unknown"`. Missing expected segments produce `completeness: "incomplete"`; neither is a completed route.

## Commands

```text
python -m tools.mazda_ti.ingest discover --source-root SOURCE --output NEW-inventory.json

python -m tools.mazda_ti.ingest collect --inventory NEW-inventory.json --source-root SOURCE \
  --destination-root RAW-OUTSIDE-GIT --output NEW-collected-inventory.json \
  --segment ROUTE:5 --adjacent 1
```

`discover` hashes only the evidence it finds and writes a format-versioned inventory independent of evaluation-result schemas. `collect` is the separate, deliberate operation that copies selected segments. At least one `--segment ROUTE:SEGMENT` is required: collection has no bulk-transfer default. `--adjacent 1` includes the available segments immediately before and after each requested segment for warmup or input identity coverage. Missing neighbors, selected unknown segments, and selected unknown routes with their requested neighbors are recorded as missing coverage; the tool never invents them.

The transfer limit is capped at 10,000,000 bytes/second and recorded on each requested segment. A final rlog with the recorded hash is reported as `duplicate` and is not copied again. A `.part` file is resumed only when it is an exact prefix of the source; the completed file is promoted only after a SHA-256 check. Symlinked rlogs, metadata, transfer paths, destination partial files, and Windows reparse points such as junctions are rejected. Changed source bytes, a bad partial prefix, or a conflicting destination are recorded as corrupt and never silently replaced.

For deterministic fixture testing, `collect --stop-after-bytes N` writes a partial inventory and exits nonzero. Running `collect` again against that inventory/source/destination resumes the partial file. It is deliberately not part of normal collection guidance.

## Inventory contract

The JSON root is:

```text
format_version: 1
kind: "mazda_ti_ingestion_inventory"
routes[]: route_id, source_revision, settings, missing_settings, invalid_settings,
          unknown_evidence, missing_segments, requested_missing_segments,
          completeness, collection_status,
          segments[]
segments[]: segment, source_path, bytes, sha256, transfer_status, integrity,
            transfer_limit_bytes_per_second, resumed_from_bytes
```

The optional root `requested_missing_routes` records explicitly selected routes that were absent from discovery. `transfer_status` is one of `not_requested`, `partial`, `complete`, `duplicate`, or a `corrupt_*` reason. `integrity` is `unknown`, `incomplete`, `verified`, or `corrupt`. Corpus curation must keep incomplete, unknown, and corrupt entries visible and decide whether they are usable; this tool never treats any of them as evaluation coverage or evidence of vehicle behavior.
