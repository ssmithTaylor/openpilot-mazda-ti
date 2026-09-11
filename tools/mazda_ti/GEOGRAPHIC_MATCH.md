# Match a recorded road window

Use geographic matching before assigning a known corner name to a new drive
window. The matcher uses raw localization positions and velocity direction;
controller requests and lane outcomes do not select the alignment.

Create a JSON request with explicit paths relative to the raw data root:

```json
{
  "format_version": 1,
  "case_id": "known-corner",
  "reference": {
    "rlogs": ["reference-route--3/rlog", "reference-route--4/rlog"],
    "start": 262,
    "end": 285,
    "anchors": [262, 269, 277, 285]
  },
  "candidate": {
    "rlogs": ["candidate-route--15/rlog", "candidate-route--16/rlog"]
  }
}
```

Declare the reference window and at least three ordered anchors from road/video
evidence before searching. Supply one route per role and enough adjacent segments
to cover both boundaries. Search all relevant available segments: no match in a
partial file set cannot establish absence from the drive.

```text
python -m tools.mazda_ti.geographic_match --request match.json --data-root PATH/TO/raw --output evidence/geographic-match
```

The immutable output directory contains `result.json` and `report.md`. The result
retains the request byte hash, raw hashes, repository source map and runtime.
`completed_search` means the search executed, including when it found no matches.
Review each match's `accepted_geographic_candidate` and rejection reasons.

The search groups observations within 25 m of the second anchor, separated by
more than 3 seconds, then searches a 30-second window on each side. It checks
heading difference below 45 degrees, ordered anchors and endpoints within 30 m,
boundary coverage, localization health, and pose gaps no larger than 0.2 seconds.
The second anchor remains tied to its proximity group. These are retrieval
criteria, not lane-clearance or physical handling tolerances. Position uncertainty
norms are reported without a calibrated rejection threshold.

Confirm the road using video or independent road evidence before assigning its
name. Nearby parallel roads can still match. Keep speed, entry position, lane
identity, rider outcome and contact status separate for each pass. In particular,
a rider-confirmed intervention in the reference does not label the candidate.
Record the chosen bounds and then use the lane-context audit and qualified
controller replay to investigate that traversal. Preserve rejected matches and
failed baseline replays alongside any later successful evaluation.

## Coarse NPZ inventory

For a large directory of offline extracts, use `geographic_inventory.py` to
make a compact shortlist before opening raw rlogs. The request pins a case,
a SHA-256-locked reference NPZ, downstream raw-rlog pointers, window, and
tolerance. The tool derives the reference polyline from the verified NPZ
samples in the window:

```json
{
  "format_version": 1,
  "case_id": "known-corner",
  "reference": {
    "extract": {"path": "reference.npz", "sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"},
    "rlogs": [{"path": "reference--0/rlog", "sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}],
    "start_mono": 262.0,
    "end_mono": 285.0,
    "tolerance_m": 25.0
  }
}
```

The verified reference extract must provide valid coordinate samples on both
sides of the pinned start/end boundaries, with no localization gap over 0.2 s
across that span. The output records the boundary samples and maximum gap.

Each searched `.npz` must contain a two-dimensional `data` array with
`float32` dtype, a string `columns` array naming its fields, and a one-dimensional
`mono` array with `float64` dtype. The `columns` array must contain exactly one
`lat` and one `lon`; only those two data columns and `mono` affect geometry.
Controller, lane, and outcome columns are ignored for matching. Ambiguous schemas,
non-monotonic clocks, and malformed references are rejected. Non-finite
coordinate rows are dropped from matching and counted; a file with no usable
coordinate rows remains in `unscannable_files` with its path, size, hash, and
reason. Outcome, controller, and lane columns cannot affect geometry, but the
whole NPZ byte stream still contributes to its manifest SHA-256.
Candidate mono ranges may differ between files: the request's mono window pins
the reference definition and is not applied to candidate rows. Run it with:

```text
python -m tools.mazda_ti.geographic_inventory --request request.json --extract-root EXTRACTS --output NEW-INVENTORY
```

The output is refused if it already exists and contains only `inventory.json`.
It records `qualification: "coarse_candidate_only"`, a canonical digest of
every searched relative path, size, and SHA-256, and per-file identity only for
hits. A changed file or path set during the scan fails the run. The digest
binds the complete searched set even when a miss is omitted from the hit list.
Manifest paths are relative to the recorded `resolved_root`, including when the
caller supplied a final junction or symlink.
Each hit is a per-file fragment; this coarse tool does not join fragments across
files. The downstream raw matcher must group contiguous fragments after raw
verification. Lexical `.` and `..` components in `extract_root` are rejected
before resolution. The explicitly supplied final root may be a symlink or
junction; the output records its spelling, link/reparse status, and resolved
target. Reparse ancestors before that final component and all nested reparse
directories/files are rejected, so the manifest's searched set is complete.

This inventory never assigns a road name and never transfers lane, controller,
or outcome labels. The raw rlog hashes are retained as downstream verification
pointers and are unverified by this tool. Before naming or using any hit, rerun
the raw float64 `tools.mazda_ti.geographic_match` workflow above against those
pinned rlogs and confirm the road independently.
