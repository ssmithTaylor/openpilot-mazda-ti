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
