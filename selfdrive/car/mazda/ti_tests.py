"""On-car tests for the Mazda TI: definitions, their validation, and the progress accountant.

The tests are DATA: ti_tests.json next to this file, read at start by the car controller (this module) and by the
Tests screen in the UI (selfdrive/ui/qt/sidebar.cc, loadTiTests), so adding or changing a definition needs no build.
A test is "these existing boolean toggles at these values, and collect N seconds of a metric, optionally per speed
bin". The screen renders entirely from the file plus what this module publishes.

Safety envelope. A definition can only switch keys in ALLOWED_SETTINGS (code, not data), and only to the one value
listed there as "on"; "off" is always the other value. It cannot set numbers or limits. Nothing here reaches the
steering command: the toggles are read by the car controller exactly as when they are switched in Settings, and this
module only READS what the car controller already computed (speed, lateral active, whether the dither was applied
on this frame) and counts it.

Validation is strict and the same in the UI: an entry with anything unexpected is ignored as a whole and reported (a
log line here, a line on the screen); never a crash and never a partial apply. A test with an `alternate` block
(switching a setting automatically between arms) is checked for shape but not supported yet: it is ignored with the
reason "not supported yet". test_mazda_ti_tests.py checks that the C++ copy of the limits and the allow-list agrees.

Progress lives in ONE params key, PROGRESS_KEY, as JSON: {<test id>: {<bin>: seconds}}, or for a test with arms
{<test id>: {<arm>: {<bin>: seconds}}}, or for a step_seconds test (ti_ident.py) {<test id>: {<bin>: {<step>:
seconds}}}, <step> the step's index as a string; such a bin needs steps[k] x target_seconds in all. <bin> is a bin label from the definition, "other" (speeds outside every bin)
or "all" (a test without bins). The tmpfs copy (params_memory) also carries "_live": what this process saw in the
last stats interval ({seq, t: wall clock, on, active, status}); test ids cannot start with "_". The tmpfs copy is written at the
stats cadence (5 s) while a test is on; the flash copy (no "_live") at most once a minute while counting, plus once
IDLE_FLUSH_S after counting stops, so progress survives ignition-off and reboot. The accountant only ever ADDS what
it counted since its last publish to whatever the tmpfs copy holds (or, after a reboot, the flash copy), so the UI
resets a test by deleting its entry from both copies. So that a publish which read the record just before the
deletion cannot bring the entry back, the UI also leaves {"_reset": {<test id>: token}} in the tmpfs copy; the next
publish drops that entry again, removes the marker, flushes flash at once and echoes the token in
_live.reset_done; the UI repeats the request until it sees the echo (no second key).
"""
import json
import math
import re
import time
from pathlib import Path

DEFINITIONS_PATH = Path(__file__).with_name("ti_tests.json")
PROGRESS_KEY = "TiTestProgress"
LIVE_KEY = "_live"
TABLE_KEY = "_table"   # in a step_seconds test's entry: ti_ident.SIGNAL_HASH of the signal its seconds were collected with
RESET_KEY = "_reset"
DT = 0.01
MS_TO_MPH = 1.0 / 0.44704

# The only param keys a definition may switch: param key -> (frogpilot_toggles attribute that shows what the car
# actually applies, the value that means "on"). Each is a plain boolean, exposed in Settings and read live by the car
# controller. Mirrored in selfdrive/ui/qt/sidebar.cc (TI_TEST_ALLOWED); test_mazda_ti_tests.py checks the two agree.
ALLOWED_SETTINGS = {"TiDither": ("ti_dither", True), "TiSweep": ("ti_sweep", True), "TiLadder": ("ti_ladder", True),
                    "TiPulses": ("ti_pulses", True), "TiIdentAll": ("ti_ident_all", True)}
# The run-all switch of the identification tests (ti_ident.py): allow-listed so ALL TESTS OFF and the screen can switch
# it, but no definition may use it; while it is on, every step_seconds test counts as switched on.
RUN_ALL_SETTING = "TiIdentAll"
RUN_ALL_TOGGLE = "ti_ident_all"
# Settings that report "applied on this frame" to the accountant (metric active_seconds).
ACTIVE_SOURCES = ("TiDither",)

# step_seconds: the identification tests of ti_ident.py, which credit whole clean chunks per (band, step) themselves.
METRICS = ("active_seconds", "engaged_seconds", "step_seconds")
BIN_BY = ("speed_mph",)
SWITCH_WHEN = ("straight_only", "between_engagements")

FILE_KEYS = {"version", "tests"}
TEST_KEYS = {"id", "title", "description", "settings", "progress", "auto_off", "alternate"}
TEST_REQUIRED = {"id", "title", "description", "settings", "progress"}
SETTING_KEYS = {"key", "on"}
PROGRESS_KEYS = {"metric", "target_seconds", "bin_by", "bins", "steps", "needs"}
PROGRESS_REQUIRED = {"metric", "target_seconds"}
AUTO_OFF_KEYS = {"at_ignition_off"}
ALTERNATE_KEYS = {"arms", "min_dwell_s", "switch_when"}
ARM_KEYS = {"id", "settings"}

MAX_TESTS = 4
MAX_SETTINGS = 3
MAX_BINS = 5
MAX_STEPS = 16
MIN_ARMS = 2
MAX_ARMS = 4
MIN_TARGET_S = 10
MAX_TARGET_S = 7200
MIN_DWELL_S = 30
MAX_DWELL_S = 3600
MAX_SPEED_MPH = 100
MAX_TITLE = 40
MAX_DESCRIPTION = 200
MAX_ID = 32
MAX_LABEL = 16
ID_RE = re.compile(r"[a-z][a-z0-9_]{0,31}")
LABEL_RE = re.compile(r"[A-Za-z0-9_.-]{1,16}")
OTHER_BIN = "other"
ALL_BIN = "all"
UNSUPPORTED = "alternate is not supported yet (test ignored)"

FLUSH_S = 60.0        # flash copy at most this often while counting
IDLE_FLUSH_S = 15.0   # ...and once this long after the last flash write when counting has stopped

# What the Tests screen shows as a test's status line, from the dither's gate (ti_dither.gate_reason). Display only.
REASON_TEXT = {
  "inactive": "Waiting: steering not engaged or interceptor not running",
  "off": "Waiting: the car has not applied the setting yet",
  "touch": "Paused: hand on the wheel",
  "rate": "Off: interceptor rate limit set too low for it",
  "blinker": "Paused: blinker or lane change",
  "holdoff": "Paused: 3 s after a touch or blinker",
  "speed": "Waiting: too slow",
  "command": "Waiting: steering command too large",
  "curve": "Waiting: curve",
  "ok": "Running",
  # the identification tests (ti_ident.TiIdent.reason)
  "other": "Waiting: another test is on",
  "band": "Waiting: speed below 19 or above 80 mph",
  "transition": "Waiting: 28-31 mph, steering mode changing",
  "gap": "Running between windows: logged, not counted",
  "input": "Off: a steering input is not a valid number",
  "regime": "Waiting: steering mode settling",
  "band_done": "Waiting: this speed window is complete",
  "extra": "Running: window complete, collecting extra",
  "lat": "Waiting: lateral demand above the limit",
  "switch": "Switching to the next test",
}


class TestDef:
  def __init__(self, id, title, description, settings, metric, target_seconds, bins, at_ignition_off, steps=(),
               needs=()):
    self.id = id
    self.title = title
    self.description = description
    self.settings = settings                 # ((param key, on value), ...)
    self.toggles = tuple((ALLOWED_SETTINGS[k][0], on) for k, on in settings)
    self.keys = frozenset(k for k, _ in settings)
    self.metric = metric
    self.target_seconds = target_seconds
    self.bins = bins                         # ((low mph, high mph, label, required), ...) sorted by low, or ()
    self.at_ignition_off = at_ignition_off
    self.steps = steps                       # step_seconds: steps to capture per bin, in the order of `bins`
    self.needs = needs                       # step_seconds: seconds each step needs per bin (0: optional, extra only)

  def enabled(self, toggles):
    """Every setting at its "on" value in the live toggles (what the car applies, after the tuning-level gate)."""
    for attr, on in self.toggles:
      v = getattr(toggles, attr, None)
      if not isinstance(v, bool) or v != on:
        return False
    return True

  def bin_for(self, speed_mph):
    """Half-open bins [low, high); speeds outside every bin count under OTHER_BIN. Unbinned: ALL_BIN."""
    if not self.bins:
      return ALL_BIN
    for low, high, label, _ in self.bins:
      if low <= speed_mph < high:
        return label
    return OTHER_BIN


def _number(x):
  return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _text(x, longest):
  return isinstance(x, str) and x.strip() != "" and len(x) <= longest


def _settings(raw):
  """(((key, on), ...), None) or (None, reason)."""
  if not isinstance(raw, list) or not 1 <= len(raw) <= MAX_SETTINGS:
    return None, f"settings must list 1-{MAX_SETTINGS} toggles"
  pairs = []
  for s in raw:
    if not isinstance(s, dict) or set(s) != SETTING_KEYS:
      return None, "each setting is exactly {key, on}"
    key, on = s["key"], s["on"]
    if not isinstance(key, str) or key not in ALLOWED_SETTINGS or key == RUN_ALL_SETTING:
      return None, f"setting {key!r} is not allowed"
    if not isinstance(on, bool) or on != ALLOWED_SETTINGS[key][1]:
      return None, f"setting {key}: on must be {str(ALLOWED_SETTINGS[key][1]).lower()}"
    if any(k == key for k, _ in pairs):
      return None, f"setting {key} twice"
    pairs.append((key, on))
  return tuple(pairs), None


def _bins(raw, steps=None, needs=None):
  """(((low, high, label, required), ...), None) or (None, reason); with `steps` (step_seconds), ((bins, steps),
  None), the step counts paired with the bins as written and kept in the sorted order."""
  if not isinstance(raw, list) or not 1 <= len(raw) <= MAX_BINS:
    return None, f"bins must list 1-{MAX_BINS} bins"
  if steps is not None:
    if not isinstance(steps, list) or len(steps) != len(raw):
      return None, "steps must list one count per bin"
    if not all(_number(n) and n == int(n) and 1 <= n <= MAX_STEPS for n in steps):
      return None, f"steps must be whole numbers 1-{MAX_STEPS}"
    if (not isinstance(needs, list) or len(needs) != len(raw) or
        any(not isinstance(n, list) or len(n) != int(k) or not all(_number(x) and 0 <= x <= MAX_TARGET_S for x in n)
            for n, k in zip(needs, steps))):
      return None, "needs must list each bin's seconds per step"
  out = []
  for i, b in enumerate(raw):
    if not isinstance(b, list) or len(b) != 4:
      return None, "each bin is [low, high, label, required]"
    low, high, label, required = b
    if not isinstance(label, str) or not LABEL_RE.fullmatch(label) or label in (OTHER_BIN, ALL_BIN):
      return None, "bad bin label"
    if not (_number(low) and _number(high) and 0 <= low < high <= MAX_SPEED_MPH):
      return None, f"bin {label}: need 0 <= low < high <= {MAX_SPEED_MPH}"
    if not isinstance(required, bool):
      return None, f"bin {label}: required must be true or false"
    out.append((float(low), float(high), label, required, int(steps[i]) if steps is not None else 0,
                tuple(float(x) for x in needs[i]) if steps is not None else ()))
  out.sort(key=lambda x: x[0])
  for a, c in zip(out, out[1:]):
    if c[0] < a[1]:
      return None, f"bins {a[2]} and {c[2]} overlap"
  if len({x[2] for x in out}) != len(out):
    return None, "bin label twice"
  if not any(x[3] for x in out):
    return None, "at least one bin must be required"
  if steps is not None:
    return (tuple(x[:4] for x in out), tuple(x[4] for x in out), tuple(x[5] for x in out)), None
  return tuple(x[:4] for x in out), None


def _alternate(a):
  """None when the block has the reserved shape, else the reason."""
  if not isinstance(a, dict) or set(a) != ALTERNATE_KEYS:
    return "alternate is exactly {arms, min_dwell_s, switch_when}"
  arms = a["arms"]
  if not isinstance(arms, list) or not MIN_ARMS <= len(arms) <= MAX_ARMS:
    return f"alternate: arms must list {MIN_ARMS}-{MAX_ARMS} arms"
  ids = set()
  for arm in arms:
    if not isinstance(arm, dict) or set(arm) != ARM_KEYS:
      return "alternate: each arm is exactly {id, settings}"
    if not isinstance(arm["id"], str) or not LABEL_RE.fullmatch(arm["id"]) or arm["id"] in ids:
      return "alternate: bad or repeated arm id"
    ids.add(arm["id"])
    _, why = _settings(arm["settings"])
    if why is not None:
      return f"alternate arm {arm['id']}: {why}"
  if not (_number(a["min_dwell_s"]) and MIN_DWELL_S <= a["min_dwell_s"] <= MAX_DWELL_S):
    return f"alternate: min_dwell_s must be {MIN_DWELL_S}-{MAX_DWELL_S} s"
  if a["switch_when"] not in SWITCH_WHEN:
    return "alternate: unknown switch_when"
  return None


def _ident_check(key, bins, steps, target, needs):
  try:
    from openpilot.selfdrive.car.mazda import ti_ident
  except Exception:
    return "identification tests not available"
  return ti_ident.check_definition(key, bins, steps, target, needs)


def validate_test(t):
  """(TestDef, None) or (None, reason). Anything unexpected rejects the whole entry; a well-formed `alternate` block
  rejects it too, with the reason UNSUPPORTED."""
  if not isinstance(t, dict):
    return None, "not an object"
  unknown = set(t) - TEST_KEYS
  if unknown:
    return None, f"unknown key {sorted(unknown)[0]}"
  missing = TEST_REQUIRED - set(t)
  if missing:
    return None, f"missing {sorted(missing)[0]}"
  if not isinstance(t["id"], str) or not ID_RE.fullmatch(t["id"]):
    return None, f"id must be a-z then up to {MAX_ID - 1} of a-z 0-9 _"
  if not _text(t["title"], MAX_TITLE):
    return None, f"title must be 1-{MAX_TITLE} characters"
  if not _text(t["description"], MAX_DESCRIPTION):
    return None, f"description must be 1-{MAX_DESCRIPTION} characters"

  settings, why = _settings(t["settings"])
  if why is not None:
    return None, why

  p = t["progress"]
  if not isinstance(p, dict):
    return None, "progress must be an object"
  unknown = set(p) - PROGRESS_KEYS
  if unknown:
    return None, f"unknown progress key {sorted(unknown)[0]}"
  missing = PROGRESS_REQUIRED - set(p)
  if missing:
    return None, f"progress: missing {sorted(missing)[0]}"
  if p["metric"] not in METRICS:
    return None, "unknown metric"
  if p["metric"] == "active_seconds" and not any(k in ACTIVE_SOURCES for k, _ in settings):
    return None, "active_seconds needs a setting that reports applied time"
  if not (_number(p["target_seconds"]) and MIN_TARGET_S <= p["target_seconds"] <= MAX_TARGET_S):
    return None, f"target_seconds must be {MIN_TARGET_S}-{MAX_TARGET_S}"
  bins, steps, needs = (), (), ()
  if ("bins" in p) != ("bin_by" in p):
    return None, "bins and bin_by go together"
  if ("steps" in p or "needs" in p) and p["metric"] != "step_seconds":
    return None, "steps go with step_seconds"
  if p["metric"] == "step_seconds" and not ("bins" in p and "steps" in p and "needs" in p):
    return None, "step_seconds needs bins, steps and needs"
  if "bins" in p:
    if p["bin_by"] not in BIN_BY:
      return None, "unknown bin_by"
    if p["metric"] == "step_seconds":
      got, why = _bins(p["bins"], p["steps"], p["needs"])
      if why is None:
        bins, steps, needs = got
    else:
      bins, why = _bins(p["bins"])
    if why is not None:
      return None, why
  if p["metric"] == "step_seconds":
    # The generator is code: the definition must describe exactly what it does (one setting, its bands, steps and
    # time per step), so the screen shows what the car really collects.
    if len(settings) != 1:
      return None, "step_seconds needs exactly one setting"
    why = _ident_check(settings[0][0], bins, steps, float(p["target_seconds"]), needs)
    if why is not None:
      return None, why

  at_off = False
  if "auto_off" in t:
    a = t["auto_off"]
    if not isinstance(a, dict) or set(a) - AUTO_OFF_KEYS or not all(isinstance(v, bool) for v in a.values()):
      return None, "auto_off is {at_ignition_off: true/false}"
    at_off = a.get("at_ignition_off", False)

  if "alternate" in t:
    why = _alternate(t["alternate"])
    return None, why if why is not None else UNSUPPORTED

  return TestDef(t["id"], t["title"], t["description"], settings, p["metric"], float(p["target_seconds"]), bins,
                 at_off, steps, needs), None


def validate_document(doc):
  """([TestDef], [error lines]). A bad file yields no tests; a bad entry is skipped on its own."""
  if not isinstance(doc, dict) or set(doc) != FILE_KEYS or not _number(doc["version"]) or doc["version"] != 1:
    return [], ["file: must be exactly {\"version\": 1, \"tests\": [...]}"]
  if not isinstance(doc["tests"], list):
    return [], ["file: tests must be a list"]
  tests, errors, ids = [], [], set()
  for i, t in enumerate(doc["tests"]):
    name = t["id"] if isinstance(t, dict) and isinstance(t.get("id"), str) and t["id"] else f"#{i + 1}"
    if i >= MAX_TESTS:
      errors.append(f"{name}: more than {MAX_TESTS} tests")
      continue
    test, why = validate_test(t)
    if test is None:
      errors.append(f"{name}: {why}")
    elif test.id in ids:
      errors.append(f"{name}: id twice")
    else:
      ids.add(test.id)
      tests.append(test)
  return tests, errors


def load_definitions(path=DEFINITIONS_PATH):
  try:
    with open(path, encoding="utf-8") as f:
      doc = json.load(f)
  except (OSError, ValueError) as e:
    return [], [f"file: cannot read ({type(e).__name__})"]
  return validate_document(doc)


def parse_progress(raw):
  """The progress mapping, cleaned, or None when absent or unusable. Entries of tests that are no longer defined are
  kept (they keep their numbers until reset); bad values are dropped; top-level keys starting with "_" (the live
  block) are left out."""
  if not raw:
    return None
  try:
    doc = json.loads(raw)
  except (ValueError, TypeError):
    return None
  if not isinstance(doc, dict):
    return None
  out = {}
  for tid, entry in doc.items():
    if not isinstance(tid, str) or tid.startswith("_") or not isinstance(entry, dict):
      continue
    clean = {}
    for k, v in entry.items():
      if k == TABLE_KEY and isinstance(v, str):
        clean[k] = v                  # the signal table a step_seconds entry was collected with
      elif _number(v) and v >= 0:
        clean[k] = float(v)
      elif isinstance(v, dict):
        clean[k] = {b: float(x) for b, x in v.items() if _number(x) and x >= 0}
    out[tid] = clean
  return out


def parse_resets(raw):
  """The UI's reset requests in the tmpfs copy: {test id: token}; empty when absent or unusable."""
  try:
    doc = json.loads(raw) if raw else None
  except (ValueError, TypeError):
    return {}
  marks = doc.get(RESET_KEY) if isinstance(doc, dict) else None
  if not isinstance(marks, dict):
    return {}
  return {tid: v for tid, v in marks.items() if isinstance(tid, str) and _number(v)}


def _log(msg):
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.warning(msg)
  except Exception:
    pass


class TestAccountant:
  """Counts each enabled test's metric per bin. update() runs every frame and is a handful of comparisons; publish()
  runs at the stats cadence. Both only read their inputs. Any unexpected error disables the accountant for the rest
  of the process (logged once) instead of reaching the car controller."""

  def __init__(self, tests, params, params_memory):
    self.tests = list(tests)
    self.params = params
    self.params_memory = params_memory
    self.pending = {}            # test id -> {(arm or None, bin): frames} counted since the last publish
    self.seen_interval = set()   # enabled at any frame since the last publish
    self.run_seen = set()        # enabled at any frame of the current measurement run (stats stamp)
    self.live_shown = False      # the last live block published listed a test as on
    self.unsaved = False
    self.last_flush = None
    self.seq = 0
    self.reset_done = {}         # test id -> the UI's reset token this process applied (echoed in the live block)
    self.pending_steps = {}      # test id -> {(band, step): seconds} credited since the last publish (step_seconds)
    self.doc = None              # the progress as of the last publish (step_seconds bookkeeping); None: not read yet
    self.step_tests = {k: t for t in self.tests if t.metric == "step_seconds" for k in t.keys}
    self.active_interval = set()  # step_seconds tests that ran clean at any frame since the last publish
    self.all_seen = False        # run all was on at any frame since the last publish
    self.failed = False

  def _fail(self, where):
    self.failed = True
    _log(f"ti_tests: accountant disabled after an error in {where}")

  def update(self, toggles, v_ego, engaged, active_keys, arms=None, credit=None):
    """One frame. `active_keys`: the ACTIVE_SOURCES applied on this frame. `arms`: {test id: arm} for tests that
    alternate (none can be loaded yet), else None. `credit`: (setting key, band, step, seconds) when an
    identification test (ti_ident) completed a clean chunk on this frame, else None."""
    if self.failed:
      return
    try:
      run_all = getattr(toggles, RUN_ALL_TOGGLE, None) is True
      self.all_seen = self.all_seen or run_all
      for t in self.tests:
        if not (t.enabled(toggles) or (run_all and t.metric == "step_seconds")):
          continue
        self.seen_interval.add(t.id)
        self.run_seen.add(t.id)
        if t.metric == "step_seconds":
          # "active" for the screen while it runs clean, not only on the frames a unit is credited
          if not t.keys.isdisjoint(active_keys):
            self.active_interval.add(t.id)
          if credit is not None and credit[0] in t.keys:
            slot = (credit[1], credit[2])
            counted = self.pending_steps.setdefault(t.id, {})
            counted[slot] = counted.get(slot, 0.0) + credit[3]
          continue
        counting = (not t.keys.isdisjoint(active_keys)) if t.metric == "active_seconds" else bool(engaged)
        if counting:
          arm = arms.get(t.id) if arms else None
          counted = self.pending.setdefault(t.id, {})
          slot = (arm, t.bin_for(v_ego * MS_TO_MPH))
          counted[slot] = counted.get(slot, 0) + 1
    except Exception:
      self._fail("update")

  def remaining(self, key, band, step):
    """Seconds still to capture for a step of the identification test switched by `key` in `band`: negative when it
    has extra; None when unknown (no such test, or a broken accountant), so nothing runs. Reads params once per process
    at most; never raises."""
    t = self.step_tests.get(key)
    if t is None or self.failed:
      return None
    try:
      if self.doc is None:
        self.doc = self._base()[0]
      entry = self.doc.get(t.id, {}).get(band, {})
      have = entry.get(str(step), 0.0) if isinstance(entry, dict) else 0.0
      have = (have if _number(have) else 0.0) + self.pending_steps.get(t.id, {}).get((band, step), 0.0)
      k = next(i for i, b in enumerate(t.bins) if b[2] == band)
      return t.needs[k][step] - have
    except Exception:
      self._fail("remaining")
      return None

  def wants_reasons(self):
    return bool(self.seen_interval) and not self.failed

  def _signal(self, t):
    try:
      from openpilot.selfdrive.car.mazda import ti_ident
      return ti_ident.SIGNAL_HASH[next(iter(t.keys))]
    except Exception:
      return None

  def _migrate(self, doc):
    """A step_seconds entry collected with another signal table (or before tables were recorded, as on fd05555) is
    kept under "<id>~<its table or 'old'>" and the test starts again from zero under its id; seconds collected with
    another waveform never count toward the current needs."""
    for t in self.tests:
      if t.metric != "step_seconds":
        continue
      table = self._signal(t)
      entry = doc.get(t.id)
      if not isinstance(entry, dict) or table is None or entry.get(TABLE_KEY) == table:
        continue
      if any(k != TABLE_KEY for k in entry):
        old = f"{t.id}~{entry.get(TABLE_KEY) or 'old'}"
        n = 2
        while old in doc:
          old = f"{t.id}~{entry.get(TABLE_KEY) or 'old'}{n}"
          n += 1
        doc[old] = entry
      doc[t.id] = {TABLE_KEY: table}
    return doc

  def _base(self):
    """(progress, resets): the tmpfs copy and the UI's reset requests in it, else the flash copy and none."""
    raw = self.params_memory.get(PROGRESS_KEY, encoding="utf8")
    doc = parse_progress(raw)
    if doc is not None:
      return self._migrate(doc), parse_resets(raw)
    doc = parse_progress(self.params.get(PROGRESS_KEY, encoding="utf8"))
    return self._migrate(doc if doc is not None else {}), {}

  def _status(self, on, reasons, notes=None):
    status = {}
    for t in self.tests:
      if t.id in on:
        for key in sorted(t.keys):
          text = REASON_TEXT.get((reasons or {}).get(key))
          if text is not None:
            status[t.id] = (text + (notes or {}).get(key, ""))[:80]
            break
    return status

  def publish(self, now, reasons=None, wall=None, notes=None, run_all=None):
    """`now`: a monotonic clock in seconds. `reasons`: {setting key: ti_dither.gate_reason code}. `wall`: the wall
    clock for the live block's "t" (default time.time()); the UI compares it with its own clock to tell a live block
    from one left over by an earlier drive. `notes`: {setting key: text appended to its status line}. `run_all`:
    (setting key of the test running now or None, reason code) while run all is on, shown as the live block's "all"
    {on, test, status}. Writes the tmpfs copy when a test was on, something is unsaved or the last
    live block still lists a test as on, and the flash copy at the rate in the module docstring."""
    if self.failed or not (self.pending or self.pending_steps or self.seen_interval or self.unsaved or
                           self.live_shown or self.all_seen):
      return
    try:
      doc, resets = self._base()
      # The UI's "Reset progress": drop the entry here too, so a publish that read the record just before the UI
      # deleted it cannot bring it back, and say so in the live block (the UI repeats the request until it sees it)
      for tid, token in resets.items():
        doc.pop(tid, None)
        self.reset_done[tid] = token
      changed = bool(self.pending) or bool(self.pending_steps) or bool(resets)
      for tid, counted in self.pending.items():
        entry = doc.setdefault(tid, {})
        for (arm, b), n in counted.items():
          into = entry
          if arm is not None:
            into = entry.get(arm)
            if not isinstance(into, dict):
              into = entry[arm] = {}
          prev = into.get(b)
          into[b] = round((prev if _number(prev) else 0.0) + n * DT, 2)
      for tid, counted in self.pending_steps.items():
        entry = doc.setdefault(tid, {})
        t = next((x for x in self.tests if x.id == tid), None)
        if t is not None and self._signal(t) is not None:
          entry[TABLE_KEY] = self._signal(t)
        for (b, step), secs in counted.items():
          into = entry.get(b)
          if not isinstance(into, dict):
            into = entry[b] = {}
          prev = into.get(str(step))
          into[str(step)] = round((prev if _number(prev) else 0.0) + secs, 2)
      on = sorted(self.seen_interval)
      active = sorted(set(self.pending) | set(self.pending_steps) | self.active_interval)
      all_block = None
      if self.all_seen:
        key, code = run_all if run_all is not None else (None, None)
        t = self.step_tests.get(key)
        text = REASON_TEXT.get(code, "")
        all_block = {"on": True, "test": t.id if t is not None else "",
                     "status": ((t.title + ": " if t is not None else "") + text + (notes or {}).get(key, ""))[:80]}
      self.pending, self.pending_steps, self.seen_interval, self.active_interval = {}, {}, set(), set()
      self.all_seen = False
      self.doc = doc
      self.seq += 1
      live = {"seq": self.seq, "t": round(time.time() if wall is None else wall, 1), "on": on, "active": active,
              "status": self._status(on, reasons, notes), "reset_done": dict(self.reset_done)}
      if all_block is not None:
        live["all"] = all_block
      self.params_memory.put(PROGRESS_KEY, json.dumps({**doc, LIVE_KEY: live}))
      self.live_shown = bool(on)
      self.unsaved = self.unsaved or changed
      if self.unsaved:
        since = None if self.last_flush is None else now - self.last_flush
        if since is None or since >= FLUSH_S or (not changed and since >= IDLE_FLUSH_S) or resets:
          self.params.put_nonblocking(PROGRESS_KEY, json.dumps(doc))
          self.last_flush = now
          self.unsaved = False
    except Exception:
      self._fail("publish")


def make_accountant(params, params_memory, path=DEFINITIONS_PATH):
  """The accountant for the valid tests in the file, or None when there are none. Never raises."""
  try:
    tests, errors = load_definitions(path)
    for e in errors:
      _log(f"ti_tests: definition ignored: {e}")
    return TestAccountant(tests, params, params_memory) if tests else None
  except Exception:
    _log("ti_tests: definitions not loaded")
    return None
