"""On-car test definitions (ti_tests.json), their validation, the progress accountant, and its use by the car
controller."""
import ast
import copy
import importlib.util
import json
import re
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car.mazda import ti_dither as td
from openpilot.selfdrive.car.mazda import ti_tests as tt

ROOT = Path(__file__).resolve().parents[3]
MPH = 0.44704


class FakeParams:
  def __init__(self):
    self.store, self.writes, self.nonblocking = {}, 0, 0

  def get(self, key, encoding=None):
    assert key == tt.PROGRESS_KEY
    return self.store.get(key)

  def put(self, key, value):
    assert key == tt.PROGRESS_KEY
    self.store[key] = value
    self.writes += 1

  def put_nonblocking(self, key, value):
    assert key == tt.PROGRESS_KEY
    self.store[key] = value
    self.nonblocking += 1


def binned(**over):
  t = {"id": "t1", "title": "Test one", "description": "d", "settings": [{"key": "TiDither", "on": True}],
       "progress": {"metric": "active_seconds", "target_seconds": 60, "bin_by": "speed_mph",
                    "bins": [[36, 40, "a", False], [43, 50, "b", True]]}}
  t.update(over)
  return t


def unbinned(**over):
  t = {"id": "t2", "title": "Test two", "description": "x", "settings": [{"key": "TiDither", "on": True}],
       "progress": {"metric": "engaged_seconds", "target_seconds": 120}}
  t.update(over)
  return t


def alternate(**over):
  a = {"arms": [{"id": "on", "settings": [{"key": "TiDither", "on": True}]},
                {"id": "off", "settings": [{"key": "TiDither", "on": True}]}],
       "min_dwell_s": 120, "switch_when": "straight_only"}
  a.update(over)
  return a


def doc(*tests):
  return {"version": 1, "tests": list(tests)}


def progress(**p):
  t = binned()
  t["progress"] = p
  return t


ON = NS(ti_dither=True)
OFF = NS(ti_dither=False)


# ---- the shipped file ------------------------------------------------------------------------------------------

def test_shipped_definitions_are_valid_and_define_the_dither_bands():
  tests, errors = tt.load_definitions()
  assert errors == []
  # The queue order on the Tests screen: the dither, then the three identification tests (ti_ident.py)
  assert [t.id for t in tests] == ["ti_dither", "ti_sweep", "ti_ladder", "ti_pulses"]
  d = tests[0]
  assert d.settings == (("TiDither", True),) and d.metric == "active_seconds" and d.target_seconds == 300
  assert d.bins == ((36, 40, "36-40", False), (43, 50, "43-50", True), (56, 62, "56-62", False))
  assert d.at_ignition_off                       # never stays armed silently across drives
  # Every bin starts above the dither's minimum speed, so all of it is reachable.
  assert min(b[0] for b in d.bins) * MPH > td.MIN_SPEED_MPS


# ---- validation ------------------------------------------------------------------------------------------------

def test_valid_binned_and_unbinned():
  tests, errors = tt.validate_document(doc(binned(), unbinned(auto_off={"at_ignition_off": True})))
  assert errors == [] and [t.id for t in tests] == ["t1", "t2"]
  assert tests[0].bins == ((36.0, 40.0, "a", False), (43.0, 50.0, "b", True))
  assert tests[1].bins == () and tests[1].target_seconds == 120 and tests[1].at_ignition_off


BIN = [[36, 40, "a", True]]


@pytest.mark.parametrize("entry", [
  binned(extra=1),                                                           # unknown key
  {k: v for k, v in binned().items() if k != "description"},                 # missing key
  binned(id="Bad-Id"), binned(id=""), binned(id="_live"), binned(id="9abc"), binned(id="a" * 33),
  binned(title=""), binned(title="   "), binned(title="x" * 41), binned(description=7), binned(description=""),
  binned(settings=[]), binned(settings={"key": "TiDither", "on": True}),
  binned(settings=[{"key": "LatDamping", "on": True}]),                     # existing, but not allow-listed
  binned(settings=[{"key": "NoSuchKey", "on": True}]),                      # not a param at all
  binned(settings=[{"key": "TiDither", "on": False}]),                      # not the allow-listed "on"
  binned(settings=[{"key": "TiDither", "on": 1}]),                          # not a boolean
  binned(settings=[{"key": "TiDither", "on": True}, {"key": "TiDither", "on": True}]),
  binned(settings=[{"key": "TiDither", "on": True, "value": 3}]),           # no numeric settings
  binned(settings=[{"key": "TiSteerMax", "on": True}]),                     # a limit: never
  progress(metric="nope", target_seconds=60),
  progress(target_seconds=60),                                               # missing metric
  progress(metric="active_seconds"),                                         # missing target
  progress(metric="active_seconds", target_seconds=5),                       # absurd targets
  progress(metric="active_seconds", target_seconds=99999),
  progress(metric="active_seconds", target_seconds=True),                    # a bool is not a number
  progress(metric="active_seconds", target_seconds=float("nan")),
  progress(metric="active_seconds", target_seconds="60"),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph"),  # bin_by without bins
  progress(metric="active_seconds", target_seconds=60, bins=BIN),            # bins without bin_by
  progress(metric="active_seconds", target_seconds=60, bin_by="rpm", bins=BIN),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=BIN * 6),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", extra=1, bins=BIN),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph",   # overlapping bins
           bins=[[36, 44, "a", True], [43, 50, "b", True]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph",   # all optional
           bins=[[36, 40, "a", False]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[[36, 40, "other", True]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[[36, 40, "all", True]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[[36, 40, "a b", True]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[[40, 36, "a", True]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[[36, 400, "a", True]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[[-1, 40, "a", True]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[[36, 40, "a", "yes"]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[[36, 40, "a"]]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph", bins=[{"low": 36}]),
  progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph",
           bins=[[36, 40, "a", True], [41, 45, "a", True]]),
  binned(auto_off={"at_ignition_off": 1}), binned(auto_off={"when_done": True}), binned(auto_off=True),
  binned(alternate=alternate(arms=[])), binned(alternate=alternate(min_dwell_s=1)),
  binned(alternate=alternate(switch_when="always")), binned(alternate=alternate(extra=1)),
  binned(alternate=alternate(arms=[{"id": "x", "settings": [{"key": "LatDamping", "on": True}]},
                                   {"id": "y", "settings": [{"key": "TiDither", "on": True}]}])),
  "not an object",
])
def test_an_invalid_entry_is_ignored_on_its_own(entry):
  tests, errors = tt.validate_document(doc(entry, unbinned()))
  assert [t.id for t in tests] == ["t2"]
  assert len(errors) == 1 and tt.UNSUPPORTED not in errors[0]


def test_a_well_formed_alternate_block_is_ignored_as_not_supported_yet():
  for when in tt.SWITCH_WHEN:
    tests, errors = tt.validate_document(doc(binned(alternate=alternate(switch_when=when)), unbinned()))
    assert [t.id for t in tests] == ["t2"]
    assert errors == [f"t1: {tt.UNSUPPORTED}"]


def test_engaged_seconds_needs_no_active_source_but_active_seconds_does(monkeypatch):
  monkeypatch.setattr(tt, "ACTIVE_SOURCES", ())
  tests, errors = tt.validate_document(doc(binned(), unbinned()))
  assert [t.id for t in tests] == ["t2"] and "applied time" in errors[0]


def test_adjacent_bins_are_not_overlapping_and_are_sorted():
  t = progress(metric="active_seconds", target_seconds=60, bin_by="speed_mph",
               bins=[[43, 50, "b", True], [36, 43, "a", False]])
  tests, errors = tt.validate_document(doc(t))
  assert errors == [] and [b[2] for b in tests[0].bins] == ["a", "b"]


@pytest.mark.parametrize("bad", [[], {"version": 2, "tests": []}, {"version": True, "tests": []},
                                 {"version": "1", "tests": []}, {"version": 1, "tests": {}},
                                 {"version": 1, "tests": [], "x": 1}, {"tests": []}])
def test_a_bad_file_yields_no_tests(bad):
  tests, errors = tt.validate_document(bad)
  assert tests == [] and len(errors) == 1


def test_duplicate_ids_and_too_many_tests():
  tests, errors = tt.validate_document(doc(binned(), binned()))
  assert len(tests) == 1 and errors == ["t1: id twice"]
  many = [unbinned(id=f"t{i}") for i in range(tt.MAX_TESTS + 2)]
  tests, errors = tt.validate_document(doc(*many))
  assert len(tests) == tt.MAX_TESTS and len(errors) == 2


def test_unreadable_or_missing_file(tmp_path):
  p = tmp_path / "ti_tests.json"
  p.write_text("{not json", encoding="utf-8")
  assert tt.load_definitions(p)[0] == [] and len(tt.load_definitions(p)[1]) == 1
  assert tt.load_definitions(tmp_path / "missing.json")[0] == []
  assert tt.make_accountant(FakeParams(), FakeParams(), p) is None
  assert tt.make_accountant(FakeParams(), FakeParams(), tmp_path / "missing.json") is None
  p.write_text(json.dumps(doc(binned(extra=1))), encoding="utf-8")
  assert tt.make_accountant(FakeParams(), FakeParams(), p) is None      # no valid test: no accountant at all


def test_validation_never_mutates_its_input():
  d = doc(binned(), unbinned(), binned(id="t3", alternate=alternate()))
  before = copy.deepcopy(d)
  tt.validate_document(d)
  assert d == before


# ---- the accountant --------------------------------------------------------------------------------------------

def accountant(*tests):
  defs, errors = tt.validate_document(doc(*tests))
  assert errors == []
  return tt.TestAccountant(defs, FakeParams(), FakeParams())


def frames(a, n, toggles=ON, mph=45.0, engaged=True, active=("TiDither",), arms=None):
  for _ in range(n):
    a.update(toggles, mph * MPH, engaged, active, arms)


def stored(store):
  return json.loads(store.store[tt.PROGRESS_KEY])


def live(a):
  return stored(a.params_memory)


def test_bin_edges_in_mps():
  t = tt.validate_document(doc(binned()))[0][0]
  # Half-open [low, high) in mph, applied to vEgo in m/s; a micro-m/s either side of each edge.
  e = 1e-6
  cases = [(36 * MPH - e, "other"), (36 * MPH + e, "a"), (40 * MPH - e, "a"), (40 * MPH + e, "other"),
           (43 * MPH - e, "other"), (43 * MPH + e, "b"), (50 * MPH - e, "b"), (50 * MPH + e, "other"),
           (25.0, "other"), (0.0, "other")]
  for v, want in cases:
    assert t.bin_for(v * tt.MS_TO_MPH) == want, v
  assert abs(43 * MPH - 19.22272) < 1e-9 and abs(50 * MPH - 22.352) < 1e-9
  assert abs(tt.MS_TO_MPH * 22.352 - 50.0) < 1e-9


def test_counts_only_applied_frames_while_enabled_and_keeps_optional_bins():
  a = accountant(binned())
  frames(a, 100)                                  # applied, 45 mph -> b (required)
  frames(a, 50, active=())                        # enabled, dither idle: nothing
  frames(a, 70, toggles=OFF)                      # not enabled: nothing, even if "active"
  frames(a, 30, mph=38)                           # applied, bin a (optional): counted and stored
  frames(a, 20, mph=60)                           # applied, outside every bin
  a.publish(0.0)
  d = live(a)
  assert d["t1"] == {"b": 1.0, "a": 0.3, "other": 0.2}
  assert {k: v for k, v in d["_live"].items() if k != "t"} == {"seq": 1, "on": ["t1"], "active": ["t1"], "status": {},
                                                                        "reset_done": {}}
  assert abs(d["_live"]["t"] - time.time()) < 60
  assert "_live" not in stored(a.params)          # the flash copy holds progress only


def test_nothing_counted_or_written_when_no_test_is_on():
  a = accountant(binned())
  frames(a, 500, toggles=OFF)
  frames(a, 500, toggles=NS())                    # toggle absent: not enabled
  frames(a, 500, toggles=NS(ti_dither=1))         # not a boolean: not enabled
  a.publish(0.0)
  a.publish(5.0)
  assert a.params_memory.writes == 0 and a.params.nonblocking == 0 and not a.run_seen and not a.pending


def test_engaged_metric_and_unbinned():
  a = accountant(unbinned())
  frames(a, 100, active=())                       # engaged counts without the dither applied
  frames(a, 100, engaged=False)
  a.publish(0.0)
  assert live(a)["t2"] == {"all": 1.0}


def test_per_arm_structure():
  a = accountant(binned(), unbinned())
  frames(a, 100, arms={"t1": "on"})
  frames(a, 50, arms={"t1": "off"}, mph=60)
  a.publish(0.0)
  d = live(a)
  assert d["t1"] == {"on": {"b": 1.0}, "off": {"other": 0.5}}
  assert d["t2"] == {"all": 1.5}                  # a test without an arm stays flat
  # and it survives a round trip through the parser
  assert tt.parse_progress(a.params_memory.store[tt.PROGRESS_KEY])["t1"] == d["t1"]


def test_live_block_seq_status_and_the_final_publish_after_switching_off():
  a = accountant(binned())
  frames(a, 10)
  a.publish(0.0, {"TiDither": "curve"})
  assert live(a)["_live"]["status"] == {"t1": tt.REASON_TEXT["curve"]}
  frames(a, 10, active=())                        # on, not applied
  a.publish(5.0, {"TiDither": "nonsense"}, 5.0)   # unknown code: no status line
  assert live(a)["_live"] == {"seq": 2, "t": 5.0, "on": ["t1"], "active": [], "status": {},
                              "reset_done": {}}
  frames(a, 10, toggles=OFF)                      # switched off: one more publish says so, then silence
  a.publish(10.0, None, 10.0)
  assert live(a)["_live"] == {"seq": 3, "t": 10.0, "on": [], "active": [], "status": {},
                              "reset_done": {}}
  writes = a.params_memory.writes
  for k in range(5):
    frames(a, 10, toggles=OFF)
    a.publish(100.0 + 5 * k)
  assert a.params_memory.writes == writes and not a.unsaved


def test_progress_accumulates_across_processes_and_reboots():
  a = accountant(binned())
  frames(a, 300)
  a.publish(0.0)
  assert a.params.nonblocking == 1                # first flush at once
  flash = a.params.store[tt.PROGRESS_KEY]
  # Ignition cycle without a reboot: tmpfs survives, a new process adds to it.
  b = tt.TestAccountant(a.tests, a.params, a.params_memory)
  frames(b, 200)
  b.publish(100.0)
  assert live(b)["t1"]["b"] == 5.0
  # Reboot: tmpfs gone, the flash copy is the base.
  c = tt.TestAccountant(a.tests, a.params, FakeParams())
  frames(c, 100)
  c.publish(0.0)
  assert json.loads(flash)["t1"]["b"] == 3.0
  assert live(c)["t1"]["b"] == 6.0


def test_flash_write_rate_is_low():
  a = accountant(binned())
  t = 0.0
  for _ in range(60):                             # five minutes of continuous counting, published every 5 s
    frames(a, 500)
    a.publish(t)
    t += 5.0
  assert a.params_memory.writes == 60
  assert a.params.nonblocking <= 6
  # Counting stops: the remainder is flushed within IDLE_FLUSH_S, then nothing more is written.
  before = a.params.nonblocking
  for _ in range(10):
    frames(a, 500, toggles=OFF)
    a.publish(t)
    t += 5.0
  assert a.params.nonblocking == before + 1
  assert stored(a.params) == {k: v for k, v in live(a).items() if k != "_live"}
  writes = a.params_memory.writes
  a.publish(t + 100)
  assert a.params_memory.writes == writes


def test_reset_is_clearing_the_entry():
  a = accountant(binned(), unbinned())
  frames(a, 300)
  a.publish(0.0)
  assert set(live(a)) == {"t1", "t2", "_live"}
  # What the UI does on "Reset progress": delete the test's entry from both copies, keep everything else.
  for store in (a.params_memory, a.params):
    d = stored(store)
    del d["t1"]
    store.store[tt.PROGRESS_KEY] = json.dumps(d)
  frames(a, 100)
  a.publish(100.0)
  assert live(a)["t1"] == {"b": 1.0}
  assert live(a)["t2"] == {"all": 4.0}


def test_entries_of_tests_no_longer_defined_are_kept():
  a = accountant(binned())
  a.params_memory.store[tt.PROGRESS_KEY] = json.dumps({"gone": {"x": 12.5}, "_live": {"seq": 9}})
  frames(a, 100)
  a.publish(0.0)
  assert live(a)["gone"] == {"x": 12.5} and live(a)["_live"]["seq"] == 1


@pytest.mark.parametrize("raw", ["", "nonsense", "[]", '{"t1": 5}', '{"t1": {"b": "x"}}', '{"t1": {"b": -3}}',
                                 '{"t1": {"b": {"c": "x"}}}', '{"t1": {"b": NaN}}'])
def test_malformed_stored_progress_reads_as_fresh(raw):
  a = accountant(binned())
  a.params_memory.store[tt.PROGRESS_KEY] = raw
  frames(a, 100)
  a.publish(0.0)
  assert live(a)["t1"]["b"] == 1.0


def test_an_internal_error_disables_the_accountant_instead_of_raising():
  a = accountant(binned())
  a.params_memory = None                          # anything at all going wrong in publish
  frames(a, 10)
  a.publish(0.0)
  assert a.failed
  frames(a, 10)
  a.publish(5.0)
  b = accountant(binned())
  b.update(ON, "fast", True, ("TiDither",))       # and in update
  assert b.failed


def test_run_stamp_lists_tests_that_were_on():
  a = accountant(binned())
  frames(a, 10, toggles=OFF)
  assert a.run_seen == set()
  frames(a, 1, active=())
  assert a.run_seen == {"t1"}



def ui_reset(store, tid, token):
  """What the UI's Reset progress does to the tmpfs copy (sidebar.cc tiTestMark): entry deleted, marker left."""
  d = json.loads(store.store[tt.PROGRESS_KEY])
  d.pop(tid, None)
  d.setdefault(tt.RESET_KEY, {})[tid] = token
  store.store[tt.PROGRESS_KEY] = json.dumps(d)


def test_reset_marker_is_honoured_echoed_removed_and_flushed():
  a = accountant(binned(), unbinned())
  frames(a, 300)
  a.publish(0.0)
  flushes = a.params.nonblocking
  ui_reset(a.params_memory, "t1", 1234567890123)
  frames(a, 100)
  a.publish(5.0)                                  # within FLUSH_S of the last flush, but a reset flushes at once
  d = live(a)
  assert d["t1"] == {"b": 1.0}                    # only what was counted since
  assert d["t2"] == {"all": 4.0}                  # other tests untouched
  assert tt.RESET_KEY not in d and d["_live"]["reset_done"] == {"t1": 1234567890123}
  assert a.params.nonblocking == flushes + 1 and stored(a.params)["t1"] == {"b": 1.0}
  assert tt.RESET_KEY not in stored(a.params)


def test_reset_survives_a_publish_that_overwrote_it():
  # The race: the car side read the record, the UI deleted the entry and left its marker, the car side wrote the
  # record back as it had read it (entry back, marker gone). The UI sees no echo and no marker and asks again; the
  # next publish drops the entry.
  a = accountant(binned())
  frames(a, 300)
  a.publish(0.0)
  before = a.params_memory.store[tt.PROGRESS_KEY]
  ui_reset(a.params_memory, "t1", 7)
  a.params_memory.store[tt.PROGRESS_KEY] = before                 # overwritten by the racing publish
  assert tt.RESET_KEY not in live(a) and 7 not in live(a)["_live"]["reset_done"].values()
  ui_reset(a.params_memory, "t1", 7)                              # the UI repeats it
  frames(a, 10)
  a.publish(5.0)
  assert live(a)["t1"] == {"b": 0.1} and live(a)["_live"]["reset_done"] == {"t1": 7}


def test_reset_after_a_reboot_keeps_the_other_tests():
  # After a reboot the tmpfs copy is empty; the UI seeds it from the flash copy (sidebar.cc tiTestMark), so the car
  # side, which then reads the tmpfs copy, still has every other entry.
  a = accountant(binned(), unbinned())
  a.params.store[tt.PROGRESS_KEY] = json.dumps({"t1": {"b": 9.0}, "t2": {"all": 50.0}})
  seeded = json.loads(a.params.store[tt.PROGRESS_KEY])
  seeded.pop("t1")
  seeded[tt.RESET_KEY] = {"t1": 3}
  a.params_memory.store[tt.PROGRESS_KEY] = json.dumps(seeded)
  frames(a, 100)
  a.publish(0.0)
  assert live(a)["t1"] == {"b": 1.0} and live(a)["t2"] == {"all": 51.0}
  assert stored(a.params) == {"t1": {"b": 1.0}, "t2": {"all": 51.0}}


def test_a_reset_marker_only_in_flash_or_malformed_is_ignored():
  a = accountant(binned())
  a.params.store[tt.PROGRESS_KEY] = json.dumps({"t1": {"b": 9.0}, tt.RESET_KEY: {"t1": 1}})
  frames(a, 100)
  a.publish(0.0)
  assert live(a)["t1"]["b"] == 10.0 and live(a)["_live"]["reset_done"] == {}
  for marks in ('"x"', '{"t1": "y"}', '[1]'):
    b = accountant(binned())
    b.params_memory.store[tt.PROGRESS_KEY] = '{"t1": {"b": 2.0}, "%s": %s}' % (tt.RESET_KEY, marks)
    frames(b, 100)
    b.publish(0.0)
    assert live(b)["t1"]["b"] == 3.0

# ---- the dither's display-only gate reason ---------------------------------------------------------------------

QUIET = dict(engaged=True, enabled=True, v_ego=25.0, steering_pressed=False, driver_torque=-1.0, blinker=False,
             base_request=50, ti_max=600, lat_demand=0.1, holdoff=0, room=6)


@pytest.mark.parametrize("change,code", [
  (dict(engaged=False), "inactive"), (dict(enabled=False), "off"), (dict(steering_pressed=True), "touch"),
  (dict(driver_torque=9.0), "touch"), (dict(room=1), "rate"), (dict(blinker=True), "blinker"),
  (dict(holdoff=10), "holdoff"), (dict(v_ego=12.0), "speed"), (dict(base_request=400), "command"),
  (dict(lat_demand=2.0), "curve"), ({}, "ok")])
def test_gate_reason_codes_have_texts(change, code):
  assert td.gate_reason(**{**QUIET, **change}) == code
  assert code in tt.REASON_TEXT


def test_gate_reason_uses_the_confirmed_touch_when_given():
  blip = {**QUIET, "driver_torque": 4.0}
  assert td.gate_reason(**blip) == "touch"                    # no state given: every reading counts
  assert td.gate_reason(**blip, touch=False) == "ok"          # the dither's own verdict: not confirmed
  assert td.gate_reason(**QUIET, touch=True) == "touch"


def test_gate_reason_ok_exactly_when_the_dither_would_fade_in():
  for change in [{}, dict(v_ego=12.0), dict(blinker=True), dict(lat_demand=2.0), dict(base_request=400)]:
    args = {**QUIET, **change}
    d = td.TiDither()
    for k in range(200):
      out = d.update(True, k, v_ego=args["v_ego"], steering_pressed=False, driver_torque=-1.0, blinker=args["blinker"],
                     base_request=args["base_request"], ti_max=600, lat_demand=args["lat_demand"], room=6)
    reason = td.gate_reason(**{**args, "holdoff": d.holdoff})
    assert (reason == "ok") == (d.envelope > 0.0), (change, reason)
    assert out == d.last


# ---- the car controller's use of it ----------------------------------------------------------------------------

def _method(name, **env):
  tree = ast.parse((ROOT / 'selfdrive/car/mazda/carcontroller.py').read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CarController')
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
  exec(compile(ast.Module(body=[fn], type_ignores=[]), name, 'exec'), env)
  return env[name]


def test_stats_stamp_names_tests_only_when_one_was_on():
  cfg = _method('ti_config', TI_DITHER_PRESET=td.ACTIVE_PRESET)
  ccp = NS(TI_STEER_MAX=600, TI_STEER_DELTA_UP=6, TI_STEER_DELTA_DOWN=15, TI_STEER_DRIVER_ALLOWANCE=15,
           TI_STEER_DRIVER_MULTIPLIER=40, TI_STEER_DELTA_UP_KNEE=600, TI_STEER_DELTA_UP_HIGH=6)
  h = NS(ccp=ccp, ti_steer_threshold=None, ti_dither_seen=False, ti_dither_frames=0, ti_tests=accountant(binned()))
  base = cfg(h)
  assert 'Tests' not in base
  frames(h.ti_tests, 1)
  assert cfg(h)['Tests'] == ['t1']
  assert {k: v for k, v in cfg(h).items() if k != 'Tests'} == base
  h.ti_tests = None                               # not a TI car, or no valid definition: unchanged payload
  assert cfg(h) == base
  del h.ti_tests                                  # older host without tests: unchanged payload
  assert cfg(h) == base


def test_publish_passes_the_dither_reason_only_while_a_test_is_on():
  calls = []
  pub = _method('publish_ti_tests', gate_reason=lambda **kw: calls.append(kw) or "curve", rate_room=td.rate_room,
                time=NS(monotonic=lambda: 0.0))
  ccp = NS(TI_STEER_MAX=600, TI_STEER_DELTA_UP=6, TI_STEER_DELTA_DOWN=15, TI_STEER_DELTA_UP_KNEE=600,
           TI_STEER_DELTA_UP_HIGH=6)
  h = NS(ti_tests=accountant(binned()), ti_dither_enabled=True, ti_dither=td.TiDither(), ccp=ccp)
  cs = NS(out=NS(vEgo=25.0, steeringPressed=False, steeringTorque=0.0, leftBlinker=False, rightBlinker=False),
          ti_lkas_allowed=True)
  cc = NS(latActive=True, leftBlinker=False, rightBlinker=False, actuators=NS(curvature=0.001))
  pub(h, cs, cc, 40)
  assert calls == []                              # nothing on: no reason computed, nothing written
  frames(h.ti_tests, 5, active=())
  pub(h, cs, cc, 40)
  assert len(calls) == 1 and calls[0]["base_request"] == 40
  assert live(h.ti_tests)["_live"]["status"] == {"t1": tt.REASON_TEXT["curve"]}
  h.ti_tests = None
  pub(h, cs, cc, 40)                              # no accountant: a no-op


def test_the_accountant_is_outside_the_steering_block():
  # The extracted steering block (what test_mazda_ti_dither.py and base_identity.py execute) must not reference it.
  spec = importlib.util.spec_from_file_location('td_tests', ROOT / 'selfdrive/car/tests/test_mazda_ti_dither.py')
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  names = set(m.BLOCK.co_names)
  for code in m.BLOCK.co_consts:
    if hasattr(code, 'co_names'):
      names |= set(code.co_names)
  assert 'ti_tests' not in names and 'publish_ti_tests' not in names and 'TI_DITHER_ACTIVE' not in names



def test_a_missing_or_broken_tests_module_leaves_the_controller_without_tests(monkeypatch):
  import sys as _sys
  tree = ast.parse((ROOT / 'selfdrive/car/mazda/carcontroller.py').read_text(encoding='utf-8'))
  guard = next(n for n in tree.body if isinstance(n, ast.Try)
               and any(isinstance(x, ast.ImportFrom) and x.module.endswith('ti_tests') for x in ast.walk(n)))
  monkeypatch.setitem(_sys.modules, 'openpilot.selfdrive.car.mazda.ti_tests', None)   # import raises ImportError
  env = {}
  exec(compile(ast.Module(body=[guard], type_ignores=[]), 'guard', 'exec'), env)
  assert env['make_accountant'] is None
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CarController')
  init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
  assign = next(n for n in init.body if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Attribute) and t.attr == 'ti_tests' for t in n.targets))
  h = NS(params=None, params_memory=None)
  exec(compile(ast.Module(body=[assign], type_ignores=[]), 'init', 'exec'),
       dict(self=h, CP=NS(flags=8), MazdaFlags=NS(TORQUE_INTERCEPTOR=8), make_accountant=None))
  assert h.ti_tests is None
  # and the module itself imports normally in this tree
  monkeypatch.delitem(_sys.modules, 'openpilot.selfdrive.car.mazda.ti_tests')
  env = {}
  exec(compile(ast.Module(body=[guard], type_ignores=[]), 'guard', 'exec'), env)
  assert env['make_accountant'] is tt.make_accountant or env['make_accountant'].__name__ == 'make_accountant'


def test_progress_is_kept_out_of_toggle_resets_and_backups():
  variables = (ROOT / 'frogpilot/common/frogpilot_variables.py').read_text(encoding='utf-8')
  excluded = re.search(r'^EXCLUDED_KEYS = \{(.*?)\}', variables, re.S | re.M).group(1)
  assert '"%s"' % tt.PROGRESS_KEY in excluded

# ---- registration ----------------------------------------------------------------------------------------------

def test_progress_key_and_allow_listed_settings_are_registered():
  cc = (ROOT / 'common/params.cc').read_text(encoding='utf-8')
  variables = (ROOT / 'frogpilot/common/frogpilot_variables.py').read_text(encoding='utf-8')
  lateral = (ROOT / 'frogpilot/ui/qt/offroad/lateral_settings.cc').read_text(encoding='utf-8')
  live_keys = re.search(r'liveUpdateKeys = \{([^}]*)\}', lateral).group(1)
  assert len(re.findall(r'\{"%s",\s*PERSISTENT\}' % tt.PROGRESS_KEY, cc)) == 1
  assert len(re.findall(r'\("%s", "", 3, ""\)' % tt.PROGRESS_KEY, variables)) == 1
  for key, (toggle, on) in tt.ALLOWED_SETTINGS.items():
    assert isinstance(on, bool)
    assert re.search(r'\{"%s",\s*PERSISTENT\}' % key, cc)
    # a plain boolean with a default of "off", read into the live toggles under this name, and notified live
    assert re.search(r'\("%s", "%d", \d, "[01]"\)' % (key, int(not on)), variables)
    assert re.search(r'toggle\.%s = \(params\.get_bool\("%s"\)' % (toggle, key), variables)
    # The identification tests are switched from the Tests screen only (it notifies the toggles itself); the dither
    # also has a Settings toggle, which must notify live.
    if key == "TiDither":
      assert '"%s"' % key in live_keys
  assert set(tt.ACTIVE_SOURCES) <= set(tt.ALLOWED_SETTINGS)


# ---- agreement with the Qt copy (selfdrive/ui/qt/sidebar.cc) -------------------------------------------------

def _cc_source():
  return (ROOT / 'selfdrive/ui/qt/sidebar.cc').read_text(encoding='utf-8')


def test_qt_allow_list_agrees():
  block = re.search(r'TI_TEST_ALLOWED\[\] = \{(.*?)\};', _cc_source(), re.S).group(1)
  allowed = {k: (t, on == 'true') for k, t, on in re.findall(r'\{"(\w+)",\s*"(\w+)",\s*(true|false)\}', block)}
  assert allowed == tt.ALLOWED_SETTINGS


def test_qt_limits_agree():
  consts = {k: float(v) for k, v in re.findall(r'constexpr (?:int|double) TI_TEST_(\w+) = ([\d.]+);', _cc_source())}
  for name in ('MAX_TESTS', 'MAX_SETTINGS', 'MAX_BINS', 'MAX_STEPS', 'MIN_ARMS', 'MAX_ARMS', 'MIN_TARGET_S', 'MAX_TARGET_S',
               'MIN_DWELL_S', 'MAX_DWELL_S', 'MAX_SPEED_MPH', 'MAX_TITLE', 'MAX_DESCRIPTION', 'MAX_ID', 'MAX_LABEL'):
    assert consts.pop(name) == getattr(tt, name), name
  # UI-only: status line length, staleness of the live block (the car publishes every 5 s), NOT APPLIED delay,
  # auto-close
  assert set(consts) == {'MAX_STATUS', 'STALE_S', 'MISMATCH_S', 'IDLE_CLOSE_MS', 'INPUT_DELAY_MS', 'RESET_POLLS',
                         'CONTROLS_STALE_S', 'GATE_STALE_S'}
  assert consts['INPUT_DELAY_MS'] >= 300 and consts['CONTROLS_STALE_S'] <= 0.5
  # a reset request is repeated for longer than two of the car side's 5 s publishes (polls are 1 s)
  assert consts['RESET_POLLS'] > 10
  assert consts['STALE_S'] > 5 and consts['IDLE_CLOSE_MS'] == 15000
  assert all(len(text) <= consts['MAX_STATUS'] for text in tt.REASON_TEXT.values())


def test_qt_key_sets_and_names_agree():
  src = _cc_source()
  lists = {k: re.findall(r'"([^"]*)"', v) for k, v in re.findall(r'const QStringList TI_TEST_(\w+) = \{([^}]*)\};', src)}
  for name in ('ACTIVE_SOURCES', 'METRICS', 'BIN_BY', 'SWITCH_WHEN', 'FILE_KEYS', 'TEST_KEYS', 'TEST_REQUIRED',
               'SETTING_KEYS', 'PROGRESS_KEYS', 'PROGRESS_REQUIRED', 'AUTO_OFF_KEYS', 'ALTERNATE_KEYS', 'ARM_KEYS'):
    assert set(lists.pop(name)) == set(getattr(tt, name)), name
  assert lists == {}
  strings = dict(re.findall(r'const char \*const TI_TEST_(\w+) = "([^"]*)";', src))
  assert strings == {'DEFINITIONS': '../car/mazda/ti_tests.json', 'PROGRESS_KEY': tt.PROGRESS_KEY,
                     'LIVE_KEY': tt.LIVE_KEY, 'RESET_KEY': tt.RESET_KEY, 'OTHER': tt.OTHER_BIN, 'ALL': tt.ALL_BIN,
                     'UNSUPPORTED': tt.UNSUPPORTED, 'CRASH_LOG': '/data/error_logs/error.txt',
                     'GATE_KEY': 'TiTestGate', 'LOCK_KEY': 'TiTestLock'}
  # the crash log the road view shows "openpilot crashed" for (onroad/alerts.cc)
  assert '"/data/error_logs/error.txt"' in (ROOT / 'selfdrive/ui/qt/onroad/alerts.cc').read_text(encoding='utf-8')
  values = (ROOT / 'selfdrive/car/mazda/values.py').read_text(encoding='utf-8')
  assert re.search(r'TORQUE_INTERCEPTOR = (\d+)', values).group(1) == '8'
  assert 'getFlags() & 8) != 0' in src


def test_definitions_path_seen_from_the_ui_resolves_to_the_file():
  # The UI runs with selfdrive/ui as its working directory (window.cc loads "../assets/fonts/...").
  assert (ROOT / 'selfdrive/ui' / '../car/mazda/ti_tests.json').resolve() == tt.DEFINITIONS_PATH.resolve()

def test_the_gate_and_lock_params_are_registered_everywhere_they_must_be():
  params = (ROOT / 'common/params.cc').read_text(encoding='utf-8')
  variables = (ROOT / 'frogpilot/common/frogpilot_variables.py').read_text(encoding='utf-8')
  for key in ('TiTestGate', 'TiTestLock'):
    assert re.search(rf'\{{"{key}", CLEAR_ON_MANAGER_START\}}', params), key      # tmpfs state: gone at a manager start
    assert f'("{key}", "", 3, "")' in variables                                     # the default table
    excluded = re.search(r'^EXCLUDED_KEYS = \{(.*?)\}', variables, re.S | re.M).group(1)
    assert f'"{key}"' in excluded                                                  # not synced or exported as a setting
  car = (ROOT / 'selfdrive/car/mazda/carcontroller.py').read_text(encoding='utf-8')
  assert 'self.params_memory.put("TiTestGate"' in car


# ---- the drive plan forces run-all mode in the car controller; the accountant must count it ---------------------------------

def test_the_plan_forced_run_all_is_credited_although_the_run_all_toggle_is_not_set():
  defs, errors = tt.load_definitions()
  assert errors == []
  sweep = next(t for t in defs if t.id == "ti_sweep")
  band = sweep.bins[0][2]
  toggles = NS(ti_dither=False, ti_sweep=False, ti_ladder=False, ti_pulses=False, ti_ident_all=False)   # the plan sets none of them
  forced = tt.TestAccountant(defs, FakeParams(), FakeParams())
  forced.update(toggles, 24 * MPH, True, (), credit=("TiSweep", band, 1, 5.0), run_all_forced=True)
  assert forced.pending_steps.get("ti_sweep", {}).get((band, 1)) == 5.0 and "ti_sweep" in forced.seen_interval
  plain = tt.TestAccountant(defs, FakeParams(), FakeParams())
  plain.update(toggles, 24 * MPH, True, (), credit=("TiSweep", band, 1, 5.0))
  assert plain.pending_steps == {}                    # with nothing on, nothing is counted (unchanged)


def test_the_car_controller_tells_the_accountant_when_run_all_is_forced():
  src = (ROOT / "selfdrive/car/mazda/carcontroller.py").read_text(encoding="utf-8")
  assert "run_all_forced=bool(self.ti_ident_all)" in src
