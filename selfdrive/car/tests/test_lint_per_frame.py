"""tools/dev/lint_per_frame.py: the static leak-shape lint over the per-frame modules, and each of its rules on a synthetic snippet."""
import importlib.util
from pathlib import Path

import pytest

_P = Path(__file__).resolve().parents[3] / "tools" / "dev" / "lint_per_frame.py"
_spec = importlib.util.spec_from_file_location("lint_per_frame_for_test", _P)
L = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(L)


def test_the_per_frame_modules_have_no_unwaived_hit():
  hits = L.lint_files(L.PER_FRAME_MODULES)
  bad = [f"{h['path']}:{h['line']} {h['code']} {h['msg']}" for h in hits if not h["waived"]]
  assert not bad, "per-frame leak shapes (fix, or waive with '# per-frame-ok: <the bound, and where it is>'):\n" + "\n".join(bad)


def test_every_listed_module_exists():
  root = _P.parents[2]
  assert all((root / m).exists() for m in L.PER_FRAME_MODULES)


@pytest.mark.parametrize("code,src", [
  ("PF1", "def f(m):\n  class V:\n    pass\n  return V()\n"),
  ("PF2", "class A:\n  def f(self, x):\n    self.cb = lambda: x\n"),
  ("PF2", "class A:\n  def f(self, x):\n    def g():\n      return x\n    self.cbs.append(g)\n"),
  ("PF2", "HOOKS = {}\ndef f(x):\n  HOOKS[x] = lambda: x\n  if len(HOOKS) > 9:\n    HOOKS.clear()\n"),
  ("PF3", "import functools\n@functools.lru_cache(maxsize=None)\ndef f(x):\n  return x\n"),
  ("PF3", "from functools import cache\n@cache\ndef f(x):\n  return x\n"),
  ("PF4", "SEEN = []\ndef f(x):\n  SEEN.append(x)\n"),
  ("PF4", "MEMO = {}\ndef f(x):\n  MEMO[x] = x * 2\n  return MEMO[x]\n"),
  ("PF5", "class A:\n  def __init__(self):\n    self.log = ''\n  def f(self, x):\n    self.log += str(x)\n"),
  ("PF5", "class A:\n  def f(self, x):\n    self.hist += [x]\n"),
])
def test_each_rule_flags_its_shape(code, src):
  hits = L.lint_source(src)
  assert any(h["code"] == code and not h["waived"] for h in hits), hits


@pytest.mark.parametrize("src", [
  "class V:\n  pass\ndef f(m):\n  return V()\n",                                 # class at module level
  "import functools\n@functools.lru_cache(maxsize=64)\ndef f(x):\n  return x\n",
  "SEEN = []\ndef f(x):\n  SEEN.append(x)\n  if len(SEEN) > 100:\n    del SEEN[:50]\n",
  "class A:\n  def __init__(self):\n    self.n = 0\n  def f(self, x):\n    self.n += x\n",
  "def f(xs):\n  acc = []\n  acc.append(1)\n  return acc\n",                        # local list: freed on return
])
def test_bounded_or_local_shapes_are_not_flagged(src):
  assert not L.lint_source(src)


def test_a_waiver_on_the_line_or_the_line_above_counts():
  src = "SEEN = []\ndef f(x):\n  # per-frame-ok: keyed by test, 3 entries\n  SEEN.append(x)\n  SEEN.append(x)  # per-frame-ok: same\n"
  hits = L.lint_source(src)
  assert hits and all(h["waived"] for h in hits)
  assert L.lint_source("SEEN = []\ndef f(x):\n  SEEN.append(x)  # per-frame-ok:\n")[0]["waived"] is False   # a reason is required


# ---- the inventory: every per-frame module carries the RUNTIME header, every new per-frame import is classified -----------------------
def test_every_per_frame_module_has_the_runtime_header():
  assert not L.check_headers(), [h["path"] for h in L.check_headers()]


def test_header_must_sit_under_the_docstring():
  assert L.header_ok('"""Doc."""\n# RUNTIME: runs in card at 100 Hz; gc disabled.\nimport math\n')
  assert L.header_ok("#!/usr/bin/env python3\n# RUNTIME: runs in card at 100 Hz\nimport math\n")
  assert not L.header_ok('"""Doc."""\nimport math\n\n\n\n\n# RUNTIME: runs in card\n')


def test_every_module_a_per_frame_entry_point_imports_is_classified():
  assert not L.check_classified(), [h["path"] for h in L.check_classified()]


def test_a_new_unclassified_ti_module_imported_by_a_per_frame_module_fails(tmp_path):
  for rel in L.ENTRY_FILES + L.PER_FRAME_MODULES:
    (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
    (tmp_path / rel).write_text('"""x"""\n')
  (tmp_path / "selfdrive/car/mazda/carcontroller.py").write_text("from openpilot.selfdrive.car.mazda import ti_new_feature\n")
  (tmp_path / "selfdrive/car/mazda/ti_new_feature.py").write_text("X = 1\n")
  hits = L.check_classified(tmp_path)
  assert [h["path"] for h in hits] == ["selfdrive/car/mazda/ti_new_feature.py"]


def test_the_inventory_page_lists_every_per_frame_module():
  page = (_P.parent / "PER_FRAME_INVENTORY.md").read_text(encoding="utf-8")
  missing = [m for m in L.PER_FRAME_MODULES if m.split("/")[-1] not in page]
  assert not missing, missing


def test_header_exempt_modules_stay_untouched_because_their_bytes_are_hashed():
  """ti_cruise_model.py / ti_cruise_taps.py are hashed into the approach table's key: a header there makes the table stale."""
  root = _P.parents[2]
  for m in L.INVENTORY.get("header_exempt", {}):
    assert "# RUNTIME:" not in (root / m).read_text(encoding="utf-8"), m
