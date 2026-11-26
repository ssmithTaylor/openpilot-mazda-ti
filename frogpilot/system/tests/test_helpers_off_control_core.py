"""ti_mcp_server and comma_logs_notifier keep off the control core (core 4: controlsd and card, real-time), set first thing in main()
before any thread starts (core-4 budget, 2026-11-26: both were seen on core 4 in the drive logs; the kernel widens a pinned mask when
the big cluster goes offline while parked, so only cores 0-3 are named)."""
import ast
import importlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
MODULES = (("frogpilot/system/ti_mcp/ti_mcp_server.py", "openpilot.frogpilot.system.ti_mcp.ti_mcp_server"),
           ("frogpilot/system/comma_logs_notifier.py", "openpilot.frogpilot.system.comma_logs_notifier"))


@pytest.mark.parametrize("rel, _name", MODULES)
def test_main_pins_before_anything_else(rel, _name):
  tree = ast.parse((ROOT / rel).read_text(encoding="utf-8"))
  main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
  first = main.body[0]
  assert isinstance(first, ast.Expr) and isinstance(first.value, ast.Call) and getattr(first.value.func, "id", None) == "off_control_core"


@pytest.mark.parametrize("_rel, name", MODULES)
@pytest.mark.parametrize("have, want", [({0, 1, 2, 3, 4, 5, 6, 7}, [0, 1, 2, 3]), ({4, 5}, None), ({2, 4}, [2])])
def test_off_control_core_names_only_little_cores(_rel, name, have, want, monkeypatch):
  mod = importlib.import_module(name)
  got = []
  monkeypatch.setattr(mod.os, "sched_getaffinity", lambda pid: set(have), raising=False)
  monkeypatch.setattr(mod.os, "sched_setaffinity", lambda pid, cores: got.append(sorted(cores)), raising=False)
  mod.off_control_core()
  assert got == ([] if want is None else [want])


@pytest.mark.parametrize("_rel, name", MODULES)
def test_off_control_core_never_raises(_rel, name, monkeypatch):
  mod = importlib.import_module(name)

  def boom(pid):
    raise OSError("no")
  monkeypatch.setattr(mod.os, "sched_getaffinity", boom, raising=False)
  mod.off_control_core()
