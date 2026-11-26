#!/usr/bin/env python3
"""Static lint for the leak shapes that matter in per-frame code (card and controlsd run with the cycle collector off).

  python tools/dev/lint_per_frame.py [files ...]        # default: the per-frame modules of tools/dev/PER_FRAME_INVENTORY.md

Flags, inside any function (not at module or class level):
  PF1  a `class` statement (a new class per call is a reference cycle: the 2026-11-25 lane-drift leak);
  PF2  a lambda or nested def stored on `self.` or into a module-level container (each call makes a new closure kept alive);
  PF3  functools.lru_cache / cache without a maxsize (an unbounded cache);
  PF4  a module-level dict / list / set written inside a function with no visible bound in that function (len(), clear(), pop,
       del, or a slice trim);
  PF5  `+=` on a `self.` attribute that is a str / list / bytes (initialised so in the class, or a str / list / bytes added).
And over the inventory (tools/dev/per_frame_inventory.json):
  PF6  a per-frame module without the `# RUNTIME: runs in <process> ...` header right under its docstring;
  PF7  a ti_*.py, lateral_* / friction_*, selfdrive/controls/lib/*.py or frogpilot/controls/[lib/]*.py module imported by controlsd,
       card, frogpilot_process or a per-frame module that
       is in neither `per_frame` nor `not_per_frame` (new per-frame code must be classified before it lands).
A line (or the line just above it) carrying `# per-frame-ok: <reason>` is waived. Exit 1 when any unwaived hit remains.
"""
import ast
import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
# The per-frame modules: tools/dev/per_frame_inventory.json (human view: tools/dev/PER_FRAME_INVENTORY.md).
INVENTORY_PATH = Path(__file__).resolve().with_name("per_frame_inventory.json")
INVENTORY = json.loads(INVENTORY_PATH.read_text(encoding="utf-8"))
PER_FRAME_MODULES = tuple(INVENTORY["per_frame"])
# What a per-frame module imports from these places must be classified in the inventory (per_frame or not_per_frame).
CLASSIFY = (re.compile(r"^selfdrive/car/mazda/ti_[^/]+\.py$"), re.compile(r"^selfdrive/controls/lib/[^/]+\.py$"),
            re.compile(r"^selfdrive/car/mazda/(lateral|friction)_[^/]+\.py$"), re.compile(r"^frogpilot/controls/(lib/)?[^/]+\.py$"))
# The processes that run our code with the collector off (frogpilot_process: 2026-11-25, the second lane_drift leak, ~1 GB/h on-road).
ENTRY_FILES = ("selfdrive/controls/controlsd.py", "selfdrive/car/card.py", "frogpilot/frogpilot_process.py")
HEADER = re.compile(r"^# RUNTIME: runs in \S")
WAIVER = re.compile(r"#\s*per-frame-ok:\s*\S")
WRITE_METHODS = {"append", "add", "update", "setdefault", "extend", "insert", "appendleft", "__setitem__"}
BOUND_METHODS = {"clear", "pop", "popitem", "popleft", "remove", "discard"}
GROW_TYPES = (ast.List, ast.ListComp, ast.JoinedStr)


def _is_str_list_bytes(node):
  if isinstance(node, GROW_TYPES):
    return True
  if isinstance(node, ast.Constant) and isinstance(node.value, (str, bytes)):
    return True
  if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in ("list", "str", "bytes", "bytearray"):
    return True
  return False


def _module_containers(tree):
  out = set()
  for n in tree.body:
    if isinstance(n, (ast.Assign, ast.AnnAssign)):
      v = n.value
      ok = isinstance(v, (ast.Dict, ast.List, ast.Set, ast.DictComp, ast.ListComp, ast.SetComp)) or (
        isinstance(v, ast.Call) and isinstance(v.func, (ast.Name, ast.Attribute)) and
        getattr(v.func, "id", getattr(v.func, "attr", "")) in ("dict", "list", "set", "defaultdict", "OrderedDict", "deque", "Counter")
        and not any(k.arg == "maxlen" for k in v.keywords))
      if ok:
        for t in (n.targets if isinstance(n, ast.Assign) else [n.target]):
          if isinstance(t, ast.Name):
            out.add(t.id)
  return out


def _base_name(node):
  while isinstance(node, (ast.Subscript, ast.Attribute)):
    node = node.value
  return node.id if isinstance(node, ast.Name) else None


def _self_attr(node):
  return isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self"


def _grow_attrs(cls):
  """self attributes a class initialises to a str / list / bytes."""
  out = set()
  for n in ast.walk(cls):
    if isinstance(n, ast.Assign) and _is_str_list_bytes(n.value):
      for t in n.targets:
        if _self_attr(t):
          out.add(t.attr)
  return out


def lint_source(src, path="<src>"):
  tree = ast.parse(src)
  lines = src.splitlines()
  containers = _module_containers(tree)
  hits = []

  def waived(ln):
    return any(WAIVER.search(lines[k]) for k in (ln - 1, ln - 2) if 0 <= k < len(lines))

  def hit(node, code, msg):
    hits.append({"path": path, "line": node.lineno, "code": code, "msg": msg, "waived": waived(node.lineno),
                 "text": lines[node.lineno - 1].strip()[:120]})

  def deco_unbounded(d):
    name = d.func if isinstance(d, ast.Call) else d
    nm = getattr(name, "attr", getattr(name, "id", ""))
    if nm == "cache":
      return True
    if nm == "lru_cache":
      if not isinstance(d, ast.Call):
        return False                       # bare @lru_cache: maxsize 128
      for k in d.keywords:
        if k.arg == "maxsize":
          return isinstance(k.value, ast.Constant) and k.value.value is None
      if d.args:
        return isinstance(d.args[0], ast.Constant) and d.args[0].value is None
      return False
    return False

  def visit_function(fn, cls_grow):
    nested_defs = set()
    bounded = set()
    writes = []
    for n in ast.walk(fn):
      if n is fn:
        continue
      if isinstance(n, ast.ClassDef):
        hit(n, "PF1", f"class {n.name} defined inside {fn.name}()")
      if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)) and not isinstance(n, ast.Lambda):
        nested_defs.add(n.name)
      if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
        for d in n.decorator_list:
          if deco_unbounded(d):
            hit(d, "PF3", f"unbounded cache on nested {n.name}()")
      if isinstance(n, ast.Assign):
        is_fn = isinstance(n.value, ast.Lambda) or (isinstance(n.value, ast.Name) and n.value.id in nested_defs)
        for t in n.targets:
          b = _base_name(t)
          if is_fn and (_self_attr(t) or (isinstance(t, ast.Subscript) and b in containers)):
            hit(n, "PF2", f"closure stored on {ast.unparse(t)} in {fn.name}()")
          if isinstance(t, ast.Subscript) and b in containers and _base_name(t) == b and isinstance(t.value, ast.Name):
            writes.append((n, b))
      if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
        b = n.func.value.id if isinstance(n.func.value, ast.Name) else None
        if b in containers and n.func.attr in WRITE_METHODS:
          writes.append((n, b))
          if any(isinstance(a, ast.Lambda) or (isinstance(a, ast.Name) and a.id in nested_defs) for a in n.args):
            hit(n, "PF2", f"closure stored into module-level {b} in {fn.name}()")
        if b in containers and n.func.attr in BOUND_METHODS:
          bounded.add(b)
        if _self_attr(n.func.value) and n.func.attr in WRITE_METHODS and \
           any(isinstance(a, ast.Lambda) or (isinstance(a, ast.Name) and a.id in nested_defs) for a in n.args):
          hit(n, "PF2", f"closure stored on {ast.unparse(n.func.value)} in {fn.name}()")
      if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "len" and n.args:
        b = _base_name(n.args[0])
        if b in containers:
          bounded.add(b)
      if isinstance(n, ast.Delete):
        for t in n.targets:
          b = _base_name(t)
          if b in containers:
            bounded.add(b)
      if isinstance(n, ast.AugAssign) and _self_attr(n.target) and isinstance(n.op, ast.Add):
        if n.target.attr in cls_grow or _is_str_list_bytes(n.value):
          hit(n, "PF5", f"+= on {ast.unparse(n.target)} (str / list / bytes) in {fn.name}()")
    for n, b in writes:
      if b not in bounded:
        hit(n, "PF4", f"module-level {b} written in {fn.name}() with no visible bound")

  def walk(body, cls_grow):
    for n in body:
      if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
        visit_function(n, cls_grow)
      elif isinstance(n, ast.ClassDef):
        walk(n.body, _grow_attrs(n))
      elif isinstance(n, (ast.If, ast.Try, ast.With)):
        walk(getattr(n, "body", []) + getattr(n, "orelse", []) + getattr(n, "finalbody", []) +
             [h for x in getattr(n, "handlers", []) for h in x.body], cls_grow)
  walk(tree.body, set())
  # PF3 on top-level / method decorators too
  for n in ast.walk(tree):
    if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
      for d in n.decorator_list:
        if deco_unbounded(d) and not any(h["line"] == d.lineno and h["code"] == "PF3" for h in hits):
          hit(d, "PF3", f"unbounded cache on {n.name}()")
  seen, out = set(), []
  for h in hits:
    k = (h["line"], h["code"])
    if k not in seen:
      seen.add(k)
      out.append(h)
  return sorted(out, key=lambda h: h["line"])


def lint_files(paths):
  out = []
  for p in paths:
    p = Path(p)
    full = p if p.is_absolute() else REPO / p
    rel = str(full.relative_to(REPO)).replace("\\", "/") if str(full).startswith(str(REPO)) else str(p)
    out += lint_source(full.read_text(encoding="utf-8"), rel)
  return out


def header_ok(src):
  """The RUNTIME header within the 3 lines after the module docstring (or the first 5 lines when there is none)."""
  tree = ast.parse(src)
  lines = src.splitlines()
  doc = tree.body[0] if tree.body and isinstance(tree.body[0], ast.Expr) and isinstance(getattr(tree.body[0], "value", None),
                                                                                         ast.Constant) else None
  start = doc.end_lineno if doc is not None else 0
  return any(HEADER.match(ln) for ln in lines[start:start + 4])


def check_headers(root=REPO):
  return [{"path": m, "line": 1, "code": "PF6", "msg": "missing '# RUNTIME: runs in <process> at <rate>; gc ...' header under the "
           "docstring", "waived": False, "text": ""} for m in PER_FRAME_MODULES
          if m not in INVENTORY.get("header_exempt", {}) and (root / m).exists() and
          not header_ok((root / m).read_text(encoding="utf-8"))]


def _imported_files(path, root=REPO):
  tree = ast.parse((root / path).read_text(encoding="utf-8"))
  out = set()
  for n in ast.walk(tree):
    mods = []
    if isinstance(n, ast.ImportFrom) and n.module:
      m = n.module.removeprefix("openpilot.")
      mods.append(m)
      mods += [f"{m}.{a.name}" for a in n.names]
    elif isinstance(n, ast.Import):
      mods += [a.name.removeprefix("openpilot.") for a in n.names]
    for m in mods:
      f = m.replace(".", "/") + ".py"
      if (root / f).exists():
        out.add(f)
  return out


def check_classified(root=REPO):
  known = set(INVENTORY["per_frame"]) | set(INVENTORY["not_per_frame"])
  hits = []
  for src in ENTRY_FILES + PER_FRAME_MODULES:
    if not (root / src).exists():
      continue
    for f in sorted(_imported_files(src, root)):
      if any(c.match(f) for c in CLASSIFY) and f not in known:
        hits.append({"path": f, "line": 1, "code": "PF7", "msg": f"imported by {src} but not classified in "
                     "tools/dev/per_frame_inventory.json (per_frame with its process, or not_per_frame with the reason)",
                     "waived": False, "text": ""})
  return hits


def main(argv=None):
  argv = sys.argv[1:] if argv is None else argv
  hits = lint_files(argv or PER_FRAME_MODULES)
  if not argv:
    hits += check_headers() + check_classified()
  for h in hits:
    print(f"{h['path']}:{h['line']}: {h['code']} {'(waived) ' if h['waived'] else ''}{h['msg']}  | {h['text']}")
  bad = [h for h in hits if not h["waived"]]
  print(f"lint_per_frame: {len(hits)} hit(s), {len(bad)} unwaived")
  return 1 if bad else 0


if __name__ == "__main__":
  sys.exit(main())
