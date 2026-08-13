#!/usr/bin/env python3
"""Verify the params key registry agrees across Python, C++ source, and the compiled extension.

Run this after `git pull` and BEFORE rebooting.

manager_init walks every key in frogpilot_default_params and calls params.get(k), which validates
against a key table compiled into params_pyx.so. A key that exists in the Python list but not in
the compiled table raises UnknownKeyName during manager startup, and the device does not boot --
it lands on the error screen with a traceback and needs a shell or a Restore to recover.

Nothing warns you before that happens. The build reports success, because from scons' point of
view nothing was wrong: the mismatch only exists between a Python list and a native artifact.

This checks three layers and distinguishes the two failure modes, which have different fixes:

  Python list  vs  common/params.cc      -- an authoring mistake; add the key and rebuild
  params.cc    vs  common/params_pyx.so  -- a build/link problem; the source is right but the
                                            artifact the device actually loads is behind it

Exits non-zero on any mismatch, so it chains:

    cd /data/openpilot && git pull && ./frogpilot/tools/check_params_sync.py && sudo reboot
"""

import os
import re
import sys

BASEDIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
VARIABLES = os.path.join(BASEDIR, "frogpilot", "common", "frogpilot_variables.py")
PARAMS_CC = os.path.join(BASEDIR, "common", "params.cc")
PARAMS_SO = os.path.join(BASEDIR, "common", "params_pyx.so")


def python_keys():
  """Keys from the frogpilot_default_params list: ("KeyName", "default", level, "stock").

  Scoped to that list specifically. The file holds other lists of the same shape -- notably
  misc_tuning_levels, which only feeds the tuning-level map and is never handed to params.get() --
  so matching every tuple in the file reports keys that are perfectly fine as unbootable."""
  with open(VARIABLES, encoding="utf8") as f:
    src = f.read()
  start = re.search(r'^frogpilot_default_params\b.*?=\s*\[', src, re.M | re.S)
  if start is None:
    raise SystemExit("could not locate frogpilot_default_params in frogpilot_variables.py")
  body = src[start.end():]
  end = re.search(r'^\]', body, re.M)
  if end is not None:
    body = body[:end.start()]
  return set(re.findall(r'^\s*\("(\w+)",\s*', body, re.M))


def source_keys():
  """Keys from the C++ registry: {"KeyName", FLAGS},"""
  with open(PARAMS_CC, encoding="utf8") as f:
    src = f.read()
  return set(re.findall(r'\{"(\w+)",\s*[A-Z_|\s]+\}', src))


def compiled_keys(candidates):
  """Which candidates appear as literals in the built extension. Substring search rather than a
  symbol read, because the table is a static initialiser -- good enough to catch a stale artifact,
  which is the only thing this needs to detect."""
  if not os.path.exists(PARAMS_SO):
    return None
  with open(PARAMS_SO, "rb") as f:
    blob = f.read()
  return {k for k in candidates if k.encode() in blob}


def main():
  if not os.path.exists(VARIABLES) or not os.path.exists(PARAMS_CC):
    print(f"cannot find sources under {BASEDIR}", file=sys.stderr)
    return 2

  py, cc = python_keys(), source_keys()
  problems = []

  missing_from_source = sorted(py - cc)
  if missing_from_source:
    problems.append(
      "IN PYTHON BUT NOT IN common/params.cc -- these will make the device unbootable:\n    " +
      "\n    ".join(missing_from_source) +
      "\n  Fix: add each to the keys map in common/params.cc, then rebuild.")

  built = compiled_keys(py)
  if built is None:
    problems.append("common/params_pyx.so does not exist -- nothing has been built yet.")
  else:
    stale = sorted((py & cc) - built)
    if stale:
      problems.append(
        "IN common/params.cc BUT NOT IN THE COMPILED params_pyx.so -- the source is correct but\n"
        "  the artifact the device loads is behind it. These will make the device unbootable:\n    " +
        "\n    ".join(stale) +
        "\n  Fix: rm -f common/params_pyx.so && scons -j4 common/params_pyx.so\n"
        "  If that relinks without recompiling and the key is still absent, the object is stale\n"
        "  too: rm -f common/libcommon.a common/params.o and build again.")

  # Not a failure: a registered key nothing defaults is unused, not dangerous. Worth surfacing
  # because it usually means a half-removed feature.
  unused = sorted(cc - py)

  print(f"python keys: {len(py)}   params.cc keys: {len(cc)}   "
        f"present in params_pyx.so: {len(built) if built is not None else 'n/a'}")

  if unused:
    print(f"\nnote: {len(unused)} key(s) in params.cc with no Python default (harmless): "
          f"{', '.join(unused[:8])}{' ...' if len(unused) > 8 else ''}")

  if problems:
    print("\n" + "=" * 72)
    for p in problems:
      print("\n  " + p)
    print("\n" + "=" * 72)
    print("\nDO NOT REBOOT until these are resolved -- the device will not come back up.")
    return 1

  print("\nall params keys agree across Python, C++ source and the compiled extension.")
  print("safe to reboot.")
  return 0


if __name__ == "__main__":
  sys.exit(main())
