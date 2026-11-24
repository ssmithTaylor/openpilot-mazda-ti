"""Curve slowing's page tells the truth (the owner's standard 2: every indication comes from what the car controller is doing, never
from the switch; tested as a property over all states; after a controls restart it is off and shows off).

ti_curve.Curve is driven through random scenarios against the simulated stock cruise of test_mazda_ti_curve.py; every status it
publishes (Curve.status(), what the car controller writes to TiCurveStatus) is given to the page's own logic (ti_pages_logic.h
curveLook, compiled from selfdrive/ui/tests/curve_page_cli.cc) and the page must show exactly the controller's line1 and line2 (or
their cut), offer TURN OFF whenever the state is not "off", and show OFF after a restart (a new Curve). Synthetic only."""
import glob
import os
import random
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

pytestmark = pytest.mark.xdist_group("curve_screen")  # compiles C++ per call: never in parallel with itself

from openpilot.selfdrive.car.mazda import ti_curve as C
from openpilot.selfdrive.car.tests.test_mazda_ti_curve import Car, plan

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "selfdrive/ui/qt/ti_pages_logic.h"
CLI = ROOT / "selfdrive/ui/tests/curve_page_cli.cc"


def _const(name):
  m = re.search(rf"constexpr int {name} = (\d+);", HEADER.read_text(encoding="utf-8"))
  assert m, name
  return int(m.group(1))


MAX_PRIMARY = _const("MAX_CURVE_PRIMARY_CHARS")
MAX_SECOND = _const("MAX_CURVE_SECOND_CHARS")


def _compiler():
  for c in ("g++", "clang++", "c++"):
    p = shutil.which(c)
    if p:
      return [p, "-std=c++17", "-Wall", "-Wshadow", "-I", str(ROOT)], None
  for vc in glob.glob(r"C:\Program Files*\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvars64.bat"):
    return None, vc
  return None, None


def page(rows):
  """rows: (have, fresh, status dict or None) -> the page's (word, primary1, primary2, second, button, write, warn) for each."""
  gcc, vcvars = _compiler()
  if gcc is None and vcvars is None:
    pytest.skip("no C++ compiler here (run on a build machine)")
  lines = []
  for have, fresh, s in rows:
    s = s or {}
    f = ["1" if have else "0", "1" if fresh else "0", s.get("state", ""), "1" if s.get("needs_more") else "0", s.get("line1", ""),
         s.get("line2", "")]
    assert all("\t" not in x and "\n" not in x for x in f)
    lines.append("\t".join(x if x != "" else "-" for x in f))
  with tempfile.TemporaryDirectory() as d:
    exe = os.path.join(d, "c.exe" if os.name == "nt" else "c")
    inp = os.path.join(d, "in.txt")
    Path(inp).write_text("\n".join(lines) + "\n", encoding="utf-8")
    if gcc is not None:
      subprocess.run(gcc + [str(CLI), "-o", exe], check=True, capture_output=True, text=True)
      run_ = subprocess.run([exe, inp], capture_output=True, text=True)
    else:
      cmd = (f'"{vcvars}" >nul && cl /nologo /std:c++17 /EHsc /W3 /I"{ROOT}" "{CLI}" /Fe:"{exe}" /Fo:"{os.path.join(d, "c.obj")}" >nul '
             f'&& "{exe}" "{inp}"')
      run_ = subprocess.run('cmd /s /c "' + cmd + '"', shell=True, capture_output=True, text=True)
  out = [r.split("\t") for r in run_.stdout.replace("\r", "").split("\n") if r.strip()]
  assert len(out) == len(rows), run_.stdout[-2000:] + run_.stderr[-2000:]
  assert all(len(r) == 7 for r in out)
  return out


def _shows(line, p1, p2, cap_lines):
  """The page's lines are the controller's line: whole, or (only when it is longer than the space) its start ending in '...'."""
  shown = p1 if not p2 else p1 + " " + p2
  if len(p1) <= cap_lines[0] and len(p2) <= cap_lines[1] and shown == line:
    return True
  if p2 and p1 + p2 == line:                 # a word longer than a line, cut hard
    return True
  last = p2 or p1
  if last.endswith("..."):
    start = (p1 + " " + p2[:-3]) if p2 else p1[:-3]
    return line.startswith(start) or line.replace(" ", "").startswith(start.replace(" ", ""))
  return False


def scenario(seed, frames=5000):
  """One drive: the switch going on and off, corner ceilings (some needing more than may be shed), restores, driver presses,
  unexpected jumps, brake, gas, planner gaps, unknown road, the cruise check. Returns every published status (sampled, and at each
  change of state or lines)."""
  rng = random.Random(seed)
  start = rng.choice([25, 35, 45, 55, 62, 70, 80, 100])
  car = Car(start, start, mode=rng.choice(["accept"] * 5 + ["ignore"]))
  cv = C.Curve()
  limit = rng.choice([None, None, start, start + 5])
  sched = sorted((rng.randrange(0, frames), rng.choice([None, None, max(20, start - rng.randint(1, 40))])) for _ in range(5))
  sched = [(0, None)] + sched
  events = {rng.randrange(100, frames): rng.choice(["button", "jump", "gas", "brake", "gap", "off", "on", "unknown", "check", "lead"])
            for _ in range(rng.randint(1, 5))}
  on_at = rng.choice([0, 0, 50, 400])
  st = {"on": False, "gas": -1, "brake": -1, "gap": -1, "btn": -1, "unknown": -1, "check": -1, "lead": False}
  out, last = [], None
  for f in range(frames):
    if f == on_at:
      st["on"] = True
    e = events.get(f)
    if e == "button":
      st["btn"] = f + 20
      car.set = max(25, car.set + rng.choice([-3, 2, 5]))
    elif e == "jump":
      car.set = max(25, car.set - 5)
    elif e in ("gas", "brake", "gap", "unknown", "check"):
      st[e] = f + rng.choice([50, 200, 400])
    elif e == "off":
      st["on"] = False
    elif e == "on":
      st["on"] = True
    elif e == "lead":
      st["lead"] = not st["lead"]
    car.tick()
    ceil = None
    for t0, c in sched:
      if f >= t0:
        ceil = c
    if f < st["unknown"]:
      p = plan(None, trusted=False, known=False)
    else:
      p = plan(ceil, limit=limit, model_k=rng.choice([0.0005, 0.0005, 0.03]), kind=rng.choice(["corner"] * 3 + ["hold"]),
               capped=ceil is not None and rng.random() < 0.2)
    inp = C.Inputs(on=st["on"], engaged=True, cruise_on=f >= st["brake"], cruise_available=True, mph=car.mph, set_mph=car.set,
                   brake=f < st["brake"], gas=f < st["gas"], lead=st["lead"], driver_button=f < st["btn"], counter=car.counter,
                   plan=p, plan_age=5.0 if f < st["gap"] else 0.1, check_active=f < st["check"], mono=f * 0.01)
    b = cv.step(f, inp)
    if b is not None:
      car.press(b)
    s = cv.status(1000.0 + f * 0.01)
    key = (s["state"], s["line1"], s["line2"], s["needs_more"])
    if key != last or f % 97 == 0:
      out.append(s)
      last = key
  return out


def test_property_the_page_shows_what_the_controller_publishes():
  rows, statuses = [], []
  for seed in range(120):
    for s in scenario(seed):
      statuses.append(s)
      rows.append((True, True, s))
  # after a controls restart: a new Curve, before its first frame and with the switch cleared (the car controller clears it)
  restart = C.Curve()
  statuses.append(restart.status(0.0))
  rows.append((True, True, statuses[-1]))
  for f in range(5):
    restart.step(f, C.Inputs(on=False, engaged=True, cruise_on=True, cruise_available=True, mph=55, set_mph=55, counter=0, plan=plan(40),
                             plan_age=0.1, driver_button=False))
  statuses.append(restart.status(1.0))
  rows.append((True, True, statuses[-1]))
  n_live = len(rows)
  # the same reports gone stale, and none at all
  for s in statuses[::50]:
    rows.append((True, False, s))
  rows.append((False, False, None))

  seen = {s["state"] for s in statuses}
  assert {"off", "ready", "slowing", "holding", "restoring", "paused", "stopped", "waiting"} <= seen, seen
  assert any(s["needs_more"] for s in statuses)
  assert any("set it yourself" in s["line2"] for s in statuses)

  got = page(rows)
  for i, ((have, fresh, s), (word, p1, p2, second, button, write, warn)) in enumerate(zip(rows, got)):
    if i < n_live:
      assert word == s["state"], (s, word)
      assert _shows(s["line1"], p1, p2, (MAX_PRIMARY, MAX_PRIMARY)), (s["line1"], p1, p2)
      assert len(p1) <= MAX_PRIMARY and len(p2) <= MAX_PRIMARY
      assert second == s["line2"] or (len(s["line2"]) > MAX_SECOND and second.endswith("...") and s["line2"].startswith(second[:-3]))
      assert len(second) <= MAX_SECOND
      # the control from the car's state, never from the switch
      assert (button == "TURN OFF") == (s["state"] != "off"), (s, button)
      assert (button == "TURN ON") == (s["state"] == "off")
      assert write == ("1" if s["state"] == "off" else "0")
      assert warn == ("1" if (s["state"] == "stopped" or s["needs_more"]) else "0")
    else:
      assert word == "noreport" and button == "TURN OFF" and write == "0" and warn == "1"
      assert (p1 + " " + p2) == "CURVE SLOWING: no report from the car"
  # every line the controller wrote in these drives fits whole (nothing was cut)
  for (have, fresh, s), (word, p1, p2, second, *_rest) in zip(rows[:n_live], got[:n_live]):
    assert (p1 if not p2 else p1 + " " + p2) == s["line1"], s["line1"]
    assert second == s["line2"], s["line2"]


def test_restart_shows_off():
  """After a controls restart curve slowing is off and the page says so with TURN ON, whatever it was doing before."""
  rows = []
  for seed in range(20):
    scenario(seed, frames=2000)                     # whatever it was doing ...
    cv = C.Curve()                                  # ... the controls restart
    rows.append((True, True, cv.status(0.0)))
    cv.step(0, C.Inputs(on=False))
    rows.append((True, True, cv.status(0.01)))
  for (_, _, s), (word, p1, p2, second, button, write, warn) in zip(rows, page(rows)):
    assert s["state"] == "off" and word == "off" and button == "TURN ON" and write == "1" and warn == "0"
    assert "OFF" in p1


def test_the_page_never_reads_the_switch():
  """sidebar.cc writes TiCurveOn and never reads it: what is shown comes only from TiCurveStatus."""
  src = (ROOT / "selfdrive/ui/qt/sidebar.cc").read_text(encoding="utf-8")
  assert "TI_CURVE_ON_KEY" in src and "TI_CURVE_STATUS_KEY" in src
  assert not re.search(r"\.get(Bool)?\(\s*(ti_pages::)?TI_CURVE_ON_KEY", src)
  assert not re.search(r"\.get(Bool)?\(\s*\"TiCurveOn\"", src)
  header = HEADER.read_text(encoding="utf-8")
  assert 'TI_CURVE_ON_KEY = "TiCurveOn"' in header and 'TI_CURVE_STATUS_KEY = "TiCurveStatus"' in header
  assert C.PARAM_ON == "TiCurveOn" and C.PARAM_STATUS == "TiCurveStatus"


def test_longest_controller_lines_fit():
  """The longest lines ti_curve.py can write (three-digit speeds, the longest stop reason) fit the page's lines without a cut."""
  reasons = ["brake pressed", "gas held", "openpilot disengaged", "cruise unavailable", "cruise off", "set speed unreadable",
             "set speed changed to 100 mph unexpectedly", "set speed did not respond", "planner stopped", "planner output late",
             "cruise check started"]
  line1 = [f"STOPPED: {r}" for r in reasons] + ["SLOWING FOR A CORNER: 100 -> 100 mph", "HOLDING FOR A CORNER: 100 -> 100 mph",
                                               "HOLDING FOR THE NEXT CORNER: 100 mph", "RESTORING: 100 -> 100 mph",
                                               "PAUSED: you pressed a cruise button", "ON: planner not running", "ON: matching the road",
                                               "HOLDING: 100 mph",
                                               "CURVE SLOWING OFF"]
  line2 = ["CORNER NEEDS MORE THAN I MAY SLOW: be ready", "Set speed 100 mph (was 100): set it yourself", "Nothing was changed",
           "shadow: the planner only logs", "back in 30 s", "the model sees a bend ahead", "next corner: holding what it needs",
           "matching the remembered road", "road differs from memory", "the cruise check is running", "back to 100 mph after it",
           "planner stopped: restoring your set speed", "planner stopped: waiting until past the corner",
           "planner output late: holding"]
  rows = [(True, True, {"state": "ready", "line1": a, "line2": b}) for a in line1 for b in line2[:1]]
  rows += [(True, True, {"state": "ready", "line1": line1[0], "line2": b}) for b in line2]
  for (_, _, s), (word, p1, p2, second, *_r) in zip(rows, page(rows)):
    assert (p1 if not p2 else p1 + " " + p2) == s["line1"]
    assert second == s["line2"]
