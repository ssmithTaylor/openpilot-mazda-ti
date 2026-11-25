"""WINDING-ROAD closed loop: a deterministic scenario generator for bend SEQUENCES and a runner that drives the real Planner (5 Hz),
the real car side Curve (100 Hz, status relayed through the real plannerd.car_side) and the phase-aware cruise-model Car of
test_mazda_ti_curve.py (plus a grade term) over it, then measures each bend. Synthetic roads only: no positions, no places.

Families (see the brief, 2026-11-25): F1 S-bends / chicanes, F2 bend sequences, F3 tightening / compound / hairpins, F4 grades,
F5 poor GPS, F6 lead vehicle, F7 stale / dropped forecast, F8 driver actions. Each scenario is a dict (gen(family, seed)) and
run(sc) returns its metrics. Not a test file itself: test_mazda_ti_winding.py runs it (pc_test shim), and
tools/mazda_sim/winding_report.py aggregates the JSON it writes.

WHAT THIS DOES NOT MODEL: steering saturation / lane position / the car's lateral capability (a bend is "allowed" at the planner's own
table need or record bound; nothing here checks that the car can steer it); the forecast is synthetic (the road's own curvature x the
measured FC_RATIO under-read, plus a modelled S-bend under-read; no noise, no lane-change or fork errors); GPS fixes are the road's own
centre line plus the injected faults (no multipath model); the cruise is the one measured model (+ an assumed grade term); no traffic
apart from a speed-capping lead; the driver does nothing unless the family says so (in particular nobody brakes for a warning)."""
import importlib.util
import json
import math
import pathlib

import numpy as np

from openpilot.selfdrive.car.mazda import ti_curve as C, ti_curve_memory as M, ti_curve_plan as P
from openpilot.selfdrive.car.mazda import ti_curve_plannerd as D

_spec = importlib.util.spec_from_file_location("tc_wind_car", pathlib.Path(__file__).parent / "test_mazda_ti_curve.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)

DT = 0.01
PREVIEW_S = 8.0
SBEND_UNDERREAD = 0.5        # the forecast reads the opposite bend at this share ...
SBEND_UNDERREAD_S = 1.5      # ... for this long after leaving the first (measured night 2: "under-reads for 1.5 s")
GRADE_K = 0.5                # share of the grade's g*sin not compensated by the cruise while it changes speed (assumption)
SETS = (35, 45, 50, 55)
MEM_STATES = ("rec", "known", "few", "none")
_KS = np.linspace(0.0003, 0.08, 16000)


def k_for(need, sign=1):
  """signed curvature whose planner need (need_hi, R14, its side's capacity) is `need` (the gentlest such)."""
  ks = sign * _KS
  n = P.D.need_hi(ks)
  ok = np.where(n <= need)[0]
  return float(ks[ok[0]]) if len(ok) else float(ks[-1])


class GradeCar(T.Car):
  """The measured cruise model, plus a grade term on SLOWING only: while it sheds speed, GRADE_K of g*sin(grade) is not compensated
  (downhill it sheds slower, uphill faster), but it always keeps at least a quarter of its level-road deceleration (the cruise brakes
  eventually). Speeding up is the level-road model (the throttle covers the grade). grade: fraction, + uphill."""
  grade = 0.0

  def tick(self):
    b = self.mph
    super().tick()
    if self.mph < b - 1e-9 and self.grade:
      d = b - self.mph
      d2 = max(0.25 * d, d - 9.81 * -self.grade * GRADE_K / P.MPH * DT)
      self.mph = b - d2


# ------------------------------------------------------------------------------------------------------------------- generator
def _bend(rng, allowed, sign, length=None):
  return {"segs": [(float(length if length is not None else rng.integers(60, 181)), k_for(allowed, sign))], "sign": sign}


def gen(family, seed):
  """-> scenario dict: set, mem, bends [{segs [(L, k)], sign}], gaps [m], plus family options."""
  rng = np.random.default_rng([int(family[1:]), seed])
  st = int(rng.choice(SETS))
  mem = str(rng.choice(MEM_STATES))
  sc = {"family": family, "seed": seed, "set": st, "mem": mem, "rec_x": int(rng.integers(3, 13)), "npass": 12, "opts": {}}
  if mem == "few":
    sc["npass"] = int(rng.integers(1, 4))
  sev = lambda: int(rng.integers(25, 51))
  sign = int(rng.choice([-1, 1]))
  bends, gaps = [], []
  if family == "F1":
    for _ in range(3):
      bends.append(_bend(rng, sev(), sign))
      gaps.append(float(rng.integers(0, 151)))
      bends.append(_bend(rng, sev(), -sign))
      gaps.append(float(rng.integers(250, 400)))
      sign = int(rng.choice([-1, 1]))
  elif family in ("F2", "F4", "F5", "F6", "F7", "F8"):
    n = int(rng.integers(5, 13)) if family == "F2" else int(rng.integers(5, 9))
    for _ in range(n):
      bends.append(_bend(rng, sev(), sign))
      gaps.append(float(rng.integers(50, 301)))
      sign = -sign if rng.random() < 0.7 else sign
  elif family == "F3":
    for kind in rng.permutation(["tighten", "compound", "hairpin", "plain"]):
      a = sev()
      if kind == "tighten":       # radius decreasing through the bend: 3 pieces
        segs = [(80.0, k_for(min(a + 12, 55), sign)), (60.0, k_for(min(a + 6, 55), sign)), (60.0, k_for(a, sign))]
      elif kind == "compound":    # a long gentle part, then the tight part (the early-dwell case)
        segs = [(float(rng.integers(150, 301)), k_for(min(a + int(rng.integers(8, 16)), 55), sign)), (float(rng.integers(50, 101)),
                k_for(a, sign))]
      elif kind == "hairpin":
        segs = [(40.0, k_for(30, sign)), (40.0, k_for(int(rng.integers(19, 25)), sign)), (40.0, k_for(30, sign))]
      else:
        segs = _bend(rng, a, sign)["segs"]
      bends.append({"segs": segs, "sign": sign, "kind": str(kind)})
      gaps.append(float(rng.integers(50, 401)))
      sign = -sign
  sc["bends"], sc["gaps"] = bends, gaps
  o = sc["opts"]
  if family == "F4":
    o["grade"] = float(rng.choice([-0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06, 0.08]))
    o["mem_grade"] = bool(rng.integers(0, 2))
  if family == "F5":
    if mem == "none":
      sc["mem"] = mem = "known"
    o["gps"] = str(rng.choice(["gap10", "gap30", "gap60", "offset", "jumps", "hdg_noise"]))
    o["gps_at"] = float(rng.uniform(0.15, 0.6))                         # share of the bend section where the fault starts
    o["gps_off_m"] = float(rng.uniform(5, 15)) * float(rng.choice([-1, 1]))
    o["gps_len_s"] = float(rng.choice([30, 60, 90]))
  if family == "F6":
    o["lead_under"] = float(rng.integers(5, 16))
    o["lead_leaves"] = int(rng.integers(1, len(bends)))                # leaves before this bend (mid-sequence)
  if family == "F7":
    o["stale"] = str(rng.choice(["win", "win", "drops"]))
    o["stale_s"] = float(rng.integers(1, 11))
    o["stale_n"] = int(rng.integers(2, 6))
  if family == "F8":
    o["act"] = str(rng.choice(["brake_res", "set_minus", "set_plus", "gas"]))
    o["act_bend"] = int(rng.integers(1, len(bends)))
    o["act_before_m"] = float(rng.integers(40, 151))
  return sc


def road(sc):
  """-> (segments [(L, k)], bend list [(start m, end m, signed peak k, kind)])"""
  segs, bl, s = [(900.0, 0.0)], [], 900.0
  for gi, (b, g) in enumerate(zip(sc["bends"], sc["gaps"])):
    # one measured entry per PIECE (a compound / tightening bend is judged piece by piece against each piece's own allowed speed);
    # kind carries "#<piece>/<pieces>" and the group's index
    for j, (L, k) in enumerate(b["segs"]):
      bl.append((s, s + L, k, f'{b.get("kind", "plain")}#{j + 1}/{len(b["segs"])}@{gi}', [(L, k)]))
      s += L
    segs += b["segs"]
    segs.append((g, 0.0))
    s += g
  segs.append((1500.0, 0.0))
  return segs, bl


def build_memory(sc, tr, bl):
  if sc["mem"] == "none":
    return None, {}
  n = len(tr["x"])
  rec = None
  info = {}
  if sc["mem"] == "rec":
    k = tr["kap"]
    tab = P.D.need_hi(k).astype(float)
    v_ok = np.full(n, np.nan)
    v_trb = np.full(n, np.nan)
    n_trb = np.zeros(n, np.uint8)
    for (a, b, kpk, kind, segs) in bl:
      i0, i1 = int(a // M.SPACING_M), int(b // M.SPACING_M)
      sel = slice(i0, i1)
      v_ok[sel] = (tab[sel] - sc["rec_x"]) * P.MPH
      v_trb[sel] = (tab[sel] - sc["rec_x"] + 3) * P.MPH
      n_trb[sel] = 3
    rec = {"v_trb": [v_trb.astype(np.float16)], "v_ok": [v_ok.astype(np.float16)], "n_trb": [n_trb]}
  mem = M.synthetic([tr], record=rec)
  mem.npass[:] = sc["npass"]
  g = sc["opts"].get("grade", 0.0)
  if sc["opts"].get("mem_grade"):
    mem.grade = np.full(n, math.atan(g))
  return mem, info


def allowed_for(sc, mem, tr, bl):
  """the planner's own target for each bend: its table need (need_hi) at the tightest point, or the record's bound."""
  out = []
  for (a, b, kpk, kind, segs) in bl:
    i0, i1 = int(a // M.SPACING_M), int(b // M.SPACING_M)
    k = tr["kap"][i0:i1]
    tab = P.D.need_hi(k).astype(float)
    if sc["mem"] == "rec" and mem is not None:
      rb = P.record_bound(mem.v_trb[i0:i1], mem.v_ok[i0:i1], k, tab, hi=True, n_trb=mem.n_trb[i0:i1])
      tab = np.where(np.isfinite(rb), np.minimum(rb, tab), tab)
    out.append(float(np.min(tab)))
  return out


# ------------------------------------------------------------------------------------------------------------------- runner
def run(sc, keep_rows=False):
  segs, bl = road(sc)
  tr = M.synthetic_trace(segs)
  mem, _ = build_memory(sc, tr, bl)
  allowed = allowed_for(sc, mem, tr, bl)
  o = sc["opts"]
  pl, cv = P.Planner(mem), C.Curve()
  st = sc["set"]
  car = GradeCar(float(st), st)
  car.grade = o.get("grade", 0.0)
  s_road = np.arange(len(tr["x"])) * M.SPACING_M
  kap = tr["kap"]
  nk = len(kap)
  kat = lambda q: kap[min(max(int(q // M.SPACING_M), 0), nk - 1)]
  end_m = s_road[-1] - 10
  rng = np.random.default_rng([sc["seed"], 7])
  sec_a, sec_b = bl[0][0], bl[-1][1]
  # fault windows by distance (GPS) / time (forecast)
  gps_a = sec_a + o.get("gps_at", 0) * (sec_b - sec_a)
  st_wins = []
  if o.get("stale") == "win":
    t_est = (sec_b - sec_a) / (st * P.MPH * 0.85)
    for _ in range(o["stale_n"]):
      t0 = 900 / (st * P.MPH) + float(rng.uniform(0, t_est))
      st_wins.append((t0, t0 + o["stale_s"]))
  owner = [float(st)]
  s, plan_d, plan_t, seq = 0.0, None, None, 0
  store = T_store()
  rows, changes, exc, alerts_ev = [], [], None, []
  prev_set = car.set
  last_exit = None          # (t, sign) of the last bend left
  in_bend_prev = None
  ev_state = {}
  lead_v = None
  gps_jump = (0.0, 0.0)
  f = 0
  try:
    for f in range(int(3000 / DT)):
      t = f * DT
      if s > end_m:
        break
      car.tick()
      if car.set != prev_set and not ev_state.get("driver_now"):
        changes.append((t, s, prev_set, car.set))
      ev_state["driver_now"] = False
      prev_set = car.set
      # which bend the car is in
      bi = next((i for i, x in enumerate(bl) if x[0] <= s <= x[1]), None)
      if in_bend_prev is not None and bi is None:
        last_exit = (t, bl[in_bend_prev][2] > 0)
      in_bend_prev = bi
      ov = {}
      # ---- F6 lead
      if "lead_under" in o:
        leave_at = bl[o["lead_leaves"]][0] - 150.0
        if s < leave_at and s > 300:
          lead_v = owner[0] - o["lead_under"]
          car.mph = min(car.mph, lead_v)
          ov["lead"] = True
      # ---- F8 driver actions
      if "act" in o:
        bi_t = o["act_bend"]
        at = bl[bi_t][0] - o["act_before_m"]
        act = o["act"]
        if act == "brake_res":
          a0 = bl[bi_t - 1][1] + 10
          if a0 <= s < at and "res" not in ev_state:
            car.mph = max(owner[0] - 15, car.mph - 3.0 * DT, 20.0)
            ov.update(brake=True, cruise_on=False)
          elif s >= at and "res" not in ev_state:
            ev_state["res"] = f
          if "res" in ev_state and ev_state["res"] <= f < ev_state["res"] + 10:
            ov["driver_button"] = True
        elif act in ("set_minus", "set_plus"):
          if s >= at and "p" not in ev_state:
            ev_state["p"] = f
            car.set += -1 if act == "set_minus" else 1
            owner[0] = float(car.set)
            ev_state["driver_now"] = True
            prev_set = car.set
          if "p" in ev_state and ev_state["p"] <= f < ev_state["p"] + 10:
            ov.update(driver_button=True, driver_set=True, driver_set_plus=act == "set_plus")
        elif act == "gas":
          if bl[bi_t][0] - 30 <= s <= bl[bi_t][1]:
            ev_state.setdefault("gas_v", car.mph)
            car.mph = max(car.mph, ev_state["gas_v"])
            ov["gas"] = True
      if f % 20 == 0:
        store.raw = json.dumps(cv.status(t)).encode()
        ref, floor, acting = D.car_side(store, t, C)
        x, y = float(np.interp(s, s_road, tr["x"])), float(np.interp(s, s_road, tr["y"]))
        i = min(int(s // M.SPACING_M), nk - 1)
        h = float(tr["hdg"][i])
        valid = True
        g = o.get("gps")
        if g:
          dur = o["gps_len_s"] * st * P.MPH
          on = gps_a <= s <= gps_a + dur
          if g.startswith("gap"):
            on = gps_a <= s <= gps_a + int(g[3:]) * st * P.MPH
            valid = not on
          elif on and g == "offset":
            x += -math.sin(h) * o["gps_off_m"]
            y += math.cos(h) * o["gps_off_m"]
          elif on and g == "jumps":
            if rng.random() < 0.15:
              gps_jump = (float(rng.uniform(-30, 30)), float(rng.uniform(-15, 15)))
            x += math.cos(h) * gps_jump[0] - math.sin(h) * gps_jump[1]
            y += math.sin(h) * gps_jump[0] + math.cos(h) * gps_jump[1]
          elif on and g == "hdg_noise":
            h = float(M.wrap(h + rng.normal(0, math.radians(12))))
            x += float(rng.normal(0, 3))
            y += float(rng.normal(0, 3))
        ahead = np.arange(0, PREVIEW_S + 0.01, 0.25)
        qs = s + ahead * car.mph * P.MPH
        kk = np.array([kat(q) for q in qs])
        if last_exit is not None and t - last_exit[0] <= SBEND_UNDERREAD_S:
          opp = (kk < 0) if last_exit[1] else (kk > 0)
          kk = np.where(opp, kk * SBEND_UNDERREAD, kk)
        kk = P.MODEL_K_SIGN * kk * np.interp(ahead, P.D.FC_LEAD_S, P.D.FC_RATIO)
        mk = (ahead, kk)
        if o.get("stale") == "drops" and sec_a - 400 <= s <= sec_b:
          mk = None
        if any(a <= t <= b for a, b in st_wins):
          mk = None
        p = pl.step(t, P.Fix(valid, x, y, h), car.mph * P.MPH, set_mph=car.set if ov.get("cruise_on", True) else None,
                    yaw_k=float(kat(s)), model_k=mk, ref_mph=ref, floor_mph=floor, acting=acting)
        plan_d, plan_t = P.plan_json(p, t, seq, "act"), t
        seq += 1
      la = float(kat(s)) * (car.mph * P.MPH) ** 2
      kw = dict(on=True, engaged=True, cruise_on=True, cruise_available=True, mph=car.mph, set_mph=car.set, lead=False,
                driver_button=False, counter=car.counter, plan=plan_d, plan_age=None if plan_t is None else t - plan_t, mono=t,
                la_now=la, desired_la=la)
      kw.update(ov)
      b = cv.step(f, C.Inputs(**kw))
      if b:
        car.press(b)
      s += car.mph * P.MPH * DT
      if f % 10 == 0:
        stt = cv.status(t)
        rows.append((round(t, 2), round(s, 1), car.set, round(car.mph, 2), stt["alert"], cv.state, owner[0],
                     bool(ov.get("brake") or ov.get("gas") or not ov.get("cruise_on", True)), bool(ov.get("lead")),
                     (plan_d or {}).get("ceil"), cv.floor))
  except Exception as e:  # noqa: BLE001  a planner / car-side exception is a finding
    import traceback
    exc = traceback.format_exc()[-1500:]
  res = measure(sc, bl, allowed, rows, changes, cv, pl, exc, s >= end_m)
  if keep_rows:
    res["rows"] = rows
    res["changes"] = changes
    res["bl"] = [(a, b, k, kind) for a, b, k, kind, _ in bl]
    res["cv_events"] = cv.events[-400:]
    res["pl_events"] = pl.events[-400:]
  return res


class T_store:
  raw = None

  def get(self, k):
    return self.raw


# ------------------------------------------------------------------------------------------------------------------- metrics
def needed_profile(s_arr, owner_arr, bl, allowed, floors_dummy=None):
  """the needed speed at each row: the driver's set speed, under each bend's allowed speed inside it, and before it the allowed
  speed + 0.05 mph per metre (about 1.1 mph/s at 50 mph: an easy cruise slowdown), after it rising 0.035 mph/m from end + 30 m."""
  need = owner_arr.astype(float).copy()
  for (a, b, kpk, kind, segs), al in zip(bl, allowed):
    pre = np.where(s_arr < a, al + (a - s_arr) * 0.05, np.inf)
    inb = np.where((s_arr >= a) & (s_arr <= b + P.HOLD_AFTER_M), al, np.inf)
    post = np.where(s_arr > b + P.HOLD_AFTER_M, al + (s_arr - b - P.HOLD_AFTER_M) * 0.035, np.inf)
    need = np.minimum(need, np.minimum(pre, np.minimum(inb, post)))
  return need


def measure(sc, bl, allowed, rows, changes, cv, pl, exc, finished):
  r = {"family": sc["family"], "seed": sc["seed"], "set": sc["set"], "mem": sc["mem"], "opts": sc["opts"], "exc": exc,
       "finished": bool(finished), "n_bends": len(bl), "allowed": allowed}
  if not rows:
    return r
  A = np.array([(x[0], x[1], x[2], x[3], x[6]) for x in rows], float)
  t, s, setv, mph, owner = A.T
  alert = np.array([x[4] or 0 for x in rows], float)
  state = [x[5] for x in rows]
  drv = np.array([x[7] for x in rows], bool)
  lead = np.array([x[8] for x in rows], bool)
  floor_now = np.array([x[10] if x[10] is not None else C.ABS_FLOOR_MPH for x in rows], float)
  need = needed_profile(s, owner, bl, allowed)
  bends = []
  for i, ((a, b, kpk, kind, segs), al) in enumerate(zip(bl, allowed)):
    inb = (s >= a) & (s <= b)
    if not inb.any():
      continue
    entry = float(mph[inb][0])
    peak = float(mph[inb].max())
    t_in = float(t[inb][0])
    pre = (t >= t_in - 20) & (t < t_in)
    pre_drv = bool(drv[pre].any() or drv[inb].any())
    al_lead = bool(lead[pre].any())
    # warned in time and through: an alert >= ALERT_LEAD_S before entry that covers the bend while over
    warn_t = t[(alert > 0) & (t >= t_in - 30) & (s <= b)]
    lead_s = float(t_in - warn_t[0]) if len(warn_t) else None
    over_rows = inb & (mph > al + 1.0)
    unwarned_over = int((over_rows & (alert <= 0)).sum())
    min_set_pre = float(setv[(t >= t_in - 25) & (s <= b)].min())
    cruise_at = float(owner[(t >= t_in - 25) & (s <= b)].max())
    needed = al + 1.0 < min(cruise_at, float(mph[pre].max()) if pre.any() else cruise_at)
    fl = float(floor_now[pre].max()) if pre.any() else 25.0
    # SET+ inside the bend above what the bend allows (the push's own raise up to max(allowed, floor) is fine)
    ups = [(ct, cs, o_, n_) for ct, cs, o_, n_ in changes if a <= cs <= b and n_ > o_]
    grp = kind.split("@")[-1]
    rest = [al2 for (a2, b2, _k, kd2, _s), al2 in zip(bl, allowed) if kd2.split("@")[-1] == grp and a2 >= a]
    bad_up = [u for u in ups if u[3] > max(min(rest), fl) + 0.01]
    bends.append({"i": i, "kind": kind, "allowed": al, "entry": round(entry, 1), "peak": round(peak, 1), "over": round(peak - al, 1),
                  "unwarned_over_rows": unwarned_over, "warn_lead_s": None if lead_s is None else round(lead_s, 1),
                  "needed": bool(needed), "min_set": min_set_pre, "floor": fl, "drv": pre_drv, "lead": al_lead,
                  "set_up_in_bend": len(ups), "bad_up": len(bad_up), "bad_up_to": [u[3] for u in bad_up][:3]})
  r["bends"] = bends
  # gap below the driver's set speed (actual), outside driver pedal / lead
  free = ~drv & ~lead
  gap = np.where(free, owner - mph, -99)
  r["max_gap"] = round(float(gap.max()), 1)
  r["gap15_s"] = round(float(((gap > 15.5)).sum() * 0.1), 1)
  r["gap15_unjustified_s"] = round(float(((gap > 15.5) & (need > owner - 15)).sum() * 0.1), 1)
  # early dwell: below the needed speed by more than 3 mph (outside driver / lead)
  dwell = free & (mph < need - 3.0)
  r["dwell_s"] = round(float(dwell.sum() * 0.1), 1)
  r["dwell_max"] = round(float(np.max(np.where(free, need - mph, 0))), 1)
  # unneeded slowdown: the dash set speed under the needed speed - 3 (it does not count lowering while the car is already there)
  r["unneeded_set_max"] = round(float(np.max(np.where(free, need - setv, 0))), 1)
  # taps (registered set-speed changes) / yo-yo
  dur_min = max(1e-6, (t[-1] - t[0]) / 60.0)
  ups = [c for c in changes if c[3] > c[2]]
  downs = [c for c in changes if c[3] < c[2]]
  r["taps"] = len(changes)
  r["taps_per_min"] = round(len(changes) / dur_min, 1)
  yo = 0
  for u in ups:
    if any(0 < d[0] - u[0] <= 5.0 for d in downs):
      yo += 1
  r["yoyo"] = yo
  # restore undone: an up-run (>= 2 ups) followed within 10 s by a down-run (>= 2 downs) and the speed never reached the restored set
  r["restore_undone"] = sum(1 for u in ups if sum(1 for d in downs if 0 < d[0] - u[0] <= 8.0) >= 2)
  # alerts: stretches and whether a bend after them needed it
  st_, on = [], None
  for k in range(len(t)):
    if alert[k] > 0 and on is None:
      on = k
    if alert[k] <= 0 and on is not None:
      st_.append((on, k - 1))
      on = None
  if on is not None:
    st_.append((on, len(t) - 1))
  al_out = []
  for a0, a1 in st_:
    just = False
    for bd, (a, b, *_rest) in zip(bends, bl):
      tin = next((t[k] for k in range(len(t)) if s[k] >= a), None)
      if tin is None:
        continue
      if t[a0] - 2 <= tin + (b - a) / max(1, mph[a0] * P.MPH) and tin <= t[a1] + 20 and bd["peak"] > bd["allowed"] + P.ALERT_MARGIN_MPH:
        just = True
    al_out.append({"t": round(float(t[a0]), 1), "s": round(float(s[a0])), "len_s": round(float(t[a1] - t[a0]) + 0.1, 1),
                   "need": float(alert[a0]), "justified": just})
  r["alerts"] = al_out
  r["states"] = sorted(set(state))
  r["stopped"] = [e for e in cv.events if e.get("kind") in ("stop", "planner_dead", "exit_armed", "needs_more")][:6]
  # stuck: at the end of the exit straight, set speed back at the driver's and the state idle
  r["end_set"] = float(setv[-1])
  r["end_owner"] = float(owner[-1])
  r["end_state"] = state[-1]
  r["stuck"] = bool(finished and (abs(setv[-1] - owner[-1]) > 0.5 or state[-1] not in ("ready", "off")))
  r["pl_events"] = {}
  for e in pl.events:
    r["pl_events"][e.get("kind", "")] = r["pl_events"].get(e.get("kind", ""), 0) + 1
  return r
