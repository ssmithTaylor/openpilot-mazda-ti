"""The receiver logging cannot change or stop qcomgpsd's fix path.

* Byte-for-byte: synthetic diag packets (position with and without fix, skipped source, GPS / GLONASS measurement reports,
  OEMDRE measurement and SV polynomial reports) go through the stock qcomgpsd (tests/stock_qcomgpsd_ref.py, a frozen copy of
  integ3 05305f0e) and the new one; every published gpsLocation / qcomGnss message must be identical bytes, with the logging
  live and under every injected fault.
* Structure: the new qcomgpsd.py minus the guarded rx_log statements is the stock file, statement for statement (AST).
* Faults: parser exception, short packet, garbage bytes, odd field types, a failing logger, the module failing to import or
  construct, the modem query's program missing, a read timeout, a hung query, a garbage reply. Each is logged at most once per
  part and that part switches itself off; nothing propagates.
* Commands: the only modem command the new code can send is the gnssconfig read form.
* No positions in any logged event.
"""
import ast
import importlib.util
import math
import random
import struct
import subprocess
import sys
import threading
import time
import types
from pathlib import Path

import pytest

# pyserial and crcmod are on the device; modemdiag only needs the names at import, and no test opens the diag port.
for _name, _stub in (("serial", types.SimpleNamespace(Serial=object)),
                     ("crcmod", types.SimpleNamespace(mkCrcFun=lambda *a, **k: (lambda data: 0)))):
  try:
    importlib.import_module(_name)
  except ImportError:
    sys.modules.setdefault(_name, _stub)

import cereal.messaging as messaging
from openpilot.system.qcomgpsd import qcomgpsd as new_mod
from openpilot.system.qcomgpsd import receiver_log as rl
from openpilot.system.qcomgpsd import gnss_status as gs
from openpilot.system.qcomgpsd import fix_quality as fq
from openpilot.system.qcomgpsd.modemdiag import DIAG_LOG_F
from openpilot.system.qcomgpsd.structs import (parse_struct, position_report, gps_measurement_report, gps_measurement_report_sv,
                                              glonass_measurement_report, glonass_measurement_report_sv,
                                              oemdre_measurement_report, oemdre_measurement_report_sv, oemdre_svpoly_report,
                                              name_to_camelcase,
                                              LOG_GNSS_GPS_MEASUREMENT_REPORT, LOG_GNSS_GLONASS_MEASUREMENT_REPORT,
                                              LOG_GNSS_POSITION_REPORT, LOG_GNSS_OEMDRE_MEASUREMENT_REPORT,
                                              LOG_GNSS_OEMDRE_SVPOLY_REPORT)

HERE = Path(__file__).resolve().parent
QDIR = HERE.parent
REF_PATH = HERE / "stock_qcomgpsd_ref.py"
LAT, LON, ALT = 0.71234567891, -1.41234567891, 123.456   # sentinel position values that must never appear in a log event


def _ref_module():
  spec = importlib.util.spec_from_file_location("stock_qcomgpsd_ref_mod", REF_PATH)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


REF = _ref_module()


# ---------------------------------------------------------------- synthetic diag packets
def pack(ss, camel, **vals):
  fmt, names = parse_struct(ss)
  if camel:
    names = [name_to_camelcase(n) for n in names]
  missing = set(vals) - set(names)
  assert not missing, missing
  return struct.pack(fmt, *[vals.get(n, 0) for n in names])


def packet(log_type, lp, log_time=1_234_567):
  inner = struct.pack('<HHQ', 12 + len(lp), log_type, log_time) + lp
  return DIAG_LOG_F, struct.pack('<BH', 0, len(inner)) + inner


def pos_payload(i=0, src=2, vdop=1.5, **kw):
  v = {"u_PosSource": src, "w_GpsWeekNumber": 2340, "q_GpsFixTimeMs": 300_000_000 + 1000 * i,
       "t_DblFinalPosLatLon[0]": LAT, "t_DblFinalPosLatLon[1]": LON, "q_FltFinalPosAlt": ALT, "q_FltHeadingRad": 0.5,
       "q_FltHeadingUncRad": 0.05 * (i % 2), "q_FltVelEnuMps[0]": 3.0, "q_FltVelEnuMps[1]": 20.0 + i, "q_FltVelEnuMps[2]": 0.1,
       "q_FltVelSigmaMps[0]": 0.2, "q_FltVelSigmaMps[1]": 0.3, "q_FltVelSigmaMps[2]": 0.4, "q_FltPdop": 1.6, "q_FltHdop": 0.9,
       "q_FltVdop": vdop, "q_FltEllipseSemimajorAxis": 4.0 + i % 5, "q_FltEllipseSemiminorAxis": 2.0, "u_EllipseConfidence": 39,
       "q_FltPosSigmaVertical": 6.0, "u_HorizontalReliability": 3, "u_NumGpsSvsUsed": 7, "u_TotalGpsSvs": 10,
       "u_NumGloSvsUsed": 4, "u_TotalGloSvs": 6, "u_NumBdsSvsUsed": 0, "u_TotalBdsSvs": 2, "w_PosVelFlag": 0x1234}
  v.update(kw)
  return pack(position_report, False, **v)


def meas_payload(glonass=False, n=3):
  hdr, sv = (glonass_measurement_report, glonass_measurement_report_sv) if glonass else (gps_measurement_report, gps_measurement_report_sv)
  h = pack(hdr, True, version=0, svCount=n, **({} if glonass else {"week": 2340}))
  sats = b''.join(pack(sv, True, svId=i + 1, carrierNoise=300 + 30 * i, goodObservations=i, measurementStatus=(1 << 13) * (i == 1),
                       miscStatus=1, azimuth=0.1 * i, elevation=0.2) for i in range(n))
  return h + sats


def oemdre_payload(n=2):
  h = pack(oemdre_measurement_report, True, version=2, svCount=n, systemRtcValid=1, gpsWeek=2340, source=1)
  sats = b''.join(pack(oemdre_measurement_report_sv, True, svId=i + 1, carrierNoise=400, measurementStatus=0b101,
                       multipathEstimateValid=1, goodParity=1) for i in range(n))
  return h + sats


def svpoly_payload():
  return pack(oemdre_svpoly_report, True, version=2, svId=5, t0=12.5, elevation=0.3)


def drive_packets(n_pos=62):
  pk = []
  for i in range(n_pos):
    pk.append(packet(LOG_GNSS_POSITION_REPORT, pos_payload(i, vdop=500.0 if i % 7 == 0 else 1.5)))
    if i % 3 == 0:
      pk.append(packet(LOG_GNSS_POSITION_REPORT, pos_payload(i, src=0)))       # stock skips it (publishes nothing)
    pk.append(packet(LOG_GNSS_GPS_MEASUREMENT_REPORT, meas_payload(False, 3 + i % 4)))
    pk.append(packet(LOG_GNSS_GLONASS_MEASUREMENT_REPORT, meas_payload(True, 2)))
    if i % 10 == 0:
      pk.append(packet(LOG_GNSS_OEMDRE_MEASUREMENT_REPORT, oemdre_payload()))
      pk.append(packet(LOG_GNSS_OEMDRE_SVPOLY_REPORT, svpoly_payload()))
      pk.append(packet(LOG_GNSS_GPS_MEASUREMENT_REPORT, meas_payload(False, 0)))
  return pk


# ---------------------------------------------------------------- running qcomgpsd.main() on the packets
class _Done(Exception):
  pass


class Recorder:
  def __init__(self, fail=False):
    self.events = []
    self.fail = fail

  def event(self, name, **kw):
    if self.fail:
      raise OSError("logger broken")
    self.events.append((name, kw))

  def __getattr__(self, name):   # warning / error / debug / exception / info
    return lambda *a, **k: self.events.append((name, a))


class FakeProc:
  def __init__(self, *a, **k):
    pass

  start = kill = join = lambda self: None


def run_main(mod, packets, monkeypatch, run=None, rec=None):
  sent = []

  class PM:
    def __init__(self, names):
      pass

    def send(self, name, msg):
      msg.logMonoTime = 0
      sent.append((name, msg.to_bytes()))

  class Diag:
    def __init__(self):
      self.it = iter(packets)

    def recv(self):
      try:
        return next(self.it)
      except StopIteration:
        raise _Done from None

  rec = rec or Recorder()
  monkeypatch.setattr(messaging, "PubMaster", PM)
  monkeypatch.setattr(mod, "ModemDiag", Diag)
  monkeypatch.setattr(mod, "wait_for_modem", lambda *a, **k: None)
  monkeypatch.setattr(mod, "setup_quectel", lambda diag: True)
  monkeypatch.setattr(mod, "gpio_init", lambda *a: None)
  monkeypatch.setattr(mod, "gpio_set", lambda *a: None)
  monkeypatch.setattr(mod, "Process", FakeProc)
  monkeypatch.setattr(mod, "signal", types.SimpleNamespace(signal=lambda *a: None, SIGINT=2, SIGTERM=15))
  monkeypatch.setattr(mod, "cloudlog", Recorder())
  monkeypatch.setattr(rl, "cloudlog", rec)
  monkeypatch.setattr(rl, "_run", run or (lambda args, timeout: subprocess.CompletedProcess(args, 0, 'response: \'+QGPSCFG: "gnssconfig",1\'', '')))
  t0 = time.monotonic()
  with pytest.raises(_Done):
    mod.main()
  return sent, rec, time.monotonic() - t0


@pytest.fixture(scope="module")
def reference():
  mp = pytest.MonkeyPatch()
  try:
    sent, _, _ = run_main(REF, drive_packets(), mp)
  finally:
    mp.undo()
  kinds = {n for n, _ in sent}
  assert kinds == {"gpsLocation", "qcomGnss"} and len(sent) > 200
  return sent


def _wait_config_thread():
  for t in threading.enumerate():
    if t.name == "gnss_config_readback":
      t.join(timeout=5)


def _faults(rec):
  return [kw for n, kw in rec.events if n == "gnss.log_fault"]


def _assert_once_per_part(rec):
  parts = [f["part"] for f in _faults(rec)]
  assert len(parts) == len(set(parts)), parts


# ---------------------------------------------------------------- byte-for-byte
def test_published_bytes_identical_with_logging_live(reference, monkeypatch):
  monkeypatch.setattr(rl, "SUMMARY_S", 0.0)       # a status line on every hook call: the most logging possible
  sent, rec, _ = run_main(new_mod, drive_packets(), monkeypatch)
  _wait_config_thread()
  assert sent == reference
  names = [n for n, _ in rec.events]
  assert "gnss.status" in names and "qcomgpsd.fix_quality" in names and "gnss.config" in names
  assert not _faults(rec)
  cfg = next(kw for n, kw in rec.events if n == "gnss.config")
  assert cfg == {"ok": True, "value": 1, "text": None}


def _raise(exc):
  def f(*a, **k):
    raise exc
  return f


FAULTS = {
  "position_parser": lambda mp: mp.setattr(gs.GnssStatus, "add_position", _raise(KeyError("u_PosSource"))),
  "fix_quality": lambda mp: mp.setattr(fq.FixQuality, "add", _raise(ZeroDivisionError())),
  "measurement_parser": lambda mp: mp.setattr(rl.ReceiverLog, "_measurement", _raise(struct.error("unpack requires a buffer"))),
  "summary": lambda mp: (mp.setattr(rl, "SUMMARY_S", 0.0), mp.setattr(gs.GnssStatus, "summary", _raise(TypeError()))),
  "constructor": lambda mp: mp.setattr(rl.ReceiverLog, "__init__", _raise(RuntimeError())),
  "import": lambda mp: mp.setitem(sys.modules, "openpilot.system.qcomgpsd.receiver_log", None),
  "measurement_values": lambda mp: mp.setattr(gs.GnssStatus, "add_measurement", _raise(ValueError("odd value"))),
}


@pytest.mark.parametrize("fault", sorted(FAULTS))
def test_published_bytes_identical_under_faults(reference, monkeypatch, fault):
  FAULTS[fault](monkeypatch)
  sent, rec, _ = run_main(new_mod, drive_packets(), monkeypatch)
  _wait_config_thread()
  assert sent == reference
  _assert_once_per_part(rec)


def test_logger_itself_failing_does_not_reach_qcomgpsd(reference, monkeypatch):
  monkeypatch.setattr(rl, "SUMMARY_S", 0.0)
  sent, rec, _ = run_main(new_mod, drive_packets(), monkeypatch, rec=Recorder(fail=True))
  _wait_config_thread()
  assert sent == reference


@pytest.mark.parametrize("kind", ["mmcli_missing", "timeout", "garbage", "nonzero"])
def test_config_readback_faults(reference, monkeypatch, kind):
  def run(args, timeout):
    if kind == "mmcli_missing":
      raise FileNotFoundError(2, "No such file or directory: 'mmcli'")
    if kind == "timeout":
      raise subprocess.TimeoutExpired(args, timeout)
    if kind == "garbage":
      return subprocess.CompletedProcess(args, 0, "\x00\xff\x1b[2J" + "Z" * 5000 + '$GPGGA,1,4807.038,N', "")
    return subprocess.CompletedProcess(args, 1, "", "error: couldn't find modem")
  sent, rec, _ = run_main(new_mod, drive_packets(), monkeypatch, run=run)
  _wait_config_thread()
  assert sent == reference
  _assert_once_per_part(rec)
  cfg = [kw for n, kw in rec.events if n == "gnss.config"]
  f = _faults(rec)
  if kind in ("mmcli_missing", "timeout"):
    assert not cfg and [x["part"] for x in f] == ["config"]
    assert f[0]["error"] in ("FileNotFoundError", "TimeoutExpired")
  else:
    assert len(cfg) == 1 and cfg[0]["value"] is None and len(cfg[0]["text"]) <= 120 and not f
    assert set(cfg[0]["text"]) <= set("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 +:\",._=-?")


def test_hung_config_query_does_not_block_the_fix_loop(reference, monkeypatch):
  release = threading.Event()
  started = threading.Event()

  def run(args, timeout):
    started.set()
    release.wait(30)            # a query that never answers (the real one is killed after CONFIG_TIMEOUT_S)
    raise subprocess.TimeoutExpired(args, timeout)
  try:
    sent, rec, dt = run_main(new_mod, drive_packets(), monkeypatch, run=run)
    assert started.wait(5)
    assert sent == reference and dt < 10
    assert not [n for n, _ in rec.events if n == "gnss.config"]
  finally:
    release.set()
    _wait_config_thread()


# ---------------------------------------------------------------- structure: the diff is only the guarded rx_log lines
def _strip_rx(tree):
  def mentions(node):
    return any(isinstance(n, ast.Name) and n.id == "rx_log" for n in ast.walk(node))

  def keep(s):
    if isinstance(s, ast.FunctionDef) and s.name == "_receiver_log":
      return False
    if isinstance(s, (ast.Assign, ast.Expr)) and mentions(s):
      return False
    if isinstance(s, ast.If) and mentions(s.test):
      return False
    return True

  for node in ast.walk(tree):
    for field in ("body", "orelse", "finalbody"):
      lst = getattr(node, field, None)
      if isinstance(lst, list) and lst and isinstance(lst[0], ast.stmt):
        setattr(node, field, [s for s in lst if keep(s)])
  return tree


def test_new_qcomgpsd_is_stock_plus_guarded_hooks_only():
  new = _strip_rx(ast.parse((QDIR / "qcomgpsd.py").read_text(encoding="utf-8")))
  old = ast.parse(REF_PATH.read_text(encoding="utf-8"))
  assert ast.dump(new) == ast.dump(old)
  # every rx_log use sits under an "is not None" guard and calls only these methods
  src = (QDIR / "qcomgpsd.py").read_text(encoding="utf-8")
  calls = {n.func.attr for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
           and isinstance(n.func.value, ast.Name) and n.func.value.id == "rx_log"}
  assert calls == {"position", "measurement", "start_config_readback"}


def test_hooks_come_after_the_stock_publish():
  lines = (QDIR / "qcomgpsd.py").read_text(encoding="utf-8").splitlines()
  for i, ln in enumerate(lines):
    if "rx_log.position(report, gps.hasFix)" in ln:
      assert "pm.send('gpsLocation', msg)" in "\n".join(lines[i - 4:i])
    if "rx_log.measurement(" in ln:
      assert "pm.send('qcomGnss', msg)" in lines[i - 2]


# ---------------------------------------------------------------- modem commands
NEW_FILES = ("receiver_log.py", "gnss_status.py", "fix_quality.py")


def test_only_command_is_the_gnssconfig_query(monkeypatch):
  assert rl.COMMANDS == (rl.GNSSCONFIG_QUERY,) == ('AT+QGPSCFG="gnssconfig"',)
  for f in NEW_FILES:
    src = (QDIR / f).read_text(encoding="utf-8")
    tree = ast.parse(src)
    docs = {id(n.value) for n in ast.walk(tree) if isinstance(n, ast.Expr)}     # docstrings are text, not commands
    at = [c.value for c in ast.walk(tree) if isinstance(c, ast.Constant) and isinstance(c.value, str) and "AT+" in c.value
          and id(c) not in docs]
    assert set(at) <= set(rl.COMMANDS), (f, at)
  # read form: the config name with no value after it
  assert rl.GNSSCONFIG_QUERY.endswith('"gnssconfig"') and "," not in rl.GNSSCONFIG_QUERY
  seen = []
  r = rl.ReceiverLog(log=lambda *a, **k: None, run=lambda args, timeout: seen.append((args, timeout)) or
                     subprocess.CompletedProcess(args, 0, '+QGPSCFG: "gnssconfig",5', ''))
  t = r.start_config_readback()
  t.join(5)
  assert seen == [(["mmcli", "-m", "any", "--timeout", "10", '--command=AT+QGPSCFG="gnssconfig"'], rl.CONFIG_TIMEOUT_S)]
  assert r.start_config_readback() is None          # once per drive


def test_real_runner_has_a_timeout_and_no_shell(monkeypatch):
  got = {}
  monkeypatch.setattr(rl.subprocess, "run", lambda args, **kw: got.update(args=args, **kw))
  rl._run(["mmcli"], 7.0)
  assert got["timeout"] == 7.0 and not got.get("shell") and got["stdin"] == subprocess.DEVNULL


def test_new_modules_open_no_port_file_or_socket():
  banned_mods = {"serial", "socket", "requests", "urllib", "http", "fcntl", "termios"}
  for f in NEW_FILES:
    tree = ast.parse((QDIR / f).read_text(encoding="utf-8"))
    for n in ast.walk(tree):
      if isinstance(n, ast.Import):
        assert not {a.name.split(".")[0] for a in n.names} & banned_mods, f
      if isinstance(n, ast.ImportFrom):
        assert (n.module or "").split(".")[0] not in banned_mods, f
      if isinstance(n, ast.Call) and isinstance(n.func, ast.Name):
        assert n.func.id != "open", f
      if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
        assert n.func.attr not in ("open", "Popen", "check_output", "call", "system"), (f, n.func.attr)


# ---------------------------------------------------------------- direct fault injection on the hooks
def _rx(clock=None):
  ev = []
  r = rl.ReceiverLog(log=lambda name, **kw: ev.append((name, kw)), clock=clock or (lambda: 0.0))
  return r, ev


def test_short_packet_disables_measurement_once():
  r, ev = _rx()
  good = meas_payload(False, 3)
  r.measurement(LOG_GNSS_GPS_MEASUREMENT_REPORT, good[:5])           # shorter than the header
  r.measurement(LOG_GNSS_GPS_MEASUREMENT_REPORT, good[:-3])          # satellite block cut short
  r.measurement(LOG_GNSS_GPS_MEASUREMENT_REPORT, good)
  f = _faults_list(ev)
  assert len(f) == 1 and f[0]["part"] == "measurement" and f[0]["error"] == "ValueError"
  assert f[0]["where"].startswith("receiver_log.py:")
  assert r.enabled["measurement"] is False and r.status.meas == {}
  r.position({"u_PosSource": 2, "q_FltHdop": 1.0}, True)              # other parts keep going
  assert r.status.pos["hdop"] == 1.0


def test_garbage_bytes_never_raise():
  rnd = random.Random(7)
  for _ in range(200):
    r, ev = _rx()
    for _ in range(5):
      lt = rnd.choice([LOG_GNSS_GPS_MEASUREMENT_REPORT, LOG_GNSS_GLONASS_MEASUREMENT_REPORT, 0x9999])
      r.measurement(lt, bytes(rnd.getrandbits(8) for _ in range(rnd.randrange(0, 400))))
    assert len([n for n, _ in ev if n == "gnss.log_fault"]) <= 1


def test_odd_position_values_never_raise():
  r, ev = _rx(clock=iter(range(0, 10_000, 10)).__next__)
  odd = [{}, {"u_PosSource": "x"}, {"q_FltHdop": float("nan"), "u_NumGpsSvsUsed": None}, {"q_FltHdop": "1.0"},
         {"q_FltEllipseSemimajorAxis": float("inf")}, None, 5]
  for rep in odd * 20:
    r.position(rep, True)
  assert len(_faults_list(ev)) <= 2


def _faults_list(ev):
  return [kw for n, kw in ev if n == "gnss.log_fault"]


def test_fault_text_carries_no_values():
  r, ev = _rx()
  r._guard("position", _raise(ValueError(f"lat {LAT}")))
  (f,) = _faults_list(ev)
  assert set(f) == {"part", "error", "where"} and str(LAT) not in repr(f)


def test_no_positions_in_any_event():
  t = [0.0]
  r, ev = _rx(clock=lambda: t[0])
  rep_fn = new_mod.dict_unpacker(position_report)[0]
  meas = meas_payload(False, 5)
  for i in range(130):
    t[0] += 1.0
    r.position(rep_fn(pos_payload(i)), True)
    r.measurement(LOG_GNSS_GPS_MEASUREMENT_REPORT, meas)
  names = {n for n, _ in ev}
  assert {"gnss.status", "qcomgpsd.fix_quality"} <= names and "gnss.log_fault" not in names
  text = repr(ev)
  for bad in ("Lat", "Lon", "lat", "lon", "Alt", "alt", "Heading", "Vel", "Fcount", "FixTime", "Week"):
    assert bad not in text, bad
  for v in (LAT, LON, ALT, math.degrees(LAT), math.degrees(LON)):
    assert f"{v:.4f}" not in text and f"{v:.3f}" not in text
  st = next(kw for n, kw in ev if n == "gnss.status")
  assert st["pos"]["used"] == {"gps": 7, "glo": 4, "bds": 0} and st["diag"]["gps"]["tracked"] == 5
  assert st["diag"]["gps"]["usable"] == 4 and st["diag"]["gps"]["cn0"]["top4"] == [42.0, 39.0, 36.0, 33.0]


def test_status_is_bounded():
  r, ev = _rx()
  for _ in range(1000):
    r.status.add_measurement("gps", [{"carrierNoise": 400, "goodObservations": 1}] * 500)
  assert len(r.status.meas["gps"]["cn0"]) == gs.MAX_SV and len(r.status.meas) == 1


def test_hook_cost_is_small():
  r, _ = _rx()
  meas = meas_payload(True, 14)
  rep = new_mod.dict_unpacker(position_report)[0](pos_payload(1))
  t0 = time.perf_counter()
  for _ in range(500):
    r.measurement(LOG_GNSS_GLONASS_MEASUREMENT_REPORT, meas)
    r.position(rep, True)
  per_call_ms = (time.perf_counter() - t0) / 1000 * 1e3
  assert per_call_ms < 2.0, per_call_ms


def test_filtered_reply_never_keeps_a_coordinate_readable():
  """re-review R2-M10: an unparsable reply that happened to hold a position-style line is not logged readable."""
  from openpilot.system.qcomgpsd.receiver_log import filtered
  out = filtered('+QGPSLOC: 111111.0,1234.567N,09876.543E,1.0,555.5,2,011.1,0.0,0.0,010101,08')
  assert "1234" not in out and "09876" not in out and "111111" not in out and "555.5" not in out
  assert not __import__("re").search(r"\d{4}", out), out
  assert filtered('+CME ERROR: 50') == '+CME ERROR: 50'              # short codes stay readable
  assert filtered('12.34,-56.78') == '#,-#'
