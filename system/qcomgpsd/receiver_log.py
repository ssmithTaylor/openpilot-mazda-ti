"""Read-only receiver logging for qcomgpsd, built so that it cannot change or stop the fix path.

qcomgpsd is the car's only source of satellite position. This module only READS what qcomgpsd already decoded or received
and writes bounded cloudlog events; it never touches a message that is published:
  * qcomgpsd calls the hooks only after pm.send() of the stock message for that report (or on the stock "skip" path that
    publishes nothing), with values the stock code has finished using;
  * every hook body runs inside _guard: any exception (parser error, short or garbage packet, odd types, a failing logger) is
    caught, reported ONCE per part per process life (one drive) as event gnss.log_fault (part, exception type, file:line; no
    message text, no values), and that part switches itself off for the rest of the drive. Nothing propagates to qcomgpsd;
  * nothing here blocks: the hooks do bounded in-memory work; the one modem query runs in its own daemon thread with a
    subprocess timeout, started once per drive;
  * no serial port, NMEA port, socket or file is opened.

Every command this module can send to the modem (COMMANDS) is a pure query:
  AT+QGPSCFG="gnssconfig"   the configuration read form (no value given) of the Quectel GNSS config command: it returns the
                            current constellation setting and changes nothing. Sent through ModemManager (mmcli --command),
                            the same channel the stock code uses for its AT commands.

Events (no positions, no place names, no raw lines from the receiver other than the bounded, character-filtered gnssconfig
reply when it cannot be parsed):
  qcomgpsd.fix_quality  once a minute (fix_quality.FixQuality): min/median/max of ellipse axes, HDOP, VDOP, vertical sigma,
                        satellites used per constellation, horizontal reliability, ellipse confidence.
  gnss.status           every 5 s (gnss_status.GnssStatus).
  gnss.config           once per drive: ok, the parsed gnssconfig value, or the filtered reply text.
  gnss.log_fault        at most once per part per drive.
"""
# RUNTIME: runs in qcomgpsd (ordinary Python process, on-road only, not on the control core, collector on). position() at
# about 1 Hz, measurement() at about 1 Hz per constellation, both after the stock pm.send; summary every SUMMARY_S; one
# readback thread per drive (CONFIG_TIMEOUT_S). Fixed size: FixQuality <= 60 values per field, GnssStatus overwritten.
import os
import re
import subprocess
import threading
import time

from openpilot.common.swaglog import cloudlog
from openpilot.system.qcomgpsd.fix_quality import FixQuality
from openpilot.system.qcomgpsd.gnss_status import GnssStatus, SUMMARY_S
from openpilot.system.qcomgpsd.structs import (dict_unpacker, gps_measurement_report, gps_measurement_report_sv,
                                              glonass_measurement_report, glonass_measurement_report_sv,
                                              LOG_GNSS_GPS_MEASUREMENT_REPORT, LOG_GNSS_GLONASS_MEASUREMENT_REPORT)

GNSSCONFIG_QUERY = 'AT+QGPSCFG="gnssconfig"'
COMMANDS = (GNSSCONFIG_QUERY,)          # every command this module can send; each is a read-only query
CONFIG_TIMEOUT_S = 20.0                 # hard bound on the readback subprocess (mmcli's own --timeout is 10 s)
PARTS = ('position', 'measurement', 'summary', 'config')
_GNSSCONFIG_RE = re.compile(r'\+QGPSCFG:\s*"gnssconfig"\s*,\s*(\d{1,3})')
_SAFE_TEXT = re.compile(r'[^A-Za-z0-9 +:",._=\-]')
_LONG_DIGITS = re.compile(r'\d+(?:\.\d+)+|\d{4,}')   # re-review R2-M10: no coordinate-like number passes through readable


def _run(args, timeout):
  return subprocess.run(args, capture_output=True, text=True, timeout=timeout, stdin=subprocess.DEVNULL)


def parse_gnssconfig(text):
  m = _GNSSCONFIG_RE.search(text or '')
  return int(m.group(1)) if m else None


def filtered(text, n=120):
  """Reply text cut to n characters and reduced to a fixed harmless alphabet (for an unparsable reply only); every decimal
  number and every run of 4 or more digits becomes '#', so a reply that happened to hold a position is not logged readable."""
  return _LONG_DIGITS.sub('#', _SAFE_TEXT.sub('?', str(text or '')))[:n]


class ReceiverLog:
  def __init__(self, log=None, clock=time.monotonic, run=None):
    self._log = log
    self._clock = clock
    self._run = run
    self.enabled = dict.fromkeys(PARTS, True)
    self.faults = {}
    self.fix_quality = FixQuality()
    self.status = GnssStatus()
    self.last_summary = None
    self._meas = {}
    try:
      self.last_summary = clock()
      self._meas = {
        LOG_GNSS_GPS_MEASUREMENT_REPORT: ('gps', *dict_unpacker(gps_measurement_report, True),
                                          *dict_unpacker(gps_measurement_report_sv, True)),
        LOG_GNSS_GLONASS_MEASUREMENT_REPORT: ('glo', *dict_unpacker(glonass_measurement_report, True),
                                              *dict_unpacker(glonass_measurement_report_sv, True)),
      }
    except Exception as e:  # noqa: BLE001
      for p in PARTS:
        self._fault(p, e)

  # ---- plumbing
  def _event(self, name, **kw):
    (self._log or cloudlog.event)(name, **kw)

  def _fault(self, part, e):
    if not self.enabled.get(part, False):
      return
    self.enabled[part] = False
    where = None
    try:
      tb = e.__traceback__
      while tb is not None and tb.tb_next is not None:
        tb = tb.tb_next
      if tb is not None:
        where = f"{os.path.basename(tb.tb_frame.f_code.co_filename)}:{tb.tb_lineno}"
    except Exception:  # noqa: BLE001
      pass
    self.faults[part] = (type(e).__name__, where)
    try:
      self._event("gnss.log_fault", part=part, error=type(e).__name__, where=where)
    except Exception:  # noqa: BLE001  the logger itself failing must not reach qcomgpsd either
      pass

  def _guard(self, part, fn, *a):
    if not self.enabled.get(part, False):
      return
    try:
      fn(*a)
    except Exception as e:  # noqa: BLE001
      self._fault(part, e)

  # ---- hooks called from qcomgpsd's diag loop (after the stock publish); never raise
  def position(self, report, has_fix):
    self._guard('position', self._position, report, has_fix)
    self._guard('summary', self._tick)

  def measurement(self, log_type, payload):
    self._guard('measurement', self._measurement, log_type, payload)
    self._guard('summary', self._tick)

  def _position(self, report, has_fix):
    self.status.add_position(report, has_fix)
    s = self.fix_quality.add(report, has_fix)
    if s is not None:
      self._event("qcomgpsd.fix_quality", **s)

  def _measurement(self, log_type, payload):
    cons, unpack_hdr, size_hdr, unpack_sv, size_sv = self._meas[log_type]
    payload = bytes(payload)
    if len(payload) < size_hdr:
      raise ValueError("short measurement report")
    n = unpack_hdr(payload)['svCount']
    if len(payload) - size_hdr != n * size_sv:
      raise ValueError("measurement report length does not match its satellite count")
    svs = [unpack_sv(payload[size_hdr + size_sv * i:size_hdr + size_sv * (i + 1)]) for i in range(min(n, 64))]
    self.status.add_measurement(cons, svs)

  def _tick(self):
    now = self._clock()
    if now - self.last_summary >= SUMMARY_S:
      self.last_summary = now
      self._event("gnss.status", **self.status.summary())

  # ---- once per drive: configuration readback in a daemon thread (never in the diag loop)
  def start_config_readback(self):
    if not self.enabled.get('config', False):
      return None
    try:
      t = threading.Thread(target=self._guard, args=('config', self._config_readback), daemon=True, name="gnss_config_readback")
      t.start()
      return t
    except Exception as e:  # noqa: BLE001
      self._fault('config', e)
      return None

  def _config_readback(self):
    r = (self._run or _run)(["mmcli", "-m", "any", "--timeout", "10", f"--command={GNSSCONFIG_QUERY}"], CONFIG_TIMEOUT_S)
    out = f"{r.stdout or ''} {r.stderr or ''}"
    value = parse_gnssconfig(out)
    self._event("gnss.config", ok=(r.returncode == 0), value=value, text=None if value is not None else filtered(out))
    self.enabled['config'] = False   # once per drive
