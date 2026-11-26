"""What the receiver is using, for the drive log: one bounded summary every SUMMARY_S seconds, no positions.

Diag reports only (the reports qcomgpsd already requests and receives; nothing new is requested from the modem):
  position report (0x1476): satellites used and detected for GPS / GLONASS / BeiDou (the report has no Galileo or SBAS
      counts), PDOP / HDOP / VDOP, error-ellipse axes and confidence, position source, failure code, the position/velocity and
      fix-event bit fields (raw, not decoded here), horizontal reliability, and the fix flag as qcomgpsd defines it;
  measurement reports (0x1477 GPS, 0x1480 GLONASS): per constellation, satellites tracked and usable (measurementNotUsable,
      bit 13, clear), C/N0 median and top four (carrier_noise in 0.1 dB-Hz per structs.py).
Ported from gnss-assist-20261126 c7b77d65 WITHOUT its NMEA source (/dev/ttyUSB1): opening that tty could not be shown to
leave ModemManager's own use of the port undisturbed, so per-talker Galileo / SBAS / QZSS / BeiDou C/N0, GSA satellites in
use per system and the 2D/3D fix type are not logged. The BeiDou (0x1756) and Galileo (0x1886) diag measurement logs are
not requested (no decoder in this tree).
"""
# RUNTIME: runs in qcomgpsd, on-road, not on the control core, collector on. add_* once per diag report (about 1 Hz each),
# called by receiver_log.ReceiverLog after the stock message was sent; summary() every SUMMARY_S. Fixed size: the position
# digest and one entry per constellation are overwritten, never appended; C/N0 lists are capped at MAX_SV.
import math

SUMMARY_S = 5.0
MAX_SV = 64
MEAS_NOT_USABLE_BIT = 13


def _r(x, nd=2):
  return round(x, nd) if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) else None


def _i(x):
  return x if isinstance(x, int) and not isinstance(x, bool) else None


def _cn0(vals):
  v = sorted((x for x in vals if x > 0), reverse=True)
  if not v:
    return None
  return {'median': round(v[len(v) // 2], 1), 'top4': [round(x, 1) for x in v[:4]]}


class GnssStatus:
  """Single-threaded (called from qcomgpsd's diag loop only)."""
  def __init__(self):
    self.pos = None             # last position-report digest (no position fields)
    self.meas = {}              # constellation -> {'tracked', 'usable', 'cn0': [...]} from the last measurement report
    self.n_pos = 0
    self.n_meas = 0

  def add_position(self, r: dict, has_fix: bool):
    self.pos = {
      'fix': bool(has_fix), 'src': _i(r.get('u_PosSource')), 'fail': _i(r.get('u_FailureCode')),
      'pv_flags': _i(r.get('w_PosVelFlag')), 'fix_events': _i(r.get('w_FixEvents')),
      'used': {'gps': _i(r.get('u_NumGpsSvsUsed')), 'glo': _i(r.get('u_NumGloSvsUsed')), 'bds': _i(r.get('u_NumBdsSvsUsed'))},
      'seen': {'gps': _i(r.get('u_TotalGpsSvs')), 'glo': _i(r.get('u_TotalGloSvs')), 'bds': _i(r.get('u_TotalBdsSvs'))},
      'pdop': _r(r.get('q_FltPdop')), 'hdop': _r(r.get('q_FltHdop')), 'vdop': _r(r.get('q_FltVdop')),
      'ell_major_m': _r(r.get('q_FltEllipseSemimajorAxis')), 'ell_minor_m': _r(r.get('q_FltEllipseSemiminorAxis')),
      'ell_conf': _i(r.get('u_EllipseConfidence')), 'h_rel': _i(r.get('u_HorizontalReliability')),
    }
    self.n_pos += 1

  def add_measurement(self, constellation: str, svs):
    """svs: per-satellite dicts as structs.dict_unpacker(..., camelcase=True) returns them."""
    tracked = usable = 0
    cn0 = []
    for sv in svs[:MAX_SV]:
      c = int(sv.get('carrierNoise') or 0)
      if c > 0 or int(sv.get('goodObservations') or 0) > 0:
        tracked += 1
        if not (int(sv.get('measurementStatus') or 0) >> MEAS_NOT_USABLE_BIT) & 1:
          usable += 1
      if c > 0:
        cn0.append(c / 10.0)
    self.meas[constellation] = {'tracked': tracked, 'usable': usable, 'cn0': cn0}
    self.n_meas += 1

  def summary(self) -> dict:
    out = {'pos': self.pos, 'n_pos': self.n_pos, 'n_meas': self.n_meas,
           'diag': {k: {'tracked': v['tracked'], 'usable': v['usable'], 'cn0': _cn0(v['cn0'])} for k, v in sorted(self.meas.items())}}
    self.pos = None
    self.meas = {}
    self.n_pos = self.n_meas = 0
    return out
