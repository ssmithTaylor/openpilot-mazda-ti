"""A once-a-minute summary of the receiver's own accuracy figures, for the drive log (no positions).

gpsLocation.horizontalAccuracy is left at 0 on purpose: locationd reads it (selfdrive/locationd/locationd.cc handle_gps: the
norm of horizontal and vertical accuracy >= SANE_GPS_UNCERTAINTY rejects the fix), so publishing it would change what locationd
accepts. The position report's horizontal figures (error-ellipse axes, HDOP, reliability) and satellites used are summarised
here instead, so drives can be analysed against the observed offsets before anyone decides to publish them. Whether the
ellipse fields are read at the right offsets of the report (structs.py pads with align_Flt[14] before PDOP) is not verified.
"""
# RUNTIME: runs in qcomgpsd, on-road, once per position report (1 Hz), not on the control core; collector on. Fixed-size: at
# most WINDOW values per field, emptied at every summary.
import math

WINDOW = 60
FIELDS = {
  'semimajor_m': 'q_FltEllipseSemimajorAxis',
  'semiminor_m': 'q_FltEllipseSemiminorAxis',
  'hdop': 'q_FltHdop',
  'vdop': 'q_FltVdop',           # what is published as verticalAccuracy today
  'sigma_vert_m': 'q_FltPosSigmaVertical',
}
COUNTS = {'gps_used': 'u_NumGpsSvsUsed', 'glo_used': 'u_NumGloSvsUsed', 'bds_used': 'u_NumBdsSvsUsed',
          'h_reliability': 'u_HorizontalReliability', 'ellipse_conf': 'u_EllipseConfidence'}


def _stats(vals):
  v = sorted(x for x in vals if isinstance(x, (int, float)) and math.isfinite(x))
  if not v:
    return None
  return [round(v[0], 2), round(v[len(v) // 2], 2), round(v[-1], 2)]   # min, median, max


class FixQuality:
  def __init__(self, window: int = WINDOW):
    self.window = window
    self.vals = {k: [] for k in (*FIELDS, *COUNTS)}
    self.n = 0
    self.n_fix = 0

  def add(self, report: dict, has_fix: bool):
    """Add one position report; returns the summary dict when the window is full (then starts a new one), else None."""
    self.n += 1
    if has_fix:
      self.n_fix += 1
      for k, f in (*FIELDS.items(), *COUNTS.items()):
        if f in report:
          self.vals[k].append(report[f])
    if self.n < self.window:
      return None
    out = {'reports': self.n, 'fixes': self.n_fix}
    out.update({k: _stats(v) for k, v in self.vals.items()})
    for v in self.vals.values():
      v.clear()
    self.n = self.n_fix = 0
    return out
