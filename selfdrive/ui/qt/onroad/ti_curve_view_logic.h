#pragma once

// Curve slowing's driving-screen element (owner's approved design, 2026-11-26: mock-up round 5, render5.py): everything that decides
// what is drawn, without Qt, so it is compiled and tested on its own (selfdrive/ui/tests/test_ti_curve_view_logic.cc, run by
// selfdrive/car/tests/test_mazda_ti_curve_view_logic.py and in the Docker image). The drawing (ti_curve_view.cc) takes every
// number from here.
//   - the input: ti_curve_plannerd's tmpfs param TiCurveView (selfdrive/car/mazda/ti_curve_view.py), parsed into fixed arrays
//     (no allocation per read);
//   - when it is drawn (visibility): curve slowing on, openpilot engaged, the car's cruise active (owner 2026-11-26: hidden while
//     cruise is cancelled and openpilot still steers), at or above 19 mph (hidden again below 18.5), the planner's
//     tick at most 1 s old; otherwise nothing at all (the chip never shows a stale number);
//   - the ribbon: the road ahead for 45 s, bends 30 % sharper than true, the near road enlarged (distance compressed beyond about
//     12 s), fitted to its bounding box inside its area on the right-hand side (layout(): clear of every other element), the fit smoothed over 1 s and replaced at once by the fitting one when
//     the smoothed one would leave the area; colours per point, the speed-back-up rings, the fade where the memory ends;
//   - the chip: red from the bend's lead (the driver's braking time + 6 s, never under 13 s; the planner computes it), else yellow
//     from 10 s; seconds to the bend counted down from the tick by the age of the data.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

namespace ti_view {

constexpr int VERSION = 1;
constexpr const char *PARAM_KEY = "TiCurveView";
constexpr double MPH = 0.44704;
constexpr double PI = 3.14159265358979323846;

// ---- screen geometry (2160 x 1080 device screen; the onroad camera widget sits inside a 30 px border) ---------------------------
constexpr int SCREEN_W = 2160;
constexpr int SCREEN_H = 1080;
constexpr int BORDER = 30;
// the ribbon's area is no longer fixed: layout() (below) places it on the right-hand side, clear of every other element (owner,
// 2026-11-26: "move the ribbon to the right-hand side and prevent it from overlapping")
constexpr double FIT_FILL = 0.94;      // the bounding box fills this share of the area
constexpr double EXAG = 1.3;           // bends drawn this much sharper than true (shape only)
constexpr double NEAR_S = 12.0;        // road nearer than this many seconds is drawn at full scale; farther road compressed
constexpr double TAU_S = 1.0;          // smoothing of the fit (scale and centre)
constexpr double INSIDE_TOL = 1.0;     // px: a smoothed fit with a point further out than this is replaced by the fitting one

// ---- visibility ---------------------------------------------------------------------------------------------------------------
constexpr double STALE_S = 1.0;        // older planner output is not drawn
constexpr double FUTURE_S = 0.25;      // a tick time this far ahead of the UI's clock is still accepted (rounding to 0.01 s)
constexpr double SHOW_MPH = 19.0;      // cruise and the comma engage from about 19 mph
constexpr double HIDE_MPH = 18.5;      // ... hidden again only below this (no flicker at the threshold)

// ---- the ribbon's look ----------------------------------------------------------------------------------------------------------
constexpr double WIDTH_NEAR = 44.0;    // px at the car, tapering by WIDTH_TAPER to the far end
constexpr double WIDTH_TAPER = 18.0;
constexpr double HALO_EXTRA = 14.0;    // the dark halo is this much wider than the ribbon
constexpr double APPROACH_WIDTH = 0.7; // a bend's approach: narrower and translucent
constexpr int ALPHA_NEUTRAL = 220, ALPHA_APPROACH = 150, ALPHA_BEND = 255, ALPHA_HALO = 150;
constexpr double FADE_M = 60.0;        // the ribbon fades out over this where the remembered road ends
constexpr double HATCH_EVERY_PX = 22.0;
constexpr double RESUME_GAP_M = 150.0; // no speed-back-up ring where the next bend's approach starts within this
constexpr double RING_EXTRA = 8.0;

// colours (render2.py / render4.py / render5.py)
struct Rgb { int r, g, b; };
constexpr Rgb NEUTRAL_RGB{255, 255, 255};
constexpr Rgb YELLOW_RGB{255, 186, 0};
constexpr Rgb RED_RGB{226, 44, 44};
constexpr Rgb GO_RGB{51, 204, 102};
constexpr Rgb HATCH_RGB{40, 0, 0};
constexpr Rgb SIGN_YELLOW{255, 204, 0};
constexpr Rgb SIGN_RED{210, 30, 40};

// ---- the chip ---------------------------------------------------------------------------------------------------------------------
// beside the MAX box: its left edge 20 px right of the box, its top level with the box's (screen x 282, y 75 for the 172 px box)
constexpr int CHIP_GAP = 20;
constexpr int YELLOW_W = 470, YELLOW_H = 204;
constexpr int RED_W = 600, RED_BAND_H = 172, RED_H = 172 + 190;
constexpr int BAND_TEXT_PX = 66;       // HANDS ON / HELP STEER: cap height about 48 px
constexpr int SECS_PX = 64;

enum Cls { NEUTRAL = 0, YELLOW = 1, RED = 2 };
// where the published bends come from (ti_curve_view.py SRC_*): the remembered road (with the ribbon), the map only, other
enum Src { SRC_RECORD = 0, SRC_MAP = 1, SRC_OTHER = 2 };

constexpr int MAX_PTS = 128;
constexpr int MAX_BENDS = 8;

struct Bend {
  int start = 0, low = 0, end = 0, dir = 0, need = 0, target = 0, cls = 0, approach = 0, resume = 0, lead_ds = 0;
};

struct Payload {
  bool ok = false;
  int v = 0, seq = 0, on = 0, mph10 = 0, floor = 0, ends = 0;
  int src = SRC_RECORD;   // optional key "src" (absent in payloads written before it: the remembered road)
  double m = -1e9;
  int n = 0;
  int d[MAX_PTS] = {};
  int k[MAX_PTS] = {};
  int nb = 0;
  Bend b[MAX_BENDS];
  double mph() const { return mph10 / 10.0; }
  double v_ms() const { return std::max(mph() * MPH, 1.0); }
};

// ---- parsing (the fixed schema ti_curve_view.payload writes; anything else: not ok) -----------------------------------------------
namespace detail {
inline const char *after_key(const char *s, const char *key) {
  const size_t kl = std::strlen(key);
  for (const char *p = std::strstr(s, key); p != nullptr; p = std::strstr(p + 1, key)) {
    if (p > s && p[-1] == '"' && p[kl] == '"' && p[kl + 1] == ':') return p + kl + 2;
  }
  return nullptr;
}
inline void ws(const char *&p) { while (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r') p++; }
inline bool num(const char *&p, double &out) {
  ws(p);
  char *e = nullptr;
  out = std::strtod(p, &e);
  if (e == p || !std::isfinite(out)) return false;
  p = e;
  return true;
}
inline bool ch(const char *&p, char c) {
  ws(p);
  if (*p != c) return false;
  p++;
  return true;
}
inline bool key_num(const char *s, const char *key, double &out) {
  const char *p = after_key(s, key);
  return p != nullptr && num(p, out);
}
// a list of fixed-length int rows: [[a, b, ...], ...]; -> rows read, or -1 on a malformed list
inline int rows(const char *s, const char *key, int width, int *out, int max_rows) {
  const char *p = after_key(s, key);
  if (p == nullptr || !ch(p, '[')) return -1;
  int n = 0;
  if (ch(p, ']')) return 0;
  while (true) {
    if (!ch(p, '[')) return -1;
    for (int j = 0; j < width; j++) {
      double x;
      if ((j > 0 && !ch(p, ',')) || !num(p, x)) return -1;
      if (n < max_rows) out[n * width + j] = (int)std::lround(x);
    }
    if (!ch(p, ']')) return -1;
    n++;
    if (ch(p, ']')) break;
    if (!ch(p, ',')) return -1;
  }
  return std::min(n, max_rows);
}
}  // namespace detail

inline bool parse(const std::string &text, Payload &p) {
  p = Payload();
  const char *s = text.c_str();
  double v, m, seq, on, mph10, fl, re;
  if (!detail::key_num(s, "v", v) || (int)v != VERSION) return false;
  if (!detail::key_num(s, "m", m) || !detail::key_num(s, "seq", seq) || !detail::key_num(s, "on", on)) return false;
  if (!detail::key_num(s, "mph", mph10) || !detail::key_num(s, "fl", fl) || !detail::key_num(s, "re", re)) return false;
  int rbuf[MAX_PTS * 2];
  int bbuf[MAX_BENDS * 10];
  const int n = detail::rows(s, "r", 2, rbuf, MAX_PTS);
  const int nb = detail::rows(s, "b", 10, bbuf, MAX_BENDS);
  if (n < 0 || nb < 0) return false;
  p.v = (int)v; p.m = m; p.seq = (int)seq; p.on = (int)on; p.mph10 = (int)mph10; p.floor = (int)fl; p.ends = (int)re;
  double src;
  p.src = detail::key_num(s, "src", src) && (int)src >= SRC_RECORD && (int)src <= SRC_OTHER ? (int)src : SRC_RECORD;
  p.n = n;
  for (int i = 0; i < n; i++) { p.d[i] = rbuf[2 * i]; p.k[i] = rbuf[2 * i + 1]; }
  p.nb = nb;
  for (int i = 0; i < nb; i++) {
    const int *q = bbuf + 10 * i;
    p.b[i] = Bend{q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9]};
  }
  p.ok = true;
  return true;
}

// ---- when it is drawn -------------------------------------------------------------------------------------------------------------
enum Hidden { SHOWN = 0, NO_DATA, SWITCHED_OFF, STALE, NOT_ENGAGED, TOO_SLOW, CRUISE_OFF };

// engaged: openpilot engaged (the UI's status ENGAGED or OVERRIDE); cruise_active: the car's cruise is on and active
// (carState.cruiseState.enabled). With cruise cancelled nothing is drawn, whether or not openpilot still steers (FrogPilot's
// always-on lateral): owner, 2026-11-26, "for now"; the element returns as soon as cruise is active again. Both inputs are in
// the drive log (carState, controlsState), so this decision needs no log of its own.
inline Hidden visibility(const Payload &p, double now, bool engaged, bool cruise_active, double ui_mph, bool was_shown) {
  if (!p.ok) return NO_DATA;
  if (!p.on) return SWITCHED_OFF;
  const double age = now - p.m;
  if (age > STALE_S || age < -FUTURE_S) return STALE;
  if (!cruise_active) return CRUISE_OFF;
  if (!engaged) return NOT_ENGAGED;
  if (ui_mph < (was_shown ? HIDE_MPH : SHOW_MPH)) return TOO_SLOW;
  return SHOWN;
}

// ---- the chip ---------------------------------------------------------------------------------------------------------------------
struct Chip {
  int cls = NEUTRAL;   // NEUTRAL: no chip
  int bend = -1;
  int mph = 0;         // red: the needed speed; yellow: the target the car slows to
  int secs = 0;        // whole seconds to the bend
  int dir = 0;         // +1 left, -1 right, 0 unknown
};

// the mark inside the chip's curve sign (owner, 2026-11-26): a bend known from the map only (no remembered record, no ribbon) shows
// "?"; any other bend keeps its mark: the curve arrow when its direction is known, else "!"
enum Mark { MARK_ARROW = 0, MARK_BANG = 1, MARK_QUESTION = 2 };
inline Mark chipMark(const Payload &p, const Chip &c) {
  if (p.src == SRC_MAP) return MARK_QUESTION;
  return c.dir != 0 ? MARK_ARROW : MARK_BANG;
}

inline double secondsTo(const Payload &p, const Bend &b, double age) {
  return std::max(0.0, b.start / p.v_ms() - std::max(0.0, age));
}

inline Chip chipFor(const Payload &p, double age) {
  Chip c;
  for (int pass = 0; pass < 2 && c.cls == NEUTRAL; pass++) {
    const int want = pass == 0 ? RED : YELLOW;
    for (int i = 0; i < p.nb; i++) {
      const Bend &b = p.b[i];
      if (b.cls != want) continue;
      const double s = secondsTo(p, b, age);
      if (s <= b.lead_ds / 10.0) {
        c.cls = want;
        c.bend = i;
        c.mph = want == RED ? b.need : b.target;
        c.secs = (int)std::floor(s + 0.5);
        c.dir = b.dir;
        break;
      }
    }
  }
  return c;
}

// ---- the ribbon -----------------------------------------------------------------------------------------------------------------
struct Ribbon {
  int n = 0;
  double d[MAX_PTS] = {};
  double x[MAX_PTS] = {};   // metres-like, left negative, before the fit
  double y[MAX_PTS] = {};   // forward
};

// the shape (render5.py ribbon_raw with the near road enlarged): heading integrates EXAG x curvature; each step's length is
// compressed by NEAR_S v / (NEAR_S v + distance)
inline void ribbonRaw(const Payload &p, Ribbon &r) {
  r.n = 0;
  const double v = p.v_ms();
  const double d0 = NEAR_S * v;
  double th = 0.0, x = 0.0, y = 0.0, prev = 0.0;
  for (int i = 0; i < p.n && r.n < MAX_PTS; i++) {
    const double d = p.d[i];
    if (d < 0.0 || d > 45.0 * v + 1.0) continue;
    const double ds = d - prev;
    prev = d;
    th += EXAG * (p.k[i] * 1e-5) * ds;
    const double f = d0 / (d0 + d);
    x += -std::sin(th) * ds * f;
    y += std::cos(th) * ds * f;
    r.d[r.n] = d; r.x[r.n] = x; r.y[r.n] = y;
    r.n++;
  }
}

// ---- layout: where the ribbon may be drawn (owner, 2026-11-26) -------------------------------------------------------------------
// Coordinates here are the camera widget's own (AnnotatedCameraWidget: (0, 0) at its top-left, inside the onroad border; narrower when
// the sidebar or the map is open). Every element this fork draws on that widget is listed by obstacles() with the rectangle it can
// occupy (mirroring annotated_camera.cc, buttons.cc, alerts.cc and FrogPilot's frogpilot_annotated_camera.cc); elements whose
// position moves with toggles or transient state are reserved over every place they can take. ribbonRect() then picks the largest
// rectangle on the right-hand side that is LAYOUT_GAP clear of all of them and inside the widget, or none (the ribbon is then not
// drawn; it never draws over anything). The ribbon's points are fitted INK inside that rectangle and the drawing is clipped to it.
struct Rect {
  double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  double w() const { return x1 - x0; }
  double h() const { return y1 - y0; }
  bool empty() const { return !(x1 > x0 && y1 > y0); }
};
inline bool overlaps(const Rect &a, const Rect &b) {
  return !a.empty() && !b.empty() && a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}
inline bool within(const Rect &a, const Rect &outer) {
  return a.x0 >= outer.x0 && a.y0 >= outer.y0 && a.x1 <= outer.x1 && a.y1 <= outer.y1;
}
inline Rect inset(const Rect &r, double d) { return Rect{r.x0 + d, r.y0 + d, r.x1 - d, r.y1 - d}; }

constexpr int UI_BORDER = 30;          // ui.h UI_BORDER_SIZE
constexpr int BTN = 192;               // buttons.h btn_size
constexpr int FP_WIDGET = 159;         // FrogPilot widget_size = img_size (144) + UI_BORDER_SIZE / 2
constexpr int ALERT_SMALL_H = 271, ALERT_MID_H = 420, ALERT_MARGIN = 40;   // alerts.cc
constexpr double LAYOUT_GAP = 20.0;    // clear space between the ribbon's rectangle and any other element
constexpr double RIBBON_MAX_W = 480.0, RIBBON_MAX_H = 560.0;   // the ribbon's rectangle at most ...
constexpr double RIBBON_MIN_W = 260.0, RIBBON_MIN_H = 240.0;   // ... and at least (smaller: not drawn at all)
constexpr double RIGHT_OF_SPEED = 270.0;  // the ribbon stays this far right of the widget's centre (the speed readout's half-width + gap)
constexpr double INK = 40.0;           // the drawing reaches this far beyond the ribbon's points (halo 33 px, car chevron 37, ring 35)

enum AlertSize { ALERT_NONE = 0, ALERT_SMALL = 1, ALERT_MID = 2, ALERT_FULL = 3 };   // cereal ControlsState.AlertSize order
enum SignalStyle { SIGNAL_OFF = 0, SIGNAL_STATIC = 1, SIGNAL_SWEEP = 2 };          // FrogPilot's turn-signal animation, blinker on

// the camera widget's state for the layout (annotated_camera.cc fills it every frame from what it already has)
struct Screen {
  int w = SCREEN_W - 2 * BORDER, h = SCREEN_H - 2 * BORDER;
  int alert = ALERT_NONE;      // the larger of controlsState's alert size and the alerts widget's last painted height
  int alert_h = 0;             // the alerts widget's last painted height less its margin (FrogPilot's alertHeight)
  bool pedals = false;         // FrogPilot "pedals on UI": brake and gas icons under the experimental button
  int signal = SIGNAL_OFF;     // FrogPilot turn-signal images while a blinker is on (static: top beside the speed; sweep: bottom)
  int signal_w = 0, signal_h = 0;
  Rect max_box{60, 45, 232, 249};   // the set-speed box with any speed-limit sign under it (drawHud set_speed_rect)
  Rect lead_text;              // FrogPilot's lead-metrics text under the lead marker (leadTextRect); empty when none
};

struct Element {
  const char *name;
  Rect r;
};
constexpr int MAX_ELEMENTS = 24;

// every element on the camera widget the ribbon must stay clear of -> how many (in out[])
inline int obstacles(const Screen &s, Element *out) {
  int n = 0;
  const double W = s.w, H = s.h, cx = s.w / 2.0;
  const Rect &mb = s.max_box;
  auto add = [&](const char *name, Rect r) { if (n < MAX_ELEMENTS && !r.empty()) out[n++] = Element{name, r}; };
  // top right: the experimental-mode button (layout margin 30, top-right) and FrogPilot's screen recorder left of it
  add("experimental button", Rect{W - UI_BORDER - BTN, UI_BORDER, W - UI_BORDER, UI_BORDER + BTN});
  add("screen recorder button", Rect{W - 2 * UI_BORDER - 2 * BTN, UI_BORDER, W - 2 * UI_BORDER - BTN, UI_BORDER + BTN});
  if (s.pedals) {   // brake at the button's x, gas half a button right of it (runs off the widget's edge), one border below
    add("pedal icons", Rect{W - UI_BORDER - BTN, 2 * UI_BORDER + BTN, W, 2 * UI_BORDER + 2 * BTN});
  }
  // the bottom row, reserved across the whole width: driver-monitoring icon (centre h - 126, or h - 156 with the road-name toggle;
  // radius 96), following-distance button (192 + 30 wide, centred on the icon's height), CEM status, lateral / longitudinal paused,
  // compass, weather (159 px, top at the icon's centre - 79), map settings button (192 x 222, bottom right), road name (h - 55)
  add("bottom row (DM icon, distance button, map button, compass, weather, CEM, paused icons, road name)",
      Rect{0, H - UI_BORDER - BTN / 2 - UI_BORDER - BTN / 2, W, H});
  // top centre: the current speed (176 px digits at y 210, unit at 290)
  add("speed readout", Rect{cx - RIGHT_OF_SPEED + LAYOUT_GAP, UI_BORDER, cx + RIGHT_OF_SPEED - LAYOUT_GAP, 310});
  // top left: the set-speed box and sign, and what FrogPilot or we put beside or under it (reserved whether shown or not)
  add("set-speed box / speed-limit sign", mb);
  add("speed-limit sources list", Rect{mb.x0, mb.y1, mb.x0 + 450, mb.y1 + 320});
  add("steer-authority box", Rect{mb.x0, mb.y1 + UI_BORDER, mb.x1, mb.y1 + UI_BORDER + 130});
  add("curve speed control icon + text", Rect{mb.x1 + UI_BORDER, mb.y0, mb.x1 + UI_BORDER + 215, mb.y0 + 215 + 110});
  add("pending speed limit", Rect{mb.x1, mb.y0, mb.x1 + UI_BORDER + 200, mb.y1});
  add("curve chip (ours, red size)", Rect{mb.x1 + CHIP_GAP, mb.y0, mb.x1 + CHIP_GAP + RED_W, mb.y0 + RED_H});
  if (s.signal == SIGNAL_STATIC) {
    add("turn signal (static, left)", Rect{cx * 0.75 - s.signal_w, s.signal_h / 2.0, cx * 0.75, s.signal_h * 1.5});
    add("turn signal (static, right)", Rect{cx * 1.25, s.signal_h / 2.0, cx * 1.25 + s.signal_w, s.signal_h * 1.5});
  } else if (s.signal == SIGNAL_SWEEP) {
    add("turn signal / blind spot (sweep)", Rect{0, H - s.signal_h - s.alert_h, W, H - s.alert_h});
  }
  if (s.alert == ALERT_FULL) {
    add("alert banner (full)", Rect{0, 0, W, H});
  } else if (s.alert == ALERT_MID) {
    add("alert banner (mid)", Rect{ALERT_MARGIN, H - ALERT_MID_H + ALERT_MARGIN, W - ALERT_MARGIN, H - ALERT_MARGIN});
  } else if (s.alert == ALERT_SMALL) {
    add("alert banner (small)", Rect{ALERT_MARGIN, H - ALERT_SMALL_H + ALERT_MARGIN, W - ALERT_MARGIN, H - ALERT_MARGIN});
  }
  add("lead metrics text", s.lead_text);
  return n;
}

// where the ribbon may draw: right-hand side, inside the widget less its border margin, LAYOUT_GAP clear of every obstacle; the
// largest such rectangle (capped at RIBBON_MAX_W x RIBBON_MAX_H; ties: the rightmost, then the lowest), or an empty one
inline Rect ribbonRect(const Screen &s) {
  Element obs[MAX_ELEMENTS];
  const int n = obstacles(s, obs);
  const double xmin = s.w / 2.0 + RIGHT_OF_SPEED, xmax = s.w - UI_BORDER, ymin = UI_BORDER, ymax = s.h - UI_BORDER;
  Rect best;
  double best_area = 0.0;
  double x1s[MAX_ELEMENTS + 1];
  int nx = 0;
  x1s[nx++] = xmax;
  for (int i = 0; i < n; i++) {
    const double x1 = obs[i].r.x0 - LAYOUT_GAP;
    if (x1 < xmax && x1 - xmin >= RIBBON_MIN_W) x1s[nx++] = x1;
  }
  for (int a = 0; a < nx; a++) {
    const double x1 = x1s[a];
    for (double wd = std::min(RIBBON_MAX_W, x1 - xmin); wd >= RIBBON_MIN_W; wd -= 20.0) {
      const double x0 = x1 - wd;
      // the obstacles in this column, as vertical intervals widened by the gap, sorted by their top
      double lo[MAX_ELEMENTS], hi[MAX_ELEMENTS];
      int m = 0;
      for (int i = 0; i < n; i++) {
        const Rect &o = obs[i].r;
        if (o.x0 - LAYOUT_GAP < x1 && o.x1 + LAYOUT_GAP > x0) {
          lo[m] = o.y0 - LAYOUT_GAP; hi[m] = o.y1 + LAYOUT_GAP;
          for (int j = m; j > 0 && lo[j] < lo[j - 1]; j--) { std::swap(lo[j], lo[j - 1]); std::swap(hi[j], hi[j - 1]); }
          m++;
        }
      }
      double y = ymin;
      for (int i = 0; i <= m; i++) {
        const double gap_end = std::min(i < m ? lo[i] : ymax, ymax);
        if (gap_end - y >= RIBBON_MIN_H) {
          const double ht = std::min(RIBBON_MAX_H, gap_end - y);
          const double area = wd * ht;
          if (area > best_area + 1.0) {
            best_area = area;
            best = Rect{x0, gap_end - ht, x1, gap_end};
          }
        }
        if (i < m) y = std::max(y, hi[i]);
      }
    }
  }
  return best;
}

// the ribbon's points go INK inside its rectangle, in the painter's coordinates (the widget's moved by ox, oy)
inline Rect pointsArea(const Rect &rr, double ox, double oy) {
  const Rect a = inset(rr, INK);
  return Rect{a.x0 + ox, a.y0 + oy, a.x1 + ox, a.y1 + oy};
}

// the chip is hidden only under a full-screen alert (it sits top left, above the small and mid banners and the bottom row)
inline bool chipAllowed(const Screen &s) { return s.alert != ALERT_FULL; }

struct Fit {
  bool init = false;
  double s = 1.0, cx = 0.0, cy = 0.0;
  int jumps = 0;
  // the points' area (the coordinates px / py are given in): ribbonRect less INK, moved to the painter's coordinates
  Rect area = pointsArea(ribbonRect(Screen()), BORDER, BORDER);
  void reset() { init = false; }
  // a new area (the layout changed): the next step starts from the fitting layout there
  void setArea(const Rect &a) {
    if (a.x0 != area.x0 || a.y0 != area.y0 || a.x1 != area.x1 || a.y1 != area.y1) { area = a; init = false; }
  }
  // -> coordinates of every ribbon point inside the area (px, py); dt: seconds since the last step
  void step(const Ribbon &r, double dt, double *px, double *py) {
    if (r.n < 1) return;
    double bx0 = r.x[0], bx1 = r.x[0], by0 = r.y[0], by1 = r.y[0];
    for (int i = 1; i < r.n; i++) {
      bx0 = std::min(bx0, r.x[i]); bx1 = std::max(bx1, r.x[i]);
      by0 = std::min(by0, r.y[i]); by1 = std::max(by1, r.y[i]);
    }
    const double aw = area.w(), ah = area.h();
    const double s_fit = FIT_FILL * std::min(aw / std::max(1.0, bx1 - bx0), ah / std::max(1.0, by1 - by0));
    const double cx_fit = (bx0 + bx1) / 2, cy_fit = (by0 + by1) / 2;
    if (!init) {
      s = s_fit; cx = cx_fit; cy = cy_fit; init = true;
    } else {
      const double a = std::min(1.0, std::max(0.0, dt) / TAU_S);
      s += a * (s_fit - s); cx += a * (cx_fit - cx); cy += a * (cy_fit - cy);
    }
    if (!place(r, px, py)) {
      s = s_fit; cx = cx_fit; cy = cy_fit; jumps++;
      place(r, px, py);
    }
  }
  bool place(const Ribbon &r, double *px, double *py) const {
    bool inside = true;
    for (int i = 0; i < r.n; i++) {
      px[i] = (area.x0 + area.x1) / 2 + (r.x[i] - cx) * s;
      py[i] = (area.y0 + area.y1) / 2 - (r.y[i] - cy) * s;
      inside = inside && px[i] >= area.x0 - INSIDE_TOL && px[i] <= area.x1 + INSIDE_TOL && py[i] >= area.y0 - INSIDE_TOL &&
               py[i] <= area.y1 + INSIDE_TOL;
    }
    return inside;
  }
};

// per point: colour class, approach (narrower, translucent), red stretch (hatched), width, alpha factor (the fade)
struct Style {
  int cls[MAX_PTS] = {};
  bool approach[MAX_PTS] = {};
  bool hatch[MAX_PTS] = {};
  double width[MAX_PTS] = {};
  double fade[MAX_PTS] = {};
};

inline void styleFor(const Payload &p, const Ribbon &r, Style &st) {
  const double last = r.n ? r.d[r.n - 1] : 0.0;
  for (int i = 0; i < r.n; i++) {
    const double d = r.d[i];
    st.cls[i] = NEUTRAL; st.approach[i] = false; st.hatch[i] = false;
    for (int j = 0; j < p.nb; j++) {
      const Bend &b = p.b[j];
      const bool a = d >= b.approach && d < b.start;
      const bool m = d >= b.start && d <= b.end;
      if (a || m) st.cls[i] = b.cls == RED ? RED : YELLOW;
      st.approach[i] = st.approach[i] || a;
      st.hatch[i] = st.hatch[i] || (m && b.cls == RED);
    }
    st.width[i] = WIDTH_NEAR - WIDTH_TAPER * d / std::max(last, 1.0);
    st.fade[i] = p.ends ? std::min(1.0, std::max(0.0, (last - d) / FADE_M)) : 1.0;
  }
}

inline int alphaFor(const Style &st, int i) {
  const int a = st.cls[i] == NEUTRAL ? ALPHA_NEUTRAL : (st.approach[i] ? ALPHA_APPROACH : ALPHA_BEND);
  return (int)std::lround(a * st.fade[i]);
}

inline double strokeWidth(const Style &st, int i) { return st.width[i] * (st.approach[i] ? APPROACH_WIDTH : 1.0); }

// the speed-back-up rings: the first ribbon point at or past each bend's resume point, unless the next bend's approach starts within
// RESUME_GAP_M of it (no speed-up there). -> how many (indices into the ribbon in out[])
inline int resumeRings(const Payload &p, const Ribbon &r, int *out, int max_out) {
  int n = 0;
  for (int i = 0; i < p.nb && n < max_out; i++) {
    const double nxt = i + 1 < p.nb ? p.b[i + 1].approach : 1e9;
    if (nxt - p.b[i].resume < RESUME_GAP_M) continue;
    for (int j = 0; j < r.n; j++) {
      if (r.d[j] >= p.b[i].resume) { out[n++] = j; break; }
    }
  }
  return n;
}

// the hatch bars across the red stretch, one every HATCH_EVERY_PX of screen length: -> how many, the point index of each in out[]
inline int hatchMarks(const Style &st, const double *px, const double *py, int n, int *out, int max_out) {
  int k = 0;
  double acc = 0.0;
  for (int i = 1; i < n && k < max_out; i++) {
    acc += std::hypot(px[i] - px[i - 1], py[i] - py[i - 1]);
    if (st.hatch[i] && acc >= HATCH_EVERY_PX) {
      acc = 0.0;
      out[k++] = i;
    }
  }
  return k;
}

// the car chevron: its position is the first point, its heading towards the fourth (render5.py)
inline double carAngle(const double *px, const double *py, int n) {
  const int j = std::min(3, n - 1);
  const double hx = px[j] - px[0], hy = py[j] - py[0];
  return (std::fabs(hx) + std::fabs(hy) > 1e-3) ? std::atan2(hy, hx) : -PI / 2;
}

// the chip's box (screen coordinates) for the MAX box's right edge and top
struct Box { int x, y, w, h; };
inline Box chipBox(const Chip &c, int max_right, int max_top) {
  const int x = max_right + CHIP_GAP;
  return c.cls == RED ? Box{x, max_top, RED_W, RED_H} : Box{x, max_top, YELLOW_W, YELLOW_H};
}

}  // namespace ti_view
