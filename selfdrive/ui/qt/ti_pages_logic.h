#pragma once

// Mazda TI on-car pages (first page, set-up page, individual tests): everything that decides how they look and what a
// control does, without Qt, so it is compiled and tested on its own
// (selfdrive/ui/tests/test_ti_pages_logic.cc, run by selfdrive/car/tests/test_mazda_ti_pages_logic.py) and drawn off the
// device by selfdrive/ui/tests/dump_ti_pages_layout.cc (the mock-up).
//   - layout: the device screen (2160 x 1080, qt_window.h DEVICE_SCREEN_SIZE), every control's rectangle on every page,
//     the fonts. sidebar.cc takes every rectangle from here. Tests: no overlaps, minimum gaps, sizes.
//   - the text of the first page: one primary status line, one second line, the experiment in plain words, the label of the
//     START control. Tests: every state fits its line.
//   - what START and STOP write: the same ordered writes as RUN ALL, the DRIVE PLAN box and ALL TESTS OFF made before this
//     rework (documented in the test, compared there). Nothing here is a gate, a limit or a controller setting.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "selfdrive/ui/qt/ti_ab_logic.h"
#include "selfdrive/ui/qt/ti_tests_logic.h"

namespace ti_pages {

// ---- the screen and the fonts ---------------------------------------------------------------------------------------
constexpr int SCREEN_W = 2160;
constexpr int SCREEN_H = 1080;
constexpr int MIN_GAP = 40;        // between any two controls
constexpr int STOP_GAP = 80;       // around STOP / ALL TESTS OFF / A/B OFF, to any other control
constexpr int BIG_MIN = 180;       // smaller dimension of LOCK, PIN A and PIN B
constexpr int ROW_TOP = 490;       // where the content of the individual pages starts (under two rows of large controls)
constexpr int BOTTOM_STRIP = 74;   // the foot of the individual pages: the plan line and the ENGAGED badge, never under content

// Font sizes in pixels (InterFont(px), the device's own font; bold for the status lines and every control label)
constexpr int PRIMARY_PX = 100;    // the primary status line
constexpr int SECOND_PX = 70;      // the second line
constexpr int EXPERIMENT_PX = 40;  // the experiment in plain words (set-up page, progress page)
constexpr int CHOSEN_PX = 52;      // the chosen experiment on the first page, in large plain words
constexpr int START_PX = 72;       // START label (its second line: START_SUB_PX)
constexpr int START_SUB_PX = 40;
constexpr int STOP_PX = 110;       // STOP
constexpr int BAND_PX = 56;        // a band still needed: "24 mph  6 min"
constexpr int PLAN_NOTE_PX = 28;   // the reasons on the plan details page
constexpr int NAV_PX = 44;         // navigation, LOCK
constexpr int ACTION_PX = 56;      // other large controls (set-up page, pins, the switch)
constexpr int DETAIL_PX = 36;      // the detail line (not needed while driving)

// Largest number of characters of each line, tested for every state against the width of its box at its font size
constexpr int MAX_PRIMARY_CHARS = 22;
constexpr int MAX_SECOND_CHARS = 36;
constexpr int MAX_EXPERIMENT_CHARS = 80;
constexpr int MAX_CHOSEN_CHARS = 60;     // at CHOSEN_PX in the 2120 px line of the first page
constexpr int MAX_DETAIL_CHARS = 70;
constexpr int MAX_PROGRESS_CHARS = 56;   // a progress line at PROGRESS_PX in a 1660 px box
constexpr int PROGRESS_PX = 44;
constexpr int MAX_START_CHARS = 28;      // START label at START_PX
constexpr int MAX_START_SUB_CHARS = 56;  // its second line at START_SUB_PX

// ---- rectangles ----------------------------------------------------------------------------------------------------
struct Rect {
  int x, y, w, h;
  int right() const { return x + w; }
  int bottom() const { return y + h; }
};

inline bool overlaps(const Rect &a, const Rect &b) {
  return a.x < b.right() && b.x < a.right() && a.y < b.bottom() && b.y < a.bottom();
}

// The clear space between two rectangles (the larger of the horizontal and vertical gaps); negative when they overlap
inline int gapBetween(const Rect &a, const Rect &b) {
  const int gx = std::max(b.x - a.right(), a.x - b.right());
  const int gy = std::max(b.y - a.bottom(), a.y - b.bottom());
  return std::max(gx, gy);
}

enum class Page { Home, Setup, Tests, Overview, Ab, Progress, Cruise, Curve, Buttons };

// Control ids (also the ids the mouse handler works with)
enum Ctl {
  CTL_NONE = 0,
  CTL_BACK = 1,          // Home: leave to the driving screen (not while locked); the other pages: back to the first page
  CTL_ALLOFF = 2,        // red: ALL TESTS OFF (Tests, Overview) / A/B OFF (Ab)
  CTL_PREV = 3,
  CTL_NEXT = 4,
  CTL_LOCK = 5,
  CTL_OVERVIEW = 6,      // Tests: the run-all overview
  CTL_OVERVIEW_BACK = 7,
  CTL_AB_PAGE = 8,       // Tests: the plain A/B switch page
  CTL_AB_BACK = 9,
  CTL_TOGGLE = 10,       // + the test's index: the on/off box of the shown test (Overview: the run-all box)
  CTL_RESET = 20,        // + the test's index
  CTL_AB_SWITCH = 30,
  CTL_PIN_A = 31,
  CTL_PIN_B = 32,
  CTL_START = 40,
  CTL_STOP = 41,
  CTL_NAV_SETUP = 42,
  CTL_NAV_INDIV = 43,    // on the progress page: the individual tests
  CTL_NAV_PROGRESS = 44, // on the first page: the progress page
  CTL_SETUP_USE = 50,
  CTL_SETUP_PREV = 51,   // previous / next setting
  CTL_SETUP_NEXT = 52,
  CTL_SETUP_B_MINUS = 53,
  CTL_SETUP_B_PLUS = 54,
  CTL_SETUP_BLOCK_MINUS = 55,
  CTL_SETUP_BLOCK_PLUS = 56,
  CTL_SETUP_RESET = 57,  // RESET PROGRESS (asks first)
  CTL_NAV_CRUISE = 45,   // on the progress page: the cruise set-speed check's own page
  CTL_CRUISE_RUN = 60,   // the cruise check's page: RUN CHECK (one press arms one run)
  CTL_CRUISE_STOP = 61,  // ... STOP CHECK
  CTL_NAV_CURVE = 46,    // on the progress page: curve slowing's own page
  CTL_CURVE_SWITCH = 62, // curve slowing's page: TURN ON / TURN OFF (its label from the car controller's report)
  CTL_NAV_BUTTONS = 47,  // on the progress page: the cruise button tests' own page
  CTL_BUTTONS_HOLD = 63, // the button tests' page: HOLD TEST (one press arms one run)
  CTL_BUTTONS_TAPS = 64, // ... TAP TEST
  CTL_BUTTONS_STOP = 65, // ... STOP TEST
};

struct Control {
  int id;
  Rect r;
  const char *name;
};

constexpr int MAX_CONTROLS = 16;

inline int footerHeight(int error_lines) {
  return BOTTOM_STRIP + (error_lines > 0 ? 30 + 50 * std::min(2, error_lines) : 0);
}

inline int rowHeight(int error_lines) {
  return SCREEN_H - 20 - footerHeight(error_lines) - ROW_TOP;
}

// The text boxes (not controls) of the first page, from the same constants
constexpr Rect HOME_PRIMARY = {20, 10, 1720, 150};
constexpr Rect HOME_SECOND = {20, 160, 1720, 90};
constexpr Rect HOME_EXPERIMENT = {20, 250, 2120, 60};
constexpr Rect HOME_DETAIL = {480, 976, 1660, 44};     // line 1: what this drive has added
constexpr Rect HOME_DETAIL2 = {480, 1020, 1660, 44};   // line 2: why tests are held, or the alternation's numbers
constexpr Rect HOME_BADGE = {20, 990, 360, 70};
// The progress page's text boxes (not controls)
constexpr Rect PROG_TITLE = {400, 10, 1340, 180};
constexpr Rect PROG_AB_HEAD = {480, 210, 1660, 56};
constexpr Rect PROG_AB_BAR_A = {480, 276, 1660, 44};
constexpr Rect PROG_AB_BAR_B = {480, 330, 1660, 44};
constexpr Rect PROG_AB_LEFT = {480, 384, 1660, 50};
constexpr Rect PROG_SPLIT = {480, 990, 1660, 70};   // the expected split of the speed range
constexpr int PROG_ROWS_TOP = 470;
constexpr int PROG_ROW_H = 125;                       // four windows, each a label, a bar and one line
// The individual pages' title row and banner (text, not controls)
constexpr Rect TITLE_PREV_NEXT = {680, 10, 400, 180};     // between PREV and NEXT
constexpr Rect TITLE_WIDE = {340, 10, 1140, 180};         // between BACK and the red box
constexpr Rect TITLE_SETUP = {400, 10, 1340, 180};          // the set-up page
constexpr Rect BANNER_RECT = {1400, 280, 740, 180};       // beside the second row of controls, under the red box

// Every control of a page. error_lines: ignored test definitions shown at the foot of the individual pages (shortens the
// on/off box). Returns the number of controls written to out (at least MAX_CONTROLS long).
inline int layout(Page pg, int error_lines, Control *out) {
  int n = 0;
  auto add = [&](int id, int x, int y, int w, int h, const char *name) { out[n++] = Control{id, Rect{x, y, w, h}, name}; };
  const int top = ROW_TOP + 10;
  const int toggle_h = rowHeight(error_lines) - 30;
  switch (pg) {
    case Page::Home:
      add(CTL_LOCK, 1780, 10, 360, 180, "LOCK");
      add(CTL_BACK, 20, 330, 360, 180, "EXIT");
      add(CTL_NAV_SETUP, 20, 550, 360, 180, "PLAN DETAILS");
      add(CTL_NAV_PROGRESS, 20, 770, 360, 180, "PROGRESS");
      add(CTL_START, 480, 330, 1660, 272, "START");
      add(CTL_STOP, 480, 692, 1660, 272, "STOP");
      break;
    case Page::Setup:                         // the plan's details, read-only; RESET PROGRESS (asks first)
      add(CTL_BACK, 20, 10, 360, 180, "BACK");
      add(CTL_LOCK, 1780, 10, 360, 180, "LOCK");
      add(CTL_SETUP_RESET, 20, 330, 360, 180, "RESET PROGRESS");
      break;
    case Page::Progress:
      add(CTL_BACK, 20, 10, 360, 180, "< START PAGE");
      add(CTL_LOCK, 1780, 10, 360, 180, "LOCK");
      add(CTL_NAV_INDIV, 20, 230, 360, 170, "INDIVIDUAL TESTS");
      add(CTL_NAV_CRUISE, 20, 440, 360, 170, "CRUISE CHECK");
      add(CTL_NAV_CURVE, 20, 690, 360, 180, "CURVE SLOWING");
      add(CTL_NAV_BUTTONS, 20, 910, 360, 160, "BUTTON TESTS");
      break;
    case Page::Buttons:                       // the cruise button tests: their own page and controls, never START
      add(CTL_BACK, 20, 10, 360, 180, "< START PAGE");
      add(CTL_LOCK, 1780, 10, 360, 180, "LOCK");
      add(CTL_BUTTONS_HOLD, 20, 270, 660, 230, "HOLD TEST");
      add(CTL_BUTTONS_TAPS, 720, 270, 660, 230, "TAP TEST");
      add(CTL_BUTTONS_STOP, 1460, 270, 680, 230, "STOP TEST");
      break;
    case Page::Curve:                         // curve slowing: its own page and one control, never START
      add(CTL_BACK, 20, 10, 360, 180, "< START PAGE");
      add(CTL_LOCK, 1780, 10, 360, 180, "LOCK");
      add(CTL_CURVE_SWITCH, 20, 270, 2120, 230, "TURN ON / TURN OFF");
      break;
    case Page::Cruise:                        // the cruise set-speed check: its own page and its own two controls, never START
      add(CTL_BACK, 20, 10, 360, 180, "< START PAGE");
      add(CTL_LOCK, 1780, 10, 360, 180, "LOCK");
      add(CTL_CRUISE_RUN, 20, 270, 1020, 230, "RUN CHECK");
      add(CTL_CRUISE_STOP, 1120, 270, 1020, 230, "STOP CHECK");
      break;
    case Page::Tests:
      add(CTL_BACK, 20, 10, 300, 180, "< START PAGE");
      add(CTL_PREV, 380, 10, 280, 180, "< PREV");
      add(CTL_NEXT, 1100, 10, 280, 180, "NEXT >");
      add(CTL_ALLOFF, 1500, 10, 640, 180, "ALL TESTS OFF");
      add(CTL_LOCK, 20, 280, 360, 180, "LOCK");
      add(CTL_OVERVIEW, 440, 280, 420, 180, "RUN-ALL PAGE >");
      add(CTL_AB_PAGE, 920, 280, 420, 180, "A/B SWITCH >");
      add(CTL_TOGGLE, 30, top, 600, toggle_h, "ON / OFF");
      add(CTL_RESET, 1730, top, 400, 110, "Reset progress");
      break;
    case Page::Overview:
      add(CTL_BACK, 20, 10, 300, 180, "< START PAGE");
      add(CTL_ALLOFF, 1500, 10, 640, 180, "ALL TESTS OFF");
      add(CTL_LOCK, 20, 280, 360, 180, "LOCK");
      add(CTL_OVERVIEW_BACK, 440, 280, 420, 180, "< TEST PAGES");
      add(CTL_TOGGLE, 30, top, 600, toggle_h, "RUN ALL ON / OFF");
      break;
    case Page::Ab:
      add(CTL_BACK, 20, 10, 300, 180, "< START PAGE");
      add(CTL_ALLOFF, 1500, 10, 640, 180, "A/B OFF");
      add(CTL_LOCK, 20, 280, 360, 180, "LOCK");
      add(CTL_AB_BACK, 440, 280, 420, 180, "< TEST PAGES");
      add(CTL_AB_SWITCH, 20, top, 560, 200, "A/B ON / OFF");
      add(CTL_PIN_A, 640, top, 460, 200, "PIN A");
      add(CTL_PIN_B, 1160, top, 460, 200, "PIN B");
      break;
  }
  return n;
}

inline bool rectOf(Page pg, int id, int error_lines, Rect &r) {
  Control c[MAX_CONTROLS];
  const int n = layout(pg, error_lines, c);
  for (int i = 0; i < n; i++) {
    if (c[i].id == id) {
      r = c[i].r;
      return true;
    }
  }
  return false;
}

// The control under (x, y), 0 for none. Controls never overlap (tested), so the order does not matter.
inline int controlAt(Page pg, int error_lines, int x, int y) {
  Control c[MAX_CONTROLS];
  const int n = layout(pg, error_lines, c);
  for (int i = 0; i < n; i++) {
    if (x >= c[i].r.x && x < c[i].r.right() && y >= c[i].r.y && y < c[i].r.bottom()) {
      return c[i].id;
    }
  }
  return CTL_NONE;
}

// The PIN B pending-confirmation panel, the A/B status panel and the table of the plain A/B page
constexpr Rect AB_PANEL = {1680, ROW_TOP + 10, 460, 200};
constexpr int AB_TABLE_TOP = ROW_TOP + 10 + 200 + 40;

// ---- colours: one meaning each, the same on every page ----------------------------------------------------------------
enum class Tone { Stopped, Running, Waiting, Held, Done };

struct ToneColor {
  int r, g, b;
  bool light;   // black text on it
};

inline ToneColor toneColor(Tone t) {
  switch (t) {
    case Tone::Running: return {0x17, 0x86, 0x44, false};   // green
    case Tone::Waiting: return {218, 202, 37, true};         // amber
    case Tone::Held: return {0xE6, 0x7E, 0x22, true};        // orange
    case Tone::Done: return {0x46, 0x5B, 0xEA, false};       // blue
    case Tone::Stopped: break;
  }
  return {0x54, 0x54, 0x54, false};                          // grey
}

inline const char *toneName(Tone t) {
  switch (t) {
    case Tone::Running: return "running";
    case Tone::Waiting: return "waiting";
    case Tone::Held: return "held";
    case Tone::Done: return "done";
    case Tone::Stopped: break;
  }
  return "stopped";
}

// ---- what is switched on ----------------------------------------------------------------------------------------------
// Plan: the drive plan (TiPlanOn); AbPlain: the plain A/B switch (TiAbOn); All: the run-all mode; Single: a single test (or
// the dither) from the individual pages. Precedence in that order.
enum class Run { Stopped, Plan, AbPlain, All, Single };

constexpr double KM_PER_MILE = 1.609344;

inline std::string mmss(double secs) {
  const int s = secs > 0 ? (int)std::ceil(secs) : 0;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%d:%02d", s / 60, s % 60);
  return buf;
}

inline int speedValue(double mph, bool metric) {
  return (int)std::lround(metric ? mph * KM_PER_MILE : mph);
}

inline const char *speedUnit(bool metric) {
  return metric ? "KM/H" : "MPH";
}

struct HomeState {
  Run run = Run::Stopped;
  bool configured = false;                 // an alternation is set up (TiAbConfig names a setting)
  bool engaged = false;
  bool metric = false;
  bool have_speed = false;
  double mph = 0;
  const char *plan_mode = "";              // the car side's plan: off hold ab tests dead; "" when absent or stale
  const char *hold_why = "";               // TiTestHold reason while fresh and held, else ""
  bool ab_fresh = false;                   // TiAbStatus is recent
  const char *ab_state = "";               // running pinned complete ...
  const char *ab_arm = "";                 // A or B
  bool ab_block = false;                   // inside a block
  double ab_block_left_s = 0;
  bool ab_wait = false;                    // a switch is waiting for a gate
  const char *ab_wait_reason = "";
  double ab_wait_s = 0;
  double ab_wait_max_s = 0;
  bool ab_complete = false;                // 20 scored minutes per side
  bool target = false;                     // a window still needs something
  int target_window = -1;
  double target_secs = 0;
  bool tests_active = false;               // the car side counted a test in its last interval
  const char *gate_reason = "";            // command curve lat when the armed test's gate is closed
  // What the car controller itself reports (the only basis of "running"): the switches only say what was asked
  bool car_plan_on = false;                // TiAbStatus.plan.on: controls sees the plan switch on
  bool live_fresh = false;                 // the test accountant's live block (TiTestProgress._live) is recent
  bool live_all_on = false;                // ... and says run-all is on (the plan forces it): what the car controller is doing
  bool live_tests_on = false;              // ... and lists a test as on
  bool tests_optional[4] = {false, false, false, false};   // the plan marks a window's tests optional: not a band, never blocks DONE
  double tests_left_s[ti_logic::N_WINDOWS] = {0, 0, 0, 0};   // required test seconds still to collect per window (step tests)
  bool ab_stored_known = false;            // the persistent record is for the alternation set up now
  double ab_stored_a_s = 0;                // ... scored seconds per side, kept across ignition cycles
  double ab_stored_b_s = 0;
  double target_min = 20.0;                // scored minutes per side that complete the alternation
};

// True when the car side's own published state confirms what the switches ask for. A switch alone never counts: a stored ON
// while controls had restarted, or while its report has stopped arriving, is "on but not running".
inline bool carRunning(const HomeState &s) {
  switch (s.run) {
    case Run::Stopped:
      return false;
    case Run::Plan: {
      const std::string mode = s.plan_mode;
      if (mode == "dead") {
        return s.ab_fresh;                       // reporting its error
      }
      if (mode == "tests") {
        return s.live_fresh && s.live_all_on;    // the plan is running the tests: the accountant must say run-all is on
      }
      return (mode == "ab" || mode == "hold") && s.ab_fresh && s.car_plan_on;
    }
    case Run::AbPlain: {
      const std::string st = s.ab_state;
      return s.ab_fresh && !st.empty() && st != "off" && st != "error";
    }
    case Run::All:
      return s.live_fresh && s.live_all_on;
    case Run::Single:
      return s.live_fresh && s.live_tests_on;
  }
  return false;
}

struct Line {
  std::string text;
  Tone tone = Tone::Stopped;
};

struct HomeLines {
  Line primary;
  std::string second;
};

struct ShortText {
  const char *key;
  const char *text;
};

// Why a switch to the other arm waits, in a few words (the names of ti_ab.PRIORITY, ti_ab_logic.h REASONS)
constexpr int SPEED_GATE_LOW_MPH = 36;
constexpr int SPEED_GATE_HIGH_MPH = 69;
constexpr int EPS_LOW_MPH = 28;
constexpr int EPS_HIGH_MPH = 31;
inline std::string waitShort(const char *name, bool metric) {
  char buf[48];
  if (std::strcmp(name, "speed") == 0) {
    std::snprintf(buf, sizeof(buf), "WAITING: %d-%d %s", speedValue(SPEED_GATE_LOW_MPH, metric), speedValue(SPEED_GATE_HIGH_MPH, metric), speedUnit(metric));
    return buf;
  }
  if (std::strcmp(name, "eps") == 0) {
    std::snprintf(buf, sizeof(buf), "AVOID %d-%d %s", speedValue(EPS_LOW_MPH, metric), speedValue(EPS_HIGH_MPH, metric), speedUnit(metric));
    return buf;
  }
  static const ShortText TABLE[] = {
    {"input", "WAITING: CHECK INPUTS"}, {"touch", "WAITING: HANDS OFF"}, {"lane_change", "WAITING: LANE CHANGE"},
    {"holdoff", "WAITING: QUIET MOMENT"}, {"margin", "WAITING: STEADY SPEED"}, {"curve", "WAITING: STRAIGHT ROAD"}, {"lat", "WAITING: STRAIGHT ROAD"},
    {"request", "WAITING: EASE STEERING"}, {"saturated", "WAITING: EASE STEERING"}, {"settle", "WAITING: SETTLING"},
  };
  for (const ShortText &t : TABLE) {
    if (std::strcmp(t.key, name) == 0) {
      return t.text;
    }
  }
  return "WAITING";
}

// Why the tests are held (ti_ab_logic.h holdText / ti_plan.HOLD_WHYS), in a few words; false for a reason that is not one
struct HoldShort {
  const char *why;
  const char *text;
  Tone tone;
};
constexpr HoldShort HOLD_SHORT[] = {
  {"plan dead", "HELD: PLAN ERROR", Tone::Held},
  {"plan missing", "HELD: NO PLAN SIGNAL", Tone::Held},
  {"plan unreadable", "HELD: PLAN UNREADABLE", Tone::Held},
  {"plan stale", "HELD: NO CONTROLS DATA", Tone::Held},
  {"alternation not at A", "WAITING: RETURN TO A", Tone::Waiting},
  {"plan says ab", "WAITING: A/B RUNNING", Tone::Waiting},
  {"plan says hold", "WAITING: HAND-OVER", Tone::Waiting},
  {"plan says off", "WAITING: PLAN STARTING", Tone::Waiting},
  {"A/B switch on", "WAITING: A/B SWITCH ON", Tone::Waiting},
  {"control loop busy", "PAUSED: LOOP BUSY", Tone::Held},
};
constexpr int N_HOLD_SHORT = 10;

inline const HoldShort *holdShort(const char *why) {
  for (const HoldShort &h : HOLD_SHORT) {
    if (std::strcmp(h.why, why) == 0) {
      return &h;
    }
  }
  return nullptr;
}

inline Line line(const std::string &t, Tone tone) {
  Line l;
  l.text = t;
  l.tone = tone;
  return l;
}

inline std::string holdSpeed(int window, bool metric) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "HOLD %d %s", speedValue(ti_logic::windowTargetMph(window), metric), speedUnit(metric));
  return buf;
}

inline std::string nowSpeed(double mph, bool metric) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "Now %d %s", speedValue(mph, metric), metric ? "km/h" : "mph");
  return buf;
}

// The alternation's speed ask when the tests have nothing left to collect and the alternation is not complete: the middle of
// the nearer of its two scored windows (35-41 and 49-56 mph)
inline int altTargetWindow(double mph) {
  return mph < 45.5 ? 1 : 2;
}

// ---- progress as progress ---------------------------------------------------------------------------------------------
inline int wholeMin(double secs) {
  return secs > 0 ? (int)std::floor(secs / 60.0 + 1e-9) : 0;
}

inline double fraction(double have, double need) {
  return need <= 0 ? 1.0 : std::min(1.0, std::max(0.0, have / need));
}

// Tenths of a minute, never rounded up to a finished target: "13.96" reads 13.9
inline double tenthsMin(double secs) {
  return secs > 0 ? std::floor(secs / 6.0 + 1e-9) / 10.0 : 0.0;
}

// "A 14.2 / 20 min, B 13.9 / 20 min" (tenths: a whole drive's progress was invisible in whole minutes)
inline std::string abProgressText(double a_s, double b_s, double target_min) {
  char buf[64];
  const double cap = 60.0 * target_min;                 // scored time beyond the target is not shown as more than the target
  std::snprintf(buf, sizeof(buf), "A %.1f / %.0f min, B %.1f / %.0f min", tenthsMin(std::min(a_s, cap)), target_min, tenthsMin(std::min(b_s, cap)),
                target_min);
  return buf;
}

// The block in progress counts toward its side only when it ends: said so, with what it has scored so far. "" outside a block.
inline std::string blockSoFar(const char *arm, double block_scored_s) {
  if (arm == nullptr || arm[0] == 0 || block_scored_s < 0.0) {
    return "";
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "block %s +%.1f min pending", arm, tenthsMin(block_scored_s));
  return buf;
}

// Driving time still needed at the alternation's speeds: the scored time still missing on both sides (the sides alternate)
inline double abMinutesLeft(double a_s, double b_s, double target_min) {
  const double t = 60.0 * target_min;
  return (std::max(0.0, t - a_s) + std::max(0.0, t - b_s)) / 60.0;
}

inline std::string abLeftText(double a_s, double b_s, double target_min, double overhead = 1.0, const std::string &block = "") {
  const double left = abMinutesLeft(a_s, b_s, target_min) * overhead;
  if (left <= 0.0) {
    return "A/B complete: the tests take over";
  }
  char buf[128];
  std::snprintf(buf, sizeof(buf), "About %d min at %d-%d mph left%s%s", (int)std::ceil(left), (int)ti_ab::PLAN_ALT_MIN_MPH + 1,
                (int)ti_ab::PLAN_ALT_MAX_MPH - 1, block.empty() ? "" : "; ", block.c_str());
  return buf;
}

// "A/B 14 of 20 min each side done": the second line of the stopped state
inline std::string stoppedSecond(bool known, double a_s, double b_s, double target_min) {
  if (!known || (a_s <= 0.0 && b_s <= 0.0)) {
    return "Nothing is running";
  }
  char buf[64];
  if (std::min(a_s, b_s) >= 60.0 * target_min) {
    std::snprintf(buf, sizeof(buf), "A/B complete: %.0f min each side", target_min);
    return buf;
  }
  std::snprintf(buf, sizeof(buf), "A/B %.1f of %.0f min each side done", tenthsMin(std::min(a_s, b_s)), target_min);
  return buf;
}

// One window of the tests: "21-27 mph" and "3 of 12 done; hold 24 mph for about 9 min" (done, total: required items)
inline std::string windowLabel(int w, bool metric) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%d-%d %s", speedValue(ti_logic::WINDOWS[w].low, metric), speedValue(ti_logic::WINDOWS[w].high, metric),
                metric ? "km/h" : "mph");
  return buf;
}

inline std::string windowRowText(int w, int done, int total, double secs_left, bool metric, bool optional = false) {
  char buf[96];
  if (total <= 0) {
    return "no test needs this window";
  }
  if (optional) {                                   // never a band to complete: only said how far it has come
    std::snprintf(buf, sizeof(buf), "%d of %d done (optional, counts if it happens)", done, total);
    return buf;
  }
  if (done >= total) {
    std::snprintf(buf, sizeof(buf), "%d of %d done", done, total);
    return buf;
  }
  std::snprintf(buf, sizeof(buf), "%d of %d done; hold %d %s for about %d min", done, total, speedValue(ti_logic::windowTargetMph(w), metric),
                metric ? "km/h" : "mph", std::max(1, (int)std::ceil(secs_left / 60.0)));
  return buf;
}

// What the tests and the alternation each get of the speed range (the plan gives 36-63 mph to the alternation until it is done)
inline std::string splitNote(bool alternation_open, bool metric) {
  char buf[128];
  if (!alternation_open) {
    return "The alternation is not running: the tests run in every window";
  }
  std::snprintf(buf, sizeof(buf), "A/B has %d-%d %s until done; tests run below %d and above %d %s", speedValue(ti_ab::PLAN_ALT_MIN_MPH + 1, metric),
                speedValue(ti_ab::PLAN_ALT_MAX_MPH - 1, metric), metric ? "km/h" : "mph", speedValue(ti_ab::PLAN_ALT_MIN_MPH, metric),
                speedValue(ti_ab::PLAN_ALT_MAX_MPH, metric), metric ? "km/h" : "mph");
  return buf;
}

// What the drive has added since START was last pressed: tests, seconds credited per window; the alternation, minutes per side.
// win_s: credited seconds per window since START; a_s, b_s: scored seconds per side since START; has_ab: an alternation is set up.
inline std::string thisDriveLine(const double win_s[ti_logic::N_WINDOWS], double a_s, double b_s, bool has_ab, bool metric) {
  std::string tests;
  double total = 0;
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    if (win_s[w] >= 1.0) {
      char buf[48];
      std::snprintf(buf, sizeof(buf), "%s%s +%d s", tests.empty() ? "" : ", ", windowLabel(w, metric).substr(0, windowLabel(w, metric).find(' ')).c_str(), (int)win_s[w]);
      tests += buf;
      total += win_s[w];
    }
  }
  char ab[48] = "";
  if (has_ab) {
    std::snprintf(ab, sizeof(ab), "A/B +%.1f / +%.1f min", a_s / 60.0, b_s / 60.0);
  }
  std::string out = "Since START: ";
  if (tests.empty() && (!has_ab || (a_s < 6.0 && b_s < 6.0))) {
    return out + "nothing counted yet";
  }
  std::string full = out + (tests.empty() ? std::string("tests +0 s") : "tests " + tests) + (has_ab ? std::string("; ") + ab : std::string());
  if ((int)full.size() <= MAX_DETAIL_CHARS) {
    return full;
  }
  char t[32];
  std::snprintf(t, sizeof(t), "tests +%s", mmss(total).c_str());
  return out + t + (has_ab ? std::string("; ") + ab : std::string());
}

// ---- the bands still needed, ONE model for the list and the status line ------------------------------------------------------------
// A band is a range of speeds with the engaged minutes still needed there. What the tests and the alternation each need is merged into
// one figure per band. While the alternation is open the range 35-64 mph is ONE band (alternation blocks and the tests' 35-41 and 49-56
// windows interleave there, ti_plan.tests_behind); once it is complete the bands are the test windows themselves (38 and 52 mph) and
// the gap between them counts nothing. Both the list and the primary line are computed from these bands and from what the car reports
// is counting, so they cannot disagree. Overheads (fades, hand-overs, wash-in, voided time, gate waits) are estimates, named here.
// MEASURED by replaying the two reference drives through the build (accept harness, ti-acceptance branch; not yet by a drive on the car):
// alternation 1.16-1.33 engaged minutes per scored minute; tests 1.06-1.75, about 1.4 outside the lowest window (where the gates close most of
// the time: the released drives 33b/33c show the strict gates open about 29% of the time at 21-27 mph).
constexpr double OVERHEAD_ALT = 1.8;         // MEASURED on the car (drives of 2026-11-22): 53-58 % of engaged A/B-mode time was scored (blocks voided by
                                             // disengaging, wash-in, waits), so about 1.8 engaged min per scored min (the replay's 1.16-1.33 is on idealised gates)
constexpr double OVERHEAD_TESTS = 1.4;       // the three higher windows
constexpr double OVERHEAD_TESTS_LOW = 1.75;  // the lowest window (21-27 mph): the top of the replay's range
constexpr int MAX_BANDS = 4;

struct Band {
  int target_mph = 0;          // the one speed to hold
  int lo = 0, hi = 0;          // where time counts: lo <= mph < hi
  int show_lo = 0, show_hi = 0;// the range in small type
  double minutes = 0;          // engaged minutes still needed (rounded up)
  bool done = false;
};

inline bool altOpen(const HomeState &s) {
  return s.configured && !s.ab_complete && abMinutesLeft(s.ab_stored_a_s, s.ab_stored_b_s, s.target_min) > 0.0;
}

// The bands for the state; returns how many (3 while the alternation is open, else 4)
inline int bandsFromState(const HomeState &s, Band out[MAX_BANDS]) {
  double t[ti_logic::N_WINDOWS];
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    t[w] = s.tests_left_s[w] * (w == 0 ? OVERHEAD_TESTS_LOW : OVERHEAD_TESTS) / 60.0;
  }
  int n = 0;
  auto win = [&](int w, int target, double minutes) {
    out[n++] = Band{target, (int)ti_logic::WINDOWS[w].low, (int)ti_logic::WINDOWS[w].high, (int)ti_logic::WINDOWS[w].low, (int)ti_logic::WINDOWS[w].high, minutes, false};
  };
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    if (s.tests_optional[w]) {
      t[w] = 0.0;                                 // optional tests take no part in the plan's minutes; the progress page shows them
    }
  }
  if (!s.tests_optional[0]) {
    win(0, ti_logic::windowTargetMph(0), t[0]);
  }
  if (altOpen(s)) {
    const double alt = abMinutesLeft(s.ab_stored_a_s, s.ab_stored_b_s, s.target_min) * OVERHEAD_ALT;
    out[n++] = Band{45, (int)ti_ab::PLAN_ALT_MIN_MPH, (int)ti_ab::PLAN_ALT_MAX_MPH, (int)ti_ab::PLAN_ALT_MIN_MPH + 1, (int)ti_ab::PLAN_ALT_MAX_MPH - 1,
                    alt + t[1] + t[2], false};
  } else {
    if (!s.tests_optional[1]) {
      win(1, ti_logic::windowTargetMph(1), t[1]);
    }
    if (!s.tests_optional[2]) {
      win(2, ti_logic::windowTargetMph(2), t[2]);
    }
  }
  if (!s.tests_optional[3]) {
    win(3, ti_logic::windowTargetMph(3), t[3]);
  }
  for (int i = 0; i < n; i++) {
    out[i].minutes = std::ceil(out[i].minutes - 1e-9);
    out[i].done = out[i].minutes <= 0.0;
  }
  return n;
}

inline double bandsTotalMinutes(const Band *b, int n) {
  double sum = 0;
  for (int i = 0; i < n; i++) {
    sum += b[i].minutes;
  }
  return sum;
}

// The plan has nothing left to collect (the alternation is set up and its minutes are done; the tests that are optional never count)
inline bool planComplete(const HomeState &s) {
  if (!s.configured) {
    return false;
  }
  Band bands[MAX_BANDS];
  const int n = bandsFromState(s, bands);
  return bandsTotalMinutes(bands, n) <= 0.0;
}

// The plan is running and its own report says it stopped after an error
inline bool planDead(const HomeState &s) {
  return s.run == Run::Plan && std::strcmp(s.plan_mode, "dead") == 0;
}

// A mode that is not the plan is on (a single test, the plain A/B switch, run-all beside an alternation): START would replace it
inline bool otherModeOn(const HomeState &s) {
  return s.run == Run::Single || s.run == Run::AbPlain || (s.run == Run::All && s.configured);
}

// The first page's line that says what is on when it is not the plan
inline std::string otherModeLine(const HomeState &s) {
  switch (s.run) {
    case Run::Single: return "A SINGLE TEST IS ON, NOT THE PLAN";
    case Run::AbPlain: return "THE A/B SWITCH IS ON, NOT THE PLAN";
    case Run::All: return s.configured ? "RUN-ALL IS ON, NOT THE PLAN" : "";
    default: return "";
  }
}

// What the middle of the first page says when the plan is complete and still running: plainly, nothing more is needed
inline std::vector<std::string> completeLines(const HomeState &s) {
  bool any_optional = false;
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    any_optional = any_optional || s.tests_optional[w];
  }
  char ab[64];
  std::snprintf(ab, sizeof(ab), "A/B: %.0f min on each side, done", s.target_min);
  return {"PLAN COMPLETE", "Nothing more is needed", ab,
          any_optional ? "Optional tests may continue: they count if they happen" : "More data still helps, but you can stop"};
}

// The still-needed band that contains `mph` (time counts there), else -1
inline int bandIndexAt(const Band *b, int n, double mph) {
  for (int i = 0; i < n; i++) {
    if (!b[i].done && mph >= b[i].lo && mph < b[i].hi) {
      return i;
    }
  }
  return -1;
}

// The band in which time is COUNTING right now, from what the car reports (a running alternation block, or a test counted in its
// last interval), else -1. Never from the speed alone.
inline int countingBand(const HomeState &s, const Band *b, int n) {
  if (!s.have_speed || !carRunning(s)) {
    return -1;
  }
  const std::string mode = s.plan_mode;
  const bool ab_side = ((s.run == Run::Plan && mode == "ab") || s.run == Run::AbPlain) && s.ab_fresh && s.ab_block && altOpen(s);
  const bool tests_side = s.tests_active && ((s.run == Run::Plan && mode == "tests") || s.run == Run::All || s.run == Run::Single);
  return (ab_side || tests_side) ? bandIndexAt(b, n, s.mph) : -1;
}

// "24 mph  6 min" / "24 mph  done"
inline std::string bandText(const Band &b, bool metric) {
  char buf[64];
  if (b.done) {
    std::snprintf(buf, sizeof(buf), "%d %s  done", speedValue(b.target_mph, metric), metric ? "km/h" : "mph");
  } else {
    std::snprintf(buf, sizeof(buf), "%d %s  %d min", speedValue(b.target_mph, metric), metric ? "km/h" : "mph", (int)b.minutes);
  }
  return buf;
}

inline std::string bandRange(const Band &b, bool metric) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%d-%d %s", speedValue(b.show_lo, metric), speedValue(b.show_hi, metric), metric ? "km/h" : "mph");
  return buf;
}

// Where nothing counts: the nearest still-needed bands below and above, "Hold 24 or 38 mph"
inline std::string nearestBands(const Band *b, int n, double mph, bool metric) {
  int lower = -1, upper = -1;
  for (int i = 0; i < n; i++) {
    if (b[i].done) {
      continue;
    }
    if (b[i].hi <= mph) {
      lower = i;
    } else if (b[i].lo > mph && upper < 0) {
      upper = i;
    }
  }
  char buf[64];
  const char *unit = metric ? "km/h" : "mph";
  if (lower >= 0 && upper >= 0) {
    std::snprintf(buf, sizeof(buf), "Hold %d or %d %s", speedValue(b[lower].target_mph, metric), speedValue(b[upper].target_mph, metric), unit);
  } else if (lower >= 0 || upper >= 0) {
    std::snprintf(buf, sizeof(buf), "Hold %d %s", speedValue(b[lower >= 0 ? lower : upper].target_mph, metric), unit);
  } else {
    return "Nothing left to collect";
  }
  return buf;
}

// The stopped page: what the whole plan needs of one drive, in plain words: "Plan needs about 12 min at 36-55 mph" or, with more
// than one band, "Plan needs about 20 min: 12 at 36-55, 8 at 64-80 mph". A finished plan says so (no minutes, no colon).
inline std::string budgetLine(const Band *b, int n, bool metric) {
  const int total = (int)bandsTotalMinutes(b, n);
  if (total <= 0) {
    return "Plan complete: nothing more is needed";
  }
  const char *unit = metric ? " km/h" : " mph";
  auto range = [&](const Band &x) { return std::to_string(speedValue(x.show_lo, metric)) + "-" + std::to_string(speedValue(x.show_hi, metric)) + unit; };
  int open = 0, last = -1;
  for (int i = 0; i < n; i++) {
    if ((int)b[i].minutes > 0) {
      open++;
      last = i;
    }
  }
  if (open == 1) {
    return "Plan needs about " + std::to_string(total) + " min at " + range(b[last]);
  }
  std::string out = "Plan needs about " + std::to_string(total) + " min: ";
  bool first = true;
  for (int i = 0; i < n; i++) {
    if ((int)b[i].minutes > 0) {
      out += std::string(first ? "" : ", ") + std::to_string((int)b[i].minutes) + " at " + std::to_string(speedValue(b[i].show_lo, metric)) + "-" +
             std::to_string(speedValue(b[i].show_hi, metric));
      first = false;
    }
  }
  return out + unit;
}

// The primary line and the second line of the first page.
inline HomeLines home(const HomeState &s) {
  HomeLines h;
  if (s.run == Run::Stopped) {
    h.primary = line(planComplete(s) ? "STOPPED: PLAN COMPLETE" : "STOPPED: TAP START", Tone::Stopped);   // off after every stop and restart, said plainly; a finished plan is not asked to start
    h.second = s.configured ? stoppedSecond(s.ab_stored_known, s.ab_stored_a_s, s.ab_stored_b_s, s.target_min) : std::string("Nothing is running");
    return h;
  }
  if (!carRunning(s)) {
    // asked for, not confirmed by the car side (restarted, silent, or never started): said plainly, never "running"
    h.primary = line("ON BUT NOT RUNNING", Tone::Held);
    h.second = "No report from the car: tap START";
    return h;
  }
  if (!s.engaged) {
    h.primary = line("WAITING: ENGAGE", Tone::Waiting);
    h.second = "Engage the comma to begin";
    return h;
  }
  const bool plan = s.run == Run::Plan;
  const HoldShort *hold = s.hold_why[0] != 0 ? holdShort(s.hold_why) : nullptr;
  if (plan && std::strcmp(s.plan_mode, "dead") == 0) {
    h.primary = line("HELD: PLAN ERROR", Tone::Held);
    h.second = "Tap STOP, then START";
    return h;
  }
  if (hold != nullptr && hold->tone == Tone::Held) {
    h.primary = line(hold->text, Tone::Held);
    h.second = "Tests wait for the car";
    return h;
  }
  if (s.run == Run::Single) {
    // a single test from the individual pages: not the plan, and its time is not the plan's bands
    h.primary = s.tests_active ? line("SINGLE TEST: RUNNING", Tone::Running) : line("SINGLE TEST: WAITING", Tone::Waiting);
    h.second = "Not the plan: START replaces it";
    return h;
  }
  if (plan && std::strcmp(s.plan_mode, "hold") == 0) {
    h.primary = line("WAITING: HAND-OVER", Tone::Waiting);
    h.second = "Switching A/B and tests";
    return h;
  }
  const bool ab_side = (plan && std::strcmp(s.plan_mode, "ab") == 0) || s.run == Run::AbPlain;
  if (ab_side) {
    if (!s.ab_fresh) {
      h.primary = line("WAITING: NO A/B DATA", Tone::Waiting);
      h.second = "Waiting for the car";
    } else if (s.ab_complete || std::strcmp(s.ab_state, "complete") == 0) {
      h.primary = line("DONE: A/B COMPLETE", Tone::Done);
      h.second = "Tests take over";
    } else if (s.ab_block) {
      const bool pinned = std::strcmp(s.ab_state, "pinned") == 0;
      h.primary = line(std::string(pinned ? "A/B PINNED: " : "A/B RUNNING: ") + (s.ab_arm[0] != 0 ? s.ab_arm : "?"), Tone::Running);
      h.second = pinned ? std::string("No alternation") : mmss(s.ab_block_left_s) + " left in block";
    } else if (s.ab_wait) {
      h.primary = line(waitShort(s.ab_wait_reason, s.metric), Tone::Waiting);
      char buf[48];
      std::snprintf(buf, sizeof(buf), "%d s of %d s", (int)s.ab_wait_s, (int)s.ab_wait_max_s);
      h.second = buf;
    } else {
      h.primary = line("WAITING: NEXT BLOCK", Tone::Waiting);
      h.second = "Starting the next block";
    }
    return h;
  }
  if (plan && std::strcmp(s.plan_mode, "tests") != 0) {
    h.primary = line("WAITING: PLAN STARTING", Tone::Waiting);
    h.second = "Waiting for the car";
    return h;
  }
  if (hold != nullptr) {
    h.primary = line(hold->text, hold->tone);
    h.second = "Tests resume by themselves";
    return h;
  }
  if (!s.have_speed) {
    h.primary = line("WAITING: NO SPEED", Tone::Waiting);
    h.second = "Waiting for the car";
    return h;
  }
  Band bands[MAX_BANDS];
  const int n = bandsFromState(s, bands);
  if (bandsTotalMinutes(bands, n) <= 0.0) {
    bool any_optional = false;
    for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
      any_optional = any_optional || s.tests_optional[w];
    }
    h.primary = line(plan && s.configured ? "DONE: ALL COMPLETE" : "DONE: TESTS COMPLETE", Tone::Done);
    h.second = any_optional ? "Optional tests continue" : "Extra data still helps";
    return h;
  }
  const int here = bandIndexAt(bands, n, s.mph);
  const ti_logic::Zone zone_here = ti_logic::zoneFor(s.mph);
  if (here < 0 && zone_here.state == ti_logic::ZoneState::Inside && s.tests_optional[zone_here.window] && s.tests_active) {
    h.primary = line("OPTIONAL: TESTS COUNT", Tone::Running);      // credited if they run, but not a band he has to complete
    h.second = "Not required: bonus data";
    return h;
  }
  if (here < 0) {
    // no band that still needs time contains this speed: nothing counts here, and the list shows the bands that do
    char buf[48];
    std::snprintf(buf, sizeof(buf), "NO COUNT AT %d %s", speedValue(s.mph, s.metric), speedUnit(s.metric));
    h.primary = line(buf, Tone::Waiting);
    h.second = nearestBands(bands, n, s.mph, s.metric);
    return h;
  }
  const Band &bd = bands[here];
  char hold_text[48];
  std::snprintf(hold_text, sizeof(hold_text), "HOLD %d %s", speedValue(bd.target_mph, s.metric), speedUnit(s.metric));
  const std::string left = std::to_string((int)bd.minutes) + " min left here";
  if (countingBand(s, bands, n) == here) {
    h.primary = line(std::string("TESTING: ") + hold_text, Tone::Running);
    h.second = left;
  } else {
    std::string t = std::string("WAITING: ") + hold_text;
    if (altOpen(s) && bd.lo == (int)ti_ab::PLAN_ALT_MIN_MPH) {
      t = "WAITING: A/B STARTING";             // the alternation's band: the plan is handing over to it
    } else if (std::strcmp(s.gate_reason, "curve") == 0 || std::strcmp(s.gate_reason, "lat") == 0) {
      t = "WAITING: STRAIGHT ROAD";
    } else if (std::strcmp(s.gate_reason, "command") == 0) {
      t = "WAITING: EASE STEERING";
    }
    h.primary = line(t, Tone::Waiting);
    h.second = left;
  }
  return h;
}

// ---- the experiment in plain words, START, the notes ---------------------------------------------------------------
inline const char *plainName(const char *setting) {
  static const ShortText TABLE[] = {
    {"SteerKP", "steering gain"}, {"LatCommitSetpoint", "commit setpoint"}, {"LatDamping", "damping"},
    {"LatFrictionComp", "friction compensation"}, {"LatOutputFilter", "output filter"}, {"LatNoFrictionRelay", "friction relay"},
  };
  for (const ShortText &t : TABLE) {
    if (std::strcmp(t.key, setting) == 0) {
      return t.text;
    }
  }
  return setting;
}

// LatNoFrictionRelay holds "no relay": its value true means the relay is OFF (ti_ab.py SETTINGS), so the page says it the way round
// the driver means it
inline bool invertedWords(const ti_ab::Setting &s) {
  return std::strcmp(s.name, "LatNoFrictionRelay") == 0;
}

inline std::string valueWord(const ti_ab::Setting &s, double v) {
  if (s.is_bool) {
    return ((v != 0.0) != invertedWords(s)) ? "on" : "off";
  }
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.2f", v);
  return buf;
}

// The order the set-up page offers the settings in: on/off settings first (the reviewed first-trial order), the number last
constexpr int N_SETUP_ORDER = 6;
constexpr const char *SETUP_ORDER[N_SETUP_ORDER] = {"LatNoFrictionRelay", "LatFrictionComp", "LatDamping", "LatOutputFilter",
                                                    "LatCommitSetpoint", "SteerKP"};

inline int setupPos(const char *name) {
  for (int i = 0; i < N_SETUP_ORDER; i++) {
    if (std::strcmp(SETUP_ORDER[i], name) == 0) {
      return i;
    }
  }
  return 0;
}

// The setting after / before `index` (an index of ti_ab::SETTINGS) in the order of the set-up page, wrapping
inline int setupStep(int index, int dir) {
  const int pos = ((setupPos(ti_ab::SETTINGS[index].name) + (dir > 0 ? 1 : -1)) % N_SETUP_ORDER + N_SETUP_ORDER) % N_SETUP_ORDER;
  return ti_ab::settingIndex(SETUP_ORDER[pos]);
}

// A soft guard: SteerKP is offered but marked (its effect is smaller than the drive-to-drive variation)
inline bool notRecommended(const char *setting) {
  return std::strcmp(setting, "SteerKP") == 0;
}

constexpr const char *NOT_RECOMMENDED_TEXT = "Not recommended yet: effect smaller than drive-to-drive variation";

// "A/B: friction relay, yours (off) against on"; setting < 0: no alternation is set up
inline std::string experimentLine(int setting, bool a_known, double a, double b) {
  if (setting < 0 || setting >= ti_ab::N_SETTINGS) {
    return "A/B: not used (choose one on SET UP A/B)";
  }
  const ti_ab::Setting &s = ti_ab::SETTINGS[setting];
  if (!a_known) {
    return std::string("A/B: ") + plainName(s.name) + ", your value cannot be read";
  }
  return std::string("A/B: ") + plainName(s.name) + ", yours (" + valueWord(s, a) + ") against " + valueWord(s, b);
}

struct StartLook {
  std::string label;   // START: tests + A/B relay
  std::string sub;     // a second line, empty when there is nothing to add
  bool enabled = true; // a tap does something
  Tone tone = Tone::Done;
};

// What the primary control says and whether a tap does anything. Running already: it says RUNNING and a tap does nothing.
inline StartLook startLook(bool configured, Run run, bool confirmed = true, bool plan_ok = true, bool tests_optional = false,
                           bool plan_complete = false, bool plan_dead = false) {
  StartLook k;
  if (!plan_ok) {                               // nothing runs from an invalid plan
    k.label = "PLAN INVALID";
    k.sub = "See PLAN DETAILS";
    k.enabled = false;
    k.tone = Tone::Held;
    return k;
  }
  if (plan_dead) {                              // the plan stopped itself: it is not running, whatever the switch says
    k.label = "PLAN STOPPED: ERROR";
    k.sub = "Tap STOP, then START";
    k.enabled = false;
    k.tone = Tone::Held;
    return k;
  }
  const std::string what = configured ? (tests_optional ? "A/B relay" : "tests + A/B relay") : "tests";
  if (plan_complete) {                          // a finished plan is not started again
    k.label = "PLAN COMPLETE";
    k.sub = "Nothing more is needed";
    switch (run) {
      case Run::Single: k.sub += "; a single test is on"; break;
      case Run::AbPlain: k.sub += "; the A/B switch is on"; break;
      case Run::All: k.sub += "; run-all is on"; break;
      default: break;
    }
    k.enabled = false;
    k.tone = Tone::Done;
    return k;
  }
  if (!confirmed && run != Run::Stopped) {      // the switch says on, the car is not running it: START starts it cleanly
    k.label = "START: " + what;
    k.sub = "The switch shows on but nothing is running";
    k.enabled = true;
    k.tone = Tone::Done;
    return k;
  }
  const bool same = run == Run::Plan || (run == Run::All && !configured);
  if (same) {
    k.label = "RUNNING: " + what;
    k.sub = "Tap STOP to stop";
    k.enabled = false;
    k.tone = Tone::Running;
    return k;
  }
  k.label = "START: " + what;
  k.enabled = true;
  k.tone = Tone::Done;
  switch (run) {
    case Run::Single: k.sub = "Replaces the single test now on"; break;
    case Run::AbPlain: k.sub = "Replaces the A/B switch now on"; break;
    case Run::All: k.sub = "Replaces run-all: adds the A/B relay"; break;
    default: break;
  }
  return k;
}

// A note on the individual-tests pages: starting a single test there ends what START began
inline std::string individualNote(Run run) {
  if (run == Run::Plan || run == Run::All) {
    return "Diagnostic page. A single test here replaces START.";
  }
  return "Diagnostic page. Run everything from the START page.";
}

inline std::string detailScored(double a_s, double b_s, double target_min, bool complete, const char *block_arm = "", double block_s = -1.0) {
  char buf[128];
  const double cap = 60.0 * target_min;                 // never "13 of 10"
  char blk[40] = "";
  if (!complete && block_arm != nullptr && block_arm[0] != 0 && block_s >= 0.0) {
    std::snprintf(blk, sizeof(blk), "; block %s +%.1f pending", block_arm, tenthsMin(block_s));
  }
  std::snprintf(buf, sizeof(buf), "Scored: A %.1f / B %.1f of %.0f min%s%s", tenthsMin(std::min(a_s, cap)), tenthsMin(std::min(b_s, cap)), target_min,
                complete ? " COMPLETE" : "", blk);
  return buf;
}

// ---- the plan START runs (selfdrive/car/mazda/ti_drive_plan.json) ----------------------------------------------------------------
// The file names what runs; the code that runs it has fixed limits, so a plan is accepted only when it names exactly those. The same
// rules as ti_drive_plan.py (test_mazda_ti_pages_logic.py compares the reasons). The driver never picks an experiment on the screen.
struct PlanFields {
  bool named = false;
  bool reasons_ok = false;               // every amount carries its reason, minimum and buffer
  bool optional_win[4] = {false, false, false, false};   // per window: its tests are optional (credited when they run, never a band, never block DONE)
  bool optional_bad = false;             // an `optional` that is not true or false
  std::vector<std::string> test_ids, windows;
  bool has_alt = false;
  std::string setting, a, b_text;       // b_text: "other", or "number" when b is a number
  bool b_is_number = false;
  double b = 0;
  double block_min = 0, target_min = 0, min_mph = 0, max_mph = 0;
};

inline std::string planProblem(const PlanFields &p) {
  if (!p.named) {
    return "no plan name";
  }
  if (p.test_ids != std::vector<std::string>{"ti_sweep", "ti_pulses", "ti_ladder"}) {
    return "tests must be sweep, pulses, ladder";
  }
  if (p.windows != std::vector<std::string>{"21-27", "35-41", "49-56", "64-80"}) {
    return "tests must cover all four windows";
  }
  if (p.optional_bad) {
    return "optional must be true or false";
  }
  if (!p.has_alt) {
    return p.reasons_ok ? "" : "an amount has no reason";
  }
  const int idx = ti_ab::settingIndex(p.setting.c_str());
  if (idx < 0) {
    return "setting is not allowed";
  }
  if (p.a != "stored") {
    return "A must be your stored value";
  }
  const ti_ab::Setting &st = ti_ab::SETTINGS[idx];
  if (st.is_bool) {
    if (p.b_is_number || p.b_text != "other") {
      return "B must be the other value";
    }
  } else if (!p.b_is_number || p.b < st.lo - 1e-9 || p.b > st.hi + 1e-9) {
    return "B is outside 0.70-1.00";
  }
  if (p.block_min < ti_ab::MIN_BLOCK_MIN || p.block_min > ti_ab::MAX_BLOCK_MIN) {
    return "block length outside 4-10 min";
  }
  if (p.target_min != ti_ab::PLAN_TARGET_MIN_DEFAULT) {
    return "target must be 10 min per side";
  }
  if (p.min_mph != ti_ab::PLAN_ALT_MIN_MPH || p.max_mph != ti_ab::PLAN_ALT_MAX_MPH) {
    return "speed range must be 35-64 mph";
  }
  return p.reasons_ok ? "" : "an amount has no reason";
}

// The plan in large plain words, or why it cannot run. has_alt with its setting: "PLAN: tests + friction relay A/B"
inline std::string planLine(bool ok, const std::string &problem, bool has_alt, const char *setting, bool tests_required = true) {
  if (!ok) {
    return "PLAN INVALID: " + problem;
  }
  if (!has_alt) {
    return "PLAN: tests";
  }
  return std::string(tests_required ? "PLAN: tests + " : "PLAN: ") + plainName(setting) + " A/B";
}

// ---- the set-up page ------------------------------------------------------------------------------------------------
// The alternation is configured with the car stopped and nothing running (it is stored as TiAbConfig, which persists; the
// switches are tmpfs and are off after every boot)
constexpr double SETUP_MAX_MPH = 3.0;

inline bool setupEditable(bool running, bool have_speed, double mph) {
  return !running && (!have_speed || mph < SETUP_MAX_MPH);
}

// "set-up needs the car stopped and nothing running" in the words of the page
inline std::string setupNote(bool running, bool have_speed, double mph) {
  if (running) {
    return "Tap STOP on the START page to change the set-up";
  }
  if (!setupEditable(running, have_speed, mph)) {
    return "Set up while parked";
  }
  return "Saved as you change it. It stays after a restart; A/B starts off.";
}

// ---- what START and STOP write ----------------------------------------------------------------------------------------
// Ordered operations, executed by TiTests::runOps (sidebar.cc). Keys of the "memory" store are tmpfs params. These sequences
// ARE what RUN ALL (TiTests::startAll), the DRIVE PLAN box (abAct control 39), ALL TESTS OFF (TiTests::allOff) and A/B OFF
// (abSetOn(false)) wrote at d20104c; the test compares them with a transcription of those.
enum class OpKind {
  MemBool,           // tmpfs param key = value
  MemRemove,         // tmpfs param key removed
  AllowedOff,        // every allow-listed test key to its off value (flash params), and the views' mismatch counters reset
  RunAllOn,          // the run-all key (TiIdentAll) to its on value
  SingleStepTestsOff,// the single step tests that are on, off
  SaveAbConfig,      // the page's alternation set-up written to TiAbConfig
  UpdateToggles,     // updateFrogPilotToggles()
};

struct Op {
  OpKind kind;
  const char *key;
  bool value;
  bool operator==(const Op &o) const {
    return kind == o.kind && value == o.value && ((key == nullptr && o.key == nullptr) || (key != nullptr && o.key != nullptr && std::strcmp(key, o.key) == 0));
  }
};

inline Op memBool(const char *k, bool v) { return Op{OpKind::MemBool, k, v}; }
inline Op memRemove(const char *k) { return Op{OpKind::MemRemove, k, false}; }
inline Op simple(OpKind k) { return Op{k, nullptr, false}; }

constexpr const char *KEY_PLAN_ON = "TiPlanOn";
constexpr const char *KEY_AB_ON = "TiAbOn";
constexpr const char *KEY_AB_PIN = "TiAbPin";

// ALL TESTS OFF: the plan off, every allow-listed key off
inline std::vector<Op> allTestsOffOps() {
  return {memBool(KEY_PLAN_ON, false), simple(OpKind::AllowedOff), simple(OpKind::UpdateToggles)};
}

// RUN ALL: the run-all mode on, one injection at a time
inline std::vector<Op> runAllOps() {
  return {memBool(KEY_PLAN_ON, false), memBool(KEY_AB_ON, false), memRemove(KEY_AB_PIN), simple(OpKind::RunAllOn),
          simple(OpKind::SingleStepTestsOff), simple(OpKind::UpdateToggles)};
}

// DRIVE PLAN on: everything off first, the set-up stored, the plain switch off, the plan on
inline std::vector<Op> drivePlanOps() {
  std::vector<Op> ops = allTestsOffOps();
  ops.push_back(simple(OpKind::SaveAbConfig));
  ops.push_back(memBool(KEY_AB_ON, false));
  ops.push_back(memRemove(KEY_AB_PIN));
  ops.push_back(memBool(KEY_PLAN_ON, true));
  return ops;
}

// The primary control: the drive plan when an alternation is set up, else run all
inline std::vector<Op> startOps(bool configured) {
  return configured ? drivePlanOps() : runAllOps();
}

// STOP: ALL TESTS OFF, and the plain A/B switch off with its pin (what A/B OFF does), so nothing is left running
inline std::vector<Op> stopOps() {
  std::vector<Op> ops = allTestsOffOps();
  ops.push_back(memBool(KEY_AB_ON, false));
  ops.push_back(memRemove(KEY_AB_PIN));
  return ops;
}

// Which sequence a START tap is: the label says "A/B relay" exactly when the plan is what runs
inline bool startRunsPlan(bool configured) {
  return configured;
}

// ---- the cruise set-speed check's page (selfdrive/car/mazda/ti_cruise_check.py) ------------------------------------------------
// Everything on it comes from what the car controller publishes (TiCruiseCheckStatus): the state, why, the rounds and their delays.
// RUN CHECK writes the trigger (TiCruiseCheck) and nothing else; it is offered only while the car reports it is free and no other mode
// is on. The page never shows "running" unless the car controller says so in a fresh report.
constexpr Rect CRUISE_TITLE = {400, 10, 1340, 180};
constexpr Rect CRUISE_BAND = {20, 520, 2120, 240};       // the status band: primary line and second line
constexpr Rect CRUISE_PRIMARY = {20, 520, 2120, 130};
constexpr Rect CRUISE_SECOND = {20, 650, 2120, 100};
constexpr Rect CRUISE_RESULT = {20, 775, 2120, 105};     // the answer, in large type
constexpr Rect CRUISE_DETAIL1 = {480, 890, 1660, 50};    // the rounds and the delays (not needed while driving)
constexpr Rect CRUISE_DETAIL2 = {480, 942, 1660, 50};
constexpr Rect CRUISE_DETAIL3 = {480, 994, 1660, 50};
constexpr int CRUISE_PRIMARY_PX = 100;
constexpr int CRUISE_SECOND_PX = 60;
constexpr int CRUISE_RESULT_PX = 76;
constexpr int MAX_CRUISE_PRIMARY_CHARS = 26;
constexpr int MAX_CRUISE_SECOND_CHARS = 56;
constexpr int MAX_CRUISE_RESULT_CHARS = 42;
constexpr double CRUISE_STALE_S = 3.0;                    // a report older than this is no report

struct CruiseRound {
  bool minus = false, plus = false;
  int minus_ms = 0, plus_ms = 0;
};

struct CruiseView {
  bool have = false;                // a status block could be read
  bool fresh = false;               // ... and it is recent (CRUISE_STALE_S)
  std::string state;                // idle refused waiting running restoring finished aborted (the controller's own word)
  std::string why;
  std::string phase = "A";          // the recipe being run
  int round = 0;
  std::vector<CruiseRound> a, b;    // rounds run with recipe A (2 frames) and recipe B (3 frames)
  int orig = 0, set = 0;            // the set speed at the start, and now (mph)
  std::string result;               // YES PARTLY NO once finished
  std::string restore;              // what the restore did
};

inline bool cruiseActive(const CruiseView &v) {
  return v.state == "waiting" || v.state == "running" || v.state == "restoring";
}

// The check is running by the car's own fresh report: START, the A/B switch and the tests wait while it is
inline bool cruiseBusy(const CruiseView &v) {
  return v.have && v.fresh && cruiseActive(v);
}

inline int cruiseOk(const std::vector<CruiseRound> &r) {
  int n = 0;
  for (const CruiseRound &x : r) {
    n += (x.minus && x.plus) ? 1 : 0;
  }
  return n;
}

// "1,1,40,40;0,0,0,0" (the CLI's form of the status rows) into rounds
inline std::vector<CruiseRound> parseCruiseRounds(const std::string &text) {
  std::vector<CruiseRound> out;
  if (text.empty() || text == "-") {
    return out;
  }
  size_t pos = 0;
  while (pos <= text.size()) {
    const size_t end = std::min(text.find(';', pos), text.size());
    const std::string item = text.substr(pos, end - pos);
    int v[4] = {0, 0, 0, 0};
    size_t p = 0;
    for (int k = 0; k < 4 && p <= item.size(); k++) {
      const size_t e = std::min(item.find(',', p), item.size());
      v[k] = std::atoi(item.substr(p, e - p).c_str());
      p = e + 1;
    }
    out.push_back(CruiseRound{v[0] != 0, v[1] != 0, v[2], v[3]});
    pos = end + 1;
  }
  return out;
}

struct CruiseLook {
  std::string word;                 // the screen's state: idle refused waiting running restoring finished aborted unavailable noreport
  std::string primary;              // the status band, very large
  Tone tone = Tone::Stopped;
  std::string second;
  std::string result;               // the answer in large type ("" when there is none yet)
  std::string detail1, detail2, detail3;
  std::string run_sub;
  bool run_enabled = false;
  std::string stop_sub;
  bool stop_enabled = false;
};

inline std::string upperFirst(const std::string &t) {
  std::string out = t;
  if (!out.empty() && out[0] >= 'a' && out[0] <= 'z') {
    out[0] = (char)(out[0] - 'a' + 'A');
  }
  return out;
}

// "SET SPEED CAN BE CHANGED: YES (3 of 3)"
inline std::string cruiseResultLine(const CruiseView &v) {
  const int best = std::max(cruiseOk(v.a), cruiseOk(v.b));
  const char *word = v.result == "YES" ? "YES" : (v.result == "NO" ? "NO" : "PARTLY");
  char buf[64];
  std::snprintf(buf, sizeof(buf), "SET SPEED CAN BE CHANGED: %s (%d of 3)", word, best);
  return buf;
}

// "Recipe A (2-frame taps): 3 of 3 rounds accepted. Recipe B not needed"
inline std::string cruiseRecipeLine(const CruiseView &v) {
  char buf[128];
  if (v.a.empty() && v.b.empty()) {
    return "No round completed";
  }
  if (v.b.empty()) {
    std::snprintf(buf, sizeof(buf), "Recipe A (2-frame taps): %d of %d rounds accepted%s", cruiseOk(v.a), (int)v.a.size(),
                  v.state == "finished" ? ". Recipe B not needed" : "");
  } else {
    std::snprintf(buf, sizeof(buf), "Recipe A (2-frame taps): %d of %d. Recipe B (3-frame taps): %d of %d", cruiseOk(v.a), (int)v.a.size(),
                  cruiseOk(v.b), (int)v.b.size());
  }
  return buf;
}

// "Delay to the change: minus 40, 40, 40 ms; plus 40, 40, 40 ms"
inline std::string cruiseDelayLine(const CruiseView &v) {
  std::string minus, plus;
  for (const std::vector<CruiseRound> *rs : {&v.a, &v.b}) {
    for (const CruiseRound &r : *rs) {
      if (r.minus) {
        minus += (minus.empty() ? "" : ", ") + std::to_string(r.minus_ms);
      }
      if (r.plus) {
        plus += (plus.empty() ? "" : ", ") + std::to_string(r.plus_ms);
      }
    }
  }
  if (minus.empty() && plus.empty()) {
    return v.a.empty() && v.b.empty() ? std::string("No round finished yet") : std::string("No press changed the set speed");
  }
  return "Delay to the change: minus " + (minus.empty() ? std::string("none") : minus + " ms") + "; plus " +
         (plus.empty() ? std::string("none") : plus + " ms");
}

inline bool cruiseNotRestored(const CruiseView &v) {
  return v.restore.rfind("Set speed is now", 0) == 0;
}

inline std::string cruiseSetLine(const CruiseView &v) {
  char buf[160];
  if (v.orig <= 0 || cruiseNotRestored(v)) {
    return upperFirst(v.restore);      // "Set speed is now X mph (was Y). Not restored: set it yourself."
  }
  std::snprintf(buf, sizeof(buf), "Set speed %d mph at the start, %d mph now%s%s", v.orig, v.set, v.restore.empty() ? "" : ". ",
                upperFirst(v.restore).c_str());
  return buf;
}

// The page, from the car's report and from what else is on (others: "" when nothing else is switched on, else why the check cannot run)
inline CruiseLook cruiseLook(const CruiseView &v, const std::string &others) {
  CruiseLook k;
  k.stop_sub = "or press the brake";
  const bool final_state = v.have && (v.state == "finished" || v.state == "aborted");
  if (final_state) {
    if (v.state == "finished") {
      k.result = cruiseResultLine(v);
    } else if (cruiseNotRestored(v)) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "SET IT YOURSELF: NOW %d MPH, WAS %d", v.set, v.orig);
      k.result = buf;                  // large: the set speed is not where it was and nothing more will be sent
    }
    k.detail1 = cruiseRecipeLine(v);
    k.detail2 = cruiseDelayLine(v);
    k.detail3 = cruiseSetLine(v);
  }
  if (!v.have || !v.fresh) {
    k.word = "noreport";
    k.primary = "NO REPORT FROM THE CAR";
    k.tone = Tone::Held;
    k.second = final_state ? "Last result below; kept until controls restart" : "The check runs only while driving";
    k.run_sub = "Needs the car's report: drive first";
    return k;
  }
  if (cruiseActive(v)) {
    k.stop_enabled = true;
    k.run_sub = "Running now: one press is one run";
  }
  if (v.state == "waiting") {
    k.word = "waiting";
    k.primary = "WAITING";
    k.tone = Tone::Waiting;
    k.second = v.why.empty() ? std::string("Checking the conditions") : upperFirst(v.why);
    k.detail1 = "Needs: engaged, cruise on, steady 30-70 mph, set 25 mph+, no car ahead";
    k.detail2 = "Gives up after 60 s of waiting; a run stops itself after 40 s";
    return k;
  }
  if (v.state == "running") {
    k.word = "running";
    char buf[48];
    std::snprintf(buf, sizeof(buf), v.phase == "B" ? "RUNNING: B ROUND %d OF 3" : "RUNNING: ROUND %d OF 3", std::max(1, v.round));
    k.primary = buf;
    k.tone = Tone::Running;
    k.second = v.why.empty() ? std::string("Hands off the cruise buttons. Brake stops it.") : "Holding: " + v.why;
    k.detail1 = cruiseRecipeLine(v);
    k.detail2 = cruiseDelayLine(v);
    k.detail3 = cruiseSetLine(v);
    return k;
  }
  if (v.state == "restoring") {
    k.word = "restoring";
    k.primary = "PUTTING SET SPEED BACK";
    k.tone = Tone::Waiting;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "To %d mph: at most 3 taps", v.orig);
    k.second = buf;
    k.detail1 = cruiseRecipeLine(v);
    k.detail2 = cruiseDelayLine(v);
    k.detail3 = cruiseSetLine(v);
    return k;
  }
  // not running: RUN CHECK is offered when nothing else is on
  if (!others.empty()) {
    k.word = "unavailable";
    k.primary = "UNAVAILABLE";
    k.tone = Tone::Held;
    k.second = upperFirst(others) + ": tap STOP on the START page";
    k.run_sub = "Not while another mode is on";
    return k;
  }
  k.run_enabled = true;
  k.run_sub = "One run: 3 rounds of set-minus, set-plus";
  if (v.state == "finished") {
    k.word = "finished";
    k.primary = "FINISHED";
    k.tone = Tone::Done;
    char buf[64];
    if (v.b.empty()) {
      std::snprintf(buf, sizeof(buf), "Recipe A (2-frame taps): %d of %d", cruiseOk(v.a), (int)v.a.size());
    } else {
      std::snprintf(buf, sizeof(buf), "Recipe A: %d of %d, recipe B: %d of %d", cruiseOk(v.a), (int)v.a.size(), cruiseOk(v.b), (int)v.b.size());
    }
    k.second = buf;
    return k;
  }
  if (v.state == "aborted") {
    k.word = "aborted";
    k.primary = "ABORTED";
    k.tone = Tone::Held;
    k.second = upperFirst(v.why);
    return k;
  }
  if (v.state == "refused") {
    k.word = "refused";
    k.primary = "REFUSED";
    k.tone = Tone::Held;
    k.second = upperFirst(v.why);
    return k;
  }
  k.word = "idle";
  k.primary = "READY: TAP RUN CHECK";
  k.tone = Tone::Stopped;
  k.second = "Not run since controls started";
  k.detail1 = "Engage with stock cruise on at 30-70 mph, no car ahead, then tap RUN CHECK";
  k.detail2 = "Taps set-minus then set-plus, 3 times; stops itself after 40 s";
  return k;
}

// ---- curve slowing's page (selfdrive/car/mazda/ti_curve.py) ---------------------------------------------------------------------
// Everything shown comes from what the car controller publishes (TiCurveStatus: Curve.status()): its line1 is the primary line,
// its line2 the second line, word for word (cut only when longer than the space). The one control writes the switch (TiCurveOn,
// tmpfs: "1" on for this drive, "0" off) and nothing else; its label comes from the car's report, never from the switch: the state
// "off" offers TURN ON, any other state TURN OFF. No fresh report: TURN OFF (it can always switch off, never on). Which other mode
// keeps curve slowing waiting is decided by the car side and said in its lines.
const char *const TI_CURVE_ON_KEY = "TiCurveOn";
const char *const TI_CURVE_STATUS_KEY = "TiCurveStatus";
constexpr Rect CURVE_TITLE = {400, 10, 1340, 180};
constexpr Rect CURVE_BAND = {20, 540, 2120, 420};        // the status band: two primary lines and the second line
constexpr Rect CURVE_PRIMARY1 = {20, 550, 2120, 125};
constexpr Rect CURVE_PRIMARY2 = {20, 675, 2120, 125};
constexpr Rect CURVE_SECOND = {20, 820, 2120, 120};
constexpr int CURVE_PRIMARY_PX = 100;
constexpr int CURVE_SECOND_PX = 64;
constexpr int MAX_CURVE_PRIMARY_CHARS = 27;               // per line; line1 takes up to two lines
constexpr int MAX_CURVE_SECOND_CHARS = 52;
constexpr double CURVE_STALE_S = CRUISE_STALE_S;          // a report older than this is no report

struct CurveView {
  bool have = false;                // a status block could be read
  bool fresh = false;               // ... and it is recent (CURVE_STALE_S)
  std::string state;                // off ready slowing holding restoring paused stopped waiting (the controller's own word)
  std::string line1, line2;
  bool needs_more = false;
};

struct CurveLook {
  std::string word;                 // the controller's state, or "noreport"
  std::string primary1, primary2;   // line1 in very large type, on one or two lines
  std::string second;               // line2 in large type
  bool warn = false;                // warning colour: stopped, a corner that needs more than may be shed, or no report
  bool on = false;                  // the band says curve slowing is on (any state but "off" in a fresh report)
  std::string button;               // TURN ON / TURN OFF
  std::string button_sub;
  const char *write = "0";          // what the control writes to TiCurveOn
};

// A line cut to max characters: the cut end is "..." (the controller's lines are written to fit; this only guards the space)
inline std::string curveCut(const std::string &t, int max) {
  if ((int)t.size() <= max) {
    return t;
  }
  return t.substr(0, std::max(0, max - 3)) + "...";
}

// line1 on at most two lines of max characters: one line when it fits; else broken after "STATE:" when both parts fit; else at the
// last space that fits; what still does not fit is cut on the second line. Joined with one space the lines give line1 back
// (or its start, when cut).
inline void curveWrap(const std::string &t, int max, std::string &l1, std::string &l2) {
  l1.clear();
  l2.clear();
  if ((int)t.size() <= max) {
    l1 = t;
    return;
  }
  const size_t colon = t.find(": ");
  if (colon != std::string::npos && (int)colon + 1 <= max && (int)(t.size() - colon - 2) <= max) {
    l1 = t.substr(0, colon + 1);
    l2 = t.substr(colon + 2);
    return;
  }
  size_t brk = t.rfind(' ', (size_t)max);
  if (brk == std::string::npos || brk == 0) {
    l1 = t.substr(0, max);
    l2 = curveCut(t.substr(max), max);
    return;
  }
  l1 = t.substr(0, brk);
  l2 = curveCut(t.substr(brk + 1), max);
}

inline CurveLook curveLook(const CurveView &v) {
  CurveLook k;
  if (!v.have || !v.fresh) {
    k.word = "noreport";
    curveWrap("CURVE SLOWING: no report from the car", MAX_CURVE_PRIMARY_CHARS, k.primary1, k.primary2);
    k.second = "TURN OFF still switches it off";
    k.warn = true;
    k.button = "TURN OFF";
    k.button_sub = "No report from the car: this only switches it off";
    k.write = "0";
    return k;
  }
  k.word = v.state;
  curveWrap(v.line1, MAX_CURVE_PRIMARY_CHARS, k.primary1, k.primary2);
  k.second = curveCut(v.line2, MAX_CURVE_SECOND_CHARS);
  k.warn = v.state == "stopped" || v.needs_more;
  k.on = v.state != "off";
  if (k.on) {
    k.button = "TURN OFF";
    k.button_sub = "Curve slowing is on: tap to switch it off";
    k.write = "0";
  } else {
    k.button = "TURN ON";
    k.button_sub = "For this drive: off again after every restart";
    k.write = "1";
  }
  return k;
}

// ---- the cruise button tests' page (selfdrive/car/mazda/ti_button_tests.py) ------------------------------------------------------
// The hold test and the tap-rate test, one page. Every line on it is the car controller's own (TiButtonTestStatus: ButtonTests.status(),
// written there to fit these boxes; cut here only if longer than the space). HOLD TEST / TAP TEST write the trigger (TiButtonTest:
// {"cmd": "hold" | "taps"}) and nothing else, and are offered only while the car's fresh report says no test runs and the screen
// knows of no other mode on; STOP TEST writes {"cmd": "stop"} while one runs. The page never shows a test running unless the car's
// fresh report says so.
const char *const TI_BUTTON_TRIGGER_KEY = "TiButtonTest";
const char *const TI_BUTTON_STATUS_KEY = "TiButtonTestStatus";
constexpr Rect BUTTONS_TITLE = {400, 10, 1340, 180};
constexpr Rect BUTTONS_BAND = {20, 520, 2120, 240};      // the status band: primary line and second line
constexpr Rect BUTTONS_PRIMARY = {20, 520, 2120, 130};
constexpr Rect BUTTONS_SECOND = {20, 650, 2120, 100};
constexpr Rect BUTTONS_RESULT = {20, 775, 2120, 105};    // the answer, in large type
constexpr Rect BUTTONS_DETAIL1 = {480, 890, 1660, 50};   // the steps seen / the three cycles' rows
constexpr Rect BUTTONS_DETAIL2 = {480, 942, 1660, 50};
constexpr Rect BUTTONS_DETAIL3 = {480, 994, 1660, 50};
constexpr int BUTTONS_PRIMARY_PX = 100;
constexpr int BUTTONS_SECOND_PX = 60;
constexpr int BUTTONS_RESULT_PX = 76;
constexpr int MAX_BUTTONS_PRIMARY_CHARS = 26;            // = ti_button_tests.MAX_LINE1 (tested on both sides)
constexpr int MAX_BUTTONS_SECOND_CHARS = 55;             // MAX_LINE2
constexpr int MAX_BUTTONS_RESULT_CHARS = 42;             // MAX_RESULT
constexpr int MAX_BUTTONS_DETAIL_CHARS = 74;             // MAX_DETAIL
constexpr int MAX_BUTTONS_SUB_CHARS = 24;                // a control's second line at START_SUB_PX in its 660 px box
constexpr double BUTTONS_STALE_S = CRUISE_STALE_S;

struct ButtonsView {
  bool have = false;                // a status block could be read
  bool fresh = false;               // ... and it is recent
  std::string state;                // idle refused waiting running restoring finished aborted (the controller's own word)
  std::string test;                 // hold taps ("" before the first run)
  std::string line1, line2, result, d1, d2, d3;
};

struct ButtonsLook {
  std::string word;                 // the controller's state, "unavailable" or "noreport"
  std::string primary, second, result, detail1, detail2, detail3;
  Tone tone = Tone::Stopped;
  bool result_warn = false;         // the large line asks the driver to set the speed (warning colour)
  bool run_enabled = false;         // HOLD TEST and TAP TEST
  std::string hold_sub, taps_sub;
  bool stop_enabled = false;
  std::string stop_sub;
};

inline bool buttonsActive(const ButtonsView &v) {
  return v.state == "waiting" || v.state == "running" || v.state == "restoring";
}

// A button test is in progress by the car's own fresh report: START, the A/B switch and the tests wait while it is
inline bool buttonsBusy(const ButtonsView &v) {
  return v.have && v.fresh && buttonsActive(v);
}

inline ButtonsLook buttonsLook(const ButtonsView &v, const std::string &others) {
  ButtonsLook k;
  k.stop_sub = "or press the brake";
  k.hold_sub = "One 0.7 s hold each way";
  k.taps_sub = "5 down, 5 up, 3 rates";
  const bool final_state = v.have && (v.state == "finished" || v.state == "aborted");
  if (v.have) {
    k.result = curveCut(v.result, MAX_BUTTONS_RESULT_CHARS);
    k.detail1 = curveCut(v.d1, MAX_BUTTONS_DETAIL_CHARS);
    k.detail2 = curveCut(v.d2, MAX_BUTTONS_DETAIL_CHARS);
    k.detail3 = curveCut(v.d3, MAX_BUTTONS_DETAIL_CHARS);
    k.result_warn = v.result.rfind("SET IT YOURSELF", 0) == 0;
  }
  if (!v.have || !v.fresh) {
    k.word = "noreport";
    k.primary = "NO REPORT FROM THE CAR";
    k.tone = Tone::Held;
    k.second = final_state ? "Last result below; kept until controls restart" : "The tests run only while driving";
    if (!final_state) {
      k.result.clear();
      k.detail1.clear();
      k.detail2.clear();
      k.detail3.clear();
    }
    k.hold_sub = k.taps_sub = "Needs the car's report";
    return k;
  }
  k.word = v.state;
  k.primary = curveCut(v.line1, MAX_BUTTONS_PRIMARY_CHARS);
  k.second = curveCut(v.line2, MAX_BUTTONS_SECOND_CHARS);
  if (buttonsActive(v)) {
    k.tone = v.state == "running" ? Tone::Running : Tone::Waiting;
    k.stop_enabled = true;
    k.hold_sub = k.taps_sub = "Running: one at a time";
    return k;
  }
  if (!others.empty()) {
    k.word = "unavailable";
    k.primary = "UNAVAILABLE";
    k.tone = Tone::Held;
    k.second = curveCut(upperFirst(others) + ": tap STOP on the START page", MAX_BUTTONS_SECOND_CHARS);
    k.hold_sub = k.taps_sub = "Another mode is on";
    return k;
  }
  k.run_enabled = true;
  k.tone = v.state == "finished" ? Tone::Done : ((v.state == "aborted" || v.state == "refused") ? Tone::Held : Tone::Stopped);
  return k;
}

}  // namespace ti_pages
