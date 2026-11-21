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
constexpr int ROW_TOP = 500;       // where the content of the individual pages starts (under two rows of large controls)

// Font sizes in pixels (InterFont(px), the device's own font; bold for the status lines and every control label)
constexpr int PRIMARY_PX = 100;    // the primary status line
constexpr int SECOND_PX = 70;      // the second line
constexpr int EXPERIMENT_PX = 40;  // the experiment in plain words (set-up page, progress page)
constexpr int CHOSEN_PX = 52;      // the chosen experiment on the first page, in large plain words
constexpr int START_PX = 72;       // START label (its second line: START_SUB_PX)
constexpr int START_SUB_PX = 40;
constexpr int STOP_PX = 110;       // STOP
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

enum class Page { Home, Setup, Tests, Overview, Ab, Progress };

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
};

struct Control {
  int id;
  Rect r;
  const char *name;
};

constexpr int MAX_CONTROLS = 16;

inline int footerHeight(int error_lines) {
  return error_lines > 0 ? 30 + 50 * std::min(2, error_lines) : 0;
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
      add(CTL_NAV_SETUP, 20, 550, 360, 180, "SET UP A/B");
      add(CTL_NAV_PROGRESS, 20, 770, 360, 180, "PROGRESS");
      add(CTL_START, 480, 330, 1660, 272, "START");
      add(CTL_STOP, 480, 692, 1660, 272, "STOP");
      break;
    case Page::Setup:
      add(CTL_BACK, 20, 10, 360, 180, "BACK");
      add(CTL_LOCK, 1780, 10, 360, 180, "LOCK");
      add(CTL_SETUP_USE, 480, 240, 1660, 150, "A/B USED / NOT USED");
      add(CTL_SETUP_PREV, 480, 430, 220, 150, "SETTING <");
      add(CTL_SETUP_NEXT, 1920, 430, 220, 150, "SETTING >");
      add(CTL_SETUP_B_MINUS, 480, 620, 220, 150, "B -");
      add(CTL_SETUP_B_PLUS, 1920, 620, 220, 150, "B +");
      add(CTL_SETUP_BLOCK_MINUS, 480, 810, 220, 150, "BLOCK -");
      add(CTL_SETUP_BLOCK_PLUS, 1920, 810, 220, 150, "BLOCK +");
      add(CTL_SETUP_RESET, 20, 330, 360, 180, "RESET PROGRESS");
      break;
    case Page::Progress:
      add(CTL_BACK, 20, 10, 360, 180, "< START PAGE");
      add(CTL_LOCK, 1780, 10, 360, 180, "LOCK");
      add(CTL_NAV_INDIV, 20, 330, 360, 180, "INDIVIDUAL TESTS");
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
};
constexpr int N_HOLD_SHORT = 9;

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

// "A 14 / 20 min, B 13 / 20 min"
inline std::string abProgressText(double a_s, double b_s, double target_min) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "A %d / %.0f min, B %d / %.0f min", wholeMin(a_s), target_min, wholeMin(b_s), target_min);
  return buf;
}

// Driving time still needed at the alternation's speeds: the scored time still missing on both sides (the sides alternate)
inline double abMinutesLeft(double a_s, double b_s, double target_min) {
  const double t = 60.0 * target_min;
  return (std::max(0.0, t - a_s) + std::max(0.0, t - b_s)) / 60.0;
}

inline std::string abLeftText(double a_s, double b_s, double target_min) {
  const double left = abMinutesLeft(a_s, b_s, target_min);
  if (left <= 0.0) {
    return "A/B complete: the tests take over";
  }
  char buf[96];
  std::snprintf(buf, sizeof(buf), "About %d min of driving at %d-%d mph left", (int)std::ceil(left), (int)ti_ab::PLAN_ALT_MIN_MPH + 1,
                (int)ti_ab::PLAN_ALT_MAX_MPH - 1);
  return buf;
}

// "A/B 14 of 20 min each side done": the second line of the stopped state
inline std::string stoppedSecond(bool known, double a_s, double b_s, double target_min) {
  if (!known || (a_s <= 0.0 && b_s <= 0.0)) {
    return "Nothing is running";
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "A/B %d of %.0f min each side done", wholeMin(std::min(a_s, b_s)), target_min);
  return buf;
}

// One window of the tests: "21-27 mph" and "3 of 12 done; hold 24 mph for about 9 min" (done, total: required items)
inline std::string windowLabel(int w, bool metric) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%d-%d %s", speedValue(ti_logic::WINDOWS[w].low, metric), speedValue(ti_logic::WINDOWS[w].high, metric),
                metric ? "km/h" : "mph");
  return buf;
}

inline std::string windowRowText(int w, int done, int total, double secs_left, bool metric) {
  char buf[96];
  if (total <= 0) {
    return "no test needs this window";
  }
  if (done >= total) {
    std::snprintf(buf, sizeof(buf), "%d of %d done", done, total);
    return buf;
  }
  std::snprintf(buf, sizeof(buf), "%d of %d done; hold %d %s for about %d min", done, total, speedValue(ti_logic::windowTargetMph(w), metric),
                metric ? "km/h" : "mph", std::max(1, (int)std::ceil(secs_left / 60.0)));
  return buf;
}

// What the tests and the alternation each get of the speed range (the plan gives 36-55 mph to the alternation until it is done)
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

// The speeds that count, nearest first below and above `mph` (the middle of the nearest windows), "Hold 24 or 38 mph"
inline bool countsNothingAt(double mph) {
  return ti_logic::zoneFor(mph).state != ti_logic::ZoneState::Inside;
}

inline std::string nearestCounting(double mph, bool metric) {
  int lower = -1, upper = -1;
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    if (ti_logic::WINDOWS[w].high <= mph) {
      lower = w;
    }
    if (ti_logic::WINDOWS[w].low > mph && upper < 0) {
      upper = w;
    }
  }
  char buf[64];
  const char *unit = metric ? "km/h" : "mph";
  if (lower >= 0 && upper >= 0) {
    std::snprintf(buf, sizeof(buf), "Hold %d or %d %s", speedValue(ti_logic::windowTargetMph(lower), metric), speedValue(ti_logic::windowTargetMph(upper), metric), unit);
  } else {
    const int w = lower >= 0 ? lower : upper;
    std::snprintf(buf, sizeof(buf), "Hold %d %s", speedValue(ti_logic::windowTargetMph(w), metric), unit);
  }
  return buf;
}

// The primary line and the second line of the first page.
inline HomeLines home(const HomeState &s) {
  HomeLines h;
  if (s.run == Run::Stopped) {
    h.primary = line("STOPPED: TAP START", Tone::Stopped);   // off after every stop and restart, said plainly
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
  if (!s.target) {
    if (plan && s.configured && !s.ab_complete) {
      const bool inside = s.mph >= ti_ab::PLAN_ALT_MIN_MPH && s.mph <= ti_ab::PLAN_ALT_MAX_MPH;
      if (inside) {
        h.primary = line("WAITING: A/B STARTING", Tone::Waiting);
        h.second = "Waiting for a quiet moment";
      } else {
        h.primary = line("A/B: " + holdSpeed(altTargetWindow(s.mph), s.metric), Tone::Waiting);
        h.second = nowSpeed(s.mph, s.metric);
      }
      return h;
    }
    h.primary = line(plan && s.configured ? "DONE: ALL COMPLETE" : "DONE: TESTS COMPLETE", Tone::Done);
    h.second = "Extra data still helps";
    return h;
  }
  const ti_logic::Zone zone = ti_logic::zoneFor(s.mph);
  const bool inside = zone.state == ti_logic::ZoneState::Inside && zone.window == s.target_window;
  if (countsNothingAt(s.mph)) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "NO COUNT AT %d %s", speedValue(s.mph, s.metric), speedUnit(s.metric));
    h.primary = line(buf, Tone::Waiting);
    h.second = nearestCounting(s.mph, s.metric);
  } else if (!inside) {
    h.primary = line(holdSpeed(s.target_window, s.metric), Tone::Waiting);
    h.second = nowSpeed(s.mph, s.metric);
  } else if (s.tests_active) {
    h.primary = line("TESTING: " + holdSpeed(s.target_window, s.metric), Tone::Running);
    h.second = mmss(s.target_secs) + " to go here";
  } else {
    std::string t = "WAITING: " + holdSpeed(s.target_window, s.metric);
    if (std::strcmp(s.gate_reason, "curve") == 0 || std::strcmp(s.gate_reason, "lat") == 0) {
      t = "WAITING: STRAIGHT ROAD";
    } else if (std::strcmp(s.gate_reason, "command") == 0) {
      t = "WAITING: EASE STEERING";
    }
    h.primary = line(t, Tone::Waiting);
    h.second = mmss(s.target_secs) + " to go here";
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
inline StartLook startLook(bool configured, Run run, bool confirmed = true) {
  StartLook k;
  const std::string what = configured ? "tests + A/B relay" : "tests";
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

inline std::string detailScored(double a_s, double b_s, double target_min, bool complete) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "Scored: A %.1f / B %.1f of %.0f min%s", a_s / 60.0, b_s / 60.0, target_min, complete ? " COMPLETE" : "");
  return buf;
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

}  // namespace ti_pages
