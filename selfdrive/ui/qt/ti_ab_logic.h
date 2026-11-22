#pragma once

// Mazda TI A/B page: the logic that does not need Qt, so it can be compiled and tested on its own
// (selfdrive/ui/tests/test_ti_ab_logic.cc, run by selfdrive/car/tests/test_mazda_ti_ab_screen_logic.py).
//   - the allow-listed settings with their hard ranges and largest A-to-B steps, mirroring selfdrive/car/mazda/ti_ab.py
//     (SETTINGS): the page only ever offers values the car side accepts, and the car side refuses anything else with a message
//   - how B and the block length are stepped, what a tap on PIN A / PIN B asks for, what may be edited while A/B is on
//   - the texts for the reasons a switch waits (ti_ab.MESSAGE) and the states

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace ti_ab {

struct Setting {
  const char *name;   // the param (one of the 15 tuning settings)
  const char *toggle; // the frogpilot_toggles key that holds the owner's value as the controller reads it (steerKp: [[0], [value]])
  bool is_bool;       // on/off; B is then always the opposite of A
  double lo;          // hard range
  double hi;
  double max_step;    // largest |A - B|
  double step;        // what one tap on B - / B + moves
};

constexpr int N_SETTINGS = 6;
constexpr Setting SETTINGS[N_SETTINGS] = {
  {"SteerKP", "steerKp", false, 0.70, 1.00, 0.10, 0.05},
  {"LatCommitSetpoint", "lat_commit_setpoint", true, 0, 1, 1, 1},
  {"LatDamping", "lat_damping", true, 0, 1, 1, 1},
  {"LatFrictionComp", "lat_friction_comp", true, 0, 1, 1, 1},
  {"LatOutputFilter", "lat_output_filter", true, 0, 1, 1, 1},
  {"LatNoFrictionRelay", "lat_no_friction_relay", true, 0, 1, 1, 1},
};
constexpr double MIN_BLOCK_MIN = 4.0;
constexpr double MAX_BLOCK_MIN = 10.0;
constexpr double DEFAULT_BLOCK_MIN = 4.0;
constexpr double BLOCK_STEP_MIN = 1.0;
constexpr double STATUS_STALE_S = 4.0;   // the car side writes the status at 2 Hz while on, and every few seconds while off

inline int settingIndex(const char *name) {
  for (int i = 0; i < N_SETTINGS; i++) {
    if (std::strcmp(SETTINGS[i].name, name) == 0) {
      return i;
    }
  }
  return -1;
}

// the setting after / before i in the list, wrapping
inline int cycleSetting(int i, int dir) {
  return ((i + (dir > 0 ? 1 : -1)) % N_SETTINGS + N_SETTINGS) % N_SETTINGS;
}

inline double roundTo(double v, double step) {
  return std::round(v / step) * step;
}

// B acceptable for the owner's stored value A (the car side's rule): inside the range, within max_step of A, and not A itself
inline bool bAllowed(const Setting &s, double a, double b) {
  if (!std::isfinite(a) || !std::isfinite(b) || a < s.lo - 1e-9 || a > s.hi + 1e-9) {
    return false;
  }
  if (s.is_bool) {
    return (a != 0.0) != (b != 0.0) && (b == 0.0 || b == 1.0);
  }
  return b >= s.lo - 1e-9 && b <= s.hi + 1e-9 && std::fabs(b - a) <= s.max_step + 1e-9 && std::fabs(b - a) >= s.step / 2.0;
}

// The B to offer first: a bool's opposite of A; a number one step above A (below at the top of the range)
inline double defaultB(const Setting &s, double a) {
  if (s.is_bool) {
    return a != 0.0 ? 0.0 : 1.0;
  }
  const double up = roundTo(a + s.step, s.step);
  return up <= s.hi + 1e-9 ? up : roundTo(a - s.step, s.step);
}

// B after a tap on B - (dir < 0) or B + (dir > 0): one step, never outside what bAllowed accepts (an end stays), and it
// jumps over A itself. A bool does not move: its B is the opposite of A.
inline double stepB(const Setting &s, double a, double b, int dir) {
  if (s.is_bool) {
    return defaultB(s, a);
  }
  double next = roundTo(b + (dir > 0 ? s.step : -s.step), s.step);
  if (std::fabs(next - a) < s.step / 2.0) {
    next = roundTo(next + (dir > 0 ? s.step : -s.step), s.step);
  }
  return bAllowed(s, a, next) ? next : b;
}

inline double stepBlock(double minutes, int dir) {
  const double next = minutes + (dir > 0 ? BLOCK_STEP_MIN : -BLOCK_STEP_MIN);
  return next < MIN_BLOCK_MIN ? MIN_BLOCK_MIN : (next > MAX_BLOCK_MIN ? MAX_BLOCK_MIN : next);
}

// The setting, B and the block length can be changed only while A/B is off (the car side takes a changed configuration as a
// reason to stop and wants the switch cycled)
inline bool canEdit(bool on) {
  return !on;
}

// PIN B (a change of the value, and the one that matters) needs a second tap within PIN_CONFIRM_S; PIN A and releasing do not
constexpr double PIN_CONFIRM_S = 3.0;

// Settings whose arms can differ only in bends: the page says so and the car side scores minutes only there
inline bool bendsOnly(const char *name) {
  return std::strcmp(name, "LatCommitSetpoint") == 0;
}

enum class Pin { None, A, B };

// armed_at: when the first tap on PIN B was made (negative: none). True when this tap, at `now`, confirms it
inline bool pinBConfirmed(double now, double armed_at) {
  return armed_at >= 0.0 && now >= armed_at && now - armed_at <= PIN_CONFIRM_S;
}

// What a tap on PIN A or PIN B asks for: the other arm pinned, or alternation again when the tapped arm is already pinned
inline Pin tapPin(Pin current, Pin tapped) {
  return current == tapped ? Pin::None : tapped;
}

inline const char *pinText(Pin p) {
  return p == Pin::A ? "A" : (p == Pin::B ? "B" : "");
}

// The words for why a switch is waiting: the reason names of ti_ab.PRIORITY
struct Reason {
  const char *name;
  const char *text;
};
constexpr int N_REASONS = 12;
constexpr Reason REASONS[N_REASONS] = {
  {"input", "bad inputs"}, {"touch", "hands on the wheel"}, {"lane_change", "lane change"},
  {"holdoff", "after a touch or blinker"}, {"speed", "speed outside 36-69 mph"}, {"eps", "near 28-31 mph"},
  {"margin", "speed not yet well inside the range"},
  {"curve", "road bends"}, {"lat", "lateral demand"}, {"request", "steering request large"},
  {"saturated", "steering saturated"}, {"settle", "settling"},
};

inline const char *reasonText(const char *name) {
  for (const Reason &r : REASONS) {
    if (std::strcmp(r.name, name) == 0) {
      return r.text;
    }
  }
  return "waiting";
}

// The drive plan's line, on both pages (selfdrive/car/mazda/ti_plan.py): what the plan is running now, what is next, and the
// alternation's scored minutes per side against its target. mode: off, hold, ab, tests, dead.
constexpr double PLAN_TARGET_MIN_DEFAULT = 10.0;
constexpr double PLAN_ALT_MIN_MPH = 35.0;
constexpr double PLAN_ALT_MAX_MPH = 64.0;
constexpr double START_MARGIN_MPH = 3.0;   // ti_ab.MARGIN_MPH: a block starts only this far inside the range (it counts across all of it)

inline std::string planLine(const char *mode, double a_s, double b_s, double target_min, bool complete) {
  char buf[220];
  const char *now = "off";
  const char *next = "";
  if (std::strcmp(mode, "ab") == 0) {
    now = "A/B alternation";
    next = "tests when the speed leaves 35-64 mph or A/B completes";
  } else if (std::strcmp(mode, "tests") == 0) {
    now = "identification tests";
    next = complete ? "tests only: A/B is complete" : "A/B when it is set up and the speed is 35-64 mph";
  } else if (std::strcmp(mode, "hold") == 0) {
    now = "handing over";
    next = "waiting for a quiet moment";
  } else if (std::strcmp(mode, "dead") == 0) {
    now = "stopped after an error";
    next = "nothing runs";
  }
  if (std::strcmp(mode, "off") == 0) {
    return std::string("Drive plan off");
  }
  std::snprintf(buf, sizeof(buf), "PLAN: %s | next: %s | A %.1f / B %.1f of %.0f min%s", now, next, a_s / 60.0, b_s / 60.0, target_min,
                complete ? " COMPLETE" : "");
  return std::string(buf);
}

// Why the tests are held, in plain words (the car controller's `tests_hold` reasons, selfdrive/car/mazda/ti_plan.HOLD_WHYS); "" for
// a reason that is not a hold. Shown on the Tests page in place of the generic status and on the A/B page.
inline const char *holdText(const char *why) {
  struct Hold {
    const char *why;
    const char *text;
  };
  static const Hold HOLDS[] = {
    {"plan dead", "Held: drive plan stopped after an error; restart needed"},
    {"plan missing", "Held: drive plan not reporting from controls"},
    {"plan unreadable", "Held: drive plan unreadable"},
    {"plan stale", "Held: controls not reporting (stale)"},
    {"alternation not at A", "Waiting: A/B alternation returning to your setting"},
    {"plan says ab", "Waiting: drive plan is running A/B"},
    {"plan says hold", "Waiting: drive plan is handing over"},
    {"plan says off", "Waiting: drive plan is starting"},
    {"A/B switch on", "Waiting: A/B alternation is on"},
    {"control loop busy", "Tests paused: control loop busy"},
  };
  for (const Hold &h : HOLDS) {
    if (std::strcmp(h.why, why) == 0) {
      return h.text;
    }
  }
  return "";
}

inline bool statusFresh(double now, double t) {
  return std::isfinite(now) && std::isfinite(t) && now - t >= -1.0 && now - t <= STATUS_STALE_S;
}

}  // namespace ti_ab
