#pragma once

// Mazda TI Tests screen: the logic that does not need Qt, so it can be compiled and tested on its own
// (selfdrive/ui/tests/test_ti_tests_logic.cc, run by selfdrive/car/tests/test_mazda_ti_screen_logic.py).
//   - the speed windows, mirroring selfdrive/car/mazda/ti_ident.py (WINDOWS, MIN_TEST_MPH, EPS_LOW_MPH, EPS_HIGH_MPH,
//     MAX_TEST_MPH, zone_for, target_window, window_target_mph): test_mazda_ti_screen_logic.py checks the numbers agree
//   - the target the screen asks the driver for
//   - the lock of the Tests page: what keeps it on screen and what always wins over it

#include <cmath>

namespace ti_logic {

constexpr int N_WINDOWS = 4;
struct Window {
  double low;
  double high;
  const char *label;
};
constexpr Window WINDOWS[N_WINDOWS] = {{21.0, 27.0, "21-27"}, {35.0, 41.0, "35-41"}, {49.0, 56.0, "49-56"}, {64.0, 80.0, "64-80"}};
constexpr double MIN_TEST_MPH = 19.0;
constexpr double EPS_LOW_MPH = 28.0;
constexpr double EPS_HIGH_MPH = 31.0;
constexpr double MAX_TEST_MPH = 80.0;

enum class ZoneState { Below, Inside, Between, Transition, Above };

struct Zone {
  ZoneState state = ZoneState::Below;
  int window = -1;   // index in WINDOWS: the window the speed is inside, else the nearest one of the same steering mode
};

inline Zone zoneFor(double mph) {
  Zone z;
  if (mph < MIN_TEST_MPH) {
    z.state = ZoneState::Below;
    return z;
  }
  if (mph >= EPS_LOW_MPH && mph < EPS_HIGH_MPH) {
    z.state = ZoneState::Transition;
    return z;
  }
  if (mph >= MAX_TEST_MPH) {
    z.state = ZoneState::Above;
    return z;
  }
  for (int i = 0; i < N_WINDOWS; i++) {
    if (WINDOWS[i].low <= mph && mph < WINDOWS[i].high) {
      z.state = ZoneState::Inside;
      z.window = i;
      return z;
    }
  }
  z.state = ZoneState::Between;
  const bool low_side = mph < EPS_LOW_MPH;
  double best = 1e9;
  for (int i = 0; i < N_WINDOWS; i++) {
    if ((WINDOWS[i].high <= EPS_LOW_MPH) != low_side) {
      continue;
    }
    const double d = std::fmax(WINDOWS[i].low - mph, mph - WINDOWS[i].high);
    if (d < best) {
      best = d;
      z.window = i;
    }
  }
  return z;
}

// mph to change to be inside window i (0 when inside)
inline double windowDistance(double mph, int i) {
  return std::fmax(std::fmax(WINDOWS[i].low - mph, mph - WINDOWS[i].high), 0.0);
}

// The speed asked for in a window: its middle in whole mph (24, 38, 52, 72)
inline int windowTargetMph(int i) {
  return int((WINDOWS[i].low + WINDOWS[i].high) / 2.0);
}

struct Target {
  int window = -1;     // -1: nothing left to collect
  double secs = 0;     // required seconds still to collect there
  int cells = 0;       // required cells (steps) still incomplete there
  double dist = 0;     // mph to change
};

// The nearest window that still needs something: the least speed change first (whole mph), then the most still
// outstanding. secs[i] and cells[i]: what window i still needs (cells 0: complete).
inline Target chooseTarget(double mph, const double secs[N_WINDOWS], const int cells[N_WINDOWS]) {
  Target t;
  long best_dist = 0;
  for (int i = 0; i < N_WINDOWS; i++) {
    if (cells[i] <= 0) {
      continue;
    }
    const double d = windowDistance(mph, i);
    const long rd = std::lround(d);
    if (t.window < 0 || rd < best_dist || (rd == best_dist && secs[i] > t.secs)) {
      t.window = i;
      t.secs = secs[i];
      t.cells = cells[i];
      t.dist = d;
      best_dist = rd;
    }
  }
  return t;
}

// ---- the lock of the Tests page ---------------------------------------------------------------------------------
// Locked, the page stays on screen: no 15 s idle close, no screen-timeout close, BACK does nothing until it is
// unlocked. It never wins over the end of the drive or over ANY alert the road view would show (driver monitoring,
// controls alerts, the crash log, controls unresponsive): those take the screen at once, as without the lock. The page
// comes back by itself, still locked, once no alert has shown for REOPEN_CLEAR_TICKS. The state is a tmpfs param
// (TiTestLock): it survives engaging and disengaging and a drive ending, not a reboot.
constexpr int UI_TICK_HZ = 20;
constexpr int REOPEN_CLEAR_TICKS = 2 * UI_TICK_HZ;   // 2 s without an alert

enum class PageDecision { Stay, Leave };

inline PageDecision pageDecision(bool locked, bool started, bool alert, bool idle_expired) {
  if (!started || alert) {
    return PageDecision::Leave;       // the end of the drive and every alert win, locked or not
  }
  if (idle_expired && !locked) {
    return PageDecision::Leave;
  }
  return PageDecision::Stay;
}

// BACK (and any other way of leaving by a tap) is ignored while locked
inline bool backAllowed(bool locked) {
  return !locked;
}

// The screen-timeout close of window.cc: skipped while locked
inline bool timeoutMayClose(bool locked) {
  return !locked;
}

class LockReopen {
public:
  // One call per UI update. True when the locked page should open again now: it is locked, the driving screen is
  // showing, the drive is on, the Tests page is allowed, and no alert has shown for REOPEN_CLEAR_TICKS.
  bool step(bool locked, bool home_showing, bool started, bool allowed, bool alert) {
    if (alert || !started) {
      clear_ = 0;
      return false;
    }
    if (clear_ < REOPEN_CLEAR_TICKS) {
      clear_++;
    }
    return locked && home_showing && allowed && clear_ >= REOPEN_CLEAR_TICKS;
  }
  void reset() { clear_ = 0; }
private:
  int clear_ = 0;
};

}  // namespace ti_logic
