// Standalone test of selfdrive/ui/qt/ti_tests_logic.h (no Qt): windows, zones, the target choice and the lock.
// Built and run by selfdrive/car/tests/test_mazda_ti_screen_logic.py (g++ or MSVC); prints "ok" and exits 0 on success.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "selfdrive/ui/qt/ti_tests_logic.h"

using namespace ti_logic;

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while (0)

static void zones() {
  struct Case { double mph; ZoneState state; int window; };
  const Case cases[] = {
    {18.9, ZoneState::Below, -1}, {19.0, ZoneState::Between, 0}, {20.9, ZoneState::Between, 0}, {21.0, ZoneState::Inside, 0},
    {26.99, ZoneState::Inside, 0}, {27.0, ZoneState::Between, 0}, {27.99, ZoneState::Between, 0},
    {28.0, ZoneState::Transition, -1}, {30.99, ZoneState::Transition, -1}, {31.0, ZoneState::Between, 1},
    {34.9, ZoneState::Between, 1}, {35.0, ZoneState::Inside, 1}, {41.0, ZoneState::Between, 1}, {44.9, ZoneState::Between, 1},
    {45.1, ZoneState::Between, 2}, {49.0, ZoneState::Inside, 2}, {56.0, ZoneState::Between, 2}, {59.9, ZoneState::Between, 2},
    {60.1, ZoneState::Between, 3}, {64.0, ZoneState::Inside, 3}, {79.9, ZoneState::Inside, 3}, {80.0, ZoneState::Above, -1},
  };
  for (const Case &c : cases) {
    const Zone z = zoneFor(c.mph);
    if (z.state != c.state || z.window != c.window) {
      std::printf("FAIL zone at %.2f mph\n", c.mph);
      failures++;
    }
  }
  // no window straddles the EPS region
  for (int i = 0; i < N_WINDOWS; i++) {
    CHECK(WINDOWS[i].high <= EPS_LOW_MPH || WINDOWS[i].low >= EPS_HIGH_MPH);
  }
  CHECK(windowTargetMph(0) == 24 && windowTargetMph(1) == 38 && windowTargetMph(2) == 52 && windowTargetMph(3) == 72);
}

static void targets() {
  double secs[N_WINDOWS] = {100, 100, 100, 100};
  int cells[N_WINDOWS] = {5, 5, 5, 5};
  Target t = chooseTarget(52.0, secs, cells);
  CHECK(t.window == 2 && t.dist == 0.0);
  t = chooseTarget(30.0, secs, cells);        // 3 below 35? no: 3 above 27 vs 5 below 35 -> the lowest window
  CHECK(t.window == 0 && std::lround(t.dist) == 3);
  t = chooseTarget(33.0, secs, cells);
  CHECK(t.window == 1);
  // 60 mph: 4 above 49-56 and 4 below 64-80: the one with more outstanding wins
  secs[2] = 50;
  secs[3] = 90;
  t = chooseTarget(60.0, secs, cells);
  CHECK(t.window == 3);
  secs[2] = 95;
  t = chooseTarget(60.0, secs, cells);
  CHECK(t.window == 2);
  // complete windows are not targets
  cells[2] = 0;
  t = chooseTarget(52.0, secs, cells);
  CHECK(t.window != 2 && t.window >= 0);
  for (int i = 0; i < N_WINDOWS; i++) {
    cells[i] = 0;
  }
  CHECK(chooseTarget(52.0, secs, cells).window == -1);
}

static void lock() {
  // unlocked: the old behaviour (idle close, end of drive, alert)
  CHECK(pageDecision(false, true, false, false) == PageDecision::Stay);
  CHECK(pageDecision(false, true, false, true) == PageDecision::Leave);
  CHECK(pageDecision(false, false, false, false) == PageDecision::Leave);
  CHECK(pageDecision(false, true, true, false) == PageDecision::Leave);
  // locked: no idle close; the end of the drive and every alert still win
  CHECK(pageDecision(true, true, false, true) == PageDecision::Stay);
  CHECK(pageDecision(true, true, false, false) == PageDecision::Stay);
  CHECK(pageDecision(true, false, false, false) == PageDecision::Leave);
  CHECK(pageDecision(true, true, true, false) == PageDecision::Leave);
  CHECK(pageDecision(true, true, true, true) == PageDecision::Leave);
  CHECK(backAllowed(false) && !backAllowed(true));
  CHECK(timeoutMayClose(false) && !timeoutMayClose(true));

  // reopening: only locked, home showing, drive on, allowed, and no alert for REOPEN_CLEAR_TICKS
  LockReopen r;
  for (int i = 0; i < REOPEN_CLEAR_TICKS - 1; i++) {
    CHECK(!r.step(true, true, true, true, false));
  }
  CHECK(r.step(true, true, true, true, false));
  CHECK(!r.step(false, true, true, true, false));        // unlocked: never
  CHECK(!r.step(true, false, true, true, false));        // another page is showing
  CHECK(!r.step(true, true, true, false, false));        // not allowed here
  CHECK(!r.step(true, true, false, true, false));        // drive over
  // an alert restarts the wait, whatever comes before
  LockReopen a;
  for (int i = 0; i < 100; i++) {
    a.step(true, true, true, true, false);
  }
  CHECK(!a.step(true, true, true, true, true));
  for (int i = 0; i < REOPEN_CLEAR_TICKS - 1; i++) {
    CHECK(!a.step(true, true, true, true, false));
  }
  CHECK(a.step(true, true, true, true, false));
  // an alert flickering every second never lets it reopen
  LockReopen f;
  for (int i = 0; i < 1000; i++) {
    CHECK(!f.step(true, true, true, true, i % UI_TICK_HZ == 0));
  }
  // engage / disengage does not touch the lock: it is not an input of the decision at all
}

int main() {
  zones();
  targets();
  lock();
  if (failures == 0) {
    std::printf("ok\n");
    return 0;
  }
  return 1;
}
