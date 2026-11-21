// Standalone test of selfdrive/ui/qt/ti_ab_logic.h (no Qt): the allow-list, stepping B and the block length, pins and texts.
// Built and run by selfdrive/car/tests/test_mazda_ti_ab_screen_logic.py (g++ or MSVC); prints "ok" and exits 0 on success.
#include <cstdio>
#include <cstring>

#include "selfdrive/ui/qt/ti_ab_logic.h"

using namespace ti_ab;

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while (0)

static void table() {
  CHECK(settingIndex("SteerKP") == 0 && settingIndex("LatDamping") == 2 && settingIndex("TiSteerMax") == -1);
  CHECK(settingIndex("ForceAutoTune") == -1 && settingIndex("TiSteerThreshold") == -1);
  for (int i = 0; i < N_SETTINGS; i++) {
    CHECK(SETTINGS[i].lo < SETTINGS[i].hi && SETTINGS[i].max_step > 0 && SETTINGS[i].max_step <= SETTINGS[i].hi - SETTINGS[i].lo + 1e-9);
    CHECK(cycleSetting(cycleSetting(i, 1), -1) == i);
  }
  CHECK(cycleSetting(N_SETTINGS - 1, 1) == 0 && cycleSetting(0, -1) == N_SETTINGS - 1);
}

static void steps() {
  const Setting &kp = SETTINGS[0];
  // stored 0.80: B walks 0.85, 0.90 and stops at the largest step (0.10)
  double b = defaultB(kp, 0.80);
  CHECK(std::fabs(b - 0.85) < 1e-9);
  for (int i = 0; i < 10; i++) {
    b = stepB(kp, 0.80, b, +1);
  }
  CHECK(std::fabs(b - 0.90) < 1e-9);
  CHECK(bAllowed(kp, 0.80, b) && !bAllowed(kp, 0.80, 0.95) && !bAllowed(kp, 0.80, 1.10));
  // downward: it jumps over A (0.80) and stops at 0.70
  for (int i = 0; i < 12; i++) {
    b = stepB(kp, 0.80, b, -1);
  }
  CHECK(std::fabs(b - 0.70) < 1e-9 && bAllowed(kp, 0.80, b));
  // at the top of the range the first offer is below A
  CHECK(std::fabs(defaultB(kp, 1.00) - 0.95) < 1e-9 && !bAllowed(kp, 0.80, 0.65));
  CHECK(!bAllowed(kp, 1.20, 1.0) && !bAllowed(kp, 0.60, 0.65));                // a stored value outside the range is refused by the car side
  CHECK(!bAllowed(kp, 0.80, 0.80) && !bAllowed(kp, 0.80, 0.82));
  // never offers what the car side refuses
  for (double a = 0.70; a <= 1.00 + 1e-9; a += 0.05) {
    double x = defaultB(kp, a);
    CHECK(bAllowed(kp, a, x));
    for (int d = -1; d <= 1; d += 2) {
      for (int i = 0; i < 20; i++) {
        x = stepB(kp, a, x, d);
        CHECK(bAllowed(kp, a, x));
      }
    }
  }
  // bools: B is the opposite of A and does not move
  const Setting &damp = SETTINGS[2];
  CHECK(defaultB(damp, 0) == 1 && defaultB(damp, 1) == 0 && stepB(damp, 0, 1, +1) == 1 && stepB(damp, 1, 0, -1) == 0);
  CHECK(bAllowed(damp, 0, 1) && bAllowed(damp, 1, 0) && !bAllowed(damp, 0, 0) && !bAllowed(damp, 1, 1) && !bAllowed(damp, 0, 0.5));
}

static void blocks() {
  CHECK(stepBlock(4, +1) == 5 && stepBlock(5, -1) == 4 && stepBlock(4, -1) == 4 && stepBlock(10, +1) == 10);
  CHECK(DEFAULT_BLOCK_MIN >= MIN_BLOCK_MIN && DEFAULT_BLOCK_MIN <= MAX_BLOCK_MIN);
}

static void pins() {
  CHECK(tapPin(Pin::None, Pin::A) == Pin::A && tapPin(Pin::None, Pin::B) == Pin::B);
  CHECK(tapPin(Pin::A, Pin::B) == Pin::B && tapPin(Pin::B, Pin::A) == Pin::A);
  CHECK(tapPin(Pin::A, Pin::A) == Pin::None && tapPin(Pin::B, Pin::B) == Pin::None);
  CHECK(std::strcmp(pinText(Pin::A), "A") == 0 && std::strcmp(pinText(Pin::B), "B") == 0 && pinText(Pin::None)[0] == 0);
  CHECK(canEdit(false) && !canEdit(true));
  CHECK(!pinBConfirmed(10.0, -1.0) && pinBConfirmed(11.0, 10.0) && pinBConfirmed(13.0, 10.0) && !pinBConfirmed(13.5, 10.0));
  CHECK(!pinBConfirmed(9.0, 10.0));
  CHECK(bendsOnly("LatCommitSetpoint") && !bendsOnly("LatDamping") && !bendsOnly("SteerKP"));
}

static void texts() {
  CHECK(std::strcmp(reasonText("curve"), "road bends") == 0 && std::strcmp(reasonText("touch"), "hands on the wheel") == 0);
  CHECK(std::strcmp(reasonText("nonsense"), "waiting") == 0);
  CHECK(statusFresh(100.0, 99.0) && statusFresh(100.0, 96.5) && !statusFresh(100.0, 95.0) && !statusFresh(100.0, 200.0));
  CHECK(!statusFresh(NAN, 1.0) && !statusFresh(1.0, NAN));
}

static void plan() {
  const std::string a = planLine("ab", 150.0, 90.0, 20.0, false);
  CHECK(a.find("A/B alternation") != std::string::npos && a.find("A 2.5 / B 1.5 of 20 min") != std::string::npos);
  CHECK(a.find("next: tests") != std::string::npos && a.find("COMPLETE") == std::string::npos);
  const std::string t = planLine("tests", 1200.0, 1200.0, 20.0, true);
  CHECK(t.find("identification tests") != std::string::npos && t.find("COMPLETE") != std::string::npos && t.find("A/B is complete") != std::string::npos);
  CHECK(planLine("hold", 0, 0, 20.0, false).find("handing over") != std::string::npos);
  CHECK(planLine("dead", 0, 0, 20.0, false).find("stopped after an error") != std::string::npos);
  CHECK(planLine("off", 0, 0, 20.0, false) == "Drive plan off");
  CHECK(PLAN_TARGET_MIN_DEFAULT == 20.0 && PLAN_ALT_MIN_MPH == 35.0 && PLAN_ALT_MAX_MPH == 56.0);
}

static void holds() {
  const char *whys[] = {"plan dead", "plan missing", "plan unreadable", "plan stale", "alternation not at A", "plan says ab",
                        "plan says hold", "plan says off", "A/B switch on"};
  for (const char *w : whys) {
    CHECK(holdText(w)[0] != 0);
    CHECK(std::strstr(holdText(w), "another test") == nullptr);
  }
  CHECK(std::strcmp(holdText("plan dead"), "Held: drive plan stopped after an error; restart needed") == 0);
  CHECK(std::strcmp(holdText("plan stale"), "Held: controls not reporting (stale)") == 0);
  CHECK(std::strcmp(holdText("alternation not at A"), "Waiting: A/B alternation returning to your setting") == 0);
  CHECK(std::strcmp(holdText("plan says ab"), "Waiting: drive plan is running A/B") == 0);
  CHECK(holdText("")[0] == 0 && holdText("something else")[0] == 0);
}

int main() {
  holds();
  plan();
  table();
  steps();
  blocks();
  pins();
  texts();
  if (failures == 0) {
    std::printf("ok\n");
    return 0;
  }
  return 1;
}
