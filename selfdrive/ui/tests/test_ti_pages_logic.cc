// Standalone test of selfdrive/ui/qt/ti_pages_logic.h (no Qt): layout, text fit, the label against what START runs, and the
// writes of START and STOP against those RUN ALL, DRIVE PLAN and ALL TESTS OFF made at d20104c.
// Built and run by selfdrive/car/tests/test_mazda_ti_pages_logic.py (g++ or MSVC); prints "ok" and exits 0 on success.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "selfdrive/ui/qt/ti_pages_logic.h"

using namespace ti_pages;

static int failures = 0;
#define CHECK(c) do { if (!(c)) { if (failures < 40) std::printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while (0)

static const Page PAGES[] = {Page::Home, Page::Setup, Page::Tests, Page::Overview, Page::Ab, Page::Progress};

static int minDim(const Rect &r) { return std::min(r.w, r.h); }

static void layoutRules() {
  for (Page pg : PAGES) {
    for (int errors : {0, 1, 2, 7}) {
      Control c[MAX_CONTROLS];
      const int n = layout(pg, errors, c);
      CHECK(n > 0 && n <= MAX_CONTROLS);
      for (int i = 0; i < n; i++) {
        CHECK(c[i].r.x >= 0 && c[i].r.y >= 0 && c[i].r.right() <= SCREEN_W && c[i].r.bottom() <= SCREEN_H);
        CHECK(c[i].r.w > 0 && c[i].r.h > 0);
        // the middle of a control is that control
        CHECK(controlAt(pg, errors, c[i].r.x + c[i].r.w / 2, c[i].r.y + c[i].r.h / 2) == c[i].id);
        for (int j = i + 1; j < n; j++) {
          CHECK(c[i].id != c[j].id);
          CHECK(!overlaps(c[i].r, c[j].r));
          CHECK(gapBetween(c[i].r, c[j].r) >= MIN_GAP);
          const bool stop = c[i].id == CTL_STOP || c[j].id == CTL_STOP || c[i].id == CTL_ALLOFF || c[j].id == CTL_ALLOFF;
          if (stop) {
            CHECK(gapBetween(c[i].r, c[j].r) >= STOP_GAP);
          }
        }
        if (c[i].id == CTL_LOCK || c[i].id == CTL_PIN_A || c[i].id == CTL_PIN_B) {
          CHECK(minDim(c[i].r) >= BIG_MIN);
        }
      }
    }
  }
}

static void homeRules() {
  Control c[MAX_CONTROLS];
  const int n = layout(Page::Home, 0, c);
  Rect start{}, stop{};
  bool have_start = false, have_stop = false;
  for (int i = 0; i < n; i++) {
    CHECK(minDim(c[i].r) >= BIG_MIN);                      // no small control on the first page
    if (c[i].id == CTL_START) { start = c[i].r; have_start = true; }
    if (c[i].id == CTL_STOP) { stop = c[i].r; have_stop = true; }
    if (c[i].id == CTL_BACK || c[i].id == CTL_NAV_SETUP || c[i].id == CTL_NAV_INDIV) {
      CHECK(c[i].r.x <= 20);                               // navigation at the screen edge
    }
  }
  CHECK(have_start && have_stop);
  CHECK(start.h * 4 >= SCREEN_H && stop.h * 4 >= SCREEN_H);
  CHECK(start.w * 10 >= SCREEN_W * 7 && stop.w * 10 >= SCREEN_W * 7);   // most of the width
  CHECK(start.x == stop.x && start.w == stop.w);
  // the text boxes of the page are not under a control
  const Rect boxes[] = {HOME_PRIMARY, HOME_SECOND, HOME_EXPERIMENT, HOME_DETAIL, HOME_DETAIL2, HOME_BADGE};
  for (const Rect &b : boxes) {
    CHECK(b.x >= 0 && b.right() <= SCREEN_W && b.bottom() <= SCREEN_H);
    for (int i = 0; i < n; i++) {
      CHECK(!overlaps(b, c[i].r));
    }
  }
  for (Page pg : {Page::Tests, Page::Overview, Page::Ab}) {
    Control d[MAX_CONTROLS];
    const int k = layout(pg, 0, d);
    for (int i = 0; i < k; i++) {
      CHECK(!overlaps(BANNER_RECT, d[i].r));
      if (d[i].id != CTL_PREV && d[i].id != CTL_NEXT) {
        CHECK(pg == Page::Tests ? true : !overlaps(TITLE_WIDE, d[i].r) || d[i].r.y > 190);
      }
      CHECK(pg != Page::Tests || !overlaps(TITLE_PREV_NEXT, d[i].r));
      CHECK(pg == Page::Tests || !overlaps(TITLE_WIDE, d[i].r));
    }
  }
  // the progress page's text boxes are not under a control and fit the screen
  Control pg_c[MAX_CONTROLS];
  const int pn = layout(Page::Progress, 0, pg_c);
  const Rect prog[] = {PROG_TITLE, PROG_AB_HEAD, PROG_AB_BAR_A, PROG_AB_BAR_B, PROG_AB_LEFT, PROG_SPLIT,
                       Rect{480, PROG_ROWS_TOP, 1660, 4 * PROG_ROW_H}};
  for (const Rect &b : prog) {
    CHECK(b.right() <= SCREEN_W && b.bottom() <= SCREEN_H);
    for (int i = 0; i < pn; i++) {
      CHECK(!overlaps(b, pg_c[i].r));
    }
  }
  // the A/B page: the status panel is not under a control, and the table (six rows of at least 30 px) ends above the page foot
  Control a[MAX_CONTROLS];
  const int m = layout(Page::Ab, 0, a);
  for (int i = 0; i < m; i++) {
    CHECK(!overlaps(AB_PANEL, a[i].r));
    CHECK(a[i].r.bottom() <= AB_TABLE_TOP);
  }
  CHECK(AB_TABLE_TOP + 6 * 30 <= SCREEN_H - 134);
}

// Width of a line at a font size, conservatively (bold capitals 0.75 em, mixed text 0.62 em)
static bool fits(const std::string &t, int px, double em, const Rect &box) {
  return t.size() * px * em <= box.w;
}

static void fontsFit() {
  CHECK(MAX_PRIMARY_CHARS * PRIMARY_PX * 0.75 <= HOME_PRIMARY.w);
  CHECK(MAX_SECOND_CHARS * SECOND_PX * 0.62 <= HOME_SECOND.w);
  CHECK(MAX_EXPERIMENT_CHARS * EXPERIMENT_PX * 0.62 <= HOME_EXPERIMENT.w);
  CHECK(MAX_DETAIL_CHARS * DETAIL_PX * 0.62 <= HOME_DETAIL.w && HOME_DETAIL.h >= DETAIL_PX + 4 && HOME_DETAIL2.h >= DETAIL_PX + 4);
  CHECK(MAX_PROGRESS_CHARS * PROGRESS_PX * 0.62 <= PROG_AB_HEAD.w && PROGRESS_PX + 4 <= PROG_AB_HEAD.h);
  Rect start{};
  rectOf(Page::Home, CTL_START, 0, start);
  CHECK(MAX_START_CHARS * START_PX * 0.70 <= start.w);
  CHECK(MAX_START_SUB_CHARS * START_SUB_PX * 0.62 <= start.w);
  CHECK(PRIMARY_PX >= 100 && SECOND_PX >= 60 && EXPERIMENT_PX >= 40 && DETAIL_PX >= 36);
  CHECK(PRIMARY_PX > SECOND_PX && SECOND_PX > EXPERIMENT_PX);
}

// The car side reports what the switches ask for (the states below test the wording, not the confirmation)
static void confirm(HomeState &s) {
  s.car_plan_on = true;
  s.live_fresh = s.live_all_on = s.live_tests_on = true;
}

static HomeLines H(HomeState s) {
  confirm(s);
  return home(s);
}

static void checkLines(const HomeState &s, int &count) {
  const HomeLines h = home(s);
  count++;
  CHECK(!h.primary.text.empty() && (int)h.primary.text.size() <= MAX_PRIMARY_CHARS);
  CHECK(!h.second.empty() && (int)h.second.size() <= MAX_SECOND_CHARS);
  CHECK(fits(h.primary.text, PRIMARY_PX, 0.75, HOME_PRIMARY));
  CHECK(fits(h.second, SECOND_PX, 0.62, HOME_SECOND));
  // stopped is the only state that says STOPPED, and it is grey
  CHECK((s.run == Run::Stopped) == (h.primary.tone == Tone::Stopped));
}

static void everyStateFits() {
  const char *plan_modes[] = {"", "off", "hold", "ab", "tests", "dead"};
  const char *holds[] = {"", "plan dead", "plan missing", "plan unreadable", "plan stale", "alternation not at A", "plan says ab",
                         "plan says hold", "plan says off", "A/B switch on", "unknown"};
  const char *gates[] = {"", "command", "curve", "lat"};
  const double speeds[] = {0, 10, 19, 22, 25, 29, 33, 38, 45, 52, 60, 72, 85, 120};
  int count = 0;
  // the tests side
  for (int run = 0; run < 5; run++)
    for (int conf = 0; conf < 2; conf++)
      for (int eng = 0; eng < 2; eng++)
        for (int have = 0; have < 2; have++)
          for (double mph : speeds)
            for (int metric = 0; metric < 2; metric++)
              for (const char *pm : plan_modes)
                for (const char *hw : holds)
                  for (int w = -1; w < ti_logic::N_WINDOWS; w++)
                    for (int active = 0; active < 2; active++)
                      for (const char *g : gates) {
                        HomeState s;
                        s.run = (Run)run; s.configured = conf; s.engaged = eng; s.have_speed = have; s.mph = mph; s.metric = metric;
                        s.plan_mode = pm; s.hold_why = hw; s.target = w >= 0; s.target_window = w; s.target_secs = 7200;
                        s.tests_active = active; s.gate_reason = g;
                        confirm(s); s.ab_fresh = true; s.ab_state = "running";
                        checkLines(s, count);
                      }
  // the alternation side
  const char *states[] = {"", "running", "pinned", "complete"};
  const char *arms[] = {"", "A", "B"};
  const char *reasons[] = {"input", "touch", "lane_change", "holdoff", "speed", "eps", "curve", "lat", "request", "saturated", "settle", "other"};
  for (int run = 0; run < 5; run++)
    for (const char *pm : plan_modes)
      for (int fresh = 0; fresh < 2; fresh++)
        for (const char *st : states)
          for (const char *arm : arms)
            for (int block = 0; block < 2; block++)
              for (int wait = 0; wait < 2; wait++)
                for (const char *r : reasons)
                  for (int complete = 0; complete < 2; complete++)
                    for (int metric = 0; metric < 2; metric++) {
                      HomeState s;
                      s.run = (Run)run; s.engaged = true; s.have_speed = true; s.mph = 45; s.metric = metric; s.plan_mode = pm;
                      s.ab_fresh = fresh; s.ab_state = st; s.ab_arm = arm; s.ab_block = block; s.ab_block_left_s = 599;
                      s.ab_wait = wait; s.ab_wait_reason = r; s.ab_wait_s = 59; s.ab_wait_max_s = 60; s.ab_complete = complete;
                      confirm(s);
                      checkLines(s, count);
                    }
  CHECK(count > 100000);
}

static void sampleStates() {
  HomeState s;
  s.run = Run::Plan; s.engaged = true; s.have_speed = true; s.mph = 24; s.plan_mode = "tests"; s.target = true; s.target_window = 0;
  s.target_secs = 100; s.tests_active = true; s.configured = true;
  CHECK(H(s).primary.text == "TESTING: HOLD 24 MPH" && H(s).primary.tone == Tone::Running && H(s).second == "1:40 to go here");
  s.mph = 38;                                    // inside another window: it counts, but not the one asked for
  CHECK(H(s).primary.text == "HOLD 24 MPH" && H(s).primary.tone == Tone::Waiting && H(s).second == "Now 38 mph");
  s.mph = 30;                                    // where nothing counts: said plainly, with the nearest speeds that do
  CHECK(H(s).primary.text == "NO COUNT AT 30 MPH" && H(s).primary.tone == Tone::Waiting && H(s).second == "Hold 24 or 38 mph");
  s.mph = 45;
  CHECK(H(s).primary.text == "NO COUNT AT 45 MPH" && H(s).second == "Hold 38 or 52 mph");
  s.mph = 17;
  CHECK(H(s).primary.text == "NO COUNT AT 17 MPH" && H(s).second == "Hold 24 mph");
  s.mph = 85;
  CHECK(H(s).primary.text == "NO COUNT AT 85 MPH" && H(s).second == "Hold 72 mph");
  s.mph = 24;
  s.mph = 24; s.tests_active = false; s.gate_reason = "curve";
  CHECK(H(s).primary.text == "WAITING: STRAIGHT ROAD");
  s = HomeState(); s.run = Run::Plan; s.engaged = true; s.plan_mode = "ab"; s.ab_fresh = true; s.ab_state = "running"; s.ab_arm = "B";
  s.ab_block = true; s.ab_block_left_s = 252;
  CHECK(H(s).primary.text == "A/B RUNNING: B" && H(s).second == "4:12 left in block");
  s.ab_block = false; s.ab_complete = true;
  CHECK(H(s).primary.text == "DONE: A/B COMPLETE" && H(s).primary.tone == Tone::Done);
  s = HomeState(); s.run = Run::Plan; s.engaged = true; s.plan_mode = "tests"; s.have_speed = true; s.mph = 45; s.configured = true;
  CHECK(H(s).primary.text == "WAITING: A/B STARTING");
  s.mph = 25;
  CHECK(H(s).primary.text == "A/B: HOLD 38 MPH");
  s.ab_complete = true;
  CHECK(H(s).primary.text == "DONE: ALL COMPLETE");
  s = HomeState();
  CHECK(H(s).primary.text == "STOPPED: TAP START" && H(s).second == "Nothing is running");
  s.configured = true; s.ab_stored_known = true; s.ab_stored_a_s = 14.7 * 60; s.ab_stored_b_s = 13.2 * 60;
  CHECK(H(s).primary.text == "STOPPED: TAP START" && H(s).second == "A/B 13 of 20 min each side done");
  s.ab_stored_known = false;
  CHECK(H(s).second == "Nothing is running");
  s.run = Run::All;
  CHECK(H(s).primary.text == "WAITING: ENGAGE");
  // km/h on a metric device
  s = HomeState(); s.run = Run::All; s.engaged = true; s.have_speed = true; s.mph = 72; s.metric = true; s.target = true; s.target_window = 3;
  CHECK(H(s).primary.text == "TESTING: HOLD 116 KM/H" || H(s).primary.text == "WAITING: HOLD 116 KM/H");
}

static void carReportRules() {
  // Whatever the switches say, "running" appears only when the car side reports it. Every combination of the switch state and the
  // car's own reports: the page never shows a running tone (or a RUNNING label) unless carRunning, and a switch that the car does
  // not confirm reads "ON BUT NOT RUNNING"; nothing asked for reads STOPPED.
  const char *plan_modes[] = {"", "off", "hold", "ab", "tests", "dead"};
  const char *states[] = {"", "off", "idle", "running", "pinned", "complete", "error"};
  int combos = 0, confirmed = 0, shown_running = 0;
  for (int run = 0; run < 5; run++)
    for (const char *pm : plan_modes)
      for (const char *st : states)
        for (int bits = 0; bits < 64; bits++) {
          HomeState s;
          s.run = (Run)run; s.plan_mode = pm; s.ab_state = st; s.ab_arm = "B";
          s.ab_fresh = bits & 1; s.car_plan_on = bits & 2; s.live_fresh = bits & 4; s.live_all_on = bits & 8; s.live_tests_on = bits & 16;
          s.engaged = true; s.have_speed = true; s.mph = 24; s.configured = true; s.target = true; s.target_window = 0; s.target_secs = 60;
          s.tests_active = bits & 32; s.ab_block = true; s.ab_block_left_s = 100;
          const HomeLines h = home(s);
          const StartLook k = startLook(true, s.run, carRunning(s));
          combos++;
          confirmed += carRunning(s);
          if (h.primary.tone == Tone::Running) {
            shown_running++;
            CHECK(carRunning(s));
          }
          if (k.label.rfind("RUNNING", 0) == 0) {
            CHECK(carRunning(s));
          }
          if (s.run == Run::Stopped) {
            CHECK(h.primary.text == "STOPPED: TAP START" && !carRunning(s));
          } else if (!carRunning(s)) {
            CHECK(h.primary.text == "ON BUT NOT RUNNING" && h.primary.tone == Tone::Held && k.enabled);
          }
        }
  CHECK(combos > 5000 && confirmed > 100 && shown_running > 50);
  // a stored ON with the car side silent (controls restarted, or its report stopped): never running
  HomeState s;
  s.run = Run::Plan; s.plan_mode = "ab"; s.engaged = true;
  CHECK(home(s).primary.text == "ON BUT NOT RUNNING" && startLook(true, Run::Plan, false).label == "START: tests + A/B relay" && startLook(true, Run::Plan, false).enabled);
  s.ab_fresh = true; s.car_plan_on = false;                       // controls reports the plan switch OFF: the screen's switch is stale
  CHECK(home(s).primary.text == "ON BUT NOT RUNNING");
  s.car_plan_on = true;
  CHECK(home(s).primary.tone != Tone::Held);
}

static void wordsCoverTheCarSide() {
  for (int i = 0; i < ti_ab::N_REASONS; i++) {
    CHECK(waitShort(ti_ab::REASONS[i].name, false) != "WAITING");
    CHECK((int)waitShort(ti_ab::REASONS[i].name, false).size() <= MAX_PRIMARY_CHARS);
    CHECK((int)waitShort(ti_ab::REASONS[i].name, true).size() <= MAX_PRIMARY_CHARS);
  }
  // every hold reason the plain-words table of the A/B logic words has a short form, and no other
  for (int i = 0; i < N_HOLD_SHORT; i++) {
    CHECK(ti_ab::holdText(HOLD_SHORT[i].why)[0] != 0);
    CHECK((int)std::strlen(HOLD_SHORT[i].text) <= MAX_PRIMARY_CHARS);
  }
  CHECK(N_HOLD_SHORT == 9 && holdShort("nonsense") == nullptr && ti_ab::holdText("nonsense")[0] == 0);
  CHECK(std::strstr(ti_ab::REASONS[4].text, "36-69") != nullptr && SPEED_GATE_LOW_MPH == 36 && SPEED_GATE_HIGH_MPH == 69);
  CHECK(std::strstr(ti_ab::REASONS[5].text, "28-31") != nullptr && EPS_LOW_MPH == 28 && EPS_HIGH_MPH == 31);
  CHECK(EPS_LOW_MPH == (int)ti_logic::EPS_LOW_MPH && EPS_HIGH_MPH == (int)ti_logic::EPS_HIGH_MPH);
}

static void experimentWords() {
  const int relay = ti_ab::settingIndex("LatNoFrictionRelay");
  // the stored value of LatNoFrictionRelay is "no relay": 1 means the relay is off
  CHECK(experimentLine(relay, true, 1, 0) == "A/B: friction relay, yours (off) against on");
  CHECK(experimentLine(relay, true, 0, 1) == "A/B: friction relay, yours (on) against off");
  CHECK(experimentLine(ti_ab::settingIndex("LatDamping"), true, 0, 1) == "A/B: damping, yours (off) against on");
  CHECK(experimentLine(0, true, 0.80, 0.85) == "A/B: steering gain, yours (0.80) against 0.85");
  CHECK(experimentLine(-1, false, 0, 0) == "A/B: not used (choose one on SET UP A/B)");
  for (int i = 0; i < ti_ab::N_SETTINGS; i++) {
    CHECK(std::strcmp(plainName(ti_ab::SETTINGS[i].name), ti_ab::SETTINGS[i].name) != 0);
    CHECK((int)experimentLine(i, true, 0.80, 0.85).size() <= MAX_EXPERIMENT_CHARS);
    CHECK((int)experimentLine(i, false, 0, 0).size() <= MAX_EXPERIMENT_CHARS);
  }
  CHECK((int)experimentLine(-1, false, 0, 0).size() <= MAX_EXPERIMENT_CHARS);
}

// The writes of d20104c, transcribed from sidebar.cc: TiTests::allOff, TiTests::startAll, abAct control 39 (DRIVE PLAN on),
// abSetOn(false)
static std::vector<Op> legacyAllTestsOff() {
  return {memBool("TiPlanOn", false), simple(OpKind::AllowedOff), simple(OpKind::UpdateToggles)};
}
static std::vector<Op> legacyRunAll() {
  return {memBool("TiPlanOn", false), memBool("TiAbOn", false), memRemove("TiAbPin"), simple(OpKind::RunAllOn),
          simple(OpKind::SingleStepTestsOff), simple(OpKind::UpdateToggles)};
}
static std::vector<Op> legacyDrivePlanOn() {
  std::vector<Op> ops = legacyAllTestsOff();       // tests.allOff();
  ops.push_back(simple(OpKind::SaveAbConfig));     // abSave();
  ops.push_back(memBool("TiAbOn", false));
  ops.push_back(memRemove("TiAbPin"));
  ops.push_back(memBool("TiPlanOn", true));
  return ops;
}
static std::vector<Op> legacyAbOff() {
  return {memBool("TiPlanOn", false), memBool("TiAbOn", false), memRemove("TiAbPin")};
}

static bool isSubsequence(const std::vector<Op> &part, const std::vector<Op> &whole) {
  size_t k = 0;
  for (const Op &o : whole) {
    if (k < part.size() && part[k] == o) {
      k++;
    }
  }
  return k == part.size();
}

static void writes() {
  CHECK(allTestsOffOps() == legacyAllTestsOff());
  CHECK(runAllOps() == legacyRunAll());
  CHECK(drivePlanOps() == legacyDrivePlanOn());
  // START: with an alternation set up exactly the DRIVE PLAN writes, without one exactly RUN ALL's
  CHECK(startOps(true) == legacyDrivePlanOn());
  CHECK(startOps(false) == legacyRunAll());
  // STOP: ALL TESTS OFF, and the plain switch off with its pin: it includes what both old red boxes did, and writes nothing else
  const std::vector<Op> stop = stopOps();
  CHECK(isSubsequence(legacyAllTestsOff(), stop) && isSubsequence(legacyAbOff(), stop));
  CHECK(stop.size() == 5);
  for (const Op &o : stop) {
    CHECK(o.kind != OpKind::MemBool || o.value == false);       // nothing is switched on by STOP
    CHECK(o.kind != OpKind::RunAllOn && o.kind != OpKind::SaveAbConfig);
  }
  // the plan start switches the plan on exactly once, last; RUN ALL never writes the plan switch on; the plan never run-all on
  const std::vector<Op> plan = startOps(true);
  int plan_on_writes = 0;
  for (const Op &o : plan) {
    if (o.kind == OpKind::MemBool && std::strcmp(o.key, "TiPlanOn") == 0 && o.value) {
      plan_on_writes++;
    }
    CHECK(o.kind != OpKind::RunAllOn);
  }
  CHECK(plan_on_writes == 1 && plan.back() == memBool("TiPlanOn", true));
  for (const Op &o : startOps(false)) {
    CHECK(!(o.kind == OpKind::MemBool && std::strcmp(o.key, "TiPlanOn") == 0 && o.value));
  }
}

static void startLabels() {
  for (int conf = 0; conf < 2; conf++) {
    for (int run = 0; run < 5; run++) {
      const StartLook k = startLook(conf, (Run)run);
      CHECK((int)k.label.size() <= MAX_START_CHARS && (int)k.sub.size() <= MAX_START_SUB_CHARS);
      CHECK((k.label.find("A/B relay") != std::string::npos) == (conf == 1));      // the label says what will run
      CHECK(k.label.rfind("START: ", 0) == 0 || k.label.rfind("RUNNING: ", 0) == 0);
      CHECK((k.label.rfind("START: ", 0) == 0) == k.enabled);
      if (k.enabled) {
        const std::vector<Op> ops = startOps(conf);
        CHECK((k.label.find("A/B relay") != std::string::npos) == (ops == legacyDrivePlanOn()));
        CHECK((k.label.find("A/B relay") == std::string::npos) == (ops == legacyRunAll()));
      }
    }
  }
  CHECK(startLook(true, Run::Stopped).label == "START: tests + A/B relay" && startLook(false, Run::Stopped).label == "START: tests");
  // a single test or the plain switch is replaced, and the screen says so
  CHECK(startLook(false, Run::Single).enabled && startLook(false, Run::Single).sub.find("single test") != std::string::npos);
  CHECK(startLook(true, Run::AbPlain).enabled && startLook(true, Run::AbPlain).sub.find("A/B switch") != std::string::npos);
  CHECK(!startLook(true, Run::Plan).enabled && !startLook(false, Run::All).enabled);
  CHECK(startLook(true, Run::All).enabled);   // run-all with an alternation set up: START moves to the plan
}

static void progressTexts() {
  CHECK(abProgressText(14.2 * 60, 13.9 * 60, 20) == "A 14 / 20 min, B 13 / 20 min");
  CHECK(abMinutesLeft(14 * 60, 13 * 60, 20) == 13.0 && abMinutesLeft(25 * 60, 21 * 60, 20) == 0.0);
  CHECK(abLeftText(14 * 60, 13 * 60, 20) == "About 13 min of driving at 36-55 mph left");
  CHECK(abLeftText(1200, 1200, 20) == "A/B complete: the tests take over");
  CHECK(fraction(10, 20) == 0.5 && fraction(30, 20) == 1.0 && fraction(-1, 20) == 0.0 && fraction(1, 0) == 1.0);
  CHECK(windowLabel(0, false) == "21-27 mph" && windowRowText(0, 3, 12, 540, false) == "3 of 12 done; hold 24 mph for about 9 min");
  CHECK(windowRowText(1, 12, 12, 0, false) == "12 of 12 done" && windowRowText(1, 0, 0, 0, false) == "no test needs this window");
  CHECK(windowRowText(3, 0, 12, 1, true) == "0 of 12 done; hold 116 km/h for about 1 min");
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    for (int m = 0; m < 2; m++) {
      CHECK((int)windowRowText(w, 11, 999, 99999, m).size() <= MAX_PROGRESS_CHARS);
      CHECK((int)(windowLabel(w, m) + "  ").size() <= 20);
    }
  }
  CHECK((int)abProgressText(99999, 99999, 20).size() <= MAX_PROGRESS_CHARS && (int)abLeftText(0, 0, 20).size() <= MAX_PROGRESS_CHARS);
  // this drive: every combination fits the detail line
  const double none[4] = {0, 0, 0, 0};
  CHECK(thisDriveLine(none, 0, 0, true, false) == "Since START: nothing counted yet");
  const double some[4] = {95, 0, 40, 0};
  CHECK(thisDriveLine(some, 240, 210, true, false) == "Since START: tests 21-27 +95 s, 49-56 +40 s; A/B +4.0 / +3.5 min");
  CHECK(thisDriveLine(some, 0, 0, false, false) == "Since START: tests 21-27 +95 s, 49-56 +40 s");
  const double big[4] = {9999, 9999, 9999, 9999};
  for (int ab = 0; ab < 2; ab++) {
    for (int m = 0; m < 2; m++) {
      CHECK((int)thisDriveLine(big, 99999, 99999, ab, m).size() <= MAX_DETAIL_CHARS);
      CHECK((int)thisDriveLine(some, 12345, 54321, ab, m).size() <= MAX_DETAIL_CHARS);
    }
  }
  CHECK(countsNothingAt(30) && countsNothingAt(33) && countsNothingAt(17) && countsNothingAt(80) && !countsNothingAt(24) && !countsNothingAt(70));
  CHECK(nearestCounting(30, false) == "Hold 24 or 38 mph");
}

static void setupOrderAndChoice() {
  // on/off settings first, the number last; every setting appears once; stepping wraps and never leaves the list
  bool seen[ti_ab::N_SETTINGS] = {false};
  for (int i = 0; i < N_SETUP_ORDER; i++) {
    const int idx = ti_ab::settingIndex(SETUP_ORDER[i]);
    CHECK(idx >= 0 && !seen[idx]);
    seen[idx] = true;
    CHECK(i == N_SETUP_ORDER - 1 ? !ti_ab::SETTINGS[idx].is_bool : ti_ab::SETTINGS[idx].is_bool);
  }
  CHECK(N_SETUP_ORDER == ti_ab::N_SETTINGS && std::strcmp(SETUP_ORDER[0], "LatNoFrictionRelay") == 0 && std::strcmp(SETUP_ORDER[N_SETUP_ORDER - 1], "SteerKP") == 0);
  int idx = ti_ab::settingIndex(SETUP_ORDER[0]);
  for (int i = 0; i < N_SETUP_ORDER; i++) {
    CHECK(std::strcmp(ti_ab::SETTINGS[idx].name, SETUP_ORDER[i]) == 0);
    idx = setupStep(idx, 1);
  }
  CHECK(std::strcmp(ti_ab::SETTINGS[idx].name, SETUP_ORDER[0]) == 0);
  CHECK(setupStep(setupStep(idx, 1), -1) == idx && std::strcmp(ti_ab::SETTINGS[setupStep(idx, -1)].name, "SteerKP") == 0);
  CHECK(notRecommended("SteerKP") && !notRecommended("LatNoFrictionRelay") && (int)std::strlen(NOT_RECOMMENDED_TEXT) <= MAX_DETAIL_CHARS);
  // the chosen experiment fits its large line, for every setting and every kind of value
  CHECK(MAX_CHOSEN_CHARS * CHOSEN_PX * 0.62 <= HOME_EXPERIMENT.w && CHOSEN_PX + 4 <= HOME_EXPERIMENT.h);
  for (int i = 0; i < ti_ab::N_SETTINGS; i++) {
    CHECK((int)experimentLine(i, true, 0.80, 0.85).size() <= MAX_CHOSEN_CHARS && (int)experimentLine(i, false, 0, 0).size() <= MAX_CHOSEN_CHARS);
  }
  CHECK((int)experimentLine(-1, false, 0, 0).size() <= MAX_CHOSEN_CHARS);
  // the split of the speed range
  CHECK(splitNote(true, false) == "A/B has 36-55 mph until done; tests run below 35 and above 56 mph");
  CHECK(splitNote(true, true).size() * DETAIL_PX * 0.62 <= PROG_SPLIT.w && splitNote(false, false).size() * DETAIL_PX * 0.62 <= PROG_SPLIT.w);
  CHECK(PROG_SPLIT.bottom() <= SCREEN_H && PROG_SPLIT.h >= DETAIL_PX + 4);
}

static PlanFields goodPlan() {
  PlanFields p;
  p.named = true;
  p.test_ids = {"ti_sweep", "ti_pulses", "ti_ladder"};
  p.windows = {"21-27", "35-41", "49-56", "64-80"};
  p.has_alt = true; p.setting = "LatNoFrictionRelay"; p.a = "stored"; p.b_text = "other";
  p.block_min = 4; p.target_min = 20; p.min_mph = 35; p.max_mph = 56;
  return p;
}

static void planAndBands() {
  PlanFields p = goodPlan();
  CHECK(planProblem(p).empty());
  struct Case { const char *why; void (*mutate)(PlanFields &); };
  const Case cases[] = {
    {"setting is not allowed", [](PlanFields &x) { x.setting = "TiSteerMax"; }},
    {"A must be your stored value", [](PlanFields &x) { x.a = "0.8"; }},
    {"B must be the other value", [](PlanFields &x) { x.b_text = "1"; }},
    {"block length outside 4-10 min", [](PlanFields &x) { x.block_min = 3; }},
    {"target must be 20 min per side", [](PlanFields &x) { x.target_min = 30; }},
    {"speed range must be 35-56 mph", [](PlanFields &x) { x.max_mph = 70; }},
    {"tests must cover all four windows", [](PlanFields &x) { x.windows = {"21-27"}; }},
    {"tests must be sweep, pulses, ladder", [](PlanFields &x) { x.test_ids = {"ti_dither"}; }},
    {"no plan name", [](PlanFields &x) { x.named = false; }},
  };
  for (const Case &c : cases) {
    PlanFields q = goodPlan();
    c.mutate(q);
    CHECK(planProblem(q) == c.why);
    CHECK(std::strlen(c.why) <= 40);
    CHECK((int)planLine(false, planProblem(q), true, "LatNoFrictionRelay").size() <= MAX_CHOSEN_CHARS);
  }
  PlanFields kp = goodPlan();
  kp.setting = "SteerKP"; kp.b_is_number = true; kp.b = 0.85;
  CHECK(planProblem(kp).empty());
  kp.b = 1.2;
  CHECK(planProblem(kp) == "B is outside 0.70-1.00");
  p.has_alt = false;
  CHECK(planProblem(p).empty() && planLine(true, "", false, "") == "PLAN: tests");
  CHECK(planLine(true, "", true, "LatNoFrictionRelay") == "PLAN: tests + friction relay A/B");
  for (int i = 0; i < ti_ab::N_SETTINGS; i++) {
    CHECK((int)planLine(true, "", true, ti_ab::SETTINGS[i].name).size() <= MAX_CHOSEN_CHARS);
  }
  CHECK(startLook(true, Run::Stopped, true, false).label == "PLAN INVALID" && !startLook(true, Run::Stopped, true, false).enabled);
  CHECK(startLook(true, Run::Stopped, true, true).enabled);

  // the bands: one figure each, the alternation and the tests merged
  const double left[4] = {6.0 * 60 / 1.3, 4.0 * 60 / 1.3, 3.0 * 60 / 1.3, 4.0 * 60 / 1.3};
  Band b[N_BANDS];
  bandsFor(left, true, 0, 0, 20, 45.0, true, b);
  CHECK(b[0].target_mph == 24 && b[2].target_mph == 72 && b[1].target_mph == 45 && b[0].minutes == 6 && b[2].minutes == 4);
  CHECK(b[1].minutes == std::ceil(40 * OVERHEAD_ALT + 7.0 - 1e-9));        // alternation 40 min, both sides still open, plus the two windows
  CHECK(!b[0].here && b[1].here && !b[2].here && !b[1].done);
  bandsFor(left, false, 0, 0, 20, 24.0, true, b);
  CHECK(b[1].minutes == 7 && b[0].here);
  const double none[4] = {0, 0, 0, 0};
  bandsFor(none, true, 1200, 1200, 20, 0, false, b);
  CHECK(b[0].done && b[1].done && b[2].done && bandsTotalMinutes(b) == 0 && !b[0].here);
  CHECK(bandText(b[0], false) == "24 mph  done");
  bandsFor(left, true, 600, 300, 20, 70, true, b);
  CHECK(b[2].here && bandText(b[2], false) == "72 mph  4 min" && bandRange(b[2], false) == "64-80 mph");
  for (int m = 0; m < 2; m++) {
    for (int i = 0; i < N_BANDS; i++) {
      CHECK((int)bandText(b[i], m).size() <= 24 && (int)bandRange(b[i], m).size() <= 24);
    }
    const double big[4] = {9e4, 9e4, 9e4, 9e4};
    bandsFor(big, true, 0, 0, 20, 45, true, b);
    CHECK((int)budgetLine(b, m).size() <= 128 && (int)bandText(b[1], m).size() <= 24);
  }
  bandsFor(left, true, 0, 0, 20, 45, true, b);
  CHECK(budgetLine(b, false).rfind("PLAN NEEDS ABOUT ", 0) == 0);
}

static void setup() {
  CHECK(setupEditable(false, false, 0) && setupEditable(false, true, 0.5) && !setupEditable(false, true, 25) && !setupEditable(true, true, 0));
  CHECK((int)setupNote(false, true, 0).size() <= MAX_EXPERIMENT_CHARS && (int)setupNote(true, true, 0).size() <= MAX_EXPERIMENT_CHARS);
  CHECK((int)individualNote(Run::Plan).size() <= MAX_EXPERIMENT_CHARS);
  CHECK((int)detailScored(1199.9, 1200, 20, true).size() <= MAX_DETAIL_CHARS);
}

int main() {
  layoutRules();
  homeRules();
  fontsFit();
  everyStateFits();
  sampleStates();
  carReportRules();
  wordsCoverTheCarSide();
  experimentWords();
  writes();
  startLabels();
  progressTexts();
  setupOrderAndChoice();
  planAndBands();
  setup();
  if (failures == 0) {
    std::printf("ok\n");
    return 0;
  }
  std::printf("%d failures\n", failures);
  return 1;
}
