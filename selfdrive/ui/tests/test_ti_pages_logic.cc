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

static const Page PAGES[] = {Page::Home, Page::Setup, Page::Tests, Page::Overview, Page::Ab, Page::Progress, Page::Cruise, Page::Curve,
                             Page::Buttons};

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
          const bool stop = c[i].id == CTL_STOP || c[j].id == CTL_STOP || c[i].id == CTL_ALLOFF || c[j].id == CTL_ALLOFF ||
                           c[i].id == CTL_CRUISE_STOP || c[j].id == CTL_CRUISE_STOP || c[i].id == CTL_CURVE_SWITCH ||
                           c[j].id == CTL_CURVE_SWITCH || c[i].id == CTL_BUTTONS_STOP || c[j].id == CTL_BUTTONS_STOP;
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
                         "plan says hold", "plan says off", "A/B switch on", "control loop busy", "unknown"};
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
                        s.plan_mode = pm; s.hold_why = hw;
                        if (w >= 0) s.tests_left_s[w] = 7200; else s.ab_stored_a_s = s.ab_stored_b_s = 100;
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
  s.run = Run::Plan; s.engaged = true; s.have_speed = true; s.mph = 24; s.plan_mode = "tests"; s.tests_active = true; s.configured = false;
  s.tests_left_s[0] = 100.0 * 60 / 1.3 / 60.0 * 1.0; s.tests_left_s[3] = 120;
  // 100 s * 1.3 / 60 = 2.2 -> 3 min left in the 24 mph band
  s.tests_left_s[0] = 100;
  CHECK(H(s).primary.text == "TESTING: HOLD 24 MPH" && H(s).primary.tone == Tone::Running && H(s).second == "3 min left here");
  s.mph = 72;                                     // another band that still needs time: it counts there too
  CHECK(H(s).primary.text == "TESTING: HOLD 72 MPH" && H(s).second == "3 min left here");
  s.tests_active = false;
  CHECK(H(s).primary.text == "WAITING: HOLD 72 MPH" && H(s).primary.tone == Tone::Waiting);
  s.tests_active = true;
  s.mph = 30;                                     // where nothing counts: said plainly, with the nearest bands that still need time
  CHECK(H(s).primary.text == "NO COUNT AT 30 MPH" && H(s).primary.tone == Tone::Waiting && H(s).second == "Hold 24 or 72 mph");
  s.mph = 17;
  CHECK(H(s).primary.text == "NO COUNT AT 17 MPH" && H(s).second == "Hold 24 mph");
  s.mph = 85;
  CHECK(H(s).primary.text == "NO COUNT AT 85 MPH" && H(s).second == "Hold 72 mph");
  s.tests_left_s[3] = 0;
  s.mph = 85;
  CHECK(H(s).second == "Hold 24 mph");                                    // 72 mph needs nothing now: only the 24 mph band is listed as needed
  s.tests_left_s[1] = s.tests_left_s[2] = 60;      // the windows 35-41 and 49-56 still need time, the alternation is not part of the plan
  s.mph = 45;                                     // the gap between them: nothing counts, and the list shows 38 and 52
  CHECK(H(s).primary.text == "NO COUNT AT 45 MPH" && H(s).second == "Hold 38 or 52 mph");
  s.mph = 24; s.tests_active = false; s.gate_reason = "curve";
  CHECK(H(s).primary.text == "WAITING: STRAIGHT ROAD");
  s = HomeState(); s.run = Run::Plan; s.engaged = true; s.plan_mode = "ab"; s.ab_fresh = true; s.ab_state = "running"; s.ab_arm = "B";
  s.ab_block = true; s.ab_block_left_s = 252;
  CHECK(H(s).primary.text == "A/B RUNNING: B" && H(s).second == "4:12 left in block");
  s.ab_block = false; s.ab_complete = true;
  CHECK(H(s).primary.text == "DONE: A/B COMPLETE" && H(s).primary.tone == Tone::Done);
  s = HomeState(); s.run = Run::Plan; s.engaged = true; s.plan_mode = "tests"; s.have_speed = true; s.mph = 45; s.configured = true;
  s.target_min = 20; s.ab_stored_a_s = s.ab_stored_b_s = 600;        // the alternation is open: 36-55 mph is one band, 45 mph is inside it
  CHECK(H(s).primary.text == "WAITING: A/B STARTING");
  s.mph = 25;                                                       // 25 mph: only the 24 mph band, which needs nothing: nothing counts
  s.tests_left_s[0] = 60;
  CHECK(H(s).primary.text == "TESTING: HOLD 24 MPH" || H(s).primary.text == "WAITING: HOLD 24 MPH");
  s.ab_stored_a_s = s.ab_stored_b_s = 1200; s.tests_left_s[0] = 0;
  CHECK(H(s).primary.text == "DONE: ALL COMPLETE");
  s = HomeState();
  CHECK(H(s).primary.text == "STOPPED: TAP START" && H(s).second == "Nothing is running");
  s.configured = true; s.ab_stored_known = true; s.ab_stored_a_s = 14.7 * 60; s.ab_stored_b_s = 13.2 * 60;
  CHECK(H(s).primary.text == "STOPPED: TAP START" && H(s).second == "A/B 13.2 of 20 min each side done");
  s.ab_stored_known = false;
  CHECK(H(s).second == "Nothing is running");
  s.run = Run::All;
  CHECK(H(s).primary.text == "WAITING: ENGAGE");
  // km/h on a metric device
  s = HomeState(); s.run = Run::All; s.engaged = true; s.have_speed = true; s.mph = 72; s.metric = true; s.tests_left_s[3] = 120;
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
          s.engaged = true; s.have_speed = true; s.mph = 24; s.configured = true; s.tests_left_s[0] = 60; s.ab_stored_a_s = s.ab_stored_b_s = 300;
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
  CHECK(N_HOLD_SHORT == 10 && holdShort("nonsense") == nullptr && ti_ab::holdText("nonsense")[0] == 0);
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
  CHECK(abProgressText(14.2 * 60, 13.9 * 60, 20) == "A 14.2 / 20 min, B 13.9 / 20 min");
  CHECK(abProgressText(70, 0, 10) == "A 1.1 / 10 min, B 0.0 / 10 min");          // a block's worth shows, not "0 / 10"
  CHECK(blockSoFar("B", 25.0) == "block B +0.4 min pending" && blockSoFar("", 25.0).empty());
  CHECK(detailScored(372, 318, 10, false, "B", 25.0) == "Scored: A 6.2 / B 5.3 of 10 min; block B +0.4 pending");
  CHECK((int)abLeftText(0, 0, 10, OVERHEAD_ALT, blockSoFar("B", 599)).size() * EXPERIMENT_PX * 0.62 <= PROG_AB_LEFT.w);
  CHECK((int)detailScored(599, 599, 10, false, "A", 599).size() <= MAX_DETAIL_CHARS);
  CHECK(OVERHEAD_ALT == 1.8);
  CHECK(abMinutesLeft(14 * 60, 13 * 60, 20) == 13.0 && abMinutesLeft(25 * 60, 21 * 60, 20) == 0.0);
  CHECK(abLeftText(14 * 60, 13 * 60, 20) == "About 13 min at 36-63 mph left");
  CHECK(abLeftText(1200, 1200, 20) == "A/B complete: the tests take over");
  CHECK(fraction(10, 20) == 0.5 && fraction(30, 20) == 1.0 && fraction(-1, 20) == 0.0 && fraction(1, 0) == 1.0);
  CHECK(windowLabel(0, false) == "21-27 mph" && windowRowText(0, 3, 12, 540, false) == "3 of 12 done; hold 24 mph for about 9 min");
  CHECK(windowRowText(0, 3, 12, 540, false, true) == "3 of 12 done (optional, counts if it happens)");
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
  CHECK(splitNote(true, false) == "A/B has 36-63 mph until done; tests run below 35 and above 64 mph");
  CHECK(splitNote(true, true).size() * DETAIL_PX * 0.62 <= PROG_SPLIT.w && splitNote(false, false).size() * DETAIL_PX * 0.62 <= PROG_SPLIT.w);
  CHECK(PROG_SPLIT.bottom() <= SCREEN_H && PROG_SPLIT.h >= DETAIL_PX + 4);
}

static PlanFields goodPlan() {
  PlanFields p;
  p.named = true; p.reasons_ok = true;
  p.test_ids = {"ti_sweep", "ti_pulses", "ti_ladder"};
  p.windows = {"21-27", "35-41", "49-56", "64-80"};
  p.has_alt = true; p.setting = "LatNoFrictionRelay"; p.a = "stored"; p.b_text = "other";
  p.block_min = 4; p.target_min = 10; p.min_mph = 35; p.max_mph = 64;
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
    {"target must be 10 min per side", [](PlanFields &x) { x.target_min = 30; }},
    {"an amount has no reason", [](PlanFields &x) { x.reasons_ok = false; }},
    {"speed range must be 35-64 mph", [](PlanFields &x) { x.max_mph = 56; }},
    {"tests must cover all four windows", [](PlanFields &x) { x.windows = {"21-27"}; }},
    {"tests must be sweep, pulses, ladder", [](PlanFields &x) { x.test_ids = {"ti_dither"}; }},
    {"no plan name", [](PlanFields &x) { x.named = false; }},
    {"optional must be true or false", [](PlanFields &x) { x.optional_bad = true; }},
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
  CHECK(startLook(true, Run::Stopped, true, true, true).label == "START: A/B relay" && planLine(true, "", true, "LatNoFrictionRelay", false) == "PLAN: friction relay A/B");

  // the bands: one figure each, the alternation and the tests merged
  HomeState hs;
  hs.configured = true; hs.target_min = 20; hs.ab_stored_a_s = hs.ab_stored_b_s = 0;
  hs.tests_left_s[0] = 6.0 * 60 / OVERHEAD_TESTS_LOW; hs.tests_left_s[1] = 4.0 * 60 / OVERHEAD_TESTS; hs.tests_left_s[2] = 3.0 * 60 / OVERHEAD_TESTS; hs.tests_left_s[3] = 4.0 * 60 / OVERHEAD_TESTS;
  Band b[MAX_BANDS];
  int n = bandsFromState(hs, b);
  CHECK(n == 3 && b[0].target_mph == 24 && b[2].target_mph == 72 && b[1].target_mph == 45 && b[0].minutes == 6 && b[2].minutes == 4);
  CHECK(b[1].minutes == std::ceil(40 * OVERHEAD_ALT + 7.0 - 1e-9));        // alternation 40 min, both sides still open, plus the two windows
  CHECK(b[1].lo == 35 && b[1].hi == 64 && bandRange(b[1], false) == "36-63 mph" && bandIndexAt(b, n, 35.5) == 1 && bandIndexAt(b, n, 45.0) == 1);
  hs.configured = false;                                                   // no alternation: the bands are the test windows
  n = bandsFromState(hs, b);
  CHECK(n == 4 && b[1].target_mph == 38 && b[2].target_mph == 52 && b[1].minutes == 4 && b[2].minutes == 3 && bandIndexAt(b, n, 45.0) == -1);
  hs.configured = true; hs.ab_stored_a_s = hs.ab_stored_b_s = 1200;       // the alternation complete: 36-55 is not a band any more, 38 and 52 are
  n = bandsFromState(hs, b);
  CHECK(n == 4 && b[1].target_mph == 38 && b[2].target_mph == 52 && bandIndexAt(b, n, 45.0) == -1 && bandIndexAt(b, n, 38.0) == 1);
  HomeState none;
  none.configured = true; none.target_min = 20; none.ab_stored_a_s = none.ab_stored_b_s = 1200;
  n = bandsFromState(none, b);
  CHECK(bandsTotalMinutes(b, n) == 0 && b[0].done && bandText(b[0], false) == "24 mph  done" && bandIndexAt(b, n, 24.0) == -1);
  n = bandsFromState(hs, b);
  CHECK(bandText(b[3], false) == "72 mph  4 min" && bandRange(b[3], false) == "64-80 mph");
  for (int m = 0; m < 2; m++) {
    HomeState big;
    big.configured = true; big.target_min = 20;
    for (int w = 0; w < 4; w++) big.tests_left_s[w] = 9e4;
    n = bandsFromState(big, b);
    for (int i = 0; i < n; i++) {
      CHECK((int)bandText(b[i], m).size() <= 24 && (int)bandRange(b[i], m).size() <= 24);
    }
    CHECK((int)budgetLine(b, n, m).size() <= 128);
  }
  hs.ab_stored_a_s = hs.ab_stored_b_s = 0;
  n = bandsFromState(hs, b);
  CHECK(budgetLine(b, n, false).rfind("Plan needs about ", 0) == 0 && budgetLine(b, n, false).find(" min: ") != std::string::npos);
  {                                                                          // plain words: one open band says its speeds; a finished plan says so
    Band one[1] = {Band{45, 35, 56, 36, 55, 12, false}};
    CHECK(budgetLine(one, 1, false) == "Plan needs about 12 min at 36-55 mph");
    one[0].minutes = 0; one[0].done = true;
    CHECK(budgetLine(one, 1, false) == "Plan complete: nothing more is needed");
    CHECK(budgetLine(one, 0, false).find("0 MIN") == std::string::npos && budgetLine(one, 0, false).back() != ':');
  }
}

// One model for the list and the status line, over every state: what is highlighted is a band where time is counting, "NO COUNT" never
// shows while a listed band contains the speed, and the minutes of a band are exactly what completing it needs at that moment.
static void listAndStatusAgree() {
  const char *plan_modes[] = {"", "off", "hold", "ab", "tests"};
  const double speeds[] = {0, 10, 20, 22, 25, 29, 33, 35, 35.5, 36, 38, 41, 45, 49, 52, 55, 55.9, 56, 60, 66, 72, 79, 80, 90};
  const double amounts[] = {0, 40, 400};
  int states = 0, highlighted = 0, nocount = 0;
  for (int run = 0; run < 5; run++)
    for (int conf = 0; conf < 2; conf++)
      for (const char *pm : plan_modes)
        for (double mph : speeds)
          for (double a : {0.0, 700.0, 1200.0})
            for (double t0 : amounts)
              for (double t1 : amounts)
                for (double t3 : amounts)
                  for (int active = 0; active < 2; active++)
                    for (int block = 0; block < 2; block++) {
                      HomeState s;
                      s.run = (Run)run; s.configured = conf; s.plan_mode = pm; s.engaged = true; s.have_speed = true; s.mph = mph;
                      s.target_min = 20; s.ab_stored_a_s = a; s.ab_stored_b_s = a; s.tests_left_s[0] = t0; s.tests_left_s[1] = t1;
                      s.tests_left_s[2] = t1; s.tests_left_s[3] = t3; s.tests_active = active; s.ab_block = block; s.ab_arm = "B";
                      s.ab_state = "running"; s.ab_fresh = true;
                      confirm(s);
                      Band b[MAX_BANDS];
                      const int n = bandsFromState(s, b);
                      const int counting = countingBand(s, b, n);
                      const HomeLines h = home(s);
                      states++;
                      // the highlighted band is one that contains the speed, still needs time, and the car reports counting
                      if (counting >= 0) {
                        highlighted++;
                        CHECK(!b[counting].done && mph >= b[counting].lo && mph < b[counting].hi && carRunning(s) && (active || block));
                      }
                      // "NO COUNT" never while a still-needed band contains the speed (and then nothing is highlighted)
                      if (h.primary.text.rfind("NO COUNT", 0) == 0) {
                        nocount++;
                        CHECK(bandIndexAt(b, n, mph) < 0 && counting < 0);
                      }
                      // a running status that counts down a band is in a listed band
                      if (h.primary.text.rfind("TESTING", 0) == 0) {
                        CHECK(bandIndexAt(b, n, mph) >= 0 && counting == bandIndexAt(b, n, mph));
                      }
                      // the minutes of a band: exactly what completing it requires now (recomputed here from the inputs)
                      const bool alt_open = conf && std::fmax(0.0, 1200.0 - a) + std::fmax(0.0, 1200.0 - a) > 0.0;
                      const double alt_min = alt_open ? (2 * std::fmax(0.0, 1200.0 - a) / 60.0) * OVERHEAD_ALT : 0.0;
                      const double w0 = t0 * OVERHEAD_TESTS_LOW / 60.0, w1 = t1 * OVERHEAD_TESTS / 60.0, w3 = t3 * OVERHEAD_TESTS / 60.0;
                      CHECK(b[0].minutes == std::ceil(w0 - 1e-9) && b[n - 1].minutes == std::ceil(w3 - 1e-9));
                      if (alt_open) {
                        CHECK(n == 3 && b[1].minutes == std::ceil(alt_min + 2 * w1 - 1e-9));
                      } else {
                        CHECK(n == 4 && b[1].minutes == std::ceil(w1 - 1e-9) && b[2].minutes == std::ceil(w1 - 1e-9));
                      }
                      CHECK(bandsTotalMinutes(b, n) >= 0);
                    }
  CHECK(states > 100000 && highlighted > 1000 && nocount > 1000);
}

static void testsOptional() {
  PlanFields p = goodPlan();
  for (int w = 0; w < 4; w++) p.optional_win[w] = true;
  CHECK(planProblem(p).empty());
  // the standard plan: the alternation alone, every test optional
  HomeState s;
  s.configured = true; s.target_min = 10; s.ab_stored_a_s = s.ab_stored_b_s = 0;
  for (int w = 0; w < 4; w++) { s.tests_optional[w] = true; s.tests_left_s[w] = 600; }
  Band b[MAX_BANDS];
  int n = bandsFromState(s, b);
  CHECK(n == 1 && b[0].target_mph == 45 && b[0].lo == 35 && b[0].hi == 64);
  CHECK(b[0].minutes == std::ceil(2 * 10 * OVERHEAD_ALT - 1e-9) && b[0].minutes == 36);      // 20 scored minutes, about 36 engaged (measured)
  CHECK(bandText(b[0], false) == "45 mph  36 min" && bandRange(b[0], false) == "36-63 mph");
  CHECK(bandsTotalMinutes(b, n) == 36);
  s.ab_stored_a_s = s.ab_stored_b_s = 300;
  n = bandsFromState(s, b);
  CHECK(n == 1 && b[0].minutes == std::ceil(2 * 5 * OVERHEAD_ALT - 1e-9));
  s.ab_stored_a_s = s.ab_stored_b_s = 600;                                  // both sides at target: no band at all, DONE
  n = bandsFromState(s, b);
  CHECK(n == 0 && bandsTotalMinutes(b, n) == 0);
  HomeState d = s;
  d.run = Run::Plan; d.plan_mode = "tests"; d.engaged = true; d.have_speed = true; d.mph = 45; d.ab_complete = true;
  confirm(d);
  CHECK(home(d).primary.text == "DONE: ALL COMPLETE" && home(d).second == "Optional tests continue");
  // the alternation still open: the optional tests never make a band or hold DONE back
  HomeState o = s;
  o.ab_stored_a_s = o.ab_stored_b_s = 0;
  o.run = Run::Plan; o.plan_mode = "tests"; o.engaged = true; o.have_speed = true; o.mph = 24; o.tests_active = true;
  confirm(o);
  CHECK(home(o).primary.text == "OPTIONAL: TESTS COUNT" && home(o).primary.tone == Tone::Running);      // a test counting at 24 mph: a bonus
  o.tests_active = false;
  CHECK(home(o).primary.text == "NO COUNT AT 24 MPH" && home(o).second == "Hold 45 mph");
  o.mph = 45; o.plan_mode = "ab"; o.ab_fresh = true; o.ab_state = "running"; o.ab_arm = "A"; o.ab_block = true; o.ab_block_left_s = 100;
  CHECK(home(o).primary.text == "A/B RUNNING: A");
  // a required window still makes its band; an optional one does not
  HomeState m = o;
  m.tests_optional[0] = false; m.tests_left_s[0] = 100;
  n = bandsFromState(m, b);
  CHECK(n == 2 && b[0].target_mph == 24 && b[1].target_mph == 45);
}

static void setup() {
  CHECK(setupEditable(false, false, 0) && setupEditable(false, true, 0.5) && !setupEditable(false, true, 25) && !setupEditable(true, true, 0));
  CHECK((int)setupNote(false, true, 0).size() <= MAX_EXPERIMENT_CHARS && (int)setupNote(true, true, 0).size() <= MAX_EXPERIMENT_CHARS);
  CHECK((int)individualNote(Run::Plan).size() <= MAX_EXPERIMENT_CHARS);
  CHECK((int)detailScored(1199.9, 1200, 20, true).size() <= MAX_DETAIL_CHARS);
}

// Fixes of the review of bf7da45: the first page never shows a finished plan as startable, a dead plan as running, a single test as the
// plan, or minutes beyond the target ("13 of 10")
static void reviewFixes() {
  HomeState s;
  s.configured = true; s.target_min = 10; s.ab_stored_known = true; s.ab_stored_a_s = 13 * 60; s.ab_stored_b_s = 12 * 60;
  for (int w = 0; w < 4; w++) s.tests_optional[w] = true;
  CHECK(home(s).second == "A/B complete: 10 min each side" && abProgressText(13 * 60, 12 * 60, 10) == "A 10.0 / 10 min, B 10.0 / 10 min");
  CHECK(detailScored(13 * 60, 12 * 60, 10, true).find("13") == std::string::npos);
  CHECK(planComplete(s));
  CHECK(home(s).primary.text == "STOPPED: PLAN COMPLETE" && home(s).primary.tone == Tone::Stopped);   // never "tap START" for a finished plan
  CHECK(!startLook(true, Run::Stopped, true, true, true, planComplete(s), false).enabled && startLook(true, Run::Stopped, true, true, true, true, false).label == "PLAN COMPLETE");
  CHECK(startLook(true, Run::Stopped, true, true, true, true, false).label.find("START") == std::string::npos);
  s.ab_stored_a_s = s.ab_stored_b_s = 6 * 60;
  CHECK(!planComplete(s) && startLook(true, Run::Stopped, true, true, true, planComplete(s), false).enabled);
  // a plan that stopped itself is not "running", and the bands are not its business
  s = HomeState(); s.run = Run::Plan; s.engaged = true; s.plan_mode = "dead"; s.ab_fresh = true; s.configured = true; s.have_speed = true; s.mph = 24;
  CHECK(planDead(s) && carRunning(s) && home(s).primary.text == "HELD: PLAN ERROR");
  const StartLook dead = startLook(true, Run::Plan, true, true, true, false, true);
  CHECK(!dead.enabled && dead.label.find("RUNNING") == std::string::npos && dead.label.find("STOPPED") != std::string::npos);
  // a single test is on: the page says so, START says what it replaces
  s = HomeState(); s.run = Run::Single; s.engaged = true; s.have_speed = true; s.mph = 24; s.configured = true; s.live_fresh = true; s.live_tests_on = true;
  s.tests_active = true;
  CHECK(otherModeOn(s) && otherModeLine(s) == "A SINGLE TEST IS ON, NOT THE PLAN" && home(s).primary.text == "SINGLE TEST: RUNNING");
  s.tests_active = false;
  CHECK(home(s).primary.text == "SINGLE TEST: WAITING" && home(s).second == "Not the plan: START replaces it");
  CHECK(startLook(true, Run::Single, true, true, true, false, false).sub == "Replaces the single test now on");
  // done states say plainly that nothing more is needed
  s = HomeState(); s.configured = true; s.target_min = 10; s.ab_stored_a_s = s.ab_stored_b_s = 600; s.engaged = true;
  for (int w = 0; w < 4; w++) s.tests_optional[w] = true;
  const std::vector<std::string> msg = completeLines(s);
  CHECK(msg.size() == 4 && msg[0] == "PLAN COMPLETE" && msg[1] == "Nothing more is needed" && msg[3].find("Optional tests may continue") == 0);
}

// The cruise set-speed check's page: its own page and controls (never the first page's START), boxes clear of controls, every
// state's lines fit, RUN CHECK only when the car's fresh report says it is free and nothing else is on, "running" only from the car
static CruiseView cruiseView(const char *state, bool fresh = true) {
  CruiseView v;
  v.have = true;
  v.fresh = fresh;
  v.state = state;
  v.orig = 45;
  v.set = 45;
  return v;
}

static void cruiseRules() {
  Control c[MAX_CONTROLS];
  const int n = layout(Page::Cruise, 0, c);
  Rect run{}, stop{};
  for (int i = 0; i < n; i++) {
    CHECK(c[i].id != CTL_START && c[i].id != CTL_STOP);
    CHECK(minDim(c[i].r) >= BIG_MIN);
    if (c[i].id == CTL_CRUISE_RUN) run = c[i].r;
    if (c[i].id == CTL_CRUISE_STOP) stop = c[i].r;
    for (const Rect &b : {CRUISE_TITLE, CRUISE_BAND, CRUISE_RESULT, CRUISE_DETAIL1, CRUISE_DETAIL2, CRUISE_DETAIL3, HOME_BADGE}) {
      CHECK(!overlaps(b, c[i].r));
    }
  }
  CHECK(run.w >= 900 && stop.w >= 900 && run.h >= 220 && gapBetween(run, stop) >= STOP_GAP);
  for (const Rect &b : {CRUISE_DETAIL1, CRUISE_DETAIL2, CRUISE_DETAIL3}) {
    CHECK(!overlaps(b, HOME_BADGE) && b.bottom() <= SCREEN_H);
  }
  // the first page has no cruise control; the progress page leads to it
  Control h[MAX_CONTROLS];
  const int hn = layout(Page::Home, 0, h);
  for (int i = 0; i < hn; i++) CHECK(h[i].id != CTL_CRUISE_RUN && h[i].id != CTL_NAV_CRUISE);
  Rect nav{};
  CHECK(rectOf(Page::Progress, CTL_NAV_CRUISE, 0, nav));
  CHECK(MAX_CRUISE_PRIMARY_CHARS * CRUISE_PRIMARY_PX * 0.75 <= CRUISE_PRIMARY.w);
  CHECK(MAX_CRUISE_SECOND_CHARS * CRUISE_SECOND_PX * 0.62 <= CRUISE_SECOND.w);
  CHECK(MAX_CRUISE_RESULT_CHARS * CRUISE_RESULT_PX * 0.62 <= CRUISE_RESULT.w);
  CHECK(CRUISE_PRIMARY_PX >= 100 && CRUISE_RESULT_PX >= 70);

  const char *states[] = {"idle", "refused", "waiting", "running", "restoring", "finished", "aborted"};
  const char *whys[] = {"", "speed not steady at the set speed", "no cruise button frames from the car", "the drive plan is on",
                        "set speed changed to 40 mph (expected 43 or 42)", "set-plus not accepted: set speed left 1 mph low",
                        "not started: speed outside 30-70 mph", "time limit 20 s reached"};
  const char *others[] = {"", "the drive plan is on", "the A/B switch is on", "a test is on", "the dither test is on"};
  for (const char *st : states) {
    for (const char *why : whys) {
      for (const char *oth : others) {
        for (bool fresh : {true, false}) {
          for (int ra = 0; ra <= 3; ra++) {
            CruiseView v = cruiseView(st, fresh);
            v.why = why;
            for (int k = 0; k < 3; k++) v.a.push_back(CruiseRound{k < ra, k < ra, 40, 60});
            if (ra == 0) for (int k = 0; k < 3; k++) v.b.push_back(CruiseRound{false, false, 0, 0});
            v.result = std::string(st) == "finished" ? (ra == 3 ? "YES" : (ra == 0 ? "NO" : "PARTLY")) : "";
            v.restore = "not restored after 3 taps";
            const CruiseLook k = cruiseLook(v, oth);
            CHECK((int)k.primary.size() <= MAX_CRUISE_PRIMARY_CHARS);
            if ((int)k.second.size() > MAX_CRUISE_SECOND_CHARS) std::printf("long second: %s\n", k.second.c_str());
            CHECK((int)k.second.size() <= MAX_CRUISE_SECOND_CHARS);
            CHECK((int)k.result.size() <= MAX_CRUISE_RESULT_CHARS);
            for (const std::string *d : {&k.detail1, &k.detail2, &k.detail3}) {
              if (d->size() * DETAIL_PX * 0.62 > CRUISE_DETAIL1.w) std::printf("long detail: %s\n", d->c_str());
              CHECK(d->size() * DETAIL_PX * 0.62 <= CRUISE_DETAIL1.w);
            }
            // the truth rules
            if (!fresh) CHECK(k.word == "noreport" && !k.run_enabled && !k.stop_enabled);
            if (fresh && cruiseActive(v)) CHECK(k.word == st && !k.run_enabled && k.stop_enabled);
            if (fresh && !cruiseActive(v)) CHECK(!k.stop_enabled && (k.run_enabled == (oth[0] == 0)));
            if (fresh && !cruiseActive(v) && oth[0] == 0) CHECK(k.word == st);
            if (fresh && !cruiseActive(v) && oth[0] != 0) CHECK(k.word == "unavailable" && k.second.find("STOP") != std::string::npos);
            CHECK((k.primary.find("RUNNING") != std::string::npos) == (fresh && std::string(st) == "running"));
            CHECK(!k.result.empty() == (std::string(st) == "finished" || (std::string(st) == "aborted" && fresh && cruiseNotRestored(v)) ||
                                        (std::string(st) == "aborted" && !fresh && cruiseNotRestored(v))));
            CHECK(cruiseBusy(v) == (fresh && cruiseActive(v)));
          }
        }
      }
    }
  }
  CruiseView y = cruiseView("finished");
  for (int k = 0; k < 3; k++) y.a.push_back(CruiseRound{true, true, 40, 60});
  y.result = "YES";
  CHECK(cruiseLook(y, "").result == "SET SPEED CAN BE CHANGED: YES (3 of 3)");
  CHECK(cruiseLook(y, "").detail2 == "Delay to the change: minus 40, 40, 40 ms; plus 60, 60, 60 ms");
  CruiseView no = cruiseView("finished");
  for (int k = 0; k < 3; k++) no.a.push_back(CruiseRound{});
  for (int k = 0; k < 3; k++) no.b.push_back(CruiseRound{});
  no.result = "NO";
  CHECK(cruiseLook(no, "").result == "SET SPEED CAN BE CHANGED: NO (0 of 3)");
  CHECK(cruiseLook(no, "").detail1 == "Recipe A (2-frame taps): 0 of 3. Recipe B (3-frame taps): 0 of 3");
  CruiseView pt = cruiseView("finished");
  pt.a = parseCruiseRounds("1,1,40,40;0,0,0,0;1,1,50,40");
  pt.result = "PARTLY";
  CHECK(pt.a.size() == 3 && pt.a[2].minus_ms == 50 && !pt.a[1].minus);
  CHECK(cruiseLook(pt, "").result == "SET SPEED CAN BE CHANGED: PARTLY (2 of 3)");
  CruiseView nr = cruiseView("aborted");
  nr.orig = 45;
  nr.set = 50;
  nr.why = "set speed changed to 50 mph (expected 45 or 44)";
  nr.restore = "Set speed is now 50 mph (was 45). Not restored: set it yourself.";
  CHECK(cruiseLook(nr, "").result == "SET IT YOURSELF: NOW 50 MPH, WAS 45");
  CHECK(cruiseLook(nr, "").detail3 == "Set speed is now 50 mph (was 45). Not restored: set it yourself.");
  CruiseView none;
  CHECK(cruiseLook(none, "").word == "noreport" && !cruiseLook(none, "").run_enabled);
  CHECK(cruiseLook(cruiseView("idle"), "").run_enabled && cruiseLook(cruiseView("idle"), "").word == "idle");
}

// Curve slowing's page: one large control, its label from the car's report; the lines are the controller's, cut only to the space
static void curveRules() {
  Control c[MAX_CONTROLS];
  const int n = layout(Page::Curve, 0, c);
  Rect sw{};
  for (int i = 0; i < n; i++) {
    CHECK(c[i].id != CTL_START && c[i].id != CTL_STOP && c[i].id != CTL_CRUISE_RUN);
    CHECK(minDim(c[i].r) >= BIG_MIN);
    if (c[i].id == CTL_CURVE_SWITCH) sw = c[i].r;
    for (const Rect &b : {CURVE_TITLE, CURVE_BAND, CURVE_PRIMARY1, CURVE_PRIMARY2, CURVE_SECOND, HOME_BADGE}) {
      CHECK(!overlaps(b, c[i].r));
    }
  }
  CHECK(sw.w * 10 >= SCREEN_W * 9 && sw.h >= 220);
  CHECK(!overlaps(CURVE_BAND, HOME_BADGE) && CURVE_BAND.bottom() <= SCREEN_H);
  Rect nav{}, cruise{};
  CHECK(rectOf(Page::Progress, CTL_NAV_CURVE, 0, nav) && rectOf(Page::Progress, CTL_NAV_CRUISE, 0, cruise));
  CHECK(minDim(nav) >= BIG_MIN && gapBetween(nav, cruise) >= 80);
  Control h[MAX_CONTROLS];
  const int hn = layout(Page::Home, 0, h);
  for (int i = 0; i < hn; i++) CHECK(h[i].id != CTL_CURVE_SWITCH && h[i].id != CTL_NAV_CURVE);
  CHECK(MAX_CURVE_PRIMARY_CHARS * CURVE_PRIMARY_PX * 0.75 <= CURVE_PRIMARY1.w);
  CHECK(MAX_CURVE_SECOND_CHARS * CURVE_SECOND_PX * 0.62 <= CURVE_SECOND.w);
  CHECK(CURVE_PRIMARY_PX >= 100 && CURVE_SECOND_PX >= 60);

  // the longest lines ti_curve.py writes, and every state, fresh or not
  const char *states[] = {"off", "ready", "slowing", "holding", "restoring", "paused", "stopped", "waiting"};
  const char *l1s[] = {"CURVE SLOWING OFF", "ON: no known corner ahead", "ON: road not known here", "SLOWING FOR A CORNER: 100 -> 25 mph",
                       "HOLDING FOR THE NEXT CORNER: 100 mph", "RESTORING: 25 -> 100 mph", "PAUSED: you changed the set speed",
                       "STOPPED: set speed changed to 100 mph unexpectedly", "STOPPED: brake pressed", "ON: waiting", ""};
  const char *l2s[] = {"", "CORNER NEEDS MORE THAN I MAY SLOW: be ready", "Set speed 100 mph (was 100): set it yourself",
                       "shadow: the planner only logs", "next corner: holding what it needs", "the model sees a bend ahead"};
  for (const char *st : states) {
    for (const char *l1 : l1s) {
      for (const char *l2 : l2s) {
        for (bool fresh : {true, false}) {
          for (bool more : {false, true}) {
            CurveView v;
            v.have = true;
            v.fresh = fresh;
            v.state = st;
            v.line1 = l1;
            v.line2 = l2;
            v.needs_more = more;
            const CurveLook k = curveLook(v);
            CHECK((int)k.primary1.size() <= MAX_CURVE_PRIMARY_CHARS && (int)k.primary2.size() <= MAX_CURVE_PRIMARY_CHARS);
            CHECK((int)k.second.size() <= MAX_CURVE_SECOND_CHARS);
            if (fresh) {
              // the controller's own words, whole (every line it writes fits)
              CHECK((k.primary2.empty() ? k.primary1 : k.primary1 + " " + k.primary2) == l1);
              CHECK(k.second == l2);
              CHECK(k.word == st);
              CHECK((k.button == "TURN OFF") == (std::string(st) != "off"));
              CHECK(std::string(k.write) == (std::string(st) == "off" ? "1" : "0"));
              CHECK(k.warn == (std::string(st) == "stopped" || more));
            } else {
              CHECK(k.word == "noreport" && k.button == "TURN OFF" && std::string(k.write) == "0" && k.warn);
              CHECK(k.primary1 + " " + k.primary2 == "CURVE SLOWING: no report from the car");
            }
          }
        }
      }
    }
  }
  CurveView none;
  CHECK(curveLook(none).word == "noreport" && std::string(curveLook(none).write) == "0");
  // a line longer than the space is cut, never wrapped past two lines
  std::string a, b;
  curveWrap("SLOWING FOR A CORNER: 55 -> 46 mph", MAX_CURVE_PRIMARY_CHARS, a, b);
  CHECK(a == "SLOWING FOR A CORNER:" && b == "55 -> 46 mph");
  curveWrap("STOPPED: set speed changed to 46 mph unexpectedly", MAX_CURVE_PRIMARY_CHARS, a, b);
  CHECK(a == "STOPPED: set speed changed" && b == "to 46 mph unexpectedly");
  curveWrap(std::string(80, 'X'), MAX_CURVE_PRIMARY_CHARS, a, b);
  CHECK((int)a.size() == MAX_CURVE_PRIMARY_CHARS && (int)b.size() == MAX_CURVE_PRIMARY_CHARS && b.substr(b.size() - 3) == "...");
  CHECK(curveCut(std::string(60, 'y'), MAX_CURVE_SECOND_CHARS).size() == (size_t)MAX_CURVE_SECOND_CHARS);
}

// The cruise button tests' page: its own page and controls (never the first page's START), boxes clear of controls, the car's lines
// shown whole (they are written to fit: ti_button_tests.py MAX_*), the test controls only when the car's fresh report says no test
// runs and nothing else is on, STOP only while one runs, "running" only from the car
static void buttonsRules() {
  Control c[MAX_CONTROLS];
  const int n = layout(Page::Buttons, 0, c);
  Rect hold{}, taps{}, stop{};
  for (int i = 0; i < n; i++) {
    CHECK(c[i].id != CTL_START && c[i].id != CTL_STOP && c[i].id != CTL_CRUISE_RUN);
    CHECK(minDim(c[i].r) >= BIG_MIN);
    if (c[i].id == CTL_BUTTONS_HOLD) hold = c[i].r;
    if (c[i].id == CTL_BUTTONS_TAPS) taps = c[i].r;
    if (c[i].id == CTL_BUTTONS_STOP) stop = c[i].r;
    for (const Rect &b : {BUTTONS_TITLE, BUTTONS_BAND, BUTTONS_RESULT, BUTTONS_DETAIL1, BUTTONS_DETAIL2, BUTTONS_DETAIL3, HOME_BADGE}) {
      CHECK(!overlaps(b, c[i].r));
    }
  }
  CHECK(hold.w >= 640 && taps.w >= 640 && stop.w >= 640 && hold.h >= 220 && gapBetween(taps, stop) >= STOP_GAP);
  // every control's labels fit its box: the label at START_PX (bold capitals), the second line at START_SUB_PX
  for (const Rect &r : {hold, taps, stop}) {
    CHECK(9 * START_PX * 0.75 <= r.w - 48);                     // "HOLD TEST" / "STOP TEST" (9 characters), "TAP TEST"
    CHECK(MAX_BUTTONS_SUB_CHARS * START_SUB_PX * 0.62 <= r.w - 48);
  }
  for (const Rect &b : {BUTTONS_DETAIL1, BUTTONS_DETAIL2, BUTTONS_DETAIL3}) {
    CHECK(!overlaps(b, HOME_BADGE) && b.bottom() <= SCREEN_H);
  }
  CHECK(MAX_BUTTONS_PRIMARY_CHARS * BUTTONS_PRIMARY_PX * 0.75 <= BUTTONS_PRIMARY.w - 48);
  CHECK(MAX_BUTTONS_SECOND_CHARS * BUTTONS_SECOND_PX * 0.62 <= BUTTONS_SECOND.w - 48);
  CHECK(MAX_BUTTONS_RESULT_CHARS * BUTTONS_RESULT_PX * 0.62 <= BUTTONS_RESULT.w - 48);
  CHECK(MAX_BUTTONS_DETAIL_CHARS * DETAIL_PX * 0.62 <= BUTTONS_DETAIL1.w);
  CHECK(BUTTONS_PRIMARY_PX >= 100 && BUTTONS_RESULT_PX >= 70);
  // the progress page leads to it; the first page has none of its controls; the progress page's four links are clear of each other
  Control h[MAX_CONTROLS];
  const int hn = layout(Page::Home, 0, h);
  for (int i = 0; i < hn; i++) CHECK(h[i].id != CTL_BUTTONS_HOLD && h[i].id != CTL_BUTTONS_TAPS && h[i].id != CTL_NAV_BUTTONS);
  Rect nav{}, curve{};
  CHECK(rectOf(Page::Progress, CTL_NAV_BUTTONS, 0, nav) && rectOf(Page::Progress, CTL_NAV_CURVE, 0, curve));
  CHECK(nav.h >= 150 && nav.w >= 300 && gapBetween(nav, curve) >= MIN_GAP && nav.bottom() <= SCREEN_H);

  const char *states[] = {"idle", "refused", "waiting", "running", "restoring", "finished", "aborted", "bogus"};
  const char *others[] = {"", "the drive plan is on", "the A/B switch is on", "a test is on", "the cruise check is running",
                          "curve slowing is on"};
  const std::string long1(40, 'X'), long2(80, 'y'), longr(60, 'Z'), longd(120, 'd');
  for (const char *st : states) {
    for (const char *oth : others) {
      for (bool fresh : {true, false}) {
        for (bool lng : {false, true}) {
          ButtonsView v;
          v.have = true;
          v.fresh = fresh;
          v.state = st;
          v.test = "hold";
          v.line1 = lng ? long1 : "HOLD TEST DONE";
          v.line2 = lng ? long2 : "Set speed back at 45 mph and checked";
          v.result = lng ? longr : (std::string(st) == "aborted" ? "SET IT YOURSELF: NOW 40 MPH, WAS 45" : "SET-: HOLD   SET+: HOLD");
          v.d1 = lng ? longd : "SET-: 45 -> 44 -> 40 (0.05, 0.55 s): a hold";
          v.d2 = v.d1;
          v.d3 = v.d1;
          const ButtonsLook k = buttonsLook(v, oth);
          CHECK((int)k.primary.size() <= MAX_BUTTONS_PRIMARY_CHARS && (int)k.second.size() <= MAX_BUTTONS_SECOND_CHARS);
          CHECK((int)k.result.size() <= MAX_BUTTONS_RESULT_CHARS);
          for (const std::string *d : {&k.detail1, &k.detail2, &k.detail3}) CHECK((int)d->size() <= MAX_BUTTONS_DETAIL_CHARS);
          CHECK((int)k.hold_sub.size() <= MAX_BUTTONS_SUB_CHARS && (int)k.taps_sub.size() <= MAX_BUTTONS_SUB_CHARS);
          CHECK((int)k.stop_sub.size() <= MAX_BUTTONS_SUB_CHARS);
          const bool active = buttonsActive(v);
          if (!fresh) CHECK(k.word == "noreport" && !k.run_enabled && !k.stop_enabled && k.tone == Tone::Held);
          if (fresh && active) CHECK(k.word == st && !k.run_enabled && k.stop_enabled);
          if (fresh && !active) CHECK(!k.stop_enabled && k.run_enabled == (oth[0] == 0));
          if (fresh && !active && oth[0] != 0) CHECK(k.word == "unavailable" && k.second.find("STOP") != std::string::npos);
          if (fresh && !active && oth[0] == 0) CHECK(k.word == st);
          if (fresh && (active || oth[0] == 0) && !lng) CHECK(k.primary == v.line1 && k.second == v.line2 && k.detail1 == v.d1);
          CHECK((k.tone == Tone::Running) == (fresh && std::string(st) == "running"));
          CHECK(buttonsBusy(v) == (fresh && active));
          // the last result stays readable without a report; nothing else does
          const bool final_state = std::string(st) == "finished" || std::string(st) == "aborted";
          if (!fresh && !final_state) CHECK(k.result.empty() && k.detail1.empty());
          if (!lng && fresh) CHECK(k.result_warn == (std::string(st) == "aborted"));
        }
      }
    }
  }
  ButtonsView none;
  CHECK(buttonsLook(none, "").word == "noreport" && !buttonsLook(none, "").run_enabled && !buttonsBusy(none));
}

int main() {
  buttonsRules();
  cruiseRules();
  curveRules();
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
  testsOptional();
  listAndStatusAgree();
  setup();
  reviewFixes();
  if (failures == 0) {
    std::printf("ok\n");
    return 0;
  }
  std::printf("%d failures\n", failures);
  return 1;
}
