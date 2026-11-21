// Prints the on-car pages as drawing lists (JSON) from selfdrive/ui/qt/ti_pages_logic.h: every rectangle and every text of the
// first page, the set-up page and the individual pages, at the device's 2160 x 1080, for a handful of states. The mock-up
// renderer (selfdrive/ui/tests/ti_pages_mockup.py) turns them into images and HTML. Not part of the device build.
//   usage: dump_ti_pages_layout > scenes.json
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "selfdrive/ui/qt/ti_pages_logic.h"

using namespace ti_pages;

static std::string esc(const std::string &s) {
  std::string o;
  for (char c : s) {
    if (c == '\n') o += "\\n";
    else if (c == '"') o += "\\\"";
    else if (c == '\\') o += "\\\\";
    else o += c;
  }
  return o;
}

struct Item {
  Rect r;
  int fill_r, fill_g, fill_b;   // -1: no fill
  int ink_r, ink_g, ink_b;
  std::string text;
  int px;
  std::string sub;
  int sub_px;
  const char *align;            // center or left
  const char *what;             // control or text
  const char *name;
};

struct Scene {
  std::string name, title;
  bool locked = false;
  std::vector<Item> items;
};

static const int GREY[3] = {0x33, 0x33, 0x33};
static const int DIM[3] = {0x1a, 0x1a, 0x1a};
static const int DIMINK[3] = {0x55, 0x55, 0x55};
static const int WHITE[3] = {255, 255, 255};
static const int BLACK[3] = {0, 0, 0};
static const int RED[3] = {201, 34, 49};
static const int GREEN[3] = {0x17, 0x86, 0x44};
static const int BLUE[3] = {0x46, 0x5B, 0xEA};
static const int AMBER[3] = {218, 202, 37};

static void box(Scene &s, Page pg, int id, const int *fill, const int *ink, const std::string &text, int px, const std::string &sub = "", int sub_px = 0) {
  Rect r{};
  rectOf(pg, id, 0, r);
  Control c[MAX_CONTROLS];
  const int n = layout(pg, 0, c);
  const char *name = "";
  for (int i = 0; i < n; i++) if (c[i].id == id) name = c[i].name;
  s.items.push_back({r, fill[0], fill[1], fill[2], ink[0], ink[1], ink[2], text, px, sub, sub_px, "center", "control", name});
}

static void textBox(Scene &s, const Rect &r, const int *ink, const std::string &text, int px, const char *align = "left") {
  s.items.push_back({r, -1, -1, -1, ink[0], ink[1], ink[2], text, px, "", 0, align, "text", ""});
}

static void lockAndEngaged(Scene &s, Page pg, bool engaged) {
  box(s, pg, CTL_LOCK, s.locked ? AMBER : GREY, s.locked ? BLACK : WHITE, s.locked ? "LOCKED\ntap to unlock" : "LOCK PAGE", NAV_PX);
  if (pg == Page::Home) {
    s.items.push_back({HOME_BADGE, engaged ? GREEN[0] : 0x54, engaged ? GREEN[1] : 0x54, engaged ? GREEN[2] : 0x54, 255, 255, 255,
                       engaged ? "● ENGAGED" : "○ NOT ENGAGED", 40, "", 0, "center", "badge", ""});
  } else {
    s.items.push_back({Rect{SCREEN_W - 520, SCREEN_H - 84, 490, 64}, engaged ? GREEN[0] : 0x54, engaged ? GREEN[1] : 0x54, engaged ? GREEN[2] : 0x54,
                       255, 255, 255, engaged ? "● ENGAGED" : "○ NOT ENGAGED", 40, "", 0, "center", "badge", ""});
  }
}

static Scene homeScene(const std::string &name, const std::string &title, HomeState hs, bool locked = false) {
  if (hs.run != Run::Stopped) {                       // the car side reports what is asked for (the scenes show the wording)
    hs.car_plan_on = true; hs.live_fresh = hs.live_all_on = hs.live_tests_on = true;
    if (std::strcmp(hs.plan_mode, "dead") == 0) hs.ab_fresh = true;
  }
  Scene s;
  s.name = name;
  s.title = title;
  s.locked = locked;
  hs.target_min = 10; hs.ab_stored_known = hs.configured; hs.ab_stored_a_s = 6.2 * 60; hs.ab_stored_b_s = 5.3 * 60;
  const bool alt_done = hs.ab_complete || name == "home-gap-alternation-done";
  if (alt_done) hs.ab_stored_a_s = hs.ab_stored_b_s = 600;
  hs.tests_left_s[0] = 6.0 * 60 / OVERHEAD_TESTS_LOW; hs.tests_left_s[1] = 4.0 * 60 / OVERHEAD_TESTS; hs.tests_left_s[2] = 3.0 * 60 / OVERHEAD_TESTS; hs.tests_left_s[3] = 4.0 * 60 / OVERHEAD_TESTS;
  for (int w = 0; w < 4; w++) hs.tests_optional[w] = true;                     // the standard plan: every test optional
  const HomeLines h = home(hs);
  const ToneColor tc = toneColor(h.primary.tone);
  const int fill[3] = {tc.r, tc.g, tc.b};
  const int *ink = tc.light ? BLACK : WHITE;
  s.items.push_back({Rect{HOME_PRIMARY.x, HOME_PRIMARY.y, HOME_PRIMARY.w, HOME_SECOND.bottom() - HOME_PRIMARY.y}, fill[0], fill[1], fill[2],
                     ink[0], ink[1], ink[2], "", 0, "", 0, "center", "band", ""});
  textBox(s, Rect{HOME_PRIMARY.x + 24, HOME_PRIMARY.y, HOME_PRIMARY.w - 48, HOME_PRIMARY.h}, ink, h.primary.text, PRIMARY_PX);
  s.items.back().text = h.primary.text;
  s.items.back().sub = "bold";
  textBox(s, Rect{HOME_SECOND.x + 24, HOME_SECOND.y, HOME_SECOND.w - 48, HOME_SECOND.h}, ink, h.second, SECOND_PX);
  const int grey_ink[3] = {0xc9, 0xc9, 0xc9};
  textBox(s, Rect{HOME_EXPERIMENT.x + 24, HOME_EXPERIMENT.y, HOME_EXPERIMENT.w, HOME_EXPERIMENT.h}, WHITE,
          planLine(true, "", hs.configured, "LatNoFrictionRelay", false), CHOSEN_PX);
  const StartLook look = startLook(hs.configured, hs.run, carRunning(hs), true, true);
  const int dark_green[3] = {0x0f, 0x4d, 0x28};
  Band bands[MAX_BANDS];
  const int n_bands = bandsFromState(hs, bands);
  const int counting = countingBand(hs, bands, n_bands);
  if (carRunning(hs)) {
    Rect area{};
    rectOf(Page::Home, CTL_START, 0, area);
    for (int i = 0; i < n_bands; i++) {
      const bool inside = bandIndexAt(bands, n_bands, hs.mph) == i && hs.have_speed;
      const Rect row{area.x, area.y + i * 70, area.w, 66};
      const int fr = i == counting ? 0x17 : (inside ? 0x6b : (bands[i].done ? 0x1f : 0x22));
      const int fg = i == counting ? 0x86 : (inside ? 0x5e : (bands[i].done ? 0x3a : 0x22));
      const int fb = i == counting ? 0x44 : (inside ? 0x10 : (bands[i].done ? 0x2a : 0x22));
      s.items.push_back({row, fr, fg, fb, 255, 255, 255, "", 0, "", 0, "center", "band", ""});
      textBox(s, Rect{row.x + 24, row.y, row.w - 420, row.h}, WHITE, (bands[i].done ? "✓ " : "") + bandText(bands[i], false), BAND_PX);
      s.items.back().sub = "bold";
      textBox(s, Rect{row.x, row.y, row.w - 24, row.h}, grey_ink, bandRange(bands[i], false), DETAIL_PX, "right");
    }
    if (bandsTotalMinutes(bands, n_bands) > 0) {
      textBox(s, Rect{area.x, area.y + n_bands * 70 + 4, area.w, 44}, grey_ink, "About " + std::to_string((int)bandsTotalMinutes(bands, n_bands)) + " min of driving left in all", DETAIL_PX);
    }
  } else {
    box(s, Page::Home, CTL_START, look.enabled ? BLUE : dark_green, WHITE, look.label, START_PX, look.sub, START_SUB_PX);
  }
  const std::string budget = hs.run == Run::Stopped ? budgetLine(bands, n_bands, false) : "";
  const bool any = hs.run != Run::Stopped;
  const int dark_red[3] = {0x6b, 0x1d, 0x25};
  box(s, Page::Home, CTL_STOP, any ? RED : dark_red, any ? WHITE : grey_ink, "STOP", STOP_PX, "stops every test and the A/B relay", START_SUB_PX);
  box(s, Page::Home, CTL_BACK, locked ? DIM : GREY, locked ? DIMINK : WHITE, "< EXIT", NAV_PX);
  box(s, Page::Home, CTL_NAV_SETUP, GREY, WHITE, "PLAN DETAILS >", NAV_PX);
  box(s, Page::Home, CTL_NAV_PROGRESS, GREY, WHITE, "PROGRESS >", NAV_PX);
  lockAndEngaged(s, Page::Home, hs.engaged);
  const double drive[4] = {95, 0, 40, 0};
  textBox(s, HOME_DETAIL, grey_ink, hs.run != Run::Stopped ? thisDriveLine(drive, 4.0 * 60, 3.5 * 60, hs.configured && !alt_done, false) : budget, DETAIL_PX);
  textBox(s, HOME_DETAIL2, grey_ink, hs.configured ? detailScored(hs.ab_stored_a_s, hs.ab_stored_b_s, 10, alt_done) : "", DETAIL_PX);
  return s;
}

static Scene setupScene() {
  Scene s;
  s.name = "plan-details";
  s.title = "Plan details (read-only; nothing is chosen on the screen)";
  const Page pg = Page::Setup;
  box(s, pg, CTL_BACK, GREY, WHITE, "< BACK", NAV_PX);
  box(s, pg, CTL_LOCK, GREY, WHITE, "LOCK PAGE", NAV_PX);
  box(s, pg, CTL_SETUP_RESET, GREY, WHITE, "RESET\nPROGRESS", NAV_PX);
  s.items.push_back({TITLE_SETUP, -1, -1, -1, 255, 255, 255, "PLAN DETAILS", 90, "bold", 0, "center", "text", ""});
  const int grey_ink[3] = {0xc9, 0xc9, 0xc9};
  const char *lines[] = {"Next drive: friction relay A/B (identification tests optional)", "Date 2026-11-23. Read from the build; nothing here is chosen on the screen.",
                         "Tests (optional, counted if they happen): sweep, pulses, ladder in all four windows",
                         "A/B: friction relay, yours (off) against on", "Blocks of 4 min, 10 min per side to finish (about 28 min at 36-55 mph)",
                         "A 6 / 10 min, B 5 / 10 min  (kept across restarts)"};
  for (int i = 0; i < 6; i++) {
    textBox(s, Rect{480, 215 + i * 56, 1660, 52}, i == 0 ? WHITE : grey_ink, lines[i], EXPERIMENT_PX);
  }
  // the reasons, as the plan file gives them (wrapped here the way the device wraps them)
  const std::string notes =
      "Alternation, 10 min per side: The question is whether removing the friction relay reduces steering oscillation (direction reversals per second) at 36-55 mph. Pass-to-pass variation is about 0.12 per second per pass of about 86 s; the one earlier comparison showed a drop of about 0.23 per second, about 1.9 times that variation. "
      "Minimum: About 4-5 passes per side (about 6-7 minutes per side) give 80 percent power for an effect of that size. It cannot show an effect as small as closed-loop replay predicted (about 0.05 per second): that would need well over an hour per side and is out of reach of any single drive. "
      "Buffer: 10 minutes per side is that minimum plus about half again, so a second drive is unlikely to be needed.   "
      "Tests, all windows (optional): Optional: the owner's decision, 'alteration alone is fine'. At the tests' signal sizes the steering wheel's response is below the angle sensor's resolution, the torque-sensor path is already measured, and the car model is frozen below the limit. They run and are credited when the road allows, everywhere outside the alternation's range from the start and inside it once the alternation is complete.";
  int y = 215 + 6 * 56 + 8;
  std::string line;
  size_t pos = 0;
  while (pos < notes.size() && y < SCREEN_H - 20) {
    size_t end = std::min(notes.size(), pos + 105);
    if (end < notes.size()) {
      const size_t sp = notes.rfind(' ', end);
      if (sp != std::string::npos && sp > pos) end = sp;
    }
    textBox(s, Rect{480, y, 1660, 34}, grey_ink, notes.substr(pos, end - pos), PLAN_NOTE_PX);
    y += 34;
    pos = end + 1;
  }
  return s;
}

static Scene progressScene() {
  Scene s;
  s.name = "progress";
  s.title = "Progress page: the alternation's sides against target, each test window";
  const Page pg = Page::Progress;
  box(s, pg, CTL_BACK, GREY, WHITE, "< START\nPAGE", NAV_PX);
  box(s, pg, CTL_NAV_INDIV, GREY, WHITE, "INDIVIDUAL\nTESTS >", NAV_PX);
  box(s, pg, CTL_LOCK, GREY, WHITE, "LOCK PAGE", NAV_PX);
  s.items.push_back({PROG_TITLE, -1, -1, -1, 255, 255, 255, "PROGRESS", 90, "bold", 0, "center", "text", ""});
  const int grey_ink[3] = {0xc9, 0xc9, 0xc9};
  textBox(s, PROG_AB_HEAD, WHITE, abProgressText(6.2 * 60, 5.3 * 60, 10), PROGRESS_PX);
  s.items.back().sub = "bold";
  s.items.push_back({PROG_AB_BAR_A, 0x33, 0x33, 0x33, 255, 255, 255, "", 0, "", 0, "center", "bar", ""});
  s.items.push_back({Rect{PROG_AB_BAR_A.x, PROG_AB_BAR_A.y, (int)(PROG_AB_BAR_A.w * fraction(6.2 * 60, 600)), PROG_AB_BAR_A.h}, BLUE[0], BLUE[1], BLUE[2], 255, 255, 255, "", 0, "", 0, "center", "bar", ""});
  s.items.push_back({PROG_AB_BAR_B, 0x33, 0x33, 0x33, 255, 255, 255, "", 0, "", 0, "center", "bar", ""});
  s.items.push_back({Rect{PROG_AB_BAR_B.x, PROG_AB_BAR_B.y, (int)(PROG_AB_BAR_B.w * fraction(5.3 * 60, 600)), PROG_AB_BAR_B.h}, AMBER[0], AMBER[1], AMBER[2], 0, 0, 0, "", 0, "", 0, "center", "bar", ""});
  textBox(s, PROG_AB_LEFT, grey_ink, abLeftText(6.2 * 60, 5.3 * 60, 10, OVERHEAD_ALT), EXPERIMENT_PX);
  const int done[4] = {3, 12, 5, 0};
  const int total[4] = {12, 12, 12, 12};
  for (int w = 0; w < ti_logic::N_WINDOWS; w++) {
    const int y = PROG_ROWS_TOP + w * PROG_ROW_H;
    textBox(s, Rect{480, y, 420, 50}, WHITE, windowLabel(w, false), PROGRESS_PX);
    s.items.back().sub = "bold";
    s.items.push_back({Rect{900, y + 8, 1240, 36}, 0x33, 0x33, 0x33, 255, 255, 255, "", 0, "", 0, "center", "bar", ""});
    const bool full = done[w] >= total[w];
    s.items.push_back({Rect{900, y + 8, std::max(24, 1240 * done[w] / total[w]), 36}, full ? GREEN[0] : BLUE[0], full ? GREEN[1] : BLUE[1], full ? GREEN[2] : BLUE[2],
                       255, 255, 255, "", 0, "", 0, "center", "bar", ""});
    textBox(s, Rect{480, y + 54, 1660, 50}, grey_ink, windowRowText(w, done[w], total[w], 540, false, true), EXPERIMENT_PX);
  }
  return s;
}

static void header(Scene &s, Page pg, const char *back, const char *red) {
  box(s, pg, CTL_BACK, GREY, WHITE, back, NAV_PX);
  box(s, pg, CTL_ALLOFF, RED, WHITE, red, 60);
  lockAndEngaged(s, pg, true);
  const int note_ink[3] = {0xa0, 0xa0, 0xa0};
  textBox(s, Rect{BANNER_RECT.x, BANNER_RECT.y, BANNER_RECT.w, 70}, note_ink, individualNote(Run::Plan), 30);
}

static Scene testsScene() {
  Scene s;
  s.name = "individual-tests";
  s.title = "Individual tests: one test (diagnostic page)";
  const Page pg = Page::Tests;
  header(s, pg, "< START\nPAGE", "ALL TESTS OFF");
  box(s, pg, CTL_PREV, DIM, DIMINK, "< PREV", 56);
  box(s, pg, CTL_NEXT, GREY, WHITE, "NEXT >", 56);
  textBox(s, TITLE_PREV_NEXT, WHITE, "TEST 1\nOF 3", 48, "center");
  box(s, pg, CTL_OVERVIEW, BLUE, WHITE, "RUN-ALL\nPAGE >", NAV_PX);
  box(s, pg, CTL_AB_PAGE, GREY, WHITE, "A/B\nSWITCH >", NAV_PX);
  const int amber_state[3] = {218, 202, 37};
  box(s, pg, CTL_TOGGLE, amber_state, BLACK, "ON · WAIT\ntap to switch off", 64);
  box(s, pg, CTL_RESET, GREY, WHITE, "Reset progress", 40);
  Rect t{};
  rectOf(pg, CTL_TOGGLE, 0, t);
  textBox(s, Rect{t.right() + 40, ROW_TOP + 10, 1000, 60}, WHITE, "Sweep", 50);
  textBox(s, Rect{t.right() + 40, ROW_TOP + 80, 1300, 200}, AMBER, "HOLD 24 MPH . window 21-27 mph . 3:20 to go  (progress bars as before)", 36);
  return s;
}

static Scene overviewScene() {
  Scene s;
  s.name = "individual-run-all";
  s.title = "Individual tests: run-all page";
  const Page pg = Page::Overview;
  header(s, pg, "< START\nPAGE", "ALL TESTS OFF");
  box(s, pg, CTL_OVERVIEW_BACK, GREY, WHITE, "< TEST PAGES", NAV_PX);
  textBox(s, TITLE_WIDE, WHITE, "RUN ALL TESTS", 56, "center");
  const int grey_state[3] = {0x54, 0x54, 0x54};
  box(s, pg, CTL_TOGGLE, grey_state, WHITE, "OFF\ntap to switch on", 64);
  return s;
}

static Scene abScene() {
  Scene s;
  s.name = "individual-ab-switch";
  s.title = "Individual tests: plain A/B switch with pins";
  const Page pg = Page::Ab;
  header(s, pg, "< START\nPAGE", "A/B OFF");
  box(s, pg, CTL_AB_BACK, GREY, WHITE, "< TEST PAGES", NAV_PX);
  textBox(s, TITLE_WIDE, WHITE, "A/B SWITCH", 56, "center");
  box(s, pg, CTL_AB_SWITCH, GREEN, WHITE, "A/B ON\ntap to stop", ACTION_PX);
  box(s, pg, CTL_PIN_A, GREY, WHITE, "PIN A", ACTION_PX);
  box(s, pg, CTL_PIN_B, GREY, WHITE, "PIN B", ACTION_PX);
  s.items.push_back({AB_PANEL, 0x11, 0x11, 0x11, 255, 255, 255, "B  0.85\nBlock 2: 3:20 left", 56, "", 0, "left", "text", ""});
  return s;
}

static void emit(const std::vector<Scene> &scenes) {
  std::printf("{\"screen\": [%d, %d], \"scenes\": [\n", SCREEN_W, SCREEN_H);
  for (size_t i = 0; i < scenes.size(); i++) {
    const Scene &s = scenes[i];
    std::printf("{\"name\": \"%s\", \"title\": \"%s\", \"locked\": %s, \"items\": [\n", esc(s.name).c_str(), esc(s.title).c_str(), s.locked ? "true" : "false");
    for (size_t k = 0; k < s.items.size(); k++) {
      const Item &it = s.items[k];
      std::printf("  {\"what\": \"%s\", \"name\": \"%s\", \"x\": %d, \"y\": %d, \"w\": %d, \"h\": %d, \"fill\": %s, \"ink\": [%d, %d, %d], "
                  "\"text\": \"%s\", \"px\": %d, \"sub\": \"%s\", \"sub_px\": %d, \"align\": \"%s\"}%s\n",
                  it.what, esc(it.name).c_str(), it.r.x, it.r.y, it.r.w, it.r.h,
                  it.fill_r < 0 ? "null" : (std::string("[") + std::to_string(it.fill_r) + "," + std::to_string(it.fill_g) + "," + std::to_string(it.fill_b) + "]").c_str(),
                  it.ink_r, it.ink_g, it.ink_b, esc(it.text).c_str(), it.px, esc(it.sub).c_str(), it.sub_px, it.align,
                  k + 1 < s.items.size() ? "," : "");
    }
    std::printf("]}%s\n", i + 1 < scenes.size() ? "," : "");
  }
  std::printf("]}\n");
}

int main() {
  std::vector<Scene> scenes;

  HomeState stopped;
  stopped.configured = true;
  stopped.engaged = true;
  scenes.push_back(homeScene("home-stopped", "First page: stopped, an alternation is set up", stopped));

  HomeState tests_only = stopped;
  tests_only.configured = false;
  scenes.push_back(homeScene("home-stopped-tests-only", "First page: stopped, no alternation set up", tests_only));

  HomeState testing;
  testing.run = Run::Plan; testing.configured = true; testing.engaged = true; testing.have_speed = true; testing.mph = 24;
  testing.plan_mode = "tests"; testing.target = true; testing.target_window = 0; testing.target_secs = 200; testing.tests_active = true;
  scenes.push_back(homeScene("home-testing", "First page: a test is counting at 24 mph", testing));

  HomeState hold = testing;
  hold.mph = 33; hold.tests_active = false;
  scenes.push_back(homeScene("home-hold-speed", "First page: speed to hold", hold, true));

  HomeState waiting = testing;
  waiting.tests_active = false; waiting.gate_reason = "curve";
  HomeState gap = testing;                               // the alternation complete: the bands are the windows, 41-49 mph counts nothing
  gap.mph = 45; gap.tests_active = false;
  gap.configured = true;
  scenes.push_back(homeScene("home-gap-alternation-done", "First page: alternation done, 45 mph is between the windows, nothing counts", gap));
  scenes.push_back(homeScene("home-waiting-straight", "First page: waiting for a straight road", waiting));

  HomeState ab;
  ab.run = Run::Plan; ab.configured = true; ab.engaged = true; ab.have_speed = true; ab.mph = 45; ab.plan_mode = "ab"; ab.ab_fresh = true;
  ab.ab_state = "running"; ab.ab_arm = "B"; ab.ab_block = true; ab.ab_block_left_s = 252;
  scenes.push_back(homeScene("home-ab-running", "First page: A/B relay running, arm B", ab));

  HomeState done = ab;
  done.ab_block = false; done.ab_complete = true; done.ab_state = "complete";
  scenes.push_back(homeScene("home-done", "First page: A/B complete", done));

  HomeState held = testing;
  held.plan_mode = "dead";
  scenes.push_back(homeScene("home-held", "First page: held by an error", held));

  HomeState single = stopped;
  single.run = Run::Single; single.have_speed = true; single.mph = 24; single.target = true; single.target_window = 0; single.target_secs = 200;
  single.tests_active = true;
  scenes.push_back(homeScene("home-single-test-on", "First page: a single test is on; START replaces it and says so", single));

  HomeState resume = stopped;
  resume.ab_stored_known = true; resume.ab_stored_a_s = 14.7 * 60; resume.ab_stored_b_s = 13.2 * 60;
  scenes.push_back(homeScene("home-stopped-resume", "First page: stopped after a fuel stop, progress kept", resume));

  HomeState dead = testing;
  dead.mph = 30; dead.tests_active = false;
  scenes.push_back(homeScene("home-nothing-counts", "First page: a speed where nothing counts", dead));

  scenes.push_back(progressScene());
  scenes.push_back(setupScene());
  scenes.push_back(testsScene());
  scenes.push_back(overviewScene());
  scenes.push_back(abScene());
  emit(scenes);
  return 0;
}
