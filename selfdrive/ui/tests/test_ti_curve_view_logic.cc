// Standalone test of selfdrive/ui/qt/onroad/ti_curve_view_logic.h (no Qt): parsing the planner's payload, when the element is drawn,
// the chip's timing and colour, the ribbon's fit inside its fixed area (smoothing and forced re-fit), the colour classes, rings, hatch
// and fade. Built and run by selfdrive/car/tests/test_mazda_ti_curve_view_logic.py (g++ / MSVC) and in the Docker image (clang, the
// device build's warnings); prints "ok" and exits 0 on success.
#include <cstdio>
#include <cstdlib>
#include <string>

#include "selfdrive/ui/qt/onroad/ti_curve_view_logic.h"

using namespace ti_view;

static int failures = 0;
#define CHECK(c) do { if (!(c)) { if (failures < 40) std::printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while (0)

// a payload as ti_curve_view.payload writes it: one left bend (red) and one right bend (yellow) ahead at 45 mph
static std::string sample(double m = 100.0, int on = 1, int start_red = 300) {
  std::string r = "[";
  for (int i = 0; i <= 90; i++) {
    const int d = i * 10;
    const int k = (d >= start_red && d <= start_red + 60) ? 3000 : ((d >= 700 && d <= 760) ? -1200 : 0);
    r += (i ? "," : "") + std::string("[") + std::to_string(d) + "," + std::to_string(k) + "]";
  }
  r += "]";
  char b[400];
  std::snprintf(b, sizeof(b), "[[%d,%d,%d,1,20,30,2,%d,%d,135],[700,730,760,-1,33,33,1,650,790,100]]", start_red, start_red + 30,
                start_red + 60, start_red - 120, start_red + 90);
  char head[200];
  std::snprintf(head, sizeof(head), "{\"v\":1,\"m\":%.2f,\"seq\":7,\"on\":%d,\"mph\":450,\"fl\":30,\"r\":", m, on);
  return std::string(head) + r + ",\"re\":0,\"b\":" + b + "}";
}

static void parsing() {
  Payload p;
  CHECK(parse(sample(), p));
  CHECK(p.ok && p.v == 1 && p.seq == 7 && p.on == 1 && p.mph10 == 450 && p.floor == 30 && p.ends == 0);
  CHECK(p.n == 91 && p.d[90] == 900 && p.k[31] == 3000 && p.k[71] == -1200);
  CHECK(p.nb == 2 && p.b[0].cls == RED && p.b[0].need == 20 && p.b[0].lead_ds == 135 && p.b[1].dir == -1 && p.b[1].target == 33);
  // "m" and "mph", "r" and "re" are told apart
  CHECK(std::fabs(p.m - 100.0) < 1e-9);
  // malformed or another version: not ok, nothing drawn
  CHECK(!parse("", p) && !p.ok);
  CHECK(!parse("{\"v\":2,\"m\":1,\"seq\":1,\"on\":1,\"mph\":1,\"fl\":1,\"r\":[],\"re\":0,\"b\":[]}", p));
  CHECK(!parse("{\"v\":1,\"m\":1,\"seq\":1,\"on\":1,\"mph\":1,\"fl\":1,\"r\":[[1,2],[3]],\"re\":0,\"b\":[]}", p));
  CHECK(parse("{\"v\":1,\"m\":1,\"seq\":1,\"on\":0,\"mph\":1,\"fl\":25,\"r\":[],\"re\":0,\"b\":[]}", p) && p.n == 0 && p.nb == 0);
  // more points than the arrays hold: cut, never overrun
  std::string big = "{\"v\":1,\"m\":1,\"seq\":1,\"on\":1,\"mph\":450,\"fl\":25,\"r\":[";
  for (int i = 0; i < 300; i++) big += (i ? ",[" : "[") + std::to_string(i) + ",0]";
  big += "],\"re\":0,\"b\":[]}";
  CHECK(parse(big, p) && p.n == MAX_PTS);
}

static void visibilityRules() {
  Payload p;
  parse(sample(100.0), p);
  CHECK(visibility(p, 100.5, true, 45.0, false) == SHOWN);
  CHECK(visibility(p, 101.01, true, 45.0, true) == STALE);              // older than 1 s: gone, chip included
  CHECK(visibility(p, 99.5, true, 45.0, true) == STALE);                // from the future (another clock): never trusted
  CHECK(visibility(p, 100.5, false, 45.0, true) == NOT_ENGAGED);
  CHECK(visibility(p, 100.5, true, 18.9, false) == TOO_SLOW);           // appears only from 19 mph
  CHECK(visibility(p, 100.5, true, 18.9, true) == SHOWN);               // ... and stays down to 18.5
  CHECK(visibility(p, 100.5, true, 18.4, true) == TOO_SLOW);
  Payload off;
  parse(sample(100.0, 0), off);
  CHECK(visibility(off, 100.5, true, 45.0, true) == SWITCHED_OFF);
  Payload none;
  CHECK(visibility(none, 100.5, true, 45.0, true) == NO_DATA);
}

static void chipTiming() {
  Payload p;
  // red bend 300 m out at 45 mph (20.1 m/s): 14.9 s; its lead 13.5 s -> no chip yet; the yellow one 700 m out: 34.8 s
  parse(sample(100.0, 1, 300), p);
  CHECK(chipFor(p, 0.0).cls == NEUTRAL);
  // 1.5 s later (the data's age): 13.4 s -> red, the NEEDED speed, its direction
  Chip c = chipFor(p, 1.5);
  CHECK(c.cls == RED && c.mph == 20 && c.dir == 1 && c.secs == 13 && c.bend == 0);
  // red bend at 260 m: 12.9 s -> red at once
  parse(sample(100.0, 1, 260), p);
  CHECK(chipFor(p, 0.0).cls == RED);
  // only a yellow bend: from 10 s, the TARGET speed
  std::string y = "{\"v\":1,\"m\":1,\"seq\":1,\"on\":1,\"mph\":450,\"fl\":30,\"r\":[],\"re\":0,\"b\":[[205,220,240,-1,33,33,1,150,270,100]]}";
  parse(y, p);
  CHECK(chipFor(p, 0.0).cls == NEUTRAL);                                // 205 m = 10.2 s
  c = chipFor(p, 0.3);
  CHECK(c.cls == YELLOW && c.mph == 33 && c.secs == 10 && c.dir == -1);
  // inside the bend (start 0): "0 s", still shown until the planner drops the bend
  y = "{\"v\":1,\"m\":1,\"seq\":1,\"on\":1,\"mph\":400,\"fl\":30,\"r\":[],\"re\":0,\"b\":[[0,10,60,1,33,33,1,0,90,100]]}";
  parse(y, p);
  c = chipFor(p, 0.9);
  CHECK(c.cls == YELLOW && c.secs == 0);
  // red wins over a nearer yellow
  y = "{\"v\":1,\"m\":1,\"seq\":1,\"on\":1,\"mph\":450,\"fl\":30,\"r\":[],\"re\":0,\"b\":[[100,110,120,1,33,33,1,50,150,100],"
      "[250,260,270,-1,20,30,2,10,300,140]]}";
  parse(y, p);
  CHECK(chipFor(p, 0.0).cls == RED && chipFor(p, 0.0).bend == 1);
  // the chip's place: right of the MAX box, level with its top
  const Box b = chipBox(Chip{RED, 0, 20, 13, 1}, 262, 75);
  CHECK(b.x == 282 && b.y == 75 && b.w == RED_W && b.h == RED_H);
}

static Rect AREA;   // the default fit's area (the stock screen's ribbon rectangle less INK, screen coordinates)

static bool allInside(const double *px, const double *py, int n) {
  for (int i = 0; i < n; i++) {
    if (px[i] < AREA.x0 - 2 || px[i] > AREA.x1 + 2 || py[i] < AREA.y0 - 2 || py[i] > AREA.y1 + 2) return false;
  }
  return true;
}

static void fitRule() {
  Payload p;
  parse(sample(), p);
  Ribbon r;
  ribbonRaw(p, r);
  CHECK(r.n == 91 && r.y[r.n - 1] > 0.0);
  // the heading change the ribbon integrates: 30 % sharper than true
  double th = 0;
  for (int i = 1; i < r.n; i++) th += EXAG * p.k[i] * 1e-5 * (r.d[i] - r.d[i - 1]);
  CHECK(std::fabs(th - (1.3 * 0.03 * 70 - 1.3 * 0.012 * 70)) < 0.15);     // 7 points of 10 m each
  // the near road is enlarged: the first 100 m take more screen length than the last 100 m
  double near = 0, far = 0;
  for (int i = 1; i <= 10; i++) near += std::hypot(r.x[i] - r.x[i - 1], r.y[i] - r.y[i - 1]);
  for (int i = r.n - 10; i < r.n; i++) far += std::hypot(r.x[i] - r.x[i - 1], r.y[i] - r.y[i - 1]);
  CHECK(near > 2.0 * far);
  Fit fit;
  double px[MAX_PTS], py[MAX_PTS];
  fit.step(r, 0.05, px, py);
  CHECK(allInside(px, py, r.n));
  // the bounding box fills the area in its tighter direction (0.94)
  double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
  for (int i = 0; i < r.n; i++) { x0 = std::min(x0, px[i]); x1 = std::max(x1, px[i]); y0 = std::min(y0, py[i]); y1 = std::max(y1, py[i]); }
  const double fill = std::max((x1 - x0) / AREA.w(), (y1 - y0) / AREA.h());
  CHECK(std::fabs(fill - FIT_FILL) < 0.01);
  // through a drive: the bend moves nearer tick by tick (5 Hz data, 20 Hz frames); every frame stays inside, the size never collapses
  for (int t = 0; t < 400; t++) {
    const int start = 300 - (t / 4) * 4;
    parse(sample(100.0, 1, std::max(start, -80)), p);
    ribbonRaw(p, r);
    fit.step(r, 0.05, px, py);
    CHECK(allInside(px, py, r.n));
    x0 = 1e9; x1 = -1e9; y0 = 1e9; y1 = -1e9;
    for (int i = 0; i < r.n; i++) { x0 = std::min(x0, px[i]); x1 = std::max(x1, px[i]); y0 = std::min(y0, py[i]); y1 = std::max(y1, py[i]); }
    CHECK(std::max((x1 - x0) / AREA.w(), (y1 - y0) / AREA.h()) >= 0.5);
  }
  // smoothing: a different shape is approached over about a second, not at once, while it fits
  Fit f2;
  parse(sample(100.0, 1, 300), p);
  ribbonRaw(p, r);
  f2.step(r, 0.05, px, py);
  const double s0 = f2.s;
  Payload q;
  parse(sample(100.0, 1, 500), q);
  Ribbon r2;
  ribbonRaw(q, r2);
  Fit target;
  target.step(r2, 0.05, px, py);
  f2.step(r2, 0.05, px, py);
  if (f2.jumps == 0) {
    CHECK(std::fabs(f2.s - s0) < std::fabs(target.s - s0));
  }
  for (int i = 0; i < 100; i++) f2.step(r2, 0.05, px, py);
  CHECK(std::fabs(f2.s - target.s) < 0.01 * target.s);
  // a smoothed fit that would leave the area is replaced at once (containment over smoothness)
  Fit f3;
  parse(sample(100.0, 1, 300), p);
  ribbonRaw(p, r);
  f3.step(r, 0.05, px, py);
  Ribbon wide = r;
  for (int i = 0; i < wide.n; i++) wide.x[i] *= 4.0;
  f3.step(wide, 0.05, px, py);
  CHECK(f3.jumps == 1 && allInside(px, py, wide.n));
  // a reset (element hidden) starts at the fitting layout again
  f3.reset();
  f3.step(r, 0.05, px, py);
  CHECK(f3.init && allInside(px, py, r.n));
}

// ---- the layout (owner, 2026-11-26: "move the ribbon to the right-hand side and prevent it from overlapping") -------------------
static bool clearOfAll(const Screen &s, const Rect &rr) {
  Element obs[MAX_ELEMENTS];
  const int n = obstacles(s, obs);
  const Rect grown{rr.x0 - LAYOUT_GAP + 0.01, rr.y0 - LAYOUT_GAP + 0.01, rr.x1 + LAYOUT_GAP - 0.01, rr.y1 + LAYOUT_GAP - 0.01};
  for (int i = 0; i < n; i++) {
    if (overlaps(grown, obs[i].r)) {
      std::printf("  overlaps %s\n", obs[i].name);
      return false;
    }
  }
  return true;
}

static bool onScreen(const Screen &s, const Rect &rr) {
  return within(rr, Rect{(double)UI_BORDER, (double)UI_BORDER, s.w - (double)UI_BORDER, s.h - (double)UI_BORDER}) &&
         rr.x0 >= s.w / 2.0 + RIGHT_OF_SPEED && rr.w() >= RIBBON_MIN_W && rr.h() >= RIBBON_MIN_H && rr.w() <= RIBBON_MAX_W &&
         rr.h() <= RIBBON_MAX_H;
}

static void layoutRules() {
  // the stock screen (2100 x 1020 camera widget): right-hand side, full width, between the top-right buttons and the bottom row
  Screen s;
  Rect rr = ribbonRect(s);
  CHECK(!rr.empty() && rr.x1 == s.w - UI_BORDER && rr.w() == RIBBON_MAX_W);
  CHECK(rr.y0 >= UI_BORDER + BTN + LAYOUT_GAP && rr.y1 <= s.h - 2 * UI_BORDER - BTN - LAYOUT_GAP && rr.h() >= 500);
  CHECK(clearOfAll(s, rr) && onScreen(s, rr));
  // the old place (left, over the driver-monitoring icon and the distance button) is never chosen
  CHECK(rr.x0 > s.w / 2.0);
  // FrogPilot's pedal icons under the experimental button: the ribbon moves left of them (its full size fits there)
  s.pedals = true;
  rr = ribbonRect(s);
  CHECK(!rr.empty() && rr.x1 <= s.w - UI_BORDER - BTN - LAYOUT_GAP && rr.w() == RIBBON_MAX_W && clearOfAll(s, rr));
  // a mid alert: the ribbon ends above the banner (shorter)
  s.alert = ALERT_MID;
  rr = ribbonRect(s);
  CHECK(!rr.empty() && rr.y1 <= s.h - ALERT_MID_H + ALERT_MARGIN - LAYOUT_GAP && clearOfAll(s, rr));
  // ... and with a sweeping turn-signal image 250 px tall above the banner as well: only the top corner left of the buttons is free,
  // the ribbon shrinks into it
  s.signal = SIGNAL_SWEEP;
  s.signal_h = 250;
  s.alert_h = ALERT_MID_H - ALERT_MARGIN;
  rr = ribbonRect(s);
  CHECK(!rr.empty() && rr.y1 <= s.h - s.alert_h - s.signal_h - LAYOUT_GAP && rr.w() < RIBBON_MAX_W && clearOfAll(s, rr));
  // ... taller still (400 px): no room anywhere, no ribbon (never drawn over)
  s.signal_h = 400;
  CHECK(ribbonRect(s).empty());
  s.signal = SIGNAL_OFF;
  s.pedals = false;
  rr = ribbonRect(s);
  CHECK(!rr.empty() && rr.y1 <= s.h - ALERT_MID_H + ALERT_MARGIN - LAYOUT_GAP && clearOfAll(s, rr));
  // a full-screen alert: no ribbon and no chip
  s.alert = ALERT_FULL;
  CHECK(ribbonRect(s).empty() && !chipAllowed(s));
  s.alert = ALERT_SMALL;
  CHECK(chipAllowed(s) && !ribbonRect(s).empty());
  // the sidebar open (the camera widget 300 px narrower): still on the right, still clear
  Screen side;
  side.w = SCREEN_W - 300 - 2 * BORDER;
  rr = ribbonRect(side);
  CHECK(!rr.empty() && rr.x1 == side.w - UI_BORDER && clearOfAll(side, rr) && onScreen(side, rr));
  // the big map (camera a quarter of the width): no room on its right-hand side, no ribbon
  Screen big;
  big.w = SCREEN_W / 4 - BORDER;
  CHECK(ribbonRect(big).empty());
  // the drawing's reach beyond the points stays inside the rectangle: halo, car chevron (incl. its outline), rings
  CHECK((WIDTH_NEAR + HALO_EXTRA + 8) / 2 <= INK);
  CHECK(std::hypot(24.0, 26.0) + 1.5 <= INK && 30.0 + 1.5 <= INK);
  CHECK(WIDTH_NEAR / 2 + RING_EXTRA + 3.5 <= INK);
  // every state this fork can show: the ribbon is clear of every element and fully on screen, or there is none
  int cases = 0, drawn = 0;
  const int widths[] = {SCREEN_W - 2 * BORDER, SCREEN_W - 300 - 2 * BORDER, SCREEN_W / 2 - BORDER, SCREEN_W / 4 - BORDER};
  const Rect boxes[] = {Rect{60, 45, 232, 249}, Rect{35, 45, 258, 447}, Rect{46, 45, 246, 437}};   // MAX; + US sign (3 digits); + EU
  const Rect leads[] = {Rect(), Rect{700, 420, 1400, 480}, Rect{1500, 560, 2100, 620}, Rect{1200, 300, 1900, 360}};
  const int sig_sizes[][2] = {{0, 0}, {150, 150}, {300, 250}, {450, 400}};
  for (int w : widths) for (const Rect &mb : boxes) for (int alert = 0; alert <= 3; alert++) for (int ped = 0; ped <= 1; ped++)
    for (int sig = 0; sig <= 2; sig++) for (const auto &ss : sig_sizes) for (const Rect &lt : leads) {
      Screen t;
      t.w = w;
      t.max_box = mb;
      t.alert = alert;
      t.alert_h = alert == ALERT_SMALL ? ALERT_SMALL_H - ALERT_MARGIN : alert == ALERT_MID ? ALERT_MID_H - ALERT_MARGIN :
                  alert == ALERT_FULL ? t.h : 0;
      t.pedals = ped != 0;
      t.signal = sig;
      t.signal_w = ss[0];
      t.signal_h = ss[1];
      t.lead_text = lt;
      const Rect r = ribbonRect(t);
      cases++;
      if (r.empty()) continue;
      drawn++;
      CHECK(clearOfAll(t, r));
      CHECK(onScreen(t, r));
    }
  CHECK(cases == 4 * 3 * 4 * 2 * 3 * 4 * 4 && drawn > cases / 4);
  std::printf("layout: %d states, ribbon drawn in %d\n", cases, drawn);
}

static void colours() {
  Payload p;
  parse(sample(), p);
  Ribbon r;
  ribbonRaw(p, r);
  Style st;
  styleFor(p, r, st);
  // index i is d = 10 i: neutral before the approach, the red approach from 180 m (translucent, narrower), the red bend 300-360
  // (hatched), neutral, the yellow approach from 650, the yellow bend 700-760, neutral
  CHECK(st.cls[10] == NEUTRAL && !st.approach[10]);
  CHECK(st.cls[20] == RED && st.approach[20] && !st.hatch[20]);
  CHECK(st.cls[33] == RED && !st.approach[33] && st.hatch[33]);
  CHECK(st.cls[50] == NEUTRAL);
  CHECK(st.cls[66] == YELLOW && st.approach[66]);
  CHECK(st.cls[72] == YELLOW && !st.approach[72] && !st.hatch[72]);
  CHECK(alphaFor(st, 10) == ALPHA_NEUTRAL && alphaFor(st, 20) == ALPHA_APPROACH && alphaFor(st, 33) == ALPHA_BEND);
  CHECK(strokeWidth(st, 20) < st.width[20] && strokeWidth(st, 33) == st.width[33]);
  CHECK(st.width[0] == WIDTH_NEAR && std::fabs(st.width[r.n - 1] - (WIDTH_NEAR - WIDTH_TAPER)) < 1e-9);
  CHECK(st.fade[r.n - 1] == 1.0);                                         // the memory goes on: no fade
  // rings: after the red bend (resume 390; the yellow approach starts at 650: 260 m later) and after the yellow one (790)
  int rings[8];
  const int nr = resumeRings(p, r, rings, 8);
  CHECK(nr == 2 && rings[0] == 39 && rings[1] == 79);
  // the next approach within 150 m of the resume point: no ring there
  p.b[1].approach = 450;
  CHECK(resumeRings(p, r, rings, 8) == 1 && rings[0] == 79);
  // the remembered road ends: the last 60 m fade to nothing
  p.ends = 1;
  styleFor(p, r, st);
  CHECK(st.fade[r.n - 1] == 0.0 && std::fabs(st.fade[r.n - 4] - 0.5) < 1e-9 && st.fade[0] == 1.0);
  // hatch marks on the red stretch only, spaced on the screen
  Fit fit;
  double px[MAX_PTS], py[MAX_PTS];
  fit.step(r, 0.05, px, py);
  int marks[MAX_PTS];
  const int nm = hatchMarks(st, px, py, r.n, marks, MAX_PTS);
  CHECK(nm >= 1);
  for (int i = 0; i < nm; i++) CHECK(st.hatch[marks[i]]);
  // the car points along the ribbon (up the screen at the start of a straight)
  CHECK(std::fabs(carAngle(px, py, r.n) + PI / 2) < 0.2);
}

int main() {
  AREA = Fit().area;
  parsing();
  visibilityRules();
  chipTiming();
  fitRule();
  colours();
  layoutRules();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("ok\n");
  return 0;
}
