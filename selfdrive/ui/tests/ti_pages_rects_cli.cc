// Prints every touch target of every on-car Tests page (ti_pages_logic.h layout(), the rectangles sidebar.cc hit-tests with), one
// per line: "<page> <error lines> <control id> <name> <x> <y> <w> <h>", plus the fixed rectangles of the first page. Compared by
// selfdrive/car/tests/test_mazda_ti_pages_rects.py with the copy taken before the restyle (ti_pages_rects_golden.txt).
#include <cstdio>

#include "selfdrive/ui/qt/ti_pages_logic.h"

using namespace ti_pages;

int main() {
  const Page pages[] = {Page::Home, Page::Setup, Page::Tests, Page::Overview, Page::Ab, Page::Progress, Page::Cruise, Page::Curve,
                        Page::Buttons};
  for (int pg = 0; pg < (int)(sizeof(pages) / sizeof(pages[0])); pg++) {
    for (int errors : {0, 1, 2, 3, 7}) {
      Control c[MAX_CONTROLS];
      const int n = layout(pages[pg], errors, c);
      for (int i = 0; i < n; i++) {
        std::printf("%d %d %d %s %d %d %d %d\n", pg, errors, c[i].id, c[i].name, c[i].r.x, c[i].r.y, c[i].r.w, c[i].r.h);
      }
    }
  }
  const Rect fixed[] = {HOME_PRIMARY, HOME_SECOND, HOME_EXPERIMENT, HOME_DETAIL, HOME_DETAIL2, HOME_BADGE};
  for (const Rect &r : fixed) std::printf("home %d %d %d %d\n", r.x, r.y, r.w, r.h);
  return 0;
}
