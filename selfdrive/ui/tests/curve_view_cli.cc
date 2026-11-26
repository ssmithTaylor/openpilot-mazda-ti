// Reads lines "<now> <engaged 0/1> <cruise active 0/1> <ui mph> <was shown 0/1> <payload JSON>" and prints, per line, what the driving screen's curve
// element decides (ti_curve_view_logic.h): "<visibility> <chip class> <chip mph> <chip secs> <chip dir> <chip bend> <ribbon points>".
// Used by selfdrive/car/tests/test_mazda_ti_curve_view_logic.py to check the C++ rules against the planner's own (ti_curve_view.py).
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#include "selfdrive/ui/qt/onroad/ti_curve_view_logic.h"

int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    double now = 0, mph = 0;
    int engaged = 0, cruise = 0, was = 0;
    in >> now >> engaged >> cruise >> mph >> was;
    std::string rest;
    std::getline(in, rest);
    ti_view::Payload p;
    ti_view::parse(rest, p);
    const int vis = ti_view::visibility(p, now, engaged != 0, cruise != 0, mph, was != 0);
    const ti_view::Chip c = ti_view::chipFor(p, now - p.m);
    ti_view::Ribbon r;
    ti_view::ribbonRaw(p, r);
    std::printf("%d %d %d %d %d %d %d\n", vis, c.cls, c.mph, c.secs, c.dir, c.bend, r.n);
  }
  return 0;
}
