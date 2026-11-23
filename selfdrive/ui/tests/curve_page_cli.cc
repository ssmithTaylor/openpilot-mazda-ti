// Curve slowing's page logic (selfdrive/ui/qt/ti_pages_logic.h curveLook) on statuses given one per line, for the property test in
// selfdrive/car/tests/test_mazda_ti_curve_screen.py (the controller's published status against what the screen shows).
// Input line, tab separated ("-" for empty): have fresh state needs_more line1 line2
// Output line, tab separated: word primary1 primary2 second button write warn
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "selfdrive/ui/qt/ti_pages_logic.h"

static std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> out;
  size_t p = 0;
  while (true) {
    const size_t e = line.find('\t', p);
    out.push_back(line.substr(p, e == std::string::npos ? std::string::npos : e - p));
    if (e == std::string::npos) break;
    p = e + 1;
  }
  for (std::string &f : out) {
    while (!f.empty() && (f.back() == '\r' || f.back() == '\n')) f.pop_back();
    if (f == "-") f.clear();
  }
  return out;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <file>\n", argv[0]);
    return 2;
  }
  std::ifstream in(argv[1]);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const std::vector<std::string> f = split(line);
    if (f.size() < 6) {
      std::printf("bad\t\t\t\t\t\t0\n");
      continue;
    }
    ti_pages::CurveView v;
    v.have = f[0] == "1";
    v.fresh = f[1] == "1";
    v.state = f[2];
    v.needs_more = f[3] == "1";
    v.line1 = f[4];
    v.line2 = f[5];
    const ti_pages::CurveLook k = ti_pages::curveLook(v);
    std::printf("%s\t%s\t%s\t%s\t%s\t%s\t%d\n", k.word.c_str(), k.primary1.c_str(), k.primary2.c_str(), k.second.c_str(), k.button.c_str(),
                k.write, k.warn ? 1 : 0);
  }
  return 0;
}
