// The cruise set-speed check's page logic (selfdrive/ui/qt/ti_pages_logic.h cruiseLook) on statuses given one per line, for the
// property test in selfdrive/car/tests/test_mazda_ti_cruise_check.py (the controller's published status against the screen's state).
// Input line, tab separated ("-" for empty): have fresh state why phase round rows_a rows_b orig set others result
// Output line: word <tab> result line <tab> RUN CHECK enabled (0/1)
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
    if (f == "-") f.clear();
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
    if (f.size() < 12) {
      std::printf("bad\t\t0\n");
      continue;
    }
    ti_pages::CruiseView v;
    v.have = f[0] == "1";
    v.fresh = f[1] == "1";
    v.state = f[2];
    v.why = f[3];
    v.phase = f[4].empty() ? "A" : f[4];
    v.round = std::atoi(f[5].c_str());
    v.a = ti_pages::parseCruiseRounds(f[6]);
    v.b = ti_pages::parseCruiseRounds(f[7]);
    v.orig = std::atoi(f[8].c_str());
    v.set = std::atoi(f[9].c_str());
    v.result = f[11];
    const ti_pages::CruiseLook k = ti_pages::cruiseLook(v, f[10]);
    std::printf("%s\t%s\t%d\n", k.word.c_str(), k.result.c_str(), k.run_enabled ? 1 : 0);
  }
  return 0;
}
