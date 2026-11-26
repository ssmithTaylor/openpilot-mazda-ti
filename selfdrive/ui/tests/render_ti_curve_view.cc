// Offscreen renderer of curve slowing's driving-screen element with the REAL drawing code (selfdrive/ui/qt/onroad/ti_curve_view.cc),
// for comparing with the approved mock-ups. Needs only Qt Gui (no cereal, no params, no message queues): built and run in the Docker
// image (ti-pages-build:focal), see tools/mazda_ti_screen/render_proof.py.
//
//   render_ti_curve_view <fonts dir> <moments dir> <out dir>
// <moments dir>/<name>.txt: one tick per line "<ui mph> <payload JSON>" (the planner's TiCurveView as written at 5 Hz, oldest first),
// <moments dir>/<name>_<bg>.png: the background (camera + the HUD's MAX box and speed), <bg> = dark / bright.
// Each moment is stepped through all its ticks at 20 frames per second (the UI's rate) so the fit is smoothed as on the car; the
// last frame (age 0, as the mock-ups) is drawn on each background -> <out dir>/<name>_<bg>.png. Prints the draw time per frame.
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <QDir>
#include <QElapsedTimer>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QImage>
#include <QPainter>

#include "selfdrive/ui/qt/onroad/ti_curve_view.h"

int main(int argc, char **argv) {
  if (argc < 4) {
    std::fprintf(stderr, "usage: render_ti_curve_view <fonts dir> <moments dir> <out dir>\n");
    return 2;
  }
  QGuiApplication app(argc, argv);
  for (const char *f : {"Bold", "SemiBold", "Medium", "Regular"}) {
    QFontDatabase::addApplicationFont(QString(argv[1]) + "/Inter-" + f + ".ttf");
  }
  const QString in_dir = argv[2], out_dir = argv[3];
  QDir().mkpath(out_dir);
  const QRect max_box(60, 45, 172, 204);              // the HUD's MAX box in the onroad widget (annotated_camera.cc drawHud)
  const QPoint origin(ti_view::BORDER, ti_view::BORDER);
  std::vector<double> times;
  QImage scratch(ti_view::SCREEN_W, ti_view::SCREEN_H, QImage::Format_ARGB32_Premultiplied);
  const QStringList names = QDir(in_dir).entryList({"*.txt"}, QDir::Files, QDir::Name);
  for (const QString &file : names) {
    const QString name = file.left(file.size() - 4);
    std::vector<std::pair<double, std::string>> ticks;
    std::ifstream f((in_dir + "/" + file).toStdString());
    std::string line;
    while (std::getline(f, line)) {
      const size_t sp = line.find(' ');
      if (sp == std::string::npos) continue;
      ticks.emplace_back(std::stod(line.substr(0, sp)), line.substr(sp + 1));
    }
    if (ticks.empty()) continue;
    for (const char *bg : {"dark", "bright"}) {
      QImage img(in_dir + "/" + name + "_" + bg + ".png");
      if (img.isNull()) continue;
      img = img.convertToFormat(QImage::Format_ARGB32_Premultiplied);
      TiCurveView view;
      for (size_t i = 0; i < ticks.size(); i++) {
        view.setPayload(ticks[i].second);
        const double m = view.payload().m;
        const bool last = i + 1 == ticks.size();
        for (int j = 0; j < 4; j++) {
          const bool final_frame = last && j == 3;
          const double now = final_frame ? m : m + 0.05 * j;
          QImage &target = final_frame ? img : scratch;
          if (!final_frame) scratch.fill(Qt::transparent);
          QPainter p(&target);
          p.translate(origin);                         // the onroad widget's coordinates, as in AnnotatedCameraWidget::paintEvent
          QElapsedTimer t;
          t.start();
          view.draw(p, now, true, ticks[i].first, max_box, origin, 0.05);
          times.push_back(t.nsecsElapsed() * 1e-6);
          p.end();
        }
      }
      img.convertToFormat(QImage::Format_RGB32).save(out_dir + "/" + name + "_" + bg + ".png");
      std::printf("%s_%s: %zu ticks, fit jumps %d\n", name.toStdString().c_str(), bg, ticks.size(), view.fit().jumps);
    }
  }
  if (!times.empty()) {
    std::sort(times.begin(), times.end());
    double sum = 0;
    for (double x : times) sum += x;
    std::printf("draw per frame (ms, this PC, raster 2160x1080, %zu frames): mean %.3f p99 %.3f max %.3f\n", times.size(),
                sum / times.size(), times[(size_t)(0.99 * (times.size() - 1))], times.back());
  }
  return 0;
}
