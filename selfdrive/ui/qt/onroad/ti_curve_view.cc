#include "selfdrive/ui/qt/onroad/ti_curve_view.h"

#include <chrono>
#include <cmath>

#include <QPainterPath>
#include <QPolygonF>

using namespace ti_view;

namespace {

QColor rgb(const Rgb &c, int alpha = 255) { return QColor(c.r, c.g, c.b, alpha); }

// the device's font (selfdrive/ui/qt/util.h InterFont), without util.h's cereal dependencies
QFont inter(double px, QFont::Weight w = QFont::Bold) {
  QFont f("Inter");
  f.setPixelSize(std::max(1, (int)std::lround(px)));
  f.setWeight(w);
  return f;
}

// text anchored like PIL's 'lm' (left, vertical middle) and 'mm' (centre)
void textLm(QPainter &p, double x, double y, const QString &s, const QFont &f, const QColor &c) {
  p.setFont(f);
  p.setPen(c);
  const double h = f.pixelSize() * 2.0;
  p.drawText(QRectF(x, y - h / 2, 2000, h), Qt::AlignLeft | Qt::AlignVCenter, s);
}

void textMm(QPainter &p, double x, double y, const QString &s, const QFont &f, const QColor &c) {
  p.setFont(f);
  p.setPen(c);
  const double h = f.pixelSize() * 2.0;
  p.drawText(QRectF(x - 1000, y - h / 2, 2000, h), Qt::AlignHCenter | Qt::AlignVCenter, s);
}

// a rounded box whose outline (width w) lies inside the rectangle, as PIL draws it
void roundBox(QPainter &p, const QRectF &r, double radius, const QColor &fill, const QColor &edge, double w) {
  p.setPen(Qt::NoPen);
  p.setBrush(fill);
  p.drawRoundedRect(r, radius, radius);
  if (w > 0) {
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(edge, w));
    p.drawRoundedRect(r.adjusted(w / 2, w / 2, -w / 2, -w / 2), std::max(0.0, radius - w / 2), std::max(0.0, radius - w / 2));
  }
}

// MUTCD W1-2 style curve arrow in a box of half-size s: a thick stem rising from the bottom, bending to `side` (+1 left, -1 right)
void curveArrow(QPainter &p, double cx, double cy, double s, int side, const QColor &col) {
  const double d = -side;
  QPolygonF pts;
  for (int i = 0; i < 40; i++) {
    const double t = i / 39.0;
    const double y = cy + s * 0.95 - t * s * 1.45;
    const double q = std::max(0.0, (t - 0.3) / 0.7);
    pts << QPointF(cx - d * s * 0.28 + d * s * 0.62 * q * q, y);
  }
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(col, s * 0.34, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPolyline(pts);
  const QPointF a = pts[34], b = pts[39];
  const double ang = std::atan2(b.y() - a.y(), b.x() - a.x());
  const double L = s * 0.62;
  QPolygonF head;
  head << QPointF(b.x() + std::cos(ang) * L * 0.75, b.y() + std::sin(ang) * L * 0.75)
       << QPointF(b.x() + std::cos(ang + 1.75) * L * 0.62, b.y() + std::sin(ang + 1.75) * L * 0.62)
       << QPointF(b.x() + std::cos(ang - 1.75) * L * 0.62, b.y() + std::sin(ang - 1.75) * L * 0.62);
  p.setPen(Qt::NoPen);
  p.setBrush(col);
  p.drawPolygon(head);
}

void diamond(QPainter &p, double cx, double cy, double r, const QColor &fill, const QColor &edge, int side, const QColor &arrow) {
  QPolygonF outer, inner;
  outer << QPointF(cx, cy - r) << QPointF(cx + r, cy) << QPointF(cx, cy + r) << QPointF(cx - r, cy);
  const double ri = r - 9;
  inner << QPointF(cx, cy - ri) << QPointF(cx + ri, cy) << QPointF(cx, cy + ri) << QPointF(cx - ri, cy);
  p.setPen(Qt::NoPen);
  p.setBrush(fill);
  p.drawPolygon(outer);
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(edge, 5, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
  p.drawPolygon(inner);
  if (side != 0) {
    curveArrow(p, cx, cy, r * 0.55, side, arrow);
  } else {
    textMm(p, cx, cy, "!", inter(r * 1.1), arrow);     // a map bend: the direction is not known (its curvature is unsigned)
  }
}

void plaque(QPainter &p, double x, double y, double w, double h, int num, const QColor &fill, const QColor &ink) {
  roundBox(p, QRectF(x, y, w, h), 14, fill, fill, 0);
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(ink, 4));
  p.drawRoundedRect(QRectF(x + 8, y + 8, w - 16, h - 16), 8, 8);
  textMm(p, x + w / 2, y + h * 0.43, QString::number(num), inter(84 * h / 140), ink);
  textMm(p, x + w / 2, y + h * 0.82, "MPH", inter(30 * h / 140), ink);
}

// a generic hands-on-wheel pictogram: rim, hub, spokes, two hands gripping the rim at nine and three o'clock
void handsWheel(QPainter &p, double cx, double cy, double r, const QColor &col, const QColor &bg) {
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(col, r * 0.15));
  p.drawEllipse(QPointF(cx, cy), r - r * 0.075, r - r * 0.075);
  p.setPen(Qt::NoPen);
  p.setBrush(col);
  p.drawEllipse(QPointF(cx, cy), r * 0.22, r * 0.22);
  p.setPen(QPen(col, r * 0.13, Qt::SolidLine, Qt::FlatCap));
  const double spokes[3][2] = {{-1, 0}, {1, 0}, {0, 1}};
  for (const auto &a : spokes) {
    p.drawLine(QPointF(cx + a[0] * r * 0.2, cy + a[1] * r * 0.2), QPointF(cx + a[0] * r * 0.9, cy + a[1] * r * 0.9));
  }
  for (int sx : {-1, 1}) {
    const double hx = cx + sx * r * 0.95, hy = cy - r * 0.05;
    p.setBrush(col);
    p.setPen(QPen(bg, std::max(2.0, r * 0.06)));
    p.drawRoundedRect(QRectF(hx - r * 0.3, hy - r * 0.42, r * 0.6, r * 0.84), r * 0.22, r * 0.22);
    p.setPen(QPen(bg, std::max(2.0, r * 0.05), Qt::SolidLine, Qt::FlatCap));
    for (double f : {-0.2, 0.0, 0.2}) {
      p.drawLine(QPointF(hx + sx * r * 0.08, hy + f * r), QPointF(hx + sx * r * 0.3, hy + f * r));
    }
    p.setPen(Qt::NoPen);
    p.drawEllipse(QRectF(hx - sx * r * 0.18 - r * 0.13, hy - r * 0.5, r * 0.26, r * 0.24));
  }
}

// the sign row of both chips: curve sign, speed plaque, seconds to the bend
void signRow(QPainter &p, double x, double y, const Chip &c, bool red, double scale) {
  const double r = 70 * scale;
  const QColor white(255, 255, 255), black(0, 0, 0);
  if (red) {
    diamond(p, x + r + 6, y + r + 6, r, rgb(SIGN_RED), white, c.dir, white);
    plaque(p, x + 2 * r + 30, y + 6, 128 * scale, 140 * scale, c.mph, white, rgb(SIGN_RED));
  } else {
    diamond(p, x + r + 6, y + r + 6, r, rgb(SIGN_YELLOW), black, c.dir, black);
    plaque(p, x + 2 * r + 30, y + 6, 128 * scale, 140 * scale, c.mph, rgb(SIGN_YELLOW), black);
  }
  textLm(p, x + 2 * r + 30 + 128 * scale + 24, y + r + 6, QString("%1 s").arg(c.secs), inter(SECS_PX), white);
}

}  // namespace

double TiCurveView::monotonicNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void TiCurveView::setPayload(const std::string &text) {
  if (text == text_) return;
  text_ = text;
  parse(text_, payload_);
}

int TiCurveView::draw(QPainter &p, double now, bool engaged, double ui_mph, const Screen &screen, const QPoint &origin, double dt) {
  const Hidden vis = visibility(payload_, now, engaged, ui_mph, shown_);
  shown_ = vis == SHOWN;
  rect_ = Rect();
  if (!shown_) {
    fit_.reset();
    return vis;
  }
  p.save();
  p.translate(-origin);                   // draw in screen coordinates (the logic's; the widget's moved by the onroad border)
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setRenderHint(QPainter::TextAntialiasing, true);
  ribbonRaw(payload_, ribbon_);
  const Rect rr = ti_view::ribbonRect(screen);     // the right-hand side, clear of every other element; empty: no room, no ribbon
  if (ribbon_.n >= 2 && !rr.empty()) {
    rect_ = rr;
    fit_.setArea(pointsArea(rr, origin.x(), origin.y()));
    fit_.step(ribbon_, dt, px_, py_);
    styleFor(payload_, ribbon_, style_);
    drawRibbon(p, QRectF(rr.x0 + origin.x(), rr.y0 + origin.y(), rr.w(), rr.h()));
  } else {
    fit_.reset();
  }
  const Chip c = chipFor(payload_, now - payload_.m);
  if (c.cls != NEUTRAL && chipAllowed(screen)) {
    const Rect &mb = screen.max_box;
    drawChip(p, c, QRect((int)mb.x0, (int)mb.y0, (int)mb.w(), (int)mb.h()).translated(origin));
  }
  p.restore();
  return vis;
}

void TiCurveView::drawRibbon(QPainter &painter, const QRectF &clip) {
  const int n = ribbon_.n;
  // one layer over the ribbon's rectangle: strokes REPLACE its pixels, as the mock-ups' layer does, so translucent runs never
  // darken where they meet; the layer is then blended onto the camera view once. The layer IS the rectangle: nothing is drawn
  // outside it (the points sit INK inside it, so nothing is cut off either)
  const QPoint lo((int)std::floor(clip.x()), (int)std::floor(clip.y()));
  const QSize sz((int)std::ceil(clip.right()) - lo.x(), (int)std::ceil(clip.bottom()) - lo.y());
  if (layer_.size() != sz) {
    layer_ = QImage(sz, QImage::Format_ARGB32_Premultiplied);
  }
  layer_.fill(Qt::transparent);
  QPainter p(&layer_);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.translate(-lo);
  p.setCompositionMode(QPainter::CompositionMode_Source);
  // 1: the soft dark halo (legible on a bright road), two passes in place of a blur; segments of equal look drawn as one polyline
  auto strokeRuns = [&](auto widthOf, auto colourOf) {
    QPolygonF run;
    double w0 = -1;
    QColor c0;
    for (int i = 0; i + 1 < n; i++) {
      const double w = std::round(widthOf(i));
      const QColor c = colourOf(i);
      if (run.isEmpty() || w != w0 || c != c0) {
        if (run.size() > 1) {
          p.setPen(QPen(c0, w0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
          p.drawPolyline(run);
        }
        run.clear();
        run << QPointF(px_[i], py_[i]);
        w0 = w;
        c0 = c;
      }
      run << QPointF(px_[i + 1], py_[i + 1]);
    }
    if (run.size() > 1) {
      p.setPen(QPen(c0, w0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      p.drawPolyline(run);
    }
  };
  p.setBrush(Qt::NoBrush);
  strokeRuns([&](int i) { return style_.width[i] + HALO_EXTRA + 8; },
             [&](int i) { return QColor(0, 0, 0, (int)std::lround(ALPHA_HALO * 0.4 * style_.fade[i])); });
  strokeRuns([&](int i) { return style_.width[i] + HALO_EXTRA - 2; },
             [&](int i) { return QColor(0, 0, 0, (int)std::lround(ALPHA_HALO * 0.8 * style_.fade[i])); });
  // 2: the colour (neutral / yellow / red; a bend's approach narrower and translucent)
  strokeRuns([&](int i) { return strokeWidth(style_, i); },
             [&](int i) {
               const Rgb &c = style_.cls[i] == RED ? RED_RGB : (style_.cls[i] == YELLOW ? YELLOW_RGB : NEUTRAL_RGB);
               return rgb(c, alphaFor(style_, i));
             });
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  // 3: dark hatch across the red stretch (colour is never the only cue)
  const int nm = hatchMarks(style_, px_, py_, n, marks_, MAX_PTS);
  p.setPen(QPen(rgb(HATCH_RGB, 230), 6, Qt::SolidLine, Qt::FlatCap));
  for (int k = 0; k < nm; k++) {
    const int i = marks_[k];
    const double seg = std::max(std::hypot(px_[i] - px_[i - 1], py_[i] - py_[i - 1]), 1e-6);
    const double tx = (px_[i] - px_[i - 1]) / seg, ty = (py_[i] - py_[i - 1]) / seg;
    const double nx = -ty, ny = tx, h = style_.width[i] / 2;
    p.drawLine(QPointF(px_[i] + nx * h - tx * 8, py_[i] + ny * h - ty * 8), QPointF(px_[i] - nx * h + tx * 8, py_[i] - ny * h + ty * 8));
  }
  // 4: the speed-back-up rings
  int rings[MAX_BENDS];
  const int nr = resumeRings(payload_, ribbon_, rings, MAX_BENDS);
  p.setPen(QPen(rgb(GO_RGB), 7));
  for (int k = 0; k < nr; k++) {
    const int j = rings[k];
    const double r = style_.width[j] / 2 + RING_EXTRA;
    p.drawEllipse(QPointF(px_[j], py_[j]), r - 3.5, r - 3.5);
  }
  // 5: the car at the start, pointing along the ribbon
  const double ang = carAngle(px_, py_, n), ca = std::cos(ang), sa = std::sin(ang);
  auto P = [&](double fx, double fy) { return QPointF(px_[0] + fx * ca - fy * sa, py_[0] + fx * sa + fy * ca); };
  QPolygonF car;
  car << P(30, 0) << P(-24, 26) << P(-12, 0) << P(-24, -26);
  p.setBrush(QColor(255, 255, 255));
  p.setPen(QPen(QColor(0, 0, 0, 220), 3, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
  p.drawPolygon(car);
  p.end();
  painter.drawImage(lo, layer_);
}

void TiCurveView::drawChip(QPainter &p, const Chip &c, const QRect &max_box) {
  const Box b = chipBox(c, max_box.x() + max_box.width(), max_box.y());
  const double X0 = b.x, Y0 = b.y;
  if (c.cls == YELLOW) {
    roundBox(p, QRectF(X0, Y0, b.w, b.h), 32, QColor(0, 0, 0, 166), rgb(SIGN_YELLOW, 230), 6);
    signRow(p, X0 + 18, Y0 + 26, c, false, 1.0);
    return;
  }
  // red (v1): a two-line word band with the hands-on-wheel symbol over the red curve sign, the needed speed and the seconds
  const QColor band = rgb(SIGN_RED);
  roundBox(p, QRectF(X0, Y0, b.w, b.h), 32, QColor(0, 0, 0, 190), band, 6);
  QPainterPath top;
  top.addRoundedRect(QRectF(X0 + 3, Y0 + 3, b.w - 6, RED_BAND_H - 3), 29, 29);
  top.addRect(QRectF(X0 + 3, Y0 + RED_BAND_H - 30, b.w - 6, 30));
  p.setPen(Qt::NoPen);
  p.setBrush(band);
  p.drawPath(top.simplified());
  handsWheel(p, X0 + 78, Y0 + 88, 50, QColor(255, 255, 255), band);
  textLm(p, X0 + 160, Y0 + 52, "HANDS ON", inter(BAND_TEXT_PX), QColor(255, 255, 255));
  textLm(p, X0 + 160, Y0 + 128, "HELP STEER", inter(BAND_TEXT_PX), QColor(255, 255, 255));
  signRow(p, X0 + 24, Y0 + RED_BAND_H + 18, c, true, 0.8);
}
