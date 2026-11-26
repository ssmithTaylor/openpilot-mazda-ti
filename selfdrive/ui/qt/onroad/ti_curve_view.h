#pragma once

// Curve slowing's driving-screen element (owner's approved design, 2026-11-26, mock-up round 5): the road-ahead ribbon drawn straight
// on the camera view, on the right-hand side clear of every other element (owner, 2026-11-26), and the chip beside the MAX box on
// approach. Every decision comes from ti_curve_view_logic.h; this class only
// keeps the parsed payload and the smoothed fit between frames and paints. Qt Gui only (no cereal, no params): the offscreen renderer
// (selfdrive/ui/tests/render_ti_curve_view.cc) draws with this very code.

#include <string>

#include <QImage>
#include <QPainter>
#include <QRect>
#include <QRectF>

#include "selfdrive/ui/qt/onroad/ti_curve_view_logic.h"

class TiCurveView {
public:
  // the planner's payload (tmpfs TiCurveView), read by the widget at most 10 Hz; parsed only when it changed
  void setPayload(const std::string &text);
  // per frame: decide (visibility, chip, where the ribbon goes) and paint. now: CLOCK_MONOTONIC seconds (the planner's clock);
  // screen: the camera widget's size and what else is on it, in the painter's coordinates (ti_view::Screen; the MAX box included);
  // origin: where the painter's (0, 0) is in the onroad window (the onroad border); dt: seconds since the last frame.
  // -> what was decided (ti_view::Hidden)
  int draw(QPainter &p, double now, bool engaged, double ui_mph, const ti_view::Screen &screen, const QPoint &origin, double dt);

  const ti_view::Payload &payload() const { return payload_; }
  const ti_view::Fit &fit() const { return fit_; }
  // the ribbon's rectangle at the last draw (the painter's coordinates); empty when no ribbon was drawn
  const ti_view::Rect &lastRibbonRect() const { return rect_; }
  static double monotonicNow();

private:
  void drawRibbon(QPainter &painter, const QRectF &clip);
  void drawChip(QPainter &p, const ti_view::Chip &c, const QRect &max_box);

  std::string text_;
  ti_view::Payload payload_;
  ti_view::Ribbon ribbon_;
  ti_view::Style style_;
  ti_view::Fit fit_;
  double px_[ti_view::MAX_PTS] = {};
  double py_[ti_view::MAX_PTS] = {};
  int marks_[ti_view::MAX_PTS] = {};
  QImage layer_;            // the ribbon is painted here first (pixels replaced, not blended: no darker joins), then composited once
  bool shown_ = false;
  ti_view::Rect rect_;
};
