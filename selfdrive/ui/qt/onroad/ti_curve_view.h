#pragma once

// Curve slowing's driving-screen element (owner's approved design, 2026-11-26, mock-up round 5): the road-ahead ribbon drawn straight
// on the camera view and the chip beside the MAX box on approach. Every decision comes from ti_curve_view_logic.h; this class only
// keeps the parsed payload and the smoothed fit between frames and paints. Qt Gui only (no cereal, no params): the offscreen renderer
// (selfdrive/ui/tests/render_ti_curve_view.cc) draws with this very code.

#include <string>

#include <QPainter>
#include <QRect>

#include "selfdrive/ui/qt/onroad/ti_curve_view_logic.h"

class TiCurveView {
public:
  // the planner's payload (tmpfs TiCurveView), read by the widget at most 10 Hz; parsed only when it changed
  void setPayload(const std::string &text);
  // per frame: decide (visibility, chip) and paint. now: CLOCK_MONOTONIC seconds (the planner's clock); max_box: the MAX box in
  // the painter's coordinates; origin: where the painter's (0, 0) is on the 2160 x 1080 screen (the onroad border); dt: seconds
  // since the last frame. -> what was decided (ti_view::Hidden)
  int draw(QPainter &p, double now, bool engaged, double ui_mph, const QRect &max_box, const QPoint &origin, double dt);

  const ti_view::Payload &payload() const { return payload_; }
  const ti_view::Fit &fit() const { return fit_; }
  static double monotonicNow();

private:
  void drawRibbon(QPainter &p);
  void drawChip(QPainter &p, const ti_view::Chip &c, const QRect &max_box);

  std::string text_;
  ti_view::Payload payload_;
  ti_view::Ribbon ribbon_;
  ti_view::Style style_;
  ti_view::Fit fit_;
  double px_[ti_view::MAX_PTS] = {};
  double py_[ti_view::MAX_PTS] = {};
  int marks_[ti_view::MAX_PTS] = {};
  bool shown_ = false;
};
