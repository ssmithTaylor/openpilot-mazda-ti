#pragma once

#include <QElapsedTimer>

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

// Fullscreen pixel exerciser for OLED burn-in. Cycles solid primaries, white and black so every
// subpixel spends equal time lit, then sweeps inversion bars to break up retained charge. Has no
// signals or slots of its own, so it needs no Q_OBJECT -- the timer connects via a lambda.
class ScreenRefreshOverlay : public QWidget {
public:
  explicit ScreenRefreshOverlay(int seconds, QWidget *parent = nullptr);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;

private:
  QElapsedTimer elapsed;
  int total_ms;
};

class FrogPilotUtilitiesPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotUtilitiesPanel(FrogPilotSettingsWindow *parent);

private:
  FrogPilotSettingsWindow *parent;

  Params params;
  Params params_memory{"/dev/shm/params"};
};
