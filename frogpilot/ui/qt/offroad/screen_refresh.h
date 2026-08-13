#pragma once

#include <functional>

#include <QElapsedTimer>
#include <QWidget>

// Fullscreen pixel exerciser for OLED burn-in. Cycles solid primaries, white and black so every
// subpixel spends equal time lit, then sweeps inversion bars to break up retained charge.
//
// Declared in its own header, with no openpilot dependencies, so both the Utilities panel and
// HomeWindow can construct it without dragging the settings headers into the onroad UI.
//
// Has no signals or slots of its own, so it needs no Q_OBJECT -- the timer connects via a lambda.
// Implemented in utilities.cc.
class ScreenRefreshOverlay : public QWidget {
public:
  explicit ScreenRefreshOverlay(int seconds, QWidget *parent = nullptr);

  // Called on close with the seconds actually completed, whether it ran out or was interrupted.
  // Lets the caller bank partial progress so a refresh cut short resumes rather than restarting.
  std::function<void(int)> on_finished;

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void closeEvent(QCloseEvent *event) override;

private:
  QElapsedTimer elapsed;
  int total_ms;
};
