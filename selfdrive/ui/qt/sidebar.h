#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <QElapsedTimer>
#include <QFrame>
#include <QMap>
#include <QMovie>
#include <QTimer>
#include <QWidget>

#include "selfdrive/ui/qt/ti_tests_logic.h"
#include "selfdrive/ui/ui.h"

typedef QPair<QPair<QString, QString>, QColor> ItemStatus;
Q_DECLARE_METATYPE(ItemStatus);

// Mazda TI on-car tests. The definitions are data (selfdrive/car/mazda/ti_tests.json, read at runtime), the car side
// is selfdrive/car/mazda/ti_tests.py, and both validate the file the same way. Nothing here sets a number or a limit:
// a test can only switch the allow-listed boolean toggles (TI_TEST_ALLOWED in sidebar.cc), exactly as the Settings
// toggles do.
struct TiTestBin {
  double low = 0;
  double high = 0;
  QString label;
  bool required = false;
  int steps = 0;                               // step_seconds: steps run in this bin; else 0
  std::vector<double> needs;                   // step_seconds: seconds each step needs (0: optional, extra only)
};

struct TiTestDef {
  QString id;
  QString title;
  std::vector<QPair<QString, bool>> settings;  // param key, the value that means "on"
  QString metric;
  double target = 0;
  std::vector<TiTestBin> bins;                 // sorted by low; empty: one bar
  bool off_at_ignition_off = false;
};

struct TiTestView {
  QString state;                   // OFF, PARTLY ON, ON · WAIT, ON · ACTIVE, NOT APPLIED
  QColor color;
  bool light = false;              // a light colour: black text on it
  bool requested = false;          // every setting at its "on" value in params
  bool any_on = false;
  bool active = false;             // the car side counted this test's metric in its last interval
  QString status;                  // the car side's status line; empty when absent or stale
  QString arm;                     // the car side's arm label; empty when absent or stale
  QMap<QString, double> seconds;   // bin label, "other" or "all" -> seconds collected
  QMap<QString, double> step_seconds;  // step_seconds tests: bin label -> its steps' seconds, each clipped to the target
  QMap<QString, double> step_extra;    // step_seconds tests: bin label -> its steps' seconds beyond the target
  QMap<QString, std::vector<double>> step_have;  // step_seconds tests: window label -> seconds collected per step
  int mismatch_ticks = 0;
};

// What closes the armed test's gate right now (the car side's TiTestGate block): a command, bend or lateral-demand
// limit, with its value and its limit. valid only while fresh.
struct TiTestGate {
  bool valid = false;
  QString test;     // setting key of the test (TiLadder ...)
  QString reason;   // command, curve, lat
  QString what;     // "steering command"
  QString unit;     // counts, 1/m, m/s2
  QString window;
  double value = 0;
  double limit = 0;
  bool rearm = false;    // back under the limit, waiting out the re-arm dwell
  bool running = false;  // the step still to capture waits; other steps of the test run meanwhile
};

class TiTests {
public:
  void load();
  void poll(const QJsonObject &toggles);
  void toggle(int i);
  void toggleAll();
  void allOff();
  void resetProgress(int i);
  void driveEnded();
  int queued();
  int queueSize() const;
  bool moveQueue(int from, int to);
  bool hasStepTests() const;
  bool overviewQueued();
  void openOverview();
  void closeOverview(int to);
  void startAll();
  bool locked();
  void setLocked(bool on);
  bool windowLeft(int i, const QString &label, double &secs, int &cells) const;
  ti_logic::Target target(const std::vector<int> &scope, double mph) const;

  bool done(int i) const;
  int countOn() const;
  int overallPercent() const;
  double have(int i, const TiTestBin &bin) const;
  static double need(const TiTestDef &def, const TiTestBin &bin);
  static QString binText(const TiTestBin &bin, bool metric);
  static QString binFor(const TiTestDef &def, double speed_mph);

  std::vector<TiTestDef> defs;
  std::vector<TiTestView> views;
  TiTestView all_view;  // the run-all mode (TiIdentAll): state and the car side's _live.all status; no seconds
  QString all_test;     // the test the run-all mode is running now (_live.all.test); empty: none or stale
  TiTestGate gate;      // the limit closing the armed test's gate now
  QStringList errors;

private:
  void setTest(int i, bool on);
  void setAll(bool on);

  QStringList logged_errors;
  QMap<QString, qint64> pending_resets;  // test id -> reset token, until the car side echoes it
  int reset_polls = 0;
  Params tests_params;
  Params tests_params_memory{"/dev/shm/params"};
};

// Full-screen Tests page, opened from the sidebar's TESTS card on the driving screen. It can be LOCKED on screen
// (ti_tests_logic.h): then it has no idle close and no BACK, but the end of a drive and every alert still take the
// screen, and it comes back by itself, locked, once no alert has shown for a moment. A page of MainWindow's stack,
// the way SettingsWindow is (window.cc). Painted and hit-tested like the sidebar; no signals or slots of its own, so
// no Q_OBJECT (as ScreenRefreshOverlay, frogpilot/ui/qt/offroad/screen_refresh.h).
class TestsWindow : public QWidget {
public:
  explicit TestsWindow(QWidget *parent = nullptr);
  bool openPage();
  bool wantsReopen(bool home_showing);
  bool isLocked() const { return locked; }

  std::function<void()> on_close;

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private:
  void leave();
  void tick();
  void refresh();
  int rowTop() const;
  int rowHeight() const;
  QRect backRect() const;
  QRect allOffRect() const;
  QRect prevRect() const;
  QRect nextRect() const;
  QRect runAllRect() const;
  QRect lockRect() const;
  QRect bannerRect() const;
  QRect toggleRect() const;
  QRect resetRect() const;
  int controlAt(const QPoint &pos) const;
  void drawBox(QPainter &p, const QRect &r, const QColor &fill, const QColor &ink, const QString &text, int px) const;
  void drawToggle(QPainter &p, const TiTestView &view, bool off_at_ignition_off) const;
  void drawTest(QPainter &p, int i);
  void drawAll(QPainter &p);
  int drawGuidance(QPainter &p, int x, int y, int w, const std::vector<int> &scope);
  int drawGate(QPainter &p, int x, int y, int w, const QString &key);

  TiTests tests;
  QTimer *ticker;
  QElapsedTimer idle;
  QElapsedTimer opened;          // input is ignored for a moment after the page opens
  int pressed_control = 0;       // the control under the last accepted press; 0: none
  int queue_pos = -1;            // the queued test on the page; -1: read the queue at the next refresh
  bool overview = false;         // the run-all overview page instead of a test's page
  bool locked = false;           // the page is locked on screen (param TiTestLock, tmpfs)
  QString overview_banner;       // set by drawAll for the banner row
  ti_logic::LockReopen reopen;
  int tick_count = 0;
  bool is_metric = false;
  bool have_speed = false;
  double speed_mph = 0;
};

class Sidebar : public QFrame {
  Q_OBJECT
  Q_PROPERTY(ItemStatus connectStatus MEMBER connect_status NOTIFY valueChanged);
  Q_PROPERTY(ItemStatus pandaStatus MEMBER panda_status NOTIFY valueChanged);
  Q_PROPERTY(ItemStatus tempStatus MEMBER temp_status NOTIFY valueChanged);
  Q_PROPERTY(QString netType MEMBER net_type NOTIFY valueChanged);
  Q_PROPERTY(int netStrength MEMBER net_strength NOTIFY valueChanged);

  // FrogPilot properties
  Q_PROPERTY(ItemStatus chipStatus MEMBER chip_status NOTIFY valueChanged)
  Q_PROPERTY(ItemStatus memoryStatus MEMBER memory_status NOTIFY valueChanged)
  Q_PROPERTY(ItemStatus storageStatus MEMBER storage_status NOTIFY valueChanged)
  Q_PROPERTY(ItemStatus testStatus MEMBER test_status NOTIFY valueChanged)

public:
  explicit Sidebar(QWidget* parent = 0);

signals:
  void openSettings(int index = 0, const QString &param = "");
  void openTests();
  void valueChanged();

public slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s, const FrogPilotUIState &fs);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void drawMetric(QPainter &p, const QPair<QString, QString> &label, QColor c, int y);
  void drawTestCard(QPainter &p);

  QPixmap home_img, flag_img, settings_img;
  bool onroad, flag_pressed, settings_pressed;
  const QMap<cereal::DeviceState::NetworkType, QString> network_type = {
    {cereal::DeviceState::NetworkType::NONE, tr("--")},
    {cereal::DeviceState::NetworkType::WIFI, tr("Wi-Fi")},
    {cereal::DeviceState::NetworkType::ETHERNET, tr("ETH")},
    {cereal::DeviceState::NetworkType::CELL2_G, tr("2G")},
    {cereal::DeviceState::NetworkType::CELL3_G, tr("3G")},
    {cereal::DeviceState::NetworkType::CELL4_G, tr("LTE")},
    {cereal::DeviceState::NetworkType::CELL5_G, tr("5G")}
  };

  const QRect home_btn = QRect(60, 860, 180, 180);
  const QRect settings_btn = QRect(50, 35, 200, 117);
  const QRect test_rect = QRect(30, 654, 240, 186);
  const QColor good_color = QColor(255, 255, 255);
  const QColor warning_color = QColor(218, 202, 37);
  const QColor danger_color = QColor(201, 34, 49);

  ItemStatus connect_status, panda_status, temp_status;
  QString net_type;
  int net_strength = 0;

  // FrogPilot variables
  ItemStatus chip_status, memory_status, storage_status, test_status;

private:
  std::unique_ptr<PubMaster> pm;

  // FrogPilot variables
  void showEvent(QShowEvent *event);
  void updateTheme();
  void updateTestStatus();

  bool isCPU;
  bool isDeveloperUI;
  bool isFahrenheit;
  bool isGPU;
  bool isIP;
  bool isMemoryUsage;
  bool isNumericalTemp;
  bool isStorageLeft;
  bool isStorageUsed;

  // Mazda TI tests card (TI car at tuning level 3, driving screen only): opens the Tests page
  bool showTest = false;
  bool tests_pressed = false;

  int testTick = UI_FREQ - 1;

  TiTests tiTests;

  Params params;

  QColor sidebar_color1;
  QColor sidebar_color2;
  QColor sidebar_color3;

  QSharedPointer<QMovie> flag_gif;
  QSharedPointer<QMovie> home_gif;
  QSharedPointer<QMovie> settings_gif;
};
