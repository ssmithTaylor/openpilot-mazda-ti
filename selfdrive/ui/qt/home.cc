#include "selfdrive/ui/qt/home.h"

#include <algorithm>

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QStackedWidget>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/offroad/experimental_mode.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/prime.h"

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "frogpilot/ui/qt/widgets/drive_stats.h"
#include "frogpilot/ui/qt/widgets/drive_summary.h"
#include "frogpilot/ui/qt/widgets/model_reviewer.h"

// HomeWindow: the container for the offroad and onroad UIs

namespace {
  // Two pixels is enough to move wear off a given emitter and far too little to notice. The view
  // loses 2*PIXEL_SHIFT of width and height so the content can slide inside a fixed outer size --
  // resizing instead would relayout everything and be plainly visible.
  constexpr int PIXEL_SHIFT = 2;
  constexpr int PIXEL_SHIFT_TICKS = 60 * UI_FREQ;

  // Long enough that the driver has gathered their things and walked away. The device stays up for
  // a while after ignition off while the car battery holds, so five minutes idle plus a five
  // minute pass sits comfortably inside that window -- and the whole thing happens unseen.
  constexpr qint64 AUTO_REFRESH_DELAY_S = 5 * 60;
  constexpr qint64 AUTO_REFRESH_DURATION_S = 5 * 60;
  constexpr qint64 AUTO_REFRESH_INTERVAL_S = 7 * 24 * 60 * 60;
  // Left unused at the end of the idle window, so a pass is not racing the power-down it was
  // scheduled around, and below which a pass is not worth starting at all.
  constexpr qint64 AUTO_REFRESH_MARGIN_S = 60;
  constexpr qint64 AUTO_REFRESH_MIN_RUN_S = 60;

  // FrogPilot's offroad shutdown setting, in the same shape frogpilot_variables computes it:
  // (setting - 3) hours from 4 up, setting * 15 minutes below. Setting 0 means the device powers
  // down as soon as it goes offroad, so there is no idle window to work in.
  qint64 idleWindowSeconds(int setting) {
    return setting >= 4 ? (qint64)(setting - 3) * 3600 : (qint64)setting * 15 * 60;
  }
}

void HomeWindow::updatePixelShift() {
  if (root_layout == nullptr || ++pixel_shift_tick < PIXEL_SHIFT_TICKS) {
    return;
  }
  pixel_shift_tick = 0;

  if (!params.getBool("PixelShift")) {
    root_layout->setContentsMargins(0, 0, 0, 0);
    return;
  }

  static const QPoint kOffsets[4] = {{0, 0}, {2 * PIXEL_SHIFT, 0},
                                     {2 * PIXEL_SHIFT, 2 * PIXEL_SHIFT}, {0, 2 * PIXEL_SHIFT}};
  pixel_shift_step = (pixel_shift_step + 1) % 4;
  const QPoint o = kOffsets[pixel_shift_step];
  root_layout->setContentsMargins(o.x(), o.y(), 2 * PIXEL_SHIFT - o.x(), 2 * PIXEL_SHIFT - o.y());
}

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  root_layout = main_layout;
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome(this);
  QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  QObject::connect(onroad, &OnroadWindow::mapPanelRequested, this, [=] { sidebar->hide(); });
  slayout->addWidget(onroad);

  body = new BodyWindow(this);
  slayout->addWidget(body);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::offroadTransition, sidebar, &Sidebar::offroadTransition);

  // FrogPilot variables
  developer_sidebar = new DeveloperSidebar(this);
  main_layout->addWidget(developer_sidebar);
  developer_sidebar->setVisible(false);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::showMapPanel(bool show) {
  onroad->showMapPanel(show);
}

void HomeWindow::updateAutoScreenRefresh(bool started) {
  // A drive starting always wins. Close immediately rather than waiting for the next tick.
  if (started) {
    offroad_since = 0;
    drove_this_cycle = true;
    if (auto_refresh != nullptr) {
      auto_refresh->close();
      auto_refresh = nullptr;
    }
    return;
  }

  // auto_refresh_enabled and idle_window_s are read once at the offroad transition rather than
  // here: this runs at UI_FREQ for the whole time the device sits parked, and neither value can
  // change without a transition.
  if (auto_refresh != nullptr || offroad_since == 0 || !auto_refresh_enabled || idle_window_s <= 0) {
    return;
  }

  // Fit inside however long the device will actually stay awake. device_shutdown_time is
  // (setting - 3) hours from setting 4 up, and setting * 15 minutes below it -- so at setting 0 it
  // powers down immediately offroad and there is no window at all, and at setting 1 there are
  // fifteen minutes. A fixed five-minute wait plus a five-minute pass silently never completed in
  // the small settings; now the wait shrinks to fit and the pass takes what is left.
  const qint64 delay = std::min<qint64>(AUTO_REFRESH_DELAY_S, idle_window_s / 3);
  const qint64 usable = idle_window_s - delay - AUTO_REFRESH_MARGIN_S;
  if (usable < AUTO_REFRESH_MIN_RUN_S) {
    return;
  }

  const qint64 now = QDateTime::currentSecsSinceEpoch();
  // Wait so this never fires while you are still parked at a light with the ignition briefly off,
  // and so the whole thing happens after you have walked away.
  if (now - offroad_since < delay) {
    return;
  }
  // Weekly, like a TV's compensation cycle. Running a bright full-screen pass after every drive
  // would itself age the panel faster than the wear it is spreading.
  const qint64 last = params.getInt("LastScreenRefresh");
  if (last != 0 && now - last < AUTO_REFRESH_INTERVAL_S) {
    return;
  }

  // Resume rather than restart. A refresh cut short by a drive banks what it completed and picks
  // up the remainder next time; only a full cycle stamps LastScreenRefresh and starts the week
  // over. Stamping at launch instead would let a five-second interruption count as a done week.
  int remaining = params.getInt("ScreenRefreshRemaining");
  if (remaining <= 0 || remaining > AUTO_REFRESH_DURATION_S) {
    remaining = AUTO_REFRESH_DURATION_S;
  }
  // Never start more than the idle window can finish. A pass cut short by power-down banks its
  // progress like any other interruption, but there is no reason to plan one that cannot complete.
  remaining = std::min<int>(remaining, (int)usable);

  drove_this_cycle = false;
  auto_refresh = new ScreenRefreshOverlay(remaining);
  auto_refresh->on_finished = [this, remaining](int completed) {
    const int left = remaining - completed;
    if (left > 0) {
      params.putInt("ScreenRefreshRemaining", left);
    } else {
      params.putInt("ScreenRefreshRemaining", 0);
      params.putInt("LastScreenRefresh", QDateTime::currentSecsSinceEpoch());
    }
  };
  QObject::connect(auto_refresh, &QObject::destroyed, this, [this]() { auto_refresh = nullptr; });
  auto_refresh->showFullScreen();
}

void HomeWindow::updateState(const UIState &s, const FrogPilotUIState &fs) {
  updatePixelShift();
  updateAutoScreenRefresh(s.scene.started);

  const SubMaster &sm = *(s.sm);

  // switch to the generic robot UI
  if (onroad->isVisible() && !body->isEnabled() && sm["carParams"].getCarParams().getNotCar()) {
    body->setEnabled(true);
    slayout->setCurrentWidget(body);
  }

  // FrogPilot variables
  if (s.scene.started) {
    if (fs.frogpilot_scene.driver_camera_timer >= UI_FREQ / 2) {
      showDriverView(true, true);
    } else {
      if (driver_view->isVisible()) {
        sidebar->setVisible(params.getBool("Sidebar") || frogpilotUIState()->frogpilot_toggles.value("debug_mode").toBool());
        slayout->setCurrentWidget(onroad);
      }

      if (fs.frogpilot_scene.map_open) {
        showSidebar(false);
      }

      developer_sidebar->setVisible(fs.frogpilot_toggles.value("developer_sidebar").toBool());

      frogpilotUIState()->frogpilot_scene.sidebars_open = developer_sidebar->isVisible() && sidebar->isVisible();
    }
  }
}

void HomeWindow::offroadTransition(bool offroad) {
  // Marks when the car went quiet, so the auto refresh can wait out a brief ignition-off before
  // taking over the screen. Only arms if a drive actually happened this power cycle -- this fires
  // on boot too, and taking over the screen two minutes after switching the device on, having
  // driven nowhere, is not what "after a drive" means.
  offroad_since = (offroad && drove_this_cycle) ? QDateTime::currentSecsSinceEpoch() : 0;
  if (offroad_since != 0) {
    // Cached here so the idle loop does not re-read them at UI_FREQ for however long the device
    // sits parked. Neither can change without another transition through this function.
    auto_refresh_enabled = params.getBool("AutoScreenRefresh");
    idle_window_s = idleWindowSeconds(params.getInt("DeviceShutdown"));
  }

  body->setEnabled(false);
  sidebar->setVisible(offroad || params.getBool("Sidebar") || frogpilotUIState()->frogpilot_toggles.value("debug_mode").toBool());
  if (offroad) {
    developer_sidebar->setVisible(false);

    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
}

void HomeWindow::showDriverView(bool show, bool started) {
  if (show) {
    if (!started) {
      emit closeSettings();
    }
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  developer_sidebar->setVisible(false);
  sidebar->setVisible(show == false);
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  if ((onroad->isVisible() || body->isVisible()) && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
    params.putBool("Sidebar", sidebar->isVisible());
  }
}

void HomeWindow::mouseDoubleClickEvent(QMouseEvent* e) {
  HomeWindow::mousePressEvent(e);
  const SubMaster &sm = *(uiState()->sm);
  if (sm["carParams"].getCarParams().getNotCar()) {
    if (onroad->isVisible()) {
      slayout->setCurrentWidget(body);
    } else if (body->isVisible()) {
      slayout->setCurrentWidget(onroad);
    }
    showSidebar(false);
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 40);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(16);

  update_notif = new QPushButton(tr("UPDATE"));
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  date = new ElidedLabel();
  header_layout->addWidget(date, 0, Qt::AlignHCenter | Qt::AlignLeft);

  version = new ElidedLabel();
  header_layout->addWidget(version, 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget *home_widget = new QWidget(this);
  {
    QHBoxLayout *home_layout = new QHBoxLayout(home_widget);
    home_layout->setContentsMargins(0, 0, 0, 0);
    home_layout->setSpacing(30);

    // left: MapSettings
    QStackedWidget *left_widget = new QStackedWidget(this);
#ifdef ENABLE_MAPS
    left_widget->addWidget(new MapSettings);
#else
    left_widget->addWidget(new QWidget);
#endif
    left_widget->addWidget(new DriveStats);

    FrogPilotDriveSummary *drive_summary = new FrogPilotDriveSummary(this);
    left_widget->addWidget(drive_summary);

    FrogPilotModelReview *model_review = new FrogPilotModelReview(this);
    left_widget->addWidget(model_review);

    left_widget->setStyleSheet("border-radius: 10px;");
    left_widget->setCurrentIndex(1);

    connect(drive_summary, &FrogPilotDriveSummary::panelClosed, [=]() {
      left_widget->setCurrentIndex(1);
    });
    connect(model_review, &FrogPilotModelReview::driveRated, [=]() {
      left_widget->setCurrentIndex(2);
    });
    connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
      static bool previouslyOnroad = false;
      if (offroad && previouslyOnroad) {
        if (frogpilotUIState()->frogpilot_scene.started_timer > 15 * 60 * UI_FREQ && frogpilotUIState()->frogpilot_toggles.value("model_randomizer").toBool()) {
          left_widget->setCurrentIndex(3);
        } else {
          left_widget->setCurrentIndex(2);
        }
      }
      previouslyOnroad = !offroad;
    });

    home_layout->addWidget(left_widget, 1);

    // right: ExperimentalModeButton, SetupWidget
    QStackedWidget *right_widget = new QStackedWidget(this);
    right_widget->setFixedWidth(750);

    QWidget *default_right = new QWidget(this);
    QVBoxLayout *default_layout = new QVBoxLayout(default_right);
    default_layout->setContentsMargins(0, 0, 0, 0);
    default_layout->setSpacing(30);

    ExperimentalModeButton *experimental_mode = new ExperimentalModeButton(this);
    QObject::connect(experimental_mode, &ExperimentalModeButton::openSettings, this, &OffroadHome::openSettings);
    default_layout->addWidget(experimental_mode, 1);

    SetupWidget *setup_widget = new SetupWidget;
    QObject::connect(setup_widget, &SetupWidget::openSettings, this, &OffroadHome::openSettings);
    default_layout->addWidget(setup_widget, 1);

    right_widget->addWidget(default_right);

    FrogPilotDriveSummary *random_events_summary = new FrogPilotDriveSummary(this, true);
    right_widget->addWidget(random_events_summary);

    right_widget->setCurrentIndex(0);

    connect(random_events_summary, &FrogPilotDriveSummary::panelClosed, [=]() {
      right_widget->setCurrentIndex(0);
    });
    connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
      static bool previouslyOnroad = false;
      if (offroad && previouslyOnroad && frogpilotUIState()->frogpilot_toggles.value("random_events").toBool()) {
        right_widget->setCurrentIndex(1);
      }
      previouslyOnroad = !offroad;
    });

    home_layout->addWidget(right_widget, 1);
  }
  center_layout->addWidget(home_widget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
      color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  date->setText(QLocale(uiState()->language.mid(5)).toString(QDateTime::currentDateTime(), "dddd, MMMM d"));
  date->setVisible(util::system_time_valid());

  version->setText(getBrand() + " v" + getVersion().left(14).trimmed() + " - " + processModelName(frogpilotUIState()->frogpilot_toggles.value("model_name").toString()));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
  }
}
