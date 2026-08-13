#include <QPainter>
#include <QTimer>

#include "frogpilot/ui/qt/offroad/utilities.h"

namespace {
  // Each solid phase lasts long enough to matter but short enough that the panel never sits on one
  // colour, which is what caused the problem in the first place.
  constexpr int PHASE_MS = 4000;
  constexpr int TICK_MS = 50;
  constexpr int BAR_WIDTH = 40;
}

ScreenRefreshOverlay::ScreenRefreshOverlay(int seconds, QWidget *parent)
    : QWidget(parent), total_ms(seconds * 1000) {
  setAttribute(Qt::WA_DeleteOnClose);
  setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
  setCursor(Qt::BlankCursor);
  elapsed.start();

  QTimer *ticker = new QTimer(this);
  QObject::connect(ticker, &QTimer::timeout, [this]() {
    if (elapsed.elapsed() >= total_ms) {
      close();
      return;
    }
    // Hold off the screen timeout; the user is deliberately not touching anything.
    device()->resetInteractiveTimeout();
    update();
  });
  ticker->start(TICK_MS);
}

void ScreenRefreshOverlay::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  const qint64 ms = elapsed.elapsed();
  const int phase = (ms / PHASE_MS) % 6;

  if (phase < 5) {
    // Solid fills: white exercises every subpixel at full drive, the primaries each exercise one,
    // and black gives the panel a rest cycle.
    static const QColor fills[5] = {Qt::white, Qt::red, Qt::green, Qt::blue, Qt::black};
    p.fillRect(rect(), fills[phase]);
  } else {
    // Sweeping inversion bars. Every pixel alternates black/white as the pattern scrolls, which is
    // what shifts trapped charge rather than just averaging brightness.
    const int offset = (ms / 16) % (BAR_WIDTH * 2);
    p.fillRect(rect(), Qt::black);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::white);
    for (int x = -BAR_WIDTH * 2 + offset; x < width(); x += BAR_WIDTH * 2) {
      p.drawRect(x, 0, BAR_WIDTH, height());
    }
  }

  // Remaining time and the exit hint. Deliberately repositioned every second -- static text here
  // would burn in exactly the way this tool exists to reduce.
  const int remaining = std::max(0, int((total_ms - ms) / 1000));
  const int step = int(ms / 1000);
  const int bw = 520, bh = 90;
  const int bx = (width() > bw) ? (step * 137) % (width() - bw) : 0;
  const int by = (height() > bh) ? (step * 89) % (height() - bh) : 0;

  p.setBrush(QColor(0, 0, 0, 190));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(bx, by, bw, bh, 12, 12);
  p.setPen(Qt::white);
  p.setFont(InterFont(40, QFont::Bold));
  p.drawText(QRect(bx, by, bw, bh), Qt::AlignCenter,
             QString("%1:%2  —  tap to stop").arg(remaining / 60).arg(remaining % 60, 2, 10, QChar('0')));
}

void ScreenRefreshOverlay::mousePressEvent(QMouseEvent *event) {
  close();
}

FrogPilotUtilitiesPanel::FrogPilotUtilitiesPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent) {
  QJsonObject shownDescriptions = QJsonDocument::fromJson(QString::fromStdString(params.get("ShownToggleDescriptions")).toUtf8()).object();
  QString className = this->metaObject()->className();

  bool forceOpenDescriptions = false;
  if (!shownDescriptions.value(className).toBool(false)) {
    forceOpenDescriptions = true;
    shownDescriptions.insert(className, true);
    params.put("ShownToggleDescriptions", QJsonDocument(shownDescriptions).toJson(QJsonDocument::Compact).toStdString());
  }

  ParamControl *debugModeToggle = new ParamControl("DebugMode", tr("Debug Mode"), tr("<b>Use FrogPilot's developer metrics on your next drive</b> to diagnose issues and improve bug reports."), "");
  if (forceOpenDescriptions) {
    debugModeToggle->showDescription();
  }
  addItem(debugModeToggle);

  FrogPilotParamValueControl *screenRefreshMinutes = new FrogPilotParamValueControl("ScreenRefreshMinutes",
    tr("Screen Refresh Duration"),
    tr("<b>How long the burn-in reduction runs for.</b> Longer helps more; fifteen minutes is a reasonable starting point."),
    "", 1, 60, tr(" minutes"), std::map<float, QString>(), 1, true);
  addItem(screenRefreshMinutes);

  ButtonControl *screenRefreshButton = new ButtonControl(tr("Reduce Screen Burn-In"), tr("START"),
    tr("<b>Cycles the whole screen through white, red, green, blue, black and sweeping bars</b> so every subpixel gets equal use. "
       "This clears temporary image retention. True burn-in is permanent wear and cannot be undone — cycling only evens it out, "
       "at the cost of aging the rest of the panel. Tap the screen at any time to stop. Offroad only."));
  QObject::connect(screenRefreshButton, &ButtonControl::clicked, [this]() {
    if (uiState()->scene.started) {
      // Never take over the display while driving.
      FrogPilotConfirmationDialog::yesorno(tr("Not while driving. Stop the car and try again."), this);
      return;
    }
    int minutes = std::clamp(params.getInt("ScreenRefreshMinutes"), 1, 60);
    if (FrogPilotConfirmationDialog::yesorno(tr("Run the screen refresh for %1 minutes? Tap the screen to stop early.").arg(minutes), this)) {
      ScreenRefreshOverlay *overlay = new ScreenRefreshOverlay(minutes * 60);
      overlay->showFullScreen();
    }
  });
  addItem(screenRefreshButton);

  ButtonControl *flashPandaButton = new ButtonControl(tr("Flash Panda"), tr("FLASH"), tr("<b>Reinstall the Panda firmware</b> to fix connection or reliability issues."));
  QObject::connect(flashPandaButton, &ButtonControl::clicked, [parent, flashPandaButton, this]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to flash the Panda firmware?"), tr("Flash"), this)) {
      std::thread([parent, flashPandaButton, this]() {
        parent->keepScreenOn = true;

        flashPandaButton->setEnabled(false);
        flashPandaButton->setValue(tr("Flashing..."));

        params_memory.putBool("FlashPanda", true);
        while (params_memory.getBool("FlashPanda")) {
          util::sleep_for(UI_FREQ);
        }

        flashPandaButton->setValue(tr("Flashed!"));

        util::sleep_for(2500);

        flashPandaButton->setValue(tr("Rebooting..."));

        util::sleep_for(2500);

        Hardware::reboot();
      }).detach();
    }
  });
  if (forceOpenDescriptions) {
    flashPandaButton->showDescription();
  }
  addItem(flashPandaButton);

  FrogPilotButtonsControl *forceStartedButton = new FrogPilotButtonsControl(tr("Force Drive State"), tr("<b>Manually set openpilot to be offroad or onroad.</b>"), "", {tr("OFFROAD"), tr("ONROAD"), tr("OFF")}, true);
  QObject::connect(forceStartedButton, &FrogPilotButtonsControl::buttonClicked, [this](int id) {
    if (id == 0) {
      params_memory.putBool("ForceOffroad", true);
      params_memory.putBool("ForceOnroad", false);

      updateFrogPilotToggles();
    } else if (id == 1) {
      params.put("CarParams", params.get("CarParamsPersistent"));
      params.put("FrogPilotCarParams", params.get("FrogPilotCarParamsPersistent"));

      params_memory.putBool("ForceOffroad", false);
      params_memory.putBool("ForceOnroad", true);

      updateFrogPilotToggles();
    } else if (id == 2) {
      params_memory.putBool("ForceOffroad", false);
      params_memory.putBool("ForceOnroad", false);

      updateFrogPilotToggles();
    }
  });
  forceStartedButton->setCheckedButton(2);
  if (forceOpenDescriptions) {
    forceStartedButton->showDescription();
  }
  addItem(forceStartedButton);

  ButtonControl *reportIssueButton = new ButtonControl(tr("Report a Bug or an Issue"), tr("REPORT"), tr("<b>Send a bug report</b> so we can help fix the problem!"));
  QObject::connect(reportIssueButton, &ButtonControl::clicked, [this]() {
    if (!frogpilotUIState()->frogpilot_scene.online) {
      ConfirmationDialog::alert(tr("Please connect to the internet before sending a report!"), this);
      return;
    }

    QStringList report_messages;
    QString crash_report = tr("I saw an alert that said \"openpilot crashed\"");
    if (QFile::exists("/data/error_logs/error.txt")) {
      report_messages << crash_report;
    }
    QStringList additional_issues = {
      tr("Acceleration feels harsh or jerky"),
      tr("An alert was unclear and I didn't know what it meant"),
      tr("Braking is too sudden or uncomfortable"),
      tr("I'm not sure if this is normal or a bug:"),
      tr("My screen froze or is stuck loading something"),
      tr("My steering wheel buttons aren't working"),
      tr("openpilot disengages when I don't expect it"),
      tr("openpilot doesn't react to stopped vehicles ahead"),
      tr("openpilot doesn't resume from a stop"),
      tr("openpilot feels sluggish or slow to respond"),
      tr("Steering feels twitchy or unnatural"),
      tr("The car doesn't follow curves well"),
      tr("The car isn't staying centered in its lane"),
      tr("Something else (please describe)")
    };
    report_messages.append(additional_issues);

    QMap<QString, bool> needs_extra_input;
    for (const QString &issue : report_messages) {
      if (issue.contains("confused") ||
          issue.contains("crashed") ||
          issue.contains("not sure") ||
          issue.contains("Something else")) {
        needs_extra_input[issue] = true;
      }
    }

    QString selected_issue = MultiOptionDialog::getSelection(tr("What's going on?"), report_messages, "", this);
    if (selected_issue.isEmpty()) {
      return;
    }

    if (needs_extra_input.value(selected_issue, false)) {
      QString extra_input = InputDialog::getText(tr("Please describe what's happening"), this, tr("Send Report"), false, 10, "", 300).trimmed();
      if (extra_input.isEmpty()) {
        return;
      }
      selected_issue += " — " + extra_input;
    }

    QJsonObject reportData;
    reportData["Issue"] = selected_issue;
    reportData["DiscordUser"] = InputDialog::getText(tr("What's your Discord username?"), this, tr("Send Report"), false, -1, QString::fromStdString(params.get("DiscordUsername"))).trimmed();

    params.putNonBlocking("DiscordUsername", reportData["DiscordUser"].toString().toStdString());
    params_memory.put("IssueReported", QJsonDocument(reportData).toJson(QJsonDocument::Compact).toStdString());

    ConfirmationDialog::alert(tr("Report Sent! Thanks for letting us know!"), this);
  });
  if (forceOpenDescriptions) {
    reportIssueButton->showDescription();
  }
  addItem(reportIssueButton);
  reportIssueButton->setVisible(QString::fromStdString(params.get("GitRemote")).toLower() == "https://github.com/frogai/openpilot.git");

  ButtonControl *resetTogglesButton = new ButtonControl(tr("Reset Toggles to Default"), tr("RESET"), tr("<b>Reset all toggles to their default values.</b>"));
  QObject::connect(resetTogglesButton, &ButtonControl::clicked, [parent, resetTogglesButton, this]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset all toggles to their default values?"), tr("Reset"), this)) {
      std::thread([parent, resetTogglesButton, this]() mutable {
        parent->keepScreenOn = true;

        resetTogglesButton->setEnabled(false);
        resetTogglesButton->setValue(tr("Resetting..."));

        params.putBool("DoToggleReset", true);

        resetTogglesButton->setValue(tr("Reset!"));

        util::sleep_for(2500);

        resetTogglesButton->setValue(tr("Rebooting..."));

        util::sleep_for(2500);

        Hardware::reboot();
      }).detach();
    }
  });
  if (forceOpenDescriptions) {
    resetTogglesButton->showDescription();
  }
  addItem(resetTogglesButton);

  ButtonControl *resetTogglesButtonStock = new ButtonControl(tr("Reset Toggles to Stock openpilot"), tr("RESET"), tr("<b>Reset all toggles to match stock openpilot.</b>"));
  QObject::connect(resetTogglesButtonStock, &ButtonControl::clicked, [parent, resetTogglesButtonStock, this]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset all toggles to match stock openpilot?"), tr("Reset"), this)) {
      std::thread([parent, resetTogglesButtonStock, this]() mutable {
        parent->keepScreenOn = true;

        resetTogglesButtonStock->setEnabled(false);
        resetTogglesButtonStock->setValue(tr("Resetting..."));

        params.putBool("DoToggleResetStock", true);

        resetTogglesButtonStock->setValue(tr("Reset!"));

        util::sleep_for(2500);

        resetTogglesButtonStock->setValue(tr("Rebooting..."));

        util::sleep_for(2500);

        Hardware::reboot();
      }).detach();
    }
  });
  if (forceOpenDescriptions) {
    resetTogglesButtonStock->showDescription();
  }
  addItem(resetTogglesButtonStock);
}
