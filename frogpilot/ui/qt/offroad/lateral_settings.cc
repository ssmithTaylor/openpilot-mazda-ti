#include <QDateTime>

#include "frogpilot/ui/qt/offroad/lateral_settings.h"

FrogPilotLateralPanel::FrogPilotLateralPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent) {
  QJsonObject shownDescriptions = QJsonDocument::fromJson(QString::fromStdString(params.get("ShownToggleDescriptions")).toUtf8()).object();
  QString className = this->metaObject()->className();

  if (!shownDescriptions.value(className).toBool(false)) {
    forceOpenDescriptions = true;
    shownDescriptions.insert(className, true);
    params.put("ShownToggleDescriptions", QJsonDocument(shownDescriptions).toJson(QJsonDocument::Compact).toStdString());
  }

  QStackedLayout *lateralLayout = new QStackedLayout();
  addItem(lateralLayout);

  FrogPilotListWidget *lateralList = new FrogPilotListWidget(this);

  ScrollView *lateralPanel = new ScrollView(lateralList, this);

  lateralLayout->addWidget(lateralPanel);

  FrogPilotListWidget *advancedLateralTuneList = new FrogPilotListWidget(this);
  FrogPilotListWidget *aolList = new FrogPilotListWidget(this);
  FrogPilotListWidget *laneChangeList = new FrogPilotListWidget(this);
  FrogPilotListWidget *lateralTuneList = new FrogPilotListWidget(this);
  FrogPilotListWidget *qolList = new FrogPilotListWidget(this);
  FrogPilotListWidget *torqueInterceptorList = new FrogPilotListWidget(this);

  ScrollView *advancedLateralTunePanel = new ScrollView(advancedLateralTuneList, this);
  ScrollView *aolPanel = new ScrollView(aolList, this);
  ScrollView *laneChangePanel = new ScrollView(laneChangeList, this);
  ScrollView *lateralTunePanel = new ScrollView(lateralTuneList, this);
  ScrollView *qolPanel = new ScrollView(qolList, this);
  ScrollView *torqueInterceptorPanel = new ScrollView(torqueInterceptorList, this);

  lateralLayout->addWidget(advancedLateralTunePanel);
  lateralLayout->addWidget(aolPanel);
  lateralLayout->addWidget(laneChangePanel);
  lateralLayout->addWidget(lateralTunePanel);
  lateralLayout->addWidget(qolPanel);
  lateralLayout->addWidget(torqueInterceptorPanel);

  // Live counters at the top of the TI panel, so a change can be judged on the spot rather than by
  // pulling an rlog afterwards. Populated by updateTorqueInterceptorStats().
  lateralLayoutRef = lateralLayout;
  torqueInterceptorPanelRef = torqueInterceptorPanel;
  tiCommandCutLabel = new LabelControl(tr("Command Cut"), "—", tr("How often openpilot sent less than it wanted to. Lower is better."));
  tiLimitedByLabel = new LabelControl(tr("Limited By"), "—", tr("Which limiter did the cutting. Rate points at Ramp-Up Rate; driver points at Driver Torque Backoff."));
  tiOutputLabel = new LabelControl(tr("Output"), "—", tr("Peak bias actually reaching the EPS, and time spent pinned at the interceptor's 600 clip."));
  tiHealthLabel = new LabelControl(tr("Interceptor Health"), "—", tr("Frames where the TI left RUN or ramped itself down. Both should stay at zero."));
  torqueInterceptorList->addItem(tiCommandCutLabel);
  torqueInterceptorList->addItem(tiLimitedByLabel);
  torqueInterceptorList->addItem(tiOutputLabel);
  tiMcpLabel = new LabelControl(tr("Telemetry Address"), tr("not running"), tr("Read-only telemetry service. Point an MCP client at this address to query tuning data from a laptop on the same network."));
  torqueInterceptorList->addItem(tiHealthLabel);
  torqueInterceptorList->addItem(tiMcpLabel);

  const std::vector<std::tuple<QString, QString, QString, QString>> lateralToggles {
    {"AdvancedLateralTune", tr("Advanced Lateral Tuning"), tr("<b>Advanced steering control changes to fine-tune how openpilot drives.</b>"), "../../frogpilot/assets/toggle_icons/icon_advanced_lateral_tune.png"},
    {"SteerDelay", parent->steerActuatorDelay != 0 ? QString(tr("Actuator Delay (Default: %1)")).arg(QString::number(parent->steerActuatorDelay, 'f', 2)) : tr("Actuator Delay"), tr("<b>The time between openpilot's steering command and the vehicle's response.</b> Increase if the vehicle reacts late; decrease if it feels jumpy. Auto-learned by default."), ""},
    {"SteerFriction", parent->friction != 0 ? QString(tr("Friction (Default: %1)")).arg(QString::number(parent->friction, 'f', 2)) : tr("Friction"), tr("<b>Compensates for steering friction.</b> Increase if the wheel sticks near center; decrease if it jitters. Auto-learned by default."), ""},
    {"SteerKP", parent->steerKp != 0 ? QString(tr("Kp Factor (Default: %1)")).arg(QString::number(parent->steerKp, 'f', 2)) : tr("Kp Factor"), tr("<b>How strongly openpilot corrects lane position.</b> Higher is tighter but twitchier; lower is smoother but slower. Auto-learned by default."), ""},
    {"SteerLatAccel", parent->latAccelFactor != 0 ? QString(tr("Lateral Acceleration (Default: %1)")).arg(QString::number(parent->latAccelFactor, 'f', 2)) : tr("Lateral Acceleration"), tr("<b>Maps steering torque to turning response.</b> Increase for sharper turns; decrease for gentler steering. Auto-learned by default."), ""},
    {"SteerRatio", parent->steerRatio != 0 ? QString(tr("Steer Ratio (Default: %1)")).arg(QString::number(parent->steerRatio, 'f', 2)) : tr("Steer Ratio"), tr("<b>The relationship between steering wheel rotation and road wheel angle.</b> Increase if steering feels too quick or twitchy; decrease if it feels too slow or weak. Auto-learned by default."), ""},
    {"ForceAutoTune", tr("Force Auto-Tune On"), tr("<b>Force-enable openpilot's live auto-tuning for \"Friction\" and \"Lateral Acceleration\".</b>"), ""},
    {"ForceAutoTuneOff", tr("Force Auto-Tune Off"), tr("<b>Force-disable openpilot's live auto-tuning for \"Friction\" and \"Lateral Acceleration\" and use the set value instead.</b>"), ""},
    {"ForceTorqueController", tr("Force Torque Controller"), tr("<b>Use torque-based steering control instead of angle-based control for smoother lane keeping, especially in curves.</b>"), ""},
    {"ResetTorqueParams", tr("Reset Learned Steering Values"), tr("<b>Throw away what openpilot has learned about your steering and start over.</b> Do this after changing anything that affects how much the car turns for a given command, otherwise it keeps using values learned from the old behaviour. Switches itself back off once done."), ""},

    {"TorqueInterceptorTune", tr("Torque Interceptor Tuning"), tr("<b>Adjust how the Torque Interceptor delivers steering.</b> Only affects cars fitted with a TI."), "../../frogpilot/assets/toggle_icons/icon_advanced_lateral_tune.png"},
    {"TiSteerMax", tr("Max Torque"), tr("<b>The most steering effort openpilot can ask the interceptor for.</b> Lower this to cap how strong assist can get. The interceptor's own onboard hardware clamps its output near 600 per its spec sheet, outside openpilot's control; values above that are for confirming the clamp actually holds, not for asking for more real torque."), ""},
    {"TiSteerDeltaUp", tr("Ramp-Up Rate"), tr("<b>How quickly steering effort is allowed to build.</b> Raise for sharper response entering corners. Too high and the interceptor treats it as unsafe and cuts assist to zero. Default 6 takes one second to reach full."), ""},
    {"TiSteerDeltaUpKnee", tr("Cautious Above"), tr("<b>The effort level above which the slower ramp rate takes over.</b> Below this the ramp-up rate applies; above it, the cautious rate. Leave at 600 to use one rate everywhere. Lower it to get quick response in normal driving while staying gentle at high effort."), ""},
    {"TiSteerDeltaUpHigh", tr("Cautious Ramp-Up Rate"), tr("<b>How quickly steering effort builds once past the level set above.</b> Only does anything if \"Cautious Above\" is below 600. Cannot exceed the main ramp-up rate."), ""},
    {"TiSteerDeltaDown", tr("Ramp-Down Rate"), tr("<b>How quickly steering effort is allowed to release.</b> Raise to hand control back faster when openpilot backs off; lower for smoother corner exits."), ""},
    {"TiSteerDriverAllowance", tr("Driver Torque Allowance"), tr("<b>How firmly you can hold the wheel before openpilot starts easing off.</b> Raise if assist fades just from resting a hand on the wheel; lower to hand over control sooner."), ""},
    {"TiSteerDriverMultiplier", tr("Driver Torque Backoff"), tr("<b>How sharply assist drops once you push past the allowance.</b> Lower it if steering gives up on you mid-corner; raise it to hand over control more readily. At the default of 40, assist is gone almost immediately."), ""},
    {"TiSteerThreshold", tr("Steering Pressed Threshold"), tr("<b>How much pressure on the wheel counts as you taking over.</b> Raise if bumps and road feedback falsely trigger a takeover; lower to have openpilot notice your input sooner."), ""},
    {"TiMcpEnabled", tr("Telemetry Service"), tr("<b>Serve the tuning counters on the local network so a laptop can read them while you drive.</b> Read-only and unauthenticated, so anyone on the same network can see driving state. Turn it off on networks you do not trust. Requires a reboot to take effect."), ""},
    {"ClearTiStats", tr("Start A New Measurement"), tr("<b>Zero the tuning counters so the next stretch of road is measured on its own.</b> The previous run's figures are kept for comparison. Turn this on just before the corner or road you want to judge a change on."), ""},
    {"TiFlagMoment", tr("Flag This Moment"), tr("<b>Mark right now as worth looking at later.</b> Tap it when the steering does something odd — a dropout, a wander, effort that does not arrive. Records the spot, the segment and what the interceptor was doing, so the drive can be reviewed without trawling the whole route."), ""},

    {"AlwaysOnLateral", tr("Always On Lateral"), tr("<b>openpilot's steering remains active even when the accelerator or brake pedals are pressed.</b>"), "../../frogpilot/assets/toggle_icons/icon_always_on_lateral.png"},
    {"AlwaysOnLateralMain", tr("Enable With Cruise Control"), tr("<b>Enable \"Always On Lateral\" whenever \"Cruise Control\" is on, even when openpilot is not engaged.</b>"), ""},
    {"AlwaysOnLateralLKAS", tr("Enable With LKAS"), tr("<b>Enable \"Always On Lateral\" whenever \"LKAS\" is on, even when openpilot is not engaged.</b>"), ""},
    {"PauseAOLOnBrake", tr("Pause on Brake Press Below"), tr("<b>Pause \"Always On Lateral\" below the set speed while the brake pedal is pressed.</b>"), ""},

    {"LaneChanges", tr("Lane Changes"), tr("<b>Allow openpilot to change lanes.</b>"), "../../frogpilot/assets/toggle_icons/icon_lane.png"},
    {"NudgelessLaneChange", tr("Automatic Lane Changes"), tr("<b>When the turn signal is on, openpilot will automatically change lanes.</b> No steering-wheel nudge required!"), ""},
    {"LaneChangeTime", tr("Lane Change Delay"), tr("<b>Delay between turn signal activation and the start of an automatic lane change.</b>"), ""},
    {"MinimumLaneChangeSpeed", tr("Minimum Lane Change Speed"), tr("<b>Lowest speed at which openpilot will change lanes.</b>"), ""},
    {"LaneDetectionWidth", tr("Minimum Lane Width"), tr("<b>Prevent automatic lane changes into lanes narrower than the set width.</b>"), ""},
    {"OneLaneChange", tr("One Lane Change Per Signal"), tr("<b>Limit automatic lane changes to one per turn-signal activation.</b>"), ""},

    {"LateralTune", tr("Lateral Tuning"), tr("<b>Miscellaneous steering control changes</b> to fine-tune how openpilot drives."), "../../frogpilot/assets/toggle_icons/icon_lateral_tune.png"},
    {"TurnDesires", tr("Force Turn Desires Below Lane Change Speed"), tr("<b>While driving below the minimum lane change speed with an active turn signal, instruct openpilot to turn left/right.</b>"), ""},
    {"NNFF", tr("Neural Network Feedforward (NNFF)"), tr("<b>Twilsonco's \"Neural Network FeedForward\" controller.</b> Uses a trained neural network model to predict steering torque based on vehicle speed, roll, and past/future planned path data for smoother, model-based steering."), ""},
    {"NNFFLite", tr("Neural Network Feedforward (NNFF) Lite"), tr("<b>A lightweight version of Twilsonco's \"Neural Network FeedForward\" controller.</b> Uses the \"look-ahead\" planned lateral jerk logic from the full model to help smoothen steering adjustments in curves, but does not use the full neural network for torque calculation."), ""},

    {"QOLLateral", tr("Quality of Life"), tr("<b>Steering control changes to fine-tune how openpilot drives.</b>"), "../../frogpilot/assets/toggle_icons/icon_quality_of_life.png"},
    {"PauseLateralSpeed", tr("Pause Steering Below"), tr("<b>Pause steering below the set speed.</b>"), ""}
  };

  for (const auto &[param, title, desc, icon] : lateralToggles) {
    AbstractControl *lateralToggle;

    if (param == "AdvancedLateralTune") {
      FrogPilotManageControl *advancedLateralTuneToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(advancedLateralTuneToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, advancedLateralTunePanel]() {
        lateralLayout->setCurrentWidget(advancedLateralTunePanel);
      });
      lateralToggle = advancedLateralTuneToggle;
    } else if (param == "SteerDelay") {
      std::vector<QString> steerDelayButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 0.01, 1, QString(), std::map<float, QString>(), 0.01, false, {}, steerDelayButton, false, false);
    } else if (param == "SteerFriction") {
      std::vector<QString> steerFrictionButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 0, 0.5, QString(), std::map<float, QString>(), 0.01, false, {}, steerFrictionButton, false, false);
    } else if (param == "SteerKP") {
      std::vector<QString> steerKPButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, parent->steerKp * 0.5, parent->steerKp * 1.5, QString(), std::map<float, QString>(), 0.01, false, {}, steerKPButton, false, false);
    } else if (param == "SteerLatAccel") {
      std::vector<QString> steerLatAccelButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, parent->latAccelFactor * 0.75, parent->latAccelFactor * 1.25, QString(), std::map<float, QString>(), 0.01, false, {}, steerLatAccelButton, false, false);
    } else if (param == "SteerRatio") {
      std::vector<QString> steerRatioButton{"Reset"};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, parent->steerRatio * 0.5, parent->steerRatio * 1.5, QString(), std::map<float, QString>(), 0.01, false, {}, steerRatioButton, false, false);

    } else if (param == "TorqueInterceptorTune") {
      FrogPilotManageControl *torqueInterceptorToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(torqueInterceptorToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, torqueInterceptorPanel]() {
        lateralLayout->setCurrentWidget(torqueInterceptorPanel);
      });
      lateralToggle = torqueInterceptorToggle;
    } else if (param == "TiSteerMax") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 100, 1200, QString(), std::map<float, QString>(), 10, true);
    // Ranges below deliberately match TI_LIMIT_BOUNDS in carcontroller.py and the clips in
    // frogpilot_variables.py. The panda applies no steering checks to MAZDA_TI_LKAS on gen1, so
    // these three layers are the entire envelope on the interceptor path; if you widen one, widen
    // all three, and think about why first.
    } else if (param == "TiSteerDeltaUp") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 30, QString(), std::map<float, QString>(), 1, true);
    } else if (param == "TiSteerDeltaUpKnee") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 100, 1200, QString(), std::map<float, QString>(), 10, true);
    } else if (param == "TiSteerDeltaUpHigh") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 30, QString(), std::map<float, QString>(), 1, true);
    } else if (param == "TiSteerDeltaDown") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 10, 100, QString(), std::map<float, QString>(), 1, true);
    } else if (param == "TiSteerDriverAllowance") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 5, 60, QString(), std::map<float, QString>(), 1, true);
    } else if (param == "TiSteerDriverMultiplier") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 20, 120, QString(), std::map<float, QString>(), 1, true);
    } else if (param == "TiSteerThreshold") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 30, QString(), std::map<float, QString>(), 1, true);

    } else if (param == "ResetTorqueParams") {
      // An action, not a state. The backend consumes the param and clears it, so a switch would
      // flip itself back and read as a failed toggle.
      ButtonControl *resetTorqueButton = new ButtonControl(title, tr("RESET"), desc);
      QObject::connect(resetTorqueButton, &ButtonControl::clicked, [this, resetTorqueButton]() {
        if (FrogPilotConfirmationDialog::yesorno(tr("Discard everything openpilot has learned about your steering? Relearning takes some driving."), this)) {
          params.putBool("ResetTorqueParams", true);
          resetTorqueButton->setText(tr("CLEARED"));
        }
      });
      lateralToggle = resetTorqueButton;
    } else if (param == "ClearTiStats") {
      ButtonControl *clearStatsButton = new ButtonControl(title, tr("START"), desc);
      QObject::connect(clearStatsButton, &ButtonControl::clicked, [this, clearStatsButton]() {
        // tmpfs. Both trigger flags are ephemeral, and the car controller has to clear them from
        // inside its 100Hz loop -- on /data that clear is two ext4 journal commits in the thread
        // that builds the steering frame.
        params_memory.putBool("ClearTiStats", true);
        clearStatsButton->setText(tr("STARTED"));
      });
      lateralToggle = clearStatsButton;
    } else if (param == "TiFlagMoment") {
      // Pressed while driving, so no confirmation dialog: one tap, immediate feedback, done. A
      // yes/no prompt here would take the driver's attention for exactly as long as the thing
      // they are trying to report. The car controller picks the flag up within a second and
      // clears it; the button text is set back by updateTorqueInterceptorStats once it has.
      tiFlagButton = new ButtonControl(title, tr("FLAG"), desc);
      QObject::connect(tiFlagButton, &ButtonControl::clicked, [this]() {
        params_memory.putBool("TiFlagMoment", true);
        tiFlagButton->setText(tr("FLAGGED"));
      });
      lateralToggle = tiFlagButton;

    } else if (param == "AlwaysOnLateral") {
      FrogPilotManageControl *aolToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(aolToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, aolPanel]() {
        lateralLayout->setCurrentWidget(aolPanel);
      });
      lateralToggle = aolToggle;
    } else if (param == "PauseAOLOnBrake") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 99, QString(), std::map<float, QString>(), 1, true);

    } else if (param == "LaneChanges") {
      FrogPilotManageControl *laneChangeToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(laneChangeToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, laneChangePanel]() {
        lateralLayout->setCurrentWidget(laneChangePanel);
      });
      lateralToggle = laneChangeToggle;
    } else if (param == "LaneChangeTime") {
      std::map<float, QString> laneChangeTimeLabels;
      for (float i = 0; i <= 5; i += 0.1) {
        laneChangeTimeLabels[i] = i == 0 ? tr("Instant") : std::lround(i / 0.1) == 1 / 0.1 ? QString::number(i, 'f', 1) + tr(" second") : QString::number(i, 'f', 1) + tr(" seconds");
      }
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 5, QString(), laneChangeTimeLabels, 0.1);
    } else if (param == "LaneDetectionWidth") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 15, QString(), std::map<float, QString>(), 0.1, true);
    } else if (param == "MinimumLaneChangeSpeed") {
      lateralToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 99, QString(), std::map<float, QString>(), 1, true);

    } else if (param == "LateralTune") {
      FrogPilotManageControl *lateralTuneToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(lateralTuneToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, lateralTunePanel]() {
        lateralLayout->setCurrentWidget(lateralTunePanel);
      });
      lateralToggle = lateralTuneToggle;

    } else if (param == "QOLLateral") {
      FrogPilotManageControl *qolLateralToggle = new FrogPilotManageControl(param, title, desc, icon);
      QObject::connect(qolLateralToggle, &FrogPilotManageControl::manageButtonClicked, [lateralLayout, qolPanel]() {
        lateralLayout->setCurrentWidget(qolPanel);
      });
      lateralToggle = qolLateralToggle;
    } else if (param == "PauseLateralSpeed") {
      std::vector<QString> pauseLateralToggles{"PauseLateralOnSignal"};
      std::vector<QString> pauseLateralToggleNames{tr("Turn Signal Only")};
      lateralToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 0, 99, QString(), std::map<float, QString>(), 1, true, pauseLateralToggles, pauseLateralToggleNames, true);

    } else {
      lateralToggle = new ParamControl(param, title, desc, icon);
    }

    toggles[param] = lateralToggle;

    if (advancedLateralTuneKeys.contains(param)) {
      advancedLateralTuneList->addItem(lateralToggle);
    } else if (aolKeys.contains(param)) {
      aolList->addItem(lateralToggle);
    } else if (laneChangeKeys.contains(param)) {
      laneChangeList->addItem(lateralToggle);
    } else if (lateralTuneKeys.contains(param)) {
      lateralTuneList->addItem(lateralToggle);
    } else if (qolKeys.contains(param)) {
      qolList ->addItem(lateralToggle);
    } else if (torqueInterceptorKeys.contains(param)) {
      torqueInterceptorList->addItem(lateralToggle);
    } else {
      lateralList->addItem(lateralToggle);

      parentKeys.insert(param);
    }

    if (FrogPilotManageControl *frogPilotManageToggle = qobject_cast<FrogPilotManageControl*>(lateralToggle)) {
      QObject::connect(frogPilotManageToggle, &FrogPilotManageControl::manageButtonClicked, [this]() {
        emit openSubPanel();
        openDescriptions(forceOpenDescriptions, toggles);
      });
    }

    QObject::connect(lateralToggle, &AbstractControl::hideDescriptionEvent, [this]() {
      update();
    });
    QObject::connect(lateralToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  QSet<QString> forceUpdateKeys = {"ForceAutoTune", "ForceAutoTuneOff", "LateralTune", "NNFF", "NudgelessLaneChange"};
  for (const QString &key : forceUpdateKeys) {
    QObject::connect(static_cast<ToggleControl*>(toggles[key]), &ToggleControl::toggleFlipped, this, &FrogPilotLateralPanel::updateToggles);
  }

  QSet<QString> rebootKeys = {"AlwaysOnLateral", "ForceTorqueController", "NNFF", "NNFFLite"};
  for (const QString &key : rebootKeys) {
    QObject::connect(static_cast<ToggleControl*>(toggles[key]), &ToggleControl::toggleFlipped, [key, this](bool state) {
      if (started) {
        if (key == "AlwaysOnLateral" && state) {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        } else if (key != "AlwaysOnLateral") {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        }
      }
    });
  }

  steerDelayToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerDelay"]);
  QObject::connect(steerDelayToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Actuator Delay</b> to its default value?"), this)) {
      params.putFloat("SteerDelay", parent->steerActuatorDelay);
      steerDelayToggle->refresh();
    }
  });

  steerFrictionToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerFriction"]);
  QObject::connect(steerFrictionToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Friction</b> to its default value?"), this)) {
      params.putFloat("SteerFriction", parent->friction);
      steerFrictionToggle->refresh();
    }
  });

  steerKPToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerKP"]);
  QObject::connect(steerKPToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Kp Factor</b> to its default value?"), this)) {
      params.putFloat("SteerKP", parent->steerKp);
      steerKPToggle->refresh();
    }
  });

  steerLatAccelToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerLatAccel"]);
  QObject::connect(steerLatAccelToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Lateral Accel</b> to its default value?"), this)) {
      params.putFloat("SteerLatAccel", parent->latAccelFactor);
      steerLatAccelToggle->refresh();
    }
  });

  steerRatioToggle = static_cast<FrogPilotParamValueButtonControl*>(toggles["SteerRatio"]);
  QObject::connect(steerRatioToggle, &FrogPilotParamValueButtonControl::buttonClicked, [parent, this]() {
    if (FrogPilotConfirmationDialog::yesorno(tr("Reset <b>Steer Ratio</b> to its default value?"), this)) {
      params.putFloat("SteerRatio", parent->steerRatio);
      steerRatioToggle->refresh();
    }
  });

  openDescriptions(forceOpenDescriptions, toggles);

  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubPanel, [lateralLayout, lateralPanel, this] {
    openDescriptions(forceOpenDescriptions, toggles);
    lateralLayout->setCurrentWidget(lateralPanel);
  });
  QObject::connect(parent, &FrogPilotSettingsWindow::updateMetric, this, &FrogPilotLateralPanel::updateMetric);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotLateralPanel::updateState);
}

void FrogPilotLateralPanel::showEvent(QShowEvent *event) {
  frogpilotToggleLevels = parent->frogpilotToggleLevels;

  steerDelayToggle->setTitle(QString(tr("Actuator Delay (Default: %1)")).arg(QString::number(parent->steerActuatorDelay, 'f', 2)));
  steerFrictionToggle->setTitle(QString(tr("Friction (Default: %1)")).arg(QString::number(parent->friction, 'f', 2)));
  steerKPToggle->setTitle(QString(tr("Kp Factor (Default: %1)")).arg(QString::number(parent->steerKp, 'f', 2)));
  steerKPToggle->updateControl(parent->steerKp * 0.5, parent->steerKp * 1.5);
  steerLatAccelToggle->setTitle(QString(tr("Lateral Accel (Default: %1)")).arg(QString::number(parent->latAccelFactor, 'f', 2)));
  steerLatAccelToggle->updateControl(parent->latAccelFactor * 0.75, parent->latAccelFactor * 1.25);
  steerRatioToggle->setTitle(QString(tr("Steer Ratio (Default: %1)")).arg(QString::number(parent->steerRatio, 'f', 2)));
  steerRatioToggle->updateControl(parent->steerRatio * 0.5, parent->steerRatio * 1.5);

  updateToggles();
}

void FrogPilotLateralPanel::updateState(const UIState &s) {
  if (!isVisible()) return;

  FrogPilotUIState &fs = *frogpilotUIState();
  started = s.scene.started;

  // These eight are the entire safety envelope on the interceptor path -- panda applies no
  // steering checks to MAZDA_TI_LKAS on gen1, so nothing downstream of openpilot double-checks
  // them. Unlike every other slider in this panel, they should not be adjustable with the car in
  // motion. ClearTiStats, TiFlagMoment and TiMcpEnabled are deliberately left out: none of them
  // change what gets commanded, and TiFlagMoment specifically exists to be used while driving.
  //
  // Gate on PARKED, not on `started`. The onroad flag follows the ignition line, so gating on it
  // locked the sliders with the engine running in Park and with the key on and the engine off --
  // exactly when tuning gets done. Same expression maps_settings.cc uses for map downloads:
  // offroad counts as parked, and onroad needs the shifter in P.
  bool parked = !started || fs.frogpilot_scene.parked;
  static const std::vector<QString> tiLimitKeys = {
    "TiSteerMax", "TiSteerDeltaUp", "TiSteerDeltaUpKnee", "TiSteerDeltaUpHigh",
    "TiSteerDeltaDown", "TiSteerDriverAllowance", "TiSteerDriverMultiplier", "TiSteerThreshold",
  };
  for (const QString &key : tiLimitKeys) {
    auto it = toggles.find(key);
    if (it != toggles.end()) {
      it->second->setEnabled(parked);
    }
  }

  if (lateralLayoutRef != nullptr && lateralLayoutRef->currentWidget() == torqueInterceptorPanelRef) {
    // Hold off the inactivity timeout while this panel is open. It would otherwise drop back to
    // the driving view precisely when you are watching the counters rather than touching anything.
    device()->resetInteractiveTimeout();

    // The counters are only written once a second, so reading the param file at the full UI rate
    // would buy nothing.
    if (++tiStatsTick >= 20) {
      tiStatsTick = 0;
      updateTorqueInterceptorStats();
    }
  }
}

void FrogPilotLateralPanel::updateTorqueInterceptorStats() {
  // Live counters come off tmpfs, where the car controller refreshes them every second. The flash
  // copy is only written once a minute now, so reading it here would show a run lagging by up to
  // that; fall back to it only when tmpfs is empty, which is the case before the first drive of a
  // boot. The previous run is a persisted snapshot and has no live counterpart.
  std::string curRaw = params_memory.get("TiTuningStats");
  if (curRaw.empty()) {
    curRaw = params.get("TiTuningStats");
  }
  QJsonObject cur = QJsonDocument::fromJson(QString::fromStdString(curRaw).toUtf8()).object();
  QJsonObject prev = QJsonDocument::fromJson(QString::fromStdString(params.get("TiTuningStatsPrevious")).toUtf8()).object();

  auto pct = [](const QJsonObject &o, const QString &key) {
    double engaged = o.value("engaged").toDouble();
    return engaged > 0.0 ? 100.0 * o.value(key).toDouble() / engaged : -1.0;
  };
  auto fmt = [](double v) { return v < 0.0 ? QString("—") : QString::number(v, 'f', 1) + "%"; };

  // Mean deficit alongside the percentage. How OFTEN the command was cut and how FAR short it
  // fell are different questions, and only the second tracks whether the car actually got what
  // openpilot asked for -- a change can shrink the deficit without moving the percentage at all.
  auto perFrame = [](const QJsonObject &o, const QString &key) {
    double engaged = o.value("engaged").toDouble();
    return engaged > 0.0 ? o.value(key).toDouble() / engaged : -1.0;
  };
  auto fmtCounts = [](double v) { return v < 0.0 ? QString("—") : QString::number(v, 'f', 1); };

  if (cur.value("engaged").toDouble() <= 0.0) {
    tiCommandCutLabel->setText(tr("no engaged driving recorded yet"));
  } else {
    QString text = QString(tr("%1, short by %2"))
                     .arg(fmt(pct(cur, "short")), fmtCounts(perFrame(cur, "deficit")));
    double was = perFrame(prev, "deficit");
    if (was >= 0.0) {
      text += QString(tr(" (was %1)")).arg(fmtCounts(was));
    }
    tiCommandCutLabel->setText(text);
  }

  tiLimitedByLabel->setText(QString(tr("rate %1, driver %2"))
                            .arg(fmt(pct(cur, "rate_limited")), fmt(pct(cur, "driver_limited"))));
  tiOutputLabel->setText(QString(tr("peak bias %1, %2 at clip"))
                         .arg(QString::number(cur.value("peak_bias").toInt()), fmt(pct(cur, "at_clip"))));

  // The car controller clears TiFlagMoment once it has recorded the flag, within a second. Until
  // then the button reads FLAGGED, so a tap still in flight looks different from one that landed
  // -- and the count going up is the confirmation that it did.
  if (tiFlagButton != nullptr) {
    tiFlagButton->setText(params_memory.getBool("TiFlagMoment") ? tr("FLAGGING") : tr("FLAG"));
    int flagged = QJsonDocument::fromJson(QString::fromStdString(params.get("TiFlaggedMoments")).toUtf8()).array().size();
    tiFlagButton->setValue(flagged > 0 ? tr("%1 saved").arg(flagged) : QString());
  }

  // Address is republished every 5s, so anything older than 15s means the service is not running.
  // On tmpfs: it is regenerated at every startup and never needed across a boot, so there was no
  // reason for a heartbeat to be writing to flash at that rate.
  QJsonObject mcp = QJsonDocument::fromJson(QString::fromStdString(params_memory.get("TiMcpAddress")).toUtf8()).object();
  qint64 age = QDateTime::currentSecsSinceEpoch() - (qint64)mcp.value("heartbeat").toDouble();
  if (mcp.isEmpty() || age > 15) {
    tiMcpLabel->setText(tr("not running"));
  } else if (!mcp.value("reachable_remotely").toBool()) {
    tiMcpLabel->setText(tr("localhost only — set TI_MCP_HOST=0.0.0.0"));
  } else {
    tiMcpLabel->setText(mcp.value("url").toString());
  }

  int notRun = cur.value("not_run").toInt();
  int ramp = cur.value("ramp").toInt();
  int viol = cur.value("viol").toInt();
  if (notRun == 0 && ramp == 0 && viol == 0) {
    tiHealthLabel->setText(tr("RUN, no violations"));
  } else {
    tiHealthLabel->setText(QString(tr("%1 not in RUN, %2 ramping, violation 0x%3"))
                           .arg(notRun).arg(ramp).arg(viol, 2, 16, QChar('0')));
  }
}

void FrogPilotLateralPanel::updateMetric(bool metric, bool bootRun) {
  static bool previousMetric;
  if (metric != previousMetric && !bootRun) {
    double distanceConversion = metric ? FOOT_TO_METER : METER_TO_FOOT;
    double speedConversion = metric ? MILE_TO_KM : KM_TO_MILE;

    params.putFloatNonBlocking("LaneDetectionWidth", params.getFloat("LaneDetectionWidth") * distanceConversion);

    params.putIntNonBlocking("MinimumLaneChangeSpeed", params.getInt("MinimumLaneChangeSpeed") * speedConversion);
    params.putIntNonBlocking("PauseAOLOnBrake", params.getInt("PauseAOLOnBrake") * speedConversion);
    params.putIntNonBlocking("PauseLateralSpeed", params.getInt("PauseLateralSpeed") * speedConversion);
  }
  previousMetric = metric;

  static std::map<float, QString> imperialDistanceLabels;
  static std::map<float, QString> imperialSpeedLabels;
  static std::map<float, QString> metricDistanceLabels;
  static std::map<float, QString> metricSpeedLabels;

  static bool labelsInitialized = false;
  if (!labelsInitialized) {
    for (int i = 0; i <= 150; ++i) {
      float key = i / 10.0f;
      imperialDistanceLabels[key] = key == 0 ? tr("Off") : i == 1 ? QString::number(i) + tr(" foot") : QString::number(key, 'f', 1) + tr(" feet");
    }

    for (int i = 0; i <= 99; ++i) {
      imperialSpeedLabels[i] = i == 0 ? tr("Off") : QString::number(i) + tr(" mph");
    }

    for (int i = 0; i <= 50; ++i) {
      float key = i / 10.0f;
      metricDistanceLabels[key] = key == 0 ? tr("Off") : i == 1 ? QString::number(i) + tr(" meter") : QString::number(key, 'f', 1) + tr(" meters");
    }

    for (int i = 0; i <= 150; ++i) {
      metricSpeedLabels[i] = i == 0 ? tr("Off") : QString::number(i) + tr(" km/h");
    }

    labelsInitialized = true;
  }

  FrogPilotParamValueControl *laneWidthToggle = static_cast<FrogPilotParamValueControl*>(toggles["LaneDetectionWidth"]);
  FrogPilotParamValueControl *minimumLaneChangeSpeedToggle = static_cast<FrogPilotParamValueControl*>(toggles["MinimumLaneChangeSpeed"]);
  FrogPilotParamValueControl *pauseAOLOnBrakeToggle = static_cast<FrogPilotParamValueControl*>(toggles["PauseAOLOnBrake"]);
  FrogPilotParamValueControl *pauseLateralToggle = static_cast<FrogPilotParamValueControl*>(toggles["PauseLateralSpeed"]);

  if (metric) {
    laneWidthToggle->updateControl(0, 5, metricDistanceLabels);

    minimumLaneChangeSpeedToggle->updateControl(0, 150, metricSpeedLabels);
    pauseAOLOnBrakeToggle->updateControl(0, 150, metricSpeedLabels);
    pauseLateralToggle->updateControl(0, 150, metricSpeedLabels);
  } else {
    laneWidthToggle->updateControl(0, 15, imperialDistanceLabels);

    minimumLaneChangeSpeedToggle->updateControl(0, 99, imperialSpeedLabels);
    pauseAOLOnBrakeToggle->updateControl(0, 99, imperialSpeedLabels);
    pauseLateralToggle->updateControl(0, 99, imperialSpeedLabels);
  }
}

void FrogPilotLateralPanel::updateToggles() {
  for (auto &[key, toggle] : toggles) {
    if (parentKeys.contains(key)) {
      toggle->setVisible(false);
    }
  }

  bool forcingAutoTune = !parent->hasAutoTune && params.getBool("ForceAutoTune");
  bool forcingAutoTuneOff = parent->hasAutoTune && params.getBool("ForceAutoTuneOff");
  bool forcingTorqueController = !parent->isAngleCar && params.getBool("ForceTorqueController");
  bool usingNNFF = parent->hasNNFFLog && params.getBool("LateralTune") && params.getBool("NNFF");

  for (auto &[key, toggle] : toggles) {
    if (parentKeys.contains(key)) {
      continue;
    }

    bool setVisible = parent->tuningLevel >= frogpilotToggleLevels[key].toDouble();

    if (key == "AlwaysOnLateralLKAS") {
      setVisible &= parent->isHKGCanFd;
      setVisible &= !parent->hasOpenpilotLongitudinal;
    }

    else if (key == "AlwaysOnLateralMain") {
      setVisible &= !parent->isHKGCanFd;
      setVisible |= parent->hasOpenpilotLongitudinal;
    }

    else if (key == "ForceAutoTune") {
      setVisible &= !parent->hasAutoTune;
      setVisible &= !parent->isAngleCar;
      setVisible &= parent->isTorqueCar || forcingTorqueController || usingNNFF;
    }

    else if (key == "ForceAutoTuneOff") {
      setVisible &= parent->hasAutoTune;
    }

    else if (key == "ForceTorqueController") {
      setVisible &= !parent->isAngleCar;
      setVisible &= !parent->isTorqueCar;
    }

    else if (key == "LaneChangeTime") {
      setVisible &= params.getBool("LaneChanges") && params.getBool("NudgelessLaneChange");
    }

    else if (key == "LaneDetectionWidth") {
      setVisible &= params.getBool("LaneChanges") && params.getBool("NudgelessLaneChange");
    }

    else if (key == "NNFF") {
      setVisible &= parent->hasNNFFLog;
      setVisible &= !parent->isAngleCar;
    }

    else if (key == "NNFFLite") {
      setVisible &= !usingNNFF;
      setVisible &= !parent->isAngleCar;
    }

    else if (key == "SteerDelay") {
      setVisible &= parent->steerActuatorDelay != 0;
    }

    else if (key == "SteerFriction") {
      setVisible &= parent->friction != 0;
      setVisible &= parent->hasAutoTune ? forcingAutoTuneOff : !forcingAutoTune;
      setVisible &= parent->isTorqueCar || forcingTorqueController || usingNNFF;
      setVisible &= !usingNNFF;
    }

    else if (key == "SteerKP") {
      setVisible &= parent->steerKp != 0;
      setVisible &= parent->isTorqueCar || forcingTorqueController || usingNNFF;
      setVisible &= !parent->isAngleCar;
    }

    else if (key == "SteerLatAccel") {
      setVisible &= parent->latAccelFactor != 0;
      setVisible &= parent->hasAutoTune ? forcingAutoTuneOff : !forcingAutoTune;
      setVisible &= parent->isTorqueCar || forcingTorqueController || usingNNFF;
      setVisible &= !usingNNFF;
    }

    else if (key == "SteerRatio") {
      setVisible &= parent->steerRatio != 0;
      setVisible &= parent->hasAutoTune ? forcingAutoTuneOff : !forcingAutoTune;
    }

    toggle->setVisible(setVisible);

    if (setVisible) {
      if (advancedLateralTuneKeys.contains(key)) {
        toggles["AdvancedLateralTune"]->setVisible(true);
      } else if (aolKeys.contains(key)) {
        toggles["AlwaysOnLateral"]->setVisible(true);
      } else if (laneChangeKeys.contains(key)) {
        toggles["LaneChanges"]->setVisible(true);
      } else if (lateralTuneKeys.contains(key)) {
        toggles["LateralTune"]->setVisible(true);
      } else if (qolKeys.contains(key)) {
        toggles["QOLLateral"]->setVisible(true);
      } else if (torqueInterceptorKeys.contains(key)) {
        toggles["TorqueInterceptorTune"]->setVisible(true);
      }
    }
  }

  openDescriptions(forceOpenDescriptions, toggles);

  update();
}
