#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotLateralPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotLateralPanel(FrogPilotSettingsWindow *parent);

signals:
  void openSubPanel();

protected:
  void showEvent(QShowEvent *event) override;

private:
  void updateMetric(bool metric, bool bootRun);
  void updateState(const UIState &s);
  void updateToggles();

  bool forceOpenDescriptions;
  bool started;

  std::map<QString, AbstractControl*> toggles;

  QSet<QString> advancedLateralTuneKeys = {"ForceAutoTune", "ForceAutoTuneOff", "ForceTorqueController", "SteerDelay", "SteerFriction", "SteerLatAccel", "SteerKP", "SteerRatio"};
  QSet<QString> aolKeys = {"AlwaysOnLateralLKAS", "AlwaysOnLateralMain", "PauseAOLOnBrake"};
  QSet<QString> laneChangeKeys = {"LaneChangeTime", "LaneDetectionWidth", "MinimumLaneChangeSpeed", "NudgelessLaneChange", "OneLaneChange"};
  QSet<QString> lateralTuneKeys = {"TurnDesires"};
  QSet<QString> qolKeys = {"PauseLateralSpeed"};
  // Every TI control must appear here. This set decides both which sub-panel a control lands in and
  // whether the parent stays reachable at all -- a key omitted here is a control that silently
  // never appears, which is how the whole panel went missing once already.
  QSet<QString> torqueInterceptorKeys = {"TiSteerMax", "TiSteerDeltaUp", "TiSteerDeltaUpKnee",
                                         "TiSteerDeltaUpHigh", "TiSteerDeltaDown", "TiSteerDriverAllowance",
                                         "TiSteerDriverMultiplier", "TiSteerThreshold", "ResetTorqueParams",
                                         "ClearTiStats", "TiFlagMoment", "TiMcpEnabled",
                                         "LatOutputFilter", "LatNoFrictionRelay",
                                         "LatStallModulation", "LatDemandCap", "LatFFLookahead",
                                         "SteerAuthorityAdvisory",
                                         "NNFF", "NNFFLite", "NNFFGainCorrection"};

  QSet<QString> parentKeys;

  FrogPilotParamValueButtonControl *steerDelayToggle;
  FrogPilotParamValueButtonControl *steerFrictionToggle;
  FrogPilotParamValueButtonControl *steerLatAccelToggle;
  FrogPilotParamValueButtonControl *steerKPToggle;
  FrogPilotParamValueButtonControl *steerRatioToggle;

  void updateTorqueInterceptorStats();

  QStackedLayout *lateralLayoutRef = nullptr;
  QWidget *torqueInterceptorPanelRef = nullptr;
  LabelControl *tiCommandCutLabel = nullptr;
  LabelControl *tiLimitedByLabel = nullptr;
  LabelControl *tiOutputLabel = nullptr;
  LabelControl *tiHealthLabel = nullptr;
  LabelControl *tiMcpLabel = nullptr;
  // Held so its text can be put back once the car controller has consumed the flag, which is the
  // only feedback the driver gets that the tap actually landed.
  ButtonControl *tiFlagButton = nullptr;
  ButtonControl *tiClearButton = nullptr;
  int tiStatsTick = 0;

  FrogPilotSettingsWindow *parent;

  QJsonObject frogpilotToggleLevels;

  Params params;
  // The TI counters and the MCP address are republished every few seconds. They live on tmpfs so
  // that cadence costs nothing: on /data each write forced an ext4 journal commit underneath the
  // camera pipeline, several times a minute, for the whole drive.
  Params params_memory{"/dev/shm/params"};
};
