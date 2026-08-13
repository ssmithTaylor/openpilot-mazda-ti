using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Car = import "car.capnp";

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# you can rename the struct, but don't change the identifier
struct FrogPilotCarControl {
  hudControl @0 :HUDControl;

  struct HUDControl {
    audibleAlert @0 :AudibleAlert;

    enum AudibleAlert {
      none @0;

      engage @1;
      disengage @2;
      refuse @3;

      warningSoft @4;
      warningImmediate @5;

      prompt @6;
      promptRepeat @7;
      promptDistracted @8;

      # Random Events
      angry @9;
      continued @10;
      dejaVu @11;
      doc @12;
      fart @13;
      firefox @14;
      goat @15;
      hal9000 @16;
      mail @17;
      nessie @18;
      noice @19;
      startup @20;
      thisIsFine @21;
      uwu @22;
    }
  }
}

struct FrogPilotCarEvent @0x81c2f05a394cf4af {
  name @0 :EventName;

  enable @1 :Bool;
  noEntry @2 :Bool;
  warning @3 :Bool;
  userDisable @4 :Bool;
  softDisable @5 :Bool;
  immediateDisable @6 :Bool;
  preEnable @7 :Bool;
  permanent @8 :Bool;
  overrideLateral @10 :Bool;
  overrideLongitudinal @9 :Bool;

  enum EventName @0xaedffd8f31e7b55d {
    blockUser @0;
    customStartupAlert @1;
    forcingStop @2;
    goatSteerSaturated @3;
    greenLight @4;
    holidayActive @5;
    laneChangeBlockedLoud @6;
    leadDeparting @7;
    noLaneAvailable @8;
    openpilotCrashed @9;
    pedalInterceptorNoBrake @10;
    speedLimitChanged @11;
    torqueNNLoad @12;
    trafficModeActive @13;
    trafficModeInactive @14;
    turningLeft @15;
    turningRight @16;

    # Random Events
    accel30 @17;
    accel35 @18;
    accel40 @19;
    dejaVuCurve @20;
    firefoxSteerSaturated @21;
    hal9000 @22;
    openpilotCrashedRandomEvent @23;
    thisIsFineSteerSaturated @24;
    toBeContinued @25;
    vCruise69 @26;
    yourFrogTriedToKillMe @27;
    youveGotMail @28;
  }
}

struct FrogPilotCarParams @0xf35cc4560bbf6ec2 {
  canUsePedal @0 :Bool;
  canUseSDSU @1 :Bool;
  fpFlags @2 :UInt32;
  isHDA2 @3 :Bool;
  openpilotLongitudinalControlDisabled @4 :Bool;
  safetyConfigs @5 :List(SafetyConfig);

  struct SafetyConfig {
    safetyParam @0 :UInt16;
  }
}

struct FrogPilotCarState @0xda96579883444c35 {
  accelPressed @0 :Bool;
  alwaysOnLateralAllowed @1 :Bool;
  alwaysOnLateralEnabled @2 :Bool;
  brakeLights @3 :Bool;
  dashboardSpeedLimit @4 :Float32;
  decelPressed @5 :Bool;
  distancePressed @6 :Bool;
  distanceLongPressed @7 :Bool;
  distanceVeryLongPressed @8 :Bool;
  ecoGear @9 :Bool;
  forceCoast @10 :Bool;
  isParked @11 :Bool;
  pauseLateral @12 :Bool;
  pauseLongitudinal @13 :Bool;
  sportGear @14 :Bool;
  trafficModeEnabled @15 :Bool;

  struct ButtonEvent {
    enum Type {
      lkas @0;
    }
  }
}

struct FrogPilotControlsState @0x80ae746ee2596b11 {
  alertStatus @0 :AlertStatus;
  alertText1 @1 :Text;
  alertText2 @2 :Text;
  alertSize @3 :AlertSize;
  alertBlinkingRate @4 :Float32;
  alertType @5 :Text;
  alertSound @6 :Car.CarControl.HUDControl.AudibleAlert;

  enum AlertSize {
    none @0;    # don't display the alert
    small @1;   # small box
    mid @2;     # mid screen
    full @3;    # full screen
  }

  enum AlertStatus {
    normal @0;       # low priority alert for user's convenience
    userPrompt @1;   # mid priority alert that might require user intervention
    critical @2;     # high priority alert that needs immediate user intervention
    frogpilot @3;    # FrogPilot startup alert
  }
}

struct FrogPilotDeviceState @0xa5cd762cd951a455 {
  freeSpace @0 :Int16;
  usedSpace @1 :Int16;
}

struct FrogPilotModelDataV2 @0xf98d843bfd7004a3 {
  turnDirection @0 :TurnDirection;

  enum TurnDirection {
    none @0;
    turnLeft @1;
    turnRight @2;
  }
}

struct FrogPilotNavigation @0xf416ec09499d9d19 {
  approachingIntersection @0 :Bool;
  approachingTurn @1 :Bool;
  navigationSpeedLimit @2 :Float32;
}

struct FrogPilotPlan @0xa1680744031fdb2d {
  accelerationJerk @0 :Float32;
  accelerationJerkStock @1 :Float32;
  cscControllingSpeed @2 :Bool;
  cscSpeed @3 :Float32;
  cscTraining @4 :Bool;
  dangerFactor @5 :Float32;
  dangerJerk @6 :Float32;
  desiredFollowDistance @7 :Int64;
  experimentalMode @8 :Bool;
  forcingStop @9 :Bool;
  forcingStopLength @10 :Float32;
  frogpilotEvents @11 :List(FrogPilotCarEvent);
  increasedStoppedDistance @12 :Float32;
  lateralCheck @13 :Bool;
  laneWidthLeft @14 :Float32;
  laneWidthRight @15 :Float32;
  maxAcceleration @16 :Float32;
  minAcceleration @17 :Float32;
  redLight @18 :Bool;
  roadCurvature @19 :Float32;
  slcMapSpeedLimit @20 :Float32;
  slcMapboxSpeedLimit @21 :Float32;
  slcNextSpeedLimit @22 :Float32;
  slcOverriddenSpeed @23 :Float32;
  slcSpeedLimit @24 :Float32;
  slcSpeedLimitOffset @25 :Float32;
  slcSpeedLimitSource @26 :Text;
  speedJerk @27 :Float32;
  speedJerkStock @28 :Float32;
  speedLimitChanged @29 :Bool;
  tFollow @30 :Float32;
  themeUpdated @31 :Bool;
  togglesUpdated @32 :Bool;
  trackingLead @33 :Bool;
  unconfirmedSlcSpeedLimit @34 :Float32;
  vCruise @35 :Float32;
  weatherDaytime @36 :Bool;
  weatherId @37 :Int16;
}

# Torque Interceptor telemetry, published by the Mazda car controller once a second.
#
# A message rather than a param because the car controller runs at 100Hz in a SCHED_FIFO thread,
# where writing a param costs either a thread spawn -- put_nonblocking creates one through
# std::async on nearly every call -- or a blocking write plus a file lock. Either was enough to
# skip a carState frame once a second on a fixed phase, which made the 20Hz consumers polling
# carState fail their all_checks and flag their own output invalid, surfacing to the driver as a
# constant "communication issue between processes". A cereal publish is a shared-memory write.
struct FrogPilotTorqueInterceptor {
  # Live interceptor state. Stable since inception, and read on its own, so typed.
  mode @0 :UInt8;                 # 0 DISCOVER, 1 OFF, 2 DRIVER_OVER, 3 RUN
  violation @1 :UInt8;            # 0 = none
  rampingDown @2 :Bool;
  version @3 :UInt8;
  lkasAllowed @4 :Bool;

  # Identity, so a saved run ties back to the segments that produced it. startedAt stays typed
  # because the run-banking logic keys on it changing.
  route @5 :Text;
  startedAt @6 :Float64;

  # The limits this run was recorded under. Landing these in every rlog and qlog makes
  # analyze_segment's standing caveat obsolete -- it no longer has to assume the params as they are
  # NOW when attributing an old drive.
  steerMax @7 :UInt16;
  steerDeltaUp @8 :UInt16;
  steerDeltaDown @9 :UInt16;
  steerDeltaUpKnee @10 :UInt16;
  steerDeltaUpHigh @11 :UInt16;
  driverAllowance @12 :UInt16;
  driverMultiplier @13 :UInt16;
  steerThreshold @14 :UInt16;

  # The counters, carried as JSON rather than as typed fields. Deliberate: this set changed three
  # times in one week, every schema change costs a full cereal rebuild on a device whose build has
  # already failed once, and typing them meant four hand-synced copies of the same field list --
  # the dict keys, the JSON payload, the publish assignments and the reverse mapping. That is the
  # N-copies-must-agree pattern that has already caused two bugs here. Adding a counter is now a
  # Python-only change. Costs ~450 bytes/s in the logs and typed access that nothing consumes:
  # every reader of these is our own Python.
  countersJson @15 :Text;
}

struct FrogPilotRadarState @0xcb9fd56c7057593a {
  leadLeft @0 :LeadData;
  leadRight @1 :LeadData;

  struct LeadData {
    dRel @0 :Float32;
    yRel @1 :Float32;
    vRel @2 :Float32;
    aRel @3 :Float32;
    vLead @4 :Float32;
    dPath @6 :Float32;
    vLat @7 :Float32;
    vLeadK @8 :Float32;
    aLeadK @9 :Float32;
    fcw @10 :Bool;
    status @11 :Bool;
    aLeadTau @12 :Float32;
    modelProb @13 :Float32;
    radar @14 :Bool;
    radarTrackId @15 :Int32 = -1;

    aLeadDEPRECATED @5 :Float32;
  }
}
