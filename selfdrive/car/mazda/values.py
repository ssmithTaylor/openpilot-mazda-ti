from dataclasses import dataclass, field
from enum import IntFlag

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarDocs, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  def __init__(self, CP):
    self.STEER_STEP = 1 # 100 Hz
    if CP.flags & MazdaFlags.GEN1:
      self.STEER_MAX = 600                # theoretical max_steer 2047
      self.STEER_DELTA_UP = 10             # torque increase per refresh
      self.STEER_DELTA_DOWN = 25           # torque decrease per refresh
      self.STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 40     # weight driver torque
      self.STEER_DRIVER_FACTOR = 1         # from dbc
      self.STEER_ERROR_MAX = 350           # max delta between torque cmd and torque motor

      self.TI_STEER_MAX = 600                # theoretical max_steer 2047
      self.TI_STEER_DELTA_UP = 6             # torque increase per refresh
      self.TI_STEER_DELTA_DOWN = 15           # torque decrease per refresh
      self.TI_STEER_DRIVER_ALLOWANCE = 15    # allowed driver torque before start limiting
      self.TI_STEER_DRIVER_MULTIPLIER = 40     # weight driver torque
      self.TI_STEER_DRIVER_FACTOR = 1         # from dbc
      self.TI_STEER_ERROR_MAX = 350           # max delta between torque cmd and torque motor
      # Two-stage climb rate. Below the knee the command may ramp at TI_STEER_DELTA_UP, above it at
      # TI_STEER_DELTA_UP_HIGH. Defaulted to the full range and the same rate, which reproduces the
      # single-slope limiter exactly -- lowering the knee is what turns the feature on.
      self.TI_STEER_DELTA_UP_KNEE = 600       # command magnitude above which the slow rate applies
      self.TI_STEER_DELTA_UP_HIGH = 6         # torque increase per refresh above the knee
    if CP.flags & (MazdaFlags.GEN2 | MazdaFlags.GEN3):
      self.STEER_MAX = 8000
      self.STEER_DELTA_UP = 45              # torque increase per refresh
      self.STEER_DELTA_DOWN = 80            # torque decrease per refresh
      self.STEER_DRIVER_ALLOWANCE = 1400     # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 5      # weight driver torque
      self.STEER_DRIVER_FACTOR = 1           # from dbc
      self.STEER_ERROR_MAX = 3500            # max delta between torque cmd and torque motor

class TI_STATE:
  DISCOVER = 0
  OFF = 1
  DRIVER_OVER = 2
  RUN = 3

@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1
  GEN2 = 2
  GEN3 = 4
  TORQUE_INTERCEPTOR = 8
  RADAR_INTERCEPTOR = 16
  NO_FSC = 32
  NO_MRCC = 64
  MANUAL_TRANSMISSION = 128

@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('mazda_2017', None))
  def init(self):
    if self.flags & MazdaFlags.GEN2:
      self.dbc_dict = dbc_dict('mazda_2019', None)
    elif self.flags & MazdaFlags.GEN1 and self.flags & MazdaFlags.RADAR_INTERCEPTOR:
      self.dbc_dict = dbc_dict('mazda_2017', 'mazda_radar')
    elif self.flags & MazdaFlags.GEN3:
      self.dbc_dict = dbc_dict('mazda_2023', None)



class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5),
    flags=MazdaFlags.GEN1
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6),
    flags=MazdaFlags.GEN1,
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0),
    flags=MazdaFlags.GEN1,
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5),
    flags=MazdaFlags.GEN1,
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video_link="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs,
    flags=MazdaFlags.GEN1,
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-24")],
    MAZDA_CX5.specs,
    flags=MazdaFlags.GEN1,
  )
  MAZDA_3_2019 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2019-24")],
    MazdaCarSpecs(mass=3000 * CV.LB_TO_KG, wheelbase=2.725, steerRatio=18.8),
    flags=MazdaFlags.GEN2,
  )
  MAZDA_CX_30 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-30 2019-22")],
    MazdaCarSpecs(mass=3375 * CV.LB_TO_KG, wheelbase=2.814, steerRatio=15.5),
    flags=MazdaFlags.GEN2,
  )
  MAZDA_CX_50 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-50 2022-24")],
    MazdaCarSpecs(mass=3375 * CV.LB_TO_KG, wheelbase=2.814, steerRatio=15.5),
    flags=MazdaFlags.GEN2,
  )
  MAZDA_3_2023 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2024-26")],
    MazdaCarSpecs(mass=3000 * CV.LB_TO_KG, wheelbase=2.725, steerRatio=18.8),
    flags=MazdaFlags.GEN3,
  )
  MAZDA_CX_30_2023 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-30 23-26")],
    MazdaCarSpecs(mass=3375 * CV.LB_TO_KG, wheelbase=2.814, steerRatio=15.5),
    flags=MazdaFlags.GEN3,
  )


# Absolute bounds on every live Torque Interceptor limit. Defined here, once, because they are
# restated in four other places -- the toggle clips in frogpilot_variables, the re-clamp in the car
# controller, the UI sliders, and the telemetry server's report of what is allowed -- and the one
# that drifts silently is whichever copy nothing executes against.
#
# These are the entire safety envelope on this path: the panda applies steering checks to the stock
# MAZDA_LKAS message but none at all to MAZDA_TI_LKAS on gen1. Chosen to leave room for the tuning
# intended and no more.
#
# Floors matter as much as ceilings, and on two of these the dangerous direction is downward:
#   TI_STEER_DELTA_DOWN -- the driver-torque cap is applied BEFORE the rate clip, so when an
#     override collapses the cap the command can still only fall at this rate. At 1 count/frame,
#     unwinding from a command of 313 is 3.1 seconds of decaying counter-torque against the
#     driver's hands; at the default 15 it is 0.21s, and the stock Mazda path uses 25.
#   TI_STEER_DRIVER_MULTIPLIER -- the cap reaches zero at |driver| = allowance + 600/multiplier.
#     At the default 40 that is 30 counts. At 10 it would be 75, against a torque sensor whose
#     range ends at 85 -- so the driver could not fully yield the command before the sensor
#     saturates. At 20 it is 45: a hard push, but one a person can actually make.
TI_LIMIT_BOUNDS = {
  # 620 deliberately exceeds the documented 600 clip by 3.3%, to test whether that clip is real.
  # "The TI discards anything above 600" is vendor documentation plus a code comment; it has never
  # been measured, because openpilot has always clamped first and so commands_above_600 is 0 in
  # every log ever taken (OPEN_QUESTIONS Q5). The three outcomes are all informative: the request
  # is clipped to 600 internally (claim confirmed, no-op), it is honoured (claim false, and the
  # ceiling is ours not the device's), or the unit raises a violation and bypasses -- which is
  # degraded assist, not runaway torque. Kept small and incremental per TI_DEVICE_SPEC section 5,
  # since the calibration is unknown and so is the CAN-count value that trips the unit's own
  # plausibility monitor. The default stays 600: nothing changes unless the slider is moved.
  "TI_STEER_MAX": (100, 620),
  "TI_STEER_DELTA_UP": (1, 15),             # stock Mazda path runs 10 against the same EPS
  "TI_STEER_DELTA_DOWN": (10, 50),
  "TI_STEER_DRIVER_ALLOWANCE": (5, 30),
  "TI_STEER_DRIVER_MULTIPLIER": (20, 60),
  "TI_STEER_DELTA_UP_KNEE": (100, 600),
  "TI_STEER_DELTA_UP_HIGH": (1, 15),
}


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph
  TI_STEER_THRESHOLD = 6
  TI_DISABLE_SPEED = 0    # kph
  TI_ENABLE_SPEED = 0     # kph

class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4
  TURN_ON = 5


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
      whitelist_ecus=[Ecu.eps, Ecu.abs, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.shiftByWire],
    )
  ],
)

DBC = CAR.create_dbc_map()
GEN1 = CAR.with_flags(MazdaFlags.GEN1)
GEN2 = CAR.with_flags(MazdaFlags.GEN2)
GEN3 = CAR.with_flags(MazdaFlags.GEN3)
