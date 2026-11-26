#!/usr/bin/env python3
import datetime
import os
import subprocess
import time
from typing import NoReturn

from timezonefinder import TimezoneFinder

import cereal.messaging as messaging
from openpilot.common.time import system_time_valid
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import AGNOS


def set_timezone(timezone):
  valid_timezones = subprocess.check_output('timedatectl list-timezones', shell=True, encoding='utf8').strip().split('\n')
  if timezone not in valid_timezones:
    cloudlog.error(f"Timezone not supported {timezone}")
    return

  cloudlog.debug(f"Setting timezone to {timezone}")
  try:
    if AGNOS:
      tzpath = os.path.join("/usr/share/zoneinfo/", timezone)
      subprocess.check_call(f'sudo su -c "ln -snf {tzpath} /data/etc/tmptime && \
                              mv /data/etc/tmptime /data/etc/localtime"', shell=True)
      subprocess.check_call(f'sudo su -c "echo \"{timezone}\" > /data/etc/timezone"', shell=True)
    else:
      subprocess.check_call(f'sudo timedatectl set-timezone {timezone}', shell=True)
  except subprocess.CalledProcessError:
    cloudlog.exception(f"Error setting timezone to {timezone}")


def set_time(new_time):
  # Correct the clock when it is off from GPS time by 10 s or more in EITHER direction. Upstream only corrected a clock
  # that was ahead (diff >= 10 s), so a clock left far BEHIND (no internet, no NTP) was never fixed from GPS.
  diff = datetime.datetime.now() - new_time
  if abs(diff) < datetime.timedelta(seconds=10):
    cloudlog.debug(f"Time diff too small: {diff}")
    return

  cloudlog.debug(f"Setting time to {new_time}")
  try:
    subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
  except subprocess.CalledProcessError:
    cloudlog.exception("timed.failed_setting_time")
    cloudlog.event("timed.set_time", ok=False, offset_before_s=round(diff.total_seconds(), 3))
    return
  # Replayability: system clock minus GPS time before and after the correction (seconds only, no position).
  after = datetime.datetime.now() - new_time
  cloudlog.event("timed.set_time", ok=True, offset_before_s=round(diff.total_seconds(), 3),
                 offset_after_s=round(after.total_seconds(), 3))


def main() -> NoReturn:
  """
    timed has two responsibilities:
    - getting the current time
    - getting the current timezone

    GPS directly gives time, and timezone is looked up from GPS position.
    AGNOS will also use NTP to update the time.
  """

  params = Params()

  # Restore timezone from param
  tz = params.get("Timezone", encoding='utf8')
  tf = TimezoneFinder()
  if tz is not None:
    cloudlog.debug("Restoring timezone from param")
    set_timezone(tz)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster(['liveLocationKalman'])
  while True:
    sm.update(1000)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    llk = sm['liveLocationKalman']
    if not llk.gpsOK or (time.monotonic() - sm.logMonoTime['liveLocationKalman']/1e9) > 0.2:
      continue

    # set time
    # TODO: account for unixTimesatmpMillis being a (usually short) time in the past
    gps_time = datetime.datetime.fromtimestamp(llk.unixTimestampMillis / 1000.)
    set_time(gps_time)

    # set timezone
    pos = llk.positionGeodetic.value
    if len(pos) == 3:
      gps_timezone = tf.timezone_at(lat=pos[0], lng=pos[1])
      if gps_timezone is None:
        cloudlog.critical(f"No timezone found based on {pos=}")
      else:
        set_timezone(gps_timezone)
        params.put_nonblocking("Timezone", gps_timezone)

    time.sleep(10)

if __name__ == "__main__":
  main()
