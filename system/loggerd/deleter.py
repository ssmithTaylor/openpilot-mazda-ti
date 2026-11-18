#!/usr/bin/env python3
import os
import shutil
import threading
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.system.loggerd.xattr_cache import getxattr

MIN_BYTES = 5 * 1024 * 1024 * 1024
MIN_PERCENT = 10

DELETE_LAST = ['boot', 'crash']

# full-resolution camera files, deleted before any log (the small qcamera.ts preview is kept)
VIDEO_SUFFIXES = ('.hevc',)

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5


def has_preserve_xattr(d: str) -> bool:
  return getxattr(os.path.join(Paths.log_root(), d), PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE


def get_preserved_segments(dirs_by_creation: list[str]) -> list[str]:
  preserved = []
  for n, d in enumerate(filter(has_preserve_xattr, reversed(dirs_by_creation))):
    if n == PRESERVE_COUNT:
      break
    date_str, _, seg_str = d.rpartition("--")

    # ignore non-segment directories
    if not date_str:
      continue
    try:
      seg_num = int(seg_str)
    except ValueError:
      continue

    # preserve segment and its prior
    preserved.append(d)
    preserved.append(f"{date_str}--{seg_num - 1}")

  return preserved


def delete_videos(delete_order: list[str]) -> bool:
  """Remove the full-resolution camera files of the earliest segment that still has any.

  The camera files are about 225 MB of a 270 MB segment, so dropping them first keeps the logs
  (rlog, qlog) and the small preview video several times longer than deleting whole segments.
  Returns True if something was removed, so the caller only falls back to deleting whole
  segments once no camera files are left.
  """
  for delete_dir in delete_order:
    delete_path = os.path.join(Paths.log_root(), delete_dir)
    try:
      names = os.listdir(delete_path)
    except OSError:
      continue

    if any(name.endswith(".lock") for name in names):
      continue

    videos = [name for name in names if name.endswith(VIDEO_SUFFIXES)]
    if not videos:
      continue

    removed = False
    for name in videos:
      try:
        cloudlog.info(f"deleting {os.path.join(delete_path, name)}")
        os.remove(os.path.join(delete_path, name))
        removed = True
      except OSError:
        cloudlog.exception(f"issue deleting {os.path.join(delete_path, name)}")
    if removed:
      return True
  return False


def deleter_thread(exit_event):
  while not exit_event.is_set():
    out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
    out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

    if out_of_percent or out_of_bytes:
      dirs = listdir_by_creation(Paths.log_root())

      # skip deleting most recent N preserved segments (and their prior segment)
      preserved_dirs = get_preserved_segments(dirs)
      delete_order = sorted(dirs, key=lambda d: (d in DELETE_LAST, d in preserved_dirs))

      # camera files go first; logs are only deleted once no camera files are left
      if delete_videos(delete_order):
        exit_event.wait(.1)
        continue

      # remove the earliest directory we can
      for delete_dir in delete_order:
        delete_path = os.path.join(Paths.log_root(), delete_dir)

        if any(name.endswith(".lock") for name in os.listdir(delete_path)):
          continue

        try:
          cloudlog.info(f"deleting {delete_path}")
          if os.path.isfile(delete_path):
            os.remove(delete_path)
          else:
            shutil.rmtree(delete_path)
          break
        except OSError:
          cloudlog.exception(f"issue deleting {delete_path}")
      exit_event.wait(.1)
    else:
      exit_event.wait(30)


def main():
  deleter_thread(threading.Event())


if __name__ == "__main__":
  main()
