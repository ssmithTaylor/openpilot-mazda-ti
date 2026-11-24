"""PC stand-in for smbus2 (the real package imports fcntl and cannot load on Windows). Calls raise loudly."""


class SMBus:
  def __init__(self, *a, **k):
    raise OSError("smbus2 is stubbed on the PC: no I2C bus")


class i2c_msg:
  @staticmethod
  def read(*a, **k):
    raise OSError("smbus2 is stubbed on the PC")

  @staticmethod
  def write(*a, **k):
    raise OSError("smbus2 is stubbed on the PC")
