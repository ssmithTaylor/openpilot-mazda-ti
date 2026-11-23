"""pytest plugin loaded by tools/dev/pc_test.py (-p pc_plugin).

The repo's root conftest.py imports openpilot.system.manager before any test; that import order is what breaks the
FrogPilot cycle car_helpers -> sentry -> frogpilot_variables -> car_helpers. pc_test skips the root conftest (it needs
the device's processes), so it reproduces only the import order here.
"""


def pytest_configure(config):
  import openpilot.system.sentry  # noqa: F401
