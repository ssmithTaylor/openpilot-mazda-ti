from tools.mazda_ti.plan_feedback_audit import cross_spectrum, lag_scan


def test_lag_scan_reports_response_delay_with_explicit_sign():
  source = [float((i * 37) % 97) for i in range(80)]
  response = [0.0] * 4 + source[:-4]
  result = lag_scan(source, response, 10)
  assert result["best"]["lag_ms"] == 200.0
  assert result["best"]["correlation"] > .99


def test_cross_spectrum_refuses_too_few_windows():
  result = cross_spectrum([0.0] * 100, [0.0] * 100)
  assert result["status"] == "insufficient_independent_windows"
