"""The real pinned compensation law must expose its flat region without injected clipping."""

import pytest

from .compensation_map import extract, measure, run
from .runtime import controller_source


ORIGINAL = 'ad14961a29f59635d5f657a43000ca2a1ade62fb'
HEADROOM_LINE = 'comp_mag = min(comp_mag, max(0.0, FRIC_COMP_KEEPOUT * u_max - abs(command)))'


def test_real_compensation_reports_flat_response_and_is_repeatable(tmp_path):
  first = run(ORIGINAL, tmp_path / 'first')
  assert first['status'] == 'failed_check'
  assert first['qualification'] == 'isolated_compensation_expression'
  assert any(gate['flat_intervals'] > 0 for gate in first['gates'])
  assert any(gate['flat_below_limit_intervals'] > 0 for gate in first['gates'])
  compensation, _ = extract(controller_source(ORIGINAL))
  assert 500 + compensation(500, 3) == pytest.approx(570)
  assert 530 + compensation(530, 3) == pytest.approx(570)
  report = (tmp_path / 'first/report.md').read_text()
  assert 'Physical-limit flat intervals' in report
  assert 'desirable duration or vehicle handling' in report
  assert first['expressions']['expression_source']
  assert run(ORIGINAL, tmp_path / 'second') == first
  assert (tmp_path / 'first/result.json').read_bytes() == (tmp_path / 'second/result.json').read_bytes()
  with pytest.raises(FileExistsError):
    run(ORIGINAL, tmp_path / 'first')


@pytest.mark.parametrize('width', [85, 170])
def test_expression_probe_distinguishes_responsive_taper_from_real_flat_mapping(width):
  source = controller_source(ORIGINAL)
  assert source.count(HEADROOM_LINE) == 1
  candidate = source.replace(HEADROOM_LINE, f'comp_mag *= min(1.0, max(0.0, FRIC_COMP_KEEPOUT * u_max - abs(command)) / {width}.0)')
  compensation, _ = extract(candidate)
  summary, failures = measure(compensation)
  assert failures == []
  assert all(gate['minimum_increment_counts'] > 0 for gate in summary)
  assert compensation(600, 3) == compensation(-600, -3) == compensation(0, 0) == 0


@pytest.mark.parametrize('old,new', [('comp_mag = min(FRIC_COMP_BASE', 'unexpected = min(FRIC_COMP_BASE'),
                                    ('FRIC_COMP_LOAD * abs(command)', 'unknown_dependency * abs(command)'),
                                    ('lat_friction_comp', 'renamed_feature')])
def test_changed_expression_shape_or_dependencies_are_rejected(old, new):
  with pytest.raises(ValueError, match='Unsupported compensation source'):
    extract(controller_source(ORIGINAL).replace(old, new))


def test_inserted_pre_withdrawal_modifier_is_rejected():
  source = controller_source(ORIGINAL)
  source = source.replace('fric_comp = self.friction_release.update(', 'fric_comp *= 2\n      fric_comp = self.friction_release.update(')
  with pytest.raises(ValueError, match='pre-withdrawal boundary changed'):
    extract(source)


def test_probe_rejects_nonfinite_and_overlimit_mapping():
  for compensation in (lambda command, gate: float('nan'), lambda command, gate: 601):
    _, failures = measure(compensation)
    assert len(failures) == 5
    assert all('out-of-envelope' in failure for failure in failures)


def test_physical_600_clip_is_distinct_from_artificial_below_limit_plateau():
  source = controller_source(ORIGINAL)
  old = 'FRIC_COMP_KEEPOUT = 0.95'
  assert source.count(old) == 1
  compensation, _ = extract(source.replace(old, 'FRIC_COMP_KEEPOUT = 1.0'))
  summaries, failures = measure(compensation)
  assert failures == []
  assert any(row['physical_limit_flat_intervals'] > 0 for row in summaries)
  assert all(row['flat_below_limit_intervals'] == 0 and row['reversed_intervals'] == 0 for row in summaries)
