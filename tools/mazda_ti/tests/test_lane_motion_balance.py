import math
import pytest

from tools.mazda_ti.lane_motion_balance import instantaneous, integrate_observations


def test_offset_circle_equilibrium_needs_faster_road_tangent_rate():
  v,k,e = 20.,.004,.5
  result = instantaneous(speed=v,offset=e,heading=0.,road_curvature=k,yaw_rate=k*v/(1-k*e))
  assert result['offset_rate_mps'] == 0
  assert result['relative_heading_rate_rps'] == 0


@pytest.mark.parametrize('sign', [-1,1])
def test_straight_road_constant_heading_closes_offset_exactly(sign):
  psi = sign*.02
  rows = [dict(available=True,mono_ns=int((1+t)*1e9),speed=20.,offset=20*math.sin(psi)*t,
               heading=psi,road_curvature=0.,yaw_rate=0.,yaw_std_rps=.01) for t in (0.,.5,1.)]
  r = integrate_observations(rows,maximum_gap_s=.5)
  assert abs(r['offset_closure_residual_m']) < 1e-12
  assert r['heading_closure_residual_rad'] == 0
  assert r['coherent_reported_yaw_std_sensitivity_rad'] == .01


@pytest.mark.parametrize('change', [{'available':False}, {'mono_ns':1000000000},
                                  {'mono_ns':2000000000}, {'yaw_std_rps':float('nan')}])
def test_missing_and_bad_time_or_uncertainty_are_not_silently_bridged(change):
  a = dict(available=True,mono_ns=1000000000,speed=20.,offset=0.,heading=0.,road_curvature=0.,yaw_rate=0.,yaw_std_rps=.01)
  b = dict(a,mono_ns=1500000000)
  b.update(change)
  with pytest.raises(ValueError):
    integrate_observations([a,b],maximum_gap_s=.5)


def test_singular_lane_coordinates_are_rejected():
  with pytest.raises(ValueError):
    instantaneous(speed=20.,offset=2.,heading=0.,road_curvature=.5,yaw_rate=.1)
