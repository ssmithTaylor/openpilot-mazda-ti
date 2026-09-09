from types import SimpleNamespace as NS

from tools.mazda_ti.legacy_input_constraints import float32

from tools.mazda_ti import legacy_nnff_probe as m


class VM:
    def update_params(self, *args):
        pass

    def calc_curvature(self, *args):
        return -.004


def test_serialized_selection_keeps_equal_value_ambiguity_and_excludes_future():
    t = NS(actualLateralAccel=float32(1.6), desiredLateralAccel=float32(1.6), f=999., output=999.)
    c = NS(curvature=float32(.004), desiredCurvature=float32(.004), lateralControlState=NS(torqueState=t))
    cs = [NS(logMonoTime=stamp, carState=NS(vEgo=20., steeringAngleDeg=1.)) for stamp in [90, 100, 110]]
    lp = [NS(logMonoTime=80, liveParameters=NS(stiffnessFactor=1., steerRatio=20., angleOffsetDeg=0., roll=0.))]
    streams = dict(carState=cs, liveParameters=lp)
    times = {k: [e.logMonoTime for e in rows] for k, rows in streams.items()}
    early = m.select_serialized(c, 105, streams, times, VM(), 'earliest')
    late = m.select_serialized(c, 105, streams, times, VM(), 'latest')
    assert early[0].logMonoTime == 90 and late[0].logMonoTime == 100
    assert early[2] == late[2] == 2
    t.f = -999.; t.output = -999.
    assert late == m.select_serialized(c, 105, streams, times, VM(), 'latest')
    c.curvature = float32(.004+1e-9)
    assert m.select_serialized(c, 105, streams, times, VM(), 'latest') is None


def test_receipt_order_prevents_reusing_a_message_in_a_short_cycle():
    t = NS(actualLateralAccel=float32(1.6), desiredLateralAccel=float32(1.6))
    c = NS(curvature=float32(.004), desiredCurvature=float32(.004), startMonoTime=91,
           lateralControlState=NS(torqueState=t))
    streams = {'carState': [NS(logMonoTime=s, carState=NS(vEgo=20., steeringAngleDeg=1.)) for s in [90, 100]],
               'liveParameters': [NS(logMonoTime=80, liveParameters=NS(stiffnessFactor=1., steerRatio=20., angleOffsetDeg=0., roll=0.))]}
    times = {k: [e.logMonoTime for e in rows] for k, rows in streams.items()}
    previous = {'carState': 90, 'liveParameters': 80}
    chosen = m.select_serialized(c, 105, streams, times, VM(), 'earliest', previous, 20)
    assert chosen[0].logMonoTime == 100
    assert chosen[2] == 1
    previous['carState'] = 100
    assert m.select_serialized(c, 105, streams, times, VM(), 'earliest', previous, 20) is None
