import math
from types import SimpleNamespace
from roki_ng.body.imu_fallback import FallbackImu

Q=(math.sqrt(.5),0,0,math.sqrt(.5))
CAL={'mounting_xyzw':[0,0,0,1], 'pan_sign':1, 'tilt_sign':-1}

def test_live_head_takes_over_and_both_missing_invalidates(monkeypatch):
    clock=[10.0];monkeypatch.setattr('time.monotonic',lambda:clock[0])
    h=SimpleNamespace(body_quaternion=lambda:Q,body_positions=lambda:[0]*30,imu_measurement_sequence=1,imu_sensor_ticks=1)
    imu=FallbackImu(CAL)
    for i in range(40):
        clock[0]+=.02;h.imu_measurement_sequence+=1;h.imu_sensor_ticks+=1
        head={'quaternion_xyzw':Q,'source_mono_ns':int(clock[0]*1e9),'sensor_timestamp_ns':i+1,'sequence':i}
        imu.tick(h,head,{'pan':0,'tilt':0},0,False,False,True)
    assert imu.active_source=='body' and imu.aligned
    clock[0]+=.02;head.update(source_mono_ns=int(clock[0]*1e9),sensor_timestamp_ns=100,sequence=100)
    imu.tick(h,head,{'pan':0,'tilt':0},0,True,True,True)
    assert imu.active_source=='head' and imu.fresh(clock[0])
    clock[0]+=.3
    imu.tick(h,None,{'pan':0,'tilt':0},0,True,True,True)
    assert not imu.fresh(clock[0])


def test_uncalibrated_or_moving_neck_cannot_be_used(monkeypatch):
    clock=[10.0];monkeypatch.setattr('time.monotonic',lambda:clock[0])
    h=SimpleNamespace(body_quaternion=lambda:Q,body_positions=lambda:[0]*30)
    imu=FallbackImu(CAL)
    head={'quaternion_xyzw':Q,'source_mono_ns':int(clock[0]*1e9),'sensor_timestamp_ns':1,'sequence':1}
    imu.tick(h,head,{'pan':0,'tilt':0},clock[0]+1,True,True,True)
    assert imu.active_source=='none' and not imu.fresh(clock[0])


def test_counter_stall_switches_and_recovery_waits_for_rest(monkeypatch):
    clock=[10.0];monkeypatch.setattr('time.monotonic',lambda:clock[0])
    h=SimpleNamespace(body_quaternion=lambda:Q,body_positions=lambda:[0]*30,imu_measurement_sequence=0,imu_sensor_ticks=0)
    imu=FallbackImu(CAL)
    def tick(advance_body=True, moving=False):
        clock[0]+=.021
        if advance_body:h.imu_measurement_sequence+=1;h.imu_sensor_ticks+=1
        head={'quaternion_xyzw':Q,'source_mono_ns':int(clock[0]*1e9),
              'sensor_timestamp_ns':int(clock[0]*1e9)}
        imu.tick(h,head,{'pan':0,'tilt':0},0,moving)
    for _ in range(35):tick()
    for _ in range(12):tick(False,True)
    assert imu.active_source=='head' and imu.fresh(clock[0])
    for _ in range(70):tick(True,True)
    assert imu.active_source=='head'
    tick(True,False)
    assert imu.active_source=='body'


def test_receive_timestamp_after_blocking_read_is_not_future(monkeypatch):
    clock=[10.0];monkeypatch.setattr('time.monotonic',lambda:clock[0])
    def read():
        clock[0]+=.004
        return Q
    imu=FallbackImu(CAL)
    imu.tick(SimpleNamespace(body_quaternion=read,body_positions=lambda:[0]*30),None,{'pan':0,'tilt':0},float('inf'))
    assert imu.active_source=='body' and imu.fresh(clock[0])


def test_head_reference_does_not_block_measured_neck_motion(tmp_path):
    from roki_ng.body.runtime import Body
    from roki_ng.parameters import Parameters
    body=Body({'simulate':True,'parameters':Parameters(tmp_path).values},lambda *a:None,lambda *a:None)
    body.body_imu=FallbackImu(CAL)
    body.body_imu.active_source='head'
    body.head={'pan':100,'tilt':-600}
    plan=list(body._head(0,0))
    assert plan[0][0]=='servo' and plan[1][0]=='drain'
    assert body.head=={'pan':0,'tilt':0}


def test_measured_neck_tracks_motion_even_when_target_is_not_reached(monkeypatch):
    from roki_ng.body.head_body_orientation import multiply, rotation
    clock=[10.];monkeypatch.setattr('time.monotonic',lambda:clock[0])
    positions=[0]*30
    h=SimpleNamespace(body_quaternion=lambda:Q,body_positions=lambda:positions,
                      imu_measurement_sequence=0,imu_sensor_ticks=0)
    imu=FallbackImu(CAL)
    for i in range(40):
        clock[0]+=.025;h.imu_measurement_sequence+=1
        head={'quaternion_xyzw':Q,'source_mono_ns':int(clock[0]*1e9),'sensor_timestamp_ns':i+1}
        imu.tick(h,head,{'pan':0,'tilt':0},0)
    assert imu.head_available
    for i in range(1,30):
        clock[0]+=.025
        positions[0]=i*50;positions[25]=i*65
        neck=multiply(rotation((0,1,0),positions[0]/(1698*1.536)),
                      rotation((1,0,0),positions[25]/(1698*1.536)))
        head={'quaternion_xyzw':multiply(Q,neck),'source_mono_ns':int(clock[0]*1e9),
              'sensor_timestamp_ns':100+i}
        imu.tick(h,head,{'pan':2400,'tilt':-2400},float('inf'),True,True)
        assert imu.active_source=='head'
        assert abs(sum(a*b for a,b in zip(imu.quaternion,Q)))>0.9999
    clock[0]+=.15
    imu.tick(h,head,{'pan':2400,'tilt':-2400},0,True,True,False)
    assert not imu.head_available


def test_stationary_measured_neck_can_align_without_prior_command(monkeypatch):
    clock=[10.];monkeypatch.setattr('time.monotonic',lambda:clock[0])
    h=SimpleNamespace(body_quaternion=lambda:Q,body_positions=lambda:[0]*30,
                      imu_measurement_sequence=0)
    imu=FallbackImu(CAL)
    for i in range(50):
        clock[0]+=.025;h.imu_measurement_sequence+=1
        head={'quaternion_xyzw':Q,'source_mono_ns':int(clock[0]*1e9),'sensor_timestamp_ns':i+1}
        imu.tick(h,head,{'pan':2000,'tilt':-2000},float('inf'))
    assert imu.head_available


def test_neck_feedback_failure_invalidates_head_without_hiding_body(monkeypatch):
    clock=[10.];monkeypatch.setattr('time.monotonic',lambda:clock[0])
    def broken():raise OSError('servo read failed')
    h=SimpleNamespace(body_quaternion=lambda:Q,body_positions=broken,imu_measurement_sequence=1)
    imu=FallbackImu(CAL);imu.aligned=True
    head={'quaternion_xyzw':Q,'source_mono_ns':int(clock[0]*1e9),'sensor_timestamp_ns':1}
    imu.tick(h,head,{'pan':0,'tilt':0},0)
    assert imu.active_source=='body' and not imu.head_available
    clock[0]+=.025;head.update(source_mono_ns=int(clock[0]*1e9),sensor_timestamp_ns=2)
    imu.tick(h,head,{'pan':0,'tilt':0},0,False,True)
    assert imu.active_source=='none' and not imu.fresh(clock[0])


def test_start_with_frozen_body_uses_live_stationary_head_and_rejects_frozen_head(monkeypatch):
    clock = [10.]
    monkeypatch.setattr('time.monotonic', lambda: clock[0])
    h = SimpleNamespace(body_quaternion=lambda: Q, body_positions=lambda: [0]*30,
                        imu_measurement_sequence=1, imu_sensor_ticks=1)
    imu = FallbackImu(CAL)
    def tick(stamp, moving=False):
        clock[0] += .025
        imu.tick(h, {'quaternion_xyzw': Q, 'source_mono_ns': int(clock[0]*1e9),
                     'sensor_timestamp_ns': stamp}, {}, 0, moving)
    for i in range(50):
        tick(i, moving=True)
    assert imu.active_source == 'none'
    for i in range(50, 100):
        tick(i)
    assert imu.active_source == 'head' and imu.local_heading
    for i in range(10):
        tick(99)
    assert imu.active_source == 'none' and not imu.fresh(clock[0])
