from types import SimpleNamespace
import math
import pytest
from roki_ng.body.body_imu import BodyImu
from roki_ng.body.runtime import RokiHardware
import struct


def test_counter_progress_keeps_stationary_sensor_fresh_and_stall_expires(monkeypatch):
    clock=[1.0];monkeypatch.setattr('time.monotonic',lambda:clock[0])
    h=SimpleNamespace(body_quaternion=lambda:(math.sqrt(.5),0,0,math.sqrt(.5)),
                      imu_measurement_sequence=7,imu_sensor_ticks=100)
    imu=BodyImu();imu.poll(h)
    clock[0]=1.1;h.imu_measurement_sequence=8;h.imu_sensor_ticks=101;imu.poll(h)
    assert imu.fresh(clock[0])
    clock[0]=1.4;imu.poll(h)
    assert not imu.fresh(clock[0])
    assert imu.state(clock[0])['measurement_stalled']
    h.imu_measurement_sequence=9;h.imu_sensor_ticks=102;imu.poll(h)
    assert imu.fresh(clock[0]) and not imu.state(clock[0])['measurement_stalled']


def test_decode_counter_and_quaternion_in_one_transaction():
    h=object.__new__(RokiHardware);calls=[]
    def read(addr,size):
        calls.append((addr,size))
        return struct.pack('<4sII4hhH',b'IMU1',0xffffffff,123,11585,0,0,11585,0,1)
    h._read_ram=read
    q=h.body_quaternion()
    assert q[0]==11585/16384 and h.imu_measurement_sequence==0xffffffff
    assert calls==[(0x150,24)]
