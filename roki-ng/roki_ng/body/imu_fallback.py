"""Select a calibrated head reference without hiding primary sensor diagnostics."""
import math
from collections import deque
import time
from types import SimpleNamespace
from roki_ng.wire import Fault
from .body_imu import BodyImu
from .head_body_orientation import body_from_head, multiply, inverse, rotation


def separation(a, b):
    return math.degrees(2 * math.acos(min(1., abs(sum(x*y for x,y in zip(a,b))))))


class FallbackImu(BodyImu):
    def __init__(self, calibration):
        super().__init__()
        self.primary = BodyImu()
        self.calibration = calibration
        self.active_source = 'none'
        self.aligned = False
        self.head_available = False
        self.alignment = (0,0,0,1)
        self.align_count = 0
        self.previous_raw = None
        self.raw_changed_at = 0
        self.recovered_at = None
        self.head_stamp = None
        self.head_advanced_at = 0.
        self.stationary_head = deque()
        self.local_heading = False
        self.reason = 'Waiting for paired sensors'
        self.last_alignment_pair = None
        self.legacy_failed_value = None
        self.neck_samples = deque(maxlen=24)
        self.neck_next_at = 0.
        self.head_reason = 'Waiting for measured neck angles'
        self.head_prediction = None
        self.head_error_deg = None

    def _measured_neck(self, hardware, head, now, can_read):
        # Zubr units are 1.536 times Kondo units; tilt is mirrored in the body map.
        if can_read and now >= self.neck_next_at:
            self.neck_next_at = now + .02
            before = time.monotonic()
            try:
                positions = hardware.body_positions()
                pan, tilt = positions[0]/1.536, -positions[25]/1.536
                if not all(math.isfinite(x) for x in (pan, tilt)):
                    raise ValueError('Invalid neck feedback')
                if not (-2800 <= pan <= 2800 and -2750 <= tilt <= 1100):
                    raise ValueError('Neck feedback outside joint limits')
                self.neck_samples.append(((before+time.monotonic())/2, pan, tilt))
            except (Fault, OSError, ValueError, IndexError, TypeError):
                self.neck_samples.clear()
        now = time.monotonic()
        if not self.neck_samples or now-self.neck_samples[-1][0] > .08:
            raise ValueError('Neck feedback missing or stale')
        stamp = head['source_mono_ns']/1e9
        closest = min(self.neck_samples, key=lambda x:abs(x[0]-stamp))
        if abs(closest[0]-stamp) > .04:
            raise ValueError('Head and neck samples are not synchronized')
        for left, right in zip(self.neck_samples, list(self.neck_samples)[1:]):
            if left[0] <= stamp <= right[0] and right[0] > left[0]:
                f = (stamp-left[0])/(right[0]-left[0])
                return {'pan':left[1]+f*(right[1]-left[1]),
                        'tilt':left[2]+f*(right[2]-left[2])}
        return {'pan':closest[1], 'tilt':closest[2]}

    def tick(self, hardware, head, neck, settled_at, moving_job=False,
             force_primary_stale=False, primary_ready=True):
        now = time.monotonic()
        if primary_ready and now >= self.primary.next_at:
            try:
                self.primary.poll(hardware)
            except (Fault, OSError):
                pass
        now = time.monotonic()  # UART polling can block; evaluate after receive.
        raw = self.primary
        if raw.quaternion != self.previous_raw:
            self.previous_raw = raw.quaternion
            self.raw_changed_at = now
        head_q = None
        self.head_reason = 'Head IMU missing or stale'
        try:
            stamp = head['sensor_timestamp_ns']
            if self.head_stamp is not None and stamp < self.head_stamp:
                self.aligned = False
                self.align_count = 0
                self.local_heading = False
                self.stationary_head.clear()
            if stamp != self.head_stamp:
                self.head_advanced_at = now
            self.head_stamp = stamp
            age = now - head['source_mono_ns']/1e9
            norm = sum(v*v for v in head['quaternion_xyzw'])
            if not .9 < norm < 1.1:
                raise ValueError('Invalid head quaternion norm')
            if 0 <= age < self.MAX_AGE_S and now-self.head_advanced_at < self.MAX_AGE_S:
                neck = self._measured_neck(hardware, head, now, primary_ready)
                head_q = body_from_head(head['quaternion_xyzw'],
                    self.calibration['pan_sign'] * neck['pan']/1698,
                    self.calibration['tilt_sign'] * neck['tilt']/1698,
                    self.calibration['mounting_xyzw'], self.alignment)
        except (KeyError, TypeError, ValueError) as exc:
            self.head_reason = str(exc)
            head_q = None
        # World yaw is session-specific; establish it only at rest with live body data.
        recent = [s for s in self.neck_samples if now-s[0] < .3]
        neck_stationary = (len(recent) >= 5 and recent[-1][0]-recent[0][0] >= .15
                           and all(max(s[i] for s in recent)-min(s[i] for s in recent) < 20
                                   for i in (1, 2)))
        pair = (raw.sequence, self.head_stamp)
        if (not self.aligned and head_q is not None and raw.fresh(now) and not moving_job
                and not force_primary_stale and neck_stationary
                and (self.last_alignment_pair is None or all(a != b for a,b in zip(pair,self.last_alignment_pair)))):
            self.last_alignment_pair = pair
            delta = multiply(raw.quaternion, inverse(head_q))
            yaw = 2 * math.atan2(delta[2], delta[3])
            self.alignment = multiply(rotation((0,0,1), yaw), self.alignment)
            head_q = multiply(rotation((0,0,1), yaw), head_q)
            if separation(raw.quaternion, head_q) < 3:
                self.align_count += 1
            else:
                self.align_count = 0
            self.aligned = self.align_count >= 25
        # A stopped Zubr cannot supply a world yaw at startup. Gravity from
        # the calibrated head still defines body pitch and roll; use local yaw.
        stalled = raw.sensor_sequence is not None and raw.error == 'IMU measurement counter stopped'
        if not self.aligned and stalled and head_q is not None and neck_stationary and not moving_job:
            samples = self.stationary_head
            if samples and (now-samples[-1][0] > self.MAX_AGE_S or
                            separation(samples[0][2], head_q) > 2):
                samples.clear()
            if not samples or samples[-1][1] != self.head_stamp:
                samples.append((now, self.head_stamp, head_q))
            if len(samples) >= 15 and samples[-1][0]-samples[0][0] >= .6:
                self.aligned = self.local_heading = True
        else:
            self.stationary_head.clear()
        self.head_prediction = head_q
        self.head_error_deg = (separation(raw.quaternion, head_q)
                               if head_q is not None and raw.fresh(now) else None)
        if head_q is not None:
            self.head_reason = ('Ready (local yaw)' if self.local_heading else 'Ready' if self.aligned
                                else 'Waiting for stationary alignment')
        head_ok = self.aligned and head_q is not None
        self.head_available = head_ok
        legacy_stuck = (raw.sensor_sequence is None and raw.quaternion is not None
                        and now-self.raw_changed_at > 1 and head_ok
                        and separation(raw.quaternion, head_q) > 4)
        if legacy_stuck:
            self.legacy_failed_value = raw.quaternion
        elif raw.quaternion != self.legacy_failed_value:
            self.legacy_failed_value = None
        legacy_stuck = legacy_stuck or self.legacy_failed_value is not None
        body_ok = raw.fresh(now) and not legacy_stuck and not force_primary_stale
        source = 'body' if body_ok else 'head' if head_ok else 'none'
        if (self.active_source == 'head' or self.local_heading) and body_ok and head_ok:
            candidate = head_q
            if self.local_heading:
                delta = multiply(raw.quaternion, inverse(head_q))
                correction = rotation((0,0,1), 2*math.atan2(delta[2], delta[3]))
                candidate = multiply(correction, head_q)
            if separation(raw.quaternion, candidate) < 3:
                if self.recovered_at is None:
                    self.recovered_at = now
            else:
                self.recovered_at = None
            if moving_job or self.recovered_at is None or now-self.recovered_at < 1:
                source = 'head'
            elif self.local_heading:
                self.alignment = multiply(correction, self.alignment)
                self.local_heading = False
                self.yaw = None  # Start a new yaw epoch for the body world frame.
        elif not body_ok:
            self.recovered_at = None
        if source == 'body' and self.local_heading and not head_ok:
            self.aligned = self.local_heading = False
            self.align_count = 0
            self.yaw = None
        self.active_source = source
        self.reason = ('Body unchanged while head detects movement' if legacy_stuck else
                       'Body unavailable' if not body_ok else 'Body healthy')
        if source == 'none':
            self._selected_stamp = None
            self.invalidate('No trustworthy IMU: ' + self.reason)
            return
        q = raw.quaternion if source == 'body' else head_q
        seq = raw.sequence if source == 'body' else head['sensor_timestamp_ns']
        stamp = raw.received_at if source == 'body' else head['source_mono_ns']/1e9
        # Do not refresh an effective sample simply because selection ran again.
        old = getattr(self, '_selected_stamp', None)
        token = (source, seq)
        if old != token:
            super().poll(SimpleNamespace(body_quaternion=lambda:q))
            self.received_at = stamp
            self.sensor_sequence = raw.sensor_sequence if source == 'body' else seq
            self.sensor_ticks = raw.sensor_ticks if source == 'body' else seq
            self._selected_stamp = token

    def state(self, now):
        value = super().state(now)
        value.update(active_source=self.active_source, fallback_reason=self.reason,
                     head_fallback_ready=self.head_available, primary=self.primary.state(now),
                     head_fallback_reason=self.head_reason,
                     head_body_quaternion_xyzw=self.head_prediction,
                     head_body_error_deg=self.head_error_deg)
        return value
