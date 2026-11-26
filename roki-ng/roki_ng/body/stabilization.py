"""Bounded stationary-crouch pitch experiment; no integral or automatic recovery."""

import math


class CrouchStabilizer:
    def __init__(self):
        self.offset_deg = 0.0
        self.filtered_pitch = None
        self.previous_walk_pitch = None
        self.last_at = None
        self.last_sequence = None
        self.reason = "disabled"
        self.saturated = False
        self.updates = 0
        self.target_deg = 0.0

    def freeze(self, reason):
        self.reason = reason
        if reason != "queue_busy":
            self.last_at = None
            self.previous_walk_pitch = None
            self.filtered_pitch = None

    def reset(self, reason):
        self.offset_deg = 0.0
        self.target_deg = 0.0
        self.saturated = False
        self.last_sequence = None
        self.freeze(reason)

    def propose(self, imu, parameters, now, *, walking=False):
        if not imu.fresh(now):
            self.freeze("imu_stale")
            return None
        if max(abs(imu.pitch), abs(imu.roll)) > 15:
            self.freeze("tilt_outside_range")
            return None
        if self.last_sequence == imu.sequence:
            return None
        self.last_sequence = imu.sequence
        elapsed = max(0., now-self.last_at) if self.last_at is not None else .02
        dt = min(.05, elapsed)  # Bound a single servo update even after a gap.
        self.last_at = now
        pitch = imu.pitch
        if walking:
            # An alternating support-leg signal cancels in a pair; a sustained
            # change of lean reaches the regulator without a long phase delay.
            previous = self.previous_walk_pitch
            self.previous_walk_pitch = pitch
            if previous is not None and elapsed <= imu.MAX_AGE_S:
                pitch = (pitch + previous) / 2
        else:
            self.previous_walk_pitch = None
        if self.filtered_pitch is None:
            self.filtered_pitch = 0.0 if walking else pitch
        else:
            filter_dt = min(imu.MAX_AGE_S, elapsed) if walking else dt
            alpha = 1 - math.exp(-filter_dt / (.12 if walking else .1))
            self.filtered_pitch += alpha * (pitch - self.filtered_pitch)
        error = parameters["stabilization.pitch_target_deg"] - self.filtered_pitch
        deadband = parameters["stabilization.pitch_deadband_deg"]
        error = math.copysign(max(0, abs(error) - deadband), error)
        demand = parameters["stabilization.pitch_kp"] * error
        limit = parameters["stabilization.max_correction_deg"]
        self.target_deg = max(-limit, min(limit, demand))
        self.saturated = abs(demand) > limit
        step = parameters["stabilization.slew_deg_s"] * dt
        self.reason = "regulating"
        return self.offset_deg + max(-step, min(step, self.target_deg - self.offset_deg))

    def commit(self, value):
        # Only an accepted servo command changes the applied correction.
        self.offset_deg = value
        self.updates += 1

    @staticmethod
    def ticks(degrees):
        value = round(math.radians(degrees) * 1698)
        return {(7, 1): value, (7, 2): -value}

    def state(self, parameters):
        return {"enabled": parameters["stabilization.enabled"], "scope": ("stand_walk_pitch" if parameters.get("stabilization.stand_walk") else "stationary_crouch_pitch"),
                "reason": self.reason, "offset_deg": self.offset_deg,
                "filtered_pitch_deg": self.filtered_pitch, "target_correction_deg": self.target_deg,
                "saturated": self.saturated, "updates": self.updates}
