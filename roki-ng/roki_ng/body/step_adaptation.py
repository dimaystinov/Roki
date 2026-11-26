"""Bounded stride reduction from IMU excursions over complete gait cycles."""


class StepAdaptation:
    def __init__(self, pitch, roll, limit):
        self.reference_pitch = pitch
        self.reference_roll = roll
        self.limit = limit
        self.peak = 0.0
        self.samples = 0
        self.scale = 1.0

    def observe(self, pitch, roll):
        self.peak = max(self.peak, abs(pitch-self.reference_pitch),
                        abs(roll-self.reference_roll))
        self.samples += 1

    def next_cycle(self):
        # Sparse polling must not authorize increasing the stride.
        if self.samples < 3:
            self.peak = 0.0
            self.samples = 0
            return self.scale
        target = max(.4, min(1.0, self.limit/max(self.limit, self.peak)))
        self.scale = min(target, self.scale+.05)
        self.peak = 0.0
        self.samples = 0
        return self.scale
