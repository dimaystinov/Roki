"""Short-lived heading hints for gaze only, never observations for kicking."""
import math


class ExpectedGaze:
    def __init__(self):
        self.targets = {}
        self.ball_sample = None
        self.velocity = (0., 0.)

    def forget_velocity(self):
        self.ball_sample = None
        self.velocity = (0., 0.)

    def ball_region(self, yaw, epoch, now, lead=0.):
        """Last visual position and a bounded rolling prediction in the current heading."""
        if self.ball_sample is None:
            return None
        x, y, stamp, saved_epoch = self.ball_sample
        age = now-stamp
        if epoch != saved_epoch or not 0 <= age <= 2:
            return None
        horizon = min(.6, age+lead)
        x += self.velocity[0]*horizon
        y += self.velocity[1]*horizon
        c, s = math.cos(yaw), math.sin(yaw)
        return x*c+y*s, y*c-x*s

    def remember(self, name, point, *, yaw, epoch, now):
        x, y = point
        if not all(math.isfinite(v) for v in (x, y, yaw, now)) or math.hypot(x, y) < .01:
            return
        if name == 'ball':
            c, s = math.cos(yaw), math.sin(yaw)
            px, py = x*c-y*s, x*s+y*c
            if self.ball_sample is not None:
                ox, oy, stamp, old_epoch = self.ball_sample
                dt = now-stamp
                if .02 <= dt <= .6 and epoch == old_epoch and math.hypot(px-ox,py-oy) <= 2.5*dt:
                    self.velocity = tuple(.5*(v+d/dt) for v,d in zip(self.velocity,(px-ox,py-oy)))
                elif dt != 0:
                    self.velocity = (0.,0.)
            self.ball_sample = px, py, now, epoch
        self.targets[name] = (math.atan2(y, x) + yaw, epoch, now)

    def pan(self, name, *, yaw, epoch, now, sign):
        target = self.targets.get(name)
        if target is None or sign not in (-1, 1) or not math.isfinite(yaw):
            return None
        bearing, saved_epoch, stamp = target
        lifetime = {'ball': 6, 'start_goal': math.inf}.get(name, 15)
        if epoch != saved_epoch or not 0 <= now - stamp < lifetime:
            return None
        if name == 'ball':
            region = self.ball_region(yaw, epoch, now)
            if region:
                bearing = yaw+math.atan2(region[1],region[0])
        relative = (bearing - yaw + math.pi) % (2 * math.pi) - math.pi
        ticks = round(relative / (sign * .00058909))
        return ticks if abs(ticks) <= 2666 else None
