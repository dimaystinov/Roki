"""Empirical extra impulse from the loaded striking foot of the simulated robot."""
import math

import numpy as np
import mujoco


CARPET_MU = .079218


class KickContact:
    # The CAD sole is rigid. Calibrate the missing horizontal impulse against
    # rollout from real kicks, while preserving the simulated contact direction.
    extra_impulse_ratio = .95

    def __init__(self, world, leg, soft=False):
        self.soft = soft
        self.foot = world.model.joint(f"{leg}_ankle_roll_joint").bodyid.item()
        self.impulse = np.zeros(2)
        self.force = np.zeros(6)
        self.completed = False

    def sample(self, world):
        if self.completed:
            return
        model, data = world.model, world.data
        ball = world.ball_geom
        touching = False
        for i, contact in enumerate(data.contact):
            if ball not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if ball == contact.geom1 else contact.geom1
            if model.geom_bodyid[other] != self.foot or contact.efc_address < 0:
                continue
            mujoco.mj_contactForce(model, data, i, self.force)
            force = contact.frame.reshape(3, 3).T @ self.force[:3]
            sign = 1 if ball == contact.geom2 else -1
            self.impulse += sign * force[:2] * model.opt.timestep
            touching = True
        if touching or not self.impulse.any():
            return
        self.completed = True
        body = model.geom_bodyid[ball]
        delta = self.extra_impulse_ratio * self.impulse / model.body_mass[body]
        v = world.ball_qvel
        if np.dot(data.qvel[v:v+2], delta) <= 0:
            return
        # A trapped or grazing ball can load the rigid sole without receiving
        # the full elastic impulse. Limit the extra contribution to the
        # simulated release speed calibrated from the two measured rollouts, keeping weaker contacts weaker.
        max_speed = .92 if self.soft else 2.40
        velocity = data.qvel[v:v+2]
        if np.linalg.norm(velocity + delta) > max_speed:
            speed_sq = velocity @ velocity
            if speed_sq >= max_speed**2:
                return
            cross = velocity @ delta
            delta_sq = delta @ delta
            delta *= (max_speed**2 - speed_sq) / (math.sqrt(cross**2 + delta_sq*(max_speed**2-speed_sq)) + cross)
        data.qvel[v:v+2] += delta
        # Additional rolling spin about the horizontal axis, in free-joint coordinates.
        spin = np.array([-delta[1], delta[0], 0.]) / model.geom_size[ball, 0]
        data.qvel[v+3:v+6] += data.xmat[body].reshape(3, 3).T @ spin
