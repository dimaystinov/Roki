"""Experimental camera-guided policy for MuJoCo trials."""
import math

from roki_ng.body.calibration import wrap
from roki_ng.supervisor.game.simple_football import SimpleFootball


class VisualDribble(SimpleFootball):
    """Experimental simulator policy. Uses camera observations and estimated pose."""
    async def step(self):
        yaw, _ = await self.g._imu()
        if self.kickoff_heading is None:
            self.kickoff_heading = yaw
        for tilt in (-1600, -2100, -1100):
            scene = await self.look(0, tilt, wait_ball=True)
            if scene and scene.get("ball"):
                break
        else:
            self.describe("search", "reacquire ball")
            return
        x, y = scene["ball"]
        own = self.p["match.own_goal"]
        target = self.p[f"field.goal.{1-own}"]
        goal_bearing = math.atan2(target["y"]-self.g.odometry.y, target["x"]-self.g.odometry.x)
        aim = wrap(goal_bearing-yaw)
        offset = (.045 if self.p["game.simple_football.kickoff_side"] == "left" else -.045)
        self.describe("visual_walk", "advance with camera feedback")
        if x < .18 and abs(y-offset) < .045:
            # Move the ball with a short walking touch along the planned lane.
            course = aim + (.33 if offset > 0 else -.33) if self.opening_touch_required else aim
            await self.g._move("game.walk", dict(kind="omni", distance_m=.16, course=course, heading=0.))
            self.opening_touch_required = False
        else:
            dx, dy = max(.015,x-.12), y-offset
            await self.g._move("game.walk", dict(kind="omni", distance_m=min(.20,math.hypot(dx,dy)),
                course=math.atan2(dy,dx), heading=0.))
