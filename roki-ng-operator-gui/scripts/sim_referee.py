"""Junior Humanoid Soccer §1.5 and §§1.15–1.17, applied at physics ticks."""
import math

import mujoco
import numpy as np


# Overall frame height 1 m, with a 50 mm crossbar.
GOAL_CLEAR_HEIGHT = .95


def painted_limits(layout):
    """Half dimensions at the outer paint edges of the 1600 x 1200 turf texture."""
    return (layout['field_length_m'] * 1.70/4.0, layout['field_width_m'] * 1.20/3.0)


class PitchReferee:
    def __init__(self, model, data, layout, players):
        self.model, self.data = model, data
        self.ball = data.joint("soccer_ball_joint")
        self.radius = model.geom("ball").size[0]
        self.ball_id = model.geom("ball").id
        # Outer edges of the painted rectangle in the scene's 1600 x 1200 turf texture.
        self.limits = painted_limits(layout)
        self.goals = {colour: layout[colour+'_goal_x_m'] for colour in ('blue','yellow')}
        self.teams = {model.body(name).id: colour for name, colour in players.items()}
        self.geom_team = {}
        for gid, body in enumerate(model.geom_bodyid):
            while body and body not in self.teams:
                body = model.body_parentid[body]
            if body:
                self.geom_team[gid] = self.teams[body]
        self.reset()

    def reset(self):
        self.last_team = None
        self.goal = None
        self.pending = None

    def neutral_reentry(self, returning):
        """§1.11: farthest unoccupied neutral point at the time of return."""
        points = [(x, y) for x in (self.goals['blue']+.90, 0., self.goals['yellow']-.90)
                  for y in (-.65, .65)]
        others = [self.data.xpos[b,:2] for b in self.teams if b != returning]
        free = [point for point in points if all(math.dist(point, other) >= .36 for other in others)
                and math.dist(point, self.ball.qpos[:2]) >= .18+self.radius]
        return max(free, key=lambda point: math.dist(point, self.ball.qpos[:2]), default=None)

    def touch(self, geom):
        if geom in self.geom_team:
            self.last_team = self.geom_team[geom]

    def update(self, previous):
        if self.goal:
            return None
        x, y, z = self.ball.qpos[:3]
        if self.pending is None:
            crossings = []
            for axis, value in enumerate((x, y)):
                limit = self.limits[axis] + self.radius
                if abs(value) <= limit:
                    continue
                delta = value-previous[axis]
                t = (math.copysign(limit,value)-previous[axis])/delta if delta else 0.
                crossings.append((max(0., min(1., t)), axis))
            if not crossings:
                return None
            fraction, axis = min(crossings)
            px, py, pz = (a + fraction*(b-a) for a,b in zip(previous,(x,y,z)))
            end = min(self.goals, key=lambda colour: abs(px-self.goals[colour]))
            if axis == 0 and abs(py)+self.radius < .5 and pz+self.radius < GOAL_CLEAR_HEIGHT:
                self.goal = 'goal_'+end
                return self.goal, [px,py,pz]
            # H = 65 cm, G = 90 cm from the goal line (rules diagram, p. 11).
            spot_x = 0.
            if self.last_team == end and px*self.goals[end] > 0:
                spot_x = self.goals[end]-math.copysign(.90,self.goals[end])
            self.pending = ([px,py,pz], spot_x, 1 if py>=0 else -1, self.last_team)
        crossing, spot_x, side, team = self.pending
        limit_y = self.limits[1]-self.radius
        spot_y = min(.65, limit_y)
        # Robot centres include the keeper and delayed visitor before kick-off.
        while any(math.hypot(self.data.xpos[b,0]-spot_x, self.data.xpos[b,1]-side*spot_y)
                  < .18+self.radius for b in self.teams):
            if spot_y >= limit_y:
                return None
            spot_y = min(spot_y+.025,limit_y)
        spot = [float(spot_x),float(side*spot_y),float(self.radius)]
        self.ball.qpos[:] = [*spot,1.,0.,0.,0.]
        self.ball.qvel[:] = 0.
        self.data.qacc_warmstart[self.model.joint("soccer_ball_joint").dofadr.item():self.model.joint("soccer_ball_joint").dofadr.item()+6] = 0.
        mujoco.mj_forward(self.model,self.data)
        self.pending = self.last_team = None
        return 'throw_in', dict(exit=crossing, position=spot, last_defended=team)

    def standalone_tick(self, previous):
        contacts = self.data.contact
        for index in np.flatnonzero((contacts.geom1==self.ball_id) | (contacts.geom2==self.ball_id)):
            contact = contacts[index]
            self.touch(contact.geom2 if contact.geom1==self.ball_id else contact.geom1)
        return self.update(previous)
