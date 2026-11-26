"""Physical Roki2 servo addresses and production motion recipes for MuJoCo."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

from roki_ng.body.motion.backends.roki2 import backend
from roki_ng.body.motion.backends.roki2.model import Robot
from roki_ng.body.motion.engine import Engine
from roki_ng.body.motion import slots
from roki_ng.body.motion import legacy_game_motion as legacy



JOINTS = {
    10: "ankle_roll", 9: "ankle_pitch", 8: "upper_knee",
    13: "lower_knee", 7: "hip_pitch", 6: "hip_roll", 5: "hip_yaw",
    4: "elbow_pitch", 3: "elbow_yaw", 2: "shoulder_roll",
    1: "shoulder_pitch", 11: "wrist_yaw", 14: "claw",
}

# Roki bus coordinates and the imported CAD model use different positive axes.
# These directions are checked against the production crouch trajectory.
REVERSED_MODEL_AXES = frozenset({
    "left_hip_yaw", "right_hip_yaw", "left_ankle_roll", "right_ankle_roll",
    "right_ankle_pitch", "right_upper_knee", "right_lower_knee",
    "left_hip_pitch", "right_elbow_pitch", "right_shoulder_pitch",
})


class ServoBridge:
    def __init__(self, model, data):
        self.model, self.data = model, data
        self.addresses = {}
        physical = Robot()
        for servo, bus, sign, *_ in physical.ACTIVESERVOS:
            if servo == 0:
                name = "head_yaw" if bus == 1 else "waist_yaw"
            elif servo == 12:
                name = "head_pitch"
            else:
                name = f"{'right' if bus == 1 else 'left'}_{JOINTS[servo]}"
            direction = -sign if name in REVERSED_MODEL_AXES else sign
            self.addresses[servo, bus] = (model.actuator(name).id, direction)
        for bus in (1, 2):
            name = f"{'right' if bus == 1 else 'left'}_lower_knee"
            sign = next(row[2] for row in physical.ACTIVESERVOS
                        if row[0] == 8 and row[1] == bus)
            direction = -sign if name in REVERSED_MODEL_AXES else sign
            self.addresses[13, bus] = (model.actuator(name).id, direction)

    def set_targets(self, values):
        for value in values:
            actuator, sign = self.addresses[value.Id, value.Sio]
            angle = (value.Data - 7500) / (1698 * sign)
            low, high = self.model.actuator_ctrlrange[actuator]
            self.data.ctrl[actuator] = max(low, min(high, angle))


def simulated_jump_rows(direction, fraction):
    rows = backend.jump_rows(direction, fraction)
    if direction == "forward":
        for row in rows:
            row[1], row[12] = round(.75 * row[1]), round(.75 * row[12])
    return rows


class MotionLibrary:
    """Generate the same servo frames used by the robot's software motion path."""

    ASSETS = backend.ASSETS

    def __init__(self, parameters):
        self.parameters = parameters
        self.model = Robot()
        self.engine = None
        self.pose = "unknown"

    def _rows(self, rows, *, legs_only=False):
        for row in rows:
            values = self.model.mapping.targets(self.model, row[1:], legs_only=legs_only)
            yield ("servo", values, int(row[0]), int(row[0])-1)

    def _slot(self, name, speed_factor=1.):
        document = json.loads((self.ASSETS / "slots" / f"{name}.json").read_text())
        if "frames" in document:
            return slots.decode(document, self.model, speed_factor)
        return list(self._rows([[round(row[0]/speed_factor), *row[1:]]
                                for row in document[name]]))

    def _jump(self, direction, fraction):
        rows = simulated_jump_rows(direction, fraction)
        turning = direction.startswith("turn_")
        if turning:
            rows.append([10] + [0]*21)
        return list(self._rows(rows, legs_only=turning))

    def _walk(self, args):
        p = self.parameters
        kind = args.get("kind", "omni")
        if kind == "omni":
            orders = legacy.omni(p, args["distance_m"]*1000,
                                 args.get("course", 0.), args.get("heading", 0.))
        else:
            observed = args.get("x_m"), args.get("y_m")
            samples = iter([observed] if all(isinstance(v, (int, float)) for v in observed) else [])
            ball = lambda: next(samples, None)
            if kind == "approach":
                orders = legacy.approach(p, args["distance_m"], args["heading"], ball)
            elif kind == "verify":
                orders = legacy.verify(p, args["heading"], ball)
            elif kind == "around":
                orders = legacy.around(p, args["distance_m"], args["course"],
                                       args["initial_heading"], args["kick_heading"], ball)
            elif kind == "kickoff":
                orders = legacy.kickoff(p, args["invert"], ball)
            else:
                raise ValueError(f"Unknown walk kind: {kind}")
        engine = Engine(p, backend=backend)
        result = []
        prepared = False
        for order in orders:
            if not prepared:
                engine.first_Leg_Is_Right_Leg = order.right
                result.extend(engine.walk_Initial_Pose(start_mixing=False))
                prepared = True
            engine.first_Leg_Is_Right_Leg = order.right
            rotation = max(-.3, min(.3, order.heading))
            result.extend(engine.walk_Cycle(order.step, order.side,
                                            rotation*(1 if order.right else -1),
                                            order.index, order.total, half=order.half))
        if prepared:
            result.extend(engine.walk_Final_Pose())
            result.extend(self._slot("Initial_Pose"))
        return result

    def steps(self, operation, args):
        p = self.parameters
        if operation == "game.walk":
            return self._walk(args)
        if operation in ("game.jump", "motion.jump"):
            return self._jump(args.get("direction", "forward"), args.get("fraction", 1.))
        if operation == "motion.kick":
            self.engine = None
            self.pose = "stand"
            engine = Engine(p, backend=backend)
            engine.kick_power = args.get("power", 80)
            return list(engine.kick(args.get("leg", "right") == "right",
                                    small=args.get("soft", False), kick_offset=args.get("offset", 0)))
        if operation in ("game.head", "motion.head"):
            return [("servo", [SimpleNamespace(Id=0, Sio=1, Data=7500+args["pan"]),
                                SimpleNamespace(Id=12, Sio=2, Data=7500+args["tilt"])],
                     args.get("frames", 10), args.get("frames", 10)-1)]
        if operation == "motion.pose":
            name = args["name"]
            if name == "base_stand":
                self.engine = None
                self.pose = "base_stand"
                return self._slot("Initial_Pose")
            if name == "head_field":
                return self.steps("motion.head", dict(pan=0, tilt=p["head.field_tilt"]))
            if name == "crouch_centered":
                self.engine = Engine(p, backend=backend)
                self.pose = name
                return [("servo", self.engine.crouch_target(True),
                         p["walk.crouch_transition_frames"],
                         p["walk.crouch_transition_frames"]-1)]
            if name == "crouch":
                self.engine = Engine(p, backend=backend)
                self.pose = name
                return list(self.engine.walk_Initial_Pose(start_mixing=False))
            if name == "stand":
                if self.pose == "base_stand":
                    return []
                if self.pose == "unknown":
                    return self.steps("motion.pose", {"name": "base_stand"})
                engine = self.engine or Engine(p, backend=backend)
                self.engine = None
                self.pose = "stand"
                return list(engine.walk_Final_Pose())
            raise ValueError(name)
        if operation == "motion.slot":
            self.engine = None
            self.pose = "unknown"
            return self._slot(args["name"], args.get("speed_factor", 1.))
        if operation == "motion.splits":
            return slots.decode(backend.splits(args.get("kind", "small") == "big"), self.model)
        if operation == "motion.get_up":
            position = args.get("position", "upright")
            return self._slot("Initial_Pose" if position == "upright" else backend.RECOVERY[position])
        raise ValueError(f"Unsupported motion: {operation}")
