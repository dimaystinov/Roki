"""Measurement fixtures and operator-confirmed calibration for the MuJoCo session."""
import asyncio
import json
import math

import mujoco
import numpy as np

from roki_ng.body import calibration
from roki_ng.body.motion.odometry import walk_delta
from roki_ng.wire import Fault


def axes_mm(dx, dy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return round(1000*(dx*c+dy*s)), round(1000*(dy*c-dx*s))


class CalibrationSession:
    def __init__(self, world):
        self.world = world
        self.enabled = False
        self.phase = "stand"
        self.job_id = self.result = self.last_args = self.fixture_args = self.fixture = None
        self.points = []
        self.recording = False
        self.scene_positions = {}
        self.floor = world.model.geom("floor").id
        self.floor_material = int(world.model.geom_matid[self.floor])
        self.floor_color = world.model.geom_rgba[self.floor].copy()
        self.feet = {world.model.joint(f"{leg}_ankle_roll_joint").bodyid.item()
                     for leg in ("right", "left")}

    def state(self):
        return dict(enabled=self.enabled, phase=self.phase, busy=self.job_id is not None,
                    result=self.result, job_id=self.job_id)

    def scene(self, enabled):
        w, m = self.world, self.world.model
        self.enabled = enabled
        self.phase, self.points = "stand", []
        self.result = self.last_args = self.fixture_args = self.fixture = self.job_id = None
        self.recording = False
        if enabled:
            for name in ("goal_yellow", "goal_blue", "walking_person", "opponent_keeper", "opponent_forward"):
                ident = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
                if ident >= 0:
                    self.scene_positions[ident] = m.body_pos[ident].copy()
                    m.body_pos[ident, 0] += 100
            m.geom_matid[self.floor] = -1
            m.geom_rgba[self.floor] = [.18, .30, .27, 1]
            self.prepare({})
        else:
            for ident, position in self.scene_positions.items():
                m.body_pos[ident] = position
            self.scene_positions.clear()
            m.geom_matid[self.floor] = self.floor_material
            m.geom_rgba[self.floor] = self.floor_color
            mujoco.mj_resetData(m, w.data)
            w.x, w.y = w.settings["layout"]["robot_spawn_xy"]
            yaw = math.radians(w.settings["layout"]["robot_yaw_deg"])
            w.data.qpos[:3] = [w.x, w.y, .29]
            w.data.qpos[3:7] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
            w.data.qpos[w.ball_qpos:w.ball_qpos+3] = [*w.settings["layout"]["ball_spawn_xy"], m.geom_size[w.ball_geom,0]]
            w.referee.reset()
            w._put_pose()

    def prepare(self, args, repeat=False):
        w = self.world
        m, d = w.model, w.data
        state_type = mujoco.mjtState.mjSTATE_INTEGRATION
        mujoco.mj_resetData(m, d)
        w.head_ramp = w.last_bgr = None
        w.pan, w.tilt = 0, -2100 if "strength" in args else -1500
        w.head_targets = (w.pan, w.tilt)
        d.joint("head_pitch_joint").qpos[0] = w.tilt / 1698
        d.ctrl[m.actuator("head_pitch").id] = w.tilt / 1698
        if repeat:
            mujoco.mj_setState(m, d, self.fixture, state_type)
        else:
            for ident in self.scene_positions:
                mocap = m.body_mocapid[ident]
                if mocap >= 0:
                    d.mocap_pos[mocap] = m.body_pos[ident]
            d.qpos[:7] = [0., 0., .29, 1., 0., 0., 0.]
            pose = args.get("fixture", "upright")
            if pose != "upright":
                angle = math.radians(75) * (1 if pose in ("front", "right") else -1)
                axis = [1,0,0] if pose in ("right","left") else [0,1,0]
                mujoco.mju_axisAngle2Quat(d.qpos[3:7], axis, angle)
                d.qpos[2] = .13
            b = w.ball_qpos
            d.qpos[b:b+3] = [8., 8., m.geom_size[w.ball_geom,0]]
            for _ in range(20):
                w.advance(.02)
            d.qpos[:2] = 0.
            w._put_pose()
            if args.get("name") == "kick_test":
                p = w.motions.parameters
                front = args.get("ball_forward_mm", p["game.kick_distance_mm"]) / 1000
                side = args.get("ball_side_mm", p["game.kick_offset_mm"]) / 1000
                side *= 1 if args.get("leg") == "left" else -1
                c, s = math.cos(w.yaw), math.sin(w.yaw)
                d.qpos[b:b+3] = [w.x+c*front-s*side, w.y+s*front+c*side, m.geom_size[w.ball_geom,0]]
                d.qvel[w.ball_qvel:w.ball_qvel+6] = 0
            self.fixture_args = dict(args)
            self.fixture = np.empty(mujoco.mj_stateSize(m, state_type))
            mujoco.mj_getState(m, d, self.fixture, state_type)
        w._put_pose()
        self.points = [(w.x, w.y)]
        self.yaw0 = self.last_yaw = w.yaw
        self.turn = self.path = 0.
        self.pitch_min = self.pitch_max = self.pitch()
        self.was_upright = self.upright(support=False)
        self.started_fallen = not self.was_upright
        self.fallen = False
        self.ball_start = d.qpos[w.ball_qpos:w.ball_qpos+2].copy()
        self.ball_points = [tuple(self.ball_start)] if args.get("name") == "kick_test" else []
        self.ball_path = 0.
        self.striking_foot = m.joint(args.get("leg","right")+"_ankle_roll_joint").bodyid.item()
        self.contact = False
        self.kick_lab = "strength" in args
        self.ball_load_n = 0.
        self.repeats = 0
        self.odo = None
        self.command_mm = 0.

    def sample_contacts(self):
        w = self.world
        for index, c in enumerate(w.data.contact):
            if c.efc_address >= 0 and w.ball_geom in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == w.ball_geom else c.geom1
                if w.model.geom_bodyid[other] == self.striking_foot:
                    self.contact = True
                    if self.kick_lab:
                        force = np.zeros(6)
                        mujoco.mj_contactForce(w.model, w.data, index, force)
                        # MuJoCo gives the wrench on geom2; reverse it when the foot is geom1.
                        vertical = (c.frame.reshape(3, 3).T @ force[:3])[2]
                        self.ball_load_n = max(self.ball_load_n, vertical if c.geom1 == w.ball_geom else -vertical)

    def pitch(self):
        qw, qx, qy, qz = self.world.data.qpos[3:7]
        return math.degrees(math.asin(max(-1., min(1., 2*(qw*qy-qx*qz)))))

    def upright(self, support=True):
        w = self.world
        q = w.data.qpos
        vertical = q[2] >= .17 and 1-2*(q[4]**2+q[5]**2) >= .5
        if not vertical or not support:
            return bool(vertical)
        return any(c.efc_address >= 0 and self.floor in (c.geom1, c.geom2)
                   and w.model.geom_bodyid[c.geom2 if c.geom1 == self.floor else c.geom1] in self.feet
                   for c in w.data.contact)

    def sample(self):
        w = self.world
        point = (w.x,w.y)
        self.path += math.dist(point, self.points[-1])
        self.points.append(point)
        if self.ball_points:
            ball = tuple(w.data.qpos[w.ball_qpos:w.ball_qpos+2])
            self.ball_path += math.dist(self.ball_points[-1], ball)
            self.ball_points.append(ball)
        self.turn += calibration.wrap(w.yaw-self.last_yaw)
        self.last_yaw = w.yaw
        pitch = self.pitch()
        self.pitch_min = min(self.pitch_min, pitch)
        self.pitch_max = max(self.pitch_max, pitch)
        vertical = self.upright(support=False)
        self.fallen |= self.was_upright and not vertical
        self.was_upright |= vertical

    def check(self, args):
        if self.job_id is not None:
            raise Fault("busy", "Calibration motion is active")
        expected = {"stand": "get_up_test", "rotation": "rotation_test",
                    "save_rotation": "rotation_test"}.get(self.phase)
        if expected and args.get("name") != expected:
            raise Fault("invalid_state", f"Complete {expected} and confirm its result first")

    def finish(self, robot, job, args, status):
        w = self.world
        dx, dy = axes_mm(w.x-self.points[0][0], w.y-self.points[0][1], self.yaw0)
        name = args["name"]
        upright = self.upright()
        self.result = r = dict(job_id=job["job_id"], test=name, args=args, dx_mm=dx, dy_mm=dy,
                               yaw_deg=round(math.degrees(self.turn),2), path_mm=round(self.path*1000),
                               upright=upright, fallen=self.fallen, started_fallen=self.started_fallen, status=status, odometry=self.odo,
                               values={}, previous={})
        r.update(start_x_mm=round(self.points[0][0]*1000), start_y_mm=round(self.points[0][1]*1000),
                 pitch_deg=round(self.pitch(),2), pitch_min_deg=round(self.pitch_min,2),
                 pitch_max_deg=round(self.pitch_max,2))
        if job.get("verification"):
            r["verification_error_deg"] = round(math.degrees(job["verification"]["error_rad"]),2)
        if "error" in job:
            r["error"] = str(job["error"])[:140]
        if self.odo is not None:
            r["odometry"] = dict(zip(("dx_mm", "dy_mm"), (round(v*1000) for v in self.odo)))
        if name == "jump_test":
            r.update(repeats=self.repeats, mean_dx_mm=round(dx/self.repeats) if self.repeats else 0,
                     mean_dy_mm=round(dy/self.repeats) if self.repeats else 0)
        if name == "kick_test":
            if self.kick_lab:
                r["ball_support_n"] = round(float(self.ball_load_n), 2)
            bx, by = axes_mm(*(w.data.qpos[w.ball_qpos:w.ball_qpos+2]-self.ball_start), self.yaw0)
            r.update(ball_contact=self.contact, ball_dx_mm=bx, ball_dy_mm=by,
                     ball_distance_mm=round(math.hypot(bx,by)), ball_path_mm=round(self.ball_path*1000))
        if status != "completed" or not upright or (name != "get_up_test" and self.fallen):
            return
        if name == "get_up_test" and self.phase == "stand":
            self.phase = "rotation"
        if name == "rotation_test":
            r["values"] = dict(job.get("calibration", {}))
            if self.phase != "ready" and set(r["values"]) == set(calibration.ROTATION_KEYS):
                self.phase = "save_rotation"
        for key in job.get("manual_parameters", []):
            distance = dy if any(k in key for k in ("_left","_right","shift_y")) else dx
            if "jump_" in key and self.repeats:
                distance = round(distance/self.repeats)
            if "right" in key or "backward" in key:
                distance = -distance
            r["values"][key] = round(distance + job.get("measurement_baseline",{}).get(key,0.))
        if r["values"] and "run_command_mm" in job:
            k = next(iter(r["values"])).removesuffix("_mm") + "_command_mm"
            r["values"][k] = round(job["run_command_mm"])
        r["previous"] = {k:robot.game.p[k] for k in r["values"]}

    def save(self, robot, body):
        r = self.result
        if (not self.enabled or self.job_id is not None or not r or r["job_id"] != body.get("job_id")
                or not r["values"] or r.get("saved")):
            raise Fault("invalid_state", "Select a completed measurement")
        values = body.get("values")
        if not isinstance(values, dict) or set(values) != set(r["values"]):
            raise Fault("invalid_argument", "Supply all measured parameters")
        if any(k.endswith("_mm") and (isinstance(v,bool) or not isinstance(v,(int,float)) or
               not math.isfinite(v) or not float(v).is_integer()) for k,v in values.items()):
            raise Fault("invalid_argument", "Distances use whole millimetres")
        if any(robot.store.values[k] != v for k,v in r["previous"].items()):
            raise Fault("conflict", "Repeat the measurement after the profile change")
        saved = robot.store.set_many(values)
        disk = json.loads(robot.store.path.read_text())
        if any(disk[k] != v for k,v in saved.items()):
            raise Fault("save_failed", "Simulator profile verification failed")
        robot.game.p.update({k:disk[k] for k in saved})
        self.result = dict(r, proposed=r["values"], values={k:disk[k] for k in saved}, saved=True)
        if r["test"] == "rotation_test":
            self.phase = "ready"
        return self.state()

    def draw(self, window):
        scene = window.user_scn
        with window.lock():
            scene.ngeom = 0
            if self.enabled:
                def line(a,b,radius,color):
                    if scene.ngeom >= scene.maxgeom:
                        return
                    geom = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                                        np.zeros(3), np.eye(3).ravel(), color)
                    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, a,b)
                    scene.ngeom += 1
                for tick in range(-20,21):
                    v = tick/10
                    line([v,-2,.002],[v,2,.002],.001,[.6,.7,.65,.65])
                    line([-2,v,.002],[2,v,.002],.001,[.6,.7,.65,.65])
                path = self.points[::max(1, math.ceil(len(self.points)/300))]
                if path:
                    path = path + self.points[-1:]
                    for a,b in zip(path,path[1:]):
                        if a != b:
                            line([*a,.009],[*b,.009],.005,[1,.8,.2,1])
                    ball_path = self.ball_points[::max(1,math.ceil(len(self.ball_points)/200))] + self.ball_points[-1:]
                    for a,b in zip(ball_path,ball_path[1:]):
                        if a != b:
                            line([*a,.012],[*b,.012],.005,[1,.4,.1,1])
                    for p,color in [(path[0],[.2,1,.4,1]),(path[-1],[1,.3,.25,1])]:
                        line([*p,.01],[*p,.06],.012,color)
        if self.enabled:
            r = self.result
            lines = [f"Calibration   {self.phase}   |   grid 100 mm"]
            if self.recording:
                dx, dy = axes_mm(self.world.x-self.points[0][0], self.world.y-self.points[0][1], self.yaw0)
                lines += [self.last_args["name"], f"X {dx} mm   Y {dy} mm   path {round(self.path*1000)} mm",
                          f"Yaw {math.degrees(self.turn):.1f} deg   pitch {self.pitch():.1f} deg   fallen {self.fallen}"]
            if r:
                lines += [f"{r['test']}   X {r['dx_mm']} mm   Y {r['dy_mm']} mm",
                          f"Path {r['path_mm']} mm   yaw {r['yaw_deg']} deg   upright {r['upright']}   fallen {r['fallen']}"]
                lines += [f"Pitch {r['pitch_deg']} deg   min {r['pitch_min_deg']}   max {r['pitch_max_deg']}"]
                lines += [f"{k}: {r['previous'][k]:g} -> {v:g}" for k,v in r["values"].items()]
                if r["odometry"] is not None:
                    lines += [f"Odometry X {r['odometry']['dx_mm']} Y {r['odometry']['dy_mm']} mm"]
                if "repeats" in r:
                    lines += [f"Jumps {r['repeats']}   mean X {r['mean_dx_mm']} Y {r['mean_dy_mm']} mm"]
                if "ball_contact" in r:
                    lines += [f"Foot contact {r['ball_contact']}   ball X {r['ball_dx_mm']} Y {r['ball_dy_mm']} mm"]
            window.set_texts((None,None,"\n".join(lines),""))
        else:
            window.clear_texts()


async def run_calibration(robot, job, args):
    game, session = robot.game, robot.calibration
    external_id = job["job_id"]
    previous = game.on_body_event
    def emit(op, message):
        if op == "job.progress":
            job.update(message, job_id=external_id)
            if args["name"] == "jump_test" and message.get("stage") == (
                    "forward" if args.get("direction") == "on_spot" else args.get("direction","forward")):
                session.repeats += 1
            if "run_command_mm" in message:
                cmd = message["run_command_mm"]
                step, _ = walk_delta(game.p, cmd-session.command_mm, first=session.odo is None)
                game._refresh_imu()
                yaw = game.measured_yaw-session.imu_origin
                if session.odo is None:
                    session.odo = [0.,0.]
                session.odo[0] += step*math.cos(yaw)
                session.odo[1] += step*math.sin(yaw)
                session.command_mm = cmd
        elif op == "calibration.ready":
            game.football_body.command("calibration.resolve", {"job_id": message["job_id"]})

    game.on_body_event = emit
    status = "failed"
    try:
        game.football_body = None
        session.prepare(args, repeat=robot.repeat_fixture)
        if "strength" in args:
            from sim_kick_lab import preview
            visual = preview(robot, args)
        game._refresh_imu()
        session.imu_origin = game.measured_yaw
        session.recording = True
        motion_args = {k:v for k,v in args.items() if k not in ("fixture","leg","ball_forward_mm","ball_side_mm","strength")}
        result = await game._body_motion("test.start", motion_args)
        job.update(result, job_id=external_id)
        job["status"] = "running"
        # Require a sustained stop, including ball rollout. Hold the physics lock
        # so the regular viewer tick never adds a second clock to this interval.
        w = robot.world
        async with w.motion_lock:
            still = 0
            for _ in range(600):
                await w.wait_frame(.02)
                ball_speed = np.linalg.norm(w.data.qvel[w.ball_qvel:w.ball_qvel+3])
                quiet = np.linalg.norm(w.data.qvel[:6]) < .035
                still = still + 1 if quiet and (args["name"] != "kick_test" or ball_speed < .012) else 0
                if still >= 15:
                    break
            if still < 15:
                raise Fault("unsettled", "Motion did not settle within 12 simulated seconds")
        status = "completed"
    except asyncio.CancelledError:
        status = "cancelled"
        raise
    except Exception as exc:
        job["error"] = str(exc)
        raise
    finally:
        game.on_body_event = previous
        session.recording = False
        session.finish(robot,job,args,status)
        session.job_id = None
        game.football_body = None
        if "strength" in args and "visual" in locals():
            from sim_kick_lab import record
            session.result["kick_lab"] = record(robot, args, visual, session.result)
