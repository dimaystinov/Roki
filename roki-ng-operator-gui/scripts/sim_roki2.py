#!/usr/bin/env python3
"""Local Roki2 soccer simulator for Simple Football and FPV Roki.

Run with the operator GUI's mjpython.  Servo targets drive the free body through MuJoCo contacts.
Rendered camera frames feed the production football vision and state machine.
"""
import argparse
import asyncio
import json
import math
import os
import queue
import sys
import tempfile
import threading
import time
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve()
WORKSPACE = HERE.parents[2]
ROBOT_COPY = Path(os.environ.get("ROKI_SIM_RUNTIME", HERE.parents[1] / "sim/.robot-runtime"))
sys.path.insert(0, str(ROBOT_COPY if (ROBOT_COPY / "roki_ng").is_dir() else WORKSPACE / "roki-ng"))
sys.path.insert(0, str(WORKSPACE / "Bioloid-main/Bioloid/sim/roki"))
sys.path.insert(0, str(WORKSPACE / "Bioloid-main/Elsiros/controllers/Robofest_TEAM"))
sys.path.insert(0, str(HERE.parents[1]))
sys.path.insert(0, str(HERE.parent))

from roki_ng.body.body_imu import BodyImu
from roki_ng.body.recovery import body_position
from roki_ng.wire import Fault
from roki_ng.detection.ball import ball_candidates
from roki_ng.body.calibration import wrap
from roki_ng.detection.football import TICK, SceneTracker, camera_geometry, goal_parts, plane_point
from roki_ng.body.motion.odometry import GameOdometry
from roki_ng.parameters import SCHEMA, Parameters
from roki_ng.body.runtime import Body
from roki_ng.body import calibration
from roki_ng.wire import page
from roki_ng.supervisor.game.simple_football import SimpleFootball
from roki_ng.body.motion.backends.roki2 import backend
from roki_ng.detection.football import ball_at_foot
from roki_ng.body.motion.engine import Engine
from roki_ng.body.motion.gait import Servo
from roki_ng.localisation.filter import PoseFilter
from roki_ng.localisation.pipeline import process_frame
from tests.fake_robot import FakeRobot
from sim_visual_dribble import VisualDribble
from sim_referee import GOAL_CLEAR_HEIGHT, PitchReferee, painted_limits
from sim_scene import apply_settings, load_settings, soccer_model, opposing_start
from sim_calibration import CalibrationSession, run_calibration
from sim_ball import CARPET_MU, KickContact
from sim_motion import ServoBridge, MotionLibrary, simulated_jump_rows

SCENE = (WORKSPACE / "Bioloid-main/Bioloid/sim/roki_lab/rokilab-main"
         / "resources/assets/scene_roki2_soccer.xml")
SETTINGS = WORKSPACE / "roki-ng-operator-gui/sim/scene_settings.json"
ROBOT_PARAMETERS = WORKSPACE / "roki-ng-operator-gui/sim/roki2_robot_parameters.json"


def sim_parameters(settings=None, robot_parameters=ROBOT_PARAMETERS):
    p = {key: value[1] for key, value in SCHEMA.items()}
    if robot_parameters and Path(robot_parameters).exists():
        saved = json.loads(Path(robot_parameters).read_text())
        p.update({key: value for key, value in saved.items() if key in p})
    if robot_parameters == ROBOT_PARAMETERS:
        p.update(json.loads((ROBOT_PARAMETERS.parent / "mujoco_calibration.json").read_text()))
    settings = settings or load_settings(SETTINGS)
    layout = settings["layout"]
    # The imported scene has yellow on +X and blue on -X.
    p["field.goal.0"] = p["field.goal.0"] | {"x": layout["yellow_goal_x_m"], "height": GOAL_CLEAR_HEIGHT}
    p["field.goal.1"] = p["field.goal.1"] | {"x": layout["blue_goal_x_m"], "height": GOAL_CLEAR_HEIGHT}
    half_length, half_width = painted_limits(layout)
    p["field.geometry"] = p["field.geometry"] | {
        "length": 2*half_length, "width": 2*half_width,
        "circle_diameter": .60, "circle_measured": True, "paint_width": .05,
        "carpet_length": layout["field_length_m"],
        "carpet_width": layout["field_width_m"]}
    if not robot_parameters or not Path(robot_parameters).exists():
        p["match.own_goal"] = 1
    return p


class PinholeProjector:
    def __init__(self, fovy_deg=65):
        self.focal = 325 / math.tan(math.radians(fovy_deg) / 2)
        self.camera_rotation = None
        self.pitch = 0.
        self.height = .4068
        self.camera_forward = .23

    def _pixel_ray(self, pixel, _quaternion):
        u, v = pixel
        x, y, z = 1., (400-u)/self.focal, (325-v)/self.focal
        if self.camera_rotation is not None:
            return self.camera_rotation @ [-y, z, -x]
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        return np.array([c*x+s*z, y, -s*x+c*z], dtype=float)

    def ground_point(self, pixel, quaternion):
        ray = self._pixel_ray(pixel, quaternion)
        if ray[2] >= -.04:
            raise ValueError("Ray above horizon")
        scale = -self.height/ray[2]
        return np.array([self.camera_forward + ray[0]*scale, ray[1]*scale])

    def bearing(self, pixel, quaternion):
        ray = self._pixel_ray(pixel, quaternion)
        return math.atan2(ray[1], ray[0])

    def image_points(self, points, quaternion):
        """Inverse of ground_point for the shared pitch-boundary detector."""
        xy = np.asarray(points, dtype=float)
        rays = np.column_stack((xy[:,0]-self.camera_forward, xy[:,1], np.full(len(xy),-self.height)))
        if self.camera_rotation is not None:
            local = rays @ self.camera_rotation
            forward, right, up = -local[:,2], local[:,0], local[:,1]
        else:
            c,s = math.cos(self.pitch), math.sin(self.pitch)
            forward = c*rays[:,0]-s*rays[:,2]
            right = -rays[:,1]
            up = s*rays[:,0]+c*rays[:,2]
        valid = forward > .01
        depth = np.where(valid,forward,1.)
        u,v = 400+self.focal*right/depth, 325-self.focal*up/depth
        return np.column_stack((u,v)), valid & (u>=0) & (u<800) & (v>=0) & (v<650)

    def upright_height(self, foot_pixel, top_pixel, quaternion):
        foot = self.ground_point(foot_pixel, quaternion)
        ray = self._pixel_ray(top_pixel, quaternion)
        scale = (foot[0]-self.camera_forward)/ray[0]
        if scale <= 0:
            raise ValueError("Top behind camera")
        return self.height + scale*ray[2]

    def project(self, image, quaternion):
        if image.shape[:2] != (650, 800):
            raise ValueError("Expected 800x650 image")
        rows, cols = np.indices((720, 720), dtype=np.float32)
        x = 4-(rows+.5)/180-self.camera_forward
        y = 2-(cols+.5)/180
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        forward = c*x+s*self.height
        up = s*x-c*self.height
        good = forward > .01
        denom = np.where(good, forward, 1.)
        u = (400-self.focal*y/denom).astype(np.float32)
        v = (325-self.focal*up/denom).astype(np.float32)
        good &= (u >= 0) & (u < 800) & (v >= 0) & (v < 650)
        u[~good] = -100
        v[~good] = -100
        return cv2.remap(image, u, v, cv2.INTER_LINEAR)


class World:
    def __init__(self, scene=SCENE, settings=None, parameters=None):
        self.settings = settings or load_settings(SETTINGS)
        self.model = soccer_model(scene, self.settings)
        apply_settings(self.model, self.settings)
        self.data = mujoco.MjData(self.model)
        self.calibration = None
        self.kick_contact = None
        self.ball_geom = self.model.geom("ball").id
        ball_body = self.model.geom_bodyid[self.ball_geom]
        radius = (parameters or sim_parameters(self.settings))["game.ball_diameter_m"] / 2
        self.model.geom_size[self.ball_geom, 0] = radius
        self.model.geom_rbound[self.ball_geom] = radius
        self.model.geom_aabb[self.ball_geom, 3:] = radius
        self.model.body_inertia[ball_body] = .4 * self.model.body_mass[ball_body] * radius**2
        # For a solid rolling sphere, torque/(mg) = (1 + I/mr²) r a/g.
        self.model.pair_friction[self.model.pair("ball_carpet").id, 3:] = 1.4 * radius * CARPET_MU
        mujoco.mj_setConst(self.model, self.data)
        self.servos = ServoBridge(self.model, self.data)
        self.motions = MotionLibrary(parameters if parameters is not None else sim_parameters(self.settings))
        self.motion_lock = asyncio.Lock()
        self.imu_callback = None
        ball_joint = self.model.joint("soccer_ball_joint").id
        self.ball_qpos = int(self.model.jnt_qposadr[ball_joint])
        self.ball_qvel = int(self.model.jnt_dofadr[ball_joint])
        self.camera_options = mujoco.MjvOption()
        self.camera_options.geomgroup[5] = 0
        self.head_shell = np.flatnonzero(self.model.geom_bodyid == self.model.body("head_1").id)
        self.head_groups = self.model.geom_group[self.head_shell].copy()
        self.renderer = mujoco.Renderer(self.model, height=650, width=800)
        self.projector = PinholeProjector(self.settings["camera"]["fovy_deg"])
        layout = self.settings["layout"]
        own = self.motions.parameters['field.goal.'+str(self.motions.parameters['match.own_goal'])]['colour']
        players = {'torso_1': own}
        if self.settings['opponents']['enabled']:
            other = 'blue' if own == 'yellow' else 'yellow'
            players.update(opponent_keeper=other, opponent_forward=other)
        self.referee = PitchReferee(self.model, self.data, layout, players)
        self.x, self.y = layout["robot_spawn_xy"]
        self.yaw = math.radians(layout["robot_yaw_deg"])
        self.pan, self.tilt = 0, -1500
        self.head_targets = (0, -1500)
        self.head_ramp = None
        self.data.joint("head_pitch_joint").qpos[0] = self.tilt / 1698
        self.data.ctrl[self.model.actuator("head_pitch").id] = self.tilt / 1698
        self.camera_forward = self.settings["camera"]["forward_m"]
        self.started_at = time.monotonic()
        self.last_bgr = None
        self.sequence = 0
        self.data.qpos[:3] = [self.x, self.y, .29]
        self.data.qpos[3:7] = [math.cos(self.yaw/2), 0., 0., math.sin(self.yaw/2)]
        self._put_pose()
        self.data.qpos[self.ball_qpos:self.ball_qpos+3] = [*layout["ball_spawn_xy"], self.model.geom_size[self.ball_geom, 0]]
        mujoco.mj_forward(self.model, self.data)

    def place_opponent(self, xy, yaw):
        if self.settings["opponents"]["enabled"]:
            idx = self.model.body("opponent_forward").mocapid.item()
            position, quat = opposing_start(xy, yaw)
            self.data.mocap_pos[idx, :2] = position
            self.data.mocap_quat[idx] = quat

    def _put_pose(self):
        self._set_camera_pose()
        mujoco.mj_forward(self.model, self.data)

    def _set_camera_pose(self):
        d = self.data
        self.x, self.y = map(float, d.qpos[:2])
        base = d.qpos[3:7]
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, base)
        rotation = rotation.reshape(3, 3)
        self.yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        geometry = camera_geometry(self.motions.parameters, self.pan, self.tilt, backend.Robot.camera)
        forward = self.camera_forward + geometry["x"]
        d.mocap_pos[0] = d.qpos[:3] + rotation @ [forward, geometry["y"], geometry["height"]-.29]
        self.projector.height = d.mocap_pos[0, 2]
        self.projector.camera_forward = forward
        self.projector.pitch = -self.tilt*TICK
        pan_q, pitch_q, mount_q = np.zeros((3, 4))
        mujoco.mju_axisAngle2Quat(pan_q, [0, 0, 1], geometry["pan"])
        mujoco.mju_axisAngle2Quat(pitch_q, [0, 1, 0], self.projector.pitch)
        mujoco.mju_mulQuat(mount_q, pan_q, pitch_q)
        mujoco.mju_mulQuat(d.mocap_quat[0], base, mount_q)

    def render(self):
        self._put_pose()
        # The modeled optical centre sits inside the CAD head cover.
        self.model.geom_group[self.head_shell] = 5
        try:
            self.renderer.update_scene(self.data, camera="sim_head_camera", scene_option=self.camera_options)
        finally:
            self.model.geom_group[self.head_shell] = self.head_groups
        self.last_bgr = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
        self.sequence += 1
        return self.last_bgr

    def command_head(self, pan, tilt, frames=10):
        indices = [self.model.actuator(name).id for name in ("head_yaw", "head_pitch")]
        start = self.data.ctrl[indices].copy()
        self.servos.set_targets([Servo(0, 1, 7500 + pan),
                                 Servo(12, 2, 7500 + tilt)])
        target = self.data.ctrl[indices].copy()
        self.data.ctrl[indices] = start
        self.head_targets = (pan, tilt)
        self.head_ramp = [indices, start, target, 0., max(.02, frames*.02)]

    async def play_steps(self, steps):
        servo_steps = [step for step in steps if step[0] == "servo" and step[1]]
        frames_total = sum(step[2] for step in servo_steps)
        if not frames_total:
            return
        async with self.motion_lock:
            for _, values, frames, _ in servo_steps:
                start = self.data.ctrl.copy()
                self.servos.set_targets(values)
                target = self.data.ctrl.copy()
                self.data.ctrl[:] = start
                for frame in range(frames):
                    self.data.ctrl[:] = start + (target-start)*(frame+1)/frames
                    self.advance(.02)
                    await asyncio.sleep(.02)
            self.last_bgr = None

    async def wait_frame(self, dt):
        self.advance(dt)
        await asyncio.sleep(dt)

    def _advance_head(self, dt):
        if self.head_ramp is not None:
            indices, start, target, elapsed, duration = self.head_ramp
            elapsed = min(duration, elapsed + dt)
            self.data.ctrl[indices] = start + (target - start) * elapsed / duration
            self.head_ramp = None if elapsed >= duration else [indices, start, target, elapsed, duration]

    def advance(self, dt=.025):
        self._advance_head(dt)
        for _ in range(max(1, round(dt/self.model.opt.timestep))):
            previous = tuple(self.data.qpos[self.ball_qpos:self.ball_qpos+3])
            mujoco.mj_step(self.model, self.data)
            if self.kick_contact is not None:
                self.kick_contact.sample(self)
            if self.calibration and self.calibration.recording:
                self.calibration.sample_contacts()
            event = None if self.calibration and self.calibration.enabled else self.referee.standalone_tick(previous)
            if event:
                print("REFEREE", event, flush=True)
                if event[0] == 'throw_in' and self.kick_contact:
                    self.kick_contact.completed = True
        self._put_pose()
        if self.imu_callback:
            self.imu_callback()
        self.pan = round(float(self.data.joint("head_yaw_joint").qpos[0]) * 1698)
        self.tilt = round(float(self.data.joint("head_pitch_joint").qpos[0]) * 1698)
        person = self.settings["person"]
        if self.calibration and self.calibration.recording:
            self.calibration.sample()
        if person["enabled"] and person["walk_amplitude_m"] and not (self.calibration and self.calibration.enabled):
            phase = 2*math.pi*(time.monotonic()-self.started_at)/person["walk_period_s"]
            self.model.body_pos[self.model.body("walking_person").id, 1] = (
                person["y_m"] + person["walk_amplitude_m"]*math.sin(phase))
            mujoco.mj_forward(self.model, self.data)


class SimLog:
    def __init__(self):
        self.body_telemetry = {}

    def log(self, source, level, message):
        if level != "DEBUG":
            print(f"[{source}:{level}] {message}", flush=True)


class SimBodyImu(BodyImu):
    """Age local physics samples against simulation time, including viewer pauses."""
    def __init__(self, world):
        super().__init__()
        self.world = world

    def fresh(self, now, max_age=BodyImu.MAX_AGE_S):
        return super().fresh(self.world.data.time, max_age)

    def poll(self, hardware):
        result = super().poll(hardware)
        self.received_at = self.last_read_at = self.world.data.time
        self.next_at = self.received_at + self.PERIOD_S
        return result


class SimBody(Body):
    test_kick_leg = None
    test_soft = False

    def _kick(self, leg, power, offset, kick_type=None, *, soft=False):
        yield from super()._kick(self.test_kick_leg or leg, power, offset, "software" if self.test_soft else kick_type, soft=soft or self.test_soft)

    def _slot(self, rows, factor=1, *, legs_only=False):
        if self.recovery_active and self.active:
            posture = self.jobs[self.active].get("initial_posture")
            if posture in ("left", "right") and getattr(self, 'side_retry_profile', None):
                posture = self.side_retry_profile
            if posture in ("stomach", "back"):
                profile = HERE.parents[1] / "sim/recovery" / f"{posture}.json"
                rows = json.loads(profile.read_text())
                self.jobs[self.active]["sim_recovery_profile"] = profile.name
        yield from super()._slot(rows, factor, legs_only=legs_only)


class GameAdapter:
    def __init__(self, world, parameters, *, imu_seed=0):
        self.imu_seed = imu_seed
        self.imu_noise_block = -1
        self.world, self.p = world, parameters
        self.info = {"phase": "starting", "reason": ""}
        self.football_body = None
        self.on_body_event = lambda op, message: None
        self.s = SimLog()
        from types import SimpleNamespace
        self.s.params = SimpleNamespace(backend=backend)
        self.revision = 1
        self.odometry = GameOdometry(parameters, "center", world.yaw)
        self.odometry.x, self.odometry.y = world.x, world.y  # Known initial placement.
        self.odometry.yaw, self.odometry.yaw_offset = world.yaw, 0.
        self.tracker = SceneTracker()
        self.pose = None
        self.scene = {}
        self.visual_window = None
        self.localiser = None
        self.localisation_sequence = -1
        self.localisation_poses = []
        self._refresh_imu()

    def _refresh_imu(self):
        sensor = np.empty(4)
        mount = math.sqrt(.5)
        # Sensor X points left, Y up and Z forward on the body PCB.
        # Reference world yaw to the robot's forward heading at spawn yaw zero.
        mounted = np.empty(4)
        mujoco.mju_mulQuat(mounted, self.world.data.qpos[3:7], [.5, .5, .5, .5])
        mujoco.mju_mulQuat(sensor, [mount, 0., 0., -mount], mounted)
        elapsed = self.world.data.time
        block, index = divmod(round(elapsed * 50), 1024)
        if block != self.imu_noise_block:
            rng = np.random.default_rng([self.imu_seed, block])
            self.imu_samples = rng.normal(size=(1024, 3))
            self.imu_noise_block = block
        config = self.world.settings["imu"]
        angles = self.imu_samples[index] * math.radians(config["noise_std_deg"])
        angles[2] += math.radians(config["yaw_drift_deg_per_min"] * elapsed / 60)
        perturbation = np.empty(4)
        mujoco.mju_euler2Quat(perturbation, angles, "xyz")
        sensor = self._perturbed_quat(perturbation, sensor)
        torso = self._perturbed_quat(perturbation, self.world.data.qpos[3:7])
        w, x, y, z = torso
        self.measured_yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        self.s.body_telemetry["body.imu"] = dict(
            valid=True, source_mono_ns=time.monotonic_ns(),
            quaternion_xyzw=sensor[[1, 2, 3, 0]].tolist())

    @staticmethod
    def _perturbed_quat(noise, reference):
        reading = np.empty(4)
        mujoco.mju_mulQuat(reading, noise, reference)
        return reading

    async def play_step(self, strategy):
        try:
            await self._imu()
            if getattr(self, 'recovery_attempt', 0):
                self.recovery_attempt = 0
                self.scene = {}
                self.tracker = SceneTracker()
                if self.football_body:
                    self.football_body.side_retry_profile = None
            return await strategy.step()
        except Fault as exc:
            if exc.code not in ("fallen", "posture_uncertain", "posture_unstable"):
                raise
        strategy.cancel_special("fall")
        self.odometry.uncertain("fallen")
        self.info.update(phase="get_up", reason="Checking posture after fall")
        count = getattr(self, 'recovery_attempt', 0)
        failures = ("get_up_failed", "fallen", "posture_uncertain", "posture_unstable")
        if count:
            self.info.update(phase="get_up_retry", reason=f"Rechecking posture, attempt {count+1}")
        try:
            if count:
                await self._body_motion("motion.slot", {"name": "Initial_Pose"})
            if self.football_body:
                # A side push can leave a leg trapped under the torso. On
                # later attempts fold the legs using the calibrated full-body slots.
                self.football_body.side_retry_profile = (None, None, "stomach", "back")[count % 4]
            await self._body_motion("motion.get_up", {"crouch": "off"})
            for _ in range(25):
                await self.world.wait_frame(.02)
            await self._imu()
        except Fault as exc:
            if exc.code not in failures:
                raise
            self.recovery_attempt = count+1
            self.info.update(reason=str(exc), phase="get_up_retry")
            self.s.log("game","WARNING",f"Recovery attempt {count+1}: {exc}")
            return
        self.recovery_attempt = 0
        if self.football_body:
            self.football_body.side_retry_profile = None
        self.scene = {}
        self.tracker = SceneTracker()
        self.info.update(phase="starting", reason="Upright after recovery")

    def _allowed(self, *, pickup=False):
        return True

    def _same_plan(self, revision):
        return revision == self.revision

    async def _prepare(self, *_args):
        await self._imu()
        if self.football_body and self.football_body.pose not in ("base_stand", "crouch"):
            await self._body_motion("motion.pose", {"name": "base_stand"})
            await self._move("game.head", {"pan": 0, "tilt": self.p["game.head_tilt"]})
        return True

    async def _settle(self, seconds):
        await asyncio.sleep(seconds)
        return True

    async def _imu(self, fallen_ok=False):
        self._refresh_imu()
        self.odometry.heading(self.measured_yaw)
        position = body_position(self.s.body_telemetry["body.imu"]["quaternion_xyzw"])
        if position != "upright" and not fallen_ok:
            raise Fault("fallen", f"Body is {position}")
        return self.measured_yaw, position != "upright"

    async def _open_localisation(self, _parameters):
        if self.world.pan != 0:
            return
        self.localiser = PoseFilter(
            (self.odometry.x, self.odometry.y, self.odometry.yaw),
            count=256, parameters=self.p, heading=self.odometry.yaw)
        self.localisation_poses = []
        self.localisation_sequence = -1
        self.visual_window = type("Window", (), {"accepted": False})()

    async def _poll_localisation(self):
        if self.localiser is None or self.visual_window is None or self.visual_window.accepted:
            return
        if self.world.last_bgr is None:
            self.world.render()
        if self.world.sequence <= self.localisation_sequence:
            return
        self.localisation_sequence = self.world.sequence
        result, _, _, _ = process_frame(
            self.world.last_bgr, None, self.world.projector, self.localiser,
            self.p, self.world.sequence, time.monotonic())
        if not result.get("valid") or result.get("fit_state") != "matched":
            self.localisation_poses = []
            return
        pose = result["candidate"]
        if (math.hypot(pose[0]-self.odometry.x, pose[1]-self.odometry.y) > .5
                or abs(wrap(pose[2]-self.odometry.yaw)) > .35):
            self.localisation_poses = []
            return
        self.localisation_poses.append(pose)
        self.localisation_poses = self.localisation_poses[-3:]
        if len(self.localisation_poses) == 3 and max(
            math.hypot(a[0]-b[0], a[1]-b[1])
            for a in self.localisation_poses for b in self.localisation_poses) <= .15:
            corrected = [sum(pose[i] for pose in self.localisation_poses)/3 for i in range(3)]
            self._refresh_imu()
            self.odometry.correct(corrected, self.measured_yaw)
            self.info["last_correction"] = dict(
                frame_sequence=self.world.sequence, pose=corrected, observations=3)
            self.visual_window.accepted = True

    async def _settle_for_recovery(self):
        # Side impacts can finish rolling onto the back during IMU preflight.
        previous = self.world.data.qpos[3:7].copy()
        still = 0
        for _ in range(125):
            await self.world.wait_frame(.02)
            current = self.world.data.qpos[3:7]
            # q and -q encode the same orientation. Require 0.24 s at rest.
            if abs(float(np.dot(previous, current))) >= math.cos(.0025 / 2):
                still += 1
                if still >= 12:
                    return
            else:
                still = 0
            previous[:] = current

    async def _body_motion(self, op, args):
        if op == "motion.get_up" or (op == "test.start" and args.get("name") == "get_up_test"):
            await self._settle_for_recovery()
        world = self.world
        if self.football_body is None:
            body = SimBody(dict(simulate=True, robot_model="roki2", parameters=dict(self.p)), lambda op, msg: self.on_body_event(op, msg), lambda *_: None)
            body.body_imu = SimBodyImu(world)
            body.simulated = False  # Run the hardware motion generator on the simulated bus.
            body._jump_rows = simulated_jump_rows
            body.control = True
            body.pose = "base_stand"
            body.head = dict(zip(("pan", "tilt"), world.head_targets))
            def sample():
                self._refresh_imu()
                return tuple(self.s.body_telemetry["body.imu"]["quaternion_xyzw"])
            body.hardware.body_quaternion = sample
            self.football_body = body
        body = self.football_body
        body.parameters.update(self.p)
        body.body_imu.poll(body.hardware)
        options = dict(args) if op == "test.start" else dict(args, football_arms=True)
        phase = self.info.get("phase")
        if op == "game.walk" and phase in ("simple_walk_to_ball", "simple_test_approach", "simple_kickoff_pass"):
            options["keep_crouch"] = True
            if phase == "simple_kickoff_pass":
                options["soft_touch"] = True
        test_kick = op == "test.start" and args.get("name") == "kick_test" and world.calibration and world.calibration.enabled
        body.test_kick_leg = world.calibration.last_args.get("leg","right") if test_kick else None
        body.test_soft = test_kick and world.calibration.last_args.get("strength") == "weak"
        job = body.command(op, options)["job_id"]
        frame_s = self.p["motion.frame_ms"] / 1000
        remaining = np.zeros(world.model.nu, dtype=int)
        targets = world.data.ctrl.copy()
        async def frames(count):
            for _ in range(count):
                moving = remaining > 0
                world.data.ctrl[moving] += (targets[moving] - world.data.ctrl[moving]) / remaining[moving]
                remaining[moving] -= 1
                await world.wait_frame(frame_s)

        last = -math.inf
        world.imu_callback = lambda: body.body_imu.poll(body.hardware)
        try:
            async with world.motion_lock:
                if test_kick or op == "motion.kick":
                    world.kick_contact = KickContact(world, body.test_kick_leg or args["leg"], args.get("soft", False) or body.test_soft)
                for instruction in body.plan:
                    if op == "game.walk" and args.get("kind") == "approach" and time.monotonic()-last >= .12:
                        scene = await self._call("detection", "football.status")
                        if scene.get("confirmed") and scene.get("ball") is not None:
                            x, y = scene["ball"]
                            body.command("game.observe", dict(job_id=job, x_m=x, y_m=y, stamp=scene["received"]))
                        last = time.monotonic()
                    kind = instruction[0]
                    if kind == "servo":
                        _, values, duration, pause = instruction
                        previous = world.data.ctrl.copy()
                        world.servos.set_targets(values)
                        for v in values:
                            index, _ = world.servos.addresses[v.Id, v.Sio]
                            targets[index] = world.data.ctrl[index]
                            remaining[index] = duration
                            body.sent_targets[v.Id, v.Sio] = body.servo_targets[v.Id, v.Sio] = v.Data
                        world.data.ctrl[:] = previous
                        await frames(pause + 1)
                    elif kind == "drain":
                        await frames(int(remaining.max()))
                    elif kind in ("sleep", "imu"):
                        await frames(max(1, math.ceil(instruction[1]/frame_s)) if kind == "sleep" else 1)
                    else:
                        raise Fault("not_supported", kind)
                await frames(int(remaining.max()))
            world.motions.pose = body.pose
            body._finish("completed")
        except Fault as exc:
            body._finish("failed", str(exc))
            raise
        finally:
            world.imu_callback = None
            world.kick_contact = None
            if body.active:
                body._finish("cancelled", "interrupted")
            if op in ("game.walk", "game.jump"):
                self.odometry.apply(job, body.jobs[job])
                self._refresh_imu()
                self.odometry.heading(self.measured_yaw)
        return body.jobs[job]

    async def _move(self, op, args, **_kwargs):
        if op == "game.head":
            self.world.command_head(args["pan"], args["tilt"])
            if self.football_body:
                self.football_body.head = dict(pan=args["pan"], tilt=args["tilt"])
            self.world.last_bgr = None
            # Rendering can slow the viewer; wait for simulated servo travel.
            finish = self.world.data.time + .38
            async with self.world.motion_lock:
                while self.world.data.time < finish:
                    await self.world.wait_frame(.02)
        elif op in ("game.jump", "game.walk"):
            await self._body_motion(op, args)
        elif op in ("motion.kick", "motion.pose", "motion.splits"):
            await self._body_motion(op, args)
        else:
            raise ValueError(f"Unsupported simulated move: {op}")
        self._refresh_imu()
        return True

    async def _call(self, worker, op, args=None):
        if worker != "detection":
            raise ValueError(f"Unsupported worker {worker}")
        if op == "football.pose":
            self.pose = args if args and args.get("active") else None
            self.tracker = SceneTracker()
            return {}
        if op == "football.decision":
            return {}
        if op != "football.status":
            raise ValueError(op)
        if self.pose is None:
            return {"valid": False}
        image, geometry, ball, point = self.observe_ball()
        projector = self.world.projector
        parts = goal_parts(image, self.p, projector, None, geometry,
                           bearing_only=self.pose.get("goal_bearing_only", False))
        scene = dict(valid=True, pose_id=self.pose["pose_id"],
                     sequence=self.world.sequence, received=time.monotonic(),
                     parameter_revision=0, ball=point, parts=parts,
                     height_m=geometry["height"], foot=(ball_at_foot(image, ball[0]["rect"])
                         if ball and self.pose.get("pan") == 0 and self.pose.get("tilt") == -2100 else None))
        self.scene = self.tracker.update(scene)
        return self.scene

    def observe_ball(self):
        image = self.world.render()
        projector = self.world.projector
        geometry = camera_geometry(self.p, self.world.pan, self.world.tilt, backend.Robot.camera)
        camera = self.world.data.camera("sim_head_camera")
        yaw = self.world.yaw
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy, _ = camera.xpos - self.world.data.qpos[:3]
        geometry.update(x=c*dx+s*dy, y=-s*dx+c*dy, height=float(camera.xpos[2]))
        heading = yaw + geometry["pan"]
        c, s = math.cos(heading), math.sin(heading)
        projector.camera_rotation = np.array([[c,s,0],[-s,c,0],[0,0,1]]) @ camera.xmat.reshape(3,3)
        ball = ball_candidates(image, self.p, projector, None, camera_height=geometry["height"])
        point = None
        if ball:
            try:
                point = plane_point(projector, ball[0]["center_px"], None, geometry,
                                    self.p["game.ball_diameter_m"]/2)
            except ValueError:
                pass
        return image, geometry, ball, point


class SimRobot(FakeRobot):
    def __init__(self, world, game, port, store=None):
        self.store = store
        self.generation = self.delay = 0
        self.play_after = 0.
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)
        self.Gst = Gst
        self.world, self.game = world, game
        self.calibration = world.calibration = CalibrationSession(world)
        self.repeat_fixture = self.switching_scene = False
        self.strategy_task = self.loop = None
        self.pipeline = self.appsrc = None
        self.motion_queue = queue.SimpleQueue()
        self.manual_task = None
        self.manual_job_id = None
        self.drive_sample = None
        super().__init__(port, robot_id="SIM-ROKI2")
        self.game_running = False
        self.mode = "IDLE"
        self.streams["camera"]["settings"]["fps"] = 15.

    def game_state(self):
        state = super().game_state()
        state.update(phase=self.game.info["phase"], reason=self.game.info["reason"],
                     decision=self.game.info["phase"], ball=self.game.info.get("ball"),
                     travel_m=self.game.odometry.path,
                     position=self.game.odometry.state(),
                     last_correction=self.game.info.get("last_correction"))
        return state

    def _require_control(self):
        if not self.client or self.owner != self.client[0]:
            raise Fault("no_control", "Acquire control")

    async def switch_scene(self, enabled):
        self._require_control()
        if type(enabled) is not bool:
            raise Fault("invalid_argument", "enabled must be boolean")
        if self.calibration.enabled == enabled:
            return self.calibration.state()
        self.switching_scene = True
        self.game_running = self.game_paused = self.game_pickup = self.game_ready = self.game_confirm = False
        self.game.revision += 1
        self.mode = "MANUAL"
        self.drive_sample = None
        tasks = [t for t in (self.strategy_task,self.manual_task) if t is not None]
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self.manual_task = self.manual_job_id = self.strategy_task = None
        while not self.motion_queue.empty():
            _, _, ident = self.motion_queue.get_nowait()
            if ident is not None:
                self.jobs[ident]["status"] = "cancelled"
        for job in self.jobs.values():
            if job.get("status") in ("running", "saving"):
                job["status"] = "cancelled"
        self.game.football_body = None
        self.game.info.update(phase="stopped", reason="")
        self.world.motions = MotionLibrary(self.game.p)
        self.calibration.scene(enabled)
        self.switching_scene = False
        return self.calibration.state()

    def _result(self, op, body):
        # The UDP transport lives on a worker thread. Scene mutation, job acceptance
        # and cancellation run on the same event loop as MuJoCo stepping.
        if (self.loop is not None and self.loop.is_running() and
                threading.current_thread() is self.thread):
            async def dispatch():
                if op == "sim.calibration.mode":
                    return await self.switch_scene(body.get("enabled"))
                return self._command(op, body)
            return asyncio.run_coroutine_threadsafe(dispatch(), self.loop).result()
        if op == "sim.calibration.mode":
            return asyncio.run(self.switch_scene(body.get("enabled")))
        return self._command(op, body)

    def _command(self, op, body):
        session = self.calibration
        if op == "sim.calibration.status":
            return session.state()
        if op == "sim.kick.preview":
            self._require_control()
            if not session.enabled or session.phase != "ready" or session.job_id or self.mode != "MANUAL":
                raise Fault("invalid_state", "Complete Rotation test and stop motion first")
            from sim_kick_lab import validate, preview
            args = validate(body)
            session.prepare(args)
            # A preview changes the physical fixture; the next Repeat must rebuild its original.
            session.fixture_args = None
            return preview(self, args)
        if op == "sim.calibration.save":
            self._require_control()
            return session.save(self,body)
        if op == "sim.calibration.repeat":
            self._require_control()
            if not session.enabled or not session.last_args or session.job_id is not None:
                raise Fault("invalid_state", "Complete a measurement before repeating")
            if session.last_args["name"] == "get_up_test" and session.phase == "rotation":
                session.phase = "stand"
            self._command("test.start", session.last_args)
            self.repeat_fixture = session.fixture_args == session.last_args
            return session.state()
        if session.enabled and (op == "game.start" or
                op.startswith("motion.") and op not in ("motion.state","motion.slots","motion.stop","motion.stop_graceful","motion.stop_hard")):
            raise Fault("busy", "Calibration owns the field")
        if op == "system.capabilities":
            result = super()._result(op, body)
            result["game"] = {"strategies": ["simple_football"], "entries": ["center", "left", "right"]}
            return result
        if op == "game.start":
            if body["strategy"] != "simple_football":
                raise Fault("not_supported", "Select simple_football")
            body = {"delay_seconds": 0, **body}
            result = super()._result(op, body)
            self.delay = body["delay_seconds"]
            self.play_after = time.monotonic() + self.delay
            self.generation += 1
            return result
        if op == "params.keys":
            prefix = body.get("prefix", "")
            if not isinstance(prefix, str):
                raise Fault("invalid_argument", "prefix must be a string")
            return page(sorted(k for k in SCHEMA if k.startswith(prefix)), body)
        if op == "params.describe":
            return self.store.describe(body["key"])
        if op == "params.get":
            if "keys" in body:
                keys = body["keys"]
                if ("key" in body or not isinstance(keys, list) or not 1 <= len(keys) <= 16
                        or not all(isinstance(k, str) for k in keys) or len(set(keys)) != len(keys)):
                    raise Fault("invalid_argument", "Expected 1..16 distinct keys")
                for key in keys:
                    self.store.describe(key)
                return {"values": {k: self.store.values[k] for k in keys}}
            key = body["key"]
            self.store.describe(key)
            return {"key": key, "value": self.store.values[key]}
        if op == "params.set":
            if not self.client or self.owner != self.client[0]:
                raise Fault("no_control", "Acquire control")
            values = body["values"] if "values" in body else {body["key"]: body["value"]}
            expected = body.get("expected_values", {})
            if "expected_value" in body:
                expected = {body["key"]: body["expected_value"]}
            if any(self.store.values[k] != v for k, v in expected.items()):
                raise Fault("conflict", "Reload changed parameters")
            saved = self.store.set_many(values)
            self.game.p.update(saved)
            if "values" in body:
                return {"values": saved}
            key = body["key"]
            return {"key": key, "value": saved[key], "apply": SCHEMA[key][4]}
        if op == "test.start":
            if not session.enabled:
                raise Fault("invalid_state", "Select calibration mode first")
            args = dict(body)
            session.check(args)
            if args.get("name") == "get_up_test":
                if args.pop("fixture", "upright") not in ("upright","front","back","left","right"):
                    raise Fault("invalid_argument", "Unknown get-up fixture")
            if args.get("name") == "kick_test":
                if "strength" in args:
                    from sim_kick_lab import validate
                    validate(args)
                    args.pop("strength")
                if args.pop("leg","right") not in ("right","left"):
                    raise Fault("invalid_argument", "Unknown striking foot")
                for key in ("ball_forward_mm","ball_side_mm"):
                    if key in args and (type(args[key]) is not int or abs(args.pop(key)) > 1500):
                        raise Fault("invalid_argument", "Ball offsets must be integer mm within 1500")
            parsed, _ = calibration.validate_args(args)
            if self.game_running or self.mode != "MANUAL" or any(
                    j["status"] in ("running", "saving") for j in self.jobs.values()):
                raise Fault("busy", "Stop motion before calibration")
            if not self.client or self.owner != self.client[0]:
                raise Fault("no_control", "Acquire control")
            ident = max((k for k in self.jobs if isinstance(k, int)), default=0) + 1
            description = calibration.describe(body["name"], parsed)
            if "progress_max" in description:
                parsed["progress_max"] = description["progress_max"]
            self.jobs[ident] = dict(job_id=ident, status="running", operation=op, progress=0, test=parsed)
            result = dict(accepted=True, job_id=ident, test=parsed, progress=0)
            session.job_id, session.last_args = ident, dict(body)
            session.result = None
            if args["name"] == "rotation_test":
                session.phase = "rotation"
            self.repeat_fixture = False
            self.motion_queue.put((op, dict(body), ident))
            return result
        if op == "test.measure":
            raise Fault("invalid_state", "Use sim.calibration.save for the current measurement")
        if op == "job.cancel":
            if not self.client or self.owner != self.client[0]:
                raise Fault("no_control", "Acquire control")
            job = self.jobs[body["job_id"]]
            if job["status"] in ("running", "saving"):
                job["status"] = "cancelled"
            return dict(job)
        if op == "test.list":
            return {"items": list(calibration.TITLES)}
        if op == "test.describe":
            return calibration.describe(body["name"], body)
        if op == "motion.slots":
            names = sorted(path.stem for path in (self.world.motions.ASSETS / "slots").glob("*.json"))
            start, limit = body.get("offset", 0), body.get("limit", 8)
            end = min(len(names), start + limit)
            return {"items": names[start:end], "total": len(names),
                    "next_offset": end if end < len(names) else None}
        result = super()._result(op, body)
        if op == "videostream.subscribe":
            self._start_video(body["rtp_port"], result["ssrc"])
        elif op in ("videostream.unsubscribe", "videostream.stop"):
            self._stop_video()
        elif op == "motion.head":
            self.motion_queue.put((op, body, None))
        elif op in ("motion.jump", "motion.kick", "motion.pose", "motion.slot",
                    "motion.splits", "motion.get_up"):
            self.motion_queue.put((op, body, result["job_id"]))
        return result

    def on_drive(self, body):
        if not self.calibration.enabled:
            self.drive_sample = body | {"received": time.monotonic()}

    async def _run_manual_motion(self, op, body, job_id):
        try:
            if op == "test.start":
                await run_calibration(self, self.jobs[job_id], body)
            elif op == "motion.get_up":
                await self.game._body_motion(op, body)
            else:
                steps = self.world.motions.steps(op, body)
                if op == "motion.kick":
                    self.world.kick_contact = KickContact(self.world, body.get("leg", "right"), body.get("soft", False))
                try:
                    await self.world.play_steps(steps)
                finally:
                    if op == "motion.kick":
                        self.world.kick_contact = None
            if job_id is not None:
                self.jobs[job_id]["status"] = "completed"
        except asyncio.CancelledError:
            if job_id is not None:
                self.jobs[job_id]["status"] = "cancelled"
            raise
        except Exception as exc:
            if job_id is not None:
                self.jobs[job_id].update(status="failed", error=str(exc))
            self.game.s.log("simulation", "ERROR", str(exc))

    def live_drive(self):
        sample = self.drive_sample
        if (self.mode == "MANUAL" and not self.calibration.enabled and not self.game_running and sample and
                time.monotonic()-sample["received"] < .35 and
                any(abs(sample.get(axis, 0)) > .05 for axis in ("x", "y", "yaw"))):
            return sample
        return None

    async def drive_cycles(self):
        motions = self.world.motions
        sample = self.live_drive()
        if sample is None:
            return
        engine = Engine(motions.parameters, backend=backend)
        engine.first_Leg_Is_Right_Leg = sample.get("y", 0) <= 0
        await self.world.play_steps(engine.walk_Initial_Pose(start_mixing=False))
        cycle = 0
        while (sample := self.live_drive()) is not None:
            speed = sample.get("speed", .5)
            p = motions.parameters
            if abs(sample.get("y", 0)) > .05:
                engine.first_Leg_Is_Right_Leg = sample["y"] < 0
            rotation = sample.get("yaw", 0)*p["walk.max_yaw_rad"]*speed
            frames = engine.walk_Cycle(sample.get("x", 0)*p["walk.max_step_mm"]*speed,
                                       abs(sample.get("y", 0))*p["walk.max_side_mm"]*speed,
                                       rotation,
                                       cycle, 1000000)
            await self.world.play_steps(frames)
            cycle += 1
        if cycle:
            await self.world.play_steps(engine.walk_Cycle(0, 0, 0, 0, 1))
        await self.world.play_steps(engine.walk_Final_Pose())
        motions.pose = "stand"

    def apply_manual(self, dt):
        if self.switching_scene:
            return
        if self.manual_task is not None and self.manual_task.done():
            if self.manual_task.cancelled() and self.calibration.job_id == self.manual_job_id:
                self.calibration.job_id = None
            if not self.manual_task.cancelled():
                self.manual_task.result()
            self.manual_task = None
            self.manual_job_id = None
        if self.manual_task is not None and (self.mode != "MANUAL" or
                self.manual_job_id is not None and
                self.jobs[self.manual_job_id]["status"] == "cancelled"):
            self.manual_task.cancel()
        while not self.motion_queue.empty():
            if self.manual_task is not None:
                break
            op, body, job_id = self.motion_queue.get_nowait()
            if job_id is not None and self.jobs[job_id]["status"] == "cancelled":
                if self.calibration.job_id == job_id:
                    self.calibration.job_id = None
                continue
            if op == "motion.head":
                self.world.command_head(body.get("pan", self.world.head_targets[0]),
                                        body.get("tilt", self.world.head_targets[1]),
                                        body.get("frames", 10))
                self.world.last_bgr = None
            else:
                self.manual_job_id = job_id
                self.manual_task = asyncio.get_running_loop().create_task(
                    self._run_manual_motion(op, body, job_id))
        if self.manual_task is None and self.live_drive():
            self.manual_task = asyncio.create_task(self.drive_cycles())

    def _start_video(self, port, ssrc):
        self._stop_video()
        launch = (
            "appsrc name=source is-live=true format=time do-timestamp=true "
            "caps=video/x-raw,format=RGB,width=800,height=650,framerate=15/1 "
            "! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast "
            "bitrate=2000 key-int-max=15 ! h264parse "
            f"! rtph264pay pt=96 config-interval=1 ssrc={ssrc} "
            f"! udpsink host=127.0.0.1 port={port} sync=false")
        self.pipeline = self.Gst.parse_launch(launch)
        self.appsrc = self.pipeline.get_by_name("source")
        self.pipeline.set_state(self.Gst.State.PLAYING)

    def _stop_video(self):
        if self.pipeline:
            self.pipeline.set_state(self.Gst.State.NULL)
        self.pipeline = self.appsrc = None

    def push_frame(self, bgr):
        if self.appsrc is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        payload = rgb.tobytes()
        buffer = self.Gst.Buffer.new_allocate(None, len(payload), None)
        buffer.fill(0, payload)
        self.appsrc.emit("push-buffer", buffer)

    def close(self):
        if self.manual_task is not None and not self.manual_task.done():
            self.manual_task.cancel()
        self._stop_video()
        super().close()


def trace_sample(robot, game, world):
    scene = game.scene or {}
    return dict(time_monotonic=time.monotonic(), mode=robot.mode,
                game_running=robot.game_running, phase=game.info.get("phase"),
                reason=game.info.get("reason"), decision=game.info.get("decision"),
                truth=dict(x_m=world.x, y_m=world.y, yaw_rad=world.yaw,
                           ball_xy_m=world.data.qpos[world.ball_qpos:world.ball_qpos+2].tolist()),
                odometry=game.odometry.state(), observed_ball=scene.get("ball"),
                last_correction=game.info.get("last_correction"),
                observed_goals=[dict(kind=part["kind"], colour=part["colour"],
                                     points=part["points"]) for part in scene.get("parts", [])],
                frame_sequence=scene.get("sequence"))


async def run(port, steps=None, viewer=True, settings_path=SETTINGS, trace_path=None, blue_attack=False, state_dir=None, visual_dribble=False):
    settings = load_settings(settings_path)
    if blue_attack:
        goal_x = settings["layout"]["blue_goal_x_m"]
        facing = 1 if goal_x > 0 else -1
        settings["layout"].update(robot_spawn_xy=[-.60*facing, 0.], robot_yaw_deg=0 if facing > 0 else 180,
                                  ball_spawn_xy=[0., 0.])
        settings["viewer"].update(distance_m=2.45, elevation_deg=-24,
                                  lookat_xyz=[.45*goal_x, 0., .14])
    parameters = sim_parameters(settings)
    if blue_attack:
        parameters["match.own_goal"] = 0
    state_dir = Path(state_dir) if state_dir else HERE.parents[1] / "sim/.local-state"
    state_dir.mkdir(parents=True, exist_ok=True)
    if not (state_dir / "parameters.json").exists():
        (state_dir / "parameters.json").write_text(json.dumps(parameters))
    store = Parameters(state_dir, robot_model="roki2")
    parameters = dict(store.values)
    world = World(settings=settings, parameters=parameters)
    game = GameAdapter(world, parameters)
    robot = SimRobot(world, game, port, store)
    robot.loop = asyncio.get_running_loop()
    strategy = None
    generation = -1
    if blue_attack:
        # The first touch has already been played in this open-play scenario.
        robot.game_role, robot.game_entry = "simple_football", "center"
        robot.mode = "GAME"
        robot.game_running = True
    trace_file = None
    if trace_path:
        trace_path = Path(trace_path)
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = trace_path.open("w")
        trace_file.write(json.dumps(dict(type="config", settings=settings), ensure_ascii=False)+"\n")
    window = mujoco.viewer.launch_passive(world.model, world.data) if viewer else None
    if window is not None:
        camera = settings["viewer"]
        window.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        window.cam.azimuth = camera["azimuth_deg"]
        window.cam.elevation = camera["elevation_deg"]
        window.cam.distance = camera["distance_m"]
        window.cam.lookat[:] = camera["lookat_xyz"]
    print(f"Roki2 simulator: FPV Roki -> 127.0.0.1:{robot.port}", flush=True)
    active = None
    last_video = 0.
    count = 0
    try:
        while (window is None or window.is_running()) and (steps is None or count < steps):
            if not world.motion_lock.locked() and not robot.calibration.enabled:
                world.advance(.025)
            robot.apply_manual(.025)
            if robot.game_running and generation != robot.generation:
                if active:
                    active.cancel()
                    await asyncio.gather(active, return_exceptions=True)
                    active = None
                generation = robot.generation
                if generation > 0:
                    layout = settings["layout"]
                    pose = GameOdometry(parameters, robot.game_entry, 0.)
                    xy = (pose.x, pose.y) if robot.game_entry != "center" else (
                        -math.cos(pose.home_heading) * math.hypot(*layout["robot_spawn_xy"]), 0.)
                    yaw = pose.yaw
                    mujoco.mj_resetData(world.model, world.data)
                    world.referee.reset()
                    world.place_opponent(xy, yaw)
                    world.data.qpos[:3] = [*xy, .29]
                    world.data.qpos[3:7] = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
                    world.data.qpos[world.ball_qpos:world.ball_qpos+3] = [*layout["ball_spawn_xy"], world.model.geom_size[world.ball_geom, 0]]
                    world.head_ramp = world.last_bgr = None
                    world.pan, world.tilt = 0, -1500
                    world.head_targets = (world.pan, world.tilt)
                    world.data.joint("head_pitch_joint").qpos[0] = world.tilt / 1698
                    world.data.ctrl[world.model.actuator("head_pitch").id] = world.tilt / 1698
                    world.motions = MotionLibrary(parameters)
                    mujoco.mj_forward(world.model, world.data)
                    world._put_pose()
                    game.__init__(world, parameters)
                strategy_type = VisualDribble if visual_dribble else SimpleFootball
                strategy = strategy_type(game, parameters, robot.game_entry, robot.delay)
                if blue_attack and generation == 0:
                    strategy.opening_touch_required = strategy.kickoff_pending = False
            if robot.game_running and not robot.game_paused and time.monotonic() >= robot.play_after:
                if active is None or active.done():
                    if active is not None:
                        try:
                            active.result()
                        except Fault as exc:
                            game.info.update(phase="stopped", reason=str(exc))
                            robot.game_running = False
                            active = None
                            continue
                        if trace_file:
                            trace_file.write(json.dumps(trace_sample(robot, game, world),
                                                        ensure_ascii=False)+"\n")
                            trace_file.flush()
                    active = robot.strategy_task = asyncio.create_task(game.play_step(strategy))
            elif active is not None:
                game.revision += 1
                active.cancel()
                await asyncio.gather(active, return_exceptions=True)
                active = robot.strategy_task = None
            now = time.monotonic()
            if now-last_video >= 1/15 and (robot.appsrc is not None or not robot.calibration.enabled):
                frame = world.render()
                robot.push_frame(frame)
                last_video = now
            if window is not None:
                robot.calibration.draw(window)
                window.sync()
            if trace_file and count % 20 == 0:
                trace_file.write(json.dumps(trace_sample(robot, game, world),
                                            ensure_ascii=False)+"\n")
            count += 1
            await asyncio.sleep(.025)
    finally:
        if active:
            active.cancel()
        robot.close()
        if trace_file:
            trace_file.close()
        if window:
            window.close()


def main():
    if sys.platform == "darwin":
        from operator_gui.mac_instance import instance
        with instance("simulator") as available:
            if available:
                run_cli()
    else:
        run_cli()


def run_cli():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8094)
    parser.add_argument("--steps", type=int)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--visual-dribble", action="store_true", help="Compare experimental camera-guided walking touches in MuJoCo")
    parser.add_argument("--blue-attack", action="store_true", help="Start Simple Football in open play facing the blue goal")
    parser.add_argument("--state-dir", type=Path, help="Separate persistent simulator parameter directory")
    parser.add_argument("--scene-settings", type=Path, default=SETTINGS)
    parser.add_argument("--trace", type=Path,
                        default=WORKSPACE / "roki-ng-operator-gui/sim/latest_trace.jsonl")
    args = parser.parse_args()
    asyncio.run(run(args.port, args.steps, viewer=not args.headless,
                    settings_path=args.scene_settings, trace_path=args.trace, blue_attack=args.blue_attack, state_dir=args.state_dir, visual_dribble=args.visual_dribble))


if __name__ == "__main__":
    main()
