"""MuJoCo match: mjpython scripts/watch_match.py (python ... --headless for replay)."""
import argparse
import asyncio
import copy
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

import cv2
import mujoco
import mujoco.viewer
import numpy as np

import sim_roki2 as sim
from roki_ng.supervisor.game.simple_goalkeeper import SimpleGoalkeeper
from sim_scene import opponent_material, opposing_start
from sim_referee import PitchReferee, painted_limits


from sim_spectators import add_spectators, move_spectators, spectator_ids


def match_model(spectators=False):
    scene = ET.parse(sim.SCENE)
    players = ET.parse(sim.SCENE.parent / scene.find("include").get("file"))
    world = scene.find("worldbody")
    if spectators:
        add_spectators(world)
    actuators = ET.SubElement(scene.getroot(), "actuator")
    for prefix in ("keeper_", "visitor_", "ally_keeper_"):
        for source, parent in [(players.find("worldbody/body[@name='torso_1']"), world),
                               (world.find("body[@name='sim_camera_mount']"), world),
                               *((servo, actuators) for servo in players.find("actuator"))]:
            node = copy.deepcopy(source)
            for element in node.iter():
                if prefix in ('keeper_', 'visitor_') and element.tag == 'geom' and 'material' in element.attrib:
                    element.set('material', opponent_material(element.get('mesh'), element.get('material')))
                for key in ("name", "joint"):
                    if key in element.attrib:
                        element.set(key, prefix + element.get(key))
            parent.append(node)
    # Preserve relative paths to the existing shared CAD and textures.
    with tempfile.NamedTemporaryFile(dir=sim.SCENE.parent, suffix=".xml") as xml:
        scene.write(xml, encoding="utf-8")
        xml.flush()
        return mujoco.MjModel.from_xml_path(xml.name)


class PlayerData:
    def __init__(self, data, model, prefix, motors):
        self.shared = data
        self.prefix = prefix
        root = model.body(prefix+"torso_1")
        start = model.jnt_qposadr[root.jntadr.item()]
        self.qpos = data.qpos[start:start+7]
        cam = model.body(prefix+"sim_camera_mount").mocapid.item()
        self.mocap_pos = data.mocap_pos[cam:cam+1]
        self.mocap_quat = data.mocap_quat[cam:cam+1]
        self.ctrl = data.ctrl[motors]

    @property
    def time(self):
        return self.shared.time

    def joint(self, name):
        return self.shared.joint(self.prefix+name)

    def camera(self, name):
        return self.shared.camera(self.prefix+name)


class PlayerWorld(sim.World):
    def __init__(self, match, prefix, xy, quat, p):
        self.match, self.prefix = match, prefix
        self.settings = match.settings
        m = match.model
        # The copied actuator block is contiguous and in the original bus order.
        roots = m.body_rootid[m.jnt_bodyid[m.actuator_trnid[:, 0]]]
        owned = np.flatnonzero(roots == m.body(prefix+"torso_1").id)
        first, last = int(owned[0]), int(owned[-1])
        if len(owned) != last-first+1:
            raise ValueError(f"Noncontiguous servo block for {prefix!r}")
        motors = slice(first, last+1)
        self.model = SimpleNamespace(nu=last-first+1, actuator_ctrlrange=m.actuator_ctrlrange[motors],
                                     actuator=lambda name: SimpleNamespace(id=m.actuator(prefix+name).id-first),
                                     joint=lambda name: m.joint(prefix+name))
        self.data = PlayerData(match.data, m, prefix, motors)
        self.data.qpos[:] = [*xy, .29, *quat]
        self.servos = sim.ServoBridge(self.model, self.data)
        self.motions = sim.MotionLibrary(p)
        self.motion_lock = asyncio.Lock()
        self.imu_callback = self.kick_contact = self.head_ramp = self.last_bgr = None
        self.projector = sim.PinholeProjector(self.settings["camera"]["fovy_deg"])
        self.camera_forward = self.settings["camera"]["forward_m"]
        self.head_targets = (0, -1500)
        self.pan, self.tilt = self.head_targets
        self.data.joint("head_pitch_joint").qpos[0] = self.tilt/1698
        self.data.ctrl[self.model.actuator("head_pitch").id] = self.tilt/1698
        self.head_shell = np.flatnonzero(m.geom_bodyid == m.body(prefix+"head_1").id)
        self.sequence = 0
        self._set_camera_pose()

    async def wait_frame(self, dt):
        await self.match.sleep(dt)

    def render(self):
        m = self.match.model
        hidden = self.head_shell
        if not self.prefix and self.match.mask is not None:
            extra = self.match.mask(self.match.data.time)
            hidden = np.concatenate((hidden, np.asarray(extra, dtype=int)))
        groups = m.geom_group[hidden].copy()
        try:
            m.geom_group[hidden] = 5
            self.match.renderer.update_scene(self.match.data, camera=self.prefix+"sim_head_camera",
                                             scene_option=self.match.options)
        finally:
            m.geom_group[hidden] = groups
        self.last_bgr = cv2.cvtColor(self.match.renderer.render(), cv2.COLOR_RGB2BGR)
        self.sequence += 1
        return self.last_bgr


class MatchAdapter(sim.GameAdapter):
    async def _move(self, op, args, **kwargs):
        if op in ("game.walk", "game.jump", "motion.kick"):
            name = self.world.prefix.rstrip("_") or "home"
            detail = args.get("direction") or args.get("leg") or args.get("kind")
            self.world.match.events.append((round(self.world.data.time, 2), name, op, detail))
        return await super()._move(op, args, **kwargs)

    async def _call(self, worker, op, args=None):
        if worker == "detection" and op == "ball.status":
            scene = self.scene or {}
            ball = scene.get("ball")
            if not ball or not scene.get("confirmed"):
                return {"running": True, "result": {"valid": False}}
            return {"running": True, "age_ms": (time.monotonic()-scene["received"])*1000,
                    "result": {"valid": True, "frame_sequence": scene["sequence"],
                               "x_m": float(ball[0]), "y_m": float(ball[1])}}
        result = await super()._call(worker, op, args)
        if (self.world.match.record and self.world.prefix == 'visitor_' and op == 'football.status'
                and result.get('confirmed') and result.get('ball')
                and result.get('pose_id') != getattr(self, 'logged_ball_pose', None)):
            self.logged_ball_pose = result['pose_id']
            self.world.match.events.append((round(self.world.data.time, 3), 'visitor', 'camera_ball',
                                            list(result['ball']), self.world.pan, self.world.tilt))
        return result


class Match:
    def __init__(self, *, side=None, keeper_y=0., start_y=0., yaw_offset=0., parameters=None, record=False, imu_seed=0, entry="center", spectators=False):
        self.settings = sim.load_settings(sim.SETTINGS)
        self.model = match_model(spectators)
        sim.apply_settings(self.model, self.settings)
        self.data = mujoco.MjData(self.model)
        self.spectators = spectator_ids(self.model) if spectators else []
        move_spectators(self.data, self.spectators, self.settings["layout"], 0.)
        self.ball_geom = self.model.geom("ball").id
        self.ball_qvel = self.model.jnt_dofadr[self.model.joint("soccer_ball_joint").id]
        self.ball = self.data.joint("soccer_ball_joint").qpos
        p = sim.sim_parameters(self.settings)
        profile = Path(__file__).resolve().parents[1]/"sim/.local-state/parameters.json"
        if profile.exists():
            p.update(json.loads(profile.read_text()))
        if parameters is not None:
            p.update(parameters)
        self.mask = None
        self.record = record
        self.trace = []
        self.next_sample = 0.
        radius = p["game.ball_diameter_m"]/2
        m = self.model
        m.geom_size[self.ball_geom, 0] = m.geom_rbound[self.ball_geom] = radius
        m.geom_aabb[self.ball_geom, 3:] = radius
        body = m.geom_bodyid[self.ball_geom]
        m.body_inertia[body] = .4*m.body_mass[body]*radius**2
        m.pair_friction[m.pair("ball_carpet").id, 3:] = 1.4*radius*sim.CARPET_MU
        mujoco.mj_setConst(m, self.data)
        self.ball[:3] = [0., 0., radius]
        self.renderer = mujoco.Renderer(m, height=650, width=800)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[5] = 0
        blue = self.settings["layout"]["blue_goal_x_m"]
        facing = math.copysign(1., blue)
        yaw = math.pi if facing < 0 else 0.
        yaw += math.radians(yaw_offset)
        xy = (-.30*facing, start_y)
        mirror, quat = opposing_start(xy, yaw)
        yellow = self.settings['layout']['yellow_goal_x_m']
        # Side entries are measured from the painted touchline, not the carpet edge.
        half_length, half_width = painted_limits(self.settings['layout'])
        p['field.geometry'] = p['field.geometry'] | {
            'length': 2*half_length,
            'width': 2*half_width,
        }
        if entry in ('left', 'right'):
            pose = sim.GameOdometry(p | {'match.own_goal': 0}, entry, 0.)
            xy, yaw = (pose.x, pose.y), pose.yaw
        self.restart_xy = tuple(self.ball[:2])
        self.ball_movement_logged = False
        self.players = []
        self.events = []
        self.contact_roots = {}
        self.last_touch = {}
        for prefix, start, rotation, role, delay, own in (
            ("", xy, [math.cos(yaw/2),0.,0.,math.sin(yaw/2)], sim.SimpleFootball, 0., 0),
            ("keeper_", (blue-.10*facing, keeper_y), quat, SimpleGoalkeeper, 0., 1),
            ("visitor_", mirror, quat, sim.SimpleFootball, 30., 1),
            ("ally_keeper_", (yellow+.10*facing, 0.), [0.,0.,0.,1.] if facing < 0 else [1.,0.,0.,0.], SimpleGoalkeeper, 0., 0),
        ):
            params = p | {"match.own_goal": own}
            if side is not None:
                params["game.simple_football.kickoff_side"] = side
            if role is SimpleGoalkeeper:
                # From the goal line the centre mark is ~1.7 m away.
                params["game.head_tilt"] = params["head.field_tilt"] + 350
            world = PlayerWorld(self, prefix, start, rotation, params)
            game = MatchAdapter(world, params, imu_seed=imu_seed*3+len(self.players))
            strategy = role(game, params, entry if not prefix else ("goalkeeper" if role is SimpleGoalkeeper else "center"), delay)
            if delay:
                strategy.opening_touch_required = False
                strategy.attack_axis = world.yaw
            self.players.append((world, game, strategy, delay))
            self.contact_roots[m.body(prefix+"torso_1").id] = (prefix.rstrip("_") or "home", world, game)
        mujoco.mj_forward(m, self.data)
        self.referee = PitchReferee(m, self.data, self.settings['layout'],
                                   {'torso_1': 'yellow', 'keeper_torso_1': 'blue', 'visitor_torso_1': 'blue', 'ally_keeper_torso_1': 'yellow'})
        self.waiters = []

    async def sleep(self, seconds):
        future = asyncio.get_running_loop().create_future()
        self.waiters.append((self.data.time+seconds, future))
        await future

    async def player(self, world, game, strategy, delay):
        name = world.prefix.rstrip("_") or "home"
        if delay:
            game.info.update(phase='start_later_watch')
            async def observe():
                while True:
                    if await strategy.watch_restart_ball():
                        return
                    await self.sleep(.02)
            watch = asyncio.create_task(observe())
            timeout = asyncio.create_task(self.sleep(delay))
            try:
                done, _ = await asyncio.wait((watch, timeout), return_when=asyncio.FIRST_COMPLETED)
                reason = 'timeout'
                if watch in done:
                    watch.result()
                    reason = 'camera'
                self.events.append((round(self.data.time,3), name, 'released', reason,
                                    (strategy.last_scene or {}).get('ball')))
            finally:
                for task in (watch, timeout):
                    task.cancel()
                await asyncio.gather(watch, timeout, return_exceptions=True)
        self.events.append((round(self.data.time, 2), name, "start"))
        print(f"MATCH {self.events[-1]}", flush=True)
        while True:
            try:
                await game.play_step(strategy)
            except sim.Fault as exc:
                self.events.append((round(self.data.time, 2), name, str(exc)))
                print(f"MATCH {self.events[-1]}", flush=True)
                return
            await self.sleep(.02)

    def advance(self):
        move_spectators(self.data, self.spectators, self.settings["layout"], self.data.time)
        dt = .02
        for world, *_ in self.players:
            world._advance_head(dt)
        for _ in range(round(dt/self.model.opt.timestep)):
            previous = tuple(self.ball[:3])
            mujoco.mj_step(self.model, self.data)
            if not self.ball_movement_logged:
                # The simulator referee can distinguish rolling from vertical settling.
                if math.dist(self.ball[:2], self.restart_xy) >= .001:
                    self.events.append((round(self.data.time, 3), 'referee', 'ball_moved', list(self.ball[:2])))
                    self.ball_movement_logged = True
            contacts = self.data.contact
            for index in np.flatnonzero((contacts.geom1 == self.ball_geom) | (contacts.geom2 == self.ball_geom)):
                hit = contacts[index]
                geom = hit.geom2 if hit.geom1 == self.ball_geom else hit.geom1
                self.referee.touch(geom)
                body = self.model.geom_bodyid[geom]
                actor = self.contact_roots.get(self.model.body_rootid[body])
                if actor:
                    name, world, game = actor
                    striking = world.kick_contact and world.kick_contact.foot == body
                    kind = 'shot_contact' if striking and not world.kick_contact.soft else 'touch'
                    key = (name, kind)
                    if self.data.time-self.last_touch.get(key,-1.) > .12:
                        self.events.append((round(self.data.time,3), name, kind, list(self.ball[:2])))
                    self.last_touch[key] = self.data.time
            for world, *_ in self.players:
                if world.kick_contact:
                    world.kick_contact.sample(self)
            event = self.referee.update(previous)
            if event:
                self.events.append((round(self.data.time,3), 'referee', *event))
                if event[0] == 'throw_in':
                    for world, *_ in self.players:
                        if world.kick_contact:
                            world.kick_contact.completed = True
                else:
                    break
        for world, *_ in self.players:
            world.pan = round(world.data.joint("head_yaw_joint").qpos[0]*1698)
            world.tilt = round(world.data.joint("head_pitch_joint").qpos[0]*1698)
            world._set_camera_pose()
            if world.imu_callback:
                world.imu_callback()
        mujoco.mj_forward(self.model, self.data)

    async def pickup(self, tasks, when, side):
        """Operator removes and replaces our forward. Other players and ball remain live."""
        await self.sleep(when)
        old, adapter, strategy, _ = self.players[0]
        tasks[0].cancel()
        await asyncio.gather(tasks[0], return_exceptions=True)
        self.events.append((round(self.data.time, 3), 'home', 'pickup', side))
        # Park off the carpet while the operator prepares the re-entry.
        old.data.qpos[:2] = [0., self.settings['layout']['field_width_m']/2 + .6]
        mujoco.mj_forward(self.model, self.data)
        await self.sleep(60. if side == 'neutral' else 1.)
        root = self.model.body('torso_1').id
        point = None
        if side == 'neutral':
            while point is None:
                point = self.referee.neutral_reentry(root)
                if point is None:
                    await self.sleep(.1)
        for joint in np.flatnonzero(self.model.body_rootid[self.model.jnt_bodyid] == root):
            q = self.data.joint(int(joint))
            qa = self.model.jnt_qposadr[joint]
            va = self.model.jnt_dofadr[joint]
            q.qpos[:] = self.model.qpos0[qa:qa+len(q.qpos)]
            q.qvel[:] = 0.
            self.data.qacc_warmstart[va:va+len(q.qvel)] = 0.
        old.data.ctrl[:] = 0.
        pose = sim.GameOdometry(adapter.p, side, 0.)
        if point is not None:
            pose.x, pose.y = point
            own = strategy.specs[adapter.p['match.own_goal']]
            pose.yaw = math.atan2(own['y']-pose.y, own['x']-pose.x)
        ball_before = self.ball.copy()
        world = PlayerWorld(self, '', (pose.x, pose.y),
                            [math.cos(pose.yaw/2), 0., 0., math.sin(pose.yaw/2)], adapter.p)
        mujoco.mj_forward(self.model, self.data)
        game = MatchAdapter(world, adapter.p, imu_seed=adapter.imu_seed)
        forward = sim.SimpleFootball(game, adapter.p, side, 0.)
        if point is not None:
            enemy = forward.specs[1-adapter.p['match.own_goal']]
            forward.attack_axis = math.atan2(enemy['y']-world.y, enemy['x']-world.x)
            forward.start_heading_pending = False
        self.players[0] = (world, game, forward, 0.)
        self.contact_roots[root] = ('home', world, game)
        self.events.append((round(self.data.time, 3), 'home', 'reentry', side,
                            [world.x, world.y], float(np.linalg.norm(self.ball-ball_before))))
        tasks[0] = asyncio.create_task(self.player(*self.players[0]))

    async def run(self, seconds, visible, *, pickup_at=None, reentry='neutral'):
        self.restart_xy = tuple(self.ball[:2])
        window = mujoco.viewer.launch_passive(self.model, self.data) if visible else None
        if window:
            window.cam.lookat[:] = [0., 0., .12]
            window.cam.distance, window.cam.elevation, window.cam.azimuth = 4.5,-35,90
        yield_loop = asyncio.sleep
        tasks = []
        try:
            with patch("time.monotonic", lambda: 100.+self.data.time), patch(
                    "time.monotonic_ns", lambda: round((100.+self.data.time)*1e9)), patch("asyncio.sleep", self.sleep):
                tasks = [asyncio.create_task(self.player(*p)) for p in self.players]
                if pickup_at is not None:
                    tasks.append(asyncio.create_task(self.pickup(tasks, pickup_at, reentry)))
                await yield_loop(0)
                while self.data.time < seconds and (not window or window.is_running()):
                    before = time.perf_counter()
                    self.advance()
                    if self.record and self.data.time >= self.next_sample:
                        sample = {"time": round(self.data.time,3), "ball": list(self.ball[:3]), "players": {}}
                        for world,game,strategy,_ in self.players:
                            name = world.prefix.rstrip("_") or 'home'
                            scene = strategy.last_scene or {}
                            sample["players"][name] = dict(phase=game.info.get("phase"),
                                torso_height_m=float(world.data.qpos[2]),
                                torso_up_z=float(1-2*sum(world.data.qpos[4:6]**2)),
                                recovery_attempt=getattr(game, 'recovery_attempt', 0),
                                imu_yaw_rad=game.measured_yaw, actual=[world.x,world.y,world.yaw], estimate=game.odometry.state(),
                                ball=scene.get("ball"), features=[p["kind"] for p in scene.get("parts",[])])
                        self.trace.append(sample)
                        self.next_sample += .25
                    for wake, future in self.waiters:
                        if wake <= self.data.time+1e-8 and not future.done():
                            future.set_result(None)
                    self.waiters = [(t,f) for t,f in self.waiters if not f.done()]
                    await yield_loop(0)
                    for task in tasks:
                        if task.done() and not task.cancelled():
                            task.result()
                    if window:
                        window.set_texts((None, None, f"Match {self.data.time:.1f}s",
                                          " | ".join(g.info["phase"] for _,g,_,d in self.players)))
                        window.sync()
                        time.sleep(max(0., .02-(time.perf_counter()-before)))
                    if self.referee.goal:
                        return self.referee.goal
                return "time" if self.data.time >= seconds else "closed"
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            if window:
                window.close()
            self.renderer.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spectators", action=argparse.BooleanOptionalAction, default=True, help="Moving spectators with bright shirts and orange shoes")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=120.)
    parser.add_argument("--entry", choices=("center", "left", "right"), default="center")
    parser.add_argument("--pickup-at", type=float)
    parser.add_argument("--reentry", choices=("neutral", "left", "right"), default="neutral",
                        help="Neutral uses the one-minute repair rule; sides are short training drills.")
    args = parser.parse_args()
    match = Match(entry=args.entry, spectators=args.spectators)
    result = asyncio.run(match.run(args.seconds, not args.headless, pickup_at=args.pickup_at, reentry=args.reentry))
    print("MATCH", json.dumps(dict(result=result, seconds=round(match.data.time, 2), events=match.events)))
