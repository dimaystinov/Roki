import asyncio
import importlib.util
import math
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "scripts/sim_roki2.py"
spec = importlib.util.spec_from_file_location("sim_roki2", SOURCE)
sim = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sim)


class SimRoki2Tests(unittest.TestCase):
    def test_body_imu_posture_and_upright_heading(self):
        from roki_ng.body.body_imu import quaternion_yaw
        world = sim.World()
        self.addCleanup(world.renderer.close)
        game = sim.GameAdapter(world, sim.sim_parameters())
        world.settings["imu"].update(noise_std_deg=0., yaw_drift_deg_per_min=0.)
        half = math.sqrt(.5)
        poses = {"upright": (1., 0., 0., 0.), "stomach": (half, 0., half, 0.),
                 "back": (half, 0., -half, 0.), "right": (half, half, 0., 0.),
                 "left": (half, -half, 0., 0.)}
        for yaw in (0., 1.1, -2.8):
            heading = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
            for expected, pose in poses.items():
                with self.subTest(yaw=yaw, expected=expected):
                    sim.mujoco.mju_mulQuat(world.data.qpos[3:7], heading, pose)
                    game._refresh_imu()
                    q = game.s.body_telemetry["body.imu"]["quaternion_xyzw"]
                    self.assertEqual(sim.body_position(q), expected)
                    if expected == "upright":
                        self.assertAlmostEqual(sim.wrap(quaternion_yaw(q) - yaw), 0.)

    def test_imu_drift_uses_sim_time_and_same_sample_for_repeated_reads(self):
        world = sim.World()
        self.addCleanup(world.renderer.close)
        game = sim.GameAdapter(world, sim.sim_parameters(), imu_seed=7)
        world.data.time = 300.
        game._refresh_imu()
        sample = game.s.body_telemetry["body.imu"]["quaternion_xyzw"]
        game._refresh_imu()
        self.assertEqual(game.s.body_telemetry["body.imu"]["quaternion_xyzw"], sample)
        world.data.time += .02
        game._refresh_imu()
        self.assertNotEqual(game.s.body_telemetry["body.imu"]["quaternion_xyzw"], sample)
        world.settings["imu"]["noise_std_deg"] = 0.
        measured, fallen = asyncio.run(game._imu())
        self.assertFalse(fallen)
        self.assertAlmostEqual(sim.wrap(measured - world.yaw), math.radians(30.002))

    def test_match_retries_fallen_player_without_resuming_strategy(self):
        from unittest.mock import AsyncMock, Mock
        game = object.__new__(sim.GameAdapter)
        game.info, game.odometry, game.s = {}, Mock(), Mock()
        game.world, game.football_body = Mock(), Mock()
        game.world.wait_frame = AsyncMock()
        fallen = sim.Fault("fallen", "lying on right side")
        game._imu = AsyncMock(side_effect=[fallen, fallen, fallen, (0., False), (0., False)])
        failed = sim.Fault("get_up_failed", "still on right")
        game._body_motion = AsyncMock(side_effect=[failed, {}, failed, {}, {}])
        strategy = Mock()
        strategy.step = AsyncMock()
        for _ in range(3):
            asyncio.run(game.play_step(strategy))
            strategy.step.assert_not_called()
        self.assertEqual(game.info["phase"], "starting")
        self.assertEqual(game.recovery_attempt, 0)
        asyncio.run(game.play_step(strategy))
        strategy.step.assert_awaited_once()

    def test_recovery_holds_stance_after_floor_contact(self):
        import json
        from tempfile import TemporaryDirectory
        from measure_odometry import measure
        with TemporaryDirectory() as directory:
            for case in ("get_up_front", "get_up_back", "get_up_left", "get_up_right"):
                with self.subTest(case=case):
                    asyncio.run(measure(case, Path(directory), sim.sim_parameters(), 40., 75.))
                    result = json.loads((Path(directory) / (case + ".json")).read_text())
                    self.assertIsNone(result["error"])
                    self.assertTrue(result["job"]["posture_verified"])
                    self.assertGreater(result["final_z_m"], .25)
                    self.assertGreater(result["final_up"], .95)

    def test_ball_spawn_uses_named_free_joint(self):
        settings = sim.load_settings(sim.SETTINGS)
        settings["layout"]["ball_spawn_xy"] = [.4, .2]
        world = sim.World(settings=settings)
        address = world.model.jnt_qposadr[world.model.joint("soccer_ball_joint").id]
        expected = world.settings["layout"]["ball_spawn_xy"]
        self.assertAlmostEqual(world.data.qpos[address], expected[0])
        self.assertAlmostEqual(world.data.qpos[address + 1], expected[1])
        self.assertAlmostEqual(world.data.qpos[address + 2], .045)

    def test_head_command_moves_model_joint(self):
        world = sim.World()
        world.command_head(600, -1500)
        for _ in range(16):
            world.advance(.025)
        self.assertGreater(world.data.joint("head_yaw_joint").qpos[0], .2)
        self.assertGreater(world.pan, 300)

    def test_game_jump_animates_real_servo_trajectory(self):
        world = sim.World()
        game = sim.GameAdapter(world, sim.sim_parameters())

        async def run():
            task = asyncio.create_task(game._move("game.jump", dict(direction="forward", fraction=1.)))
            maximum = 0.
            while not task.done():
                maximum = max(maximum,
                              abs(float(world.data.joint("left_ankle_pitch_joint").qpos[0])),
                              abs(float(world.data.joint("right_ankle_pitch_joint").qpos[0])))
                await asyncio.sleep(.02)
            await task
            return maximum

        self.assertGreater(asyncio.run(run()), .05)

    def test_centre_restart_holds_stance_while_verifying_single_pass(self):
        settings = sim.load_settings(sim.SETTINGS)
        settings["layout"].update(robot_spawn_xy=[.30, 0.], robot_yaw_deg=180.)
        parameters = sim.sim_parameters(settings)
        parameters.update({"match.own_goal": 0, "game.simple_football.kickoff_side": "left"})
        world = sim.World(settings=settings, parameters=parameters)
        game = sim.GameAdapter(world, parameters)
        strategy = sim.SimpleFootball(game, parameters, "center", 0)
        first_contact = []
        move, advance = game._move, world.advance
        active = None
        ball = world.model.geom("ball").id
        root = world.model.jnt_bodyid[0]

        async def track_move(op, args, **kwargs):
            nonlocal active
            active = op
            return await move(op, args, **kwargs)

        def track_contact(dt):
            advance(dt)
            for contact in world.data.contact:
                if ball not in (contact.geom1, contact.geom2):
                    continue
                other = contact.geom2 if contact.geom1 == ball else contact.geom1
                body_id = world.model.geom_bodyid[other]
                if not first_contact and world.model.body_rootid[body_id] == root:
                    first_contact.append((active, body_id))

        game._move, world.advance = track_move, track_contact
        try:
            asyncio.run(strategy.placed_kickoff())
            right_foot = world.model.joint("right_ankle_roll_joint").bodyid.item()
            self.assertEqual(first_contact, [("motion.kick", right_foot)])
            jobs = list(game.football_body.jobs.values())
            kicks = [j["status"] for j in jobs if j["operation"] == "motion.kick"]
            self.assertEqual(kicks, ["completed"])
            self.assertEqual(game.info["phase"], "simple_kickoff_verify")
            for _ in range(100):
                world.advance(.02)
                game._refresh_imu()
                quaternion = game.s.body_telemetry["body.imu"]["quaternion_xyzw"]
                self.assertEqual(sim.body_position(quaternion), "upright")
                self.assertGreater(world.data.qpos[2], .17)
        finally:
            world.renderer.close()

    def test_rendered_camera_feeds_production_ball_and_goal_vision(self):
        world = sim.World()
        parameters = sim.sim_parameters()
        game = sim.GameAdapter(world, parameters)

        async def observe():
            await game._call("detection", "football.pose", {"active": True, "pose_id": 1})
            await game._call("detection", "football.status")
            ball_scene = await game._call("detection", "football.status")
            world.tilt = 0
            await game._call("detection", "football.pose", {"active": True, "pose_id": 2})
            await game._call("detection", "football.status")
            goal_scene = await game._call("detection", "football.status")
            return ball_scene, goal_scene

        ball, goal = asyncio.run(observe())
        self.assertIsNotNone(ball["ball"])
        self.assertAlmostEqual(ball["ball"][0], .60, delta=.07)
        self.assertTrue(any(part["kind"] == "colour" and part["colour"] == "blue"
                            for part in goal["parts"]))
        self.assertFalse(any(part["colour"] == "blue" for part in ball["parts"]))

    def test_simple_football_moves_robot_and_odometry_together(self):
        world = sim.World()
        parameters = sim.sim_parameters()
        game = sim.GameAdapter(world, parameters)
        strategy = sim.SimpleFootball(game, parameters, "center", 1)
        strategy.entry = "left"
        strategy.entry_pending = True
        origin = world.x, world.y
        asyncio.run(strategy.step())
        self.assertEqual(game.info["phase"], "simple_entry")
        self.assertGreater(math.dist(origin, (world.x, world.y)), .001)
        expected = sim.GameOdometry(parameters, "center", game.odometry.home_heading)
        expected.x, expected.y, expected.yaw_offset = *origin, 0.
        for ident, job in game.football_body.jobs.items():
            expected.apply(ident, job)
        self.assertGreater(expected.path, 0.)
        self.assertAlmostEqual(game.odometry.x, expected.x)
        self.assertAlmostEqual(game.odometry.y, expected.y)

    def test_manual_fpv_drive_and_head_change_virtual_robot(self):
        world = sim.World()
        parameters = sim.sim_parameters()
        game = sim.GameAdapter(world, parameters)
        robot = sim.SimRobot(world, game, 0)
        try:
            robot.mode = "MANUAL"
            async def drive_then_head():
                robot.on_drive({"x": 1., "y": 0., "yaw": 0.})
                robot.apply_manual(.025)
                await robot.manual_task
                robot.motion_queue.put(("motion.head", {"pan": 500, "tilt": -1000}, None))
                robot.apply_manual(.025)
            asyncio.run(drive_then_head())
            self.assertEqual(world.head_targets, (500, -1000))
            for _ in range(16):
                world.advance(.025)
            self.assertGreater(world.pan, 300)
        finally:
            robot.close()

    def test_manual_jump_animates_joints_and_finishes_job(self):
        world = sim.World()
        game = sim.GameAdapter(world, sim.sim_parameters())
        robot = sim.SimRobot(world, game, 0)
        robot.mode = "MANUAL"

        async def run():
            robot.motion_queue.put(("motion.jump", {"direction": "forward", "fraction": 1.}, "test-job"))
            robot.jobs["test-job"] = {"status": "running"}
            robot.apply_manual(.025)
            maximum = 0.
            while robot.manual_task is not None and not robot.manual_task.done():
                maximum = max(maximum, abs(float(world.data.joint("left_ankle_pitch_joint").qpos[0])))
                await asyncio.sleep(.02)
            await robot.manual_task
            return maximum

        try:
            self.assertGreater(asyncio.run(run()), .05)
            self.assertEqual(robot.jobs["test-job"]["status"], "completed")
            self.assertGreater(world.x, -.6)
        finally:
            robot.close()

    def test_slot_catalog_lists_real_motion_assets(self):
        world = sim.World()
        robot = sim.SimRobot(world, sim.GameAdapter(world, sim.sim_parameters()), 0)
        try:
            catalog = robot._result("motion.slots", {"offset": 0, "limit": 1000})
            self.assertIn("Initial_Pose", catalog["items"])
            self.assertTrue(all((world.motions.ASSETS / "slots" / f"{name}.json").exists()
                                for name in catalog["items"]))
        finally:
            robot.close()


if __name__ == "__main__":
    unittest.main()
