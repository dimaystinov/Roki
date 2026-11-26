import sys
import unittest
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "roki-ng"))
sys.path.insert(0, str(ROOT / "roki-ng-operator-gui/scripts"))
sys.path.insert(0, str(ROOT / "Bioloid-main/Bioloid/sim/roki"))
sys.path.insert(0, str(ROOT / "Bioloid-main/Elsiros/controllers/Robofest_TEAM"))

from roki_ng.body.motion.backends.roki2 import backend
from roki_ng.body.motion.backends.roki2.model import Robot
from roki_ng.body.motion.engine import Engine
from roki_ng.parameters import SCHEMA
from sim_motion import ServoBridge, MotionLibrary


SCENE = (ROOT / "Bioloid-main/Bioloid/sim/roki_lab/rokilab-main"
         / "resources/assets/scene_roki2_soccer.xml")


class SimMotionTests(unittest.TestCase):
    def test_every_physical_servo_has_mujoco_actuator(self):
        model = mujoco.MjModel.from_xml_path(str(SCENE))
        bridge = ServoBridge(model, mujoco.MjData(model))
        addresses = {(row[0], row[1]) for row in Robot().ACTIVESERVOS}
        addresses.update({(13, 1), (13, 2)})
        self.assertEqual(set(bridge.addresses), addresses)

    def test_servo_command_moves_joint_through_physics(self):
        model = mujoco.MjModel.from_xml_path(str(SCENE))
        data = mujoco.MjData(model)
        bridge = ServoBridge(model, data)
        from roki_ng.body.motion.gait import Servo
        bridge.set_targets([Servo(0, 1, 8000), Servo(12, 2, 7000)])
        self.assertNotEqual(float(data.ctrl[model.actuator("head_yaw").id]), 0.)
        for _ in range(100):
            mujoco.mj_step(model, data)
        self.assertGreater(abs(float(data.joint("head_yaw_joint").qpos[0])), .05)
        self.assertGreater(abs(float(data.joint("head_pitch_joint").qpos[0])), .05)

    def test_real_crouch_generator_stays_upright(self):
        model = mujoco.MjModel.from_xml_path(str(SCENE))
        data = mujoco.MjData(model)
        data.qpos[:3] = [-.6, 0, .29]
        data.qpos[3] = 1
        bridge = ServoBridge(model, data)
        engine = Engine({key: spec[1] for key, spec in SCHEMA.items()}, backend=backend)
        for command in engine.walk_Initial_Pose(start_mixing=False):
            if command[0] != "servo":
                continue
            _, values, frames, _ = command
            before = data.ctrl.copy()
            bridge.set_targets(values)
            target = data.ctrl.copy()
            for frame in range(frames):
                data.ctrl[:] = before + (target - before) * (frame + 1) / frames
                for _ in range(round(.02 / model.opt.timestep)):
                    mujoco.mj_step(model, data)
        self.assertGreater(float(data.qpos[2]), .25)

    def test_short_walk_on_free_base_reaches_requested_distance(self):
        import asyncio
        import sim_roki2 as sim
        settings = sim.load_settings(sim.SETTINGS)
        # Mirroring the origin places the other forward inside the test robot.
        settings["opponents"]["enabled"] = False
        settings["layout"].update(robot_spawn_xy=[0., 0.], robot_yaw_deg=0., ball_spawn_xy=[1., 1.])
        parameters = sim.sim_parameters(settings)
        world = sim.World(settings=settings, parameters=parameters)
        game = sim.GameAdapter(world, parameters)
        advance = world.advance

        def step(dt):
            advance(dt)
            self.assertGreater(world.data.qpos[2], .21)

        try:
            for _ in range(20):
                world.advance(.02)
            start = world.x, world.y
            world.advance = step
            asyncio.run(game._move("game.walk", dict(kind="omni", distance_m=.12, course=0., heading=0.)))
            self.assertAlmostEqual(world.x-start[0], .12, delta=.035)
            self.assertLess(abs(world.y-start[1]), .025)
        finally:
            world.renderer.close()

    def test_production_motion_sources_cover_operator_commands(self):
        library = MotionLibrary({key: spec[1] for key, spec in SCHEMA.items()})
        commands = [
            ("game.walk", dict(kind="omni", distance_m=.1, course=0., heading=0.)),
            ("game.jump", dict(direction="forward", fraction=1.)),
            ("motion.kick", dict(leg="right", power=60, offset=0)),
            ("motion.pose", dict(name="base_stand")),
            ("motion.slot", dict(name="Initial_Pose")),
            ("motion.splits", dict(kind="small")),
            ("motion.get_up", dict(position="upright")),
        ]
        for operation, args in commands:
            with self.subTest(operation=operation):
                steps = library.steps(operation, args)
                self.assertTrue(any(step[0] == "servo" for step in steps))
                self.assertTrue(all(step[2] > 0 for step in steps if step[0] == "servo"))

    def test_stand_from_unknown_uses_robot_base_pose(self):
        library = MotionLibrary({key: spec[1] for key, spec in SCHEMA.items()})
        expected = MotionLibrary({key: spec[1] for key, spec in SCHEMA.items()}).steps(
            "motion.pose", {"name": "base_stand"})
        actual = library.steps("motion.pose", {"name": "stand"})
        self.assertEqual([(x[0], x[2]) for x in actual if x[0] == "servo"],
                         [(x[0], x[2]) for x in expected if x[0] == "servo"])
        self.assertEqual(library.pose, "base_stand")


if __name__ == "__main__":
    unittest.main()
