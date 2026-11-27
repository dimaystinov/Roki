import asyncio
import json
from pathlib import Path
import sys
from unittest.mock import patch

import msgpack
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import sim_roki2 as sim


@pytest.fixture
def simulator(tmp_path):
    parameters = sim.sim_parameters()
    (tmp_path / "parameters.json").write_text(json.dumps(parameters))
    store = sim.Parameters(tmp_path, robot_model="roki2")
    with patch.object(sim.mujoco, "Renderer"):
        world = sim.World(parameters=parameters)
    robot = sim.SimRobot(world, sim.GameAdapter(world, parameters), 0, store)
    robot.client = ("127.0.0.1", 10000)
    robot.owner = robot.client[0]

    async def advance(dt):
        world.advance(dt)

    world.wait_frame = advance
    try:
        yield robot
    finally:
        robot.close()


async def finish(robot):
    robot.apply_manual(.025)
    with patch("time.monotonic", lambda: 100 + robot.world.data.time), \
         patch("time.monotonic_ns", lambda: round((100 + robot.world.data.time) * 1e9)):
        await robot.manual_task
    state = robot._command("sim.calibration.status", {})
    assert not state["busy"]
    assert len(msgpack.packb(state)) < 1300
    return state["result"]


def test_exclusive_series_and_repeat_from_exact_center(simulator):
    robot = simulator
    run = lambda name, **args: robot._command("test.start", dict(name=name, **args))

    async def session():
        robot.loop = asyncio.get_running_loop()
        robot.localisation_enabled = True
        assert robot._result("data.snapshot", {"topic":"localisation.state"})["topic"] == "localisation.state"
        with pytest.raises(sim.Fault):
            run("run_test", mode="short")
        robot.owner = None
        assert not robot._command("sim.calibration.status", {})["enabled"]
        with pytest.raises(sim.Fault):
            await robot.switch_scene(True)
        robot.owner = robot.client[0]
        robot.mode = "MANUAL"
        robot._command("game.start", {"strategy": "simple_football"})
        robot.strategy_task = asyncio.create_task(asyncio.Event().wait())
        strategy = robot.strategy_task
        robot.motion_queue.put(("motion.head", dict(pan=500), None))
        await robot.switch_scene(True)
        assert strategy.cancelled()
        assert not robot.game_running and robot.motion_queue.empty()
        with pytest.raises(sim.Fault):
            robot._command("game.start", {"strategy": "simple_football"})
        run("get_up_test", fixture="back")
        stand = await finish(robot)
        assert stand["upright"] and robot.calibration.phase == "rotation"
        initial = dict(robot.game.p)
        ident = run("rotation_test")["job_id"]
        advance = robot.world.wait_frame
        entered = asyncio.Event()

        async def pause_frame(dt):
            await advance(dt)
            entered.set()
            await asyncio.Event().wait()

        robot.world.wait_frame = pause_frame
        robot.apply_manual(.025)
        await entered.wait()
        robot._command("job.cancel", dict(job_id=ident))
        robot.apply_manual(.025)
        await asyncio.gather(robot.manual_task, return_exceptions=True)
        assert robot.calibration.result["status"] == "cancelled"
        assert not robot.calibration.state()["busy"]
        robot.world.wait_frame = advance
        run("rotation_test")
        rotation = await finish(robot)
        assert robot.calibration.phase == "ready" and rotation["saved"]
        saved = dict(rotation["values"])
        assert saved and all(robot.game.p[k] == robot.store.values[k] == v for k,v in saved.items())
        disk = json.loads(robot.store.path.read_text())
        assert all(disk[k] == v for k,v in saved.items())
        observed_parameters = []

        async def check_walking_profile(dt):
            if robot.game.football_body is not None and not observed_parameters:
                observed_parameters.append({k: robot.game.football_body.parameters[k] for k in saved})
            await advance(dt)

        robot.world.wait_frame = check_walking_profile
        run("run_test", mode="short")
        first = await finish(robot)
        assert observed_parameters == [saved]
        robot.world.wait_frame = advance
        assert robot.calibration.points[0] == (0., 0.)
        assert first["upright"] and first["odometry"]["dx_mm"] > 200
        robot.world.data.qpos[0] = .8
        robot.world.data.qvel[0] = 1.
        robot._command("sim.calibration.repeat", {})
        second = await finish(robot)
        assert robot.calibration.points[0] == (0., 0.)
        assert first["job_id"] != second["job_id"]
        for key in ("dx_mm","dy_mm","yaw_deg","path_mm","odometry"):
            assert first[key] == second[key]
        with pytest.raises(sim.Fault):
            robot._command("sim.calibration.save", dict(job_id=first["job_id"], values=first["values"]))
        await robot.switch_scene(False)
        assert not robot.game_running and robot.mode == "MANUAL"
        robot._command("game.start", {"strategy": "simple_football"})
        assert robot.game_running

    asyncio.run(session())


def test_kick_measures_selected_foot_and_miss(simulator):
    robot = simulator

    async def session():
        await robot.switch_scene(True)
        robot.calibration.phase = "ready"
        for offset, contact in [(100, True), (650, False)]:
            robot._command("test.start", dict(name="kick_test", leg="left", ball_forward_mm=offset))
            result = await finish(robot)
            assert result["status"] == "completed"
            assert result["ball_contact"] is contact
            assert (result["ball_distance_mm"] > 100) == contact
            assert (result["ball_path_mm"] > 100) == contact
            assert robot.calibration.points[0] == (0., 0.)
        # A mechanically successful job can end with a fallen model.
        robot.world.data.qpos[2] = .08
        robot.calibration.sample()
        robot.calibration.finish(robot, {"job_id": 100}, {"name":"run_test"}, "completed")
        assert robot.calibration.result["fallen"] and not robot.calibration.result["upright"]
        assert not robot.calibration.result["values"]

    asyncio.run(session())


def test_viewer_wall_time_does_not_break_imu_continuity(simulator):
    world = simulator.world
    imu = sim.SimBodyImu(world)
    from types import SimpleNamespace
    hardware = SimpleNamespace(body_quaternion=lambda: [0., 0., 0., 1.])
    with patch("time.monotonic", side_effect=[100., 100., 103., 103., 106., 106.]):
        imu.poll(hardware)
        epoch = imu.yaw_epoch
        world.data.time += .02
        imu.poll(hardware)
        assert imu.yaw_epoch == epoch
        world.data.time += .2
        imu.poll(hardware)
        assert imu.yaw_epoch == epoch + 1


def test_standard_test_measure_saves_operator_input_idempotently(simulator):
    robot = simulator
    key = "motion.run_10_mm"
    robot.jobs[7] = dict(job_id=7, operation="test.start", status="completed",
                        manual_parameters=[key], run_command_mm=640.)
    body = dict(job_id=7, values={key: 330.})
    expected = {key: 330., "motion.run_10_command_mm": 640.}
    for _ in range(2):
        assert robot._result("test.measure", body) == dict(job_id=7, saved=expected)
    assert all(robot.store.values[k] == robot.game.p[k] == v for k,v in expected.items())
    robot.jobs[7]["status"] = "failed"
    with pytest.raises(sim.Fault):
        robot._result("test.measure", body)


def test_colour_diagnostics_use_rendered_frame_and_saved_profile(simulator):
    import numpy as np

    robot = simulator
    robot.mode = "MANUAL"
    profiles = robot._result("detection.list", {})["profiles"]
    assert {"orange_ball", "green_field"} <= set(profiles)
    with pytest.raises(sim.Fault, match="Start runtime camera"):
        robot._result("detection.start", {})
    robot._result("camera.start", {})
    robot._result("detection.start", {"profile": "orange_ball"})
    frame = np.full((650, 800, 3), (0, 120, 255), np.uint8)
    robot.push_frame(frame)
    status = robot._result("detection.status", {})
    assert status["frames"] == 1 and status["result"]["profile"] == "orange_ball"
    assert not status["video"]["requested"]
    robot._result("detection.select", {"profile": "green_field"})
    output = robot.diagnostic.process(frame, True)
    assert output.shape == frame.shape and not np.array_equal(output, frame)
    assert robot.diagnostic.result["profile"] == "green_field"
    key = "vision.green_field.pixels_min"
    new_min = robot.store.values[key] + 1
    robot._result("params.set", {"values": {key: new_min}})
    assert robot.diagnostic.result is None
    assert robot._result("params.get", {"keys": [key]})["values"][key] == new_min
    robot._result("camera.stop", {})
    assert not robot._result("detection.status", {})["running"]
    assert not robot.video_availability()["detection"]
