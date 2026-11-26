import asyncio
import math
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import sim_roki2 as sim


@pytest.mark.parametrize("recovery, pitch_range", [(-.04, (-5., -2.)), (.04, (1., 3.))])
def test_regular_kick_recovers_at_selected_pitch(recovery, pitch_range):
    settings = sim.load_settings(sim.SETTINGS)
    settings["opponents"]["enabled"] = settings["person"]["enabled"] = False
    settings["layout"].update(robot_spawn_xy=[0., 0.], robot_yaw_deg=0.,
                              ball_spawn_xy=[1.3, 1.])
    params = sim.sim_parameters(settings)
    params.update({"kick.body_tilt": .09, "kick.recovery_tilt": recovery})
    with patch.object(sim.mujoco, "Renderer"):
        world = sim.World(settings=settings, parameters=params)
    game = sim.GameAdapter(world, params)

    async def advance(dt):
        world.advance(dt)
        w, x, y, z = world.data.qpos[3:7]
        assert world.data.qpos[2] > .17 and 1 - 2*(x*x + y*y) > .5
        return math.degrees(math.asin(2*(w*y - x*z)))

    async def run():
        for _ in range(20):
            await advance(.02)
        world.wait_frame = advance
        job = await game._body_motion("test.start", dict(name="kick_test", mode="regular"))
        assert job["status"] == "completed"
        pitches = [await advance(.02) for _ in range(100)]
        assert pitch_range[0] < pitches[-1] < pitch_range[1]
        assert max(pitches[-25:]) - min(pitches[-25:]) < .2

    with patch("time.monotonic", lambda: 100 + world.data.time), \
            patch("time.monotonic_ns", lambda: round((100 + world.data.time) * 1e9)):
        asyncio.run(run())
