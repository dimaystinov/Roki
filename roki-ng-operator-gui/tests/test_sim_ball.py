import asyncio
import math
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import sim_roki2 as sim


@pytest.mark.parametrize("soft, forward, lower, upper", [
    (True, .11, .45, .55), (True, .12, .45, .55), (True, .13, .45, .55),
    (False, .11, 2.8, 3.3), (False, .12, 2.8, 3.3), (False, .13, 2.8, 3.3),
    (False, .5, 0., .001),  # A commanded kick outside reach must leave the ball still.
])
def test_kick_rollout(soft, forward, lower, upper):
    p = sim.sim_parameters()
    settings = sim.load_settings(sim.SETTINGS)
    settings["opponents"]["enabled"] = False  # Unobstructed carpet rollout.
    settings["layout"].update(robot_spawn_xy=[-1.6, 0], robot_yaw_deg=0,
                              ball_spawn_xy=[-1.6 + forward, -.05])
    world = sim.World(settings=settings, parameters=p)
    game = sim.GameAdapter(world, p)
    xy = world.data.qpos[world.ball_qpos:world.ball_qpos + 2]
    velocity = world.data.qvel[world.ball_qvel:world.ball_qvel + 2]

    async def simulated_sleep(_seconds):
        pass  # The motion generator already advances the physics clock.

    try:
        with patch("asyncio.sleep", simulated_sleep), patch("time.monotonic", lambda: 100 + world.data.time), \
                patch("time.monotonic_ns", lambda: round((100 + world.data.time) * 1e9)):
            for _ in range(30):
                world.advance(.02)
            start = xy.copy()
            asyncio.run(game._move("motion.kick", dict(leg="right", soft=soft,
                                                       power=38 if soft else 100, offset=-10)))
            for _ in range(400):
                world.advance(.02)
            assert math.hypot(*velocity) < .002
            assert lower <= math.dist(start, xy) <= upper
    finally:
        world.renderer.close()
