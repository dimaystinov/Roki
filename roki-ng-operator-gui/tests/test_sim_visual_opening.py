"""Decisions at the edge of the calibrated foot window during a kickoff."""
import asyncio
from pathlib import Path
import sys
from unittest.mock import AsyncMock, Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from sim_roki2 import VisualOpening


@pytest.mark.parametrize("points, obstacles, next_motion", [
    ([(.115, -.05)], [], "kick"),
    ([(.134, -.05), (.24, -.05)], [], "forward"),
    ([(.134, -.05), None], [], "kick"),
    ([(.134, -.05)], [((.30, -.05), .07)], "kick"),
])
def test_opening_with_hidden_toe(points, obstacles, next_motion):
    strategy = object.__new__(VisualOpening)
    strategy.g = Mock()
    strategy.g._same_plan.return_value = True
    strategy.pending_toe_pass = (50, 50)
    strategy.p = {"game.ball_diameter_m": .09}
    strategy.describe = Mock()
    strategy.look = AsyncMock(side_effect=[
        dict(ball=point, sequence=i, frame_obstacles=obstacles)
        for i, point in enumerate(points)
    ])
    strategy.jump = AsyncMock()
    strategy.strike = AsyncMock(return_value=True)
    asyncio.run(strategy.place_and_pass(1))
    assert strategy.look.await_count == len(points)
    if next_motion == "kick":
        strategy.strike.assert_awaited_once_with(dict(leg="right", soft=True, power=38, offset=0))
        strategy.jump.assert_not_awaited()
    else:
        strategy.jump.assert_awaited_once_with(next_motion, 1., alignment=True)
        strategy.strike.assert_not_awaited()
