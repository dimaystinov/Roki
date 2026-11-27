import asyncio
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from roki_ng.parameters import Parameters
from roki_ng.supervisor.game.simple_goalkeeper import SimpleGoalkeeper
from roki_ng.supervisor.head_menu import menu_tree
from test_game import observation, setup


def keeper(tmp_path, ball=(.4, .2)):
    g = SimpleNamespace(revision=1, info={}, _same_plan=lambda r: r == g.revision,
        _prepare=AsyncMock(return_value=True), _imu=AsyncMock(return_value=(0., False)),
        _call=AsyncMock(return_value={}), _move=AsyncMock(return_value=True),
        s=SimpleNamespace(log=lambda *a: None))
    k = SimpleGoalkeeper(g, Parameters(tmp_path).values, 'goalkeeper', 0)
    from roki_ng.body.motion.odometry import GameOdometry
    g.odometry = GameOdometry(k.p, 'goalkeeper', 0.)
    k.look = AsyncMock(return_value={'ball': ball, 'parts': [], 'received': time.monotonic()})
    return k, g


@pytest.mark.parametrize('ball,direction', [((.4,.2),'left'), ((.4,-.2),'right'), ((.4,.05),None), (None,None)])
def test_only_one_bounded_lateral_jump(tmp_path, ball, direction):
    k, g = keeper(tmp_path, ball)
    asyncio.run(k.step())
    calls = g._move.await_args_list
    if direction:
        assert len(calls) == 1 and calls[0].args[0] == 'game.jump'
        assert calls[0].args[1]['direction'] == direction
        assert calls[0].args[1]['fraction']*k.p[f'motion.jump_{direction}_mm'] <= 20.01
    else:
        assert not calls


def test_close_ball_never_raises_head_for_goal(tmp_path):
    k, _ = keeper(tmp_path)
    asyncio.run(k.step())
    assert k.look.await_args_list[0].args == (0, k.p['game.head_tilt'])
    assert k.look.await_count == 1


def test_scan_uses_new_ball_not_pre_scan_position(tmp_path):
    k, g = keeper(tmp_path, (1.5,.3))
    k.look.side_effect = [{'ball': (1.5,.3), 'parts': []},
        {'ball': None, 'parts': [dict(kind='colour', colour=k.enemy, points=[[2.,0.]], area=100)]},
        {'ball': (.4,-.2), 'parts': [], 'received': time.monotonic()}]
    asyncio.run(k.step())
    assert k.look.await_args_list[1].args == (0, 0)
    assert g._move.await_args.args[1]['direction'] == 'right'


def test_pause_during_scan_never_moves(tmp_path):
    k, g = keeper(tmp_path)
    async def paused(*args):
        g.revision += 1
        return {'ball': (.4,.2), 'parts': []}
    k.look.side_effect = paused
    asyncio.run(k.step())
    assert not g._move.await_count


def test_ball_direction_changes_next_jump(tmp_path):
    k, g = keeper(tmp_path)
    asyncio.run(k.step())
    k.observe_motion_ball(observation(-.2, 2))
    k.look.return_value = {'ball': (.4,-.2), 'parts': [], 'received': time.monotonic()}
    asyncio.run(k.step())
    assert [c.args[1]['direction'] for c in g._move.await_args_list] == ['left','right']


def test_live_observation_is_consumed_during_body_job(tmp_path):
    async def run():
        g, s, _ = setup(tmp_path)
        k = SimpleGoalkeeper(g, s.params.values, 'goalkeeper', 0)
        g.simple = k
        k.move_ball = (.4,.2)
        worker = s.workers['motherboard']
        original = worker.call
        count = 0
        async def call(op, args=None, **kw):
            nonlocal count
            if op == 'job.status':
                count += 1
                return {'status': 'running' if count < 3 else 'completed', 'pose': 'base_stand'}
            return await original(op, args, **kw)
        worker.call = call
        await g._wait_job({'job_id': '1'})
        assert k.live_sequence >= 2
        assert any(op == 'ball.status' for op, _ in s.workers['detection'].calls)
        assert g.odometry is None
    asyncio.run(run())


def test_voice_menu_exposes_separate_keeper():
    game = next(i for i in menu_tree().children if i.label == 'Game')
    simple = next(i for i in game.children if i.label == 'Simple football')
    for colour in simple.children:
        keeper_item = next(i for i in colour.children if i.label == 'Goalkeeper')
        assert keeper_item.children[0].args == {'strategy': 'simple_goalkeeper'}


def test_stale_ball_never_dispatches_jump(tmp_path):
    k, g = keeper(tmp_path)
    k.look.return_value['received'] -= 1
    asyncio.run(k.step())
    assert not g._move.await_count


def test_recovery_requires_new_goal_direction(tmp_path):
    k, g = keeper(tmp_path)
    k.cancel_special('fall')
    asyncio.run(k.step())
    assert k.reacquire_heading
    assert not g._move.await_count
    assert k.look.await_args_list[1].args == (0, 0)


def test_keeper_colour_hint_does_not_require_posts_or_opening(monkeypatch, tmp_path):
    np = pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    from roki_ng.detection.football import goal_parts
    binary = np.zeros((650, 800), np.uint8)
    binary[280:300, 400:404] = 255
    turf = np.zeros_like(binary)
    turf[300:] = 255
    masks = {'green_field': turf, 'white_marking': np.zeros_like(binary),
             'blue_posts': binary, 'yellow_posts': binary}
    monkeypatch.setattr('roki_ng.vision.colour_masks.mask', lambda lab, params, name: masks.get(name, masks['white_marking']))
    projector = SimpleNamespace(_pixel_ray=lambda *a: [1., 0., -.2])
    args = (np.zeros((650,800,3), np.uint8), Parameters(tmp_path).values,
            projector, [0,0,0,1], {'pan':0})
    assert not goal_parts(*args)
    relaxed = goal_parts(*args, bearing_only=True)
    assert len(relaxed) == 2
    assert all(part['kind'] == 'colour' for part in relaxed)


def test_new_role_has_no_odometry_or_entry(tmp_path):
    async def run():
        g, s, session = setup(tmp_path)
        status = await g.start(session, {'strategy': 'simple_goalkeeper', 'delay_seconds': 30})
        assert status['role'] == 'simple_goalkeeper'
        assert isinstance(g.simple, SimpleGoalkeeper) and g.odometry is None
        assert not g.simple.kickoff_pending and not g.simple.entry_pending
        await g.stop()
    asyncio.run(run())


@pytest.mark.parametrize('offset', [-.17, .17])
def test_keeper_walks_laterally_to_cover_visible_offset(tmp_path, offset):
    import math
    k, g = keeper(tmp_path, (.65, offset))
    asyncio.run(k.step())
    op, args = g._move.await_args.args
    assert op == 'game.walk'
    assert args['distance_m'] == pytest.approx(.12)
    assert args['course'] == pytest.approx(math.copysign(math.pi/2, offset))


def test_trial_keeper_closes_then_returns_on_loss(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry

    k, game = keeper(tmp_path, ball=(.48, .08))
    k.p['sim.keeper_intercept'] = True
    odom = game.odometry = GameOdometry(k.p, 'goalkeeper', 0.)
    from roki_ng.body.motion.backends.roki2 import backend
    game.s.params = SimpleNamespace(backend=backend)
    game._settle = AsyncMock(return_value=True)
    k.walk = AsyncMock()
    k.look.return_value.update(confirmed=True)
    asyncio.run(k.step())
    distance, course = k.walk.await_args.args
    assert 0 < distance <= .14
    assert 0 < course < .3

    odom.x, odom.y = odom.field_point((.18, 0.))
    k.look.return_value.update(ball=None)
    asyncio.run(k.step())
    distance, course = k.walk.await_args.args
    assert 0 < distance <= .16
    assert abs(course) == pytest.approx(3.14159)


@pytest.mark.parametrize('point, blocks', [
    ((.46, .05), True), ((.46, .20), False),
    ((.50, .10), False), ((.54, .05), False)])
def test_keeper_split_reads_incoming_direction(tmp_path, point, blocks):
    k, game = keeper(tmp_path)
    game._settle = AsyncMock(return_value=True)
    k.move = AsyncMock(return_value=True)
    now = time.monotonic()
    first = dict(ball=(.50,.05), pose_id=2, confirmed=True, received=now-.10)
    game._call.return_value = dict(first, ball=point, received=now)
    assert asyncio.run(k.block_incoming(first, 1)) is blocks
    assert [call.args[0] for call in k.move.await_args_list] == (
        ['motion.splits', 'motion.pose'] if blocks else [])


@pytest.mark.parametrize('side,leg', [(-1,'right'), (1,'left')])
def test_keeper_finds_ball_at_side_and_clears(tmp_path, side, leg):
    from roki_ng.body.motion.backends.roki2 import backend

    k, g = keeper(tmp_path)
    g.s.params = SimpleNamespace(backend=backend)
    k.ball_side = side
    k.p['game.soft_kick.right.toe_x_mm'] = 50.
    ball = (.11, side*k.p['game.kick_offset_mm']/1000)
    k.look.return_value = dict(ball=ball, confirmed=True, received=time.monotonic())
    scene = asyncio.run(k.find_near_ball(1))
    pan, tilt = k.look.await_args.args
    assert pan*backend.Robot.camera.pan_sign*side > 0 and tilt < -1500
    k.heading = 0.
    k.move = AsyncMock(return_value=True)
    assert asyncio.run(k.clear_at_foot(scene, ball, 0.))
    assert k.move.await_args.args == ('motion.kick', dict(leg=leg,power=80,offset=0))
    assert not asyncio.run(k.clear_at_foot(scene, (.36,ball[1]), 0.))


@pytest.mark.parametrize('own', [0, 1])
@pytest.mark.parametrize('side', [-1, 1])
def test_keeper_checks_goal_area_before_fallback_jump_and_split(tmp_path, own, side):
    from roki_ng.body.motion.odometry import GameOdometry

    k, game = keeper(tmp_path)
    p = k.p | {'match.own_goal': own}
    goal = p[f'field.goal.{own}']
    inward = 1 if goal['x'] < 0 else -1
    p['field.mark.00'] = dict(enabled=True, kind='line', x=goal['x'] + inward*.30,
                              y=0., size=1.20, angle=1.5707963)
    k = SimpleGoalkeeper(game, p, 'goalkeeper', 0)
    odom = game.odometry = GameOdometry(p, 'goalkeeper', 0.)
    odom.y = side * .485
    k.track_deadline = time.monotonic() + 1
    out = 'left' if side * inward > 0 else 'right'
    back = 'right' if out == 'left' else 'left'
    assert not k.motion_fresh('game.jump', dict(direction=out, fraction=1.))
    assert k.motion_fresh('game.jump', dict(direction=back, fraction=1.))
    assert not k.motion_fresh('motion.splits', {})
    odom.y = 0.
    assert k.motion_fresh('motion.splits', {})
    assert not k.motion_fresh('game.walk', dict(distance_m=.12, course=0.))
    odom.uncertain('fallen')
    assert not k.motion_fresh('game.jump', dict(direction=out, fraction=1.))
