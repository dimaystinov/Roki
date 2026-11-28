import asyncio
import math
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from roki_ng.body.motion.backends.roki2 import backend
from roki_ng.supervisor.game.simple_football import SimpleFootball
from roki_ng.detection.football import assemble_goals
from roki_ng.detection.football import shot_clear
from roki_ng.detection.football import camera_geometry
from roki_ng.detection.football import body_point
from roki_ng.detection.football import SceneTracker
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault
from test_game import setup, until


def harness(tmp_path, ball=(.09, .052), goals=None, entry='center', delay=10):
    p = Parameters(tmp_path).values
    g = SimpleNamespace(revision=1, info={}, _allowed=lambda: True, _same_plan=lambda r: True,
                        s=SimpleNamespace(log=lambda *a: None, params=SimpleNamespace(backend=backend)),
                        _imu=AsyncMock(return_value=(0., False)), _call=AsyncMock(return_value={}),
                        _settle=AsyncMock(return_value=True),
                        _move=AsyncMock(return_value=True))
    strategy = SimpleFootball(g, p, entry, delay)
    goal = dict(colour=strategy.enemy, ends=[[1., -.5], [1., .5]], elements='posts')
    strategy.observe = AsyncMock(return_value=(ball, [goal] if goals is None else goals))
    return strategy, g


def commands(g):
    return [(c.args[0], c.args[1]) for c in g._move.await_args_list]


def test_opening_missing_ball_does_not_turn_body_blindly(tmp_path):
    strategy, game = harness(tmp_path, ball=None, delay=0)
    strategy.kickoff_pending = False
    strategy.look = AsyncMock(return_value=None)
    strategy.search_sector = AsyncMock()
    asyncio.run(strategy.step())
    strategy.search_sector.assert_not_awaited()
    assert not any(op == 'game.jump' for op, _ in commands(game))


def test_opening_close_ball_found_at_side_pan_does_not_turn_body(tmp_path):
    strategy, game = harness(tmp_path, ball=None, delay=0)
    strategy.kickoff_pending = False
    strategy.look = AsyncMock(return_value={'ball': (.14, -.06), 'received': time.monotonic()})
    asyncio.run(strategy.step())
    assert not any(op == 'game.jump' for op, _ in commands(game))


def test_close_approach_keeps_toe_clear_of_ball(tmp_path):
    strategy, game = harness(tmp_path, ball=(.24, -.052), goals=[])
    strategy.opening_touch_required = False
    strategy.kickoff_pending = False
    strategy.kickoff_heading = 0.
    strategy.last_scene = {'received': time.monotonic()}
    strategy.shot_deadline = time.monotonic() + .5
    asyncio.run(strategy.step())
    walks = [args for operation, args in commands(game) if operation == 'game.walk']
    assert len(walks) == 1
    assert 0 < walks[0]['distance_m'] <= .08


def test_opening_walk_keeps_crouch_between_ball_updates(tmp_path):
    strategy, game = harness(tmp_path)

    async def run():
        assert await strategy.walk(.12)
        assert await strategy.walk(.08)

    asyncio.run(run())
    assert [op for op, _ in commands(game)] == ['game.walk', 'game.walk']
    assert all(args['keep_crouch'] for _, args in commands(game))
    assert strategy.approach_crouched


def test_opening_turn_is_followed_by_walk_without_second_turn(tmp_path):
    strategy, game = harness(tmp_path, ball=(.34, -.04), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading = 0.

    async def run():
        for _ in range(2):
            strategy.last_scene = {'received': time.monotonic()}
            await strategy.step()

    asyncio.run(run())
    assert [op for op, _ in commands(game)] == ['game.jump', 'game.walk']
    assert commands(game)[0][1]['direction'] == 'turn_left'


def test_opening_approach_tracks_ball_in_one_job(tmp_path):
    strategy, game = harness(tmp_path, ball=(.35, -.04), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading = 0.
    strategy.kickoff_heading_done = True
    strategy.last_scene = {'received': time.monotonic()}
    asyncio.run(strategy.step())
    operation, args = commands(game)[0]
    assert operation == 'game.walk'
    assert args['kind'] == 'approach' and args['stop_x_m'] == strategy.p['game.kick_distance_mm'] / 1000
    assert args['target_y_m'] == -.052
    assert args['x_m'] == .35 and args['y_m'] == -.04
    assert args['keep_crouch']


def test_visible_ball_twenty_centimetres_away_continues_walking(tmp_path):
    strategy, game = harness(tmp_path, ball=(.206, -.023), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading = 0.
    strategy.kickoff_heading_done = True
    strategy.last_scene = {'received': time.monotonic(), 'foot':
        {'leg': 'right', 'aligned': False, 'gap_ratio': 1.0,
         'ball_rect': [0, 0, 10, 10], 'tips': [[0, 0, 0, 0], [0, 0, 0, 0]]}}
    asyncio.run(strategy.step())
    assert commands(game)[0][0] == 'game.walk'
    assert commands(game)[0][1]['kind'] == 'approach'


def test_visible_large_foot_gap_blocks_soft_kick(tmp_path):
    strategy, game = harness(tmp_path, ball=(.117, -.058), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading = 0.
    strategy.kickoff_heading_done = True
    strategy.last_scene = {'received': time.monotonic(), 'foot':
        {'leg': 'right', 'aligned': False, 'gap_ratio': 1.0,
         'ball_rect': [0, 0, 10, 10], 'tips': [[0, 0, 0, 0], [0, 0, 0, 0]]}}
    asyncio.run(strategy.step())
    assert game.info['kick_setup']['in_reach'] is False
    assert all(op != 'motion.kick' for op, _ in commands(game))
    assert commands(game)[0][0] == 'game.walk'


def test_opening_near_foot_continues_walking_without_jump(tmp_path):
    strategy, game = harness(tmp_path, ball=(.141, -.034), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading_done = True
    strategy.last_scene = {'received': time.monotonic(), 'foot':
        {'leg': 'right', 'aligned': False, 'gap_ratio': 1.2,
         'ball_rect': [0, 0, 10, 10], 'tips': [[0, 0, 0, 0], [0, 0, 0, 0]]}}
    asyncio.run(strategy.step())
    assert [op for op, _ in commands(game)] == ['game.walk']
    assert commands(game)[0][1]['kind'] == 'approach'


def test_ball_behind_toe_is_not_kicked_or_jumped_over(tmp_path):
    strategy, game = harness(tmp_path, ball=(.066, -.063), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading_done = True
    strategy.last_scene = {'received': time.monotonic(), 'foot':
        {'leg': 'right', 'aligned': True, 'gap_ratio': .3,
         'ball_rect': [0, 0, 10, 10], 'tips': [[0, 0, 0, 0], [0, 0, 0, 0]]}}
    asyncio.run(strategy.step())
    assert game.info['kick_setup']['in_reach'] is False
    assert not commands(game)


def test_soft_kick_reobserves_after_leaving_walk_crouch(tmp_path):
    strategy, game = harness(tmp_path, ball=(.117, -.052), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading_done = True
    strategy.approach_crouched = True

    async def run():
        strategy.last_scene = {'received': time.monotonic()}
        await strategy.step()
        assert [op for op, _ in commands(game)] == ['motion.pose']
        strategy.last_scene = {'received': time.monotonic()}
        await strategy.step()

    asyncio.run(run())
    assert [op for op, _ in commands(game)] == ['motion.pose', 'motion.kick']


def test_soft_kick_is_retried_when_ball_did_not_move(tmp_path):
    strategy, game = harness(tmp_path, ball=(.117, -.052), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading_done = True

    async def run():
        strategy.last_scene = {'received': time.monotonic()}
        await strategy.step()
        assert strategy.opening_touch_required
        strategy.last_scene = {'received': time.monotonic()}
        await strategy.step()

    asyncio.run(run())
    assert [op for op, _ in commands(game)] == ['motion.kick', 'motion.kick']


def test_soft_kick_confirms_ball_movement_before_open_play(tmp_path):
    strategy, game = harness(tmp_path, ball=(.117, -.052), delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading_done = True

    async def run():
        strategy.last_scene = {'received': time.monotonic()}
        await strategy.step()
        strategy.observe.return_value = ((.20, -.052), [])
        strategy.last_scene = {'received': time.monotonic()}
        await strategy.step()

    asyncio.run(run())
    assert not strategy.opening_touch_required
    assert game.info['phase'] == 'simple_open_play'
    assert [op for op, _ in commands(game)] == ['motion.kick']


def test_ready_ball_kicks_immediately_without_map_or_further_alignment(tmp_path):
    s, g = harness(tmp_path)
    asyncio.run(s.step())
    assert commands(g) == [('motion.kick', {'leg': 'left', 'power': 100, 'offset': -10})]


def test_small_heading_error_without_clear_opening_orbits_instead_of_turning(tmp_path):
    s, g = harness(tmp_path, goals=[dict(colour='blue', ends=[[3., .1], [3., 1.1]])])
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'game.jump'
    assert commands(g)[0][1]['direction'] in ('left', 'right')
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_crossing_heading_target_does_not_reverse_turn_jump(tmp_path):
    s, g = harness(tmp_path, ball=(.2, 0.), goals=[dict(colour='blue', ends=[[1., .2], [1., .4]])])
    asyncio.run(s.step())
    assert commands(g)[0][1]['direction'] == 'turn_left'
    s.observe.return_value = ((.2, 0.), [dict(colour='blue', ends=[[1., -.4], [1., -.2]])])
    asyncio.run(s.step())
    assert commands(g)[1][1]['direction'] in ('left', 'right')


def test_lost_ball_at_jump_limit_repositions_instead_of_stalling(tmp_path):
    s, g = harness(tmp_path, ball=None)
    s.near, s.jumps = True, 15
    asyncio.run(s.step())
    assert commands(g) == [('game.jump', {'direction': 'backward', 'fraction': 1.})]
    assert s.jumps == 0


def test_pause_resume_during_scene_close_never_dispatches_old_action(tmp_path):
    async def run():
        s, g = harness(tmp_path)
        g._same_plan = lambda r: r == g.revision
        async def changed(*a, **kw):
            g.revision += 2
        g._call.side_effect = changed
        assert not await s.move('motion.kick', {'leg': 'left'})
        assert not commands(g)
    asyncio.run(run())


def test_cancelled_goal_scan_does_not_start_another_head_job(tmp_path):
    async def run():
        s, g = harness(tmp_path)
        g._same_plan = lambda r: r == g.revision
        g._prepare = AsyncMock(return_value=True)
        async def changed(*a, **kw):
            g.revision += 2
            return None
        s.look = AsyncMock(side_effect=changed)
        assert await SimpleFootball.observe(s, goals=True) == (None, [])
        assert s.look.await_count == 1
        assert not commands(g)
    asyncio.run(run())


def test_cancelled_imu_read_does_not_retarget_kickoff(tmp_path):
    async def run():
        s, g = harness(tmp_path, delay=0)
        g._same_plan = lambda r: r == g.revision
        async def changed():
            g.revision += 2
            s.cancel_special('pause')
            return 0., False
        g._imu.side_effect = changed
        await s.step()
        assert s.deadline is None and not s.kickoff_pending
        assert not commands(g)
    asyncio.run(run())


def test_shot_log_distinguishes_own_goal_veto_and_foot_alignment(tmp_path):
    goals = [dict(colour=c, ends=[[1., -.5], [1., .5]]) for c in ('yellow', 'blue')]
    s, g = harness(tmp_path, ball=(.09, 0.), goals=goals)
    lines = []
    g.s.log = lambda source, level, message: lines.append(message)
    asyncio.run(s.step())
    assert any('own_goal' in line and 'foot_alignment' in line for line in lines)
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_near_ball_uses_jumps_only(tmp_path):
    s, g = harness(tmp_path, ball=(.24, .052))
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'game.jump'
    assert s.jumps == 1
    s.observe.return_value = ((.26, .052), s.observe.return_value[1])
    asyncio.run(s.step())
    assert all(op != 'game.walk' for op, _ in commands(g))


def test_fifteenth_jump_checks_strike_before_reposition(tmp_path):
    s, g = harness(tmp_path)
    s.near, s.jumps = True, 15
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'motion.kick'
    s, g = harness(tmp_path, ball=(.2, .052))
    s.near, s.jumps = True, 15
    asyncio.run(s.step())
    assert commands(g)[0] == ('game.jump', {'direction': 'backward', 'fraction': 1.})
    assert s.jumps == 0


def test_failed_reposition_does_not_reset_attempt_in_place(tmp_path):
    s, g = harness(tmp_path, ball=(.2, .052))
    s.near, s.jumps = True, 15
    # A cancelled movement invalidates the whole plan, not another fine pass.
    g._move.return_value = False
    asyncio.run(s.step())
    assert len(commands(g)) == 1
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_no_goal_or_own_goal_in_corridor_never_kicks(tmp_path):
    for goals in ([], [dict(colour='yellow', ends=[[1., -.5], [1., .5]])]):
        s, g = harness(tmp_path, goals=goals)
        asyncio.run(s.step())
        assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_own_goal_veto_applies_even_with_opponent_goal(tmp_path):
    goals = [dict(colour=colour, ends=[[1., -.5], [1., .5]]) for colour in ('blue', 'yellow')]
    s, g = harness(tmp_path, goals=goals)
    asyncio.run(s.step())
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_map_coordinates_and_absolute_yaw_do_not_choose_action(tmp_path):
    outcomes = []
    for yaw in (-2.4, 0., 1.8):
        s, g = harness(tmp_path, ball=(.2, .052))
        for spec in s.specs:
            spec['x'], spec['y'] = yaw*100, -yaw*100
        g._imu.return_value = (yaw, False)
        asyncio.run(s.step())
        outcomes.append(commands(g))
    assert outcomes[0] == outcomes[1] == outcomes[2]


def test_delayed_or_disabled_center_has_no_kickoff(tmp_path):
    s, _ = harness(tmp_path, delay=10)
    assert not s.kickoff_pending
    s.p['game.forward.kick_off_ride'] = False
    assert not SimpleFootball(s.g, s.p, 'center', 0).kickoff_pending


@pytest.mark.parametrize('entry', ['left', 'right'])
def test_entry_issued_once_even_if_execution_unknown(tmp_path, entry):
    s, g = harness(tmp_path, entry=entry, delay=0)
    g._move.side_effect = Fault('body_unavailable', 'unknown outcome')
    with pytest.raises(Fault):
        asyncio.run(s.step())
    assert not s.entry_pending and not s.kickoff_pending
    assert commands(g)[0][1]['distance_m'] == .2


@pytest.mark.parametrize('sign,side', [(1, 'left'), (-1, 'right')])
def test_center_turns_then_uses_two_right_soft_kicks(tmp_path, sign, side):
    s, g = harness(tmp_path, ball=None, goals=[], delay=0)
    s.p['game.simple_football.kickoff_side'] = side
    diagonal = 1.4 + sign*math.radians(45)
    g._imu.side_effect = [(1.4, False), (diagonal, False), (diagonal, False),
                          (diagonal-sign*math.radians(12.5), False)]
    asyncio.run(s.step())
    actions = commands(g)
    assert [op for op, args in actions] == ['game.jump', 'motion.pose', 'game.walk', 'motion.kick', 'motion.pose', 'motion.kick']
    assert actions[0][1]['direction'] == ('turn_left' if sign > 0 else 'turn_right')
    assert actions[1][1] == {'name': 'base_stand'}
    assert actions[2][1]['kind'] == 'omni'
    assert actions[3][1]['leg'] == 'right'
    assert actions[3] == actions[5]
    assert actions[3][1]['soft'] and actions[3][1]['power'] == 38
    s.observe.assert_not_awaited()
    assert not s.kickoff_pending and s.opening_touch_required
    x, y = s.placed_ball_reference
    for displacement in (.03, .21):
        g._call.return_value = dict(ball=(x+displacement, y), pose_id=s.pose_id, confirmed=True, valid=True, received=time.monotonic())
        asyncio.run(s.step())
        assert s.opening_touch_required == (displacement < .20)
    assert commands(g) == actions


def test_expired_kickoff_cannot_strike_without_goal(tmp_path):
    s, g = harness(tmp_path, goals=[])
    s.deadline, s.kickoff_heading = time.monotonic()-1, 0.
    asyncio.run(s.step())
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_scene_requires_distinct_consistent_frames_and_pose(tmp_path):
    tracker = SceneTracker()
    frame = dict(pose_id=1, sequence=1, ball=[.2, 0.], parts=[])
    assert tracker.update(frame)['ball'] is None
    assert tracker.update(frame)['ball'] is None
    assert tracker.update(frame | {'sequence': 2})['ball'] == [.2, 0.]
    assert tracker.update(frame | {'pose_id': 2, 'sequence': 3})['ball'] is None


@pytest.mark.parametrize('combination', ['posts', 'left_bar', 'right_bar'])
def test_any_two_goal_elements_without_map(combination):
    posts = [dict(kind='post', colour='blue', points=[point]) for point in ([1., -.5], [1., .5])]
    bar = dict(kind='bar', colour='blue', points=[[1., -.5], [1., .5]])
    parts = posts if combination == 'posts' else [posts[0 if combination == 'left_bar' else 1], bar]
    goal = assemble_goals(parts, [dict(colour='blue', width=1., x=999, y=-999)])[0]
    assert shot_clear([.1, 0.], goal, .095)
    assert not shot_clear([.1, .49], goal, .095)
    assert not shot_clear([2., 0.], goal, .095)


def test_single_post_and_ambiguous_pairs_not_goal():
    post = lambda x, y: dict(kind='post', colour='blue', points=[[x, y]])
    specs = [dict(colour='blue', width=1.)]
    assert assemble_goals([post(1, 0)], specs) == []
    assert assemble_goals([post(1, -.5), post(1, .5), post(3, -.5), post(3, .5)], specs) == []


def test_camera_reference_and_pan_are_body_relative(tmp_path):
    p = Parameters(tmp_path).values
    from roki_ng.vision.camera_geometry import CameraGeometry
    model = CameraGeometry(height_m=.43, reference_tilt=-1000, forward_m=.04)
    geometry = camera_geometry(p, 0, -1000, model)
    assert geometry['height'] == .43
    assert body_point([1., 0.], geometry) == pytest.approx([1.04, 0.])
    assert camera_geometry(p, 0, -1500, model)['height'] < .43
    assert body_point([1., 0.], camera_geometry(p, -1778, -1000, model))[1] > .85


def test_simple_football_initialises_dead_reckoning_without_visual_localisation(tmp_path, monkeypatch):
    monkeypatch.setattr(time, "CLOCK_BOOTTIME", getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC), raising=False)
    async def run():
        game, sup, session = setup(tmp_path)
        calls = []
        async def step(self):
            assert abs(self.g.odometry.x) == pytest.approx(.30)
            assert not self.p['game.use_visual_localisation']
            calls.append(1)
            await asyncio.sleep(.01)
        monkeypatch.setattr(SimpleFootball, 'step', step)
        await game.start(session, {'strategy': 'simple_football', 'entry': 'center'})
        await until(lambda: calls)
        await game.stop()
        start = next(args for op, args in sup.workers['detection'].calls if op == 'ball.start')
        assert start['football']
    asyncio.run(run())


def test_kickoff_expiry_during_pre_dispatch_await_cannot_send_kick(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.simple = SimpleFootball(game, sup.params.values, 'center', 0)
        game.simple.kickoff_pending = False
        game.simple.deadline = time.monotonic()+30
        async def invalidate(op, args=None, **kwargs):
            game.simple.deadline = time.monotonic()-1
            return {}
        sup.workers['detection'].call = invalidate
        assert not await game.simple.move('motion.kick', {'leg': 'left'})
        assert not any(op == 'motion.kick' for op, _ in sup.workers['motherboard'].calls)
        assert game.simple.deadline is None
    asyncio.run(run())


def test_unknown_kick_consumes_special_permission_before_acceptance(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.simple = SimpleFootball(game, sup.params.values, 'center', 0)
        game.simple.kickoff_pending = False
        game.simple.deadline = time.monotonic()+30
        game.simple.shot_deadline = time.monotonic()+.5
        original = sup.workers['motherboard'].call
        async def lost(op, args=None, **kwargs):
            if op == 'motion.kick':
                raise Fault('timeout', 'Unknown execution')
            return await original(op, args, **kwargs)
        sup.workers['motherboard'].call = lost
        with pytest.raises(Fault):
            await game.simple.move('motion.kick', {'leg': 'left'})
        assert game.simple.deadline is None and not game.simple.kickoff_pending
        assert game.motion_inflight  # Coordinator must resolve/cancel, not blindly repeat.
    asyncio.run(run())


def test_expiry_during_walk_requests_graceful_stop(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.simple = SimpleFootball(game, sup.params.values, 'center', 0)
        game.simple.kickoff_pending = False
        game.simple.deadline = time.monotonic()-1
        original = sup.workers['motherboard'].call
        count = 0
        async def status(op, args=None, **kwargs):
            nonlocal count
            if op == 'job.status':
                count += 1
                return {'status': 'running' if count == 1 else 'cancelled'}
            return await original(op, args, **kwargs)
        sup.workers['motherboard'].call = status
        result = await game._wait_job({'job_id': 'test'})
        assert result['status'] == 'cancelled'
        assert any(op == 'motion.stop_graceful' for op, _ in sup.workers['motherboard'].calls)
        assert game.simple.deadline is None
    asyncio.run(run())


def test_pause_drops_kickoff_reference(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.info['running'] = True
        game.simple = SimpleFootball(game, sup.params.values, 'center', 0)
        game.simple.deadline = time.monotonic()+30
        game.simple.kickoff_heading = 1.
        await game.pause()
        assert game.simple.deadline is None and game.simple.kickoff_heading is None
        assert not game.simple.kickoff_pending
    asyncio.run(run())


def test_expired_scene_never_dispatches_kick(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.simple = SimpleFootball(game, sup.params.values, 'center', 10)
        game.simple.shot_deadline = time.monotonic()-1
        assert not await game.simple.move('motion.kick', {'leg': 'left'})
        assert not any(op == 'motion.kick' for op, _ in sup.workers['motherboard'].calls)
    asyncio.run(run())


def test_simple_menu_goal_selection_before_entry_does_not_start_motion():
    from roki_ng.supervisor.head_menu import HeadMenu
    async def run():
        calls = []
        async def command(op, args):
            calls.append((op, args))
            return {'colour': args['colour']}
        menu = HeadMenu(command, AsyncMock(), lambda *a: None, lambda *a: None)
        menu.press('ok')  # Game
        assert menu.selected.label == 'Football'
        assert 'Simple football' not in [i.label for i in menu.selected.children]
        menu.press('right')
        assert menu.selected.label == 'Simple football'
        menu.press('ok')
        assert menu.selected.label == 'Own yellow goal'
        menu.press('back')
        assert menu.selected.label == 'Simple football'
        assert not calls
        menu.press('ok')
        menu.press('ok')
        await menu.action
        assert calls == [('menu.own_goal', {'colour': 'yellow'})]
        assert menu.selected.label == 'Left'
        menu.press('right')
        menu.press('ok')
        assert menu.selected.args['strategy'] == 'simple_football'
        assert menu.selected.args['entry'] == 'center'
        assert len(calls) == 1

        menu.game_active, menu.job = True, 'game1'
        await menu.game_finished(dict(job_id='game1', running=True, pickup=True,
                                      pickup_ready=True, pickup_confirmation_required=False))
        for _ in range(3):
            menu.press('right')
        assert menu.selected.label == 'Simple football'
        menu.press('ok')
        assert menu.selected.label == 'Own yellow goal'
        assert len(calls) == 1
    asyncio.run(run())


def test_goal_sweep_stops_at_opening_and_keeps_it_until_motion(tmp_path):
    async def run():
        s, g = harness(tmp_path)
        g._prepare = AsyncMock(return_value=True)
        scene = lambda parts: dict(received=time.monotonic(), parts=parts, ball=(.1, .05), sequence=8,
                                   parameter_revision=2)
        opening = dict(kind='opening', colour=s.enemy, points=[[1., -.5], [1., .5]])
        s.look = AsyncMock(side_effect=[scene([opening]), scene([]), scene([]), scene([])])
        assert (await SimpleFootball.observe(s, goals=True))[1][0]['shootable']
        assert s.look.await_count == 2
        assert (await SimpleFootball.observe(s))[1][0]['shootable']
        await s.move('game.jump', dict(direction='left', fraction=1))
        assert (await SimpleFootball.observe(s))[1] == []
    asyncio.run(run())


def test_slow_goal_pass_uses_individual_capture_age(tmp_path, monkeypatch):
    async def run():
        s, g = harness(tmp_path)
        g._prepare = AsyncMock(return_value=True)
        clock = [100.]
        from roki_ng.supervisor.game import simple_football
        monkeypatch.setattr(simple_football, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
        def look(pan, tilt, *, wait_ball=False):
            clock[0] += 3.1
            own = dict(kind='opening', colour=s.own, points=[[1., -.5], [1., .5]])
            enemy = own | {'colour': s.enemy}
            return dict(received=clock[0], parts=[own if clock[0] < 104 else enemy],
                        ball=(.1, .05), sequence=1, parameter_revision=3)
        s.look = AsyncMock(side_effect=look)
        ball, goals = await SimpleFootball.observe(s, goals=True)
        assert ball and [goal['colour'] for goal in goals] == [s.enemy]
        assert s.shot_deadline == clock[0]+.5
    asyncio.run(run())


def test_side_ball_is_found_before_body_search_and_rechecked_first(tmp_path):
    async def run():
        s, _ = harness(tmp_path, ball=None)
        s.turn, s.search_sector = AsyncMock(), AsyncMock()
        hit = {'ball': (1.4, -1.3)}
        s.look = AsyncMock(side_effect=[None, hit, hit])
        await s.step()
        await s.step()
        tilt = s.p['game.head_tilt']
        assert [c.args for c in s.look.await_args_list] == [(0, s.p['head.field_tilt']+650), (1100, tilt), (0, s.p['head.field_tilt']+650)]
        assert s.turn.await_count == 2
        s.search_sector.assert_not_awaited()
    asyncio.run(run())


@pytest.mark.parametrize('occupied,age,expected', [
    ([], 0., 'motion.kick'), ([(0.,1.)], 0., None), ([], 2., None),
    ([(0.,.45)], 0., 'game.jump')])
def test_choose_free_corner_before_strike(tmp_path, occupied, age, expected):
    s,g = harness(tmp_path)
    x = s.p['game.kick_distance_mm']/1000
    y = s.p['game.kick_offset_mm']/1000
    s.near = True
    s.shot_deadline = time.monotonic()+.5
    s.observe.return_value = ((x,y), [dict(colour=s.enemy, shootable=True,
        ends=[[1.,y-.25],[1.,y+.75]], opening_source={'observed_at':time.monotonic()-age,
                                                        'occupied':occupied})])
    asyncio.run(s.step())
    assert [op for op,_ in commands(g)] == ([] if expected is None else [expected])


def test_ball_rolled_away_after_heading_change(tmp_path):
    s, g = harness(tmp_path, ball=(.1, -.052), goals=[])
    s.near, s.jumps = True, 9
    s.last_scene = {'received': time.monotonic(), 'ball_sequence': 1}
    asyncio.run(s.step())
    s.observe.return_value = ((.33, -.05), [])
    s.last_scene['ball_sequence'] = 2
    asyncio.run(s.step())
    asyncio.run(s.step())
    assert s.near and s.jumps == 9 and not commands(g)
    s.last_scene['ball_sequence'] = 3
    asyncio.run(s.step())
    assert not s.near and s.jumps == 0 and s.leg_sign is None
    assert not commands(g)
    s.last_scene['ball_sequence'] = 4
    asyncio.run(s.step())
    assert commands(g)[-1][0] == 'game.jump'
    assert commands(g)[-1][1]['direction'] == 'forward'


@pytest.mark.parametrize('low,high', [(-.5,.5), (-.23,.77)])
def test_strike_when_straight_path_is_open_even_off_corner_axis(tmp_path, low, high):
    s, g = harness(tmp_path)
    y = -.052
    s.shot_deadline = time.monotonic()+.5
    s.observe.return_value = ((.1,y), [dict(colour=s.enemy, shootable=True,
        ends=[[1.,y+low],[1.,y+high]], opening_source={'observed_at':time.monotonic(),
                                                       'occupied':[]})])
    asyncio.run(s.step())
    assert [op for op,_ in commands(g)] == ['motion.kick']


def test_small_miss_of_open_lane_adjusts_instead_of_scanning_again(tmp_path):
    s, g = harness(tmp_path)
    ball = (.1, -.052)
    # The padded opening begins just left of the straight kick.
    margin = s.p['game.ball_diameter_m']/2+.05
    s.observe.return_value = (ball, [dict(colour=s.enemy, shootable=True,
        ends=[[2.,ball[1]+.005-margin],[2.,ball[1]+.38+margin]],
        opening_source={'observed_at':time.monotonic(),'occupied':[]})])
    asyncio.run(s.step())
    assert [op for op, _ in commands(g)] == ['game.jump']
    assert commands(g)[0][1]['direction'] == 'turn_left'

def test_reorientation_before_first_walk_does_not_schedule_goal_glance(tmp_path):
    s, g = harness(tmp_path, ball=(.4,.7), goals=[])
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'game.jump'
    assert s.approach_glance_after is None
    s.observe.return_value = ((.9,0.), [])
    s.last_scene = {'received':time.monotonic()}
    s.shot_deadline = s.last_scene['received']+.5
    asyncio.run(s.step())
    assert commands(g)[-1][0] == 'game.walk'
    assert s.approach_glance_after is not None


def test_opening_walks_to_visible_ball_without_jump_or_goal_scan(tmp_path):
    s, g = harness(tmp_path, ball=(.32, .03), delay=0)
    s.kickoff_pending = False
    s.kickoff_heading = -math.radians(15)
    s.last_scene = {'received': time.monotonic()}
    asyncio.run(s.step())
    assert [op for op, args in commands(g)] == ['game.walk']
    assert commands(g)[0][1]['kind'] == 'approach'
    assert commands(g)[0][1]['stop_x_m'] == s.p['game.kick_distance_mm'] / 1000
    assert s.observe.await_args.kwargs == {'goals': False}


def test_first_opportunity_after_side_pass_strikes_opponent_goal(tmp_path):
    s, g = harness(tmp_path, ball=(.12, .052), goals=[], delay=0)
    s.kickoff_pending = False
    s.kickoff_heading = -math.pi/3
    s.last_scene = {'received': time.monotonic()}
    asyncio.run(s.step())
    assert not s.opening_touch_required
    s.shot_deadline = time.monotonic()+.5
    s.observe.return_value = ((.1, -.052), [dict(colour=s.enemy, shootable=True,
        ends=[[1., -.4], [1., .4]], opening_source={'observed_at':time.monotonic(), 'occupied':[]})])
    asyncio.run(s.step())
    assert [op for op, args in commands(g)] == ['motion.kick', 'motion.kick']
    assert commands(g)[0][1]['soft'] and not commands(g)[1][1].get('soft')

@pytest.mark.parametrize('sign,leg,support', [(1, 'left', 'right'), (-1, 'right', 'left')])
def test_soft_kick_uses_selected_foot_calibration(tmp_path, sign, leg, support):
    strategy, game = harness(tmp_path, ball=(.12, sign*.052), goals=[], delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading = 0.
    strategy.p['game.simple_football.kickoff_side'] = 'left'
    strategy.p[f'game.soft_kick.{leg}.forward_trim_mm'] = -15.
    game._imu.return_value = (math.radians(60), False)
    strategy.last_scene = {'received': time.monotonic(), 'ball_sequence': 17}
    asyncio.run(strategy.step())
    setup = game.info['kick_setup']
    assert (setup['leg'], setup['support'], setup['frame']) == (leg, support, 17)
    assert setup['front_error_mm'] == 35 and not setup['in_reach']
    assert commands(game)[-1][0] == 'game.walk'

@pytest.mark.parametrize('gap_mm,op', [(10,'motion.kick'), (20,'motion.kick'),
                                       (30,'motion.kick'), (40,'game.walk')])
def test_right_toe_opening_touch(tmp_path, gap_mm, op):
    s, g = harness(tmp_path, ball=((50 + 45 + gap_mm)/1000, -.052), goals=[], delay=0)
    s.p['game.soft_kick.right.toe_x_mm'] = 50.
    s.kickoff_pending = False
    s.kickoff_heading = -math.pi/3
    s.last_scene = {'received': time.monotonic()}
    asyncio.run(s.step())
    assert commands(g)[-1][0] == op
    assert g.info['kick_setup']['toe_gap_mm'] == gap_mm
    if op == 'game.walk':
        assert commands(g)[-1][1]['kind'] == 'approach'


def test_visually_ready_ball_takes_approximate_open_goal(tmp_path):
    now = time.monotonic()
    s, g = harness(tmp_path, ball=(.15, -.052), goals=[], delay=0)
    s.opening_touch_required = s.kickoff_pending = False
    s.near = True
    s.observe.return_value = ((.15, -.052), [dict(colour=s.enemy, shootable=True,
        ends=[[1.5, .13], [1.5, .42]], opening_source={'observed_at': now-2, 'occupied': []})])
    s.last_scene = {'received': now, 'foot': {'leg':'right', 'aligned':True, 'gap_ratio':.35}}
    asyncio.run(s.step())
    assert commands(g)[-1] == ('motion.kick', {'leg':'right','power':100,'offset':-10})

@pytest.mark.parametrize('side', ['left', 'right'])
def test_center_heading_residual_allows_two_soft_kicks(tmp_path, side):
    strategy, game = harness(tmp_path, ball=None, goals=[], delay=0)
    strategy.p['game.simple_football.kickoff_side'] = side
    asyncio.run(strategy.step())
    asyncio.run(strategy.step())
    strategy.observe.assert_not_awaited()
    actions = commands(game)
    assert [op for op, _ in actions].count('motion.kick') == 2
    assert actions[-1][1]['soft']
    assert not strategy.kickoff_pending


@pytest.mark.parametrize('walk_sent', [False, True])
def test_ball_touching_right_toe_can_soft_kick(tmp_path, walk_sent):
    strategy, game = harness(tmp_path, ball=(.0854, -.088), goals=[], delay=0)
    strategy.kickoff_pending = False
    strategy.kickoff_heading_done = True
    strategy.opening_walk_sent = walk_sent
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    strategy.last_scene = {'received': time.monotonic()}
    asyncio.run(strategy.step())
    assert commands(game)[0][0] == 'motion.kick'
    assert commands(game)[0][1]['soft']


@pytest.mark.parametrize('leg, side', [('right', -.052), ('left', .052)])
def test_open_play_kicks_at_visible_foot_with_recent_goal_bearing(tmp_path, leg, side):
    strategy, game = harness(tmp_path, ball=(.115, side), goals=[])
    strategy.opening_touch_required = False
    now = time.monotonic()
    strategy.last_scene = {'confirmed': True, 'received': now,
                           'foot': {'leg': leg, 'aligned': True, 'gap_ratio': .25}}
    strategy.gaze_imu = lambda: (.1, 7)
    strategy.expected_gaze.remember('enemy_goal', (1., 0.), yaw=.1, epoch=7, now=now)
    asyncio.run(strategy.step())
    assert [(op, args['leg']) for op, args in commands(game)] == [('motion.kick', leg)]


def test_open_play_waits_for_new_ball_after_adjustment(tmp_path):
    strategy, game = harness(tmp_path, ball=(.115, -.052), goals=[])
    strategy.opening_touch_required = False
    now = time.monotonic()
    strategy.ball_refresh_after = now
    strategy.last_scene = {'confirmed': True, 'received': now, 'ball_received': now - .1}
    asyncio.run(strategy.step())
    assert not commands(game)
    assert game.info['phase'] == 'simple_ball_recheck'


@pytest.mark.parametrize('ball,heading,operation', [
    ((.55, -.07), 0., 'game.walk'),
    ((.24, .14), -.65, 'game.jump'),
    ((.115, -.052), 0., 'motion.kick'),
])
def test_attack_after_centre_pass(tmp_path, ball, heading, operation):
    strategy, game = harness(tmp_path, ball=ball, goals=[])
    strategy.placed_kickoff_started = True
    strategy.opening_touch_required = strategy.kickoff_pending = False
    strategy.attack_axis = heading
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    strategy.last_scene = dict(confirmed=True, received=time.monotonic())
    asyncio.run(strategy.step())
    issued = commands(game)
    assert len(issued) == 1
    assert issued[0][0] == operation
    if operation == 'motion.kick':
        assert issued[0][1]['leg'] == 'right'
        assert issued[0][1]['power'] == 80
    if operation == 'game.jump':
        assert issued[0][1]['direction'] == 'left'


@pytest.mark.parametrize('age,confirmed', [(1., True), (0., False)])
def test_post_pass_attack_requires_current_confirmed_ball(tmp_path, age, confirmed):
    ball = (.115, -.052)
    strategy, game = harness(tmp_path, ball=ball, goals=[])
    strategy.attack_axis = 0.
    strategy.last_scene = dict(confirmed=confirmed, ball_received=time.monotonic()-age)
    asyncio.run(strategy.attack_after_pass(ball, []))
    assert not commands(game)

def test_target_middle_of_largest_gap_then_take_clear_shot(tmp_path):
    ball = (.115, -.052)
    strategy, game = harness(tmp_path, ball=ball)
    strategy.attack_axis = 0.
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    strategy.last_scene = dict(confirmed=True, received=time.monotonic())
    strategy.opening_touch_required = False
    # Current shot passes left of a keeper near one post. The centre of
    # the open portion is to the right of the robot (negative body Y).
    goal = dict(colour=strategy.enemy, shootable=True, ends=[[1., -.5], [1., .5]],
                opening_source={'occupied': [[.65, .90]]})
    asyncio.run(strategy.attack_after_pass(ball, [goal], rechecked=True))
    assert commands(game)[-1][1]['direction'] == 'turn_right'
    assert strategy.attack_heading == pytest.approx(math.atan2(-.175-ball[1], 1.-ball[0]))
    strategy.last_scene = dict(confirmed=True, received=time.monotonic())
    asyncio.run(strategy.attack_after_pass(ball, [goal], rechecked=True))
    assert [op for op, _ in commands(game)] == ['game.jump', 'motion.kick']


def test_ball_moved_during_final_goal_check(tmp_path):
    strategy, game = harness(tmp_path, ball=(.45, -.06), goals=[])
    strategy.attack_axis = 0.
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    strategy.last_scene = dict(confirmed=True, received=time.monotonic())
    strategy.placed_kickoff_started = strategy.check_shot_view = True
    strategy.opening_touch_required = strategy.kickoff_pending = False
    asyncio.run(strategy.step())
    strategy.observe.assert_awaited_with(goals=True, final_aim=True)
    assert [op for op, _ in commands(game)] == ['game.walk']


def test_fall_during_opening_pass_reacquires_ball(tmp_path):
    strategy, game = harness(tmp_path, ball=(.45, -.05), goals=[])
    strategy.placed_kickoff_started = strategy.opening_touch_required = True
    strategy.cancel_special('fall')
    assert strategy.opening_touch_required and not strategy.kickoff_pending
    asyncio.run(strategy.step())
    strategy.observe.assert_awaited()
    assert all(op != 'motion.kick' or args.get('soft') for op, args in commands(game))


def test_central_ball_without_a_visible_keeper_aims_at_centre(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry

    ball = (.115, -.052)
    strategy, game = harness(tmp_path, ball=ball, goals=[])
    strategy.opening_touch_required = False
    strategy.attack_axis = 0.
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    game.odometry = GameOdometry(strategy.p, 'center', 0.)
    strategy.last_scene = dict(confirmed=True, received=time.monotonic())
    asyncio.run(strategy.attack_after_pass(ball, [], rechecked=True))
    assert [op for op, _ in commands(game)] == ['motion.kick']


def test_far_search_changes_viewpoint_before_another_sector(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry

    strategy, game = harness(tmp_path, ball=None, goals=[])
    strategy.opening_touch_required = strategy.kickoff_pending = False
    strategy.attack_axis = 0.
    strategy.look = AsyncMock(return_value=None)
    game.odometry = GameOdometry(strategy.p, 'center', 0.)
    asyncio.run(strategy.step())
    assert game.info['phase'] == 'simple_search_walk'
    op, args = commands(game)[0]
    assert op == 'game.walk'
    assert 0 < args['distance_m'] <= .32
    assert abs(args['course']) > .1


@pytest.mark.parametrize('side,direction', [('left','turn_left'), ('right','turn_right')])
def test_fixed_kickoff_is_independent_of_gaze_and_measured_rotation(tmp_path, side, direction):
    async def play(yaw):
        strategy, game = harness(tmp_path, delay=0)
        strategy.p['game.simple_football.kickoff_side'] = side
        game._imu = AsyncMock(side_effect=[(.2,False),(yaw,False),(yaw,False)])
        # Both tests must override the opposite configured fallback from a single frame.
        strategy.p['game.simple_football.kickoff_side'] = 'right' if side=='left' else 'left'
        strategy.look = AsyncMock(return_value={'frame_obstacles': [((.62,-.2 if side=='left' else .2),.09)]})
        strategy.opening_pass_course = lambda *a,**k: pytest.fail('adaptive lane used at kickoff')
        await strategy.placed_kickoff()
        strategy.look.assert_awaited_once_with(0, strategy.p['head.field_tilt'], single_frame=True)
        return commands(game)
    first, second = [asyncio.run(play(yaw)) for yaw in (-1., .8)]
    # The yaw_offset is the low-level walking heading reference.
    normalized = lambda seq: [(op,{k:v for k,v in args.items() if k!='yaw_offset'}) for op,args in seq]
    assert normalized(first)==normalized(second)
    assert [a['direction'] for op,a in first if op=='game.jump'] == [direction]*sum(op=='game.jump' for op,_ in first)
    assert sum(op=='game.walk' for op,_ in first)==1
    kicks=[a for op,a in first if op=='motion.kick']
    assert len(kicks)==1 and all(a['leg']=='right' and a['soft'] and a['power']==38 for a in kicks)


def test_diagonal_contact_quantized_to_measured_millimetres(tmp_path):
    s, g = harness(tmp_path)
    s.p['game.ball_diameter_m'] = .09
    s.look = AsyncMock(return_value={'ball':(.125952046, -.021930344)})
    assert asyncio.run(s.finish_diagonal_alignment(g.revision,50,52))
    assert not commands(g)


def test_diagonal_prefers_immediate_contact_with_edge_of_ball(tmp_path):
    s, g = harness(tmp_path)
    s.p['game.ball_diameter_m'] = .09
    s.look = AsyncMock(return_value={'ball':(.125, -.013)})
    assert asyncio.run(s.finish_diagonal_alignment(g.revision,50,52))
    assert not commands(g)


def test_diagonal_small_miss_keeps_heading_until_next_toe_frame(tmp_path):
    s, g = harness(tmp_path)
    s.p['game.ball_diameter_m'] = .09
    s.look = AsyncMock(side_effect=[{'ball':(.125, .002)}, {'ball':(.125,-.038)}])
    assert asyncio.run(s.finish_diagonal_alignment(g.revision,50,52))
    assert [(op,a['direction']) for op,a in commands(g)] == [('game.jump','left')]


@pytest.mark.parametrize('scene', [
    None,
    {'frame_obstacles': []},
    {'frame_obstacles': [((.60, 0.), .10)]},
    {'frame_obstacles': [((.60, .08), .08)]},
    {'frame_obstacles': [((.60, .30), .08), ((.60, -.30), .08)]},
])
def test_kickoff_prefers_left_when_blockage_is_unclear(tmp_path, scene):
    strategy, _ = harness(tmp_path, delay=0)
    strategy.p['game.simple_football.kickoff_side'] = 'right'
    strategy.look = AsyncMock(return_value=scene)
    strategy.diagonal_pass = AsyncMock()
    asyncio.run(strategy.placed_kickoff())
    assert strategy.kickoff_side == 'left'
    strategy.diagonal_pass.assert_awaited_once_with((.30, 0.), kickoff=True)


@pytest.mark.parametrize('side', [-1, 1])
def test_delayed_visitor_watches_until_ball_moves(tmp_path, side):
    forward, game = harness(tmp_path, delay=30)
    forward.look = AsyncMock(side_effect=[
        {}, {'ball': (.3, 0), 'received': 1.}, {'ball': (.30, side*.005), 'received': 1.2},
        {'ball': (.38, side*.02), 'received': 1.4}])
    async def observe():
        return [await forward.watch_restart_ball() for _ in range(4)]
    assert asyncio.run(observe()) == [False, False, False, True]
    assert forward.landing_pan * game.s.params.backend.Robot.camera.pan_sign * side > 0
    assert forward.restart_roll_side == side
    assert not forward.inspect_feet_first
    assert forward.waiting_sample == ((.38, side*.02), 1.4)
    game._move.assert_not_awaited()


@pytest.mark.parametrize('side', [-1, 1])
def test_incoming_diagonal_starts_pursuit_in_seen_direction(tmp_path, side):
    forward, _ = harness(tmp_path, delay=30)
    forward.look = AsyncMock(side_effect=[
        {'ball': (.40, 0.), 'received': 1.},
        {'ball': (.35, side*.04), 'received': 1.2},
    ])
    forward.turn = AsyncMock(return_value=False)
    async def play():
        assert not await forward.watch_restart_ball()
        assert await forward.watch_restart_ball()
        await forward.step()
    asyncio.run(play())
    forward.turn.assert_awaited_once_with(side * math.radians(15), tolerance=4)
    assert not forward.restart_pivot


def test_short_jump_tracks_landing_without_reusing_contact(tmp_path):
    strategy, game = harness(tmp_path)
    strategy.opening_touch_required = False
    strategy.last_scene = {'ball': (.16, -.05)}
    asyncio.run(strategy.short_foot_jump(.02, 0.))
    assert strategy.landing_pan * game.s.params.backend.Robot.camera.pan_sign < 0
    assert strategy.last_scene is None
    assert strategy.ball_refresh_after is not None
    strategy.cancel_special('pickup')
    assert strategy.landing_pan is None


@pytest.mark.parametrize('lateral', [-.2, .2])
def test_confirmed_opening_pass_leaves_circle_in_one_walk(tmp_path, lateral):
    strategy, game = harness(tmp_path, delay=0)
    strategy.kickoff_pending = False
    strategy.placed_ball_reference = (.1, -.05)
    game._prepare = AsyncMock(return_value=True)
    scene = dict(ball=(.50, lateral), received=time.monotonic(), confirmed=True,
                 valid=True, pose_id=strategy.pose_id, parts=[])
    game._call.return_value = scene
    asyncio.run(strategy.step())
    assert not strategy.opening_touch_required and strategy.placed_ball_reference is None
    [(op, args)] = commands(game)
    assert op == 'game.walk'
    x = args['distance_m'] * math.cos(args['course'])
    y = args['distance_m'] * math.sin(args['course'])
    assert math.hypot(x-.10, y+.05) > strategy.p['field.geometry']['circle_diameter']/2


def test_kickoff_chase_avoids_blocked_lanes(tmp_path):
    strategy, game = harness(tmp_path)
    obstacle = dict(kind='obstacle', points=[(.25, 0.)], radius=.12)
    asyncio.run(strategy.follow_kickoff((.5,0.), (.1,0.), {'parts':[obstacle]}))
    assert not commands(game)


@pytest.mark.parametrize('age,looks', [(19., 1), (1., 0)])
def test_final_keeper_recheck_is_single_and_requires_fresh_toe(tmp_path, age, looks):
    ball = (.115, -.052)
    s,g = harness(tmp_path,ball=ball,goals=[])
    s.opening_touch_required = s.kickoff_pending = False
    s.attack_axis = 0.
    s.p['game.soft_kick.right.toe_x_mm'] = 50.
    s.keeper_aim = (0., time.monotonic()-age,None)
    s.keeper_glance_at = s.keeper_aim[1]
    s.g.info['shot_aim'] = {'source':'keeper_window','fraction':.5}
    s.last_scene = dict(confirmed=True, received=time.monotonic())
    s.look = AsyncMock(return_value=None)
    asyncio.run(s.attack_after_pass(ball, []))
    assert s.look.await_count == looks
    if looks:
        assert not commands(g)
        assert 'shot_aim' not in g.info
        s.last_scene = dict(confirmed=False, received=time.monotonic())
        asyncio.run(s.attack_after_pass(ball, []))
        assert not commands(g)
        s.last_scene = dict(confirmed=True, received=time.monotonic())
        asyncio.run(s.attack_after_pass(ball, []))
        assert s.look.await_count == 1
    assert [op for op,_ in commands(g)] == ['motion.kick']


def test_lost_close_ball_searches_for_rebound_on_last_seen_side(tmp_path):
    strategy, game = harness(tmp_path, ball=None, delay=0)
    strategy.kickoff_pending = strategy.opening_touch_required = False
    strategy.near = True
    strategy.last_ball_side = -1
    strategy.empty_sectors = 1
    strategy.expected_pan = lambda name: 450
    strategy.look = AsyncMock(return_value=None)
    strategy.search_sector = AsyncMock()
    asyncio.run(strategy.step())
    calls = strategy.look.await_args_list
    assert calls[0].args[0] == 450 and calls[0].kwargs['wait_ball']
    shallow = min(0, strategy.p['head.field_tilt'] + 650)
    side_pan = round(-1100 / game.s.params.backend.Robot.camera.pan_sign)
    assert [c.args for c in calls if c.args[1] == shallow] == [
        (side_pan, shallow), (-side_pan, shallow)]
    assert all(not c.kwargs['wait_ball'] for c in calls[1:])
    strategy.search_sector.assert_awaited_once()
    assert not commands(game)


@pytest.mark.parametrize('scene_kind', ['clear', 'opponent', 'blocked', 'stale', 'near'])
def test_kickoff_follow_when_ball_leaves_view(tmp_path, scene_kind):
    strategy, game = harness(tmp_path)
    strategy.kickoff_pending = False
    strategy.p['field.geometry']['circle_diameter'] = .60
    strategy.placed_ball_reference = (.12, -.05)
    game._prepare = AsyncMock(return_value=True)
    obstacles = {'opponent': [((.21, 0.), .06)], 'blocked': [
        ((.14, y), .13) for y in (-.12, 0., .12)]}.get(scene_kind, [])
    scene = dict(valid=True, confirmed=True, ball=None, parts=[], frame_obstacles=obstacles,
                 received=time.monotonic() - (2 if scene_kind == 'stale' else 0))
    strategy.look = AsyncMock(return_value=scene)
    strategy.walk = AsyncMock(return_value=True)
    asyncio.run(strategy.step())
    if scene_kind in ('clear', 'near', 'opponent'):
        distance, angle = strategy.walk.await_args.args
        assert 0 < distance <= .30
        if scene_kind != 'opponent':
            assert distance == pytest.approx(.30)
        assert (abs(angle) > .1) == (scene_kind == 'opponent')
        assert strategy.pass_segments_left == 2
        if scene_kind == 'near':
            scene['ball'] = (.12, -.05)
            asyncio.run(strategy.step())
            assert strategy.pass_segments_left == 0
            strategy.walk.assert_awaited_once()
    else:
        strategy.walk.assert_not_awaited()
    # The opening command was already sent. A stale frame cannot prove contact.
    assert not strategy.opening_touch_required


def test_placed_pass_walks_until_right_toe_in_reach_and_rechecks_miss(tmp_path):
    s, game = harness(tmp_path)
    s.kickoff_pending = False
    s.pending_toe_pass = (50., 52.)
    game._prepare = AsyncMock(return_value=True)
    near = (.05+s.p['game.ball_diameter_m']/2+.020, -.052)
    def frame(ball):
        return dict(valid=True, confirmed=True, ball=ball, received=time.monotonic())
    s.look = AsyncMock(side_effect=lambda *a, **kw: frame(s.test_ball))
    s.walk = AsyncMock(return_value=True)
    s.move = AsyncMock(return_value=True)
    s.test_ball = (near[0]+.08,near[1])
    asyncio.run(s.step())
    s.walk.assert_awaited_once()
    s.move.assert_not_awaited()
    s.test_ball = near
    asyncio.run(s.step())
    assert s.move.await_args.args[0] == 'motion.kick' and s.placed_ball_reference == near
    asyncio.run(s.step())
    assert s.pending_toe_pass and s.placed_ball_reference is None
    s.move.assert_awaited_once()


@pytest.mark.parametrize('opponent_y,operation', [(.20, 'motion.kick'), (.85, 'game.jump')])
def test_pressure_at_ball_changes_free_window_priority(tmp_path, opponent_y, operation):
    ball = (.115, -.052)
    strategy, game = harness(tmp_path, ball=ball)
    strategy.opening_touch_required = False
    strategy.attack_axis = 0.
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    strategy.last_scene = dict(confirmed=True, received=time.monotonic(),
                               parts=[dict(kind='obstacle', points=[(.13, opponent_y)], radius=.08)])
    goal = dict(colour=strategy.enemy, shootable=True, ends=[[1., -.5], [1., .5]],
                opening_source={'occupied': [[.65, .90]]})
    asyncio.run(strategy.attack_after_pass(ball, [goal], rechecked=True))
    assert commands(game)[0][0] == operation


def test_ball_region_uses_field_velocity_and_consumes_estimate(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry
    strategy, game = harness(tmp_path)
    odom = game.odometry = GameOdometry(strategy.p, 'center', 0)
    now = time.monotonic()
    strategy.remember_near_ball(dict(confirmed=True, received=now-.8, ball=(.6, 0)))
    strategy.remember_near_ball(dict(confirmed=True, received=now-.4, ball=(.6, .10)))
    strategy.walk = AsyncMock(return_value=True)
    assert asyncio.run(strategy.approach_ball_region())
    distance, bearing = strategy.walk.await_args.args
    assert 0 < distance <= .25 and bearing > math.atan2(.10,.6)
    assert not asyncio.run(strategy.approach_ball_region())
    strategy.remember_near_ball(dict(confirmed=True, received=now, ball=(.5, .1)))
    odom.pose_revision += 1
    assert not asyncio.run(strategy.approach_ball_region())


def test_missing_image_after_opening_resumes_pursuit_without_claiming_contact(tmp_path):
    strategy, game = harness(tmp_path)
    strategy.kick_check = (None, (.12,-.05), 50, 50)
    strategy.look = AsyncMock(return_value=None)
    asyncio.run(strategy.check_opening_kick(game.revision))
    assert strategy.kick_check is None and strategy.placed_ball_reference == (.12,-.05)
    assert game.info['phase'] == 'simple_kick_unconfirmed'


def test_no_prediction_walk_through_observed_robot(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry
    strategy, game = harness(tmp_path)
    game.odometry = GameOdometry(strategy.p, 'center', 0)
    strategy.remember_near_ball(dict(confirmed=True, received=time.monotonic(), ball=(.55,0)))
    strategy.last_scene = dict(parts=[dict(kind='obstacle', points=[(.18,0)], radius=.10)])
    strategy.walk = AsyncMock()
    assert not asyncio.run(strategy.approach_ball_region())
    strategy.walk.assert_not_awaited()


def test_recent_relative_ball_survives_uncertain_global_position(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry
    strategy, game = harness(tmp_path)
    odom = game.odometry = GameOdometry(strategy.p, 'center', 0)
    odom.uncertain('fall')
    strategy.remember_near_ball(dict(confirmed=True, received=time.monotonic(), ball=(.45, .05)))
    strategy.walk = AsyncMock(return_value=True)
    assert asyncio.run(strategy.approach_ball_region())
    assert strategy.walk.await_args.args[0] <= .12
    strategy.remember_near_ball(dict(confirmed=True, received=time.monotonic(), ball=(.45, .05)))
    odom.path += .20
    assert not asyncio.run(strategy.approach_ball_region())


def test_toe_check_uses_first_current_ball_even_if_toe_missing(tmp_path):
    strategy, game = harness(tmp_path)
    strategy.pending_toe_pass = (55, 52)
    samples = []

    async def observe(role, op, args=None):
        if op == 'football.status':
            samples.append(op)
            return dict(valid=True, confirmed=False, pose_id=strategy.pose_id,
                        received=time.monotonic(), ball=(.119, -.052), parts=[])
        return {}
    game._call = observe
    scene = asyncio.run(strategy.look(0, -2100))
    assert samples == ['football.status']
    assert scene['ball'] and not scene.get('foot')


def test_open_play_uses_start_imu_without_map_posts_or_toe(tmp_path):
    strategy, game = harness(tmp_path)
    strategy.opening_touch_required = False
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    strategy.gaze_imu = lambda: (0., 7)
    strategy.expected_gaze.remember('start_goal', (1., 0.), yaw=0., epoch=7, now=time.monotonic())
    strategy.last_scene = dict(confirmed=True, received=time.monotonic(), parts=[])
    asyncio.run(strategy.attack_after_pass((.115, -.052), []))
    assert commands(game)[0][0] == 'motion.kick'
    strategy.gaze_imu = lambda: (0., 8)
    assert strategy.remembered_goal_heading() is None


def test_prolonged_blind_search_after_fall_keeps_retrying_without_spinning(tmp_path):
    strategy, game = harness(tmp_path, ball=None, goals=[], entry='center', delay=0)
    strategy.kickoff_pending = strategy.opening_touch_required = False
    strategy.last_usable_frame = time.monotonic()-20
    strategy.remembered_goal_heading = lambda: .5
    strategy.look = AsyncMock(return_value=None)
    strategy.turn = AsyncMock(return_value=True)
    strategy.search_sector = AsyncMock()

    async def run():
        await strategy.step()
        await strategy.step()
    asyncio.run(run())
    assert game.info['phase'] == 'simple_search_no_image'
    assert strategy.look.await_count > 1
    strategy.turn.assert_awaited_once()
    strategy.search_sector.assert_not_awaited()


def test_open_play_keeps_rolling_ball_destination_across_turn(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry
    strategy, game = harness(tmp_path)
    strategy.opening_touch_required = False
    odom = game.odometry = GameOdometry(strategy.p, 'center', 0)
    now = time.monotonic()
    for age, y in ((.3, .24), (.1, .55)):
        strategy.remember_near_ball(dict(confirmed=True, received=now-age, ball=(.18,y)))
    track = strategy.search_track
    assert math.isclose(math.hypot(*track.velocity), 1.55, abs_tol=.001)
    strategy.turn = AsyncMock(return_value=True)
    strategy.walk = AsyncMock(return_value=True)
    assert asyncio.run(strategy.approach_ball_region())
    assert strategy.search_track is track
    odom.yaw = (odom.yaw + strategy.turn.await_args.args[0] + math.pi) % (2*math.pi) - math.pi
    assert asyncio.run(strategy.approach_ball_region())
    strategy.walk.assert_awaited_once()
    assert strategy.search_track is None


def test_measured_roll_uses_frames_from_same_camera_pose(tmp_path):
    tracker = SceneTracker()
    frame = dict(pose_id=4, sequence=10, received=8., ball=(.5, .10), parts=[])
    assert tracker.update(frame)['roll_m_s'] is None
    tracked = tracker.update(dict(frame, sequence=11, received=8.1, ball=(.5, .14)))
    assert tracked['roll_m_s'] == pytest.approx((0., .4))

    from roki_ng.body.motion.odometry import GameOdometry
    strategy, game = harness(tmp_path)
    odom = game.odometry = GameOdometry(strategy.p, 'center', 0)
    odom.yaw = math.pi/2
    strategy.remember_near_ball(tracked)
    assert strategy.search_track.velocity == (0., 0.)  # Original opening behaviour.
    strategy.opening_touch_required = False
    strategy.search_track = None
    strategy.remember_near_ball(dict(frame, confirmed=True, ball=(.5, .12)))
    strategy.remember_near_ball(tracked)
    assert strategy.search_track.velocity == pytest.approx((-.3, 0.))

    assert tracker.update(dict(frame, pose_id=5, sequence=12, received=8.2))['roll_m_s'] is None


def test_repeated_empty_search_walks_to_circle_then_scans_both_halves(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry
    strategy, game = harness(tmp_path, ball=None, goals=[])
    strategy.opening_touch_required = strategy.kickoff_pending = False
    strategy.look = AsyncMock(return_value={'confirmed': True, 'parts': []})
    strategy.empty_sectors = 1
    odom = game.odometry = GameOdometry(strategy.p, 'center', 0)
    odom.x, odom.y, odom.yaw = .8, 0., math.pi
    odom.uncertain('fallen')
    asyncio.run(strategy.step())
    op, args = commands(game)[0]
    assert op == 'game.walk' and args['distance_m'] == .16
    assert abs(args['course']) < .001
    odom.x = 0.
    strategy.walk = AsyncMock(return_value=True)
    strategy.turn = AsyncMock(return_value=True)
    async def sweep():
        for sector in range(6):
            await strategy.search_field(odom)
            error = strategy.turn.await_args.args[0]
            assert math.degrees(error) == pytest.approx(60.)
            odom.yaw = (odom.yaw+error+math.pi)%(2*math.pi)-math.pi
        assert strategy.field_sector == 6 and strategy.field_sector_heading is None
        strategy.walk.assert_not_awaited()
        odom.yaw = 0.
        await strategy.search_field(odom)
        strategy.walk.assert_awaited_once()
    asyncio.run(sweep())


@pytest.mark.parametrize('direction', [-1, 1])
def test_centre_arc_changes_heading_during_one_forward_gait(tmp_path, direction):
    from roki_ng.body.motion.legacy_game_motion import kickoff_arc
    strategy, game = harness(tmp_path, delay=0)
    strategy.p['game.simple_football.kickoff_side'] = 'left' if direction==1 else 'right'
    strategy.place_and_pass = AsyncMock()
    asyncio.run(strategy.placed_kickoff())
    walks = [args for op,args in commands(game) if op=='game.walk']
    assert len(walks)==1 and walks[0]['kind']=='kickoff_arc'
    args = walks[0]
    orders = list(kickoff_arc(strategy.p, args['distance_m'], args['course'], args['invert']))
    advance = [c for c in orders if c.step > 0]
    assert len(advance) >= 3
    assert all(direction*(b.heading-a.heading)>0 for a,b in zip(advance,advance[1:]))
    assert direction*(advance[-1].heading-advance[0].heading) > math.radians(45)
    strategy.place_and_pass.assert_awaited_once()
