import math
import pytest
from roki_ng.supervisor.game.expected_gaze import ExpectedGaze


def test_target_rotates_opposite_body_turn_and_respects_pan_sign():
    gaze = ExpectedGaze()
    gaze.remember('ball', (1, 0), yaw=0, epoch=1, now=10)
    assert gaze.pan('ball', yaw=math.pi/4, epoch=1, now=11, sign=1) == pytest.approx(-1333, abs=1)
    assert gaze.pan('ball', yaw=math.pi/4, epoch=1, now=11, sign=-1) == pytest.approx(1333, abs=1)


def test_old_or_reset_targets_are_not_used():
    gaze = ExpectedGaze()
    gaze.remember('post', (2, 1), yaw=0, epoch=1, now=10)
    assert gaze.pan('post', yaw=0, epoch=2, now=11, sign=1) is None
    assert gaze.pan('post', yaw=0, epoch=1, now=31, sign=1) is None


def test_invalid_and_behind_targets_do_not_command_head():
    gaze = ExpectedGaze()
    gaze.remember('ball', (float('nan'), 0), yaw=0, epoch=1, now=10)
    assert gaze.pan('ball', yaw=0, epoch=1, now=11, sign=1) is None
    gaze.remember('ball', (1, 0), yaw=0, epoch=1, now=10)
    assert gaze.pan('ball', yaw=math.pi, epoch=1, now=11, sign=1) is None


def test_strategy_preserves_post_identity_and_never_makes_hint_shootable(tmp_path):
    import time
    from roki_ng.parameters import Parameters
    from test_simple_football import harness
    from test_goal_memory import sample
    s, g = harness(tmp_path)
    g.s.params = Parameters(tmp_path, robot_model='roki2')
    g.s.body_telemetry = {'body.imu': sample() | {'yaw_epoch': 1}}
    scene = dict(received=time.monotonic(), ball=[.2, .03], parts=[
        dict(colour=s.enemy, kind='opening', points=[[2., -.1], [2., .9]])])
    s.remember_gaze(scene)
    assert s.expected_post_side == 0
    assert s.expected_pan('post') < 0
    original = s.expected_gaze.targets['post']
    assert s.expected_pan('enemy_goal') > 0
    s.remember_gaze(dict(received=time.monotonic(), ball=None, parts=[
        dict(colour=s.enemy, kind='colour', points=[[1., 1.]])]))
    assert s.expected_gaze.targets['post'] == original
    assert s.goal_memory is None and not s.goal_views
    g.s.body_telemetry['body.imu'] = sample(math.pi/4) | {'yaw_epoch': 1}
    assert s.expected_pan('post') < -1300
    assert s.expected_pan('enemy_goal') == pytest.approx((math.atan2(.4, 2)-math.pi/4)/.00058909, abs=1)
    s.cancel_special('fall')
    assert s.expected_pan('post') is None
    assert s.expected_pan('enemy_goal') is None


def test_reacquiring_close_ball_does_not_undo_goal_heading(tmp_path):
    import asyncio
    from unittest.mock import AsyncMock
    from test_simple_football import harness, commands
    s, g = harness(tmp_path, ball=None, goals=[])
    s.near = True
    s.look = AsyncMock(return_value={'ball': [.10, .065], 'sequence': 1})
    s.log_search_found = lambda *args: None
    asyncio.run(s.step())
    assert not commands(g)


def test_colour_only_does_not_send_robot_orbiting_around_ready_ball(tmp_path):
    import asyncio
    from test_simple_football import harness, commands
    s, g = harness(tmp_path, ball=(.10, .052), goals=[
        dict(colour='blue', ends=[[2., -.4], [2., .4]], shootable=False)])
    s.near = True
    asyncio.run(s.step())
    assert not commands(g)
    assert s.goal_scan_needed


def test_wide_clear_corridor_triggers_shot_without_extra_turn(tmp_path):
    import asyncio
    import time
    from unittest.mock import patch
    from test_simple_football import harness, commands
    lane = [[2., -.15], [2., .6]]
    s, g = harness(tmp_path, ball=(.10, .052), goals=[
        dict(colour='blue', ends=lane, shootable=True,
             opening_source={'occupied': [], 'observed_at': time.monotonic()})])
    s.near = True
    s.opening_touch_required = False
    s.last_scene = dict(confirmed=True, received=time.monotonic(),
                        foot=dict(aligned=True, leg='left', gap_ratio=.15))
    with patch('roki_ng.supervisor.game.simple_football.corner_lane', return_value=lane):
        asyncio.run(s.step())
    assert [op for op, _ in commands(g)] == ['motion.kick']


def test_alignment_does_not_rescan_goals_on_four_second_timer(tmp_path):
    import asyncio
    from test_simple_football import harness
    s, g = harness(tmp_path, ball=(.22, .052), goals=[])
    s.near = True
    s.goal_scan_needed = False
    s.last_goal_scan = -math.inf
    asyncio.run(s.step())
    assert s.observe.await_args.kwargs == {'goals': False}


def test_initial_close_ball_search_enters_alignment_without_body_turn(tmp_path):
    import asyncio
    from unittest.mock import AsyncMock
    from test_simple_football import harness, commands
    s, g = harness(tmp_path, ball=None, goals=[])
    s.look = AsyncMock(return_value={'ball': [.11, .055], 'sequence': 1})
    s.log_search_found = lambda *args: None
    asyncio.run(s.step())
    assert s.near
    assert not commands(g)

def test_repeat_goal_scan_waits_while_ball_is_rechecked(tmp_path):
    import asyncio
    import time
    from test_simple_football import harness
    s, g = harness(tmp_path, ball=(.10, .052), goals=[])
    s.opening_touch_required = False
    s.near = s.goal_scan_needed = True
    s.last_goal_scan = time.monotonic()
    asyncio.run(s.step())
    assert s.observe.await_args.kwargs == {'goals': False}
    assert s.goal_scan_needed
    s.last_goal_scan -= 3
    asyncio.run(s.step())
    assert s.observe.await_args.kwargs == {'goals': True}


def test_start_goal_from_imu_before_observation(tmp_path):
    from roki_ng.parameters import Parameters
    from test_simple_football import harness
    from test_goal_memory import sample
    s, g = harness(tmp_path)
    g.s.params = Parameters(tmp_path, robot_model='roki2')
    g.s.body_telemetry = {'body.imu': sample(.4) | {'yaw_epoch': 1}}
    s.remember_start_heading()
    assert s.expected_pan('enemy_goal') == 0
    g.s.body_telemetry['body.imu'] = sample(.7) | {'yaw_epoch': 1}
    s.remember_start_heading()
    assert s.expected_pan('enemy_goal') == pytest.approx(-.3/.00058909, abs=1)
    g.s.body_telemetry['body.imu']['yaw_epoch'] = 2
    assert s.expected_pan('enemy_goal') is None


def test_opening_pass_uses_initial_goal_after_body_turn(tmp_path):
    from test_simple_football import harness
    from test_goal_memory import sample
    s, g = harness(tmp_path)
    g.s.body_telemetry = {'body.imu': sample(.4) | {'yaw_epoch': 1}}
    s.remember_start_heading()
    g.s.body_telemetry['body.imu'] = sample(.6) | {'yaw_epoch': 1}
    for side, sign in [('left', 1), ('right', -1)]:
        s.p['game.simple_football.kickoff_side'] = side
        assert s.opening_pass_course(.6) == pytest.approx(sign*math.radians(15)-.2)


def test_ball_hint_loses_precision_after_translation_and_visual_reset(tmp_path):
    import time
    from roki_ng.body.motion.odometry import GameOdometry
    from roki_ng.parameters import Parameters
    from test_simple_football import harness
    from test_goal_memory import sample

    strategy, game = harness(tmp_path)
    game.s.params = Parameters(tmp_path, robot_model='roki2')
    game.s.body_telemetry = {'body.imu': sample() | {'yaw_epoch': 1}}
    odom = game.odometry = GameOdometry(strategy.p, 'center', 0.)
    strategy.remember_gaze(dict(received=time.monotonic(), ball=(.4,.05), parts=[]))
    odom.apply(1, {'segments':[dict(index=1, dx_m=.06, dy_m=0., imu_yaw_rad=0.)]})
    assert strategy.expected_pan('ball') is not None
    odom.apply(2, {'segments':[dict(index=1, dx_m=.28, dy_m=0., imu_yaw_rad=0.)]})
    assert strategy.expected_pan('ball') is not None
    assert strategy.ball_pan_spread > 1700
    odom.correct((0.,0.,0.), 0.)
    assert strategy.expected_pan('ball') is None
