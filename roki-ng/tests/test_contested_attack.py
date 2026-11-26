import asyncio
import math
import time

import cv2
import pytest

from roki_ng.detection.football import goal_parts
from roki_ng.supervisor.game.expected_gaze import ExpectedGaze
from test_football_vision import picture, Projector
from test_simple_football import commands, harness


def test_rolling_ball_prediction_expires_and_drops_after_own_walk():
    gaze = ExpectedGaze()
    for t, y in [(3., 0.), (3.1, .08)]:
        gaze.remember('ball', (.6,y), yaw=0., epoch=1, now=t)
    assert gaze.ball_region(0.,1,3.4)[1] > .08
    assert gaze.ball_region(0.,2,3.4) is None
    assert gaze.ball_region(0.,1,6.) is None
    gaze.forget_velocity()
    assert gaze.ball_region(0.,1,3.4) is None


@pytest.mark.parametrize('side', [-1, 1])
def test_bypass_to_other_side_of_obstacle(tmp_path, side):
    ball = (.65, -.052)
    strategy, game = harness(tmp_path, ball=ball, goals=[])
    strategy.attack_axis = 0.
    strategy.last_scene = dict(confirmed=True, received=time.monotonic(),
        parts=[dict(kind='obstacle',points=[[.25,side*.055]],radius=.08)])
    asyncio.run(strategy.attack_after_pass(ball, []))
    [(op, payload)] = commands(game)
    assert op == 'game.walk'
    assert payload['course']*side < 0


def test_only_identifiable_single_post_refines_heading(tmp_path):
    s, _ = harness(tmp_path)
    s.expected_gaze.targets.update(enemy_goal=(0.,1,10.),post_0=(-.28,1,10.),post_1=(.28,1,10.))
    s.correct_single_post((1., 0.), (0.,1), 11.)
    assert s.attack_heading is None
    s.correct_single_post((1.,math.tan(.31)),(0.,1),11.)
    assert s.attack_heading == pytest.approx(.03)
    assert not s.goal_views


def test_field_silhouette_with_white_line_and_orange_ball(tmp_path):
    img, p = picture(tmp_path, left=False,right=False,bar=False)
    cv2.line(img, (12,540),(780,540),(255,255,255),8)
    cv2.rectangle(img,(270,475),(320,605),(30,30,30),-1)
    cv2.circle(img,(470,580),16,(0,128,255),-1)
    parts = goal_parts(img,p,Projector(),(),dict(height=.70,x=0.,y=0.,pan=0.))
    obstacles = [part for part in parts if part['kind']=='obstacle']
    assert len(obstacles)==1
    assert obstacles[0]['rect'][0] == 270


def test_loose_ball_after_shot_is_reacquired_and_struck_again(tmp_path):
    strategy, game = harness(tmp_path, goals=[])
    strategy.opening_touch_required = False
    strategy.attack_axis = 0.
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    async def replay():
        for ball, verified in [((.115,-.052), True), ((.48,.08),False),
                               ((.115,-.052),False), ((.115,-.052),True)]:
            strategy.last_scene = dict(confirmed=True, received=time.monotonic())
            await strategy.attack_after_pass(ball, [], rechecked=verified)
    asyncio.run(replay())
    assert [op for op, _ in commands(game)] == ['motion.kick', 'game.walk', 'motion.kick']


def test_clear_ray_policy_uses_confirmed_strike_without_extra_turn(tmp_path):
    ball = (.115, -.052)
    strategy, game = harness(tmp_path, ball=ball)
    strategy.attack_axis = 0.
    strategy.opening_touch_required = False
    strategy.p['game.simple_football.take_clear_shot'] = True
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    goal = dict(colour=strategy.enemy, shootable=True, ends=[[1., -.5], [1., .5]],
                opening_source={'occupied': [[.65, .90]]})
    strategy.last_scene = dict(confirmed=True, received=time.monotonic())
    asyncio.run(strategy.attack_after_pass(ball, [goal]))
    assert not commands(game)
    asyncio.run(strategy.attack_after_pass(ball, [goal], rechecked=True))
    assert [op for op, _ in commands(game)] == ['motion.kick']


def test_uncertain_map_requires_visual_aim_but_accepts_fresh_goal(tmp_path):
    from roki_ng.body.motion.odometry import GameOdometry
    ball = (.115, -.052)
    strategy, game = harness(tmp_path, ball=ball)
    game.odometry = GameOdometry(strategy.p, 'center', 0.)
    game.odometry.path = 1.3
    strategy.attack_axis = 0.
    strategy.opening_touch_required = False
    strategy.p['game.soft_kick.right.toe_x_mm'] = 50.
    strategy.last_scene = dict(confirmed=True, received=time.monotonic())
    asyncio.run(strategy.attack_after_pass(ball, [], rechecked=True))
    assert not commands(game)
    assert strategy.goal_scan_needed
    clear_goal = dict(colour=strategy.enemy, shootable=True, ends=[[1., -.4],[1., .4]],
                      opening_source={'occupied': []})
    asyncio.run(strategy.attack_after_pass(ball, [clear_goal], rechecked=True))
    assert commands(game)[-1][0] == 'motion.kick'
