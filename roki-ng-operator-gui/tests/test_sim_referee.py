import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import mujoco
import pytest
from sim_referee import PitchReferee


@pytest.fixture
def pitch():
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
    <body name="home" pos="1 1 .1"><geom size=".1"/></body>
    <body name="away" pos="-1 -1 .1"><geom size=".1"/></body>
    <body><freejoint name="soccer_ball_joint"/><geom name="ball" size=".045"/></body>
    </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    return PitchReferee(model,data,dict(field_length_m=4.,field_width_m=3.,
                        blue_goal_x_m=-1.7,yellow_goal_x_m=1.7),{'away':'blue','home':'yellow'})


@pytest.mark.parametrize('team,x,y,spot',[
 ('blue',-1.2,1.5,[-.8,.65]), ('yellow',-1.2,-1.5,[0.,-.65]),
 ('yellow',1.2,-1.5,[.8,-.65]),(None,0.,1.5,[0.,.65]),
 ('blue',-1.95,-.8,[-.8,-.65]),('blue',1.95,.8,[0.,.65])])
def test_whole_ball_and_last_team(pitch,team,x,y,spot):
    pitch.last_team=team
    pitch.ball.qpos[:3]=[x,y,.25]
    pitch.ball.qvel[:]=3.
    kind,payload=pitch.update((x*.9,y*.9,.25))
    assert kind=='throw_in'
    assert payload['position'][:2]==pytest.approx(spot)
    assert list(pitch.ball.qvel)==[0.]*6
    assert pitch.update(payload['position']) is None


def test_obstruction_moves_towards_same_touchline(pitch):
    pitch.data.xpos[pitch.model.body('home').id,:2]=[0.,.65]
    pitch.ball.qpos[:3]=[-.2,1.5,.1]
    _,info=pitch.update((-.2,1.3,.1))
    assert info['position'][0]==0 and .85<info['position'][1]<1.20


def test_line_goal_and_over_crossbar(pitch):
    pitch.ball.qpos[:3]=[0.,pitch.limits[1]+.04,.4]
    assert pitch.update((0.,1.3,.4)) is None
    pitch.ball.qpos[:3]=[-1.95,.2,.2]
    assert pitch.update((-1.8,.2,.2))[0]=='goal_blue'
    pitch.reset()
    pitch.ball.qpos[:3]=[-1.95,.2,.7]
    assert pitch.update((-1.8,.2,.7))[0]=='goal_blue'
    pitch.reset()
    pitch.ball.qpos[:3]=[-1.95,.2,.92]
    assert pitch.update((-1.8,.2,.92))[0]=='throw_in'


def test_pickup_skips_occupied_farthest_point(pitch):
    pitch.ball.qpos[:2] = [.7, .5]
    home = pitch.model.body('home').id
    away = pitch.model.body('away').id
    assert pitch.neutral_reentry(home) == pytest.approx((-.8, -.65))
    pitch.data.xpos[away, :2] = [-.8, -.65]
    assert pitch.neutral_reentry(home) == pytest.approx((-.8, .65))
