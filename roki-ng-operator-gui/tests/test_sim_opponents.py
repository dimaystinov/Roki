import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import sim_roki2 as sim


@pytest.mark.parametrize("name", ["opponent_keeper", "opponent_forward"])
def test_opponent_visible_fixed_and_blocks_ball(name):
    world = sim.World()
    model, data = world.model, world.data
    try:
        root = model.body(name).id
        initial = data.body(name).xpos.copy()
        geom_ids = np.flatnonzero(model.body_rootid[model.geom_bodyid] == root)
        world.command_head(0, 0)
        for _ in range(45):
            world.advance(.02)
        world.renderer.enable_segmentation_rendering()
        world.renderer.update_scene(data, camera="sim_head_camera", scene_option=world.camera_options)
        labels = world.renderer.render()
        assert np.any(np.isin(labels[:, :, 0], geom_ids) &
                      (labels[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM))

        # Roll onto one foot from directly in front of each stationary player.
        data.qpos[world.ball_qpos:world.ball_qpos+3] = initial[0]+.24, initial[1]+.05, .045
        data.qvel[world.ball_qvel:world.ball_qvel+6] = [-1, 0, 0, 0, -1/.045, 0]
        mujoco.mj_forward(model, data)
        hit = False
        for _ in range(500):
            mujoco.mj_step(model, data)
            for c in data.contact:
                if world.ball_geom in (c.geom1, c.geom2) and (c.geom1 in geom_ids or c.geom2 in geom_ids):
                    hit = True
        assert hit
        np.testing.assert_array_equal(data.body(name).xpos, initial)
    finally:
        world.renderer.close()


@pytest.mark.parametrize("entry", ["center", "left", "right"])
def test_forward_mirrors_restart_pose(entry):
    world = sim.World()
    try:
        pose = sim.GameOdometry(sim.sim_parameters(), entry, 0.)
        world.place_opponent((pose.x, pose.y), pose.yaw)
        mujoco.mj_forward(world.model, world.data)
        forward = world.data.body("opponent_forward")
        np.testing.assert_allclose(forward.xpos[:2], -np.array([pose.x,pose.y])*max(1., .40/np.hypot(pose.x,pose.y)))
        np.testing.assert_allclose(forward.xmat.reshape(3, 3)[:2, 0],
                                   [-np.cos(pose.yaw), -np.sin(pose.yaw)], atol=1e-12)
    finally:
        world.renderer.close()


def test_match_players_own_disjoint_servos_and_mirrored_roles():
    from watch_match import Match, SimpleGoalkeeper

    match = Match()
    try:
        home, keeper, visitor, ally = match.players
        assert isinstance(ally[2], SimpleGoalkeeper)
        assert isinstance(keeper[2], SimpleGoalkeeper)
        assert [p[3] for p in match.players] == [0., 0., 30., 0.]
        assert [p[2].enemy for p in match.players] == ['blue', 'yellow', 'yellow', 'blue']
        np.testing.assert_allclose(visitor[0].data.qpos[:2], [-.40, 0.])
        for world, *_ in match.players:
            root = match.model.body(world.prefix+'torso_1').id
            covers = np.flatnonzero((match.model.geom_bodyid == root)
                                    & (match.model.geom_dataid == match.model.mesh('torso_1_mesh_6').id)
                                    & (match.model.geom_matid >= 0))
            colour = 'Opaque(0,0,255)' if world.prefix in ('keeper_', 'visitor_') else 'Plastic_-_Translucent_Glossy_(Red)'
            assert match.model.geom_matid[covers].tolist() == [match.model.material(colour).id]
            assert len(world.data.ctrl) == len(world.servos.addresses)
            for index, _ in world.servos.addresses.values():
                assert 0 <= index < len(world.data.ctrl)
        assert not np.shares_memory(home[0].data.ctrl, visitor[0].data.ctrl)
        assert not np.shares_memory(keeper[0].data.ctrl, visitor[0].data.ctrl)
    finally:
        match.renderer.close()


def test_child_shoes_share_ball_material_and_stay_outside_during_walk():
    from sim_spectators import CHILD, move_spectators, spectator_ids
    from watch_match import match_model
    model = match_model(True)
    data = mujoco.MjData(model)
    ids = spectator_ids(model)
    ball = model.geom('ball')
    shoes = [model.geom(f'shoe_{CHILD}_{sign}') for sign in (-1,1)]
    for shoe in shoes:
        np.testing.assert_equal(ball.rgba, shoe.rgba)
        np.testing.assert_equal(ball.matid, shoe.matid)
    positions = []
    for t in np.arange(0,14,.1):
        move_spectators(data,ids,dict(blue_goal_x_m=-1.7,field_width_m=3.),t)
        mujoco.mj_kinematics(model,data)
        positions.append([data.mocap_pos[torso,:2].copy() for torso,_ in ids])
        for shoe in shoes:
            y_radius = np.linalg.norm(data.geom_xmat[shoe.id].reshape(3,3)[1]*shoe.size)
            assert abs(data.geom_xpos[shoe.id,1])-y_radius>1.2
    assert (np.linalg.norm(np.diff(positions,axis=0),axis=2)>0.).all()
