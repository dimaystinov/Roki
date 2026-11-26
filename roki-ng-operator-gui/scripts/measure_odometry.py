"""Measure the robot's calibration jobs in a clear MuJoCo scene. Never writes robot profiles."""
import argparse
import asyncio
import json
import math
from pathlib import Path
from unittest.mock import patch

import sim_roki2 as sim

CASES = ['short','long','side_left','side_right','spot','backwards','rotation',
         'jump_forward','jump_backward','jump_left','jump_right','turn_left','turn_right',
         'walk_check','diagonal_check','turn_walk_check',
         'custom','jump_on_spot_series','get_up_front','get_up_back','get_up_left','get_up_right','get_up','regular_kick','jump_forward_series','jump_backward_series','jump_left_series','jump_right_series']


async def measure(case, directory, params, yaw_deg=0., fall_deg=90.):
    settings = sim.load_settings(sim.SETTINGS)
    settings['opponents']['enabled'] = settings['person']['enabled'] = False
    settings['layout'].update(robot_spawn_xy=[0.,0.], robot_yaw_deg=yaw_deg, ball_spawn_xy=[1.3,1.])
    w = sim.World(settings=settings, parameters=params)
    g = sim.GameAdapter(w, params)
    if case.startswith('get_up_'):
        angle = math.radians(fall_deg) * (1 if case in ('get_up_front','get_up_right') else -1)
        side = case in ('get_up_left','get_up_right')
        tilt = [math.cos(angle/2), math.sin(angle/2) if side else 0.,
                0. if side else math.sin(angle/2), 0.]
        heading = w.data.qpos[3:7].copy()
        w.data.qpos[2] = .13
        sim.mujoco.mju_mulQuat(w.data.qpos[3:7], heading, tilt)
        sim.mujoco.mj_forward(w.model, w.data)
    for _ in range(20):
        w.advance(.02)
    start = (w.x,w.y,w.yaw)
    low = w.data.qpos[2]
    samples = []
    fall = False
    original = w.advance
    def step(dt):
        nonlocal low, fall
        original(dt)
        q = w.data.qpos
        low = min(low, float(q[2]))
        fall |= q[2]<.17 or (1-2*(q[4]**2+q[5]**2)) < .5
        body = g.football_body
        job = body.jobs.get(body.active,{}) if body else {}
        samples.append([round(w.data.time,3),w.x,w.y,w.yaw,float(q[2]),job.get('stage'),job.get('progress',0)])
        if body and body.calibration_pending is not None:
            # Read-only trial acknowledges the calibration into this temporary body only.
            ident = body.calibration_pending
            body.jobs[ident]['saved'] = True
            body.calibration_pending = None
        if w.data.time > 180:
            raise RuntimeError("calibration exceeded 180 simulated seconds")
    w.advance = step
    async def fast_sleep(_):
        pass
    error = None
    job = {}
    try:
        with patch('time.monotonic',lambda:100+w.data.time), patch('time.monotonic_ns',lambda:round((100+w.data.time)*1e9)), patch('asyncio.sleep',fast_sleep):
            if case.endswith('_series'):
                op,args='test.start',dict(name='jump_test',direction=case.removeprefix('jump_').removesuffix('_series'),count=10)
            elif case=='custom':
                op,args='test.start',dict(name='run_test',mode='custom',cycles=10,step_mm=24)
            elif case == 'regular_kick' or case.startswith('get_up'):
                op,args='test.start',dict(name='kick_test',mode='regular') if case=='regular_kick' else dict(name='get_up_test')
            elif case=='turn_walk_check':
                await g._move('game.jump',dict(direction='turn_left',fraction=1.))
                await g._prepare(params,'center')
                op,args='game.walk',dict(kind='omni',distance_m=.18,course=0.,heading=0.,yaw_offset=-w.yaw)
            elif case.startswith('jump_') or case.startswith('turn_'):
                op, args = 'game.jump',dict(direction=case.removeprefix('jump_'),fraction=1.)
            elif case.endswith('_check'):
                op,args='game.walk',dict(kind='omni',distance_m=.18,course=.4 if case=='diagonal_check' else 0.,heading=0.,yaw_offset=0.)
            else:
                op,args='test.start',({'name':'rotation_test'} if case=='rotation' else dict(name='run_test',mode=case))
            try:
                job=await g._body_motion(op,args)
            except (sim.Fault,RuntimeError) as exc:
                error=str(exc)
                if g.football_body and g.football_body.jobs:
                    job=next(reversed(g.football_body.jobs.values()))
            for _ in range(100 if case.startswith("get_up") else 20):
                step(.02)
        result=dict(case=case,command=[op,args],start=start,dx_m=w.x-start[0],dy_m=w.y-start[1],
                    yaw_rad=sim.wrap(w.yaw-start[2]),min_z_m=low,fall=bool(fall),error=error,
                    final_upright=bool(w.data.qpos[2]>.17 and 1-2*sum(w.data.qpos[4:6]**2)>.5),
                    spawn_yaw_deg=yaw_deg,fixture_tilt_deg=fall_deg,
                    final_z_m=float(w.data.qpos[2]),final_up=float(1-2*sum(w.data.qpos[4:6]**2)),
                    job=job,odometry=g.odometry.state(),time_s=w.data.time,
                    columns=['t','x','y','yaw','z','stage','progress'],samples=samples)
        (directory/(case+'.json')).write_text(json.dumps(result,indent=None)+'\n')
        print(case,result['dx_m'],result['dy_m'],result['yaw_rad'],bool(fall),error,flush=True)
    finally:
        w.renderer.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('cases',nargs='+',choices=CASES)
    parser.add_argument('--parameters',type=Path,required=True)
    parser.add_argument('--yaw-deg',type=float,default=0.)
    parser.add_argument('--fall-deg',type=float,default=90.)
    opts=parser.parse_args()
    opts.directory.mkdir(exist_ok=True,parents=True)
    p=json.loads(opts.parameters.read_text())
    for case in opts.cases:
        asyncio.run(measure(case,opts.directory,p,opts.yaw_deg,opts.fall_deg))
