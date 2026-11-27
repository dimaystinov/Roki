"""Paired tactical trials. Each invocation runs one manifest entry in its own process."""
from contextlib import nullcontext
import mujoco.viewer
import argparse
import asyncio
import gzip
import importlib.util
import os
import json
from pathlib import Path

import numpy as np
from watch_match import Match

CASES = ('centre', 'one_post', 'lost_ball', 'contested', 'rebound')


def schedule():
    rows = []
    for level, keeper_y in enumerate((0., -.16, .22, .35, -.29)):
        for case in CASES:
            for side in ('left', 'right'):
                for quick in (False, True):
                    rows.append(dict(number=len(rows)+1, level=level, case=case, side=side,
                                     keeper_y=keeper_y, quick=quick,
                                     split='selection' if level<3 else 'holdout'))
    return rows


def setup(row, profile, **restart):
    sign = 1 if row['side']=='left' else -1
    level = row['level']
    offset = (0., .02, -.035, .04, -.015)[level]
    match = Match(side=row['side'], keeper_y=row['keeper_y'], start_y=sign*offset,
                  yaw_offset=sign*(0, 4, -6, 7, -3)[level], parameters=profile, record=True, imu_seed=(row['number']-1)//2, **restart)
    world, game, strategy, _ = match.players[0]
    strategy.p['game.simple_football.take_clear_shot'] = row['quick']
    opponent = os.environ.get("ROKI_SIM_OPPONENT")
    if opponent:
        spec = importlib.util.spec_from_file_location("roki_ng.supervisor.game.fixed_opponent", opponent)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        other, adapter, _, delay = match.players[2]
        visitor = module.SimpleFootball(adapter, adapter.p, "center", delay)
        visitor.opening_touch_required = False
        visitor.attack_axis = other.yaw
        match.players[2] = (other, adapter, visitor, delay)
    # Keep the opening block as the original control. Pair both policies
    # against identical keepers and an identical pursuing opponent.
    for index in (1, 3):
        match.players[index][2].p['sim.keeper_intercept'] = True
    if row['number'] > 20:
        match.players[2][2].p['sim.direct_pursuit'] = True
        strategy.p['sim.precise_strike'] = row['quick']
    case = row['case']
    if case == 'one_post':
        model = match.model
        goal = model.body('goal_blue').id
        geoms = np.flatnonzero(model.geom_bodyid == goal)
        # Keep only the selected front upright in the attacker's rendered frame.
        positions = model.geom_pos[geoms]
        upright = (sign * positions[:, 1] > 0.4) & (positions[:, 2] > 0.25)
        masked = geoms[~upright]
        match.mask = lambda t: masked
    if case == 'lost_ball':
        match.mask = lambda t: [match.ball_geom] if 6 <= t < 10 or 33 <= t < 36 else []
    if case == 'contested':
        match.players[2][2].p['sim.direct_pursuit'] = True
    if case == 'rebound':
        match.players[1][2].p['sim.keeper_intercept'] = True
    return match


def run(row, directory, profile, visible=False):
    match = setup(row, profile)
    start_ball = list(match.ball[:2])
    start_velocity = list(match.data.qvel[match.ball_qvel:match.ball_qvel+2])
    if any(start_ball) or any(start_velocity):
        raise ValueError("Kickoff requires a stationary ball at midfield")
    score = {'goal_blue': 0, 'goal_yellow': 0}
    events, trace = [], []
    elapsed = 0.
    shared = match.model, match.data
    viewer = mujoco.viewer.launch_passive(*shared) if visible else None
    try:
        while elapsed < 300.-1e-6:
            title = f"Match {row['number']}/100 | {row['case']} | RED {score['goal_blue']} : {score['goal_yellow']} BLUE"
            outcome = asyncio.run(match.run(300.-elapsed, visible, title=title, viewer=viewer))
            if outcome == 'closed':
                raise SystemExit("Viewer closed before final whistle")
            for prefix, bounds in match.keeper_foot_limits.items():
                match.events.append([match.data.time, prefix.rstrip('_'), 'feet_extent_xy', bounds.tolist()])
            events.extend([round(elapsed+t,3), *event] for t,*event in match.events)
            trace.extend(s | {'time': round(elapsed+s['time'],3)} for s in match.trace)
            elapsed += match.data.time
            if outcome == 'time':
                break
            score[outcome] += 1
            print(f"GOAL {elapsed:.3f}s red {score['goal_blue']} : {score['goal_yellow']} blue", flush=True)
            if elapsed < 300.-1e-6:
                with viewer.lock() if viewer else nullcontext():
                    match = setup(row, profile, shared=shared, visitor_kickoff=outcome=='goal_blue', elapsed=elapsed)
                kicking = 'visitor' if outcome == 'goal_blue' else 'home'
                events.append([round(elapsed,3), 'referee','kickoff', kicking, list(match.ball[:2])])
    finally:
        if viewer:
            viewer.close()
    result = ('goal_blue' if score['goal_blue'] > score['goal_yellow'] else
              'goal_yellow' if score['goal_yellow'] > score['goal_blue'] else 'time')
    name = f"{row['number']:03}"
    with gzip.open(directory/f'trace-{name}.jsonl.gz', 'wt') as out:
        for sample in trace:
            out.write(json.dumps(sample)+'\n')
    mouth = match.players[0][2].specs[1 - match.players[0][2].p['match.own_goal']]
    doc = dict(row, start_ball_xy=start_ball, start_ball_velocity_xy=start_velocity, result=result, seconds=elapsed, events=events, score=score,
               blue_goal_x_m=mouth['x'], blue_inner_width_m=mouth['width'], blue_centre_y_m=mouth['y'],
               ball_radius_m=float(match.model.geom_size[match.ball_geom, 0]))
    tmp = directory/f'match-{name}.tmp'
    tmp.write_text(json.dumps(doc, indent=2)+'\n')
    tmp.replace(directory/f'match-{name}.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--number', type=int, choices=range(1,101))
    parser.add_argument('--parameters', type=Path)
    parser.add_argument("--visible", action="store_true")
    args=parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=True)
    rows=schedule()
    if args.number is None:
        (args.directory/'manifest.json').write_text(json.dumps(rows,indent=2)+'\n')
    else:
        run(rows[args.number-1],args.directory,json.loads(args.parameters.read_text()), args.visible)
