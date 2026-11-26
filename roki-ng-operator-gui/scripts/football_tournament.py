"""Paired tactical trials. Each invocation runs one manifest entry in its own process."""
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


def setup(row, profile):
    sign = 1 if row['side']=='left' else -1
    level = row['level']
    offset = (0., .02, -.035, .04, -.015)[level]
    match = Match(side=row['side'], keeper_y=row['keeper_y'], start_y=sign*offset,
                  yaw_offset=sign*(0, 4, -6, 7, -3)[level], parameters=profile, record=True, imu_seed=(row['number']-1)//2)
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
    case = row['case']
    if case != 'centre':
        strategy.kickoff_pending = strategy.opening_touch_required = False
        strategy.attack_axis = world.yaw
    if case == 'one_post':
        model = match.model
        goal = model.body('goal_blue').id
        geoms = np.flatnonzero(model.geom_bodyid == goal)
        # Keep only the selected front upright in the attacker's rendered frame.
        keep = geoms[np.argmax(model.geom_pos[geoms,0] + sign*model.geom_pos[geoms,1])]
        masked = geoms[geoms != keep]
        match.mask = lambda t: masked
    if case == 'lost_ball':
        match.ball[:2] = [-.22, sign*.26]
        match.mask = lambda t: [match.ball_geom] if 6 <= t < 10 or 33 <= t < 36 else []
    if case == 'contested':
        match.ball[:2] = [-.21, sign*(.07+abs(offset))]
    if case == 'rebound':
        match.ball[:2] = [-1.35, row['keeper_y']]
        match.data.qvel[match.ball_qvel:match.ball_qvel+2] = [-1.35, sign*.04]
    return match


def run(row, directory, profile, visible=False):
    match = setup(row, profile)
    result = asyncio.run(match.run(75., visible))
    name = f"{row['number']:03}"
    with gzip.open(directory/f'trace-{name}.jsonl.gz', 'wt') as out:
        for sample in match.trace:
            out.write(json.dumps(sample)+'\n')
    doc = dict(row, result=result, seconds=match.data.time, events=match.events)
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
