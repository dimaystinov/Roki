"""Ten matched left/right trials with differing ball bearings and keeper positions."""
import argparse
import asyncio
import json
from pathlib import Path
from watch_match import Match

LAYOUTS = [(0., 0., 0.), (.16,.025,4.), (-.16,-.025,-4.), (.30,-.045,7.), (-.30,.045,-7.)]

def trial(number, directory, parameters=None):
    side = 'left' if number % 2 else 'right'
    keeper, start, yaw = LAYOUTS[(number-1)//2]
    match = Match(side=side, keeper_y=keeper, start_y=start, yaw_offset=yaw, parameters=parameters, record=True)
    result = asyncio.run(match.run(120., False))
    summary = dict(number=number, side=side, keeper_y=keeper, start_y=start, yaw_deg=yaw,
                   result=result, seconds=round(match.data.time,2), events=match.events)
    directory.mkdir(parents=True, exist_ok=True)
    (directory/f'trace-{number:02d}.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in match.trace))
    (directory/f'match-{number:02d}.json').write_text(json.dumps(summary,indent=2)+'\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('number', type=int, choices=range(1,11))
    parser.add_argument('directory', type=Path)
    parser.add_argument('--parameters', type=Path)
    args = parser.parse_args()
    trial(args.number, args.directory, json.loads(args.parameters.read_text()) if args.parameters else None)
