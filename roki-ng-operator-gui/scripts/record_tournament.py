"""Checkpoint completed MuJoCo matches in the local experiment repository."""
import argparse
import json
import subprocess
import time
from pathlib import Path

from football_results import generate


def checkpoint(directory, last):
    rows = sorted(directory.glob('match-*.json'))
    if len(rows)==last:
        return last
    generate(directory)
    files = [directory/'metrics.csv', directory/'REPORT.md']
    for row in rows:
        number = int(row.stem.split('-')[-1])
        files.extend((row, directory/f'trace-{number:03}.jsonl.gz', directory/'logs'/f'{number}.log'))
    # Only snapshot results for which the writer published the atomic summary.
    subprocess.run(['git','add','--',*files], check=True)
    subprocess.run(['git','commit','--only','-qm',f'Record {len(rows)} completed tournament matches','--',*files],check=True)
    return len(rows)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    total=len(json.loads((args.directory/'manifest.json').read_text()))
    last=checkpoint(args.directory,-1) if list(args.directory.glob('match-*.json')) else 0
    while last<total:
        time.sleep(12)
        count=len(list(args.directory.glob('match-*.json')))
        if count==total or count//10>last//10:
            last=checkpoint(args.directory,last)
