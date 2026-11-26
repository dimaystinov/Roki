"""Summarize completed, paired MuJoCo episodes using physics contacts and goal crossings."""
import argparse
import csv
import gzip
import json
import math
from pathlib import Path
from statistics import median


def metrics(match, trace):
    events = match['events']
    kicks = 0
    contacts = set()
    second_shots = set()
    last_blocked = None
    blocks = 0
    for _, player, action, *_ in events:
        if player == 'home':
            if action == 'motion.kick':
                kicks += 1
            elif action == 'shot_contact' and kicks not in contacts:
                if last_blocked is not None and kicks != last_blocked:
                    second_shots.add(kicks)
                contacts.add(kicks)
                last_blocked = None
        elif action in ('touch', 'shot_contact') and kicks in contacts and last_blocked != kicks:
            last_blocked = kicks
            blocks += 1
    errors = []
    detour_samples = 0
    for sample in trace:
        home = sample['players']['home']
        pose = home['estimate']
        errors.append(100*math.dist(home['actual'][:2], (pose['x_m'], pose['y_m'])))
        detour_samples += home['phase'] == 'simple_bypass'
    outcome = match['result']
    return dict(number=match['number'], case=match.get('case'), side=match['side'], split=match.get('split'),
                policy='B' if match.get('quick') else 'A', result=outcome,
                points={'time':1,'goal_blue':3,'goal_yellow':0}[outcome],
                shot_commands=kicks, distinct_shots=len(contacts), opponent_touches_after_shot=blocks,
                second_shots=len(second_shots), detour_samples=detour_samples,
                xy_median_cm=round(median(errors),1), xy_final_cm=round(errors[-1],1))


def rating(matches):
    wins = sum(m['result']=='goal_blue' for m in matches)
    losses = sum(m['result']=='goal_yellow' for m in matches)
    return sum(m['points'] for m in matches), wins-losses, sum(m['distinct_shots']>0 for m in matches)


def generate(directory):
    manifest = json.loads((directory/'manifest.json').read_text())
    entries = {}
    for m in manifest:
        name = f"{m['number']:03}"
        p = directory/f'match-{name}.json'
        if not p.exists():
            continue
        match = json.loads(p.read_text())
        if any(match[key]!=val for key,val in m.items()):
            raise ValueError(f'Manifest mismatch in {p}')
        with gzip.open(directory/f'trace-{name}.jsonl.gz','rt') as lines:
            entries[m['number']] = metrics(match, (json.loads(line) for line in lines))
    if not entries:
        return
    with (directory/'metrics.csv').open('w') as f:
        writer = csv.DictWriter(f,fieldnames=next(iter(entries.values())).keys())
        writer.writeheader()
        writer.writerows(entries.values())
    paired = [r for n,r in entries.items() if ((n-1)^1)+1 in entries]
    lines = ['# Турнир simple_football', '', f'Завершено {len(entries)} из {len(manifest)} эпизодов. Полных пар {len(paired)//2}. В таблице учитываются только полные пары.',
             '', '| Серия | Версия | Игр | Победы | 0:0 | Поражения | Очки | Игр с атакующим ударом | Повторные удары после контакта соперника |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    ratings = {}
    for split in ('selection','holdout'):
        for policy in ('A','B'):
            group = [r for r in paired if r['split']==split and r['policy']==policy]
            ratings[split,policy] = rating(group)
            counts = [sum(r['result']==s for r in group) for s in ('goal_blue','time','goal_yellow')]
            points,_,shoot = rating(group)
            lines.append('| '+' | '.join(map(str,[split,policy,len(group),*counts,points,shoot,sum(r['second_shots'] for r in group)]))+' |')
    if len(entries)==len(manifest):
        selected = ratings['selection','B'] > ratings['selection','A']
        confirmed = ratings['holdout','B'] > ratings['holdout','A']
        choice = 'B' if selected and confirmed else 'A'
        lines += ['',f'По заранее выбранному правилу остаётся **{choice}**. Критерии по порядку — очки, разница голов, число матчей с ударом. При равенстве сохраняется A.']
    lines += ['', '| Ситуация | Полных пар | Победы A/B | Матчи с ударом A/B |','|---|---:|---:|---:|']
    for case in dict.fromkeys(m['case'] for m in manifest):
        sides = [[r for r in paired if r['case']==case and r['policy']==p] for p in ('A','B')]
        wins = '/'.join(str(sum(r['result']=='goal_blue' for r in side)) for side in sides)
        shots = '/'.join(str(sum(r['distinct_shots']>0 for r in side)) for side in sides)
        lines.append(f'| {case} | {len(sides[0])} | {wins} | {shots} |')
    lines += ['','[Матрица сценариев](manifest.json) · [Измерения по каждому матчу](metrics.csv) · [Протокол и критерии](PLAN.md)',
              '', 'Команды включают слабые стартовые удары. Атакующий удар требует контакта активной стопы при сильном ударе. Повторный удар требует новой команды и контакта соперника после предыдущего удара. Начальный наброс в сценарии rebound сам по себе в эту статистику не входит.']
    (directory/'REPORT.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    generate(parser.parse_args().directory)
