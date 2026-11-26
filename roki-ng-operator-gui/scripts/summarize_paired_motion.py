#!/usr/bin/env python3
"""Compare physical encoder traces with MuJoCo joint angles by servo address."""

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "roki-ng"), str(Path(__file__).parent)]
from sim_motion import ServoBridge
from sim_servo_stand import SCENE
from roki_ng.motion.model import Robot
from roki_ng.motion.slots import MIRRORED, joints


def summarize(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    real = [row for row in rows if row["source"] == "real" and row.get("valid") and row.get("measured")]
    fresh = []
    seen = set()
    for row in real:
        key = row.get("sequence")
        if key is not None and key in seen:
            continue
        seen.add(key)
        fresh.append(row)
    real = fresh
    sim = [row for row in rows if row["source"] == "sim"]
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    bridge = ServoBridge(model, mujoco.MjData(model))
    names = {(servo, bus): name for name, (servo, bus) in joints(Robot()).items()}
    baseline = rows[0]["snapshot"]["data"]["measured"]
    summaries = []
    for (servo, bus), (actuator, direction) in sorted(bridge.addresses.items()):
        unit = servo*2+bus-1
        if unit >= len(baseline) or baseline[unit] is None:
            continue
        joint = int(model.actuator_trnid[actuator, 0])
        qpos = int(model.jnt_qposadr[joint])
        name = model.joint(joint).name
        mirror = -1 if names.get((servo, bus)) in MIRRORED else 1
        factor = 1698*1.536*direction*mirror
        pairs = [(row["measured"][unit], min(sim, key=lambda item: abs(item["t_s"]-(row["t_s"]-(row.get("age_ms") or 0)/1000)))["qpos"][qpos]*factor)
                 for row in real if row["measured"][unit] is not None]
        if not pairs:
            continue
        real_change = pairs[-1][0]-baseline[unit]
        sim_change = pairs[-1][1]-baseline[unit]
        errors = [a-b for a, b in pairs]
        first_ctrl = sim[0]["ctrl"][actuator]
        commanded_excursion = max(abs((row["ctrl"][actuator]-first_ctrl)*factor)
                                  for row in sim)
        if commanded_excursion < 40:
            continue
        summaries.append(dict(name=name, unit=unit, real_delta=round(real_change, 1),
                           sim_delta=round(sim_change, 1),
                           commanded_excursion=round(commanded_excursion, 1),
                           final_error=round(errors[-1], 1),
                           rmse=round(math.sqrt(sum(e*e for e in errors)/len(errors)), 1)))
    return dict(file=path.name, operation=next(row["operation"] for row in rows if row["source"] == "command"),
                joints=summaries, max_rmse=max((item["rmse"] for item in summaries), default=0))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = [summarize(path) for path in args.traces]
    payload = json.dumps(result, ensure_ascii=False, indent=2)+"\n"
    if args.output:
        args.output.write_text(payload)
    print(payload)
