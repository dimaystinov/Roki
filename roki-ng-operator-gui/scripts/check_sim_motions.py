#!/usr/bin/env python3
"""Exercise the Roki2 motion catalog on the MuJoCo calibration stand."""

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve()
ROOT = HERE.parents[2]
sys.path[:0] = [str(ROOT / "roki-ng"), str(HERE.parent),
                str(ROOT / "Bioloid-main/Bioloid/sim/roki"),
                str(ROOT / "Bioloid-main/Elsiros/controllers/Robofest_TEAM")]

from roki_ng.parameters import SCHEMA
from sim_motion import MotionLibrary
from sim_servo_stand import ServoStand


def catalog(library):
    commands = [
        ("game.walk", dict(kind="omni", distance_m=.1, course=0., heading=0.)),
        ("game.walk", dict(kind="omni", distance_m=.1, course=math.pi/2, heading=0.)),
        ("game.walk", dict(kind="omni", distance_m=.1, course=0., heading=.2)),
        ("motion.pose", dict(name="base_stand")),
        ("motion.pose", dict(name="crouch")),
        ("motion.pose", dict(name="crouch_centered")),
        ("motion.pose", dict(name="stand")),
        ("motion.pose", dict(name="head_field")),
        ("motion.head", dict(pan=600, tilt=-1000)),
        ("motion.head", dict(pan=-600, tilt=-1000)),
        ("motion.get_up", dict(position="upright")),
        ("motion.get_up", dict(position="stomach")),
        ("motion.get_up", dict(position="back")),
        ("motion.get_up", dict(position="left")),
        ("motion.get_up", dict(position="right")),
        ("motion.splits", dict(kind="small")),
        ("motion.splits", dict(kind="big")),
        ("motion.kick", dict(leg="left", power=60, offset=0)),
        ("motion.kick", dict(leg="right", power=60, offset=0)),
    ]
    commands.extend(("motion.jump", dict(direction=d, fraction=1.))
                    for d in ("forward", "backward", "left", "right", "turn_left", "turn_right"))
    commands.extend(("motion.slot", dict(name=path.stem))
                    for path in sorted((library.ASSETS / "slots").glob("*.json")))
    return commands


def run(output):
    library = MotionLibrary({key: spec[1] for key, spec in SCHEMA.items()})
    stand = ServoStand()
    model, data = stand.model, stand.data
    dt = float(model.opt.timestep)
    results = []
    for operation, args in catalog(library):
        label = f"{operation}:{args.get('name', args.get('direction', args.get('leg', args.get('kind', ''))))}"
        try:
            steps = library.steps(operation, args)
            mujoco.mj_resetData(model, data)
            data.qpos[:3] = stand.root_position
            data.qpos[3] = 1.
            mujoco.mj_forward(model, data)
            frames_total = 0
            changed = set()
            clipped = set()
            for step in steps:
                if len(step) < 4:
                    continue
                kind, values, frames, _ = step
                if kind != "servo" or not values:
                    continue
                start = data.ctrl.copy()
                stand.bridge.set_targets(values)
                target = data.ctrl.copy()
                changed.update(int(i) for i in np.flatnonzero(abs(target-start) > 1e-5))
                for value in values:
                    actuator, sign = stand.bridge.addresses[value.Id, value.Sio]
                    requested = (value.Data-7500)/(1698*sign)
                    if abs(requested-target[actuator]) > 1e-6:
                        clipped.add(model.actuator(actuator).name)
                data.ctrl[:] = start
                frames_total += frames
                for frame in range(frames):
                    data.ctrl[:] = start + (target-start)*(frame+1)/frames
                    for _ in range(round(.02/dt)):
                        mujoco.mj_step(model, data)
                        data.qpos[:3] = stand.root_position
                        data.qpos[3:7] = [1., 0., 0., 0.]
                        data.qvel[:6] = 0.
                if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                    raise ValueError("nonfinite MuJoCo state")
            results.append(dict(operation=operation, args=args, status="ok",
                                frames=frames_total, duration_s=frames_total*.02,
                                changed_actuators=len(changed), clipped=sorted(clipped)))
        except Exception as exc:
            results.append(dict(operation=operation, args=args, status="failed", error=str(exc)))
    report = dict(total=len(results), passed=sum(r["status"] == "ok" for r in results),
                  results=results)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
    print(json.dumps(dict(total=report["total"], passed=report["passed"],
                          failures=[r for r in results if r["status"] != "ok"],
                          clipped=[(r["operation"], r["args"], r["clipped"])
                                   for r in results if r.get("clipped")]), ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "roki-ng-operator-gui/sim/calibration/all_sim_motions.json")
    run(parser.parse_args().output)
