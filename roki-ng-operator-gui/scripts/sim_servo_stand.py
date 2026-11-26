#!/usr/bin/env python3
"""Suspended Roki2 MuJoCo stand and preliminary servo trace comparison."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve()
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "roki-ng"))
sys.path.insert(0, str(HERE.parent))

from roki_ng.body.motion.gait import Servo
from sim_motion import ServoBridge

SCENE = (ROOT / "Bioloid-main/Bioloid/sim/roki_lab/rokilab-main"
         / "resources/assets/scene_roki2_soccer.xml")


class ServoStand:
    def __init__(self, clearance_m=.1, *, kp=50., kv=1., damping=.41,
                 frictionloss=.03):
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        self.data = mujoco.MjData(self.model)
        self.root_position = [-.6, 0., .29 + clearance_m]
        self.model.body_pos[self.model.body("calibration_stand").id] = [-.6, 0., 0.]
        self.data.qpos[:3] = self.root_position
        self.data.qpos[3:7] = [1., 0., 0., 0.]
        self.bridge = ServoBridge(self.model, self.data)
        self.head_actuator = self.model.actuator("head_yaw").id
        self.head_joint = self.model.joint("head_yaw_joint").id
        self.model.actuator_gainprm[self.head_actuator, 0] = kp
        self.model.actuator_biasprm[self.head_actuator, 1:3] = [-kp, -kv]
        self.model.dof_damping[self.model.jnt_dofadr[self.head_joint]] = damping
        self.model.dof_frictionloss[self.model.jnt_dofadr[self.head_joint]] = frictionloss
        self.ramp = None
        mujoco.mj_forward(self.model, self.data)

    @property
    def head_counts(self):
        return float(self.data.joint("head_yaw_joint").qpos[0])*16384/(2*math.pi)

    def command_head(self, pan, frames=20):
        initial = float(self.data.ctrl[self.head_actuator])
        self.bridge.set_targets([Servo(0, 1, 7500+pan)])
        target = float(self.data.ctrl[self.head_actuator])
        self.data.ctrl[self.head_actuator] = initial
        self.ramp = [initial, target, 0., frames*.02]

    def advance(self, seconds):
        steps = max(0, round(seconds/self.model.opt.timestep))
        dt = float(self.model.opt.timestep)
        for _ in range(steps):
            if self.ramp is not None:
                initial, target, elapsed, duration = self.ramp
                elapsed = min(duration, elapsed+dt)
                self.data.ctrl[self.head_actuator] = initial+(target-initial)*elapsed/duration
                self.ramp = None if elapsed >= duration else [initial, target, elapsed, duration]
            mujoco.mj_step(self.model, self.data)
            # This simulated torso is supported by a stand; the physical trace
            # may have been recorded with the robot standing on the floor.
            self.data.qpos[:3] = self.root_position
            self.data.qpos[3:7] = [1., 0., 0., 0.]
            self.data.qvel[:6] = 0.


def compare_trace(rows, **settings):
    stand = ServoStand(**settings)
    samples = []
    previous = rows[0]["t"]
    for row in rows:
        stand.advance(max(0., row["t"]-previous))
        previous = row["t"]
        if row["kind"] == "command":
            stand.command_head(row["pan"], row.get("frames", 20))
        elif row["kind"] == "sample" and row.get("measured") is not None:
            samples.append(dict(time_s=row["t"]-rows[0]["t"], target=row["pan"],
                                measured=row["measured"], simulated=stand.head_counts))
    rmse = math.sqrt(sum((r["measured"]-r["simulated"])**2 for r in samples)/len(samples))
    return dict(settings=settings, samples=samples, rmse_counts=rmse,
                max_abs_error_counts=max(abs(r["measured"]-r["simulated"]) for r in samples))


def show_trace(rows, **settings):
    """Replay the same command trace in a separate visible stand window."""
    import mujoco.viewer

    stand = ServoStand(**settings)
    with mujoco.viewer.launch_passive(stand.model, stand.data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = [-.65, 0., .38]
        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -14
        previous = rows[0]["t"]
        for row in rows:
            if not viewer.is_running():
                return
            remaining = max(0., row["t"]-previous)
            previous = row["t"]
            while remaining > 0 and viewer.is_running():
                dt = min(.02, remaining)
                stand.advance(dt)
                viewer.sync()
                time.sleep(dt)
                remaining -= dt
            if row["kind"] == "command":
                stand.command_head(row["pan"], row.get("frames", 20))
        while viewer.is_running():
            viewer.sync()
            time.sleep(.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--kp", type=float, default=50.)
    parser.add_argument("--kv", type=float, default=1.)
    parser.add_argument("--damping", type=float, default=.41)
    parser.add_argument("--frictionloss", type=float, default=.03)
    parser.add_argument("--viewer", action="store_true", help="show a separate stand window")
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.trace.read_text().splitlines()]
    result = compare_trace(rows, kp=args.kp, kv=args.kv,
                           damping=args.damping, frictionloss=args.frictionloss)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({key: result[key] for key in ("rmse_counts", "max_abs_error_counts")}))
    if args.viewer:
        show_trace(rows, kp=args.kp, kv=args.kv,
                   damping=args.damping, frictionloss=args.frictionloss)


if __name__ == "__main__":
    main()
