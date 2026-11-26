#!/usr/bin/env python3
"""Record one physical Roki2 motion and the same MuJoCo servo frames."""

import argparse
import asyncio
import json
import math
import sys
import time
from pathlib import Path

import mujoco
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "roki-ng"), str(Path(__file__).parent),
                str(ROOT / "Bioloid-main/Bioloid/sim/roki"),
                str(ROOT / "Bioloid-main/Elsiros/controllers/Robofest_TEAM")]

from roki_ng.client import Client
from roki_ng.motion.slots import MIRRORED, joints
from roki_ng.parameters import SCHEMA
from sim_motion import MotionLibrary
from sim_servo_stand import ServoStand


async def capture(operation, args, output, host):
    library = MotionLibrary({key: spec[1] for key, spec in SCHEMA.items()})
    steps = library.steps(operation, args)
    duration = sum(step[2] for step in steps if step[0] == "servo")*.02
    stand = ServoStand()
    client = Client(host)
    rows = []
    start = time.monotonic()

    def record(source, **values):
        rows.append(dict(t_s=round(time.monotonic()-start, 6), source=source, **values))

    async def real_samples(done):
        while not done.is_set():
            sample = await client.request("data.snapshot", {"topic": "body.servos"})
            payload = sample.get("data", {})
            record("real", measured=payload.get("measured"), sent=payload.get("sent"),
                   valid=sample.get("valid") and payload.get("valid"),
                   sequence=payload.get("sequence"), age_ms=sample.get("age_ms"),
                   source_mono_ns=sample.get("source_mono_ns"))
            await asyncio.sleep(.05)

    async def simulated():
        for step in steps:
            if step[0] != "servo" or len(step) < 4 or not step[1]:
                continue
            _, values, frames, _ = step
            before = stand.data.ctrl.copy()
            stand.bridge.set_targets(values)
            target = stand.data.ctrl.copy()
            stand.data.ctrl[:] = before
            for frame in range(frames):
                stand.data.ctrl[:] = before+(target-before)*(frame+1)/frames
                stand.advance(.02)
                record("sim", qpos=stand.data.qpos.tolist(), ctrl=stand.data.ctrl.tolist())
                await asyncio.sleep(.02)
        # Servo interpolation ends before the joint reaches its final angle.
        # Record the settling period as well, matching the real telemetry window.
        for _ in range(50):
            stand.advance(.02)
            record("sim", qpos=stand.data.qpos.tolist(), ctrl=stand.data.ctrl.tolist())
            await asyncio.sleep(.02)

    done = asyncio.Event()
    sampler = sim_task = None
    try:
        await client.connect()
        status = await client.request("system.status")
        if status["state"] != "IDLE" or status.get("owner") is not None or not status.get("motion_ready"):
            raise RuntimeError(f"Robot not idle and ready: {status}")
        baseline = await client.request("data.snapshot", {"topic": "body.servos"})
        record("baseline", snapshot=baseline)
        measured = baseline.get("data", {}).get("measured") or []
        names = {(servo, bus): name for name, (servo, bus) in joints(library.model).items()}
        for (servo, bus), (actuator, direction) in stand.bridge.addresses.items():
            unit = servo*2+bus-1
            if unit >= len(measured) or measured[unit] is None:
                continue
            mirror = -1 if names.get((servo, bus)) in MIRRORED else 1
            angle = measured[unit]/(1698*1.536*direction*mirror)
            joint = int(stand.model.actuator_trnid[actuator, 0])
            address = int(stand.model.jnt_qposadr[joint])
            low, high = stand.model.jnt_range[joint]
            stand.data.qpos[address] = min(high, max(low, angle))
            stand.data.ctrl[actuator] = stand.data.qpos[address]
        mujoco.mj_forward(stand.model, stand.data)
        await client.request("control.acquire")
        await client.request("mode.set", {"mode": "MANUAL"})
        accepted = await client.request(operation, args)
        record("command", operation=operation, args=args, reply=accepted)
        sampler = asyncio.create_task(real_samples(done))
        sim_task = asyncio.create_task(simulated())
        job_id = accepted.get("job_id")
        deadline = time.monotonic()+max(15., duration*2+10)
        if job_id:
            while time.monotonic() < deadline:
                job = await client.request("job.status", {"job_id": job_id})
                record("job", job=job)
                if job["status"] in ("completed", "failed", "cancelled"):
                    break
                await asyncio.sleep(.2)
            else:
                raise TimeoutError(f"Physical motion {operation} exceeded {deadline-start:.1f}s")
        else:
            await asyncio.sleep(max(1., duration))
        await sim_task
        record("result", physical_status=job["status"] if job_id else "accepted")
    finally:
        done.set()
        if sampler:
            await asyncio.gather(sampler, return_exceptions=True)
        if sim_task and not sim_task.done():
            sim_task.cancel()
            await asyncio.gather(sim_task, return_exceptions=True)
        if client.lease_epoch:
            try:
                await client.request("mode.set", {"mode": "IDLE"})
                await client.request("control.release")
            except Exception as exc:
                record("cleanup_error", error=str(exc))
        await client.close()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text("".join(json.dumps(row, ensure_ascii=False)+"\n" for row in rows))
    print(json.dumps(dict(operation=operation, output=str(output), samples=sum(
        row["source"] == "real" for row in rows), sim_frames=sum(row["source"] == "sim" for row in rows),
        result=next((row for row in reversed(rows) if row["source"] == "result"), None)), ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation")
    parser.add_argument("args", help="JSON object")
    parser.add_argument("--host", default="192.168.0.244")
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args()
    asyncio.run(capture(options.operation, json.loads(options.args), options.output, options.host))
