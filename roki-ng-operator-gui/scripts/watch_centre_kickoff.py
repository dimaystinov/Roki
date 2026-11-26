"""Watch ten centre restarts in a single MuJoCo window (run with mjpython)."""
import asyncio
import json
import time

import mujoco
import mujoco.viewer

from compare_simple_kickoffs import sim, trial


class Display:
    def __init__(self):
        self.window = None
        self.attempt = 0

    def start(self, world):
        self.attempt += 1
        if self.window is None:
            self.model = world.model
            self.data = mujoco.MjData(self.model)
            mujoco.mj_copyData(self.data, self.model, world.data)
            self.window = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False)
            self.window.cam.lookat[:] = [-.75, 0., .15]
            self.window.cam.distance = 3.05
            self.window.cam.azimuth = 90
            self.window.cam.elevation = -25
        self.next_frame = 0.
        self.frame(world, "Centre restart", 0)
        time.sleep(1.)

    def frame(self, world, phase, dt):
        if not self.window.is_running():
            raise SystemExit
        if world.data.time >= self.next_frame:
            with self.window.lock():
                mujoco.mj_copyData(self.data, self.model, world.data)
            self.window.set_texts((None, None, f"Attempt {self.attempt}/10", phase))
            self.window.sync()
            self.next_frame = world.data.time + .032
        time.sleep(dt)


async def main():
    display = Display()
    params = sim.sim_parameters()
    profile = sim.HERE.parents[1] / "sim/.local-state/parameters.json"
    if profile.exists():
        params.update(json.loads(profile.read_text()))
    try:
        for _ in range(10):
            result = await trial(params["game.simple_football.kickoff_side"], 0., 0.,
                                 distance=.30, display=display)
            display.window.set_texts((None, None, f"Attempt {display.attempt}/10", result["result"]))
            display.window.sync()
            print(f"Attempt {display.attempt}: {result['result']}", flush=True)
            time.sleep(3.)
    finally:
        if display.window:
            display.window.close()


if __name__ == "__main__":
    asyncio.run(main())
