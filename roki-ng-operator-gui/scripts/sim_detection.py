"""FPV diagnostic output from the simulated robot's rendered BGR camera."""
import time

from roki_ng.detection.football import goal_parts, render_scene
from roki_ng.detection.worker import colour_blobs
from roki_ng.parameters import COLOUR_DEFAULTS
from roki_ng.vision.vision_diagnostics import render
from roki_ng.wire import Fault, choice


class SimDetection:
    def __init__(self, game):
        self.game = game
        self.mode = "colour"
        self.profile = "orange_ball"
        self.revision = 0
        self.stop()

    def stop(self):
        self.running = False
        self.frames = 0
        self.result = None
        self.last_frame = self.last_video = 0.

    def invalidate(self):
        self.result = None
        self.last_frame = self.last_video = 0.
        self.revision += 1

    def command(self, op, args, robot):
        if op == "detection.list":
            return dict(detector="colour_blobs", profiles=list(COLOUR_DEFAULTS), max_blobs=4,
                        coordinates="image_pixels", classifies_ball=False,
                        modes=["colour", "ball", "football"], video_source="detection")
        if op == "detection.start":
            if self.running:
                raise Fault("busy", "Detector already running")
            if robot.mode not in ("MANUAL", "GAME"):
                raise Fault("invalid_state", "Select MANUAL or start game")
            if not robot.camera_running:
                raise Fault("not_ready", "Start runtime camera before detection")
            mode = choice(args, "mode", "colour", ("colour", "ball", "football"))
            if mode != "colour" and robot.game_running:
                raise Fault("busy", "Stop the game before starting diagnostics")
            self.profile = choice(args, "profile", "orange_ball", tuple(COLOUR_DEFAULTS))
            self.mode = mode
            self.running = True
        elif op == "detection.select":
            if not self.running or self.mode != "colour":
                raise Fault("invalid_state", "Start colour diagnostics before selecting a profile")
            self.profile = choice(args, "profile", None, tuple(COLOUR_DEFAULTS))
            self.invalidate()
        elif op == "detection.stop":
            self.stop()
        elif op != "detection.status":
            raise Fault("not_supported", op)
        now = time.monotonic()
        requested = "detection" in robot.outputs
        return dict(state="ready", running=self.running, mode=self.mode, profile=self.profile,
                    frames=self.frames, error=None, result=self.result,
                    age_ms=round((now-self.last_frame)*1000) if self.result else None,
                    video=dict(requested=requested, publishing=requested and now-self.last_video < 2,
                               error=None))

    def process(self, image, requested):
        now = time.monotonic()
        if not self.running or now-self.last_frame < .2:
            return None
        world = self.game.world
        result = dict(frame_sequence=world.sequence, size=[image.shape[1], image.shape[0]],
                      parameter_revision=self.revision, sensor_timestamp_ns=time.monotonic_ns())
        output = None
        if self.mode == "colour":
            debug = {}
            blobs, count = colour_blobs(image, self.game.p, self.profile, debug=debug)
            result.update(profile=self.profile, blobs=blobs, total_blobs=count,
                          area_rejected=debug["area_rejected"])
            if requested:
                output = render(image, {self.profile: debug["mask"]}, blobs,
                                f"{self.profile} frame={world.sequence} blobs={count}")
        else:
            debug = {} if requested and self.mode == "ball" else None
            _, geometry, _, point = self.game.observe_ball(image, debug=debug)
            result.update(valid=point is not None, reason="ok" if point is not None else "no_ball")
            if point is not None:
                result.update(x_m=float(point[0]), y_m=float(point[1]))
            if requested and self.mode == "football":
                parts = goal_parts(image, self.game.p, world.projector, None, geometry)
                output = render_scene(image, dict(parts=parts, ball=point, height_m=geometry["height"]),
                                      "Football diagnostics")
            elif requested:
                output = render(image, debug["masks"], debug["objects"], result["reason"])
        result["processing_us"] = round((time.monotonic()-now)*1e6)
        self.result = result
        self.frames += 1
        self.last_frame = now
        if output is not None:
            self.last_video = now
        return output
