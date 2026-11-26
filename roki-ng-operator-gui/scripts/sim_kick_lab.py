"""Measure the camera observation and record the outcome of each kick fixture."""
import hashlib
import json
import math
import time

import cv2
from roki_ng.detection.football import ball_at_foot

from roki_ng.wire import Fault
from sim_kick_grid import target


def validate(body):
    leg, strength = body.get("leg"), body.get("strength")
    if leg not in ("right", "left") or strength not in ("weak", "strong"):
        raise Fault("invalid_argument", "Select foot and strength")
    args = dict(name="kick_test", mode="regular", leg=leg, strength=strength)
    for key in ("ball_forward_mm", "ball_side_mm"):
        value = body.get(key)
        if type(value) is not int or abs(value) > 1500:
            raise Fault("invalid_argument", "Place the ball in integer millimetres within 1500")
        args[key] = value
    return args


def profile_key(robot):
    setup = {"parameters": {k: v for k, v in robot.game.p.items() if k.startswith(("kick.", "walk.", "camera.", "vision.")) or k == "game.ball_diameter_m"},
             "camera": robot.world.settings["camera"], "lighting": robot.world.settings["lighting"]}
    return hashlib.sha256(json.dumps(setup, sort_keys=True).encode()).hexdigest()


def trials(path, profile, args):
    if not path.exists():
        return []
    with path.open() as source:
        rows = (json.loads(line) for line in source)
        return [r for r in rows if r.get("profile") == profile and r["leg"] == args["leg"] and r["strength"] == args["strength"]]


def preview(robot, args):
    image, _, candidates, point = robot.game.observe_ball()
    frame = image.copy()
    result = {"visible": bool(candidates), "placement": [args["ball_forward_mm"], args["ball_side_mm"]]}
    if candidates:
        x, y = candidates[0]["center_px"]
        x1, y1, x2, y2 = candidates[0]["rect"]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 240, 255), 2)
        result.update(u=int(x), v=int(y))
        feet = ball_at_foot(image, candidates[0]["rect"])
        if feet:
            for a, y_tip, b in feet["tips"]:
                cv2.line(frame, (a, y_tip), (b, y_tip), (255, 205, 60), 3)
            result.update(nearest_foot=feet["leg"], aligned=feet["aligned"], gap_ratio=feet["gap_ratio"])
    if point is not None:
        x, y = (round(float(v)*1000) for v in point)
        result.update(x_mm=x, y_mm=y, range_mm=round(math.hypot(x, y)))
    directory = robot.store.path.parent / "kick-lab"
    directory.mkdir(exist_ok=True)
    path = directory / f"camera-{time.time_ns()}.png"
    if not cv2.imwrite(str(path), frame):
        raise Fault("io_error", "Could not save head camera image")
    result["image"] = path.resolve().as_uri()
    recommendation = target(trials(directory/"trials.jsonl", profile_key(robot), args), result["placement"])
    if recommendation:
        result["target"] = recommendation
    return result


def record(robot, args, visual, result):
    path = robot.store.path.parent / "kick-lab" / "trials.jsonl"
    usable = (visual.get("range_mm") is not None and result["status"] == "completed"
              and result["ball_contact"] and result["ball_dx_mm"] > 0
              and result["upright"] and not result["fallen"])
    profile = profile_key(robot)
    row = dict(profile=profile, leg=args["leg"], strength=args["strength"], placement=[args["ball_forward_mm"], args["ball_side_mm"]],
               camera={k:v for k,v in visual.items() if k != "target"}, usable=usable, contact=result["ball_contact"], fallen=result["fallen"],
               upright=result["upright"], ball_support_n=result["ball_support_n"], pitch=result["pitch_deg"], travel_mm=result["ball_distance_mm"],
               forward_mm=result["ball_dx_mm"], lateral_mm=result["ball_dy_mm"])
    with path.open("a") as out:
        out.write(json.dumps(row) + "\n")
    samples = [t["camera"] for t in trials(path, profile, args) if t["usable"]]
    bounds = {key: [min(s[key] for s in samples), max(s[key] for s in samples)]
              for key in ("x_mm", "y_mm", "range_mm") if samples}
    return dict(camera={k:v for k,v in visual.items() if k != "target"}, usable=usable, count=len(samples), bounds=bounds)
