"""Validated appearance settings for the Roki2 MuJoCo soccer scene."""
import copy
import json
import math
from pathlib import Path

import numpy as np


DEFAULTS = {
    "imu": {"noise_std_deg": 0.2, "yaw_drift_deg_per_min": 6.0},
    "field": {
        "style": "solid",
        "primary_rgb": [63, 104, 92],
        "secondary_rgb": [44, 145, 53],
        "stripe_count": 8,
    },
    "lighting": {"sun": 1.0, "fill": 1.0, "ambient": 0.30},
    "colors": {
        "yellow_goal_rgb": [242, 204, 26],
        "blue_goal_rgb": [38, 89, 217],
        "ball_rgb": [255, 110, 12],
    },
    "camera": {"fovy_deg": 65.0, "forward_m": 0.04},
    "viewer": {"azimuth_deg": 145.0, "elevation_deg": -32.0,
               "distance_m": 5.5, "lookat_xyz": [0.0, 0.0, 0.3]},
    "opponents": {"enabled": True},
    "training": {"extra_balls": False},
    "person": {"enabled": False, "x_m": 1.7, "y_m": 0.0,
               "walk_amplitude_m": 0.0, "walk_period_s": 6.0},
    "layout": {
        "field_length_m": 4.0,
        "field_width_m": 3.0,
        "yellow_goal_x_m": 1.7,
        "blue_goal_x_m": -1.7,
        "robot_spawn_xy": [0.6, 0.0],
        "robot_yaw_deg": 180.0,
        "ball_spawn_xy": [0.0, 0.0],
    },
}


def validate_settings(changes):
    if not isinstance(changes, dict) or set(changes) - set(DEFAULTS):
        raise ValueError("Scene settings must contain known sections")
    result = copy.deepcopy(DEFAULTS)
    for section, values in changes.items():
        if not isinstance(values, dict) or set(values) - set(result[section]):
            raise ValueError(f"Unknown {section} scene setting")
        result[section].update(values)
    field = result["field"]
    if field["style"] not in ("original", "solid", "stripes"):
        raise ValueError("Field style must be original, solid or stripes")
    if type(field["stripe_count"]) is not int or not 2 <= field["stripe_count"] <= 32:
        raise ValueError("stripe_count must be 2..32")
    for group, values in ((field, ("primary_rgb", "secondary_rgb")),
                          (result["colors"], tuple(result["colors"]))):
        for key in values:
            rgb = group[key]
            if not isinstance(rgb, list) or len(rgb) != 3 or any(
                type(channel) is not int or not 0 <= channel <= 255 for channel in rgb
            ):
                raise ValueError(f"{key} must be an RGB list")
    for key in ("sun", "fill", "ambient"):
        value = result["lighting"][key]
        if type(value) not in (int, float) or not 0 <= value <= 3:
            raise ValueError(f"{key} must be 0..3")
    if not 20 <= result["camera"]["fovy_deg"] <= 120:
        raise ValueError("fovy_deg must be 20..120")
    if not 0 <= result["camera"]["forward_m"] <= .4:
        raise ValueError("forward_m must be 0..0.4")
    viewer = result["viewer"]
    for key in ("azimuth_deg", "elevation_deg"):
        if type(viewer[key]) not in (int, float) or not -360 <= viewer[key] <= 360:
            raise ValueError(f"{key} must be an angle")
    if type(viewer["distance_m"]) not in (int, float) or not 1 <= viewer["distance_m"] <= 20:
        raise ValueError("distance_m must be 1..20")
    if not isinstance(viewer["lookat_xyz"], list) or len(viewer["lookat_xyz"]) != 3 or any(
        type(value) not in (int, float) or not -10 <= value <= 10
        for value in viewer["lookat_xyz"]
    ):
        raise ValueError("lookat_xyz must contain three coordinates")
    if type(result["training"]["extra_balls"]) is not bool:
        raise ValueError("training.extra_balls must be boolean")
    if type(result["opponents"]["enabled"]) is not bool:
        raise ValueError("opponents.enabled must be boolean")
    person = result["person"]
    if type(person["enabled"]) is not bool:
        raise ValueError("person.enabled must be boolean")
    for key in ("x_m", "y_m"):
        if type(person[key]) not in (int, float) or not -6 <= person[key] <= 6:
            raise ValueError(f"person.{key} must be a field coordinate")
    if type(person["walk_amplitude_m"]) not in (int, float) or not 0 <= person["walk_amplitude_m"] <= 2:
        raise ValueError("walk_amplitude_m must be 0..2")
    if type(person["walk_period_s"]) not in (int, float) or not 1 <= person["walk_period_s"] <= 60:
        raise ValueError("walk_period_s must be 1..60")
    layout = result["layout"]
    for key in ("field_length_m", "field_width_m"):
        if type(layout[key]) not in (int, float) or not 1 <= layout[key] <= 12:
            raise ValueError(f"{key} must be 1..12 metres")
    for key in ("yellow_goal_x_m", "blue_goal_x_m", "robot_yaw_deg"):
        if type(layout[key]) not in (int, float) or not -360 <= layout[key] <= 360:
            raise ValueError(f"{key} must be a bounded number")
    for key in ("robot_spawn_xy", "ball_spawn_xy"):
        xy = layout[key]
        if not isinstance(xy, list) or len(xy) != 2 or any(
            type(value) not in (int, float) or not -6 <= value <= 6 for value in xy
        ):
            raise ValueError(f"{key} must contain two metre coordinates")
    imu = result["imu"]
    for key, value in imu.items():
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"Invalid IMU value for {key}")
    if imu["noise_std_deg"] < 0:
        raise ValueError("IMU noise standard deviation must be nonnegative")
    return result


def load_settings(path):
    path = Path(path)
    return validate_settings(json.loads(path.read_text()) if path.exists() else {})


def recolor_field(source, style, primary, secondary, stripe_count):
    output = source.copy()
    if style == "original":
        return output
    # Keep centre circle, lines and spots from the original painted texture.
    paint = (source.min(axis=2) > 205) & ((source.max(axis=2)-source.min(axis=2)) < 35)
    rows, cols = source.shape[:2]
    colour_a = np.asarray(primary, dtype=np.uint8)
    colour_b = np.asarray(secondary, dtype=np.uint8)
    bands = (np.arange(cols)*stripe_count//cols) % 2
    if style == "solid":
        output[:] = colour_a
    else:
        output[:] = np.where(bands[None, :, None] == 0, colour_a, colour_b)
    output[paint] = source[paint]
    return output


def apply_settings(model, settings):
    field = settings["field"]
    texture = model.texture("fieldtex").id
    offset = int(model.tex_adr[texture])
    height, width = int(model.tex_height[texture]), int(model.tex_width[texture])
    length = height*width*3
    pixels = model.tex_data[offset:offset+length].reshape(height, width, 3)
    pixels[:] = recolor_field(pixels, field["style"], field["primary_rgb"],
                              field["secondary_rgb"], field["stripe_count"])
    for name, key in (("goalY", "yellow_goal_rgb"), ("goalB", "blue_goal_rgb"),
                      ("ball", "ball_rgb")):
        model.mat_rgba[model.material(name).id, :3] = (
            np.asarray(settings["colors"][key], dtype=float)/255)
    for name in ("sun", "fill"):
        light = model.light(name).id
        model.light_diffuse[light] *= settings["lighting"][name]
    model.vis.headlight.ambient[:] = settings["lighting"]["ambient"]
    model.cam_fovy[model.camera("sim_head_camera").id] = settings["camera"]["fovy_deg"]
    layout = settings["layout"]
    model.geom_size[model.geom("floor").id, :2] = [
        layout["field_length_m"]/2, layout["field_width_m"]/2]
    model.body_pos[model.body("goal_yellow").id, 0] = layout["yellow_goal_x_m"]
    model.body_pos[model.body("goal_blue").id, 0] = layout["blue_goal_x_m"]
    person = settings["person"]
    model.body_pos[model.body("walking_person").id, :2] = (
        [person["x_m"], person["y_m"]] if person["enabled"] else [100., 100.])

def opponent_material(mesh, material):
    # Mesh 6 contains both torso covers. Other red pieces include electronics.
    return 'Opaque(0,0,255)' if mesh == 'torso_1_mesh_6' and material else material


def soccer_model(scene, settings):
    """Reuse the neutral Roki CAD meshes and contact surfaces for both opponents."""
    import mujoco
    spec = mujoco.MjSpec.from_file(str(scene))
    layout = settings["layout"]
    for name, colour, size, x, y, z in (
        ('yellow_cabinet', [.86,.67,.07,1], [.23,.43,.57], layout['yellow_goal_x_m']+.85, .40,.57),
        ('blue_poster', [.05,.24,.88,1], [.018,.50,.32], layout['blue_goal_x_m']-.85,-.35,.90),
    ):
        spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, rgba=colour,
                                size=size, pos=[x,y,z], contype=0, conaffinity=0)
    if settings.get('training', {}).get('extra_balls'):
        source = spec.geom('ball')
        for index, xy in enumerate(((-.38,.18),(-.68,-.30))):
            name = f'training_ball_{index}'
            body = spec.worldbody.add_body(name=name, pos=[*xy, source.size[0]])
            body.add_freejoint()
            body.add_geom(name=name, type=source.type, size=source.size, mass=source.mass, material=source.material)
            pair = spec.pair('ball_carpet')
            spec.add_pair(geomname1=name, geomname2='floor', condim=pair.condim, friction=pair.friction)
    if not settings["opponents"]["enabled"]:
        return spec.compile()
    neutral = spec.body("torso_1")

    def copy_links(source, fixed):
        for geom in source.geoms:
            fixed.add_geom(type=geom.type, meshname=geom.meshname,
                           material=opponent_material(geom.meshname, geom.material),
                           pos=geom.pos, quat=geom.quat, fromto=geom.fromto, size=geom.size, rgba=geom.rgba,
                           group=geom.group, contype=geom.contype, conaffinity=geom.conaffinity,
                           friction=geom.friction)
        for link in source.bodies:
            child = fixed.add_body(pos=link.pos, quat=link.quat)
            copy_links(link, child)

    layout = settings["layout"]
    blue_x = layout["blue_goal_x_m"]
    facing = 1 if blue_x < layout["yellow_goal_x_m"] else -1
    keeper = spec.worldbody.add_body(name="opponent_keeper",
                                    pos=[blue_x + facing*.10, 0., neutral.pos[2]],
                                    quat=[1.,0.,0.,0.] if facing > 0 else [0.,0.,0.,1.])
    copy_links(neutral, keeper)
    xy, quat = opposing_start(layout["robot_spawn_xy"], math.radians(layout["robot_yaw_deg"]))
    forward = spec.worldbody.add_body(name="opponent_forward", mocap=True,
                                      pos=[*xy, neutral.pos[2]], quat=quat)
    copy_links(neutral, forward)
    return spec.compile()


def opposing_start(xy, yaw):
    """Rotate the starting pose half a turn about the field centre (0, 0)."""
    angle = (yaw + math.pi) / 2
    distance = math.hypot(*xy)
    # The defending starter and its feet are outside the 60 cm circle (§1.3).
    scale = max(1., .40/distance) if distance else 1.
    return (-xy[0]*scale, -xy[1]*scale), (math.cos(angle), 0., 0., math.sin(angle))
