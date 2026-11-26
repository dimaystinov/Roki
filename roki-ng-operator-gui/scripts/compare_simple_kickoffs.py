"""Ten paired, first-goal MuJoCo trials from outside the centre circle."""
import argparse
import asyncio
import json
import math
from collections import Counter
from pathlib import Path
from unittest.mock import patch

import sim_roki2 as sim
from sim_visual_dribble import VisualDribble


class Finished(Exception):
    pass


async def trial(side, lateral, heading, vision_first=False, dribble=False, open_play=False, distance=.60, ball_x=0., display=None):
    settings = sim.load_settings(sim.SETTINGS)
    settings["layout"].update(ball_spawn_xy=[ball_x, 0.], robot_spawn_xy=[ball_x+distance, lateral], robot_yaw_deg=180.+heading)
    p = sim.sim_parameters(settings)
    profile = sim.HERE.parents[1] / "sim/.local-state/parameters.json"
    if profile.exists():
        p.update(json.loads(profile.read_text()))
    p.update({"game.simple_football.kickoff_side": side, "match.own_goal": 0})
    if vision_first:
        p["game.forward.kick_off_ride"] = False
    world = sim.World(settings=settings, parameters=p)
    game = sim.GameAdapter(world, p)
    strategy = (VisualDribble if dribble else sim.SimpleFootball)(game, p, "center", 0)
    if open_play:
        strategy.opening_touch_required = strategy.kickoff_pending = False
    foot_bodies = {world.model.joint(f"{leg}_ankle_roll_joint").bodyid.item(): leg for leg in ("left", "right")}
    foot_contacts = []
    active_kick = None
    if display:
        display.start(world)
    ball_geom = world.model.geom("ball").id
    robot_root = world.model.jnt_bodyid[0]
    original_advance = world.advance
    phases = Counter()
    min_range = math.inf
    touched = False
    touches = []
    last_contact = -math.inf
    max_progress = 0.
    last_x = 0.
    kick_commands = 0
    original_move = game._move
    async def move(op, args, **kwargs):
        nonlocal kick_commands, active_kick
        if op == "motion.kick":
            kick_commands += 1
            active_kick = args.get("leg")
        try:
            return await original_move(op, args, **kwargs)
        finally:
            active_kick = None

    def advance(dt):
        nonlocal min_range, touched, max_progress, last_x, last_contact
        original_advance(dt)
        if display:
            display.frame(world, game.info["phase"], dt)
        x,y,z = map(float,world.data.qpos[world.ball_qpos:world.ball_qpos+3])
        min_range = min(min_range, math.hypot(world.x-x,world.y-y))
        max_progress = max(max_progress, -x)
        for contact in world.data.contact:
            if ball_geom in (contact.geom1,contact.geom2):
                other = contact.geom2 if contact.geom1 == ball_geom else contact.geom1
                if world.model.body_rootid[world.model.geom_bodyid[other]] == robot_root:
                    foot = foot_bodies.get(int(world.model.geom_bodyid[other]))
                    if foot and (not foot_contacts or world.data.time-last_contact >= .15 or foot_contacts[-1][1:] != [foot,active_kick]):
                        foot_contacts.append([round(world.data.time,3), foot, active_kick])
                    touched = True
                    if (world.data.time-last_contact >= .15 and
                        (not touches or math.dist((x,y), touches[-1][1:]) >= .03)):
                        touches.append([round(world.data.time,3), x, y])
                    last_contact = world.data.time
        radius = p["game.ball_diameter_m"]/2
        for colour,direction in (("blue",-1),("yellow",1)):
            line = direction*settings["layout"][colour+"_goal_x_m"]+radius
            if direction*last_x < line <= direction*x and abs(y)+radius < .5 and z+radius < .60:
                if colour == "blue" and not open_play and len(touches) < 2:
                    raise Finished("direct_first_touch")
                raise Finished("goal_"+colour)
        if abs(y)-radius > p["field.geometry"]["width"]/2 or abs(x)-radius > max(
                abs(settings["layout"][c+"_goal_x_m"]) for c in ("yellow","blue")):
            raise Finished("out")
        last_x = x
        if world.data.qpos[2] < .17:
            raise Finished("fall")
        if world.data.time >= 90:
            raise Finished("time")
    async def delay(dt, *_args, **_kwargs):
        if not world.motion_lock.locked():
            for _ in range(max(1,round(dt/.02))):
                advance(.02)
    game._move = move
    world.advance = advance
    # The strategy and frame timestamps use the same clock as the physics.
    with patch("asyncio.sleep",delay), patch("time.monotonic",lambda:100.+world.data.time), patch("time.monotonic_ns",lambda:round((100.+world.data.time)*1e9)):
        try:
            while True:
                await strategy.step()
                phases[game.info["phase"]] += 1
                advance(.02)
        except Finished as exc:
            result = str(exc)
        except sim.Fault as exc:
            result = exc.code
        finally:
            world.renderer.close()
    return dict(ball_start_x_m=ball_x,open_play=open_play,start_distance_m=distance,foot_contacts=foot_contacts,dribble=dribble,vision_first=vision_first,side=side,start_y=lateral,heading_offset_deg=heading,result=result,
                seconds=round(world.data.time,2),contact=touched,touches=touches,kick_commands=kick_commands,
                closest_body_to_ball_m=round(min_range,3),max_blue_progress_m=round(max_progress,3),
                ball_end_xy=world.data.qpos[world.ball_qpos:world.ball_qpos+2].tolist(),
                phase=game.info["phase"],completed_phases=dict(phases))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    parser.add_argument("--vision-first", action="store_true")
    parser.add_argument("--dribble", action="store_true")
    parser.add_argument("--open-play", action="store_true", help="Test the production strategy after kickoff at ten ball placements")
    args = parser.parse_args()
    if args.open_play and (args.vision_first or args.dribble):
        parser.error("--open-play tests the production strategy")
    with args.output.open("x") as out:
        if args.open_play:
            placements = ((.60,0,0), (.60,.05,5), (.60,-.05,-5), (.35,0,0),
                          (.35,.045,0), (.35,-.045,0), (.18,.045,0), (.18,-.045,0),
                          (.13,.045,0), (.13,-.045,0))
            for distance, lateral, heading in placements:
                result = asyncio.run(trial("right", lateral, heading, open_play=True, distance=distance))
                print(json.dumps(result), file=out, flush=True)
            return
        for y,heading in ((0,0),(.05,0),(-.05,0),(0,5),(0,-5)):
            for side in ("left","right"):
                result = asyncio.run(trial(side,y,heading,args.vision_first,args.dribble))
                print(json.dumps(result),file=out,flush=True)
                print(side,y,heading,result["result"],flush=True)


if __name__ == "__main__":
    main()
