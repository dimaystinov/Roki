"""Camera-guided football with IMU heading and relative odometry during visual outages."""
import asyncio
import math
import time
import json
from dataclasses import dataclass

from roki_ng.supervisor.game.expected_gaze import ExpectedGaze
from roki_ng.body.motion.odometry import GameOdometry, jump_delta
from roki_ng.body.calibration import wrap
from roki_ng.detection.football import assemble_goals
from roki_ng.detection.football import corner_lane, free_goal_windows, shot_clear
from roki_ng.detection.football import own_goal_blocks
from roki_ng.wire import Fault
from roki_ng.body.body_imu import quaternion_yaw
from roki_ng.body.recovery import body_position
from roki_ng.supervisor.game.ball_kick_test import kick_alignment


@dataclass(frozen=True)
class BallRegion:
    point: tuple
    stamp: float
    velocity: tuple
    pose_revision: int
    path: float
    source: str


class SimpleFootball:
    def __init__(self, game, parameters, entry, delay, *, danger=False):
        self.danger = danger
        self.last_usable_frame = None
        self.blind_heading_checked = False
        self.opening_touch_required = not danger and entry == 'center'
        self.start_heading_pending = True
        self.attack_attempts = 0
        self.goal_memory = None
        self.own_veto_reported = False
        self.remembered_shot = False
        self.danger_approaches = 0
        self.kick_direction = 'straight'
        self.angle_target = None
        self.angle_attempts = 0
        self.fine_heading_attempts = 0
        self.goal_scan_needed = True
        self.last_goal_scan = -math.inf
        self.approach_glance_after = None
        self.landing_pan = None
        self.g, self.p, self.entry = game, parameters | {'game.use_visual_localisation': False}, entry
        self.entry_pending = entry in ('left', 'right')
        self.kickoff_pending = not danger and entry == 'center' and not delay and parameters['game.forward.kick_off_ride']
        self.attack_heading = None
        self.keeper_glance_at = -math.inf
        self.final_keeper_check = False
        self.rebound_turn_stamp = None
        self.search_track = None
        self.near_track = None
        self.predict_after_jump = False
        self.keeper_aim = None
        self.attack_seen_at = -math.inf
        self.check_shot_view = False
        self.trimmed_shot = False
        self.field_ball = None
        self.last_open_shot = -math.inf
        self.detour_side = 0
        self.attack_axis = None
        self.pass_check_after = 0.
        self.pass_search_index = 0
        self.central_lane_checked = False
        self.placed_kickoff_started = False
        self.pass_segments_left = 0
        self.opening_jump_pending = False
        self.opening_right_jumps_left = 0
        self.pending_toe_pass = None
        self.toe_camera_revision = None
        self.kick_check = None
        self.placed_ball_reference = None
        self.placed_ball_pass = False
        self.deadline = self.kickoff_heading = None
        self.pose_id = 0
        self.scan = 0
        self.sectors_left = 6
        self.goal_views = []
        self.goal_hint = None
        self.inspect_feet_first = True
        self.waiting_ball = self.waiting_sample = self.restart_roll_side = None
        self.restart_pivot = False
        self.turn_ball = None
        self.approach_crouched = False
        self.kickoff_heading_done = False
        self.opening_walk_sent = False
        self.soft_touch_before = None
        self.soft_touch_attempts = 0
        self.expected_gaze = ExpectedGaze()
        self.expected_post_side = None
        self.near = False
        self.ball_range = math.inf
        self.fine_radius = math.hypot(parameters["game.kick_distance_mm"]/1000+.04,
                                      parameters["game.kick_offset_mm"]/1000+.025)
        self.far_frames = 0
        self.jumps = 0
        self.leg_sign = None
        self.reset_leg_switch()
        self.orbit_sign = 1
        self.orbit_face_ball = False
        self.last_ball_side = 1
        self.empty_sectors = 0
        self.field_sector = 0
        self.field_sector_heading = None
        self.last_score = None
        self.score_kind = None
        self.last_turn_error = None
        self.stagnant = 0
        self.last_scene = None
        self.shot_deadline = None
        self.ball_refresh_after = None
        self.aligned_radius = None
        self.displaced_frame = None
        self.note = 'starting'
        self.specs = [parameters[f'field.goal.{i}'] for i in (0, 1)]
        colours = [spec['colour'] for spec in self.specs]
        if not danger and (any(c not in ('blue', 'yellow', 'white') for c in colours)
                           or colours[0] == colours[1] != 'white'):
            raise Fault('configuration', 'Unsupported goal colours')
        # Keep team identity separate when both physical frames are white.
        self.specs = [dict(spec, colour=f'white_{i}') if spec['colour'] == 'white' else spec
                      for i, spec in enumerate(self.specs)]
        self.own = self.specs[parameters['match.own_goal']]['colour']
        self.enemy = self.specs[1-parameters['match.own_goal']]['colour']

    def associate_posts(self, parts):
        odom = getattr(self.g, 'odometry', None)
        result = []
        for part in parts:
            white = part['colour'] == 'white'
            if not white:
                result.append(part)
            if part['kind'] not in ('opening', 'post') or (not white and part['kind'] != 'post'):
                continue
            if not odom or odom.confidence == 'degraded':
                continue
            points = part['points']
            matches = []
            for spec in self.specs:
                if not white and spec['colour'] != part['colour']:
                    continue
                ends = [odom.body_point((spec['x'], spec['y']+sign*spec['width']/2))
                        for sign in (-1, 1)]
                centre = odom.body_point((spec['x'], spec['y']))
                bearing = math.atan2(centre[1], centre[0])
                ends.sort(key=lambda e: wrap(math.atan2(e[1], e[0])-bearing), reverse=True)
                for side in (0, 1):
                    error = max(math.dist(pt, ends[(side+k)%2]) for k,pt in enumerate(points))
                    matches.append((error, spec['colour'], side, ends, centre))
            matches.sort(key=lambda item: item[0])
            if not matches or matches[0][0] > min(.38, .14+odom.position_error_m()):
                continue
            if len(matches)>1 and matches[1][0]-matches[0][0] < .12:
                continue
            _, colour, side, ends, centre = matches[0]
            if len(points)==2:
                result.append(dict(part, colour=colour))
            else:
                observed = points[0]
                result.append(dict(part, kind='mapped_post', colour=colour,
                                   side=('left','right')[side],
                                   goal_center=[v+p-e for v,p,e in zip(centre,observed,ends[side])]))
        return result

    def cancel_special(self, reason):
        self.search_track = None
        self.near_track = None
        self.predict_after_jump = False
        self.keeper_aim = None
        self.keeper_glance_at = -math.inf
        self.final_keeper_check = False
        self.empty_sectors = self.field_sector = 0
        self.field_sector_heading = None
        if reason == 'fall' and self.opening_touch_required:
            # The placement is lost. Reacquire visually with the weak-touch
            # restriction still active, even if an opening motion was sent.
            self.placed_kickoff_started = False
        self.field_ball = None
        self.check_shot_view = self.trimmed_shot = False
        self.start_heading_pending = False
        self.pass_segments_left = 0
        self.opening_jump_pending = False
        self.opening_right_jumps_left = 0
        self.pending_toe_pass = None
        self.toe_camera_revision = None
        self.kick_check = None
        self.placed_ball_reference = None
        self.placed_ball_pass = False
        self.inspect_feet_first = True
        self.waiting_ball = self.waiting_sample = self.restart_roll_side = None
        self.restart_pivot = False
        self.turn_ball = None
        self.approach_crouched = False
        self.kickoff_heading_done = False
        self.opening_walk_sent = False
        self.soft_touch_before = None
        self.soft_touch_attempts = 0
        self.expected_gaze = ExpectedGaze()
        self.expected_post_side = None
        self.goal_views.clear()
        self.goal_hint = None
        self.scan, self.sectors_left = 0, 6
        self.orbit_face_ball = False
        self.reset_leg_switch()
        self.goal_scan_needed = True
        self.last_goal_scan = -math.inf
        self.approach_glance_after = None
        self.landing_pan = None
        self.fine_heading_attempts = 0
        if self.kick_direction == 'kickoff_left':
            self.kick_direction = 'straight'
            self.g.info['kick_direction'] = 'straight'
        self.angle_target = None
        self.angle_attempts = 0
        self.danger_approaches = 0
        self.forget_goal(reason)
        if self.kickoff_pending or self.deadline is not None:
            self.g.s.log('game', 'INFO', f'Simple football kickoff cancelled: {reason}')
        self.kickoff_pending = False
        self.deadline = self.kickoff_heading = None
        self.entry_pending = False
        self.near = False
        self.jumps = self.stagnant = 0
        self.leg_sign = self.last_scene = self.last_score = None
        self.last_turn_error = self.score_kind = None
        self.shot_deadline = None
        self.aligned_radius = None
        self.displaced_frame = None

    def forget_goal(self, reason):
        if self.goal_memory is not None:
            self.g.s.log('game', 'INFO', f'Simple goal memory cancelled: {reason}')
        self.goal_memory = None

    def check_goal_memory(self, imu=None):
        """Check every published IMU sample, including while awaiting body/head jobs."""
        memory = self.goal_memory
        if memory is None:
            return False
        imu = imu if imu is not None else self.g.s.body_telemetry.get('body.imu', {})
        stamp = imu.get('source_mono_ns')
        now = time.monotonic()
        reason = None
        if now >= memory['until']:
            reason = '5 second expiry'
        elif (not imu.get('valid') or type(stamp) is not int
              or not 0 <= (time.monotonic_ns()-stamp)/1e9 < .25
              or stamp < memory['stamp'] or stamp-memory['stamp'] > 250_000_000):
            reason = 'IMU stale, missing or reset'
        else:
            try:
                if body_position(imu['quaternion_xyzw']) != 'upright':
                    reason = 'body not upright'
                elif abs(wrap(quaternion_yaw(imu['quaternion_xyzw'])-memory['yaw'])) > math.radians(10):
                    reason = 'heading departed by more than 10 degrees'
            except (KeyError, ValueError, TypeError, Fault):
                reason = 'invalid body orientation'
        if reason:
            self.forget_goal(reason)
            return False
        memory['stamp'] = stamp
        return True

    def expired(self):
        return self.deadline is not None and time.monotonic() >= self.deadline

    def describe(self, phase, reason):
        left = '' if self.deadline is None else f' kickoff={max(0, self.deadline-time.monotonic()):.1f}s'
        self.g.info.pop('selected_motion', None)
        self.note = f'{phase} jump={self.jumps}/15{left} {reason}'
        self.g.info.update(phase=phase, reason=reason, decision=phase)
        self.g.s.log('game', 'INFO', ('Danger football: ' if self.danger else 'Simple football: ')+self.note)

    def log_search_found(self, pan, tilt, scene):
        x, y = scene['ball']
        self.g.s.log('game', 'INFO', f'Ball head search: FOUND pan={pan} tilt={tilt} x={x*1000:.0f} y={y*1000:.0f}mm')

    def reset_leg_switch(self):
        self.leg_switched = False
        self.leg_candidate = None
        self.leg_sample = None

    def consider_leg_switch(self, y_mm, sample):
        """One change per attempt, two distinct agreeing frames, >=60mm benefit."""
        if self.leg_switched or self.leg_sign is None or sample is None:
            return
        if sample == self.leg_sample:
            return
        self.leg_sample = sample
        sign = -1 if y_mm < 0 else 1
        offset = self.p['game.kick_offset_mm']
        benefit = abs(y_mm-self.leg_sign*offset)-abs(y_mm-sign*offset)
        if sign == self.leg_sign or abs(y_mm) < 30 or benefit < 60:
            self.leg_candidate = None
            return
        if self.leg_candidate is not None and sign == self.leg_candidate[0] and abs(y_mm-self.leg_candidate[1]) <= 80:
            old = self.leg_sign
            self.leg_sign, self.leg_switched = sign, True
            self.leg_candidate = None
            self.g.s.log('game', 'INFO', f"Fine leg switched once: {'left' if old > 0 else 'right'} -> {'left' if sign > 0 else 'right'}; y={y_mm:.0f}mm benefit={benefit:.0f}mm jumps={self.jumps}/15 frame={sample}")
        else:
            self.leg_candidate = (sign, y_mm)

    async def strike(self, args, *, default_jump=False):
        stage = 'opening' if self.opening_touch_required else 'play'
        method = self.p.get(f'game.simple_football.{stage}_strike', 'strategy')
        jump = default_jump if method == 'strategy' else method == 'jump'
        self.describe('strike_jump' if jump else 'strike_foot',
                      'forward jump to strike' if jump else f"kick with {args['leg']} foot")
        odom = getattr(self.g, 'odometry', None)
        ball = (self.last_scene or {}).get('ball')
        estimated = None
        if ball and odom:
            # Intermediate search point on the attempted strike axis. A new
            # camera measurement supersedes it even if the ball never moved.
            advance = .35 if args.get('soft') or jump else .65
            estimated = odom.field_point((ball[0]+advance, ball[1])), odom.pose_revision
        if jump:
            if self.remembered_shot and not self.check_goal_memory():
                return False
            completed = await self.jump('forward', 1.)
            self.forget_goal('strike attempted')
        else:
            completed = await self.move('motion.kick', args)
        if completed and estimated and estimated[1] == odom.pose_revision:
            self.search_track = BallRegion(estimated[0], time.monotonic(), (0., 0.), estimated[1], odom.path, 'kick_direction')
            self.near_track = None
        return completed

    async def move(self, op, args):
        revision = self.g.revision
        detail = args.get('direction') or args.get('leg') or args.get('name') or args.get('kind', '')
        self.g.info['selected_motion'] = ' '.join(filter(None, (op, detail)))
        if op == "motion.kick" and self.opening_touch_required and not args.get("soft"):
            return False
        if self.expired():
            self.cancel_special('30 second deadline')
            return False
        self.last_scene = None
        await self.g._call('detection', 'football.pose', {'active': False, 'note': self.note})
        if not self.g._same_plan(revision):
            self.cancel_special('plan changed before motion')
            return False
        if self.approach_crouched and op != 'game.walk':
            if not await self.g._move('motion.pose', {'name': 'base_stand'}):
                self.cancel_special('approach stance interrupted')
                return False
            self.approach_crouched = False
        if op == 'motion.kick' and self.remembered_shot and not self.check_goal_memory():
            return False
        if hasattr(self, 'motion_fresh') and not self.motion_fresh(op, args):
            return False
        if op in ('game.walk', 'game.jump', 'motion.kick'):
            self.expected_gaze.forget_velocity()
        self.goal_views.clear()  # Body-relative endpoints cannot survive a body motion.
        self.sectors_left = 6
        result = await self.g._move(op, args, monitor_ball=op == 'game.walk' and args.get('kind') != 'kickoff' and self.g.info.get('phase') not in ('simple_entry', 'simple_predicted_approach', 'simple_search_walk'))
        if not result:
            self.cancel_special('motion interrupted')
        elif op == 'game.walk':
            self.approach_crouched = args.get('keep_crouch', False)
        if result and not self.opening_touch_required and op in ('game.jump', 'game.walk', 'motion.pose', 'motion.kick'):
            self.ball_refresh_after = time.monotonic()
        if op == 'motion.kick':
            self.forget_goal('strike attempted')
            self.g.s.log('game', 'INFO' if result else 'WARNING',
                         f'Simple kick: {"completed" if result else "not completed"}; {self.g.info.get("reason", "")}')
        return result

    async def walk(self, distance, course=0.):
        self.forget_goal('walking; require a new opening observation')
        revision = self.g.revision
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return False
        return await self.move('game.walk', dict(kind='omni', distance_m=distance,
                                                course=course, heading=0., yaw_offset=wrap(-yaw),
                                                keep_crouch=self.opening_touch_required))

    def remember_near_ball(self, scene):
        odom = getattr(self.g, 'odometry', None)
        if not odom or not scene.get('confirmed') or not scene.get('ball'):
            return
        ball = scene['ball']
        point = odom.field_point(ball)
        stamp = scene['received']
        velocity = (0., 0.)
        previous = self.search_track
        if previous and previous.source == 'camera' and previous.pose_revision == odom.pose_revision:
            dt = stamp - previous.stamp
            if dt <= 0 and not self.opening_touch_required and scene.get('roll_m_s'):
                return
            speed_limit = .9 if self.opening_touch_required else 2.5
            if 0 < dt < 1.5 and math.dist(point, previous.point) < speed_limit * dt:
                velocity = tuple((x-y)/dt for x,y in zip(point, previous.point))
        roll = scene.get('roll_m_s')
        # Jitter in a single projection pair can resemble a fast roll. Use
        # the within-pose derivative when an independent displacement agrees.
        if not self.opening_touch_required and roll and math.hypot(*velocity) > .12:
            c, sn = math.cos(odom.yaw), math.sin(odom.yaw)
            vx, vy = roll
            dx, dy = c*vx-sn*vy, sn*vx+c*vy
            if dx*velocity[0]+dy*velocity[1] > .7*math.hypot(dx, dy)*math.hypot(*velocity):
                velocity = ((dx+velocity[0])/2, (dy+velocity[1])/2)
        self.search_track = BallRegion(point, stamp, velocity, odom.pose_revision, odom.path, 'camera')
        if math.hypot(*ball) > .55:
            self.near_track = None
            return
        velocity = (0., 0.)
        if self.near_track and self.near_track[3] == odom.pose_revision:
            previous, old_stamp, old_velocity, _ = self.near_track
            dt = stamp-old_stamp
            if dt <= 0:
                return
            if dt < 1 and math.dist(previous, point) < .30*dt:
                velocity = tuple(.5*v+.5*(x-y)/dt for v,x,y in zip(old_velocity, point, previous))
        self.near_track = point, stamp, velocity, odom.pose_revision

    async def approach_ball_region(self):
        """Use one short displacement per independent estimate, then observe again."""
        track = self.search_track
        odom = getattr(self.g, 'odometry', None)
        if not track or not odom:
            return False
        self.search_track = None
        age = time.monotonic()-track.stamp
        local_only = odom.confidence == 'degraded'
        rolling = (not self.opening_touch_required and
                   (track.source == 'kick_direction' or math.hypot(*track.velocity) > .12))
        local_age = 5 if rolling else 3
        if track.pose_revision != odom.pose_revision or not 0 <= age < (local_age if local_only else 8):
            return False
        if local_only and not 0 <= odom.path-track.path < .15:
            return False
        geometry = self.p['field.geometry']
        target = []
        for x, v, extent in zip(track.point, track.velocity, (geometry['length'], geometry['width'])):
            margin = max(0., extent/2 - .15)
            # Lead the rolling ball by the time to complete a short approach.
            lead = min(.75, age + .30) if rolling else min(.7, age)
            predicted = x + v * lead
            target.append(predicted if local_only else max(-margin, min(margin, predicted)))
        x, y = odom.body_point(target)
        distance = math.hypot(x, y)
        if distance < .16:
            return False
        bearing = math.atan2(y, x)
        # Search allowance, not a measured statistical confidence interval.
        radius = (.06 if track.source == 'camera' else .25) + .12*age + .55*max(0., odom.path-track.path)
        self.describe('simple_predicted_approach',
                      f'last ball area x={x:.2f} y={y:+.2f}m age={age:.1f}s '
                      f'radius={radius:.2f}m source={track.source}')
        if abs(bearing) > math.radians(55):
            if await self.turn(bearing) and rolling and self.rebound_turn_stamp != track.stamp:
                # A turn changes heading, but it has not reached the search point.
                self.rebound_turn_stamp = track.stamp
                self.search_track = track
        else:
            travel = min(.12 if local_only else .25, max(.05, distance-max(.12, radius/2)))
            scene = self.last_scene or {}
            for part in scene.get('parts', []):
                if part.get('kind') == 'obstacle':
                    ox, oy = part['points'][0]
                    c, sn = math.cos(bearing), math.sin(bearing)
                    if 0 <= ox*c+oy*sn <= travel+.13 and abs(oy*c-ox*sn) < part['radius']+.13:
                        return False
            return await self.walk(travel, bearing)
        return True

    async def follow_hidden_ball(self, scene):
        if not self.predict_after_jump or not self.near_track:
            return False
        self.predict_after_jump = False
        point, stamp, velocity, revision = self.near_track
        age = time.monotonic()-stamp
        odom = self.g.odometry
        if not 0 <= age <= 2 or revision != odom.pose_revision or not scene or not scene.get('confirmed'):
            return False
        if not 0 <= time.monotonic()-scene.get('received', 0) <= .5:
            return False
        x, y = odom.body_point(tuple(a+min(age,.5)*v for a,v in zip(point, velocity)))
        if x <= 0 or math.hypot(x,y) > .55:
            return False
        lateral = y-math.copysign(self.p['game.kick_offset_mm']/1000, y)
        direction = ('left' if lateral > 0 else 'right') if abs(lateral) > .055 else 'forward'
        if (direction == 'forward' and x < .13) or self.jump_corridor_blocked(scene, direction, .5):
            return False
        self.describe('simple_ball_prediction', f'ball hidden; expected x={x:.2f} y={y:+.2f}m; half jump {direction}')
        self.last_scene = scene
        await self.jump(direction, .5)
        return True

    async def jump(self, direction, fraction=1., *, alignment=False):
        scene = self.last_scene or {}
        if alignment and direction in ('forward', 'backward', 'left', 'right') and 0 <= time.monotonic()-scene.get('received', 0) <= .5:
            intended = direction
            if self.jump_corridor_blocked(scene, direction, fraction):
                _, oy = min(scene['frame_obstacles'], key=lambda item: math.hypot(*item[0]))[0]
                alternatives = ('right', 'left') if oy >= 0 else ('left', 'right')
                direction = next((side for side in alternatives if not self.jump_corridor_blocked(scene, side, fraction)), None)
                if direction is None:
                    self.describe('simple_pass_blocked', 'both lateral landing corridors blocked; reobserve')
                    return False
                self.describe('simple_jump_detour', f'{intended} blocked; jump {direction}, then reobserve')
        if self.goal_memory is not None:
            if direction in ('left', 'right'):
                self.goal_memory['side_mm'] += abs(self.p[f'motion.jump_{direction}_mm'])*max(.1, min(1., fraction))
                if self.goal_memory['side_mm'] > 60:
                    self.forget_goal('lateral command budget exceeded 60mm')
            elif direction != 'forward':
                self.forget_goal('turn or backward reposition')
        if alignment:
            self.jumps += 1
        completed = await self.move('game.jump', dict(direction=direction, fraction=max(.1, min(1., fraction))))
        self.predict_after_jump = bool(completed and alignment and self.near_track and not direction.startswith('turn_'))
        return completed

    async def turn(self, angle, *, alignment=False, tolerance=15):
        if abs(angle) <= math.radians(tolerance):
            return True
        key = 'motion.jump_yaw_ccw' if angle > 0 else 'motion.jump_yaw_cw'
        return await self.jump('turn_left' if angle > 0 else 'turn_right', abs(angle)/abs(self.p[key]), alignment=alignment)

    async def search_field(self, odom):
        """Visit the centre circle first, then inspect both halves from inside."""
        if not odom:
            await self.search_sector()
            return
        geometry = self.p['field.geometry']
        length, width = geometry['length'], geometry['width']
        points = ((0., 0.), (.24*length, .23*width), (-.24*length, .23*width),
                  (-.24*length, -.23*width), (.24*length, -.23*width))
        index = self.field_sector // 6 % len(points)
        x, y = odom.body_point(points[index])
        distance = math.hypot(x, y)
        travelling = self.field_sector_heading is None and distance > .24
        if travelling:
            bearing = math.atan2(y, x)
        else:
            if self.field_sector_heading is None:
                self.field_sector_heading = odom.yaw
            bearing = wrap(self.field_sector_heading + math.radians(60)*(1+self.field_sector%6) - odom.yaw)
        # Point the lens into the intended sector while the body catches up.
        # A head view counts only when that sector is inside the camera's range.
        sign = self.g.s.params.backend.Robot.camera.pan_sign
        pan = max(-2200, min(2200, round(bearing/(sign*.00058909))))
        tilt = self.p['game.head_tilt'] if travelling else min(0, self.p['head.field_tilt']+500)
        revision = self.g.revision
        self.describe('simple_search', f'viewpoint {index}; looking into sector {1+self.field_sector%6}/6')
        scene = await self.look(pan, tilt)
        if not self.g._same_plan(revision):
            return
        if scene and scene.get('ball'):
            self.empty_sectors = self.field_sector = 0
            self.field_sector_heading = None
            self.last_scene = scene
            self.ball_refresh_after = None
            self.log_search_found(pan, tilt, scene)
            await self.attack_after_pass(scene['ball'], assemble_goals(scene.get('parts', []), self.specs))
            return
        if not scene or not scene.get('confirmed'):
            return
        if travelling:
            self.describe('simple_search_walk', f'search viewpoint {index}; remaining {distance:.2f}m')
            if abs(bearing) > math.radians(55):
                await self.turn(bearing)
                return
            c, sn = math.cos(bearing), math.sin(bearing)
            for part in scene.get('parts', []):
                if part.get('kind') == 'obstacle':
                    ox, oy = part['points'][0]
                    if 0 < ox*c+oy*sn < .44 and abs(oy*c-ox*sn) < part['radius']+.13:
                        await self.turn(bearing+math.copysign(math.radians(35), -(oy*c-ox*sn)))
                        return
            travel = .16 if odom.confidence == 'degraded' else .32
            await self.walk(min(travel, distance-.10), bearing)
            return
        if abs(bearing) <= math.radians(75):
            self.field_sector += 1
            if self.field_sector % 6 == 0:
                self.field_sector_heading = None
        if abs(bearing) > math.radians(15):
            await self.turn(bearing)

    async def search_sector(self):
        revision = self.g.revision
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        target = wrap(yaw+math.pi/3)
        for _ in range(3):
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
            error = wrap(target-yaw)
            if abs(error) <= math.radians(15) or (self.near and self.jumps >= 15):
                return
            if not await self.turn(error, alignment=self.near):
                return

    async def look(self, pan, tilt, *, wait_ball=False, single_frame=False):
        if self.expired() or not self.g._allowed() or getattr(self.g, 'vision_retry_at', 0):
            return None
        revision = self.g.revision
        await self.g._call('detection', 'football.pose', {'active': False, 'note': self.note})
        if not self.g._same_plan(revision) or getattr(self.g, 'vision_retry_at', 0):
            return None
        toe_view = self.pending_toe_pass is not None and pan == 0 and tilt == -2100
        if not toe_view or self.toe_camera_revision != revision:
            self.toe_camera_revision = None
            if not await self.g._move('game.head', {'pan': pan, 'tilt': tilt}):
                return None
            if not await self.g._settle(.10):
                return None
            if toe_view:
                self.toe_camera_revision = revision
        if not self.g._same_plan(revision):
            return None
        self.pose_id += 1
        pose_id = self.pose_id
        await self.g._call('detection', 'football.pose', dict(active=True, pose_id=pose_id,
                         pan=pan, tilt=tilt, after=time.monotonic(), note=self.note,
                         height_m=getattr(self, 'observation_height', None),
                         kick_feedback=(tilt == -2100 and (self.pending_toe_pass is not None or self.kick_check is not None)),
                         goal_bearing_only=getattr(self, 'goal_bearing_only', False),
                         goal_veto_enabled=not self.danger and not getattr(self, 'goal_bearing_only', False)))
        last_confirmed = None
        near_inspection = (not self.danger and pan == 0 and tilt <= -1700
                           and (self.pending_toe_pass is not None or self.kick_check is not None
                                or self.near or self.ball_range <= .35))
        wait_seconds = 1.05 if wait_ball else .65
        if near_inspection:
            # One fresh exposure at kickoff; two in open play if the ball tracker
            # has to rebuild its history after a head motion at 5 FPS.
            wait_seconds = .24 if self.pending_toe_pass is not None else .44
        until = time.monotonic() + wait_seconds
        while self.g._same_plan(revision) and time.monotonic() < until and not self.expired():
            scene = await self.g._call('detection', 'football.status')
            if not self.g._same_plan(revision) or getattr(self.g, 'vision_retry_at', 0):
                return None
            received = scene.get('received')
            if (scene.get('valid') and scene.get('pose_id') == pose_id
                    and isinstance(received, (int, float)) and 0 <= time.monotonic()-received <= .5):
                self.last_usable_frame = time.monotonic()
                self.blind_heading_checked = False
                scene = dict(scene, ball=scene.get('ball'), parts=self.associate_posts(scene.get('parts') or []))
                if ((single_frame and (not near_inspection or scene.get('confirmed')))
                        or (near_inspection and scene.get('ball') and not scene.get('confirmed'))):
                    if scene.get('confirmed'):
                        self.remember_gaze(scene)
                        self.remember_near_ball(scene)
                    return scene
                # Tracker marks the first frame unconfirmed; wait for the second.
                if scene.get('confirmed'):
                    if last_confirmed is None or scene.get('received') != last_confirmed.get('received'):
                        self.remember_gaze(scene)
                        self.remember_near_ball(scene)
                    last_confirmed = scene
                if scene.get('confirmed') and (not wait_ball or scene.get('ball')):
                    return scene
            await asyncio.sleep(.025)
        if (last_confirmed and self.g._same_plan(revision) and not self.expired()
                and 0 <= time.monotonic()-last_confirmed['received'] <= .5):
            return last_confirmed
        return None

    def gaze_imu(self):
        imu = getattr(self.g.s, 'body_telemetry', {}).get('body.imu', {})
        stamp = imu.get('source_mono_ns')
        if (not imu.get('valid') or type(stamp) is not int
                or not 0 <= (time.monotonic_ns()-stamp)/1e9 < .25):
            return None
        try:
            if body_position(imu['quaternion_xyzw']) != 'upright':
                return None
            return quaternion_yaw(imu['quaternion_xyzw']), imu.get('yaw_epoch')
        except (KeyError, ValueError, TypeError, Fault):
            return None

    def remember_start_heading(self):
        if not self.start_heading_pending:
            return
        imu = self.gaze_imu()
        if imu is None:
            return
        yaw, epoch = imu
        pose = GameOdometry(self.p, self.entry, yaw)
        goal = self.specs[1-self.p['match.own_goal']]
        relative = wrap(math.atan2(goal['y']-pose.y, goal['x']-pose.x)-pose.yaw)
        self.expected_gaze.remember('start_goal', (math.cos(relative), math.sin(relative)),
                                    yaw=yaw, epoch=epoch, now=time.monotonic())
        self.start_heading_pending = False
        self.g.s.log('game', 'INFO', f'Start goal bearing: {self.enemy} '
                     f'imu={math.degrees(wrap(yaw+relative)):.1f}deg entry={self.entry}')

    def remembered_goal_heading(self):
        imu = self.gaze_imu()
        if imu is not None:
            for name in ('enemy_goal', 'start_goal'):
                target = self.expected_gaze.targets.get(name)
                if target is not None and target[1] == imu[1]:
                    if name == 'start_goal' or 0 <= time.monotonic()-target[2] < 15:
                        return target[0]
        return None

    def opening_pass_course(self, yaw, diagonal_deg=15):
        heading = self.remembered_goal_heading()
        if heading is None:
            heading = self.kickoff_heading if self.kickoff_heading is not None else yaw
        sign = 1 if self.p['game.simple_football.kickoff_side'] == 'left' else -1
        return wrap(heading + sign*math.radians(diagonal_deg) - yaw)

    def expected_pan(self, name):
        imu = self.gaze_imu()
        if imu is None:
            return None
        geometry = self.g.s.params.backend.Robot.camera
        odom = getattr(self.g, 'odometry', None)
        if name == 'ball':
            self.ball_pan_spread = 0
        if name == 'ball' and odom and self.field_ball:
            point, stamp, path, epoch, revision = self.field_ball
            travel = odom.path - path
            if odom.confidence == 'degraded' or revision != odom.pose_revision:
                # A map correction invalidates field coordinates. The last camera
                # bearing remains useful if we barely moved in the same IMU epoch.
                if 0 <= travel <= .08 and epoch == imu[1] and 0 <= time.monotonic()-stamp <= 1.5:
                    self.ball_pan_spread = 900
                    return self.expected_gaze.pan('ball', yaw=imu[0], epoch=imu[1],
                                                  now=time.monotonic(), sign=geometry.pan_sign)
                return None
            if imu[1] == epoch and 0 <= time.monotonic()-stamp < 4 and .025 < travel < .55:
                x, y = odom.body_point(point)
                ticks = round(math.atan2(y,x)/(geometry.pan_sign*.00058909))
                # Absolute position bias cancels between two observations in the same map frame.
                relative_error = .025 + .55 * travel
                distance = math.hypot(x, y)
                if abs(ticks) <= 2666 and distance > 0:
                    self.ball_pan_spread = round(math.asin(min(1., relative_error / distance)) / .00058909)
                    return ticks
            if travel > .025:
                # The old bearing can still seed a broad camera sweep.
                self.ball_pan_spread = 1778
        if name == 'enemy_goal' and name not in self.expected_gaze.targets:
            name = 'start_goal'
        return self.expected_gaze.pan(name, yaw=imu[0], epoch=imu[1],
                                      now=time.monotonic(), sign=geometry.pan_sign)

    def remember_field_direction(self, scene, imu, stamp):
        if not scene.get('confirmed'):
            return
        parts = scene['parts']
        if any(p['colour']==self.own and p['kind']=='opening' for p in parts):
            return
        reference = self.expected_gaze.targets.get('side_anchor')
        if reference is None or reference[1]!=imu[1]:
            reference = self.expected_gaze.targets.get('start_goal')
        if reference is None or reference[1]!=imu[1]:
            return
        axis = wrap(reference[0]-imu[0])
        candidates = []
        for part in parts:
            if part['kind'] not in ('field_box','cross_row'):
                continue
            x,y = part['points'][0]
            bearing = math.atan2(y,x)
            if abs(wrap(bearing-axis))>math.radians(35):
                continue
            if part['kind']=='cross_row' and abs(wrap(part['normal']-axis))>math.radians(45):
                continue
            candidates.append((bearing,part))
        if not candidates:
            return
        first = candidates[0][0]
        if any(abs(wrap(angle-first))>math.radians(12) for angle,_ in candidates):
            return
        angle = first + sum(wrap(angle-first) for angle,_ in candidates)/len(candidates)
        self.expected_gaze.remember('enemy_goal',(math.cos(angle),math.sin(angle)),
                                    yaw=imu[0],epoch=imu[1],now=stamp)

    def remember_gaze(self, scene):
        imu = self.gaze_imu()
        if imu is None:
            return
        stamp = scene.get('received', 0)
        if not 0 <= time.monotonic()-stamp < .5:
            return
        kwargs = dict(yaw=imu[0], epoch=imu[1], now=stamp)
        if scene.get('ball'):
            self.expected_gaze.remember('ball', scene['ball'], **kwargs)
            odom = getattr(self.g, 'odometry', None)
            if odom and odom.confidence != 'degraded':
                self.field_ball = odom.field_point(scene['ball']), stamp, odom.path, imu[1], odom.pose_revision
        # Opening endpoints have visual left/right identity. A colour blob alone
        # cannot establish which post it is, and must not overwrite that identity.
        openings = [p for p in scene.get('parts', [])
                    if p['colour'] == self.enemy and p['kind'] == 'opening']
        if not openings:
            self.remember_field_direction(scene, imu, stamp)
        if len(openings) == 1:
            ends = openings[0]['points']
            centre = tuple((a+b)/2 for a, b in zip(*ends))
            self.expected_gaze.remember('side_anchor', centre, **kwargs)
            self.expected_gaze.remember('enemy_goal', centre, **kwargs)
            if self.expected_post_side is None:
                self.expected_post_side = min(range(2), key=lambda i: abs(math.atan2(ends[i][1], ends[i][0])))
            self.expected_gaze.remember('post', ends[self.expected_post_side], **kwargs)
            for index, end in enumerate(ends):
                self.expected_gaze.remember(f'post_{index}', end, **kwargs)
        elif not openings:
            # One isolated upright can update the centre only if an earlier
            # opening identifies that same post unambiguously in IMU bearings.
            hints = [p for p in scene.get('parts', []) if p['colour'] == self.enemy and p['kind']=='colour']
            if len(hints) == 1:
                part = hints[0]
                x1,y1,x2,y2 = part.get('rect', (0,0,0,0))
                if y2-y1 > 1.5*(x2-x1) and x2>x1 and part.get('points'):
                    self.correct_single_post(part['points'][0], imu, stamp)

    def correct_single_post(self, point, imu, stamp):
        centre = self.expected_gaze.targets.get('enemy_goal')
        posts = [self.expected_gaze.targets.get(f'post_{i}') for i in range(2)]
        if any(t is None or t[1] != imu[1] or not 0 <= stamp-t[2] < 4 for t in [centre,*posts]):
            return
        observed = imu[0]+math.atan2(point[1],point[0])
        errors = sorted((abs(wrap(observed-post[0])),i) for i,post in enumerate(posts))
        (best,index),(other,_) = errors
        if best <= math.radians(10) and other-best >= math.radians(6):
            # Keep measurements tied to the original full opening's lifetime.
            self.attack_heading = wrap(observed+wrap(centre[0]-posts[index][0]))
            self.attack_seen_at = min(t[2] for t in posts)


    def visual_adjustment(self, foot):
        x1, y1, x2, y2 = foot['ball_rect']
        tip = foot['tips'][int(foot['leg'] == 'right')]
        diameter = self.p['game.ball_diameter_m']
        # Image-right is body-right. Use small corrections and reobserve after each.
        dx = max(-.025, min(.04, (foot['gap_ratio']-.32)*diameter))
        dy = max(-.03, min(.03, (tip[0]+tip[2]-x1-x2)*.5/max(1, y2-y1)*diameter))
        return dx, dy

    async def short_foot_jump(self, dx, dy):
        if abs(dy) > abs(dx):
            direction, error = ('left' if dy > 0 else 'right'), abs(dy)
        else:
            direction, error = ('forward' if dx >= 0 else 'backward'), abs(dx)
        travel_mm = self.p[f'motion.jump_{direction}_mm']
        fraction = min(.5, 1000*min(.020, error)/travel_mm)
        self.describe('simple_foot_jump', f'short {direction} jump; reobserve ball after landing')
        ball = (self.last_scene or {}).get('ball')
        if await self.jump(direction, fraction, alignment=True) and ball and not self.opening_touch_required:
            if self.g.info.get('phase') == 'simple_jump_detour':
                self.landing_pan = None
                return
            # Predict the viewing direction only. The next motion still needs a new frame.
            x, y = ball
            forward, lateral = jump_delta(self.p, direction, max(.1, fraction))
            bearing = math.atan2(y-lateral, max(.04, x-forward))
            tick = self.g.s.params.backend.Robot.camera.pan_sign * .00058909
            self.landing_pan = max(-1000, min(1000, round(bearing/tick)))

    def ball_tilt(self):
        return (self.p.get('game.simple_football.near_head_tilt', -1800)
                if self.near or self.ball_range <= .35 else self.p['game.head_tilt'])

    async def observe(self, goals=False, *, glance=False, final_aim=False):
        revision = self.g.revision
        if not await self.g._prepare(self.p, self.entry) or not self.g._same_plan(revision):
            return None, []
        self.remember_start_heading()
        began = time.monotonic()
        ball_scene = None

        def record(scene):
            now = time.monotonic()
            rev = scene.get('parameter_revision')
            self.goal_views = [(stamp, r, parts) for stamp, r, parts in self.goal_views
                               if r == rev and 0 <= now-stamp < 3]
            if scene['parts'] and 0 <= now-scene['received'] < 3:
                self.goal_views.append((scene['received'], rev, [
                    part | {'observed_at': scene['received']} for part in scene['parts']]))
                self.goal_views = self.goal_views[-16:]

        if glance:
            scene = await self.look(self.expected_pan('enemy_goal') or 0, self.p['head.field_tilt'])
            if not self.g._same_plan(revision):
                return None, []
            if scene:
                record(scene)
        elif goals:
            field = self.p['head.field_tilt']
            if final_aim:
                field = min(0, field+400)  # Include the crossbar as well as the post bases.
            poses = [(0, field), (-1778, field), (-2666, 0), (0, 0), (1778, field), (2666, 0)]
            hint, self.goal_hint = self.goal_hint, None
            predicted = self.expected_pan('enemy_goal')
            if predicted is None:
                predicted = self.expected_pan('post')
            centred_pan = None
            # Near the ball, one directed glance suffices before lowering the head.
            # If no direction is known, retain the normal two-view search.
            attempts = 1 if self.near and predicted is not None else 2
            for index in range(attempts):
                pose = poses[self.scan % len(poses)]
                if index == 0 and predicted is not None:
                    pose = (predicted, field)
                    self.g.s.log('game', 'INFO', f'Expected enemy goal gaze: pan={predicted}')
                elif index == 1 and centred_pan is not None:
                    pose = (centred_pan, field)
                elif index == 1 and predicted is not None:
                    pose = (max(-2666, min(2666, predicted + (450 if self.scan % 2 else -450))), field)
                    self.scan += 1
                elif index == 0 and hint and 0 <= began-hint[1] < 5 and hint[0] != pose:
                    pose = hint[0]  # Direction only; never authorizes a kick.
                else:
                    self.scan += 1
                    self.sectors_left = max(0, self.sectors_left-1)
                scene = await self.look(*pose)
                if not self.g._same_plan(revision):
                    return None, []
                if scene:
                    record(scene)
                    if final_aim:
                        hints = [part for part in scene['parts'] if part['colour'] == self.enemy and part['kind'] == 'colour']
                        if hints:
                            left, right = max(hints, key=lambda part: part['area'])['bearings']
                            centre = wrap(left+wrap(right-left)/2)
                            tick = self.g.s.params.backend.Robot.camera.pan_sign*.00058909
                            centred_pan = max(-2666, min(2666, round(centre/tick)))
                    if scene.get('ball'):
                        ball_scene = scene
                    if any(goal['colour'] == self.enemy and goal['shootable']
                           for goal in assemble_goals(scene['parts'], self.specs)):
                        self.goal_hint = (pose, scene['received'])
                        break
                if time.monotonic()-began > 1.8:
                    break
        foot_view = not self.danger and self.p.get('game.soft_kick.right.toe_x_mm', 0) > 0 and (self.near or self.ball_range <= .35)
        startup = self.inspect_feet_first and not self.danger
        self.inspect_feet_first = False
        turn_ball = self.turn_ball
        self.turn_ball = None
        ball_pan = 0 if foot_view or startup else self.expected_pan('ball')
        near_tilt = -2100 if startup or foot_view else self.ball_tilt()
        if self.landing_pan is not None:
            ball_pan, self.landing_pan = self.landing_pan, None
        if turn_ball:
            bearing, old_range, old_yaw = turn_ball
            yaw, _ = await self.g._imu()
            sign = self.g.s.params.backend.Robot.camera.pan_sign
            ball_pan = max(-2666, min(2666, round(wrap(bearing+old_yaw-yaw)/(sign*.00058909))))
        scene = await self.look(ball_pan or 0, near_tilt, wait_ball=True)
        roll_side, self.restart_roll_side = self.restart_roll_side, None
        if roll_side and (not scene or not scene.get('ball')) and self.g._same_plan(revision):
            # A diagonal pass can cross the entire front view during a head move.
            # Inspect the exit side before returning to the centre of the field.
            side_pan = round(roll_side * math.radians(42) /
                             (self.g.s.params.backend.Robot.camera.pan_sign * .00058909))
            for attempt in range(5):
                scene = await self.look(side_pan, self.p['game.simple_football.near_head_tilt'], wait_ball=True)
                if not self.g._same_plan(revision) or (scene and scene.get('ball')):
                    break
                if attempt < 4:
                    self.describe('restart_track_turn', 'follow the last visible roll past head travel')
                    if not await self.turn(roll_side * math.radians(15), tolerance=4):
                        break
        if (startup or ball_pan) and (not scene or not scene.get('ball')) and self.g._same_plan(revision):
            scene = await self.look(0, self.ball_tilt(), wait_ball=True)
        if turn_ball and scene and scene.get('ball'):
            if abs(math.hypot(*scene['ball'])-old_range) > max(.12, .25*old_range):
                self.describe('simple_ball_recheck', 'range changed during turn; reobserve before walking')
                self.expected_gaze.targets.pop('ball', None)
                scene = None
        if not scene or not self.g._same_plan(revision):
            return None, []
        record(scene)
        if ball_scene and ball_scene.get('parameter_revision') != scene.get('parameter_revision'):
            ball_scene = None
        ball_received = scene['received']
        ball_sequence = scene.get('sequence')
        if not final_aim and not scene.get('ball') and ball_scene and time.monotonic()-ball_scene['received'] <= .5:
            scene = scene | {'ball': ball_scene['ball'], 'foot': ball_scene.get('foot')}
            ball_received = ball_scene['received']
            ball_sequence = ball_scene.get('sequence')
        self.last_scene = scene | {'ball_sequence': ball_sequence, 'ball_received': ball_received}
        if scene.get('height_m') is not None:
            self.g.s.log('game', 'INFO', f"Observation camera height={scene['height_m']:.3f}m")
        self.shot_deadline = min([ball_received+.5] + [stamp+3 for stamp, _, _ in self.goal_views])
        self.g.info['ball'] = dict(x_m=scene['ball'][0], y_m=scene['ball'][1]) if scene['ball'] else None
        parts = [part for _, _, view in reversed(self.goal_views) for part in view]
        goals_seen = assemble_goals(parts, self.specs)
        return scene['ball'], goals_seen

    async def observe_keeper(self, ball, yaw):
        """Take one directed view; store the aiming bearing before looking down."""
        self.keeper_glance_at = time.monotonic()
        revision = self.g.revision
        self.describe('simple_keeper_view', 'one look into the opponent goal')
        view = await self.look(0, -220)
        if not self.g._same_plan(revision):
            return
        goals = assemble_goals((view or {}).get('parts', []), self.specs)
        goal = next((g for g in goals if g['colour'] == self.enemy and g.get('shootable')), None)
        source = (goal or {}).get('opening_source', {})
        keepers = source.get('keepers', [])
        windows = free_goal_windows(dict(goal, opening_source={'occupied': keepers}) if goal else None,
                                    self.p['game.ball_diameter_m']/2+.025)
        if windows:
            left, right = max(windows, key=lambda window: math.dist(*window))
            x,y = ((a+b)/2 for a,b in zip(left,right))
            fraction = math.dist(goal['ends'][0], (x,y))/math.dist(*goal['ends'])
            heading = wrap(yaw+math.atan2(y-ball[1],x-ball[0]))
            odom = getattr(self.g,'odometry',None)
            self.keeper_aim = (heading, time.monotonic(), odom.field_point(ball) if odom else None)
            self.g.info['shot_aim'] = dict(frame=view.get('sequence'), keepers=keepers, line_gaps=source.get('line_gaps',[]),
                penalty_box=source.get('penalty_box',False), source='keeper_window' if keepers else 'posts_center',
                fraction=fraction, heading_rad=heading, feet_px=source.get('feet_px'), rect=source['rect'])
            await self.g._call('detection', 'football.decision', {'shot_aim': self.g.info['shot_aim']})
            self.g.s.log('game','INFO',f"Visual shot target {self.g.info['shot_aim']}")
        elif not goal:
            posts = [p for p in (view or {}).get('parts', [])
                     if p['kind'] == 'mapped_post' and p['colour'] == self.enemy]
            odom = getattr(self.g, 'odometry', None)
            if len(posts) == 1:
                x,y = posts[0]['goal_center']
                source_name = 'single_' + posts[0]['side']
            elif odom and odom.confidence != 'degraded':
                spec = self.specs[1-self.p['match.own_goal']]
                x,y = odom.body_point((spec['x'],spec['y']))
                source_name = 'map_center'
            else:
                self.inspect_feet_first = True
                return
            heading = wrap(yaw+math.atan2(y-ball[1],x-ball[0]))
            odom = getattr(self.g,'odometry',None)
            self.keeper_aim = (heading, time.monotonic(), odom.field_point(ball) if odom else None)
            self.g.info['shot_aim'] = dict(frame=(view or {}).get('sequence'), keepers=[],
                                           source=source_name, fraction=.5, heading_rad=heading)
        self.inspect_feet_first = True

    async def attack_after_pass(self, ball, goals, *, rechecked=False):
        """Move behind the observed ball, then put the striking foot on the shot line."""
        revision = self.g.revision
        yaw, _ = await self.g._imu()
        scene = self.last_scene or {}
        stamp = scene.get('ball_received', scene.get('received', 0))
        if not self.g._same_plan(revision) or not scene.get('confirmed') or not 0 <= time.monotonic()-stamp <= .5:
            return
        x, y = ball
        distance = math.hypot(x, y)
        self.ball_range = distance
        goal = next((g for g in goals if g['colour'] == self.enemy and g.get('shootable')), None)
        radius = self.p['game.ball_diameter_m']/2
        if goal and 'keepers' in goal.get('opening_source', {}):
            source = goal['opening_source']
            goal = dict(goal, opening_source=dict(source, occupied=source['keepers']))
        windows = free_goal_windows(goal, radius+.05)
        if windows:
            def window_score(window):
                mx, my = ((a+b)/2 for a, b in zip(*window))
                return (round(math.dist(*window), 3), -abs(math.atan2(my-y, mx-x)))
            best = max(windows, key=window_score)
            gx, gy = ((a+b)/2 for a, b in zip(*best))
            self.attack_heading = wrap(yaw + math.atan2(gy-y, gx-x))
            self.attack_seen_at = time.monotonic()
        target = self.attack_heading if time.monotonic()-self.attack_seen_at < 5 and self.attack_heading is not None else self.attack_axis
        odom = getattr(self.g, 'odometry', None)
        map_tolerance = None
        if odom and time.monotonic()-self.attack_seen_at >= 5:
            map_tolerance = 0.
        if map_tolerance is not None and odom.confidence != 'degraded':
            enemy = self.specs[1-self.p['match.own_goal']]
            aim_y = enemy['y']
            ball_field = odom.field_point(ball)
            if (not self.central_lane_checked and not self.placed_kickoff_started and
                    .23 < x < .40 and abs(y) < .06 and math.hypot(*ball_field) < .09):
                self.central_lane_checked = True
                view = await self.look(0, self.p['head.field_tilt'])
                if not self.g._same_plan(revision):
                    return
                screened = any(part['kind'] == 'obstacle' and x < part['points'][0][0] < x+.65
                               and abs(part['points'][0][1]-y) < part['radius']+radius+.09
                               for part in (view or {}).get('parts', []))
                if screened:
                    close = await self.look(0, self.ball_tilt(), wait_ball=True)
                    if not self.g._same_plan(revision):
                        return
                    if close and close.get('ball') and math.dist(close['ball'],ball) < .06:
                        self.describe('simple_carry_wide', 'screened at midfield; play a diagonal touch')
                        await self.diagonal_pass(close['ball'], kickoff=False)
                return  # The previous toe observation predates the head scan.
            gx, gy = odom.body_point((enemy['x'], aim_y))
            goal_range = math.hypot(gx-x, gy-y)
            allowance = odom.position_error_m()
            if goal_range > max(.25, allowance):
                target = wrap(yaw+math.atan2(gy-y,gx-x))
                # The entire projected ray uncertainty must fit inside the mouth.
                free_half_width = max(0., enemy['width']/2-radius-allowance)
                map_tolerance = math.atan2(free_half_width, goal_range + allowance)
        visual_aim = (self.keeper_aim and time.monotonic()-self.keeper_aim[1] <= 18
                      and (self.keeper_aim[2] is None or odom and
                           math.dist(odom.field_point(ball),self.keeper_aim[2]) < .18))
        if visual_aim:
            target = self.keeper_aim[0]
            map_tolerance = None
        # As in Roki_2_Soccer, use the foot on the observed side. Only the
        # right foot has a measured metric window, so a left strike needs
        # visual confirmation at the toe as well as a left-sided ball.
        foot = scene.get('foot')
        visible_reach = bool(foot and foot['aligned'] and -.05 <= foot['gap_ratio'] <= .4)
        use_left = (not self.opening_touch_required and visible_reach and
                    foot['leg'] == 'left' and y > .020 and distance <= .22)
        leg = 'left' if use_left else 'right'
        offset = self.p['game.kick_offset_mm']/1000
        toe = self.p['game.soft_kick.right.toe_x_mm']/1000
        if use_left:
            offset = -offset
        ready = (foot['leg'] == leg and visible_reach
                 if foot else toe > 0 and -.01 <= x-radius-toe <= .04 and abs(y+offset) <= .024)
        # Both estimates come from this camera frame. A partially obscured toe
        # can miss the dark tip even with the ball inside its measured reach.
        calibrated = (leg == 'right' and toe > 0 and -.012 <= x-radius-toe <= .04 and abs(y+offset) <= .027)
        ready |= calibrated and (not foot or foot['leg'] == leg)
        # Compare the nearest observed opponent surface to the ball.
        clearance = min((math.dist(ball, part['points'][0]) - part['radius']
                         for part in scene.get('parts', []) if part['kind'] == 'obstacle'),
                        default=math.inf)
        blocked = any(0 < part['points'][0][0]-x < .5 and
                      abs(part['points'][0][1]-y) <= part['radius']+radius
                      for part in scene.get('parts', []) if part['kind'] == 'obstacle')
        fast_shot = (not self.opening_touch_required and not blocked and ready and clearance <= .30 and
                     any(shot_clear(ball, goal | {'ends': window}, 0.) for window in windows))
        if not fast_shot and distance <= .22 and ready and self.keeper_aim and not visual_aim and not self.final_keeper_check:
            # Do this even when odometry supplies a plausible fallback course.
            self.final_keeper_check = True
            self.trimmed_shot = False
            self.keeper_aim = None
            self.g.info.pop('shot_aim', None)
            self.g.s.log('game','INFO','Refreshing stale goal view before strike')
            await self.observe_keeper(ball, yaw)
            if self.g._same_plan(revision):
                self.ball_refresh_after = time.monotonic()
            return
        if target is None and odom:
            if odom.confidence == 'degraded':
                target = wrap(odom.home_heading - odom.yaw_offset)
            else:
                enemy = self.specs[1-self.p['match.own_goal']]
                gx, gy = odom.body_point((enemy['x'], enemy['y']))
                target = wrap(yaw + math.atan2(gy-y, gx-x))
        if target is None:
            target = self.remembered_goal_heading()
        if target is None:
            self.goal_scan_needed = self.near = True
            return
        error = wrap(target-yaw)
        aim_limit = 12
        known_lane = target is not None and abs(error) <= math.radians(aim_limit)
        if distance > .24:
            self.trimmed_shot = False
        if (self.placed_kickoff_started and not ready and .18 < distance < .28
                and abs(error) < math.radians(18) and self.keeper_glance_at == -math.inf):
            # Aim before entering striking reach; a large turn at the toe can
            # sweep the ball aside. The next frame places the foot on this ray.
            await self.observe_keeper(ball,yaw)
            return
        if visual_aim and not ready and .18 <= distance <= .30 and abs(error)>math.radians(6):
            self.describe('simple_aim_adjust', 'align before the final approach')
            await self.turn(error,tolerance=6)
            return
        if fast_shot:
            on_target = True
        elif distance <= .22 and ready:
            if not self.final_keeper_check and not rechecked and not known_lane and not windows and time.monotonic()-self.keeper_glance_at > 12:
                await self.observe_keeper(ball,yaw)
                return
            if self.p.get('sim.precise_strike') and visual_aim and not self.trimmed_shot and abs(error) > math.radians(3):
                self.trimmed_shot = True
                self.describe('simple_aim_adjust', 'reduce free-window heading error once at the toe')
                await self.turn(error, tolerance=3)
                return
            if visual_aim and abs(error) > math.radians(8 if self.trimmed_shot else 5):
                self.trimmed_shot = True
                self.describe('simple_aim_adjust', 'turn towards visual free-window centre')
                await self.turn(error, tolerance=5)
                return
            if not visual_aim and not rechecked and not (calibrated and known_lane):
                self.check_shot_view = True
                self.describe('simple_aim_check', 'check opponent goal, then confirm ball at toe')
                return
            clear_ray = any(shot_clear(ball, goal | {'ends': window}, 0.) for window in windows)
            take_shot = clear_ray and self.p['game.simple_football.take_clear_shot']
            if windows and not take_shot and not self.trimmed_shot and abs(error) > math.radians(4):
                self.trimmed_shot = True
                self.describe('simple_aim_adjust', f'centre of free opening {math.degrees(error):+.1f}deg')
                await self.turn(error, tolerance=4)
                return
            if windows:
                on_target = clear_ray
            elif goal:
                on_target = shot_clear(ball, goal, radius)
            else:
                tolerance = math.radians(15)
                if map_tolerance is not None and not (ready and known_lane) and not rechecked:
                    tolerance = min(tolerance, map_tolerance)
                    if abs(error) >= tolerance:
                        self.goal_scan_needed = self.near = True
                        self.describe('simple_aim_check', 'odometry uncertainty requires a goal observation')
                        return
                on_target = abs(error) < tolerance
        else:
            on_target = False
        if on_target:
            own = next((g for g in goals if g['colour'] == self.own), None)
            if own_goal_blocks(ball, own, radius) or (goal and goal.get('opening_source', {}).get('occupied') and not windows):
                self.goal_scan_needed = self.near = True
                return
            record = self.g.info.get('shot_aim') or {}
            self.g.info['strike_course'] = dict(under_pressure=fast_shot, heading_rad=target, residual_rad=error,
                                                source=record.get('source') if visual_aim else 'current_goal_or_map',
                                                fraction=record.get('fraction') if visual_aim else None)
            self.shot_deadline = stamp+.5
            reach_source = 'visible toe' if visible_reach else 'projected ball at calibrated foot'
            self.describe('simple_kick', f'{reach_source}; {len(windows)} free windows; aim error {math.degrees(error):+.1f}deg')
            if await self.strike(dict(leg=leg, power=80, offset=max(-60, min(80, round(abs(y)*1000)-62)))):
                self.near = False
                self.goal_scan_needed = True
                self.attack_heading = None
                self.trimmed_shot = False
                self.keeper_aim = None
                self.keeper_glance_at = -math.inf
                self.final_keeper_check = False
                self.last_open_shot = time.monotonic()
                self.ball_range = .65  # Inspect the outgoing ball and any rebound.
            return
        # Leave space for turning and lateral foot placement before closing.
        turning = abs(error) > math.radians(14) or (ready and distance <= .22 and bool(windows))
        setback = .22 if turning else max(.10, toe+radius+.015)
        dx = x - setback*math.cos(error) - offset*math.sin(error)
        dy = y - setback*math.sin(error) + offset*math.cos(error)
        if turning and distance < .36:
            if abs(dy) <= .075 and dx >= -.075:
                self.describe('simple_attack_turn', 'ball behind approach point; align to shot')
                await self.turn(error, tolerance=10)
                return
            if dx < -.045:
                direction = 'backward'
            elif abs(dy) > .06:
                direction = 'left' if dy > 0 else 'right'
            else:
                self.describe('simple_attack_turn', 'turn towards goal behind ball')
                await self.turn(error, tolerance=10)
                return
            if self.p.get('sim.direct_pursuit') and direction in ('left', 'right') and x > toe+radius+.04:
                self.describe('simple_flank_walk', 'sidestep behind observed ball')
                await self.walk(min(.14, abs(dy)), math.copysign(math.pi/2, dy))
                return
            self.describe('simple_attack_orbit', f'{direction}; keep clearance from ball')
            await self.jump(direction, min(1., max(.35, max(abs(dx),abs(dy))/.04)))
            return
        if abs(math.atan2(y, x)) > math.radians(45) and distance > .30:
            await self.turn(math.atan2(y, x))
            return
        travel = math.hypot(dx, dy)
        self.near = distance < .32
        self.describe('simple_attack_walk', f'place {leg} foot behind ball; range={distance:.3f}m')
        imu = self.gaze_imu()
        if distance > .35 and imu:
            region = self.expected_gaze.ball_region(yaw, imu[1], time.monotonic(), lead=.30)
            if region:
                dx += max(-.08,min(.08,region[0]-x))
                dy += max(-.08,min(.08,region[1]-y))
        course = math.atan2(dy, dx)
        c, sn = math.cos(course), math.sin(course)
        for part in scene.get('parts', []):
            if part['kind'] != 'obstacle':
                continue
            ox, oy = part['points'][0]
            along, side = ox*c+oy*sn, oy*c-ox*sn
            clearance = .10+part['radius']
            if .08 < along < min(travel+.04, .42) and abs(side) < clearance:
                if not self.detour_side:
                    self.detour_side = (-1 if side > 0 else 1) if abs(side) > .02 else (1 if self.p['game.simple_football.kickoff_side']=='left' else -1)
                course += self.detour_side * math.atan2(clearance+.03-abs(side), max(.06,along-.10))
                self.describe('simple_bypass', f'avoid field obstacle on side {self.detour_side:+d}')
                break
        else:
            self.detour_side = 0
        # Cover a diagonal setup in one walking job. Reserve fine hops for
        # the final toe adjustment, where a full gait cycle could touch the ball.
        if (self.p.get('game.simple_football.live_approach') and not self.detour_side
                and dx > .025 and x > .10 and abs(y) < .30 and abs(course) < math.radians(65)):
            # Express the desired ball position in the current body frame.
            # The gait holds that heading and recomputes both errors from the camera.
            self.describe('simple_live_approach', 'track ball to striking position in one walking job')
            await self.move('game.walk', dict(kind='approach', distance_m=distance, heading=0.,
                yaw_offset=wrap(-yaw), x_m=x, y_m=y, stamp=stamp, live_approach=True,
                stop_x_m=max(.08, x-dx), target_y_m=y-dy))
        elif distance < .19 and travel < .035:
            await self.short_foot_jump(dx, dy)
        else:
            stride = .22
            if self.p.get('sim.direct_pursuit') and not self.detour_side and distance > .45 and abs(course) < math.radians(30):
                stride = min(.38, distance-.22)
            await self.walk(min(stride, travel), course)

    async def watch_restart_ball(self):
        """Use only the head while waiting. Return true on confirmed ball displacement."""
        self.describe('start_later_watch', 'stationary; observing ball')
        pan = self.expected_pan('ball') or 0
        scene = await self.look(pan, self.p['game.head_tilt'], wait_ball=True)
        ball = (scene or {}).get('ball')
        if not ball:
            return False
        self.last_scene = scene
        stamp = scene.get('ball_received', scene.get('received', time.monotonic()))
        previous = self.waiting_sample
        self.waiting_sample = (tuple(ball), stamp)
        baseline = self.waiting_ball
        if baseline is None:
            self.waiting_ball = tuple(ball)
            return False
        if math.dist(ball, baseline) <= self.p['game.ball_diameter_m']/2:
            return False
        # Aim the first pursuit frame at the observed roll. Body-frame coordinates
        # are comparable because the delayed player has kept its feet stationary.
        x, y = ball
        self.inspect_feet_first = False
        self.ball_range = math.hypot(x, y)
        self.last_ball_side = 1 if y >= 0 else -1
        tick = self.g.s.params.backend.Robot.camera.pan_sign * .00058909
        lateral = y - baseline[1]
        self.restart_roll_side = (1 if lateral > 0 else -1) if abs(lateral) > .015 else None
        self.restart_pivot = bool(self.restart_roll_side and x < baseline[0] - .025 and x < .5)
        aim_x, aim_y = x, y
        if previous:
            (px, py), prior_stamp = previous
            dt = stamp - prior_stamp
            if .02 <= dt <= .6:
                # Lead the next camera exposure by at most 15 cm. The gait
                # still uses the next confirmed detection to plan the approach.
                dx, dy = x - px, y - py
                gain = min(.20 / dt, .15 / max(math.hypot(dx, dy), 1e-6))
                aim_x, aim_y = x + gain*dx, y + gain*dy
        self.landing_pan = max(-2666, min(2666, round(math.atan2(aim_y, aim_x) / tick)))
        return True

    async def placed_kickoff(self):
        """Approach the placed ball on a curved gait, then finish the diagonal pass."""
        revision = self.g.revision
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        self.kickoff_pending = False
        self.placed_kickoff_started = True
        self.remember_start_heading()
        self.kickoff_side = self.p['game.simple_football.kickoff_side']
        self.attack_axis = self.kickoff_heading = yaw
        toe = self.p['game.soft_kick.right.toe_x_mm']
        side = self.p['game.kick_offset_mm'] + self.p['game.soft_kick.right.outward_trim_mm']
        depth = (toe + self.p['game.soft_kick.right.forward_trim_mm'] + 20)/1000 + self.p['game.ball_diameter_m']/2
        sign = 1 if self.kickoff_side == 'left' else -1
        final = sign*math.radians(60)
        # Put the ball ahead of the right toe after turning towards the pass.
        dx = .30 - depth*math.cos(final) - side/1000*math.sin(final)
        dy = -depth*math.sin(final) + side/1000*math.cos(final)
        distance = max(.04, math.hypot(dx,dy)+self.p['game.simple_football.placed_approach_trim_mm']/1000)
        self.describe('simple_placed_approach', f'centre kickoff arc {distance*1000:.0f}mm')
        if not await self.g._move('game.walk', dict(kind='kickoff_arc', distance_m=distance,
                course=math.atan2(dy,dx), invert=sign, yaw_offset=wrap(-yaw))) or not self.g._same_plan(revision):
            return
        if not await self.g._move('motion.pose', {'name':'base_stand'}) or not self.g._same_plan(revision):
            return
        self.approach_crouched = False
        target = wrap(yaw + sign*math.radians(60))
        direction = 'turn_left' if sign > 0 else 'turn_right'
        full_turn = abs(self.p['motion.jump_yaw_ccw' if sign > 0 else 'motion.jump_yaw_cw'])
        for _ in range(4):
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
            error = sign*wrap(target-yaw)
            if error <= full_turn/2:
                break
            self.describe('simple_pass_heading', f'full {direction} jump; remaining {math.degrees(error):.1f}deg')
            if not await self.jump(direction, 1.) or not self.g._same_plan(revision):
                return
        self.opening_jump_pending = True
        self.opening_right_jumps_left = 0
        self.pending_toe_pass = (toe, side)
        await self.place_and_pass(revision)

    async def diagonal_pass(self, ball, *, kickoff):
        if kickoff:
            return await self.placed_kickoff()
        revision = self.g.revision
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        self.kickoff_pending = False
        self.remember_start_heading()
        self.placed_kickoff_started = True
        self.attack_axis = self.kickoff_heading = yaw
        # Aim outside the goal mouth even from the centre of the small field.
        side_name = self.p['game.simple_football.kickoff_side']
        target = wrap(yaw + self.opening_pass_course(yaw, diagonal_deg=45))
        self.describe('simple_pass_heading', 'face diagonal for open-play pass')
        for _ in range(8):
            error = wrap(target-yaw)
            if abs(error) <= math.radians(6):
                break
            if not await self.turn(error, tolerance=6) or not self.g._same_plan(revision):
                return
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
        if not await self.g._move('motion.pose', {'name': 'base_stand'}) or not self.g._same_plan(revision):
            return
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        angle = wrap(yaw-self.kickoff_heading)
        leg = 'right'
        toe = self.p['game.soft_kick.right.toe_x_mm']
        # Field trials undershot the right-foot setup. Compensate planned travel.
        front = (toe + 20 + self.p['game.ball_diameter_m']*500 if toe
                 else self.p['game.kick_distance_mm'])
        front += self.p['game.soft_kick.right.forward_trim_mm'] - 15
        side = self.p['game.kick_offset_mm'] + self.p['game.soft_kick.right.outward_trim_mm']
        bx, by = ball
        dx = bx*math.cos(angle)+by*math.sin(angle)-front/1000 + self.p['game.simple_football.placed_approach_trim_mm']/1000
        dy = -bx*math.sin(angle)+by*math.cos(angle)+side/1000
        self.describe('simple_placed_approach', f'ball at {bx*1000:.0f},{by*1000:.0f}mm; approach {leg} foot')
        if not await self.g._move('game.walk', dict(kind='omni', distance_m=math.hypot(dx, dy),
                course=math.atan2(dy, dx), heading=0., yaw_offset=wrap(-yaw))):
            return
        if not self.g._same_plan(revision):
            return

        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        self.placed_ball_pass = True
        try:
            if not await self.finish_diagonal_alignment(revision, toe, side):
                return
            self.describe('simple_placed_pass', f'weak diagonal {side_name}; heading error {math.degrees(wrap(target-yaw)):.1f}deg')
            if not await self.strike(dict(leg=leg, power=38, soft=True, offset=round(side)-62)):
                return
        finally:
            self.placed_ball_pass = False

    async def follow_kickoff(self, ball, centre, scene):
        """One walking job from the kick to the ball's flank, clearing the centre circle."""
        x, y = ball
        distance = math.hypot(x, y)
        if x <= 0 or distance < .12:
            return
        radius = self.p['field.geometry']['circle_diameter']/2
        # At this instant the placed ball's previous position is the centre mark
        # expressed in our body frame. No walking or body turn has intervened.
        cx, cy = centre
        bearing = math.atan2(y, x)
        for side in (1, -1):
            course = bearing + side*math.atan2(.19, distance)
            c, sn = math.cos(course), math.sin(course)
            along = cx*c + cy*sn
            cross = cy*c - cx*sn
            exit_distance = along + math.sqrt(max(0., (radius+.06)**2-cross**2))
            travel = max(distance-.10, exit_distance)
            if any(0 < ox*c+oy*sn < travel+.10 and abs(oy*c-ox*sn) < obstacle['radius']+.10
                   for obstacle in scene.get('parts', []) if obstacle['kind'] == 'obstacle'
                   for ox, oy in obstacle['points'][:1]):
                continue
            self.describe('simple_kickoff_chase', f'one walking job; ball {distance:.2f}m, travel {travel:.2f}m')
            revision = self.g.revision
            initial_yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision) or not await self.walk(travel, course):
                return
            if self.attack_axis is None:
                return
            # Face the opponent goal before looking down. The ball can now be
            # beside the foot and outside a camera still pointing along the pass.
            enemy = self.specs[1-self.p['match.own_goal']]
            goal_range = math.hypot(enemy['x'], enemy['y'])
            attack = self.attack_axis-initial_yaw
            heading = initial_yaw + math.atan2(cy+goal_range*math.sin(attack)-y,
                                               cx+goal_range*math.cos(attack)-x)
            self.describe('simple_chase_align', 'align body from IMU; reacquire the ball at the feet')
            for _ in range(8):
                yaw, _ = await self.g._imu()
                if not self.g._same_plan(revision):
                    return
                error = wrap(heading-yaw)
                if abs(error) < math.radians(8):
                    break
                if not await self.turn(error, tolerance=7):
                    return
            self.near = self.inspect_feet_first = True
            self.goal_scan_needed = False
            return

    async def finish_diagonal_alignment(self, revision, toe_mm, offset_mm):
        """Keep the pass heading while the same nearby ball is at the right foot."""
        previous = None
        for attempt in range(3):
            self.describe('simple_ball_recheck', 'diagonal approach complete; checking ball at right toe')
            close = await self.look(0, -2100, wait_ball=True)
            if not self.g._same_plan(revision) or not close or not close.get('ball'):
                return False
            x, y = close['ball']
            ball_radius_mm = self.p['game.ball_diameter_m']*500
            gap = round(x*1000 - toe_mm - ball_radius_mm)
            lateral = round(y*1000 + offset_mm)
            self.g.s.log('game', 'INFO', f'Diagonal right toe: gap={gap}mm lateral={lateral:+d}mm')
            # Once the approach is complete, even an off-centre contact is useful.
            # A ball overlapping the foot's striking axis can be played immediately.
            if -12 <= gap <= 40 and abs(lateral) <= max(30, ball_radius_mm):
                return True
            if attempt == 2 or gap < -20 or gap > 75 or abs(lateral) > 65:
                return False
            if previous is not None and math.dist((x,y), previous) > .10:
                return False
            previous = (x, y)
            await self.short_foot_jump((gap-20)/1000, lateral/1000)
            if not self.g._same_plan(revision):
                return False
        return False

    async def chase_pass_segment(self, scene, revision):
        """Advance along the kick heading, checking the path again after each segment."""
        if not self.g._same_plan(revision):
            return
        if not scene or not scene.get('valid') or not scene.get('confirmed'):
            self.pass_segments_left = 0
            odom = getattr(self.g, 'odometry', None)
            if odom:
                cx, cy = self.pass_centre
                self.search_track = BallRegion(odom.field_point((cx+.40, cy)), time.monotonic(),
                                                   (0., 0.), odom.pose_revision, odom.path, 'kick_direction')
                await self.approach_ball_region()
            return
        if not 0 <= time.monotonic() - scene['received'] <= .5 or not self.g._same_plan(revision):
            return
        ball = scene.get('ball')
        if ball and (ball[0] <= 0 or math.hypot(*ball) <= .20):
            self.pass_segments_left = 0
            self.inspect_feet_first = True
            return
        course = math.atan2(ball[1], ball[0]) if ball else 0.
        radius = self.p['field.geometry']['circle_diameter'] / 2
        cx, cy = self.pass_centre
        obstacles = list(scene.get('frame_obstacles', []))
        obstacles.extend((part['points'][0], part['radius']) for part in scene.get('parts', [])
                         if part['kind'] == 'obstacle')
        # Check the swept segment, including both endpoints. If already close
        # to an opponent, allow only a course that increases separation.
        for offset in (0, 35, -35, 65, -65, 90, -90):
            direction = course + math.radians(offset)
            c, s = math.cos(direction), math.sin(direction)
            along_centre = cx*c + cy*s
            cross_centre = cy*c - cx*s
            remaining = along_centre + math.sqrt(max(0., radius**2 - cross_centre**2))
            if math.hypot(cx, cy) >= radius or remaining <= .01:
                self.pass_segments_left = 0
                return
            distance = min(radius, remaining, math.hypot(*ball) - .14 if ball else math.inf)
            for (x, y), obstacle_radius in obstacles:
                along = x*c + y*s
                nearest = min(distance, max(0., along))
                if along >= 0 and math.hypot(x-nearest*c, y-nearest*s) < obstacle_radius + .13:
                    break
            else:
                self.describe('simple_pass_follow', f'follow diagonal {distance:.2f}m, course {math.degrees(direction):+.0f}deg')
                if await self.walk(distance, direction) and self.g._same_plan(revision):
                    self.pass_centre = (cx-distance*c, cy-distance*s)
                    self.pass_segments_left -= 1
                return
        self.describe('simple_pass_blocked', 'opponent blocks the pass corridor; recheck camera')

    def opening_ball_at_foot(self, scene, toe_mm, side_mm):
        x, y = scene['ball']
        gap = x - toe_mm/1000 - self.p['game.ball_diameter_m']/2
        lateral = y + side_mm/1000
        foot = scene.get('foot')
        aligned = abs(lateral) <= .030
        in_range = -.012 <= gap <= .030
        if foot:
            aligned = foot['leg'] == 'right' and foot['aligned']
            in_range = -.08 <= foot['gap_ratio'] <= .34
        return gap, lateral, aligned, in_range

    def jump_corridor_blocked(self, scene, direction, fraction=1.):
        travel = abs(self.p[f'motion.jump_{direction}_mm'])*max(.1, min(1., fraction))/1000
        dx, dy = {'forward': (1,0), 'backward': (-1,0), 'left': (0,1), 'right': (0,-1)}[direction]
        for (ox, oy), radius in scene.get('frame_obstacles', []):
            along, across = ox*dx+oy*dy, oy*dx-ox*dy
            if along > 0 and math.hypot(max(0., along-travel), across) < radius+.13:
                return True
        return False

    def pass_jump_blocked(self, scene, direction):
        if self.jump_corridor_blocked(scene, direction):
            self.describe('simple_pass_blocked', f'opponent blocks {direction} pass jump')
            return True
        return False

    async def visual_opening_pass(self, revision):
        toe, side = self.pending_toe_pass
        self.opening_jump_pending = False
        scene = await self.look(0, -2100)
        if not self.g._same_plan(revision):
            return
        if not scene or not scene.get('ball'):
            if await self.follow_hidden_ball(scene):
                return
            self.describe('visual_ball_recheck', 'inspect expected foot area from a second angle')
            scene = await self.look(0, -1700, single_frame=True)
            if not self.g._same_plan(revision):
                return
            if not scene or not scene.get('ball'):
                self.pending_toe_pass = None
                if self.placed_kickoff_started and self.opening_touch_required:
                    # The placed-ball manoeuvre just reached the centre mark.
                    # Try once, then verify or pursue; contact remains unknown.
                    self.describe('simple_expected_touch', 'weak touch at nominal centre placement')
                    if await self.strike(dict(leg='right', soft=True, power=38, offset=0)) and self.g._same_plan(revision):
                        self.kick_check = (None, None, toe, side)
                return
        depth = toe/1000 + self.p['game.ball_diameter_m']/2 + .02
        lateral = side/1000
        x, y = scene['ball']
        # At the edge of the calibrated window another view can change the choice.
        clearance = min((math.dist((x, y), pos) - radius
                         for pos, radius in scene.get('frame_obstacles', ())), default=math.inf)
        error = max(abs(x-depth), abs(abs(y)-lateral))
        if not scene.get('foot') and .015 < error < .03 and clearance > .30:
            check = await self.look(0, -1700, single_frame=True)
            if not self.g._same_plan(revision):
                return
            if check and check.get('ball'):
                scene = check
                x, y = scene['ball']
        self.last_scene = scene
        foot = scene.get('foot')
        self.g.s.log('game', 'INFO', f"Opening frame {scene.get('sequence')}: ball={x:.3f},{y:+.3f}m toe={foot['leg'] if foot else 'unconfirmed'} obstacles={len(scene.get('frame_obstacles', []))}")
        if foot:
            leg = foot['leg']
            aligned = foot['aligned'] and -.08 <= foot['gap_ratio'] <= .40
            reason = 'visible ball in toe window'
            dx, dy = self.visual_adjustment(foot)
        else:
            leg = 'right' if y < 0 else 'left'
            dx, dy = x-depth, y-math.copysign(lateral, y)
            aligned = max(abs(dx), abs(dy)) <= .02
            reason = 'visible ball in calibrated foot window'
        if aligned:
            self.describe('visual_opening_kick', reason)
            self.pending_toe_pass = None
            if await self.strike(dict(leg=leg, soft=True, power=38, offset=0)) and self.g._same_plan(revision):
                self.kick_check = (scene.get('kick_view'), (x,y), toe, side)
            return
        # Roki_2_Soccer fine_adjustment_before_kick: correct fore/aft first,
        # then lateral error with the calibrated directional jumping slot.
        # Reobserve after each landing so a displaced ball cannot drive two moves.
        if foot and not foot['aligned'] and abs(dy) > .010 and -.12 <= foot['gap_ratio'] <= .55:
            direction = 'left' if dy > 0 else 'right'
        elif abs(dx) > .010:
            direction = 'forward' if dx > 0 else 'backward'
        elif abs(dy) > .010:
            direction = 'left' if dy > 0 else 'right'
        else:
            # Sub-centimetre trim when projection and toe visibility disagree.
            await self.short_foot_jump(dx, dy)
            return
        self.describe('visual_opening_jump', f'{direction} jump; recheck ball after landing')
        await self.jump(direction, 1., alignment=True)

    async def place_and_pass(self, revision):
        if self.p.get('game.simple_football.opening_strike') == 'foot':
            return await self.visual_opening_pass(revision)
        toe_mm, side_mm = self.pending_toe_pass
        if self.opening_jump_pending:
            while True:
                direction = 'right' if self.opening_right_jumps_left else 'forward'
                scene = await self.g._call('detection', 'football.status')
                if not self.g._same_plan(revision):
                    return
                if scene and 0 <= time.monotonic()-scene.get('received', 0) <= .5 and self.pass_jump_blocked(scene, direction):
                    return
                if direction == 'forward':
                    break
                self.describe('simple_pass_side_jump', f'right jump {4-self.opening_right_jumps_left}/3')
                if not await self.jump('right', 1.) or not self.g._same_plan(revision):
                    return
                self.opening_right_jumps_left -= 1
            self.opening_jump_pending = False
            self.pending_toe_pass = None
            self.describe('simple_placed_pass', 'fixed opening forward jump')
            if await self.strike(dict(leg='right', soft=True, power=38, offset=0), default_jump=True) and self.g._same_plan(revision):
                self.kick_check = (None, None, toe_mm, side_mm)
            return
        self.describe('simple_toe_check', 'check the ball against the calibrated foot position')
        scene = await self.look(0, -2100)
        if not self.g._same_plan(revision):
            return
        if not scene or not scene.get('ball') or not 0 <= time.monotonic()-scene.get('received',0) <= .5:
            self.pending_toe_pass = None
            self.placed_kickoff_started = False
            await self.approach_ball_region()
            return
        x, y = scene['ball']
        gap, lateral, aligned, in_range = self.opening_ball_at_foot(scene, toe_mm, side_mm)
        self.g.s.log('game', 'INFO', f'Opening forward hop: gap={gap*1000:.0f}mm lateral={lateral*1000:+.0f}mm')
        travel = self.p['motion.jump_forward_mm']/1000
        if self.pass_jump_blocked(scene, 'forward'):
            return
        if gap > travel and not (aligned and in_range):
            self.describe('simple_toe_hop', 'one forward approach jump; recheck ball')
            await self.jump('forward', 1.)
            return
        self.pending_toe_pass = None
        self.toe_camera_revision = None
        self.describe('simple_placed_pass', 'play the ball with one full forward jump')
        if not await self.strike(dict(leg='right', soft=True, power=38, offset=0), default_jump=True):
            return
        if self.g._same_plan(revision):
            self.kick_check = (scene.get('kick_view'), (x,y), toe_mm, side_mm)
            self.describe('simple_kickoff_verify', 'verify ball against pitch markings after the forward jump')

    async def check_opening_kick(self, revision):
        from roki_ng.detection.kick_feedback import kick_result
        before, point, toe, side = self.kick_check
        scene = await self.look(0, -2100, wait_ball=True)
        if not self.g._same_plan(revision):
            return
        if not scene or not scene.get('confirmed') or not 0 <= time.monotonic()-scene.get('received', 0) <= .5:
            self.describe('simple_kick_unconfirmed', 'no usable image; search along attempted pass')
            self.kick_check = None
            self.placed_ball_reference = point or (toe/1000 + self.p['game.ball_diameter_m']/2, -side/1000)
            return
        if scene.get('ball'):
            gap, lateral, aligned, in_range = self.opening_ball_at_foot(scene, toe, side)
            radius = self.p['game.ball_diameter_m']/2
            reach = self.p['motion.jump_forward_mm']/1000
            reachable = -radius <= gap <= reach + radius and abs(lateral) <= radius + .030
            self.g.s.log('game', 'INFO', f'After opening jump: gap={gap*1000:.0f}mm lateral={lateral*1000:+.0f}mm reachable={reachable}')
            if (aligned and in_range) or reachable:
                self.kick_check = None
                self.pending_toe_pass = (toe, side)
                self.describe('simple_pass_retry', 'ball still at right toe; repeat forward pass jump')
                return
        if point is None:
            if not scene.get('confirmed') or not 0 <= time.monotonic()-scene.get('received', 0) <= .5:
                return
            # No pre-jump measurement in the fixed opening. Resume tracking
            # from the nominal toe position without claiming measured contact.
            self.kick_check = None
            self.placed_ball_reference = (toe/1000 + self.p['game.ball_diameter_m']/2, -side/1000)
            self.pass_check_after = time.monotonic()
            self.pass_search_index = 0
            return
        shift = kick_result(before,scene.get('kick_view')) if scene.get('ball') else None
        if shift is not None and shift <= .35:
            self.kick_check = None
            self.pending_toe_pass = (toe,side)
            self.describe('simple_kick_missed', f'ball stayed on the same marking ({shift:.2f} diameters); align again')
            return
        if shift is not None and shift >= .75:
            self.describe('simple_kick_moved', f'ball moved {shift:.2f} diameters against pitch markings')
        else:
            self.describe('simple_kick_unconfirmed', 'displacement uncertain; reacquire the ball')
            if not scene.get('confirmed'):
                return
            if scene.get('ball') and math.dist(scene['ball'],point) <= self.p['game.ball_diameter_m']*.5:
                self.kick_check = None
                self.pending_toe_pass = (toe,side)
                return
            # The current view is usable for pursuit even when registering
            # the paint across the kick was ambiguous. Keep contact unconfirmed.
        self.kick_check = None
        self.placed_ball_reference = point
        self.pass_check_after = time.monotonic()
        self.pass_search_index = 0

    async def step(self):
        if self.last_usable_frame is None:
            self.last_usable_frame = time.monotonic()
        revision = self.g.revision
        if self.restart_pivot:
            self.restart_pivot = False
            side = self.restart_roll_side
            self.describe('restart_track_turn', 'intercept close diagonal roll')
            if not await self.turn(side * math.radians(15), tolerance=4) or not self.g._same_plan(revision):
                return
            self.landing_pan = round(side * math.radians(42) /
                                     (self.g.s.params.backend.Robot.camera.pan_sign * .00058909))
            self.ball_range = min(self.ball_range, .35)

        if self.expired():
            self.cancel_special('30 second deadline')
        if self.entry_pending:
            self.entry_pending = False  # Unknown execution can never repeat entry.
            if not self.danger:
                self.remember_start_heading()
                yaw, _ = await self.g._imu()
                odom = getattr(self.g, 'odometry', None)
                if odom:
                    enemy = self.specs[1-self.p['match.own_goal']]
                    x, y = odom.body_point((enemy['x'],enemy['y']))
                    self.attack_axis = wrap(yaw+math.atan2(y,x))
            self.describe('simple_entry', 'advance into field')
            distance = self.p['game.simple_football.entry_forward_mm']/1000
            if distance:
                await self.walk(distance)
            return
        if self.danger:
            await self.danger_ball_step()
            return
        if self.kick_check is not None:
            await self.check_opening_kick(revision)
            return
        if self.pending_toe_pass is not None:
            await self.place_and_pass(revision)
            return
        if self.kickoff_pending:
            await self.placed_kickoff()
            return
        if self.placed_ball_reference is not None or self.pass_segments_left:
            if not await self.g._prepare(self.p, self.entry) or not self.g._same_plan(revision):
                return
            scene = await self.look(0, self.p['game.head_tilt'])
            if not self.g._same_plan(revision):
                return
            if not scene or not scene.get('confirmed') or not 0 <= time.monotonic()-scene.get('received', 0) <= .5:
                scene = {}
            ball = scene.get('ball')
            centre = self.placed_ball_reference
            if centre is not None:
                self.placed_ball_reference = None
                self.opening_touch_required = False
                reason = 'track observed ball after opening kick' if ball else 'ball left view after completed pass'
                self.describe('simple_open_play', reason)
                if ball:
                    await self.follow_kickoff(ball, centre, scene)
                    return
                self.pass_centre = centre
                self.pass_segments_left = 3
            await self.chase_pass_segment(scene, revision)
            return

        if self.placed_kickoff_started and self.opening_touch_required:
            # Resume visual play after interrupted placement; retain the weak first touch.
            self.placed_kickoff_started = False

        revision = self.g.revision
        odom = getattr(self.g, 'odometry', None)
        if self.empty_sectors >= 2 and odom and not self.opening_touch_required:
            if await self.g._prepare(self.p, self.entry) and self.g._same_plan(revision):
                await self.search_field(odom)
            return
        # Moving opponents require a new visual check even on kickoff.
        scan_goals = not self.opening_touch_required and self.near and self.goal_scan_needed and time.monotonic()-self.last_goal_scan >= 2.
        if self.landing_pan is not None:
            scan_goals = False
        if scan_goals:
            self.last_goal_scan = time.monotonic()
            self.goal_scan_needed = False
        self.g.s.log('game', 'INFO', 'Simple observation: '+(
            'scan goals' if scan_goals else 'ball only; defer goal scan'))
        self.g.info.pop('kick_setup', None)
        rechecked = self.check_shot_view
        self.check_shot_view = False
        if rechecked:
            self.goal_views.clear()
            ball, goals = await self.observe(goals=True, final_aim=True)
        else:
            ball, goals = await self.observe(goals=scan_goals)
        if not self.g._same_plan(revision):
            return
        if self.expired():
            self.cancel_special('30 second deadline')
            return
        if ball is not None and self.ball_refresh_after is not None:
            scene = self.last_scene or {}
            if scene.get('ball_received', scene.get('received', 0)) <= self.ball_refresh_after:
                self.describe('simple_ball_recheck', 'waiting for ball frame after motion')
                return
            self.ball_refresh_after = None
        if ball is None:
            if self.near and await self.follow_hidden_ball(self.last_scene):
                return
            if self.approach_glance_after != math.inf:
                self.approach_glance_after = None
            self.leg_candidate = None
            if not self.opening_touch_required and await self.approach_ball_region():
                return
            self.forget_goal('ball lost')
            self.describe('simple_search', 'no fresh ball')
            if self.near and self.jumps >= 15:
                self.describe('simple_new_attempt', '15 jumps; ball lost, restart search without reposition')
                self.near, self.jumps, self.leg_sign = False, 0, None
                self.fine_heading_attempts = 0
                return
            self.orbit_face_ball = False
            # Centre was just inspected. Search both sides at the ball tilt
            # before spending a body turn; inspect the last known side first.
            tilt = self.ball_tilt()
            sign = self.g.s.params.backend.Robot.camera.pan_sign
            poses = [(round(1100 * side / sign), tilt)
                     for side in (self.last_ball_side, -self.last_ball_side)]
            if not self.near:
                # The shallow view covers the goalmouth and distant rebounds.
                poses.insert(0, (0, min(0, self.p['head.field_tilt']+650)))
            if 0 <= time.monotonic()-self.last_open_shot < 6:
                poses.insert(0, (0, self.p['head.field_tilt']))
            predicted = self.expected_pan('ball')
            if predicted is not None:
                spread = getattr(self, 'ball_pan_spread', 0)
                if spread >= 500:
                    # Cover both edges of the predicted sector after its centre.
                    edges = (predicted + self.last_ball_side * spread, predicted - self.last_ball_side * spread)
                    poses[:0] = [(max(-2666, min(2666, edge)), tilt) for edge in edges]
                if abs(predicted) > 250:
                    poses.insert(0, (predicted, tilt))
            # A missed close view can mean the ball rolled out of the foot camera.
            # Inspect the same side farther out before rotating the body again.
            if self.near and self.empty_sectors and not self.opening_touch_required:
                shallow = min(0, self.p['head.field_tilt'] + 650)
                poses.extend((round(1100 * side / sign), shallow)
                             for side in (self.last_ball_side, -self.last_ball_side))
            for pan, tilt in dict.fromkeys(poses):
                # Give a predicted ball time to reappear behind an obstacle.
                # Elsewhere a confirmed empty observation suffices to move on.
                dwell = self.opening_touch_required or self.empty_sectors == 0 or pan == predicted
                scene = await self.look(pan, tilt, wait_ball=dwell)
                if not self.g._same_plan(revision) or self.expired():
                    return
                if scene and scene.get('ball'):
                    x, y = scene['ball']
                    if abs(y) > .02:
                        self.last_ball_side = 1 if y > 0 else -1
                    self.empty_sectors = self.field_sector = 0
                    self.field_sector_heading = None
                    self.log_search_found(pan, tilt, scene)
                    if not self.opening_touch_required and self.attack_axis is not None:
                        self.last_scene = scene
                        await self.attack_after_pass(scene['ball'], assemble_goals(scene.get('parts', []), self.specs))
                        return
                    self.ball_range = math.hypot(x, y)
                    if x > 0 and self.ball_range <= self.fine_radius and not self.near:
                        self.near = True
                        self.reset_leg_switch()
                        self.jumps = self.fine_heading_attempts = 0
                        self.goal_scan_needed = True
                    # A close ball belongs beside the striking foot, not on the
                    # body centreline. Turning here undoes the goal alignment.
                    if not self.near or x <= 0:
                        await self.turn(math.atan2(y, x), alignment=self.near)
                    return
            if self.opening_touch_required and await self.approach_ball_region():
                return
            odom = getattr(self.g, 'odometry', None)
            if ((odom is None or odom.confidence == 'degraded')
                    and time.monotonic() - self.last_usable_frame > 12):
                # Turning the body cannot reveal a ball while the entire image
                # is missing. Continue requesting views from this search sector.
                self.describe('simple_search_no_image', 'no camera or reliable field pose; reacquire along goal bearing')
                if not self.blind_heading_checked:
                    self.blind_heading_checked = True
                    heading = self.remembered_goal_heading()
                    if heading is not None:
                        yaw, _ = await self.g._imu()
                        if self.g._same_plan(revision):
                            await self.turn(wrap(heading-yaw))
                await self.g._settle(.20)
                return
            self.empty_sectors += 1
            if self.opening_touch_required and self.empty_sectors == 1:
                self.inspect_feet_first = True
                return
            if self.near and self.empty_sectors == 1 and not self.opening_touch_required:
                self.inspect_feet_first = True
                await self.jump('backward', 1.)
                return
            odom = getattr(self.g, 'odometry', None)
            if self.empty_sectors >= 2 and not self.opening_touch_required:
                await self.search_field(odom)
                return
            if (not self.near and odom and odom.confidence != 'degraded'
                    and self.empty_sectors % 3 != 0 and self.attack_axis is not None):
                # Change the sightline when an opponent screens the distant ball.
                # Approach a flank observation point, staying inside the field.
                enemy = self.specs[1-self.p['match.own_goal']]
                direction = 1 if enemy['x'] > 0 else -1
                side = 1 if self.p['game.simple_football.kickoff_side'] == 'left' else -1
                if (self.empty_sectors-1)//6 % 2:
                    side = -side
                gx, gy = odom.body_point((enemy['x']-.55*direction,
                                         side*self.p['field.geometry']['width']*.24))
                if math.hypot(gx, gy) > .18:
                    course = math.atan2(gy, gx)
                    if abs(course) > math.radians(60):
                        await self.turn(course)
                    else:
                        self.describe('simple_search_walk', 'change viewpoint and rescan goal flank')
                        await self.walk(min(.32, math.hypot(gx, gy)), course)
                    return
            self.g.s.log('game', 'INFO', 'Ball head search: no ball in scanned poses; turn body to next sector')
            await self.search_sector()
            return

        self.empty_sectors = self.field_sector = 0
        self.field_sector_heading = None
        if (self.placed_kickoff_started or self.attack_axis is not None) and not self.opening_touch_required:
            await self.attack_after_pass(ball, goals, rechecked=rechecked)
            return

        x, y = ball
        if self.soft_touch_before is not None:
            old_x, old_y = self.soft_touch_before
            self.soft_touch_before = None
            if math.hypot(x-old_x, y-old_y) >= .035:
                self.opening_touch_required = False
                self.near, self.jumps, self.leg_sign = False, 0, None
                self.goal_scan_needed = True
                self.describe('simple_open_play', 'opening pass moved the ball; attack')
                return
            if self.soft_touch_attempts >= 2:
                self.opening_touch_required = False
                self.g.info['opening_touch_failed'] = True
                self.goal_scan_needed = True
                self.describe('simple_open_play', 'two soft kicks did not move ball; attack normally')
                return
            self.describe('simple_kickoff_retry', 'ball did not move; re-align and retry soft kick')
        if abs(y) > .02:
            self.last_ball_side = 1 if y > 0 else -1
        distance, course = math.hypot(x, y), math.atan2(y, x)
        self.ball_range = distance
        if not self.near:
            self.aligned_radius = self.displaced_frame = None
        # Rotation alone preserves range. Recheck a large outward change from
        # the striking foot while stationary before reusing any shot geometry.
        if (self.aligned_radius is not None and
                distance - self.aligned_radius > self.p['game.ball_diameter_m']):
            frame = (self.last_scene or {}).get('ball_sequence')
            self.goal_scan_needed = False
            if frame is None or self.displaced_frame is None or frame == self.displaced_frame:
                self.displaced_frame = frame
                self.describe('simple_ball_recheck', 'range increased; verify ball before moving')
                return
            self.forget_goal('ball displaced from foot')
            self.goal_views.clear()
            self.shot_deadline = self.aligned_radius = self.displaced_frame = None
            self.near, self.jumps, self.leg_sign = False, 0, None
            self.fine_heading_attempts = 0
            self.reset_leg_switch()
            self.describe('simple_new_attempt', 'ball displaced from foot; reacquire at new position')
            return
        self.displaced_frame = None
        if distance <= self.fine_radius:
            self.approach_glance_after = None
        observation = self.last_scene or {}
        foot = observation.get('foot')
        if not (foot and not self.danger and distance < .24 and
                0 <= time.monotonic()-observation.get('ball_received', observation.get('received', 0)) < .5):
            foot = None
        visual_ready = bool(foot and foot['aligned'] and -.05 <= foot['gap_ratio'] <= .40)
        if visual_ready:
            self.near = True
        if self.near and distance > self.fine_radius and not visual_ready:
            self.describe('simple_new_attempt', 'ball left foot alignment zone; finish alignment attempt')
            self.near, self.jumps, self.leg_sign = False, 0, None
            self.fine_heading_attempts = 0
            return
        if not self.near:
            self.orbit_face_ball = False
            self.leg_sign = None
            if distance <= self.fine_radius:
                self.near = True
                self.reset_leg_switch()
                self.jumps = self.fine_heading_attempts = 0
        enemy = next((g for g in goals if g['colour'] == self.enemy), None)
        own = next((g for g in goals if g['colour'] == self.own), None)
        margin = self.p['game.ball_diameter_m']/2+.05
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        kickoff = self.deadline is not None
        opening = (enemy or {}).get('opening_source', {})
        observed = opening.get('observed_at', -math.inf)
        aimed = bool(enemy and enemy.get('shootable') and 'occupied' in opening
                     and 0 <= time.monotonic()-observed < 1.5)
        lane = corner_lane(ball, enemy, margin) if aimed else None
        if self.danger:
            error, corridor = 0., True  # Explicit reckless role: no goal-direction veto.
        elif enemy:
            centre = [(a+b)/2 for a, b in zip(*(lane or enemy['ends']))]
            error = math.atan2(centre[1]-y, centre[0]-x)
            corridor = bool(lane and shot_clear(ball, enemy | {'ends': lane}, 0.))
            if lane and not visual_ready:
                self.shot_deadline = min(self.shot_deadline or observed, observed+1.5)
        else:
            error, corridor = 0., False
        own_blocks = own_goal_blocks(ball, own, self.p['game.ball_diameter_m']/2)
        own_veto = bool(own_blocks)
        if own_veto:
            self.g.s.log('game', 'INFO', 'Own goal veto: '+json.dumps(dict(
                ball_frame=(self.last_scene or {}).get('ball_sequence'), ball_m=ball,
                blocks=own_blocks), separators=(',', ':')))
        if own_veto or self.own_veto_reported:
            await self.g._call('detection', 'football.decision', {'blocks': own_blocks})
            if not self.g._same_plan(revision):
                return
            self.own_veto_reported = own_veto
        # The camera may see the ball but not both toe tips. With a measured
        # right toe use the projected near edge and the tested 4 cm reach.
        stamp = observation.get('ball_received', observation.get('received', 0))
        fresh = observation.get('confirmed', False) and 0 <= time.monotonic()-stamp < .5
        toe = self.p.get('game.soft_kick.right.toe_x_mm', 0)
        right_gap = x*1000 - self.p['game.ball_diameter_m']*500 - toe
        right_side = y*1000 + self.p['game.kick_offset_mm'] + self.p['game.soft_kick.right.outward_trim_mm']
        calibrated_ready = (not self.opening_touch_required and not self.danger and not foot and fresh and toe > 0 and
                            0 <= right_gap-self.p['game.soft_kick.right.forward_trim_mm'] <= 40
                            and abs(right_side) <= 20)
        approximate = bool(visual_ready and enemy and enemy.get('shootable') and
                           0 <= time.monotonic()-observed < 3 and opening.get('occupied') == []
                           and abs(error) <= math.radians(20) and not own_veto)
        if (fresh and (visual_ready or calibrated_ready) and not self.opening_touch_required
                and not self.danger and not own_veto and not opening.get('occupied')):
            if not enemy:
                imu = self.gaze_imu()
                target = self.expected_gaze.targets.get('enemy_goal') or self.expected_gaze.targets.get('start_goal')
                if imu and target and imu[1] == target[1] and 0 <= time.monotonic()-target[2] < 15:
                    error = wrap(target[0]-imu[0])
                    approximate = abs(error) <= math.radians(20)
            elif enemy.get('shootable') and 0 <= time.monotonic()-observed < 3:
                approximate = abs(error) <= math.radians(20)
            if corridor or approximate:
                self.near = True
                if calibrated_ready:
                    self.leg_sign = -1
        if approximate:
            corridor = True
            self.shot_deadline = stamp+.5
        if own_veto:
            corridor = False
        self.remembered_shot = False
        if self.leg_sign is None:
            self.leg_sign = -1 if y < 0 else 1
        if self.near:
            self.consider_leg_switch(y*1000, (self.last_scene or {}).get('ball_sequence'))
        front = x*1000-self.p['game.kick_distance_mm']
        side = y*1000-self.leg_sign*self.p['game.kick_offset_mm']
        blockers = []
        if own_veto: blockers.append('own_goal')
        if not self.danger and enemy is None and not approximate: blockers.append('no_opponent_goal')
        elif not self.danger and enemy and not enemy.get('shootable', False): blockers.append('goal_colour_only')
        elif not corridor and not own_veto: blockers.append('heading' if kickoff else 'outside_goal_opening')
        if x <= 0: blockers.append('ball_behind')
        if front > 10 and not (visual_ready or calibrated_ready): blockers.append('ball_far_forward')
        if abs(side) > 20: blockers.append('foot_alignment')
        if distance >= .2: blockers.append('ball_range')
        age = ((time.monotonic()-self.last_scene['received'])*1000
               if self.last_scene else None)
        self.g.s.log('game', 'INFO',
                    f"Simple ball: x={x*1000:.0f} y={y*1000:.0f} d={distance*1000:.0f}mm "
                    f"age_ms={round(age) if age is not None else 'unknown'} "
                    f"leg={'left' if self.leg_sign > 0 else 'right'} front={front:.0f} side={side:.0f}mm")
        self.g.s.log('game', 'INFO',
                    f"Simple shot: enemy={enemy['colour'] if enemy else 'none'} "
                    f"own={own['colour'] if own else 'none'} yaw_error={math.degrees(error):.1f}deg "
                    f"blocked={','.join(blockers) or 'none'}")

        if self.opening_touch_required or (not self.near and distance <= .35):
            if not self.last_scene or not 0 <= time.monotonic()-self.last_scene.get('ball_received', self.last_scene['received']) <= .5:
                return
            if self.kickoff_heading is None and self.opening_touch_required:
                self.kickoff_heading = yaw
            if self.opening_touch_required:
                pass_course = self.opening_pass_course(yaw)
                if self.leg_sign is None:
                    self.leg_sign = 1 if (foot['leg'] if foot else
                                           ('left' if y >= 0 else 'right')) == 'left' else -1
                leg = 'left' if self.leg_sign > 0 else 'right'
                support = 'right' if leg == 'left' else 'left'
                target_x = self.p['game.kick_distance_mm'] + self.p[f'game.soft_kick.{leg}.forward_trim_mm']
                toe_x = self.p['game.soft_kick.right.toe_x_mm'] if leg == 'right' else 0.
                toe_gap = None
                if toe_x > 0:
                    near_edge = x*1000 - self.p['game.ball_diameter_m']*500
                    toe_gap = near_edge - toe_x - self.p['game.soft_kick.right.forward_trim_mm']
                    target_x = x*1000 - toe_gap + 20.
                lateral = self.p['game.kick_offset_mm'] + self.p[f'game.soft_kick.{leg}.outward_trim_mm']
                target_y = lateral if leg == 'left' else -lateral
                dx, dy = x - target_x/1000, y - target_y/1000
                # The measured toe gap is useful for placing the last step.
                # At contact, a partly occluded ball has an imprecise range.
                if foot:
                    in_reach = x >= .08 and foot['leg'] == leg and visual_ready
                elif toe_gap is not None:
                    in_reach = (x >= .08 and round(toe_gap, 6) <= 30.
                                and abs(dy) <= .45*self.p['game.ball_diameter_m'])
                else:
                    in_reach = x > 0 and abs(dx) <= .025 and abs(dy) <= .020
                setup = dict(leg=leg, support=support, x_mm=round(x*1000), y_mm=round(y*1000),
                             front_error_mm=round(dx*1000), side_error_mm=round(dy*1000),
                             toe_gap_mm=None if toe_gap is None else round(toe_gap, 1),
                             in_reach=in_reach, frame=self.last_scene.get('ball_sequence'))
                if foot:
                    setup['visual_gap_ratio'] = foot['gap_ratio']
                self.g.info['kick_setup'] = setup
                if not self.kickoff_heading_done and distance > .20 and abs(pass_course) > math.radians(8):
                    self.describe('simple_pass_heading', 'face diagonal before placing foot')
                    self.turn_ball = (course, distance, yaw)
                    self.kickoff_heading_done = await self.turn(
                        max(-math.radians(15), min(math.radians(15), pass_course)), tolerance=0)
                    return
                if setup['in_reach']:
                    if own_veto or any(shot_clear(ball, goal, 0., 0.) for goal in goals):
                        self.describe('simple_kickoff_wait', 'visible goal in pass lane')
                        return
                    if self.approach_crouched:
                        self.describe('simple_kick_prepare', 'stand and refresh ball before soft kick')
                        if await self.g._move('motion.pose', {'name': 'base_stand'}):
                            self.approach_crouched = False
                            self.last_scene = None
                        return
                    self.describe('simple_kickoff_pass', 'soft diagonal kick')
                    completed = await self.strike(dict(leg=leg,
                        power=38, offset=max(-60, min(80, round(abs(y)*1000)-62)), soft=True))
                    if completed:
                        self.soft_touch_before = (x, y)
                        self.soft_touch_attempts += 1
                        self.describe('simple_kickoff_verify', 'reobserve ball to confirm soft touch')
                    return
            else:
                dx = x-self.p['game.kick_distance_mm']/1000
                dy = y-self.leg_sign*self.p['game.kick_offset_mm']/1000
            clearance = (target_x/1000 if self.opening_touch_required else
                         self.p['game.kick_distance_mm']/1000 + .06)
            if not self.opening_touch_required and foot and not visual_ready and x <= clearance + .01:
                if abs(dy) > .02:
                    await self.short_foot_jump(0., dy)
                else:
                    await self.short_foot_jump(*self.visual_adjustment(foot))
                return
            if distance <= .45 or self.opening_touch_required:
                # Keep a margin between the advancing toe and the ball. The
                # next observation will decide whether a shorter step is needed.
                if self.opening_touch_required and self.opening_walk_sent:
                    self.describe('simple_kickoff_recheck', 'recheck striking foot after approach')
                    return
                if self.opening_touch_required and x < .08:
                    self.describe('simple_ball_too_close', 'ball behind striking toe; do not kick blindly')
                    return
                if self.opening_touch_required and (
                        x > clearance + .005 or abs(dy) > .02 or foot and not visual_ready):
                    self.opening_walk_sent = True
                    target_x = max(.08, clearance)
                    self.forget_goal('live ball approach')
                    self.describe('simple_walk_to_ball', f'tracking ball during approach; distance={distance*1000:.0f}mm')
                    await self.move('game.walk', dict(kind='approach', distance_m=distance,
                        heading=0., yaw_offset=wrap(-yaw), x_m=x, y_m=y,
                        stamp=self.last_scene.get('ball_received', self.last_scene['received']),
                        stop_x_m=target_x, target_y_m=target_y/1000, keep_crouch=True))
                    return
                travel = min(.16, math.hypot(dx, dy), max(0., x-clearance))
                if travel <= .005:
                    self.near = True
                    return
                self.describe('simple_walk_to_ball', f'walking to ball; distance={distance*1000:.0f}mm')
                await self.walk(travel, math.atan2(dy, dx))
                return
        if not self.near:
            if abs(course) > math.radians(15):
                await self.turn(course)
                return
            # Same finite, live-feedback approach used by Ball approach and kick.
            # No distance-minus-450mm command and no centimetre omni walks.
            if not self.last_scene:
                return
            if self.shot_deadline is None:
                return
            stamp = min(self.last_scene['received'], self.shot_deadline-.5)
            if not 0 <= time.monotonic()-stamp <= .5:
                return
            self.forget_goal('streaming approach')
            self.describe('simple_test_approach', f'original streaming approach; distance={distance*1000:.0f}mm')
            completed = await self.move('game.walk', dict(kind='approach', distance_m=distance,
                heading=course, yaw_offset=wrap(-yaw), x_m=x, y_m=y, stamp=stamp))
            if completed and self.approach_glance_after is None:
                self.approach_glance_after = time.monotonic()+4
            return

        ready, direction, fraction, front, side = kick_alignment(
            self.p, x*1000, y*1000, self.leg_sign*self.p['game.kick_offset_mm'])
        ready = (ready or (calibrated_ready and self.leg_sign == -1)) and x > 0
        if foot:
            ready = visual_ready
            self.leg_sign = 1 if foot['leg'] == 'left' else -1
        strike_leg = 'left' if self.leg_sign > 0 else 'right'
        if ready:
            self.aligned_radius = distance if self.aligned_radius is None else min(self.aligned_radius, distance)
        if corridor and ready:
            if self.expired() or not self.g._same_plan(revision):
                return
            if self.approach_crouched:
                self.describe('simple_kick_prepare', 'refresh ball after leaving approach crouch')
                self.approach_crouched = False
                if not await self.move('motion.pose', {'name': 'base_stand'}):
                    self.approach_crouched = True
                return
            power = 60 if kickoff else 100
            self.describe('simple_kick', f'foot={strike_leg} visual={visual_ready} approximate_goal={approximate} power={power}')
            completed = await self.strike(dict(leg=strike_leg, power=power,
                offset=max(-60, min(80, int(abs(y)*1000)-62))))
            if completed:
                self.near, self.jumps, self.leg_sign = False, 0, None
                self.fine_heading_attempts = 0
                await self.g._settle(1.)
            return
        if foot and not visual_ready:
            if abs(side) > 20:
                await self.short_foot_jump(0., side/1000)
            else:
                await self.short_foot_jump(*self.visual_adjustment(foot))
            return
        if self.jumps >= 15:
            # End this attempt without the old backward/orbit manoeuvre.
            self.describe('simple_new_attempt', '15 jumps; reobserve without blind strike')
            self.near, self.jumps, self.leg_sign = False, 0, None
            self.fine_heading_attempts = 0
            self.orbit_face_ball = False
            return
        self.orbit_face_ball = False
        if ready:
            # First establish an actual free lane. Blind sideways orbiting made
            # the foot alignment worse and then turned the body away again.
            if (not own_veto and lane and 0 < abs(error) <= math.pi/2
                    and self.fine_heading_attempts < 6):
                self.fine_heading_attempts += 1
                self.describe('simple_fine_heading', f'confirmed direction; attempt={self.fine_heading_attempts}; total budget=15 jumps')
                await self.turn(max(-math.pi/18, min(math.pi/18, error)), alignment=True, tolerance=0)
            else:
                self.goal_scan_needed = True
                self.describe('simple_goal_search', 'hold foot alignment; '+','.join(blockers))
            return
        if x <= 0:
            self.describe('simple_search', 'ball behind; reobserve, no blind forward jump')
            return
        self.describe('simple_test_fine', f'jump {self.jumps+1}/15 {direction} front={front:.0f} side={side:.0f}mm')
        await self.jump(direction, fraction, alignment=True)

    async def reposition(self, reason):
        self.describe('simple_reposition', reason)
        if self.deadline is not None:
            self.cancel_special(reason)
        if await self.jump('backward'):
            self.jumps = self.stagnant = 0
            self.leg_sign = self.last_score = self.score_kind = self.last_turn_error = None

    async def danger_ball_step(self):
        """Ball approach/kick test repeated as a game, with a 15-jump budget."""
        revision = self.g.revision
        observed = await self.g._original_observe(self.p, self.entry, settle=.5)
        if not self.g._same_plan(revision):
            return
        ball = self.g._usable(observed)
        if ball is None:
            self.approach_glance_after = None
            self.leg_candidate = None
            if self.jumps >= 15:
                self.near, self.leg_sign, self.jumps = False, None, 0
                self.danger_approaches = 0
            self.describe('danger_search', 'no fresh ball; retain fine alignment budget')
            for pan in (0, -1778, 1778):
                scene = await self.look(pan, self.p['game.head_tilt'])
                if not self.g._same_plan(revision):
                    return
                if scene and scene.get('ball'):
                    self.log_search_found(pan, self.p['game.head_tilt'], scene)
                    await self.turn(math.atan2(scene['ball'][1], scene['ball'][0]))
                    return
            self.empty_sectors += 1
            if self.near and self.empty_sectors == 1 and not self.opening_touch_required:
                self.inspect_feet_first = True
                await self.jump('backward', 1.)
                return
            self.g.s.log('game', 'INFO', 'Ball head search: no ball in scanned poses; turn body to next sector')
            await self.search_sector()
            return
        x, y = ball['x_m']*1000, ball['y_m']*1000
        distance = math.hypot(x, y)/1000
        self.g.s.log('game', 'INFO', f'Danger ball test: x={x:.0f} y={y:.0f}mm jumps={self.jumps}/15')
        if not self.near:
            self.aligned_radius = self.displaced_frame = None
        # Rotation alone preserves range. Recheck a large outward change from
        # the striking foot while stationary before reusing any shot geometry.
        if (self.aligned_radius is not None and
                distance - self.aligned_radius > self.p['game.ball_diameter_m']):
            frame = (self.last_scene or {}).get('ball_sequence')
            self.goal_scan_needed = False
            if frame is None or self.displaced_frame is None or frame == self.displaced_frame:
                self.displaced_frame = frame
                self.describe('simple_ball_recheck', 'range increased; verify ball before moving')
                return
            self.forget_goal('ball displaced from foot')
            self.goal_views.clear()
            self.shot_deadline = self.aligned_radius = self.displaced_frame = None
            self.near, self.jumps, self.leg_sign = False, 0, None
            self.fine_heading_attempts = 0
            self.reset_leg_switch()
            self.describe('simple_new_attempt', 'ball displaced from foot; reacquire at new position')
            return
        self.displaced_frame = None
        if distance > .35 and self.approach_glance_after is None:
            self.approach_glance_after = time.monotonic()+4
        if distance <= .35:
            self.approach_glance_after = None
        if self.near and distance > .35:
            self.describe('danger_new_attempt', 'ball left 350mm zone; finish this alignment attempt')
            self.near, self.leg_sign, self.jumps = False, None, 0
            self.danger_approaches = 0
            return
        if not self.near and distance > .35:
            course = math.atan2(y, x)
            if abs(course) > math.radians(15):
                await self.turn(course)
                return  # Fresh observation after facing the ball, just like the test.
            if self.danger_approaches >= 3:
                self.describe('danger_new_attempt', 'three approaches completed; reobserve for a new attempt')
                self.danger_approaches = 0
                return
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
            stamp = time.monotonic()-(self.g._capture_now()-ball['sensor_timestamp_ns'])/1e9
            if not 0 <= time.monotonic()-stamp <= .5:
                return
            self.danger_approaches += 1
            self.describe('danger_test_approach', f'original streaming approach; distance={distance*1000:.0f}mm')
            await self.move('game.walk', dict(kind='approach', distance_m=distance,
                heading=course, yaw_offset=wrap(-yaw), x_m=ball['x_m'], y_m=ball['y_m'], stamp=stamp))
            return
        if not self.near:
            self.near = True
            self.reset_leg_switch()
            self.leg_sign = -1 if y < 0 else 1
            self.jumps = 0
            self.describe('danger_test_fine', f"leg={'left' if self.leg_sign > 0 else 'right'}; no more walking")
        self.consider_leg_switch(y, ball.get('sensor_timestamp_ns'))
        ready, direction, fraction, front, side = kick_alignment(self.p, x, y, self.leg_sign*self.p['game.kick_offset_mm'])
        if ready:
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
            if self.kick_direction != 'straight':
                kickoff = self.kick_direction == 'kickoff_left'
                angle = 60 if kickoff else 30
                attempts = 4 if kickoff else 2
                if self.angle_target is None:
                    self.angle_target = wrap(yaw+math.radians(angle)*(1 if self.kick_direction in ('left', 'kickoff_left') else -1))
                error = wrap(self.angle_target-yaw)
                if abs(error) > math.radians(10) and self.angle_attempts < attempts and self.jumps < 15:
                    self.angle_attempts += 1
                    self.describe('danger_angled_kick', f'{self.kick_direction} {angle}deg; remaining={math.degrees(error):.1f}deg attempt={self.angle_attempts}/{attempts}')
                    await self.turn(error, alignment=True, tolerance=10)
                    return  # Turn moves the ball relative to the feet: always reobserve/re-align.
            stamp = time.monotonic()-(self.g._capture_now()-ball['sensor_timestamp_ns'])/1e9
            self.shot_deadline = stamp+.5
            power = 60 if self.kick_direction == 'kickoff_left' else 100
            self.describe('danger_test_kick', f'power={power} front={front:.0f} side={side:.0f}mm')
            completed = await self.move('motion.kick', dict(leg='right' if y < 0 else 'left', power=power,
                offset=max(-60, min(80, int(abs(y))-62))))
            if completed:
                if self.kick_direction == 'kickoff_left':
                    self.kick_direction = 'straight'
                    self.g.info['kick_direction'] = 'straight'
                self.angle_target = None
                self.angle_attempts = 0
                self.near, self.leg_sign, self.jumps = False, None, 0
                self.danger_approaches = 0
                await self.g._settle(1.)
            return
        if self.jumps >= 15:
            self.describe('danger_new_attempt', '15 alignment jumps; no blind strike, reobserve')
            self.near, self.leg_sign, self.jumps = False, None, 0
            self.danger_approaches = 0
            return
        self.describe('danger_test_fine', f'jump {self.jumps+1}/15 {direction} fraction={fraction:.2f} front={front:.0f} side={side:.0f}mm')
        await self.jump(direction, fraction, alignment=True)
