"""Visual goalkeeper with lateral positioning in front of its own goal."""
import math
import time

from roki_ng.body.calibration import wrap
from roki_ng.supervisor.game.simple_football import SimpleFootball


class SimpleGoalkeeper(SimpleFootball):
    goal_bearing_only = True

    def __init__(self, game, parameters, entry, delay):
        super().__init__(game, parameters, entry, delay)
        self.kickoff_pending = self.entry_pending = False
        self.heading = None
        self.reacquire_heading = False
        self.goal_at = self.scan_at = 0.
        self.goal_offset = None
        self.live_sequence = -1
        self.live_ball = None
        self.live_at = 0.
        self.ball_speed = 0.
        self.move_ball = None
        self.motion_changed = False
        self.track_deadline = 0.
        self.search_view = 0
        self.last_close_pan = None
        self.ball_side = -1
        own = self.specs[self.p['match.own_goal']]
        enemy = self.specs[1 - self.p['match.own_goal']]
        self.inward = 1 if enemy['x'] > own['x'] else -1
        fronts = [m for key, m in self.p.items() if key.startswith('field.mark.')
                  and m['enabled'] and m['kind'] == 'line'
                  and abs(math.cos(m['angle'])) < .12 and abs(m['y'] - own['y']) < .07
                  and m['size'] >= own['width'] * .9
                  and .08 < self.inward * (m['x'] - own['x']) < .8]
        front = min(fronts, key=lambda m: abs(m['x'] - own['x'])) if fronts else None
        self.area_depth = abs(front['x'] - own['x']) if front else .30
        self.area_half_width = front['size'] / 2 if front else own['width'] / 2
        self.area_y = front['y'] if front else own['y']


    async def find_near_ball(self, revision):
        # y is positive to body-left. The servo sign belongs to the camera mount.
        sign = self.g.s.params.backend.Robot.camera.pan_sign
        poses = ((self.ball_side*1500, -1900), (-self.ball_side*1500, -1900),
                 (0, -2100), (self.ball_side*1950, -950), (-self.ball_side*1950, -950))
        pan, tilt = poses[self.search_view % len(poses)]
        self.search_view += 1
        self.describe('keeper_ball_search', 'look along goal line and beside feet')
        scene = await self.look(round(pan/sign), tilt, wait_ball=True)
        return scene if self.g._same_plan(revision) else None

    async def clear_at_foot(self, scene, ball, yaw):
        if not scene.get('confirmed') or time.monotonic()-scene['received'] > .5:
            return False
        x, y = ball
        leg = 'right' if y < 0 else 'left'
        toe = self.p.get(f'game.soft_kick.{leg}.toe_x_mm', 0.)/1000
        # The simulated feet are mirror images and share the measured toe length.
        if leg == 'left' and not toe:
            toe = self.p.get('game.soft_kick.right.toe_x_mm', 0.)/1000
        gap = x-self.p['game.ball_diameter_m']/2-toe
        offset = self.p['game.kick_offset_mm']/1000
        if toe <= 0 or not -.01 <= gap <= .04 or abs(abs(y)-offset) > .027:
            return False
        # The keeper restores its starting heading before calling this method.
        odom = getattr(self.g, 'odometry', None)
        if odom and odom.confidence != 'degraded':
            enemy = self.specs[1-self.p['match.own_goal']]
            gx, gy = odom.body_point((enemy['x'], enemy['y']))
            if abs(math.atan2(gy-y, gx-x)) > math.radians(30):
                return False
        elif abs(wrap(self.heading-yaw)) > math.radians(12):
            return False
        self.describe('keeper_clear', f'clear toward opponent end with {leg} foot')
        await self.move('motion.kick', dict(leg=leg, power=80, offset=0))
        return True

    async def block_incoming(self, scene, revision):
        """Predict a crossing at the keeper's feet from two stationary camera frames."""
        if not scene or not scene.get('confirmed') or not scene.get('ball'):
            return False
        x, y = scene['ball']
        if not (.12 < x < 2. and abs(y) < .8):
            return False
        # Sample at camera cadence so the tracker can associate a fast ball
        # between adjacent frames. Use a longer baseline for the velocity fit.
        for _ in range(5):
            if not await self.g._settle(.02) or not self.g._same_plan(revision):
                return False
            latest = await self.g._call('detection', 'football.status')
            if (not self.g._same_plan(revision) or latest.get('received') is None
                    or latest.get('pose_id') != scene.get('pose_id')):
                return False
            dt = latest['received'] - scene['received']
            if dt >= .055:
                break
        if (not latest.get('confirmed') or not latest.get('ball') or not .055 <= dt <= .35
                or not 0 <= time.monotonic()-latest['received'] <= .25):
            return False
        nx, ny = latest['ball']
        closing = (x-nx)/dt
        if closing < .25 or nx <= 0:
            return False
        arrival = nx/closing
        crossing_y = ny + (ny-y)*arrival/dt
        if not (.12 <= arrival <= 2.0 and abs(crossing_y) <= .24):
            return False
        self.describe('keeper_split', f'incoming ball in {arrival:.2f}s at {crossing_y:+.2f}m')
        if await self.move('motion.splits', dict(kind='small', crouch='off')) and self.g._same_plan(revision):
            remaining = latest['received'] + arrival + .1 - time.monotonic()
            if remaining > 0:
                await self.g._settle(remaining)
            if self.g._same_plan(revision):
                await self.move('motion.pose', dict(name='base_stand'))
        return True

    def motion_fresh(self, op, args):
        if op == 'game.jump' and args.get('direction') in ('left', 'right'):
            if time.monotonic() >= self.track_deadline:
                self.describe('keeper_hold', 'ball expired before lateral jump; reobserve')
                return False
        dx = dy = 0.
        reach_y = .11  # Both feet plus lateral swing in normal stance.
        if op == 'game.walk':
            distance, course = args['distance_m'], args['course']
            dx, dy = distance * math.cos(course), distance * math.sin(course)
        elif op == 'game.jump' and args['direction'] in ('left', 'right'):
            side = args['direction']
            dy = self.p[f'motion.jump_{side}_mm'] * args['fraction'] / 1000
            dy *= 1 if side == 'left' else -1
        elif op == 'motion.kick':
            dx = .08  # Forward excursion in addition to the standing foot envelope.
        elif op == 'motion.splits':
            reach_y = .26
        else:
            return True
        odom = getattr(self.g, 'odometry', None)
        if odom is None or odom.confidence == 'degraded':
            self.describe('keeper_boundary', 'position uncertain; watch ball from here')
            return False
        own = self.specs[self.p['match.own_goal']]
        x, y = odom.field_point((dx, dy))
        depth = self.inward * (x - own['x'])
        # Side-stepping also accumulates fore/aft slip. Reserve space for the
        # feet and for travel since the last independent position correction.
        slip = min(.08, .12 * max(0., odom.path - odom.anchor_path))
        if (.045 <= depth <= self.area_depth - .075 - slip and
                abs(y - self.area_y) <= self.area_half_width - reach_y - slip):
            return True
        self.describe('keeper_boundary', 'projected movement reaches goal-area marking')
        return False

    def cancel_special(self, reason):
        super().cancel_special(reason)
        self.heading = None
        self.reacquire_heading = True
        self.goal_at = self.scan_at = 0.
        self.goal_offset = self.live_ball = None

    def observe_motion_ball(self, observation):
        """Consume detector updates while the finite jump runs; never chain jumps."""
        result = observation.get('result') or {}
        age, seq = observation.get('age_ms'), result.get('frame_sequence')
        valid = (observation.get('running') and not observation.get('error') and result.get('valid')
                 and type(age) in (int, float) and 0 <= age <= 500
                 and type(seq) is int and seq > self.live_sequence
                 and all(type(result.get(k)) in (int, float) and math.isfinite(result[k])
                         for k in ('x_m', 'y_m')))
        if valid:
            point = (result['x_m'], result['y_m'])
            stamp = time.monotonic()-age/1000
            if self.live_ball is not None and stamp > self.live_at:
                self.ball_speed = math.dist(point, self.live_ball)/(stamp-self.live_at)
            self.live_ball, self.live_at, self.live_sequence = point, stamp, seq
            changed = self.move_ball is not None and abs(point[1]-self.move_ball[1]) > .10
        else:
            changed = time.monotonic()-self.live_at > .5
        if changed and self.move_ball is not None and not self.motion_changed:
            self.motion_changed = True
            self.g.s.log('game', 'INFO', 'Simple keeper: ball shifted/lost during jump; finish this jump, reobserve before next')

    async def step(self):
        revision = self.g.revision
        if not await self.g._prepare(self.p, self.entry) or not self.g._same_plan(revision):
            return
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        if self.heading is None:
            self.heading = yaw  # Operator places keeper facing the opponent goal.
        observed = await self.g._call('detection', 'ball.status')
        if not self.g._same_plan(revision):
            return
        self.observe_motion_ball(observed)
        # Look down first: a close or moving ball has priority over a goal scan.
        if self.p.get('sim.keeper_intercept') and self.last_close_pan is not None:
            scene = await self.look(self.last_close_pan, -1850, wait_ball=True)
            self.last_close_pan = None
        else:
            scene = await self.look(0, self.p['game.head_tilt'])
        if not self.g._same_plan(revision):
            return
        ball = scene.get('ball') if scene else None
        if self.p.get('sim.keeper_intercept'):
            if ball is None:
                scene = await self.find_near_ball(revision)
                if not self.g._same_plan(revision):
                    return
                ball = (scene or {}).get('ball')
            if ball:
                self.ball_side = 1 if ball[1] > 0 else -1
                self.search_view = 0
                if math.hypot(*ball) < .48 and abs(ball[1]) > .07:
                    bearing = math.atan2(ball[1], ball[0])
                    tick = .00058909*self.g.s.params.backend.Robot.camera.pan_sign
                    self.last_close_pan = max(-2450, min(2450, round(bearing/tick)))
            if await self.block_incoming(scene, revision) or not self.g._same_plan(revision):
                return
        urgent = ball is not None and (math.hypot(*ball) < .7 or self.ball_speed > .15)
        if time.monotonic()-self.scan_at >= 4 and (not urgent or self.reacquire_heading):
            self.scan_at = time.monotonic()
            self.describe('keeper_goal_scan', 'head up; colour bearing only')
            goal_scene = await self.look(0, 0)
            if not self.g._same_plan(revision):
                return
            hints = [p for p in (goal_scene or {}).get('parts', [])
                     if p['colour'] == self.enemy and p['kind'] == 'colour']
            if hints:
                hint = max(hints, key=lambda p: p['area'])
                bearing = math.atan2(hint['points'][0][1], hint['points'][0][0])
                yaw, _ = await self.g._imu()
                if not self.g._same_plan(revision):
                    return
                if self.reacquire_heading:
                    self.heading = wrap(yaw+bearing)
                    self.reacquire_heading = False
                # Compass bearing relative to the initial facing direction.
                # A single post/colour patch is not a metric goal centre.
                self.goal_offset = wrap(yaw+bearing-self.heading)
                self.goal_at = time.monotonic()
                self.g.s.log('game', 'INFO', f'Simple keeper: enemy colour bearing={math.degrees(self.goal_offset):.1f}deg')
            scene = await self.look(0, self.p['game.head_tilt'])
            if not self.g._same_plan(revision):
                return
            ball = scene.get('ball') if scene else None  # Never reuse pre-scan ball.
        self.last_scene = scene
        if self.reacquire_heading:
            self.describe('keeper_hold', 'recovery: need opponent colour to restore heading')
            return
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        error = wrap(self.heading-yaw)
        if abs(error) > math.radians(15):
            self.describe('keeper_heading', 'restore starting course by IMU')
            await self.turn(max(-.35, min(.35, error)))
            return
        if self.p.get('sim.keeper_intercept'):
            if ball and await self.clear_at_foot(scene, ball, yaw):
                return
            odom = getattr(self.g, 'odometry', None)
            if odom and odom.confidence != 'degraded':
                home_x, home_y = odom.body_point((odom.home_x, odom.home_y))
                own = self.specs[self.p['match.own_goal']]
                depth = (odom.x-own['x'])*math.cos(odom.home_heading)
                fresh = ball is not None and scene.get('confirmed') and time.monotonic()-scene['received'] < .5
                if depth > .21 and (not fresh or ball[0] < 0 or math.hypot(*ball) > .90):
                    self.describe('keeper_return', 'recover goal position')
                    await self.walk(min(.16, math.hypot(home_x, home_y)), math.atan2(home_y, home_x))
                    return
                if fresh:
                    x, y = ball
                    if 0. < x < .32 and abs(y) < .40:
                        toe = self.p.get('game.soft_kick.right.toe_x_mm', 0.)/1000
                        target_x = toe + self.p['game.ball_diameter_m']/2 + .015
                        dx = max(-.06, min(x-target_x, .25-depth))
                        dy = y-math.copysign(self.p['game.kick_offset_mm']/1000, y)
                        _, field_y = odom.field_point((dx, dy))
                        if toe > 0 and abs(field_y-own['y']) < own['width']/2-.07 and math.hypot(dx,dy) > .025:
                            self.describe('keeper_clear_setup', 'align near foot for clearance')
                            await self.walk(min(.10, math.hypot(dx,dy)), math.atan2(dy,dx))
                            return
                    step = min(.10, .25-depth, x-.10)
                    _, field_y = odom.field_point(ball)
                    if (.17 < x < .68 and step > .025 and abs(y) < .23
                            and abs(field_y-own['y']) < own['width']/2-.10):
                        self.describe('keeper_intercept', 'close on visible ball inside goal width')
                        course = math.atan2(y, x)
                        await self.walk(step, course)
                        return
        if ball is None:
            self.describe('keeper_hold', 'no fresh ball; no blind lateral pursuit')
            return
        x, y = ball
        self.g.info['ball'] = dict(x_m=x, y_m=y)
        # A distant colour bearing can veto outward movement, but cannot prove
        # that the robot remains between its own posts. No fake field position.
        if (time.monotonic()-self.goal_at < 8 and self.goal_offset is not None
                and abs(self.goal_offset) > math.radians(6) and y*self.goal_offset < 0):
            self.describe('keeper_hold', 'enemy bearing vetoes further outward displacement')
            return
        if x <= 0 or abs(y) <= .10:
            self.describe('keeper_hold', 'ball centred or behind; no forward movement')
            return
        odom = getattr(self.g, 'odometry', None)
        if (odom and getattr(odom, 'confidence', None) != 'degraded'
                and .45 < x < 1.0 and abs(y) > .12 and time.monotonic()-scene['received'] < .5):
            # Shift once while the ball is still in front. Stay inside the goal mouth.
            distance = min(.12, abs(y)-.04)
            _, projected_y = odom.field_point((0., math.copysign(distance, y)))
            own = self.specs[self.p['match.own_goal']]
            if abs(projected_y-own['y']) <= own['width']/2-.12:
                self.describe('keeper_track', 'short lateral walk to cover observed ball')
                await self.walk(distance, math.copysign(math.pi/2, y))
                return
        direction = 'left' if y > 0 else 'right'
        fraction = min(1., 20/max(1., self.p[f'motion.jump_{direction}_mm']))
        self.move_ball, self.motion_changed = ball, False
        self.track_deadline = scene['received']+.5
        self.describe('keeper_track', f'ball x={x*1000:.0f} y={y*1000:.0f}mm; one {direction} jump, nominal <=20mm')
        await self.jump(direction, fraction)
        self.move_ball = None
