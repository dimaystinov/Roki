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
        return True

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
        scene = await self.look(0, self.p['game.head_tilt'])
        if not self.g._same_plan(revision):
            return
        ball = scene.get('ball') if scene else None
        if self.p.get('sim.keeper_intercept'):
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
