"""Finite software-walking recipes from Roki_2_Soccer/class_Motion_real.py.

Strategy selection stays in the supervisor. These recipes describe one original
manoeuvre; the worker supplies polled yaw/live ball data and executes/drains each
cycle. No camera, controller access, sleeps or recovery are hidden here.
"""
import math
from dataclasses import dataclass

from roki_ng.body.calibration import wrap
from roki_ng.body.motion.odometry import forward_yields
from roki_ng.wire import Fault


def clip(value, limit):
    return max(-limit, min(limit, value))


def yields(p):
    first, cycle = forward_yields(p)
    if first <= 0 or cycle <= 0:
        raise Fault('configuration', 'Walking yields require positive first and subsequent steps')
    return first, cycle, p['motion.side_right_20_mm']/20, p['motion.side_left_20_mm']/20


def bounded(count):
    if not 0 <= count <= 256:
        raise Fault('configuration', 'Original manoeuvre exceeds 256 cycles; check motion calibration')
    return count


@dataclass(frozen=True)
class Cycle:
    step: float
    side: float
    heading: float
    index: int
    total: int
    right: bool = True
    half: bool = False
    yaw_gain: float = 1.


def omni(p, distance_mm, direction, heading, *, one_off=True, soft_touch=False, trim_cycles=0):
    """near_distance_omni_motion: preserve original count and ramp formulas."""
    first, stride, right_yield, left_yield = yields(p)
    displacement = distance_mm * math.sin(direction)
    right = displacement <= 0
    side_yield = right_yield if right else left_yield
    n = math.floor(abs(distance_mm*math.cos(direction)-first*30/64)/stride*64/30)+2
    n = max(n, math.floor(abs(displacement)/side_yield)+1)
    if soft_touch:
        n *= 2  # Spread the opening touch over smaller foot movements.
    step = distance_mm*math.cos(direction)/(first+stride*(n-1))*64
    total = n+2
    side = abs(displacement/total*20/side_yield)
    decelerate = step > 15 and total > 4
    if decelerate:
        total += 1
    # Shorten the cruise while preserving the entry and braking cycles.
    total -= min(int(trim_cycles), max(0, total - (4 if decelerate else 2)))
    for i in range(bounded(total)):
        current = step/4 if i == 0 else step/2 if i == 1 else step
        if decelerate:
            if i == total-1: current = step/3
            if i == total-2: current = step*2/3
        yield Cycle(current, side, heading, i, total+(not one_off), right)


def live_ball_cycle(p, observed, target_x, target_y, index, total, right, heading):
    if observed is None:
        return None
    x, y = observed
    dx, dy = x-target_x, y-target_y
    if abs(y) > .30 or (abs(dx) <= .014 and abs(dy) <= .020):
        return None
    _, stride, _, _ = yields(p)
    # Align sideways before closing the last centimetres, without stepping
    # backwards merely because the ball is wide of the striking foot.
    advance = max(0., dx-max(0., abs(dy)-.020)) if dx > 0 else dx
    limit = min(30., .35*p['walk.max_step_mm'])
    step = clip(advance*1000*64/stride*.70, limit)*min(1., (index+1)/3)
    side = clip(dy*400*(-1 if right else 1), 16.)
    return Cycle(step, side, heading, index, total, right=right)


def approach(p, distance, heading, ball, *, stop_x_m=None, target_y_m=None, live_approach=False):
    """far_distance_straight_approach_streaming, with bounded live feedback."""
    # Match the 35% manual walking speed tested on the field.
    forward_limit = min(30., .35*p['walk.max_step_mm'])
    if stop_x_m is not None:
        # Opening approach: one walking job, with a new camera observation at
        # every gait cycle. Reserve the final foot-alignment zone for jumps.
        _, stride_yield, _, _ = yields(p)
        total = 80
        support_right = None
        for i in range(total):
            observed = ball()
            if observed is None:
                break
            x, y = observed
            lateral = 0. if target_y_m is None else y-target_y_m
            if support_right is None:
                support_right = lateral <= 0
            if live_approach:
                order = live_ball_cycle(p, observed, stop_x_m, target_y_m, i, total, support_right, heading)
                if order is None:
                    break
                yield order
                continue
            remaining = x-stop_x_m
            if abs(y) > .30:
                break
            if remaining <= .006 and (target_y_m is None or remaining >= -.006) and abs(lateral) <= .020:
                break
            # Keep the support-leg convention through lateral sign changes.
            side = max(-3., min(3., lateral * (-250 if support_right else 250)))
            # Reduce longitudinal travel while aligning laterally at the foot.
            advance = remaining - max(0., abs(lateral)-.020)
            commanded = min(max(0., advance)*1000*64/stride_yield,
                            max(0., advance)*100.)
            step = min(forward_limit*min(1., (i+1)/3), commanded)
            if remaining < -.006:
                step = max(-2., remaining*100.)
            yield Cycle(step, side if target_y_m is not None else 0., heading,
                        i, total, right=support_right)
        return
    # Settle and let the supervisor reobserve the ball after each short walk.
    total = 6
    factor = 1.
    for i in range(total):
        observed = ball()
        if observed is None:
            break
        x, y = observed
        distance, course = math.hypot(x, y), math.atan2(y, x)
        discontinue = distance-p['game.kick_approach_mm']/1000-.1 < .1
        factor = factor*.6 if discontinue else min(1., factor/.6)
        ramp = min(1., (i+1)/3)
        ramp = min(ramp, (total-i)/3)
        step = forward_limit*factor*ramp
        # The supervisor turns to the ball before a far approach. Reacquire
        # the bearing between walks instead of swapping support legs in stride.
        if i and abs(course) > math.radians(15) and not discontinue:
            break
        yield Cycle(step, 0., heading, i, i+1 if factor < .6 else total)
        if factor < .6:
            break


def verify(p, heading, ball):
    """verify_ball_position, original 50-cycle cap and 30/20 mm orders."""
    _, stride, right_yield, left_yield = yields(p)
    previous, step = None, 0.
    kick_right = None
    for i in range(50):
        observed = ball()
        if observed is None:
            break
        x, y = observed[0]*1000, observed[1]*1000
        # Original verify_ball_position keeps the chosen foot inside +/-20 mm.
        # Using the sign on every frame makes centre-line noise alternate sides.
        if kick_right is None:
            kick_right = y <= 0
        if y > 20:
            kick_right = False
        elif y < -20:
            kick_right = True
        side = y + p['game.kick_offset_mm'] if kick_right else y-p['game.kick_offset_mm']
        front = x-p['game.kick_approach_mm']
        if i == 0 and front <= 30 and abs(side) < 20:
            break
        right = side <= 0
        if previous is not None and previous != right:
            yield Cycle(step, 0, heading, 1, 3, previous, half=True, yaw_gain=1.1)
        forward_order = front*64/stride
        side_order = side*20/(right_yield if right else left_yield)
        step = clip(forward_order, 30)
        last = forward_order <= 30 and abs(side_order) < 20
        yield Cycle(step, min(20, abs(side_order)), heading, i, i+1 if last else 50,
                    right, yaw_gain=1.1)
        previous = right
        if last:
            break


def around(p, distance, course, initial_heading, kick_heading, ball):
    """go_Around_Ball, called after the original turn towards the ball."""
    _, _, right_yield, left_yield = yields(p)
    alpha = wrap(kick_heading-initial_heading)
    reference = initial_heading
    if course*alpha > 0:
        alpha = wrap(alpha-course)
        reference = wrap(initial_heading+course)
    previous = True
    for cycle in omni(p, distance*1000-p['game.kick_approach_mm'], 0,
                      wrap(initial_heading+course), one_off=False):
        previous = cycle.right
        yield cycle
    right = alpha >= 0
    if previous != right:
        yield Cycle(0, 0, reference, 1, 3, previous, half=True)
    side_yield = right_yield if right else left_yield*.75
    if side_yield > 400:
        raise Fault('configuration', 'Side yield exceeds original 0.2 m orbit diameter')
    increment = math.copysign(2*math.asin(side_yield/400), alpha)
    total = bounded(round(abs(alpha/increment))+1)
    for i in range(1, total):
        observed = ball()
        if observed is None:
            break
        step = clip(observed[0]*1000-p['game.kick_approach_mm'], 30)
        yield Cycle(step, 15, reference+i*increment, i, total, right)


def kickoff_arc(p, chord_m, course, direction):
    """Scaled kick_off_ride: turn onto the tangent and sweep 60° while advancing."""
    half = math.radians(30)
    tangent = course - direction*half
    entry = max(1, math.ceil(abs(tangent)/.2))
    for i in range(entry):
        yield Cycle(0, 0, tangent*(i+1)/entry, i, entry+1)
    arc_mm = chord_m*1000*half/math.sin(half)
    for cycle in omni(p, arc_mm, 0., 0.):
        fraction = (cycle.index+.5)/cycle.total
        yield Cycle(cycle.step, 0., tangent + direction*2*half*fraction,
                    cycle.index, cycle.total, cycle.right)


def kickoff(p, invert, ball):
    """Centre kickoff: 20 cm from the marker, then original kick_off_ride.

    The original manoeuvre begins on the centre-circle edge.  Our centre entry
    is on the centre mark, so its calibrated 200 mm straight lead-in restores
    that starting point before the unmodified turn/arc/straight sequence.
    """
    # ``omni`` uses the calibrated run yields, so 200 is a requested physical
    # distance in millimetres rather than an arbitrary cycle count.
    yield from omni(p, 200., 0., 0.)
    _, stride, _, _ = yields(p)
    angle = math.radians(30)
    count = math.ceil(angle/.2)
    for i in range(count):
        yield Cycle(0, 0, invert*angle/count*(i+1), i, count+1)
    delta = -2*angle
    count = math.ceil(abs(delta/.2))
    increment = delta/count
    radius = .55/2/math.sin(angle)
    step = radius*abs(increment)*1000*64/stride*1.1
    count -= 2
    for i in range(count):
        current = step/3 if i == 0 else step*2/3 if i == 1 else step
        if i == count-1: current = step-(step-30)*2/3
        if i == count-2: current = step-(step-30)/3
        yield Cycle(current, 0, (angle+increment*i)*invert, i+1, count+2)
    accelerate = 30-step > 22
    count = bounded(math.ceil(abs(.6*1000/stride))+int(accelerate)+1)
    for i in range(count):
        current = 30
        if accelerate:
            if i == 0: current = 10
            if i == 1: current = 20
        if i == count-1: current = 10
        if i == count-2: current = 20
        # The caller adds current yaw for this relative visual correction.
        observed = ball()
        heading = -angle*invert if observed is None else math.atan2(observed[1], observed[0])/5
        yield Cycle(current, 0, heading, i+1, count+2, yaw_gain=1. if observed is None else 0.)
