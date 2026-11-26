"""Explicit xyzw rotations. All axes are in the BODY IMU frame (+Y up).

A calibrated mounting and world-frame alignment are required. This module does
not assume servo targets are measured angles or activate motion fallback.
"""
import math


def normalized(q):
    if len(q)!=4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Invalid quaternion')
    n=math.sqrt(sum(v*v for v in q))
    if n < 1e-8:raise ValueError('Zero quaternion')
    return tuple(v/n for v in q)


def multiply(a,b):
    x,y,z,w=normalized(a);X,Y,Z,W=normalized(b)
    return normalized((w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                       w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z))


def inverse(q):
    x,y,z,w=normalized(q)
    return -x,-y,-z,w


def rotation(axis,angle):
    if not math.isfinite(angle) or len(axis)!=3 or not all(math.isfinite(v) for v in axis):
        raise ValueError('Invalid rotation')
    n=math.sqrt(sum(v*v for v in axis))
    if n<1e-8:raise ValueError('Zero axis')
    return (*[v/n*math.sin(angle/2) for v in axis],math.cos(angle/2))


def body_from_head(head, pan_rad, tilt_rad, mounting_xyzw, world_alignment_xyzw):
    neck=multiply(rotation((0,1,0),pan_rad),rotation((1,0,0),tilt_rad))
    relative=multiply(neck,mounting_xyzw)
    return multiply(multiply(world_alignment_xyzw,head),inverse(relative))
