#!/usr/bin/env python3
from geometry_msgs.msg import (Point, Pose, Quaternion, TransformStamped,
                               Vector3)
from builtin_interfaces.msg import Time

def cast_inputs_to_float(func):
    def wrapper(*args, **kwargs):
        args = tuple(float(a) for a in args)  # Convert positional args to float
        kwargs = {k: float(v) for k, v in kwargs.items()}  # Convert keyword args to float
        return func(*args, **kwargs)
    return wrapper

## Pose ##
##
def ToPose(p, o):
    return Pose(position=ToPoint(p), orientation=ToQuaternion(o))


## Point ##
##
@cast_inputs_to_float
def NewPoint(x, y, z):
    return Point(x=x, y=y, z=z)


def ToPoint(p):
    return NewPoint(*p)


def FromPoint(p):
    return (p.x, p.y, p.z)


## Orientation ##
##
@cast_inputs_to_float
def NewQuaternion(x, y, z, w):
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


def ToQuaternion(q):
    return NewQuaternion(*q)

def FromQuaternion(o):
    return (o.x, o.y, o.z, o.w)


## Vector3 ##
##
@cast_inputs_to_float
def NewVector3(x, y, z):
    return Vector3(x=x, y=y, z=z)


def ToVector3(p):
    return NewVector3(*p)


def FromVector3(v):
    return (v.x, v.y, v.z)


## TransformStamped ##
##
def FromTransformStamped(msg):
    return (FromVector3(msg.transform.translation),
            FromQuaternion(msg.transform.rotation))


def ToTransformStamped(translation, rotation, stamp, child, parent):
    t = TransformStamped()
    t.header.stamp = stamp.to_msg()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation = ToVector3(translation)
    t.transform.rotation = ToQuaternion(rotation)
    return t

# eof
