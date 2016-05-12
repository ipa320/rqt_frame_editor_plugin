#!/usr/bin/env python

from geometry_msgs.msg import (Point, Pose, Quaternion, TransformStamped,
                               Vector3)


## Pose ##
##
def ToPose(p, o):
    return Pose(position=ToPoint(p), orientation=ToQuaternion(o))


## Point ##
##
def NewPoint(x, y, z):
    return Point(x=x, y=y, z=z)


def ToPoint(p):
    return NewPoint(*p)


def FromPoint(p):
    return (p.x, p.y, p.z)


## Orientation ##
##
def NewQuaternion(x, y, z, w):
    return Quaternion(x=x, y=y, z=z, w=w)


def ToQuaternion(q):
    return NewQuaternion(*q)


def FromQuaternion(o):
    return (o.x, o.y, o.z, o.w)


## Vector3 ##
##
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
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation = ToVector3(translation)
    t.transform.rotation = ToQuaternion(rotation)
    return t

# eof
