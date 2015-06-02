#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, Vector3, Quaternion


## Pose ##
##
def ToPose(p, o):
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    pose.orientation.x = o[0]
    pose.orientation.y = o[1]
    pose.orientation.z = o[2]
    pose.orientation.w = o[3]
    return pose


## Point ##
##
def ToPoint(p):
    point = Point()
    point.x = p[0]
    point.y = p[1]
    point.z = p[2]
    return point

def NewPoint(x, y, z):
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point

def FromPoint(p):
    return (p.x, p.y, p.z)


## Orientation ##
##
def NewQuaternion(x, y, z, w):
    quat = Quaternion()
    quat.x = x
    quat.y = y
    quat.z = z
    quat.w = w
    return quat

def FromQuaternion(o):
    return (o.x, o.y, o.z, o.w)


## Vector3 ##
##
def ToVector3(p):
    point = Vector3()
    point.x = p[0]
    point.y = p[1]
    point.z = p[2]
    return point

def NewVector3(x, y, z):
    point = Vector3()
    point.x = x
    point.y = y
    point.z = z
    return point

# eof
