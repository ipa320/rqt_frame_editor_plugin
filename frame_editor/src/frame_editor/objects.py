#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

import time

import rospy
import rosparam

import tf

from frame_editor.constructors_geometry import *
from frame_editor.constructors_std import *
from frame_editor.srv import *
from geometry_msgs.msg import Pose

from visualization_msgs.msg import InteractiveMarkerControl, Marker


class Frame(object):

    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    __id_counter = -1

    def __init__(self, name, position=(0,0,0), orientation=(0,0,0,1), parent="world", style="none"):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.parent = parent
        self.style = style

        self.hidden = False
        self.marker = None

    @classmethod
    def create_new_id(cls):
        cls.__id_counter = cls.__id_counter + 1
        return cls.__id_counter

    @property
    def pose(self):
        return ToPose(self.position, self.orientation)

    def print_all(self):
        print "  {} (parent: {}) {} {}".format(self.name, self.parent, self.position, self.orientation)

    def broadcast(self):
        Frame.broadcaster.sendTransform(self.position, self.orientation,
            rospy.Time.now(),
            self.name, self.parent)

    def value(self, symbol):
        if symbol == 'x':
            return self.position[0]
        elif symbol == 'y':
            return self.position[1]
        elif symbol == 'z':
            return self.position[2]
        else:
            rpy = tf.transformations.euler_from_quaternion(self.orientation)
            if symbol == 'a':
                return rpy[0]
            elif symbol == 'b':
                return rpy[1]
            elif symbol == 'c':
                return rpy[2]

    def set_value(self, symbol, value):
        if symbol in ['x', 'y', 'z']:
            position = list(self.position)
            if symbol == 'x':
                position[0] = value
            elif symbol == 'y':
                position[1] = value
            elif symbol == 'z':
                position[2] = value
            self.position = tuple(position)
        else:
            rpy = list(tf.transformations.euler_from_quaternion(self.orientation))
            if symbol == 'a':
                rpy[0] = value
            elif symbol == 'b':
                rpy[1] = value
            elif symbol == 'c':
                rpy[2] = value
            self.orientation = tuple(tf.transformations.quaternion_from_euler(*rpy))


class Object_Geometry(Frame):

    def __init__(self, name, position, orientation, parent, style):

        super(Object_Geometry, self).__init__(name, position, orientation, parent, style)

        ## TODO: put this into interface_marker.py
        self.marker = Marker()
        self.marker.scale = NewVector3(1, 1, 1)
        self.marker.pose = Pose()
        self.marker.color = NewColor(0.0, 0.5, 0.5, 0.75)
        self.marker.id = Frame.create_new_id()
        #self.marker.lifetime = 0 # forever
        self.update_marker()

    def update_marker(self):
        #self.marker.header.frame_id = self.parent
        self.marker.header.stamp = rospy.Time.now()

class Object_Plane(Object_Geometry):

    def __init__(self, name, position, orientation, parent, length=1.0, width=1.0):

        self.length = length
        self.width = width

        super(Object_Plane, self).__init__(name, position, orientation, parent, "plane")

    def update_marker(self):
        super(Object_Plane, self).update_marker()

        l = self.length*0.5
        w = self.width*0.5

        self.marker.type = Marker.TRIANGLE_LIST
        self.marker.points = [
            NewPoint(-l, -w, 0.0), NewPoint(l, -w, 0.0), NewPoint(-l, w, 0.0),
            NewPoint( l, -w, 0.0), NewPoint(l,  w, 0.0), NewPoint(-l, w, 0.0)
        ]


class Object_Cube(Object_Geometry):

    def __init__(self, name, position, orientation, parent, length=1.0, width=1.0, height=1.0):

        self.length = length
        self.width = width
        self.height = height

        super(Object_Cube, self).__init__(name, position, orientation, parent, "cube")

    def update_marker(self):
        super(Object_Cube, self).update_marker()

        self.marker.type = Marker.CUBE
        self.marker.scale = NewVector3(self.length, self.width, self.height)


class Object_Sphere(Object_Geometry):

    def __init__(self, name, position, orientation, parent, diameter=1.0):

        self.diameter = diameter

        super(Object_Sphere, self).__init__(name, position, orientation, parent, "sphere")

    def update_marker(self):
        super(Object_Sphere, self).update_marker()

        self.marker.type = Marker.SPHERE
        self.marker.scale = NewVector3(self.diameter, self.diameter, self.diameter)


class Object_Axis(Object_Geometry):

    def __init__(self, name, position, orientation, parent, length=1.0, width=0.05):

        self.length = length
        self.width = width

        super(Object_Axis, self).__init__(name, position, orientation, parent, "axis")

    def update_marker(self):
        super(Object_Axis, self).update_marker()

        self.marker.type = Marker.ARROW
        self.marker.scale = NewVector3(self.length, self.width, self.width)


class Object_Mesh(Object_Geometry):

    def __init__(self, name, position, orientation, parent, mesh_path, scale=1.0):

        self.scale = scale
        self.path = mesh_path

        super(Object_Mesh, self).__init__(name, position, orientation, parent, "mesh")

    def update_marker(self):
        super(Object_Mesh, self).update_marker()

        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_resource = "file:"+self.path
        self.marker.scale = NewVector3(self.scale, self.scale, self.scale)

# eof
