#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node


import tf_transformations as tft
import tf2_ros

import yaml

from frame_editor_py.constructors_geometry import *
from frame_editor_py.constructors_std import *
from frame_editor.srv import *
from frame_editor_py import utils_tf

from geometry_msgs.msg import Pose

from visualization_msgs.msg import InteractiveMarkerControl, Marker
import rospkg


class Frame(object):

    tf_broadcaster = None
    tf_buffer = None
    tf_listener = None
    node = None

    __id_counter = -1

    def __init__(self, name, position=(0,0,0), orientation=(0,0,0,1), parent="world", style="none", group=""):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.parent = parent
        self.style = style
        self.color = (0.0, 0.5, 0.5, 0.75)
        self.group = group
        self.pinned_frame = None

        self.hidden = False
        self.marker = None

    @staticmethod
    def init_tf(node, static=True):
        node.get_logger().warn("############################")
        if Frame.tf_buffer is None:
            Frame.node = node
            if static:
                node.get_logger().info("GOING STATIC")
                Frame.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(node)
            else:
                node.get_logger().info("GOING DYNAMIC")
                Frame.tf_broadcaster = tf2_ros.TransformBroadcaster(node)
            Frame.tf_buffer = tf2_ros.Buffer()
            Frame.tf_listener = tf2_ros.TransformListener(Frame.tf_buffer, node, spin_thread=True)

    @staticmethod
    def was_published_by_frameeditor(name):
        tf2_structure_in_yaml = Frame.tf_buffer.all_frames_as_yaml()
        tf2_structure = yaml.load(tf2_structure_in_yaml, Loader=yaml.Loader)
        try:
            bc = tf2_structure[name]["broadcaster"]
            return bc == Frame.node.get_name()
        except KeyError:
            Frame.node.get_logger().error("KEYERR")
            return False
        
    @classmethod
    def create_new_id(cls):
        cls.__id_counter = cls.__id_counter + 1
        return cls.__id_counter

    @property
    def pose(self):
        return ToPose(self.position, self.orientation)

    def print_all(self, node:Node):
        node.get_logger().info("  {} (parent: {}) {} {} {}".format(self.name, self.parent, self.position, self.orientation, self.group))

    def value(self, symbol):
        if symbol == 'x':
            return self.position[0]
        elif symbol == 'y':
            return self.position[1]
        elif symbol == 'z':
            return self.position[2]
        else:
            rpy = tft.euler_from_quaternion(self.orientation)
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
            rpy = list(tft.euler_from_quaternion(self.orientation))
            if symbol == 'a':
                rpy[0] = value
            elif symbol == 'b':
                rpy[1] = value
            elif symbol == 'c':
                rpy[2] = value
            self.orientation = tuple(tft.quaternion_from_euler(*rpy))

    def set_group(self, group):
        self.group = group  
        
    @staticmethod
    def can_transform(target_frame, source_frame, time_):
        return utils_tf.can_transform(
            Frame.tf_buffer, target_frame, source_frame, time_)

    @staticmethod
    def wait_for_transform(target_frame, source_frame, timeout):
        return utils_tf.wait_for_transform(
            Frame.tf_buffer, target_frame, source_frame, timeout)


class Object_Geometry(Frame):

    def __init__(self, name, position, orientation, parent, style, group):

        super(Object_Geometry, self).__init__(name, position, orientation, parent, style, group)

        ## TODO: put this into interface_marker.py
        self.marker = Marker()
        self.marker.scale = NewVector3(1, 1, 1)
        self.marker.pose = Pose()
        self.marker.color = NewColor(self.color[0], self.color[1], self.color[2], self.color[3])
        self.marker.id = Frame.create_new_id()
        #self.marker.lifetime = 0 # forever
        self.update_marker()

    def update_marker(self):
        self.marker.header.frame_id = self.name
        now = self.node.get_clock().now()
        self.marker.header.stamp = now.to_msg()

    def set_color(self, color):
        self.color = color
        self.marker.color = NewColor(color[0], color[1], color[2], color[3])
        self.update_marker()
        #self.marker.update()


class Object_Plane(Object_Geometry):

    def __init__(self, name, position, orientation, parent, length=1.0, width=1.0, group=""):

        self.length = length
        self.width = width

        super(Object_Plane, self).__init__(name, position, orientation, parent, "plane", group)

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

    def __init__(self, name, position, orientation, parent, length=1.0, width=1.0, height=1.0, group=""):

        self.length = length
        self.width = width
        self.height = height

        super(Object_Cube, self).__init__(name, position, orientation, parent, "cube", group)

    def update_marker(self):
        super(Object_Cube, self).update_marker()

        self.marker.type = Marker.CUBE
        self.marker.scale = NewVector3(self.length, self.width, self.height)


class Object_Sphere(Object_Geometry):

    def __init__(self, name, position, orientation, parent, diameter=1.0, group=""):

        self.diameter = diameter

        super(Object_Sphere, self).__init__(name, position, orientation, parent, "sphere", group)

    def update_marker(self):
        super(Object_Sphere, self).update_marker()

        self.marker.type = Marker.SPHERE
        self.marker.scale = NewVector3(self.diameter, self.diameter, self.diameter)


class Object_Axis(Object_Geometry):

    def __init__(self, name, position, orientation, parent, length=1.0, width=0.05, group=""):

        self.length = length
        self.width = width

        super(Object_Axis, self).__init__(name, position, orientation, parent, "axis", group)

    def update_marker(self):
        super(Object_Axis, self).update_marker()

        self.marker.type = Marker.ARROW
        self.marker.scale = NewVector3(self.length, self.width, self.width)


class Object_Mesh(Object_Geometry):

    def __init__(self, name, position, orientation, parent, package=None, mesh_path="", scale=1.0, group=""):

        self.scale = scale
        self.path = mesh_path
        self.package = package

        super(Object_Mesh, self).__init__(name, position, orientation, parent, "mesh", group)

    def set_size(self, size):
        self.scale = size
        self.update_marker()
        #self.marker.update()


    def update_marker(self):
        super(Object_Mesh, self).update_marker()

        self.marker.type = Marker.MESH_RESOURCE
        if self.package == "" or self.package is None:
            self.marker.mesh_resource = "file:"+self.path
        else:
            self.marker.mesh_resource = "file:"+rospkg.RosPack().get_path(self.package)+"/"+self.path

        self.marker.scale = NewVector3(self.scale, self.scale, self.scale)

# eof
