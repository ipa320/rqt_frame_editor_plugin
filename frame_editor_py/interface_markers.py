#!/usr/bin/env python3
import copy
import rclpy

from frame_editor_py.interface import Interface

from visualization_msgs.msg import Marker


class FrameEditor_Markers(Interface):

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

        self.publisher = self.editor.node.create_publisher(Marker, "frame_editor_marker", 10)
        self.last_publish_time = self.editor.node.get_clock().now()
        self.publish_period = rclpy.duration.Duration(seconds=2.0)


    def update(self, editor, level, elements):

        ## Publish all changed markers

        for element in elements:
            if not element:
                continue
            if element.marker:
                self.publish_marker(element)


    def publish_marker(self, element):

        element.update_marker() ## ToDo

        marker = copy.deepcopy(element.marker) # copy

        marker.header.frame_id = element.name
        marker.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg() # zero time
        marker.ns = "frame_editor_markers"
        marker.frame_locked = True # Tells rviz to retransform the marker into the current location of the specified frame every update cycle.

        if element.hidden:
            marker.action = Marker.DELETE
        else:
            marker.action = Marker.ADD

        if element.style == "mesh":
            if element.path == "" or element.path is None:
                marker.action = Marker.DELETE

        self.publisher.publish(marker)


    def broadcast(self, editor):
        ## Publish with own rate
        time_now = self.editor.node.get_clock().now()
        if (time_now - self.last_publish_time) >= self.publish_period:

            ## Update all markers
            self.update(editor, 0, editor.frames.values())

            self.last_publish_time = time_now

# eof
