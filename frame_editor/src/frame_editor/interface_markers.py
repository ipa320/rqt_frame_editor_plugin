#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

import copy
import rospy

from visualization_msgs.msg import Marker


class FrameEditor_Markers:

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

        self.publisher = rospy.Publisher("frame_editor_marker", Marker, queue_size=10)


    def update(self, editor, level, elements):

        for element in elements:
            if not element:
                continue

            if element.marker:

                marker = copy.deepcopy(element.marker) # copy

                marker.header.frame_id = element.name
                marker.header.stamp = rospy.Time() # zero time
                marker.ns = "frame_editor_markers"

                if element.hidden:
                    marker.action = Marker.DELETE
                else:
                    marker.action = Marker.ADD

                self.publisher.publish(marker)

# eof
