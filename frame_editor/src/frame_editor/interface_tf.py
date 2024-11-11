#!/usr/bin/env python

import rospy

from frame_editor.constructors_geometry import ToTransformStamped
from frame_editor.interface import Interface
from frame_editor.objects import Frame


class FrameEditor_TF(Interface):

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

    def update(self, editor, level, elements):
        now = rospy.Time.now()
        transforms = []
        for element in elements:
            if element is not None and (level & 1 == 1 or level & 4 == 4):
                transforms.append(ToTransformStamped(
                element.position, element.orientation, now, element.name, element.parent))
        if len(transforms) > 0:
            Frame.tf_broadcaster.sendTransform(transforms)
# eof
