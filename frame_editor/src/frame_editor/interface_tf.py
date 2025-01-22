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

        # change if there is a pose change
        change = False
        for element in elements:
            if element is not None and (level & 1 == 1 or level & 4 == 4):
                change = True

        # publish all transforms if any changed (required for tf2 static)
        if change:
            transforms = [
                ToTransformStamped(
                    f.position, f.orientation, now, f.name, f.parent)
                for f in editor.frames.values()]
            Frame.tf_broadcaster.sendTransform(transforms)

# eof
