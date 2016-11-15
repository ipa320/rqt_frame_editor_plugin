#!/usr/bin/env python

import rospy

from frame_editor.constructors_geometry import ToTransformStamped
from frame_editor.interface import Interface
from frame_editor.objects import Frame


class FrameEditor_TF(Interface):

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

    def broadcast(self, editor):
        #print "> Broadcasting"
        now = rospy.Time.now()
        transforms = [
            ToTransformStamped(
                f.position, f.orientation, now, f.name, f.parent)
            for f in editor.frames.values()]
        Frame.tf_broadcaster.sendTransform(transforms)

# eof
