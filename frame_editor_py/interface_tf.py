#!/usr/bin/env python

import rclpy

from frame_editor_py.constructors_geometry import ToTransformStamped
from frame_editor_py.interface import Interface
from frame_editor_py.objects import Frame


class FrameEditor_TF(Interface):

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

    def broadcast(self, editor):
        #print "> Broadcasting"
        now = self.editor.node.get_clock().now()
        transforms = [
            ToTransformStamped(
                f.position, f.orientation, now, f.name, f.parent)
            for f in editor.frames.values()]
        Frame.tf_broadcaster.sendTransform(transforms)

# eof
