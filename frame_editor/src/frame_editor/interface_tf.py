#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

import rospy

from frame_editor.interface import Interface
from frame_editor.objects import Frame


class FrameEditor_TF(Interface):

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

    def broadcast(self, editor):
        #print "> Broadcasting"

        for frame in editor.frames.values():
            Frame.broadcaster.sendTransform(
                frame.position,
                frame.orientation,
                rospy.Time.now(),
                frame.name,
                frame.parent
            )

# eof
