#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

import rospy
import tf

from frame_editor.objects import *
from frame_editor.constructors_geometry import *
from frame_editor.constructors_std import *
from frame_editor.srv import *


class FrameEditor_Services:

    def __init__(self, frame_editor):

        self.editor = frame_editor

        rospy.Service("align_frame", AlignFrame, self.callback_align_frame)
        rospy.Service("edit_frame", EditFrame, self.callback_edit_frame)
        rospy.Service("get_frame", GetFrame, self.callback_get_frame)
        rospy.Service("remove_frame", RemoveFrame, self.callback_remove_frame)
        rospy.Service("set_frame", SetFrame, self.callback_set_frame)


    def callback_align_frame(self, request):
        print "> Request to align frame", request.name, "with frame", request.source_name, "mode", request.mode

        response = AlignFrameResponse()
        response.error_code = 0

        if request.name == "":
            print " Error: No name given"
            response.error_code = 1

        elif request.source_name == "":
            print " Error: No source name given"
            response.error_code = 3

        elif request.name not in self.editor.frames:
            print " Error: Frame not found:", request.name
            response.error_code = 2

        else:
            frame = self.editor.frames[request.name]

            m = request.mode
            mode = []
            if m & 1: mode.append("x")
            if m & 2: mode.append("y")
            if m & 4: mode.append("z")
            if m & 8: mode.append("a")
            if m & 16: mode.append("b")
            if m & 32: mode.append("c")

            self.editor.align_frame(frame, request.source_name, mode)

        return response

    def callback_edit_frame(self, request):
        print "> Request to edit frame", request.name

        response = EditFrameResponse()
        response.error_code = 0

        if request.name == "":
            ## Reset
            self.editor.select_frame(None)

        elif request.name not in self.editor.frames:
            print " Error: Frame not found:", request.name
            response.error_code = 2

        else:
            ## Set
            self.editor.select_frame(self.editor.frames[request.name])

        return response

    def callback_get_frame(self, request):
        print "> Request to get frame", request.name

        response = GetFrameResponse()
        response.error_code = 0

        if request.name == "":
            print " Error: No name given"
            response.error_code = 1

        elif request.name not in self.editor.frames:
            print " Error: Frame not found:", request.name
            response.error_code = 2

        else:
            f = self.editor.frames[request.name]
            f.print_all()
            response.name = f.name
            response.parent = f.parent
            response.pose = ToPose(f.position, f.orientation)

        return response

    def callback_remove_frame(self, request):
        print "> Request to remove frame", request.name

        response = RemoveFrameResponse()
        response.error_code = 0

        if request.name == "":
            print " Error: No name given"
            response.error_code = 1

        elif request.name not in self.frames:
            print " Error: Frame not found:", request.name
            response.error_code = 2

        else:
            self.editor.remove_frame(request.name)

        return response


    def callback_set_frame(self, request):
        print "> Request to set (or add) frame", request.name

        response = SetFrameResponse()
        response.error_code = 0

        if request.name == "":
            print " Error: No name given"
            response.error_code = 1

        else:
            if request.parent == "":
                request.parent = "world"

            f = Frame(request.name, FromPoint(request.pose.position), FromQuaternion(request.pose.orientation))
            self.editor.add_frame(f)

        return response

# eof