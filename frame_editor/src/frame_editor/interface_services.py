#!/usr/bin/env python

import copy
import time

import rospy
import os

from frame_editor.objects import *
from frame_editor.commands import *
from frame_editor.interface import Interface

from frame_editor.constructors_geometry import *
from frame_editor.constructors_std import *

from frame_editor.srv import *


class FrameEditor_Services(Interface):

    def __init__(self, frame_editor):

        self.editor = frame_editor

        rospy.Service("~align_frame", AlignFrame, self.callback_align_frame)
        rospy.Service("~edit_frame", EditFrame, self.callback_edit_frame)
        rospy.Service("~get_frame", GetFrame, self.callback_get_frame)
        rospy.Service("~get_frame_names", GetFrameNames, self.callback_get_frame_names)
        rospy.Service("~remove_frame", RemoveFrame, self.callback_remove_frame)
        rospy.Service("~set_frame", SetFrame, self.callback_set_frame)
        rospy.Service("~set_parent", SetParentFrame, self.callback_set_parent_frame)
        rospy.Service("~copy_frame", CopyFrame, self.callback_copy_frame)

        rospy.Service("~load_yaml", LoadYaml, self.callback_load_yaml)
        rospy.Service("~save_yaml", SaveYaml, self.callback_save_yaml)


    def callback_align_frame(self, request):
        rospy.loginfo("> Request to align frame {} with frame {} mode {}".format(request.name, request.source_name, request.mode))

        response = AlignFrameResponse()
        response.error_code = 0

        if request.name == "":
            rospy.logerr(" Error: No name given")
            response.error_code = 1

        elif request.source_name == "":
            rospy.logerr(" Error: No source name given")
            response.error_code = 3

        elif request.name not in self.editor.frames:
            rospy.logerr(" Error: Frame not found: {}".format(request.name))
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

            self.editor.command(Command_AlignElement(self.editor, frame, request.source_name, mode))

        return response


    def callback_edit_frame(self, request):
        rospy.loginfo("> Request to edit frame {}".format(request.name))

        response = EditFrameResponse()
        response.error_code = 0

        if request.name == "":
            ## Reset
            self.editor.command(Command_SelectElement(self.editor, None))

        elif request.name not in self.editor.frames:
            rospy.logerr(" Error: Frame not found: {}".format(request.name))
            response.error_code = 2

        else:
            ## Set
            self.editor.command(Command_SelectElement(self.editor, self.editor.frames[request.name]))

        return response


    def callback_get_frame(self, request):
        rospy.loginfo("> Request to get frame {}".format(request.name))

        response = GetFrameResponse()
        response.error_code = 0

        if request.name == "":
            rospy.logerr(" Error: No name given")
            response.error_code = 1

        elif request.name not in self.editor.frames:
            rospy.logerr(" Error: Frame not found: {}".format(request.name))
            response.error_code = 2

        else:
            f = self.editor.frames[request.name]
            f.print_all()
            response.name = f.name
            response.parent = f.parent
            response.pose = ToPose(f.position, f.orientation)

        return response
    

    def callback_get_frame_names(self, request):
        rospy.loginfo("> Request to get frame names")

        response = GetFrameNamesResponse()
        response.error_code = 0

        for frame in self.editor.frames.values():
            response.names.append(frame.name)

        return response


    def callback_remove_frame(self, request):
        rospy.loginfo("> Request to remove frame {}".format(request.name))

        response = RemoveFrameResponse()
        response.error_code = 0

        if request.name == "":
            rospy.logerr(" Error: No name given")
            response.error_code = 1

        elif request.name not in self.editor.frames:
            rospy.logerr(" Error: Frame not found: {}".format(request.name))
            response.error_code = 2

        else:
            self.editor.command(Command_RemoveElement(self.editor, self.editor.frames[request.name]))

        return response


    def callback_set_frame(self, request):
        rospy.loginfo("> Request to set (or add) frame {} {}".format(request.name, request.parent))

        response = SetFrameResponse()

        if request.name == "":
            rospy.logerr(" Error: No name given")
            response.error_code = 1
            return response

        if request.parent == "":
            if request.name in self.editor.frames:
                request.parent = self.editor.frames[request.name].parent
            else:
                rospy.logerr("Error: No parent given and frame previously not existing")
                response.error_code = 2
                return response

        f = Frame(request.name,
                  FromPoint(request.pose.position),
                  FromQuaternion(request.pose.orientation),
                  request.parent)
        self.editor.command(Command_AddElement(self.editor, f))
        
        response.error_code = 0
        return response


    def callback_set_parent_frame(self, request):
        rospy.loginfo("> Request to set parent_frame {} {}".format(request.name, request.parent))

        response = SetParentFrameResponse()
        response.error_code = 0

        if request.name == "":
            rospy.logerr(" Error: No frame_name given")
            response.error_code = 1

        elif request.parent == "":
            rospy.logerr(" Error: No parent_name given")
            response.error_code = 2

        else:
            f = self.editor.frames[request.name]
            self.editor.command(Command_SetParent(self.editor, f, request.parent, request.keep_absolute))

        return response

    def callback_load_yaml(self, request):
        rospy.loginfo("> Request to load yaml file:'{}'".format(request.filename))

        response = LoadYamlResponse()
        try:
            self.editor.load_file(os.path.expanduser(request.filename))
            response.success = True
            response.message = "file loaded"
        except Exception as e:
            response.success = False
            response.message = "Exception: {}".format(str(e))

        return response

    def callback_save_yaml(self, request):
        rospy.loginfo("> Request to save yaml file to:'{}'".format(request.filename))

        response = SaveYamlResponse()
        try:
            self.editor.save_file(os.path.expanduser(request.filename))
            response.success = True
            response.message = "file saved"
        except Exception as e:
            response.success = False
            response.message = "Exception: {}".format(str(e))

        return response

    def callback_copy_frame(self, request):
        rospy.loginfo("> Request to copy frame '{}' with new name '{}' and new parent name '{}'".format(request.source_name, request.name, request.parent))

        response = CopyFrameResponse()
        response.error_code = 0

        if request.name == "":
            rospy.logerr(" Error: No name given")
            response.error_code = 1

        elif request.source_name == "":
            rospy.logerr(" Error: No source name given")
            response.error_code = 3

        else:
            t = time.time()

            try:
                # If not existing yet: create frame
                if request.name not in self.editor.frames:
                    rospy.loginfo(">> add")

                    # No parent specified: use source's parent
                    if request.parent == "":
                        if request.source_name in self.editor.frames:
                            request.parent = self.editor.frames[request.source_name].parent
                        else:
                            rospy.logerr(" Error: No parent name given")
                            response.error_code = 3
                            return response

                    Frame.wait_for_transform(request.source_name, request.parent, rospy.Duration(1.0))
                    self.editor.command(Command_CopyElement(self.editor, request.name, request.source_name, request.parent))
                    Frame.wait_for_transform(request.parent, request.name, rospy.Duration(1.0))


                else:
                    frame = self.editor.frames[request.name]

                    Frame.wait_for_transform(request.source_name, request.parent, rospy.Duration(1.0))
                    if (request.parent != "") and (frame.parent != request.parent):
                        rospy.loginfo(">> rebase")
                        self.editor.command(Command_RebaseElement(self.editor, frame, request.source_name, request.parent))
                    else:
                        rospy.loginfo(">> align")
                        self.editor.command(Command_AlignElement(self.editor, frame, request.source_name, ['x', 'y', 'z', 'a', 'b', 'c']))
                    Frame.wait_for_transform(frame.parent, frame.name, rospy.Duration(1.0))

            except Exception as e:
                rospy.logerr("Error: unhandled exception {}".format(e))
                response.error_code = 9

        return response

# eof
