#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

import time

import rospy
import rosparam

import tf

from frame_editor.objects import *
from frame_editor.constructors_geometry import *
from frame_editor.constructors_std import *
from frame_editor.srv import *
from geometry_msgs.msg import Pose

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarkerControl, Marker


class FrameEditor:

    def __init__(self):
        self.frames = {}
        self.active_frame = None

        self.server = InteractiveMarkerServer("frame_editor_interactive")

        rospy.Service("align_frame", AlignFrame, self.callback_align_frame)
        rospy.Service("edit_frame", EditFrame, self.callback_edit_frame)
        rospy.Service("get_frame", GetFrame, self.callback_get_frame)
        rospy.Service("remove_frame", RemoveFrame, self.callback_remove_frame)
        rospy.Service("set_frame", SetFrame, self.callback_set_frame)

        self.set_marker_settings(["x", "y", "z", "a", "b", "c"])

        self.observers = []

    def clear_all(self):
        print "> Deleting all frames"
        self.make_interactive(None)
        self.frames = {}
        time.sleep(0.1)
        self.update_obsevers(1+2+4) # update GUI

    def update_frame(self, frame):
        if frame:
            frame.broadcast() # update tf
        self.make_interactive(self.active_frame) # update marker (just in case)
        time.sleep(0.1) # wait a bit for tf # TODO better way?
        self.update_obsevers(4) # update GUI


    def update_obsevers(self, level):
        for observer in self.observers:
            observer.update(self, level)


    def set_marker_settings(self, arrows, frame=None, scale=0.25):
        '''arrows is a list with any number of the following strings: ["x", "y", "z", "a", "b", "c"].'''

        if frame:
            style = frame.style
        else:
            style = "none"

        ## Marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = ""
        int_marker.name = ""
        int_marker.description = "Frame Editor"
        int_marker.pose = Pose()
        int_marker.scale = scale

        if "x" in arrows:
            control = InteractiveMarkerControl()
            control.name = "move_x"
            control.orientation = NewQuaternion(1, 0, 0, 1)
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control);

        if "y" in arrows:
            control = InteractiveMarkerControl()
            control.name = "move_y"
            control.orientation = NewQuaternion(0, 1, 0, 1)
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control);

        if "z" in arrows:
            control = InteractiveMarkerControl()
            control.name = "move_z"
            control.orientation = NewQuaternion(0, 0, 1, 1)
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control);

        if "a" in arrows:
            control = InteractiveMarkerControl()
            control.name = "rotate_x"
            control.orientation = NewQuaternion(1, 0, 0, 1)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control);

        if "b" in arrows:
            control = InteractiveMarkerControl()
            control.name = "rotate_y"
            control.orientation = NewQuaternion(0, 1, 0, 1)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control);


        if "c" in arrows:
            control = InteractiveMarkerControl()
            control.name = "rotate_z"
            control.orientation = NewQuaternion(0, 0, 1, 1)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control);


        ## Style ##
        ##
        style_marker = Marker()
        style_marker.scale = NewVector3(0.75*scale, 0.75*scale, 0.75*scale)
        style_marker.color = NewColor(0.0, 0.5, 0.5, 0.75)

        if style != "none":
            style_marker = frame.marker

        style_control = InteractiveMarkerControl()
        style_control.always_visible = True
        style_control.markers.append(style_marker)

        if style != "none":
            int_marker.controls.append(style_control)


        self.int_marker = int_marker


    def add_frame(self, frame):
        print "> Adding frame:"
        frame.print_all()

        self.frames[frame.name] = frame

        self.update_obsevers(1)

    def remove_frame(self, name):
        print "> Removing frame", name

        ## If active frame is deleted, deactivate
        if self.active_frame and name == self.active_frame.name:
            self.make_interactive(None)

        del self.frames[name]

        self.update_obsevers(1)


    def get_tf_frames(self):
        return Frame.listener.getFrameStrings()


    def align_frame(self, frame, source_name, mode):

        (position, orientation) = frame.listener.lookupTransform(frame.parent, source_name, rospy.Time(0))

        pos = list(frame.position)
        if "x" in mode:
            pos[0] = position[0]
        if "y" in mode:
            pos[1] = position[1]
        if "z" in mode:
            pos[2] = position[2]
        frame.position = tuple(pos)

        rpy = list(tf.transformations.euler_from_quaternion(frame.orientation))
        rpy_new = tf.transformations.euler_from_quaternion(orientation)
        if "a" in mode:
            rpy[0] = rpy_new[0]
        if "b" in mode:
            rpy[1] = rpy_new[1]
        if "c" in mode:
            rpy[2] = rpy_new[2]
        if "a" in mode or "b" in mode or "c" in mode:
            frame.orientation = tf.transformations.quaternion_from_euler(*rpy)

        self.update_frame(frame)


    def broadcast(self):
        #print "> Broadcasting"
        for frame in self.frames.values():
            frame.broadcast()


    ## INTERACTION ##
    ##
    def make_interactive(self, frame):

        ## Stop currently active frame
        if self.active_frame is not None:
            self.server.erase(self.active_frame.name)
            self.server.applyChanges()

        self.active_frame = frame

        if frame is not None:
            self.set_marker_settings(["x", "y", "z", "a", "b", "c"], frame)

            self.int_marker.name = frame.name
            self.int_marker.header.frame_id = frame.parent
            self.int_marker.pose = frame.pose

            self.server.insert(self.int_marker, self.callback_marker)
            self.server.applyChanges()

        self.update_obsevers(2)

    def callback_marker(self, feedback):
        self.active_frame.position = FromPoint(feedback.pose.position)
        self.active_frame.orientation = FromQuaternion(feedback.pose.orientation)

        self.update_obsevers(4)


    ## PRINT ##
    ##
    def print_all(self):
        print "> Printing all frames"

        for frame in self.frames:
            frame.print_all()


    ## FILE I/O ##
    ##
    def load_file(self, filename, namespace = None):
        print "> Loading file"
        data = rosparam.load_file(filename, namespace)[0][0]
        print data
        self.load_data(data)

    def load_params(self, namespace):
        if not rosparam.list_params(namespace):
            print "> No data to load"
        else:
            data = rosparam.get_param(namespace)
            self.load_data(data)

    def load_data(self, data):

        ## Import data
        for name, frame in data["frames"].items():
            t = frame["position"]
            o = frame["orientation"]

            if "style" in frame:
                style = frame["style"]
            else:
                style = "none"

            if "data" in frame:
                dat = frame["data"]

            position = (t["x"], t["y"], t["z"])
            orientation = (o["x"], o["y"], o["z"], o["w"])

            if style == "plane":
                f = Object_Plane(name, position, orientation, frame["parent"], dat["length"], dat["width"])
            elif style == "cube":
                f = Object_Cube(name, position, orientation, frame["parent"], dat["length"], dat["width"], dat["height"])
            elif style == "sphere":
                f = Object_Sphere(name, position, orientation, frame["parent"], dat["diameter"])
            elif style == "axis":
                f = Object_Axis(name, position, orientation, frame["parent"], dat["length"], dat["width"])
            else:
                f = Frame(name, position, orientation, frame["parent"])
            
            self.add_frame(f)

        print "> Loading done"

    def save_file(self, filename, namespace):

        ## Data
        data = {}
        frames = {}

        for frame in self.frames.values():
            t = {}
            t["x"] = frame.position[0]
            t["y"] = frame.position[1]
            t["z"] = frame.position[2]

            o = {}
            o["x"] = frame.orientation[0]
            o["y"] = frame.orientation[1]
            o["z"] = frame.orientation[2]
            o["w"] = frame.orientation[3]

            f = {}
            f["parent"] = frame.parent
            f["position"] = t
            f["orientation"] = o

            f["style"] = frame.style

            if frame.style == "plane":
                f["data"] = { "length": frame.length, "width":frame.width }
            
            elif frame.style == "cube":
                f["data"] = { "length": frame.length, "width": frame.width, "height": frame.height }
            
            elif frame.style == "sphere":
                f["data"] = { "diameter": frame.diameter }
            
            elif frame.style == "axis":
                f["data"] = { "length": frame.length, "width": frame.width }

            frames[frame.name] = f

        data["frames"] = frames

        ## To parameter server
        rospy.set_param(namespace, data)
        print rospy.get_param(namespace)

        ## Dump param to file
        print "Saving to file", filename
        rosparam.dump_params(filename, namespace)
        print "Saving done"



    ## SERVICE CALLBACKS ##
    ##
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

        elif request.name not in self.frames:
            print " Error: Frame not found:", request.name
            response.error_code = 2

        else:
            frame = self.frames[request.name]

            m = request.mode
            mode = []
            if m & 1: mode.append("x")
            if m & 2: mode.append("y")
            if m & 4: mode.append("z")
            if m & 8: mode.append("a")
            if m & 16: mode.append("b")
            if m & 32: mode.append("c")

            self.align_frame(frame, request.source_name, mode)

        return response

    def callback_edit_frame(self, request):
        print "> Request to edit frame", request.name

        response = EditFrameResponse()
        response.error_code = 0

        if request.name == "":
            ## Reset
            self.make_interactive(None)

        elif request.name not in self.frames:
            print " Error: Frame not found:", request.name
            response.error_code = 2

        else:
            ## Set
            self.make_interactive(self.frames[request.name])

        return response

    def callback_get_frame(self, request):
        print "> Request to get frame", request.name

        response = GetFrameResponse()
        response.error_code = 0

        if request.name == "":
            print " Error: No name given"
            response.error_code = 1

        elif request.name not in self.frames:
            print " Error: Frame not found:", request.name
            response.error_code = 2

        else:
            f = self.frames[request.name]
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
            self.remove_frame(request.name)

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
            self.add_frame(f)

        return response


if __name__ == "__main__":

    rospy.init_node('frame_editor')

    editor = FrameEditor()
    editor.load_params(rospy.get_name())

    print "Frame editor ready!"
    rate = rospy.Rate(100) # hz
    while not rospy.is_shutdown():
        editor.broadcast()
        rate.sleep()

# eof
