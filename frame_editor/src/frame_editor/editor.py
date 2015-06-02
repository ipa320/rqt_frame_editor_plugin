#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

import time

import rospy
import rosparam

import tf

from frame_editor.constructors_geometry import *
from frame_editor.constructors_std import *
from frame_editor.srv import *
from geometry_msgs.msg import Pose

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarkerControl, Marker


class Frame:

    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    def __init__(self, name, position=(0,0,0), orientation=(0,0,0,1), parent="world", style="none"):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.parent = parent
        self.style = style

    @property
    def pose(self):
        return ToPose(self.position, self.orientation)

    def print_all(self):
        print "  {} (parent: {}) {} {}".format(self.name, self.parent, self.position, self.orientation)

    def broadcast(self):
        Frame.broadcaster.sendTransform(self.position, self.orientation,
            rospy.Time.now(),
            self.name, self.parent)

    def value(self, symbol):
        if symbol == 'x':
            return self.position[0]
        elif symbol == 'y':
            return self.position[1]
        elif symbol == 'z':
            return self.position[2]
        else:
            rpy = tf.transformations.euler_from_quaternion(self.orientation)
            if symbol == 'a':
                return rpy[0]
            elif symbol == 'b':
                return rpy[1]
            elif symbol == 'c':
                return rpy[2]

    def set_value(self, symbol, value):
        if symbol in ['x', 'y', 'z']:
            position = list(self.position)
            if symbol == 'x':
                position[0] = value
            elif symbol == 'y':
                position[1] = value
            elif symbol == 'z':
                position[2] = value
            self.position = tuple(position)
        else:
            rpy = list(tf.transformations.euler_from_quaternion(self.orientation))
            if symbol == 'a':
                rpy[0] = value
            elif symbol == 'b':
                rpy[1] = value
            elif symbol == 'c':
                rpy[2] = value
            self.orientation = tf.transformations.quaternion_from_euler(*rpy)


class FrameEditor:

    def __init__(self):
        self.frames = {}
        self.active_frame = None

        self.server = InteractiveMarkerServer("frame_editor_interactive")

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


    def set_marker_settings(self, arrows, style="none", scale=0.25):
        '''arrows is a list with any number of the following strings: ["x", "y", "z", "a", "b", "c"].'''

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

        if style == "cube":
            style_marker.type = Marker.CUBE
        elif style == "sphere":
            style_marker.type = Marker.SPHERE
        elif style == "plane":
            style_marker.type = Marker.TRIANGLE_LIST
            style_marker.points = [
                NewPoint(-0.5, -0.5, 0.0), NewPoint(0.5, -0.5, 0.0), NewPoint(-0.5, 0.5, 0.0),
                NewPoint(0.5, -0.5, 0.0), NewPoint(0.5, 0.5, 0.0), NewPoint(-0.5, 0.5, 0.0)
            ]
        else:
            style = "none" # just in case

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
            self.set_marker_settings(["x", "y", "z", "a", "b", "c"], frame.style)

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

            f = Frame(name,
                (t["x"], t["y"], t["z"]),
                (o["x"], o["y"], o["z"], o["w"]),
                frame["parent"],
                style
                )
            
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
