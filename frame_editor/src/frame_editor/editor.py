#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...


import rospy
import rosparam

import tf

from frame_editor.srv import *
from geometry_msgs.msg import Pose

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarkerControl


## HELPERS ##
##
def Position(p):
    return (p.x, p.y, p.z)

def Orientation(o):
    return (o.x, o.y, o.z, o.w)

def ToPose(p, o):
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    pose.orientation.x = o[0]
    pose.orientation.y = o[1]
    pose.orientation.z = o[2]
    pose.orientation.w = o[3]
    return pose



class Frame:
    def __init__(self, name, position=(0,0,0), orientation=(0,0,0,1), parent="world"):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.parent = parent

    @property
    def pose(self):
        return ToPose(self.position, self.orientation)

    def print_all(self):
        print "  {} (parent: {}) {} {}".format(self.name, self.parent, self.position, self.orientation)



class Frame_Editor:
    def __init__(self):
        self.frames = {}
        self.active_frame = None

        self.broadcaster = tf.TransformBroadcaster()

        self.server = InteractiveMarkerServer(rospy.get_name()+"_interactive")

        rospy.Service("edit_frame", EditFrame, self.callback_edit_frame)
        rospy.Service("get_frame", GetFrame, self.callback_get_frame)
        rospy.Service("remove_frame", RemoveFrame, self.callback_remove_frame)
        rospy.Service("set_frame", SetFrame, self.callback_set_frame)

        self.set_marker_style(["x", "y", "z", "a", "b", "c"])


    def set_marker_style(self, style, scale=0.25):
        '''style is a list with any number of the following strings: ["x", "y", "z", "a", "b", "c"].'''

        ## Marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = ""
        int_marker.name = ""
        int_marker.description = "Frame Editor"
        int_marker.pose = Pose()
        int_marker.scale = scale

        if "x" in style:
            control = InteractiveMarkerControl()
            control.name = "move_x"
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control);

        if "y" in style:
            control = InteractiveMarkerControl()
            control.name = "move_y"
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control);

        if "z" in style:
            control = InteractiveMarkerControl()
            control.name = "move_z"
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control);

        if "a" in style:
            control = InteractiveMarkerControl()
            control.name = "rotate_x"
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control);

        if "b" in style:
            control = InteractiveMarkerControl()
            control.name = "rotate_y"
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control);


        if "c" in style:
            control = InteractiveMarkerControl()
            control.name = "rotate_z"
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control);

        self.int_marker = int_marker


    def add_frame(self, frame):
        print "> Adding frame:"
        frame.print_all()

        self.frames[frame.name] = frame

    def remove_frame(self, name):
        print "> Removing frame", name
        del self.frames[name]


    def broadcast(self):
        #print "> Broadcasting"
        for frame in self.frames.values():
            self.broadcaster.sendTransform(frame.position, frame.orientation,
                rospy.Time.now(),
                frame.name, frame.parent)


    ## INTERACTION ##
    ##
    def make_interactive(self, frame):

        ## Stop currently active frame
        if self.active_frame is not None:
            self.server.erase(self.active_frame.name)
            self.server.applyChanges()

        self.active_frame = frame

        if frame is None:
            return # nothing more to do here

        self.int_marker.name = frame.name
        self.int_marker.header.frame_id = frame.parent
        self.int_marker.pose = frame.pose

        self.server.insert(self.int_marker, self.callback_marker)
        self.server.applyChanges()

    def callback_marker(self, feedback):
        self.active_frame.position = Position(feedback.pose.position)
        self.active_frame.orientation = Orientation(feedback.pose.orientation)


    ## PRINT ##
    ##
    def print_all(self):
        print "> Printing all frames"

        for frame in self.frames:
            frame.print_all()


    ## FILE I/O ##
    ##
    def load_params(self, namespace):

        ## Load yaml file
        data = rosparam.get_param(namespace)

        ## Import data
        for name, frame in data["frames"].items():
            t = frame["position"]
            o = frame["orientation"]

            f = Frame(name, 
                (t["x"], t["y"], t["z"]), 
                (o["x"], o["y"], o["z"], o["w"]), 
                frame["parent"])
            
            self.add_frame(f)

        print "> Loading done"

    def save_params(self):
        ## TODO
        raise NotImplemented()


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

            f = Frame(request.name, Position(request.pose.position), Orientation(request.pose.orientation))
            self.add_frame(f)

        return response


if __name__ == "__main__":

    rospy.init_node('frame_editor')

    editor = Frame_Editor()
    editor.load_params(rospy.get_name())

    print "Frame editor ready!"
    rate = rospy.Rate(100) # hz
    while not rospy.is_shutdown():
        editor.broadcast()
        rate.sleep()

# eof
