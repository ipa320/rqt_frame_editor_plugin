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

        ## Marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = ""
        int_marker.name = ""
        int_marker.description = "Frame Editor"

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(rotate_control);

        self.int_marker = int_marker


    def add_frame(self, frame):
        print "> Adding frame:"
        frame.print_all()

        self.frames[frame.name] = frame

    def remove_frame(self, name):
        del self.frames[name]


    def broadcast(self):
        #print "> Broadcasting"
        for frame in self.frames.values():
            self.broadcaster.sendTransform(frame.position, frame.orientation,
                rospy.Time.now(),
                frame.name, frame.parent)


    def make_interactive(self, frame):

        ## Stop currently active frame
        if self.active_frame is not None:
            self.server.remove(self.int_marker)

        self.active_frame = frame

        if frame is None:
            return # nothing more to do here

        self.int_marker.name = frame.name
        self.int_marker.header.frame_id = frame.parent

        self.server.insert(self.int_marker, self.callback_marker)
        self.server.applyChanges()

    def callback_marker(self, feedback):
        print feedback


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
        print "> Request to edit frame"

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
        print "> Request to get frame"

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
        print "> Request to remove frame"

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
        print "> Request to set (or add) frame"

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
