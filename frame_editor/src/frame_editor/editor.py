#!/usr/bin/env python

import time
import threading
import yaml

import rospy
import rosparam

from frame_editor.objects import *
from frame_editor.commands import *

from frame_editor.constructors_geometry import *
from frame_editor.constructors_std import *

from python_qt_binding import QtCore
from python_qt_binding.QtWidgets import QUndoStack


class FrameEditor(QtCore.QObject):

    def __init__(self):
        super(FrameEditor, self).__init__()

        self.frames = {}
        self.active_frame = None
        
        ## Undo/Redo
        self.observers = []
        self.undo_level = 0
        self.undo_elements = []
        self.undo_stack = QUndoStack()
        self.undo_stack.indexChanged.connect(self.undo_stack_changed)
        self.__command_lock = threading.Lock()

        self.namespace = "frame_editor"

    ## Undo/Redo ##
    ##
    @QtCore.Slot(int)
    def undo_stack_changed(self, idx):
        '''Updates all observers, whenever a command has been undone/redone'''
        self.update_obsevers(self.undo_level)

    def add_undo_level(self, level, elements=None):
        '''Used by commands to add a level for updating'''
        self.undo_level = self.undo_level | level
        if elements:
            self.undo_elements.extend(elements)

    def command(self, command):
        '''Push a command to the stack (blocking)'''
        with self.__command_lock:
            self.undo_stack.push(command)


    def update_obsevers(self, level):
        '''Updates all registered observers and resets the undo_level'''
        for observer in self.observers:
            observer.update(self, level, self.undo_elements)
        self.undo_level = 0
        self.undo_elements = []

    def broadcast(self):
        for observer in self.observers:
            observer.broadcast(self)

    @staticmethod
    def tf_dict():
        y = Frame.tf_buffer.all_frames_as_yaml()
        d = yaml.load(y)
        if isinstance(d, dict):
            return d
        else:
            rospy.logwarn('Got invalid yaml from tf2: '+y)
            return {}

    @staticmethod
    def frame_is_temporary(frame_id):
        return frame_id.startswith('_')

    @staticmethod
    def all_frame_ids(include_temp=True):
        return [f for f in FrameEditor.tf_dict() if
                not FrameEditor.frame_is_temporary(f) or include_temp]

    def iter_frames(self, include_temp=True):
        for f in self.frames.itervalues():
            if not self.frame_is_temporary(f.name) or include_temp:
                yield f


    ## PRINT ##
    ##
    def print_all(self):
        print "> Printing all frames"

        for frame in self.frames:
            frame.print_all()


    ## FILE I/O ##
    ##
    def load_file(self, file_name):
        if file_name:
            print "> Loading file"
            data = rosparam.load_file(file_name, self.namespace)[0][0]
            self.load_data(data)
        else:
            ## Clear everything
            self.command(Command_ClearAll(self))

        self.undo_stack.clear()

        return True

    def load_params(self, namespace):
        if not rosparam.list_params(namespace):
            print "> No data to load"
        else:
            data = rosparam.get_param(namespace)
            self.load_data(data)

    def load_data(self, data):

        self.undo_stack.beginMacro("Import file")

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
                if "color" in dat:
                    color = dat["color"]
                else:
                    color = (0.0, 0.5, 0.5, 0.75)

            position = (t["x"], t["y"], t["z"])
            orientation = (o["x"], o["y"], o["z"], o["w"])

            if style == "plane":
                f = Object_Plane(name, position, orientation, frame["parent"], dat["length"], dat["width"])
                f.set_color(color)
            elif style == "cube":
                f = Object_Cube(name, position, orientation, frame["parent"], dat["length"], dat["width"], dat["height"])
                f.set_color(color)
            elif style == "sphere":
                f = Object_Sphere(name, position, orientation, frame["parent"], dat["diameter"])
                f.set_color(color)
            elif style == "axis":
                f = Object_Axis(name, position, orientation, frame["parent"], dat["length"], dat["width"])
                f.set_color(color)
            elif style == "mesh":
                f = Object_Mesh(name, position, orientation, frame["parent"], dat["package"], dat["path"], dat["scale"])
                f.set_color(color)
            else:
                f = Frame(name, position, orientation, frame["parent"])
            
            self.command(Command_AddElement(self, f))

        self.undo_stack.endMacro()

        print "> Loading done"

    def save_file(self, filename):

        ## Data
        data = {}
        frames = {}

        for frame in self.iter_frames(include_temp=False):
            t = {}
            t["x"] = float(frame.position[0])
            t["y"] = float(frame.position[1])
            t["z"] = float(frame.position[2])

            o = {}
            o["x"] = float(frame.orientation[0])
            o["y"] = float(frame.orientation[1])
            o["z"] = float(frame.orientation[2])
            o["w"] = float(frame.orientation[3])

            f = {}
            f["parent"] = frame.parent
            f["position"] = t
            f["orientation"] = o

            f["style"] = frame.style

            if frame.style == "plane":
                f["data"] = { "length": frame.length, "width":frame.width, "color": frame.color }
            
            elif frame.style == "cube":
                f["data"] = { "length": frame.length, "width": frame.width, "height": frame.height , "color": frame.color}
            
            elif frame.style == "sphere":
                f["data"] = { "diameter": frame.diameter, "color": frame.color }
            
            elif frame.style == "axis":
                f["data"] = { "length": frame.length, "width": frame.width, "color": frame.color }

            elif frame.style == "mesh":
                f["data"] = { "package" : frame.package, "path" : frame.path, "scale" : frame.scale, "color": frame.color }

            frames[frame.name] = f

        data["frames"] = frames

        ## To parameter server
        rospy.set_param(self.namespace, data)
        print rospy.get_param(self.namespace)

        ## Dump param to file
        print "Saving to file", filename
        rosparam.dump_params(filename, self.namespace)
        print "Saving done"

        return True


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
