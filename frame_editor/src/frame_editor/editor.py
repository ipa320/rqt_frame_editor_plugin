#!/usr/bin/env python

import os
import sys

import time
import threading
import yaml

import rospy
import rosparam
import rospkg

from frame_editor.objects import *
from frame_editor.commands import *

from frame_editor.constructors_geometry import *
from frame_editor.constructors_std import *

from python_qt_binding import QtCore
from python_qt_binding.QtWidgets import QUndoStack

## Views
from frame_editor.interface_interactive_marker import FrameEditor_InteractiveMarker
from frame_editor.interface_services import FrameEditor_Services
from frame_editor.interface_markers import FrameEditor_Markers
from frame_editor.interface_tf import FrameEditor_TF


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
                if "package" not in dat:
                    dat["package"] = ""

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
                self.update_file_format(frame)
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

    def update_file_format(self, frame):
        if frame.package == "" and frame.path != "":
            try:
                import rospkg
                import os
                from python_qt_binding import QtWidgets
                rospackage = rospkg.get_package_name(frame.path)
                if rospackage is not None:
                    rel_path = os.path.relpath(frame.path , rospkg.RosPack().get_path(rospackage))
                    reply = QtWidgets.QMessageBox.question(None, "Convert absolute path to rospack+relative path?",
                    "The absolute path to your selected mesh can be converted to rospack+relative path."+
                    "This gives you more reliabilaty to reuse your saved configuration"+
                    "if your meshes are stored in rospackages\n\n"+
                    "Do you want to convert your configuration?\n"+
                    "Convert:\n'{}'\nto:\n'{}' and\n '{}'\n".format(frame.path, rospackage, rel_path),
                    QtWidgets.QMessageBox.Yes |
                    QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.Yes)

                    if reply == QtWidgets.QMessageBox.Yes:
                        print "Saving: package:", rospackage, "+ relative path:", rel_path
                        frame.package = rospackage
                        frame.path = rel_path
                        return
            except:
                # Do nothing if conversion fails
                pass
        else:
            # Do nothing if conversion not needed
            pass

    def run(self):
        print "> Going for some spins"
        rate = rospy.Rate(200) # hz
        while not rospy.is_shutdown():
            self.broadcast()
            rate.sleep()

    def parse_args(self, argv):
        ## Args ##
        ##
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        parser.add_argument("-l", "--load", action="append",
                      dest="file",
                      help="Load a file at startup. [rospack filepath/file]")

        args, unknowns = parser.parse_known_args(argv)
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        ## Load file ##
        ##
        if args.file:
            arg_path = args.file[0].split()
            if len(arg_path) == 1:
                #load file
                filename = arg_path[0]
                print "Loading", filename
                success = self.load_file(str(filename))
            elif len(arg_path) == 2:
                #load rospack
                rospack = rospkg.RosPack()
                filename = os.path.join(rospack.get_path(arg_path[0]), arg_path[1])
                print "Loading", filename
                success = self.load_file(str(filename))
            else:
                print "Load argument not understood! --load", arg_path
                print "Please use --load 'myRosPackage pathInMyPackage/myYaml.yaml'"
                print "or use --load 'fullPathToMyYaml.yaml'"
                success = None

            if success:
                return filename
            elif success == False:
                print "ERROR LOADING FILE"
            return ''

    def init_views(self):
        ## Views
        self.interface_tf = FrameEditor_TF(self)
        self.interactive = FrameEditor_InteractiveMarker(self)
        self.services = FrameEditor_Services(self)
        self.interface_markers = FrameEditor_Markers(self)

if __name__ == "__main__":

    rospy.init_node('frame_editor')

    editor = FrameEditor()
    # editor.load_params(rospy.get_name())

    editor.parse_args(sys.argv[1:])
    editor.init_views()

    print "Frame editor ready!"
    editor.run()

# eof
