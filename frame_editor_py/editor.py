#!/usr/bin/env python3

import os
import sys

import time
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python import get_package_share_directory

from frame_editor_py.objects import *
from frame_editor_py.commands import *

from frame_editor_py.constructors_geometry import *
from frame_editor_py.constructors_std import *

from python_qt_binding import QtCore
from python_qt_binding.QtWidgets import QUndoStack

## Views
from frame_editor_py.interface_interactive_marker import FrameEditor_InteractiveMarker
from frame_editor_py.interface_services import FrameEditor_Services
from frame_editor_py.interface_markers import FrameEditor_Markers
from frame_editor_py.interface_tf import FrameEditor_TF
import rclpy.logging as logging
import random

class FrameEditor(QtCore.QObject):

    def __init__(self, context):
        self.static = FrameEditor.parse_args_static(context.argv())

        # super(FrameEditor, self).__init__('frame_editor')  # Initialize ROS 2 Node
        random_suffix = str(random.random()).replace('.', '')  # Convert to string and remove the decimal point
        self.node = Node(f"frame_editor_{random_suffix}")
        self.node.get_logger().info("now init Frame!")
        Frame.init_tf(self.node,self.static)

        super(FrameEditor, self).__init__()  #
        
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
        self.full_file_path = None
        self.hz = 200
        self.filter_style = "hide"


    def get_file_name(self):
        if self.full_file_path is None:
            return ""
        else:
            return os.path.basename(self.full_file_path)

    def get_full_file_path(self):
        if self.full_file_path is None:
            return ""
        else:
            return self.full_file_path

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
        d = yaml.safe_load(y)
        # logging.get_logger("frame_editor").warn(f'{Frame.tf_buffer.frame_id}')
        if isinstance(d, dict):
            return d
        if isinstance(d, list) and len(d) == 0:
            return {}
        else:
            logging.get_logger("frame_editor").warn('Got invalid yaml from tf2: '+y)
            return {}

    @staticmethod
    def frame_is_temporary(frame_id):
        return frame_id.startswith('_')

    @staticmethod
    def all_frame_ids(include_temp=True):
        print(FrameEditor.tf_dict())
        return [f for f in FrameEditor.tf_dict() if
                not FrameEditor.frame_is_temporary(f) or include_temp]

    def all_editor_frame_ids(self, include_temp=True):
        return [f for f in self.frames.keys() if
                not FrameEditor.frame_is_temporary(f) or include_temp]

    def iter_frames(self, include_temp=True):
        for f in self.frames.values():
            if not self.frame_is_temporary(f.name) or include_temp:
                yield f


    ## PRINT ##
    ##
    def print_all(self):
        self.node.get_logger().info("> Printing all frames")

        for frame in self.frames:
            frame.print_all(self.node)


    ## FILE I/O ##
    ##
    def load_file(self, file_name):
        if file_name:
            self.node.get_logger().info(f"> Loading file {file_name}")
            data = yaml.safe_load(open(file_name, 'r'))
            # data = self.node.get_parameter([Parameter(self.namespace, value=data)])  # ROS 2 - To set parameter
            self.load_data(data)
        else:
            ## Clear everything
            self.command(Command_ClearAll(self))

        self.undo_stack.clear()

        self.full_file_path = file_name
        return True

    def load_params(self, namespace):
        params = self.node.get_parameters_by_prefix(self.namespace)
        if not params:
            self.node.get_logger().info("> No data to load")
        else:
            data = params[0].value
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
                
            if "group" in frame:
                group = frame["group"]
            else:
                group = ""

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
                f = Frame(name, position, orientation, frame["parent"], group=group)

            self.command(Command_AddElement(self, f))

        self.undo_stack.endMacro()

        self.node.get_logger().info("> Loading done")

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
            f["group"] = frame.group

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
        self.node.set_parameters([Parameter(self.namespace, value=data)])

        # Getting the parameter and printing its value
        param = self.get_parameter(self.namespace)
        self.node.get_logger().info(param.value)
        
        ## Dump param to file
        if filename == '':
            filename = self.full_file_path
        self.node.get_logger().info("Saving to file {}".format(filename))
        
        parameters = self.node.get_parameters_by_prefix(self.namespace)
        for param in parameters:
            if isinstance(param, Parameter):
                data[param.name] = param.value
            else:
                self.node.get_logger().info(f"Unknown parameter type: {type(param)}")
                
        with open(filename, 'w') as file:
            yaml.dump(data, file)
            
        
        self.node.get_logger().info("Saving done")

        self.full_file_path = filename
        return True

    def update_file_format(self, frame):
        if frame.package == "" and frame.path != "":
            try:
                from ament_index_python import get_package_share_directory
                import os
                from python_qt_binding import QtWidgets
                # Example use: get the path of the package
                rospackage = get_package_share_directory(frame.package)  # Getting package share directory in ROS 2
                if rospackage is not None:
                    rel_path = os.path.relpath(frame.path, rospackage)
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
                        self.node.get_logger().info("Saving: package: {} + relative path: {}".format(rospackage, rel_path))
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
        if not self.static:
            self.node.get_logger().info("> Going for some spins with rate {}".format(self.hz))
        else:
            self.node.get_logger().info("> Going to static broadcaster")
        rate = self.node.create_rate(self.hz) # hz
        while rclpy.ok():
            self.broadcast()
            rate.sleep()
        
        self.node.get_logger().info("> Shutting down Frameeditor")
        rclpy.shutdown()
            

    @staticmethod
    def parse_args_static(argv):
        from argparse import ArgumentParser
        parser = ArgumentParser()
        static = False
        parser.add_argument("-s", "--static", action="store_true", help="Use static tf broadcaster") 
        args, unknowns = parser.parse_known_args(argv)
        static = args.static
        return static

    def parse_args(self, argv):
        ## Args ##
        ##
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        #parser.add_argument("-q", "--quiet", action="store_true",
        #              dest="quiet",
        #              help="Put plugin in silent mode")
        parser.add_argument("-l", "--load", action="append",
                      dest="file",
                      help="Load a file at startup. [rospack filepath/file]")
        parser.add_argument(
            "--filter_style",  
            type=str,  
            choices=["grey", "hide"],  
            help="Choose the filter style: 'grey' or 'hide' (default: 'hide')",
            default="hide",  
        )        
        
        parser.add_argument("-r", "--rate", type=int, help="Rate for broadcasting. Does not involve tf frames. Only effective for non-static broadcaster.")
        parser.add_argument("-s", "--static", action="store_true", help="Use static tf broadcaster") 

        
        if '--ros-args' in argv:
            argv = argv[:argv.index('--ros-args')]
        args, unknowns = parser.parse_known_args(argv)
        self.node.get_logger().info('arguments: {}'.format(args))
        if unknowns:
            self.node.get_logger().info('unknown parameters found: {}'.format(unknowns))

        self.static = args.static

        if args.rate:
            self.hz = args.rate
        else:
            self.hz = 100
            
        if args.filter_style:
            self.filter_style = args.filter_style

        ## Load file ##
        if args.file:
            arg_path = args.file[0].split()
            if len(arg_path) == 1:
                #load file
                filename = arg_path[0]
                self.node.get_logger().info("Loading {}".format(filename))
                success = self.load_file(str(filename))
            elif len(arg_path) == 2:
                #load rospack
                
                package_name = arg_path[0]
                file_path_in_package = arg_path[1]
                package_share_directory = get_package_share_directory(package_name)
                filename = os.path.join(package_share_directory, file_path_in_package)

                self.node.get_logger().info("Loading {}".format(filename))
                success = self.load_file(str(filename))
            else:
                self.node.get_logger().error("Load argument not understood! --load {}".format(arg_path))
                self.node.get_logger().error("Please use --load 'myRosPackage pathInMyPackage/myYaml.yaml'")
                self.node.get_logger().error("or use --load 'fullPathToMyYaml.yaml'")
                success = None

            if success:
                return filename
            elif success == False:
                self.node.get_logger().error("ERROR LOADING FILE")
            return ''

    def init_views(self):
        ## Views
        self.interface_tf = FrameEditor_TF(self)
        self.interactive = FrameEditor_InteractiveMarker(self)
        self.services = FrameEditor_Services(self)
        self.interface_markers = FrameEditor_Markers(self)
    
    def shutdown(self):
        """Clean up and shut down the ROS 2 system."""
        self.node.get_logger().info("Shutting down FrameNode node")
        rclpy.shutdown()


if __name__ == "__main__":

    rclpy.init()
    editor = FrameEditor(sys.argv[1:])

    editor.parse_args(sys.argv[1:])
    editor.init_views()

    editor.node.get_logger().info("Frame editor ready!")
    editor.run()

# eof
