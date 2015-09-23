#!/usr/bin/env python

import os
import math

import rospy
import rospkg
import tf
import actionlib

from qt_gui.plugin import Plugin
from qt_gui_py_common.worker_thread import WorkerThread

from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Signal, Slot

from frame_editor.editor import Frame, FrameEditor
from frame_editor.commands import *
from frame_editor.constructors_geometry import *

from toolbox.msg import *
from toolbox.srv import *


class FrameEditorGUI(Plugin):

    signal_update = QtCore.Signal(int)

    def __init__(self, context):
        super(FrameEditorGUI, self).__init__(context)

        self.setObjectName('FrameEditorGUI')

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

        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        ## Editor ##
        ##
        self.namespace = "frame_editor"
        self.filename = ""

        self.editor = FrameEditor()
        #self.editor.load_params(self.namespace)
        self.editor.observers.append(self)

        self.signal_update.connect(self.update_all)

        self._update_thread = WorkerThread(self._update_thread_run, self._update_finished)

        self.old_frame_list = []
        self.old_selected = ""

        ## Create QWidget ##
        ##
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('frame_editor'), 'src/frame_editor', 'FrameEditorGUI.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('FrameEditorGUIUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        ## Undo View
        self._widget.undo_frame.layout().addWidget(QtGui.QUndoView(self.editor.undo_stack))


        ## Load file ##
        ##
        if args.file:
            arg_path = args.file[0].split()
            if len(arg_path) == 1:
                #load file
                filename = arg_path[0]
                print "Loading", filename
                self.editor.load_file(str(filename), self.namespace)
            elif len(arg_path) == 2:
                #load rospack
                rospack = rospkg.RosPack()
                filename = os.path.join(rospack.get_path(arg_path[0]), arg_path[1])
                print "Loading", filename
                self.editor.load_file(str(filename), self.namespace)
            else:
                print "Load argument not understood! --load", arg_path
                print "Please use --load 'myRosPackage pathInMyPackage/myYaml.yaml'"
                print "or use --load 'fullPathToMyYaml.yaml'"


        ## Connections ##
        ##
        self._widget.btn_open.clicked.connect(self.open_file)
        self._widget.btn_save.clicked.connect(self.save_file)
        self._widget.btn_saveAs.clicked.connect(self.save_as_file)

        self._widget.btn_add.clicked.connect(self.btn_add_clicked)
        self._widget.btn_delete.clicked.connect(self.btn_delete_clicked)
        self._widget.list_frames.currentTextChanged.connect(self.selected_frame_changed)
        self._widget.btn_refresh.clicked.connect(self.update_tf_list)
        self._widget.btn_clear.clicked.connect(self.clear_all)

        self._widget.btn_set_parent_rel.clicked.connect(self.btn_set_parent_rel_clicked)
        self._widget.btn_set_parent_abs.clicked.connect(self.btn_set_parent_abs_clicked)
        self._widget.btn_set_pose.clicked.connect(self.btn_set_pose_clicked)
        self._widget.btn_set_position.clicked.connect(self.btn_set_position_clicked)
        self._widget.btn_set_orientation.clicked.connect(self.btn_set_orientation_clicked)
        self._widget.btn_set_x.clicked.connect(self.btn_set_x_clicked)
        self._widget.btn_set_y.clicked.connect(self.btn_set_y_clicked)
        self._widget.btn_set_z.clicked.connect(self.btn_set_z_clicked)
        self._widget.btn_set_a.clicked.connect(self.btn_set_a_clicked)
        self._widget.btn_set_b.clicked.connect(self.btn_set_b_clicked)
        self._widget.btn_set_c.clicked.connect(self.btn_set_c_clicked)

        self._widget.btn_reset_position_rel.clicked.connect(self.btn_reset_position_rel_clicked)
        self._widget.btn_reset_position_abs.clicked.connect(self.btn_reset_position_abs_clicked)
        self._widget.btn_reset_orientation_rel.clicked.connect(self.btn_reset_orientation_rel_clicked)
        self._widget.btn_reset_orientation_abs.clicked.connect(self.btn_reset_orientation_abs_clicked)

        self._widget.btn_call_service.clicked.connect(self.btn_call_service_clicked)
        self._widget.btn_call_action.clicked.connect(self.btn_call_action_clicked)

        self._widget.txt_x.editingFinished.connect(self.x_valueChanged)
        self._widget.txt_y.editingFinished.connect(self.y_valueChanged)
        self._widget.txt_z.editingFinished.connect(self.z_valueChanged)
        self._widget.txt_a.editingFinished.connect(self.a_valueChanged)
        self._widget.txt_b.editingFinished.connect(self.b_valueChanged)
        self._widget.txt_c.editingFinished.connect(self.c_valueChanged)

        self._widget.btn_rad.toggled.connect(self.update_fields)

        self._widget.combo_style.currentIndexChanged.connect(self.frame_style_changed)
        self._widget.btn_style_color.clicked.connect(self.btn_style_color_clicked)
        self._widget.btn_style_color.setEnabled(False)

        self._update_thread.start()

        self.update_all(3)



    def _update_thread_run(self):
        print "> Going for some spins"
        rate = rospy.Rate(200) # hz
        while not rospy.is_shutdown():
            self.editor.broadcast()
            rate.sleep()

    @Slot()
    def _update_finished(self):
        print "> Shutting down"


    def update(self, editor, level, elements):
        self.signal_update.emit(level)


    @Slot(int)
    def update_all(self, level):
        ## Update list widgets
        if level & 1:
            self.update_frame_list()
            self.update_tf_list()

        ## Update the currently selected frame
        if level & 2:
            self.update_active_frame()

        ## Update only text fields, spin boxes,...
        if level & 4:
            self.update_fields()


    @Slot()
    def update_tf_list(self):
        self._widget.list_tf.clear()
        self._widget.list_tf.addItems(self.editor.get_tf_frames())
        self._widget.list_tf.sortItems()

    def update_frame_list(self):
        new_list = self.editor.frames.keys()

        ## Add missing
        items = []
        for item in new_list:
            if item not in self.old_frame_list:
                items.append(item)
        self._widget.list_frames.addItems(items)

        ## Delete removed
        for item in self.old_frame_list:
            if item not in new_list:
                if self._widget.list_frames.currentItem() and item == self._widget.list_frames.currentItem().text():
                    self._widget.list_frames.setCurrentItem(None)
                found = self._widget.list_frames.findItems(item, QtCore.Qt.MatchExactly)
                self._widget.list_frames.takeItem(self._widget.list_frames.row(found[0]))

        self._widget.list_frames.sortItems()

        self.old_frame_list = new_list


    def update_active_frame(self):
        if not self.editor.active_frame:
            self.old_selected = ""
            self._widget.list_frames.setCurrentItem(None)
            self._widget.box_edit.setEnabled(False)
            return # deselect and quit

        self._widget.box_edit.setEnabled(True)

        name = self.editor.active_frame.name
        if name == self.old_selected:
            return # no change

        ## Select item in list
        items = self._widget.list_frames.findItems(name, QtCore.Qt.MatchExactly)
        self._widget.list_frames.setCurrentItem(items[0])

        self.update_fields()

        self.old_selected = name


    @Slot()
    def update_fields(self):

        f = self.editor.active_frame
        if not f:
            return

        w = self._widget

        w.txt_name.setText(f.name)
        w.txt_parent.setText(f.parent)

        ## Relative
        w.txt_x.setValue(f.position[0])
        w.txt_y.setValue(f.position[1])
        w.txt_z.setValue(f.position[2])

        rot = tf.transformations.euler_from_quaternion(f.orientation)
        if self._widget.btn_deg.isChecked():
            rot = (180.0*rot[0]/math.pi, 180.0*rot[1]/math.pi, 180.0*rot[2]/math.pi)

        w.txt_a.setValue(rot[0])
        w.txt_b.setValue(rot[1])
        w.txt_c.setValue(rot[2])

        ## Absolute
        (position, orientation) = f.listener.lookupTransform('world', f.name, rospy.Time(0))
        ## TODO, tf is sometimes too slow! values may still be the old ones

        w.txt_abs_x.setValue(position[0])
        w.txt_abs_y.setValue(position[1])
        w.txt_abs_z.setValue(position[2])

        rot = tf.transformations.euler_from_quaternion(orientation)
        if self._widget.btn_deg.isChecked():
            rot = (180.0*rot[0]/math.pi, 180.0*rot[1]/math.pi, 180.0*rot[2]/math.pi)
        w.txt_abs_a.setValue(rot[0])
        w.txt_abs_b.setValue(rot[1])
        w.txt_abs_c.setValue(rot[2])

        ## Style
        self._widget.combo_style.setCurrentIndex(self._widget.combo_style.findText(f.style))


    @Slot(str)
    def selected_frame_changed(self, name):
        if name == "":
            return

        if not self.editor.active_frame or (self.editor.active_frame.name != name):
            self.editor.command(Command_SelectElement(self.editor, self.editor.frames[name]))

        if self.editor.active_frame.style != "none":
            self._widget.btn_style_color.setEnabled(True)
        else:
            self._widget.btn_style_color.setEnabled(False)

    ## BUTTONS ##
    ##
    @Slot()
    def open_file(self):
        files = QtGui.QFileDialog.getOpenFileNames(self._widget,
            "Select one or more files to open", "",
            "YAML files(*.yaml)");
        for filename in files:
            self.editor.load_file(str(filename), self.namespace)
        if len(files) == 1:
            self.filename = files[0]

    @Slot()
    def save_file(self):
        if self.filename == "":
            print "No filename set"
        else:
            self.editor.save_file(self.filename, self.namespace)

    @Slot()
    def save_as_file(self):
        filename = str(QtGui.QFileDialog.getSaveFileName(self._widget,
            "Save file as...", self.filename,
            "YAML files(*.yaml)")[0])
        if filename:
            self.filename = filename
            self.save_file()


    @Slot()
    def clear_all(self):
        self.editor.command(Command_ClearAll(self.editor))

    @Slot(bool)
    def btn_add_clicked(self, checked):

        ## Get Name ##
        ##
        name, ok = QtGui.QInputDialog.getText(self._widget, "Add New Frame", "Name:", QtGui.QLineEdit.Normal, "my_frame");

        while True:
            if not ok or name == "":
                return

            if name not in self.editor.frames:
                break

            name, ok = QtGui.QInputDialog.getText(self._widget, "Add New Frame", "Name (must be unique):", QtGui.QLineEdit.Normal, "my_frame");

        ## Get Parent ##
        ##
        all_frames = self.editor.get_tf_frames()
        if not all_frames:
            all_frames = ["world"]
        print all_frames
        parent, ok = QtGui.QInputDialog.getItem(self._widget, "Add New Frame", "Parent Name:", all_frames);

        if not ok or parent == "":
            return

        self.editor.command(Command_AddElement(self.editor, Frame(name, parent=parent)))
            


    @Slot(bool)
    def btn_delete_clicked(self, checked):
        item = self._widget.list_frames.currentItem()
        if not item:
            return
        self.editor.command(Command_RemoveElement(self.editor, self.editor.frames[item.text()]))
        

    ## PARENTING ##
    ##
    @Slot(bool)
    def btn_set_parent_rel_clicked(self, checked):
        self.set_parent(False)

    @Slot(bool)
    def btn_set_parent_abs_clicked(self, checked):
        self.set_parent(True)

    def set_parent(self, keep_absolute):
        parent = self._widget.list_tf.currentItem()
        if not parent:
            return # none selected

        if parent.text() == self.editor.active_frame.name:
            return # you can't be your own parent

        self.editor.command(Command_SetParent(self.editor, self.editor.active_frame, parent.text(), keep_absolute))


    ## SET BUTTONS ##
    ##
    @Slot(bool)
    def btn_set_pose_clicked(self, checked):
        self.set_pose(["x", "y", "z", "a", "b", "c"])

    @Slot(bool)
    def btn_set_position_clicked(self, checked):
        self.set_pose(["x", "y", "z"])

    @Slot(bool)
    def btn_set_orientation_clicked(self, checked):
        self.set_pose(["a", "b", "c"])

    @Slot(bool)
    def btn_set_x_clicked(self, checked):
        self.set_pose(["x"])
    @Slot(bool)
    def btn_set_y_clicked(self, checked):
        self.set_pose(["y"])
    @Slot(bool)
    def btn_set_z_clicked(self, checked):
        self.set_pose(["z"])
    @Slot(bool)
    def btn_set_a_clicked(self, checked):
        self.set_pose(["a"])
    @Slot(bool)
    def btn_set_b_clicked(self, checked):
        self.set_pose(["b"])
    @Slot(bool)
    def btn_set_c_clicked(self, checked):
        self.set_pose(["c"])

    def set_pose(self, mode):
        source = self._widget.list_tf.currentItem()
        if not source:
            return # none selected

        frame = self.editor.active_frame
        self.editor.command(Command_AlignElement(self.editor, frame, source.text(), mode))


    ## RESET BUTTONS ##
    ##
    @Slot(bool)
    def btn_reset_position_rel_clicked(self, checked):
        self.editor.command(Command_SetPosition(self.editor, self.editor.active_frame, (0, 0, 0)))

    @Slot(bool)
    def btn_reset_position_abs_clicked(self, checked):
        (position, orientation) = self.editor.active_frame.listener.lookupTransform(self.editor.active_frame.parent, "world", rospy.Time(0))
        self.editor.command(Command_SetPosition(self.editor, self.editor.active_frame, position))

    @Slot(bool)
    def btn_reset_orientation_rel_clicked(self, checked):
        self.editor.command(Command_SetOrientation(self.editor, self.editor.active_frame, (0, 0, 0, 1)))

    @Slot(bool)
    def btn_reset_orientation_abs_clicked(self, checked):
        (position, orientation) = self.editor.active_frame.listener.lookupTransform(self.editor.active_frame.parent, "world", rospy.Time(0))
        self.editor.command(Command_SetOrientation(self.editor, self.editor.active_frame, orientation))


    ## CALL BUTTONS ##
    ##
    @Slot(bool)
    def btn_call_service_clicked(self, checked):
        '''Calls a service to request pose data'''

        service_name = self._widget.txt_call_service.text()

        if service_name == "":
            print "Error: No service name has been set!"
            return

        frame = self.editor.active_frame

        ## Service request
        request = GetPoseServiceRequest()
        request.frame_name = frame.name
        request.parent_name = frame.parent
        request.default_pose = frame.pose

        service = rospy.ServiceProxy(service_name, GetPoseService)
        result = service(request)
        print result

        ## Check result
        if request.frame_name != result.frame_name:
            print "Names don't match: ", request.frame_name, result.frame_name
            return

        ## Set result
        if frame.parent != result.parent_name:
            self.editor.command(Command_SetParent(self.editor, frame, result.parent_name, False))

        self.editor.command(Command_SetPose(self.editor, frame, FromPoint(result.pose.position, FromQuaternion(result.pose.orientation))))


    @Slot(bool)
    def btn_call_action_clicked(self, checked):
        '''Starts an action to request pose data'''

        action_name = self._widget.txt_call_action.text()
        if action_name == "":
            print "Error: No action name has been set!"
            return

        frame = self.editor.active_frame

        ## Create client
        print "> Waiting for action server"
        client = actionlib.SimpleActionClient(action_name, GetPoseAction)
        client.wait_for_server()

        ## Action request
        goal = GetPoseGoal()
        goal.frame_name = frame.name
        goal.parent_name = frame.parent
        goal.default_pose = frame.pose

        print "> Calling action and waiting for result"
        client.send_goal(goal, feedback_cb=self.action_feedback_callback)
        ok = client.wait_for_result()
        print "> Action completed, result", ok

        if not ok:
            return

        ## Set result
        result = client.get_result()

        if frame.parent != result.parent_name:
            self.editor.command(Command_SetParent(self.editor, frame, result.parent_name, False))

        self.editor.command(Command_SetPose(self.editor, frame, FromPoint(result.pose.position, FromQuaternion(result.pose.orientation))))

    @Slot(bool)
    def btn_style_color_clicked(self, checked):
        color = QtGui.QColorDialog.getColor(QtGui.QColor(255,255,255,255), None, "Select Color", options=QtGui.QColorDialog.ShowAlphaChannel)
        self.editor.command(Command_SetStyleColor(self.editor, self.editor.active_frame, color.getRgbF()))

    def action_feedback_callback(self, feedback):
        frame = self.editor.active_frame

        if frame.parent != result.parent_name:
            self.editor.command(Command_SetParent(self.editor, frame, result.parent_name, False))

        self.editor.command(Command_SetPose(self.editor, frame, FromPoint(result.pose.position, FromQuaternion(result.pose.orientation))))


    ## Spin Boxes ##
    ##
    def set_value(self, widget, symbol):
        frame = self.editor.active_frame
        value = widget.value()

        ## Deg to rad
        if self._widget.btn_deg.isChecked() and symbol in ['a', 'b', 'c']:
            value = value * math.pi / 180.0

        if frame.value(symbol) != value:
            self.editor.command(Command_SetValue(self.editor, self.editor.active_frame, symbol, value))

    @Slot()
    def x_valueChanged(self):
        self.set_value(self._widget.txt_x, 'x')
    @Slot()
    def y_valueChanged(self):
        self.set_value(self._widget.txt_y, 'y')
    @Slot()
    def z_valueChanged(self):
        self.set_value(self._widget.txt_z, 'z')
    @Slot()
    def a_valueChanged(self):
        self.set_value(self._widget.txt_a, 'a')
    @Slot()
    def b_valueChanged(self):
        self.set_value(self._widget.txt_b, 'b')
    @Slot()
    def c_valueChanged(self):
        self.set_value(self._widget.txt_c, 'c')


    ## FRAME STYLE ##
    ##
    @Slot(int)
    def frame_style_changed(self, id):
        style = self._widget.combo_style.currentText().lower()
        if self.editor.active_frame.style != style:
            if style == "mesh":
                path = QtGui.QFileDialog.getOpenFileName(None, 'Open Mesh', '/home', 'Mesh Files (*.stl *.dae)')[0]
            else:
                path = None
            self.editor.command(Command_SetStyle(self.editor, self.editor.active_frame, style, path))

            if style != "none":
                self._widget.btn_style_color.setEnabled(True)
            else:
                self._widget.btn_style_color.setEnabled(False)


    ## PLUGIN ##
    ##
    def shutdown_plugin(self):
        self._update_thread.kill()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

# eof
