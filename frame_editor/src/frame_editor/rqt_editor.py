#!/usr/bin/env python

import os

import rospy
import rospkg
import tf
import actionlib

from qt_gui.plugin import Plugin
from qt_gui_py_common.worker_thread import WorkerThread

from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Signal, Slot

from frame_editor.editor import Frame, FrameEditor, Position, Orientation

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
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns


        ## Editor ##
        ##
        self.editor = FrameEditor()
        self.editor.load_params("frame_editor")
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


        ## Connections ##
        ##
        self._widget.btn_add.clicked.connect(self.btn_add_clicked)
        self._widget.btn_delete.clicked.connect(self.btn_delete_clicked)
        self._widget.list_frames.currentTextChanged.connect(self.selected_frame_changed)
        self._widget.btn_refresh.clicked.connect(self.update_tf_list)

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

        self._update_thread.start()

        self.update_all(3)



    def _update_thread_run(self):
        print "> Going for some spins"
        rate = rospy.Rate(100) # hz
        while not rospy.is_shutdown():
            self.editor.broadcast()
            rate.sleep()

    @Slot()
    def _update_finished(self):
        print "> Shutting down"


    def update(self, editor, level):
        self.signal_update.emit(level)


    @Slot(int)
    def update_all(self, level):
        if level & 1:
            self.update_frame_list()
            self.update_tf_list()

        if level & 2:
            self.update_active_frame()

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
                found = self._widget.list_frames.findItems(item, QtCore.Qt.MatchExactly)
                self._widget.list_frames.takeItem(self._widget.list_frames.row(found[0]))

        self._widget.list_frames.sortItems()

        self.old_frame_list = new_list


    def update_active_frame(self):
        if not self.editor.active_frame:
            self._widget.list_frames.setCurrentItem(None)
            return # deselect and quit

        name = self.editor.active_frame.name
        if name == self.old_selected:
            return # no change

        ## Select item in list
        items = self._widget.list_frames.findItems(name, QtCore.Qt.MatchExactly)
        self._widget.list_frames.setCurrentItem(items[0])

        self.update_fields()

        self.old_selected = name


    def update_fields(self):

        f = self.editor.active_frame
        w = self._widget

        w.txt_name.setText(f.name)
        w.txt_parent.setText(f.parent)

        ## Relative
        w.txt_x.setText(str(f.position[0]))
        w.txt_y.setText(str(f.position[1]))
        w.txt_z.setText(str(f.position[2]))

        rot = tf.transformations.euler_from_quaternion(f.orientation)
        w.txt_a.setText(str(rot[0]))
        w.txt_b.setText(str(rot[1]))
        w.txt_c.setText(str(rot[2]))

        ## Absolute
        (position, orientation) = f.listener.lookupTransform('world', f.name, rospy.Time(0))

        w.txt_abs_x.setText(str(position[0]))
        w.txt_abs_y.setText(str(position[1]))
        w.txt_abs_z.setText(str(position[2]))

        rot = tf.transformations.euler_from_quaternion(orientation)
        w.txt_abs_a.setText(str(rot[0]))
        w.txt_abs_b.setText(str(rot[1]))
        w.txt_abs_c.setText(str(rot[2]))



    @Slot(str)
    def selected_frame_changed(self, name):
        if name == "":
            return
        print name
        self.editor.make_interactive(self.editor.frames[name])


    ## BUTTONS ##
    ##
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

        self.editor.add_frame(Frame(name, parent=parent))
            


    @Slot(bool)
    def btn_delete_clicked(self, checked):
        item = self._widget.list_frames.currentItem()
        if not item:
            return

        self.editor.remove_frame(item.text())


    @Slot(bool)
    def btn_set_parent_rel_clicked(self, checked):
        parent = self._widget.list_tf.currentItem()
        if not parent:
            return # none selected

        if parent.text() == self.editor.active_frame.name:
            return # you can't be your own parent

        self.editor.active_frame.parent = parent.text()

        self.editor.update_frame(self.editor.active_frame)


    @Slot(bool)
    def btn_set_parent_abs_clicked(self, checked):
        parent = self._widget.list_tf.currentItem()
        if not parent:
            return # none selected

        frame = self.editor.active_frame

        if parent.text() == frame.name:
            return # you can't be your own parent

        frame.parent = parent.text()

        ## Set pose
        (position, orientation) = frame.listener.lookupTransform(parent.text(), frame.name, rospy.Time(0))
        frame.position = position
        frame.orientation = orientation

        self.editor.update_frame(frame)


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
        target = self._widget.list_tf.currentItem()
        if not target:
            return # none selected

        frame = self.editor.active_frame

        (position, orientation) = frame.listener.lookupTransform(frame.parent, target.text(), rospy.Time(0))

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

        self.editor.update_frame(frame)


    @Slot(bool)
    def btn_reset_position_rel_clicked(self, checked):
        self.editor.active_frame.position = (0, 0, 0)
        self.editor.update_frame(self.editor.active_frame)

    @Slot(bool)
    def btn_reset_position_abs_clicked(self, checked):
        (position, orientation) = self.editor.active_frame.listener.lookupTransform(self.editor.active_frame.parent, "world", rospy.Time(0))
        self.editor.active_frame.position = position
        self.editor.update_frame(self.editor.active_frame)

    @Slot(bool)
    def btn_reset_orientation_rel_clicked(self, checked):
        self.editor.active_frame.orientation = (0, 0, 0, 1)
        self.editor.update_frame(self.editor.active_frame)

    @Slot(bool)
    def btn_reset_orientation_abs_clicked(self, checked):
        (position, orientation) = self.editor.active_frame.listener.lookupTransform(self.editor.active_frame.parent, "world", rospy.Time(0))
        self.editor.active_frame.orientation = orientation
        self.editor.update_frame(self.editor.active_frame)


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
        frame.parent = result.parent_name
        frame.position = Position(result.pose.position)
        frame.orientation = Orientation(result.pose.orientation)

        self.editor.update_frame(frame)


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

        frame.parent = result.parent_name
        frame.position = Position(result.pose.position)
        frame.orientation = Orientation(result.pose.orientation)

        self.editor.update_frame(frame)


    def action_feedback_callback(self, feedback):
        frame = self.editor.active_frame

        frame.parent = feedback.parent_name
        frame.position = Position(feedback.pose.position)
        frame.orientation = Orientation(feedback.pose.orientation)

        self.editor.update_frame(frame)


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