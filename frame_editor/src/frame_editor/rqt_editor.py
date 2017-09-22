#!/usr/bin/env python

import os
import math

import rospy
import rospkg
import tf
import actionlib

from qt_gui_py_common.worker_thread import WorkerThread

from python_qt_binding import loadUi, QtCore, QtWidgets
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Slot

from frame_editor.editor import Frame, FrameEditor
from frame_editor.commands import *
from frame_editor.constructors_geometry import *

from project_plugin import ProjectPlugin

from frame_editor.interface import Interface

## Views
from frame_editor.interface_gui import FrameEditor_StyleWidget


class FrameEditorGUI(ProjectPlugin, Interface):

    signal_update = QtCore.Signal(int)

    def __init__(self, context):
        super(FrameEditorGUI, self).__init__(context)

        self.setObjectName('FrameEditorGUI')

        self.file_type = "YAML files(*.yaml)"


        self.filename = ""

        filename = self.editor.parse_args(context.argv())
        if filename:
            self.set_current_file(filename)


        ## Update thread ##
        ##
        self._update_thread.start()
        self.update_all(3)


    def create_editor(self):
        editor = FrameEditor()

        editor.observers.append(self)

        self.signal_update.connect(self.update_all)

        self._update_thread = WorkerThread(self._update_thread_run, self._update_finished)

        self.old_frame_list = []
        self.old_selected = ""

        return editor


    def create_main_widget(self):

        ## Main widget
        widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('frame_editor'), 'src/frame_editor', 'FrameEditorGUI.ui')
        loadUi(ui_file, widget)
        widget.setObjectName('FrameEditorGUIUi')

        #if context.serial_number() > 1:
        #    widget.setWindowTitle(widget.windowTitle() + (' (%d)' % context.serial_number()))


        ## Undo View
        #widget.undo_frame.layout().addWidget(QtWidgets.QUndoView(self.editor.undo_stack))


        ## Views
        self.editor.init_views()
        self.interface_style = FrameEditor_StyleWidget(self.editor)

        widget.style_frame.layout().addWidget(self.interface_style.get_widget())

        ## Connections ##
        ##
        widget.btn_add.clicked.connect(self.btn_add_clicked)
        widget.btn_delete.clicked.connect(self.btn_delete_clicked)
        widget.btn_duplicate.clicked.connect(self.btn_duplicate_clicked)
        widget.list_frames.currentTextChanged.connect(self.selected_frame_changed)
        widget.btn_refresh.clicked.connect(self.update_tf_list)

        widget.btn_set_parent_rel.clicked.connect(self.btn_set_parent_rel_clicked)
        widget.btn_set_parent_abs.clicked.connect(self.btn_set_parent_abs_clicked)
        widget.btn_set_pose.clicked.connect(self.btn_set_pose_clicked)
        widget.btn_set_position.clicked.connect(self.btn_set_position_clicked)
        widget.btn_set_orientation.clicked.connect(self.btn_set_orientation_clicked)
        widget.btn_set_x.clicked.connect(self.btn_set_x_clicked)
        widget.btn_set_y.clicked.connect(self.btn_set_y_clicked)
        widget.btn_set_z.clicked.connect(self.btn_set_z_clicked)
        widget.btn_set_a.clicked.connect(self.btn_set_a_clicked)
        widget.btn_set_b.clicked.connect(self.btn_set_b_clicked)
        widget.btn_set_c.clicked.connect(self.btn_set_c_clicked)

        widget.btn_reset_position_rel.clicked.connect(self.btn_reset_position_rel_clicked)
        widget.btn_reset_position_abs.clicked.connect(self.btn_reset_position_abs_clicked)
        widget.btn_reset_orientation_rel.clicked.connect(self.btn_reset_orientation_rel_clicked)
        widget.btn_reset_orientation_abs.clicked.connect(self.btn_reset_orientation_abs_clicked)

        widget.txt_x.editingFinished.connect(self.x_valueChanged)
        widget.txt_y.editingFinished.connect(self.y_valueChanged)
        widget.txt_z.editingFinished.connect(self.z_valueChanged)
        widget.txt_a.editingFinished.connect(self.a_valueChanged)
        widget.txt_b.editingFinished.connect(self.b_valueChanged)
        widget.txt_c.editingFinished.connect(self.c_valueChanged)

        widget.btn_rad.toggled.connect(self.update_fields)

        widget.combo_style.currentIndexChanged.connect(self.frame_style_changed)

        return widget


    def _update_thread_run(self):
        self.editor.run()

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
        self.widget.list_tf.clear()
        self.widget.list_tf.addItems(
            sorted(self.editor.all_frame_ids(include_temp=False)))

    def update_frame_list(self):
        new_list = self.editor.frames.keys()

        ## Add missing
        items = []
        for item in new_list:
            if item not in self.old_frame_list:
                items.append(item)
        self.widget.list_frames.addItems(items)

        ## Delete removed
        for item in self.old_frame_list:
            if item not in new_list:
                if self.widget.list_frames.currentItem() and item == self.widget.list_frames.currentItem().text():
                    self.widget.list_frames.setCurrentItem(None)
                found = self.widget.list_frames.findItems(item, QtCore.Qt.MatchExactly)
                self.widget.list_frames.takeItem(self.widget.list_frames.row(found[0]))

        self.widget.list_frames.sortItems()

        self.old_frame_list = new_list


    def update_active_frame(self):
        if not self.editor.active_frame:
            self.old_selected = ""
            self.widget.list_frames.setCurrentItem(None)
            self.widget.box_edit.setEnabled(False)
            return # deselect and quit

        self.widget.box_edit.setEnabled(True)

        name = self.editor.active_frame.name
        if name == self.old_selected:
            return # no change

        ## Select item in list
        items = self.widget.list_frames.findItems(name, QtCore.Qt.MatchExactly)
        self.widget.list_frames.setCurrentItem(items[0])

        self.update_fields()

        self.old_selected = name


    @Slot()
    def update_fields(self):

        f = self.editor.active_frame
        if not f:
            return

        w = self.widget

        w.txt_name.setText(f.name)
        w.txt_parent.setText(f.parent)

        ## Relative
        w.txt_x.setValue(f.position[0])
        w.txt_y.setValue(f.position[1])
        w.txt_z.setValue(f.position[2])

        rot = tf.transformations.euler_from_quaternion(f.orientation)
        if self.widget.btn_deg.isChecked():
            rot = (180.0*rot[0]/math.pi, 180.0*rot[1]/math.pi, 180.0*rot[2]/math.pi)

        w.txt_a.setValue(rot[0])
        w.txt_b.setValue(rot[1])
        w.txt_c.setValue(rot[2])

        txt_abs_pos = (w.txt_abs_x, w.txt_abs_y, w.txt_abs_z)
        txt_abs_rot = (w.txt_abs_a, w.txt_abs_b, w.txt_abs_c)

        ## Absolute
        try:
            position, orientation = FromTransformStamped(
                f.tf_buffer.lookup_transform('world', f.name, rospy.Time(0)))
            for txt, p in zip(txt_abs_pos, position):
                txt.setEnabled(True)
                txt.setValue(p)
            rot = tf.transformations.euler_from_quaternion(orientation)
            if self.widget.btn_deg.isChecked():
                rot = map(math.degrees, rot)
            for txt, r in zip(txt_abs_rot, rot):
                txt.setEnabled(True)
                txt.setValue(r)
        except:
            for txt in txt_abs_rot + txt_abs_pos:
                txt.setEnabled(False)

        ## Style
        self.widget.combo_style.setCurrentIndex(self.widget.combo_style.findText(f.style))


    @Slot(str)
    def selected_frame_changed(self, name):
        if name == "":
            return

        if not self.editor.active_frame or (self.editor.active_frame.name != name):
            self.editor.command(Command_SelectElement(self.editor, self.editor.frames[name]))


    ## BUTTONS ##
    ##
    def write_file(self, file_name):
        return self.editor.save_file(file_name)


    @Slot()
    def clear_all(self):
        self.editor.command(Command_ClearAll(self.editor))

    @Slot(bool)
    def btn_add_clicked(self, checked):
        # Get a unique frame name
        existing_frames = set(self.editor.all_frame_ids())

        name, ok = QtWidgets.QInputDialog.getText(self.widget, "Add New Frame", "Name:", QtWidgets.QLineEdit.Normal, "my_frame");

        while ok and name in existing_frames:
            name, ok = QtWidgets.QInputDialog.getText(self.widget, "Add New Frame", "Name (must be unique):", QtWidgets.QLineEdit.Normal, "my_frame")
        if not ok:
            return

        if not existing_frames:
            available_parents = ["world"]
        else:
            available_parents = self.editor.all_frame_ids(include_temp=False)
        parent, ok = QtWidgets.QInputDialog.getItem(self.widget, "Add New Frame", "Parent Name:", sorted(available_parents))


        if not ok or parent == "":
            return

        self.editor.command(Command_AddElement(self.editor, Frame(name, parent=parent)))



    @Slot(bool)
    def btn_duplicate_clicked(self, checked):
        item = self.widget.list_frames.currentItem()
        if not item:
            return
        source_name = item.text()
        parent_name = self.editor.frames[source_name].parent

        # Get a unique frame name
        existing_frames = set(self.editor.all_frame_ids())

        name, ok = QtWidgets.QInputDialog.getText(self.widget, "Duplicate Frame", "Name:", QtWidgets.QLineEdit.Normal, source_name);

        while ok and name in existing_frames:
            name, ok = QtWidgets.QInputDialog.getText(self.widget, "Duplicate Frame", "Name (must be unique):", QtWidgets.QLineEdit.Normal, source_name)
        if not ok:
            return

        self.editor.command(Command_CopyElement(self.editor, name, source_name, parent_name))



    @Slot(bool)
    def btn_delete_clicked(self, checked):
        item = self.widget.list_frames.currentItem()
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
        parent = self.widget.list_tf.currentItem()
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
        source = self.widget.list_tf.currentItem()
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
        position, orientation = FromTransformStamped(
            self.editor.active_frame.tf_buffer.lookup_transform(
                self.editor.active_frame.parent, "world", rospy.Time(0)))
        self.editor.command(Command_SetPosition(self.editor, self.editor.active_frame, position))

    @Slot(bool)
    def btn_reset_orientation_rel_clicked(self, checked):
        self.editor.command(Command_SetOrientation(self.editor, self.editor.active_frame, (0, 0, 0, 1)))

    @Slot(bool)
    def btn_reset_orientation_abs_clicked(self, checked):
        position, orientation = FromTransformStamped(
            self.editor.active_frame.listener.lookupTransform(
                self.editor.active_frame.parent, "world", rospy.Time(0)))
        self.editor.command(Command_SetOrientation(self.editor, self.editor.active_frame, orientation))


    ## Spin Boxes ##
    ##
    def set_value(self, widget, symbol):
        frame = self.editor.active_frame
        value = widget.value()

        ## Deg to rad
        if self.widget.btn_deg.isChecked() and symbol in ['a', 'b', 'c']:
            value = value * math.pi / 180.0

        if frame.value(symbol) != value:
            self.editor.command(Command_SetValue(self.editor, self.editor.active_frame, symbol, value))

    @Slot()
    def x_valueChanged(self):
        self.set_value(self.widget.txt_x, 'x')
    @Slot()
    def y_valueChanged(self):
        self.set_value(self.widget.txt_y, 'y')
    @Slot()
    def z_valueChanged(self):
        self.set_value(self.widget.txt_z, 'z')
    @Slot()
    def a_valueChanged(self):
        self.set_value(self.widget.txt_a, 'a')
    @Slot()
    def b_valueChanged(self):
        self.set_value(self.widget.txt_b, 'b')
    @Slot()
    def c_valueChanged(self):
        self.set_value(self.widget.txt_c, 'c')


    ## FRAME STYLE ##
    ##
    @Slot(int)
    def frame_style_changed(self, id):
        style = self.widget.combo_style.currentText().lower()
        if self.editor.active_frame.style != style:
            self.editor.command(Command_SetStyle(self.editor, self.editor.active_frame, style))


    ## PLUGIN ##
    ##
    def shutdown_plugin(self):
        super(FrameEditorGUI, self).shutdown_plugin()
        self._update_thread.kill()

# eof
