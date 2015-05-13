#!/usr/bin/env python

import os

import rospy
import rospkg
import tf

from qt_gui.plugin import Plugin
from qt_gui_py_common.worker_thread import WorkerThread

from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Signal, Slot

from frame_editor.editor import Frame, FrameEditor


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
        (position, orientation) = self.editor.listener.lookupTransform('world', f.name, rospy.Time(0))

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