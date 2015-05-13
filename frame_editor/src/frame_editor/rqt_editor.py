#!/usr/bin/env python

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from qt_gui_py_common.worker_thread import WorkerThread

from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Signal, Slot

from frame_editor.editor import Frame, FrameEditor


class FrameEditorGUI(Plugin):

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

        self._update_thread = WorkerThread(self._update_thread_run, self._update_finished)


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


        self._update_thread.start()


    def _update_thread_run(self):
        print "> Going for some spins"
        rate = rospy.Rate(100) # hz
        while not rospy.is_shutdown():
            self.editor.broadcast()
            rate.sleep()

    @Slot()
    def _update_finished(self):
        print "> Shutting down"


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
        print "NARF2"

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