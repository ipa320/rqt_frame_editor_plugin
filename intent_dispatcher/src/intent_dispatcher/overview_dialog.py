#!/usr/bin/python

## Python ##
##
import os
import sys
import thread
import threading
import time

## ROS ##
##
import rospy
import rospkg

from qt_gui.plugin import Plugin

## Qt ##
##
from python_qt_binding import loadUi
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding.QtCore import Qt

from intent_dispatcher import dispatcher
from intent_dispatcher import yaml_io
from intent_dispatcher.provider_wizard import ProviderWizard

from intent_dispatcher.srv import *

## ToDo make actual service calls instead of directly calling dispatcher functions


class Overview_Dialog(Plugin):

    def __init__(self, context):
        super(Overview_Dialog, self).__init__(context)

        self.setObjectName('Intent Dispatcher Overview')

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


        ## Dispatcher ##
        ##
        self.disp = dispatcher.Dispatcher()
        #self.disp.set_chooser(widget.choose_provider)


        ## Load settings ##
        ##
        self.filename = rospy.get_param('~filename', None)
        if self.filename:
            self.load_file(self.filename)


        self.row = -1


        ## Load UI file ##
        ##
        self._widget = QtGui.QWidget()
        self._widget.setObjectName('Intent Dispatcher Overview')
        ui_file = os.path.join(rospkg.RosPack().get_path('intent_dispatcher'), 'ui', 'overview_dialog.ui')
        ui = loadUi(ui_file, self._widget)
        self.ui = ui

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        ## Undo View
        self._widget.undo_frame.layout().addWidget(QtGui.QUndoView(self.disp.undo_stack))

        ## Table ##
        ##
        table = self.ui.table_services
        table.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QtGui.QAbstractItemView.SingleSelection)
        table.setSelectionBehavior(QtGui.QAbstractItemView.SelectRows)
        table.verticalHeader().setDefaultSectionSize(80);
        table.currentCellChanged.connect(self.provider_selected)

        ## GUI ##
        ##
        ui.btnAddTool.clicked.connect(self.btn_add_tool_cb)
        ui.btnAddAction.clicked.connect(self.btn_add_action_cb)
        ui.btnAddService.clicked.connect(self.btn_add_service_cb)
        ui.btn_saveAs.clicked.connect(self.on_btn_save_as_clicked)
        ui.btn_save.clicked.connect(self.on_btn_save_clicked)
        ui.btn_open.clicked.connect(self.on_btn_open_clicked)

        self.update_table()



    ## FILE I/O ##
    ##
    def load_file(self, filename, namespace = None):

        ## ToDo: clean up

        self.disp.undo_stack.beginMacro("Import file")

        if filename:
            yaml_io.import_yaml(self.disp, filename)
            self.filename = filename

        self.disp.undo_stack.endMacro()

    @QtCore.Slot()
    def on_btn_save_as_clicked(self):
        ## Ask for filename
        saveDialog = QtGui.QFileDialog(self._widget, 'Save settings', QtCore.QDir.currentPath(), 'YAML files(*.yaml)')
        saveDialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
        saveDialog.selectFile(self.filename)
        if (saveDialog.exec_()):
            filename = str(saveDialog.selectedFiles()[0])
            yaml_io.export_yaml(self.disp, filename)
            self.filename = filename

    @QtCore.Slot()
    def on_btn_save_clicked(self):
        if self.filename:
            yaml_io.export_yaml(self.disp, self.filename)

    @QtCore.Slot()
    def on_btn_open_clicked(self):
        files = QtGui.QFileDialog.getOpenFileNames(self._widget,
            "Select one or more files to open", "",
            "YAML files(*.yaml)");

        for filename in files:
            self.load_file(str(filename))

        if len(files) == 1:
            self.filename = files[0]
        else:
            self.filename = None


    @QtCore.Slot(int)
    def provider_selected(self, currentRow):
        pass


    @QtCore.Slot(int)
    def btn_add_tool_cb(self):
        print "add tool"


    @QtCore.Slot()
    def btn_add_action_cb(self):
        print "add action"


    @QtCore.Slot()
    def btn_add_service_cb(self):
        print "add service"
        wiz = ProviderWizard()
        wiz.update_table()
        wiz.exec_()

        print "Selected:", wiz.selected_service

        if wiz.selected_service is not None:

            service_type = wiz.selected_service[1].split("/")

            ## Find all tools that fit ##
            ##
            tool_list = []
            for proxy in self.disp.proxies.values():
                if proxy.proxy_type_module == service_type[0] and proxy.proxy_type_name == service_type[1]:
                    tool_list.append(proxy.proxy_name)


            if not tool_list:
                ## Add a new tool ##
                ##
                request = AddProxy()
                request.proxy_type = "service" # "action" or "service"
                request.proxy_type_module = service_type[0]  # e.g. "std_srvs"
                request.proxy_type_name = service_type[1] # e.g. "Empty"

                input_dialog = QtGui.QInputDialog()
                input_dialog.setComboBoxItems(tool_list)
                input_dialog.setLabelText("Select tool for this provider:")
                input_dialog.setOptions(QtGui.QInputDialog.UseListViewForComboBoxItems)
                done = input_dialog.exec_()
                request.proxy_name = input_dialog.textValue() # e.g. "my_new_service"

                request.name = "..." # human readable name
                request.description = "..." # human readable description
                request.icon_path = ""

                self.disp.add_proxy_callback(request)
                print ">>>", self.disp.proxies

                user_choice = request.proxy_name

            else:
                ## Ask for tool to select ##
                ##
                input_dialog = QtGui.QInputDialog()
                input_dialog.setComboBoxItems(tool_list)
                input_dialog.setLabelText("Select tool for this provider:")
                input_dialog.setOptions(QtGui.QInputDialog.UseListViewForComboBoxItems)

                done = input_dialog.exec_()

                user_choice = input_dialog.textValue()

            print user_choice


            ## Add provider ##
            ##
            request = AddProvider()

            request.provider_type_module = service_type[0] # e.g. "std_srvs"
            request.provider_type_name = service_type[1] # e.g. "Empty"
            request.provider_name = wiz.selected_service[0] # e.g. "my_original_service"
            request.proxy_name = user_choice # e.g. "my_new_service"
            request.name = "..." # human readable name
            request.description = "..." # human readable description
            request.icon_path = ""

            self.disp.add_provider_callback(request)

            self.update_table()


    @QtCore.Slot()
    def update_table(self):
        '''Runs in main Qt thread (necessary for using QIcons)'''

        print "update"

        services = []
        actions = []
        for proxy in self.disp.proxies.values():
            for provider in proxy.providers.values():
                if proxy.proxy_type == "service":
                    services.append(provider)
                elif proxy.proxy_type == "action":
                    actions.append(provider)

        ## Table ##
        ##
        self.ui.table_services.setRowCount(len(services))
        self.ui.table_actions.setRowCount(len(actions))

        i = 0
        for proxy in self.disp.proxies.values():
            for provider in proxy.providers.values():
                if proxy.proxy_type == "service":
                    table = self.ui.table_services
                elif proxy.proxy_type == "action":
                     table = self.ui.table_actions

                table.setItem(i, 0, QtGui.QTableWidgetItem(provider.proxy_name))
                table.setItem(i, 1, QtGui.QTableWidgetItem(provider.provider_name))

                text = "Package: " + provider.provider_type_module + "\nType: " + provider.provider_type_name
                table.setItem(i, 2, QtGui.QTableWidgetItem(text))

                table.setItem(i, 3, QtGui.QTableWidgetItem("..."))

                i = i+1

        #self.adjustSize()



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



## Qt ##
##
def qt_loop():
    global widget

    app = QtGui.QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False) # runs in background and doesn't quit once a dialog is closed

    ## Widget (must be created in the main Qt thread) ##
    ##
    widget = Overview_Dialog()
    widget.resize(1024, 768)
    #widget.move(300, 300)
    widget.setWindowTitle('Intent Dispatcher')
    widget.setModal(True)

    app.exec_()



## Main ##
##
def main():

    ## ROS ##
    ##
    rospy.init_node('intent_dispatcher_gui')


    ## Qt ##
    ##
    thread.start_new_thread( qt_loop, () )


    ## For Testing ##
    ##
    time.sleep(1)
    QtCore.QMetaObject.invokeMethod(widget, "show", Qt.QueuedConnection);


    ## ROS loop ##
    ##
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

    return


if __name__ == '__main__':
    main()

# eof
