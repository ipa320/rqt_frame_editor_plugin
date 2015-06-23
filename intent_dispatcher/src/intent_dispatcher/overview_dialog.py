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
from python_qt_binding.QtCore import Signal, Slot

from intent_dispatcher import dispatcher
from intent_dispatcher import yaml_io
from intent_dispatcher.provider_wizard import ProviderWizard
from intent_dispatcher.proxy_wizard import ProxyWizard

from intent_dispatcher.srv import *
from intent_dispatcher.commands import *

## ToDo make actual service calls instead of directly calling dispatcher functions


class Overview_Dialog(Plugin):

    signal_update = QtCore.Signal(int)

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
        self.disp.observers.append(self)

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
        ui.btn_add_service_tool.clicked.connect(self.btn_add_service_tool_cb)
        ui.btn_add_service.clicked.connect(self.btn_add_service_cb)
        ui.btn_delete_service.clicked.connect(self.btn_delete_service_cb)

        ui.btn_add_action_tool.clicked.connect(self.btn_add_action_tool_cb)
        ui.btn_add_action.clicked.connect(self.btn_add_action_cb)
        ui.btn_delete_action.clicked.connect(self.btn_delete_action_cb)
        
        ui.btn_saveAs.clicked.connect(self.on_btn_save_as_clicked)
        ui.btn_save.clicked.connect(self.on_btn_save_clicked)
        ui.btn_open.clicked.connect(self.on_btn_open_clicked)

        self.signal_update.connect(self.update_all)

        self.update_table()


    def update(self, dispatcher, level, elements):
        self.signal_update.emit(level)

    @Slot(int)
    def update_all(self, level):
        if level & 1:
            self.update_table()

        #if level & 2:



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


    ## ADD/DELETE BUTTONS ##
    ##
    @QtCore.Slot()
    def btn_add_service_tool_cb(self):

        ## Select type
        wiz = ProxyWizard(is_action=False)
        wiz.update_table()
        wiz.exec_()

        print "Selected:", wiz.selected_element

        ## Get Name
        name, ok = QtGui.QInputDialog.getText(self._widget, "Add New Service Tool", "Name:", QtGui.QLineEdit.Normal, "my_service_tool");
        while True:
            if not ok or name == "":
                return
            elif name not in self.disp.proxies:
                break
            else:
                name, ok = QtGui.QInputDialog.getText(self._widget, "Add New Service Tool", "Name (must be unique):", QtGui.QLineEdit.Normal, name);

        ## Add proxy
        (proxy_type_module, proxy_type_name) = wiz.selected_element[1].split("/")

        request = Proxy_Request("service", proxy_type_module, proxy_type_name, name, name, "...", None)
        self.disp.command(Command_AddProxy(self.disp, request))


    @QtCore.Slot()
    def btn_add_action_tool_cb(self):
        wiz = ProxyWizard(is_action=True)
        wiz.update_table()
        wiz.exec_()

        print "Selected:", wiz.selected_element


    @QtCore.Slot()
    def btn_delete_service_cb(self):
        row = self._widget.table_services.currentRow()
        if row < 0:
            return # nothing selected

        proxy_name = self._widget.table_services.item(row, 0).text()
        provider_name = self._widget.table_services.item(row, 2).text()

        if provider_name == "-":
            self.disp.command(Command_RemoveProxy(self.disp, proxy_name))
        else:
            self.disp.command(Command_RemoveProvider(self.disp, provider_name, proxy_name))

    @QtCore.Slot()
    def btn_delete_action_cb(self):
        row = self._widget.table_actions.currentRow()
        if row < 0:
            return # nothing selected
        #proxy_name = self._widget.table_services.item(row, 0).text()
        #provider_name = self._widget.table_services.item(row, 2).text()
        #self.disp.command(Command_RemoveProvider(self.disp, provider_name, proxy_name))

    @QtCore.Slot()
    def btn_add_service_cb(self):
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
                input_dialog = QtGui.QInputDialog()
                input_dialog.setComboBoxItems(tool_list)
                input_dialog.setLabelText("Select tool for this provider:")
                input_dialog.setOptions(QtGui.QInputDialog.UseListViewForComboBoxItems)
                done = input_dialog.exec_()
                name = input_dialog.textValue() # e.g. "my_new_service"

                request = Proxy_Request("service", service_type[0], service_type[1], name, name, "...", "")
                self.disp.command(Command_AddProxy(self.disp, request))

                user_choice = name

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
            request = Provider_Request(service_type[0], service_type[1], wiz.selected_service[0], user_choice, user_choice, "...", "")
            self.disp.command(Command_AddProvider(self.disp, request))


    @QtCore.Slot()
    def btn_add_action_cb(self):
        pass



    @QtCore.Slot()
    def update_table(self):
        '''Runs in main Qt thread (necessary for using QIcons)'''

        print "update"

        services = []
        actions = []
        num_services = 0
        num_actions = 0
        for proxy in self.disp.proxies.values():
            if proxy.proxy_type == "service":
                for provider in proxy.providers.values():
                    services.append(provider)
                    num_services = num_services+1
                if len(proxy.providers.values()) < 1:
                    num_services = num_services+1

            elif proxy.proxy_type == "action":
                for provider in proxy.providers.values():
                    actions.append(provider)
                    num_actions = num_actions+1
                if len(proxy.providers.values()) < 1:
                    num_actions = num_actions+1

        ## Table ##
        ##
        self.ui.table_services.setRowCount(num_services)
        self.ui.table_actions.setRowCount(num_actions)

        i = 0
        for proxy in self.disp.proxies.values():
            row = i

            if proxy.proxy_type == "service":
                table = self.ui.table_services
            elif proxy.proxy_type == "action":
                table = self.ui.table_actions

            for provider in proxy.providers.values():
                table.setItem(i, 0, QtGui.QTableWidgetItem(provider.proxy_name))
                table.setItem(i, 2, QtGui.QTableWidgetItem(provider.provider_name))

                text = "Package: " + provider.provider_type_module + "\nType: " + provider.provider_type_name
                table.setItem(i, 1, QtGui.QTableWidgetItem(text))

                table.setItem(i, 3, QtGui.QTableWidgetItem("..."))

                i = i+1

            ## In case there are no providers
            if len(proxy.providers.values()) < 1:
                table.setItem(i, 0, QtGui.QTableWidgetItem(proxy.proxy_name))
                table.setItem(i, 1, QtGui.QTableWidgetItem("-"))
                table.setItem(i, 2, QtGui.QTableWidgetItem("-"))
                table.setItem(i, 3, QtGui.QTableWidgetItem("-"))
                i = i+1
            elif len(proxy.providers.values()) > 1:
                table.setSpan(row, 0, i-row, 1)

        #self.adjustSize()



    ## PLUGIN ##
    ##
    def shutdown_plugin(self):
        #self._update_thread.kill()
        pass

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
