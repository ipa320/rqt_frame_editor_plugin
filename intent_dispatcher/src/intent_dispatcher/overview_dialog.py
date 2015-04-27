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


class Overview_Dialog(QtGui.QDialog):

    def __init__(self):
        super(Overview_Dialog, self).__init__()

        self.disp = dispatcher.Dispatcher()
        #self.disp.set_chooser(widget.choose_provider)

        self.row = -1

        ## Load UI file ##
        ##
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('intent_dispatcher')
        ui_file = os.path.join(package_path, 'ui', 'overview_dialog.ui')
        ui = loadUi(ui_file, self)
        self.ui = ui

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
        ui.btnSaveAs.clicked.connect(self.on_btn_save_as_clicked)

        self.update_table()




    ## Import/Export ##
    ##
    @QtCore.Slot(int)
    def on_btn_save_as_clicked(self):
        yaml_io.export_yaml("my_settings.yaml")



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

        self.adjustSize()


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
