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

from intent_dispatcher.provider_lookup import ProviderLookup


class ProviderWizard(QtGui.QDialog):

    def __init__(self):
        super(ProviderWizard, self).__init__()

        self.row = -1

        ## Load UI file ##
        ##
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('intent_dispatcher')
        ui_file = os.path.join(package_path, 'ui', 'provider_wizard.ui')
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
        self.rejected.connect(self.rejected_cb)
        self.event = threading.Event()

        # ui.btn_expert.toggled.connect(self.set_expert)
        ui.btnOk.clicked.connect(self.btn_ok_cb)
        ui.btnOk.setEnabled(False)

        return

    @QtCore.Slot(int)
    def provider_selected(self, currentRow):
        self.ui.btnOk.setEnabled(True)

    @QtCore.Slot(int)
    def btn_ok_cb(self):
        self.row = self.ui.table_services.currentRow()
        self.selected_service = self.services[self.row]
        self.accept() # close dialog

    @QtCore.Slot()
    def rejected_cb(self):
        self.row = -1 # none selected
        self.selected_service = None


    @QtCore.Slot()
    def update_table(self):
        '''Runs in main Qt thread (necessary for using QIcons)'''

        print "update"

        p = ProviderLookup()
        services, actions = p.lookup_providers()
        self.services = services
        self.actions = actions
        #print actions
        #print services


        ## Table ##
        ##
        self.ui.table_services.setRowCount(len(services))

        for i, service in enumerate(services):
            print service

            item = QtGui.QTableWidgetItem(service[0])
            self.ui.table_services.setItem(i, 0, item)

            ## Type
            service_type = service[1].split("/")
            text = "Package: " + service_type[0] + "\nType: " + service_type[1]
            item = QtGui.QTableWidgetItem(text)
            self.ui.table_services.setItem(i, 1, item)

            item = QtGui.QTableWidgetItem(service[2])
            self.ui.table_services.setItem(i, 2, item)


        self.adjustSize()


## Qt ##
##
def qt_loop():
    global widget

    app = QtGui.QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False) # runs in background and doesn't quit once a dialog is closed

    ## Widget (must be created in the main Qt thread) ##
    ##
    widget = ProviderWizard()
    #widget.resize(1024, 768)
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
    QtCore.QMetaObject.invokeMethod(widget, "update_table", Qt.QueuedConnection);
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
