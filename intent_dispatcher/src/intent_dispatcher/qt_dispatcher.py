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
import actionlib

## Qt ##
##
from python_qt_binding import loadUi
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding.QtCore import Qt


from intent_dispatcher import dispatcher



class Dispatcher_Dialog(QtGui.QDialog):

    def __init__(self, parent=None):
        super(Dispatcher_Dialog, self).__init__(parent)

        self.row = -1

        ## Load UI file ##
        ##
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('intent_dispatcher')
        ui_file = os.path.join(package_path, 'ui', 'dispatcher.ui')
        ui = loadUi(ui_file, self)
        self.ui = ui

        ## Table ##
        ##
        table = self.ui.table_providers
        table.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QtGui.QAbstractItemView.SingleSelection)
        table.setSelectionBehavior(QtGui.QAbstractItemView.SelectRows)
        table.verticalHeader().setDefaultSectionSize(100);
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
        self.row = self.ui.table_providers.currentRow()
        self.event.set() # wake up chooser
        self.accept() # close dialog

    @QtCore.Slot()
    def rejected_cb(self):
        self.row = -1 # none selected
        self.event.set() # wake up chooser
        self.reject() # close dialog



    def choose_provider(self, intent, providers):
        '''Runs in callback thread'''

        ## Checks ##
        ##
        if len(providers) == 0:
            return None

        print "choose"
        self.intent = intent
        self.providers = providers

        ## Start dialog ##
        ##
        QtCore.QMetaObject.invokeMethod(self, "update_table", Qt.QueuedConnection);
        QtCore.QMetaObject.invokeMethod(self, "show", Qt.QueuedConnection);

        print "wait"
        self.event.wait()
        self.event.clear()

        self.ui.table_providers.clearSelection()
        self.ui.table_providers.setRowCount(0)
        self.ui.btnOk.setEnabled(False)

        print "return"
        names = providers.keys()
        return providers[names[self.row]]


    @QtCore.Slot()
    def update_table(self):
        '''Runs in main Qt thread (necessary for using QIcons)'''

        print "update"
        intent = self.intent
        providers = self.providers

        ## Text field ##
        ##
        self.ui.txtIntentDescription.setText(intent.name+": "+intent.description)

        ## Table ##
        ##
        self.ui.table_providers.setRowCount(len(providers))

        for i, provider in enumerate(providers.values()):

            title = provider.name + ": " + provider.description

            # item = QtGui.QTableWidgetItem(provider.name)
            # self.ui.table_providers.setItem(i, 1, item)

            # item = QtGui.QTableWidgetItem(provider.description)
            # self.ui.table_providers.setItem(i, 2, item)

            #item = QtGui.QTableWidgetItem(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_FileIcon), "")
            print provider.icon_path
            icon = QtGui.QIcon(provider.icon_path);
            item = QtGui.QTableWidgetItem(icon, title)
            self.ui.table_providers.setItem(i, 0, item)

        self.adjustSize()


## Qt ##
##
def qt_loop():
    global widget

    app = QtGui.QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False) # runs in background and doesn't quit once a dialog is closed

    ## Widget (must be created in the main Qt thread) ##
    ##
    widget = Dispatcher_Dialog()
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


    ## Dispatcher ##
    ##
    time.sleep(1) ## TODO
    disp = dispatcher.Dispatcher()
    disp.set_chooser(widget.choose_provider)


    ## For Testing ##
    ##
    #time.sleep(1)
    #QtCore.QMetaObject.invokeMethod(widget, "show", Qt.QueuedConnection);


    ## ROS loop ##
    ##
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

    return


if __name__ == '__main__':
    main()

# eof
