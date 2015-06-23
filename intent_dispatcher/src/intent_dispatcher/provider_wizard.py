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

    def __init__(self, is_action):
        super(ProviderWizard, self).__init__()

        self.is_action = is_action
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
        table = self.ui.table_elements
        table.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QtGui.QAbstractItemView.SingleSelection)
        table.setSelectionBehavior(QtGui.QAbstractItemView.SelectRows)
        table.verticalHeader().setDefaultSectionSize(80);
        table.currentCellChanged.connect(self.element_selected)

        ## GUI ##
        ##
        self.rejected.connect(self.rejected_cb)
        self.event = threading.Event()

        # ui.btn_expert.toggled.connect(self.set_expert)
        ui.btnOk.clicked.connect(self.btn_ok_cb)
        ui.btnOk.setEnabled(False)


    @QtCore.Slot(int)
    def element_selected(self, currentRow):
        self.ui.btnOk.setEnabled(True)

    @QtCore.Slot(int)
    def btn_ok_cb(self):
        self.row = self.ui.table_elements.currentRow()
        if self.is_action:
            self.selected_element = self.actions[self.row]
        else:
            self.selected_element = self.services[self.row]
        self.accept() # close dialog

    @QtCore.Slot()
    def rejected_cb(self):
        self.row = -1 # none selected
        self.selected_element = None


    @QtCore.Slot()
    def update_table(self):
        '''Runs in main Qt thread (necessary for using QIcons)'''

        ## Lookup ##
        ##
        p = ProviderLookup()
        services, actions = p.lookup_providers()

        if self.is_action:
            elements = actions
        else:
            elements = services

        self.services = services
        self.actions = actions

        ## Table ##
        ##
        self.ui.table_elements.setRowCount(len(elements))

        for i, element in enumerate(elements):
            
            item = QtGui.QTableWidgetItem(element[0])
            self.ui.table_elements.setItem(i, 0, item)

            ## Type
            element_type = element[1].split("/")
            text = "Package: " + element_type[0] + "\nType: " + element_type[1]
            item = QtGui.QTableWidgetItem(text)
            self.ui.table_elements.setItem(i, 1, item)

            item = QtGui.QTableWidgetItem(element[2])
            self.ui.table_elements.setItem(i, 2, item)

        self.adjustSize()

# eof
