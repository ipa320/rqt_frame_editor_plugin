#!/usr/bin/env python

## Python ##
##
import importlib
import threading

## ROS ##
##
import rospy
import actionlib

from intent_dispatcher.proxy import *
from intent_dispatcher.provider import *
from intent_dispatcher.srv import *
from intent_dispatcher.commands import Command_AddProxy, Command_AddProvider

from python_qt_binding import QtCore
from python_qt_binding.QtGui import QUndoStack


class Dispatcher(QtCore.QObject):

    def __init__(self):
        super(Dispatcher, self).__init__()

        self.proxies = {}

        ## Register service ##
        ##
        rospy.Service("register_proxy",       AddProxy, self.add_proxy_callback)
        rospy.Service("register_provider", AddProvider, self.add_provider_callback)

        self.set_chooser(text_chooser)

        ## Undo/Redo
        self.observers = []
        self.undo_level = 0
        self.undo_elements = []
        self.undo_stack = QUndoStack()
        self.undo_stack.indexChanged.connect(self.undo_stack_changed)
        self.__command_lock = threading.Lock()


    ## Undo/Redo ##
    ##
    @QtCore.Slot(int)
    def undo_stack_changed(self, idx):
        '''Updates all observers, whenever a command has been undone/redone'''
        self.update_obsevers(self.undo_level)

    def add_undo_level(self, level, elements=None):
        '''Used by commands to add a level for updating'''
        self.undo_level = self.undo_level | level
        if elements:
            self.undo_elements.extend(elements)

    def command(self, command):
        '''Push a command to the stack (blocking)'''
        with self.__command_lock:
            self.undo_stack.push(command)

    def update_obsevers(self, level):
        '''Updates all registered observers and resets the undo_level'''
        for observer in self.observers:
            observer.update(self, level, self.undo_elements)
        self.undo_level = 0
        self.undo_elements = []



    def set_chooser(self, chooser):
        self.default_chooser = chooser

    
    def add_proxy_callback(self, req):
        if req.proxy_name in self.proxies:
            print "Proxy already exists:", self.proxies[req.proxy_name]
        else:
            self.command(Command_AddProxy(self, req))

        return AddProxyResponse()


    def add_provider_callback(self, req):
        if not req.proxy_name in self.proxies:
            print "Proxy does not exist", req.proxy_name
        else:
            self.command(Command_AddProvider(self, req))
        
        return AddProviderResponse()
   

def text_chooser(providers):
    if len(providers) == 0:
        return None
    else:
        print "Which action should be taken?"
        for i, provider in enumerate(providers):
            print i,":", provider

        ## Let user decide ##
        ##
        names = providers.keys()
        try:
            num = int(raw_input('Number: '))
            return providers[names[num]]
        except:
            print "Not a number or number not valid. Failed. Returning!"
            return None ## TODO

    
if __name__ == "__main__":

    rospy.init_node('intent_dispatcher')

    dispatcher = Dispatcher()

    print "Dispatcher ready: Call service '/register_intent' to register a new service."
    rospy.spin()
    
# eof
