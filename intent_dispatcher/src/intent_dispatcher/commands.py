#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

from dispatcher import *
from python_qt_binding.QtGui import QUndoCommand


class Command_AddProvider(QUndoCommand):

    def __init__(self, dispatcher, request):
        QUndoCommand.__init__(self, "Select")
        self.dispatcher = dispatcher

        self.new_provider = Provider(request)
        self.proxy = self.dispatcher.proxies[self.new_provider.proxy_name]

    def redo(self):
        self.proxy.add_provider(self.new_provider.provider_name, self.new_provider)
        self.dispatcher.add_undo_level(1+2, [self.proxy, self.new_provider]) ## TODO couple level and element

    def undo(self):
        self.proxy.remove_provider(self.new_provider.provider_name)
        self.dispatcher.add_undo_level(1+2, [self.proxy, self.new_provider]) ## TODO

# eof
