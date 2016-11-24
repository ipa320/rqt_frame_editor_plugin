#!/usr/bin/env python

from python_qt_binding.QtCore import QObject

class Interface(object):

    def __init__(self, frame_editor):
        pass

    def update(self, editor, level, elements):
        pass

    def broadcast(self, editor):
        pass

# eof
