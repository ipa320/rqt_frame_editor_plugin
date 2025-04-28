#!/usr/bin/env python3
from python_qt_binding.QtCore import QObject

class Interface(QObject):

    def __init__(self, frame_editor):
        super(QObject, self).__init__()

    def update(self, editor, level, elements):
        pass

    def broadcast(self, editor):
        pass

# eof
