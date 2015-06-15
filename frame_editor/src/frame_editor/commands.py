#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

from python_qt_binding.QtGui import QUndoCommand


class Command_SelectElement(QUndoCommand):

    def __init__(self, editor, element):
        QUndoCommand.__init__(self, "Select")
        self.editor = editor

        self.new_element = element
        self.old_element = editor.active_frame

    def redo(self):
        self.editor.active_frame = self.new_element
        self.editor.add_undo_level(2)

    def undo(self):
        self.editor.active_frame = self.old_element
        self.editor.add_undo_level(2)


class Command_AddElement(QUndoCommand):

    def __init__(self, editor, element):
        QUndoCommand.__init__(self, "Add")
        self.editor = editor

        self.element = element

    def redo(self):
        self.editor.frames[self.element.name] = self.element
        self.editor.add_undo_level(1)

    def undo(self):
        del self.editor.frames[self.element.name]
        self.editor.add_undo_level(1)


class Command_RemoveElement(QUndoCommand):

    def __init__(self, editor, element):
        QUndoCommand.__init__(self, "Remove")
        self.editor = editor

        self.element = element
        
        if editor.active_frame is element:
            self.was_active = True
        else:
            self.was_active = False

    def redo(self):
        if self.was_active:
            self.editor.active_frame = None
            self.editor.add_undo_level(2)

        del self.editor.frames[self.element.name]
        self.editor.add_undo_level(1)

    def undo(self):
        self.editor.frames[self.element.name] = self.element
        self.editor.add_undo_level(1)

        if self.was_active:
            self.editor.active_frame = self.element
            self.editor.add_undo_level(2)



class Command_ClearAll(QUndoCommand):

    def __init__(self, editor):
        QUndoCommand.__init__(self, "Clear all")
        self.editor = editor

        self.elements = editor.frames
        self.active_element = editor.active_frame

    def redo(self):
        self.editor.active_frame = None
        self.editor.frames = {}
        self.editor.add_undo_level(1+2)

    def undo(self):
        self.editor.active_frame = self.active_element
        self.editor.frames = self.elements
        self.editor.add_undo_level(1+2)

# eof
