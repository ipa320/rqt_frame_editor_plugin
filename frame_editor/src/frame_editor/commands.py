#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

import rospy
import tf

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


class Command_AlignElement(QUndoCommand):

    def __init__(self, editor, element, source_name, mode):
        QUndoCommand.__init__(self, "Align")
        self.editor = editor

        self.element = element

        self.old_position = element.position
        self.old_orientation = element.orientation

        ## New Pose ##
        ##
        (position, orientation) = element.listener.lookupTransform(element.parent, source_name, rospy.Time(0))

        ## Position
        pos = list(element.position)
        if "x" in mode:
            pos[0] = position[0]
        if "y" in mode:
            pos[1] = position[1]
        if "z" in mode:
            pos[2] = position[2]
        self.new_position = tuple(pos)

        ## Orientation
        rpy = list(tf.transformations.euler_from_quaternion(element.orientation))
        rpy_new = tf.transformations.euler_from_quaternion(orientation)
        if "a" in mode:
            rpy[0] = rpy_new[0]
        if "b" in mode:
            rpy[1] = rpy_new[1]
        if "c" in mode:
            rpy[2] = rpy_new[2]

        if "a" in mode or "b" in mode or "c" in mode:
            self.new_orientation = tf.transformations.quaternion_from_euler(*rpy)
        else:
            self.new_orientation = self.old_orientation

    def redo(self):
        self.element.position = self.new_position
        self.element.orientation = self.new_orientation
        self.element.broadcast()
        self.editor.add_undo_level(4)

    def undo(self):
        self.element.position = self.old_position
        self.element.orientation = self.old_orientation
        self.element.broadcast()
        self.editor.add_undo_level(4)

# eof
