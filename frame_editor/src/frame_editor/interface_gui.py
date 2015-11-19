#!/usr/bin/env python


## TODO: DISCLAIMER, LICENSE, STUFF,...

from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtGui import QWidget, QPushButton
from python_qt_binding.QtCore import Signal, Slot

from frame_editor.commands import Command_SetGeometry


class FrameEditor_StyleWidget(object):

    def __init__(self, frame_editor):
        self.editor = frame_editor
        self.editor.observers.append(self)

        self.old_frame = None

        self.layout = QtGui.QGridLayout()
        self.widget = QWidget()
        self.widget.setLayout(self.layout)

        self.mesh_label = QtGui.QLineEdit("File:")
        self.mesh_label.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Fixed)
        self.mesh_button = QtGui.QPushButton("Open")
        self.mesh_button.clicked.connect(self.btn_open_mesh_clicked)

        self.diameter_label = QtGui.QLabel("Diameter:")
        self.diameter_spinbox = QtGui.QDoubleSpinBox()
        self.diameter_spinbox.editingFinished.connect(self.diameter_changed)

        self.length_label = QtGui.QLabel("Length:")
        self.length_spinbox = QtGui.QDoubleSpinBox()
        self.length_spinbox.editingFinished.connect(self.length_changed)

        self.width_label = QtGui.QLabel("Width:")
        self.width_spinbox = QtGui.QDoubleSpinBox()
        self.width_spinbox.editingFinished.connect(self.width_changed)

        self.height_label = QtGui.QLabel("Height:")
        self.height_spinbox = QtGui.QDoubleSpinBox()
        self.height_spinbox.editingFinished.connect(self.height_changed)

        self.layout.addWidget(self.mesh_label, 0, 0)
        self.layout.addWidget(self.mesh_button, 0, 1)
        self.layout.addWidget(self.diameter_label, 1, 0)
        self.layout.addWidget(self.diameter_spinbox, 1, 1)
        self.layout.addWidget(self.length_label, 2, 0)
        self.layout.addWidget(self.length_spinbox, 2, 1)
        self.layout.addWidget(self.width_label, 3, 0)
        self.layout.addWidget(self.width_spinbox, 3, 1)
        self.layout.addWidget(self.height_label, 4, 0)
        self.layout.addWidget(self.height_spinbox, 4, 1)


    def get_widget(self):
        return self.widget

    def update(self, editor, level, elements):
        
        if level & 2:
            ## Check for change
            if self.editor.active_frame is not self.old_frame:
                self.update_widget(self.editor.active_frame)

        if level & 4:
            if self.editor.active_frame is not None:
                self.update_values(self.editor.active_frame)
    
    def update_widget(self, frame):

        ## Clear layout
        #while self.layout.count():
        #    child = self.layout.takeAt(0)
        #    child.widget().deleteLater()

        self.mesh_label.hide()
        self.mesh_button.hide()
        self.diameter_label.hide()
        self.diameter_spinbox.hide()
        self.length_label.hide()
        self.length_spinbox.hide()
        self.width_label.hide()
        self.width_spinbox.hide()
        self.height_label.hide()
        self.height_spinbox.hide()

        if frame is None or frame.style == "none":
            self.widget.setEnabled(False)
            return

        if frame.style == "mesh":
            self.mesh_label.show()
            self.mesh_button.show()
        elif frame.style == "sphere":
            self.diameter_label.show()
            self.diameter_spinbox.show()
        else:
            self.length_label.show()
            self.length_spinbox.show()
            self.width_label.show()
            self.width_spinbox.show()
            if frame.style == "cube":
                self.height_label.show()
                self.height_spinbox.show()

        self.widget.setEnabled(True)

    def update_values(self, frame):
        if frame is None or frame.style == "none":
            return

        if frame.style == "mesh":
            self.mesh_label.setText(frame.path)
        elif frame.style == "sphere":
            self.diameter_spinbox.setValue(frame.diameter)
        else:
            self.length_spinbox.setValue(frame.length)
            self.width_spinbox.setValue(frame.width)
            if frame.style == "cube":
                self.height_spinbox.setValue(frame.height)

    @Slot(float)
    def diameter_changed(self):
        if self.editor.active_frame.diameter != self.diameter_spinbox.value():
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "diameter", self.diameter_spinbox.value()))

    @Slot(float)
    def length_changed(self):
        if self.editor.active_frame.length != self.length_spinbox.value():
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "length", self.length_spinbox.value()))

    @Slot(float)
    def width_changed(self):
        if self.editor.active_frame.width != self.width_spinbox.value():
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "width", self.width_spinbox.value()))

    @Slot(float)
    def height_changed(self):
        if self.editor.active_frame.height != self.height_spinbox.value():
            self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "height", self.height_spinbox.value()))


    @Slot()
    def btn_open_mesh_clicked(self):
        path = QtGui.QFileDialog.getOpenFileName(None, 'Open Mesh', '/home', 'Mesh Files (*.stl *.dae)')[0]
        self.editor.command(Command_SetGeometry(self.editor, self.editor.active_frame, "path", path))

# eof
