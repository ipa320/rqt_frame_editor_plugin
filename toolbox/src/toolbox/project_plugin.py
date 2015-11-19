#!/usr/bin/env python

from qt_gui.plugin import Plugin
from qt_gui_py_common.worker_thread import WorkerThread

from python_qt_binding import loadUi, QtGui, QtCore
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Signal, Slot


class ProjectPlugin(Plugin):

    def __init__(self, context):
        super(ProjectPlugin, self).__init__(context)

        ## Editor
        self.editor = self.create_editor()
        self.editor.undo_stack.cleanChanged.connect(self.clean_changed)

        ## Main widget
        self.widget = self.create_main_widget()
        context.add_widget(self.widget)

        ## Menus
        self.create_menus()

        ## File
        self.file_name = ""
        self.set_current_file("")
        self.load_file("") # loads empty.xml


    def create_editor(self):
        raise NotImplementedError

    def create_main_widget(self):
        raise NotImplementedError

    def create_menus(self):

        ## ACTIONS ##
        ##
        newAction = QtGui.QAction("&New", self)
        newAction.setShortcuts(QtGui.QKeySequence.New)
        newAction.setStatusTip("Create a new file")
        newAction.setIcon(QtGui.QIcon.fromTheme("document-new"))
        newAction.triggered.connect(self.new_file)

        openAction = QtGui.QAction("&Open", self)
        openAction.setShortcuts(QtGui.QKeySequence.Open)
        openAction.setStatusTip("Open a file")
        openAction.setIcon(QtGui.QIcon.fromTheme("document-open"))
        openAction.triggered.connect(self.open)

        saveAction = QtGui.QAction("&Save", self)
        saveAction.setShortcuts(QtGui.QKeySequence.Save)
        saveAction.setStatusTip("Save file")
        saveAction.setIcon(QtGui.QIcon.fromTheme("document-save"))
        saveAction.triggered.connect(self.save)

        saveAsAction = QtGui.QAction("Save_&As", self)
        saveAsAction.setShortcuts(QtGui.QKeySequence.SaveAs)
        saveAsAction.setStatusTip("Save file as...")
        saveAsAction.setIcon(QtGui.QIcon.fromTheme("document-save-as"))
        saveAsAction.triggered.connect(self.save_as)

        undoAction = self.editor.undo_stack.createUndoAction(self, self.tr("&Undo"))
        undoAction.setShortcuts(QtGui.QKeySequence.Undo)
        undoAction.setIcon(QtGui.QIcon.fromTheme("edit-undo"))
        redoAction = self.editor.undo_stack.createRedoAction(self, self.tr("&Redo"))
        redoAction.setShortcuts(QtGui.QKeySequence.Redo)
        redoAction.setIcon(QtGui.QIcon.fromTheme("edit-redo"))


        ## Menu
        file_menu = self.widget.menuBar.addMenu("&File")
        file_menu.addAction(newAction)
        file_menu.addAction(openAction)
        file_menu.addAction(saveAction)
        file_menu.addAction(saveAsAction)

        edit_menu = self.widget.menuBar.addMenu("&Edit")

        edit_menu.addAction(undoAction)
        edit_menu.addAction(redoAction)

        ## Tool bar
        tool_bar = self.widget.mainToolBar

        undoButton = QtGui.QToolButton()
        undoButton.setDefaultAction(undoAction)
        redoButton = QtGui.QToolButton()
        redoButton.setDefaultAction(redoAction)

        tool_bar.addAction(newAction)
        tool_bar.addAction(openAction)
        tool_bar.addAction(saveAction)
        tool_bar.addAction(saveAsAction)
        tool_bar.addSeparator()
        tool_bar.addWidget(undoButton)
        tool_bar.addWidget(redoButton)
        #tool_bar.addSeparator()


    def new_file(self):
        if self.ok_to_continue():
            ## Create new empty app-root
            #self.editor.undo_stack.clear()
            self.load_file("")

            ## Set file name to undefined
            self.set_current_file("")

    def open(self):
        if self.ok_to_continue():
            file_name, stuff = QtGui.QFileDialog.getOpenFileName(self.widget,
                "Select a file to open", ".", "XML files(*.xml)")

            if not file_name == "":
                self.load_file(file_name)

    def load_file(self, file_name):
        if not self.editor.load_file(file_name):
            print "ERROR LOADING FILE"
            return False
        else:
            self.set_current_file(file_name)
            return True

    def ok_to_continue(self):
        """If the file has been modified, the user is asked, whether he wants to save it first or not.
        Returns True when the file has been saved or was unmodified.
        Returns False when the user reconsiders in case of an unmodified file.
        """
        if self.widget.isWindowModified():
            reply = QtGui.QMessageBox.warning(self.widget, "pitasc Qt",
                "The file has been modified.\nDo you want to save your changes?",
                QtGui.QMessageBox.Yes | QtGui.QMessageBox.Default,
                QtGui.QMessageBox.No,
                QtGui.QMessageBox.Cancel | QtGui.QMessageBox.Escape)

            if reply == QtGui.QMessageBox.Yes:
                return self.save()
            elif reply == QtGui.QMessageBox.Cancel:
                return False

        return True

    def save(self):
        """Calls save_as or save_file
        """
        if self.file_name == "":
            return self.save_as()
        else:
            return self.save_file(self.file_name)

    def save_as(self):
        file_name, stuff = QtGui.QFileDialog.getSaveFileName(None, "Save File", ".", "XML files (*.xml)")
        if file_name == "":
            return False
        else:
            return self.save_file(file_name)

    def save_file(self, file_name):
        if not self.write_file(file_name):
            print "Saving canceled"
            return False
        else:
            self.set_current_file(file_name)
            print "File saved"
            return True

    def write_file(self, file_name):
        raise NotImplementedError

    def set_current_file(self, file_name):
        ## Set clean
        self.editor.undo_stack.setClean()
        self.widget.setWindowModified(False)

        ## File name
        self.file_name = file_name

        ## Window title
        shown_name = "Untitled"
        if not self.file_name == "":
            shown_name = self.stripped_name(file_name)
            # recent files...
        self.widget.setWindowTitle(self.tr('{} [*] - {}'.format(shown_name, "pitasc Qt")))

    def stripped_name(self, full_name):
        return QtCore.QFileInfo(full_name).fileName()

    def clean_changed(self, is_clean):
        self.widget.setWindowModified(not is_clean)



    ## PLUGIN ##
    ##
    def shutdown_plugin(self):
        #self._update_thread.kill()

        ## Ask for permission to close
        if self.widget.isWindowModified():
            reply = QtGui.QMessageBox.warning(self.widget, "pitasc Qt",
                "The file has been modified.\nDo you want to save your changes before exiting (Save As...)?",
                QtGui.QMessageBox.Yes | QtGui.QMessageBox.Default,
                QtGui.QMessageBox.No)

            if reply == QtGui.QMessageBox.Yes:
                self.save_as()


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

# eof
