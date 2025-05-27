#!/usr/bin/env python3
import os
import math

import rclpy
import tf_transformations as tft

from qt_gui_py_common.worker_thread import WorkerThread

from python_qt_binding import loadUi, QtCore, QtWidgets
from python_qt_binding.QtWidgets import QWidget, QTreeWidgetItem, QTreeWidget, QProgressBar
from python_qt_binding.QtCore import Slot, Qt, QTimer, QItemSelectionModel
from python_qt_binding.QtGui import QColor

from frame_editor_py.editor import Frame, FrameEditor
from frame_editor_py.commands import *
from frame_editor_py.constructors_geometry import *

from frame_editor_py.project_plugin import ProjectPlugin

from frame_editor_py.interface import Interface

## Views
from frame_editor_py.interface_gui import FrameEditor_StyleWidget, get_package_name_from_path, get_rel_path_in_ros2
from ament_index_python.packages import get_package_share_directory

class LoadingTreeWidgetItem(QTreeWidgetItem):
    def __init__(self, parent, load_time=0.5):
        super().__init__(parent)
        self.parent = parent
        
        # Create a QProgressBar to simulate loading
        self.progress_bar = QProgressBar()
        self.load_increments = int(load_time / (100/1000))
        self.progress_bar.setRange(0, self.load_increments)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)  # Hide the text inside the progress bar
        
        # Set the progress bar widget to the tree item
        self.parent.setItemWidget(self, 0, self.progress_bar)

        # Add a timer to simulate progress increment
        self.timer = QTimer(self.parent)
        self.timer.timeout.connect(self.update_progress)
        self.timer.start(100)  # Update every 100 ms

    def update_progress(self):
        """Update progress bar value to simulate loading."""
        current_value = self.progress_bar.value()
        
        if current_value < self.progress_bar.maximum():
            self.progress_bar.setValue(current_value + 1)
        else:
            self.timer.stop()  # Stop the timer when loading completes
            self.progress_bar.setValue(0)  # Reset the progress bar or hide it

class FrameEditorGUI(ProjectPlugin, Interface):

    signal_update = QtCore.Signal(int)
    signal_update_tf = QtCore.Signal(bool, bool)
    signal_load_animation = QtCore.Signal()

    def __init__(self, context):
        super(FrameEditorGUI, self).__init__(context)

        self.setObjectName('FrameEditorGUI')

        self.file_type = "YAML files(*.yaml)"

        self.editor.parse_args(context.argv())

        # Update filename display
        self.update_current_filename()


        ## Update thread ##
        ##
        self._update_thread.start()
        self.update_all(3)


    def create_editor(self, context):
        editor = FrameEditor(context)

        editor.observers.append(self)

        self.signal_update.connect(self.update_all)
        self.signal_update_tf.connect(self.update_frame_buffer)
        self.signal_load_animation.connect(self.set_tf_loading_animation)

        self._update_thread = WorkerThread(self._update_thread_run, self._update_finished)

        self.old_selected = ""

        return editor


    def create_main_widget(self):

        ## Main widget
        widget = QWidget()
        package_name = "frame_editor"
        file_path_in_package = "ui/FrameEditorGUI.ui"
        package_path = get_package_share_directory(package_name)
        ui_file = os.path.join(package_path, file_path_in_package)
        # ui_file = os.path.join(rospkg.RosPack().get_path('frame_editor'), 'src/frame_editor', 'FrameEditorGUI.ui')
        loadUi(ui_file, widget)
        widget.setObjectName('FrameEditorGUIUi')

        widget.setWindowTitle("frame editor")

        ## Views
        self.editor.init_views()
        self.interface_style = FrameEditor_StyleWidget(self.editor)

        widget.style_frame.layout().addWidget(self.interface_style.get_widget())

        ## Connections ##
        ##
        widget.btn_add.clicked.connect(self.btn_add_clicked)
        widget.btn_delete.clicked.connect(self.btn_delete_clicked)
        widget.btn_duplicate.clicked.connect(self.btn_duplicate_clicked)
        widget.list_frames.currentItemChanged.connect(self.selected_frame_changed)
        widget.list_tf.currentItemChanged.connect(self.update_measurement)
        widget.pinned_box.stateChanged.connect(self.set_pinned)

        widget.btn_refresh.clicked.connect(lambda: self.signal_update_tf.emit(True, True))
        
        widget.btn_set_parent_rel.clicked.connect(self.btn_set_parent_rel_clicked)
        widget.btn_set_parent_abs.clicked.connect(self.btn_set_parent_abs_clicked)
        widget.btn_set_pose.clicked.connect(self.btn_set_pose_clicked)
        widget.btn_set_position.clicked.connect(self.btn_set_position_clicked)
        widget.btn_set_orientation.clicked.connect(self.btn_set_orientation_clicked)
        widget.btn_set_x.clicked.connect(self.btn_set_x_clicked)
        widget.btn_set_y.clicked.connect(self.btn_set_y_clicked)
        widget.btn_set_z.clicked.connect(self.btn_set_z_clicked)
        widget.btn_set_a.clicked.connect(self.btn_set_a_clicked)
        widget.btn_set_b.clicked.connect(self.btn_set_b_clicked)
        widget.btn_set_c.clicked.connect(self.btn_set_c_clicked)
        
        widget.searchLine.textChanged.connect(self.update_search_suggestions)   
        widget.search_tf.textChanged.connect(self.update_tf_list)   

        widget.btn_reset_position_rel.clicked.connect(self.btn_reset_position_rel_clicked)
        widget.btn_reset_position_abs.clicked.connect(self.btn_reset_position_abs_clicked)
        widget.btn_reset_orientation_rel.clicked.connect(self.btn_reset_orientation_rel_clicked)
        widget.btn_reset_orientation_abs.clicked.connect(self.btn_reset_orientation_abs_clicked)

        widget.txt_x.editingFinished.connect(self.x_valueChanged)
        widget.txt_y.editingFinished.connect(self.y_valueChanged)
        widget.txt_z.editingFinished.connect(self.z_valueChanged)
        widget.txt_a.editingFinished.connect(self.a_valueChanged)
        widget.txt_b.editingFinished.connect(self.b_valueChanged)
        widget.txt_c.editingFinished.connect(self.c_valueChanged)

        widget.txt_group.editingFinished.connect(self.group_valueChanged)

        widget.btn_rad.toggled.connect(self.update_fields)

        widget.combo_style.currentIndexChanged.connect(self.frame_style_changed)

        self.last_selected_frame = None
        widget.pinned_box.setEnabled(False)

        return widget


    def _update_thread_run(self):
        self.editor.run()

    @Slot()
    def _update_finished(self):
        print("> Shutting down")


    def update(self, editor, level, elements):
        self.signal_update.emit(level)


    @Slot(int)
    def update_all(self, level):
        if level & 0:
            self.update_current_filename()

        ## Update list widgets
        if level & 1:
            self.update_frame_list()
            self.update_tf_list()

        ## Update the currently selected frame
        if level & 2:
            self.update_active_frame()

        ## Update only text fields, spin boxes,...
        if level & 4:
            self.update_fields()

    def update_tf_list_search(self, search_query=""):
        """
        Updates the tree with items that match the search query and groups them
        under two collapsible categories.
        """
        # Clear the existing items in the tree
        self.widget.list_tf.clear()
        
        # Create root items for grouping
        frame_group = QTreeWidgetItem(self.widget.list_tf)
        frame_group.setText(0, "Frames") 
        frame_group.setExpanded(True)  
        frame_group.setFlags(frame_group.flags() & ~Qt.ItemIsSelectable)
        
        other_group = QTreeWidgetItem(self.widget.list_tf)
        other_group.setText(0, "Others") 
        other_group.setFlags(other_group.flags() & ~Qt.ItemIsSelectable)

        other_group.setExpanded(True)  
        
        items = sorted(self.editor.all_frame_ids(include_temp=False))
        # Loop through the frames and add them to the appropriate group
        for item in items:
            tree_item = QTreeWidgetItem()  # Create a new tree item
            tree_item.setText(0, item)  # Set the text for the item (first column)
            
            # Check if the item is part of self.editor.frames.keys()
            if item in self.editor.frames.keys():
                group = frame_group  # Add to the 'Frames' group
            else:
                group = other_group  # Add to the 'Others' group
            
            # Apply grey styling based on the search query and filter style
            if search_query.lower() in item.lower() or self.editor.filter_style == "grey":
                if self.editor.filter_style == "grey":
                    if search_query.lower() in item.lower():
                        tree_item.setForeground(0, Qt.black)  # Set normal color for matching items
                    else:
                        tree_item.setForeground(0, QColor(169, 169, 169))  # Grey out non-matching items
                group.addChild(tree_item)  # Add the item to the corresponding group

        # Sort the items after adding them
        self.widget.list_tf.sortItems(0, Qt.AscendingOrder)

    def set_tf_loading_animation(self):
        # Clear the existing items in the tree
        self.widget.list_tf.clear()

        # Add the loading animation item to the root
        LoadingTreeWidgetItem(self.widget.list_tf, load_time=self.get_sleep_time()*0.99)  # This creates the loading item with a progress bar

        self.widget.list_tf.expandAll()
        
    @Slot()
    def update_tf_list(self):
        """
        Updates the displayed tree items based on the search query entered in searchLine.
        """
        search_query = self.widget.search_tf.text()
        self.update_tf_list_search(search_query)
        
    
    
    #############################
    # ## SEARCH FUNCTIONALITY
    def update_frame_list(self, search_query=""):
        """
        Updates the tree with items that match the search query.
        Group frames based on their `group` attribute, with collapsible groups.
        Non-group frames are added without grouping or collapsibility.
        If filter_style is 'hide', top-level items are hidden only if they do not match the search query.
        """
        # Clear the existing items in the tree
        self.widget.list_frames.clear()

        # Get the frame names (or keys) from self.editor.frames
        items = sorted(self.editor.frames.keys())  # Sorting the items
        
        # Dictionary to hold the group items, to ensure only one root per group
        group_list = {}

        # Loop through the frames to create group root items
        for element in items:
            group = self.editor.frames[element].group
            if group != "":  # If the frame has a group
                if group not in group_list:  # Only create the group root item once
                    frame_group = QTreeWidgetItem(self.widget.list_frames)
                    frame_group.setText(0, group)  # Set the group name
                    frame_group.setExpanded(True)  # Make the group expanded by default
                    frame_group.setFlags(frame_group.flags() & ~Qt.ItemIsSelectable)
                    group_list[group] = frame_group

        # Loop through the frames and add them to the appropriate group or main list
        for item in items:
            tree_item = QTreeWidgetItem() 
            tree_item.setText(0, item)
            tree_item.setFlags(tree_item.flags() | Qt.ItemIsSelectable)

            # Check if the frame has a group
            group = self.editor.frames[item].group
            if group != "":  # If the frame belongs to a group
                # Add the frame as a child item of the respective group
                group_item = group_list[group]
            else:  # If it doesn't belong to any group
                # Just add the frame as a root item without grouping
                group_item = self.widget.list_frames

            
            # Apply grey styling based on the search query and filter style
            match_found = search_query.lower() in item.lower()  # Check if the item matches the search query
            

            if match_found or self.editor.filter_style == "grey":
                if self.editor.filter_style == "grey":
                    if match_found:
                        tree_item.setForeground(0, Qt.black)  # Set normal color for matching items
                    else:
                        tree_item.setForeground(0, QColor(169, 169, 169))  # Grey out non-matching items

                # If filter_style is 'hide', skip adding top-level items that don't match the search query
                if self.editor.filter_style == "hide" and group_item == self.widget.list_frames and not match_found:
                    continue  # Skip adding the top-level item if it doesn't match the search query

                # Add the tree item to the appropriate group or directly to the list
                if isinstance(group_item, QTreeWidgetItem):  # Ensure group_item is a QTreeWidgetItem
                    group_item.addChild(tree_item)  # Add to group
                else:
                    self.widget.list_frames.addTopLevelItem(tree_item)  # Add to main list directly

        # Sort the items after adding them
        self.widget.list_frames.sortItems(0, Qt.AscendingOrder)


      
    def update_search_suggestions(self):
        """
        Updates the displayed tree items based on the search query entered in searchLine.
        """
        search_query = self.widget.searchLine.text()
        self.update_frame_list(search_query)
    #############################

    def update_active_frame(self):
        if not self.editor.active_frame:
            self.old_selected = ""
            self.widget.list_frames.setCurrentItem(None)
            self.widget.box_edit.setEnabled(False)
            return  # Deselect and quit

        self.widget.box_edit.setEnabled(True)

        name = self.editor.active_frame.name
        if name == self.old_selected:
            return  # No change

        # Search for the item by name in both top-level and child items
        found_item = None

        # First, search in top-level items
        top_level_items = self.widget.list_frames.findItems(name, Qt.MatchExactly)
        
        # If not found at the top level, search lower
        if not top_level_items:
            for i in range(self.widget.list_frames.topLevelItemCount()):
                top_item = self.widget.list_frames.topLevelItem(i)
                found_item = self.find_item_in_children(top_item, name)
                if found_item:
                    break
        else:
            found_item = top_level_items[0]

        if found_item:
            # Set the found item as the current item
            self.widget.list_frames.setCurrentItem(found_item)
            self.update_fields()

        self.old_selected = name

    def set_pinned(self, state):
        if state == Qt.Checked:
            self.editor.active_frame.pinned_frame = self.widget.list_tf.currentItem()
        elif state == Qt.Unchecked:
            self.editor.active_frame.pinned_frame = None

    def update_measurement(self,current: QTreeWidgetItem=None, previous: QTreeWidgetItem=None):
        if current:
            if not (current.flags() & Qt.ItemIsSelectable):
                # Current item is NOT selectable, revert to previous if valid
                if previous is not None:
                    self.widget.list_tf.setCurrentItem(previous)
                    self.widget.list_tf.setFocus()  
                    self.widget.list_tf.repaint()
                else:
                    self.widget.list_tf.setCurrentItem(None)
                
        source = self.widget.list_tf.currentItem()
        if not source:
            self.widget.pinned_box.setEnabled(False)
            return # none selected
        self.widget.pinned_box.setEnabled(True)
        source_name = source.text(0)
        frame = self.editor.active_frame
        try:
            position, orientation = FromTransformStamped(
                    frame.tf_buffer.lookup_transform(source_name, frame.name, rclpy.time.Time()))
            
            rot = tft.euler_from_quaternion(orientation)
            if self.widget.btn_deg.isChecked():
                rot = (180.0*rot[0]/math.pi, 180.0*rot[1]/math.pi, 180.0*rot[2]/math.pi)

            pos_str = [f"{p:.4f}" for p in position]
            rot_str = [f"{r:.4f}" for r in rot]
        except (tf2_ros.ConnectivityException, tf2_ros.LookupException):
            pos_str  = ['N/A', 'N/A', 'N/A']
            rot_str = pos_str

        self.widget.valx.setText(pos_str[0])
        self.widget.valy.setText(pos_str[1])
        self.widget.valz.setText(pos_str[2])
        
        self.widget.vala.setText(rot_str[0])
        self.widget.valb.setText(rot_str[1])
        self.widget.valc.setText(rot_str[2])


    def find_item_in_children(self, parent_item, name):
        """
        Recursively search for the item in the children of a given parent item.
        """
        # Loop through all child items of the parent item
        for i in range(parent_item.childCount()):
            child_item = parent_item.child(i)
            if child_item.text(0) == name:
                return child_item  # Return the item if it matches

        return None  # Return None if no match is found in this branch

    @Slot()
    def update_fields(self):

        f = self.editor.active_frame
        if not f:
            return

        w = self.widget

        w.txt_name.setText(f.name)
        w.txt_parent.setText(f.parent)
        w.txt_group.setText(f.group)
        self.update_measurement()

        ## Relative
        w.txt_x.setValue(f.position[0])
        w.txt_y.setValue(f.position[1])
        w.txt_z.setValue(f.position[2])

        rot = tft.euler_from_quaternion(f.orientation)
        if self.widget.btn_deg.isChecked():
            rot = (180.0*rot[0]/math.pi, 180.0*rot[1]/math.pi, 180.0*rot[2]/math.pi)

        w.txt_a.setValue(rot[0])
        w.txt_b.setValue(rot[1])
        w.txt_c.setValue(rot[2])

        txt_abs_pos = (w.txt_abs_x, w.txt_abs_y, w.txt_abs_z)
        txt_abs_rot = (w.txt_abs_a, w.txt_abs_b, w.txt_abs_c)

        ## Absolute
        try:
            position, orientation = FromTransformStamped(
                f.tf_buffer.lookup_transform('world', f.name, rclpy.Time(0)))
            for txt, p in zip(txt_abs_pos, position):
                txt.setEnabled(True)
                txt.setValue(p)
            rot = tft.euler_from_quaternion(orientation)
            if self.widget.btn_deg.isChecked():
                rot = map(math.degrees, rot)
            for txt, r in zip(txt_abs_rot, rot):
                txt.setEnabled(True)
                txt.setValue(r)
        except:
            for txt in txt_abs_rot + txt_abs_pos:
                txt.setEnabled(False)

        ## Style
        self.widget.combo_style.setCurrentIndex(self.widget.combo_style.findText(f.style))


    @Slot(QTreeWidgetItem, QTreeWidgetItem)
    def selected_frame_changed(self, item, previous):
        # 'item' is the currently selected item (QTreeWidgetItem)
        if item is None:
            return

        name = item.text(0)  # Get the text of the selected item
        
        if name not in self.editor.frames: 
            return
        
        if name == "":
            return

        # Perform the selection logic as before
        if not self.editor.active_frame or (self.editor.active_frame.name != name):
            self.editor.command(Command_SelectElement(self.editor, self.editor.frames[name]))
            self.widget.pinned_box.blockSignals(True)
            if self.editor.active_frame.pinned_frame is not None:
                # if previously you selected a non-pinned-frame, save it for reset purposes
                if not self.editor.frames[previous.text(0)].pinned_frame:
                    self.last_selected_frame = self.widget.list_tf.currentItem()
                self.widget.pinned_box.setChecked(True)
                self.widget.list_tf.setCurrentItem(self.editor.active_frame.pinned_frame)
            else:
                if self.last_selected_frame:
                    self.widget.list_tf.setCurrentItem(self.last_selected_frame)
                self.widget.pinned_box.setChecked(False)
            self.widget.pinned_box.blockSignals(False)
        self.update_measurement()
       


    ## BUTTONS ##
    ##
    def write_file(self, file_name):
        return self.editor.save_file(file_name)


    @Slot()
    def clear_all(self):
        self.editor.command(Command_ClearAll(self.editor))

    def get_valid_frame_name(self, window_title, default_name="my_frame"):

        existing_tf_frames = set(self.editor.all_frame_ids())
        existing_editor_frames = set(self.editor.all_editor_frame_ids())

        name, ok = QtWidgets.QInputDialog.getText(self.widget, window_title, "Name:", QtWidgets.QLineEdit.Normal, default_name);

        # allow recreating if frame was published by frameditor node originally
        while ok and name in existing_editor_frames or (name in existing_tf_frames and not Frame.was_published_by_frameeditor(name)):
            name, ok = QtWidgets.QInputDialog.getText(self.widget, window_title, "Name (must be unique):", QtWidgets.QLineEdit.Normal, default_name)
        if not ok:
            return None
        return name

        
    @Slot(bool)
    def btn_add_clicked(self, checked):
        
        name = self.get_valid_frame_name("Add New Frame")
        if not name:
            return

        available_parents = self.editor.all_frame_ids(include_temp=False)
        if not available_parents:
            available_parents = ["world"]

        parent, ok = QtWidgets.QInputDialog.getItem(self.widget, "Add New Frame", "Parent Name:", sorted(available_parents))

        if not ok or parent == "":
            return

        self.editor.command(Command_AddElement(self.editor, Frame(name, parent=parent)))
        self.signal_update_tf.emit(False, False)

    @Slot(bool)
    def btn_duplicate_clicked(self, checked):
        item = self.widget.list_frames.currentItem()
        if not item:
            return
        source_name = item.text(0)
        parent_name = self.editor.frames[source_name].parent

        name = self.get_valid_frame_name("Duplicate Frame", default_name=source_name)
        if not name:
            return

        self.editor.command(Command_CopyElement(self.editor, name, source_name, parent_name))
        self.signal_update_tf.emit(False, False)

    def get_sleep_time(self):
        return max(5.0 / self.editor.hz, 0.1)

    @Slot(bool, bool)
    def update_frame_buffer(self, animation=False, reset_buffer=True):
        if animation:
            self.signal_load_animation.emit()
        sleep_time = self.get_sleep_time()*1000/2  # Time takes time in ms
        if reset_buffer:
            self.timer_clear_buffer = QTimer(self)
            self.timer_clear_buffer.setSingleShot(True)  # Run only once
            self.timer_clear_buffer.timeout.connect(Frame.tf_buffer.clear)
            self.timer_clear_buffer.start(int(sleep_time))
        
        self.timer_update_list = QTimer(self)
        self.timer_update_list.setSingleShot(True)  # Run only once
        self.timer_update_list.timeout.connect(self.update_tf_list)
        self.timer_update_list.start(int(sleep_time*2))  


    @Slot(bool)
    def btn_delete_clicked(self, checked):
        item = self.widget.list_frames.currentItem()
        if not item:
            return
        self.editor.command(Command_RemoveElement(self.editor, self.editor.frames[item.text(0)]))
        self.signal_update_tf.emit(True, True)
        
    ## PARENTING ##
    ##
    @Slot(bool)
    def btn_set_parent_rel_clicked(self, checked):
        self.set_parent(False)

    @Slot(bool)
    def btn_set_parent_abs_clicked(self, checked):
        self.set_parent(True)

    def set_parent(self, keep_absolute):
        parent = self.widget.list_tf.currentItem()
        if not parent:
            return # none selected

        if parent.text(0) == self.editor.active_frame.name:
            return # you can't be your own parent

        self.editor.command(Command_SetParent(self.editor, self.editor.active_frame, parent.text(0), keep_absolute))


    ## SET BUTTONS ##
    ##
    @Slot(bool)
    def btn_set_pose_clicked(self, checked):
        self.set_pose(["x", "y", "z", "a", "b", "c"])

    @Slot(bool)
    def btn_set_position_clicked(self, checked):
        self.set_pose(["x", "y", "z"])

    @Slot(bool)
    def btn_set_orientation_clicked(self, checked):
        self.set_pose(["a", "b", "c"])

    @Slot(bool)
    def btn_set_x_clicked(self, checked):
        self.set_pose(["x"])
    @Slot(bool)
    def btn_set_y_clicked(self, checked):
        self.set_pose(["y"])
    @Slot(bool)
    def btn_set_z_clicked(self, checked):
        self.set_pose(["z"])
    @Slot(bool)
    def btn_set_a_clicked(self, checked):
        self.set_pose(["a"])
    @Slot(bool)
    def btn_set_b_clicked(self, checked):
        self.set_pose(["b"])
    @Slot(bool)
    def btn_set_c_clicked(self, checked):
        self.set_pose(["c"])

    def set_pose(self, mode):
        source = self.widget.list_tf.currentItem()
        if not source:
            return # none selected

        frame = self.editor.active_frame
        self.editor.command(Command_AlignElement(self.editor, frame, source.text(0), mode))


    ## RESET BUTTONS ##
    ##
    @Slot(bool)
    def btn_reset_position_rel_clicked(self, checked):
        self.editor.command(Command_SetPosition(self.editor, self.editor.active_frame, (0, 0, 0)))

    @Slot(bool)
    def btn_reset_position_abs_clicked(self, checked):
        position, orientation = FromTransformStamped(
            self.editor.active_frame.tf_buffer.lookup_transform(
                self.editor.active_frame.parent, "world", rclpy.Time(0)))
        self.editor.command(Command_SetPosition(self.editor, self.editor.active_frame, position))

    @Slot(bool)
    def btn_reset_orientation_rel_clicked(self, checked):
        self.editor.command(Command_SetOrientation(self.editor, self.editor.active_frame, (0, 0, 0, 1)))

    @Slot(bool)
    def btn_reset_orientation_abs_clicked(self, checked):
        position, orientation = FromTransformStamped(
            self.editor.active_frame.listener.lookupTransform(
                self.editor.active_frame.parent, "world", rclpy.Time(0)))
        self.editor.command(Command_SetOrientation(self.editor, self.editor.active_frame, orientation))


    ## Spin Boxes ##
    ##
    def set_value(self, widget, symbol):
        frame = self.editor.active_frame
        value = widget.value()

        ## Deg to rad
        if self.widget.btn_deg.isChecked() and symbol in ['a', 'b', 'c']:
            value = value * math.pi / 180.0

        if frame.value(symbol) != value:
            self.editor.command(Command_SetValue(self.editor, self.editor.active_frame, symbol, value))

    @Slot()
    def x_valueChanged(self):
        self.set_value(self.widget.txt_x, 'x')
    @Slot()
    def y_valueChanged(self):
        self.set_value(self.widget.txt_y, 'y')
    @Slot()
    def z_valueChanged(self):
        self.set_value(self.widget.txt_z, 'z')
    @Slot()
    def a_valueChanged(self):
        self.set_value(self.widget.txt_a, 'a')
    @Slot()
    def b_valueChanged(self):
        self.set_value(self.widget.txt_b, 'b')
    @Slot()
    def c_valueChanged(self):
        self.set_value(self.widget.txt_c, 'c')

    @Slot()
    def group_valueChanged(self):
        value = self.widget.txt_group.text()
        if self.editor.active_frame.group != value:
            self.editor.command(Command_SetGroup(self.editor, self.editor.active_frame, value))



    ## FRAME STYLE ##
    ##
    @Slot(int)
    def frame_style_changed(self, id):
        style = self.widget.combo_style.currentText().lower()
        if self.editor.active_frame.style != style:
            self.editor.command(Command_SetStyle(self.editor, self.editor.active_frame, style))


    ## PLUGIN ##
    ##
    def shutdown_plugin(self):
        super(FrameEditorGUI, self).shutdown_plugin()
        self._update_thread.kill()

# eof
