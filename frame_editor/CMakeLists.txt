cmake_minimum_required(VERSION 2.8.3)
project(frame_editor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  tf
  message_generation
  dynamic_reconfigure
  rqt_gui
  rqt_gui_py
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AlignFrame.srv
  EditFrame.srv
  GetFrame.srv
  RemoveFrame.srv
  SetFrame.srv
  SetParentFrame.srv
  CopyFrame.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Add dynamic reconfigure api
#generate_dynamic_reconfigure_options(
#  cfg/xyz.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frame_editor
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  tf
  message_runtime
  dynamic_reconfigure
  rqt_gui
  rqt_gui_py
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
#)

## Declare a cpp library
# add_library(frame_editor
#   src/${PROJECT_NAME}/frame_editor.cpp
# )

## Declare a cpp executable
#add_executable(frame_editor_node
#  src/frame_editor_node.cpp
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(frame_editor_node
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${PROJECT_NAME}_gencfg
#)

## Specify libraries to link a library or executable target against
#target_link_libraries(frame_editor_node
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS
                      src/frame_editor/commands.py
                      src/frame_editor/constructors_std.py
                      src/frame_editor/interface_gui.py
                      src/frame_editor/interface_markers.py
                      src/frame_editor/interface_services.py
                      src/frame_editor/objects.py
                      src/frame_editor/rqt_editor.py
                      src/frame_editor/constructors_geometry.py
                      src/frame_editor/editor.py
                      src/frame_editor/interface_interactive_marker.py
                      src/frame_editor/interface.py
                      src/frame_editor/interface_tf.py
                      src/frame_editor/project_plugin.py
                      src/frame_editor/utils_tf.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        src/frame_editor/FrameEditorGUI.ui
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/frame_editor)

install(DIRECTORY launch etc
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
        plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation
# install(TARGETS frame_editor frame_editor_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_frame_editor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
