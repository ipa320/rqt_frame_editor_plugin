cmake_minimum_required(VERSION 3.5)
project(frame_editor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services, and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AlignFrame.srv"
  "srv/EditFrame.srv"
  "srv/GetFrame.srv"
  "srv/GetFrameNames.srv"
  "srv/RemoveFrame.srv"
  "srv/SetFrame.srv"
  "srv/SetParentFrame.srv"
  "srv/CopyFrame.srv"
  "srv/LoadYaml.srv"
  "srv/SaveYaml.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)


install(FILES
  frame_editor_py/FrameEditorGUI.ui
  DESTINATION share/${PROJECT_NAME}/ui
)

install(DIRECTORY etc
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS frame_editor_py/editor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}_py)

ament_package()
#############
## Testing ##
#############

# Add testing if needed
# ament_add_gtest(${PROJECT_NAME}-test test/test_frame_editor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
