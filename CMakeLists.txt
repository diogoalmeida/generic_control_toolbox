cmake_minimum_required(VERSION 2.8.3)
project(generic_control_toolbox)

find_package(
  catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  actionlib
  geometry_msgs
  visualization_msgs
  cmake_modules
  eigen_conversions
  kdl_parser
  trac_ik_lib
  sensor_msgs
  tf_conversions
  realtime_tools
  hardware_interface
  controller_interface
  tf
)

catkin_python_setup()

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  ArmInfo.msg
)

# add_action_files(
# 	DIRECTORY action
# 	FILES
# )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rosbag roscpp rospy actionlib geometry_msgs visualization_msgs
                 cmake_modules eigen_conversions kdl_parser sensor_msgs
                 tf_conversions realtime_tools hardware_interface
                 controller_interface tf trac_ik_lib
  INCLUDE_DIRS include
  LIBRARIES matrix_parser kdl_manager wrench_manager controller_template
            marker_manager controller_action_node ros_control_interface
            bag_manager tf_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(bag_manager src/bag_manager.cpp)
target_link_libraries(bag_manager ${catkin_LIBRARIES})
add_dependencies(bag_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(matrix_parser src/matrix_parser.cpp)
target_link_libraries(matrix_parser ${catkin_LIBRARIES})
add_dependencies(matrix_parser ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(kdl_manager src/kdl_manager.cpp src/manager_base.cpp src/matrix_parser.cpp)
target_link_libraries(kdl_manager ${catkin_LIBRARIES})
add_dependencies(kdl_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(wrench_manager src/wrench_manager.cpp src/manager_base.cpp)
target_link_libraries(wrench_manager ${catkin_LIBRARIES})
add_dependencies(wrench_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(marker_manager src/marker_manager.cpp)
target_link_libraries(marker_manager ${catkin_LIBRARIES})
add_dependencies(marker_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(controller_template src/controller_template.cpp src/bag_manager.cpp)
target_link_libraries(controller_template ${catkin_LIBRARIES})
add_dependencies(controller_template ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(controller_action_node src/controller_action_node.cpp)
target_link_libraries(controller_action_node ${catkin_LIBRARIES})
add_dependencies(controller_action_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(ros_control_interface src/ros_control_interface.cpp)
target_link_libraries(ros_control_interface ${catkin_LIBRARIES})
add_dependencies(ros_control_interface ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(tf_manager src/tf_manager.cpp)
target_link_libraries(tf_manager ${catkin_LIBRARIES})
add_dependencies(tf_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(PROGRAMS src/generic_control_toolbox/manage_actionlib.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
