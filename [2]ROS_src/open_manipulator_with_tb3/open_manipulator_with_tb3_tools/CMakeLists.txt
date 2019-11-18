################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_with_tb3_tools)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  moveit_msgs
  trajectory_msgs
  open_manipulator_msgs
  trajectory_msgs
  ar_track_alvar_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  smach
  smach_ros
  trac_ik_lib
)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs nav_msgs moveit_msgs trajectory_msgs open_manipulator_msgs ar_track_alvar_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface smach smach_ros trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(open_manipulator_with_tb3_moveit_bridge src/moveit_bridge.cpp)
add_dependencies(open_manipulator_with_tb3_moveit_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_with_tb3_moveit_bridge ${catkin_LIBRARIES})

add_executable(open_manipulator_with_tb3_gazebo_bridge src/gazebo_bridge.cpp)
add_dependencies(open_manipulator_with_tb3_gazebo_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_with_tb3_gazebo_bridge ${catkin_LIBRARIES})

add_executable(open_manipulator_with_tb3_gripper_server src/gripper_server.cpp)
add_dependencies(open_manipulator_with_tb3_gripper_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_with_tb3_gripper_server ${catkin_LIBRARIES})

add_executable(trac_ik_teleop src/trac_ik_teleop.cpp)
target_link_libraries(trac_ik_teleop
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/pick_and_place_state_machine
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS open_manipulator_with_tb3_moveit_bridge open_manipulator_with_tb3_gazebo_bridge open_manipulator_with_tb3_gripper_server trac_ik_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
