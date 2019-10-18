#rviz
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch open_manipulator_with_tb3_description open_manipulator_with_tb3_rviz.launch

#bring up
[turtlebot3]
export TURTLEBOT3_MODEL=${TB3_MODEL}
ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=om_with_tb3 set_lidar_frame_id:=om_with_tb3/base_scan
  #/om_with_tb3/joint_states (publish)

ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_realsense.launch

[remote PC]
ROS_NAMESPACE=om_with_tb3 roslaunch open_manipulator_with_tb3_tools om_with_tb3_robot.launch
 #/om_with_tb3/joint_states (subscribe)
 #/tf (publish)
 #/tf_static (publish)

#trac_ik
ROS_NAMESPACE=om_with_tb3 roslaunch open_manipulator_with_tb3_tools trac_ik_teleop.launch

#teleop
//ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
//python trac_ik_pub.py
ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_teleop turtlebot3_teleop_xboxone_jspark.launch

#slam
[remote PC]
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch open_manipulator_with_tb3_tools slam.launch use_platform:=true

ROS_NAMESPACE=om_with_tb3 rosrun map_server map_saver -f ~/map

#navigation
[remote PC]
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch open_manipulator_with_tb3_tools navigation.launch use_platform:=true map_file:=$HOME/map.yaml

#MoveIt
[remote PC]
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch open_manipulator_with_tb3_tools manipulation.launch use_platform:=true
