#ifndef TRACK_IK_TELEOP_H
#define TRACK_IK_TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <open_manipulator_msgs/KinematicsPose.h>
#include <open_manipulator_msgs/JointPosition.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <termios.h>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

namespace trac_ik_teleop
{

class TracIkTeleop
{
 private:
  #define NUM_OF_JOINT  4

  ros::NodeHandle nh;  
  ros::Subscriber current_joint_states_sub_;  
  ros::Subscriber target_kinematics_pose_sub_;  
  ros::Publisher current_kinematics_pose_pub_;
  ros::Publisher target_joint_states_pub_;
  ros::Publisher target_joint_trajectory_point_pub_;
  struct termios oldt_;

  double timeout;
  double eps = 1e-5;
  int rc;  

  KDL::Chain chain;
  KDL::Frame current_pose_frame;  
  KDL::Frame target_pose_frame;  
  KDL::JntArray ll, ul; //lower joint limits, upper joint limit    
  KDL::JntArray current_q_jntarray;     
  KDL::JntArray target_q_jntarray;  

  open_manipulator_msgs::KinematicsPose current_pose_msg;
  open_manipulator_msgs::JointPosition target_q_msg;
  //sensor_msgs::JointState target_q_msg;

  std_msgs::Float64MultiArray target_point_msg;
  
  std::string chain_start, chain_end, urdf_param;
  std::vector<std::string> joint_name;
 
  TRAC_IK::TRAC_IK *tracik_solver;
  KDL::ChainFkSolverPos_recursive *fk_solver;

 public:

  TracIkTeleop(std::string chain_start, std::string chain_end, std::string urdf_param);
  ~TracIkTeleop();

  void init();

  void initSubscriber();
  void initPublisher();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void targetkinematicsposeCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg); 

};
}

#endif //OPEN_MANIPULATOR_TELEOP_H