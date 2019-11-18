#include "open_manipulator_with_tb3_tools/trac_ik_teleop.h"

using namespace trac_ik_teleop;

TracIkTeleop::TracIkTeleop(std::string _chain_start, std::string _chain_end, std::string _urdf_param)
    :nh(""),   
     chain_start(""),
     chain_end(""),
     urdf_param("")
{
  chain_start = _chain_start;
  chain_end = _chain_end;
  urdf_param = _urdf_param;
  init();
  initPublisher();
  initSubscriber();  
}

TracIkTeleop::~TracIkTeleop()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

void TracIkTeleop::initSubscriber()
{
  current_joint_states_sub_ = nh.subscribe("joint_states", 10, &TracIkTeleop::jointStatesCallback, this);
  target_kinematics_pose_sub_ = nh.subscribe("target_kinematics_pose", 10, &TracIkTeleop::targetkinematicsposeCallback, this);  
}

void TracIkTeleop::initPublisher()
{  
  current_kinematics_pose_pub_ = nh.advertise<open_manipulator_msgs::KinematicsPose>("current_kinematics_pose", 10);     
  target_joint_states_pub_ = nh.advertise<open_manipulator_msgs::JointPosition>("target_joint_states", 10);     
  target_joint_trajectory_point_pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_trajectory_point", 10);     
}

void TracIkTeleop::targetkinematicsposeCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  KDL::Rotation rot =  KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  KDL::Vector target_position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  target_pose_frame = KDL::Frame(rot, target_position);

  rc = tracik_solver->CartToJnt(current_q_jntarray, target_pose_frame, target_q_jntarray);  
  printf("track_ik success?  %d\n", rc);

  target_point_msg.data.at(0) = 0.5;
  if(rc > 0)
  {
    for(int i=0; i<chain.getNrOfJoints(); i++)
    {
      target_q_msg.joint_name.at(i) = joint_name[i];
      target_q_msg.position.at(i) = target_q_jntarray.data(i);
      target_point_msg.data.at(i+1) = target_q_jntarray.data(i);
    }       
  }
  else
  {
    for(int i=0; i<chain.getNrOfJoints(); i++)
    {
      target_q_msg.joint_name.at(i) = joint_name[i];
      target_q_msg.position.at(i) = current_q_jntarray.data(i);
      target_point_msg.data.at(i+1) = current_q_jntarray.data(i);
    }  
  }  
  target_joint_states_pub_.publish(target_q_msg);
  target_joint_trajectory_point_pub_.publish(target_point_msg);

  double cx, cy, cz, cw;
  double tx, ty, tz, tw;
  current_pose_frame.M.GetQuaternion(cx, cy, cz, cw);
  target_pose_frame.M.GetQuaternion(tx, ty, tz, tw);
  printf("current_q_jntarray %f %f %f %f\n", current_q_jntarray.data(0), current_q_jntarray.data(1), current_q_jntarray.data(2), current_q_jntarray.data(3));
  printf("target_q_jntarray  %f %f %f %f\n", target_q_jntarray.data(0), target_q_jntarray.data(1), target_q_jntarray.data(2), target_q_jntarray.data(3));    
  printf("current_pose_frame %f %f %f %f %f %f %f\n", current_pose_frame.p.x(), current_pose_frame.p.y(), current_pose_frame.p.z(), cx, cy, cz, cw);
  printf("target_pose_frame  %f %f %f %f %f %f %f\n\n", target_pose_frame.p.x(), target_pose_frame.p.y(), target_pose_frame.p.z(), tx, ty, tz, tw);      
}

void TracIkTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{   
  //JointState to Jnt Array for solver  
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))
    {
      current_q_jntarray(0) = (msg->position.at(i));
      joint_name.push_back("joint1");
    }
    else if(!msg->name.at(i).compare("joint2"))
    {
      current_q_jntarray(1) = (msg->position.at(i));
      joint_name.push_back("joint2");
    }
    else if(!msg->name.at(i).compare("joint3"))
    {
      current_q_jntarray(2) = (msg->position.at(i));
      joint_name.push_back("joint3");
    }
    else if(!msg->name.at(i).compare("joint4"))
    {
      current_q_jntarray(3) = (msg->position.at(i));
      joint_name.push_back("joint4");
    }
  }
  //printf("%f %f %f %f \n", current_q_jntarray(0), current_q_jntarray(1), current_q_jntarray(2), current_q_jntarray(3));
  
  //fk_solver
  fk_solver->JntToCart(current_q_jntarray, current_pose_frame);
  double rx, ry, rz, rw;
  current_pose_frame.M.GetQuaternion(rx, ry, rz, rw);

  current_pose_msg.pose.position.x = current_pose_frame.p.x();
  current_pose_msg.pose.position.y = current_pose_frame.p.y();
  current_pose_msg.pose.position.z = current_pose_frame.p.z();
  current_pose_msg.pose.orientation.x = rx;
  current_pose_msg.pose.orientation.y = ry;
  current_pose_msg.pose.orientation.z = rz;
  current_pose_msg.pose.orientation.w = rw;
  current_kinematics_pose_pub_.publish(current_pose_msg);  
  
  //printf("current_q_jntarray   %f %f %f %f\n", current_q_jntarray.data(0), current_q_jntarray.data(1), current_q_jntarray.data(2), current_q_jntarray.data(3));
  //printf("current_pose_frame   %f %f %f %f %f %f %f\n\n", current_pose_frame.p.x(), current_pose_frame.p.y(), current_pose_frame.p.z(), rx, ry, rz, rw);
}

void TracIkTeleop::init()
{  
  tracik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param);
  bool valid = tracik_solver->getKDLChain(chain); 
  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }
  valid = tracik_solver->getKDLLimits(ll, ul); 
  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }  
  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());
  ROS_INFO("Using %d joints", chain.getNrOfJoints()); 

  current_q_jntarray.resize(chain.getNrOfJoints());
  target_q_jntarray.resize(chain.getNrOfJoints());
  target_point_msg.data.resize(chain.getNrOfJoints() + 1);
  target_q_msg.joint_name.resize(chain.getNrOfJoints());
  target_q_msg.position.resize(chain.getNrOfJoints());

  fk_solver = new KDL::ChainFkSolverPos_recursive(chain);

  /*
  printf("%s\n",chain.getSegment(0).getName().c_str());
  printf("%s\n",chain.getSegment(1).getName().c_str());
  printf("%s\n",chain.getSegment(2).getName().c_str());
  printf("%s\n",chain.getSegment(3).getName().c_str());
  printf("%s\n\n",chain.getSegment(4).getName().c_str());
  printf("%s\n",chain.getSegment(0).getJoint().getName().c_str());
  printf("%s\n",chain.getSegment(1).getJoint().getName().c_str());
  printf("%s\n",chain.getSegment(2).getJoint().getName().c_str());
  printf("%s\n",chain.getSegment(3).getJoint().getName().c_str());
  printf("%d\n",chain.getNrOfSegments());
  printf("%d\n",chain.getNrOfJoints());  
  printf("ll data is %f %f %f %f \n",ll.data(0), ll.data(1), ll.data(2), ll.data(3));
  printf("ul data is %f %f %f %f \n",ul.data(0), ul.data(1), ul.data(2), ul.data(3));
  */
}

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "trac_ik_teleop");
  ros::NodeHandle p_nh("~"); 
  
  //get launch parameters
  std::string chain_start_main, chain_end_main, urdf_param_main;
  p_nh.param("chain_start", chain_start_main, std::string(""));
  p_nh.param("chain_end", chain_end_main, std::string(""));
  p_nh.param("urdf_param", urdf_param_main, std::string("/om_with_tb3/robot_description"));

  if (chain_start_main == "" || chain_end_main == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  ROS_INFO_STREAM("chain_start :" << chain_start_main << "chain_end :" << chain_end_main); 

  //class construction
  TracIkTeleop tik_teleop(chain_start_main, chain_end_main, urdf_param_main);

  //loop
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


















