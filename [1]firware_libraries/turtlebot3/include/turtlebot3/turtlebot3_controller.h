/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef TURTLEBOT3_CONTROLLER_H_
#define TURTLEBOT3_CONTROLLER_H_

#include <RC100.h>

#include <geometry_msgs/Twist.h>

//#include <open_manipulator_libs.h>

#define CONST_VEL 0.4

#define VELOCITY_LINEAR_X                0.01   // m/s
#define VELOCITY_LINEAR_Y                0.01   // m/s
#define VELOCITY_ANGULAR_Z               0.1    // rad/s

#define DEBUG_SERIAL SerialBT2

class Turtlebot3Controller
{
 public:
  Turtlebot3Controller();
  ~Turtlebot3Controller();

  bool init(float max_lin_vel, float max_ang_vel, uint8_t scale_lin_vel = 1, uint8_t scale_ang_vel = 1);

  void getRCdata(float *get_cmd_vel, double *get_time_joint_gripper, bool &RCinput);
  //void getRCdata(OpenManipulator* open_manipulator, float *get_cmd_vel, double *get_time_joint_gripper, bool &RCinput);

 private:
  geometry_msgs::Twist cmd_vel_;
  RC100 rc100_;

  double const_cmd_vel_;

  float max_lin_vel_;
  float min_lin_vel_;
  float max_ang_vel_;
  float min_ang_vel_;
  uint8_t scale_lin_vel_;
  uint8_t scale_ang_vel_;
};

#endif // TURTLEBOT3_CONTROLLER_H_
