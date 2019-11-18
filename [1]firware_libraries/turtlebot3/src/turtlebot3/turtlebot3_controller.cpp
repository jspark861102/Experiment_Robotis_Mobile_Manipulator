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

#include "../../include/turtlebot3/turtlebot3_controller.h"

Turtlebot3Controller::Turtlebot3Controller()
{
  const_cmd_vel_ = CONST_VEL;
}

Turtlebot3Controller::~Turtlebot3Controller()
{
  DEBUG_SERIAL.end();
}

bool Turtlebot3Controller::init(float max_lin_vel, float max_ang_vel, uint8_t scale_lin_vel, uint8_t scale_ang_vel)
{
  DEBUG_SERIAL.begin(57600);
  // 57600bps baudrate for RC100 control
  rc100_.begin(1);  

  max_lin_vel_ = max_lin_vel;
  min_lin_vel_ = (-1)*max_lin_vel;
  max_ang_vel_ = max_ang_vel;
  min_ang_vel_ = (-1)*max_ang_vel;
  scale_lin_vel_ = scale_lin_vel;
  scale_ang_vel_ = scale_ang_vel;

  DEBUG_SERIAL.println("Success to init Controller");
  return true;
}


// four diffential drive
//void Turtlebot3Controller::getRCdata(OpenManipulator* open_manipulator, float *get_cmd_vel, double *get_time_joint_gripper, bool &RCinput)
void Turtlebot3Controller::getRCdata(float *get_cmd_vel, double *get_time_joint_gripper, bool &RCinput)
{
  uint16_t received_data = 0;

  static float lin_x = 0.0, ang_z = 0.0;
  static double time_joint_gripper[4] = {0.0, 0.0, 0.0, 0.0};
  static double time_joint_gripper_a[4] = {0.0, -1.57, 1.20, 0.6};
  static double time_joint_gripper_b[4] = {0.0, 0.0, 0.0, 0.0};
  static double time_joint_gripper_c[4] = {0.0, 0.8, 0.0, -0.8};
  static double time_joint_gripper_d[4] = {0.0, 0.8, -1.0, 1.57};
  static bool joint_RCinput = false;
  
  if (rc100_.available())
  {
    received_data = rc100_.readData();

    if (received_data & RC100_BTN_U)
    {
      lin_x += VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_D)
    {
      lin_x -= VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_L)
    {
      ang_z += VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_R)
    {
      ang_z -= VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_6)
    {
      lin_x = const_cmd_vel_;
      ang_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      lin_x = 0.0;
      ang_z = 0.0;
    }
    else if (received_data & RC100_BTN_1)
    {
      joint_RCinput = true;
      for (int index = 0; index < 4; index++)
      {
        time_joint_gripper[index] = time_joint_gripper_a[index];
      }           
    }
    else if (received_data & RC100_BTN_2)
    {
      joint_RCinput = true;
      for (int index = 0; index < 4; index++)
      {
        time_joint_gripper[index] = time_joint_gripper_b[index];
      }      
    }
    else if (received_data & RC100_BTN_3)
    {
      joint_RCinput = true;
      for (int index = 0; index < 4; index++)
      {
        time_joint_gripper[index] = time_joint_gripper_c[index];
      } 
      //open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(0.0, 0.0, 0.007), 0.16);
    }
    else if (received_data & RC100_BTN_4)
    {
      for (int index = 0; index < 4; index++)
      {
        time_joint_gripper[index] = time_joint_gripper_d[index];
      } 
    }
    else
    {
      lin_x = lin_x;
      ang_z = ang_z;

      joint_RCinput = joint_RCinput;      
      for (int index = 0; index < 4; index++)
      {
        time_joint_gripper[index] = time_joint_gripper[index];
      }      
    }

    lin_x = constrain(lin_x, min_lin_vel_, max_lin_vel_);
    ang_z = constrain(ang_z, min_ang_vel_, max_ang_vel_);

    get_cmd_vel[0] = lin_x;
    get_cmd_vel[1] = ang_z;
    get_cmd_vel[2] = 0.0; //lin_y

    RCinput = joint_RCinput;
     for (int index = 0; index < 4; index++)
      {
        get_time_joint_gripper[index] = time_joint_gripper[index];
      }          
  }
}

/*
//four mecanum wheel drive
void Turtlebot3Controller::getRCdata(float *get_cmd_vel, double *get_time_joint_gripper, bool &RCinput)
{
  uint16_t received_data = 0;

  static float lin_x = 0.0, ang_z = 0.0, lin_y = 0.0;
  static double time_joint_gripper[4] = {0.0, 0.0, 0.0, 0.0};
  static double time_joint_gripper_a[4] = {0.0, -1.57, 1.20, 0.6};
  static double time_joint_gripper_b[4] = {0.0, 0.0, 0.0, 0.0};
  static double time_joint_gripper_c[4] = {0.0, 0.8, 0.0, -0.8};
  static double time_joint_gripper_d[4] = {0.0, 0.8, -1.0, 1.57};
  static bool joint_RCinput = false;
  
  if (rc100_.available())
  {
    received_data = rc100_.readData();

    if (received_data & RC100_BTN_U)
    {
      lin_x += VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_D)
    {
      lin_x -= VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_L)
    {
      lin_y += VELOCITY_LINEAR_Y * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_R)
    {
      lin_y -= VELOCITY_LINEAR_Y * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_6)
    {
      lin_x = const_cmd_vel_;
      lin_y = 0.0;
      ang_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      lin_x = 0.0;
      lin_y = 0.0;
      ang_z = 0.0;
    }    
    else if (received_data & RC100_BTN_2)
    {
      ang_z += VELOCITY_ANGULAR_Z * scale_ang_vel_;          
    }
    else if (received_data & RC100_BTN_4)
    {
      ang_z -= VELOCITY_ANGULAR_Z * scale_ang_vel_;          
    }
    else if (received_data & RC100_BTN_1)
    {
      joint_RCinput = true;
      for (int index = 0; index < 4; index++)
      {
        time_joint_gripper[index] = time_joint_gripper_a[index];
      }           
    }
    else if (received_data & RC100_BTN_3)
    {
      joint_RCinput = true;
      for (int index = 0; index < 4; index++)
      {
        time_joint_gripper[index] = time_joint_gripper_c[index];
      } 
      //open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(0.0, 0.0, 0.007), 0.16);
    }    
    else
    {
      lin_x = lin_x;
      lin_y = lin_y;
      ang_z = ang_z;

      joint_RCinput = joint_RCinput;      
      for (int index = 0; index < 4; index++)
      {
        time_joint_gripper[index] = time_joint_gripper[index];
      }      
    }

    lin_x = constrain(lin_x, min_lin_vel_, max_lin_vel_);
    lin_y = constrain(lin_y, min_lin_vel_, max_lin_vel_);
    ang_z = constrain(ang_z, min_ang_vel_, max_ang_vel_);

    get_cmd_vel[0] = lin_x;
    get_cmd_vel[1] = ang_z;
    get_cmd_vel[2] = lin_y;

    RCinput = joint_RCinput;
     for (int index = 0; index < 4; index++)
      {
        get_time_joint_gripper[index] = time_joint_gripper[index];
      }          
  }
}*/
