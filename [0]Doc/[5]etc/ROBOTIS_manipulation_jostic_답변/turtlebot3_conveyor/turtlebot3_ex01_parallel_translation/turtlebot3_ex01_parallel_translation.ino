#include "turtlebot3_conveyer_config.h"
#include "Chain.h"
#include <DynamixelWorkbench.h>

#define BAUDRATE  1000000
#define DEVICE_NAME ""

#define MANIPULATOR

RC100 rc100;
DynamixelStatus conveyer;

DynamixelWorkbench dxl_wb;
uint8_t conveyer_joint[4] = {JOINT_L_R, JOINT_R_R, JOINT_L_F, JOINT_R_F};
uint8_t conveyer_wheel[4] = {WHEEL_L_R, WHEEL_R_R, WHEEL_L_F, WHEEL_R_F};

float previous_time[3] = {0.0, 0.0, 0.0};

void setup()
{
  Serial.begin(57600);
  // while(!Serial);

  rc100.begin(1);

  dxl_wb.begin(DEVICE_NAME, BAUDRATE);

  for (uint8_t cnt = 21; cnt < 29; cnt++)
  {
    dxl_wb.ping(cnt);
    dxl_wb.itemWrite(cnt, "Torque_Enable", true);
  }


  dxl_wb.addSyncWrite("Goal_Position");
  dxl_wb.addSyncWrite("Goal_Velocity");

  dxl_wb.syncWrite(conveyer_joint, 4, "Goal_Position", conveyer.setJointAngle());
  dxl_wb.syncWrite(conveyer_wheel, 4, "Goal_Velocity", conveyer.setWheelVel());

#ifdef MANIPULATOR
  initManipulator();
#endif
}

void loop()
{
  float present_time = (float)(millis()/1000.0f);

  getRC100Data();
  if(present_time - previous_time[0] >= LOOP_TIME)
  {
    dxl_wb.syncWrite(conveyer_joint, 4, "Goal_Position", conveyer.setJointAngle());
    dxl_wb.syncWrite(conveyer_wheel, 4, "Goal_Velocity", conveyer.setWheelVel());

    previous_time[0] = (float)(millis()/1000.0f);
  } 

#ifdef MANIPULATOR
  //solve Kinematics
  if(present_time-previous_time[1] >= ROBOT_STATE_UPDATE_TIME)
  {
    updateAllJointAngle();
    chain.forward();
    
    previous_time[1] = (float)(millis()/1000.0f);
  }

  //Joint Control
  if(present_time-previous_time[2] >= ACTUATOR_CONTROL_TIME)
  {
    chain.setPresentTime((float)(millis()/1000.0f));
    chain.jointControl();

    previous_time[2] = (float)(millis()/1000.0f);
  }
#endif
}

void getRC100Data()
{
  static bool motor_command = true;

  if (rc100.available())
  {    
    int rcData = rc100.readData();

    if (rcData & RC100_BTN_5)
    {
      motor_command = !motor_command;
    }

    if (motor_command)
    {
      conveyer.getDirection(rcData);
      delay(1);

      conveyer.setParams();

#ifdef MANIPULATOR
      if (rcData & RC100_BTN_6)
      {

        std::vector<float> goal_position;

        goal_position.push_back(0.0f);
        goal_position.push_back(-95.0f * DEG2RAD);
        goal_position.push_back(79.0f * DEG2RAD);
        goal_position.push_back(-20.0f * DEG2RAD);

        chain.jointMove(goal_position, 2.0f);
      }
#endif
    }
    else
    {
#ifdef MANIPULATOR
      if (rcData & RC100_BTN_U)
        chain.setMove(TOOL, OM_MATH::makeVector3(0.007f, 0.0, 0.0), 0.16f);
      else if (rcData & RC100_BTN_D)
        chain.setMove(TOOL, OM_MATH::makeVector3(-0.007f, 0.0, 0.0), 0.16f);
      else if (rcData & RC100_BTN_L)
        chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.007f, 0.0), 0.16f);
      else if (rcData & RC100_BTN_R)
        chain.setMove(TOOL, OM_MATH::makeVector3(0.0, -0.007f, 0.0), 0.16f);
      else if (rcData & RC100_BTN_1)
        chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.0, 0.007f), 0.16f);
      else if (rcData & RC100_BTN_3)
        chain.setMove(TOOL, OM_MATH::makeVector3(0.0, 0.0, -0.007f), 0.16f);
      else if (rcData & RC100_BTN_2)
      {
        float grip_value = chain.getComponentToolValue(TOOL) + 0.030f;
        if (grip_value >= 0.907f)
          grip_value = 0.907f;

        chain.toolMove(TOOL, grip_value);
      }
      else if (rcData & RC100_BTN_4)
      {
        float grip_value = chain.getComponentToolValue(TOOL) - 0.030f;
        if (grip_value <= -1.130f)
          grip_value = -1.130f;

        chain.toolMove(TOOL, grip_value);
      }
      else if (rcData & RC100_BTN_5)
      {
        std::vector<float> goal_position;

        goal_position.push_back(0.0f);
        goal_position.push_back(-60.0f * DEG2RAD);
        goal_position.push_back(20.0f * DEG2RAD);
        goal_position.push_back(40.0f * DEG2RAD);

        chain.jointMove(goal_position, 1.0f);
      }
      else if (rcData & RC100_BTN_6)
      {
        std::vector<float> goal_position;

        goal_position.push_back(0.0f);
        goal_position.push_back(0.0f);
        goal_position.push_back(0.0f);
        goal_position.push_back(0.0f);

        chain.jointMove(goal_position, 1.0f);
      }
#endif
    }
  }
}