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

#include "../../include/turtlebot3/turtlebot3_motor_driver.h"

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  //left_wheel_id_(DXL_LEFT_ID),
  //right_wheel_id_(DXL_RIGHT_ID)
  left_rear_wheel_id_(DXL_LEFT_REAR_ID), right_rear_wheel_id_(DXL_RIGHT_REAR_ID),
  left_front_wheel_id_(DXL_LEFT_FRONT_ID), right_front_wheel_id_(DXL_RIGHT_FRONT_ID)
{
  torque_ = false;
  dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
}

bool Turtlebot3MotorDriver::init(String turtlebot3)
{
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }

  // Enable Dynamixel Torque
  //setTorque(true);
  setTorque(left_rear_wheel_id_, true);
  setTorque(right_rear_wheel_id_, true);
  setTorque(left_front_wheel_id_, true);
  setTorque(right_front_wheel_id_, true);

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  
  if (turtlebot3 == "Burger")
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  else if (turtlebot3 == "Waffle or Waffle Pi")
    dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
  else
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

//bool Turtlebot3MotorDriver::setTorque(bool onoff)
bool Turtlebot3MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  torque_ = onoff;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if(dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

bool Turtlebot3MotorDriver::getTorque() // this should be modified to function of each motor
{
  return torque_;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  //setTorque(false);
  setTorque(left_rear_wheel_id_, false);
  setTorque(right_rear_wheel_id_, false);
  setTorque(left_front_wheel_id_, false);
  setTorque(right_front_wheel_id_, false);

  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}

//bool Turtlebot3MotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
bool Turtlebot3MotorDriver::readEncoder(uint8_t id, int32_t &value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  //// Set parameter
  //dxl_addparam_result = groupSyncReadEncoder_->addParam(left_wheel_id_);
  //if (dxl_addparam_result != true)
  //  return false;

  //dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
  //if (dxl_addparam_result != true)
  //  return false;

  //// Syncread present position
  //dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  //if (dxl_comm_result != COMM_SUCCESS)
  // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  //// Check if groupSyncRead data of Dynamixels are available
  //dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  //if (dxl_getdata_result != true)
  //  return false;

  //dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  //if (dxl_getdata_result != true)
  //  return false;

  //// Get data
  //left_value  = groupSyncReadEncoder_->getData(left_wheel_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  //right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(id);
  if (dxl_addparam_result != true)
    return false; 

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  value  = groupSyncReadEncoder_->getData(id,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);


  groupSyncReadEncoder_->clearParam();
  return true;
}

//bool Turtlebot3MotorDriver::writeVelocity(int64_t left_value, int64_t right_value)
bool Turtlebot3MotorDriver::writeVelocity(int64_t lr_value, int64_t rr_value, int64_t lf_value, int64_t rf_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  //uint8_t left_data_byte[4] = {0, };
  //uint8_t right_data_byte[4] = {0, };

  //left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_value));
  //left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_value));
  //left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_value));
  //left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_value));

  //dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_wheel_id_, (uint8_t*)&left_data_byte);
  //if (dxl_addparam_result != true)
  //  return false;

  //right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_value));
  //right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_value));
  //right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_value));
  //right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_value));

  //dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_wheel_id_, (uint8_t*)&right_data_byte);
  //if (dxl_addparam_result != true)
  //  return false;

  uint8_t lr_data_byte[4] = {0, };
  uint8_t rr_data_byte[4] = {0, };
  uint8_t lf_data_byte[4] = {0, };
  uint8_t rf_data_byte[4] = {0, };

  lr_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(lr_value));
  lr_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(lr_value));
  lr_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(lr_value));
  lr_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(lr_value));

  rr_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(rr_value));
  rr_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(rr_value));
  rr_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(rr_value));
  rr_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(rr_value));

  lf_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(lf_value));
  lf_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(lf_value));
  lf_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(lf_value));
  lf_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(lf_value));

  rf_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(rf_value));
  rf_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(rf_value));
  rf_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(rf_value));
  rf_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(rf_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_rear_wheel_id_, (uint8_t*)&lr_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_rear_wheel_id_, (uint8_t*)&rr_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_front_wheel_id_, (uint8_t*)&lf_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_front_wheel_id_, (uint8_t*)&rf_data_byte);
  if (dxl_addparam_result != true)
    return false;



  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

/*
// four diffential drive
//bool Turtlebot3MotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value);
bool Turtlebot3MotorDriver::controlMotor(const float wheel_radius, const float separation_x, const float separation_y, float* value)
{
  bool dxl_comm_result = false;  

  double wheel1_spd_cmd, wheel2_spd_cmd, wheel3_spd_cmd, wheel4_spd_cmd;
  double lin_vel1, lin_vel2, lin_vel3, lin_vel4;

  float goal_linear_velocity = value[0];
  float goal_angular_velocity = value[1];  

  // four diffential drive
  wheel1_spd_cmd = goal_linear_velocity - (sqrt(separation_x * separation_x + separation_y * separation_y) * goal_angular_velocity) * cos(atan(+separation_y / separation_x));
  wheel2_spd_cmd = goal_linear_velocity + (sqrt(separation_x * separation_x + separation_y * separation_y) * goal_angular_velocity) * cos(atan(-separation_y / separation_x));
  wheel3_spd_cmd = goal_linear_velocity - (sqrt(separation_x * separation_x + separation_y * separation_y) * goal_angular_velocity) * cos(atan(-separation_y / separation_x));
  wheel4_spd_cmd = goal_linear_velocity + (sqrt(separation_x * separation_x + separation_y * separation_y) * goal_angular_velocity) * cos(atan(+separation_y / separation_x));

  lin_vel1  = constrain(wheel1_spd_cmd  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  lin_vel2  = constrain(wheel2_spd_cmd  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  lin_vel3  = constrain(wheel3_spd_cmd  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  lin_vel4  = constrain(wheel4_spd_cmd  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

  //float wheel_velocity_cmd[2];

  //float lin_vel; = value[LEFT];
  //float ang_vel = value[RIGHT];

  //wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  //wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  //wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  //wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

  dxl_comm_result = writeVelocity((int64_t)lin_vel1, (int64_t)lin_vel2, (int64_t)lin_vel3, (int64_t)lin_vel4);  
  if (dxl_comm_result == false)
    return false;

  return true;
}
*/

//four mecanum wheel drive
bool Turtlebot3MotorDriver::controlMotor(const float wheel_radius, const float separation_x, const float separation_y, float* value)
{
  bool dxl_comm_result = false;  

  double wheel1_spd_cmd, wheel2_spd_cmd, wheel3_spd_cmd, wheel4_spd_cmd;
  double lin_vel1, lin_vel2, lin_vel3, lin_vel4;

  float goal_linear_x_velocity = value[0];
  float goal_angular_velocity  = value[1];
  float goal_linear_y_velocity = value[2];    
  
  //wheel1_spd_cmd = (goal_linear_x_velocity + goal_linear_y_velocity - (separation_x + separation_y) * goal_angular_velocity);
  //wheel2_spd_cmd = (goal_linear_x_velocity - goal_linear_y_velocity + (separation_x + separation_y) * goal_angular_velocity);
  //wheel3_spd_cmd = (goal_linear_x_velocity - goal_linear_y_velocity - (separation_x + separation_y) * goal_angular_velocity);
  //wheel4_spd_cmd = (goal_linear_x_velocity + goal_linear_y_velocity + (separation_x + separation_y) * goal_angular_velocity);

  wheel1_spd_cmd = (goal_linear_x_velocity + goal_linear_y_velocity - (separation_x) * goal_angular_velocity);
  wheel2_spd_cmd = (goal_linear_x_velocity - goal_linear_y_velocity + (separation_x) * goal_angular_velocity);
  wheel3_spd_cmd = (goal_linear_x_velocity - goal_linear_y_velocity - (separation_x) * goal_angular_velocity);
  wheel4_spd_cmd = (goal_linear_x_velocity + goal_linear_y_velocity + (separation_x) * goal_angular_velocity);

  lin_vel1  = constrain(wheel1_spd_cmd  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  lin_vel2  = constrain(wheel2_spd_cmd  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  lin_vel3  = constrain(wheel3_spd_cmd  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  lin_vel4  = constrain(wheel4_spd_cmd  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);


  //float wheel_velocity_cmd[2];

  //float lin_vel; = value[LEFT];
  //float ang_vel = value[RIGHT];

  //wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  //wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  //wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  //wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

  dxl_comm_result = writeVelocity((int64_t)lin_vel1, (int64_t)lin_vel2, (int64_t)lin_vel3, (int64_t)lin_vel4);  
  if (dxl_comm_result == false)
    return false;

  return true;
}
