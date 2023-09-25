#pragma once

#include <vector>
#include <string>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

// Defaults
#define PROTOCOL_VERSION 2.0
#define BAUDRATE 3000000
#define DEVICENAME "/dev/ttyUSB0"
// Instruction Addresses for DXL MX-28
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
// Data Byte Lengths
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_POSITION 4

#define TORQUE_ON 1
#define TORQUE_OFF 0

#include "sence_hardware/joint.hpp"

namespace sence_hardware
{

  class SenceHardwareInterface : public hardware_interface::RobotHW
  {

  public:
    SenceHardwareInterface(ros::NodeHandle &robot_nh);
    ~SenceHardwareInterface();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh) override;
    void read(const ros::Time &time, const ros::Duration &period) override;
    void write(const ros::Time &time, const ros::Duration &period) override;

  private:
    std::vector<Joint> joints;

    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_pos_interface;

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    dynamixel::GroupSyncWrite groupSyncWrite;
    dynamixel::GroupSyncRead groupSyncRead;

    uint8_t dxl_error;
    int dxl_comm_result;
    bool dxl_addparam_result;
    bool dxl_getdata_result;
    uint8_t param_goal_position[4];
  };

} // namespace sence_hardware
