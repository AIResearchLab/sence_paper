#pragma once

#define FREQUENCY_HERTZ 10

namespace sence_hardware
{
class SenceHardwareInterface :
  public hardware_interface::RobotHW,
  public hardware_interface::HardwareInterface
{
public:
  SenceHardwareInterface(){}
  ~SenceHardwareInterface(){}

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

  double get_update_duration(){
    return (1 / FREQUENCY_HERTZ);
  }

protected:
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  // hardware joint handles
  std::vector<HardwareJoint> joints;

  // simulation flag
  bool simulated;
};

}  // namespace sence_hardware
