#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

#define NUM_MOTORS 12
#define RATE_HERTZ 50

class MyRobotHardware
{
public:
  MyRobotHardware() : nh_("~") {
    
    // Initialize joint interfaces
    std::vector<hardware_interface::JointStateHandle> state_handles;
    std::vector<hardware_interface::JointHandle> pos_handles;
    std::vector<double> positions(12, 0.0);

    for (int i = 0; i < NUM_MOTORS; i++) {
        std::string joint_name = "joint_" + std::to_string(i + 1);

        // Create and register joint state handle
        hardware_interface::JointStateHandle state_handle(joint_name, &positions[i]);
        jnt_state_interface_.registerHandle(state_handle);
        state_handles.push_back(state_handle);

        // Create and register joint position handle
        hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(joint_name), &cmd_[i]);
        jnt_pos_interface_.registerHandle(pos_handle);
        pos_handles.push_back(pos_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);


    // Start the control loop
    control_loop_ = nh_.createTimer(ros::Duration(0.1), &MyRobotHardware::controlLoop, this);
  }

  void controlLoop(const ros::TimerEvent& event)
  {
    // Update joint states
    for (size_t i = 0; i < NUM_MOTORS; ++i)
    {
      // In a real implementation, read the joint states from your hardware
      // Here, we will simply increment the joint positions for demonstration purposes
      pos_[i] += 0.01;
      vel_[i] = 0.0;
      eff_[i] = 0.0;

      //// DYNAMIXEL READ STUFF GOES HERE ////
    }

    // Execute joint commands
    for (size_t i = 0; i < NUM_MOTORS; ++i)
    {
      // In a real implementation, send the joint commands to your hardware
      // Here, we will simply display the commands for demonstration purposes
      ROS_INFO_STREAM("Joint " << i + 1 << " command: " << cmd_[i]);

      //// DYNAMIXEL WRITE STUFF GOES HERE ////
    }
  }

private:
  static const int NUM_JOINTS = NUM_MOTORS;
  double pos_[NUM_JOINTS];
  double vel_[NUM_JOINTS];
  double eff_[NUM_JOINTS];
  double cmd_[NUM_JOINTS];

  ros::NodeHandle nh_;
  ros::Timer control_loop_;

  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_robot_hardware");
  ros::NodeHandle nh;

  MyRobotHardware robot;

  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(RATE_HERTZ);  // Control loop rate

  while (ros::ok())
  {
    robot.controlLoop(ros::TimerEvent());  // Call control loop explicitly

    cm.update(ros::Time::now(), ros::Duration(1.0 / RATE_HERTZ));  // Update the controllers

    rate.sleep();
  }

  return 0;
}
