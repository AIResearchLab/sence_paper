#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include "sence_hardware/sence_hardware_interface.hpp"

using namespace sence_hardware;

int main(int argc, char** argv) {

  ros::init(argc, argv, "sence_hardware_node");

  ros::NodeHandle root_nh; // root namespace
  ros::NodeHandle robot_nh("~"); // sence namespace

  SenceHardwareInterface hw(robot_nh);
  if (!hw.init(root_nh, robot_nh)) {
    ROS_INFO("Failed to initialize!");
    return 1;
  } else { ROS_INFO("Started OK..."); }

  controller_manager::ControllerManager cm(&hw, robot_nh);

  ros::AsyncSpinner cm_spinner(0); // arg is the number of threads to use (0 is all of them)
  cm_spinner.start();

  // get the config rate setting or use 1Hz default
  float config_hertz;
  robot_nh.getParam("rate_hertz", config_hertz);
  if (config_hertz == 0) {
    printf(">>>>>> No rate found in config, using default 1Hz");
    config_hertz = 1;
  } else {
    printf(">>>>>> Found rate in config: %f\n", config_hertz);
  }

  ros::Rate rate_hertz(config_hertz);

  ROS_INFO("Entering loop...");
  ros::Time prev_time = ros::Time::now();

  while (ros::ok()) {
    const ros::Time current_time = ros::Time::now();
    const ros::Duration period = current_time - prev_time;
    prev_time = current_time;
    hw.read(current_time, period);
    cm.update(current_time, period);
    hw.write(current_time, period);
    rate_hertz.sleep();
    ROS_INFO("looped ok");
  }

  cm_spinner.stop();

  return 0;
}