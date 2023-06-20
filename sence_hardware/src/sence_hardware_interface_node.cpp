#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <sence_hardware/sence_hardware_interface.hpp>

using namespace sence_hardware;

// main node running loop for ros control
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sence_hardware_interface_node");

  ros::NodeHandle nh;
  ros::NodeHandle rnh("~");  // robot private namespace

  SenceHardwareInterface hw;
  const bool success hw.init(nh, rnh);

  ros::Duration period(hw.get_update_duration());  // update rate

  // create controller manager
  controller_manager::ControllerManager cm(&hw, rnh);

  ros::AsyncSpinner cm_spinner(1);  // controller manager callback thread
  cm_spinner.start();

  ROS_INFO("sence_hardware_interface_node started");

  while (ros::ok())
  {
    hw.read(ros::Time::now(), period);

    cm.update(ros::Time::now(), period);

    hw.write(ros::Time::now(), period);

    period.sleep();
  }

  cm_spinner.stop();
  return 0;
}
