#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <hardware_interface/joint_command_interface.h> //interface for sending commands
#include <hardware_interface/joint_state_interface.h> //interface for reading state
#include <hardware_interface/robot_hw.h> // hardware interface base class

#include <dynamixel_sdk/dynamixel_sdk.h> //import that sdk

#define JOINTS_NUM 12
#define RATE_HERTZ 200

class SenceHardwareInterface : public hardware_interface::RobotHW { //define our child class of robotHW

public:
  struct joint {
    double pos;
    double vel;
    double eff;
    double cmd;
  };

public:
  SenceHardwareInterface() {} //class constructor
  ~SenceHardwareInterface() {}//destructor

  
double get_update_duration(){return 1/RATE_HERTZ;} 
bool init(ros::NodeHandle &nh, ros::NodeHandle &robot_nh) {
      // get joint names
      std::vector<std::string> joint_names;
      robot_nh.getParam("joints", joint_names);
      ROS_INFO("found %i joints", (int)joint_names.size());

      // Initialize joint interfaces
      std::vector<hardware_interface::JointStateHandle> state_handles;
      std::vector<hardware_interface::JointHandle> pos_handles;

      for (int i = 0; i < joint_names.size(); i++) {
	
	joint j = {0, 0, 0, 0};
	joints.push_back(j);

        // Create and register joint state handle
        hardware_interface::JointStateHandle state_handle(joint_names[i], &j.pos, &j.vel, &j.eff);
        jnt_state_interface_.registerHandle(state_handle);
        state_handles.push_back(state_handle);

        // Create and register joint position handle
        hardware_interface::JointHandle pos_handle(state_handle, &j.cmd);
        jnt_pos_interface_.registerHandle(pos_handle);
        pos_handles.push_back(pos_handle);
      }

      registerInterface(&jnt_state_interface_);
      registerInterface(&jnt_pos_interface_);

      return true;
/**
      // connect and register the joint state interface
      hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
      jnt_state_interface.registerHandle(state_handle_a);

      hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
      jnt_state_interface.registerHandle(state_handle_b);

      registerInterface(&jnt_state_interface);

      // connect and register the joint position interface
      hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
      jnt_pos_interface.registerHandle(pos_handle_a);

      hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
      jnt_pos_interface.registerHandle(pos_handle_b);

      registerInterface(&jnt_pos_interface);
**/
}
    void read(const ros::Time& time, const ros::Duration& period) {
      printf("now reading");

      //// DYNAMIXEL READ STUFF GOES HERE ////

    // Update joint states
    for (size_t i = 0; i < joints.size(); ++i)
    {
      // In a real implementation, read the joint states from your hardware
     // Here, we will simply increment the joint positions for demonstration purposes
	//update the arrays with the read values
      //pos_[i] += 0.01;
      //vel_[i] = 0.0;
      //eff_[i] = 0.0;
      ROS_INFO_STREAM("Joint " << i + 1 << " command: " << joints[i].pos);
    }
    }

    void write(const ros::Time& time, const ros::Duration& period) {
      printf("now writing");

    // Execute joint commands
    for (size_t i = 0; i < joints.size(); ++i)
    {
      // In a real implementation, send the joint commands to your hardware
      // Here, we will simply display the commands for demonstration purposes
      ROS_INFO_STREAM("Joint " << i + 1 << " command: " << joints[i].cmd);

      //// DYNAMIXEL WRITE STUFF GOES HERE ////
    }
    }


private:
  std::vector<joint> joints;  

  hardware_interface::JointStateInterface jnt_state_interface_; //instantiate interface classes
  hardware_interface::PositionJointInterface jnt_pos_interface_;
};

main(int argc, char** argv)
{
  ros::init(argc, argv, "sence_hardware_node");
    ros::NodeHandle nh; // global namespace
    ros::NodeHandle rnh("~"); // robot private namespace
  SenceHardwareInterface hw; //instantiate our class
  hw.init(nh, rnh);
  controller_manager::ControllerManager cm(&hw); //instantiate the controller manager class with our robot class as an argument

  ros::Duration period(hw.get_update_duration()); // update rate
while (ros::ok()) { //main loop
    hw.read(ros::Time::now(), period); //sense
    cm.update(ros::Time::now(), period);
    hw.write(ros::Time::now(), period); //act
    period.sleep(); //wait
  }
}
