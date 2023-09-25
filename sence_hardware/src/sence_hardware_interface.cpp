#include "sence_hardware/sence_hardware_interface.hpp"
#include "sence_hardware/joint.hpp"

namespace sence_hardware
{

  namespace
  {

    std::vector<Joint> init_joints(const ros::NodeHandle &robot_nh)
    {
      // get joint names from the parameter server
      std::vector<std::string> joint_names;
      robot_nh.getParam("joints", joint_names);
      printf(">>>>>> Found %i joints in config!\n", (int)joint_names.size());

      std::vector<Joint> joints;
      int id = 1;
      for (auto &joint_name : joint_names)
      {
        joints.push_back({id, joint_name, 0, 0, 0, 0});
        ++id;
      }
      return joints;
    }
    // some functions to convert between radians and dynamixel ticks

    double tick2Rad(uint32_t tick) { return tick * (2 * M_PI / 4096) - M_PI; }

    uint32_t rad2Tick(double rad) { return (rad + M_PI) * (4096 / (2 * M_PI)); }
  }

  SenceHardwareInterface::SenceHardwareInterface(ros::NodeHandle &robot_nh) :
    joints(init_joints(robot_nh)),
    portHandler(dynamixel::PortHandler::getPortHandler(DEVICENAME)),
    packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)),
    groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION),
    groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION),
    dxl_comm_result(COMM_TX_FAIL),
    dxl_error(0),
    dxl_addparam_result(false),
    dxl_getdata_result(false),
    param_goal_position{0, 0, 0, 0}
  {
  }

  SenceHardwareInterface::~SenceHardwareInterface()
  {
    printf("bye-bye...");
    // Disable all dynamixels Torque
    for (const auto &joint : joints)
    {
      const int &id = joint.id;
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_OFF, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        ROS_INFO_STREAM("Dynamixel Error: ID " + std::to_string(id) + " Not Found\n");
      }
      else if (dxl_error != 0)
      {
        ROS_INFO("Dynamixel Error: Bad Instruction\n");
      }
      else
      {
        // ROS_INFO("Succeeded to enable torque.");
        printf("Disabled torque to motor ID: %d\n", id);
      }
      portHandler->closePort();
    }
  }

  bool SenceHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh)
  {

    for (auto &joint : joints)
    {
      hardware_interface::JointStateHandle state_handle(joint.name, &joint.pos, &joint.vel, &joint.eff);
      joint_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(state_handle, &joint.cmd);
      joint_pos_interface.registerHandle(pos_handle);
    }

    registerInterface(&joint_state_interface);

    registerInterface(&joint_pos_interface);
    //      Dynamixel init
    ////////////////////////////
    // Open port
    if (portHandler->openPort())
    {
      ROS_INFO("Succeeded to open the port!\n");
    }
    else
    {
      ROS_INFO("Failed to open the port!\n");
      return false;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
      ROS_INFO("Succeeded to set the baudrate!\n");
    }
    else
    {
      ROS_INFO("Failed to set the baudrate!\n");
      return false;
    }

    // Enable all dynamixels Torque
    for (const auto &joint : joints)
    {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint.id, ADDR_TORQUE_ENABLE, TORQUE_ON, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        ROS_INFO("Dynamixel Error: ID Not Found\n");
        return false;
      }
      else if (dxl_error != 0)
      {
        ROS_INFO("Dynamixel Error: Bad Instruction\n");
        return false;
      }
      else
      {
        // ROS_INFO("Succeeded to enable torque.");
        printf("Enabled torque to motor ID: %d\n", joint.id);
      }
    }

    // Add parameter storage for all dynamixels present position value
    for (const auto &joint : joints)
    {
      dxl_addparam_result = groupSyncRead.addParam(joint.id);
      if (dxl_addparam_result != true)
      {
        ROS_INFO("GroupSyncRead addParam failed!");
        return false;
      }
      else
      {
        ROS_INFO("GroupSyncRead addParam OK.");
      }
    }

    return true;
  }

  void SenceHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
  {
    // ROS_INFO("now reading...");
    ros::Duration second(1);

    // Syncread present position
    do
    {
      dxl_comm_result = groupSyncRead.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
        printf("read error... waiting for response... (check physical connection)\n");
        second.sleep();
      }
    } while (dxl_comm_result != COMM_SUCCESS);

    uint32_t read_value;
    for (auto &joint : joints)
    {
      dxl_getdata_result = false;

      do
      {
        dxl_getdata_result = groupSyncRead.isAvailable(joint.id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

        if (dxl_getdata_result)
        {
          read_value = groupSyncRead.getData(joint.id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
          joint.pos = tick2Rad(read_value);
          printf("read [id:%03d] ticks: %03f pos:%03d\n", joint.id, joint.pos, read_value);
        }
        else
        {
          printf("waiting for response from dynamixel...\n");
          second.sleep();
        }
      } while (!dxl_getdata_result);
    }
    dxl_comm_result = COMM_SUCCESS;
  }

  void SenceHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
  {
    // ROS_INFO("now writing...");

    for (const auto &joint : joints)
    {
      // get goal position value from cmd converted from radians to ticks
      uint32_t write_value = rad2Tick(joint.cmd);
      // Allocate goal position value into byte array
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(write_value));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(write_value));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(write_value));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(write_value));

      // Add goal position value to the Syncwrite storage
      dxl_addparam_result = groupSyncWrite.addParam(joint.id, param_goal_position);
      if (dxl_addparam_result != true)
      {
        ROS_INFO("GroupSyncWrite addParam failed...");
      }
      else
      {
        // ROS_INFO("GroupSyncWrite addParam success!");
        printf("writing [id:%03d] cmd:%03f ticks:%03d\n", joint.id, joint.cmd, write_value);
      }
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
      ROS_INFO("GroupSyncWrite txpacket failed...");
    }
    else
    {
      // ROS_INFO("GroupSyncWrite success!");
    }
    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
  }
} // namespace sence_hardware
