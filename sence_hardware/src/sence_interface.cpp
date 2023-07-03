#include "sence_hardware_interface.hpp"

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define NUMBER_OF_MOTORS 12

namespace sence_hardware
{

int dxl_comm_result = COMM_TX_FAIL;               // Communication result
bool dxl_addparam_result = false;                 // addParam result
bool dxl_getdata_result = false;                  // GetParam result

int dxl_goal_positions[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Goal positions array

uint8_t dxl_error = 0;                            // Dynamixel error
uint8_t param_goal_position[4];

int32_t dxl_present_positions[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Present positions array

bool SenceHardWareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh)
{
  // get hw mode
  robot_nh.getParam("simulated", simulated);
  if(simulated)
  {
    ROS_INFO("hardware simulated");
  }

  // get joint names
  std::vector<std::string> joint_names;
  robot_nh.getParam("joints", joint_names);
  ROS_INFO("found %i joints", (int)joint_names.size());

  // destroy any old resources
  joints.clear();

  // generate hw joint resource list
  for(auto name : joint_names)
  {
    HardwareJoint joint;
    joint.name = name;
    joints.push_back(joint);
    ROS_INFO("generated joint: %s", joint.name.c_str());
  }

  // init hw resources
  for(HardwareJoint& joint : joints)
  {
    joint.position_state = 0;
    joint.velocity_state = 0;
    joint.effort_state = 0;
    joint.velocity_command = 0;

    //joint.client = std::make_shared<HWRESTClient>();
    ros::NodeHandle jnh("~/" + joint.name);
    joint.client->init(jnh); // TODO implement simulation client

    hardware_interface::JointStateHandle jointStateHandle(joint.name, &joint.position_state, &joint.velocity_state, &joint.effort_state);
    joint_state_interface.registerHandle(jointStateHandle);

    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint.velocity_command);
    velocity_joint_interface.registerHandle(jointVelocityHandle);

    ROS_INFO("registered joint: %s", joint.name.c_str());
  }

  //Register interfaces
  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);

        //// init dynamixel stuff

            // Initialize PortHandler instance
            // Set the port path
            // Get methods and members of PortHandlerLinux or PortHandlerWindows
            dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

            // Initialize PacketHandler instance
            // Set the protocol version
            // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
            dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

            // Initialize GroupSyncWrite instance
            dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

            // Initialize Groupsyncread instance for Present Position
            dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

            // Open port
            if (portHandler->openPort())
            {
                printf("Succeeded to open the port!\n");
            }
            else
            {
                printf("Failed to open the port!\n");
                printf("Press any key to terminate...\n");
                getch();
                return 0;
            }

            // Set port baudrate
            if (portHandler->setBaudRate(BAUDRATE))
            {
                printf("Succeeded to set the baudrate!\n");
            }
            else
            {
                printf("Failed to set the baudrate!\n");
                printf("Press any key to terminate...\n");
                getch();
                return 0;
            }

            // Enable All Dynamixels Torque
            for (int id = 1; id <= NUMBER_OF_MOTORS; id++) {

                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                }
                else if (dxl_error != 0)
                {
                    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                }
                else
                {
                    printf("Dynamixel#%d has been successfully connected \n", id);
                }
            }

            // Add parameter storage for all Dynamixels present position value
            for (int id = 1; id <= NUMBER_OF_MOTORS; id++) {

                dxl_addparam_result = groupSyncRead.addParam(id);
                if (dxl_addparam_result != true)
                {
                    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", id);
                    return 0;
                }
            }

        //// end dynamixel init stuff
}

void SenceHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  for(HardwareJoint& joint : joints) // get feedback from hw
  {
    if (simulated) {
      joint.velocity_state = joint.velocity_command; // loopback on simulation
      joint.position_state += joint.velocity_state * period.toSec(); // fix integration
    }
    else {
        // position_state[i] = client->get_position();
        joint.velocity_state = joint.client->get_velocity(joint.velocity_state);
    }
  }
        //// dynamixel sdk sync read stuff
        
            do {
            // Syncread present positions
            dxl_comm_result = groupSyncRead.txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            } else {
                for (int id = 1; id <= NUMBER_OF_MOTORS; id++) {
                    if (groupSyncRead.getError(id, &dxl_error)) {
                        printf("[ID:%03d] %s\n", id, packetHandler->getRxPacketError(dxl_error));
                    }
                }
            }

            // Check if groupsynread data is available for all dynamixels
            for (int id = 1; id <= NUMBER_OF_MOTORS; id++) {
                
            }


            // Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            if (dxl_getdata_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
                return 0;
            }

            // Check if groupsyncread data of Dynamixel#2 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            if (dxl_getdata_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
                return 0;
            }

            // Get all dynamixels present position values
            for (int index = 0; index < NUMBER_OF_MOTORS; id++) {
                dxl_present_positions[index] = groupSyncRead.getData(index + 1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

            for (int id = 1; id <= NUMBER_OF_MOTORS; id++) {
                printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t", id, dxl_goal_positions[id - 1], dxl_present_positions[id - 1]);
                printf("\n");
            }

            }while(needs_to_move(dxl_goal_positions, dxl_present_positions));
        
        //// end of dynamixel read stuff
}

bool needs_to_move (int[] goal_positions, int[] present_positions) {
    for (int index = 0; index < NUMBER_OF_MOTORS; id++) {
        if (abs(goal_positions[index] - present_positions[index]) > DXL_MOVING_STATUS_THRESHOLD) {
            return true;
        }
    }
    return false;
}

void TrackInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if(simulated) return; // do nothing if in simulation

  // probs apply command limits here

  for(HardwareJoint& joint : joints) // publish commands to hw
  {
    joint.client->set_velocity(joint.velocity_command);
  }

        //// dynamixel sdk sync write stuff

            // Allocate goal position value into byte array
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_positions[index]));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_positions[index]));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_positions[index]));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_positions[index]));

            // Add goal position value to the Syncwrite storage
            for (int id = 1; id <= NUMBER_OF_MOTORS; id++) {

                dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_position);
                if (dxl_addparam_result != true)
                {
                fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", id);
                return 0;
                }
            }

            // Syncwrite goal position
            dxl_comm_result = groupSyncWrite.txPacket();
            if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

            // Clear syncwrite parameter storage
            groupSyncWrite.clearParam();

        //// end dynamixel write stuff

}

} // namespace sence_hardware
