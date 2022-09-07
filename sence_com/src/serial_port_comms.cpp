#include <ostream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <sstream>
#include <queue>
#include <cmath>

#include <chrono>
#include <thread>
#include <sence_msgs/SENCE.h>
#include <sence_msgs/Target_Buffer.h>
#include <sence_msgs/Target.h>

#include "ros/publisher.h"
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include "ros/ros.h"
#include "Configuration.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

using namespace std;
using std::vector;

//feedback structure 44 elements for parsing
interface_struct feedback_updates[44];
//command structure 24 commands elements for parsing
interface_struct command_updates[44];


#define RateOfUpdate 10 //20Hz Hz return structure
#define RateOfPublish 50 //20Hz Hz return structure

#define M_PI    3.14159265358979323846  /* pi */

#define SPEED_RPM_TICK 0.229
#define POSITION_DYNA 0.088

#define DYNA_BAUD 3000000

DynamixelWorkbench dxl_wb;


sence_msgs::SENCE control_system;

float convertDynamixelPoseToFloatPose(uint16_t value){
    return (float)value * POSITION_DYNA * M_PI / 180;
}

int16_t convertFloatPoseToDynamixelPose(float value){
    float convertedValue = round(value * 180 / M_PI / POSITION_DYNA);
    return (int16_t)convertedValue;
}

uint16_t convertFloatTargetSpeedToDynamixelSpeed(float radiansPerSecond){
    float RPM = abs(9.549297 * radiansPerSecond);
    return (uint16_t)((int)(round(RPM / SPEED_RPM_TICK)));
}

float convertDynamixelSpeedToFloatFeedbackSpeed(uint16_t feedbackValue){
    int16_t convertedSpeedFromUnsignedInteger = (int16_t)feedbackValue;
    float RPM = SPEED_RPM_TICK * (float)convertedSpeedFromUnsignedInteger;
    return (float)RPM / 9.549297;
}

uint64_t getClockTime()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}


int32_t resend_targets(){
    int32_t pos;
    int32_t vel;
        switch (i)
        {
        case D_1:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Left.J0.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Back_Left.J0.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_2:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Left.J1.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Back_Left.J1.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_3:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Left.J2.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Back_Left.J2.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_4:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Right.J0.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Back_Right.J0.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_5:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Right.J1.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Back_Right.J1.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_6:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Right.J2.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Back_Right.J2.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_7:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Right.J0.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Front_Right.J0.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_8:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Right.J1.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Front_Right.J1.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_9:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Right.J2.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Front_Right.J2.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_10:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Left.J0.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Front_Left.J0.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_11:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Left.J1.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Front_Left.J1.TARGET_POSITION);
            dxl_wb.itemWrite(i, "Profile_Velocity", vel, &log);
            dxl_wb.itemWrite(i, "Goal_Position", pos, &log);
            break;
        case D_12:
            pos = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Left.J2.TARGET_VELOCITY);
            vel = convertFloatPoseToDynamixelPose(control_system.Front_Left.J2.TARGET_POSITION);
            break;
        default:
            // code block
            std::cout << "Not Handled Yet";
        }
    }
}

void updateTargetsCallback(const sence_msgs::Target_Buffer::ConstPtr &msg)
{
    const char *log;
    for (int i = 0; i < msg->count; i++) {
        sence_msgs::Target target = msg->Buffer_To_Send[i];
        std::cout << "Message recieved for target : " << target.TARGET_ID << " with updates" << std::endl;
        int32_t pos;
        int32_t vel;
        switch (target.TARGET_ID)
        {
        case D_1:
            control_system.Back_Left.J0.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Left.J0.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_2:
            control_system.Back_Left.J1.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Left.J1.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_3:
            control_system.Back_Left.J2.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Left.J2.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_4:
            control_system.Back_Right.J0.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Right.J0.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_5:
            control_system.Back_Right.J1.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Right.J1.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_6:
            control_system.Back_Right.J2.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Right.J2.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_7:
            control_system.Front_Right.J0.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Right.J0.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_8:
            control_system.Front_Right.J1.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Right.J1.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_9:
            control_system.Front_Right.J2.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Right.J2.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_10:
            control_system.Front_Left.J0.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Left.J0.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_11:
            control_system.Front_Left.J1.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Left.J1.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        case D_12:
            control_system.Front_Left.J2.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Left.J2.TARGET_VELOCITY = target.TARGET_VELOCITY;
            // pos = convertFloatTargetSpeedToDynamixelSpeed(target.TARGET_VELOCITY);
            // vel = convertFloatPoseToDynamixelPose(target.TARGET_POSITION);
            // dxl_wb.itemWrite(target.TARGET_ID, "Profile_Velocity", vel, &log);
            // dxl_wb.itemWrite(target.TARGET_ID, "Goal_Position", pos, &log);
            break;
        default:
            // code block
            std::cout << "Not Handled Yet";
        }
    }
}
void assignTemperatureDynamixelFeedback(int jID, int32_t temperature)
{
    switch (jID)
    {
    case D_1:
        control_system.Back_Left.J0.PRESENT_TEMPERATURE = temperature;
        break;
    case D_2:
        control_system.Back_Left.J1.PRESENT_TEMPERATURE = temperature;
        break;
    case D_3:
        control_system.Back_Left.J2.PRESENT_TEMPERATURE = temperature;
        break;
    case D_4:
        control_system.Back_Right.J0.PRESENT_TEMPERATURE = temperature;
        break;
    case D_5:
        control_system.Back_Right.J1.PRESENT_TEMPERATURE = temperature;
        break;
    case D_6:
        control_system.Back_Right.J2.PRESENT_TEMPERATURE = temperature;
        break;
    case D_7:
        control_system.Front_Right.J0.PRESENT_TEMPERATURE = temperature;
        break;
    case D_8:
        control_system.Front_Right.J1.PRESENT_TEMPERATURE = temperature;
        break;
    case D_9:
        control_system.Front_Right.J2.PRESENT_TEMPERATURE = temperature;
        break;
    case D_10:
        control_system.Front_Left.J0.PRESENT_TEMPERATURE = temperature;
        break;
    case D_11:
        control_system.Front_Left.J1.PRESENT_TEMPERATURE = temperature;
        break;
    case D_12:
        control_system.Front_Left.J2.PRESENT_TEMPERATURE = temperature;
        break;
    default:
        // code block
        std::cout << "Not Handled Yet";
    }
}

void assignVelocityDynamixelFeedback(int jID, int32_t velocity)
{
    switch (jID)
    {
    case D_1:
        control_system.Back_Left.J0.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_2:
        control_system.Back_Left.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_3:
        control_system.Back_Left.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_4:
        control_system.Back_Right.J0.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_5:
        control_system.Back_Right.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_6:
        control_system.Back_Right.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_7:
        control_system.Front_Right.J0.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_8:
        control_system.Front_Right.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_9:
        control_system.Front_Right.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_10:
        control_system.Front_Left.J0.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_11:
        control_system.Front_Left.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    case D_12:
        control_system.Front_Left.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatFeedbackSpeed(velocity);
        break;
    default:
        // code block
        std::cout << "Not Handled Yet";
    }
}

void assignLoadDynamixelFeedback(int jID, int32_t load)
{
    switch (jID)
    {
    case D_1:
        control_system.Back_Left.J0.PRESENT_LOAD = load;
        break;
    case D_2:
        control_system.Back_Left.J1.PRESENT_LOAD = load;
        break;
    case D_3:
        control_system.Back_Left.J2.PRESENT_LOAD = load;
        break;
    case D_4:
        control_system.Back_Right.J0.PRESENT_LOAD =load;
        break;
    case D_5:
        control_system.Back_Right.J1.PRESENT_LOAD =load;
        break;
    case D_6:
        control_system.Back_Right.J2.PRESENT_LOAD =load;
        break;
    case D_7:
        control_system.Front_Right.J0.PRESENT_LOAD =load;
        break;
    case D_8:
        control_system.Front_Right.J1.PRESENT_LOAD =load;
        break;
    case D_9:
        control_system.Front_Right.J2.PRESENT_LOAD =load;
        break;
    case D_10:
        control_system.Front_Left.J0.PRESENT_LOAD =load;
        break;
    case D_11:
        control_system.Front_Left.J1.PRESENT_LOAD =load;
        break;
    case D_12:
        control_system.Front_Left.J2.PRESENT_LOAD =load;
        break;
    default:
        // code block
        std::cout << "Not Handled Yet";
    }
}

void assignPositionDynamixelFeedback(int jID, int32_t position)
{
    switch (jID)
    {
    case D_1:
        control_system.Back_Left.J0.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_2:
        control_system.Back_Left.J1.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_3:
        control_system.Back_Left.J2.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_4:
        control_system.Back_Right.J0.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_5:
        control_system.Back_Right.J1.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_6:
        control_system.Back_Right.J2.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_7:
        control_system.Front_Right.J0.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_8:
        control_system.Front_Right.J1.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_9:
        control_system.Front_Right.J2.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_10:
        control_system.Front_Left.J0.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_11:
        control_system.Front_Left.J1.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    case D_12:
        control_system.Front_Left.J2.PRESENT_POSITION = convertDynamixelPoseToFloatPose(position);
        break;
    default:
        // code block
        std::cout << "Not Handled Yet";
    }
}

int feedback_index=0;
void mass_read_data(){
    int32_t get_data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    interface_struct to_assign[9];

    const char *log;
    uint16_t model_number = 0;
    bool result = false;
    result = dxl_wb.initBulkRead(&log);
    if (result == false)
    {
        printf("Log: %s\n", log);
    }
    else
    {
        printf("Log: %s\n", log);
    }
    int updates=0;
    while(updates<9){
        result = dxl_wb.addBulkReadParam(feedback_updates[feedback_index].actuator_id, feedback_updates[feedback_index].feedback_string, &log);
        to_assign[updates] = interface_struct{feedback_updates[feedback_index].actuator_id, feedback_updates[feedback_index].feedback_string};
        feedback_updates[feedback_index];
        if (result == false)
        {
            printf("Log: %s\n", log);
            printf("Failed to add bulk read position param\n");
            printf("updates = %d\n",updates);
            printf("feedback_index = %d\n",feedback_index);
            cout << feedback_updates[feedback_index].actuator_id << feedback_updates[feedback_index].feedback_string << std::endl;
        }
        else
        {
            printf("Log: %s\n", log);
        } 
        updates++;
        if(feedback_index==43){
            feedback_index=0;
        }else{
            feedback_index++;
        }
        if(updates==9){
            break;
        }        
    } 
    
      result = dxl_wb.bulkRead(&log);
      if (result == false)
      {
        printf("Log: %s\n", log);
        printf("Failed to bulk read\n");
      }

      result = dxl_wb.getBulkReadData(&get_data[0], &log);
      std::cout << result << std::endl;
      if (result == false)
      {
        printf("Log: %s\n", log);
      }
//       else
//       {
//         std::cout << (int)get_data[0] << ", " << (int)get_data[1]  << std::endl;
//   }
    for(int i = 0;i<9;i++){
        if (strcmp(to_assign[i].feedback_string, "Present_Position") == 0){
            assignPositionDynamixelFeedback(to_assign[i].actuator_id,get_data[i]);
        }else if (strcmp(to_assign[i].feedback_string, "Present_Velocity") == 0){
            assignVelocityDynamixelFeedback(to_assign[i].actuator_id,get_data[i]);
        }
        else if (strcmp(to_assign[i].feedback_string, "Present_Temperature") == 0){
            assignTemperatureDynamixelFeedback(to_assign[i].actuator_id,get_data[i]);
        }
        else if (strcmp(to_assign[i].feedback_string, "Present_Load") == 0){
            assignLoadDynamixelFeedback(to_assign[i].actuator_id,get_data[i]);
        }
    }

    dxl_wb.clearBulkReadParam();    
    }

command_index=0;
void command_actutors(){

    result = dxl_wb.initBulkWrite(&log);
    if (result == false)
    {
        printf("%s\n", log);
    }
    else
    {
        printf("%s\n", log);
    }
    int updates=0;
    int value_to_parse = 0
    while(updates<9){

        result = dxl_wb.addBulkReadParam(command_updates[command_index].actuator_id, command_updates[command_index].feedback_string, &log);
        to_assign[updates] = interface_struct{feedback_updates[feedback_index].actuator_id, feedback_updates[feedback_index].feedback_string};
        feedback_updates[feedback_index];
        if (result == false)
        {
            printf("Log: %s\n", log);
            printf("Failed to add bulk read position param\n");
            printf("updates = %d\n",updates);
            printf("feedback_index = %d\n",feedback_index);
            cout << feedback_updates[feedback_index].actuator_id << feedback_updates[feedback_index].feedback_string << std::endl;
        }
        else
        {
            printf("Log: %s\n", log);
        } 
        updates++;
        if(feedback_index==43){
            feedback_index=0;
        }else{
            feedback_index++;
        }
        if(updates==9){
            break;
        }        
    } 

    result = dxl_wb.addBulkWriteParam(dxl_id[0], "Goal_Position", goal_position[0], &log);
    if (result == false)
    {
      printf("%s\n", log);
      printf("Failed to add bulk write position param\n");
    }
    else
    {
      printf("%s\n", log);
    }

    result = dxl_wb.addBulkWriteParam(dxl_id[1], "LED", led[0], &log);
    if (result == false)
    {
      printf("%s\n", log);
      printf("Failed to add bulk write led param\n");
    }
    else
    {
      printf("%s\n", log);
    }

    result = dxl_wb.bulkWrite(&log);
    if (result == false)
    {
      printf("%s\n", log);
      printf("Failed to bulk write\n");
    }

}

int main(int argc, char **argv)
{
    const char *log;
    const char* port_name = "/dev/ttyUSB0";
    ros::init(argc, argv, "SENCE_Serial_Interface");
    ros::NodeHandle node;
    ros::Rate rate(1000);


    //initialise the feedback command system
    const char* feedback_strings[4]
        = { "Present_Position", "Present_Velocity", "Present_Temperature", "Present_Load" };
    int idx = 0;
    for(int fs=0;fs<=3;fs++){
        for(int dyI=1;dyI<=12;dyI++){
            feedback_updates[idx] = interface_struct{dyI,feedback_strings[fs]};
            idx++;   
        }
    }

    
    const char* command_strings[4]
        = { "Profile_Velocity", "Goal_Position"};
    idx = 0;
    for(int fs=0;fs<=1;fs++){
        for(int dyI=1;dyI<=12;dyI++){
            command_updates[idx] = interface_struct{dyI,command_strings[fs]};
            idx++;
        }
    }
    
    

    //wb_result = dxl_wb.init(port_name, baud, &log);
    bool wb_result = dxl_wb.init(port_name, DYNA_BAUD,&log);
    if(!wb_result){
        cout << "Failed to connect, quitting" << std::endl;
        return 0;
    } else {
        //update system parameters.
        cout << "So far so good" << std::endl;
    }

    //Next initialize the dynamixel data and check for actuators
    uint16_t model_number = 0;
    bool result;
    for (int i = 1; i <= 12; i++) {
        result = dxl_wb.ping(i, &model_number, &log);
        if (result == false)
        {
            printf("Log: %s\n", log);
            printf("Failed to ping\n");
            printf("It looks like the U2D2 cannot find all the required components, please validate that all dynamixels are detectable\n");
            return 0;
        }
        else
        {
            printf("Succeed to ping\n");
            printf("id : %d, model_number : %d\n", i, model_number);
            
            int32_t get_data = 0;

            result = dxl_wb.itemRead(i, "Present_Position", &get_data, &log);
            cout << "Position of " << i<<" at "<< get_data << " with result " << result << std::endl;
            cout << log << std::endl;
            result = dxl_wb.itemRead(i, "Present_Velocity", &get_data, &log);
            cout << "Velocity of " << i<<" at "<< get_data << " with result " << result << std::endl;
            cout << log << std::endl;
            
            result = dxl_wb.itemRead(i, "Present_Temperature", &get_data, &log);
            cout << "Temperature of " << i<<" at "<< get_data << " with result " << result << std::endl;
            cout << log << std::endl;
            
            result = dxl_wb.itemRead(i, "Present_Load", &get_data, &log);
            cout << "Load of " << i<<" at "<< get_data << " with result " << result << std::endl;
            cout << log << std::endl;

            if (i==1 || i==4 || i==7 || i==10){
            result = dxl_wb.itemWrite(i, "Position_P_Gain", 425, &log);
            cout << "read gain for " << i<<" as "<< get_data << " with result " << result << std::endl;
            cout << log << std::endl;
            }

            dxl_wb.goalPosition(i, (int32_t)0);

            result = dxl_wb.torqueOn(i ,&log);
            cout << "torque = " << result <<" for "<< i << std::endl;
            cout << log << std::endl;

            result = dxl_wb.itemRead(i, "Torque_Enable", &get_data, &log);
            cout << "torque check = " << get_data <<" for "<< i << std::endl;
            cout << log << std::endl;

            result = dxl_wb.itemWrite(i, "Profile_Velocity", 10, &log);
            cout << "wrote goal velocity " << i<<" to "<< 10 << " with result " << result << std::endl;
            cout << log << std::endl;

            result = dxl_wb.itemWrite(i, "Goal_Position", 35, &log);
            cout << "wrote goal position " << i<<" to "<< 35 << " with result " << result << std::endl;
            cout << log << std::endl;

        }
    }


    cout << "All devices found" << std::endl;

    ros::Publisher system_publisher = node.advertise<sence_msgs::SENCE>("sence_state", 3);

    ros::Subscriber target_updater = node.subscribe("sence_target", 2, updateTargetsCallback);

    uint64_t write_serial_time = getClockTime();
    uint64_t publish_feedback_time = getClockTime();
    uint64_t mass_read_time = getClockTime();
    uint64_t time_now = getClockTime();
 
   
    std::this_thread::sleep_for(std::chrono::seconds(3));

    int publishFrameID = 0;

    while (ros::ok())
    {
        time_now = getClockTime();

        //this if statement is to publish at 30 HZ the feedback of motor positions
        //uint32 seq
        //time stamp
        //string frame_id
        if ((time_now - mass_read_time) >= RateOfUpdate)
        {
            mass_read_time = getClockTime();
            mass_read_data();
        }

        if ((time_now - publish_feedback_time) >= RateOfPublish)
        {
            command_actutors();
            ++publishFrameID;
            publish_feedback_time = getClockTime();
            control_system.Header.frame_id = "frame" + std::to_string(publishFrameID);
            control_system.Header.seq = publish_feedback_time;
            control_system.Header.stamp = ros::Time::now();
            system_publisher.publish(control_system);

        }

            
        
        ros::spinOnce();
        //cout << "Looping, will pause if serial error" << time_now << std::endl;
        rate.sleep();
    }

    return 0;
}