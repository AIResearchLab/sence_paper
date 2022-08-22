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
#include "serial/serial.h"
#include "Configuration.h"

using namespace std;
using std::vector;

#define M_PI    3.14159265358979323846  /* pi */

#define SPEED_RPM_TICK 0.229
#define POSITION_DYNA 0.088

#define FeedbackPublishRate 25 //20 Hz return structure
#define WriteSerialRate 50     //20Hz

vector<uint8_t> commandVector;

sence_msgs::SENCE control_system;

std::string detectPort()
{

    //todo update code to find open CM board, need to verify pid/vid values
    std::string hardware_serial_processor = "10c4:ea60"; // If the description of the device contains this then it isn't the coms cable.
    std::string port;
    std::string ids;
    std::cout << "Finding required device..." << ids<< std::endl;
    // Goes through through all the ports and checks for a device with a vid_pid of a Teensy or Arduino Uno
    std::vector<serial::PortInfo> devices_found = serial::list_ports();
    std::cout << devices_found.size() << ids<< std::endl;
    //std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    for (serial::PortInfo &element : devices_found)
    {
            std::cout << "Evaluating a device:" << std::endl;

        if (element.hardware_id != "n/a")
        {
            ids = element.hardware_id;
            std::cout << "Hardware Info: " << ids<< std::endl;
            
            if (ids.find(hardware_serial_processor) != std::string::npos)
            {
                port = element.port;
                std::cout << "Accepted port as custom serial interface object:" << ids;
                return port;
            }

            std::string ids = element.hardware_id;
            std::cout << "Detail port:" << ids;
        }
    }
    throw "No port found matching description!";
}

vector<uint8_t> writeSerial(serial::Serial &opencm_serial, vector<uint8_t> commandVector)
{
    if(commandVector.empty()){
        return commandVector;
    }
    //according to the hardware interface, we should send the vector size as the payload
    //uint8_t payloadSize = commandVector.size() + 2;
    uint8_t payloadSize = commandVector.size();

    //inserts the payload at the first element
    commandVector.insert(commandVector.begin(), payloadSize);
    //then inserts the value 0 to the first element resulting in a header value of zero
    commandVector.insert(commandVector.begin(), 0xFF);

    //push the footer to the back
    commandVector.push_back(0);
    commandVector.push_back(0);

    uint8_t commandArray[commandVector.size()];
    std::copy(commandVector.begin(), commandVector.end(), commandArray);
    opencm_serial.write(commandArray, commandVector.size());
    commandVector.clear();
    return commandVector;
}

float convertDynamixelPoseToFloatPose(uint16_t value){
    return (float)value * POSITION_DYNA * M_PI / 180;
}

uint16_t convertFloatPoseToDynamixelPose(float value){
    float convertedValue = round(value * 180 / M_PI / POSITION_DYNA);
    if(convertedValue<0){
        throw "NULL ERROR OCCURED";
    }
    return (uint16_t)convertedValue;
}

uint16_t convertFloatTargetSpeedToDynamixelSpeed(float radiansPerSecond){
    float RPM = abs(9.549297 * radiansPerSecond);
    return (uint16_t)round(RPM / SPEED_RPM_TICK);
}

uint16_t convertDynamixelSpeedToFloatFeedbackSpeed(uint16_t feedbackValue){
    int16_t convertedSpeedFromUnsignedInteger = (int16_t)feedbackValue;
    float RPM = SPEED_RPM_TICK * (float)convertedSpeedFromUnsignedInteger;
    return (float)(RPM / 9.549297);
}

uint64_t getClockTime()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void addItem(uint8_t motor, uint8_t cmd, uint16_t data)
{
    commandVector.push_back(motor);
    commandVector.push_back(cmd);
    commandVector.push_back(UPPER_BYTE(data));
    commandVector.push_back(LOWER_BYTE(data));
}

bool switch_op = false;
void construct_commands(){
    commandVector.clear()
    if (switch_op==true){
    switch_op = false;
    uint16_t conversion_positionD_1 = convertFloatPoseToDynamixelPose(control_system.Back_Left.J0.TARGET_POSITION);
    uint16_t conversion_velocityD_1 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Left.J0.TARGET_VELOCITY);
    uint16_t conversion_positionD_2 = convertFloatPoseToDynamixelPose(control_system.Back_Left.J1.TARGET_POSITION);
    uint16_t conversion_velocityD_2 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Left.J1.TARGET_VELOCITY);
    uint16_t conversion_positionD_3 = convertFloatPoseToDynamixelPose(control_system.Back_Left.J2.TARGET_POSITION);
    uint16_t conversion_velocityD_3 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Left.J2.TARGET_VELOCITY);

    uint16_t conversion_positionD_4 = convertFloatPoseToDynamixelPose(control_system.Back_Right.J0.TARGET_POSITION);
    uint16_t conversion_velocityD_4 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Right.J0.TARGET_VELOCITY);
    uint16_t conversion_positionD_5 = convertFloatPoseToDynamixelPose(control_system.Back_Right.J1.TARGET_POSITION);
    uint16_t conversion_velocityD_5 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Right.J1.TARGET_VELOCITY);
    uint16_t conversion_positionD_6 = convertFloatPoseToDynamixelPose(control_system.Back_Right.J2.TARGET_POSITION);
    uint16_t conversion_velocityD_6 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Back_Right.J2.TARGET_VELOCITY);
    addItem(D_1, TARGET_POSITION, conversion_positionD_1);
    addItem(D_1, TARGET_VELOCITY, conversion_velocityD_1);
    addItem(D_2, TARGET_POSITION, conversion_positionD_2);
    addItem(D_2, TARGET_VELOCITY, conversion_velocityD_2);
    addItem(D_3, TARGET_POSITION, conversion_positionD_3);
    addItem(D_3, TARGET_VELOCITY, conversion_velocityD_3);
    addItem(D_4, TARGET_POSITION, conversion_positionD_4);
    addItem(D_4, TARGET_VELOCITY, conversion_velocityD_4);
    addItem(D_5, TARGET_POSITION, conversion_positionD_5);
    addItem(D_5, TARGET_VELOCITY, conversion_velocityD_5);
    addItem(D_6, TARGET_POSITION, conversion_positionD_6);
    addItem(D_6, TARGET_VELOCITY, conversion_velocityD_6);
    }else{
    switch_op = true;
    uint16_t conversion_positionD_7 = convertFloatPoseToDynamixelPose(control_system.Front_Right.J0.TARGET_POSITION);
    uint16_t conversion_velocityD_7 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Right.J0.TARGET_VELOCITY);
    uint16_t conversion_positionD_8 = convertFloatPoseToDynamixelPose(control_system.Front_Right.J1.TARGET_POSITION);
    uint16_t conversion_velocityD_8 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Right.J1.TARGET_VELOCITY);
    uint16_t conversion_positionD_9 = convertFloatPoseToDynamixelPose(control_system.Front_Right.J2.TARGET_POSITION);
    uint16_t conversion_velocityD_9 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Right.J2.TARGET_VELOCITY);

    uint16_t conversion_positionD_10 = convertFloatPoseToDynamixelPose(control_system.Front_Left.J0.TARGET_POSITION);
    uint16_t conversion_velocityD_10 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Left.J0.TARGET_VELOCITY);
    uint16_t conversion_positionD_11 = convertFloatPoseToDynamixelPose(control_system.Front_Left.J1.TARGET_POSITION);
    uint16_t conversion_velocityD_11 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Left.J1.TARGET_VELOCITY);
    uint16_t conversion_positionD_12 = convertFloatPoseToDynamixelPose(control_system.Front_Left.J2.TARGET_POSITION);
    uint16_t conversion_velocityD_12 = convertFloatTargetSpeedToDynamixelSpeed(control_system.Front_Left.J2.TARGET_VELOCITY);
    addItem(D_7, TARGET_POSITION, conversion_positionD_7);
    addItem(D_7, TARGET_VELOCITY, conversion_velocityD_7);
    addItem(D_8, TARGET_POSITION, conversion_positionD_8);
    addItem(D_8, TARGET_VELOCITY, conversion_velocityD_8);
    addItem(D_9, TARGET_POSITION, conversion_positionD_9);
    addItem(D_9, TARGET_VELOCITY, conversion_velocityD_9);
    addItem(D_10, TARGET_POSITION, conversion_positionD_10);
    addItem(D_10, TARGET_VELOCITY, conversion_velocityD_10);
    addItem(D_11, TARGET_POSITION, conversion_positionD_11);
    addItem(D_11, TARGET_VELOCITY, conversion_velocityD_11);
    addItem(D_12, TARGET_POSITION, conversion_positionD_12);
    addItem(D_12, TARGET_VELOCITY, conversion_velocityD_12);
    }
}

void updateTargetsCallback(const sence_msgs::Target_Buffer::ConstPtr &msg)
{
    for (int i = 0; i < msg->count; i++) {
        sence_msgs::Target target = msg->Buffer_To_Send[i];
        std::cout << "Message recieved for target : " << target.TARGET_ID << " with updates" << std::endl;
        switch (target.TARGET_ID)
        {
        case D_1:
            control_system.Back_Left.J0.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Left.J0.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_2:
            control_system.Back_Left.J1.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Left.J1.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_3:
            control_system.Back_Left.J2.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Left.J2.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_4:
            control_system.Back_Right.J0.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Right.J0.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_5:
            control_system.Back_Right.J1.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Right.J1.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_6:
            control_system.Back_Right.J2.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Back_Right.J2.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_7:
            control_system.Front_Right.J0.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Right.J0.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_8:
            control_system.Front_Right.J1.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Right.J1.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_9:
            control_system.Front_Right.J2.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Right.J2.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_10:
            control_system.Front_Left.J0.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Left.J0.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_11:
            control_system.Front_Left.J1.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Left.J1.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        case D_12:
            control_system.Front_Left.J2.TARGET_POSITION = target.TARGET_POSITION;
            control_system.Front_Left.J2.TARGET_VELOCITY = target.TARGET_VELOCITY;
            break;
        default:
            // code block
            std::cout << "Not Handled Yet";
        }
    }
}
void assignTemperatureDynamixelFeedback(int jID, uint16_t temperature)
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

void assignVelocityDynamixelFeedback(int jID, uint16_t velocity)
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

void assignPositionDynamixelFeedback(int jID, uint16_t position)
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

void processIncomingMessage(int cmd_id, int motor, uint16_t data)
{
    //std::cout << "id: " << cmd_id << " - motor: " << motor << " - value: "<< data << std::endl;
    switch (cmd_id)
    {
    case PRESENT_POSITION:
        assignPositionDynamixelFeedback(motor, data);
        break;
    case PRESENT_VELOCITY:
        assignVelocityDynamixelFeedback(motor, data);
        break;
    case PRESENT_TEMP:
        assignTemperatureDynamixelFeedback(motor, data);
        break;
    default:
        // code block
        std::cout << "Not Handled Yet " << cmd_id << "\n";
    }
    return;
}

void readSerial(serial::Serial &opencm_serial, int recursiveCounter)
{

    uint8_t check_buffer[2];
    if (recursiveCounter > 5)
    {
        return;
    }
    // only reads if 3 bytes are available which should be header value + payload
    if (opencm_serial.available() > 0)
    {
        //std::cout << "Reading Available Input Data at "<< getClockTime() << std::endl;
        opencm_serial.read(check_buffer, 2);
        // flushes if header value isn't correct
        if (check_buffer[0] != 255)
        {
            opencm_serial.flushInput();
            ROS_ERROR_STREAM("Read front: check:" << check_buffer);
            return;
        }
        else
        {
            uint8_t payload = check_buffer[1];
            //add two for footer read
            uint8_t message_buffer[payload + 2];
            opencm_serial.read(message_buffer, payload + 2);
            //TODO: add comment about for loop structure
            for (int i = 0; i < payload / 4; i++)
            {
                uint8_t motorId = message_buffer[i * 4];
                uint8_t int_id = message_buffer[i * 4 + 1];
                uint16_t full_byte = INT_JOIN_BYTE(message_buffer[i * 4 + 2], message_buffer[i * 4 + 3]);
                processIncomingMessage(unsigned(int_id), unsigned(motorId), full_byte);
            }

            if (message_buffer[sizeof(message_buffer) - 1] != 0 && message_buffer[sizeof(message_buffer) - 2] != 0)
            {
                ROS_ERROR_STREAM("Read back check" << message_buffer[payload - 1]);
                opencm_serial.flushInput();
            }

            readSerial(opencm_serial, recursiveCounter++);
        }
    }
}


int main(int argc, char **argv)
{
    //Start off by initialising the leg connection and variables to 0
    control_system.Back_Left.J0.TARGET_POSITION=0;
    control_system.Back_Left.J0.TARGET_VELOCITY=0;
    control_system.Back_Left.J0.PRESENT_VELOCITY=0;
    control_system.Back_Left.J0.PRESENT_POSITION=0;
    control_system.Back_Left.J0.PRESENT_TEMPERATURE=0;
    control_system.Back_Left.J1 = control_system.Back_Left.J0;
    control_system.Back_Left.J2 = control_system.Back_Left.J0;
    control_system.Back_Right = control_system.Back_Left;
    control_system.Front_Left = control_system.Back_Left;
    control_system.Front_Right = control_system.Back_Left;

    std::string port = detectPort();
    std::cout << "Port for openCM is" << port << std::endl;
    ros::init(argc, argv, "SENCE_Serial_Interface");
    ros::NodeHandle node;
    ros::Rate rate(1000);

    ros::Publisher system_publisher = node.advertise<sence_msgs::SENCE>("sence_state", 3);

    ros::Subscriber target_updater = node.subscribe("sence_target", 2, updateTargetsCallback);

    uint64_t write_serial_time = getClockTime();
    uint64_t publish_feedback_time = getClockTime();
    uint64_t time_now = getClockTime();

    //In this section we open up the port on the appropraite baud rate to listen to the autodetected teensy and establish a connection
    serial::Serial opencm_serial(port, BAUD_RATE, serial::Timeout::simpleTimeout(1000));

    if (opencm_serial.isOpen())
    {
        opencm_serial.flush();
        opencm_serial.flushInput();
        opencm_serial.flushOutput();
        std::cout << "Port is open and active" << std::endl;
    }
    else
    {
        std::cout << "Port was not opened correctly" << std::endl;
        return 0;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));

    while (1)
    {
        readSerial(opencm_serial, 5);
        if (control_system.Back_Left.J0.PRESENT_POSITION != 0 ||
            control_system.Back_Right.J0.PRESENT_POSITION != 0 ||
            control_system.Front_Left.J0.PRESENT_POSITION != 0 ||
            control_system.Front_Right.J0.PRESENT_POSITION != 0)
        {
            std::cout << "syncingSystems" << std::endl;
            control_system.Back_Left.J0.TARGET_POSITION=control_system.Back_Left.J0.PRESENT_POSITION;
            control_system.Back_Left.J0.TARGET_VELOCITY=0;
            control_system.Back_Left.J1.TARGET_POSITION=control_system.Back_Left.J1.PRESENT_POSITION;
            control_system.Back_Left.J1.TARGET_VELOCITY=0;
            control_system.Back_Left.J2.TARGET_POSITION=control_system.Back_Left.J1.PRESENT_POSITION;
            control_system.Back_Left.J2.TARGET_VELOCITY=0;

            control_system.Front_Left.J0.TARGET_POSITION=control_system.Front_Left.J0.PRESENT_POSITION;
            control_system.Front_Left.J0.TARGET_VELOCITY=0;
            control_system.Front_Left.J1.TARGET_POSITION=control_system.Front_Left.J1.PRESENT_POSITION;
            control_system.Front_Left.J1.TARGET_VELOCITY=0;
            control_system.Front_Left.J2.TARGET_POSITION=control_system.Front_Left.J1.PRESENT_POSITION;
            control_system.Front_Left.J2.TARGET_VELOCITY=0;

            control_system.Front_Right.J0.TARGET_POSITION=control_system.Front_Right.J0.PRESENT_POSITION;
            control_system.Front_Right.J0.TARGET_VELOCITY=0;
            control_system.Front_Right.J1.TARGET_POSITION=control_system.Front_Right.J1.PRESENT_POSITION;
            control_system.Front_Right.J1.TARGET_VELOCITY=0;
            control_system.Front_Right.J2.TARGET_POSITION=control_system.Front_Right.J1.PRESENT_POSITION;
            control_system.Front_Right.J2.TARGET_VELOCITY=0;

            control_system.Back_Right.J0.TARGET_POSITION=control_system.Back_Right.J0.PRESENT_POSITION;
            control_system.Back_Right.J0.TARGET_VELOCITY=0;
            control_system.Back_Right.J1.TARGET_POSITION=control_system.Back_Right.J1.PRESENT_POSITION;
            control_system.Back_Right.J1.TARGET_VELOCITY=0;
            control_system.Back_Right.J2.TARGET_POSITION=control_system.Back_Right.J1.PRESENT_POSITION;
            control_system.Back_Right.J2.TARGET_VELOCITY=0;

            break;
        }
    }

    std::cout << "Starting Main Communication" << std::endl;

    int publishFrameID = 0;

    while (ros::ok())
    {
        time_now = getClockTime();

        readSerial(opencm_serial, 2);
        commandVector = writeSerial(opencm_serial, commandVector);

        //this if statement is to publish at 30 HZ the feedback of motor positions
        //uint32 seq
        //time stamp
        //string frame_id
        if ((time_now - publish_feedback_time) >= FeedbackPublishRate)
        {
            ++publishFrameID;
            publish_feedback_time = getClockTime();
            control_system.Header.frame_id = "frame" + std::to_string(publishFrameID);
            control_system.Header.seq = publish_feedback_time;
            control_system.Header.stamp = ros::Time::now();
            system_publisher.publish(control_system);
        }

        if ((time_now - write_serial_time) >= WriteSerialRate)
        {
            construct_commands();
            write_serial_time = getClockTime();
        }

        ros::spinOnce();
        cout << "Looping, will pause if serial error" << time_now << std::endl;
        rate.sleep();
    }
    opencm_serial.close();

    return 0;
}