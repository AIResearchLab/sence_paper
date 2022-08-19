#include <ostream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <sstream>
#include <queue>

#include <sence_msgs/SENCE.h>
#include <sence_msgs/Target_Buffer.h>

#include "ros/publisher.h"
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include "Configuration.h"
#include "serial_functions.h"

#define FeedbackPublishRate 25 //20 Hz return structure
#define WriteSerialRate 25     //20Hz

sence_msgs::SENCE control_system;

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

void assignVelocityDynamixelFeedback(int jID, uint16_t velocity)
{
    switch (jID)
    {
    case D_1:
        control_system.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_2:
        control_system.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_3:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_4:
        control_system.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_5:
        control_system.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_6:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_7:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_8:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_9:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_10:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_11:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_12:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
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
        control_system.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_2:
        control_system.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_3:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_4:
        control_system.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_5:
        control_system.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_6:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_7:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_8:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_9:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_10:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_11:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_12:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    default:
        // code block
        std::cout << "Not Handled Yet";
    }
}

void assignTemperatureDynamixelFeedback(int jID, uint16_t temperature)
{
    switch (jID)
    {
    case D_1:
        control_system.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_2:
        control_system.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_3:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_4:
        control_system.J1.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_5:
        control_system.J2.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_6:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_7:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_8:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_9:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_10:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_11:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
        break;
    case D_12:
        control_system.J3.PRESENT_VELOCITY = convertDynamixelSpeedToFloatSpeed(velocity);
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
    case DYN_TEMPERATURE:
        assignTemperatureDynamixelFeedback(motor, data);
        break;
    default:
        // code block
        std::cout << "Not Handled Yet " << cmd_id << "\n";
    }
    return;
}


int main(int argc, char **argv)
{
    //Start off by initialising the leg connection and variables to 0
    control_system.Back_Left.J0.TARGET_POSITION=0
    control_system.Back_Left.J0.TARGET_VELOCITY=0
    control_system.Back_Left.J0.PRESENT_VELOCITY=0
    control_system.Back_Left.J0.PRESENT_POSITION=0
    control_system.Back_Left.J0.PRESENT_TEMPERATURE=0
    control_system.Back_Left.J1 = control_system.Back_Left.J0
    control_system.Back_Left.J2 = control_system.Back_Left.J0
    control_system.Back_Right = control_system.Back_Left
    control_system.Back_Right = control_system.Front_Left
    control_system.Back_Right = control_system.Front_Right

    std::string port = detectPort();
    std::cout << "Port for openCM is" << port << std::endl;
    ros::init(argc, argv, "SENCE_Serial_Interface");
    ros::NodeHandle node;
    ros::Rate rate(1000);

    ros::Publisher system_publisher = node.advertise<sence_msgs::SENCE>("randle_state", 3);

    ros::Subscriber target_updater = node.subscribe("randle_target", 2, updateTargetsCallback);
    ros::Subscriber force_updater = node.subscribe("/hub_0/sensor_0", 1, forceSensorCallback);

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

    while (1)
    {
        readSerial(opencm_serial, 5);
        if (control_system.Back_Left.J0.PRESENT_POSITION != 0 ||
            control_system.Back_Right.J0.PRESENT_POSITION != 0 ||
            control_system.Front_Left.J0.PRESENT_POSITION != 0 ||
            control_system.Front_Right.J0.PRESENT_POSITION != 0)
        {
            std::cout << "syncingSystems" << std::endl;
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
            //cout << "Sending at " << time_now << std::endl;
            addCommandsToSend();
            write_serial_time = getClockTime();
        }

        rate.sleep();
        ros::spinOnce();
    }
    opencm_serial.close();

    return 0;
}