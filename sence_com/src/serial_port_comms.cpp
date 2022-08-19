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

int main(int argc, char **argv)
{
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
        readSerial(opencm_serial, 2);
        if (control_system.J1.PRESENT_POSITION != 0 &&
            control_system.J2.PRESENT_POSITION != 0 && control_system.J3.PRESENT_POSITION != 0)
        {
            std::cout << "syncingSystems" << std::endl;
            baseSyncronization();

            break;
        }
    }
    
    control_system.J0.TARGET_POSITION = 0;

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