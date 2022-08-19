#include "serial_com_functions.h"

std::string detectPort()
{
    //todo update code to find open CM board, need to verify pid/vid values
    std::string hardware_serial_processor = "fff1:ff48"; // If the description of the device contains this then it isn't the coms cable.
    std::string port;
    std::string ids;

    // Goes through through all the ports and checks for a device with a vid_pid of a Teensy or Arduino Uno
    std::vector<serial::PortInfo> devices_found = serial::list_ports();
    //std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

    for (serial::PortInfo &element : devices_found)
    {
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