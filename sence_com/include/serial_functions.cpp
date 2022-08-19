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