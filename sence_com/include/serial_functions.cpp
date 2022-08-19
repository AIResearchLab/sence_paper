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