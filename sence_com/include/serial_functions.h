#include <chrono>
#include <ostream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <sstream>
#include <queue>
#include <cmath>
#include "serial/serial.h"
using namespace std;
using std::vector;


vector<uint8_t> writeSerial(serial::Serial &teensy_serial, vector<uint8_t> commandVector);

uint64_t getClockTime();
float convertDynamixelPoseToFloatPose(uint16_t value, uint16_t offset);
uint16_t convertFloatPoseToDynamixelPose(float value, uint16_t offset);
uint16_t convertFloatSpeedToDynamixelSpeed(float radiansPerSecond);
float convertDynamixelSpeedToFloatSpeed(uint16_t dynamixelMeasuredSpeed);
std::string detectPort();