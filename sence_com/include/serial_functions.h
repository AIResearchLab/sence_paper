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

#define M_PI    3.14159265358979323846  /* pi */

#define SPEED_RPM_TICK 0.229
#define POSITION_DYNA 0.088

vector<uint8_t> writeSerial(serial::Serial &teensy_serial, vector<uint8_t> commandVector);

uint64_t getClockTime();


float convertDynamixelPoseToFloatPose(uint16_t value);
uint16_t convertFloatPoseToDynamixelPose(float value);
uint16_t convertFloatTargetSpeedToDynamixelSpeed(float radiansPerSecond);
uint16_t convertDynamixelSpeedToFloatFeedbackSpeed(uint16_t feedbackValue);

float convertDynamixelSpeedToFloatSpeed(uint16_t dynamixelMeasuredSpeed);
std::string detectPort();