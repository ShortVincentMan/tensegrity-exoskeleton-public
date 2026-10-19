#pragma once

#include <Arduino.h>

struct ImuData {
    float yaw;
    float pitch;
    float roll;
    bool valid;
    unsigned long updatedMs;
    uint32_t packetCount;
    uint32_t badPacketCount;
};

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

void imuBegin();
void imuUpdate();
bool imuSensorsFresh();
const ImuData &imuUpperData();
const ImuData &imuForearmData();
float imuElbowAngleDegrees();
bool imuAverageNeutralQuaternion(Quaternion &result, bool requireAuto);
