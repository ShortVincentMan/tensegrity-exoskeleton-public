#include "imu.h"

#include <HardwareSerial.h>
#include <math.h>

#include "config.h"
#include "pins.h"

namespace {
constexpr uint8_t PACKET_SIZE = 19;
constexpr uint8_t PACKET_HEADER = 0xAA;
HardwareSerial upperImu(1);
HardwareSerial forearmImu(2);
ImuData upperData = {0, 0, 0, false, 0, 0, 0};
ImuData forearmData = {0, 0, 0, false, 0, 0, 0};
uint8_t upperPacket[PACKET_SIZE] = {};
uint8_t forearmPacket[PACKET_SIZE] = {};
uint8_t upperIndex = 0;
uint8_t forearmIndex = 0;
Quaternion neutralRelative = {1, 0, 0, 0};

int16_t readInt16Le(const uint8_t *bytes) {
    return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) |
                                (static_cast<uint16_t>(bytes[1]) << 8));
}

bool validChecksum(const uint8_t *packet) {
    uint8_t checksum = 0;
    for (uint8_t index = 2; index < 18; ++index) checksum += packet[index];
    return checksum == packet[18];
}

void decodePacket(const uint8_t *packet, ImuData &data) {
    data.yaw = readInt16Le(&packet[3]) * 0.01f;
    data.pitch = readInt16Le(&packet[5]) * 0.01f;
    data.roll = readInt16Le(&packet[7]) * 0.01f;
    data.valid = true;
    data.updatedMs = millis();
    ++data.packetCount;
}

void updateParser(HardwareSerial &port, uint8_t *packet, uint8_t &index,
                 ImuData &data) {
    while (port.available()) {
        const uint8_t byteRead = port.read();
        if (index == 0) {
            if (byteRead == PACKET_HEADER) packet[index++] = byteRead;
            continue;
        }
        if (index == 1) {
            if (byteRead == PACKET_HEADER) packet[index++] = byteRead;
            else index = 0;
            continue;
        }
        packet[index++] = byteRead;
        if (index == PACKET_SIZE) {
            if (validChecksum(packet)) decodePacket(packet, data);
            else ++data.badPacketCount;
            index = 0;
        }
    }
}

Quaternion normalize(Quaternion value) {
    const float norm = sqrtf(value.w * value.w + value.x * value.x +
                             value.y * value.y + value.z * value.z);
    if (norm < 1.0e-6f) return {1, 0, 0, 0};
    return {value.w / norm, value.x / norm, value.y / norm, value.z / norm};
}

Quaternion conjugate(Quaternion value) {
    return {value.w, -value.x, -value.y, -value.z};
}

Quaternion multiply(Quaternion first, Quaternion second) {
    return {
        first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z,
        first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y,
        first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x,
        first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w};
}

Quaternion eulerToQuaternion(float yaw, float pitch, float roll) {
    const float cy = cosf(yaw * PI / 360.0f), sy = sinf(yaw * PI / 360.0f);
    const float cp = cosf(pitch * PI / 360.0f), sp = sinf(pitch * PI / 360.0f);
    const float cr = cosf(roll * PI / 360.0f), sr = sinf(roll * PI / 360.0f);
    return normalize({cr * cp * cy + sr * sp * sy,
                      sr * cp * cy - cr * sp * sy,
                      cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy});
}

Quaternion currentRelativeQuaternion() {
    const Quaternion upper = eulerToQuaternion(upperData.yaw, upperData.pitch,
                                                upperData.roll);
    const Quaternion forearm = eulerToQuaternion(forearmData.yaw,
                                                  forearmData.pitch,
                                                  forearmData.roll);
    return normalize(multiply(conjugate(upper), forearm));
}

bool toggleIsAuto() {
    return digitalRead(AUTO_SWITCH_PIN) == LOW && digitalRead(STOP_SWITCH_PIN) != LOW;
}

bool toggleIsStop() {
    return digitalRead(STOP_SWITCH_PIN) == LOW && digitalRead(AUTO_SWITCH_PIN) != LOW;
}
}

void imuBegin() {
    upperImu.begin(IMU_BAUD, SERIAL_8N1, IMU1_RX_PIN, IMU1_TX_PIN);
    forearmImu.begin(IMU_BAUD, SERIAL_8N1, IMU2_RX_PIN, IMU2_TX_PIN);
}

void imuUpdate() {
    updateParser(upperImu, upperPacket, upperIndex, upperData);
    updateParser(forearmImu, forearmPacket, forearmIndex, forearmData);
}

bool imuSensorsFresh() {
    const unsigned long now = millis();
    return upperData.valid && forearmData.valid &&
           now - upperData.updatedMs <= IMU_TIMEOUT_MS &&
           now - forearmData.updatedMs <= IMU_TIMEOUT_MS;
}

const ImuData &imuUpperData() { return upperData; }
const ImuData &imuForearmData() { return forearmData; }

float imuElbowAngleDegrees() {
    const Quaternion change = normalize(
        multiply(conjugate(neutralRelative), currentRelativeQuaternion()));
    const float boundedW = constrain(fabsf(change.w), 0.0f, 1.0f);
    return 2.0f * acosf(boundedW) * 180.0f / PI;
}

bool imuAverageNeutralQuaternion(Quaternion &result, bool requireAuto) {
    constexpr int REQUIRED_SAMPLES = 100;
    int sampleCount = 0;
    float sumW = 0, sumX = 0, sumY = 0, sumZ = 0;
    const unsigned long startedMs = millis();
    while (sampleCount < REQUIRED_SAMPLES && millis() - startedMs < 5000) {
        imuUpdate();
        if ((requireAuto && !toggleIsAuto()) || (!requireAuto && !toggleIsStop())) return false;
        if (!imuSensorsFresh()) continue;
        Quaternion sample = currentRelativeQuaternion();
        if (sampleCount > 0 && sample.w * sumW + sample.x * sumX +
            sample.y * sumY + sample.z * sumZ < 0) {
            sample = {-sample.w, -sample.x, -sample.y, -sample.z};
        }
        sumW += sample.w;
        sumX += sample.x;
        sumY += sample.y;
        sumZ += sample.z;
        ++sampleCount;
        delay(10);
    }
    if (sampleCount != REQUIRED_SAMPLES) return false;
    result = normalize({sumW / sampleCount, sumX / sampleCount,
                        sumY / sampleCount, sumZ / sampleCount});
    neutralRelative = result;
    return true;
}
