#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

HardwareSerial IMU1(1);
HardwareSerial IMU2(2);

constexpr int IMU1_RX = 33;
constexpr int IMU1_TX = 32;
constexpr int IMU2_RX = 26;
constexpr int IMU2_TX = 25;
constexpr unsigned long IMU_TIMEOUT_MS = 500;

struct RVCData {
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  bool valid = false;
  unsigned long lastPacketTime = 0;
};

RVCData imu1Data;
RVCData imu2Data;

uint8_t imu1Buffer[19];
uint8_t imu2Buffer[19];
int imu1Index = 0;
int imu2Index = 0;
unsigned long lastPrint = 0;

bool validChecksum(const uint8_t *packet) {
  uint8_t checksum = 0;
  for (int index = 2; index < 18; ++index) checksum += packet[index];
  return checksum == packet[18];
}

void decodePacket(const uint8_t *packet, RVCData &data) {
  const int16_t yawRaw = static_cast<int16_t>((packet[4] << 8) | packet[3]);
  const int16_t pitchRaw = static_cast<int16_t>((packet[6] << 8) | packet[5]);
  const int16_t rollRaw = static_cast<int16_t>((packet[8] << 8) | packet[7]);

  data.yaw = yawRaw * 0.01f;
  data.pitch = pitchRaw * 0.01f;
  data.roll = rollRaw * 0.01f;
  data.valid = true;
  data.lastPacketTime = millis();
}

void readRVC(HardwareSerial &serial, uint8_t *buffer, int &index, RVCData &data) {
  while (serial.available()) {
    const uint8_t byteRead = serial.read();

    if (index == 0) {
      if (byteRead == 0xAA) buffer[index++] = byteRead;
      continue;
    }

    if (index == 1) {
      if (byteRead == 0xAA) {
        buffer[index++] = byteRead;
      } else {
        index = 0;
      }
      continue;
    }

    buffer[index++] = byteRead;
    if (index >= 19) {
      if (validChecksum(buffer)) decodePacket(buffer, data);
      index = 0;
    }
  }
}

bool imuConnected(const RVCData &data) {
  return data.valid && millis() - data.lastPacketTime < IMU_TIMEOUT_MS;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  IMU1.begin(115200, SERIAL_8N1, IMU1_RX, IMU1_TX);
  IMU2.begin(115200, SERIAL_8N1, IMU2_RX, IMU2_TX);

  Serial.println("================================");
  Serial.println("Tensegrity Exoskeleton IMU Test");
  Serial.println("================================");
  Serial.println("IMU1: RX 33 / TX 32");
  Serial.println("IMU2: RX 26 / TX 25");
}

void loop() {
  readRVC(IMU1, imu1Buffer, imu1Index, imu1Data);
  readRVC(IMU2, imu2Buffer, imu2Index, imu2Data);

  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    const bool imu1OK = imuConnected(imu1Data);
    const bool imu2OK = imuConnected(imu2Data);

    Serial.println("--------------------------------");

    Serial.print("IMU1: ");
    if (imu1OK) {
      Serial.print("CONNECTED | Pitch: ");
      Serial.print(imu1Data.pitch, 1);
      Serial.println(" deg");
    } else {
      Serial.println("NOT CONNECTED");
    }

    Serial.print("IMU2: ");
    if (imu2OK) {
      Serial.print("CONNECTED | Pitch: ");
      Serial.print(imu2Data.pitch, 1);
      Serial.println(" deg");
    } else {
      Serial.println("NOT CONNECTED");
    }

    if (imu1OK && imu2OK) {
      Serial.print("ARM ANGLE: ");
      Serial.print(fabs(imu2Data.pitch - imu1Data.pitch), 1);
      Serial.println(" deg");
    } else {
      Serial.println("ARM ANGLE: unavailable");
    }
  }
}
