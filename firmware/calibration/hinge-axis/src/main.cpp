#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

HardwareSerial IMU1(1);
HardwareSerial IMU2(2);

#define IMU1_RX 26
#define IMU1_TX 25
#define IMU2_RX 33
#define IMU2_TX 32

struct RVCData {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
  bool valid = false;
  unsigned long lastUpdate = 0;
};

struct Quaternion {
  float w, x, y, z;
};

struct Vector3 {
  float x, y, z;
};

RVCData imu1Data, imu2Data;
uint8_t imu1Buffer[19], imu2Buffer[19];
int imu1Index = 0, imu2Index = 0;

Quaternion neutralRelative = {1, 0, 0, 0};
Vector3 hingeAxis = {1, 0, 0};
bool neutralCalibrated = false;
bool axisCalibrated = false;
bool hingeZeroCalibrated = false;
float hingeZeroOffset = 0.0f;
unsigned long lastLivePrint = 0;

// ---------------- RVC packet handling ----------------

bool validChecksum(uint8_t *packet) {
  uint8_t checksum = 0;
  for (int i = 2; i < 18; i++) checksum += packet[i];
  return checksum == packet[18];
}

void decodePacket(uint8_t *packet, RVCData &data) {
  int16_t yawRaw   = (int16_t)((packet[4] << 8) | packet[3]);
  int16_t pitchRaw = (int16_t)((packet[6] << 8) | packet[5]);
  int16_t rollRaw  = (int16_t)((packet[8] << 8) | packet[7]);

  data.yaw = yawRaw * 0.01f;
  data.pitch = pitchRaw * 0.01f;
  data.roll = rollRaw * 0.01f;
  data.valid = true;
  data.lastUpdate = millis();
}

void readRVC(HardwareSerial &port, uint8_t *buffer, int &index, RVCData &data) {
  while (port.available()) {
    uint8_t b = (uint8_t)port.read();

    if (index == 0) {
      if (b == 0xAA) buffer[index++] = b;
      continue;
    }

    if (index == 1) {
      if (b == 0xAA) buffer[index++] = b;
      else index = 0;
      continue;
    }

    buffer[index++] = b;

    if (index == 19) {
      if (validChecksum(buffer)) decodePacket(buffer, data);
      index = 0;
    }
  }
}

void updateIMUs() {
  readRVC(IMU1, imu1Buffer, imu1Index, imu1Data);
  readRVC(IMU2, imu2Buffer, imu2Index, imu2Data);
}

bool sensorsFresh() {
  unsigned long now = millis();
  return imu1Data.valid && imu2Data.valid &&
         now - imu1Data.lastUpdate < 250 &&
         now - imu2Data.lastUpdate < 250;
}

// ---------------- Quaternion math ----------------

Quaternion normalizeQuaternion(Quaternion q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1.0e-6f) return {1, 0, 0, 0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

Quaternion conjugateQuaternion(Quaternion q) {
  return {q.w, -q.x, -q.y, -q.z};
}

Quaternion multiplyQuaternion(Quaternion a, Quaternion b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

float quaternionDot(Quaternion a, Quaternion b) {
  return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

Quaternion eulerToQuaternion(float yawDeg, float pitchDeg, float rollDeg) {
  float yaw = yawDeg * DEG_TO_RAD * 0.5f;
  float pitch = pitchDeg * DEG_TO_RAD * 0.5f;
  float roll = rollDeg * DEG_TO_RAD * 0.5f;

  float cy = cosf(yaw),   sy = sinf(yaw);
  float cp = cosf(pitch), sp = sinf(pitch);
  float cr = cosf(roll),  sr = sinf(roll);

  // Standard intrinsic yaw(Z)-pitch(Y)-roll(X) composition.
  return normalizeQuaternion({
    cr*cp*cy + sr*sp*sy,
    sr*cp*cy - cr*sp*sy,
    cr*sp*cy + sr*cp*sy,
    cr*cp*sy - sr*sp*cy
  });
}

Quaternion getRelativeOrientation() {
  Quaternion upper = eulerToQuaternion(
    imu1Data.yaw, imu1Data.pitch, imu1Data.roll);
  Quaternion forearm = eulerToQuaternion(
    imu2Data.yaw, imu2Data.pitch, imu2Data.roll);
  return normalizeQuaternion(
    multiplyQuaternion(conjugateQuaternion(upper), forearm));
}

Quaternion getChangeFromNeutral() {
  Quaternion change = normalizeQuaternion(multiplyQuaternion(
    conjugateQuaternion(neutralRelative), getRelativeOrientation()));

  // q and -q are the same rotation. Keep the shortest representation.
  if (change.w < 0.0f) {
    change.w = -change.w;
    change.x = -change.x;
    change.y = -change.y;
    change.z = -change.z;
  }
  return change;
}

float vectorMagnitude(Vector3 v) {
  return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

Vector3 normalizeVector(Vector3 v) {
  float n = vectorMagnitude(v);
  if (n < 1.0e-6f) return {0, 0, 0};
  return {v.x/n, v.y/n, v.z/n};
}

float vectorDot(Vector3 a, Vector3 b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

float totalRotationDegrees(Quaternion q) {
  float w = constrain(fabsf(q.w), 0.0f, 1.0f);
  return 2.0f * acosf(w) * RAD_TO_DEG;
}

// Swing-twist decomposition: retain only rotation parallel to hingeAxis.
float getSignedHingeAngle() {
  Quaternion q = getChangeFromNeutral();
  float parallel = q.x*hingeAxis.x + q.y*hingeAxis.y + q.z*hingeAxis.z;

  Quaternion twist = normalizeQuaternion({
    q.w,
    hingeAxis.x * parallel,
    hingeAxis.y * parallel,
    hingeAxis.z * parallel
  });

  float signedSinHalf = twist.x*hingeAxis.x +
                        twist.y*hingeAxis.y +
                        twist.z*hingeAxis.z;
  float angle = 2.0f * atan2f(signedSinHalf, twist.w) * RAD_TO_DEG;

  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float getElbowAngle() {
  return getSignedHingeAngle() - hingeZeroOffset;
}

// ---------------- Calibration ----------------

bool waitForFreshSensors(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (!sensorsFresh()) {
    updateIMUs();
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

void recordNeutral() {
  Serial.println("# Hold arm straight and completely motionless...");
  neutralCalibrated = false;
  axisCalibrated = false;
  hingeZeroCalibrated = false;
  hingeZeroOffset = 0.0f;

  if (!waitForFreshSensors(5000)) {
    Serial.println("# ERROR: IMU data unavailable");
    return;
  }

  Serial.println("# Collecting 100 neutral samples...");
  Quaternion first = getRelativeOrientation();
  float sw = 0, sx = 0, sy = 0, sz = 0;
  int count = 0;
  unsigned long start = millis();
  unsigned long lastSample = 0;

  while (count < 100) {
    updateIMUs();
    unsigned long now = millis();
    if (now - start > 10000) {
      Serial.println("# ERROR: Neutral calibration timed out");
      return;
    }
    if (sensorsFresh() && now - lastSample >= 20) {
      Quaternion q = getRelativeOrientation();
      if (quaternionDot(q, first) < 0.0f) {
        q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z;
      }
      sw += q.w; sx += q.x; sy += q.y; sz += q.z;
      count++;
      lastSample = now;
    }
  }

  neutralRelative = normalizeQuaternion({sw/count, sx/count, sy/count, sz/count});
  neutralCalibrated = true;
  Serial.println("# Neutral calibration complete");
  Serial.println("# Next enter: axis");
}

void calibrateHingeAxis() {
  if (!neutralCalibrated) {
    Serial.println("# ERROR: Run z first");
    return;
  }

  Serial.println("# AXIS CALIBRATION STARTING");
  Serial.println("# Keep upper arm and palm orientation fixed.");
  Serial.println("# Perform 5 slow full elbow curls for 15 seconds.");
  delay(2000);

  Vector3 sum = {0, 0, 0};
  Vector3 referenceAxis = {0, 0, 0};
  bool haveReference = false;
  int accepted = 0;
  unsigned long start = millis();
  unsigned long lastSample = 0;

  while (millis() - start < 15000) {
    updateIMUs();
    unsigned long now = millis();
    if (!sensorsFresh() || now - lastSample < 20) continue;
    lastSample = now;

    Quaternion q = getChangeFromNeutral();
    float sinHalf = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z);
    if (sinHalf < 1.0e-5f) continue;

    float angleRad = 2.0f * atan2f(sinHalf, q.w);
    float angleDeg = angleRad * RAD_TO_DEG;

    // Very small rotations have an unreliable axis; exclude them.
    if (angleDeg < 15.0f || angleDeg > 150.0f) continue;

    Vector3 sampleAxis = {q.x/sinHalf, q.y/sinHalf, q.z/sinHalf};

    if (!haveReference) {
      referenceAxis = sampleAxis;
      haveReference = true;
    }
    if (vectorDot(sampleAxis, referenceAxis) < 0.0f) {
      sampleAxis.x = -sampleAxis.x;
      sampleAxis.y = -sampleAxis.y;
      sampleAxis.z = -sampleAxis.z;
    }

    // Weight deeper flexion more heavily than near-neutral samples.
    sum.x += sampleAxis.x * angleRad;
    sum.y += sampleAxis.y * angleRad;
    sum.z += sampleAxis.z * angleRad;
    accepted++;
  }

  if (accepted < 100 || vectorMagnitude(sum) < 1.0e-4f) {
    Serial.println("# ERROR: Insufficient curl motion; axis not saved");
    return;
  }

  hingeAxis = normalizeVector(sum);
  axisCalibrated = true;
  hingeZeroCalibrated = false;
  hingeZeroOffset = 0.0f;

  Serial.print("AXIS,");
  Serial.print(hingeAxis.x, 6); Serial.print(',');
  Serial.print(hingeAxis.y, 6); Serial.print(',');
  Serial.print(hingeAxis.z, 6); Serial.print(',');
  Serial.println(accepted);
  Serial.println("# Hinge-axis calibration complete");
  Serial.println("# Hold the arm straight, then enter: zeroaxis");
}

void calibrateHingeZero() {
  if (!axisCalibrated) {
    Serial.println("# ERROR: Run z, then axis");
    return;
  }

  Serial.println("# Hold arm straight in the testing pose and remain motionless...");
  float sum = 0.0f;
  int count = 0;
  unsigned long start = millis();
  unsigned long lastSample = 0;

  while (count < 100) {
    updateIMUs();
    unsigned long now = millis();
    if (now - start > 10000) {
      Serial.println("# ERROR: Hinge zero calibration timed out");
      return;
    }
    if (sensorsFresh() && now - lastSample >= 20) {
      sum += getSignedHingeAngle();
      count++;
      lastSample = now;
    }
  }

  hingeZeroOffset = sum / count;
  hingeZeroCalibrated = true;
  Serial.print("HINGE_ZERO,");
  Serial.println(hingeZeroOffset, 2);
  Serial.println("# Post-axis zero complete");
  Serial.println("# Commands: live, c0, c45, c90, c120, raw");
}

// ---------------- Measurement ----------------

bool collectSamples(float &mean, float &sd, float &minimum,
                    float &maximum, int requestedSamples) {
  if (!axisCalibrated || !hingeZeroCalibrated) return false;
  float sum = 0.0f, sumSq = 0.0f;
  minimum = 10000.0f;
  maximum = -10000.0f;
  int count = 0;
  unsigned long start = millis();
  unsigned long lastSample = 0;

  while (count < requestedSamples) {
    updateIMUs();
    unsigned long now = millis();
    if (now - start > 10000) return false;
    if (sensorsFresh() && now - lastSample >= 20) {
      float angle = getElbowAngle();
      sum += angle;
      sumSq += angle*angle;
      if (angle < minimum) minimum = angle;
      if (angle > maximum) maximum = angle;
      count++;
      lastSample = now;
    }
  }

  mean = sum/count;
  float variance = sumSq/count - mean*mean;
  sd = sqrtf(fmaxf(variance, 0.0f));
  return true;
}

void captureTrial(float referenceAngle) {
  if (!axisCalibrated || !hingeZeroCalibrated) {
    Serial.println("# ERROR: Run z, axis, then zeroaxis");
    return;
  }
  Serial.println("# Hold position motionless...");
  float mean, sd, minimum, maximum;
  if (!collectSamples(mean, sd, minimum, maximum, 100)) {
    Serial.println("# ERROR: Could not collect trial");
    return;
  }
  Serial.print("RESULT,");
  Serial.print(referenceAngle, 2); Serial.print(',');
  Serial.print(mean, 2); Serial.print(',');
  Serial.print(mean-referenceAngle, 2); Serial.print(',');
  Serial.print(sd, 2); Serial.print(',');
  Serial.print(minimum, 2); Serial.print(',');
  Serial.print(maximum, 2); Serial.println(",100");
}

void printLive() {
  if (!axisCalibrated || !hingeZeroCalibrated || !sensorsFresh()) {
    Serial.println("# ERROR: Run z, axis, zeroaxis; verify both IMUs");
    return;
  }
  Quaternion change = getChangeFromNeutral();
  Serial.print("LIVE,");
  Serial.print(millis()); Serial.print(',');
  Serial.print(getElbowAngle(), 2); Serial.print(',');
  Serial.println(totalRotationDegrees(change), 2);
}

void printRawComparison() {
  if (!axisCalibrated || !hingeZeroCalibrated || !sensorsFresh()) {
    Serial.println("# ERROR: Run z, axis, zeroaxis; verify both IMUs");
    return;
  }
  Quaternion change = getChangeFromNeutral();
  Serial.print("COMPARE,HINGE_DEG,");
  Serial.print(getElbowAngle(), 2);
  Serial.print(",TOTAL_3D_DEG,");
  Serial.println(totalRotationDegrees(change), 2);
}

void handleCommand(String command) {
  command.trim();
  if (command.equalsIgnoreCase("z")) recordNeutral();
  else if (command.equalsIgnoreCase("axis")) calibrateHingeAxis();
  else if (command.equalsIgnoreCase("zeroaxis")) calibrateHingeZero();
  else if (command.equalsIgnoreCase("live")) printLive();
  else if (command.equalsIgnoreCase("raw")) printRawComparison();
  else if (command.length() > 1 &&
          (command.charAt(0) == 'c' || command.charAt(0) == 'C')) {
    captureTrial(command.substring(1).toFloat());
  } else {
    Serial.println("# Commands: z, axis, zeroaxis, live, raw, c0, c45, c90, c120");
  }
}

void setup() {
  Serial.begin(115200);
  IMU1.begin(115200, SERIAL_8N1, IMU1_RX, IMU1_TX);
  IMU2.begin(115200, SERIAL_8N1, IMU2_RX, IMU2_TX);
  delay(1000);

  Serial.println("# D33-HINGE-AXIS: SWING-TWIST-02");
  Serial.println("# Upper-arm IMU: UART1 RX26 TX25");
  Serial.println("# Forearm IMU: UART2 RX33 TX32");
  Serial.println("# Serial Monitor: 115200, Newline");
  Serial.println("# Commands: z, axis, zeroaxis, live, raw, c0, c45, c90, c120");
  Serial.println("TYPE,REFERENCE_DEG,MEASURED_DEG,ERROR_DEG,STD_DEV_DEG,MIN_DEG,MAX_DEG,SAMPLES");
}

void loop() {
  updateIMUs();
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }
}