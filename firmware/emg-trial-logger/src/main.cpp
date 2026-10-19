
#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

// ============================================================================
// SPOTTER-STYLE ASSIST-AS-NEEDED CONTROLLER + TRIAL LOGGER
// ============================================================================
// Hardware:
//   - ESP32
//   - Two UART BNO08x IMUs: upper arm and forearm
//   - Two A4988 drivers with separate STEP/DIR pins and shared ENABLE
//   - One motor per A4988; both motors wind and unwind together
//   - SPDT toggle: AUTO on GPIO 21, STOP on GPIO 22, common to GND
//   - Manual pretension/wind button on GPIO 23
//   - A4988 MS1/MS2/MS3 grounded, so the drivers use full-step mode
//
// Control idea:
//   SHADOW: follow the arm with intentional cable slack.
//   ASSIST: after an elevated hold and a slow downward dip, remove the slack
//           and restore a small, bounded amount of the lost angle.
//   RELEASE: slowly give the cable slack back, then return to SHADOW.
//
// Important limitations:
//   - IMUs do not measure physiological muscle fatigue. The dip detector is a
//     motion-based fatigue proxy. Intentional slow lowering can look the same.
//   - motorPositionSteps is open-loop. If either stepper stalls or skips, the
//     estimated spool position becomes wrong. Re-zero at the straight-arm,
//     just-taut condition before every test. A spool encoder is the real fix.
//   - The cable/angle relationship uses the Day 32 endpoint estimate because
//     direct on-body cable measurement was not repeatable under variable slack
//     and compliant structural motion.
//
// Serial Monitor setup:
//   - Baud: 921600
//   - Line ending: Newline
//
// Commands used for every trial:
//   session,20260924_VL01
//   trial,A01,ASSISTED,1,0.0
//   start
//   mark,FLEXION_START          (optional)
//   end
//
// Allowed condition codes: ASSISTED, MOTOR_OFF, NO_DEVICE.
// For MOTOR_OFF or NO_DEVICE, place the switch at STOP and run "senseonly"
// while the arm is straight before sending "start". The motors remain off.
// For ASSISTED, use the normal STOP -> pretension -> straight arm -> AUTO
// sequence before sending "start".
//
// GPIO 27 goes HIGH for 100 ms at TRIAL_START. Connect it only to a compatible
// isolated/digital trigger input on the EMG system. Otherwise leave it unused.
// ============================================================================

// ============================================================================
// HARDWARE PINS
// ============================================================================
HardwareSerial upperIMU(1);
HardwareSerial forearmIMU(2);

#define UPPER_RX 33
#define UPPER_TX 32
#define FOREARM_RX 26
#define FOREARM_TX 25

#define MOTOR1_STEP_PIN 19
#define MOTOR1_DIR_PIN 18
#define MOTOR2_STEP_PIN 17
#define MOTOR2_DIR_PIN 16
#define ENABLE_PIN 4

#define AUTO_SWITCH_PIN 21
#define STOP_SWITCH_PIN 22
#define MANUAL_WIND_BUTTON 23
#define SYNC_OUT_PIN 27

const bool MOTOR1_WIND_DIRECTION_LEVEL = HIGH;
const bool MOTOR2_WIND_DIRECTION_LEVEL = HIGH;
const bool DRIVER_ENABLE_LEVEL = LOW;    // A4988 ENABLE is active-low.

// ============================================================================
// MEASURED / ESTIMATED MECHANICS
// ============================================================================
const float SPOOL_TRAVEL_PER_REV_MM = 56.5f;
const float SAFE_CABLE_TRAVEL_MM = 200.0f;
const int MOTOR_FULL_STEPS_PER_REV = 200;

// MS1/MS2/MS3 are grounded, so the A4988 is in full-step mode.
const int MICROSTEP_FACTOR = 1;
const long PULSES_PER_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEP_FACTOR;

// Day 32 observation: the IMU calculation reports about 83 degrees when the
// physical elbow moves through about 100 degrees.
const float MEASURED_ANGLE_AT_PHYSICAL_100_DEG = 83.0f;

const long MAX_WIND_STEPS =
    lroundf((SAFE_CABLE_TRAVEL_MM / SPOOL_TRAVEL_PER_REV_MM) * PULSES_PER_REV);
const float STEPS_PER_MEASURED_DEGREE =
    (float)MAX_WIND_STEPS / MEASURED_ANGLE_AT_PHYSICAL_100_DEG;

// SHADOW runs slightly behind the just-taut cable position. A negative spool
// position is allowed only by this small amount so the controller can create
// slack even at the straight-arm position.
const float SHADOW_SLACK_MM = 5.0f;
const long SHADOW_SLACK_STEPS =
    lroundf((SHADOW_SLACK_MM / SPOOL_TRAVEL_PER_REV_MM) * PULSES_PER_REV);
const long MIN_UNWIND_STEPS = -SHADOW_SLACK_STEPS;

// ============================================================================
// MOTION CONTROL
// ============================================================================
const unsigned long CONTROL_PERIOD_US = 10000;  // 100 Hz state/control update
const unsigned long LOG_PERIOD_US = 20000;      // 50 Hz CSV output
const unsigned long RATE_WINDOW_MS = 1000;
const unsigned long SYNC_PULSE_MS = 100;
const unsigned long USB_SERIAL_BAUD = 921600;
const unsigned long SENSOR_TIMEOUT_MS = 250;
const float MAX_VALID_ANGLE_DEG = 150.0f;

const float ANGLE_FILTER_ALPHA = 0.15f;
const float VELOCITY_FILTER_ALPHA = 0.20f;

const long POSITION_DEADBAND_STEPS = 3L * MICROSTEP_FACTOR;
const float KP_STEP_RATE = 4.0f;
const float MIN_STEP_RATE = 75.0f * MICROSTEP_FACTOR;
const float SHADOW_MAX_STEP_RATE = 400.0f * MICROSTEP_FACTOR;
const float ASSIST_MAX_STEP_RATE = 300.0f * MICROSTEP_FACTOR;
const float RELEASE_UNWIND_STEP_RATE = 60.0f * MICROSTEP_FACTOR;
const float MANUAL_WIND_STEP_RATE = 150.0f * MICROSTEP_FACTOR;

// ============================================================================
// FATIGUE-PROXY / SPOTTER PARAMETERS
// ============================================================================
// A candidate lift must remain above this angle for this long before a dip can
// trigger assistance.
const float MIN_ELEVATED_ANGLE_DEG = 25.0f;
const unsigned long MIN_ELEVATED_TIME_MS = 2500;

// A downward drift must exceed this angle and persist this long.
const float DIP_TRIGGER_DEG = 3.0f;
const unsigned long DIP_CONFIRM_MS = 400;

// Fast lowering is treated as intentional motion and does not trigger assist.
// Slow intentional lowering still cannot be distinguished from fatigue by IMU.
const float INTENTIONAL_LOWERING_SPEED_DEG_S = -18.0f;

// Assistance only restores a small part of the lost motion.
const float MAX_ASSIST_CORRECTION_DEG = 5.0f;
const float ASSIST_RECOVERY_BAND_DEG = 1.5f;
const float RECOVERY_VELOCITY_DEG_S = 4.0f;
const unsigned long RECOVERY_CONFIRM_MS = 300;
const unsigned long MAX_ASSIST_TIME_MS = 2500;
const float ASSIST_CANCEL_ANGLE_DEG = 12.0f;

const unsigned long RELEASE_SETTLE_MS = 300;
const unsigned long ASSIST_COOLDOWN_MS = 1500;

const unsigned long SWITCH_DEBOUNCE_MS = 50;

// ============================================================================
// DATA TYPES AND STATE
// ============================================================================
struct RVCData {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
  bool valid = false;
  unsigned long updatedMs = 0;
  uint32_t packetCount = 0;
  uint32_t badPacketCount = 0;
};

struct RVCParser {
  uint8_t packet[19];
  int index = 0;
};

struct Quaternion {
  float w, x, y, z;
};

enum ControllerState {
  STATE_DISABLED,
  STATE_CALIBRATING,
  STATE_SHADOW,
  STATE_ASSIST,
  STATE_RELEASE,
  STATE_FAULT
};

enum TogglePosition {
  TOGGLE_UNKNOWN,
  TOGGLE_AUTO,
  TOGGLE_STOP,
  TOGGLE_BETWEEN,
  TOGGLE_INVALID
};

RVCData upperData, forearmData;
RVCParser upperParser, forearmParser;
Quaternion neutralRelative = {1, 0, 0, 0};

ControllerState controllerState = STATE_DISABLED;
TogglePosition rawToggle = TOGGLE_UNKNOWN;
TogglePosition stableToggle = TOGGLE_UNKNOWN;

bool stopSeenSinceBoot = false;
bool autoTransitionAuthorized = false;
bool zeroed = false;
bool sensingOnly = false;
bool trialLogging = false;
bool syncPulseActive = false;

unsigned long rawToggleChangedMs = 0;
unsigned long lastControlUs = 0;
unsigned long lastStepUs = 0;
unsigned long stateEnteredMs = 0;
unsigned long trialStartUs = 0;
unsigned long lastLogUs = 0;
unsigned long lastRateWindowMs = 0;
unsigned long syncPulseStartedMs = 0;

uint32_t previousUpperPacketCount = 0;
uint32_t previousForearmPacketCount = 0;
uint32_t controlTicksThisWindow = 0;
float measuredUpperPacketHz = 0.0f;
float measuredForearmPacketHz = 0.0f;
float measuredControlHz = 0.0f;

float filteredAngle = 0.0f;
float previousFilteredAngle = 0.0f;
float filteredVelocity = 0.0f;

long motorPositionSteps = 0;
long commandedTargetSteps = 0;
float commandedMaxStepRate = SHADOW_MAX_STEP_RATE;

unsigned long elevatedStartMs = 0;
unsigned long dipStartMs = 0;
unsigned long recoveryStartMs = 0;
unsigned long releaseSettledStartMs = 0;
unsigned long assistCooldownUntilMs = 0;

float preDipReferenceAngle = 0.0f;
float assistTargetAngle = 0.0f;

String serialLine;
String sessionId = "UNSET";
String trialId = "UNSET";
String trialCondition = "UNSET";
int trialRep = 0;
float trialLoadKg = NAN;

// ============================================================================
// BASIC HELPERS
// ============================================================================
const char *stateName() {
  if (sensingOnly) return "MOTOR_OFF";
  switch (controllerState) {
    case STATE_CALIBRATING: return "CALIBRATING";
    case STATE_SHADOW: return "FOLLOW";
    case STATE_ASSIST: return "ASSIST";
    case STATE_RELEASE: return "RELEASE";
    case STATE_FAULT: return "FAULT";
    default: return "DISABLED";
  }
}

const char *toggleName(TogglePosition position) {
  switch (position) {
    case TOGGLE_AUTO: return "AUTO";
    case TOGGLE_STOP: return "STOP";
    case TOGGLE_BETWEEN: return "BETWEEN";
    case TOGGLE_INVALID: return "INVALID";
    default: return "UNKNOWN";
  }
}

bool automaticStateActive() {
  return controllerState == STATE_SHADOW ||
         controllerState == STATE_ASSIST ||
         controllerState == STATE_RELEASE;
}

void setDriverEnabled(bool enabled) {
  digitalWrite(ENABLE_PIN, enabled ? DRIVER_ENABLE_LEVEL : !DRIVER_ENABLE_LEVEL);
}

TogglePosition readTogglePosition() {
  bool autoSelected = digitalRead(AUTO_SWITCH_PIN) == LOW;
  bool stopSelected = digitalRead(STOP_SWITCH_PIN) == LOW;

  if (autoSelected && !stopSelected) return TOGGLE_AUTO;
  if (stopSelected && !autoSelected) return TOGGLE_STOP;
  if (!autoSelected && !stopSelected) return TOGGLE_BETWEEN;
  return TOGGLE_INVALID;
}

bool rawToggleIsAuto() {
  return readTogglePosition() == TOGGLE_AUTO;
}

float spoolRevolutions() {
  return (float)motorPositionSteps / (float)PULSES_PER_REV;
}

float cableTravelMm() {
  return spoolRevolutions() * SPOOL_TRAVEL_PER_REV_MM;
}

long tautStepsForAngle(float measuredAngle) {
  long steps = lroundf(measuredAngle * STEPS_PER_MEASURED_DEGREE);
  return constrain(steps, 0L, MAX_WIND_STEPS);
}

long shadowStepsForAngle(float measuredAngle) {
  long steps = tautStepsForAngle(measuredAngle) - SHADOW_SLACK_STEPS;
  return constrain(steps, MIN_UNWIND_STEPS, MAX_WIND_STEPS);
}

void resetFatigueDetector() {
  elevatedStartMs = 0;
  dipStartMs = 0;
  recoveryStartMs = 0;
  preDipReferenceAngle = 0.0f;
}

void enterState(ControllerState nextState) {
  sensingOnly = false;
  controllerState = nextState;
  stateEnteredMs = millis();
  releaseSettledStartMs = 0;
  recoveryStartMs = 0;
  Serial.print("#EVENT,STATE,");
  Serial.println(stateName());
}

void disableController(const char *reason, bool fault = false) {
  bool wasActive = controllerState != STATE_DISABLED &&
                   controllerState != STATE_FAULT;
  controllerState = fault ? STATE_FAULT : STATE_DISABLED;
  sensingOnly = false;
  commandedTargetSteps = motorPositionSteps;
  resetFatigueDetector();
  setDriverEnabled(false);

  if (wasActive || fault) {
    Serial.print(fault ? "#EVENT,FAULT," : "#EVENT,STOPPED,");
    Serial.println(reason);
  }
}

// ============================================================================
// UART-RVC IMU PARSING
// ============================================================================
bool validChecksum(const uint8_t *packet) {
  uint8_t checksum = 0;
  for (int i = 2; i < 18; ++i) checksum += packet[i];
  return checksum == packet[18];
}

int16_t readInt16LE(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void decodePacket(const uint8_t *packet, RVCData &output) {
  output.yaw = readInt16LE(&packet[3]) * 0.01f;
  output.pitch = readInt16LE(&packet[5]) * 0.01f;
  output.roll = readInt16LE(&packet[7]) * 0.01f;
  output.valid = true;
  output.updatedMs = millis();
  output.packetCount++;
}

void updateParser(HardwareSerial &port, RVCParser &parser, RVCData &output) {
  while (port.available()) {
    uint8_t byteRead = port.read();

    if (parser.index == 0 && byteRead != 0xAA) continue;
    if (parser.index == 1 && byteRead != 0xAA) {
      parser.index = (byteRead == 0xAA) ? 1 : 0;
      continue;
    }

    parser.packet[parser.index++] = byteRead;

    if (parser.index == 19) {
      if (parser.packet[0] == 0xAA && parser.packet[1] == 0xAA &&
          validChecksum(parser.packet)) {
        decodePacket(parser.packet, output);
      } else {
        output.badPacketCount++;
      }
      parser.index = 0;
    }
  }
}

bool sensorsFresh() {
  unsigned long now = millis();
  return upperData.valid && forearmData.valid &&
         now - upperData.updatedMs <= SENSOR_TIMEOUT_MS &&
         now - forearmData.updatedMs <= SENSOR_TIMEOUT_MS;
}

// ============================================================================
// RELATIVE ORIENTATION / ELBOW ANGLE
// ============================================================================
float radiansFromDegrees(float degrees) {
  return degrees * PI / 180.0f;
}

Quaternion normalizeQuaternion(Quaternion quaternion) {
  float norm = sqrtf(quaternion.w * quaternion.w +
                     quaternion.x * quaternion.x +
                     quaternion.y * quaternion.y +
                     quaternion.z * quaternion.z);
  if (norm < 1.0e-6f) return {1, 0, 0, 0};
  return {quaternion.w / norm, quaternion.x / norm,
          quaternion.y / norm, quaternion.z / norm};
}

Quaternion conjugate(Quaternion quaternion) {
  return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

Quaternion multiply(Quaternion first, Quaternion second) {
  return {
    first.w*second.w - first.x*second.x - first.y*second.y - first.z*second.z,
    first.w*second.x + first.x*second.w + first.y*second.z - first.z*second.y,
    first.w*second.y - first.x*second.z + first.y*second.w + first.z*second.x,
    first.w*second.z + first.x*second.y - first.y*second.x + first.z*second.w
  };
}

Quaternion eulerToQuaternion(float yawDeg, float pitchDeg, float rollDeg) {
  float cy = cosf(radiansFromDegrees(yawDeg) * 0.5f);
  float sy = sinf(radiansFromDegrees(yawDeg) * 0.5f);
  float cp = cosf(radiansFromDegrees(pitchDeg) * 0.5f);
  float sp = sinf(radiansFromDegrees(pitchDeg) * 0.5f);
  float cr = cosf(radiansFromDegrees(rollDeg) * 0.5f);
  float sr = sinf(radiansFromDegrees(rollDeg) * 0.5f);

  return normalizeQuaternion({
    cr*cp*cy + sr*sp*sy,
    sr*cp*cy - cr*sp*sy,
    cr*sp*cy + sr*cp*sy,
    cr*cp*sy - sr*sp*cy
  });
}

Quaternion currentRelativeQuaternion() {
  Quaternion upperQ =
      eulerToQuaternion(upperData.yaw, upperData.pitch, upperData.roll);
  Quaternion forearmQ =
      eulerToQuaternion(forearmData.yaw, forearmData.pitch, forearmData.roll);
  return normalizeQuaternion(multiply(conjugate(upperQ), forearmQ));
}

float elbowAngleDegrees() {
  Quaternion change = normalizeQuaternion(
      multiply(conjugate(neutralRelative), currentRelativeQuaternion()));
  float boundedW = constrain(fabsf(change.w), 0.0f, 1.0f);
  return 2.0f * acosf(boundedW) * 180.0f / PI;
}

bool averageNeutralQuaternion(Quaternion &result, bool requireAuto) {
  const int REQUIRED_SAMPLES = 100;
  int sampleCount = 0;
  float sumW = 0.0f;
  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  unsigned long startedMs = millis();

  while (sampleCount < REQUIRED_SAMPLES && millis() - startedMs < 5000) {
    updateParser(upperIMU, upperParser, upperData);
    updateParser(forearmIMU, forearmParser, forearmData);

    // Automatic calibration requires AUTO. Motor-off sensing calibration
    // requires the debounced switch to remain at STOP.
    if (requireAuto && !rawToggleIsAuto()) return false;
    if (!requireAuto && stableToggle != TOGGLE_STOP) return false;
    if (!sensorsFresh()) continue;

    Quaternion sample = currentRelativeQuaternion();
    if (sampleCount > 0 &&
        (sample.w*sumW + sample.x*sumX + sample.y*sumY + sample.z*sumZ) < 0) {
      sample = {-sample.w, -sample.x, -sample.y, -sample.z};
    }

    sumW += sample.w;
    sumX += sample.x;
    sumY += sample.y;
    sumZ += sample.z;
    sampleCount++;
    delay(10);
  }

  if (sampleCount != REQUIRED_SAMPLES) return false;
  result = normalizeQuaternion({
    sumW / sampleCount,
    sumX / sampleCount,
    sumY / sampleCount,
    sumZ / sampleCount
  });
  return true;
}

// ============================================================================
// STEPPER OUTPUT
// ============================================================================
bool emitSharedStep(bool wind) {
  if (wind && motorPositionSteps >= MAX_WIND_STEPS) return false;
  if (!wind && motorPositionSteps <= MIN_UNWIND_STEPS) return false;

  digitalWrite(MOTOR1_DIR_PIN,
               wind ? MOTOR1_WIND_DIRECTION_LEVEL
                    : !MOTOR1_WIND_DIRECTION_LEVEL);
  digitalWrite(MOTOR2_DIR_PIN,
               wind ? MOTOR2_WIND_DIRECTION_LEVEL
                    : !MOTOR2_WIND_DIRECTION_LEVEL);
  delayMicroseconds(1);

  digitalWrite(MOTOR1_STEP_PIN, HIGH);
  digitalWrite(MOTOR2_STEP_PIN, HIGH);
  delayMicroseconds(3);
  digitalWrite(MOTOR1_STEP_PIN, LOW);
  digitalWrite(MOTOR2_STEP_PIN, LOW);

  motorPositionSteps += wind ? 1 : -1;
  return true;
}

void serviceAutomaticStepper() {
  if (!automaticStateActive() || stableToggle != TOGGLE_AUTO) return;

  long error = commandedTargetSteps - motorPositionSteps;
  if (labs(error) <= POSITION_DEADBAND_STEPS) return;

  bool wind = error > 0;
  float maximumRate = commandedMaxStepRate;

  // RELEASE must give cable back slowly. If the arm flexes farther while
  // releasing, winding is allowed at normal shadow speed to avoid excess slack.
  if (controllerState == STATE_RELEASE && !wind) {
    maximumRate = RELEASE_UNWIND_STEP_RATE;
  }

  float proportionalRate = KP_STEP_RATE * fabsf((float)error);
  float stepRate = maximumRate < MIN_STEP_RATE
                       ? maximumRate
                       : constrain(proportionalRate, MIN_STEP_RATE, maximumRate);
  unsigned long intervalUs = (unsigned long)(1000000.0f / stepRate);
  unsigned long nowUs = micros();

  if (nowUs - lastStepUs >= intervalUs) {
    emitSharedStep(wind);
    lastStepUs = nowUs;
  }
}

void serviceManualWind() {
  // Manual pretension is only available while the debounced toggle is at STOP
  // and automatic control is disabled. Switching to AUTO redefines this just-
  // taut position as zero after IMU calibration.
  bool manualAllowed = stableToggle == TOGGLE_STOP &&
                       controllerState == STATE_DISABLED &&
                       !sensingOnly;
  bool buttonPressed = digitalRead(MANUAL_WIND_BUTTON) == LOW;

  if (!manualAllowed || !buttonPressed) {
    if (controllerState == STATE_DISABLED) setDriverEnabled(false);
    return;
  }

  setDriverEnabled(true);
  unsigned long intervalUs =
      (unsigned long)(1000000.0f / MANUAL_WIND_STEP_RATE);
  unsigned long nowUs = micros();

  if (nowUs - lastStepUs >= intervalUs) {
    emitSharedStep(true);
    lastStepUs = nowUs;
  }
}

// ============================================================================
// SPOTTER STATE MACHINE
// ============================================================================
void beginRelease(const char *reason) {
  Serial.print("#EVENT,ASSIST_END,");
  Serial.println(reason);
  enterState(STATE_RELEASE);
  resetFatigueDetector();
}

void beginAssist() {
  float lostAngle = preDipReferenceAngle - filteredAngle;
  float correction = constrain(lostAngle, 0.0f, MAX_ASSIST_CORRECTION_DEG);
  assistTargetAngle = constrain(filteredAngle + correction,
                                0.0f,
                                MEASURED_ANGLE_AT_PHYSICAL_100_DEG);

  enterState(STATE_ASSIST);
  Serial.print("#EVENT,ASSIST_TRIGGER,target_deg=");
  Serial.print(assistTargetAngle, 2);
  Serial.print(",dip_deg=");
  Serial.println(lostAngle, 2);
}

void updateShadowDetector(unsigned long nowMs) {
  commandedTargetSteps = shadowStepsForAngle(filteredAngle);
  commandedMaxStepRate = SHADOW_MAX_STEP_RATE;

  if ((long)(nowMs - assistCooldownUntilMs) < 0) return;

  if (filteredAngle < MIN_ELEVATED_ANGLE_DEG) {
    resetFatigueDetector();
    return;
  }

  if (elevatedStartMs == 0) {
    elevatedStartMs = nowMs;
    preDipReferenceAngle = filteredAngle;
    return;
  }

  if (filteredAngle > preDipReferenceAngle) {
    preDipReferenceAngle = filteredAngle;
    dipStartMs = 0;
  }

  // A fast descent is assumed intentional. Restart the observation window at
  // the new angle rather than acting like a spotter.
  if (filteredVelocity <= INTENTIONAL_LOWERING_SPEED_DEG_S) {
    elevatedStartMs = nowMs;
    preDipReferenceAngle = filteredAngle;
    dipStartMs = 0;
    return;
  }

  bool observedLongEnough = nowMs - elevatedStartMs >= MIN_ELEVATED_TIME_MS;
  bool dippedEnough = preDipReferenceAngle - filteredAngle >= DIP_TRIGGER_DEG;

  if (!observedLongEnough || !dippedEnough) {
    dipStartMs = 0;
    return;
  }

  if (dipStartMs == 0) {
    dipStartMs = nowMs;
    return;
  }

  if (nowMs - dipStartMs >= DIP_CONFIRM_MS) beginAssist();
}

void updateAssist(unsigned long nowMs) {
  // ASSIST removes the shadow slack and commands a bounded recovery angle.
  commandedTargetSteps = tautStepsForAngle(assistTargetAngle);
  commandedMaxStepRate = ASSIST_MAX_STEP_RATE;

  if (filteredAngle <= ASSIST_CANCEL_ANGLE_DEG) {
    beginRelease("ARM_LOWERED");
    return;
  }

  bool recoveredAngle =
      filteredAngle >= assistTargetAngle - ASSIST_RECOVERY_BAND_DEG;
  bool risingAgain = filteredVelocity >= RECOVERY_VELOCITY_DEG_S;

  if (recoveredAngle || risingAgain) {
    if (recoveryStartMs == 0) recoveryStartMs = nowMs;
    if (nowMs - recoveryStartMs >= RECOVERY_CONFIRM_MS) {
      beginRelease(recoveredAngle ? "ANGLE_RECOVERED" : "UPWARD_MOTION");
      return;
    }
  } else {
    recoveryStartMs = 0;
  }

  if (nowMs - stateEnteredMs >= MAX_ASSIST_TIME_MS) {
    beginRelease("ASSIST_TIMEOUT");
  }
}

void updateRelease(unsigned long nowMs) {
  // The target follows the current arm angle but restores the intentional
  // shadow slack. serviceAutomaticStepper() rate-limits unwinding.
  commandedTargetSteps = shadowStepsForAngle(filteredAngle);
  commandedMaxStepRate = SHADOW_MAX_STEP_RATE;

  long error = commandedTargetSteps - motorPositionSteps;
  if (labs(error) <= POSITION_DEADBAND_STEPS) {
    if (releaseSettledStartMs == 0) releaseSettledStartMs = nowMs;
    if (nowMs - releaseSettledStartMs >= RELEASE_SETTLE_MS) {
      assistCooldownUntilMs = nowMs + ASSIST_COOLDOWN_MS;
      enterState(STATE_SHADOW);
      resetFatigueDetector();
    }
  } else {
    releaseSettledStartMs = 0;
  }
}

void controlTick() {
  if (!automaticStateActive() && !sensingOnly) return;

  if (!zeroed || !sensorsFresh()) {
    if (automaticStateActive()) {
      disableController("SENSOR_TIMEOUT_OR_NOT_ZEROED", true);
    }
    return;
  }

  float measuredAngle = elbowAngleDegrees();
  if (!isfinite(measuredAngle) || measuredAngle > MAX_VALID_ANGLE_DEG) {
    disableController("INVALID_ANGLE", true);
    return;
  }

  float oldFilteredAngle = filteredAngle;
  filteredAngle += ANGLE_FILTER_ALPHA * (measuredAngle - filteredAngle);

  float controlPeriodSeconds = CONTROL_PERIOD_US / 1000000.0f;
  float rawVelocity = (filteredAngle - oldFilteredAngle) / controlPeriodSeconds;
  filteredVelocity +=
      VELOCITY_FILTER_ALPHA * (rawVelocity - filteredVelocity);
  previousFilteredAngle = oldFilteredAngle;

  // Motor-off mode calculates the same angle and velocity without enabling
  // either driver or running the assistance state machine.
  if (sensingOnly) return;

  unsigned long nowMs = millis();
  if (controllerState == STATE_SHADOW) updateShadowDetector(nowMs);
  else if (controllerState == STATE_ASSIST) updateAssist(nowMs);
  else if (controllerState == STATE_RELEASE) updateRelease(nowMs);
}

// ============================================================================
// TOGGLE CONTROL AND AUTOMATIC ZEROING
// ============================================================================
void startAutomaticController() {
  autoTransitionAuthorized = false;
  setDriverEnabled(false);
  enterState(STATE_CALIBRATING);
  Serial.println("#EVENT,ZEROING,HOLD_ARM_STRAIGHT_MOTIONLESS_AND_CABLE_JUST_TAUT");

  Quaternion averagedNeutral;
  if (!averageNeutralQuaternion(averagedNeutral, true)) {
    zeroed = false;
    disableController("ZERO_FAILED_OR_AUTO_CANCELLED", true);
    return;
  }

  if (!rawToggleIsAuto()) {
    zeroed = false;
    disableController("AUTO_LEFT_DURING_ZERO");
    return;
  }

  neutralRelative = averagedNeutral;
  zeroed = true;

  // The straight-arm, just-taut spool position is the reference. SHADOW then
  // unwinds SHADOW_SLACK_MM from this reference.
  motorPositionSteps = 0;
  commandedTargetSteps = 0;
  filteredAngle = 0.0f;
  previousFilteredAngle = 0.0f;
  filteredVelocity = 0.0f;
  assistTargetAngle = 0.0f;
  assistCooldownUntilMs = millis() + ASSIST_COOLDOWN_MS;
  resetFatigueDetector();

  lastControlUs = micros();
  setDriverEnabled(true);
  enterState(STATE_SHADOW);
  Serial.println("#EVENT,AUTO_READY,FOLLOW_ACTIVE");
}

void startSensingOnly() {
  if (stableToggle != TOGGLE_STOP) {
    Serial.println("#ERROR,SENSE_ONLY_REQUIRES_STOP_SWITCH");
    return;
  }

  setDriverEnabled(false);
  controllerState = STATE_DISABLED;
  sensingOnly = true;
  zeroed = false;
  Serial.println("#EVENT,SENSE_ZEROING,HOLD_ARM_STRAIGHT_AND_MOTIONLESS");

  Quaternion averagedNeutral;
  if (!averageNeutralQuaternion(averagedNeutral, false)) {
    sensingOnly = false;
    Serial.println("#ERROR,SENSE_ZERO_FAILED");
    return;
  }

  neutralRelative = averagedNeutral;
  zeroed = true;
  filteredAngle = 0.0f;
  previousFilteredAngle = 0.0f;
  filteredVelocity = 0.0f;
  motorPositionSteps = 0;
  commandedTargetSteps = 0;
  stateEnteredMs = millis();
  lastControlUs = micros();
  Serial.println("#EVENT,SENSE_ONLY_READY,MOTORS_DISABLED");
}

void stopSensingOnly() {
  sensingOnly = false;
  zeroed = false;
  setDriverEnabled(false);
  Serial.println("#EVENT,SENSE_ONLY_STOPPED");
}

void handleStableToggleChange(TogglePosition newPosition) {
  Serial.print("#EVENT,TOGGLE,");
  Serial.println(toggleName(newPosition));

  if (newPosition == TOGGLE_STOP) {
    stopSeenSinceBoot = true;
    autoTransitionAuthorized = true;
    zeroed = false;
    disableController("TOGGLE_STOP");
    return;
  }

  if (newPosition == TOGGLE_AUTO) {
    if (!stopSeenSinceBoot || !autoTransitionAuthorized) {
      disableController("MOVE_TO_STOP_BEFORE_AUTO");
      Serial.println("#ERROR,AUTO_BLOCKED,MOVE_SWITCH_TO_STOP_THEN_BACK_TO_AUTO");
      return;
    }
    startAutomaticController();
    return;
  }

  // Between-contact and electrically invalid states are fail-safe disabled.
  disableController(newPosition == TOGGLE_INVALID ?
                    "INVALID_TOGGLE_STATE" : "TOGGLE_BETWEEN");
}

void updateToggle() {
  TogglePosition currentRaw = readTogglePosition();

  // Leaving the AUTO electrical contact stops pulses immediately. A later
  // debounced transition back to AUTO still requires a STOP visit first.
  if (automaticStateActive() && currentRaw != TOGGLE_AUTO) {
    setDriverEnabled(false);
    controllerState = STATE_DISABLED;
    zeroed = false;
    autoTransitionAuthorized = false;
    Serial.println("#EVENT,STOPPED,LEFT_AUTO_CONTACT");
  }

  if (currentRaw != rawToggle) {
    rawToggle = currentRaw;
    rawToggleChangedMs = millis();
  }

  if (currentRaw != stableToggle &&
      millis() - rawToggleChangedMs >= SWITCH_DEBOUNCE_MS) {
    stableToggle = currentRaw;
    handleStableToggleChange(stableToggle);
  }
}

// ============================================================================
// STRUCTURED TRIAL LOGGING
// ============================================================================
uint32_t droppedLogRows = 0;

String csvSafe(String value) {
  value.trim();
  value.replace(",", "_");
  value.replace(" ", "_");
  value.replace("\r", "");
  value.replace("\n", "");
  return value;
}

void printCsvFloat(float value, int decimals) {
  if (isfinite(value)) Serial.print(value, decimals);
}

void printCsvHeader() {
  Serial.println(
      "record_type,time_s,session_id,trial_id,condition,rep,load_kg,"
      "upper_yaw_deg,upper_pitch_deg,upper_roll_deg,"
      "forearm_yaw_deg,forearm_pitch_deg,forearm_roll_deg,"
      "raw_elbow_angle_deg,filtered_angle_deg,angular_velocity_deg_s,"
      "controller_state,toggle,zeroed,sensors_fresh,fault_flag,"
      "motor_position_steps,target_steps,step_error_steps,cable_travel_mm,"
      "reference_angle_deg,assist_target_deg,state_elapsed_ms,"
      "upper_age_ms,forearm_age_ms,upper_packet_hz,forearm_packet_hz,"
      "control_hz,upper_bad_packets,forearm_bad_packets,sync_out,dropped_log_rows");
}

void updateMeasuredRates() {
  unsigned long nowMs = millis();
  if (lastRateWindowMs == 0) {
    lastRateWindowMs = nowMs;
    previousUpperPacketCount = upperData.packetCount;
    previousForearmPacketCount = forearmData.packetCount;
    controlTicksThisWindow = 0;
    return;
  }

  unsigned long elapsedMs = nowMs - lastRateWindowMs;
  if (elapsedMs < RATE_WINDOW_MS) return;

  measuredUpperPacketHz =
      (upperData.packetCount - previousUpperPacketCount) * 1000.0f / elapsedMs;
  measuredForearmPacketHz =
      (forearmData.packetCount - previousForearmPacketCount) * 1000.0f / elapsedMs;
  measuredControlHz = controlTicksThisWindow * 1000.0f / elapsedMs;

  previousUpperPacketCount = upperData.packetCount;
  previousForearmPacketCount = forearmData.packetCount;
  controlTicksThisWindow = 0;
  lastRateWindowMs = nowMs;
}

void serviceSyncPulse() {
  if (syncPulseActive && millis() - syncPulseStartedMs >= SYNC_PULSE_MS) {
    digitalWrite(SYNC_OUT_PIN, LOW);
    syncPulseActive = false;
  }
}

void printDataRow() {
  unsigned long nowMs = millis();
  unsigned long upperAge = upperData.valid ? nowMs - upperData.updatedMs : 999999UL;
  unsigned long forearmAge = forearmData.valid ? nowMs - forearmData.updatedMs : 999999UL;
  bool fresh = sensorsFresh();
  float rawAngle = (zeroed && fresh) ? elbowAngleDegrees() : NAN;
  float elapsedSeconds = (micros() - trialStartUs) / 1000000.0f;

  Serial.print("DATA,");
  Serial.print(elapsedSeconds, 4); Serial.print(',');
  Serial.print(sessionId); Serial.print(',');
  Serial.print(trialId); Serial.print(',');
  Serial.print(trialCondition); Serial.print(',');
  Serial.print(trialRep); Serial.print(',');
  printCsvFloat(trialLoadKg, 3); Serial.print(',');

  printCsvFloat(upperData.yaw, 2); Serial.print(',');
  printCsvFloat(upperData.pitch, 2); Serial.print(',');
  printCsvFloat(upperData.roll, 2); Serial.print(',');
  printCsvFloat(forearmData.yaw, 2); Serial.print(',');
  printCsvFloat(forearmData.pitch, 2); Serial.print(',');
  printCsvFloat(forearmData.roll, 2); Serial.print(',');

  printCsvFloat(rawAngle, 2); Serial.print(',');
  printCsvFloat(zeroed ? filteredAngle : NAN, 2); Serial.print(',');
  printCsvFloat(zeroed ? filteredVelocity : NAN, 2); Serial.print(',');
  Serial.print(stateName()); Serial.print(',');
  Serial.print(toggleName(stableToggle)); Serial.print(',');
  Serial.print(zeroed ? 1 : 0); Serial.print(',');
  Serial.print(fresh ? 1 : 0); Serial.print(',');
  Serial.print(controllerState == STATE_FAULT ? 1 : 0); Serial.print(',');

  Serial.print(motorPositionSteps); Serial.print(',');
  Serial.print(commandedTargetSteps); Serial.print(',');
  Serial.print(commandedTargetSteps - motorPositionSteps); Serial.print(',');
  Serial.print(cableTravelMm(), 2); Serial.print(',');
  Serial.print(preDipReferenceAngle, 2); Serial.print(',');
  Serial.print(assistTargetAngle, 2); Serial.print(',');
  Serial.print(nowMs - stateEnteredMs); Serial.print(',');
  Serial.print(upperAge); Serial.print(',');
  Serial.print(forearmAge); Serial.print(',');
  Serial.print(measuredUpperPacketHz, 1); Serial.print(',');
  Serial.print(measuredForearmPacketHz, 1); Serial.print(',');
  Serial.print(measuredControlHz, 1); Serial.print(',');
  Serial.print(upperData.badPacketCount); Serial.print(',');
  Serial.print(forearmData.badPacketCount); Serial.print(',');
  Serial.print(syncPulseActive ? 1 : 0); Serial.print(',');
  Serial.println(droppedLogRows);
}

void serviceTrialLogger() {
  if (!trialLogging) return;

  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - lastLogUs;
  if (elapsedUs < LOG_PERIOD_US) return;

  if (elapsedUs >= 3UL * LOG_PERIOD_US) {
    droppedLogRows += elapsedUs / LOG_PERIOD_US - 1;
    lastLogUs = nowUs;
  } else {
    lastLogUs += LOG_PERIOD_US;
  }
  printDataRow();
}

void configureTrial(String payload) {
  int first = payload.indexOf(',');
  int second = payload.indexOf(',', first + 1);
  int third = payload.indexOf(',', second + 1);

  if (first < 1 || second < 0 || third < 0) {
    Serial.println("#ERROR,USE_trial_TRIALID_CONDITION_REP_LOADKG");
    return;
  }

  trialId = csvSafe(payload.substring(0, first));
  String conditionCode = csvSafe(payload.substring(first + 1, second));
  conditionCode.toUpperCase();
  if (conditionCode == "MOTOR_OFF") {
    trialCondition = "Device Motor Off";
  } else if (conditionCode == "ASSISTED") {
    trialCondition = "Closed-Loop Assisted";
  } else if (conditionCode == "NO_DEVICE") {
    trialCondition = "No Device (optional)";
  } else {
    Serial.println("#ERROR,CONDITION_MUST_BE_MOTOR_OFF_ASSISTED_OR_NO_DEVICE");
    return;
  }
  trialRep = payload.substring(second + 1, third).toInt();
  trialLoadKg = payload.substring(third + 1).toFloat();

  Serial.print("#TRIAL_CONFIG,");
  Serial.print(trialId); Serial.print(',');
  Serial.print(trialCondition); Serial.print(',');
  Serial.print(trialRep); Serial.print(',');
  Serial.println(trialLoadKg, 3);
}

void startTrialLogging() {
  if (sessionId == "UNSET" || trialId == "UNSET" ||
      trialCondition == "UNSET" || trialRep <= 0) {
    Serial.println("#ERROR,CONFIGURE_SESSION_AND_TRIAL_BEFORE_START");
    return;
  }
  if (!zeroed || !sensorsFresh()) {
    Serial.println("#ERROR,ZERO_AND_VERIFY_IMUS_BEFORE_START");
    return;
  }
  if (trialCondition == "Closed-Loop Assisted" && !automaticStateActive()) {
    Serial.println("#ERROR,ASSISTED_TRIAL_REQUIRES_AUTO_FOLLOW_STATE");
    return;
  }
  if ((trialCondition == "Device Motor Off" ||
       trialCondition == "No Device (optional)") && !sensingOnly) {
    Serial.println("#ERROR,MOTOR_OFF_OR_NO_DEVICE_REQUIRES_SENSEONLY");
    return;
  }

  trialStartUs = micros();
  lastLogUs = trialStartUs - LOG_PERIOD_US;
  droppedLogRows = 0;
  trialLogging = true;
  digitalWrite(SYNC_OUT_PIN, HIGH);
  syncPulseStartedMs = millis();
  syncPulseActive = true;
  Serial.println("#EVENT,TRIAL_START");
}

void stopTrialLogging() {
  trialLogging = false;
  digitalWrite(SYNC_OUT_PIN, LOW);
  syncPulseActive = false;
  Serial.print("#EVENT,TRIAL_END,dropped_rows=");
  Serial.println(droppedLogRows);
}

void printStatus() {
  Serial.print("#STATUS,state=");
  Serial.print(stateName());
  Serial.print(",toggle=");
  Serial.print(toggleName(stableToggle));
  Serial.print(",zeroed=");
  Serial.print(zeroed);
  Serial.print(",logging=");
  Serial.print(trialLogging);
  Serial.print(",filtered_deg=");
  Serial.print(filteredAngle, 2);
  Serial.print(",velocity_deg_s=");
  Serial.print(filteredVelocity, 2);
  Serial.print(",upper_hz=");
  Serial.print(measuredUpperPacketHz, 1);
  Serial.print(",forearm_hz=");
  Serial.print(measuredForearmPacketHz, 1);
  Serial.print(",control_hz=");
  Serial.println(measuredControlHz, 1);
}

void printRates() {
  Serial.print("#RATES,upper_hz=");
  Serial.print(measuredUpperPacketHz, 1);
  Serial.print(",forearm_hz=");
  Serial.print(measuredForearmPacketHz, 1);
  Serial.print(",control_hz=");
  Serial.print(measuredControlHz, 1);
  Serial.print(",upper_bad=");
  Serial.print(upperData.badPacketCount);
  Serial.print(",forearm_bad=");
  Serial.println(forearmData.badPacketCount);
}

void testAssist() {
  if (controllerState != STATE_SHADOW || stableToggle != TOGGLE_AUTO) {
    Serial.println("#ERROR,TEST_ASSIST_REQUIRES_FOLLOW_AND_AUTO");
    return;
  }

  preDipReferenceAngle = constrain(filteredAngle + DIP_TRIGGER_DEG,
                                   0.0f,
                                   MEASURED_ANGLE_AT_PHYSICAL_100_DEG);
  beginAssist();
}

void printHelp() {
  Serial.println("#COMMAND,session_ID");
  Serial.println("#COMMAND,trial_TRIALID_CONDITION_REP_LOADKG");
  Serial.println("#COMMAND,senseonly_or_senseoff_for_motor_off_trials");
  Serial.println("#COMMAND,start_end_mark_LABEL_header_rates_status_testassist_stop_help");
}

void handleSerialCommand(String command) {
  command.trim();
  String lower = command;
  lower.toLowerCase();

  if (lower.startsWith("session,")) {
    sessionId = csvSafe(command.substring(8));
    Serial.print("#SESSION,");
    Serial.println(sessionId);
  } else if (lower.startsWith("trial,")) {
    configureTrial(command.substring(6));
  } else if (lower.startsWith("mark,")) {
    Serial.print("#MARK,");
    Serial.print(trialLogging ? (micros() - trialStartUs) / 1000000.0f : -1.0f, 4);
    Serial.print(',');
    Serial.println(csvSafe(command.substring(5)));
  } else if (lower == "start") startTrialLogging();
  else if (lower == "end") stopTrialLogging();
  else if (lower == "senseonly") startSensingOnly();
  else if (lower == "senseoff") stopSensingOnly();
  else if (lower == "header") printCsvHeader();
  else if (lower == "rates") printRates();
  else if (lower == "status") printStatus();
  else if (lower == "testassist") testAssist();
  else if (lower == "stop") {
    stopTrialLogging();
    autoTransitionAuthorized = false;
    zeroed = false;
    disableController("SERIAL_STOP");
    Serial.println("#ACTION,MOVE_SWITCH_TO_STOP_BEFORE_RESTART");
  } else if (lower == "help") printHelp();
  else if (command.length() > 0) Serial.println("#ERROR,UNKNOWN_COMMAND,use_help");
}

void serviceSerial() {
  while (Serial.available()) {
    char character = (char)Serial.read();
    if (character == '\n' || character == '\r') {
      if (serialLine.length() > 0) {
        handleSerialCommand(serialLine);
        serialLine = "";
      }
    } else if (serialLine.length() < 160) {
      serialLine += character;
    }
  }
}

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================
void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  upperIMU.begin(115200, SERIAL_8N1, UPPER_RX, UPPER_TX);
  forearmIMU.begin(115200, SERIAL_8N1, FOREARM_RX, FOREARM_TX);

  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_STEP_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(AUTO_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STOP_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MANUAL_WIND_BUTTON, INPUT_PULLUP);
  pinMode(SYNC_OUT_PIN, OUTPUT);

  digitalWrite(MOTOR1_STEP_PIN, LOW);
  digitalWrite(MOTOR2_STEP_PIN, LOW);
  digitalWrite(MOTOR1_DIR_PIN, !MOTOR1_WIND_DIRECTION_LEVEL);
  digitalWrite(MOTOR2_DIR_PIN, !MOTOR2_WIND_DIRECTION_LEVEL);
  digitalWrite(SYNC_OUT_PIN, LOW);
  setDriverEnabled(false);
  serialLine.reserve(160);

  rawToggle = readTogglePosition();
  rawToggleChangedMs = millis();

  Serial.println("#D33_SPOTTER_ASSIST_TRIAL_LOGGER");
  Serial.println("#STARTUP,SELECT_STOP_PRETENSION_STRAIGHTEN_THEN_SELECT_AUTO");
  Serial.print("#MAX_WIND_STEPS,");
  Serial.println(MAX_WIND_STEPS);
  Serial.print("#FOLLOW_SLACK_STEPS,");
  Serial.println(SHADOW_SLACK_STEPS);
  Serial.print("#USB_SERIAL_BAUD,");
  Serial.println(USB_SERIAL_BAUD);
  printCsvHeader();
  printHelp();
}

void loop() {
  updateParser(upperIMU, upperParser, upperData);
  updateParser(forearmIMU, forearmParser, forearmData);
  updateMeasuredRates();
  serviceSyncPulse();

  updateToggle();
  serviceSerial();

  unsigned long nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
    lastControlUs = nowUs;
    controlTicksThisWindow++;
    controlTick();
  }

  serviceAutomaticStepper();
  serviceManualWind();
  serviceTrialLogger();
}
