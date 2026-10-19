#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
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
bool liveOutput = false;
bool baselineMode = false;
bool recording = false;
unsigned long trialStartMs = 0, lastLogMs = 0, skippedLogRows = 0;
unsigned long assistCount = 0;
float actualControlDt = 0.01f;
const unsigned long LOG_PERIOD_MS = 40; // 25 Hz; actual timestamps are recorded.

unsigned long rawToggleChangedMs = 0;
unsigned long lastControlUs = 0;
unsigned long lastStepUs = 0;
unsigned long lastLivePrintMs = 0;
unsigned long stateEnteredMs = 0;
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
// ============================================================================
// BASIC HELPERS
// ============================================================================
const char *stateName() {
  switch (controllerState) {
    case STATE_CALIBRATING: return "CALIBRATING";
    case STATE_SHADOW: return "SHADOW";
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
  digitalWrite(ENABLE_PIN, (enabled && !baselineMode) ? DRIVER_ENABLE_LEVEL : !DRIVER_ENABLE_LEVEL);
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
  controllerState = nextState;
  stateEnteredMs = millis();
  releaseSettledStartMs = 0;
  recoveryStartMs = 0;
  Serial.print("STATE,");
  Serial.println(stateName());
}
void disableController(const char *reason, bool fault = false) {
  bool wasActive = controllerState != STATE_DISABLED &&
                   controllerState != STATE_FAULT;
  controllerState = fault ? STATE_FAULT : STATE_DISABLED;
  commandedTargetSteps = motorPositionSteps;
  resetFatigueDetector();
  setDriverEnabled(false);
  if (wasActive || fault) {
    Serial.print(fault ? "FAULT," : "STOPPED,");
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
bool averageNeutralQuaternion(Quaternion &result) {
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
    // Moving away from AUTO cancels zeroing before any motor is enabled.
    if (!rawToggleIsAuto()) return false;
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
  if (baselineMode) return false;
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
  if (baselineMode || !automaticStateActive() || stableToggle != TOGGLE_AUTO) return;
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
  if (baselineMode) { setDriverEnabled(false); return; }
  // Manual pretension is only available while the debounced toggle is at STOP
  // and automatic control is disabled. Switching to AUTO redefines this just-
  // taut position as zero after IMU calibration.
  bool manualAllowed = stableToggle == TOGGLE_STOP &&
                       controllerState == STATE_DISABLED;
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
  Serial.print("ASSIST_END,");
  Serial.println(reason);
  enterState(STATE_RELEASE);
  resetFatigueDetector();
}
void beginAssist() {
  if (baselineMode) return;
  assistCount++;
  float lostAngle = preDipReferenceAngle - filteredAngle;
  float correction = constrain(lostAngle, 0.0f, MAX_ASSIST_CORRECTION_DEG);
  assistTargetAngle = constrain(filteredAngle + correction,
                                0.0f,
                                MEASURED_ANGLE_AT_PHYSICAL_100_DEG);
  enterState(STATE_ASSIST);
  Serial.print("ASSIST_TRIGGER,target_deg=");
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
  if (!automaticStateActive()) return;
  if (!zeroed || !sensorsFresh()) {
    disableController("SENSOR_TIMEOUT_OR_NOT_ZEROED", true);
    return;
  }
  float measuredAngle = elbowAngleDegrees();
  if (!isfinite(measuredAngle) || measuredAngle > MAX_VALID_ANGLE_DEG) {
    disableController("INVALID_ANGLE", true);
    return;
  }
  float oldFilteredAngle = filteredAngle;
  filteredAngle += ANGLE_FILTER_ALPHA * (measuredAngle - filteredAngle);
  float controlPeriodSeconds = actualControlDt;
  float rawVelocity = (filteredAngle - oldFilteredAngle) / controlPeriodSeconds;
  filteredVelocity +=
      VELOCITY_FILTER_ALPHA * (rawVelocity - filteredVelocity);
  previousFilteredAngle = oldFilteredAngle;
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
  Serial.println("ZEROING,HOLD_ARM_STRAIGHT_MOTIONLESS_AND_CABLE_JUST_TAUT");
  Quaternion averagedNeutral;
  if (!averageNeutralQuaternion(averagedNeutral)) {
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
  Serial.println("AUTO_READY,SHADOW_ACTIVE");
}
void handleStableToggleChange(TogglePosition newPosition) {
  Serial.print("TOGGLE,");
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
      Serial.println("AUTO_BLOCKED,MOVE_SWITCH_TO_STOP_THEN_BACK_TO_AUTO");
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
    Serial.println("STOPPED,LEFT_AUTO_CONTACT");
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
// SERIAL DIAGNOSTICS (NOT REQUIRED FOR NORMAL OPERATION)
// ============================================================================
void printStatus() {
  Serial.print("STATUS,state=");
  Serial.print(stateName());
  Serial.print(",toggle=");
  Serial.print(toggleName(stableToggle));
  Serial.print(",zeroed=");
  Serial.print(zeroed);
  Serial.print(",sensors_fresh="); Serial.print(sensorsFresh());
  Serial.print(",upper_age_ms="); Serial.print(millis() - upperData.updatedMs);
  Serial.print(",forearm_age_ms="); Serial.print(millis() - forearmData.updatedMs);
  Serial.print(",angle_deg=");
  Serial.print(zeroed && sensorsFresh() ? elbowAngleDegrees() : -1.0f, 2);
  Serial.print(",filtered_deg=");
  Serial.print(filteredAngle, 2);
  Serial.print(",velocity_deg_s=");
  Serial.print(filteredVelocity, 2);
  Serial.print(",spool_steps=");
  Serial.print(motorPositionSteps);
  Serial.print(",target_steps=");
  Serial.print(commandedTargetSteps);
  Serial.print(",spool_rev=");
  Serial.print(spoolRevolutions(), 3);
  Serial.print(",cable_mm=");
  Serial.print(cableTravelMm(), 1);
  Serial.print(",reference_deg=");
  Serial.print(preDipReferenceAngle, 2);
  Serial.print(",assist_target_deg=");
  Serial.println(assistTargetAngle, 2);
}
void testAssist() {
  if (controllerState != STATE_SHADOW || stableToggle != TOGGLE_AUTO) {
    Serial.println("TEST_ASSIST_REJECTED,SHADOW_AND_AUTO_REQUIRED");
    return;
  }
  preDipReferenceAngle = constrain(filteredAngle + DIP_TRIGGER_DEG,
                                   0.0f,
                                   MEASURED_ANGLE_AT_PHYSICAL_100_DEG);
  beginAssist();
}
void handleSerialCommand(String command) {
  command.trim();
  command.toLowerCase();
  if (command == "record") {
    if (recording) { Serial.println("RECORD_ALREADY_ACTIVE"); return; }
    if (!zeroed || !sensorsFresh() || !automaticStateActive()) {
      Serial.println("RECORD_REJECTED,REQUIRE_FRESH_ZEROED_AUTO"); printStatus(); return;
    }
    recording = true; liveOutput = false;
    trialStartMs = millis(); lastLogMs = trialStartMs; skippedLogRows = 0; assistCount = 0;
    resetFatigueDetector();
    Serial.println("HEADER,time_ms,mode,state,zeroed,sensors_fresh,upper_age_ms,forearm_age_ms,upper_yaw_deg,upper_pitch_deg,upper_roll_deg,forearm_yaw_deg,forearm_pitch_deg,forearm_roll_deg,relative_rotation_deg,filtered_angle_deg,velocity_deg_s,reference_deg,dip_deg,dip_pending,assist_active,assist_target_deg,commanded_position_steps,emitted_position_steps,commanded_rate_limit_steps_s,driver_enabled,assist_count,skipped_log_rows");
  } else if (command == "end") {
    recording = false;
    Serial.println("RECORD_ENDED");
  } else if (command == "baseline" || command == "assisted") {
    if (stableToggle != TOGGLE_STOP || recording) {
      Serial.println("MODE_REJECTED,END_RECORDING_AND_SELECT_STOP"); return;
    }
    baselineMode = command == "baseline";
    setDriverEnabled(false);
    Serial.println(baselineMode ? "MODE,BASELINE" : "MODE,ASSISTED");
  } else if (command == "status") printStatus();
  else if (command == "live") {
    liveOutput = !liveOutput;
    Serial.println(liveOutput ? "LIVE_ON" : "LIVE_OFF");
  } else if (command == "testassist") {
    if (recording || baselineMode) Serial.println("TEST_ASSIST_REJECTED,DURING_RECORDING_OR_BASELINE");
    else testAssist();
  }
  else if (command == "stop") {
    autoTransitionAuthorized = false;
    zeroed = false;
    disableController("SERIAL_STOP");
    Serial.println("MOVE_SWITCH_TO_STOP_BEFORE_RESTART");
  } else if (command == "help") {
    Serial.println("COMMANDS:baseline,assisted,record,end,status,live,testassist,stop,help");
  } else if (command.length() > 0) {
    Serial.println("UNKNOWN_COMMAND,use_help");
  }
}
void serviceSerial() {
  while (Serial.available()) {
    char character = (char)Serial.read();
    if (character == '\n' || character == '\r') {
      if (serialLine.length() > 0) {
        handleSerialCommand(serialLine);
        serialLine = "";
      }
    } else if (serialLine.length() < 80) {
      serialLine += character;
    }
  }
}
void printLiveData() {
  if (recording || !liveOutput || millis() - lastLivePrintMs < 100) return;
  lastLivePrintMs = millis();
  Serial.print("LIVE,state=");
  Serial.print(stateName());
  Serial.print(",angle=");
  Serial.print(filteredAngle, 2);
  Serial.print(",velocity=");
  Serial.print(filteredVelocity, 2);
  Serial.print(",steps=");
  Serial.print(motorPositionSteps);
  Serial.print(",target=");
  Serial.print(commandedTargetSteps);
  Serial.print(",reference=");
  Serial.print(preDipReferenceAngle, 2);
  Serial.print(",assist_target=");
  Serial.println(assistTargetAngle, 2);
}
// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

// A complete row is queued only when sufficient TX capacity exists. No blocking
// Serial.flush() or delay is introduced into normal stepping/control.
void logCsv() {
  unsigned long now = millis();
  if (!recording || now - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = now;
  bool fresh = sensorsFresh();
  float angle = zeroed && fresh ? elbowAngleDegrees() : NAN;
  float filtered = zeroed && fresh ? filteredAngle : NAN;
  float velocity = zeroed && fresh ? filteredVelocity : NAN;
  char row[640];
  int n = snprintf(row, sizeof(row),
    "DATA,%lu,%s,%s,%d,%d,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%ld,%ld,%.1f,%d,%lu,%lu\n",
    now - trialStartMs, baselineMode ? "BASELINE" : "ASSISTED", stateName(),
    zeroed, fresh, now - upperData.updatedMs, now - forearmData.updatedMs,
    upperData.yaw, upperData.pitch, upperData.roll,
    forearmData.yaw, forearmData.pitch, forearmData.roll,
    angle, filtered, velocity, preDipReferenceAngle,
    zeroed && fresh ? preDipReferenceAngle - filteredAngle : NAN,
    dipStartMs != 0, controllerState == STATE_ASSIST, assistTargetAngle,
    commandedTargetSteps, motorPositionSteps, commandedMaxStepRate,
    digitalRead(ENABLE_PIN) == DRIVER_ENABLE_LEVEL, assistCount, skippedLogRows);
  if (n <= 0 || n >= (int)sizeof(row) || Serial.availableForWrite() < n) {
    skippedLogRows++; return;
  }
  Serial.write((const uint8_t*)row, n);
}

void setup() {
  Serial.setTxBufferSize(2048);
  Serial.begin(115200); // USB logging only; both IMU UARTs remain 115200.
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
  digitalWrite(MOTOR1_STEP_PIN, LOW);
  digitalWrite(MOTOR2_STEP_PIN, LOW);
  digitalWrite(MOTOR1_DIR_PIN, !MOTOR1_WIND_DIRECTION_LEVEL);
  digitalWrite(MOTOR2_DIR_PIN, !MOTOR2_WIND_DIRECTION_LEVEL);
  setDriverEnabled(false);
  serialLine.reserve(80);
  rawToggle = readTogglePosition();
  rawToggleChangedMs = millis();
  Serial.println("D33_SPOTTER_ASSIST_CONTROLLER");
  Serial.println("STARTUP:SELECT_STOP,PRETENSION_WITH_GPIO23,STRAIGHTEN_ARM,SELECT_AUTO");
  Serial.print("MAX_WIND_STEPS,");
  Serial.println(MAX_WIND_STEPS);
  Serial.print("SHADOW_SLACK_STEPS,");
  Serial.println(SHADOW_SLACK_STEPS);
  Serial.println("COMMANDS:baseline,assisted,record,end,status,live,testassist,stop,help");
}
void loop() {
  updateParser(upperIMU, upperParser, upperData);
  updateParser(forearmIMU, forearmParser, forearmData);
  updateToggle();
  serviceSerial();
  unsigned long nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
    actualControlDt = (nowUs - lastControlUs) / 1000000.0f;
    lastControlUs = nowUs;
    controlTick();
  }
  serviceAutomaticStepper();
  serviceManualWind();
  printLiveData();
  logCsv();
}
