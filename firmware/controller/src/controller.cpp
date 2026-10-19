#include "controller.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "imu.h"
#include "motor.h"
#include "pins.h"

namespace {

enum class State { Disabled, Calibrating, Shadow, Assist, Release, Fault };
enum class Toggle { Unknown, Auto, Stop, Between, Invalid };

constexpr long PULSES_PER_REV = MOTOR_FULL_STEPS_PER_REV;
const long maxWindSteps = lroundf((SAFE_CABLE_TRAVEL_MM / SPOOL_TRAVEL_PER_REV_MM) * PULSES_PER_REV);
const long shadowSlackSteps = lroundf((SHADOW_SLACK_MM / SPOOL_TRAVEL_PER_REV_MM) * PULSES_PER_REV);
const long minUnwindSteps = -shadowSlackSteps;
const float stepsPerMeasuredDegree = static_cast<float>(maxWindSteps) / MEASURED_ANGLE_AT_PHYSICAL_100_DEG;

State state = State::Disabled;
Toggle rawToggle = Toggle::Unknown;
Toggle stableToggle = Toggle::Unknown;
Quaternion neutralRelative = {1, 0, 0, 0};
bool stopSeenSinceBoot = false;
bool autoTransitionAuthorized = false;
bool zeroed = false;
bool sensingOnly = false;
bool trialLogging = false;
bool syncPulseActive = false;
unsigned long rawToggleChangedMs = 0;
unsigned long lastControlUs = 0;
unsigned long stateEnteredMs = 0;
unsigned long elevatedStartMs = 0;
unsigned long dipStartMs = 0;
unsigned long recoveryStartMs = 0;
unsigned long releaseSettledStartMs = 0;
unsigned long assistCooldownUntilMs = 0;
unsigned long trialStartUs = 0;
unsigned long lastLogUs = 0;
unsigned long syncPulseStartedMs = 0;
unsigned long lastRateWindowMs = 0;
uint32_t previousUpperPacketCount = 0;
uint32_t previousForearmPacketCount = 0;
uint32_t controlTicksThisWindow = 0;
uint32_t droppedLogRows = 0;
float measuredUpperPacketHz = 0;
float measuredForearmPacketHz = 0;
float measuredControlHz = 0;
float filteredAngle = 0;
float filteredVelocity = 0;
float preDipReferenceAngle = 0;
float assistTargetAngle = 0;
long commandedTargetSteps = 0;
float commandedMaxStepRate = SHADOW_MAX_STEP_RATE;
String serialLine;
String sessionId = "UNSET";
String trialId = "UNSET";
String trialCondition = "UNSET";
int trialRep = 0;
float trialLoadKg = NAN;

const char *stateName() {
    if (sensingOnly) return "MOTOR_OFF";
    switch (state) {
        case State::Calibrating: return "CALIBRATING";
        case State::Shadow: return "FOLLOW";
        case State::Assist: return "ASSIST";
        case State::Release: return "RELEASE";
        case State::Fault: return "FAULT";
        default: return "DISABLED";
    }
}

const char *toggleName(Toggle value) {
    switch (value) {
        case Toggle::Auto: return "AUTO";
        case Toggle::Stop: return "STOP";
        case Toggle::Between: return "BETWEEN";
        case Toggle::Invalid: return "INVALID";
        default: return "UNKNOWN";
    }
}

bool automaticActive() {
    return state == State::Shadow || state == State::Assist || state == State::Release;
}

Toggle readToggle() {
    const bool autoSelected = digitalRead(AUTO_SWITCH_PIN) == LOW;
    const bool stopSelected = digitalRead(STOP_SWITCH_PIN) == LOW;
    if (autoSelected && !stopSelected) return Toggle::Auto;
    if (stopSelected && !autoSelected) return Toggle::Stop;
    if (!autoSelected && !stopSelected) return Toggle::Between;
    return Toggle::Invalid;
}

void resetFatigueDetector() {
    elevatedStartMs = 0;
    dipStartMs = 0;
    recoveryStartMs = 0;
    preDipReferenceAngle = 0;
}

void enterState(State next) {
    state = next;
    stateEnteredMs = millis();
    releaseSettledStartMs = 0;
    recoveryStartMs = 0;
    Serial.print("#EVENT,STATE,");
    Serial.println(stateName());
}

void disableController(const char *reason, bool fault = false) {
    const bool wasActive = automaticActive();
    state = fault ? State::Fault : State::Disabled;
    sensingOnly = false;
    commandedTargetSteps = motorPosition();
    resetFatigueDetector();
    motorSetEnabled(false);
    if (wasActive || fault) {
        Serial.print(fault ? "#EVENT,FAULT," : "#EVENT,STOPPED,");
        Serial.println(reason);
    }
}

long tautStepsForAngle(float angle) {
    return constrain(lroundf(angle * stepsPerMeasuredDegree), 0L, maxWindSteps);
}

long shadowStepsForAngle(float angle) {
    return constrain(tautStepsForAngle(angle) - shadowSlackSteps, minUnwindSteps, maxWindSteps);
}

void beginRelease(const char *reason) {
    Serial.print("#EVENT,ASSIST_END,");
    Serial.println(reason);
    enterState(State::Release);
    resetFatigueDetector();
}

void beginAssist() {
    const float lostAngle = preDipReferenceAngle - filteredAngle;
    const float correction = constrain(lostAngle, 0.0f, MAX_ASSIST_CORRECTION_DEG);
    assistTargetAngle = constrain(filteredAngle + correction, 0.0f, MEASURED_ANGLE_AT_PHYSICAL_100_DEG);
    enterState(State::Assist);
    Serial.print("#EVENT,ASSIST_TRIGGER,target_deg=");
    Serial.print(assistTargetAngle, 2);
    Serial.print(",dip_deg=");
    Serial.println(lostAngle, 2);
}

void updateShadow(unsigned long nowMs) {
    commandedTargetSteps = shadowStepsForAngle(filteredAngle);
    commandedMaxStepRate = SHADOW_MAX_STEP_RATE;
    if (static_cast<long>(nowMs - assistCooldownUntilMs) < 0) return;
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
    if (filteredVelocity <= INTENTIONAL_LOWERING_SPEED_DEG_S) {
        elevatedStartMs = nowMs;
        preDipReferenceAngle = filteredAngle;
        dipStartMs = 0;
        return;
    }
    if (nowMs - elevatedStartMs < MIN_ELEVATED_TIME_MS ||
        preDipReferenceAngle - filteredAngle < DIP_TRIGGER_DEG) {
        dipStartMs = 0;
        return;
    }
    if (dipStartMs == 0) dipStartMs = nowMs;
    if (nowMs - dipStartMs >= DIP_CONFIRM_MS) beginAssist();
}

void updateAssist(unsigned long nowMs) {
    commandedTargetSteps = tautStepsForAngle(assistTargetAngle);
    commandedMaxStepRate = ASSIST_MAX_STEP_RATE;
    if (filteredAngle <= ASSIST_CANCEL_ANGLE_DEG) {
        beginRelease("ARM_LOWERED");
        return;
    }
    const bool recovered = filteredAngle >= assistTargetAngle - ASSIST_RECOVERY_BAND_DEG;
    const bool rising = filteredVelocity >= RECOVERY_VELOCITY_DEG_S;
    if (recovered || rising) {
        if (recoveryStartMs == 0) recoveryStartMs = nowMs;
        if (nowMs - recoveryStartMs >= RECOVERY_CONFIRM_MS) {
            beginRelease(recovered ? "ANGLE_RECOVERED" : "UPWARD_MOTION");
            return;
        }
    } else {
        recoveryStartMs = 0;
    }
    if (nowMs - stateEnteredMs >= MAX_ASSIST_TIME_MS) beginRelease("ASSIST_TIMEOUT");
}

void updateRelease(unsigned long nowMs) {
    commandedTargetSteps = shadowStepsForAngle(filteredAngle);
    commandedMaxStepRate = SHADOW_MAX_STEP_RATE;
    if (labs(commandedTargetSteps - motorPosition()) <= POSITION_DEADBAND_STEPS) {
        if (releaseSettledStartMs == 0) releaseSettledStartMs = nowMs;
        if (nowMs - releaseSettledStartMs >= RELEASE_SETTLE_MS) {
            assistCooldownUntilMs = nowMs + ASSIST_COOLDOWN_MS;
            enterState(State::Shadow);
            resetFatigueDetector();
        }
    } else {
        releaseSettledStartMs = 0;
    }
}

void controlTick() {
    if (!automaticActive() && !sensingOnly) return;
    if (!zeroed || !imuSensorsFresh()) {
        if (automaticActive()) disableController("SENSOR_TIMEOUT_OR_NOT_ZEROED", true);
        return;
    }
    const float measuredAngle = imuElbowAngleDegrees();
    if (!isfinite(measuredAngle) || measuredAngle > MAX_ARM_ANGLE_DEG) {
        disableController("INVALID_ANGLE", true);
        return;
    }
    const float oldAngle = filteredAngle;
    filteredAngle += ANGLE_FILTER_ALPHA * (measuredAngle - filteredAngle);
    const float rawVelocity = (filteredAngle - oldAngle) / (CONTROL_PERIOD_US / 1000000.0f);
    filteredVelocity += VELOCITY_FILTER_ALPHA * (rawVelocity - filteredVelocity);
    if (sensingOnly) return;
    const unsigned long nowMs = millis();
    if (state == State::Shadow) updateShadow(nowMs);
    else if (state == State::Assist) updateAssist(nowMs);
    else if (state == State::Release) updateRelease(nowMs);
}

void startAutomaticController() {
    autoTransitionAuthorized = false;
    motorSetEnabled(false);
    enterState(State::Calibrating);
    Serial.println("#EVENT,ZEROING,HOLD_ARM_STRAIGHT_MOTIONLESS_AND_CABLE_JUST_TAUT");
    Quaternion averagedNeutral;
    if (!imuAverageNeutralQuaternion(averagedNeutral, true)) {
        zeroed = false;
        disableController("ZERO_FAILED_OR_AUTO_CANCELLED", true);
        return;
    }
    if (readToggle() != Toggle::Auto) {
        zeroed = false;
        disableController("AUTO_LEFT_DURING_ZERO", true);
        return;
    }
    neutralRelative = averagedNeutral;
    zeroed = true;
    motorResetPosition();
    commandedTargetSteps = 0;
    filteredAngle = 0;
    filteredVelocity = 0;
    assistTargetAngle = 0;
    assistCooldownUntilMs = millis() + ASSIST_COOLDOWN_MS;
    resetFatigueDetector();
    lastControlUs = micros();
    motorSetEnabled(true);
    enterState(State::Shadow);
    Serial.println("#EVENT,AUTO_READY,FOLLOW_ACTIVE");
}

void startSensingOnly() {
    if (stableToggle != Toggle::Stop) {
        Serial.println("#ERROR,SENSE_ONLY_REQUIRES_STOP_SWITCH");
        return;
    }
    motorSetEnabled(false);
    state = State::Disabled;
    sensingOnly = true;
    zeroed = false;
    Serial.println("#EVENT,SENSE_ZEROING,HOLD_ARM_STRAIGHT_AND_MOTIONLESS");
    Quaternion averagedNeutral;
    if (!imuAverageNeutralQuaternion(averagedNeutral, false)) {
        sensingOnly = false;
        Serial.println("#ERROR,SENSE_ZERO_FAILED");
        return;
    }
    neutralRelative = averagedNeutral;
    zeroed = true;
    filteredAngle = 0;
    filteredVelocity = 0;
    motorResetPosition();
    commandedTargetSteps = 0;
    stateEnteredMs = millis();
    lastControlUs = micros();
    Serial.println("#EVENT,SENSE_ONLY_READY,MOTORS_DISABLED");
}

void stopSensingOnly() {
    sensingOnly = false;
    zeroed = false;
    motorSetEnabled(false);
    Serial.println("#EVENT,SENSE_ONLY_STOPPED");
}

void handleToggle(Toggle next) {
    Serial.print("#EVENT,TOGGLE,");
    Serial.println(toggleName(next));
    if (next == Toggle::Stop) {
        stopSeenSinceBoot = true;
        autoTransitionAuthorized = true;
        zeroed = false;
        disableController("TOGGLE_STOP");
    } else if (next == Toggle::Auto) {
        if (!stopSeenSinceBoot || !autoTransitionAuthorized) {
            disableController("MOVE_TO_STOP_BEFORE_AUTO");
            Serial.println("#ERROR,AUTO_BLOCKED,MOVE_SWITCH_TO_STOP_THEN_BACK_TO_AUTO");
        } else {
            startAutomaticController();
        }
    } else {
        disableController(next == Toggle::Invalid ? "INVALID_TOGGLE_STATE" : "TOGGLE_BETWEEN");
    }
}

void updateToggle() {
    const Toggle currentRaw = readToggle();
    if (automaticActive() && currentRaw != Toggle::Auto) {
        motorSetEnabled(false);
        state = State::Disabled;
        zeroed = false;
        autoTransitionAuthorized = false;
        Serial.println("#EVENT,STOPPED,LEFT_AUTO_CONTACT");
    }
    if (currentRaw != rawToggle) {
        rawToggle = currentRaw;
        rawToggleChangedMs = millis();
    }
    if (currentRaw != stableToggle && millis() - rawToggleChangedMs >= SWITCH_DEBOUNCE_MS) {
        stableToggle = currentRaw;
        handleToggle(stableToggle);
    }
}

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

void printHeader() {
    Serial.println("record_type,time_s,session_id,trial_id,condition,rep,load_kg,upper_yaw_deg,upper_pitch_deg,upper_roll_deg,forearm_yaw_deg,forearm_pitch_deg,forearm_roll_deg,raw_elbow_angle_deg,filtered_angle_deg,angular_velocity_deg_s,controller_state,toggle,zeroed,sensors_fresh,fault_flag,motor_position_steps,target_steps,step_error_steps,cable_travel_mm,reference_angle_deg,assist_target_deg,state_elapsed_ms,upper_age_ms,forearm_age_ms,upper_packet_hz,forearm_packet_hz,control_hz,upper_bad_packets,forearm_bad_packets,sync_out,dropped_log_rows");
}

void updateRates() {
    const unsigned long now = millis();
    if (lastRateWindowMs == 0) {
        lastRateWindowMs = now;
        previousUpperPacketCount = imuUpperData().packetCount;
        previousForearmPacketCount = imuForearmData().packetCount;
        return;
    }
    const unsigned long elapsed = now - lastRateWindowMs;
    if (elapsed < RATE_WINDOW_MS) return;
    measuredUpperPacketHz = (imuUpperData().packetCount - previousUpperPacketCount) * 1000.0f / elapsed;
    measuredForearmPacketHz = (imuForearmData().packetCount - previousForearmPacketCount) * 1000.0f / elapsed;
    measuredControlHz = controlTicksThisWindow * 1000.0f / elapsed;
    previousUpperPacketCount = imuUpperData().packetCount;
    previousForearmPacketCount = imuForearmData().packetCount;
    controlTicksThisWindow = 0;
    lastRateWindowMs = now;
}

void printDataRow() {
    const unsigned long now = millis();
    const ImuData &upper = imuUpperData();
    const ImuData &forearm = imuForearmData();
    const bool fresh = imuSensorsFresh();
    const float rawAngle = zeroed && fresh ? imuElbowAngleDegrees() : NAN;
    const auto age = [now](const ImuData &data) { return data.valid ? now - data.updatedMs : 999999UL; };
    Serial.print("DATA,"); Serial.print((micros() - trialStartUs) / 1000000.0f, 4); Serial.print(',');
    Serial.print(sessionId); Serial.print(','); Serial.print(trialId); Serial.print(',');
    Serial.print(trialCondition); Serial.print(','); Serial.print(trialRep); Serial.print(',');
    printCsvFloat(trialLoadKg, 3); Serial.print(',');
    printCsvFloat(upper.yaw, 2); Serial.print(','); printCsvFloat(upper.pitch, 2); Serial.print(','); printCsvFloat(upper.roll, 2); Serial.print(',');
    printCsvFloat(forearm.yaw, 2); Serial.print(','); printCsvFloat(forearm.pitch, 2); Serial.print(','); printCsvFloat(forearm.roll, 2); Serial.print(',');
    printCsvFloat(rawAngle, 2); Serial.print(','); printCsvFloat(zeroed ? filteredAngle : NAN, 2); Serial.print(','); printCsvFloat(zeroed ? filteredVelocity : NAN, 2); Serial.print(',');
    Serial.print(stateName()); Serial.print(','); Serial.print(toggleName(stableToggle)); Serial.print(','); Serial.print(zeroed); Serial.print(','); Serial.print(fresh); Serial.print(','); Serial.print(state == State::Fault); Serial.print(',');
    Serial.print(motorPosition()); Serial.print(','); Serial.print(commandedTargetSteps); Serial.print(','); Serial.print(commandedTargetSteps - motorPosition()); Serial.print(',');
    Serial.print(motorPosition() * SPOOL_TRAVEL_PER_REV_MM / PULSES_PER_REV, 2); Serial.print(','); Serial.print(preDipReferenceAngle, 2); Serial.print(','); Serial.print(assistTargetAngle, 2); Serial.print(','); Serial.print(now - stateEnteredMs); Serial.print(',');
    Serial.print(age(upper)); Serial.print(','); Serial.print(age(forearm)); Serial.print(','); Serial.print(measuredUpperPacketHz, 1); Serial.print(','); Serial.print(measuredForearmPacketHz, 1); Serial.print(','); Serial.print(measuredControlHz, 1); Serial.print(','); Serial.print(upper.badPacketCount); Serial.print(','); Serial.print(forearm.badPacketCount); Serial.print(','); Serial.print(syncPulseActive); Serial.print(','); Serial.println(droppedLogRows);
}

void serviceLogger() {
    if (!trialLogging || micros() - lastLogUs < LOG_PERIOD_US) return;
    const unsigned long elapsed = micros() - lastLogUs;
    if (elapsed >= 3UL * LOG_PERIOD_US) {
        droppedLogRows += elapsed / LOG_PERIOD_US - 1;
        lastLogUs = micros();
    } else {
        lastLogUs += LOG_PERIOD_US;
    }
    printDataRow();
}

void configureTrial(String payload) {
    const int first = payload.indexOf(','), second = payload.indexOf(',', first + 1), third = payload.indexOf(',', second + 1);
    if (first < 1 || second < 0 || third < 0) {
        Serial.println("#ERROR,USE_trial_TRIALID_CONDITION_REP_LOADKG");
        return;
    }
    trialId = csvSafe(payload.substring(0, first));
    String condition = csvSafe(payload.substring(first + 1, second));
    condition.toUpperCase();
    if (condition == "MOTOR_OFF") trialCondition = "Device Motor Off";
    else if (condition == "ASSISTED") trialCondition = "Closed-Loop Assisted";
    else if (condition == "NO_DEVICE") trialCondition = "No Device (optional)";
    else {
        Serial.println("#ERROR,CONDITION_MUST_BE_MOTOR_OFF_ASSISTED_OR_NO_DEVICE");
        return;
    }
    trialRep = payload.substring(second + 1, third).toInt();
    trialLoadKg = payload.substring(third + 1).toFloat();
    Serial.print("#TRIAL_CONFIG,"); Serial.print(trialId); Serial.print(','); Serial.print(trialCondition); Serial.print(','); Serial.print(trialRep); Serial.print(','); Serial.println(trialLoadKg, 3);
}

void startTrial() {
    if (sessionId == "UNSET" || trialId == "UNSET" || trialCondition == "UNSET" || trialRep <= 0) {
        Serial.println("#ERROR,CONFIGURE_SESSION_AND_TRIAL_BEFORE_START"); return;
    }
    if (!zeroed || !imuSensorsFresh()) {
        Serial.println("#ERROR,ZERO_AND_VERIFY_IMUS_BEFORE_START"); return;
    }
    if (trialCondition == "Closed-Loop Assisted" && !automaticActive()) {
        Serial.println("#ERROR,ASSISTED_TRIAL_REQUIRES_AUTO_FOLLOW_STATE"); return;
    }
    if ((trialCondition == "Device Motor Off" || trialCondition == "No Device (optional)") && !sensingOnly) {
        Serial.println("#ERROR,MOTOR_OFF_OR_NO_DEVICE_REQUIRES_SENSEONLY"); return;
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

void serviceSerialCommand(String command) {
    command.trim();
    String lower = command;
    lower.toLowerCase();
    if (lower.startsWith("session,")) { sessionId = csvSafe(command.substring(8)); Serial.print("#SESSION,"); Serial.println(sessionId); }
    else if (lower.startsWith("trial,")) configureTrial(command.substring(6));
    else if (lower.startsWith("mark,")) { Serial.print("#MARK,"); Serial.print(trialLogging ? (micros() - trialStartUs) / 1000000.0f : -1.0f, 4); Serial.print(','); Serial.println(csvSafe(command.substring(5))); }
    else if (lower == "start") startTrial();
    else if (lower == "end") { trialLogging = false; digitalWrite(SYNC_OUT_PIN, LOW); syncPulseActive = false; Serial.print("#EVENT,TRIAL_END,dropped_rows="); Serial.println(droppedLogRows); }
    else if (lower == "senseonly") startSensingOnly();
    else if (lower == "senseoff") stopSensingOnly();
    else if (lower == "header") printHeader();
    else if (lower == "rates") { Serial.print("#RATES,upper_hz="); Serial.print(measuredUpperPacketHz, 1); Serial.print(",forearm_hz="); Serial.print(measuredForearmPacketHz, 1); Serial.print(",control_hz="); Serial.println(measuredControlHz, 1); }
    else if (lower == "status") { Serial.print("#STATUS,state="); Serial.print(stateName()); Serial.print(",toggle="); Serial.print(toggleName(stableToggle)); Serial.print(",zeroed="); Serial.print(zeroed); Serial.print(",logging="); Serial.println(trialLogging); }
    else if (lower == "testassist" && state == State::Shadow && stableToggle == Toggle::Auto) { preDipReferenceAngle = filteredAngle + DIP_TRIGGER_DEG; beginAssist(); }
    else if (lower == "stop") { trialLogging = false; autoTransitionAuthorized = false; zeroed = false; disableController("SERIAL_STOP"); Serial.println("#ACTION,MOVE_SWITCH_TO_STOP_BEFORE_RESTART"); }
    else if (lower == "help") { Serial.println("#COMMAND,session_ID"); Serial.println("#COMMAND,trial_TRIALID_CONDITION_REP_LOADKG"); Serial.println("#COMMAND,senseonly,senseoff,start,end,mark_LABEL,header,rates,status,testassist,stop,help"); }
    else if (command.length() > 0) Serial.println("#ERROR,UNKNOWN_COMMAND,use_help");
}

void serviceSerial() {
    while (Serial.available()) {
        const char character = static_cast<char>(Serial.read());
        if (character == '\n' || character == '\r') {
            if (serialLine.length() > 0) { serviceSerialCommand(serialLine); serialLine = ""; }
        } else if (serialLine.length() < 160) serialLine += character;
    }
}

}

void setupController() {
    Serial.begin(SERIAL_BAUD);
    pinMode(STOP_SWITCH_PIN, INPUT_PULLUP);
    pinMode(AUTO_SWITCH_PIN, INPUT_PULLUP);
    pinMode(MANUAL_WIND_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SYNC_OUT_PIN, OUTPUT);
    digitalWrite(SYNC_OUT_PIN, LOW);
    imuBegin();
    motorBegin();
    rawToggle = readToggle();
    rawToggleChangedMs = millis();
    serialLine.reserve(160);
    Serial.println("#D33_SPOTTER_ASSIST_TRIAL_LOGGER");
    Serial.println("#STARTUP,SELECT_STOP_PRETENSION_STRAIGHTEN_THEN_SELECT_AUTO");
    Serial.print("#MAX_WIND_STEPS,"); Serial.println(maxWindSteps);
    Serial.print("#FOLLOW_SLACK_STEPS,"); Serial.println(shadowSlackSteps);
    Serial.print("#USB_SERIAL_BAUD,"); Serial.println(SERIAL_BAUD);
    printHeader();
}

void loopController() {
    imuUpdate();
    updateRates();
    if (syncPulseActive && millis() - syncPulseStartedMs >= SYNC_PULSE_MS) {
        digitalWrite(SYNC_OUT_PIN, LOW);
        syncPulseActive = false;
    }
    updateToggle();
    serviceSerial();
    const unsigned long nowUs = micros();
    if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
        lastControlUs = nowUs;
        ++controlTicksThisWindow;
        controlTick();
    }
    motorServiceAutomatic(automaticActive(), stableToggle == Toggle::Auto, state == State::Release);
    motorServiceManual(stableToggle == Toggle::Stop && state == State::Disabled && !sensingOnly,
                       digitalRead(MANUAL_WIND_BUTTON_PIN) == LOW);
    serviceLogger();
}
