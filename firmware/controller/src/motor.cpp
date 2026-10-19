#include "motor.h"

#include <math.h>

#include "config.h"
#include "pins.h"

namespace {
constexpr uint8_t MOTOR1_WIND_DIRECTION_LEVEL = HIGH;
constexpr uint8_t MOTOR2_WIND_DIRECTION_LEVEL = HIGH;
constexpr uint8_t DRIVER_ENABLE_LEVEL = LOW;
constexpr long PULSES_PER_REV = MOTOR_FULL_STEPS_PER_REV;
constexpr long MAX_WIND_STEPS = static_cast<long>((SAFE_CABLE_TRAVEL_MM / SPOOL_TRAVEL_PER_REV_MM) * PULSES_PER_REV);
constexpr long MIN_UNWIND_STEPS = -static_cast<long>((SHADOW_SLACK_MM / SPOOL_TRAVEL_PER_REV_MM) * PULSES_PER_REV);
long currentPosition = 0;
long targetPosition = 0;
float targetRate = SHADOW_MAX_STEP_RATE;
unsigned long lastStepUs = 0;

bool emitStep(bool wind) {
    if (wind && currentPosition >= MAX_WIND_STEPS) return false;
    if (!wind && currentPosition <= MIN_UNWIND_STEPS) return false;
    digitalWrite(MOTOR1_DIR_PIN, wind ? MOTOR1_WIND_DIRECTION_LEVEL : !MOTOR1_WIND_DIRECTION_LEVEL);
    digitalWrite(MOTOR2_DIR_PIN, wind ? MOTOR2_WIND_DIRECTION_LEVEL : !MOTOR2_WIND_DIRECTION_LEVEL);
    delayMicroseconds(1);
    digitalWrite(MOTOR1_STEP_PIN, HIGH);
    digitalWrite(MOTOR2_STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(MOTOR1_STEP_PIN, LOW);
    digitalWrite(MOTOR2_STEP_PIN, LOW);
    currentPosition += wind ? 1 : -1;
    return true;
}

void serviceStep(float maximumRate, bool allowed) {
    const long error = targetPosition - currentPosition;
    if (!allowed || labs(error) <= POSITION_DEADBAND_STEPS) return;
    const bool wind = error > 0;
    const float proportionalRate = KP_STEP_RATE * fabsf(static_cast<float>(error));
    const float stepRate = maximumRate < MIN_STEP_RATE ? maximumRate : constrain(proportionalRate, MIN_STEP_RATE, maximumRate);
    const unsigned long intervalUs = static_cast<unsigned long>(1000000.0f / stepRate);
    const unsigned long nowUs = micros();
    if (nowUs - lastStepUs >= intervalUs) {
        emitStep(wind);
        lastStepUs = nowUs;
    }
}
}

void motorBegin() {
    pinMode(MOTOR1_STEP_PIN, OUTPUT);
    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_STEP_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR1_STEP_PIN, LOW);
    digitalWrite(MOTOR2_STEP_PIN, LOW);
    digitalWrite(MOTOR1_DIR_PIN, !MOTOR1_WIND_DIRECTION_LEVEL);
    digitalWrite(MOTOR2_DIR_PIN, !MOTOR2_WIND_DIRECTION_LEVEL);
    motorSetEnabled(false);
}

void motorSetEnabled(bool enabled) {
    digitalWrite(MOTOR_ENABLE_PIN, enabled ? DRIVER_ENABLE_LEVEL : !DRIVER_ENABLE_LEVEL);
}

void motorSetTarget(long targetSteps, float maximumRate) {
    targetPosition = targetSteps;
    targetRate = maximumRate;
}

void motorServiceAutomatic(bool active, bool autoSelected, bool releasing) {
    if (!active || !autoSelected) return;
    const float maximumRate = releasing && targetPosition < currentPosition ? RELEASE_UNWIND_STEP_RATE : targetRate;
    serviceStep(maximumRate, true);
}

void motorServiceManual(bool allowed, bool buttonPressed) {
    if (!allowed || !buttonPressed) {
        if (!allowed) motorSetEnabled(false);
        return;
    }
    motorSetEnabled(true);
    targetPosition = MAX_WIND_STEPS;
    serviceStep(MANUAL_WIND_STEP_RATE, true);
}

long motorPosition() { return currentPosition; }

void motorResetPosition() {
    currentPosition = 0;
    targetPosition = 0;
}
