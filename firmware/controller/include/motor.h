#pragma once

#include <Arduino.h>

void motorBegin();
void motorSetEnabled(bool enabled);
void motorSetTarget(long targetSteps, float maximumRate);
void motorServiceAutomatic(bool active, bool autoSelected, bool releasing);
void motorServiceManual(bool allowed, bool buttonPressed);
long motorPosition();
void motorResetPosition();
