#include "potentiometers.h"

#include "hardware.h"

#include <stdint.h>

namespace {
constexpr hardware::AnalogPin kThrottlePin = hardware::AnalogPin::A2;
constexpr hardware::AnalogPin kBrakePin = hardware::AnalogPin::A0;
constexpr float kFilterAlpha = 0.2f;

float gThrottle = 0.0f;
float gBrake = 0.0f;

float readTicks(hardware::AnalogPin pin) {
    int32_t raw = static_cast<int32_t>(hardware::readAdc(pin));
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 1023) {
        raw = 1023;
    }
    return static_cast<float>(raw);
}
}

namespace Potentiometers {

void init() {
    hardware::configureInput(kThrottlePin);
    hardware::configureInput(kBrakePin);
    hardware::initAdc();

    gThrottle = readTicks(kThrottlePin);
    gBrake = readTicks(kBrakePin);
}

void update() {
    const float t = readTicks(kThrottlePin);
    const float b = readTicks(kBrakePin);
    gThrottle += kFilterAlpha * (t - gThrottle);
    gBrake += kFilterAlpha * (b - gBrake);
}

void getValues(float& throttle, float& brake) {
    throttle = gThrottle;
    brake = gBrake;
}

}