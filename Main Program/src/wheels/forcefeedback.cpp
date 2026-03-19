#include "forcefeedback.h"

#include "hardware.h"

#include <cmath>
#include <stdint.h>

namespace {
constexpr uint8_t kForwardPwmPin = 9;
constexpr uint8_t kReversePwmPin = 10;
constexpr uint8_t kDriverEnablePin = 8;

constexpr int kMaxPwm = 255;
constexpr int kMinActivePwm = 45;
constexpr int kSlewPerUpdate = 9;
constexpr int kSlewToZeroPerUpdate = 36;
constexpr int kSlewOnDirectionFlipPerUpdate = 72;
constexpr float kTorqueDeadband = 0.002f;

bool g_outputEnabled = true;
int g_appliedPwm = 0;

int clampSignedPwm(int pwm) {
    if (pwm > kMaxPwm) {
        return kMaxPwm;
    }
    if (pwm < -kMaxPwm) {
        return -kMaxPwm;
    }
    return pwm;
}

bool haveOppositeSigns(int a, int b) {
    return ((a > 0 && b < 0) || (a < 0 && b > 0));
}

int slewToward(int current, int target, int maxStep) {
    if (maxStep <= 0) {
        return target;
    }
    if (target > current) {
        const int next = current + maxStep;
        return (next > target) ? target : next;
    }
    if (target < current) {
        const int next = current - maxStep;
        return (next < target) ? target : next;
    }
    return current;
}

void applySignedPwm(int signedPwm) {
    signedPwm = clampSignedPwm(signedPwm);
    if (signedPwm > 0) {
        platform::pwmWrite(kForwardPwmPin, static_cast<uint8_t>(signedPwm));
        platform::pwmWrite(kReversePwmPin, 0);
    } else if (signedPwm < 0) {
        platform::pwmWrite(kForwardPwmPin, 0);
        platform::pwmWrite(kReversePwmPin, static_cast<uint8_t>(-signedPwm));
    } else {
        platform::pwmWrite(kForwardPwmPin, 0);
        platform::pwmWrite(kReversePwmPin, 0);
    }
}

int torqueToPwm(float normalizedTorque) {
    if (normalizedTorque > 1.0f) {
        normalizedTorque = 1.0f;
    }
    if (normalizedTorque < -1.0f) {
        normalizedTorque = -1.0f;
    }
    if (std::fabs(normalizedTorque) <= kTorqueDeadband) {
        return 0;
    }

    int magnitude = static_cast<int>(std::round(std::fabs(normalizedTorque) * static_cast<float>(kMaxPwm)));
    if (magnitude > 0 && magnitude < kMinActivePwm) {
        magnitude = kMinActivePwm;
    }
    return (normalizedTorque >= 0.0f) ? magnitude : -magnitude;
}

void setDriverEnabled(bool enabled) {
    platform::digitalWrite(kDriverEnablePin, enabled ? platform::PinState::High : platform::PinState::Low);
}
}

void ForceFeedback_setup() {
    platform::pinMode(kForwardPwmPin, platform::PinMode::Output);
    platform::pinMode(kReversePwmPin, platform::PinMode::Output);
    platform::pinMode(kDriverEnablePin, platform::PinMode::Output);

    g_outputEnabled = true;
    g_appliedPwm = 0;
    applySignedPwm(0);
    setDriverEnabled(true);
}

void ForceFeedback_loop(float normalizedTorque) {
    if (!g_outputEnabled) {
        ForceFeedback_stop();
        return;
    }

    const int targetPwm = torqueToPwm(normalizedTorque);
    if (haveOppositeSigns(g_appliedPwm, targetPwm)) {
        g_appliedPwm = slewToward(g_appliedPwm, 0, kSlewOnDirectionFlipPerUpdate);
    } else if (targetPwm == 0) {
        g_appliedPwm = slewToward(g_appliedPwm, 0, kSlewToZeroPerUpdate);
    } else {
        g_appliedPwm = slewToward(g_appliedPwm, targetPwm, kSlewPerUpdate);
    }
    applySignedPwm(g_appliedPwm);
}

void ForceFeedback_stop() {
    g_appliedPwm = 0;
    applySignedPwm(0);
}

void ForceFeedback_setEnabled(bool enabled) {
    g_outputEnabled = enabled;
    if (!enabled) {
        ForceFeedback_stop();
    }
    setDriverEnabled(enabled);
}

int ForceFeedback_getAppliedPwm() {
    return g_appliedPwm;
}