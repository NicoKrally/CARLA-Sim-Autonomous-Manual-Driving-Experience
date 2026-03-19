#include "button_blinkers.h"

#include "hardware.h"

#include <stdint.h>

namespace {
constexpr uint8_t kPaddleCount = 2;
constexpr uint8_t kPaddlePins[kPaddleCount] = {19, 22};
constexpr uint8_t kLeftBlinkerPin = 11;
constexpr uint8_t kRightBlinkerPin = 13;
constexpr uint32_t kBlinkPeriodMs = 500;
constexpr uint8_t kBlinkBrightnessPercent = 20;
constexpr uint32_t kPaddleDebounceMs = 120;

bool g_wasPressed[kPaddleCount] = {false, false};
bool g_leftBlinkState = false;
bool g_rightBlinkState = false;
bool g_lastPublishedLeftState = false;
bool g_lastPublishedRightState = false;
bool g_hasPublishedBlinkState = false;
uint32_t g_lastBlinkMs = 0;
uint32_t g_lastPaddlePressMs = 0;

enum class ActiveBlinker : uint8_t {
    None = 0,
    Left,
    Right
};

ActiveBlinker g_activeBlinker = ActiveBlinker::None;

void setActiveBlinker(ActiveBlinker requested, uint32_t now) {
    if (g_activeBlinker == requested) {
        return;
    }

    g_activeBlinker = requested;
    if (requested == ActiveBlinker::Left) {
        g_leftBlinkState = true;
        g_rightBlinkState = false;
    } else if (requested == ActiveBlinker::Right) {
        g_leftBlinkState = false;
        g_rightBlinkState = true;
    } else {
        g_leftBlinkState = false;
        g_rightBlinkState = false;
    }
    g_lastBlinkMs = now;
}

void publishBlinkerState(bool force = false) {
    if (!force && g_hasPublishedBlinkState &&
        g_lastPublishedLeftState == g_leftBlinkState &&
        g_lastPublishedRightState == g_rightBlinkState) {
        return;
    }

    g_lastPublishedLeftState = g_leftBlinkState;
    g_lastPublishedRightState = g_rightBlinkState;
    g_hasPublishedBlinkState = true;

    char message[7];
    message[0] = 'B';
    message[1] = 'L';
    message[2] = ':';
    message[3] = g_leftBlinkState ? '1' : '0';
    message[4] = ',';
    message[5] = g_rightBlinkState ? '1' : '0';
    message[6] = '\0';
    platform::serialWriteLine(message);
}
}

void ButtonBlinkers_setup() {
    for (uint8_t i = 0; i < kPaddleCount; ++i) {
        platform::pinMode(kPaddlePins[i], platform::PinMode::InputPullup);
    }

    platform::pinMode(kLeftBlinkerPin, platform::PinMode::Output);
    platform::pinMode(kRightBlinkerPin, platform::PinMode::Output);
    platform::pwmWrite(kLeftBlinkerPin, 0);
    platform::pwmWrite(kRightBlinkerPin, 0);
    publishBlinkerState(true);
}

void ButtonBlinkers_applyHostCommand(char command) {
    const uint32_t now = platform::millis();
    if (command == 'L' || command == 'l') {
        setActiveBlinker(ActiveBlinker::Left, now);
    } else if (command == 'R' || command == 'r') {
        setActiveBlinker(ActiveBlinker::Right, now);
    } else if (command == '0') {
        setActiveBlinker(ActiveBlinker::None, now);
    }
}

void ButtonBlinkers_loop() {
    bool pressed[kPaddleCount];
    const uint32_t now = platform::millis();

    for (uint8_t i = 0; i < kPaddleCount; ++i) {
        pressed[i] = (platform::digitalRead(kPaddlePins[i]) == platform::PinState::Low);
        if (pressed[i] && !g_wasPressed[i]) {
            if ((now - g_lastPaddlePressMs) >= kPaddleDebounceMs) {
                g_lastPaddlePressMs = now;
                const ActiveBlinker requested = (i == 0U) ? ActiveBlinker::Left : ActiveBlinker::Right;

                if (g_activeBlinker == requested) {
                    setActiveBlinker(ActiveBlinker::None, now);
                } else {
                    setActiveBlinker(requested, now);
                }
            }
        }
        g_wasPressed[i] = pressed[i];
    }

    if ((g_activeBlinker != ActiveBlinker::None) && ((now - g_lastBlinkMs) >= kBlinkPeriodMs)) {
        g_lastBlinkMs = now;
        if (g_activeBlinker == ActiveBlinker::Left) {
            g_leftBlinkState = !g_leftBlinkState;
            g_rightBlinkState = false;
        } else {
            g_rightBlinkState = !g_rightBlinkState;
            g_leftBlinkState = false;
        }
    }

    const uint8_t pwmValue = static_cast<uint8_t>((255UL * static_cast<unsigned long>(kBlinkBrightnessPercent)) / 100UL);
    platform::pwmWrite(kLeftBlinkerPin, g_leftBlinkState ? pwmValue : 0);
    platform::pwmWrite(kRightBlinkerPin, g_rightBlinkState ? pwmValue : 0);
    publishBlinkerState();
}