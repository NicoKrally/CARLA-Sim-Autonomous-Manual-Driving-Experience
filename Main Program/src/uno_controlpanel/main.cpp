#include "hardware.h"

#include <stdint.h>
#include <stdlib.h>

namespace {
constexpr uint8_t kButtonCount = 4;
constexpr hardware::Pin kButtonPins[kButtonCount] = {
    hardware::Pin::D1,
    hardware::Pin::D2,
    hardware::Pin::D3,
    hardware::Pin::D4
};
constexpr uint8_t kButtonEventIds[kButtonCount] = {5, 0, 2, 3};
constexpr hardware::Pin kJoystickVrxPin = hardware::Pin::A0;
constexpr hardware::Pin kJoystickVryPin = hardware::Pin::A1;
constexpr hardware::Pin kJoystickSwPin = hardware::Pin::D0;
constexpr hardware::Pin kStatusLedPin = hardware::Pin::D13;
constexpr uint32_t kDebounceMs = 20;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kJoystickPublishIntervalMs = 50;
constexpr int kJoystickDeltaThreshold = 4;

bool g_rawPressed[kButtonCount] = {false, false, false, false};
bool g_stablePressed[kButtonCount] = {false, false, false, false};
uint32_t g_lastRawChangeMs[kButtonCount] = {0, 0, 0, 0};

int g_lastJoystickVrx = -1;
int g_lastJoystickVry = -1;
bool g_lastJoystickSwPressed = false;
uint32_t g_lastJoystickPublishMs = 0;

void beginSerialBurst() {
    hardware::uartEnableTx(kSerialBaud);
}

void endSerialBurst() {
    hardware::uartFlushAndDisable();
    hardware::configureInputPullup(kButtonPins[0]);
    hardware::configureInputPullup(kJoystickSwPin);
}

void updateStatusLed() {
    bool anyPressed = false;
    for (uint8_t i = 0; i < kButtonCount; ++i) {
        if (g_stablePressed[i]) {
            anyPressed = true;
            break;
        }
    }
    hardware::writeDigital(kStatusLedPin, anyPressed);
}

void publishButtonPress(uint8_t buttonIndex) {
    if (buttonIndex >= kButtonCount) {
        return;
    }

    beginSerialBurst();
    hardware::uartWrite("A");
    hardware::uartWriteInt32(static_cast<int32_t>(kButtonEventIds[buttonIndex]));
    hardware::uartWrite("\r\n");
    endSerialBurst();
}

void publishJoystickState(int vrx, int vry, bool swPressed) {
    beginSerialBurst();
    hardware::uartWrite("JOY:");
    hardware::uartWriteInt32(static_cast<int32_t>(vrx));
    hardware::uartWrite(",");
    hardware::uartWriteInt32(static_cast<int32_t>(vry));
    hardware::uartWrite(",");
    hardware::uartWriteInt32(swPressed ? 1 : 0);
    hardware::uartWrite("\r\n");
    endSerialBurst();
}

bool isPressed(hardware::Pin pin) {
    return !hardware::readDigital(pin);
}
}

int main() {
    hardware::initSystemTimer();
    hardware::initAdc();

    for (uint8_t i = 0; i < kButtonCount; ++i) {
        hardware::configureInputPullup(kButtonPins[i]);
    }
    hardware::configureInputPullup(kJoystickSwPin);

    hardware::configureOutput(kStatusLedPin);
    hardware::writeDigital(kStatusLedPin, false);

    hardware::enableInterrupts();

    while (true) {
        const uint32_t now = hardware::millis();
        bool changed = false;

        for (uint8_t i = 0; i < kButtonCount; ++i) {
            const bool rawPressed = isPressed(kButtonPins[i]);

            if (rawPressed != g_rawPressed[i]) {
                g_rawPressed[i] = rawPressed;
                g_lastRawChangeMs[i] = now;
            }

            if ((now - g_lastRawChangeMs[i]) >= kDebounceMs && g_stablePressed[i] != g_rawPressed[i]) {
                g_stablePressed[i] = g_rawPressed[i];
                changed = true;
                if (!g_stablePressed[i]) {
                    publishButtonPress(i);
                }
            }
        }

        if (changed) {
            updateStatusLed();
        }

        const int joystickVrx = static_cast<int>(hardware::readAdc(kJoystickVrxPin));
        const int joystickVry = static_cast<int>(hardware::readAdc(kJoystickVryPin));
        const bool joystickSwPressed = isPressed(kJoystickSwPin);

        const bool joystickChanged =
            (g_lastJoystickVrx < 0) ||
            (abs(joystickVrx - g_lastJoystickVrx) >= kJoystickDeltaThreshold) ||
            (abs(joystickVry - g_lastJoystickVry) >= kJoystickDeltaThreshold) ||
            (joystickSwPressed != g_lastJoystickSwPressed);

        const bool publishIntervalElapsed =
            (g_lastJoystickPublishMs == 0U) || ((now - g_lastJoystickPublishMs) >= kJoystickPublishIntervalMs);

        g_lastJoystickVrx = joystickVrx;
        g_lastJoystickVry = joystickVry;
        g_lastJoystickSwPressed = joystickSwPressed;

        if ((joystickChanged || publishIntervalElapsed) && !g_rawPressed[0] && !g_stablePressed[0]) {
            publishJoystickState(joystickVrx, joystickVry, joystickSwPressed);
            g_lastJoystickPublishMs = now;
        }
    }
}