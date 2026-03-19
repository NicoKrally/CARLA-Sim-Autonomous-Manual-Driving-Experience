#include "button_blinkers.h"
#include "forcefeedback.h"
#include "hardware.h"
#include "rotaryencoder.h"

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr uint32_t kSerialBaud = 250000;
constexpr uint32_t kControlPeriodUs = 4000;

int g_lastPwm = 0;

int clampInt(int value, int minValue, int maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

void trimLine(char* text) {
    size_t len = strlen(text);
    size_t start = 0;
    while (start < len && isspace(static_cast<unsigned char>(text[start])) != 0) {
        ++start;
    }

    size_t end = len;
    while (end > start && isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
        --end;
    }

    const size_t outLen = end - start;
    if (start > 0 && outLen > 0) {
        memmove(text, text + start, outLen);
    }
    text[outLen] = '\0';
}

void handleLine(char* line) {
    trimLine(line);
    if (line[0] == '\0') {
        return;
    }

    if (strncmp(line, "BLCMD:", 6) == 0 && line[6] != '\0') {
        ButtonBlinkers_applyHostCommand(line[6]);
        return;
    }

    const int pwm = static_cast<int>(strtol(line, nullptr, 10));
    g_lastPwm = clampInt(pwm, -255, 255);
}

void setup() {
    platform::serialInit(kSerialBaud);
    RotaryEncoder_setup();
    ForceFeedback_setup();
    ButtonBlinkers_setup();
    platform::enableInterrupts();
    platform::serialWriteLine("wheel ready");
}

void loop() {
    static uint32_t lastControlUs = 0;
    const uint32_t nowUs = platform::micros();
    if (lastControlUs != 0U && (nowUs - lastControlUs) < kControlPeriodUs) {
        return;
    }
    lastControlUs = nowUs;

    const long ticks = RotaryEncoder_getTicks();
    platform::serialWriteInt32(static_cast<int32_t>(ticks));

    static char line[32];
    static uint8_t length = 0;

    char c = '\0';
    while (platform::serialReadChar(c)) {
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            if (length > 0U) {
                line[length] = '\0';
                handleLine(line);
                length = 0;
            }
        } else if (length + 1U < sizeof(line)) {
            line[length++] = c;
        }
    }

    ForceFeedback_loop(static_cast<float>(g_lastPwm) / 255.0f);
    ButtonBlinkers_loop();
}

}

int main() {
    setup();
    while (true) {
        loop();
    }
    return 0;
}