#include "rotaryencoder.h"

#include "hardware.h"

#include <stdint.h>

namespace {
constexpr uint8_t kEncA = 2;
constexpr uint8_t kEncB = 3;

volatile long g_encoderTicks = 0;
volatile uint8_t g_prevState = 0;
volatile int8_t g_stepAccum = 0;

void encoderISR() {
    const uint8_t a = (platform::digitalRead(kEncA) == platform::PinState::High) ? 1U : 0U;
    const uint8_t b = (platform::digitalRead(kEncB) == platform::PinState::High) ? 1U : 0U;
    const uint8_t state = static_cast<uint8_t>((a << 1) | b);

    static const int8_t kTransitionDelta[16] = {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };

    const uint8_t index = static_cast<uint8_t>((g_prevState << 2) | state);
    const int8_t delta = kTransitionDelta[index];

    if (delta != 0) {
        const int8_t accum = static_cast<int8_t>(g_stepAccum + delta);
        if (accum >= 2) {
            ++g_encoderTicks;
            g_stepAccum = 0;
        } else if (accum <= -2) {
            --g_encoderTicks;
            g_stepAccum = 0;
        } else {
            g_stepAccum = accum;
        }
    } else if (state != g_prevState) {
        g_stepAccum = 0;
    }

    g_prevState = state;
}
}

void RotaryEncoder_setup() {
    platform::pinMode(kEncA, platform::PinMode::InputPullup);
    platform::pinMode(kEncB, platform::PinMode::InputPullup);

    const uint8_t a = (platform::digitalRead(kEncA) == platform::PinState::High) ? 1U : 0U;
    const uint8_t b = (platform::digitalRead(kEncB) == platform::PinState::High) ? 1U : 0U;
    g_prevState = static_cast<uint8_t>((a << 1) | b);
    g_stepAccum = 0;

    platform::attachInterrupt(kEncA, encoderISR, platform::InterruptMode::Change);
    platform::attachInterrupt(kEncB, encoderISR, platform::InterruptMode::Change);
}

long RotaryEncoder_getTicks() {
    long ticks = 0;
    platform::disableInterrupts();
    ticks = g_encoderTicks;
    platform::enableInterrupts();
    return ticks;
}