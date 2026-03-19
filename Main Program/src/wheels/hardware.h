#pragma once

#include <stdint.h>

namespace platform {

enum class PinMode : uint8_t {
    InputPullup = 0,
    Output = 1
};

enum class PinState : uint8_t {
    Low = 0,
    High = 1
};

enum class InterruptMode : uint8_t {
    Change = 0,
    Rising = 1,
    Falling = 2
};

using IsrHandler = void (*)();

void serialInit(uint32_t baud);
bool serialReadChar(char& value);
void serialWrite(const char* text);
void serialWriteLine(const char* text);
void serialWriteInt32(int32_t value);

void pinMode(uint8_t pin, PinMode mode);
PinState digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, PinState state);
void pwmWrite(uint8_t pin, uint8_t duty);
uint16_t analogRead(uint8_t pin);

void attachInterrupt(uint8_t pin, IsrHandler handler, InterruptMode mode);
void disableInterrupts();
void enableInterrupts();

uint32_t millis();
uint32_t micros();
void sleepMs(uint32_t ms);

}