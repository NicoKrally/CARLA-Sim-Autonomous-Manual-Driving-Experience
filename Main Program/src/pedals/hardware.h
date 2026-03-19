#pragma once

#include <stdint.h>

namespace hardware {

enum class AnalogPin : uint8_t {
    A0 = 0,
    A1 = 1,
    A2 = 2,
    A3 = 3,
    A4 = 4,
    A5 = 5
};

void initSystemTimer();
uint32_t millis();
void delayMs(uint32_t ms);

void initAdc();
void configureInput(AnalogPin pin);
uint16_t readAdc(AnalogPin pin);

void initUart(uint32_t baud);
void writeUartChar(char c);
void writeUart(const char* text);
void writeUartLine(const char* text);
void writeUartInt32(int32_t value);

}