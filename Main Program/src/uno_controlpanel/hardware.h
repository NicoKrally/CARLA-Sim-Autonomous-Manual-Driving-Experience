#pragma once

#include <stdint.h>

namespace hardware {

enum class Pin : uint8_t {
    D0 = 0,
    D1 = 1,
    D2 = 2,
    D3 = 3,
    D4 = 4,
    D13 = 13,
    A0 = 14,
    A1 = 15
};

void initSystemTimer();
uint32_t millis();

void configureInputPullup(Pin pin);
void configureOutput(Pin pin);
bool readDigital(Pin pin);
void writeDigital(Pin pin, bool high);

void initAdc();
uint16_t readAdc(Pin analogPin);

void uartEnableTx(uint32_t baud);
void uartWriteChar(char c);
void uartWrite(const char* text);
void uartWriteLine(const char* text);
void uartWriteInt32(int32_t value);
void uartFlushAndDisable();

void enableInterrupts();

}