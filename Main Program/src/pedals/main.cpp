#include "hardware.h"
#include "potentiometers.h"

#include <avr/interrupt.h>
#include <stdint.h>

namespace {
uint32_t g_lastPrint = 0;
constexpr uint32_t kPrintIntervalMs = 20;
}

int main() {
    hardware::initSystemTimer();
    Potentiometers::init();
    hardware::initUart(115200);

    sei();

    hardware::delayMs(500);
    hardware::writeUartLine("Pedal system initialized");

    while (true) {
        Potentiometers::update();

        float throttle = 0.0f;
        float brake = 0.0f;
        Potentiometers::getValues(throttle, brake);

        const uint32_t now = hardware::millis();
        if ((now - g_lastPrint) >= kPrintIntervalMs) {
            g_lastPrint = now;
            hardware::writeUart("T:");
            hardware::writeUartInt32(static_cast<int32_t>(throttle));
            hardware::writeUart(",B:");
            hardware::writeUartInt32(static_cast<int32_t>(brake));
            hardware::writeUart("\r\n");
        }
    }
}