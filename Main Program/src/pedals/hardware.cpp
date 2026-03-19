#include "hardware.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

namespace {

struct AnalogDef {
    volatile uint8_t* ddr;
    volatile uint8_t* port;
    uint8_t bit;
    uint8_t channel;
};

volatile uint32_t g_millis = 0;

AnalogDef analogDef(hardware::AnalogPin pin) {
#if defined(__AVR_ATmega328P__)
    switch (pin) {
        case hardware::AnalogPin::A0: return {&DDRC, &PORTC, PC0, 0};
        case hardware::AnalogPin::A1: return {&DDRC, &PORTC, PC1, 1};
        case hardware::AnalogPin::A2: return {&DDRC, &PORTC, PC2, 2};
        case hardware::AnalogPin::A3: return {&DDRC, &PORTC, PC3, 3};
        case hardware::AnalogPin::A4: return {&DDRC, &PORTC, PC4, 4};
        case hardware::AnalogPin::A5: return {&DDRC, &PORTC, PC5, 5};
        default: return {&DDRC, &PORTC, PC0, 0};
    }
#elif defined(__AVR_ATmega32U4__)
    switch (pin) {
        case hardware::AnalogPin::A0: return {&DDRF, &PORTF, PF7, 7};
        case hardware::AnalogPin::A1: return {&DDRF, &PORTF, PF6, 6};
        case hardware::AnalogPin::A2: return {&DDRF, &PORTF, PF5, 5};
        case hardware::AnalogPin::A3: return {&DDRF, &PORTF, PF4, 4};
        case hardware::AnalogPin::A4: return {&DDRF, &PORTF, PF1, 1};
        case hardware::AnalogPin::A5: return {&DDRF, &PORTF, PF0, 0};
        default: return {&DDRF, &PORTF, PF7, 7};
    }
#else
#error "Unsupported MCU for pedals hardware backend."
#endif
}

void writeUnsigned(uint32_t value) {
    char buffer[10];
    uint8_t length = 0;

    if (value == 0U) {
        hardware::writeUartChar('0');
        return;
    }

    while (value > 0U && length < sizeof(buffer)) {
        buffer[length++] = static_cast<char>('0' + (value % 10U));
        value /= 10U;
    }

    while (length > 0U) {
        hardware::writeUartChar(buffer[--length]);
    }
}

}

ISR(TIMER0_COMPA_vect) {
    ++g_millis;
}

namespace hardware {

void initSystemTimer() {
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        g_millis = 0;
    }

    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS01) | _BV(CS00);
    OCR0A = static_cast<uint8_t>((F_CPU / 64UL / 1000UL) - 1UL);
    TIMSK0 |= _BV(OCIE0A);
}

uint32_t millis() {
    uint32_t now = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        now = g_millis;
    }
    return now;
}

void delayMs(uint32_t ms) {
    const uint32_t start = millis();
    while ((millis() - start) < ms) {
    }
}

void initAdc() {
    ADMUX = _BV(REFS0);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
#if defined(MUX5)
    ADCSRB &= static_cast<uint8_t>(~_BV(MUX5));
#endif
}

void configureInput(AnalogPin pin) {
    const AnalogDef def = analogDef(pin);
    const uint8_t mask = static_cast<uint8_t>(_BV(def.bit));
    *def.ddr &= static_cast<uint8_t>(~mask);
    *def.port &= static_cast<uint8_t>(~mask);
}

uint16_t readAdc(AnalogPin pin) {
    const AnalogDef def = analogDef(pin);

#if defined(MUX5)
    if ((def.channel & 0x08U) != 0U) {
        ADCSRB |= _BV(MUX5);
    } else {
        ADCSRB &= static_cast<uint8_t>(~_BV(MUX5));
    }
    ADMUX = static_cast<uint8_t>(_BV(REFS0) | (def.channel & 0x07U));
#else
    ADMUX = static_cast<uint8_t>(_BV(REFS0) | (def.channel & 0x0FU));
#endif

    ADCSRA |= _BV(ADSC);
    while ((ADCSRA & _BV(ADSC)) != 0U) {
    }

    return ADC;
}

void initUart(uint32_t baud) {
    const uint16_t ubrr = static_cast<uint16_t>((F_CPU / (16UL * baud)) - 1UL);

#if defined(__AVR_ATmega328P__)
    UBRR0H = static_cast<uint8_t>(ubrr >> 8);
    UBRR0L = static_cast<uint8_t>(ubrr & 0xFFU);
    UCSR0A = 0;
    UCSR0B = _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
#elif defined(__AVR_ATmega32U4__)
    UBRR1H = static_cast<uint8_t>(ubrr >> 8);
    UBRR1L = static_cast<uint8_t>(ubrr & 0xFFU);
    UCSR1A = 0;
    UCSR1B = _BV(TXEN1);
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
#endif
}

void writeUartChar(char c) {
#if defined(__AVR_ATmega328P__)
    while ((UCSR0A & _BV(UDRE0)) == 0U) {
    }
    UDR0 = static_cast<uint8_t>(c);
#elif defined(__AVR_ATmega32U4__)
    while ((UCSR1A & _BV(UDRE1)) == 0U) {
    }
    UDR1 = static_cast<uint8_t>(c);
#endif
}

void writeUart(const char* text) {
    while (*text != '\0') {
        writeUartChar(*text++);
    }
}

void writeUartLine(const char* text) {
    writeUart(text);
    writeUart("\r\n");
}

void writeUartInt32(int32_t value) {
    if (value < 0) {
        writeUartChar('-');
        const int64_t magnitude = -static_cast<int64_t>(value);
        writeUnsigned(static_cast<uint32_t>(magnitude));
    } else {
        writeUnsigned(static_cast<uint32_t>(value));
    }
}

}