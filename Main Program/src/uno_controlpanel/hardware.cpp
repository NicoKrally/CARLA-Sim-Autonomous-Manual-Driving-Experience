#include "hardware.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

namespace {

struct PinDef {
    volatile uint8_t* ddr;
    volatile uint8_t* port;
    volatile uint8_t* pin;
    uint8_t bit;
    bool isAnalog;
    uint8_t adcChannel;
};

volatile uint32_t g_millis = 0;

PinDef pinDef(hardware::Pin pin) {
#if defined(__AVR_ATmega328P__)
    switch (pin) {
        case hardware::Pin::D0: return {&DDRD, &PORTD, &PIND, PD0, false, 0};
        case hardware::Pin::D1: return {&DDRD, &PORTD, &PIND, PD1, false, 0};
        case hardware::Pin::D2: return {&DDRD, &PORTD, &PIND, PD2, false, 0};
        case hardware::Pin::D3: return {&DDRD, &PORTD, &PIND, PD3, false, 0};
        case hardware::Pin::D4: return {&DDRD, &PORTD, &PIND, PD4, false, 0};
        case hardware::Pin::D13: return {&DDRB, &PORTB, &PINB, PB5, false, 0};
        case hardware::Pin::A0: return {&DDRC, &PORTC, &PINC, PC0, true, 0};
        case hardware::Pin::A1: return {&DDRC, &PORTC, &PINC, PC1, true, 1};
        default: return {&DDRD, &PORTD, &PIND, PD0, false, 0};
    }
#elif defined(__AVR_ATmega32U4__)
    switch (pin) {
        case hardware::Pin::D0: return {&DDRD, &PORTD, &PIND, PD2, false, 0};
        case hardware::Pin::D1: return {&DDRD, &PORTD, &PIND, PD3, false, 0};
        case hardware::Pin::D2: return {&DDRD, &PORTD, &PIND, PD1, false, 0};
        case hardware::Pin::D3: return {&DDRD, &PORTD, &PIND, PD0, false, 0};
        case hardware::Pin::D4: return {&DDRD, &PORTD, &PIND, PD4, false, 0};
        case hardware::Pin::D13: return {&DDRC, &PORTC, &PINC, PC7, false, 0};
        case hardware::Pin::A0: return {&DDRF, &PORTF, &PINF, PF7, true, 7};
        case hardware::Pin::A1: return {&DDRF, &PORTF, &PINF, PF6, true, 6};
        default: return {&DDRD, &PORTD, &PIND, PD2, false, 0};
    }
#else
#error "Unsupported MCU for control panel backend."
#endif
}

void writeUnsigned(uint32_t value) {
    char buffer[10];
    uint8_t length = 0;

    if (value == 0U) {
        hardware::uartWriteChar('0');
        return;
    }

    while (value > 0U && length < sizeof(buffer)) {
        buffer[length++] = static_cast<char>('0' + (value % 10U));
        value /= 10U;
    }

    while (length > 0U) {
        hardware::uartWriteChar(buffer[--length]);
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

void configureInputPullup(Pin pin) {
    const PinDef def = pinDef(pin);
    const uint8_t mask = static_cast<uint8_t>(_BV(def.bit));
    *def.ddr &= static_cast<uint8_t>(~mask);
    *def.port |= mask;
}

void configureOutput(Pin pin) {
    const PinDef def = pinDef(pin);
    const uint8_t mask = static_cast<uint8_t>(_BV(def.bit));
    *def.ddr |= mask;
}

bool readDigital(Pin pin) {
    const PinDef def = pinDef(pin);
    const uint8_t mask = static_cast<uint8_t>(_BV(def.bit));
    return ((*def.pin) & mask) != 0U;
}

void writeDigital(Pin pin, bool high) {
    const PinDef def = pinDef(pin);
    const uint8_t mask = static_cast<uint8_t>(_BV(def.bit));
    if (high) {
        *def.port |= mask;
    } else {
        *def.port &= static_cast<uint8_t>(~mask);
    }
}

void initAdc() {
    ADMUX = _BV(REFS0);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
#if defined(MUX5)
    ADCSRB &= static_cast<uint8_t>(~_BV(MUX5));
#endif
}

uint16_t readAdc(Pin analogPin) {
    const PinDef def = pinDef(analogPin);

#if defined(MUX5)
    if ((def.adcChannel & 0x08U) != 0U) {
        ADCSRB |= _BV(MUX5);
    } else {
        ADCSRB &= static_cast<uint8_t>(~_BV(MUX5));
    }
    ADMUX = static_cast<uint8_t>(_BV(REFS0) | (def.adcChannel & 0x07U));
#else
    ADMUX = static_cast<uint8_t>(_BV(REFS0) | (def.adcChannel & 0x0FU));
#endif

    ADCSRA |= _BV(ADSC);
    while ((ADCSRA & _BV(ADSC)) != 0U) {
    }

    return ADC;
}

void uartEnableTx(uint32_t baud) {
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

void uartWriteChar(char c) {
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

void uartWrite(const char* text) {
    while (*text != '\0') {
        uartWriteChar(*text++);
    }
}

void uartWriteLine(const char* text) {
    uartWrite(text);
    uartWrite("\r\n");
}

void uartWriteInt32(int32_t value) {
    if (value < 0) {
        uartWriteChar('-');
        const int64_t magnitude = -static_cast<int64_t>(value);
        writeUnsigned(static_cast<uint32_t>(magnitude));
    } else {
        writeUnsigned(static_cast<uint32_t>(value));
    }
}

void uartFlushAndDisable() {
#if defined(__AVR_ATmega328P__)
    while ((UCSR0A & _BV(UDRE0)) == 0U) {
    }
    while ((UCSR0A & _BV(TXC0)) == 0U) {
    }
    UCSR0A |= _BV(TXC0);
    UCSR0B = 0;
#elif defined(__AVR_ATmega32U4__)
    while ((UCSR1A & _BV(UDRE1)) == 0U) {
    }
    while ((UCSR1A & _BV(TXC1)) == 0U) {
    }
    UCSR1A |= _BV(TXC1);
    UCSR1B = 0;
#endif
}

void enableInterrupts() {
    sei();
}

}