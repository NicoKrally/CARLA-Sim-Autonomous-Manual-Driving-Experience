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
    bool valid;
    bool analog;
    uint8_t adcChannel;
};

volatile uint32_t g_timeMs = 0;
platform::IsrHandler g_int0Handler = nullptr;
platform::IsrHandler g_int1Handler = nullptr;
bool g_timerInitialized = false;
bool g_timer1PwmInitialized = false;

PinDef getPinDef(uint8_t pin) {
#if defined(__AVR_ATmega328P__)
    switch (pin) {
        case 2: return {&DDRD, &PORTD, &PIND, PD2, true, false, 0};
        case 3: return {&DDRD, &PORTD, &PIND, PD3, true, false, 0};
        case 8: return {&DDRB, &PORTB, &PINB, PB0, true, false, 0};
        case 9: return {&DDRB, &PORTB, &PINB, PB1, true, false, 0};
        case 10: return {&DDRB, &PORTB, &PINB, PB2, true, false, 0};
        case 11: return {&DDRB, &PORTB, &PINB, PB3, true, false, 0};
        case 13: return {&DDRB, &PORTB, &PINB, PB5, true, false, 0};
        case 18: return {&DDRC, &PORTC, &PINC, PC0, true, true, 0};
        case 19: return {&DDRC, &PORTC, &PINC, PC1, true, true, 1};
        case 20: return {&DDRC, &PORTC, &PINC, PC2, true, true, 2};
        case 21: return {&DDRC, &PORTC, &PINC, PC3, true, true, 3};
        case 22: return {&DDRC, &PORTC, &PINC, PC4, true, true, 4};
        case 23: return {&DDRC, &PORTC, &PINC, PC5, true, true, 5};
        default: return {nullptr, nullptr, nullptr, 0, false, false, 0};
    }
#elif defined(__AVR_ATmega32U4__)
    switch (pin) {
        case 2: return {&DDRD, &PORTD, &PIND, PD1, true, false, 0};
        case 3: return {&DDRD, &PORTD, &PIND, PD0, true, false, 0};
        case 8: return {&DDRB, &PORTB, &PINB, PB4, true, false, 0};
        case 9: return {&DDRB, &PORTB, &PINB, PB5, true, false, 0};
        case 10: return {&DDRB, &PORTB, &PINB, PB6, true, false, 0};
        case 11: return {&DDRB, &PORTB, &PINB, PB7, true, false, 0};
        case 13: return {&DDRC, &PORTC, &PINC, PC7, true, false, 0};
        case 18: return {&DDRF, &PORTF, &PINF, PF7, true, true, 7};
        case 19: return {&DDRF, &PORTF, &PINF, PF6, true, true, 6};
        case 20: return {&DDRF, &PORTF, &PINF, PF5, true, true, 5};
        case 21: return {&DDRF, &PORTF, &PINF, PF4, true, true, 4};
        case 22: return {&DDRF, &PORTF, &PINF, PF1, true, true, 1};
        case 23: return {&DDRF, &PORTF, &PINF, PF0, true, true, 0};
        default: return {nullptr, nullptr, nullptr, 0, false, false, 0};
    }
#else
#error "Unsupported MCU for wheels backend."
#endif
}

void initSystemTimerIfNeeded() {
    if (g_timerInitialized) {
        return;
    }

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        g_timeMs = 0;
    }

    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS01) | _BV(CS00);
    OCR0A = static_cast<uint8_t>((F_CPU / 64UL / 1000UL) - 1UL);
    TIMSK0 |= _BV(OCIE0A);

    g_timerInitialized = true;
}

void initTimer1PwmIfNeeded() {
    if (g_timer1PwmInitialized) {
        return;
    }

    const PinDef pin9 = getPinDef(9);
    const PinDef pin10 = getPinDef(10);

    if (pin9.valid) {
        *pin9.ddr |= static_cast<uint8_t>(_BV(pin9.bit));
    }
    if (pin10.valid) {
        *pin10.ddr |= static_cast<uint8_t>(_BV(pin10.bit));
    }

    TCCR1A = _BV(WGM10) | _BV(COM1A1) | _BV(COM1B1);
    TCCR1B = _BV(WGM12) | _BV(CS11);
    OCR1A = 0;
    OCR1B = 0;

    g_timer1PwmInitialized = true;
}

void writeUnsigned(uint32_t value) {
    char buffer[10];
    uint8_t length = 0;

    if (value == 0U) {
        platform::serialWrite("0");
        return;
    }

    while (value > 0U && length < sizeof(buffer)) {
        buffer[length++] = static_cast<char>('0' + (value % 10U));
        value /= 10U;
    }

    while (length > 0U) {
        char out[2] = {buffer[--length], '\0'};
        platform::serialWrite(out);
    }
}

uint8_t modeToSenseBits(platform::InterruptMode mode) {
    switch (mode) {
        case platform::InterruptMode::Rising:
            return 0x03U;
        case platform::InterruptMode::Falling:
            return 0x02U;
        case platform::InterruptMode::Change:
        default:
            return 0x01U;
    }
}

}

ISR(TIMER0_COMPA_vect) {
    ++g_timeMs;
}

ISR(INT0_vect) {
    if (g_int0Handler != nullptr) {
        g_int0Handler();
    }
}

ISR(INT1_vect) {
    if (g_int1Handler != nullptr) {
        g_int1Handler();
    }
}

namespace platform {

void serialInit(uint32_t baud) {
    initSystemTimerIfNeeded();

    const uint16_t ubrr = static_cast<uint16_t>((F_CPU / (16UL * baud)) - 1UL);

#if defined(__AVR_ATmega328P__)
    UBRR0H = static_cast<uint8_t>(ubrr >> 8);
    UBRR0L = static_cast<uint8_t>(ubrr & 0xFFU);
    UCSR0A = 0;
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
#elif defined(__AVR_ATmega32U4__)
    UBRR1H = static_cast<uint8_t>(ubrr >> 8);
    UBRR1L = static_cast<uint8_t>(ubrr & 0xFFU);
    UCSR1A = 0;
    UCSR1B = _BV(RXEN1) | _BV(TXEN1);
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
#endif
}

bool serialReadChar(char& value) {
#if defined(__AVR_ATmega328P__)
    if ((UCSR0A & _BV(RXC0)) == 0U) {
        return false;
    }
    value = static_cast<char>(UDR0);
    return true;
#elif defined(__AVR_ATmega32U4__)
    if ((UCSR1A & _BV(RXC1)) == 0U) {
        return false;
    }
    value = static_cast<char>(UDR1);
    return true;
#else
    (void)value;
    return false;
#endif
}

void serialWrite(const char* text) {
    while (*text != '\0') {
#if defined(__AVR_ATmega328P__)
        while ((UCSR0A & _BV(UDRE0)) == 0U) {
        }
        UDR0 = static_cast<uint8_t>(*text++);
#elif defined(__AVR_ATmega32U4__)
        while ((UCSR1A & _BV(UDRE1)) == 0U) {
        }
        UDR1 = static_cast<uint8_t>(*text++);
#else
        ++text;
#endif
    }
}

void serialWriteLine(const char* text) {
    serialWrite(text);
    serialWrite("\r\n");
}

void serialWriteInt32(int32_t value) {
    if (value < 0) {
        serialWrite("-");
        const int64_t magnitude = -static_cast<int64_t>(value);
        writeUnsigned(static_cast<uint32_t>(magnitude));
    } else {
        writeUnsigned(static_cast<uint32_t>(value));
    }
    serialWrite("\r\n");
}

void pinMode(uint8_t pin, PinMode mode) {
    const PinDef def = getPinDef(pin);
    if (!def.valid) {
        return;
    }

    const uint8_t mask = static_cast<uint8_t>(_BV(def.bit));
    if (mode == PinMode::Output) {
        *def.ddr |= mask;
    } else {
        *def.ddr &= static_cast<uint8_t>(~mask);
        *def.port |= mask;
    }
}

PinState digitalRead(uint8_t pin) {
    const PinDef def = getPinDef(pin);
    if (!def.valid) {
        return PinState::High;
    }

    const uint8_t mask = static_cast<uint8_t>(_BV(def.bit));
    return (((*def.pin) & mask) != 0U) ? PinState::High : PinState::Low;
}

void digitalWrite(uint8_t pin, PinState state) {
    const PinDef def = getPinDef(pin);
    if (!def.valid) {
        return;
    }

    const uint8_t mask = static_cast<uint8_t>(_BV(def.bit));
    if (state == PinState::High) {
        *def.port |= mask;
    } else {
        *def.port &= static_cast<uint8_t>(~mask);
    }
}

void pwmWrite(uint8_t pin, uint8_t duty) {
    if (pin == 9U) {
        initTimer1PwmIfNeeded();
        OCR1A = duty;
        return;
    }

    if (pin == 10U) {
        initTimer1PwmIfNeeded();
        OCR1B = duty;
        return;
    }

    if (duty > 0U) {
        digitalWrite(pin, PinState::High);
    } else {
        digitalWrite(pin, PinState::Low);
    }
}

uint16_t analogRead(uint8_t pin) {
    const PinDef def = getPinDef(pin);
    if (!def.valid || !def.analog) {
        return 0;
    }

    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

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

void attachInterrupt(uint8_t pin, IsrHandler handler, InterruptMode mode) {
    const uint8_t sense = modeToSenseBits(mode);

#if defined(__AVR_ATmega328P__)
    if (pin == 2U) {
        g_int0Handler = handler;
        EICRA = static_cast<uint8_t>((EICRA & static_cast<uint8_t>(~(_BV(ISC01) | _BV(ISC00)))) |
                                     ((sense & 0x03U) << ISC00));
        EIMSK |= _BV(INT0);
    } else if (pin == 3U) {
        g_int1Handler = handler;
        EICRA = static_cast<uint8_t>((EICRA & static_cast<uint8_t>(~(_BV(ISC11) | _BV(ISC10)))) |
                                     ((sense & 0x03U) << ISC10));
        EIMSK |= _BV(INT1);
    }
#elif defined(__AVR_ATmega32U4__)
    if (pin == 3U) {
        g_int0Handler = handler;
        EICRA = static_cast<uint8_t>((EICRA & static_cast<uint8_t>(~(_BV(ISC01) | _BV(ISC00)))) |
                                     ((sense & 0x03U) << ISC00));
        EIMSK |= _BV(INT0);
    } else if (pin == 2U) {
        g_int1Handler = handler;
        EICRA = static_cast<uint8_t>((EICRA & static_cast<uint8_t>(~(_BV(ISC11) | _BV(ISC10)))) |
                                     ((sense & 0x03U) << ISC10));
        EIMSK |= _BV(INT1);
    }
#else
    (void)pin;
    (void)handler;
    (void)mode;
#endif
}

void disableInterrupts() {
    cli();
}

void enableInterrupts() {
    sei();
}

uint32_t millis() {
    uint32_t now = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        now = g_timeMs;
    }
    return now;
}

uint32_t micros() {
    uint32_t ms = 0;
    uint8_t ticks = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ms = g_timeMs;
        ticks = TCNT0;
        if ((TIFR0 & _BV(OCF0A)) != 0U && ticks < OCR0A) {
            ++ms;
            ticks = TCNT0;
        }
    }

    return (ms * 1000UL) + (static_cast<uint32_t>(ticks) * 4UL);
}

void sleepMs(uint32_t ms) {
    const uint32_t start = millis();
    while ((millis() - start) < ms) {
    }
}

}