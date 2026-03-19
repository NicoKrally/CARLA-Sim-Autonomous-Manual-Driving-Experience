#include "position.h"

#include "rotaryencoder.h"

long getEncoderCount() {
    return RotaryEncoder_getTicks();
}